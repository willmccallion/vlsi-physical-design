//! Flat arrays for searches confined to one window.
//!
//! Every track crossing inside the window gets an index, and every search
//! state on a crossing a slot: one per heading while no via is recent, and
//! one per side the recent via came from and number of crossings back it
//! landed. Costs, predecessors and step costs then live in arrays reused
//! from search to search through generation stamps, not in hash maps.

use super::{Heading, RecentVia, Search, State, StepCosts, Visits};
use crate::access::Node;
use pare_common::db::indices::LayerId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::rect::Rect;
use std::cell::RefCell;

/// Most state slots a dense window may hold; searches of larger windows
/// keep their states in a hash map.
const MAX_SLOTS: usize = 1 << 22;

/// No predecessor.
const NONE: u32 = u32::MAX;

/// The crossings of one layer inside the window.
#[derive(Clone, Copy, Debug)]
struct Span {
    first_track: u32,
    tracks: u32,
    first_cross: u32,
    crosses: u32,
    base: usize,
}

/// The crossings inside a window, numbered layer by layer.
#[derive(Debug)]
struct Numbering {
    min_layer: usize,
    spans: Vec<Span>,
    nodes: usize,
    /// Most crossings back along its track a recent via can lie.
    reach: u32,
}

impl Numbering {
    fn new(search: &Search<'_>, window: &Rect<Dbu>) -> Self {
        let tech = search.tech;
        let min_layer = tech.min_layer.index();
        let mut spans = Vec::new();
        let mut nodes = 0;
        let mut reach = 0;
        for l in min_layer..=tech.max_layer.index() {
            let layer = LayerId::new(l);
            let lt = tech.layer(layer);
            let ((a0, a1), (b0, b1)) = lt.across_along_rect(window);
            let tracks = lt.tracks_within(a0, a1);
            let crosses = lt.cross_within(b0, b1);
            let span = Span {
                first_track: tracks.start,
                tracks: tracks.end - tracks.start,
                first_cross: crosses.start,
                crosses: crosses.end - crosses.start,
                base: nodes,
            };
            nodes += span.tracks as usize * span.crosses as usize;
            spans.push(span);
            let neighbours = [l.checked_sub(1), Some(l + 1)];
            let via_reach = neighbours
                .into_iter()
                .flatten()
                .map(LayerId::new)
                .filter(|&other| tech.min_layer <= other && other <= tech.max_layer)
                .map(|other| search.via_reach(layer, other))
                .max()
                .unwrap_or_default();
            let at: Vec<Dbu> = crosses.map(|c| lt.cross[c as usize].at).collect();
            reach = reach.max(steps_within(&at, via_reach));
        }
        Self {
            min_layer,
            spans,
            nodes,
            reach,
        }
    }

    const fn slots_per_node(&self) -> usize {
        3 + 2 * (2 * self.reach as usize + 1)
    }

    fn node_index(&self, n: Node) -> Option<usize> {
        let span = self
            .spans
            .get(n.layer.index().checked_sub(self.min_layer)?)?;
        let t = n.track.checked_sub(span.first_track)?;
        let c = n.cross.checked_sub(span.first_cross)?;
        (t < span.tracks && c < span.crosses)
            .then(|| span.base + t as usize * span.crosses as usize + c as usize)
    }

    fn node_at(&self, index: usize) -> Node {
        let l = self.spans.partition_point(|s| s.base <= index) - 1;
        let span = self.spans[l];
        let offset = index - span.base;
        let crosses = span.crosses as usize;
        Node {
            layer: LayerId::new(self.min_layer + l),
            track: span.first_track + crate::tech::to_u32(offset / crosses),
            cross: span.first_cross + crate::tech::to_u32(offset % crosses),
        }
    }

    fn slot(&self, state: State) -> Option<usize> {
        let node = self.node_index(state.node)?;
        let within = match state.recent_via {
            None => match state.heading {
                Heading::Still => 0,
                Heading::Up => 1,
                Heading::Down => 2,
            },
            Some(RecentVia { from, cross }) => {
                let side = usize::from(from > state.node.layer);
                let back = i64::from(state.node.cross) - i64::from(cross);
                let reach = i64::from(self.reach);
                if back.abs() > reach {
                    return None;
                }
                let offset = usize::try_from(back + reach).ok()?;
                3 + side * (2 * self.reach as usize + 1) + offset
            }
        };
        Some(node * self.slots_per_node() + within)
    }
}

/// The most steps along `at`, a sorted run of crossings, that stay closer
/// than `reach` to where they started.
fn steps_within(at: &[Dbu], reach: Dbu) -> u32 {
    let mut most = 0;
    let mut end = 0;
    for start in 0..at.len() {
        end = end.max(start);
        while end + 1 < at.len() && at[end + 1] - at[start] < reach {
            end += 1;
        }
        most = most.max(end - start);
    }
    crate::tech::to_u32(most)
}

/// Arrays reused by one thread's dense searches.
#[derive(Debug, Default)]
struct Buffers {
    cost: Vec<f64>,
    from: Vec<u32>,
    seen: Vec<u32>,
    seen_generation: u32,
    wire: Vec<f64>,
    via: Vec<f64>,
    priced: Vec<u8>,
    priced_generation: Vec<u32>,
    pricing: u32,
}

thread_local! {
    static POOL: RefCell<Vec<Buffers>> = const { RefCell::new(Vec::new()) };
}

impl Buffers {
    fn take(slots: usize, nodes: usize) -> Self {
        let mut b = POOL.with_borrow_mut(Vec::pop).unwrap_or_default();
        if b.seen.len() < slots {
            b.cost.resize(slots, 0.0);
            b.from.resize(slots, NONE);
            b.seen.resize(slots, 0);
        }
        if b.priced.len() < nodes {
            b.wire.resize(nodes, 0.0);
            b.via.resize(nodes, 0.0);
            b.priced.resize(nodes, 0);
            b.priced_generation.resize(nodes, 0);
        }
        b.pricing = b.pricing.wrapping_add(1);
        if b.pricing == 0 {
            b.priced_generation.fill(0);
            b.pricing = 1;
        }
        b
    }
}

/// Dense bookkeeping for the searches of one window.
pub(super) struct Dense {
    numbering: Numbering,
    buffers: Buffers,
}

impl Dense {
    /// Numbers the window's crossings, or `None` if it holds too many.
    pub(super) fn new(search: &Search<'_>, window: &Rect<Dbu>) -> Option<Self> {
        let numbering = Numbering::new(search, window);
        let slots = numbering.nodes.checked_mul(numbering.slots_per_node())?;
        if slots > MAX_SLOTS {
            return None;
        }
        let buffers = Buffers::take(slots, numbering.nodes);
        Some(Self { numbering, buffers })
    }

    /// Whether `node` lies inside the window.
    pub(super) fn covers(&self, node: Node) -> bool {
        self.numbering.node_index(node).is_some()
    }

    /// The state table and step costs of one search.
    pub(super) fn split(&mut self) -> (DenseVisits<'_>, CachedCosts<'_>) {
        let b = &mut self.buffers;
        let visits = DenseVisits {
            numbering: &self.numbering,
            cost: &mut b.cost,
            from: &mut b.from,
            seen: &mut b.seen,
            generation: &mut b.seen_generation,
        };
        let costs = CachedCosts {
            numbering: &self.numbering,
            wire: &mut b.wire,
            via: &mut b.via,
            priced: &mut b.priced,
            priced_generation: &mut b.priced_generation,
            pricing: b.pricing,
        };
        (visits, costs)
    }
}

impl Drop for Dense {
    fn drop(&mut self) {
        let buffers = std::mem::take(&mut self.buffers);
        POOL.with_borrow_mut(|pool| pool.push(buffers));
    }
}

/// Search states in flat arrays, valid while `seen` matches the generation.
pub(super) struct DenseVisits<'a> {
    numbering: &'a Numbering,
    cost: &'a mut [f64],
    from: &'a mut [u32],
    seen: &'a mut [u32],
    generation: &'a mut u32,
}

impl Visits for DenseVisits<'_> {
    fn clear(&mut self) {
        *self.generation = self.generation.wrapping_add(1);
        if *self.generation == 0 {
            self.seen.fill(0);
            *self.generation = 1;
        }
    }

    fn best(&self, state: State) -> Option<f64> {
        let slot = self.numbering.slot(state)?;
        (self.seen[slot] == *self.generation).then(|| self.cost[slot])
    }

    fn record(&mut self, state: State, cost: f64, from: Option<State>) {
        let Some(slot) = self.numbering.slot(state) else {
            return;
        };
        self.seen[slot] = *self.generation;
        self.cost[slot] = cost;
        self.from[slot] = from
            .and_then(|f| self.numbering.slot(f))
            .map_or(NONE, crate::tech::to_u32);
    }

    fn path_to(&self, end: State) -> Vec<Node> {
        let per_node = self.numbering.slots_per_node();
        let mut path = vec![end.node];
        let mut slot = self.numbering.slot(end).map_or(NONE, crate::tech::to_u32);
        while slot != NONE && self.seen[slot as usize] == *self.generation {
            let prev = self.from[slot as usize];
            if prev == NONE {
                break;
            }
            path.push(self.numbering.node_at(prev as usize / per_node));
            slot = prev;
        }
        path.reverse();
        path
    }
}

/// Step costs kept for every search through one window.
pub(super) struct CachedCosts<'a> {
    numbering: &'a Numbering,
    wire: &'a mut [f64],
    via: &'a mut [f64],
    /// Which of a node's step costs are known: wire is bit 0, via bit 1.
    priced: &'a mut [u8],
    priced_generation: &'a mut [u32],
    pricing: u32,
}

impl CachedCosts<'_> {
    /// The known step costs of node `i`, as `priced` bits.
    const fn known(&mut self, i: usize) -> u8 {
        if self.priced_generation[i] != self.pricing {
            self.priced_generation[i] = self.pricing;
            self.priced[i] = 0;
        }
        self.priced[i]
    }
}

impl StepCosts for CachedCosts<'_> {
    fn wire(&mut self, search: &Search<'_>, lower: Node) -> f64 {
        let Some(i) = self.numbering.node_index(lower) else {
            return super::FreshCosts.wire(search, lower);
        };
        if self.known(i) & 1 == 0 {
            self.wire[i] = super::FreshCosts.wire(search, lower);
            self.priced[i] |= 1;
        }
        self.wire[i]
    }

    fn via(&mut self, search: &Search<'_>, lower: Node) -> f64 {
        let Some(i) = self.numbering.node_index(lower) else {
            return super::FreshCosts.via(search, lower);
        };
        if self.known(i) & 2 == 0 {
            self.via[i] = super::FreshCosts.via(search, lower);
            self.priced[i] |= 2;
        }
        self.via[i]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steps_within_counts_the_longest_run_closer_than_the_reach() {
        let at = [Dbu(0), Dbu(10), Dbu(20), Dbu(50), Dbu(55)];

        assert_eq!(steps_within(&at, Dbu(21)), 2);
        assert_eq!(steps_within(&at, Dbu(20)), 1);
        assert_eq!(steps_within(&at, Dbu(1)), 0);
    }
}
