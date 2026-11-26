//! Flat arrays for searches confined to one window.
//!
//! Every track crossing inside the window gets an index, and every search
//! state on a crossing a slot: one per heading while no via is recent, and
//! one per side the recent via came from, number of crossings back and
//! number of tracks across it landed, as far as a via reaches on the
//! crossing's layer. Costs, predecessors and step costs then live in arrays
//! reused from search to search through generation stamps, not in hash
//! maps.

use super::{Heading, RecentVia, Search, State, StepCosts, Visits};
use crate::access::Node;
use crate::tech::{CrossIdx, TrackIdx};
use pare_common::db::indices::LayerId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::Interval;
use pare_common::geom::rect::Rect;
use std::cell::RefCell;

/// Most state slots a dense window may hold; searches of larger windows
/// keep their states in a hash map.
const MAX_SLOTS: usize = 1 << 22;

/// No predecessor.
const NONE: u32 = u32::MAX;

/// The crossings of one layer inside the window, and the slots of the
/// states on them.
#[derive(Clone, Copy, Debug)]
struct Span {
    first_track: TrackIdx,
    tracks: u32,
    first_cross: CrossIdx,
    crosses: u32,
    /// The index of its first crossing among the window's.
    base: usize,
    /// The first slot of its states.
    first_slot: usize,
    /// Most crossings back along its track a recent via can lie.
    reach: u32,
    /// Most tracks across a recent via can lie, after jogs.
    across_reach: u32,
}

impl Span {
    const fn nodes(&self) -> usize {
        self.tracks as usize * self.crosses as usize
    }

    /// Slots for one side's recent vias: one per crossing back and track
    /// across.
    const fn recent_slots(&self) -> usize {
        (2 * self.reach as usize + 1) * (2 * self.across_reach as usize + 1)
    }

    const fn slots_per_node(&self) -> usize {
        3 + 2 * self.recent_slots()
    }

    /// The index of `n`, a node on this span's layer, among its crossings.
    fn local_index(&self, n: Node) -> Option<usize> {
        let t = n.track.steps_from(self.first_track)?;
        let c = n.cross.steps_from(self.first_cross)?;
        (t < self.tracks && c < self.crosses)
            .then(|| t as usize * self.crosses as usize + c as usize)
    }

    /// The node at `local`, an index among its crossings, on `layer`.
    fn node_at(&self, layer: LayerId, local: usize) -> Node {
        let crosses = self.crosses as usize;
        Node {
            layer,
            track: self
                .first_track
                .plus(crate::tech::index_u32(local / crosses)),
            cross: self
                .first_cross
                .plus(crate::tech::index_u32(local % crosses)),
        }
    }
}

/// The crossings inside a window, numbered layer by layer, each layer's
/// states given as many slots as its own reach for recent vias needs.
#[derive(Debug)]
struct Numbering {
    min_layer: LayerId,
    spans: Vec<Span>,
    nodes: usize,
    slots: usize,
}

impl Numbering {
    fn new(search: &Search<'_>, window: &Rect<Dbu>) -> Self {
        let tech = search.tech;
        let mut spans = Vec::new();
        let mut nodes = 0;
        let mut slots = 0;
        for layer in tech.routable_layers() {
            let lt = tech.layer(layer);
            let (Interval { lo: a0, hi: a1 }, Interval { lo: b0, hi: b1 }) =
                lt.across_along_rect(window);
            let tracks = lt.tracks_within(a0, a1);
            let crosses = lt.cross_within(b0, b1);
            let via_reach = search
                .via_layers(layer)
                .map(|other| search.via_reach(layer, other))
                .max()
                .unwrap_or_default();
            let at: Vec<Dbu> = crosses.clone().map(|c| lt.cross[c].at).collect();
            let span = Span {
                first_track: tracks.start,
                tracks: crate::tech::index_u32(tracks.len()),
                first_cross: crosses.start,
                crosses: crate::tech::index_u32(crosses.len()),
                base: nodes,
                first_slot: slots,
                reach: steps_within(&at, via_reach),
                across_reach: steps_within(lt.tracks.as_slice(), via_reach),
            };
            nodes += span.nodes();
            slots += span.nodes() * span.slots_per_node();
            spans.push(span);
        }
        Self {
            min_layer: tech.min_layer,
            spans,
            nodes,
            slots,
        }
    }

    fn span(&self, layer: LayerId) -> Option<&Span> {
        self.spans
            .get(layer.index().checked_sub(self.min_layer.index())?)
    }

    fn node_index(&self, n: Node) -> Option<usize> {
        let span = self.span(n.layer)?;
        Some(span.base + span.local_index(n)?)
    }

    /// The node whose states hold `slot`.
    fn node_of_slot(&self, slot: usize) -> Node {
        let l = self.spans.partition_point(|s| s.first_slot <= slot) - 1;
        let span = &self.spans[l];
        let local = (slot - span.first_slot) / span.slots_per_node();
        span.node_at(LayerId::new(self.min_layer.index() + l), local)
    }

    /// The slot of `state`, which its recent jog does not change: states
    /// are told apart by [`State::key`].
    fn slot(&self, state: State) -> Option<usize> {
        let span = self.span(state.node.layer)?;
        let local = span.local_index(state.node)?;
        let within = match state.recent_via {
            None => match state.heading {
                Heading::Still => 0,
                Heading::Up => 1,
                Heading::Down => 2,
            },
            Some(RecentVia { from, track, cross }) => {
                let side = usize::from(from > state.node.layer);
                let back = state.node.cross.offset_from(cross);
                let over = i64::from(state.node.track.0) - i64::from(track.0);
                let (reach, across) = (i64::from(span.reach), i64::from(span.across_reach));
                if back.abs() > reach || over.abs() > across {
                    return None;
                }
                let along = usize::try_from(back + reach).ok()?;
                let across_offset = usize::try_from(over + across).ok()?;
                let offset = across_offset * (2 * span.reach as usize + 1) + along;
                3 + side * span.recent_slots() + offset
            }
        };
        Some(span.first_slot + local * span.slots_per_node() + within)
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
    crate::tech::index_u32(most)
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
    jog: Vec<f64>,
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
            b.jog.resize(nodes, 0.0);
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
        if numbering.slots > MAX_SLOTS {
            return None;
        }
        let buffers = Buffers::take(numbering.slots, numbering.nodes);
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
            jog: &mut b.jog,
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
            .map_or(NONE, crate::tech::index_u32);
    }

    fn path_to(&self, end: State) -> Vec<Node> {
        let mut path = vec![end.node];
        let mut slot = self
            .numbering
            .slot(end)
            .map_or(NONE, crate::tech::index_u32);
        while slot != NONE && self.seen[slot as usize] == *self.generation {
            let prev = self.from[slot as usize];
            if prev == NONE {
                break;
            }
            path.push(self.numbering.node_of_slot(prev as usize));
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
    jog: &'a mut [f64],
    /// Which of a node's step costs are known: wire is bit 0, via bit 1,
    /// jog bit 2.
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

    fn via(&mut self, search: &Search<'_>, lower: Node, upper: Node) -> f64 {
        let Some(i) = self.numbering.node_index(lower) else {
            return super::FreshCosts.via(search, lower, upper);
        };
        if self.known(i) & 2 == 0 {
            self.via[i] = super::FreshCosts.via(search, lower, upper);
            self.priced[i] |= 2;
        }
        self.via[i]
    }

    fn jog(&mut self, search: &Search<'_>, lower: Node) -> f64 {
        let Some(i) = self.numbering.node_index(lower) else {
            return super::FreshCosts.jog(search, lower);
        };
        if self.known(i) & 4 == 0 {
            self.jog[i] = super::FreshCosts.jog(search, lower);
            self.priced[i] |= 4;
        }
        self.jog[i]
    }
}

#[cfg(test)]
mod tests;
