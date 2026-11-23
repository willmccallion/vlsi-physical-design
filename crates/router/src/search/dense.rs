//! Flat arrays for searches confined to one window.
//!
//! Every track crossing inside the window gets an index, and every search
//! state on a crossing a slot: one per heading while no via is recent, and
//! one per side the recent via came from, number of crossings back and
//! number of tracks across it landed; each of those once with no recent
//! jog and once per number of crossings back a recent jog lies. Costs,
//! predecessors and step costs then live in arrays reused from search to
//! search through generation stamps, not in hash maps.

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

/// The crossings of one layer inside the window.
#[derive(Clone, Copy, Debug)]
struct Span {
    first_track: TrackIdx,
    tracks: u32,
    first_cross: CrossIdx,
    crosses: u32,
    base: usize,
}

/// The crossings inside a window, numbered layer by layer.
#[derive(Debug)]
struct Numbering {
    min_layer: LayerId,
    spans: Vec<Span>,
    nodes: usize,
    /// Most crossings back along its track a recent via can lie.
    reach: u32,
    /// Most tracks across a recent via can lie, after jogs.
    across_reach: u32,
    /// Most crossings back along its track a recent jog can lie; none when
    /// no jog in the window can be crowded, so none is ever held.
    jog_reach: Option<u32>,
}

impl Numbering {
    fn new(search: &Search<'_>, window: &Rect<Dbu>) -> Self {
        let tech = search.tech;
        let mut spans = Vec::new();
        let mut nodes = 0;
        let mut reach = 0;
        let mut across_reach = 0;
        let mut jog_reach = None;
        for layer in tech.routable_layers() {
            let lt = tech.layer(layer);
            let (Interval { lo: a0, hi: a1 }, Interval { lo: b0, hi: b1 }) =
                lt.across_along_rect(window);
            let tracks = lt.tracks_within(a0, a1);
            let crosses = lt.cross_within(b0, b1);
            let span = Span {
                first_track: tracks.start,
                tracks: crate::tech::index_u32(tracks.len()),
                first_cross: crosses.start,
                crosses: crate::tech::index_u32(crosses.len()),
                base: nodes,
            };
            nodes += span.tracks as usize * span.crosses as usize;
            spans.push(span);
            let via_reach = search
                .via_layers(layer)
                .map(|other| search.via_reach(layer, other))
                .max()
                .unwrap_or_default();
            if let Some(k) = farthest_jog_crowding(search, layer, crosses.clone()) {
                jog_reach = Some(jog_reach.unwrap_or(0).max(k));
            }
            let at: Vec<Dbu> = crosses.map(|c| lt.cross[c].at).collect();
            reach = reach.max(steps_within(&at, via_reach));
            across_reach = across_reach.max(steps_within(lt.tracks.as_slice(), via_reach));
        }
        Self {
            min_layer: tech.min_layer,
            spans,
            nodes,
            reach,
            across_reach,
            jog_reach,
        }
    }

    /// Slots for one side's recent vias: one per crossing back and track
    /// across.
    const fn recent_slots(&self) -> usize {
        (2 * self.reach as usize + 1) * (2 * self.across_reach as usize + 1)
    }

    /// Slots for the recent jog: none, or one per crossing back it lies.
    const fn jog_slots(&self) -> usize {
        match self.jog_reach {
            Some(k) => 2 + 2 * k as usize,
            None => 1,
        }
    }

    const fn slots_per_node(&self) -> usize {
        (3 + 2 * self.recent_slots()) * self.jog_slots()
    }

    fn node_index(&self, n: Node) -> Option<usize> {
        let span = self
            .spans
            .get(n.layer.index().checked_sub(self.min_layer.index())?)?;
        let t = n.track.steps_from(span.first_track)?;
        let c = n.cross.steps_from(span.first_cross)?;
        (t < span.tracks && c < span.crosses)
            .then(|| span.base + t as usize * span.crosses as usize + c as usize)
    }

    fn node_at(&self, index: usize) -> Node {
        let l = self.spans.partition_point(|s| s.base <= index) - 1;
        let span = self.spans[l];
        let offset = index - span.base;
        let crosses = span.crosses as usize;
        Node {
            layer: LayerId::new(self.min_layer.index() + l),
            track: span
                .first_track
                .plus(crate::tech::index_u32(offset / crosses)),
            cross: span
                .first_cross
                .plus(crate::tech::index_u32(offset % crosses)),
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
            Some(RecentVia { from, track, cross }) => {
                let side = usize::from(from > state.node.layer);
                let back = state.node.cross.offset_from(cross);
                let over = i64::from(state.node.track.0) - i64::from(track.0);
                let (reach, across) = (i64::from(self.reach), i64::from(self.across_reach));
                if back.abs() > reach || over.abs() > across {
                    return None;
                }
                let along = usize::try_from(back + reach).ok()?;
                let across_offset = usize::try_from(over + across).ok()?;
                let offset = across_offset * (2 * self.reach as usize + 1) + along;
                3 + side * self.recent_slots() + offset
            }
        };
        let jog = match (state.recent_jog, self.jog_reach) {
            (None, _) => 0,
            (Some(_), None) => return None,
            (Some(cross), Some(k)) => {
                let back = state.node.cross.offset_from(cross);
                1 + usize::try_from(back + i64::from(k))
                    .ok()
                    .filter(|&j| j <= 2 * k as usize)?
            }
        };
        Some(node * self.slots_per_node() + within * self.jog_slots() + jog)
    }
}

/// The most crossings back along their track from a jog at one of `crosses`
/// of `layer` that a via could crowd it; none when no via could.
fn farthest_jog_crowding(
    search: &Search<'_>,
    layer: LayerId,
    crosses: impl Iterator<Item = CrossIdx>,
) -> Option<u32> {
    crosses
        .flat_map(|jog| {
            search
                .jog_crowding_crosses(layer, jog)
                .filter_map(move |c| c.steps_from(jog).or_else(|| jog.steps_from(c)))
        })
        .max()
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
        let per_node = self.numbering.slots_per_node();
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
mod tests {
    use super::*;

    use crate::occupancy::{Occupancy, Owner};
    use crate::search::{Costs, Marks};
    use crate::tech::{LeafClockPins, RouteTech};
    use pare_common::config::RoutingConfig;
    use pare_common::db::indices::NetId;

    #[test]
    fn a_recent_via_a_jog_left_on_the_next_track_gets_its_own_slot() {
        let db = crate::tests::tiny();
        let config = RoutingConfig {
            min_layer: Some("M1".to_string()),
            ..RoutingConfig::default()
        };
        let tech = RouteTech::new(&db, &config, LeafClockPins::default()).unwrap();
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(NetId(0)),
            costs: Costs {
                via: 50.0,
                conflict: 8.0,
                marker: 32.0,
                decay: 0.95,
            },
            guide: None,
            own_vias: None,
            ripped: &[],
        };
        let numbering = Numbering::new(&search, &db.die_area);
        let node = Node {
            layer: LayerId::new(0),
            track: TrackIdx(2),
            cross: CrossIdx(3),
        };
        let landed_on = |track: TrackIdx| State {
            node,
            heading: Heading::Still,
            recent_via: Some(RecentVia {
                from: LayerId::new(1),
                track,
                cross: node.cross,
            }),
            recent_jog: None,
        };

        let here = numbering.slot(landed_on(TrackIdx(2)));
        let beside = numbering.slot(landed_on(TrackIdx(1)));

        assert!(here.is_some() && beside.is_some() && here != beside);
    }

    /// `f` of the tiny design's search on M1 and up, with M1's spacing
    /// scaled to `spacing_halves` halves of itself.
    fn with_m1_spacing<R>(spacing_halves: i64, f: impl FnOnce(&Search<'_>) -> R) -> R {
        let db = crate::tests::tiny();
        let config = RoutingConfig {
            min_layer: Some("M1".to_string()),
            ..RoutingConfig::default()
        };
        let mut tech = RouteTech::new(&db, &config, LeafClockPins::default()).unwrap();
        let m1 = &mut tech.layers[LayerId::new(0)];
        m1.spacing = m1.spacing * spacing_halves / 2;
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(NetId(0)),
            costs: Costs {
                via: 50.0,
                conflict: 8.0,
                marker: 32.0,
                decay: 0.95,
            },
            guide: None,
            own_vias: None,
            ripped: &[],
        };
        f(&search)
    }

    #[test]
    fn a_jog_on_m1_can_be_crowded_only_once_its_spacing_reaches_past_a_crossing() {
        let farthest = |search: &Search<'_>| {
            let m1 = LayerId::new(0);
            let crosses = search
                .tech
                .layer(m1)
                .cross
                .iter_enumerated()
                .map(|(c, _)| c);
            farthest_jog_crowding(search, m1, crosses)
        };

        let (plain, wide) = (with_m1_spacing(2, farthest), with_m1_spacing(3, farthest));

        assert_eq!((plain, wide), (None, Some(1)));
    }

    #[test]
    fn a_state_a_step_past_a_recent_jog_gets_its_own_slot() {
        let numbering = with_m1_spacing(2, |search| Numbering::new(search, &search.db.die_area));
        let node = Node {
            layer: LayerId::new(0),
            track: TrackIdx(2),
            cross: CrossIdx(3),
        };
        let past = |recent_jog| State {
            node,
            heading: Heading::Up,
            recent_via: None,
            recent_jog,
        };

        let slots = [None, Some(CrossIdx(2))].map(|j| numbering.slot(past(j)));

        assert!(slots.iter().all(Option::is_some) && slots[0] != slots[1]);
    }

    #[test]
    fn steps_within_counts_the_longest_run_closer_than_the_reach() {
        let at = [
            Dbu::new(0),
            Dbu::new(10),
            Dbu::new(20),
            Dbu::new(50),
            Dbu::new(55),
        ];

        assert_eq!(steps_within(&at, Dbu::new(21)), 2);
        assert_eq!(steps_within(&at, Dbu::new(20)), 1);
        assert_eq!(steps_within(&at, Dbu::new(1)), 0);
    }
}
