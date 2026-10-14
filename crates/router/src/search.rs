//! The A* search over track crossings and what it costs.
//!
//! Costs are wirelength, a fixed via cost, a penalty for every occupant of
//! another net the new metal would short or crowd, and accumulated conflict
//! history. Obstacles cost so much that a route through one exists only
//! when nothing else does. A guide limits the search to gcells near a net's
//! global route.

use crate::access::{AccessPoint, Node};
use crate::global::GCell;
use crate::occupancy::{LocalOccupancy, Occupancy, Owner, TrackRef};
use crate::tech::RouteTech;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::LayerId;
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::shapes::wire_rect;
use rustc_hash::FxHashMap;
use std::collections::BinaryHeap;

/// Cost of crossing an obstacle, in DBU of wirelength.
const OBSTACLE_COST: f64 = 1e9;

/// Heuristic weight of the first, near-optimal search of a connection.
const FOCUSED_WEIGHT: f64 = 1.0;

/// Search effort before the near-optimal search gives way to a greedy one.
const FOCUSED_EXPANSIONS: usize = 2_000_000;

/// Heuristic weight of the greedy search that guarantees a path quickly.
const GREEDY_WEIGHT: f64 = 4.0;

/// Search effort before a connection gives up in one window.
const MAX_EXPANSIONS: usize = 4_000_000;

/// Tunable costs of the search.
#[derive(Clone, Copy, Debug)]
pub struct Costs {
    /// Cost of one via, in DBU of wirelength.
    pub via: f64,
    /// Cost per conflicting occupant this round.
    pub penalty: f64,
    /// Cost per unit of history at a location.
    pub history: f64,
}

/// Accumulated conflict history, by track and coarse position along it.
#[derive(Debug, Default)]
pub struct History {
    bucket: Dbu,
    map: FxHashMap<(TrackRef, i64), f64>,
}

impl History {
    pub(crate) fn new(bucket: Dbu) -> Self {
        Self {
            bucket: bucket.max(Dbu(1)),
            map: FxHashMap::default(),
        }
    }

    const fn key(&self, t: TrackRef, along: Dbu) -> (TrackRef, i64) {
        (t, along.get().div_euclid(self.bucket.get()))
    }

    /// Total history of the buckets `[lo, hi]` overlaps on a track.
    fn over(&self, t: TrackRef, lo: Dbu, hi: Dbu) -> f64 {
        if self.map.is_empty() {
            return 0.0;
        }
        let (_, first) = self.key(t, lo);
        let (_, last) = self.key(t, hi);
        (first..=last)
            .map(|b| self.map.get(&(t, b)).copied().unwrap_or(0.0))
            .sum()
    }

    pub(crate) fn bump(&mut self, t: TrackRef, along: Dbu, amount: f64) {
        *self.map.entry(self.key(t, along)).or_insert(0.0) += amount;
    }
}

/// How far a net's detailed route may stray from its global route. It grows
/// each time the net is ripped up, so a corridor that other nets fill
/// cannot trap it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Slack {
    /// Gcells around each guide cell, in the plane.
    pub(crate) gcells: i64,
    /// Layers above and below each guide cell.
    pub(crate) layers: usize,
}

impl Slack {
    pub(crate) const fn after_rip_ups(count: u8) -> Self {
        match count {
            0 | 1 => Self {
                gcells: 1,
                layers: 1,
            },
            2 | 3 => Self {
                gcells: 2,
                layers: 2,
            },
            _ => Self {
                gcells: 4,
                layers: usize::MAX,
            },
        }
    }
}

/// The gcells a net's detailed route may use: its global route grown by a
/// [`Slack`].
#[derive(Debug)]
pub(crate) struct Guide {
    origin: Point<Dbu>,
    gcell: Dbu,
    reach: i64,
    x0: i64,
    y0: i64,
    width: i64,
    height: i64,
    min_layer: usize,
    layers: usize,
    bits: Vec<u64>,
}

impl Guide {
    pub(crate) fn new(
        cells: &[GCell],
        origin: Point<Dbu>,
        gcell: Dbu,
        tech: &RouteTech,
        slack: Slack,
    ) -> Self {
        let reach = slack.gcells;
        let x0 = cells.iter().map(|c| i64::from(c.x)).min().unwrap_or(0) - reach;
        let y0 = cells.iter().map(|c| i64::from(c.y)).min().unwrap_or(0) - reach;
        let x1 = cells.iter().map(|c| i64::from(c.x)).max().unwrap_or(0) + reach;
        let y1 = cells.iter().map(|c| i64::from(c.y)).max().unwrap_or(0) + reach;
        let min_layer = tech.min_layer.index();
        let layers = tech.max_layer.index() - min_layer + 1;
        let (width, height) = (x1 - x0 + 1, y1 - y0 + 1);
        let total = usize::try_from(width * height).unwrap_or(0) * layers;
        let mut guide = Self {
            origin,
            gcell,
            reach,
            x0,
            y0,
            width,
            height,
            min_layer,
            layers,
            bits: vec![0; total.div_ceil(64)],
        };
        for c in cells {
            let l = c.layer.index() - min_layer;
            for dl in
                l.saturating_sub(slack.layers)..=l.saturating_add(slack.layers).min(layers - 1)
            {
                for dy in -reach..=reach {
                    for dx in -reach..=reach {
                        if let Some(i) = guide.index(dl, i64::from(c.x) + dx, i64::from(c.y) + dy) {
                            guide.bits[i / 64] |= 1 << (i % 64);
                        }
                    }
                }
            }
        }
        guide
    }

    /// The extent of one gcell.
    pub(crate) fn cell_rect(&self, cell: GCell) -> Rect<Dbu> {
        let min = self.origin
            + Point::new(
                self.gcell * i64::from(cell.x),
                self.gcell * i64::from(cell.y),
            );
        Rect::new(min, min + Point::new(self.gcell, self.gcell))
    }

    /// The area this guide allows around `cells`.
    pub(crate) fn area(&self, cells: &[GCell]) -> Option<Rect<Dbu>> {
        cells
            .iter()
            .map(|&c| self.cell_rect(c))
            .reduce(|a, b| a.union(&b))
            .map(|r| r.expand(self.gcell * self.reach))
    }

    fn index(&self, layer: usize, x: i64, y: i64) -> Option<usize> {
        let (lx, ly) = (x - self.x0, y - self.y0);
        if layer >= self.layers || lx < 0 || ly < 0 || lx >= self.width || ly >= self.height {
            return None;
        }
        usize::try_from((layer as i64 * self.height + ly) * self.width + lx).ok()
    }

    pub(crate) fn contains(&self, layer: LayerId, p: Point<Dbu>) -> bool {
        let Some(l) = layer.index().checked_sub(self.min_layer) else {
            return false;
        };
        let x = (p.x - self.origin.x).get().div_euclid(self.gcell.get());
        let y = (p.y - self.origin.y).get().div_euclid(self.gcell.get());
        self.index(l, x, y)
            .is_some_and(|i| self.bits[i / 64] & (1 << (i % 64)) != 0)
    }
}

/// One A* search context.
pub(crate) struct Search<'a> {
    pub(crate) db: &'a NetlistDB,
    pub(crate) tech: &'a RouteTech,
    pub(crate) occupancy: &'a Occupancy,
    pub(crate) overlay: Option<&'a LocalOccupancy>,
    pub(crate) history: &'a History,
    pub(crate) owner: Owner,
    pub(crate) costs: Costs,
    pub(crate) guide: Option<&'a Guide>,
}

/// A position in the search: a node, the way the path moves along the
/// node's track, and the via the path last took if it is still within cut
/// spacing, since a via back through the same cut there would break that
/// spacing. Paths never turn back along a track, so once one moves out of
/// a via's reach it cannot return to crowd it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct State {
    node: Node,
    heading: Heading,
    recent_via: Option<RecentVia>,
}

/// Which way a path runs along its current track.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
enum Heading {
    /// It has not moved along the track yet.
    Still,
    /// Toward higher crossings.
    Up,
    /// Toward lower crossings.
    Down,
}

/// A via the path took onto its current layer, close enough behind that
/// the cut it used is still crowded.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct RecentVia {
    /// The layer the via came from.
    from: LayerId,
    /// Where it landed, along the current layer's tracks.
    at: Dbu,
}

impl State {
    const fn start(node: Node) -> Self {
        Self {
            node,
            heading: Heading::Still,
            recent_via: None,
        }
    }
}

/// A search state's priority, smallest estimated total cost first.
#[derive(Clone, Copy, PartialEq)]
struct Entry {
    f: f64,
    g: f64,
    state: State,
}

impl Eq for Entry {}

impl Ord for Entry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .f
            .total_cmp(&self.f)
            .then_with(|| self.g.total_cmp(&other.g))
            .then_with(|| self.state.cmp(&other.state))
    }
}

impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Search<'_> {
    fn point(&self, n: Node) -> Point<Dbu> {
        self.tech.layer(n.layer).point(n.track, n.cross)
    }

    /// Cost of using an access point: its stub, its vias, and any conflicts
    /// their metal would have on the routable layers.
    pub(crate) fn access_cost(&self, a: &AccessPoint) -> f64 {
        let node = self.point(a.node);
        let mut cost = if node == a.at {
            0.0
        } else {
            self.wire_cost(a.node.layer, node, a.at)
        };
        for &via in &a.vias {
            cost += self.costs.via;
            for s in &self.db.library.via(via).shapes {
                if let LayerRef::Routing(l) = s.layer
                    && self.tech.routable(l)
                {
                    cost += self.conflict_cost(l, &s.rect.translate(a.at));
                }
            }
        }
        cost
    }

    /// Cost of other owners' metal that `rect` on `layer` would conflict with.
    /// Cost of other owners' metal that `rect` on `layer` would conflict
    /// with, plus the conflict history of the tracks it runs on.
    fn conflict_cost(&self, layer: LayerId, rect: &Rect<Dbu>) -> f64 {
        let mut cost = 0.0;
        for (_, o) in self
            .occupancy
            .rect_conflicts(self.tech, layer, rect, self.owner)
        {
            cost += match o.owner {
                Owner::Obstacle => OBSTACLE_COST,
                Owner::Net(_) => self.costs.penalty,
            };
        }
        if let Some(overlay) = self.overlay {
            let clashes = overlay
                .rect_conflicts(self.tech, layer, rect, self.owner)
                .count();
            cost = self.costs.penalty.mul_add(clashes as f64, cost);
        }
        let lt = self.tech.layer(layer);
        let ((a0, a1), (b0, b1)) = lt.across_along_rect(rect);
        for track in lt.tracks_within(a0, a1) {
            let t = TrackRef { layer, track };
            cost = self
                .costs
                .history
                .mul_add(self.history.over(t, b0, b1), cost);
        }
        cost
    }

    fn wire_cost(&self, layer: LayerId, a: Point<Dbu>, b: Point<Dbu>) -> f64 {
        let length = ((a.x - b.x).abs() + (a.y - b.y).abs()).to_f64();
        let width = self.tech.layer(layer).width;
        let conflict = wire_rect(a, b, width).map_or(0.0, |r| self.conflict_cost(layer, &r));
        length + conflict
    }

    fn via_cost(&self, bottom: LayerId, at: Point<Dbu>) -> f64 {
        let via = self.tech.via_above(bottom);
        let top = LayerId::new(bottom.index() + 1);
        self.costs.via
            + self.conflict_cost(bottom, &via.bottom.translate(at))
            + self.conflict_cost(top, &via.top.translate(at))
    }

    pub(crate) fn allowed(&self, layer: LayerId, p: Point<Dbu>) -> bool {
        self.guide.is_none_or(|g| g.contains(layer, p))
    }

    /// Center distance below which two vias between `a` and `b` crowd each
    /// other's cuts.
    fn cut_reach(&self, a: LayerId, b: LayerId) -> Dbu {
        self.tech.via_above(a.min(b)).cut_reach
    }

    /// The recent via of a path that steps along its track to `along`.
    fn after_step(
        &self,
        layer: LayerId,
        recent: Option<RecentVia>,
        along: Dbu,
    ) -> Option<RecentVia> {
        recent.filter(|v| (along - v.at).abs() < self.cut_reach(layer, v.from))
    }

    /// Whether a via from `state` to `to` would crowd the cut of the via the
    /// path just came through.
    fn crowds_recent_via(&self, state: State, to: LayerId, along: Dbu) -> bool {
        state.recent_via.is_some_and(|v| {
            v.from == to && (along - v.at).abs() < self.cut_reach(state.node.layer, to)
        })
    }

    /// States reachable from `state` in one step, with the step's cost.
    fn neighbours(&self, state: State, window: &Rect<Dbu>, out: &mut Vec<(State, f64)>) {
        out.clear();
        let n = state.node;
        let lt = self.tech.layer(n.layer);
        let here = lt.point(n.track, n.cross);
        let steps = [
            (n.cross.checked_sub(1), Heading::Down),
            (Some(n.cross + 1), Heading::Up),
        ];
        for (next, heading) in steps {
            let turns_back = matches!(
                (state.heading, heading),
                (Heading::Up, Heading::Down) | (Heading::Down, Heading::Up)
            );
            let Some(next) = next.filter(|_| !turns_back) else {
                continue;
            };
            if (next as usize) < lt.cross.len() {
                let m = Node { cross: next, ..n };
                let p = lt.point(m.track, m.cross);
                if window.contains(p) && self.allowed(m.layer, p) {
                    let moved = State {
                        node: m,
                        heading,
                        recent_via: self.after_step(
                            n.layer,
                            state.recent_via,
                            lt.cross[next as usize].at,
                        ),
                    };
                    out.push((moved, self.wire_cost(n.layer, here, p)));
                }
            }
        }
        let cross = lt.cross[n.cross as usize];
        let up = LayerId::new(n.layer.index() + 1);
        if n.layer < self.tech.max_layer
            && self.allowed(up, here)
            && !self.crowds_recent_via(state, up, cross.at)
            && let (Some(track), Some(c)) = (cross.up, lt.track_up[n.track as usize])
        {
            let m = Node {
                layer: up,
                track,
                cross: c,
            };
            out.push((self.via_state(m, n.layer), self.via_cost(n.layer, here)));
        }
        if n.layer > self.tech.min_layer {
            let below = LayerId::new(n.layer.index() - 1);
            if self.allowed(below, here)
                && !self.crowds_recent_via(state, below, cross.at)
                && let (Some(track), Some(c)) = (cross.down, lt.track_down[n.track as usize])
            {
                let m = Node {
                    layer: below,
                    track,
                    cross: c,
                };
                out.push((self.via_state(m, n.layer), self.via_cost(below, here)));
            }
        }
    }

    /// The state on `node` just after a via from `from`.
    fn via_state(&self, node: Node, from: LayerId) -> State {
        let lt = self.tech.layer(node.layer);
        State {
            node,
            heading: Heading::Still,
            recent_via: Some(RecentVia {
                from,
                at: lt.cross[node.cross as usize].at,
            }),
        }
    }

    /// Cheapest path from any source to any target; returns the nodes from
    /// source to target and the index of the target's access point.
    pub(crate) fn find(
        &self,
        sources: &[(Node, f64)],
        targets: &FxHashMap<Node, (usize, f64)>,
        window: &Rect<Dbu>,
    ) -> Option<(Vec<Node>, usize)> {
        self.find_weighted(sources, targets, window, FOCUSED_WEIGHT, FOCUSED_EXPANSIONS)
            .or_else(|| self.find_weighted(sources, targets, window, GREEDY_WEIGHT, MAX_EXPANSIONS))
    }

    /// A* with the heuristic scaled by `weight`, giving up after `limit`
    /// expansions unless a target has been reached.
    fn find_weighted(
        &self,
        sources: &[(Node, f64)],
        targets: &FxHashMap<Node, (usize, f64)>,
        window: &Rect<Dbu>,
        weight: f64,
        limit: usize,
    ) -> Option<(Vec<Node>, usize)> {
        let goal = targets
            .keys()
            .map(|&n| {
                let p = self.point(n);
                Rect::new(p, p)
            })
            .reduce(|a, b| a.union(&b))?;
        let goal_layers: Vec<usize> = {
            let mut l: Vec<usize> = targets.keys().map(|n| n.layer.index()).collect();
            l.sort_unstable();
            l.dedup();
            l
        };
        let h = |n: Node| {
            let layers = goal_layers
                .iter()
                .map(|&l| l.abs_diff(n.layer.index()))
                .min()
                .unwrap_or(0) as f64;
            let plane = distance_to(&goal, self.point(n)) as f64;
            weight * self.costs.via.mul_add(layers, plane)
        };

        let mut best: FxHashMap<State, (f64, Option<State>)> = FxHashMap::default();
        let mut heap = BinaryHeap::new();
        for &(n, g) in sources {
            let state = State::start(n);
            if best.get(&state).is_none_or(|&(b, _)| g < b) {
                let _ = best.insert(state, (g, None));
                heap.push(Entry {
                    f: g + h(n),
                    g,
                    state,
                });
            }
        }
        let mut finish: Option<(f64, State, usize)> = None;
        let mut buf = Vec::new();
        let mut expansions = 0;
        while let Some(Entry { f, g, state }) = heap.pop() {
            if finish.is_some_and(|(cost, _, _)| f >= cost) {
                break;
            }
            if best.get(&state).is_some_and(|&(b, _)| g > b) {
                continue;
            }
            if let Some(&(k, extra)) = targets.get(&state.node)
                && finish.is_none_or(|(cost, _, _)| g + extra < cost)
            {
                finish = Some((g + extra, state, k));
            }
            expansions += 1;
            if expansions > limit {
                break;
            }
            self.neighbours(state, window, &mut buf);
            for &(m, step) in &buf {
                let ng = g + step;
                if best.get(&m).is_none_or(|&(b, _)| ng < b) {
                    let _ = best.insert(m, (ng, Some(state)));
                    heap.push(Entry {
                        f: ng + h(m.node),
                        g: ng,
                        state: m,
                    });
                }
            }
        }
        let (_, end, k) = finish?;
        let mut path = vec![end.node];
        let mut cur = end;
        while let Some(&(_, Some(prev))) = best.get(&cur) {
            path.push(prev.node);
            cur = prev;
        }
        path.reverse();
        Some((path, k))
    }
}

/// Manhattan distance from a point to a rectangle.
fn distance_to(r: &Rect<Dbu>, p: Point<Dbu>) -> i64 {
    let dx = (r.min.x - p.x).max(p.x - r.max.x).max(Dbu::ZERO);
    let dy = (r.min.y - p.y).max(p.y - r.max.y).max(Dbu::ZERO);
    (dx + dy).get()
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::parser::{def, lef};
    use pare_common::util::config::RoutingConfig;
    use std::path::{Path, PathBuf};

    fn fixture(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../common/tests/fixtures")
            .join(name)
    }

    fn tiny() -> NetlistDB {
        let units = def::read_units(&fixture("tiny.def")).unwrap();
        let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
        def::parse(&fixture("tiny.def"), library).unwrap()
    }

    /// Runs on one track between two vias through the same cut, as the run's
    /// length and the reach of that cut.
    fn returns_through_a_cut(tech: &RouteTech, path: &[Node]) -> Vec<(Dbu, Dbu)> {
        let mut out = Vec::new();
        let mut i = 1;
        while i < path.len() {
            let (from, onto) = (path[i - 1], path[i]);
            if from.layer == onto.layer {
                i += 1;
                continue;
            }
            let mut j = i;
            while j + 1 < path.len()
                && path[j + 1].layer == onto.layer
                && path[j + 1].track == onto.track
            {
                j += 1;
            }
            if path.get(j + 1).is_some_and(|n| n.layer == from.layer) {
                let lt = tech.layer(onto.layer);
                let run =
                    (lt.cross[path[j].cross as usize].at - lt.cross[onto.cross as usize].at).abs();
                out.push((run, tech.via_above(from.layer.min(onto.layer)).cut_reach));
            }
            i = j + 1;
        }
        out
    }

    #[test]
    fn a_path_leaves_a_cut_crowded_by_its_last_via_before_coming_back_through_it() {
        let db = tiny();
        let config = RoutingConfig {
            min_layer: Some("M1".to_string()),
            ..RoutingConfig::default()
        };
        let tech = RouteTech::new(&db, &config).unwrap();
        let occupancy = Occupancy::new(&tech, Dbu(1000));
        let history = History::new(Dbu(1000));
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            history: &history,
            owner: Owner::Net(0),
            costs: Costs {
                via: 50.0,
                penalty: 1000.0,
                history: 0.0,
            },
            guide: None,
        };
        let m1 = LayerId(0);
        let lt = tech.layer(m1);
        let node = |y: i64| Node {
            layer: m1,
            track: lt.tracks.binary_search(&Dbu(y)).unwrap() as u32,
            cross: lt.cross.partition_point(|c| c.at < Dbu(1100)) as u32,
        };
        let targets: FxHashMap<Node, (usize, f64)> =
            std::iter::once((node(300), (0, 0.0))).collect();

        let (path, _) = search
            .find(&[(node(100), 0.0)], &targets, &db.die_area)
            .unwrap();

        assert_eq!(path.last(), Some(&node(300)));
        for (run, reach) in returns_through_a_cut(&tech, &path) {
            assert!(
                run >= reach,
                "a via returns after {run} of a {reach} reach: {path:?}"
            );
        }
    }
}
