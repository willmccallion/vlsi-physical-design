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

/// A search state's priority, smallest estimated total cost first.
#[derive(Clone, Copy, PartialEq)]
struct Entry {
    f: f64,
    g: f64,
    node: Node,
}

impl Eq for Entry {}

impl Ord for Entry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .f
            .total_cmp(&self.f)
            .then_with(|| self.g.total_cmp(&other.g))
            .then_with(|| self.node.cmp(&other.node))
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

    /// Whether a via from `n` to `to_layer` would land within cut spacing of
    /// the via this path took one short step earlier on the same cut.
    fn crowds_previous_via(
        &self,
        node: Node,
        to_layer: LayerId,
        parents: &FxHashMap<Node, (f64, Option<Node>)>,
    ) -> bool {
        let parent = |x: Node| parents.get(&x).and_then(|&(_, p)| p);
        let Some(prev) = parent(node) else {
            return false;
        };
        if prev.layer != node.layer || prev.track != node.track {
            return false;
        }
        let Some(before) = parent(prev) else {
            return false;
        };
        if before.layer != to_layer {
            return false;
        }
        let lower = node.layer.min(to_layer);
        let (a, b) = (self.point(node), self.point(prev));
        let gap = (a.x - b.x).abs().max((a.y - b.y).abs());
        gap < self.tech.via_above(lower).cut_reach
    }

    /// Neighbours of `n` with their step cost.
    fn neighbours(
        &self,
        n: Node,
        window: &Rect<Dbu>,
        parents: &FxHashMap<Node, (f64, Option<Node>)>,
        out: &mut Vec<(Node, f64)>,
    ) {
        out.clear();
        let lt = self.tech.layer(n.layer);
        let here = lt.point(n.track, n.cross);
        for next in [n.cross.checked_sub(1), Some(n.cross + 1)]
            .into_iter()
            .flatten()
        {
            if (next as usize) < lt.cross.len() {
                let m = Node { cross: next, ..n };
                let p = lt.point(m.track, m.cross);
                if window.contains(p) && self.allowed(m.layer, p) {
                    out.push((m, self.wire_cost(n.layer, here, p)));
                }
            }
        }
        let cross = lt.cross[n.cross as usize];
        let up = LayerId::new(n.layer.index() + 1);
        if n.layer < self.tech.max_layer
            && self.allowed(up, here)
            && !self.crowds_previous_via(n, up, parents)
            && let (Some(track), Some(c)) = (cross.up, lt.track_up[n.track as usize])
        {
            let m = Node {
                layer: LayerId::new(n.layer.index() + 1),
                track,
                cross: c,
            };
            out.push((m, self.via_cost(n.layer, here)));
        }
        if n.layer > self.tech.min_layer
            && self.allowed(LayerId::new(n.layer.index() - 1), here)
            && !self.crowds_previous_via(n, LayerId::new(n.layer.index() - 1), parents)
            && let (Some(track), Some(c)) = (cross.down, lt.track_down[n.track as usize])
        {
            let below = LayerId::new(n.layer.index() - 1);
            let m = Node {
                layer: below,
                track,
                cross: c,
            };
            out.push((m, self.via_cost(below, here)));
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

        let mut best: FxHashMap<Node, (f64, Option<Node>)> = FxHashMap::default();
        let mut heap = BinaryHeap::new();
        for &(n, g) in sources {
            if best.get(&n).is_none_or(|&(b, _)| g < b) {
                let _ = best.insert(n, (g, None));
                heap.push(Entry {
                    f: g + h(n),
                    g,
                    node: n,
                });
            }
        }
        let mut finish: Option<(f64, Node, usize)> = None;
        let mut buf = Vec::new();
        let mut expansions = 0;
        while let Some(Entry { f, g, node }) = heap.pop() {
            if finish.is_some_and(|(cost, _, _)| f >= cost) {
                break;
            }
            if best.get(&node).is_some_and(|&(b, _)| g > b) {
                continue;
            }
            if let Some(&(k, extra)) = targets.get(&node)
                && finish.is_none_or(|(cost, _, _)| g + extra < cost)
            {
                finish = Some((g + extra, node, k));
            }
            expansions += 1;
            if expansions > limit {
                break;
            }
            self.neighbours(node, window, &best, &mut buf);
            for &(m, step) in &buf {
                let ng = g + step;
                if best.get(&m).is_none_or(|&(b, _)| ng < b) {
                    let _ = best.insert(m, (ng, Some(node)));
                    heap.push(Entry {
                        f: ng + h(m),
                        g: ng,
                        node: m,
                    });
                }
            }
        }
        let (_, end, k) = finish?;
        let mut path = vec![end];
        let mut cur = end;
        while let Some(&(_, Some(prev))) = best.get(&cur) {
            path.push(prev);
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
