//! The A* search over track crossings and what it costs.
//!
//! Costs are wirelength, a fixed via cost, a penalty for every piece of
//! another net's wiring the new metal or cuts would short or crowd, and
//! accumulated conflict history. Obstacles, and other nets' pins and
//! reserved pin access, never move, so they cost so much that a route
//! through one exists only when nothing else does. A guide limits the
//! search to gcells near a net's global route.

mod dense;

use crate::access::{AccessPoint, Node};
use crate::global::GCell;
use crate::occupancy::{LocalOccupancy, Occupancy, Origin, Owner, Proximity, TrackRef, too_close};
use crate::tech::RouteTech;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{CutLayerId, LayerId, NonDefaultRuleId, ViaId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::shapes::wire_rect;
use rustc_hash::FxHashMap;
use std::collections::BinaryHeap;

/// Cost of crossing an obstacle or another net's fixed metal, in DBU of
/// wirelength.
const OBSTACLE_COST: f64 = 1e9;

/// Highest conflict penalty, far below the cost of an obstacle so that a
/// route never prefers fixed metal to wiring that can move.
pub(crate) const MAX_PENALTY: f64 = OBSTACLE_COST * 1e-3;

/// How hard one A* run tries.
#[derive(Clone, Copy, Debug)]
struct Effort {
    /// Scale of the heuristic; above one trades optimality for speed.
    weight: f64,
    /// Expansions before giving up unless a target has been reached.
    limit: usize,
}

/// The first, near-optimal search of a connection.
const FOCUSED: Effort = Effort {
    weight: 1.0,
    limit: 2_000_000,
};

/// The greedy search that finds a path quickly when the focused one gives
/// up; past its limit the connection gives up in this window.
const GREEDY: Effort = Effort {
    weight: 4.0,
    limit: 4_000_000,
};

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

    /// Every bucket's amount, by track and bucket.
    pub(crate) fn entries(&self) -> Vec<(TrackRef, i64, f64)> {
        let mut out: Vec<(TrackRef, i64, f64)> =
            self.map.iter().map(|(&(t, b), &a)| (t, b, a)).collect();
        out.sort_unstable_by_key(|&(t, b, _)| (t, b));
        out
    }

    /// Sets one bucket's amount, as [`Self::entries`] gave it.
    pub(crate) fn set(&mut self, t: TrackRef, bucket: i64, amount: f64) {
        let _ = self.map.insert((t, bucket), amount);
    }
}

/// Gcells around each global route cell a net's detailed route may use.
pub(crate) const GUIDE_SLACK_GCELLS: i64 = 1;

/// Layers above and below each global route cell a net's detailed route
/// may use.
const GUIDE_SLACK_LAYERS: usize = 1;

/// The gcells a net's detailed route may use: its global route grown by
/// [`GUIDE_SLACK_GCELLS`] and [`GUIDE_SLACK_LAYERS`].
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
    pub(crate) fn new(cells: &[GCell], origin: Point<Dbu>, gcell: Dbu, tech: &RouteTech) -> Self {
        let reach = GUIDE_SLACK_GCELLS;
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
            for dl in l.saturating_sub(GUIDE_SLACK_LAYERS)
                ..=l.saturating_add(GUIDE_SLACK_LAYERS).min(layers - 1)
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

/// A via and where it is placed.
type PlacedVia = (ViaId, Point<Dbu>);

/// The vias a net already has, which its new vias must not crowd.
#[derive(Clone, Debug)]
pub(crate) struct OwnVias {
    bucket: Dbu,
    /// The non-default rule the net follows.
    rule: Option<NonDefaultRuleId>,
    vias: FxHashMap<(i64, i64), Vec<PlacedVia>>,
}

impl OwnVias {
    /// An empty set for a net under `rule`, bucketed so that any via that
    /// could crowd another lies in a neighbouring bucket.
    pub(crate) fn new(db: &NetlistDB, tech: &RouteTech, rule: Option<NonDefaultRuleId>) -> Self {
        let lib = &db.library;
        let widest = lib
            .vias
            .iter()
            .flat_map(|v| v.shapes.iter())
            .map(|s| s.rect.width().max(s.rect.height()))
            .max()
            .unwrap_or_default();
        let spacing = (0..lib.routing_layers.len())
            .map(|l| tech.wire_spacing(rule, LayerId::new(l)))
            .chain(lib.cut_layers.iter().filter_map(|c| c.spacing))
            .max()
            .unwrap_or_default();
        Self {
            bucket: (widest + spacing).max(Dbu(1)),
            rule,
            vias: FxHashMap::default(),
        }
    }

    const fn key(&self, p: Point<Dbu>) -> (i64, i64) {
        (
            p.x.get().div_euclid(self.bucket.get()),
            p.y.get().div_euclid(self.bucket.get()),
        )
    }

    pub(crate) fn add(&mut self, via: ViaId, at: Point<Dbu>) {
        self.vias.entry(self.key(at)).or_default().push((via, at));
    }

    /// How many of the vias `via` at `at` would crowd: their cuts or metal
    /// on a shared layer neither touch nor keep the spacing.
    pub(crate) fn crowded(
        &self,
        db: &NetlistDB,
        tech: &RouteTech,
        via: ViaId,
        at: Point<Dbu>,
    ) -> usize {
        let (x, y) = self.key(at);
        let shapes = &db.library.via(via).shapes;
        let spacing = |layer: LayerRef| match layer {
            LayerRef::Routing(l) => tech.wire_spacing(self.rule, l),
            LayerRef::Cut(c) => db.library.cut_layers[c.index()].spacing.unwrap_or_default(),
        };
        (x - 1..=x + 1)
            .flat_map(|bx| (y - 1..=y + 1).map(move |by| (bx, by)))
            .filter_map(|k| self.vias.get(&k))
            .flatten()
            .filter(|&&(other, p)| {
                let theirs = &db.library.via(other).shapes;
                shapes.iter().any(|a| {
                    theirs.iter().any(|b| {
                        a.layer == b.layer
                            && too_close(
                                spacing(a.layer),
                                &a.rect.translate(at),
                                &b.rect.translate(p),
                            )
                    })
                })
            })
            .count()
    }
}

/// One A* search context.
#[derive(Clone, Copy)]
pub(crate) struct Search<'a> {
    pub(crate) db: &'a NetlistDB,
    pub(crate) tech: &'a RouteTech,
    pub(crate) occupancy: &'a Occupancy,
    pub(crate) overlay: Option<&'a LocalOccupancy>,
    pub(crate) history: &'a History,
    pub(crate) owner: Owner,
    pub(crate) costs: Costs,
    pub(crate) guide: Option<&'a Guide>,
    /// Vias the net already has, if new vias should keep clear of them.
    pub(crate) own_vias: Option<&'a OwnVias>,
    /// Nets the worker is routing again whose wiring in the shared
    /// occupancy therefore does not count; what they keep near the worker
    /// is in the overlay.
    pub(crate) ripped: &'a [Owner],
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
    /// The crossing it landed on, along the current track.
    cross: u32,
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

    /// The non-default rule the net being routed follows.
    pub(crate) fn rule(&self) -> Option<NonDefaultRuleId> {
        match self.owner {
            Owner::Net(n) => self.tech.net_rule(n as usize),
            Owner::Obstacle => None,
        }
    }

    /// The width of the net's wires on `layer`.
    pub(crate) fn width(&self, layer: LayerId) -> Dbu {
        self.tech.wire_width(self.rule(), layer)
    }

    /// The spacing the net's metal on `layer` keeps from other metal.
    fn spacing(&self, layer: LayerId) -> Dbu {
        self.tech.wire_spacing(self.rule(), layer)
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
                let rect = s.rect.translate(a.at);
                match s.layer {
                    LayerRef::Routing(l) if self.tech.routable(l) => {
                        cost += self.conflict_cost(l, &rect);
                    }
                    LayerRef::Routing(_) => {}
                    LayerRef::Cut(c) => cost += self.cut_cost(c, &rect),
                }
            }
        }
        cost
    }

    /// Cost of conflicting with one occupant of the shared occupancy.
    fn occupant_cost(&self, owner: Owner, origin: Origin) -> f64 {
        match (owner, origin) {
            (Owner::Obstacle, _) | (Owner::Net(_), Origin::Fixed) => OBSTACLE_COST,
            (Owner::Net(_), Origin::Wiring) if self.ripped.contains(&owner) => 0.0,
            (Owner::Net(_), Origin::Wiring) => self.costs.penalty,
        }
    }

    /// Cost of other owners' cuts that a cut at `rect` on `cut` would touch
    /// or crowd.
    fn cut_cost(&self, cut: CutLayerId, rect: &Rect<Dbu>) -> f64 {
        let mut cost = 0.0;
        for o in self.occupancy.cut_conflicts(cut, rect, self.owner) {
            cost += self.occupant_cost(o.owner, o.origin);
        }
        if let Some(overlay) = self.overlay {
            let clashes = overlay.cut_conflicts(cut, rect, self.owner).count();
            cost = self.costs.penalty.mul_add(clashes as f64, cost);
        }
        cost
    }

    /// Cost of other owners' metal that a whole shape `rect` on `layer`
    /// would conflict with, plus the conflict history of the tracks it runs
    /// on.
    pub(crate) fn conflict_cost(&self, layer: LayerId, rect: &Rect<Dbu>) -> f64 {
        self.metal_cost(layer, rect, 0.0)
    }

    /// [`Self::conflict_cost`] for one step of a wire, which also pays
    /// `alongside` for each shape it would violate the spacing of should
    /// the wire go on alongside it, as `TritonRoute` prices fixed shapes.
    fn step_conflict_cost(&self, layer: LayerId, rect: &Rect<Dbu>) -> f64 {
        self.metal_cost(layer, rect, self.costs.penalty)
    }

    fn metal_cost(&self, layer: LayerId, rect: &Rect<Dbu>, alongside: f64) -> f64 {
        let mut cost = 0.0;
        for c in self
            .occupancy
            .rect_conflicts(self.tech, layer, rect, self.owner)
        {
            cost += match c.proximity {
                Proximity::Within => self.occupant_cost(c.occupant.owner, c.occupant.origin),
                Proximity::Alongside => alongside,
            };
        }
        if let Some(overlay) = self.overlay {
            for c in overlay.rect_conflicts(self.tech, layer, rect, self.owner) {
                cost += match c.proximity {
                    Proximity::Within => self.costs.penalty,
                    Proximity::Alongside => alongside,
                };
            }
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

    /// Cost of a whole wire from `a` to `b`: its length and its conflicts.
    fn wire_cost(&self, layer: LayerId, a: Point<Dbu>, b: Point<Dbu>) -> f64 {
        let length = ((a.x - b.x).abs() + (a.y - b.y).abs()).to_f64();
        let conflict =
            wire_rect(a, b, self.width(layer)).map_or(0.0, |r| self.conflict_cost(layer, &r));
        length + conflict
    }

    /// Cost of one step of a wire from `a` to `b`, which may go on alongside
    /// what it meets.
    fn step_cost(&self, layer: LayerId, a: Point<Dbu>, b: Point<Dbu>) -> f64 {
        let length = ((a.x - b.x).abs() + (a.y - b.y).abs()).to_f64();
        let conflict =
            wire_rect(a, b, self.width(layer)).map_or(0.0, |r| self.step_conflict_cost(layer, &r));
        length + conflict
    }

    /// Cost of a via up from `bottom` at `at`, apart from crowding the
    /// net's own vias.
    fn via_cost(&self, bottom: LayerId, at: Point<Dbu>) -> f64 {
        let via = self.tech.via_above(bottom);
        let top = LayerId::new(bottom.index() + 1);
        let cuts: f64 = via
            .cuts
            .iter()
            .map(|c| self.cut_cost(bottom.cut_above(), &c.translate(at)))
            .sum();
        self.costs.via
            + self.conflict_cost(bottom, &via.bottom.translate(at))
            + self.conflict_cost(top, &via.top.translate(at))
            + cuts
    }

    /// Cost of a via up from `bottom` at `at` crowding the net's own vias.
    fn own_via_cost(&self, bottom: LayerId, at: Point<Dbu>) -> f64 {
        let via = self.tech.via_above(bottom).via;
        let crowded = self
            .own_vias
            .map_or(0, |own| own.crowded(self.db, self.tech, via, at));
        self.costs.penalty * crowded as f64
    }

    pub(crate) fn allowed(&self, layer: LayerId, p: Point<Dbu>) -> bool {
        self.guide.is_none_or(|g| g.contains(layer, p))
    }

    /// Center distance below which two vias between `a` and `b` crowd each
    /// other's cuts.
    fn cut_reach(&self, a: LayerId, b: LayerId) -> Dbu {
        self.tech.via_above(a.min(b)).cut_reach
    }

    /// The along-track extent, relative to the via's center, of the metal
    /// on `layer` of the via between `layer` and `other`.
    fn pad_along(&self, layer: LayerId, other: LayerId) -> (Dbu, Dbu) {
        let via = self.tech.via_above(layer.min(other));
        let pad = if other > layer { via.bottom } else { via.top };
        self.tech.layer(layer).across_along_rect(&pad).1
    }

    /// How far along a track a via onto `layer` crowds later vias off it:
    /// its cut while a via back through the same cut would be too close,
    /// its metal while another via's metal could end short of it by less
    /// than the spacing.
    fn via_reach(&self, layer: LayerId, from: LayerId) -> Dbu {
        let (lo, hi) = self.pad_along(layer, from);
        let neighbours = [layer.index().checked_sub(1), Some(layer.index() + 1)];
        let widest = neighbours
            .into_iter()
            .flatten()
            .map(LayerId::new)
            .filter(|&l| self.tech.min_layer <= l && l <= self.tech.max_layer)
            .map(|l| {
                let (a, b) = self.pad_along(layer, l);
                (-a).max(b)
            })
            .max()
            .unwrap_or_default();
        let metal = (-lo).max(hi) + widest + self.spacing(layer);
        metal.max(self.cut_reach(layer, from))
    }

    /// The recent via of a path that steps along its track to `along`.
    fn after_step(
        &self,
        layer: LayerId,
        recent: Option<RecentVia>,
        along: Dbu,
    ) -> Option<RecentVia> {
        let lt = self.tech.layer(layer);
        recent.filter(|v| {
            (along - lt.cross[v.cross as usize].at).abs() < self.via_reach(layer, v.from)
        })
    }

    /// Whether a via from `state` to `to` would crowd the via the path just
    /// came through: their cuts when both pass through one cut, or their
    /// metal on this layer when it would end short of touching by less
    /// than the spacing, a gap `OpenROAD` flags even where wire fills it.
    fn crowds_recent_via(&self, state: State, to: LayerId, along: Dbu) -> bool {
        let layer = state.node.layer;
        state.recent_via.is_some_and(|v| {
            let at = self.tech.layer(layer).cross[v.cross as usize].at;
            let cuts = v.from == to && (along - at).abs() < self.cut_reach(layer, to);
            let (a0, a1) = self.pad_along(layer, v.from);
            let (b0, b1) = self.pad_along(layer, to);
            let gap = (along + b0 - (at + a1)).max(at + a0 - (along + b1));
            cuts || (gap > Dbu::ZERO && gap < self.spacing(layer))
        })
    }

    /// States reachable from `state` in one step, with the step's cost.
    fn neighbours(
        &self,
        state: State,
        window: &Rect<Dbu>,
        costs: &mut impl StepCosts,
        out: &mut Vec<(State, f64)>,
    ) {
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
                    let lower = Node {
                        cross: n.cross.min(m.cross),
                        ..n
                    };
                    out.push((moved, costs.wire(self, lower)));
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
            let step = costs.via(self, n) + self.own_via_cost(n.layer, here);
            out.push((Self::via_state(m, n.layer), step));
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
                let step = costs.via(self, m) + self.own_via_cost(below, here);
                out.push((Self::via_state(m, n.layer), step));
            }
        }
    }

    /// The state on `node` just after a via from `from`.
    const fn via_state(node: Node, from: LayerId) -> State {
        State {
            node,
            heading: Heading::Still,
            recent_via: Some(RecentVia {
                from,
                cross: node.cross,
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
        WindowCache::new(self, *window).find(self, sources, targets)
    }

    /// One A* run with the given effort.
    fn find_with(
        &self,
        sources: &[(Node, f64)],
        targets: &FxHashMap<Node, (usize, f64)>,
        window: &Rect<Dbu>,
        effort: Effort,
        visits: &mut impl Visits,
        costs: &mut impl StepCosts,
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
            effort.weight * self.costs.via.mul_add(layers, plane)
        };

        visits.clear();
        let mut heap = BinaryHeap::new();
        for &(n, g) in sources {
            let state = State::start(n);
            if visits.best(state).is_none_or(|b| g < b) {
                visits.record(state, g, None);
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
            if visits.best(state).is_some_and(|b| g > b) {
                continue;
            }
            if let Some(&(k, extra)) = targets.get(&state.node)
                && finish.is_none_or(|(cost, _, _)| g + extra < cost)
            {
                finish = Some((g + extra, state, k));
            }
            expansions += 1;
            if expansions > effort.limit {
                break;
            }
            self.neighbours(state, window, costs, &mut buf);
            for &(m, step) in &buf {
                let ng = g + step;
                if visits.best(m).is_none_or(|b| ng < b) {
                    visits.record(m, ng, Some(state));
                    heap.push(Entry {
                        f: ng + h(m.node),
                        g: ng,
                        state: m,
                    });
                }
            }
        }
        let (_, end, k) = finish?;
        Some((visits.path_to(end), k))
    }
}

/// Where a search keeps the cheapest known cost of each state it reached
/// and the state it came from.
trait Visits {
    /// Forgets every state.
    fn clear(&mut self);
    fn best(&self, state: State) -> Option<f64>;
    fn record(&mut self, state: State, cost: f64, from: Option<State>);
    /// The nodes of the cheapest known path to `end`, from its source.
    fn path_to(&self, end: State) -> Vec<Node>;
}

/// Visits kept in a hash map, for windows too large to number densely.
#[derive(Default)]
struct HashVisits(FxHashMap<State, (f64, Option<State>)>);

impl Visits for HashVisits {
    fn clear(&mut self) {
        self.0.clear();
    }

    fn best(&self, state: State) -> Option<f64> {
        self.0.get(&state).map(|&(cost, _)| cost)
    }

    fn record(&mut self, state: State, cost: f64, from: Option<State>) {
        let _ = self.0.insert(state, (cost, from));
    }

    fn path_to(&self, end: State) -> Vec<Node> {
        let mut path = vec![end.node];
        let mut cur = end;
        while let Some(&(_, Some(prev))) = self.0.get(&cur) {
            path.push(prev.node);
            cur = prev;
        }
        path.reverse();
        path
    }
}

/// The cost of each step a search takes, apart from crowding the net's own
/// vias.
trait StepCosts {
    /// The wire step from `lower` to the next crossing along its track.
    fn wire(&mut self, search: &Search<'_>, lower: Node) -> f64;
    /// The via from `lower` up to the layer above.
    fn via(&mut self, search: &Search<'_>, lower: Node) -> f64;
}

/// Step costs computed afresh every time.
struct FreshCosts;

impl StepCosts for FreshCosts {
    fn wire(&mut self, search: &Search<'_>, lower: Node) -> f64 {
        let lt = search.tech.layer(lower.layer);
        let a = lt.point(lower.track, lower.cross);
        let b = lt.point(lower.track, lower.cross + 1);
        search.step_cost(lower.layer, a, b)
    }

    fn via(&mut self, search: &Search<'_>, lower: Node) -> f64 {
        search.via_cost(lower.layer, search.point(lower))
    }
}

/// The searches of one window, sharing the costs of the steps they take.
///
/// Step costs are kept between searches, so every search through one cache
/// must see the same occupancy, overlay, history, owner and penalty; only
/// the net's own vias, which [`Search::own_vias`] holds, may change.
pub(crate) struct WindowCache {
    window: Rect<Dbu>,
    dense: Option<dense::Dense>,
}

impl WindowCache {
    /// A cache for searches of `search`'s net inside `window`.
    pub(crate) fn new(search: &Search<'_>, window: Rect<Dbu>) -> Self {
        Self {
            window,
            dense: dense::Dense::new(search, &window),
        }
    }

    /// Cheapest path from any source to any target inside the window, as
    /// [`Search::find`] gives it.
    pub(crate) fn find(
        &mut self,
        search: &Search<'_>,
        sources: &[(Node, f64)],
        targets: &FxHashMap<Node, (usize, f64)>,
    ) -> Option<(Vec<Node>, usize)> {
        let window = self.window;
        let indexed = self
            .dense
            .as_mut()
            .filter(|d| sources.iter().all(|&(n, _)| d.covers(n)));
        let Some(dense) = indexed else {
            let mut visits = HashVisits::default();
            let mut run =
                |e| search.find_with(sources, targets, &window, e, &mut visits, &mut FreshCosts);
            return run(FOCUSED).or_else(|| run(GREEDY));
        };
        let (mut visits, mut costs) = dense.split();
        let mut run = |e| search.find_with(sources, targets, &window, e, &mut visits, &mut costs);
        run(FOCUSED).or_else(|| run(GREEDY))
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
            own_vias: None,
            ripped: &[],
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
