//! The A* search over track crossings and what it costs.
//!
//! Every edge of the track graph costs as `TritonRoute` prices an edge of
//! its grid graph (`FlexGridGraph::getCosts`): its length (a via counts as
//! long as its cost), plus the round's conflict cost per unit of length if
//! its metal or cuts would short or crowd another net's wiring, plus the
//! round's marker cost per unit of length if it runs where a violation was
//! marked lately. Obstacles, and other nets' pins and reserved pin access,
//! never move, so they cost so much that a route through one exists only
//! when nothing else does. An edge the [`EdgeRules`] block around pins and
//! obstructions costs `TritonRoute`'s block cost on top. A guide limits the
//! search to gcells near a net's global route.
// NOTE: TritonRoute adds one per turn (FlexGridGraph::getNextPathCost) and
// prices off-guide edges in getCosts; PARE has no turn cost and makes the
// guide a hard bound, widened only when it holds no route.

mod dense;
mod edge_rules;
mod marks;

pub(crate) use edge_rules::EdgeRules;
use edge_rules::{GridCost, Relief};
pub(crate) use marks::Marks;

use crate::access::{AccessPoint, Node};
use crate::bucket::{Bucket, SpatialHash};
use crate::global::{GCell, Gcells};
use crate::occupancy::{
    LocalOccupancy, Occupancy, Origin, Owner, Proximity, too_close, too_close_along,
};
use crate::tech::RouteTech;
use crate::tech::{CrossIdx, TrackIdx};
use crate::wiring::Edge;
use pare_common::convert::SaturatingFrom;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{CutLayerId, LayerId, NetId, NonDefaultRuleId, ViaId};
use pare_common::db::library::LayerRef;
use pare_common::db::shapes::{wire_rect, wire_rect_with_ends};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::Interval;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rustc_hash::FxHashMap;
use std::collections::BinaryHeap;

/// Cost of crossing an obstacle or another net's fixed metal, in DBU of
/// wirelength.
// NOTE: getCosts charges ggFixedShapeCost_ per unit length and BLOCKCOST *
// minWidth * 20 when blocked; PARE makes all fixed metal one near-wall price.
const OBSTACLE_COST: f64 = 1e9;

/// Cost per unit of length of an edge with grid cost, beyond its length:
/// `TritonRoute`'s `GRIDCOST` default of 2.
const GRID_COST: f64 = 2.0;

/// Cost of a blocked edge per minimum width of its layer, a twentieth of
/// it: `TritonRoute`'s `BLOCKCOST` default of 32.
const BLOCK_COST: f64 = 32.0;

/// Minimum widths of its layer a blocked edge costs `BLOCK_COST` per, as
/// `FlexGridGraph::getCosts` charges `BLOCKCOST * minWidth * 20`.
const BLOCK_COST_WIDTHS: f64 = 20.0;

/// How hard one A* run tries.
#[derive(Clone, Copy, Debug)]
struct Effort {
    /// Scale of the heuristic; above one trades optimality for speed.
    weight: f64,
    /// Expansions before giving up unless a target has been reached.
    limit: usize,
}

/// The first, near-optimal search of a connection. PARE tunable.
// NOTE: FlexGridGraph's A* is unweighted and unbounded; PARE weighs its
// heuristic and caps expansions to bound each search's time.
const FOCUSED: Effort = Effort {
    weight: 1.0,
    limit: 2_000_000,
};

/// The greedy search that finds a path quickly when the focused one gives
/// up; past its limit the connection gives up in this window. PARE tunable.
const GREEDY: Effort = Effort {
    weight: 4.0,
    limit: 4_000_000,
};

/// The costs of one round of the search.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Costs {
    /// Cost of one via, in DBU of wirelength; also its length when its
    /// conflict and marker costs are reckoned.
    pub via: f64,
    /// Cost per unit of length of an edge that conflicts with another net's
    /// wiring.
    pub conflict: f64,
    /// Cost per unit of length of an edge that runs where a violation was
    /// marked.
    pub marker: f64,
    /// Factor each marker counter is scaled by after every piece a worker
    /// routes.
    pub decay: f64,
}

/// What an edge's metal and cuts meet: fixed metal, and wiring that can
/// move.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Meets {
    fixed: bool,
    wiring: bool,
}

impl Meets {
    /// Everything either meets.
    const fn and(self, other: Self) -> Self {
        Self {
            fixed: self.fixed || other.fixed,
            wiring: self.wiring || other.wiring,
        }
    }

    const fn any(self) -> bool {
        self.fixed || self.wiring
    }
}

/// Whether a shape's conflicts with fixed metal count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FixedMetal {
    /// They count.
    Counts,
    /// They do not: beside the routed net's own block pin, or for the via
    /// up from its own access point.
    Relieved,
}

/// The cost of one edge of the graph, part by part.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Price {
    length: f64,
    conflict: f64,
    marker: f64,
    obstacle: f64,
    grid: f64,
    blocked: f64,
}

impl Price {
    fn total(self) -> f64 {
        self.length + self.conflict + self.marker + self.obstacle + self.grid + self.blocked
    }
}

/// Where the owners an edge meets are told: nowhere while searching, into
/// an [`Explanation`] while tracing.
pub(crate) trait CostParts {
    /// Notes that an edge's metal or cuts meet `owner`'s.
    fn meets(&mut self, owner: Owner);
}

/// Owners met, forgotten.
struct Total;

impl CostParts for Total {
    #[inline(always)]
    fn meets(&mut self, _: Owner) {}
}

/// What a route costs a search, part by part.
#[derive(Clone, Debug, Default, PartialEq)]
pub(crate) struct Explanation {
    /// Wire length, in database units.
    pub length: f64,
    /// Vias, and what they cost for being vias.
    pub vias: usize,
    pub via_cost: f64,
    /// What conflicting with other nets' wiring costs.
    pub conflict: f64,
    /// Whose metal or cuts the route meets, in the order first met.
    pub met: Vec<Owner>,
    /// What running where violations were marked costs.
    pub marker: f64,
    /// What crossing fixed metal costs.
    pub obstacle: f64,
    /// What crowding the net's own vias costs.
    pub own_vias: f64,
    /// What `TritonRoute`'s grid cost adds: running against layers'
    /// preferred directions, and near other nets' macro and I/O access
    /// points.
    pub grid: f64,
    /// What taking edges the rules around pins and obstructions block
    /// costs.
    pub blocked: f64,
}

impl Explanation {
    /// The whole cost.
    pub fn total(&self) -> f64 {
        self.length
            + self.via_cost
            + self.conflict
            + self.marker
            + self.obstacle
            + self.own_vias
            + self.grid
            + self.blocked
    }
}

impl CostParts for Explanation {
    fn meets(&mut self, owner: Owner) {
        if !self.met.contains(&owner) {
            self.met.push(owner);
        }
    }
}

/// Gcells around each global route cell a net's detailed route may use.
pub(crate) const GUIDE_SLACK_GCELLS: Gcells = Gcells(1);

/// Layers above and below each global route cell a net's detailed route
/// may use.
const GUIDE_SLACK_LAYERS: usize = 1;

/// The gcells a net's detailed route may use: its global route grown by
/// [`GUIDE_SLACK_GCELLS`] and [`GUIDE_SLACK_LAYERS`].
#[derive(Debug)]
pub(crate) struct Guide {
    origin: Point<Dbu>,
    gcell: Dbu,
    reach: Gcells,
    x0: Gcells,
    y0: Gcells,
    width: Gcells,
    height: Gcells,
    min_layer: usize,
    layers: usize,
    bits: Vec<u64>,
}

impl Guide {
    /// The guide around the global route through `cells`; none for a route
    /// of no cells.
    pub(crate) fn new(
        cells: &[GCell],
        origin: Point<Dbu>,
        gcell: Dbu,
        tech: &RouteTech,
    ) -> Option<Self> {
        let reach = GUIDE_SLACK_GCELLS;
        let column = |c: &GCell| Gcells::from(c.x);
        let row = |c: &GCell| Gcells::from(c.y);
        let x0 = cells.iter().map(column).min()? - reach;
        let y0 = cells.iter().map(row).min()? - reach;
        let x1 = cells.iter().map(column).max()? + reach;
        let y1 = cells.iter().map(row).max()? + reach;
        let min_layer = tech.min_layer.index();
        let layers = tech.max_layer.index() - min_layer + 1;
        let (width, height) = (x1 - x0 + Gcells(1), y1 - y0 + Gcells(1));
        let total = usize::saturating_from(width.get() * height.get()) * layers;
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
                for dy in (-reach).through(reach) {
                    for dx in (-reach).through(reach) {
                        if let Some(i) = guide.index(dl, column(c) + dx, row(c) + dy) {
                            guide.bits[i / 64] |= 1 << (i % 64);
                        }
                    }
                }
            }
        }
        Some(guide)
    }

    /// The extent of one gcell.
    pub(crate) fn cell_rect(&self, cell: GCell) -> Rect<Dbu> {
        let min = self.origin
            + Point::new(
                Gcells::from(cell.x).length(self.gcell),
                Gcells::from(cell.y).length(self.gcell),
            );
        Rect::new(min, min + Point::new(self.gcell, self.gcell))
    }

    /// The area this guide allows around `cells`.
    pub(crate) fn area(&self, cells: &[GCell]) -> Option<Rect<Dbu>> {
        cells
            .iter()
            .map(|&c| self.cell_rect(c))
            .reduce(|a, b| a.union(&b))
            .map(|r| r.expand(self.reach.length(self.gcell)))
    }

    fn index(&self, layer: usize, x: Gcells, y: Gcells) -> Option<usize> {
        let (lx, ly) = (x - self.x0, y - self.y0);
        let outside = lx < Gcells(0) || ly < Gcells(0) || lx >= self.width || ly >= self.height;
        if layer >= self.layers || outside {
            return None;
        }
        let (lx, ly) = (lx.get(), ly.get());
        usize::try_from(
            (i64::saturating_from(layer) * self.height.get() + ly) * self.width.get() + lx,
        )
        .ok()
    }

    pub(crate) fn contains(&self, layer: LayerId, p: Point<Dbu>) -> bool {
        let Some(l) = layer.index().checked_sub(self.min_layer) else {
            return false;
        };
        let x = Gcells::holding(p.x - self.origin.x, self.gcell);
        let y = Gcells::holding(p.y - self.origin.y, self.gcell);
        self.index(l, x, y)
            .is_some_and(|i| self.bits[i / 64] & (1 << (i % 64)) != 0)
    }
}

/// A via and where it is placed.
type PlacedVia = (ViaId, Point<Dbu>);

/// The vias a net already has, which its new vias must not crowd.
#[derive(Clone, Debug)]
pub(crate) struct OwnVias {
    /// The non-default rule the net follows.
    rule: Option<NonDefaultRuleId>,
    vias: SpatialHash<PlacedVia>,
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
            rule,
            vias: SpatialHash::new(
                Point::new(Dbu::ZERO, Dbu::ZERO),
                (widest + spacing).max(Dbu::new(1)),
            ),
        }
    }

    pub(crate) fn add(&mut self, via: ViaId, at: Point<Dbu>) {
        self.vias.insert(self.vias.bucket(at), (via, at));
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
        let shapes = &db.library.via(via).shapes;
        self.near(&Rect::new(at, at))
            .filter(|&&(other, p)| {
                let theirs = &db.library.via(other).shapes;
                shapes.iter().any(|a| {
                    theirs.iter().any(|b| {
                        a.layer == b.layer
                            && too_close(
                                self.spacing(db, tech, a.layer),
                                &a.rect.translate(at),
                                &b.rect.translate(p),
                            )
                    })
                })
            })
            .count()
    }

    /// How many of the vias a piece of metal `rect` on `layer` would crowd
    /// along the layer's tracks: their metal there lies across the same
    /// tracks as `rect` and ends short of it by less than the spacing.
    fn crowded_along(
        &self,
        db: &NetlistDB,
        tech: &RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
    ) -> usize {
        let lt = tech.layer(layer);
        let on = LayerRef::Routing(layer);
        let spacing = self.spacing(db, tech, on);
        self.near(rect)
            .filter(|&&(via, p)| {
                db.library.via(via).shapes.iter().any(|s| {
                    s.layer == on && too_close_along(lt, spacing, rect, &s.rect.translate(p))
                })
            })
            .count()
    }

    /// The vias filed close enough to `rect` that their shapes could crowd
    /// a shape within it.
    fn near(&self, rect: &Rect<Dbu>) -> impl Iterator<Item = &PlacedVia> {
        let one = Point::new(Bucket(1), Bucket(1));
        self.vias.between(
            self.vias.bucket(rect.min) - one,
            self.vias.bucket(rect.max) + one,
        )
    }

    /// The spacing the net's shapes on `layer` keep.
    fn spacing(&self, db: &NetlistDB, tech: &RouteTech, layer: LayerRef) -> Dbu {
        match layer {
            LayerRef::Routing(l) => tech.wire_spacing(self.rule, l),
            LayerRef::Cut(c) => db.library.cut_layers[c].spacing.unwrap_or_default(),
        }
    }
}

/// One A* search context.
#[derive(Clone, Copy)]
pub(crate) struct Search<'a> {
    pub(crate) db: &'a NetlistDB,
    pub(crate) tech: &'a RouteTech,
    pub(crate) occupancy: &'a Occupancy,
    pub(crate) overlay: Option<&'a LocalOccupancy>,
    /// Where violations were marked lately.
    pub(crate) marks: &'a Marks,
    pub(crate) owner: Owner,
    pub(crate) costs: Costs,
    pub(crate) guide: Option<&'a Guide>,
    /// Vias the net already has, if new vias should keep clear of them.
    pub(crate) own_vias: Option<&'a OwnVias>,
    /// Which edges around pins and obstructions are blocked.
    pub(crate) rules: &'a EdgeRules,
    /// Nets the worker is routing again whose wiring in the shared
    /// occupancy therefore does not count; what they keep near the worker
    /// is in the overlay.
    pub(crate) ripped: &'a [Owner],
}

/// A position in the search: a node, the way the path moves along the
/// node's track, the via the path last took if it is still within cut
/// spacing, since a via back through the same cut there would break that
/// spacing, and the crossing of the jog it took onto its track while a via
/// ahead could still end short of the jog's metal. Paths never turn back
/// along a track, so once one moves out of a via's or a jog's reach it
/// cannot return to crowd it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct State {
    node: Node,
    heading: Heading,
    recent_via: Option<RecentVia>,
    recent_jog: Option<CrossIdx>,
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
    /// The track it landed on, the current one or one a jog left.
    track: TrackIdx,
    /// The crossing it landed on.
    cross: CrossIdx,
}

impl State {
    /// What tells this state apart from others in a search: all of it but
    /// the recent jog, which rides along with the cheapest path to the rest.
    // NOTE: FlexGridGraph keeps a turn's length on the wavefront, outside
    // the cost a grid node is visited at; PARE likewise leaves the recent jog
    // out, which spares the slots a jog would multiply, at the price of a
    // cheaper path's jog sometimes refusing a via a pruned path could take.
    const fn key(self) -> Self {
        Self {
            recent_jog: None,
            ..self
        }
    }

    const fn start(node: Node) -> Self {
        Self {
            node,
            heading: Heading::Still,
            recent_via: None,
            recent_jog: None,
        }
    }
}

/// A lower bound, weighted by the search's effort, on the cost from a node
/// to the nearest target: the distance to the box around the targets plus
/// a via to each layer between the node and the nearest target layer.
struct GoalEstimate {
    goal: Rect<Dbu>,
    layers: Vec<usize>,
    weight: f64,
}

impl GoalEstimate {
    /// The estimate toward `targets` at `effort`; none for no target.
    fn toward(
        search: &Search<'_>,
        targets: &FxHashMap<Node, (usize, f64)>,
        effort: Effort,
    ) -> Option<Self> {
        let goal = targets
            .keys()
            .map(|&n| {
                let p = search.point(n);
                Rect::new(p, p)
            })
            .reduce(|a, b| a.union(&b))?;
        let mut layers: Vec<usize> = targets.keys().map(|n| n.layer.index()).collect();
        layers.sort_unstable();
        layers.dedup();
        Some(Self {
            goal,
            layers,
            weight: effort.weight,
        })
    }

    /// The estimate from node `n`.
    fn at_node(&self, search: &Search<'_>, n: Node) -> f64 {
        let layers = self
            .layers
            .iter()
            .map(|&l| l.abs_diff(n.layer.index()))
            .min()
            .unwrap_or(0) as f64;
        let plane = distance_to(&self.goal, search.point(n)).to_f64();
        self.weight * search.costs.via.mul_add(layers, plane)
    }
}

/// Which way a via climbs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Climb {
    Up,
    Down,
}

/// Which way a jog crosses its layer's tracks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Across {
    Up,
    Down,
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
            Owner::Net(n) => self.tech.net_rule(n),
            Owner::Obstacle => None,
        }
    }

    /// The width of the net's wires on `layer`.
    /// The net the search routes; none for a search of no net's.
    const fn net(&self) -> Option<NetId> {
        match self.owner {
            Owner::Net(n) => Some(n),
            Owner::Obstacle => None,
        }
    }

    pub(crate) fn width(&self, layer: LayerId) -> Dbu {
        self.tech.wire_width(self.rule(), layer)
    }

    /// The spacing the net's metal on `layer` keeps from other metal.
    fn spacing(&self, layer: LayerId) -> Dbu {
        self.tech.wire_spacing(self.rule(), layer)
    }

    /// Cost of using an access point: its stub and its vias, each priced
    /// as an edge of the graph by what its metal and cuts meet. The first
    /// via, up from the pin, pays nothing for fixed metal and cuts, as
    /// `initMazeCost_ap_helper` overrides the fixed shape cost of the via up
    /// from an access point on its pin's layer.
    pub(crate) fn access_cost(&self, a: &AccessPoint) -> f64 {
        let node = self.point(a.node);
        let mut cost = if node == a.at {
            0.0
        } else {
            self.wire_price(a.node.layer, node, a.at).total()
        };
        for (k, &via) in a.vias.iter().enumerate() {
            let fixed = if k == 0 {
                FixedMetal::Relieved
            } else {
                FixedMetal::Counts
            };
            let mut meets = Meets::default();
            for s in &self.db.library.via(via).shapes {
                let rect = s.rect.translate(a.at);
                meets = meets.and(match s.layer {
                    LayerRef::Routing(l) if self.tech.routable(l) => {
                        self.meets_counting(l, &rect, false, fixed, &mut Total)
                    }
                    LayerRef::Routing(_) => Meets::default(),
                    LayerRef::Cut(c) => self.cut_meets(c, &rect, fixed, &mut Total),
                });
            }
            cost += self.price(self.costs.via, meets, false).total();
        }
        cost
    }

    /// Whether one occupant of the shared occupancy is fixed metal, wiring
    /// that counts as a conflict, or wiring the worker is routing again.
    fn occupant(&self, owner: Owner, origin: Origin) -> Meets {
        match (owner, origin) {
            (Owner::Obstacle, _) | (Owner::Net(_), Origin::Fixed) => Meets {
                fixed: true,
                wiring: false,
            },
            (Owner::Net(_), Origin::Wiring) if self.ripped.contains(&owner) => Meets::default(),
            (Owner::Net(_), Origin::Wiring) => Meets {
                fixed: false,
                wiring: true,
            },
        }
    }

    /// What other owners' cuts a cut at `rect` on `cut` would touch or
    /// crowd, telling `parts` whose they are, with fixed cuts counting or
    /// not as `fixed` says.
    fn cut_meets(
        &self,
        cut: CutLayerId,
        rect: &Rect<Dbu>,
        fixed: FixedMetal,
        parts: &mut impl CostParts,
    ) -> Meets {
        let mut meets = Meets::default();
        for o in self.occupancy.cut_conflicts(cut, rect, self.owner) {
            let mut met = self.occupant(o.owner, o.origin);
            met.fixed &= fixed == FixedMetal::Counts;
            if met.any() {
                parts.meets(o.owner);
            }
            meets = meets.and(met);
        }
        if let Some(overlay) = self.overlay {
            for o in overlay.cut_conflicts(cut, rect, self.owner) {
                parts.meets(o.owner);
                meets.wiring = true;
            }
        }
        meets
    }

    /// What other owners' metal a shape `rect` on `layer` would conflict
    /// with, telling `parts` whose it is. With `alongside`, metal it would
    /// violate the spacing of should a wire go on alongside it counts as a
    /// conflict too, as for one step of a wire.
    fn metal_meets(
        &self,
        layer: LayerId,
        rect: &Rect<Dbu>,
        alongside: bool,
        parts: &mut impl CostParts,
    ) -> Meets {
        self.meets_counting(layer, rect, alongside, FixedMetal::Counts, parts)
    }

    /// [`Self::metal_meets`], with fixed metal counting or not as `fixed`
    /// says.
    fn meets_counting(
        &self,
        layer: LayerId,
        rect: &Rect<Dbu>,
        alongside: bool,
        fixed: FixedMetal,
        parts: &mut impl CostParts,
    ) -> Meets {
        let mut meets = Meets::default();
        for c in self
            .occupancy
            .rect_conflicts(self.tech, layer, rect, self.owner)
        {
            let mut met = match c.proximity {
                Proximity::Within => self.occupant(c.occupant.owner, c.occupant.origin),
                Proximity::Alongside if alongside => Meets {
                    fixed: false,
                    wiring: true,
                },
                Proximity::Alongside => Meets::default(),
            };
            met.fixed &= fixed == FixedMetal::Counts;
            if met.any() {
                parts.meets(c.occupant.owner);
            }
            meets = meets.and(met);
        }
        if let Some(overlay) = self.overlay {
            for c in overlay.rect_conflicts(self.tech, layer, rect, self.owner) {
                if c.proximity == Proximity::Within || alongside {
                    parts.meets(c.occupant.owner);
                    meets.wiring = true;
                }
            }
        }
        meets
    }

    /// The price of an edge of `length` that `meets` what it meets and runs
    /// where a violation was marked if `marked`: its length, and the
    /// round's conflict and marker costs per unit of it, as `TritonRoute`
    /// prices an edge of its grid graph (`FlexGridGraph::getCosts`). Fixed
    /// metal costs an obstacle.
    fn price(&self, length: f64, meets: Meets, marked: bool) -> Price {
        Price {
            length,
            conflict: if meets.wiring {
                self.costs.conflict * length
            } else {
                0.0
            },
            marker: if marked {
                self.costs.marker * length
            } else {
                0.0
            },
            obstacle: if meets.fixed { OBSTACLE_COST } else { 0.0 },
            grid: 0.0,
            blocked: 0.0,
        }
    }

    /// What taking `edge`, leaving a node on `layer`, costs beyond its
    /// price for being blocked to the net.
    // NOTE: getCosts charges the minimum width of the layer a via is
    // entered from, which differs by direction; PARE prices a via the same
    // both ways, by its lower layer.
    fn block_price(&self, edge: Edge, layer: LayerId) -> f64 {
        if self.rules.blocked(self.tech, self.net(), edge) {
            let min_width = self.db.library.routing_layers[layer].min_width();
            BLOCK_COST * BLOCK_COST_WIDTHS * min_width.to_f64()
        } else {
            0.0
        }
    }

    /// What a shape `rect` on `layer` laid by itself, such as a patch,
    /// would conflict with: an obstacle's cost for fixed metal, and the
    /// conflict cost over the length it runs along its layer for wiring.
    /// Its own length costs nothing, so that shapes of different extents
    /// compare by their conflicts alone.
    pub(crate) fn shape_cost(&self, layer: LayerId, rect: &Rect<Dbu>) -> f64 {
        let (_, Interval { lo: b0, hi: b1 }) = self.tech.layer(layer).across_along_rect(rect);
        let meets = self.metal_meets(layer, rect, false, &mut Total);
        let price = self.price((b1 - b0).to_f64(), meets, false);
        price.conflict + price.obstacle
    }

    /// Price of a whole wire from `a` to `b`, such as an access stub.
    fn wire_price(&self, layer: LayerId, a: Point<Dbu>, b: Point<Dbu>) -> Price {
        let length = a.manhattan(b).to_f64();
        let meets = wire_rect(a, b, self.width(layer)).map_or_else(Meets::default, |r| {
            self.metal_meets(layer, &r, false, &mut Total)
        });
        self.price(length, meets, false)
    }

    /// Price of the wire step from `lower` to the next crossing along its
    /// track, telling `parts` whose metal it meets. It is marked when either
    /// end is.
    // NOTE: FlexGridGraph::getMarkerCostAdj charges the node stepped into;
    // PARE also charges the step leaving a marked node.
    fn step_price(&self, lower: Node, parts: &mut impl CostParts) -> Price {
        self.planar_price(Edge::Wire(lower), parts)
    }

    /// Price of the jog from `lower` across to the same crossing of the
    /// next track, telling `parts` whose metal it meets.
    fn jog_price(&self, lower: Node, parts: &mut impl CostParts) -> Price {
        self.planar_price(Edge::Jog(lower), parts)
    }

    /// What grid cost `edge`, a wire step, jog or via `length` long, pays:
    /// [`GRID_COST`] per unit of its length on a jog against its layer's
    /// preferred direction, as `FlexGridGraph::initEdges` marks one, and on
    /// any edge of a line out of another net's macro or I/O access point,
    /// but on none of a line out of the net's own.
    fn grid_price(&self, edge: Edge, length: f64) -> f64 {
        let charged = match self.rules.grid_cost(self.net(), edge) {
            GridCost::AsLaidOut => matches!(edge, Edge::Jog(_)),
            GridCost::Off => false,
            GridCost::On => true,
        };
        if charged { GRID_COST * length } else { 0.0 }
    }

    /// Price of `edge`, a wire step or a jog, for its length, what its
    /// metal meets, the marks at its ends, its grid cost and being blocked.
    fn planar_price(&self, edge: Edge, parts: &mut impl CostParts) -> Price {
        let (a, b) = edge.ends();
        let length = self.point(a).manhattan(self.point(b)).to_f64();
        let meets = self.planar_meets(edge, parts);
        let marked = self.marks.planar(a) || self.marks.planar(b);
        Price {
            grid: self.grid_price(edge, length),
            blocked: self.block_price(edge, a.layer),
            ..self.price(length, meets, marked)
        }
    }

    /// What the metal of `edge`, a wire step or a jog, meets, telling
    /// `parts` whose it is. Its metal stops at an end where the net's own
    /// access point touches its pin, as the routed wire does. Where the
    /// net's own block pins relieve an end, only the fixed metal the other
    /// end's wire comes near counts, or none when they relieve both.
    // NOTE: TritonRoute charges fixed shapes per node entered, so one edge
    // costs differently by direction; PARE prices an edge the same both
    // ways, by its whole metal unless a block pin relieves an end.
    fn planar_meets(&self, edge: Edge, parts: &mut impl CostParts) -> Meets {
        let (a, b) = edge.ends();
        let layer = a.layer;
        let width = self.width(layer);
        let extension = |n: Node| {
            if self.rules.cut_off_at(self.net(), n) {
                Dbu::ZERO
            } else {
                width / 2
            }
        };
        let metal = wire_rect_with_ends(
            self.point(a),
            self.point(b),
            width,
            extension(a),
            extension(b),
        );
        let Some(metal) = metal else {
            return Meets::default();
        };
        let relieved =
            |parts: &mut _| self.meets_counting(layer, &metal, true, FixedMetal::Relieved, parts);
        match self.rules.relief(self.tech, self.net(), edge) {
            Relief::None => self.metal_meets(layer, &metal, true, parts),
            Relief::Whole => relieved(parts),
            Relief::Partly { charged } => {
                let end = self.end_square(charged);
                relieved(parts).and(self.metal_meets(layer, &end, false, parts))
            }
        }
    }

    /// The square a wire of the net covers around `node`.
    fn end_square(&self, node: Node) -> Rect<Dbu> {
        let half = self.width(node.layer) / 2;
        let p = self.point(node);
        Rect::new(
            Point::new(p.x - half, p.y - half),
            Point::new(p.x + half, p.y + half),
        )
    }

    /// Price of the via up from `lower` to `upper`, apart from crowding the
    /// net's own vias, telling `parts` whose metal and cuts it meets. A via
    /// is as long as its cost. Up from the net's own access point it pays
    /// nothing for fixed metal and cuts, as `initMazeCost_ap_helper`
    /// overrides the fixed shape cost of an access point's via.
    fn via_price(&self, lower: Node, upper: Node, parts: &mut impl CostParts) -> Price {
        let at = self.point(lower);
        let via = self.tech.via_above(lower.layer);
        let top = upper.layer;
        let fixed = if self.rules.via_access_at(self.net(), lower) {
            FixedMetal::Relieved
        } else {
            FixedMetal::Counts
        };
        let mut meets = self
            .meets_counting(lower.layer, &via.bottom.translate(at), false, fixed, parts)
            .and(self.meets_counting(top, &via.top.translate(at), false, fixed, parts));
        for c in &via.cuts {
            let rect = c.translate(at);
            meets = meets.and(self.cut_meets(lower.layer.cut_above(), &rect, fixed, parts));
        }
        let edge = Edge::Via { lower, upper };
        Price {
            grid: self.grid_price(edge, self.costs.via),
            blocked: self.block_price(edge, lower.layer),
            ..self.price(self.costs.via, meets, self.marks.via(lower))
        }
    }

    /// What routing `edges` costs this search, part by part: their length,
    /// their vias, their conflicts with whose metal and cuts, the marks they
    /// run through, and crowding the net's own vias.
    pub(crate) fn explain(&self, edges: &[Edge]) -> Explanation {
        let mut parts = Explanation::default();
        for edge in edges {
            let price = match *edge {
                Edge::Wire(lower) => {
                    let price = self.step_price(lower, &mut parts);
                    parts.length += price.length;
                    price
                }
                Edge::Via { lower, upper } => {
                    let price = self.via_price(lower, upper, &mut parts);
                    parts.vias += 1;
                    parts.via_cost += price.length;
                    parts.own_vias += self.own_via_cost(lower.layer, self.point(lower));
                    price
                }
                Edge::Jog(lower) => {
                    let price = self.jog_price(lower, &mut parts);
                    parts.length += price.length;
                    parts.own_vias += self.own_jog_cost(lower);
                    price
                }
            };
            parts.grid += price.grid;
            parts.conflict += price.conflict;
            parts.marker += price.marker;
            parts.obstacle += price.obstacle;
            parts.blocked += price.blocked;
        }
        parts
    }

    /// Cost of a via up from `bottom` at `at` crowding the net's own vias:
    /// a conflict over the via's length.
    fn own_via_cost(&self, bottom: LayerId, at: Point<Dbu>) -> f64 {
        let via = self.tech.via_above(bottom).via;
        let crowds = self
            .own_vias
            .is_some_and(|own| own.crowded(self.db, self.tech, via, at) > 0);
        if crowds {
            self.costs.conflict * self.costs.via
        } else {
            0.0
        }
    }

    /// Cost of the jog from `lower` across to the next track crowding the
    /// net's own vias along their tracks, such as the access via its first
    /// node was reached from: a conflict over the jog's length.
    // NOTE: FlexGridGraph::getNextPathCost charges a turn within
    // viaForbiddenTurnLen of the path's last via; PARE checks the jog's
    // metal against the net's via pads, which also sees access vias. A pad
    // short of the jog across the tracks is left alone, since the next jog
    // of a chain closes that gap.
    fn own_jog_cost(&self, lower: Node) -> f64 {
        let Some(own) = self.own_vias else {
            return 0.0;
        };
        let crowds = self
            .jog_metal(lower)
            .is_some_and(|r| own.crowded_along(self.db, self.tech, lower.layer, &r) > 0);
        if !crowds {
            return 0.0;
        }
        let lt = self.tech.layer(lower.layer);
        let length = lt.tracks[lower.track.plus(1)] - lt.tracks[lower.track];
        self.costs.conflict * length.to_f64()
    }

    /// The metal of the jog from `lower` across to the next track.
    fn jog_metal(&self, lower: Node) -> Option<Rect<Dbu>> {
        let (from, to) = Edge::Jog(lower).ends();
        wire_rect(self.point(from), self.point(to), self.width(lower.layer))
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
    fn pad_along(&self, layer: LayerId, other: LayerId) -> Interval {
        let (_, along) = self
            .tech
            .layer(layer)
            .across_along_rect(&self.pad(layer, other));
        along
    }

    /// The metal on `layer`, relative to the via's center, of the via
    /// between `layer` and `other`.
    fn pad(&self, layer: LayerId, other: LayerId) -> Rect<Dbu> {
        let via = self.tech.via_above(layer.min(other));
        if other > layer { via.bottom } else { via.top }
    }

    /// How far along a track a via onto `layer` crowds later vias off it:
    /// its cut while a via back through the same cut would be too close,
    /// its metal while another via's metal could end short of it by less
    /// than the spacing.
    fn via_reach(&self, layer: LayerId, from: LayerId) -> Dbu {
        let Interval { lo, hi } = self.pad_along(layer, from);
        let metal = (-lo).max(hi) + self.widest_pad_along(layer) + self.spacing(layer);
        metal.max(self.cut_reach(layer, from))
    }

    /// The layers next to `layer` that a via off it may reach.
    fn via_layers(&self, layer: LayerId) -> impl Iterator<Item = LayerId> + '_ {
        [layer.index().checked_sub(1), Some(layer.index() + 1)]
            .into_iter()
            .flatten()
            .map(LayerId::new)
            .filter(|&l| self.tech.min_layer <= l && l <= self.tech.max_layer)
    }

    /// How far along its track, from the via's center, the metal on `layer`
    /// of the longest via off it reaches.
    fn widest_pad_along(&self, layer: LayerId) -> Dbu {
        self.via_layers(layer)
            .map(|l| {
                let Interval { lo, hi } = self.pad_along(layer, l);
                (-lo).max(hi)
            })
            .max()
            .unwrap_or_default()
    }

    /// How far along its track from a jog a via could still end short of
    /// the jog's metal by less than the spacing.
    fn jog_reach(&self, layer: LayerId) -> Dbu {
        let width = self.width(layer);
        width - width / 2 + self.widest_pad_along(layer) + self.spacing(layer)
    }

    /// Whether a via between `layer` and `other` at `via_along` would end
    /// short of the metal of a jog at `jog_along`, along their track, by
    /// less than the spacing; the via lies on the track the jog spans.
    fn via_crowds_jog(
        &self,
        layer: LayerId,
        other: LayerId,
        jog_along: Dbu,
        via_along: Dbu,
    ) -> bool {
        let width = self.width(layer);
        let (j0, j1) = (jog_along - width / 2, jog_along + (width - width / 2));
        let Interval { lo, hi } = self.pad_along(layer, other);
        let gap = (via_along + lo - j1).max(j0 - (via_along + hi));
        gap > Dbu::ZERO && gap < self.spacing(layer)
    }

    /// The crossings of `layer` at which some via would crowd the metal of
    /// a jog at `jog`.
    fn jog_crowding_crosses(
        &self,
        layer: LayerId,
        jog: CrossIdx,
    ) -> impl Iterator<Item = CrossIdx> + '_ {
        let lt = self.tech.layer(layer);
        let jog_along = lt.cross[jog].at;
        let reach = self.jog_reach(layer);
        let within = move |c: &CrossIdx| (lt.cross[*c].at - jog_along).abs() < reach;
        let up = std::iter::successors(Some(jog.next()), |c| Some(c.next()))
            .take_while(move |c| c.index() < lt.cross.len())
            .take_while(within);
        let down = std::iter::successors(jog.prev(), |c| c.prev()).take_while(within);
        up.chain(down).filter(move |&c| {
            self.via_layers(layer)
                .any(|other| self.via_crowds_jog(layer, other, jog_along, lt.cross[c].at))
        })
    }

    /// The recent jog of a path at crossing `at` of `layer` after a jog at
    /// `jog`: kept while a via at `at` or further on, away from the jog,
    /// could still crowd it.
    fn hold_jog(&self, layer: LayerId, jog: CrossIdx, at: CrossIdx) -> Option<CrossIdx> {
        let here = at.offset_from(jog);
        self.jog_crowding_crosses(layer, jog)
            .map(|c| c.offset_from(jog))
            .any(|k| here == 0 || (k.signum() == here.signum() && k.abs() >= here.abs()))
            .then_some(jog)
    }

    /// Whether a via from `state` to `to` at `along` would crowd the jog
    /// the path took onto its track.
    // NOTE: FlexGridGraph::getNextPathCost charges a via within
    // viaForbiddenTurnLen of the last turn; PARE refuses it, as it refuses a
    // via crowding the last one, since explaining a route sees no path order.
    fn via_crowds_recent_jog(&self, state: State, to: LayerId, along: Dbu) -> bool {
        let layer = state.node.layer;
        state.recent_jog.is_some_and(|j| {
            self.via_crowds_jog(layer, to, self.tech.layer(layer).cross[j].at, along)
        })
    }

    /// The recent via of a path that steps along its track to `along`.
    fn after_step(
        &self,
        layer: LayerId,
        recent: Option<RecentVia>,
        along: Dbu,
    ) -> Option<RecentVia> {
        let lt = self.tech.layer(layer);
        recent.filter(|v| (along - lt.cross[v.cross].at).abs() < self.via_reach(layer, v.from))
    }

    /// The recent via of a path that jogs across to `track`: kept while it
    /// lies across fewer than its reach away, where the along-track check
    /// of a later via still applies, as if the via were on this track.
    fn after_jog(
        &self,
        layer: LayerId,
        recent: Option<RecentVia>,
        track: TrackIdx,
    ) -> Option<RecentVia> {
        let lt = self.tech.layer(layer);
        recent.filter(|v| {
            (lt.tracks[track] - lt.tracks[v.track]).abs() < self.via_reach(layer, v.from)
        })
    }

    /// Whether a via from `state` to `to` would crowd the via the path just
    /// came through: their cuts when both pass through one cut, or their
    /// metal on this layer when it would end short of touching by less
    /// than the spacing, a gap `OpenROAD` flags even where wire fills it.
    fn crowds_recent_via(&self, state: State, to: LayerId, along: Dbu) -> bool {
        let layer = state.node.layer;
        state.recent_via.is_some_and(|v| {
            let at = self.tech.layer(layer).cross[v.cross].at;
            let cuts = v.from == to && (along - at).abs() < self.cut_reach(layer, to);
            let Interval { lo: a0, hi: a1 } = self.pad_along(layer, v.from);
            let Interval { lo: b0, hi: b1 } = self.pad_along(layer, to);
            let gap = (along + b0 - (at + a1)).max(at + a0 - (along + b1));
            cuts || (gap > Dbu::ZERO && gap < self.spacing(layer))
        })
    }

    /// Whether the jog from `state` across from `lower` would crowd the via
    /// the path just came through along its tracks: the jog's metal ends
    /// short of the via's metal on this layer by less than the spacing.
    // NOTE: FlexGridGraph::getNextPathCost charges a turn within
    // viaForbiddenTurnLen of the last via; PARE refuses it, as it refuses a
    // via crowding the last one, since explaining a route sees no path order.
    fn jog_crowds_recent_via(&self, state: State, lower: Node) -> bool {
        let layer = state.node.layer;
        let lt = self.tech.layer(layer);
        state.recent_via.is_some_and(|v| {
            let pad = self
                .pad(layer, v.from)
                .translate(lt.point(v.track, v.cross));
            self.jog_metal(lower)
                .is_some_and(|metal| too_close_along(lt, self.spacing(layer), &metal, &pad))
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
        let steps = [
            (n.cross.prev(), Heading::Down),
            (Some(n.cross.next()), Heading::Up),
        ];
        for (next, heading) in steps {
            let turns_back = matches!(
                (state.heading, heading),
                (Heading::Up, Heading::Down) | (Heading::Down, Heading::Up)
            );
            let Some(next) = next.filter(|_| !turns_back) else {
                continue;
            };
            if next.index() < lt.cross.len() {
                let m = Node { cross: next, ..n };
                let p = lt.point(m.track, m.cross);
                if window.contains(p) && self.allowed(m.layer, p) {
                    let moved = State {
                        node: m,
                        heading,
                        recent_via: self.after_step(n.layer, state.recent_via, lt.cross[next].at),
                        recent_jog: state
                            .recent_jog
                            .and_then(|j| self.hold_jog(n.layer, j, next)),
                    };
                    let lower = Node {
                        cross: n.cross.min(m.cross),
                        ..n
                    };
                    out.push((moved, costs.wire(self, lower)));
                }
            }
        }
        out.extend(self.jog_move(state, Across::Up, window, costs));
        out.extend(self.jog_move(state, Across::Down, window, costs));
        out.extend(self.via_move(state, Climb::Up, costs));
        out.extend(self.via_move(state, Climb::Down, costs));
    }

    /// The jog from `state` across to the same crossing of the next track
    /// up or down, with what it costs, priced at its lower track; none where
    /// no jog may run or it would leave `window` or the guide.
    fn jog_move(
        &self,
        state: State,
        across: Across,
        window: &Rect<Dbu>,
        costs: &mut impl StepCosts,
    ) -> Option<(State, f64)> {
        let n = state.node;
        let track = match across {
            Across::Up => n.track.plus(1),
            Across::Down => TrackIdx::new(n.track.index().checked_sub(1)?),
        };
        let m = Node { track, ..n };
        let lower = n.min(m);
        if !self.tech.jog_allowed(lower) || self.jog_crowds_recent_via(state, lower) {
            return None;
        }
        let p = self.point(m);
        if !window.contains(p) || !self.allowed(m.layer, p) {
            return None;
        }
        let moved = State {
            node: m,
            heading: Heading::Still,
            recent_via: self.after_jog(n.layer, state.recent_via, track),
            recent_jog: self.hold_jog(n.layer, n.cross, n.cross),
        };
        Some((moved, costs.jog(self, lower) + self.own_jog_cost(lower)))
    }

    /// The via from `state` to the layer above or below, with what it
    /// costs, priced at its lower end; none where there is no such layer,
    /// the via is not allowed there or crowds the last one, or the other
    /// layer has no track or cross there.
    fn via_move(
        &self,
        state: State,
        climb: Climb,
        costs: &mut impl StepCosts,
    ) -> Option<(State, f64)> {
        let n = state.node;
        let lt = self.tech.layer(n.layer);
        let here = lt.point(n.track, n.cross);
        let cross = lt.cross[n.cross];
        let (to, track, c) = match climb {
            Climb::Up => (
                (n.layer < self.tech.max_layer).then(|| LayerId::new(n.layer.index() + 1)),
                cross.up,
                lt.track_up[n.track],
            ),
            Climb::Down => (
                (n.layer > self.tech.min_layer).then(|| LayerId::new(n.layer.index() - 1)),
                cross.down,
                lt.track_down[n.track],
            ),
        };
        let to = to?;
        if !self.allowed(to, here)
            || self.crowds_recent_via(state, to, cross.at)
            || self.via_crowds_recent_jog(state, to, cross.at)
        {
            return None;
        }
        let m = Node {
            layer: to,
            track: track?,
            cross: c?,
        };
        let (lower, upper) = match climb {
            Climb::Up => (n, m),
            Climb::Down => (m, n),
        };
        let step = costs.via(self, lower, upper) + self.own_via_cost(lower.layer, here);
        Some((Self::via_state(m, n.layer), step))
    }

    /// The state on `node` just after a via from `from`.
    const fn via_state(node: Node, from: LayerId) -> State {
        State {
            node,
            heading: Heading::Still,
            recent_via: Some(RecentVia {
                from,
                track: node.track,
                cross: node.cross,
            }),
            recent_jog: None,
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
        let estimate = GoalEstimate::toward(self, targets, effort)?;
        let h = |n: Node| estimate.at_node(self, n);

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
        self.0.get(&state.key()).map(|&(cost, _)| cost)
    }

    fn record(&mut self, state: State, cost: f64, from: Option<State>) {
        let _ = self.0.insert(state.key(), (cost, from.map(State::key)));
    }

    fn path_to(&self, end: State) -> Vec<Node> {
        let mut path = vec![end.node];
        let mut cur = end.key();
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
    fn via(&mut self, search: &Search<'_>, lower: Node, upper: Node) -> f64;
    /// The jog from `lower` across to the next track.
    fn jog(&mut self, search: &Search<'_>, lower: Node) -> f64;
}

/// Step costs computed afresh every time.
struct FreshCosts;

impl StepCosts for FreshCosts {
    fn wire(&mut self, search: &Search<'_>, lower: Node) -> f64 {
        search.step_price(lower, &mut Total).total()
    }

    fn via(&mut self, search: &Search<'_>, lower: Node, upper: Node) -> f64 {
        search.via_price(lower, upper, &mut Total).total()
    }

    fn jog(&mut self, search: &Search<'_>, lower: Node) -> f64 {
        search.jog_price(lower, &mut Total).total()
    }
}

/// The searches of one window, sharing the costs of the steps they take.
///
/// Step costs are kept between searches, so every search through one cache
/// must see the same occupancy, overlay, marks, owner and costs; only the
/// net's own vias, which [`Search::own_vias`] holds, may change.
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
fn distance_to(r: &Rect<Dbu>, p: Point<Dbu>) -> Dbu {
    let dx = (r.min.x - p.x).max(p.x - r.max.x).max(Dbu::ZERO);
    let dy = (r.min.y - p.y).max(p.y - r.max.y).max(Dbu::ZERO);
    dx + dy
}

#[cfg(test)]
mod tests {
    use super::edge_rules::NO_RULES;
    use super::*;
    use crate::tech::RouteSetup;
    use crate::tech::{Cross, CrossIdx, LayerTech, TrackIdx};
    use crate::tests::tiny;
    use pare_common::config::RoutingConfig;
    use pare_common::db::indices::NetId;
    use pare_common::db::library::Direction;

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
                let run = (lt.cross[path[j].cross].at - lt.cross[onto.cross].at).abs();
                out.push((run, tech.via_above(from.layer.min(onto.layer)).cut_reach));
            }
            i = j + 1;
        }
        out
    }

    #[test]
    fn a_global_route_of_no_cells_gives_no_guide() {
        let db = tiny();
        let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();

        let guide = Guide::new(&[], db.die_area.min, Dbu::new(1000), &tech);

        assert!(guide.is_none());
    }

    #[test]
    fn a_path_leaves_a_cut_crowded_by_its_last_via_before_coming_back_through_it() {
        let db = tiny();
        let config = RoutingConfig {
            min_layer: Some("M1".to_string()),
            ..RoutingConfig::default()
        };
        let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(NetId(0)),
            costs: COSTS,
            guide: None,
            own_vias: None,
            rules: &NO_RULES,
            ripped: &[],
        };
        let m1 = LayerId(0);
        let lt = tech.layer(m1);
        let node = |y: i64| Node {
            layer: m1,
            track: TrackIdx::new(lt.tracks.as_slice().binary_search(&Dbu::new(y)).unwrap()),
            cross: CrossIdx::new(
                lt.cross
                    .as_slice()
                    .partition_point(|c| c.at < Dbu::new(1100)),
            ),
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

    /// A search for net 0 of the tiny design on M1 and up, with nothing
    /// routed, at `costs`.
    fn open_search<'a>(
        db: &'a NetlistDB,
        tech: &'a RouteTech,
        occupancy: &'a Occupancy,
        marks: &'a Marks,
        costs: Costs,
    ) -> Search<'a> {
        Search {
            db,
            tech,
            occupancy,
            overlay: None,
            marks,
            owner: Owner::Net(NetId(0)),
            costs,
            guide: None,
            own_vias: None,
            rules: &NO_RULES,
            ripped: &[],
        }
    }

    fn m1_tech(db: &NetlistDB) -> RouteTech {
        let config = RoutingConfig {
            min_layer: Some("M1".to_string()),
            ..RoutingConfig::default()
        };
        RouteTech::new(db, &RouteSetup::from(&config)).unwrap()
    }

    /// The first crossing of `layer` that `wanted` accepts, on track 1.
    fn crossing_where(tech: &RouteTech, layer: LayerId, wanted: impl Fn(&Cross) -> bool) -> Node {
        let cross = tech
            .layer(layer)
            .cross
            .iter_enumerated()
            .find(|(_, c)| wanted(c));
        Node {
            layer,
            track: TrackIdx(1),
            cross: cross.unwrap().0,
        }
    }

    #[test]
    fn a_path_to_the_next_track_jogs_across_when_vias_cost_more() {
        let db = tiny();
        let tech = m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let costs = Costs {
            via: 10_000.0,
            ..COSTS
        };
        let search = open_search(&db, &tech, &occupancy, &marks, costs);
        let from = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
        let to = Node {
            track: TrackIdx(2),
            ..from
        };
        let targets: FxHashMap<Node, (usize, f64)> = std::iter::once((to, (0, 0.0))).collect();

        let (path, _) = search.find(&[(from, 0.0)], &targets, &db.die_area).unwrap();

        assert_eq!(path, [from, to]);
    }

    #[test]
    fn no_jog_leaves_a_crossing_the_layer_above_does_not_cross() {
        let db = tiny();
        let tech = m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
        let n = crossing_where(&tech, LayerId(1), |c| c.up.is_none());
        let mut out = Vec::new();

        search.neighbours(State::start(n), &db.die_area, &mut FreshCosts, &mut out);

        assert!(
            out.iter()
                .all(|(s, _)| s.node.track == n.track || s.node.layer != n.layer)
        );
    }

    #[test]
    fn a_jog_keeps_a_recent_via_only_while_it_lies_within_reach_across() {
        let db = tiny();
        let tech = m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
        let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
        let recent = Some(RecentVia {
            from: LayerId(1),
            track: n.track,
            cross: n.cross,
        });
        let far = tech.layer(n.layer).tracks.len() - 1;

        assert_eq!(search.after_jog(n.layer, recent, n.track), recent);
        assert_eq!(search.after_jog(n.layer, recent, TrackIdx::new(far)), None);
    }

    const COSTS: Costs = Costs {
        via: 50.0,
        conflict: 8.0,
        marker: 32.0,
        decay: 0.95,
    };

    /// The tiny design on M1 and up, with another net's wire along M1's
    /// track 1 from crossing 1 to crossing 3.
    fn beside_a_wire(db: &NetlistDB) -> (RouteTech, Occupancy) {
        let config = RoutingConfig {
            min_layer: Some("M1".to_string()),
            ..RoutingConfig::default()
        };
        let tech = RouteTech::new(db, &RouteSetup::from(&config)).unwrap();
        let lt = tech.layer(LayerId(0));
        let other = wire_rect(
            lt.point(TrackIdx(1), CrossIdx(1)),
            lt.point(TrackIdx(1), CrossIdx(3)),
            lt.width,
        )
        .unwrap();
        let mut occupancy = Occupancy::new(&tech, Dbu::new(1000));
        occupancy.add_rect(
            &tech,
            LayerId(0),
            &other,
            Owner::Net(NetId(1)),
            Origin::Wiring,
        );
        (tech, occupancy)
    }

    fn step(track: u32, cross: u32) -> Node {
        Node {
            layer: LayerId(0),
            track: TrackIdx(track),
            cross: CrossIdx(cross),
        }
    }

    fn step_length(tech: &RouteTech, lower: Node) -> f64 {
        let lt = tech.layer(lower.layer);
        let (a, b) = (
            lt.point(lower.track, lower.cross),
            lt.point(lower.track, lower.cross.next()),
        );
        a.manhattan(b).to_f64()
    }

    #[test]
    fn a_step_over_another_nets_wire_pays_the_conflict_cost_per_unit_of_length() {
        let db = tiny();
        let (tech, occupancy) = beside_a_wire(&db);
        let marks = Marks::default();
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(NetId(0)),
            costs: COSTS,
            guide: None,
            own_vias: None,
            rules: &NO_RULES,
            ripped: &[],
        };
        let lower = step(1, 1);
        let length = step_length(&tech, lower);

        let price = search.step_price(lower, &mut Total);

        let (conflict, total) = (COSTS.conflict * length, (1.0 + COSTS.conflict) * length);
        assert!((price.conflict - conflict).abs() < 1e-9, "{price:?}");
        assert!((price.total() - total).abs() < 1e-9);
    }

    #[test]
    fn a_free_jog_costs_its_length_and_the_grid_cost_per_unit_of_it() {
        let db = tiny();
        let (tech, _) = beside_a_wire(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(NetId(0)),
            costs: COSTS,
            guide: None,
            own_vias: None,
            rules: &NO_RULES,
            ripped: &[],
        };
        let lower = step(1, 4);
        let lt = tech.layer(lower.layer);
        let length = (lt.tracks[TrackIdx(2)] - lt.tracks[TrackIdx(1)]).to_f64();

        let price = search.jog_price(lower, &mut Total);

        let total = (1.0 + GRID_COST) * length;
        assert!((price.total() - total).abs() < 1e-9, "{price:?}");
    }

    /// The point `along` the track at `across` of `lt`.
    fn across_along_point(lt: &LayerTech, across: Dbu, along: Dbu) -> Point<Dbu> {
        match lt.direction {
            Direction::Horizontal => Point::new(along, across),
            Direction::Vertical => Point::new(across, along),
        }
    }

    /// The net's own vias: one up from M1 on the track of `jog`, whose pad
    /// ends `gap` short of the jog's metal along that track.
    fn own_via_short_of(db: &NetlistDB, tech: &RouteTech, jog: Node, gap: Dbu) -> OwnVias {
        let lt = tech.layer(jog.layer);
        let via = tech.via_above(jog.layer);
        let (_, pad) = lt.across_along_rect(&via.bottom);
        let jog_end = lt.cross[jog.cross].at + lt.width / 2;
        let at = across_along_point(lt, lt.tracks[jog.track], jog_end + gap - pad.lo);
        let mut own = OwnVias::new(db, tech, None);
        own.add(via.via, at);
        own
    }

    #[test]
    fn a_jog_whose_metal_crowds_the_nets_own_via_pays_a_conflict_over_its_length() {
        let db = tiny();
        let tech = m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let jog = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
        let own = own_via_short_of(&db, &tech, jog, tech.layer(jog.layer).spacing / 2);
        let search = Search {
            own_vias: Some(&own),
            ..open_search(&db, &tech, &occupancy, &marks, COSTS)
        };
        let lt = tech.layer(jog.layer);
        let jogged = Node {
            track: jog.track.plus(1),
            ..jog
        };
        let length = (lt.tracks[jogged.track] - lt.tracks[jog.track]).to_f64();
        let mut out = Vec::new();

        search.neighbours(State::start(jog), &db.die_area, &mut FreshCosts, &mut out);

        let (_, cost) = out.iter().find(|(s, _)| s.node == jogged).unwrap();
        let expected = (1.0 + GRID_COST + COSTS.conflict) * length;
        assert!((cost - expected).abs() < 1e-9, "{cost} against {expected}");
    }

    #[test]
    fn a_jog_whose_metal_overlaps_the_nets_own_via_pays_nothing_for_it() {
        let db = tiny();
        let tech = m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let jog = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
        let overlap = tech.layer(jog.layer).width;
        let own = own_via_short_of(&db, &tech, jog, -overlap);
        let search = Search {
            own_vias: Some(&own),
            ..open_search(&db, &tech, &occupancy, &marks, COSTS)
        };

        assert!(search.own_jog_cost(jog).abs() < 1e-9);
    }

    #[test]
    fn a_jog_stopping_a_track_short_of_the_nets_own_via_pays_nothing_for_it() {
        let db = tiny();
        let tech = m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let jog = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
        let lt = tech.layer(jog.layer);
        let via = tech.via_above(jog.layer);
        let at = lt.point(jog.track.plus(2), jog.cross);
        let mut own = OwnVias::new(&db, &tech, None);
        own.add(via.via, at);
        let (a, b) = (
            lt.point(jog.track, jog.cross),
            lt.point(jog.track.plus(1), jog.cross),
        );
        let metal = wire_rect(a, b, lt.width).unwrap();
        assert!(too_close(lt.spacing, &metal, &via.bottom.translate(at)));
        let search = Search {
            own_vias: Some(&own),
            ..open_search(&db, &tech, &occupancy, &marks, COSTS)
        };

        assert!(search.own_jog_cost(jog).abs() < 1e-9);
    }

    /// The jogs `search` offers from `node` just after a via from M2 landed
    /// at `via_cross` on its track.
    fn jogs_after_a_via(search: &Search<'_>, node: Node, via_cross: CrossIdx) -> Vec<Node> {
        let state = State {
            node,
            heading: Heading::Still,
            recent_via: Some(RecentVia {
                from: LayerId(1),
                track: node.track,
                cross: via_cross,
            }),
            recent_jog: None,
        };
        let mut out = Vec::new();
        search.neighbours(state, &search.db.die_area, &mut FreshCosts, &mut out);
        out.into_iter()
            .map(|(s, _)| s.node)
            .filter(|m| m.layer == node.layer && m.track != node.track)
            .collect()
    }

    #[test]
    fn no_jog_leaves_where_its_metal_would_end_short_of_the_last_via_along_the_track() {
        let db = tiny();
        let tech = wide_m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
        let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
        let lt = tech.layer(n.layer);
        let pad = search.pad(n.layer, LayerId(1));
        let via_metal = pad.translate(lt.point(n.track, n.cross.next()));
        let jog_metal = search.jog_metal(n).unwrap();
        assert!(too_close_along(lt, lt.spacing, &jog_metal, &via_metal));

        let jogs = jogs_after_a_via(&search, n, n.cross.next());

        assert_eq!(jogs, []);
    }

    #[test]
    fn a_jog_leaves_from_the_crossing_the_last_via_landed_on() {
        let db = tiny();
        let tech = m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
        let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());

        let jogs = jogs_after_a_via(&search, n, n.cross);

        assert!(jogs.contains(&Node {
            track: n.track.plus(1),
            ..n
        }));
    }

    /// The tiny design on M1 and up, M1 spacing half as wide again, so that
    /// metal one crossing along a track ends short of the spacing.
    fn wide_m1_tech(db: &NetlistDB) -> RouteTech {
        let mut tech = m1_tech(db);
        let m1 = &mut tech.layers[LayerId(0)];
        m1.spacing = m1.spacing * 3 / 2;
        tech
    }

    /// Whether `search` offers a via up from `node`, reached by a step from
    /// the jog the path took at crossing `jog` of its track.
    fn via_up_after_a_jog(search: &Search<'_>, node: Node, jog: CrossIdx) -> bool {
        let state = State {
            node,
            heading: Heading::Down,
            recent_via: None,
            recent_jog: Some(jog),
        };
        search.via_move(state, Climb::Up, &mut FreshCosts).is_some()
    }

    #[test]
    fn no_via_leaves_where_its_metal_would_end_short_of_the_last_jog_along_the_track() {
        let db = tiny();
        let tech = wide_m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
        let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
        let lt = tech.layer(n.layer);
        let (jog_along, via_along) = (lt.cross[n.cross.next()].at, lt.cross[n.cross].at);
        assert!(search.via_crowds_jog(n.layer, LayerId(1), jog_along, via_along));

        assert!(!via_up_after_a_jog(&search, n, n.cross.next()));
    }

    #[test]
    fn a_via_leaves_from_the_crossing_the_last_jog_ran_across() {
        let db = tiny();
        let tech = wide_m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
        let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());

        assert!(via_up_after_a_jog(&search, n, n.cross));
    }

    #[test]
    fn a_jog_is_remembered_only_where_a_via_ahead_could_crowd_it() {
        let db = tiny();
        let (plain, wide) = (m1_tech(&db), wide_m1_tech(&db));
        let occupancy = Occupancy::new(&plain, Dbu::new(1000));
        let marks = Marks::default();
        let n = crossing_where(&plain, LayerId(0), |c| c.up.is_some());
        let held = |tech: &RouteTech| {
            let search = open_search(&db, tech, &occupancy, &marks, COSTS);
            search.hold_jog(n.layer, n.cross, n.cross)
        };

        assert_eq!((held(&plain), held(&wide)), (None, Some(n.cross)));
    }

    #[test]
    fn a_hashed_visit_is_found_again_whatever_recent_jog_it_carried() {
        let node = step(1, 3);
        let carrying = |recent_jog| State {
            node,
            heading: Heading::Up,
            recent_via: None,
            recent_jog,
        };
        let mut visits = HashVisits::default();

        visits.record(carrying(None), 5.0, None);

        assert_eq!(visits.best(carrying(Some(CrossIdx(2)))), Some(5.0));
    }

    #[test]
    fn explaining_a_jog_counts_its_crowding_the_nets_own_via() {
        let db = tiny();
        let tech = m1_tech(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let marks = Marks::default();
        let jog = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
        let own = own_via_short_of(&db, &tech, jog, tech.layer(jog.layer).spacing / 2);
        let search = Search {
            own_vias: Some(&own),
            ..open_search(&db, &tech, &occupancy, &marks, COSTS)
        };
        let paid = search.jog_price(jog, &mut Total).total() + search.own_jog_cost(jog);

        let explanation = search.explain(&[Edge::Jog(jog)]);

        assert!(explanation.own_vias > 0.0, "{explanation:?}");
        assert!((explanation.total() - paid).abs() < 1e-9);
    }

    #[test]
    fn a_step_over_wiring_being_routed_again_costs_only_its_length() {
        let db = tiny();
        let (tech, occupancy) = beside_a_wire(&db);
        let marks = Marks::default();
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(NetId(0)),
            costs: COSTS,
            guide: None,
            own_vias: None,
            rules: &NO_RULES,
            ripped: &[Owner::Net(NetId(1))],
        };
        let lower = step(1, 1);

        let price = search.step_price(lower, &mut Total);

        assert!(
            (price.total() - step_length(&tech, lower)).abs() < 1e-9,
            "{price:?}"
        );
    }

    #[test]
    fn a_step_from_a_marked_crossing_pays_the_marker_cost_per_unit_of_length() {
        let db = tiny();
        let (tech, _) = beside_a_wire(&db);
        let occupancy = Occupancy::new(&tech, Dbu::new(1000));
        let mut marks = Marks::default();
        marks.mark_planar(step(4, 2));
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(NetId(0)),
            costs: COSTS,
            guide: None,
            own_vias: None,
            rules: &NO_RULES,
            ripped: &[],
        };
        let (into, away) = (step(4, 1), step(4, 3));

        let (into_mark, beyond) = (
            search.step_price(into, &mut Total),
            search.step_price(away, &mut Total),
        );

        let marker = COSTS.marker * step_length(&tech, into);
        assert!((into_mark.marker - marker).abs() < 1e-9, "{into_mark:?}");
        assert!(beyond.marker.abs() < 1e-9, "{beyond:?}");
    }

    #[test]
    fn explaining_a_route_names_whose_wiring_it_meets_and_adds_up_to_its_cost() {
        let db = tiny();
        let (tech, occupancy) = beside_a_wire(&db);
        let marks = Marks::default();
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(NetId(0)),
            costs: COSTS,
            guide: None,
            own_vias: None,
            rules: &NO_RULES,
            ripped: &[],
        };
        let edges = [Edge::Wire(step(1, 0)), Edge::Wire(step(1, 1))];
        let paid: f64 = edges
            .iter()
            .map(|e| match *e {
                Edge::Wire(n) => FreshCosts.wire(&search, n),
                Edge::Via { lower, upper } => FreshCosts.via(&search, lower, upper),
                Edge::Jog(lower) => search.jog_price(lower, &mut Total).total(),
            })
            .sum();

        let explanation = search.explain(&edges);

        assert_eq!(explanation.met, vec![Owner::Net(NetId(1))]);
        assert!(
            (explanation.total() - paid).abs() < 1e-9,
            "{explanation:?} against {paid}"
        );
    }
}
