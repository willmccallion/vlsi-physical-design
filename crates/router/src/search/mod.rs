//! The A* search over track crossings and what it costs.
//!
//! Every edge of the track graph costs as `TritonRoute` prices an edge of
//! its grid graph: its length (a via counts as long as its cost), plus the
//! round's conflict cost per unit of length if its metal or cuts would
//! short or crowd another net's wiring, plus the round's marker cost per
//! unit of length if it runs where a violation was marked lately.
//! Obstacles, and other nets' pins and reserved pin access, never move, so
//! they cost so much that a route through one exists only when nothing
//! else does. A guide limits the search to gcells near a net's global
//! route.

mod dense;
mod marks;

pub(crate) use marks::Marks;

use crate::access::{AccessPoint, Node};
use crate::global::GCell;
use crate::occupancy::{LocalOccupancy, Occupancy, Origin, Owner, Proximity, too_close};
use crate::tech::RouteTech;
use crate::wiring::Edge;
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

/// The cost of one edge of the graph, part by part.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Price {
    length: f64,
    conflict: f64,
    marker: f64,
    obstacle: f64,
}

impl Price {
    fn total(self) -> f64 {
        self.length + self.conflict + self.marker + self.obstacle
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
}

impl Explanation {
    /// The whole cost.
    pub fn total(&self) -> f64 {
        self.length + self.via_cost + self.conflict + self.marker + self.obstacle + self.own_vias
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
    /// Where violations were marked lately.
    pub(crate) marks: &'a Marks,
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

    /// Cost of using an access point: its stub and its vias, each priced
    /// as an edge of the graph by what its metal and cuts meet.
    pub(crate) fn access_cost(&self, a: &AccessPoint) -> f64 {
        let node = self.point(a.node);
        let mut cost = if node == a.at {
            0.0
        } else {
            self.wire_price(a.node.layer, node, a.at).total()
        };
        for &via in &a.vias {
            let mut meets = Meets::default();
            for s in &self.db.library.via(via).shapes {
                let rect = s.rect.translate(a.at);
                meets = meets.and(match s.layer {
                    LayerRef::Routing(l) if self.tech.routable(l) => {
                        self.metal_meets(l, &rect, false, &mut Total)
                    }
                    LayerRef::Routing(_) => Meets::default(),
                    LayerRef::Cut(c) => self.cut_meets(c, &rect, &mut Total),
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
    /// crowd, telling `parts` whose they are.
    fn cut_meets(&self, cut: CutLayerId, rect: &Rect<Dbu>, parts: &mut impl CostParts) -> Meets {
        let mut meets = Meets::default();
        for o in self.occupancy.cut_conflicts(cut, rect, self.owner) {
            let met = self.occupant(o.owner, o.origin);
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
        let mut meets = Meets::default();
        for c in self
            .occupancy
            .rect_conflicts(self.tech, layer, rect, self.owner)
        {
            let met = match c.proximity {
                Proximity::Within => self.occupant(c.occupant.owner, c.occupant.origin),
                Proximity::Alongside if alongside => Meets {
                    fixed: false,
                    wiring: true,
                },
                Proximity::Alongside => Meets::default(),
            };
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
    /// prices an edge of its grid graph. Fixed metal costs an obstacle.
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
        }
    }

    /// What a shape `rect` on `layer` laid by itself, such as a patch,
    /// would conflict with: an obstacle's cost for fixed metal, and the
    /// conflict cost over the length it runs along its layer for wiring.
    /// Its own length costs nothing, so that shapes of different extents
    /// compare by their conflicts alone.
    pub(crate) fn shape_cost(&self, layer: LayerId, rect: &Rect<Dbu>) -> f64 {
        let (_, (b0, b1)) = self.tech.layer(layer).across_along_rect(rect);
        let meets = self.metal_meets(layer, rect, false, &mut Total);
        let price = self.price((b1 - b0).to_f64(), meets, false);
        price.conflict + price.obstacle
    }

    /// Price of a whole wire from `a` to `b`, such as an access stub.
    fn wire_price(&self, layer: LayerId, a: Point<Dbu>, b: Point<Dbu>) -> Price {
        let length = ((a.x - b.x).abs() + (a.y - b.y).abs()).to_f64();
        let meets = wire_rect(a, b, self.width(layer)).map_or_else(Meets::default, |r| {
            self.metal_meets(layer, &r, false, &mut Total)
        });
        self.price(length, meets, false)
    }

    /// Price of the wire step from `lower` to the next crossing along its
    /// track, telling `parts` whose metal it meets. It is marked when either
    /// end is.
    fn step_price(&self, lower: Node, parts: &mut impl CostParts) -> Price {
        let upper = Node {
            cross: lower.cross + 1,
            ..lower
        };
        let (a, b) = (self.point(lower), self.point(upper));
        let length = ((a.x - b.x).abs() + (a.y - b.y).abs()).to_f64();
        let meets = wire_rect(a, b, self.width(lower.layer)).map_or_else(Meets::default, |r| {
            self.metal_meets(lower.layer, &r, true, parts)
        });
        let marked = self.marks.planar(lower) || self.marks.planar(upper);
        self.price(length, meets, marked)
    }

    /// Price of the via up from `lower`, apart from crowding the net's own
    /// vias, telling `parts` whose metal and cuts it meets. A via is as long
    /// as its cost.
    fn via_price(&self, lower: Node, parts: &mut impl CostParts) -> Price {
        let at = self.point(lower);
        let via = self.tech.via_above(lower.layer);
        let top = LayerId::new(lower.layer.index() + 1);
        let mut meets = self
            .metal_meets(lower.layer, &via.bottom.translate(at), false, parts)
            .and(self.metal_meets(top, &via.top.translate(at), false, parts));
        for c in &via.cuts {
            meets = meets.and(self.cut_meets(lower.layer.cut_above(), &c.translate(at), parts));
        }
        self.price(self.costs.via, meets, self.marks.via(lower))
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
                Edge::Via { lower, .. } => {
                    let price = self.via_price(lower, &mut parts);
                    parts.vias += 1;
                    parts.via_cost += price.length;
                    parts.own_vias += self.own_via_cost(lower.layer, self.point(lower));
                    price
                }
            };
            parts.conflict += price.conflict;
            parts.marker += price.marker;
            parts.obstacle += price.obstacle;
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
        search.step_price(lower, &mut Total).total()
    }

    fn via(&mut self, search: &Search<'_>, lower: Node) -> f64 {
        search.via_price(lower, &mut Total).total()
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
        let marks = Marks::default();
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(0),
            costs: COSTS,
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
        let tech = RouteTech::new(db, &config).unwrap();
        let lt = tech.layer(LayerId(0));
        let other = wire_rect(lt.point(1, 1), lt.point(1, 3), lt.width).unwrap();
        let mut occupancy = Occupancy::new(&tech, Dbu(1000));
        let _ = occupancy.add_rect(&tech, LayerId(0), &other, Owner::Net(1), Origin::Wiring);
        (tech, occupancy)
    }

    fn step(track: u32, cross: u32) -> Node {
        Node {
            layer: LayerId(0),
            track,
            cross,
        }
    }

    fn step_length(tech: &RouteTech, lower: Node) -> f64 {
        let lt = tech.layer(lower.layer);
        let (a, b) = (
            lt.point(lower.track, lower.cross),
            lt.point(lower.track, lower.cross + 1),
        );
        ((a.x - b.x).abs() + (a.y - b.y).abs()).to_f64()
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
            owner: Owner::Net(0),
            costs: COSTS,
            guide: None,
            own_vias: None,
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
            owner: Owner::Net(0),
            costs: COSTS,
            guide: None,
            own_vias: None,
            ripped: &[Owner::Net(1)],
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
        let occupancy = Occupancy::new(&tech, Dbu(1000));
        let mut marks = Marks::default();
        marks.mark_planar(step(4, 2));
        let search = Search {
            db: &db,
            tech: &tech,
            occupancy: &occupancy,
            overlay: None,
            marks: &marks,
            owner: Owner::Net(0),
            costs: COSTS,
            guide: None,
            own_vias: None,
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
            owner: Owner::Net(0),
            costs: COSTS,
            guide: None,
            own_vias: None,
            ripped: &[],
        };
        let edges = [Edge::Wire(step(1, 0)), Edge::Wire(step(1, 1))];
        let paid: f64 = edges
            .iter()
            .map(|e| match *e {
                Edge::Wire(n) => FreshCosts.wire(&search, n),
                Edge::Via { lower, .. } => FreshCosts.via(&search, lower),
            })
            .sum();

        let explanation = search.explain(&edges);

        assert_eq!(explanation.met, vec![Owner::Net(1)]);
        assert!(
            (explanation.total() - paid).abs() < 1e-9,
            "{explanation:?} against {paid}"
        );
    }
}
