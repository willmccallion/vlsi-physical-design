//! What `TritonRoute`'s maze cost set-up marks on the edges of its grid
//! around pins and obstructions (`FlexDRWorker::initMazeCost`): which edges
//! are blocked, costing a net that takes one `BLOCKCOST`, and where a net's
//! own block pins relieve it of fixed metal ([`Relief`]).
//!
//! An edge is blocked or open by the last of these to touch it, latest
//! first:
//!
//! - a block pin's access point: the ways it allows are open to its own
//!   net and blocked to every other (`modBlockedEdgesForMacroPin`);
//! - an I/O pin: no via where none of its access points takes one, and no
//!   jog across the tracks where one of them leaves along them
//!   (`initMazeCost_planarTerm`);
//! - an access point: the ways it allows are open, the others blocked
//!   (`initMazeCost_ap`);
//! - a cell pin's shape: wires inside it are open, reopening those an
//!   obstruction over the pin blocks;
//! - an obstruction or routing blockage: wires and vias inside it are
//!   blocked (`initMazeCost_fixedObj`).
//!
//! A node inside a shape counts with its boundary, as `getIdxBox` takes the
//! grid lines a box reaches.
// NOTE: TritonRoute sets these in an order that depends on which nets a
// worker has routed: a block pin's access ways close to other nets only once
// its own net is routed (mazeNetEnd). PARE takes every net as routed, so
// they are always closed to others; and where one stage both opens and
// blocks an edge from its two ends, PARE keeps it blocked.

mod lines;
mod relief;

pub(crate) use lines::GridCost;
use lines::Lines;
pub(crate) use relief::Relief;
use relief::{Band, bands_of, relief_of};

use crate::access::{AccessPoint, Node, PinKind, Way, Ways};
use crate::bucket::{SpatialHash, buckets_between};
use crate::tech::RouteTech;
use crate::wiring::Edge;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{IdVec, LayerId, NetId};
use pare_common::db::library::{Direction, LayerRef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rustc_hash::FxHashMap;

/// The rules every search of one detailed routing run prices edges by.
#[derive(Debug)]
pub(crate) struct EdgeRules {
    /// Cell obstructions and routing blockages on the routable layers.
    obstructions: Shapes,
    /// Cell pin shapes on the routable layers.
    cell_pins: Shapes,
    /// Access points and I/O pins, by the node they mark.
    nodes: FxHashMap<Node, NodeRules>,
    /// The bands around each net's block pins, for nets with any.
    bands: FxHashMap<NetId, Vec<Band>>,
    /// The grid cost lines out of macro and I/O pins' access points.
    lines: Lines,
}

/// Rules that block nothing, for searches outside a routing run.
#[cfg(test)]
pub(crate) static NO_RULES: std::sync::LazyLock<EdgeRules> =
    std::sync::LazyLock::new(EdgeRules::none);

/// Rectangles on layers, filed by the square buckets they touch.
#[derive(Debug)]
struct Shapes(SpatialHash<(LayerId, Rect<Dbu>)>);

impl Shapes {
    fn new(origin: Point<Dbu>, bucket: Dbu) -> Self {
        Self(SpatialHash::new(origin, bucket))
    }

    fn add(&mut self, layer: LayerId, rect: Rect<Dbu>) {
        for key in buckets_between(self.0.bucket(rect.min), self.0.bucket(rect.max)) {
            self.0.insert(key, (layer, rect));
        }
    }

    /// Whether a rectangle on `layer` holds `p`, its boundary included.
    fn hold(&self, layer: LayerId, p: Point<Dbu>) -> bool {
        self.0
            .in_bucket(self.0.bucket(p))
            .iter()
            .any(|&(l, r)| l == layer && r.contains(p))
    }
}

/// What marks one node.
#[derive(Clone, Debug)]
struct NodeRules {
    /// The access points standing on it, on their pin's own layer.
    points: Vec<PointRule>,
    /// The ways out of it an I/O pin over it blocks.
    io_blocked: Ways,
}

/// An access point on its pin's own layer: whose it is, the ways it allows
/// and what kind of pin it reaches.
#[derive(Clone, Copy, Debug)]
struct PointRule {
    net: NetId,
    ways: Ways,
    kind: PinKind,
}

impl EdgeRules {
    /// The rules of `db`'s shapes and of `access`, each net's access
    /// candidates by pin, filing shapes in square buckets of side `bucket`.
    pub(crate) fn new(
        db: &NetlistDB,
        tech: &RouteTech,
        access: &IdVec<NetId, Vec<Vec<AccessPoint>>>,
        bucket: Dbu,
    ) -> Self {
        let mut rules = Self {
            obstructions: Shapes::new(db.die_area.min, bucket),
            cell_pins: Shapes::new(db.die_area.min, bucket),
            nodes: FxHashMap::default(),
            bands: FxHashMap::default(),
            lines: Lines::default(),
        };
        rules.add_cells(db, tech);
        for s in &db.routing_blockages {
            if let LayerRef::Routing(l) = s.layer
                && tech.routable(l)
            {
                rules.obstructions.add(l, s.rect);
            }
        }
        rules.add_access_points(db, tech, access);
        rules.add_io_pins(db, tech, access);
        for (net, candidates) in access.iter_enumerated() {
            let bands = bands_of(db, tech, net, candidates);
            if !bands.is_empty() {
                let _ = rules.bands.insert(net, bands);
            }
        }
        rules
    }

    /// Rules that block nothing, for searches outside a routing run.
    #[cfg(test)]
    fn none() -> Self {
        Self {
            obstructions: Shapes::new(Point::default(), Dbu::new(1)),
            cell_pins: Shapes::new(Point::default(), Dbu::new(1)),
            nodes: FxHashMap::default(),
            bands: FxHashMap::default(),
            lines: Lines::default(),
        }
    }

    /// Files every cell's obstructions and pin shapes on routable layers.
    fn add_cells(&mut self, db: &NetlistDB, tech: &RouteTech) {
        for (id, c) in db.cells.iter_enumerated() {
            let m = db.cell_macro(id);
            let placed = |s: &pare_common::db::library::Shape| match s.layer {
                LayerRef::Routing(l) if tech.routable(l) => {
                    Some((l, c.orient.place_rect(&s.rect, m.size, c.location)))
                }
                _ => None,
            };
            for (l, r) in m.obstructions.iter().filter_map(placed) {
                self.obstructions.add(l, r);
            }
            for (l, r) in m
                .pins
                .iter()
                .flat_map(|p| p.shapes.iter())
                .filter_map(placed)
            {
                self.cell_pins.add(l, r);
            }
        }
    }

    /// Notes every candidate access point on its pin's own layer at its
    /// node, with the grid cost lines out of those of macro and I/O pins;
    /// one reaching its pin through vias from below stands on a node
    /// `TritonRoute`'s grid leaves plain.
    fn add_access_points(
        &mut self,
        db: &NetlistDB,
        tech: &RouteTech,
        access: &IdVec<NetId, Vec<Vec<AccessPoint>>>,
    ) {
        for (net, pins) in access.iter_enumerated() {
            for (&pin, candidates) in db.nets[net].pins.iter().zip(pins) {
                let kind = PinKind::of(db, pin);
                for a in candidates.iter().filter(|a| a.pin_layer == a.node.layer) {
                    self.node_mut(a.node).points.push(PointRule {
                        net,
                        ways: a.ways,
                        kind,
                    });
                    if !kind.is_standard_in_routing() {
                        self.lines.add(tech, net, a.node);
                    }
                }
            }
        }
    }

    /// Notes, at every node inside an I/O pin's shape on a routable layer,
    /// the ways out the pin blocks there, as `initMazeCost_planarTerm`
    /// judges them from the pin's access points on that layer.
    fn add_io_pins(
        &mut self,
        db: &NetlistDB,
        tech: &RouteTech,
        access: &IdVec<NetId, Vec<Vec<AccessPoint>>>,
    ) {
        for io in &db.io_pins {
            let candidates = db.nets[io.net]
                .pins
                .iter()
                .zip(&access[io.net])
                .find(|&(&p, _)| p == io.pin)
                .map_or(&[][..], |(_, c)| c.as_slice());
            for shape in io.placed_shapes() {
                let LayerRef::Routing(layer) = shape.layer else {
                    continue;
                };
                if !tech.routable(layer) {
                    continue;
                }
                let blocked = io_blocked(candidates.iter().filter(|a| a.node.layer == layer));
                let lt = tech.layer(layer);
                let (across, along) = lt.across_along_rect(&shape.rect);
                for track in lt.tracks_within(across.lo, across.hi) {
                    for cross in lt.cross_within(along.lo, along.hi) {
                        let node = Node {
                            layer,
                            track,
                            cross,
                        };
                        let rules = self.node_mut(node);
                        rules.io_blocked = rules.io_blocked.union(blocked);
                    }
                }
            }
        }
    }

    fn node_mut(&mut self, node: Node) -> &mut NodeRules {
        self.nodes.entry(node).or_insert_with(|| NodeRules {
            points: Vec::new(),
            io_blocked: Ways::NONE,
        })
    }

    /// Whether `edge` is blocked to `net`, or to every net for a search of
    /// no net's.
    pub(crate) fn blocked(&self, tech: &RouteTech, net: Option<NetId>, edge: Edge) -> bool {
        let ends = ends_with_ways(edge);
        let points = || {
            ends.into_iter().flat_map(|(n, way)| {
                self.nodes
                    .get(&n)
                    .into_iter()
                    .flat_map(|r| &r.points)
                    .map(move |p| (p, way))
            })
        };
        let block_access = points()
            .filter(|(p, way)| p.kind == PinKind::Block && way.is_planar() && p.ways.contains(*way))
            .map(|(p, _)| Some(p.net) != net);
        if let Some(blocked) = verdict(block_access) {
            return blocked;
        }
        let io = ends.into_iter().any(|(n, way)| {
            self.nodes
                .get(&n)
                .is_some_and(|r| r.io_blocked.contains(way))
        });
        if io {
            return true;
        }
        if let Some(blocked) = verdict(points().map(|(p, way)| !p.ways.contains(way))) {
            return blocked;
        }
        let at = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
        if !matches!(edge, Edge::Via { .. })
            && ends
                .iter()
                .any(|&(n, _)| self.cell_pins.hold(n.layer, at(n)))
        {
            return false;
        }
        ends.iter()
            .any(|&(n, _)| self.obstructions.hold(n.layer, at(n)))
    }

    /// The grid cost `edge` pays `net`, or a search of no net's, beyond
    /// that of running against its layer's preferred direction.
    pub(crate) fn grid_cost(&self, net: Option<NetId>, edge: Edge) -> GridCost {
        self.lines.grid_cost(net, edge)
    }

    /// Which ends of `edge`, a wire or a jog, `net`'s own block pins
    /// relieve of fixed metal; a via is never relieved.
    pub(crate) fn relief(&self, tech: &RouteTech, net: Option<NetId>, edge: Edge) -> Relief {
        let Some(bands) = net.and_then(|n| self.bands.get(&n)) else {
            return Relief::None;
        };
        let horizontal = match edge {
            Edge::Wire(n) => tech.layer(n.layer).direction == Direction::Horizontal,
            Edge::Jog(n) => tech.layer(n.layer).direction == Direction::Vertical,
            Edge::Via { .. } => return Relief::None,
        };
        relief_of(tech, bands, edge.ends(), horizontal)
    }
}

/// The ways out of an I/O pin's nodes on one layer its access points
/// `candidates` there leave blocked: the via up or down where none of them
/// takes it, and jogs across the tracks where one of them leaves along
/// them.
fn io_blocked<'a>(candidates: impl Iterator<Item = &'a AccessPoint>) -> Ways {
    let allowed = candidates.fold(Ways::NONE, |ways, a| ways.union(a.ways));
    let mut blocked = Ways::NONE;
    for via in [Way::Up, Way::Down] {
        if !allowed.contains(via) {
            blocked = blocked.with(via);
        }
    }
    if allowed.has_along() {
        blocked = blocked.with(Way::AcrossLower).with(Way::AcrossHigher);
    }
    blocked
}

/// Each end of `edge` with the way the edge leaves it by.
const fn ends_with_ways(edge: Edge) -> [(Node, Way); 2] {
    let (a, b) = edge.ends();
    let (from_a, from_b) = match edge {
        Edge::Wire(_) => (Way::AlongHigher, Way::AlongLower),
        Edge::Jog(_) => (Way::AcrossHigher, Way::AcrossLower),
        Edge::Via { .. } => (Way::Up, Way::Down),
    };
    [(a, from_a), (b, from_b)]
}

/// What one stage of the rules decides for an edge from the blocks
/// (`true`) and openings (`false`) it makes there: nothing without any,
/// else blocked if any blocks it.
fn verdict(marks: impl Iterator<Item = bool>) -> Option<bool> {
    marks.reduce(|blocked, next| blocked || next)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::access::access_points;
    use crate::detailed::DetailedRouter;
    use crate::tech::RouteSetup;
    use crate::tests::{macro_edge, tiny};
    use pare_common::config::RoutingConfig;

    fn tech_of(db: &NetlistDB) -> RouteTech {
        RouteTech::new(db, &RouteSetup::from(&RoutingConfig::default())).unwrap()
    }

    /// The node of `layer` at (`x`, `y`), a crossing of its tracks.
    fn node_at(tech: &RouteTech, layer: LayerId, x: i64, y: i64) -> Node {
        let lt = tech.layer(layer);
        let (across, along) = lt.across_along(Point::new(Dbu::new(x), Dbu::new(y)));
        let node = Node {
            layer,
            track: lt.tracks_within(across, across).start,
            cross: lt.cross_within(along, along).start,
        };
        assert_eq!(
            lt.point(node.track, node.cross),
            Point::new(Dbu::new(x), Dbu::new(y))
        );
        node
    }

    /// No access point for any pin of any net of `db`.
    fn no_access(db: &NetlistDB) -> IdVec<NetId, Vec<Vec<AccessPoint>>> {
        db.nets
            .iter()
            .map(|n| vec![Vec::new(); n.pins.len()])
            .collect()
    }

    /// Every pin's access points as pin access finds them, judged against
    /// the design's fixed shapes.
    fn found_access(db: &NetlistDB, tech: &RouteTech) -> IdVec<NetId, Vec<Vec<AccessPoint>>> {
        let config = RoutingConfig::default();
        let judge =
            DetailedRouter::new(db, tech, crate::gcell_size(tech, &config)).into_pin_judge();
        db.nets
            .iter_enumerated()
            .map(|(net, n)| {
                let clean = |layer: LayerId, metal: &[Rect<Dbu>]| judge.clean(net, layer, metal);
                n.pins
                    .iter()
                    .map(|&p| access_points(db, tech, p, &clean))
                    .collect()
            })
            .collect()
    }

    /// The macro edge fixture's metal3, the layer of its block's pins.
    const M3: LayerId = LayerId(2);

    #[test]
    fn an_access_point_blocks_the_ways_out_it_does_not_allow() {
        let db = macro_edge();
        let tech = tech_of(&db);
        let rules = EdgeRules::new(&db, &tech, &found_access(&db, &tech), Dbu::new(4000));
        let access = node_at(&tech, M3, 11390, 2380);

        assert!(rules.blocked(&tech, Some(NetId(0)), Edge::Jog(access)));
    }

    #[test]
    fn a_way_a_block_pins_access_point_allows_is_open_to_its_net_alone() {
        let db = macro_edge();
        let tech = tech_of(&db);
        let rules = EdgeRules::new(&db, &tech, &found_access(&db, &tech), Dbu::new(4000));
        let leaving = Edge::Wire(node_at(&tech, M3, 11210, 2380));

        assert!(!rules.blocked(&tech, Some(NetId(0)), leaving));
        assert!(rules.blocked(&tech, None, leaving));
    }

    #[test]
    fn wires_and_vias_at_a_node_inside_an_obstruction_are_blocked() {
        let db = macro_edge();
        let tech = tech_of(&db);
        let rules = EdgeRules::new(&db, &tech, &no_access(&db), Dbu::new(4000));
        let in_strip = node_at(&tech, M3, 11390, 2660);
        let above = node_at(&tech, LayerId(3), 11390, 2660);

        assert!(rules.blocked(
            &tech,
            Some(NetId(0)),
            Edge::Wire(node_at(&tech, M3, 11210, 2660))
        ));
        let via = Edge::Via {
            lower: in_strip,
            upper: above,
        };
        assert!(rules.blocked(&tech, Some(NetId(0)), via));
    }

    #[test]
    fn a_wire_inside_a_cell_pin_under_an_obstruction_is_open_but_not_its_via() {
        let mut db = tiny();
        let ram = db.cells[db.cell_id("ram0").unwrap()].macro_id;
        let m2 = LayerRef::Routing(LayerId(1));
        for s in db.library.macros[ram]
            .obstructions
            .iter_mut()
            .filter(|s| s.layer == m2)
        {
            s.rect.max.y = Dbu::new(4000);
        }
        let tech = tech_of(&db);
        let rules = EdgeRules::new(&db, &tech, &no_access(&db), Dbu::new(1000));
        let on_pin = node_at(&tech, LayerId(1), 4900, 3800);
        let via = Edge::Via {
            lower: on_pin,
            upper: node_at(&tech, LayerId(2), 4900, 3800),
        };

        assert!(!rules.blocked(&tech, Some(NetId(0)), Edge::Wire(on_pin)));
        assert!(rules.blocked(&tech, Some(NetId(0)), via));
    }

    #[test]
    fn an_io_pin_left_along_its_track_blocks_jogs_and_vias_inside_it() {
        let db = tiny();
        let tech = tech_of(&db);
        let io = &db.io_pins[db.io_pin_id("in").unwrap()];
        let at = node_at(&tech, LayerId(1), 1100, 5900);
        let mut access = no_access(&db);
        let k = db.nets[io.net]
            .pins
            .iter()
            .position(|&p| p == io.pin)
            .unwrap();
        access[io.net][k] = vec![AccessPoint {
            node: at,
            at: tech.layer(at.layer).point(at.track, at.cross),
            pin_layer: LayerId(1),
            vias: Vec::new(),
            ways: Ways::NONE.with(Way::AlongLower),
        }];
        let rules = EdgeRules::new(&db, &tech, &access, Dbu::new(1000));
        let inside = node_at(&tech, LayerId(1), 1100, 5800);
        let via = Edge::Via {
            lower: inside,
            upper: node_at(&tech, LayerId(2), 1100, 5800),
        };
        let along = Edge::Wire(node_at(&tech, LayerId(1), 1100, 5700));

        assert!(rules.blocked(&tech, Some(io.net), Edge::Jog(inside)));
        assert!(rules.blocked(&tech, Some(io.net), via));
        assert!(!rules.blocked(&tech, Some(io.net), along));
    }

    /// The macro edge fixture's rules with its pins' access points.
    fn macro_edge_rules() -> (RouteTech, EdgeRules) {
        let db = macro_edge();
        let tech = tech_of(&db);
        let rules = EdgeRules::new(&db, &tech, &found_access(&db, &tech), Dbu::new(4000));
        (tech, rules)
    }

    #[test]
    fn a_wire_out_of_a_block_pin_the_way_it_is_left_is_relieved_at_both_ends() {
        let (tech, rules) = macro_edge_rules();
        let leaving = Edge::Wire(node_at(&tech, M3, 11210, 2380));

        assert_eq!(rules.relief(&tech, Some(NetId(0)), leaving), Relief::Whole);
    }

    #[test]
    fn a_wire_ending_beside_a_block_pin_is_charged_at_its_far_end_only() {
        let (tech, rules) = macro_edge_rules();
        let far = node_at(&tech, M3, 10830, 2380);

        let relief = rules.relief(&tech, Some(NetId(0)), Edge::Wire(far));

        assert_eq!(relief, Relief::Partly { charged: far });
    }

    #[test]
    fn a_jog_beside_a_block_pin_left_along_its_track_is_not_relieved() {
        let (tech, rules) = macro_edge_rules();
        let access = node_at(&tech, M3, 11390, 2380);

        assert_eq!(
            rules.relief(&tech, Some(NetId(0)), Edge::Jog(access)),
            Relief::None
        );
    }

    #[test]
    fn another_nets_wire_beside_a_block_pin_is_not_relieved() {
        let (tech, rules) = macro_edge_rules();
        let leaving = Edge::Wire(node_at(&tech, M3, 11210, 2380));

        assert_eq!(rules.relief(&tech, None, leaving), Relief::None);
    }

    #[test]
    fn a_line_out_of_a_macro_access_point_takes_grid_cost_off_its_net_and_onto_others() {
        let (tech, rules) = macro_edge_rules();
        let leaving = Edge::Wire(node_at(&tech, M3, 11210, 2380));

        assert_eq!(rules.grid_cost(Some(NetId(0)), leaving), GridCost::Off);
        assert_eq!(rules.grid_cost(None, leaving), GridCost::On);
    }

    #[test]
    fn a_line_out_of_a_macro_access_point_takes_in_the_vias_along_it() {
        let (tech, rules) = macro_edge_rules();
        let via = Edge::Via {
            lower: node_at(&tech, M3, 11390, 2380),
            upper: node_at(&tech, LayerId(3), 11390, 2380),
        };

        assert_eq!(rules.grid_cost(None, via), GridCost::On);
    }

    #[test]
    fn a_line_out_of_a_macro_access_point_ends_ten_widths_out() {
        let (tech, rules) = macro_edge_rules();
        let far = Edge::Wire(node_at(&tech, M3, 9310, 2380));

        assert_eq!(rules.grid_cost(None, far), GridCost::AsLaidOut);
    }

    #[test]
    fn an_edge_away_from_pins_and_obstructions_is_open() {
        let db = tiny();
        let tech = tech_of(&db);
        let rules = EdgeRules::new(&db, &tech, &found_access(&db, &tech), Dbu::new(1000));

        let free = Edge::Wire(node_at(&tech, LayerId(1), 100, 100));

        assert!(!rules.blocked(&tech, Some(NetId(0)), free));
    }
}
