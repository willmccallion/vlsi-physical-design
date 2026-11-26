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
    let judge = DetailedRouter::new(db, tech, crate::gcell_size(tech, &config)).into_pin_judge();
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
fn a_wire_stops_at_its_own_nets_access_point_touching_its_pin() {
    let (tech, rules) = macro_edge_rules();
    let access = node_at(&tech, M3, 11390, 2380);

    assert!(rules.cut_off_at(Some(NetId(0)), access));
    assert!(!rules.cut_off_at(None, access));
}

#[test]
fn the_via_up_from_an_access_point_that_takes_it_is_its_nets_access_via() {
    let db = tiny();
    let tech = tech_of(&db);
    let pin = db.io_pins[db.io_pin_id("in").unwrap()].pin;
    let net = db.pins[pin].net;
    let at = node_at(&tech, LayerId(1), 1100, 5900);
    let mut access = no_access(&db);
    let k = db.nets[net].pins.iter().position(|&p| p == pin).unwrap();
    access[net][k] = vec![AccessPoint {
        node: at,
        at: tech.layer(at.layer).point(at.track, at.cross),
        pin_layer: LayerId(1),
        vias: Vec::new(),
        ways: Ways::NONE.with(Way::Up),
    }];
    let rules = EdgeRules::new(&db, &tech, &access, Dbu::new(1000));

    assert!(rules.via_access_at(Some(net), at));
    assert!(!rules.via_access_at(None, at));
}

#[test]
fn an_edge_away_from_pins_and_obstructions_is_open() {
    let db = tiny();
    let tech = tech_of(&db);
    let rules = EdgeRules::new(&db, &tech, &found_access(&db, &tech), Dbu::new(1000));

    let free = Edge::Wire(node_at(&tech, LayerId(1), 100, 100));

    assert!(!rules.blocked(&tech, Some(NetId(0)), free));
}
