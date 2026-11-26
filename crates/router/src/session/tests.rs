use super::*;
use crate::tests::fixture;
use pare_common::config::RoutingConfig;
use pare_common::db::design::{NonDefaultLayer, NonDefaultRule, PlacementStatus};
use pare_common::db::parser::{def, lef};
use std::path::Path;

const fn cell(layer: u8, x: u16, y: u16) -> GCell {
    GCell {
        layer: LayerId(layer),
        x,
        y,
    }
}

fn center(c: GCell) -> Point<Dbu> {
    Point::new(
        Dbu::new(i64::from(c.x) * 10 + 5),
        Dbu::new(i64::from(c.y) * 10 + 5),
    )
}

fn terminal(pin: usize, cell: GCell, pin_layer: u8) -> PinTerminal {
    PinTerminal {
        pin: PinId::new(pin),
        cell,
        pin_layer: LayerId(pin_layer),
    }
}

/// A gcell's column, row and layer.
type At = (i64, i64, u8);

/// Each segment as its ends' gcell coordinates and layers, sorted.
fn ends(segments: &[RouteSegment]) -> Vec<(At, At)> {
    let at = |p: &RoutePoint| {
        (
            (p.at.x.get() - 5) / 10,
            (p.at.y.get() - 5) / 10,
            u8::try_from(p.layer.index()).unwrap(),
        )
    };
    let mut out: Vec<_> = segments
        .iter()
        .map(|s| {
            let (a, b) = (at(&s.from), at(&s.to));
            if a <= b { (a, b) } else { (b, a) }
        })
        .collect();
    out.sort_unstable();
    out
}

/// The tiny design placed, with a one-pin net `spare` on `u7` ahead of
/// `out`, which follows a wide rule.
fn tiny_with_a_rule_after_a_spare_net() -> NetlistDB {
    let text = std::fs::read_to_string(fixture("tiny.def"))
        .unwrap()
        .replace("NETS 5 ;", "NETS 6 ;")
        .replace("- out (", "- spare ( u7 Y ) ;\n- out (");
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    let mut db = def::parse_str(&text, Path::new("tiny.def"), library).unwrap();
    for (name, x) in [("u5", 2000), ("u6", 3000)] {
        let cell = db.cells.iter_mut().find(|c| c.name == name).unwrap();
        cell.location = Point::new(Dbu::new(x), Dbu::new(4000));
        cell.status = PlacementStatus::Placed;
    }
    let rule = db.non_default_rules.push(NonDefaultRule {
        name: "DW".to_string(),
        hard_spacing: true,
        layers: vec![NonDefaultLayer {
            layer: LayerId(0),
            width: Dbu::new(200),
            spacing: Some(Dbu::new(300)),
            wire_extension: None,
        }],
        vias: Vec::new(),
        via_rules: Vec::new(),
        min_cuts: Vec::new(),
    });
    let out = db.net_id("out").unwrap();
    db.nets[out].non_default_rule = Some(rule);
    db
}

/// Gives `net`'s route a stub one gcell out and back from its first
/// cell, which no router would choose, so that a route routed again
/// shows.
fn add_a_stub(session: &mut GlobalSession, net: NetId) {
    let placed = session.routes.trees.remove(&net).unwrap();
    let mut tree = session.grid.lift(placed);
    let from = tree.paths[0][0];
    let (nx, _) = session.grid.size();
    let x = if from.x + 1 < nx {
        from.x + 1
    } else {
        from.x - 1
    };
    tree.paths.push(vec![from, GCell { x, ..from }]);
    let _ = session.routes.trees.insert(net, session.grid.place(tree));
}

#[test]
fn a_restored_trial_puts_back_the_routes_it_changed_and_their_usage() {
    let (mut db, config) = crate::tests::placed_tiny();
    let mut session = GlobalSession::new(&db, &RouteSetup::from(&config), &mut |_| {}).unwrap();
    let u5 = db.cell_id("u5").unwrap();
    let nets: Vec<NetId> = db.cells[u5].pins.iter().map(|&p| db.pins[p].net).collect();
    let net = *nets
        .iter()
        .find(|n| session.routes.trees.contains_key(n))
        .unwrap();
    add_a_stub(&mut session, net);
    let routes = |s: &GlobalSession, db: &NetlistDB| -> Vec<Option<NetRoute>> {
        db.nets.ids().map(|n| s.route(db, n)).collect()
    };
    let (before, usage) = (routes(&session, &db), session.grid.usage_by_edge());
    let home = db.cells[u5].location;
    session.begin_trial();
    db.cells[u5].location = Point::new(Dbu::new(6000), home.y);
    session.update(&db, &nets);
    assert_ne!(routes(&session, &db), before);

    db.cells[u5].location = home;
    session.restore_trial();
    session.update(&db, &nets);

    assert_eq!(routes(&session, &db), before);
    assert_eq!(session.grid.usage_by_edge(), usage);
}

#[test]
fn detailed_routing_takes_each_nets_rule_as_it_is_after_renumbering() {
    let mut db = tiny_with_a_rule_after_a_spare_net();
    let config = RoutingConfig::default();
    let mut session = GlobalSession::new(&db, &RouteSetup::from(&config), &mut |_| {}).unwrap();
    let u7 = db.cell_id("u7").unwrap();
    let spare = db.net_id("spare").unwrap();
    let renumbering = db.remove(&[u7], &[spare]).unwrap();
    session.renumber(&renumbering);

    let (tech, ..) = session.into_parts(&db);

    let rules: Vec<_> = db.nets.ids().map(|n| tech.net_rule(n)).collect();
    let wanted: Vec<_> = db.nets.iter().map(|n| n.non_default_rule).collect();
    assert_eq!(rules, wanted);
}

#[test]
fn a_straight_run_is_one_wire_and_pins_climb_from_their_own_layer() {
    let tree = vec![vec![
        cell(1, 0, 0),
        cell(1, 1, 0),
        cell(1, 2, 0),
        cell(1, 3, 0),
    ]];
    let terminals = [terminal(0, cell(1, 0, 0), 0), terminal(1, cell(1, 3, 0), 0)];

    let segments = route_segments(&center, &tree, &terminals);

    assert_eq!(
        ends(&segments),
        vec![
            ((0, 0, 0), (0, 0, 1)),
            ((0, 0, 1), (3, 0, 1)),
            ((3, 0, 0), (3, 0, 1)),
        ]
    );
}

#[test]
fn a_run_breaks_where_a_branch_joins_it_turns_or_climbs() {
    let tree = vec![
        vec![
            cell(2, 0, 1),
            cell(2, 1, 1),
            cell(2, 2, 1),
            cell(2, 3, 1),
            cell(1, 3, 1),
            cell(1, 3, 0),
        ],
        vec![cell(2, 1, 1), cell(1, 1, 1), cell(1, 1, 2), cell(1, 1, 3)],
    ];
    let terminals = [
        terminal(0, cell(2, 0, 1), 2),
        terminal(1, cell(1, 3, 0), 1),
        terminal(2, cell(1, 1, 3), 1),
    ];

    let segments = route_segments(&center, &tree, &terminals);

    assert_eq!(
        ends(&segments),
        vec![
            ((0, 1, 2), (1, 1, 2)),
            ((1, 1, 1), (1, 1, 2)),
            ((1, 1, 1), (1, 3, 1)),
            ((1, 1, 2), (3, 1, 2)),
            ((3, 0, 1), (3, 1, 1)),
            ((3, 1, 1), (3, 1, 2)),
        ]
    );
}
