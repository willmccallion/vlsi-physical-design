//! Routing entry checks on the shared fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, PlacementStatus, RouteElem};
use pare_common::db::indices::{CutLayerId, LayerId};
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::util::config::RoutingConfig;
use pare_router::access::Node;
use pare_router::occupancy::{Occupancy, Origin, Owner, Slot};
use pare_router::tech::RouteTech;
use pare_router::wiring::{Edge, NetRoute};
use pare_router::{RouteError, route};
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

#[test]
fn routing_refuses_a_design_with_unplaced_cells() {
    let mut db = tiny();

    let result = route(&mut db, &RoutingConfig::default());

    assert_eq!(
        result,
        Err(RouteError::UnplacedCells {
            count: 2,
            first: "u5".to_string(),
        })
    );
}

#[test]
fn removing_a_nets_wiring_keeps_its_fixed_metal() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu(1000));
    let layer = LayerId(1);
    let around = |p: Point<Dbu>| Rect::new(p, p).expand(Dbu(50));
    let pin = around(tech.layer(layer).point(0, 0));
    let wire = around(tech.layer(layer).point(0, 20));
    let owner = Owner::Net(0);
    let mut tracks = occupancy.add_rect(&tech, layer, &pin, owner, Origin::Fixed);
    tracks.extend(occupancy.add_rect(&tech, layer, &wire, owner, Origin::Wiring));
    let slots: Vec<Slot> = tracks.into_iter().map(Slot::Track).collect();

    occupancy.remove_wiring(&slots, owner);

    let other = Owner::Net(1);
    assert_eq!(
        occupancy.rect_conflicts(&tech, layer, &pin, other).count(),
        1
    );
    assert_eq!(
        occupancy.rect_conflicts(&tech, layer, &wire, other).count(),
        0
    );
}

#[test]
fn cuts_of_different_nets_conflict_within_the_cut_spacing() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu(1000));
    let v1 = CutLayerId::new(0);
    let cut = |x: i64, y: i64| {
        Rect::new(
            Point::new(Dbu(x), Dbu(y)),
            Point::new(Dbu(x + 100), Dbu(y + 100)),
        )
    };
    let _ = occupancy.add_cut(v1, &cut(0, 0), Owner::Net(0), Origin::Wiring);

    let close = occupancy
        .cut_conflicts(v1, &cut(210, 0), Owner::Net(1))
        .count();
    let apart = occupancy
        .cut_conflicts(v1, &cut(220, 0), Owner::Net(1))
        .count();
    let diagonal = occupancy
        .cut_conflicts(v1, &cut(190, 190), Owner::Net(1))
        .count();
    let same_net = occupancy
        .cut_conflicts(v1, &cut(150, 0), Owner::Net(0))
        .count();

    assert_eq!((close, apart, diagonal, same_net), (1, 0, 0, 0));
}

#[test]
fn removing_a_nets_wiring_keeps_its_fixed_cuts() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu(1000));
    let v1 = CutLayerId::new(0);
    let cut = |x: i64| {
        Rect::new(
            Point::new(Dbu(x), Dbu(0)),
            Point::new(Dbu(x + 100), Dbu(100)),
        )
    };
    let owner = Owner::Net(0);
    let fixed = occupancy.add_cut(v1, &cut(0), owner, Origin::Fixed);
    let wired = occupancy.add_cut(v1, &cut(5000), owner, Origin::Wiring);

    occupancy.remove_wiring(&[Slot::Cut(fixed), Slot::Cut(wired)], owner);

    let other = Owner::Net(1);
    assert_eq!(occupancy.cut_conflicts(v1, &cut(150), other).count(), 1);
    assert_eq!(occupancy.cut_conflicts(v1, &cut(5150), other).count(), 0);
}

#[test]
fn wires_keep_the_table_spacing_from_a_long_wide_shape_and_the_minimum_from_a_wire() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let m2 = tech.layer(LayerId(1));
    let stripe = Rect::new(Point::new(Dbu(0), Dbu(0)), Point::new(Dbu(400), Dbu(2000)));
    let wire = Rect::new(Point::new(Dbu(0), Dbu(0)), Point::new(Dbu(100), Dbu(2000)));

    assert_eq!(m2.spacing_around(&stripe), (Dbu(200), Dbu(110)));
    assert_eq!(m2.spacing_around(&wire), (Dbu(110), Dbu(110)));
}

#[test]
fn routing_refuses_pins_of_different_nets_that_overlap() {
    let mut db = tiny();
    for cell in &mut db.cells {
        cell.status = PlacementStatus::Placed;
    }
    let nand = db
        .library
        .macros
        .iter()
        .position(|m| m.name == "NAND2")
        .unwrap();
    let pins = &mut db.library.macros[nand].pins;
    let a = pins.iter().position(|p| p.name == "A").unwrap();
    let b = pins.iter().position(|p| p.name == "B").unwrap();
    pins[b].shapes = pins[a].shapes.clone();

    let result = route(&mut db, &RoutingConfig::default());

    assert_eq!(
        result,
        Err(RouteError::OverlappingPins {
            count: 2,
            cell: "u3".to_string(),
            nets: ("n1".to_string(), "n2".to_string()),
        })
    );
}

#[test]
fn consecutive_wire_edges_become_one_wire() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let node = |cross| Node {
        layer: LayerId(1),
        track: 0,
        cross,
    };
    let route = NetRoute {
        edges: vec![
            Edge::Wire(node(2)),
            Edge::Wire(node(0)),
            Edge::Wire(node(1)),
        ],
        ..NetRoute::default()
    };

    let elems = route.elems(&tech);

    let lt = tech.layer(LayerId(1));
    assert_eq!(
        elems,
        [RouteElem::Wire {
            layer: LayerId(1),
            from: lt.point(0, 0),
            to: lt.point(0, 3),
        }]
    );
}
