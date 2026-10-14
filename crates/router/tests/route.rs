//! Routing entry checks on the shared fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, PlacementStatus, RouteElem};
use pare_common::db::indices::LayerId;
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::util::config::RoutingConfig;
use pare_router::access::Node;
use pare_router::occupancy::{Occupancy, Origin, Owner};
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

    occupancy.remove_wiring(&tracks, owner);

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
