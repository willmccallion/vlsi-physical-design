//! Pin access on the shared fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{LayerId, PinId};
use pare_common::db::library::LayerRef;
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::util::config::RoutingConfig;
use pare_router::access::{AccessPoint, Node, access_points};
use pare_router::tech::RouteTech;
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

fn cell_pin(db: &NetlistDB, cell: &str, pin: &str) -> PinId {
    let c = db.cell_id(cell).unwrap();
    let m = db.cell_macro(c);
    *db.cells[c.index()]
        .pins
        .iter()
        .find(|&&p| matches!(db.pins[p.index()].owner, PinOwner::Cell { pin: mp, .. } if m.pin(mp).name == pin))
        .unwrap()
}

#[test]
fn routing_defaults_to_the_second_layer_and_up() {
    let db = tiny();

    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();

    assert_eq!((tech.min_layer, tech.max_layer), (LayerId(1), LayerId(2)));
}

#[test]
fn crossings_merge_the_tracks_of_both_neighbouring_layers() {
    let db = tiny();

    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();

    let first: Vec<i64> = tech.layer(LayerId(1)).cross[..6]
        .iter()
        .map(|c| c.at.get())
        .collect();
    assert_eq!(first, [100, 200, 300, 500, 600, 700]);
}

#[test]
fn an_m1_pin_is_reached_by_the_one_via_that_fits_inside_it() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();

    let points = access_points(&db, &tech, cell_pin(&db, "u1", "A"), &|_, _| true);

    assert_eq!(
        points,
        [AccessPoint {
            node: Node {
                layer: LayerId(1),
                track: 1,
                cross: 3
            },
            at: Point::new(Dbu(300), Dbu(500)),
            pin_layer: LayerId(0),
            vias: vec![db.library.via_id("M1M2").unwrap()],
        }]
    );
}

#[test]
fn an_io_pin_on_a_routable_layer_is_reached_on_its_own_layer() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let pin = db.io_pins[db.io_pin_id("in").unwrap().index()].pin;

    let points = access_points(&db, &tech, pin, &|_, _| true);

    assert!(!points.is_empty());
    assert!(
        points
            .iter()
            .all(|p| p.node.layer == LayerId(1) && p.pin_layer == LayerId(1))
    );
}

fn io_pin(db: &NetlistDB, name: &str) -> PinId {
    db.io_pins[db.io_pin_id(name).unwrap().index()].pin
}

/// Every crossing of its own layer's tracks inside a shape of `pin`.
fn crossings_inside(db: &NetlistDB, tech: &RouteTech, pin: PinId) -> Vec<Point<Dbu>> {
    let mut found = Vec::new();
    for shape in db.pin_shapes(pin) {
        let LayerRef::Routing(layer) = shape.layer else {
            continue;
        };
        if !tech.routable(layer) {
            continue;
        }
        let lt = tech.layer(layer);
        let ((a0, a1), (b0, b1)) = lt.across_along_rect(&shape.rect);
        for track in lt.tracks_within(a0, a1) {
            for cross in lt.cross_within(b0, b1) {
                found.push(lt.point(track, cross));
            }
        }
    }
    found
}

#[test]
fn crossings_whose_wire_end_is_not_clean_are_dropped() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let pin = io_pin(&db, "in");
    let unclean = Point::new(Dbu(1100), Dbu(5900));
    assert!(crossings_inside(&db, &tech, pin).contains(&unclean));

    let points = access_points(&db, &tech, pin, &|_, m: &Rect<Dbu>| !m.contains(unclean));

    assert!(!points.is_empty());
    assert!(points.iter().all(|a| a.at != unclean), "{points:?}");
}

#[test]
fn a_pin_with_no_clean_crossing_inside_is_reached_from_a_crossing_beyond_its_end() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let pin = io_pin(&db, "in");
    let bottom = Dbu(5800);
    let reaches_below = |_: LayerId, m: &Rect<Dbu>| m.min.y < bottom - Dbu(100);

    let points = access_points(&db, &tech, pin, &reaches_below);

    assert!(!points.is_empty());
    for a in &points {
        let node = tech.layer(a.node.layer).point(a.node.track, a.node.cross);
        assert!(node.y < bottom && a.at.y == bottom, "{a:?}");
    }
}

#[test]
fn a_pin_with_no_clean_access_keeps_its_crossings() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let pin = io_pin(&db, "in");

    let unjudged = access_points(&db, &tech, pin, &|_, _| true);
    let none_clean = access_points(&db, &tech, pin, &|_, _| false);

    assert_eq!(none_clean, unjudged);
}
