//! Pin access on the shared fixture design.

use super::{macro_edge, tiny};
use crate::access::ways::Way;
use crate::access::{AccessPoint, Node, Ways, access_points};
use crate::detailed::DetailedRouter;
use crate::shared_access::SharedAccess;
use crate::tech::RouteSetup;
use crate::tech::{CrossIdx, RouteTech, TrackIdx};
use pare_common::config::RoutingConfig;
use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{LayerId, PinId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::Interval;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

fn cell_pin(db: &NetlistDB, cell: &str, pin: &str) -> PinId {
    let c = db.cell_id(cell).unwrap();
    let m = db.cell_macro(c);
    *db.cells[c]
        .pins
        .iter()
        .find(|&&p| matches!(db.pins[p].owner, PinOwner::Cell { pin: mp, .. } if m.pin(mp).name == pin))
        .unwrap()
}

#[test]
fn routing_defaults_to_the_second_layer_and_up() {
    let db = tiny();

    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();

    assert_eq!((tech.min_layer, tech.max_layer), (LayerId(1), LayerId(2)));
}

#[test]
fn crossings_merge_the_tracks_of_both_neighbouring_layers() {
    let db = tiny();

    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();

    let first: Vec<i64> = tech.layer(LayerId(1)).cross.as_slice()[..6]
        .iter()
        .map(|c| c.at.get())
        .collect();
    assert_eq!(first, [100, 200, 300, 500, 600, 700]);
}

#[test]
fn an_m1_pin_is_reached_by_the_one_via_that_fits_inside_it() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();

    let points = access_points(&db, &tech, cell_pin(&db, "u1", "A"), &|_, _| true);

    assert_eq!(
        points,
        [AccessPoint {
            node: Node {
                layer: LayerId(1),
                track: TrackIdx(1),
                cross: CrossIdx(3)
            },
            at: Point::new(Dbu::new(300), Dbu::new(500)),
            pin_layer: LayerId(0),
            vias: vec![db.library.via_id("M1M2").unwrap()],
            ways: Ways::ALL,
        }]
    );
}

#[test]
fn an_io_pin_on_a_routable_layer_is_reached_on_its_own_layer() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();
    let pin = db.io_pins[db.io_pin_id("in").unwrap()].pin;

    let points = access_points(&db, &tech, pin, &|_, _| true);

    assert_ne!(points, []);
    assert!(
        points
            .iter()
            .all(|p| p.node.layer == LayerId(1) && p.pin_layer == LayerId(1))
    );
}

fn io_pin(db: &NetlistDB, name: &str) -> PinId {
    db.io_pins[db.io_pin_id(name).unwrap()].pin
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
        let (Interval { lo: a0, hi: a1 }, Interval { lo: b0, hi: b1 }) =
            lt.across_along_rect(&shape.rect);
        for track in lt.tracks_within(a0, a1) {
            for cross in lt.cross_within(b0, b1) {
                found.push(lt.point(track, cross));
            }
        }
    }
    found
}

#[test]
fn crossings_with_no_clean_way_out_are_dropped() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();
    let pin = io_pin(&db, "in");
    let unclean = Point::new(Dbu::new(1100), Dbu::new(5900));
    assert!(crossings_inside(&db, &tech, pin).contains(&unclean));
    let clear_of_it = |_: LayerId, metal: &[Rect<Dbu>]| !metal.iter().any(|m| m.contains(unclean));

    let points = access_points(&db, &tech, pin, &clear_of_it);

    assert_ne!(points, []);
    assert!(points.iter().all(|a| a.at != unclean), "{points:?}");
}

#[test]
fn a_pin_with_no_crossing_inside_that_has_a_way_out_is_reached_from_a_crossing_beyond_its_end() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();
    let pin = io_pin(&db, "in");
    let bottom = Dbu::new(5800);
    let only_with_a_stub = |_: LayerId, metal: &[Rect<Dbu>]| metal.len() == 2;

    let points = access_points(&db, &tech, pin, &only_with_a_stub);

    assert_ne!(points, []);
    for a in &points {
        let node = tech.layer(a.node.layer).point(a.node.track, a.node.cross);
        assert!(node.y < bottom && a.at.y == bottom, "{a:?}");
    }
}

#[test]
fn a_pin_with_no_way_out_keeps_its_crossings_with_every_way_closed() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();
    let pin = io_pin(&db, "in");
    let place = |a: &AccessPoint| (a.node, a.at);

    let unjudged = access_points(&db, &tech, pin, &|_, _| true);
    let none_clean = access_points(&db, &tech, pin, &|_, _| false);

    assert_eq!(
        none_clean.iter().map(place).collect::<Vec<_>>(),
        unjudged.iter().map(place).collect::<Vec<_>>()
    );
    assert!(
        none_clean.iter().all(|a| a.ways == Ways::NONE),
        "{none_clean:?}"
    );
}

/// The tiny fixture with its I/O pin `in` stretched to a micron long, down
/// from the die's top edge.
fn tiny_with_a_long_io_pin() -> (NetlistDB, PinId) {
    let mut db = tiny();
    let io = db.io_pin_id("in").unwrap();
    db.io_pins[io].shapes[0].rect = Rect::new(
        Point::new(Dbu::new(-50), Dbu::ZERO),
        Point::new(Dbu::new(50), Dbu::new(1000)),
    );
    let pin = db.io_pins[io].pin;
    (db, pin)
}

#[test]
fn a_way_whose_wire_would_end_inside_the_pin_is_closed() {
    let (db, pin) = tiny_with_a_long_io_pin();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();
    let middle = Point::new(Dbu::new(1100), Dbu::new(5500));

    let points = access_points(&db, &tech, pin, &|_, _| true);

    let at_middle = points.iter().find(|a| a.at == middle).unwrap();
    let across = Ways::NONE.with(Way::AcrossLower).with(Way::AcrossHigher);
    assert_eq!(at_middle.ways, across);
}

#[test]
fn an_io_pin_left_on_its_own_layer_gets_no_via_way() {
    let (db, pin) = tiny_with_a_long_io_pin();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();

    let points = access_points(&db, &tech, pin, &|_, _| true);

    assert_ne!(points, []);
    assert!(
        points
            .iter()
            .all(|a| !a.ways.contains(Way::Up) && !a.ways.contains(Way::Down)),
        "{points:?}"
    );
}

#[test]
fn a_crossing_inside_a_block_pin_between_obstruction_strips_is_left_away_from_the_block() {
    let db = macro_edge();
    let config = RoutingConfig::default();
    let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
    let judge = DetailedRouter::new(&db, &tech, crate::gcell_size(&tech, &config)).into_pin_judge();
    let pin = cell_pin(&db, "ram", "P[0]");
    let net = db.pins[pin].net;
    let clean = |layer: LayerId, metal: &[Rect<Dbu>]| judge.clean(net, layer, metal);

    let points = access_points(&db, &tech, pin, &clean);

    let found: Vec<(Point<Dbu>, Ways)> = points.iter().map(|a| (a.at, a.ways)).collect();
    let crossing = Point::new(Dbu::new(11390), Dbu::new(2380));
    assert_eq!(found, [(crossing, Ways::NONE.with(Way::AlongLower))]);
}

#[test]
fn shared_candidates_equal_those_found_for_each_pin() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();
    let shared = SharedAccess::new(&db, &tech);
    let cell_pins = db.cells.iter().flat_map(|c| c.pins.iter().copied());

    let mut compared = 0;
    for pin in cell_pins {
        if let Some(derived) = shared.candidates(&db, pin) {
            assert_eq!(
                derived,
                access_points(&db, &tech, pin, &|_, _| true),
                "{pin:?}"
            );
            assert_eq!(shared.count(&db, pin), Some(derived.len()));
            compared += 1;
        }
    }

    assert!(compared > 0);
}
