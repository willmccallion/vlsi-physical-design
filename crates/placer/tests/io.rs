//! I/O pin placement on the shared fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::config::IoLayerNames;
use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::library::LayerRef;
use pare_common::db::parser::{def, lef};
use pare_placer::io::{IoLayers, IoPlacementError, place_io_pins};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

fn tiny() -> NetlistDB {
    tiny_edited(|text| text)
}

/// The tiny design from its DEF as `edit` rewrites it.
fn tiny_edited(edit: impl Fn(String) -> String) -> NetlistDB {
    let text = edit(std::fs::read_to_string(fixture("tiny.def")).unwrap());
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse_str(&text, &fixture("tiny.def"), library).unwrap()
}

fn layers(db: &NetlistDB) -> IoLayers {
    let names = IoLayerNames {
        horizontal: "M3".to_string(),
        vertical: "M2".to_string(),
    };
    IoLayers::from_names(&names, &db.library).unwrap()
}

fn pin<'a>(db: &'a NetlistDB, name: &str) -> &'a pare_common::db::design::IoPin {
    db.io_pins.iter().find(|p| p.name == name).unwrap()
}

#[test]
fn fixed_pins_stay_where_they_are() {
    let mut db = tiny();
    let before = pin(&db, "in").clone();
    let io = layers(&db);

    let _ = place_io_pins(&mut db, io).unwrap();

    assert_eq!(pin(&db, "in"), &before);
}

#[test]
fn a_placed_pin_moves_onto_the_boundary_on_the_layer_crossing_its_edge() {
    let mut db = tiny();
    let die = db.die_area;
    let io = layers(&db);

    let placed = place_io_pins(&mut db, io).unwrap();

    let out = pin(&db, "out");
    let shapes: Vec<_> = out.placed_shapes().collect();
    assert_eq!(placed.pins, 1);
    assert_eq!(out.status, PlacementStatus::Placed);
    assert_eq!(shapes.len(), 1);
    let rect = shapes[0].rect;
    let on_side = rect.min.x == die.min.x || rect.max.x == die.max.x;
    let on_end = rect.min.y == die.min.y || rect.max.y == die.max.y;
    let expected = if on_side { io.horizontal } else { io.vertical };
    assert!(
        on_side || on_end,
        "{rect:?} is not on the boundary of {die:?}"
    );
    assert_eq!(shapes[0].layer, LayerRef::Routing(expected));
    assert!(die.contains(rect.min) && die.contains(rect.max));
}

#[test]
fn a_pin_layer_running_the_wrong_way_is_rejected() {
    let db = tiny();
    let names = IoLayerNames {
        horizontal: "M2".to_string(),
        vertical: "M3".to_string(),
    };

    let result = IoLayers::from_names(&names, &db.library);

    assert_eq!(
        result,
        Err(IoPlacementError::WrongDirection {
            layer: "M2".to_string(),
            direction: "horizontal",
        })
    );
}

#[test]
fn a_pin_layer_that_is_no_routing_layer_is_rejected() {
    let db = tiny();
    let names = IoLayerNames {
        horizontal: "M9".to_string(),
        vertical: "M2".to_string(),
    };

    let result = IoLayers::from_names(&names, &db.library);

    assert_eq!(
        result,
        Err(IoPlacementError::UnknownLayer("M9".to_string()))
    );
}

#[test]
fn a_pin_layer_without_tracks_is_an_error() {
    let mut db = tiny_edited(|text| text.replace("TRACKS Y 200 DO 15 STEP 400 LAYER M3 ;", ""));
    let io = layers(&db);

    let result = place_io_pins(&mut db, io);

    assert_eq!(result, Err(IoPlacementError::NoTracks("M3".to_string())));
}

/// One track per pin layer, each within a micron of a corner, where no
/// pin may go.
#[test]
fn a_boundary_without_free_slots_for_the_pins_is_an_error() {
    let mut db = tiny_edited(|text| {
        text.replace("DO 15 STEP 400 LAYER M3", "DO 1 STEP 400 LAYER M3")
            .replace(
                "TRACKS X 100 DO 30 STEP 200 LAYER M2",
                "TRACKS X 100 DO 1 STEP 200 LAYER M2",
            )
    });
    let io = layers(&db);

    let result = place_io_pins(&mut db, io);

    assert_eq!(
        result,
        Err(IoPlacementError::TooFewSlots { pins: 1, slots: 0 })
    );
}
