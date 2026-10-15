//! I/O pin placement on the shared fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::library::LayerRef;
use pare_common::db::parser::{def, lef};
use pare_common::util::config::IoPlacementConfig;
use pare_placer::io::{IoLayers, place_io_pins};
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

fn layers(db: &NetlistDB) -> IoLayers {
    let config = IoPlacementConfig {
        horizontal_layer: Some("M3".to_string()),
        vertical_layer: Some("M2".to_string()),
        ..IoPlacementConfig::default()
    };
    IoLayers::from_config(&config, &db.library).unwrap()
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
fn placement_needs_both_layers_configured() {
    let db = tiny();
    let config = IoPlacementConfig {
        vertical_layer: Some("M2".to_string()),
        ..IoPlacementConfig::default()
    };

    let result = IoLayers::from_config(&config, &db.library);

    assert!(result.is_err());
}
