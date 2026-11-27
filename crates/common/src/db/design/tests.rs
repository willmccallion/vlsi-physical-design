use super::*;
use crate::db::parser::{def, lef};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

fn tiny() -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse(&fixture("tiny.def"), library).unwrap()
}

const fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(pt(x0, y0), pt(x1, y1))
}

#[test]
fn a_row_holds_sites_only_with_some_in_each_direction() {
    let row = tiny().rows[0].clone();

    assert!(row.has_sites());
    assert!(
        !Row {
            num_x: 0,
            ..row.clone()
        }
        .has_sites()
    );
    assert!(!Row { num_y: 0, ..row }.has_sites());
}

/// A wire 20 wide from `from` to `to`, extending 3 past `from` and 5 past
/// `to`.
fn wire(from: Point<Dbu>, to: Point<Dbu>) -> SpecialWire {
    SpecialWire {
        layer: LayerId::new(0),
        width: Dbu::new(20),
        from,
        to,
        extensions: (Dbu::new(3), Dbu::new(5)),
        shape: None,
    }
}

#[test]
fn a_special_wire_covers_its_ends_extensions_whichever_way_it_runs() {
    assert_eq!(wire(pt(0, 100), pt(50, 100)).rect(), rect(-3, 90, 55, 110));
    assert_eq!(wire(pt(50, 100), pt(0, 100)).rect(), rect(-5, 90, 53, 110));
    assert_eq!(wire(pt(100, 0), pt(100, 50)).rect(), rect(90, -3, 110, 55));
    assert_eq!(wire(pt(100, 50), pt(100, 0)).rect(), rect(90, -5, 110, 53));
}

#[test]
fn a_pin_s_reference_layer_is_the_lowest_it_has_a_shape_on() {
    let db = tiny();
    let layer = |name: &str| db.library.routing_layer_id(name).unwrap();
    let u1 = db.cell_id("u1").unwrap();
    let a = db
        .library
        .cell_macro(db.cells[u1].macro_id)
        .pin_id("A")
        .unwrap();
    let io = db.io_pins[db.io_pin_id("in").unwrap()].pin;

    assert_eq!(
        db.pin_reference_layer(db.cell_pin(u1, a).unwrap()),
        Some(layer("M1"))
    );
    assert_eq!(db.pin_reference_layer(io), Some(layer("M2")));
}

/// Horizontal tracks of `layer` from `start`, `count` of them `step` apart.
fn y_tracks(layer: LayerId, start: i64, count: u32, step: i64) -> TrackDef {
    TrackDef {
        axis: TrackAxis::Y,
        start: Dbu::new(start),
        count,
        step: Dbu::new(step),
        layers: vec![layer],
    }
}

#[test]
fn several_track_patterns_average_their_span_over_their_tracks_rounding_up() {
    let mut db = tiny();
    let m3 = db.library.routing_layer_id("M3").unwrap();
    db.tracks = vec![y_tracks(m3, 0, 3, 10), y_tracks(m3, 5, 2, 10)];

    assert_eq!(db.average_track_spacing(m3), Dbu::new(4));
}

fn coords(values: &[i64]) -> Vec<Dbu> {
    values.iter().copied().map(Dbu::new).collect()
}

#[test]
fn a_layer_without_tracks_gets_one_per_pitch_from_half_a_pitch_or_its_offset() {
    let mut db = tiny();
    db.tracks.clear();
    let m3 = db.library.routing_layer_id("M3").unwrap();

    let halfway = db.layer_tracks(m3);
    db.library.routing_layers[m3].offset_y = Some(Dbu::ZERO);
    let from_edge = db.layer_tracks(m3);

    assert_eq!(
        halfway,
        coords(&(0..15).map(|k| 200 + 400 * k).collect::<Vec<_>>())
    );
    assert_eq!(
        from_edge,
        coords(&(0..15).map(|k| 400 * k).collect::<Vec<_>>())
    );
}

#[test]
fn tracks_whose_wire_would_leave_the_die_are_dropped() {
    let mut db = tiny();
    let m3 = db.library.routing_layer_id("M3").unwrap();
    db.tracks = vec![y_tracks(m3, 50, 3, 50), y_tracks(m3, 5850, 3, 50)];

    assert_eq!(db.layer_tracks(m3), coords(&[100, 150, 5850, 5900]));
}

#[test]
fn a_row_of_sites_stacked_upward_spans_each_step_and_its_last_site() {
    let row = Row {
        num_x: 1,
        num_y: 3,
        step: pt(0, 2000),
        ..tiny().rows[0].clone()
    };

    // The core site is 200 by 2000.
    assert_eq!(row.bbox(&tiny().library), rect(0, 0, 200, 6000));
}

/// The cell named `name`.
fn cell(db: &NetlistDB, name: &str) -> CellId {
    db.cell_id(name).unwrap()
}

#[test]
fn a_placed_cell_covers_its_macro_from_its_location() {
    let db = tiny();
    let u1 = cell(&db, "u1");

    assert_eq!(db.num_cells(), 8);
    assert_eq!(db.cell_size(u1), pt(400, 2000));
    assert_eq!(db.cell_bbox(u1), rect(200, 0, 600, 2000));
}

#[test]
fn the_core_is_the_box_around_the_rows_not_of_pad_sites() {
    let mut db = tiny();
    db.rows.truncate(1);
    let core_rows = db.core_area();
    let site = db.rows[0].site;
    db.library.sites[site].class = SiteClass::Pad;

    let pad_rows = db.core_area();

    assert_eq!(core_rows, rect(0, 0, 6000, 2000));
    assert_eq!(pad_rows, db.die_area);
}

#[test]
fn a_layers_one_track_pattern_sets_its_spacing_and_tracks() {
    let db = tiny();
    let m3 = db.library.routing_layer_id("M3").unwrap();

    assert_eq!(db.average_track_spacing(m3), Dbu::new(400));
    assert_eq!(
        db.layer_tracks(m3),
        coords(&(0..15).map(|k| 200 + 400 * k).collect::<Vec<_>>())
    );
}

#[test]
fn the_span_of_several_track_patterns_runs_from_their_first_track() {
    let mut db = tiny();
    let m3 = db.library.routing_layer_id("M3").unwrap();
    db.tracks = vec![y_tracks(m3, 100, 3, 10), y_tracks(m3, 105, 2, 10)];

    assert_eq!(db.average_track_spacing(m3), Dbu::new(4));
}

#[test]
fn a_layer_without_tracks_or_pitch_has_no_tracks() {
    let mut db = tiny();
    db.tracks.clear();
    let m3 = db.library.routing_layer_id("M3").unwrap();
    db.library.routing_layers[m3].pitch_y = Dbu::ZERO;

    assert_eq!(db.layer_tracks(m3), Vec::new());
}

/// The connection named `name`, as [`NetlistDB::pin_name`] gives it.
fn pin(db: &NetlistDB, name: &str) -> PinId {
    db.pins.ids().find(|&p| db.pin_name(p) == name).unwrap()
}

#[test]
fn a_cell_pins_shapes_are_placed_with_its_cell() {
    let db = tiny();

    let rects: Vec<Rect<Dbu>> = db
        .pin_shapes(pin(&db, "u1/Y"))
        .iter()
        .map(|s| s.rect)
        .collect();

    assert_eq!(
        rects,
        [rect(450, 200, 550, 1800), rect(450, 1000, 500, 1100)]
    );
}

#[test]
fn a_pin_sits_at_the_mean_of_its_shapes_corners_truncated() {
    let db = tiny();

    let positions = ["u1/A", "u1/Y", "PIN in"].map(|name| db.average_pin_position(pin(&db, name)));

    // u1/Y's corners average to x 487.5; the pin `in` is turned about its point.
    assert_eq!(positions, [pt(300, 500), pt(487, 1025), pt(1100, 5900)]);
}
