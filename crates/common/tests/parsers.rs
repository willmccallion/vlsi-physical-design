//! Parser and writer tests against small fixture designs.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, PlacementStatus, RouteElem};
use pare_common::db::indices::{CellId, CutLayerId, LayerId, PinId};
use pare_common::db::library::{Direction, LayerRef, Library, MacroClass, Shape};
use pare_common::db::parser::{bookshelf, def, lef};
use pare_common::db::writer;
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

const fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu(x), Dbu(y))
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::from_corners(pt(x0, y0), pt(x1, y1))
}

fn tiny_library() -> Library {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    lef::parse_files(&[fixture("tiny.lef")], units).unwrap()
}

fn tiny_design() -> NetlistDB {
    def::parse(&fixture("tiny.def"), tiny_library()).unwrap()
}

fn cell(db: &NetlistDB, name: &str) -> CellId {
    db.cell_id(name).unwrap()
}

fn cell_pin(db: &NetlistDB, cell_name: &str, pin_name: &str) -> PinId {
    let c = cell(db, cell_name);
    let m = db.cell_macro(c);
    *db.cells[c.index()]
        .pins
        .iter()
        .find(|&&p| {
            matches!(db.pins[p.index()].owner,
                pare_common::db::design::PinOwner::Cell { pin, .. } if m.pin(pin).name == pin_name)
        })
        .unwrap()
}

#[test]
fn def_units_are_read_before_the_body() {
    assert_eq!(
        def::read_units(&fixture("tiny.def")).unwrap(),
        DbuPerMicron::new(1000).unwrap()
    );
}

#[test]
fn lef_routing_layers_carry_direction_pitch_offset_width_and_spacing() {
    let lib = tiny_library();

    let m1 = lib.layer(LayerId(0));
    let m2 = lib.layer(LayerId(1));
    let m3 = lib.layer(LayerId(2));
    assert_eq!(lib.routing_layers.len(), 3);
    assert_eq!(
        (m1.direction, m1.track_pitch(), m1.offset_y),
        (Direction::Horizontal, Dbu(200), Some(Dbu(100)))
    );
    assert_eq!(
        (m2.direction, m2.pitch_x, m2.pitch_y),
        (Direction::Vertical, Dbu(200), Dbu(250))
    );
    assert_eq!(m2.offset_y, Some(Dbu(125)));
    assert_eq!(m3.offset_x, None);
}

#[test]
fn lef_min_spacing_comes_from_plain_spacing_or_the_first_table_entry() {
    let lib = tiny_library();

    let spacings: Vec<Dbu> = lib.routing_layers.iter().map(|l| l.min_spacing).collect();
    assert_eq!(spacings, [Dbu(100), Dbu(110), Dbu(200)]);
}

#[test]
fn lef_cut_layers_sit_between_routing_layers() {
    let lib = tiny_library();

    assert_eq!(lib.cut_layers.len(), 2);
    assert_eq!(lib.layer_ref("V2"), Some(LayerRef::Cut(CutLayerId(1))));
    assert_eq!(lib.layer_ref("poly"), None);
}

#[test]
fn lef_default_vias_are_found_per_cut() {
    let lib = tiny_library();

    let via = lib.via(lib.default_via(CutLayerId(0)).unwrap());
    assert_eq!(via.name, "M1M2");
    assert_eq!((via.bottom(), via.top()), (LayerId(0), LayerId(1)));
}

#[test]
fn lef_macros_keep_pin_shapes_and_drop_shapes_on_masterslice_layers() {
    let lib = tiny_library();

    let inv = lib.cell_macro(lib.macro_id("INV").unwrap());
    assert_eq!(inv.size, pt(400, 2000));
    assert_eq!(inv.obstructions.len(), 1);
    let y = inv.pin(inv.pin_id("Y").unwrap());
    assert_eq!(y.shapes.len(), 2);
    assert_eq!(y.reference_point(), Some(pt(300, 1000)));
    let ram = lib.cell_macro(lib.macro_id("RAM").unwrap());
    assert_eq!(ram.class, MacroClass::Block);
}

#[test]
fn def_reads_rows_tracks_and_die_area() {
    let db = tiny_design();

    assert_eq!(db.name, "tiny");
    assert_eq!(db.die_area, rect(0, 0, 6000, 6000));
    assert_eq!(db.rows.len(), 3);
    assert_eq!(db.rows[1].orient, Orientation::FS);
    assert_eq!(db.rows[2].bbox(&db.library), rect(0, 4000, 4000, 6000));
    assert_eq!(db.tracks[2].layers, [LayerId(2)]);
}

#[test]
fn def_viarule_vias_generate_a_cut_array_with_enclosing_metal() {
    let db = tiny_design();

    let via = db.library.via(db.library.via_id("M1M2_2CUT").unwrap());
    let cuts: Vec<Rect<Dbu>> = via
        .shapes
        .iter()
        .filter(|s| s.layer == LayerRef::Cut(CutLayerId(0)))
        .map(|s| s.rect)
        .collect();
    assert_eq!(cuts, [rect(-150, -50, -50, 50), rect(50, -50, 150, 50)]);
    assert!(via.shapes.contains(&Shape {
        layer: LayerRef::Routing(LayerId(0)),
        rect: rect(-150, -100, 150, 100)
    }));
}

#[test]
fn def_components_keep_status_and_orientation() {
    let db = tiny_design();

    let status = |n: &str| db.cells[cell(&db, n).index()].status;
    assert_eq!(status("u1"), PlacementStatus::Placed);
    assert_eq!(status("u3"), PlacementStatus::Fixed);
    assert_eq!(status("u5"), PlacementStatus::Unplaced);
    assert_eq!(status("u6"), PlacementStatus::Unplaced);
    assert_eq!(db.cells[cell(&db, "u4").index()].orient, Orientation::FS);
}

#[test]
fn pin_positions_follow_the_cell_orientation() {
    let db = tiny_design();

    assert_eq!(db.pin_position(cell_pin(&db, "u1", "A")), pt(300, 500));
    assert_eq!(db.pin_position(cell_pin(&db, "u2", "A")), pt(1100, 3500));
    assert_eq!(db.pin_position(cell_pin(&db, "u7", "A")), pt(900, 5500));
}

#[test]
fn io_pin_shapes_rotate_about_the_pin_location() {
    let db = tiny_design();

    let pin = &db.io_pins[db.io_pin_id("in").unwrap().index()];
    let shapes: Vec<Shape> = pin.placed_shapes().collect();
    assert_eq!(shapes[0].rect, rect(1050, 5800, 1150, 6000));
    assert_eq!(db.pin_position(pin.pin), pt(1100, 5900));
}

#[test]
fn def_nets_connect_cell_and_io_pins() {
    let db = tiny_design();

    let out = &db.nets[db.net_id("out").unwrap().index()];
    assert_eq!(out.pins.len(), 2);
    let n3 = &db.nets[db.net_id("n3").unwrap().index()];
    assert_eq!(n3.pins.len(), 6);
    assert!((db.nets[db.net_id("n2").unwrap().index()].weight - 2.0).abs() < f64::EPSILON);
}

#[test]
fn def_regular_wiring_switches_layers_at_vias() {
    let db = tiny_design();

    let n3 = &db.nets[db.net_id("n3").unwrap().index()];
    let m1m2 = db.library.via_id("M1M2").unwrap();
    let m2m3 = db.library.via_id("M2M3").unwrap();
    assert_eq!(
        n3.routing,
        [
            RouteElem::Wire {
                layer: LayerId(0),
                from: pt(2500, 1000),
                to: pt(2700, 1000)
            },
            RouteElem::Via {
                via: m1m2,
                at: pt(2700, 1000)
            },
            RouteElem::Wire {
                layer: LayerId(1),
                from: pt(2700, 1000),
                to: pt(2700, 3500)
            },
            RouteElem::Via {
                via: m2m3,
                at: pt(2700, 3500)
            },
            RouteElem::Wire {
                layer: LayerId(2),
                from: pt(2700, 3500),
                to: pt(5000, 3500)
            },
        ]
    );
}

#[test]
fn def_special_wiring_keeps_widths_shapes_and_vias() {
    let db = tiny_design();

    let vdd = &db.special_nets[0];
    assert_eq!(vdd.wires.len(), 2);
    assert_eq!(vdd.wires[0].rect(), rect(0, 1900, 6000, 2100));
    assert_eq!(vdd.wires[1].shape.as_deref(), Some("STRIPE"));
    assert_eq!(
        vdd.vias,
        [(db.library.via_id("M1M2_2CUT").unwrap(), pt(4800, 4000))]
    );
}

#[test]
fn def_written_and_read_back_is_identical() {
    let db = tiny_design();

    let mut text = Vec::new();
    writer::def::write(&db, &mut text).unwrap();
    let reread = def::parse_str(
        &String::from_utf8(text).unwrap(),
        Path::new("written.def"),
        tiny_library(),
    )
    .unwrap();

    assert_eq!(reread, db);
}

#[test]
fn def_with_an_unknown_macro_is_rejected() {
    let src =
        "UNITS DISTANCE MICRONS 1000 ;\nCOMPONENTS 1 ;\n- x NOPE ;\nEND COMPONENTS\nEND DESIGN\n";

    let err = def::parse_str(src, Path::new("bad.def"), tiny_library()).unwrap_err();

    assert!(err.to_string().contains("bad.def:3"), "{err}");
    assert!(err.to_string().contains("NOPE"), "{err}");
}

#[test]
fn def_with_an_unknown_macro_pin_is_rejected() {
    let src = "UNITS DISTANCE MICRONS 1000 ;\nCOMPONENTS 1 ;\n- x INV ;\nEND COMPONENTS\nNETS 1 ;\n- n ( x Q ) ;\nEND NETS\nEND DESIGN\n";

    let err = def::parse_str(src, Path::new("bad.def"), tiny_library()).unwrap_err();

    assert!(err.to_string().contains("'Q'"), "{err}");
}

#[test]
fn def_at_a_different_resolution_than_the_library_is_rejected() {
    let src = "UNITS DISTANCE MICRONS 2000 ;\nEND DESIGN\n";

    assert!(def::parse_str(src, Path::new("bad.def"), tiny_library()).is_err());
}

#[test]
fn lef_values_off_the_dbu_grid_are_rejected() {
    let units = DbuPerMicron::new(100).unwrap();

    let err = lef::parse_files(&[fixture("tiny.lef")], units).unwrap_err();

    assert!(err.to_string().contains("tiny.lef"), "{err}");
}

fn tiny_bookshelf() -> NetlistDB {
    bookshelf::parse(&fixture("bookshelf/tiny.aux"), 6).unwrap()
}

#[test]
fn bookshelf_rows_keep_their_gaps() {
    let db = tiny_bookshelf();

    let ys: Vec<Dbu> = db.rows.iter().map(|r| r.origin.y).collect();
    assert_eq!(ys, [Dbu(0), Dbu(3200)]);
    assert_eq!(db.rows[0].bbox(&db.library), rect(0, 0, 4000, 1600));
}

#[test]
fn bookshelf_die_covers_rows_and_fixed_terminals() {
    let db = tiny_bookshelf();

    assert_eq!(db.die_area, rect(-400, 0, 4000, 4800));
}

#[test]
fn bookshelf_pins_sit_at_their_offset_from_the_cell_center() {
    let db = tiny_bookshelf();

    let a0 = &db.cells[cell(&db, "a0").index()];
    let a1 = &db.cells[cell(&db, "a1").index()];
    assert_eq!(db.pin_position(a0.pins[0]), pt(350, 750));
    assert_eq!(db.pin_position(a1.pins[0]), pt(750, 3950));
}

#[test]
fn bookshelf_terminals_are_fixed_pads_and_tall_nodes_are_blocks() {
    let db = tiny_bookshelf();

    let class = |n: &str| db.cell_macro(cell(&db, n)).class;
    assert_eq!(
        db.cells[cell(&db, "p0").index()].status,
        PlacementStatus::Fixed
    );
    assert_eq!(class("p0"), MacroClass::Pad);
    assert_eq!(class("a3"), MacroClass::Block);
    assert!(db.cells[cell(&db, "a0").index()].status.is_movable());
}

#[test]
fn bookshelf_nets_are_named_by_index_unless_named() {
    let db = tiny_bookshelf();

    let names: Vec<&str> = db.nets.iter().map(|n| n.name.as_str()).collect();
    assert_eq!(names, ["n0", "clk"]);
}

#[test]
fn bookshelf_placement_written_as_pl_reads_back() {
    let mut db = tiny_bookshelf();
    let a2 = cell(&db, "a2");
    db.cells[a2.index()].location = pt(1250, 3200);
    db.cells[a2.index()].orient = Orientation::FS;
    let path = std::env::temp_dir().join(format!("pare-test-{}.pl", std::process::id()));

    writer::bookshelf::write_pl(&db, &path).unwrap();
    let mut reread = tiny_bookshelf();
    bookshelf::read_placement(&mut reread, &path).unwrap();
    std::fs::remove_file(&path).unwrap();

    assert_eq!(reread.cells, db.cells);
}
