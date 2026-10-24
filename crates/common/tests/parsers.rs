//! Parser and writer tests against small fixture designs.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{
    GCellGridDef, NetlistDB, PlacementStatus, RouteElem, TrackAxis, TrackDef,
};
use pare_common::db::indices::{CellId, CutLayerId, LayerId, PinId};
use pare_common::db::library::{Direction, LayerRef, Library, MacroClass, Shape};
use pare_common::db::parser::{bookshelf, def, lef};
use pare_common::db::rules::SpacingTable;
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
fn lef_parallel_run_spacing_tables_are_read_whole() {
    let lib = tiny_library();

    let Some(SpacingTable::ParallelRunLength(table)) = lib.routing_layers[1].spacing_table.clone()
    else {
        panic!("M2 has a parallel run length table");
    };

    assert_eq!(table.run_lengths, [Dbu(0), Dbu(500)]);
    assert_eq!(
        table.rows,
        [
            (Dbu(0), vec![Dbu(110), Dbu(110)]),
            (Dbu(300), vec![Dbu(110), Dbu(200)])
        ]
    );
}

#[test]
fn spacing_grows_only_once_both_width_and_run_reach_their_thresholds() {
    let table = tiny_library().routing_layers[1]
        .spacing_table
        .clone()
        .unwrap();

    assert_eq!(table.spacing(Dbu(100), Dbu(100), Dbu(900)), Dbu(110));
    assert_eq!(table.spacing(Dbu(300), Dbu(100), Dbu(500)), Dbu(110));
    assert_eq!(table.spacing(Dbu(100), Dbu(300), Dbu(501)), Dbu(200));
    assert_eq!(table.spacing(Dbu(400), Dbu(100), Dbu(-50)), Dbu(110));
    assert_eq!(table.max_spacing(), Dbu(200));
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
fn def_reads_the_gcell_grid() {
    let db = tiny_design();

    assert_eq!(
        db.gcell_grid,
        [GCellGridDef {
            axis: TrackAxis::X,
            start: Dbu(0),
            count: 4,
            step: Dbu(2000),
        }]
    );
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
fn a_pin_position_at_another_location_moves_with_the_cell() {
    let db = tiny_design();
    let pin = cell_pin(&db, "u2", "A");
    let cell = db.cells.iter().position(|c| c.name == "u2").unwrap();
    let moved = db.cells[cell].location + pt(400, -2000);

    assert_eq!(
        db.pin_position_at(pin, moved),
        db.pin_position(pin) + pt(400, -2000)
    );
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
                to: pt(2700, 1000),
                extensions: (None, None),
            },
            RouteElem::Via {
                via: m1m2,
                at: pt(2700, 1000)
            },
            RouteElem::Wire {
                layer: LayerId(1),
                from: pt(2700, 1000),
                to: pt(2700, 3500),
                extensions: (None, None),
            },
            RouteElem::Via {
                via: m2m3,
                at: pt(2700, 3500)
            },
            RouteElem::Wire {
                layer: LayerId(2),
                from: pt(2700, 3500),
                to: pt(5000, 3500),
                extensions: (None, None),
            },
        ]
    );
}

#[test]
fn def_regular_wiring_patches_are_rectangles_offset_from_the_current_point() {
    let src = "UNITS DISTANCE MICRONS 1000 ;\nCOMPONENTS 2 ;\n- x INV + PLACED ( 0 0 ) N ;\n\
               - y INV + PLACED ( 5000 0 ) N ;\nEND COMPONENTS\nNETS 1 ;\n- n ( x Y ) ( y A )\n\
               + ROUTED M1 ( 2500 1000 ) ( 2700 * ) M1M2\n\
               NEW M2 ( 2700 1000 ) RECT ( -50 -100 50 0 ) ;\nEND NETS\nEND DESIGN\n";

    let db = def::parse_str(src, Path::new("patch.def"), tiny_library()).unwrap();

    let n = &db.nets[db.net_id("n").unwrap().index()];
    assert_eq!(
        n.routing[2],
        RouteElem::Patch {
            layer: LayerId(1),
            rect: rect(2650, 900, 2750, 1000),
        }
    );
    assert_eq!(n.routing.len(), 3);
}

#[test]
fn def_regular_wiring_patches_survive_writing_and_reading_back() {
    let mut db = tiny_design();
    let n3 = db.net_id("n3").unwrap();
    db.nets[n3.index()].routing.push(RouteElem::Patch {
        layer: LayerId(2),
        rect: rect(4950, 3450, 5100, 3550),
    });

    let mut text = Vec::new();
    writer::def::write(&db, &mut text).unwrap();
    let written = String::from_utf8(text).unwrap();
    let reread = def::parse_str(&written, Path::new("written.def"), tiny_library()).unwrap();

    assert!(
        written.contains("M3 ( 4950 3450 ) RECT ( 0 0 150 100 )"),
        "{written}"
    );
    assert_eq!(reread, db);
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
fn lef_top_of_stack_vias_are_read_but_never_the_default() {
    let text = std::fs::read_to_string(fixture("tiny.lef")).unwrap().replacen(
        "VIA M1M2 DEFAULT",
        "VIA M1M2_TOP DEFAULT TOPOFSTACKONLY\n  LAYER V1 ;\n    RECT -0.05 -0.05 0.05 0.05 ;\n  \
         LAYER M1 ;\n    RECT -0.05 -0.1 0.05 0.1 ;\n  LAYER M2 ;\n    RECT -0.05 -0.1 0.05 0.3 ;\n\
         END M1M2_TOP\n\nVIA M1M2 DEFAULT",
        1,
    );
    let path = std::env::temp_dir().join(format!("pare-top-of-stack-{}.lef", std::process::id()));
    std::fs::write(&path, text).unwrap();
    let units = def::read_units(&fixture("tiny.def")).unwrap();

    let lib = lef::parse_files(std::slice::from_ref(&path), units);
    std::fs::remove_file(&path).unwrap();

    let lib = lib.unwrap();
    assert!(lib.via_id("M1M2_TOP").is_some());
    assert_eq!(
        lib.via(lib.default_via(CutLayerId(0)).unwrap()).name,
        "M1M2"
    );
}

/// Parses the fixture DEF with one more via in its `VIAS` section.
fn tiny_with_def_via(via: &str, tag: &str) -> Result<NetlistDB, String> {
    let text = std::fs::read_to_string(fixture("tiny.def"))
        .unwrap()
        .replacen("VIAS 1 ;", &format!("VIAS 2 ;\n{via}"), 1);
    let path = std::env::temp_dir().join(format!("pare-{tag}-{}.def", std::process::id()));
    std::fs::write(&path, text).unwrap();
    let parsed = def::parse(&path, tiny_library()).map_err(|e| e.to_string());
    std::fs::remove_file(&path).unwrap();
    parsed
}

#[test]
fn a_def_via_identical_to_a_lef_via_of_its_name_is_accepted() {
    let via = "- M1M2 + RECT M1 ( -50 -100 ) ( 50 100 ) + RECT V1 ( -50 -50 ) ( 50 50 ) \
               + RECT M2 ( -50 -100 ) ( 50 100 ) ;";

    let db = tiny_with_def_via(via, "same-via");

    assert!(db.is_ok(), "{db:?}");
}

#[test]
fn a_def_via_differing_from_a_lef_via_of_its_name_leaves_the_lef_via_in_use() {
    let via = "- M1M2 + RECT M1 ( -50 -100 ) ( 50 100 ) + RECT V1 ( -50 -50 ) ( 50 50 ) \
               + RECT M2 ( -50 -300 ) ( 50 300 ) ;";

    let db = tiny_with_def_via(via, "other-via").unwrap();

    assert_eq!(
        db.library.via(db.library.via_id("M1M2").unwrap()),
        tiny_library().via(tiny_library().via_id("M1M2").unwrap())
    );
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

#[test]
fn obstructions_use_minimum_spacing_unless_the_lef_turns_it_off() {
    let text = std::fs::read_to_string(fixture("tiny.lef"))
        .unwrap()
        .replacen("UNITS", "USEMINSPACING OBS OFF ;\n\nUNITS", 1);
    let path =
        std::env::temp_dir().join(format!("pare-min-spacing-obs-{}.lef", std::process::id()));
    std::fs::write(&path, text).unwrap();
    let units = def::read_units(&fixture("tiny.def")).unwrap();

    let off = lef::parse_files(std::slice::from_ref(&path), units);
    std::fs::remove_file(&path).unwrap();

    assert!(tiny_library().rules.min_spacing_for_obstructions);
    assert!(!off.unwrap().rules.min_spacing_for_obstructions);
}

#[test]
fn tracks_whose_wire_would_leave_the_die_are_not_routing_tracks() {
    let mut db = tiny_design();
    db.die_area = rect(0, 0, 3000, 6000);

    let tracks = db.layer_tracks(LayerId(1));

    assert_eq!(tracks.first(), Some(&Dbu(100)));
    assert_eq!(tracks.last(), Some(&Dbu(2900)));
    assert_eq!(tracks.len(), 15);
}

/// The axis of the tracks layer 1 of the tiny design prefers.
fn preferred_axis(db: &NetlistDB) -> TrackAxis {
    match db.library.layer(LayerId(1)).direction {
        Direction::Horizontal => TrackAxis::Y,
        Direction::Vertical => TrackAxis::X,
    }
}

fn pattern(axis: TrackAxis, start: i64, count: u32, step: i64) -> TrackDef {
    TrackDef {
        axis,
        start: Dbu(start),
        count,
        step: Dbu(step),
        layers: vec![LayerId(1)],
    }
}

#[test]
fn one_track_pattern_spaces_its_tracks_by_its_step() {
    let mut db = tiny_design();
    db.tracks = vec![pattern(preferred_axis(&db), 100, 10, 200)];

    assert_eq!(db.average_track_spacing(LayerId(1)), Dbu(200));
}

#[test]
fn several_track_patterns_space_their_tracks_by_span_over_count_rounded_up() {
    let mut db = tiny_design();
    let axis = preferred_axis(&db);
    db.tracks = vec![pattern(axis, 0, 5, 200), pattern(axis, 100, 5, 200)];

    assert_eq!(db.average_track_spacing(LayerId(1)), Dbu(90));
}
