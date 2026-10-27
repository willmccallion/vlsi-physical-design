//! Reading the DEF sections that constrain placement and routing, and
//! writing them back, against the fixture design with entries added to it.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, RegionKind, RouteElem};
use pare_common::db::indices::{CutLayerId, RegionId};
use pare_common::db::library::{LayerRef, Library};
use pare_common::db::parser::{def, lef};
use pare_common::db::writer;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

/// The fixture text with each `(old, new)` replacement applied once.
fn edited(name: &str, edits: &[(&str, &str)]) -> String {
    let mut text = std::fs::read_to_string(fixture(name)).unwrap();
    for (old, new) in edits {
        assert!(text.contains(old), "fixture {name} has no '{old}'");
        text = text.replacen(old, new, 1);
    }
    text
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

/// Parses the fixture DEF with edits applied, over the fixture LEF.
fn design_with(edits: &[(&str, &str)]) -> Result<NetlistDB, String> {
    def::parse_str(
        &edited("tiny.def", edits),
        Path::new("edited.def"),
        tiny_library(),
    )
    .map_err(|e| e.to_string())
}

#[test]
fn special_net_rects_and_wire_extensions_are_kept() {
    let db = design_with(&[(
        "  NEW M3 400 + SHAPE STRIPE ( 5000 0 ) ( * 6000 )\n  NEW M1 0 ( 4800 4000 ) M1M2_2CUT ;",
        "  NEW M3 400 + SHAPE STRIPE ( 5000 0 50 ) ( * 6000 70 )\n  NEW M1 0 ( 4800 4000 ) \
         M1M2_2CUT\n  + RECT M2 ( 0 0 ) ( 300 100 ) ;",
    )])
    .unwrap();

    let vdd = &db.special_nets[0];
    assert_eq!(vdd.wires[1].rect(), rect(4800, -50, 5200, 6070));
    assert_eq!(vdd.rects.len(), 1);
    assert_eq!(vdd.rects[0].rect, rect(0, 0, 300, 100));
    assert_eq!(vdd.rects[0].shape, None);
}

#[test]
fn regular_wire_end_extensions_stay_on_their_wire_ends() {
    let db = design_with(&[(
        "  + ROUTED M1 ( 2500 1000 ) ( 2700 * ) M1M2\n  NEW M2 ( 2700 1000 ) ( 2700 3500 ) M2M3\n  \
         NEW M3 ( 2700 3500 ) ( 5000 3500 ) ;",
        "  + ROUTED M1 ( 2500 1000 30 ) ( 2700 * 0 ) M1M2 ( 2700 3500 ) M2M3 ( 5000 * 0 ) ;",
    )])
    .unwrap();

    let n3 = &db.nets[db.net_id("n3").unwrap().index()];
    let extensions: Vec<_> = n3
        .routing
        .iter()
        .filter_map(|e| match *e {
            RouteElem::Wire { extensions, .. } => Some(extensions),
            RouteElem::Via { .. } | RouteElem::Patch { .. } => None,
        })
        .collect();
    assert_eq!(
        extensions,
        [
            (Some(Dbu(30)), Some(Dbu(0))),
            (None, None),
            (None, Some(Dbu(0))),
        ]
    );
    let mut text = Vec::new();
    writer::def::write(&db, &mut text).unwrap();
    let reread = def::parse_str(
        &String::from_utf8(text).unwrap(),
        Path::new("written.def"),
        tiny_library(),
    )
    .unwrap();
    assert_eq!(reread.nets, db.nets);
}

#[test]
fn a_special_net_rect_takes_the_shape_option_before_it() {
    let db = design_with(&[(
        "  NEW M1 0 ( 4800 4000 ) M1M2_2CUT ;",
        "  NEW M1 0 ( 4800 4000 ) M1M2_2CUT\n  + RECT M2 ( 0 0 ) ( 300 100 )\n  \
         + SHAPE STRIPE + RECT M2 ( 0 200 ) ( 300 300 ) + RECT M2 ( 0 400 ) ( 300 500 ) ;",
    )])
    .unwrap();

    let shapes: Vec<Option<&str>> = db.special_nets[0]
        .rects
        .iter()
        .map(|r| r.shape.as_deref())
        .collect();
    assert_eq!(shapes, [None, Some("STRIPE"), Some("STRIPE")]);

    let mut text = Vec::new();
    pare_common::db::writer::def::write(&db, &mut text).unwrap();
    let written = String::from_utf8(text).unwrap();
    assert!(
        written.contains("+ RECT M2 ( 0 0 ) ( 300 100 )\n  + SHAPE STRIPE\n  + RECT M2 ( 0 200 )"),
        "{written}"
    );
    let reread = def::parse_str(&written, Path::new("written.def"), tiny_library()).unwrap();
    assert_eq!(reread.special_nets, db.special_nets);
}

#[test]
fn via_patterns_drop_the_cuts_they_mark_absent() {
    let db = design_with(&[(
        "+ ENCLOSURE 0 50 0 50 + ROWCOL 1 2 ;",
        "+ ENCLOSURE 0 50 0 50 + ROWCOL 2 3 + PATTERN 1_A_1_R2F ;",
    )])
    .unwrap();

    let via = db.library.via(db.library.via_id("M1M2_2CUT").unwrap());
    let cuts: Vec<Rect<Dbu>> = via
        .shapes
        .iter()
        .filter(|s| matches!(s.layer, LayerRef::Cut(_)))
        .map(|s| s.rect)
        .collect();
    assert_eq!(
        cuts,
        [
            rect(-250, -150, -150, -50),
            rect(150, -150, 250, -50),
            rect(-250, 50, -150, 150),
            rect(-50, 50, 50, 150),
            rect(150, 50, 250, 150),
        ]
    );
}

#[test]
fn via_layers_may_be_listed_in_any_order() {
    let db = design_with(&[("+ LAYERS M1 V1 M2", "+ LAYERS M1 M2 V1")]);

    assert!(db.is_ok(), "{db:?}");
}

#[test]
fn groups_assign_matching_components_to_their_region() {
    let db = design_with(&[
        (
            "COMPONENTS 8 ;",
            "REGIONS 2 ;\n- left ( 0 0 ) ( 1000 2000 ) ( 0 2000 ) ( 500 4000 ) + TYPE FENCE ;\n\
             - right ( 3000 0 ) ( 6000 6000 ) ;\nEND REGIONS\n\nCOMPONENTS 8 ;",
        ),
        (
            "- ram0 RAM + FIXED ( 4000 0 ) N ;",
            "- ram0 RAM + FIXED ( 4000 0 ) N + REGION right ;",
        ),
        (
            "END NETS",
            "END NETS\n\nGROUPS 1 ;\n- g u* + REGION left ;\nEND GROUPS",
        ),
    ])
    .unwrap();

    assert_eq!(db.regions[0].rects.len(), 2);
    assert_eq!(db.regions[0].kind, RegionKind::Fence);
    assert_eq!(db.regions[1].kind, RegionKind::Inclusive);
    let region = |name: &str| db.cells[db.cell_id(name).unwrap().index()].region;
    assert_eq!(region("u2"), Some(RegionId(0)));
    assert_eq!(region("u7"), Some(RegionId(0)));
    assert_eq!(region("ram0"), Some(RegionId(1)));
}

#[test]
fn group_members_are_ranked_as_opendb_lists_them() {
    let db = design_with(&[
        (
            "COMPONENTS 8 ;",
            "REGIONS 2 ;\n- left ( 0 0 ) ( 6000 4000 ) ;\n- right ( 3000 0 ) ( 6000 6000 ) ;\n\
             END REGIONS\n\nCOMPONENTS 8 ;",
        ),
        (
            "END NETS",
            "END NETS\n\nGROUPS 2 ;\n- g2 ram0 + REGION right ;\n- g u7 u1 u* + REGION left ;\n\
             END GROUPS",
        ),
    ])
    .unwrap();

    let rank = |name: &str| db.cells[db.cell_id(name).unwrap().index()].group_rank;
    let ranks: Vec<u32> = ["u7", "u6", "u5", "u4", "u3", "u2", "u1"]
        .iter()
        .map(|n| rank(n))
        .collect();
    assert_eq!(ranks, [0, 1, 2, 3, 4, 5, 6]);
    assert_eq!(db.regions[0].group_order, Some(1));
    assert_eq!(db.regions[1].group_order, Some(0));
}

#[test]
fn groups_may_come_before_the_regions_and_components_they_name() {
    let db = design_with(&[(
        "COMPONENTS 8 ;",
        "GROUPS 1 ;\n- g u2 + REGION late ;\nEND GROUPS\n\nREGIONS 1 ;\n\
         - late ( 0 0 ) ( 10 10 ) ;\nEND REGIONS\n\nCOMPONENTS 8 ;",
    )])
    .unwrap();

    assert_eq!(
        db.cells[db.cell_id("u2").unwrap().index()].region,
        Some(RegionId(0))
    );
}

#[test]
fn a_group_without_a_region_assigns_nothing() {
    let db = design_with(&[
        (
            "COMPONENTS 8 ;",
            "REGIONS 1 ;\n- a ( 0 0 ) ( 10 10 ) ;\nEND REGIONS\nCOMPONENTS 8 ;",
        ),
        (
            "END NETS",
            "END NETS\nGROUPS 2 ;\n- g1 u1 + REGION a ;\n- g2 u1 ;\nEND GROUPS",
        ),
    ])
    .unwrap();

    assert_eq!(
        db.cells[db.cell_id("u1").unwrap().index()].region,
        Some(RegionId(0))
    );
}

#[test]
fn a_component_in_two_regions_is_rejected() {
    let err = design_with(&[
        (
            "COMPONENTS 8 ;",
            "REGIONS 2 ;\n- a ( 0 0 ) ( 10 10 ) ;\n- b ( 0 0 ) ( 20 20 ) ;\nEND REGIONS\n\
             COMPONENTS 8 ;",
        ),
        (
            "END NETS",
            "END NETS\nGROUPS 2 ;\n- g1 u1 + REGION a ;\n- g2 u* + REGION b ;\nEND GROUPS",
        ),
    ])
    .unwrap_err();

    assert!(err.contains("assigned to two regions"), "{err}");
}

#[test]
fn non_default_rules_attach_to_their_nets() {
    let db = design_with(&[
        (
            "COMPONENTS 8 ;",
            "NONDEFAULTRULES 1 ;\n- wide + HARDSPACING\n  + LAYER M2 WIDTH 200 SPACING 150\n  \
             + LAYER M3 WIDTH 400 WIREEXT 100\n  + VIA M1M2 + VIARULE M1M2GEN + MINCUTS V1 2 ;\n\
             END NONDEFAULTRULES\n\nCOMPONENTS 8 ;",
        ),
        ("+ WEIGHT 2 ;", "+ WEIGHT 2 + NONDEFAULTRULE wide ;"),
    ])
    .unwrap();

    let rule = &db.non_default_rules[0];
    assert!(rule.hard_spacing);
    assert_eq!(rule.layers[0].spacing, Some(Dbu(150)));
    assert_eq!(rule.layers[1].wire_extension, Some(Dbu(100)));
    assert_eq!(rule.min_cuts, [(CutLayerId(0), 2)]);
    let n2 = &db.nets[db.net_id("n2").unwrap().index()];
    assert!(n2.non_default_rule.is_some());
}

#[test]
fn constraint_sections_are_written_and_read_back_unchanged() {
    let db = design_with(&[
        (
            "COMPONENTS 8 ;",
            "NONDEFAULTRULES 1 ;\n- wide + LAYER M2 WIDTH 200 SPACING 150 + VIA M1M2 ;\n\
             END NONDEFAULTRULES\nREGIONS 1 ;\n- r ( 0 0 ) ( 1000 2000 ) + TYPE GUIDE ;\n\
             END REGIONS\nCOMPONENTS 8 ;",
        ),
        ("- u1 INV + PLACED ( 200 0 ) N ;", "- u1 INV + PLACED ( 200 0 ) N + REGION r ;"),
        ("+ WEIGHT 2 ;", "+ WEIGHT 2 + NONDEFAULTRULE wide ;"),
        (
            "  NEW M3 400 + SHAPE STRIPE ( 5000 0 ) ( * 6000 )\n  NEW M1 0 ( 4800 4000 ) M1M2_2CUT ;",
            "  NEW M3 400 + SHAPE STRIPE ( 5000 0 50 ) ( * 6000 )\n  NEW M1 0 ( 4800 4000 ) \
             M1M2_2CUT\n  + RECT M2 ( 0 0 ) ( 300 100 ) ;",
        ),
    ])
    .unwrap();

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
