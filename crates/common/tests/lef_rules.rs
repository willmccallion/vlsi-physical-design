//! Reading LEF design rules, against the fixture library with statements
//! added to it.

#![allow(clippy::unwrap_used)]

use pare_common::db::indices::{CutLayerId, LayerId};
use pare_common::db::library::{LayerRef, Library, SignalUse};
use pare_common::db::parser::{def, lef};
use pare_common::db::rules::{
    CellSide, ConvexCornerSpacing, CutApproach, CutSpacing, CutSpacingCondition, EdgeType,
    EndOfLineSpacing, Lef58CutSpacing, Lef58CutSpacingCondition, MinStep, MinimumCut, ParallelEdge,
    SpacingTable, TwoWidthsRow, TwoWidthsTable, WidthRangeSpacing,
};
use pare_common::geom::dbu::{Dbu, DbuArea};
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};

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

/// Parses the fixture LEF with edits applied, from a file of its own so
/// tests can run in parallel.
fn library_with(edits: &[(&str, &str)], tag: &str) -> Result<Library, String> {
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let n = NEXT.fetch_add(1, Ordering::Relaxed);
    let path =
        std::env::temp_dir().join(format!("pare-rules-{tag}-{}-{n}.lef", std::process::id()));
    std::fs::write(&path, edited("tiny.lef", edits)).unwrap();
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let lib = lef::parse_files(std::slice::from_ref(&path), units).map_err(|e| e.to_string());
    std::fs::remove_file(&path).unwrap();
    lib
}

const M2: LayerId = LayerId(1);
const M3: LayerId = LayerId(2);

#[test]
fn two_widths_tables_are_read_with_their_run_lengths() {
    let lib = library_with(
        &[(
            "  SPACING 0.2 ;\n  SPACING 0.3 RANGE 1.0 10.0 ;",
            "  SPACINGTABLE TWOWIDTHS\n    WIDTH 0.0 0.2 0.25\n    WIDTH 0.4 PRL 0.5 0.25 0.3 ;",
        )],
        "two-widths",
    )
    .unwrap();

    let m3 = lib.layer(M3);
    assert_eq!(m3.min_spacing, Dbu(200));
    assert_eq!(
        m3.spacing_table,
        Some(SpacingTable::TwoWidths(TwoWidthsTable {
            rows: vec![
                TwoWidthsRow {
                    width: Dbu(0),
                    run_length: None,
                    spacings: vec![Dbu(200), Dbu(250)],
                },
                TwoWidthsRow {
                    width: Dbu(400),
                    run_length: Some(Dbu(500)),
                    spacings: vec![Dbu(250), Dbu(300)],
                },
            ],
        }))
    );
}

#[test]
fn width_range_spacing_is_kept_apart_from_the_minimum_spacing() {
    let lib = library_with(&[], "range").unwrap();

    let m3 = lib.layer(M3);
    assert_eq!(m3.min_spacing, Dbu(200));
    assert_eq!(
        m3.rules.width_range_spacing,
        [WidthRangeSpacing {
            spacing: Dbu(300),
            min_width: Dbu(1000),
            max_width: Dbu(10000),
        }]
    );
}

#[test]
fn metal_rules_are_read_from_statements_and_lef57_lef58_properties() {
    let lib = library_with(
        &[(
            "    WIDTH 0.3 0.11 0.2 ;\nEND M2",
            "    WIDTH 0.3 0.11 0.2 ;\n  MINWIDTH 0.1 ;\n  MAXWIDTH 5.0 ;\n  AREA 0.05 ;\n  \
             SPACING 0.12 ENDOFLINE 0.11 WITHIN 0.03 ;\n  \
             PROPERTY LEF57_SPACING \"SPACING 0.13 ENDOFLINE 0.11 WITHIN 0.03 PARALLELEDGE 0.1 \
             WITHIN 0.05 TWOEDGES ;\" ;\n  \
             PROPERTY LEF57_MINSTEP \"MINSTEP 0.05 MAXEDGES 1 ;\" ;\n  \
             PROPERTY LEF58_CORNERSPACING \"CORNERSPACING CONVEXCORNER EXCEPTEOL 0.11\n    \
             WIDTH 0.0 SPACING 0.1\n    WIDTH 0.2 SPACING 0.2 ;\" ;\n  \
             MINIMUMCUT 2 WIDTH 0.4 FROMABOVE LENGTH 0.4 WITHIN 0.8 ;\nEND M2",
        )],
        "metal",
    )
    .unwrap();

    let rules = &lib.layer(M2).rules;
    assert_eq!(rules.min_width, Some(Dbu(100)));
    assert_eq!(rules.max_width, Some(Dbu(5000)));
    assert_eq!(rules.min_area, Some(DbuArea(50_000)));
    assert_eq!(
        rules.end_of_line,
        [
            EndOfLineSpacing {
                spacing: Dbu(120),
                eol_width: Dbu(110),
                within: Dbu(30),
                parallel_edge: None,
            },
            EndOfLineSpacing {
                spacing: Dbu(130),
                eol_width: Dbu(110),
                within: Dbu(30),
                parallel_edge: Some(ParallelEdge {
                    spacing: Dbu(100),
                    within: Dbu(50),
                    two_edges: true,
                }),
            },
        ]
    );
    assert_eq!(
        rules.min_steps,
        [MinStep {
            length: Dbu(50),
            max_edges: Some(1),
        }]
    );
    assert_eq!(
        rules.corner_spacing,
        Some(ConvexCornerSpacing {
            except_eol: Some(Dbu(110)),
            spacing_by_width: vec![(Dbu(0), Dbu(100)), (Dbu(200), Dbu(200))],
        })
    );
    assert_eq!(
        rules.minimum_cuts,
        [MinimumCut {
            cuts: 2,
            width: Dbu(400),
            within: None,
            from: Some(CutApproach::Above),
            length: Some((Dbu(400), Dbu(800))),
        }]
    );
}

#[test]
fn cut_spacing_counts_a_center_rule_as_its_value_less_the_cut_width() {
    let lib = library_with(
        &[(
            "LAYER V2\n  TYPE CUT ;\n  WIDTH 0.1 ;",
            "LAYER V2\n  TYPE CUT ;\n  WIDTH 0.1 ;\n  SPACING 0.25 CENTERTOCENTER ;\n  \
             SPACING 0.1 SAMENET ;\n  SPACING 0.2 ADJACENTCUTS 3 WITHIN 0.3 ;\n  \
             ENCLOSURE BELOW 0.01 0.03 ;",
        )],
        "cut",
    )
    .unwrap();

    let v2 = &lib.cut_layers[1];
    assert_eq!(v2.spacing, Some(Dbu(150)));
    assert_eq!(v2.rules.spacings.len(), 3);
    assert_eq!(
        v2.rules.spacings[2],
        CutSpacing {
            spacing: Dbu(200),
            center_to_center: false,
            same_net: false,
            condition: CutSpacingCondition::AdjacentCuts {
                cuts: 3,
                within: Dbu(300),
            },
        }
    );
    assert_eq!(v2.rules.enclosures[0].side, Some(CutApproach::Below));
}

#[test]
fn cut_spacing_properties_are_kept_apart_from_the_spacing_statements() {
    let lib = library_with(
        &[(
            "LAYER V2\n  TYPE CUT ;\n  WIDTH 0.1 ;",
            "LAYER V2\n  TYPE CUT ;\n  WIDTH 0.1 ;\n  SPACING 0.1 ;\n  \
             PROPERTY LEF57_SPACING \"SPACING 0.13 CENTERTOCENTER PARALLELOVERLAP ;\" ;",
        )],
        "cut-property",
    )
    .unwrap();

    let v2 = &lib.cut_layers[1];
    assert_eq!(v2.rules.spacings.len(), 1);
    assert_eq!(
        v2.rules.lef58_spacings,
        vec![Lef58CutSpacing {
            spacing: Dbu(130),
            center_to_center: true,
            same_net: false,
            condition: Lef58CutSpacingCondition::ParallelOverlap,
        }]
    );
}

#[test]
fn cut_spacing_property_forms_the_model_cannot_hold_are_errors() {
    let err = library_with(
        &[(
            "LAYER V2\n  TYPE CUT ;\n  WIDTH 0.1 ;",
            "LAYER V2\n  TYPE CUT ;\n  WIDTH 0.1 ;\n  \
             PROPERTY LEF58_SPACING \"SPACING 0.13 ADJACENTCUTS 3 WITHIN 0.2 ;\" ;",
        )],
        "cut-property-unsupported",
    )
    .unwrap_err();

    assert!(
        err.contains("unsupported LEF58 cut SPACING option 'ADJACENTCUTS'"),
        "{err}"
    );
}

#[test]
fn via_generation_rules_keep_their_enclosures_and_cut_array() {
    let lib = library_with(&[], "via-rule").unwrap();

    let rule = &lib.via_rules[0];
    assert_eq!(rule.name, "M1M2GEN");
    assert_eq!(rule.cut, CutLayerId(0));
    assert_eq!(
        (rule.bottom.layer, rule.top.layer),
        (LayerId(0), LayerId(1))
    );
    assert_eq!(rule.bottom.enclosure, (Dbu(0), Dbu(50)));
    assert_eq!(rule.cut_rect, rect(-50, -50, 50, 50));
    assert_eq!(rule.cut_pitch, pt(200, 200));
}

#[test]
fn library_rules_cover_grid_stacks_same_net_spacing_and_cell_edges() {
    let lib = library_with(
        &[
            (
                "MANUFACTURINGGRID 0.005 ;",
                "MANUFACTURINGGRID 0.005 ;\nPROPERTYDEFINITIONS\n  MACRO LEF58_EDGETYPE STRING ;\n  \
                 LIBRARY LEF58_CELLEDGESPACINGTABLE STRING \"CELLEDGESPACINGTABLE\n    \
                 EDGETYPE 1 2 0.4\n    EDGETYPE 2 2 0.0 ;\" ;\nEND PROPERTYDEFINITIONS",
            ),
            (
                "MACRO INV\n  CLASS CORE ;",
                "MAXVIASTACK 2 RANGE M1 M3 ;\nSPACING\n  SAMENET V1 V2 0.0 STACK ;\nEND SPACING\n\n\
                 MACRO INV\n  PROPERTY LEF58_EDGETYPE \"EDGETYPE LEFT 1 ; EDGETYPE RIGHT 2 ;\" ;\n  \
                 CLASS CORE ;",
            ),
        ],
        "library",
    )
    .unwrap();

    assert_eq!(lib.rules.manufacturing_grid, Some(Dbu(5)));
    let stack = lib.rules.max_via_stack.unwrap();
    assert_eq!((stack.vias, stack.range), (2, Some((LayerId(0), M3))));
    let same_net = lib.rules.same_net_spacing[0];
    assert_eq!(
        (same_net.first, same_net.second, same_net.stack),
        (
            LayerRef::Cut(CutLayerId(0)),
            LayerRef::Cut(CutLayerId(1)),
            true
        )
    );
    assert_eq!(lib.rules.cell_edge_spacing.len(), 2);
    assert_eq!(lib.rules.cell_edge_spacing[0].spacing, Dbu(400));
    let inv = lib.cell_macro(lib.macro_id("INV").unwrap());
    assert_eq!(
        inv.edges
            .iter()
            .map(|e| (e.side, e.edge_type.clone()))
            .collect::<Vec<_>>(),
        [
            (CellSide::Left, EdgeType("1".into())),
            (CellSide::Right, EdgeType("2".into()))
        ]
    );
}

#[test]
fn lowercase_pin_uses_are_read() {
    let lib = library_with(&[("    USE POWER ;", "    USE power ;")], "lowercase").unwrap();

    let inv = lib.cell_macro(lib.macro_id("INV").unwrap());
    let vdd = inv.pin(inv.pin_id("VDD").unwrap());
    assert_eq!(vdd.signal_use, SignalUse::Power);
}

#[test]
fn rule_options_the_model_cannot_hold_are_errors() {
    let err = library_with(
        &[("  SPACING 0.2 ;", "  SPACING 0.2 NOTCHLENGTH 0.3 ;")],
        "unsupported",
    )
    .unwrap_err();

    assert!(
        err.contains("unsupported metal SPACING option 'NOTCHLENGTH'"),
        "{err}"
    );
}
