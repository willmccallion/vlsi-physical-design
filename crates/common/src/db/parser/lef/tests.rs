use super::*;
use crate::db::indices::MacroPinId;
use crate::db::library::{CoreKind, MacroClass, SiteClass};
use crate::db::rules::ClearanceMeasure;
use crate::testing::capture_logs;
use std::path::Path;
use std::sync::atomic::{AtomicUsize, Ordering};

/// The tiny LEF with each `(old, new)` replaced once, parsed, and the lines
/// parsing it logged.
fn library_with(edits: &[(&str, &str)]) -> (Result<Library, ParseError>, Vec<String>) {
    static NEXT: AtomicUsize = AtomicUsize::new(0);
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/tiny.lef");
    let mut text = std::fs::read_to_string(fixture).unwrap();
    for (old, new) in edits {
        assert!(text.contains(old), "the tiny LEF has no '{old}'");
        text = text.replacen(old, new, 1);
    }
    let n = NEXT.fetch_add(1, Ordering::Relaxed);
    let path = std::env::temp_dir().join(format!("pare-lef-{}-{n}.lef", std::process::id()));
    std::fs::write(&path, text).unwrap();
    let mut parsed = None;
    let logs = capture_logs(|| {
        parsed = Some(parse_files(
            std::slice::from_ref(&path),
            DbuPerMicron::new(1000).unwrap(),
        ));
    });
    std::fs::remove_file(&path).unwrap();
    (parsed.unwrap(), logs)
}

fn library(edits: &[(&str, &str)]) -> Library {
    library_with(edits).0.unwrap()
}

/// The lines parsing the edited LEF logged, which must parse.
fn warnings(edits: &[(&str, &str)]) -> Vec<String> {
    let (parsed, logs) = library_with(edits);
    let _ = parsed.unwrap();
    logs
}

/// The error the edited LEF gives.
fn error(edits: &[(&str, &str)]) -> String {
    library_with(edits).0.unwrap_err().to_string()
}

/// Asserts `logs` is exactly one warning naming `what`.
fn assert_one_warning(logs: &[String], what: &str) {
    assert_eq!(logs.len(), 1, "{logs:?}");
    assert!(logs[0].contains(what), "{logs:?}");
}

#[test]
fn layer_statements_on_the_list_or_about_antennas_carry_nothing() {
    assert!(is_ignored(LAYER_IGNORED, "THICKNESS"));
    assert!(is_ignored(LAYER_IGNORED, "ANTENNAAREARATIO"));
    assert!(!is_ignored(LAYER_IGNORED, "PITCH"));
}

#[test]
fn the_fixture_parses_without_warnings() {
    assert_eq!(warnings(&[]), Vec::<String>::new());
}

const GRID: &str = "MANUFACTURINGGRID 0.005 ;";

#[test]
fn a_clearance_measure_is_read() {
    let lib = library(&[(GRID, "MANUFACTURINGGRID 0.005 ;\nCLEARANCEMEASURE MAXXY ;")]);

    assert_eq!(lib.rules.clearance_measure, ClearanceMeasure::MaxXy);
}

#[test]
fn a_lef_non_default_rule_is_skipped_with_a_warning() {
    let logs = warnings(&[(
        GRID,
        "MANUFACTURINGGRID 0.005 ;\nNONDEFAULTRULE wide\n  LAYER M1\n    WIDTH 0.2 ;\n  END M1\nEND wide",
    )]);

    assert_one_warning(&logs, "LEF NONDEFAULTRULE");
}

#[test]
fn an_unknown_statement_is_skipped_with_one_warning_per_kind() {
    let logs = warnings(&[(GRID, "MANUFACTURINGGRID 0.005 ;\nFOOBAR 1 2 ;\nFOOBAR 3 ;")]);

    assert_one_warning(&logs, "ignoring library statement 'FOOBAR'");
}

const M3_WIDTH: &str = "  WIDTH 0.2 ;";

#[test]
fn a_minimum_enclosed_area_is_kept() {
    let lib = library(&[(M3_WIDTH, "  WIDTH 0.2 ;\n  MINENCLOSEDAREA 0.3 ;")]);

    let m3 = lib.routing_layer_id("M3").unwrap();
    assert_eq!(lib.layer(m3).rules.min_enclosed_areas.len(), 1);
}

#[test]
fn layer_statements_that_carry_nothing_are_skipped_silently_and_others_warn() {
    let quiet_routing = warnings(&[(M3_WIDTH, "  WIDTH 0.2 ;\n  THICKNESS 0.1 ;")]);
    let loud_routing = warnings(&[(M3_WIDTH, "  WIDTH 0.2 ;\n  FOO 1 ;")]);
    let v2 = "  WIDTH 0.1 ;\nEND V2";
    let quiet_cut = warnings(&[(v2, "  WIDTH 0.1 ;\n  THICKNESS 0.1 ;\nEND V2")]);
    let loud_cut = warnings(&[(v2, "  WIDTH 0.1 ;\n  FOO 1 ;\nEND V2")]);

    assert_eq!(quiet_routing, Vec::<String>::new());
    assert_one_warning(&loud_routing, "ignoring layer statement 'FOO'");
    assert_eq!(quiet_cut, Vec::<String>::new());
    assert_one_warning(&loud_cut, "ignoring layer statement 'FOO'");
}

#[test]
fn routing_layers_must_alternate_with_cut_layers() {
    let skipped_cut = error(&[("LAYER V2\n  TYPE CUT ;\n  WIDTH 0.1 ;\nEND V2\n", "")]);

    assert!(
        skipped_cut.contains("breaks the routing/cut alternation"),
        "{skipped_cut}"
    );
}

#[test]
fn a_library_property_other_than_an_edge_table_is_skipped_with_a_warning() {
    let logs = warnings(&[(
        GRID,
        "MANUFACTURINGGRID 0.005 ;\nPROPERTYDEFINITIONS\n  LIBRARY LEF58_FOO STRING \"x y\" ;\n\
         END PROPERTYDEFINITIONS",
    )]);

    assert_one_warning(&logs, "LEF58_FOO");
}

const INV_A_PORT: &str = "      LAYER M1 ;\n        RECT 0.05 0.4 0.15 0.6 ;";

#[test]
fn geometry_with_a_spacing_or_rule_width_of_its_own_is_refused() {
    for option in ["SPACING 0.1", "DESIGNRULEWIDTH 0.2"] {
        let refused = error(&[(
            INV_A_PORT,
            &format!("      LAYER M1 {option} ;\n        RECT 0.05 0.4 0.15 0.6 ;"),
        )]);

        assert!(refused.contains("are not supported"), "{refused}");
    }
}

#[test]
fn geometry_classes_are_skipped_and_polygons_refused() {
    let classed = warnings(&[(INV_A_PORT, &format!("      CLASS CORE ;\n{INV_A_PORT}"))]);
    let polygon = error(&[(
        INV_A_PORT,
        "      LAYER M1 ;\n        POLYGON 0 0 1 0 1 1 ;",
    )]);

    assert_eq!(classed, Vec::<String>::new());
    assert!(
        polygon.contains("POLYGON geometry is not supported"),
        "{polygon}"
    );
}

const RAM_START: &str = "MACRO RAM\n  CLASS BLOCK ;";
const RAM_OBS: &str = "  OBS\n    LAYER M1 ;\n      RECT 0 0 2.0 4.0 ;";

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::from_corners(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

#[test]
fn a_macro_origin_shifts_its_pins_obstructions_and_vias() {
    let lib = library(&[
        (RAM_START, "MACRO RAM\n  CLASS BLOCK ;\n  ORIGIN 0.5 0.5 ;"),
        (RAM_OBS, &format!("{RAM_OBS}\n    VIA 1.0 1.0 M2M3 ;")),
    ]);

    let ram = lib.cell_macro(lib.macro_id("RAM").unwrap());
    let m3 = lib.routing_layer_id("M3").unwrap();
    assert_eq!(
        ram.pins[MacroPinId::new(0)].shapes[0].rect,
        rect(1400, 4300, 1600, 4500)
    );
    let via_metal = ram
        .obstructions
        .iter()
        .find(|s| s.layer == LayerRef::Routing(m3))
        .unwrap();
    assert_eq!(via_metal.rect, rect(1400, 1400, 1600, 1600));
}

const M1M2_HEAD: &str = "VIA M1M2 DEFAULT\n  LAYER V1 ;";

#[test]
fn a_generated_via_is_read_like_any_other() {
    let logs = warnings(&[(M1M2_HEAD, "VIA M1M2 DEFAULT GENERATED\n  LAYER V1 ;")]);

    assert_eq!(logs, Vec::<String>::new());
}

#[test]
fn rule_generated_and_polygon_vias_are_refused() {
    let generated = error(&[(
        M1M2_HEAD,
        "VIA M1M2 DEFAULT\n  VIARULE M1M2GEN ;\n  LAYER V1 ;",
    )]);
    let polygon = error(&[(
        M1M2_HEAD,
        "VIA M1M2 DEFAULT\n  LAYER V1 ;\n    POLYGON 0 0 1 0 1 1 ;",
    )]);

    assert!(
        generated.contains("VIARULE-generated LEF vias are not supported"),
        "{generated}"
    );
    assert!(polygon.contains("POLYGON is not supported"), "{polygon}");
}

#[test]
fn via_resistance_is_skipped_silently_and_other_via_statements_warn() {
    let quiet = warnings(&[(
        M1M2_HEAD,
        "VIA M1M2 DEFAULT\n  RESISTANCE 1.5 ;\n  LAYER V1 ;",
    )]);
    let loud = warnings(&[(M1M2_HEAD, "VIA M1M2 DEFAULT\n  FOO 1 ;\n  LAYER V1 ;")]);

    assert_eq!(quiet, Vec::<String>::new());
    assert_one_warning(&loud, "ignoring via statement 'FOO'");
}

#[test]
fn a_site_takes_its_class_and_symmetry() {
    let lib = library(&[(
        "SITE core",
        "SITE pad\n  CLASS PAD ;\n  SYMMETRY X Y ;\n  SIZE 1 BY 1 ;\nEND pad\n\nSITE core",
    )]);

    let pad = lib.site(lib.site_id("pad").unwrap());
    assert_eq!(pad.class, SiteClass::Pad);
    assert!(pad.symmetry.x && pad.symmetry.y);
}

const NAND2_START: &str = "MACRO NAND2\n  CLASS CORE ;";

#[test]
fn a_macro_class_may_have_a_subclass() {
    let lib = library(&[(NAND2_START, "MACRO NAND2\n  CLASS CORE TIEHIGH ;")]);

    let nand2 = lib.cell_macro(lib.macro_id("NAND2").unwrap());
    assert_eq!(nand2.class, MacroClass::Core(CoreKind::TieHigh));
}

#[test]
fn a_density_block_is_skipped_silently() {
    let logs = warnings(&[(
        NAND2_START,
        "MACRO NAND2\n  CLASS CORE ;\n  DENSITY\n    LAYER M1 ;\n      RECT 0 0 0.6 2.0 50 ;\n  END",
    )]);

    assert_eq!(logs, Vec::<String>::new());
}

#[test]
fn macro_statements_that_carry_nothing_are_skipped_silently_and_others_warn() {
    let quiet = warnings(&[(
        NAND2_START,
        "MACRO NAND2\n  CLASS CORE ;\n  FOREIGN NAND2 0 0 ;",
    )]);
    let loud = warnings(&[(NAND2_START, "MACRO NAND2\n  CLASS CORE ;\n  FOO ;")]);

    assert_eq!(quiet, Vec::<String>::new());
    assert_one_warning(&loud, "ignoring macro statement 'FOO'");
}

#[test]
fn an_unknown_pin_statement_warns() {
    let logs = warnings(&[(
        "  PIN B\n    DIRECTION INPUT ;",
        "  PIN B\n    DIRECTION INPUT ;\n    FOO ;",
    )]);

    assert_one_warning(&logs, "ignoring pin statement 'FOO'");
}

#[test]
fn a_via_rule_must_name_the_metal_layers_its_cut_touches() {
    let wrong = error(&[(
        "  LAYER M2 ;\n    ENCLOSURE 0 0.05 ;\n  LAYER V1 ;",
        "  LAYER M3 ;\n    ENCLOSURE 0 0.05 ;\n  LAYER V1 ;",
    )]);

    assert!(
        wrong.contains("names metal layers its cut does not touch"),
        "{wrong}"
    );
}
