use super::*;
use crate::db::parser::lef;
use crate::testing::capture_logs;
use std::path::PathBuf;

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

fn tiny_library() -> Library {
    lef::parse_files(&[fixture("tiny.lef")], DbuPerMicron::new(1000).unwrap()).unwrap()
}

/// A design of `body` between the DEF header and its end, over the tiny LEF.
fn design(body: &str) -> Result<NetlistDB, ParseError> {
    let text = format!(
        "VERSION 5.8 ;\nDESIGN t ;\nUNITS DISTANCE MICRONS 1000 ;\n\
         DIEAREA ( 0 0 ) ( 6000 6000 ) ;\n{body}\nEND DESIGN\n"
    );
    parse_str(&text, Path::new("t.def"), tiny_library())
}

/// The design of `body` and the lines parsing it logged.
fn design_logged(body: &str) -> (Result<NetlistDB, ParseError>, Vec<String>) {
    let mut parsed = None;
    let logs = capture_logs(|| parsed = Some(design(body)));
    (parsed.unwrap(), logs)
}

#[test]
fn units_after_a_section_are_not_looked_for() {
    let path = std::env::temp_dir().join(format!("pare-def-units-{}.def", std::process::id()));
    std::fs::write(
        &path,
        "DESIGN t ;\nCOMPONENTS 0 ;\nEND COMPONENTS\nUNITS DISTANCE MICRONS 1000 ;\nEND DESIGN\n",
    )
    .unwrap();

    let units = read_units(&path);

    std::fs::remove_file(&path).unwrap();
    assert!(units.is_err());
}

#[test]
fn an_extension_is_skipped_with_one_warning() {
    let (db, logs) = design_logged(
        "BEGINEXT \"tag\"\n  CREATOR \"tool\" ;\nENDEXT\nBEGINEXT \"other\"\nENDEXT\n\
         COMPONENTS 1 ;\n- u1 INV ;\nEND COMPONENTS",
    );

    assert_eq!(db.unwrap().cells.len(), 1);
    assert_eq!(logs.len(), 1, "{logs:?}");
    assert!(
        logs[0].contains("ignoring extension 'BEGINEXT'"),
        "{logs:?}"
    );
}

#[test]
fn sections_and_statements_that_carry_nothing_are_skipped_silently() {
    let (db, logs) = design_logged(
        "HISTORY made by hand ;\n\
         SCANCHAINS 1 ;\n- chain1 + START PIN in + STOP PIN out ;\nEND SCANCHAINS\n\
         COMPONENTS 1 ;\n- u1 INV ;\nEND COMPONENTS",
    );

    assert_eq!(db.unwrap().cells.len(), 1);
    assert_eq!(logs, Vec::<String>::new());
}

#[test]
fn a_section_that_is_not_read_is_skipped_with_a_warning() {
    let (db, logs) = design_logged(
        "FILLS 1 ;\n- LAYER M1 RECT ( 0 0 ) ( 10 10 ) ;\nEND FILLS\n\
         COMPONENTS 1 ;\n- u1 INV ;\nEND COMPONENTS",
    );

    assert_eq!(db.unwrap().cells.len(), 1);
    assert_eq!(logs.len(), 1, "{logs:?}");
    assert!(logs[0].contains("ignoring section 'FILLS'"), "{logs:?}");
}

#[test]
fn only_options_that_carry_nothing_are_skipped_without_a_warning() {
    let (db, logs) = design_logged(
        "COMPONENTS 3 ;\n- u1 INV + SOURCE DIST ;\n- u2 INV + ANTENNAMODEL OXIDE1 ;\n\
         - u3 INV + MYOPTION 3 ;\nEND COMPONENTS",
    );

    assert_eq!(db.unwrap().cells.len(), 3);
    assert_eq!(logs.len(), 1, "{logs:?}");
    assert!(
        logs[0].contains("ignoring component option 'MYOPTION'"),
        "{logs:?}"
    );
}

const M1M2_RECTS: &str = "+ RECT V1 ( -50 -50 ) ( 50 50 ) + RECT M1 ( -50 -100 ) ( 50 100 )";

#[test]
fn a_def_via_named_as_a_lef_via_warns_only_when_it_differs() {
    let (same, same_logs) = design_logged(&format!(
        "VIAS 1 ;\n- M1M2 {M1M2_RECTS} + RECT M2 ( -50 -100 ) ( 50 100 ) ;\nEND VIAS"
    ));
    let (_, differing_logs) = design_logged(&format!(
        "VIAS 1 ;\n- M1M2 {M1M2_RECTS} + RECT M2 ( -50 -150 ) ( 50 150 ) ;\nEND VIAS"
    ));

    assert!(same.is_ok());
    assert_eq!(same_logs, Vec::<String>::new());
    assert_eq!(differing_logs.len(), 1, "{differing_logs:?}");
    assert!(
        differing_logs[0].contains("using the LEF via"),
        "{differing_logs:?}"
    );
}

#[test]
fn a_cover_component_keeps_its_status() {
    let db = design("COMPONENTS 1 ;\n- u1 INV + COVER ( 200 0 ) N ;\nEND COMPONENTS").unwrap();

    assert_eq!(db.cells[CellId::new(0)].status, PlacementStatus::Cover);
}

/// The message of the error parsing `body` gives.
fn error(body: &str) -> String {
    design(body).unwrap_err().to_string()
}

#[test]
fn halos_and_pin_polygons_are_refused() {
    let halo = error("COMPONENTS 1 ;\n- u1 INV + HALO 1 1 1 1 ;\nEND COMPONENTS");
    let polygon =
        error("PINS 1 ;\n- p + NET n + POLYGON M1 ( 0 0 ) ( 10 0 ) ( 10 10 ) ;\nEND PINS");

    assert!(halo.contains("+ HALO is not supported"), "{halo}");
    assert!(
        polygon.contains("POLYGON geometry is not supported"),
        "{polygon}"
    );
}

#[test]
fn a_pin_and_its_net_take_the_use_it_gives() {
    let db = design(
        "PINS 1 ;\n- vdd + NET vdd + USE POWER ;\nEND PINS\n\
         NETS 1 ;\n- clk + USE CLOCK ;\nEND NETS",
    )
    .unwrap();

    let pin = &db.io_pins[db.io_pin_id("vdd").unwrap()];
    assert_eq!(pin.signal_use, SignalUse::Power);
    assert_eq!(db.nets[pin.net].signal_use, SignalUse::Power);
    assert_eq!(
        db.nets[db.net_id("clk").unwrap()].signal_use,
        SignalUse::Clock
    );
}

#[test]
fn a_pin_may_have_one_port_only() {
    let one = design("PINS 1 ;\n- p + NET n + PORT + LAYER M2 ( 0 0 ) ( 10 10 ) ;\nEND PINS");
    let two = error(
        "PINS 1 ;\n- p + NET n + PORT + LAYER M2 ( 0 0 ) ( 10 10 ) \
         + PORT + LAYER M2 ( 0 0 ) ( 10 10 ) ;\nEND PINS",
    );

    assert!(one.is_ok());
    assert!(two.contains("multiple PORTs are not supported"), "{two}");
}

#[test]
fn a_pin_named_twice_is_refused() {
    let twice = error("PINS 2 ;\n- p + NET a ;\n- p + NET b ;\nEND PINS");

    assert!(twice.contains("duplicate pin 'p'"), "{twice}");
}

#[test]
fn pins_no_net_lists_are_still_connected() {
    let db = design("PINS 1 ;\n- p + NET floating ;\nEND PINS").unwrap();

    let pin = &db.io_pins[db.io_pin_id("p").unwrap()];
    assert_eq!(db.nets[pin.net].name, "floating");
}

#[test]
fn a_net_listing_a_pin_of_another_net_is_refused() {
    let wrong = error("PINS 1 ;\n- p + NET a ;\nEND PINS\nNETS 1 ;\n- b ( PIN p ) ;\nEND NETS");

    assert!(wrong.contains("pin 'p'"), "{wrong}");
}

#[test]
fn a_special_net_takes_its_use_and_refuses_polygons() {
    let db = design("SPECIALNETS 1 ;\n- gnd + USE GROUND ;\nEND SPECIALNETS").unwrap();
    let polygon =
        error("SPECIALNETS 1 ;\n- gnd + POLYGON M1 ( 0 0 ) ( 10 0 ) ( 10 10 ) ;\nEND SPECIALNETS");

    let gnd = db.special_nets.iter().find(|n| n.name == "gnd").unwrap();
    assert_eq!(gnd.signal_use, SignalUse::Ground);
    assert!(polygon.contains("+ POLYGON is not supported"), "{polygon}");
}

/// The layers of the wires `wiring` routes net `n` on.
fn wire_layers(wiring: &str) -> Result<Vec<LayerId>, ParseError> {
    let db = design(&format!("NETS 1 ;\n- n + ROUTED {wiring} ;\nEND NETS"))?;
    Ok(db.nets[db.net_id("n").unwrap()]
        .routing
        .iter()
        .filter_map(|e| match e {
            RouteElem::Wire { layer, .. } => Some(*layer),
            _ => None,
        })
        .collect())
}

#[test]
fn taper_style_and_mask_options_of_wiring_are_skipped() {
    let layers = wire_layers("M2 TAPER ( 100 100 ) MASK 1 ( 500 * ) STYLE 2 ( * 900 )").unwrap();

    assert_eq!(layers.len(), 2);
}

#[test]
fn virtual_wiring_is_refused() {
    let wiring = wire_layers("M2 ( 100 100 ) VIRTUAL ( 500 100 )")
        .unwrap_err()
        .to_string();

    assert!(
        wiring.contains("VIRTUAL wiring is not supported"),
        "{wiring}"
    );
}

#[test]
fn a_via_takes_the_wiring_to_its_other_layer_up_or_down() {
    let lib = tiny_library();
    let id = |name: &str| lib.routing_layer_id(name).unwrap();

    let down = wire_layers("M2 ( 100 100 ) ( 500 * ) M1M2 ( * 900 )").unwrap();
    let up = wire_layers("M1 ( 100 100 ) ( 500 * ) M1M2 ( * 900 )").unwrap();
    let apart = wire_layers("M3 ( 100 100 ) ( 500 * ) M1M2 ( * 900 )").unwrap_err();

    assert_eq!(down, [id("M2"), id("M1")]);
    assert_eq!(up, [id("M1"), id("M2")]);
    assert!(apart.to_string().contains("does not touch"), "{apart}");
}

#[test]
fn a_pattern_matches_names_by_its_stars() {
    assert!(matches_pattern("u*", "u12"));
    assert!(matches_pattern("a*b*c", "axxbyyc"));
    assert!(matches_pattern("a*b*c", "abc"));
    assert!(!matches_pattern("a*b*c", "axxcyyb"));
    assert!(!matches_pattern("a*z", "abc"));
    assert!(!matches_pattern("ab*ba", "aba"));
    assert!(matches_pattern("exact", "exact"));
    assert!(!matches_pattern("exact", "exactly"));
}

#[test]
fn vias_have_the_same_geometry_with_the_same_cut_and_shapes_in_any_order() {
    let lib = tiny_library();
    let via = |name: &str| lib.via(lib.via_id(name).unwrap()).clone();
    let m1m2 = via("M1M2");
    let mut reordered = m1m2.clone();
    reordered.shapes.reverse();
    let other_cut = ViaDef {
        cut: via("M2M3").cut,
        ..m1m2.clone()
    };
    let mut fewer = m1m2.clone();
    let _ = fewer.shapes.pop();
    let mut moved = m1m2.clone();
    moved.shapes[0].rect = moved.shapes[0]
        .rect
        .translate(Point::new(Dbu::new(5), Dbu::ZERO));
    let mut doubled = m1m2.clone();
    doubled.shapes[1] = doubled.shapes[0];

    assert!(same_geometry(&m1m2, &reordered));
    for other in [&other_cut, &fewer, &moved] {
        assert!(!same_geometry(&m1m2, other), "{other:?}");
    }
    assert!(!same_geometry(&doubled, &m1m2));
}
