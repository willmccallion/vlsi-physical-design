use super::*;
use crate::placement::{PlacementStart, run_placement};
use crate::plan::FlowPlan;
use crate::repair::Constraints;
use crate::testing::{
    cts_fixture, half_height_tiny, scratch_dir, timed, timed_config, tiny, tiny_config,
};
use pare_common::db::indices::{MacroId, MacroPinId};
use pare_common::db::parser::{lef, liberty};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_router::{NetGuide, SavePoint};

fn pin_of(library: &Library, master: &str, pin: &str) -> (MacroId, MacroPinId) {
    let (id, found) = library
        .macros
        .iter_enumerated()
        .find(|(_, m)| m.name == master)
        .unwrap();
    (id, found.pin_id(pin).unwrap())
}

#[test]
fn register_clock_pins_and_the_signal_pins_of_pads_make_clock_nets_leaves() {
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[cts_fixture("clock.lef")], units).unwrap();
    let source = std::fs::read_to_string(cts_fixture("clock.lib"))
        .unwrap()
        .replace("cell (BUF_X1) {", "cell (BUF_X1) { pad_cell : true;");
    let cells = liberty::parse_str(&source, Path::new("clock.lib")).unwrap();

    let leaves = leaf_clock_pins(&library, &cells);

    let expected: LeafClockPins = [
        pin_of(&library, "DFF_X1", "CK"),
        pin_of(&library, "DFF_SLOW", "CK"),
        pin_of(&library, "DFF_SLOWER", "CK"),
        pin_of(&library, "BUF_X1", "A"),
        pin_of(&library, "BUF_X1", "Z"),
    ]
    .into_iter()
    .collect();
    assert_eq!(leaves, expected);
}

#[test]
fn without_a_repair_the_leaf_clock_pins_come_from_the_liberty_of_the_config() {
    let config = timed_config("", &scratch_dir("config-leaves"));
    let db = timed();
    let liberty = load_liberty(&config.input.liberty_files).unwrap();

    let leaves = design_leaf_clock_pins(&config, &db, None).unwrap();

    assert_eq!(leaves, leaf_clock_pins(&db.library, &liberty));
    assert_ne!(leaves, LeafClockPins::default());
}

#[test]
fn without_liberty_files_no_clock_net_has_leaves() {
    let leaves = design_leaf_clock_pins(&tiny_config(""), &tiny(), None).unwrap();

    assert_eq!(leaves, LeafClockPins::default());
}

#[test]
fn a_save_point_is_global_or_a_round_number() {
    assert_eq!(parse_save_point("global"), Ok(SavePoint::Global));
    assert_eq!(parse_save_point("0"), Ok(SavePoint::Round(0)));
    assert_eq!(parse_save_point("12"), Ok(SavePoint::Round(12)));
}

#[test]
fn a_save_point_that_is_neither_names_what_it_takes() {
    let error = parse_save_point("first").unwrap_err();

    assert!(
        error.contains("first") && error.contains("global"),
        "{error}"
    );
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

#[test]
fn guides_are_written_net_by_net_in_the_ispd_format() {
    let db = tiny();
    let layer = |name: &str| db.library.routing_layer_id(name).unwrap();
    let guides = [
        NetGuide {
            net: db.net_id("n3").unwrap(),
            boxes: vec![
                (layer("M1"), rect(2000, 0, 4000, 2000)),
                (layer("M3"), rect(2000, 2000, 6000, 4000)),
            ],
        },
        NetGuide {
            net: db.net_id("in").unwrap(),
            boxes: vec![(layer("M2"), rect(0, 0, 2000, 6000))],
        },
    ];
    let path = scratch_dir("guides").join("tiny.guide");

    write_guides(&path, &db, &guides).unwrap();

    assert_eq!(
        std::fs::read_to_string(&path).unwrap(),
        "n3\n(\n2000 0 4000 2000 M1\n2000 2000 6000 4000 M3\n)\n\
             in\n(\n0 0 2000 6000 M2\n)\n"
    );
}

#[test]
fn no_guides_write_an_empty_file() {
    let path = scratch_dir("no-guides").join("tiny.guide");

    write_guides(&path, &tiny(), &[]).unwrap();

    assert_eq!(std::fs::read_to_string(&path).unwrap(), "");
}

#[test]
fn a_routed_image_keeps_the_die_in_proportion() {
    let dir = scratch_dir("routed-image");

    write_routed(&half_height_tiny(), &dir).unwrap();

    assert_eq!(
        image::image_dimensions(dir.join("routed.png")).unwrap(),
        (ROUTED_IMAGE_PIXELS, ROUTED_IMAGE_PIXELS / 2)
    );
}

#[test]
fn repair_on_global_routes_propagates_the_clocks_without_clock_trees() {
    let config = timed_config("clock_tree = false", &scratch_dir("propagated"));
    let mut db = timed();
    let plan = FlowPlan::resolve(&config, &db).unwrap();
    let mut report = Report::default();
    let mut repair = run_placement(
        &config,
        &plan,
        &mut db,
        PlacementStart::Scratch,
        &mut report,
    )
    .unwrap()
    .unwrap();

    let _ = global_route_with_repair(&config, &mut db, &mut repair, &mut report).unwrap();

    let timed = repair.timed().unwrap();
    assert!(timed.sdc.clocks.iter().all(|clock| clock.propagated));
}

#[test]
fn without_constraints_the_design_is_still_repaired_on_global_routes() {
    let config = timed_config("", &scratch_dir("untimed-repair"));
    let mut db = timed();
    let plan = FlowPlan::resolve(&config, &db).unwrap();
    let mut report = Report::default();
    let mut repair = run_placement(
        &config,
        &plan,
        &mut db,
        PlacementStart::Scratch,
        &mut report,
    )
    .unwrap()
    .unwrap();
    let ports = ["clk".to_string()];
    repair.constraints = Constraints::Ports(&ports);
    report.phases.clear();

    let session = global_route_with_repair(&config, &mut db, &mut repair, &mut report).unwrap();

    assert!(session.is_some());
    let names: Vec<&str> = report.phases.iter().map(|phase| phase.name).collect();
    assert_eq!(
        names,
        [
            "global routing",
            "design repair on global routes",
            "legalization on global routes"
        ]
    );
    assert!(report.global_route_repair.unwrap().timing.is_none());
}
