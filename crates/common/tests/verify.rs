//! Placement and routing checker tests on small hand-built designs.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, PlacementStatus, RouteElem};
use pare_common::db::indices::{CellId, CutLayerId, LayerId, NetId};
use pare_common::db::library::LayerRef;
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::placement::{PlacementViolation, placement_violations};
use pare_common::verify::routing::check_routing;
use pare_common::verify::shapes::Owner;
use std::path::{Path, PathBuf};

const M1: LayerId = LayerId(0);
const M2: LayerId = LayerId(1);

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

const fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu(x), Dbu(y))
}

/// An empty 10x10 um design over the fixture library.
fn empty_design() -> NetlistDB {
    let units = DbuPerMicron::new(1000).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    let mut db = NetlistDB::new("t".into(), library);
    db.die_area = Rect::new(pt(0, 0), pt(10_000, 10_000));
    db
}

const fn wire(layer: LayerId, from: Point<Dbu>, to: Point<Dbu>) -> RouteElem {
    RouteElem::Wire { layer, from, to }
}

fn via(db: &NetlistDB, cut: u8, at: Point<Dbu>) -> RouteElem {
    RouteElem::Via {
        via: db.library.default_via(CutLayerId(cut)).unwrap(),
        at,
    }
}

fn routed_net(db: &mut NetlistDB, name: &str, routing: Vec<RouteElem>) -> NetId {
    let net = db.net_or_insert(name);
    db.nets[net.index()].routing = routing;
    net
}

fn inv(db: &mut NetlistDB, name: &str, at: Point<Dbu>) -> CellId {
    let inv = db.library.macro_id("INV").unwrap();
    db.add_cell(
        name.into(),
        inv,
        at,
        Orientation::N,
        PlacementStatus::Placed,
    )
    .unwrap()
}

/// Two inverters with their A pins (M1, 100 wide, y 400..600) on one net.
fn two_inverters_on_a_net(db: &mut NetlistDB) -> NetId {
    let u1 = inv(db, "u1", pt(0, 0));
    let u2 = inv(db, "u2", pt(1000, 0));
    let net = db.net_or_insert("n");
    for cell in [u1, u2] {
        let a = db.cell_macro(cell).pin_id("A").unwrap();
        let _ = db.connect_cell_pin(cell, a, net);
    }
    net
}

#[test]
fn collinear_overlapping_wires_of_two_nets_are_a_short() {
    let mut db = empty_design();
    let _ = routed_net(&mut db, "a", vec![wire(M2, pt(1000, 0), pt(1000, 2000))]);
    let _ = routed_net(&mut db, "b", vec![wire(M2, pt(1000, 1500), pt(1000, 4000))]);

    let report = check_routing(&db);

    assert_eq!(report.shorts.len(), 1);
    assert_eq!(report.shorts[0].layer, LayerRef::Routing(M2));
}

#[test]
fn a_t_junction_between_two_nets_is_a_short() {
    let mut db = empty_design();
    let _ = routed_net(&mut db, "a", vec![wire(M1, pt(0, 500), pt(1000, 500))]);
    let _ = routed_net(&mut db, "b", vec![wire(M1, pt(500, 500), pt(500, 1500))]);

    assert_eq!(check_routing(&db).shorts.len(), 1);
}

#[test]
fn a_via_landing_on_another_nets_wire_is_a_short() {
    let mut db = empty_design();
    let _ = routed_net(&mut db, "a", vec![wire(M2, pt(1000, 0), pt(1000, 2000))]);
    let b = via(&db, 0, pt(1000, 1000));
    let _ = routed_net(&mut db, "b", vec![b]);

    let shorts = check_routing(&db).shorts;

    assert_eq!(shorts.len(), 1);
    assert_eq!(shorts[0].layer, LayerRef::Routing(M2));
}

#[test]
fn wire_ends_extended_by_half_width_touch_and_short() {
    let mut db = empty_design();
    let _ = routed_net(&mut db, "a", vec![wire(M1, pt(0, 500), pt(1000, 500))]);
    let _ = routed_net(&mut db, "b", vec![wire(M1, pt(1100, 500), pt(2000, 500))]);

    assert_eq!(check_routing(&db).shorts.len(), 1);
}

#[test]
fn wires_exactly_min_spacing_apart_are_clean() {
    let mut db = empty_design();
    let _ = routed_net(&mut db, "a", vec![wire(M1, pt(0, 500), pt(1000, 500))]);
    let _ = routed_net(&mut db, "b", vec![wire(M1, pt(0, 700), pt(1000, 700))]);

    let report = check_routing(&db);

    assert!(report.shorts.is_empty());
    assert!(report.spacing.is_empty());
}

#[test]
fn wires_closer_than_min_spacing_are_a_spacing_violation() {
    let mut db = empty_design();
    let _ = routed_net(&mut db, "a", vec![wire(M1, pt(0, 500), pt(1000, 500))]);
    let _ = routed_net(&mut db, "b", vec![wire(M1, pt(0, 680), pt(1000, 680))]);

    let report = check_routing(&db);

    assert!(report.shorts.is_empty());
    assert_eq!(report.spacing.len(), 1);
    assert_eq!(report.spacing[0].distance_sq, 80 * 80);
}

#[test]
fn pins_joined_through_vias_and_an_upper_wire_are_connected() {
    let mut db = empty_design();
    let net = two_inverters_on_a_net(&mut db);
    let routing = vec![
        via(&db, 0, pt(100, 500)),
        wire(M2, pt(100, 500), pt(1100, 500)),
        via(&db, 0, pt(1100, 500)),
    ];
    db.nets[net.index()].routing = routing;

    let report = check_routing(&db);

    assert!(report.opens.is_empty(), "{:?}", report.opens);
    assert!(report.shorts.is_empty(), "{:?}", report.shorts);
}

#[test]
fn a_wire_passing_over_a_pin_on_another_layer_does_not_connect_it() {
    let mut db = empty_design();
    let net = two_inverters_on_a_net(&mut db);
    let routing = vec![
        via(&db, 0, pt(100, 500)),
        wire(M2, pt(100, 500), pt(1100, 500)),
    ];
    db.nets[net.index()].routing = routing;

    let opens = check_routing(&db).opens;

    assert_eq!(opens.len(), 1);
    assert_eq!(opens[0].components, 2);
}

#[test]
fn an_unrouted_net_is_open() {
    let mut db = empty_design();
    let net = two_inverters_on_a_net(&mut db);

    let opens = check_routing(&db).opens;

    assert_eq!(opens.len(), 1);
    assert_eq!(opens[0].net, net);
}

#[test]
fn a_wire_through_an_unused_pin_shorts_to_it() {
    let mut db = empty_design();
    let net = two_inverters_on_a_net(&mut db);
    db.nets[net.index()].routing = vec![wire(M1, pt(100, 500), pt(1100, 500))];

    let shorts = check_routing(&db).shorts;

    assert!(
        shorts.iter().any(|s| s.other == Owner::Obstacle),
        "{shorts:?}"
    );
}

#[test]
fn wiring_stats_count_wrong_way_wires_and_vias() {
    let mut db = empty_design();
    let net = two_inverters_on_a_net(&mut db);
    let routing = vec![
        via(&db, 0, pt(100, 500)),
        wire(M2, pt(100, 500), pt(1100, 500)),
        via(&db, 0, pt(1100, 500)),
    ];
    db.nets[net.index()].routing = routing;

    let stats = check_routing(&db).stats;

    assert_eq!(stats.vias, 2);
    assert_eq!(stats.wrong_way, 1);
    assert_eq!(stats.wirelength[M2.index()], Dbu(1000));
}

fn tiny_placed() -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse(&fixture("tiny.def"), library).unwrap()
}

fn cell(db: &NetlistDB, name: &str) -> CellId {
    db.cell_id(name).unwrap()
}

#[test]
fn fixture_placement_violations_are_overlaps_and_a_wrong_orientation() {
    let db = tiny_placed();
    let (u1, u5, u6, u7) = (
        cell(&db, "u1"),
        cell(&db, "u5"),
        cell(&db, "u6"),
        cell(&db, "u7"),
    );

    let mut found = placement_violations(&db);
    found.sort_by_key(|v| format!("{v:?}"));

    assert_eq!(
        found,
        [
            PlacementViolation::Overlap(u1, u5),
            PlacementViolation::Overlap(u1, u6),
            PlacementViolation::Overlap(u5, u6),
            PlacementViolation::WrongOrientation(u7),
        ]
    );
}

#[test]
fn a_movable_cell_on_a_fixed_macro_overlaps_it() {
    let mut db = tiny_placed();
    let u4 = cell(&db, "u4");
    db.cells[u4.index()].location = pt(4000, 2000);

    assert!(
        placement_violations(&db).contains(&PlacementViolation::Overlap(u4, cell(&db, "ram0")))
    );
}

#[test]
fn a_cell_between_sites_is_off_site() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    db.cells[u1.index()].location = pt(250, 0);

    assert!(placement_violations(&db).contains(&PlacementViolation::OffSite(u1)));
}

#[test]
fn a_cell_between_rows_is_not_in_a_row() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    db.cells[u1.index()].location = pt(200, 100);

    assert!(placement_violations(&db).contains(&PlacementViolation::NotInRow(u1)));
}

#[test]
fn a_y_mirrored_cell_is_legal_in_its_row() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    db.cells[u1.index()].orient = Orientation::FN;

    assert!(
        !placement_violations(&db)
            .iter()
            .any(|v| matches!(v, PlacementViolation::WrongOrientation(c) if *c == u1))
    );
}

#[test]
fn two_cuts_of_one_net_closer_than_cut_spacing_are_a_spacing_violation() {
    let mut db = empty_design();
    let (a, b) = (via(&db, 0, pt(1000, 1000)), via(&db, 0, pt(1000, 1150)));
    let _ = routed_net(&mut db, "a", vec![a, b]);

    let report = check_routing(&db);

    let cut = report
        .spacing
        .iter()
        .filter(|v| v.layer == LayerRef::Cut(CutLayerId(0)))
        .count();
    assert_eq!(cut, 1, "{:?}", report.spacing);
}
