//! Placement and routing checker tests on small hand-built designs.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{
    NetlistDB, PlacementStatus, Region, RegionKind, RouteElem, SpecialNet, SpecialWire,
};
use pare_common::db::indices::{CellId, CutLayerId, LayerId, NetId, RegionId};
use pare_common::db::library::{LayerRef, SignalUse};
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
fn two_wires_of_one_net_closer_than_min_spacing_are_a_spacing_violation() {
    let mut db = empty_design();
    let routing = vec![
        wire(M1, pt(0, 500), pt(1000, 500)),
        wire(M1, pt(1180, 500), pt(2000, 500)),
    ];
    let _ = routed_net(&mut db, "a", routing);

    let report = check_routing(&db);

    assert!(report.shorts.is_empty());
    assert_eq!(report.spacing.len(), 1);
    assert_eq!(report.spacing[0].distance_sq, 80 * 80);
}

#[test]
fn a_gap_in_one_net_filled_by_its_own_wire_is_clean() {
    let mut db = empty_design();
    let routing = vec![
        wire(M1, pt(0, 500), pt(1000, 500)),
        wire(M1, pt(1180, 500), pt(2000, 500)),
        wire(M1, pt(900, 500), pt(1300, 500)),
    ];
    let _ = routed_net(&mut db, "a", routing);

    assert!(check_routing(&db).spacing.is_empty());
}

#[test]
fn a_gap_filled_by_two_overlapping_pieces_of_the_net_is_clean() {
    let mut db = empty_design();
    let routing = vec![
        wire(M1, pt(0, 500), pt(1000, 500)),
        wire(M1, pt(1180, 500), pt(2000, 500)),
        wire(M1, pt(1000, 500), pt(1080, 500)),
        wire(M1, pt(1060, 500), pt(1180, 500)),
    ];
    let _ = routed_net(&mut db, "a", routing);

    assert!(check_routing(&db).spacing.is_empty());
}

#[test]
fn two_vias_of_one_net_must_keep_their_metal_apart_even_when_wired() {
    let mut db = empty_design();
    let routing = vec![
        via(&db, 0, pt(1000, 500)),
        via(&db, 0, pt(1000, 780)),
        wire(M2, pt(1000, 500), pt(1000, 780)),
    ];
    let _ = routed_net(&mut db, "a", routing);

    let report = check_routing(&db);

    let on_m2: Vec<_> = report
        .spacing
        .iter()
        .filter(|v| v.layer == LayerRef::Routing(M2))
        .collect();
    assert_eq!(on_m2.len(), 1, "{:?}", report.spacing);
    assert_eq!(on_m2[0].distance_sq, 80 * 80);
}

#[test]
fn a_wire_running_long_beside_a_wide_shape_needs_the_table_spacing() {
    let mut db = empty_design();
    let _ = routed_net(&mut db, "a", vec![wire(M2, pt(600, 1000), pt(600, 3000))]);
    let _ = routed_net(&mut db, "b", vec![wire(M2, pt(600, 4000), pt(600, 4300))]);
    db.special_nets.push(SpecialNet {
        name: "VDD".into(),
        signal_use: SignalUse::Power,
        connections: Vec::new(),
        wires: vec![SpecialWire {
            layer: M2,
            width: Dbu(400),
            from: pt(200, 1000),
            to: pt(200, 3000),
            extensions: (Dbu::ZERO, Dbu::ZERO),
            shape: Some("STRIPE".into()),
        }],
        rects: Vec::new(),
        vias: Vec::new(),
    });

    let spacing = check_routing(&db).spacing;

    assert_eq!(spacing.len(), 1, "{spacing:?}");
    assert_eq!(
        (spacing[0].other, spacing[0].distance_sq),
        (Owner::Obstacle, 150 * 150)
    );
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
fn fixture_placement_violations_are_unplaced_cells_and_a_wrong_orientation() {
    let db = tiny_placed();
    let (u5, u6, u7) = (cell(&db, "u5"), cell(&db, "u6"), cell(&db, "u7"));

    let mut found = placement_violations(&db);
    found.sort_by_key(|v| format!("{v:?}"));

    assert_eq!(
        found,
        [
            PlacementViolation::Unplaced(u5),
            PlacementViolation::Unplaced(u6),
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
fn stacked_cells_give_one_overlap_per_movable_cell() {
    let mut db = tiny_placed();
    let stacked = ["u1", "u2", "u4", "u7"].map(|name| cell(&db, name));
    for c in stacked {
        db.cells[c.index()].location = pt(1000, 2000);
        db.cells[c.index()].orient = Orientation::FS;
    }

    let overlapping: Vec<CellId> = placement_violations(&db)
        .into_iter()
        .filter_map(|v| match v {
            PlacementViolation::Overlap(a, _) => Some(a),
            _ => None,
        })
        .collect();

    assert_eq!(overlapping, stacked);
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

/// Adds a region of `kind` over `rects` and assigns `members` to it.
fn add_region(db: &mut NetlistDB, kind: RegionKind, rects: &[Rect<Dbu>], members: &[CellId]) {
    let id = RegionId::new(db.regions.len());
    db.regions.push(Region {
        name: format!("r{}", id.index()),
        rects: rects.to_vec(),
        kind,
    });
    for &c in members {
        db.cells[c.index()].region = Some(id);
    }
}

fn region_violations(db: &NetlistDB) -> Vec<PlacementViolation> {
    placement_violations(db)
        .into_iter()
        .filter(|v| {
            matches!(
                v,
                PlacementViolation::OutsideRegion(_) | PlacementViolation::InFence(_)
            )
        })
        .collect()
}

/// u1 sits at x 200..600, y 0..2000 in the fixture.
const fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(pt(x0, y0), pt(x1, y1))
}

#[test]
fn a_fence_member_inside_its_fence_is_legal() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    add_region(&mut db, RegionKind::Fence, &[rect(0, 0, 1000, 2000)], &[u1]);

    assert!(region_violations(&db).is_empty());
}

#[test]
fn a_fence_member_spanning_two_abutting_rectangles_is_inside() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    let rects = [rect(0, 0, 400, 2000), rect(400, 0, 1000, 2000)];
    add_region(&mut db, RegionKind::Fence, &rects, &[u1]);

    assert!(region_violations(&db).is_empty());
}

#[test]
fn a_fence_member_sticking_out_is_outside_its_region() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    add_region(&mut db, RegionKind::Fence, &[rect(0, 0, 400, 2000)], &[u1]);

    assert_eq!(
        region_violations(&db),
        [PlacementViolation::OutsideRegion(u1)]
    );
}

#[test]
fn an_inclusive_region_member_must_lie_inside_too() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    add_region(
        &mut db,
        RegionKind::Inclusive,
        &[rect(0, 0, 400, 2000)],
        &[u1],
    );

    assert_eq!(
        region_violations(&db),
        [PlacementViolation::OutsideRegion(u1)]
    );
}

#[test]
fn a_guide_region_constrains_nothing() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    add_region(
        &mut db,
        RegionKind::Guide,
        &[rect(5000, 0, 6000, 2000)],
        &[u1],
    );

    assert!(region_violations(&db).is_empty());
}

#[test]
fn a_cell_outside_every_fence_may_not_overlap_one() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    add_region(&mut db, RegionKind::Fence, &[rect(500, 0, 900, 2000)], &[]);

    assert_eq!(region_violations(&db), [PlacementViolation::InFence(u1)]);
}

#[test]
fn a_cell_outside_every_fence_may_overlap_an_inclusive_region() {
    let mut db = tiny_placed();
    add_region(
        &mut db,
        RegionKind::Inclusive,
        &[rect(500, 0, 900, 2000)],
        &[],
    );

    assert!(region_violations(&db).is_empty());
}

#[test]
fn a_cell_abutting_a_fence_does_not_overlap_it() {
    let mut db = tiny_placed();
    add_region(&mut db, RegionKind::Fence, &[rect(600, 0, 900, 2000)], &[]);

    assert!(region_violations(&db).is_empty());
}
