use super::*;
use pare_common::db::design::PlacementStatus;
use pare_common::db::indices::SiteId;
use pare_common::db::library::{CoreKind, Macro, MacroClass, Symmetry};
use pare_common::db::parser::{def, lef};
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

const fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
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
fn every_kind_has_its_report_name() {
    let names = [
        (PlacementViolationKind::Unplaced, "Unplaced"),
        (PlacementViolationKind::OutsideDie, "Outside die"),
        (PlacementViolationKind::Overlap, "Overlap"),
        (PlacementViolationKind::InBlockage, "In blockage"),
        (PlacementViolationKind::NotInRow, "Not in a row"),
        (PlacementViolationKind::OffSite, "Off site"),
        (
            PlacementViolationKind::WrongOrientation,
            "Wrong orientation",
        ),
        (PlacementViolationKind::OutsideRegion, "Outside region"),
        (PlacementViolationKind::InFence, "In a fence"),
    ];

    for (kind, name) in names {
        assert_eq!(kind.name(), name);
    }
}

#[test]
fn a_cell_sticking_out_of_its_row_on_the_right_is_not_in_a_row() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    db.cells[u1].location = pt(5800, 0);

    assert!(placement_violations(&db).contains(&PlacementViolation::NotInRow(u1)));
}

#[test]
fn a_cell_sticking_out_of_its_row_on_the_left_is_not_in_a_row() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    db.cells[u1].location = pt(-200, 0);

    assert!(placement_violations(&db).contains(&PlacementViolation::NotInRow(u1)));
}

#[test]
fn sites_are_counted_from_the_rows_origin() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    db.rows[0].origin = pt(100, 0);
    db.rows[0].step = pt(300, 0);
    db.cells[u1].location = pt(400, 0);

    assert!(!placement_violations(&db).contains(&PlacementViolation::OffSite(u1)));
}

#[test]
fn a_row_without_a_site_step_has_no_site_grid() {
    let mut db = tiny_placed();
    let u1 = cell(&db, "u1");
    db.library.sites[SiteId(0)].size = pt(600, 2000);
    db.rows[0].step = pt(0, 0);
    db.cells[u1].location = pt(100, 0);

    let found = placement_violations(&db);

    assert!(!found.contains(&PlacementViolation::OffSite(u1)));
    assert!(!found.contains(&PlacementViolation::NotInRow(u1)));
}

/// Adds a movable cell of a master `size` big at `at`, over u1, which
/// stands at (200, 0).
fn add_degenerate_cell(db: &mut NetlistDB, size: Point<Dbu>, at: Point<Dbu>) {
    let degenerate = Macro::new(
        "degenerate".into(),
        MacroClass::Core(CoreKind::Regular),
        size,
        Symmetry::default(),
        None,
    );
    let master = db.library.add_macro(degenerate).unwrap();
    let _ = db
        .add_cell(
            "degenerate".into(),
            master,
            at,
            Orientation::N,
            PlacementStatus::Placed,
        )
        .unwrap();
}

fn overlaps_of(db: &NetlistDB) -> Vec<PlacementViolation> {
    placement_violations(db)
        .into_iter()
        .filter(|v| v.kind() == PlacementViolationKind::Overlap)
        .collect()
}

#[test]
fn a_cell_without_width_overlaps_nothing() {
    let mut db = tiny_placed();
    add_degenerate_cell(&mut db, pt(0, 500), pt(300, 100));

    assert_eq!(overlaps_of(&db), []);
}

#[test]
fn a_cell_without_height_overlaps_nothing() {
    let mut db = tiny_placed();
    add_degenerate_cell(&mut db, pt(100, 0), pt(300, 300));

    assert_eq!(overlaps_of(&db), []);
}

#[test]
fn a_rectangle_ending_on_a_square_boundary_covers_no_square_past_it() {
    let r = Rect::new(pt(-100, 0), pt(100, 200));

    let squares: Vec<_> = bins_covering(&r, Dbu::new(100)).collect();

    let bin = |column, row| Bin { column, row };
    assert_eq!(squares, [bin(-1, 0), bin(-1, 1), bin(0, 0), bin(0, 1)]);
}
