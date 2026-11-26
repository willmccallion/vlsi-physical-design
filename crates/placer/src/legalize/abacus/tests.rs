use super::*;
use pare_common::db::design::{Region, RegionKind, Row};
use pare_common::db::indices::SiteId;
use pare_common::db::library::{CoreKind, Library, Macro, MacroClass, Site, SiteClass, Symmetry};
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::geom::orient::Orientation;
use pare_verify::placement::placement_violations;

fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

fn legalize_zoned(db: &mut NetlistDB) -> Result<(), LegalizeError> {
    let zones = Zones::new(db).unwrap();
    legalize(db, &zones)
}

/// Two 10-site rows (N then FS) of 10 x 100 sites over a 100 x 200 die.
fn design(cells: &[(Point<Dbu>, PlacementStatus, i64)]) -> NetlistDB {
    let mut lib = Library::new(DbuPerMicron::new(1000).unwrap());
    let symmetry = Symmetry {
        x: true,
        y: true,
        r90: false,
    };
    let site = lib
        .add_site(Site {
            name: "s".into(),
            class: SiteClass::Core,
            size: pt(10, 100),
            symmetry,
        })
        .unwrap();
    let mut db = NetlistDB::new("t".into(), lib);
    db.die_area = Rect::new(pt(0, 0), pt(100, 200));
    for (k, orient) in [Orientation::N, Orientation::FS].into_iter().enumerate() {
        db.rows.push(Row {
            name: format!("r{k}"),
            site: SiteId(0),
            origin: pt(0, 100 * i64::saturating_from(k)),
            orient,
            num_x: 10,
            num_y: 1,
            step: pt(10, 0),
        });
    }
    for (i, &(at, status, width)) in cells.iter().enumerate() {
        let name = format!("m{i}");
        let m = db
            .library
            .add_macro(Macro::new(
                name.clone(),
                MacroClass::Core(CoreKind::Regular),
                pt(width, 100),
                symmetry,
                Some(site),
            ))
            .unwrap();
        let _ = db.add_cell(name, m, at, Orientation::N, status).unwrap();
    }
    db
}

/// Fences the left half of both rows and assigns `members` to it.
fn fence_left_half(db: &mut NetlistDB, members: &[CellId]) {
    let fence = db.regions.push(Region {
        name: "f".into(),
        rects: vec![Rect::new(pt(0, 0), pt(50, 200))],
        kind: RegionKind::Fence,
        group_order: None,
    });
    for &m in members {
        db.cells[m].region = Some(fence);
    }
}

#[test]
fn a_fence_member_far_outside_its_fence_is_legalized_inside_it() {
    let mut db = design(&[(pt(80, 0), PlacementStatus::Placed, 20)]);
    fence_left_half(&mut db, &[CellId(0)]);

    legalize_zoned(&mut db).unwrap();

    assert!(
        db.cells[CellId(0)].location.x <= Dbu::new(30),
        "{:?}",
        db.cells[CellId(0)].location
    );
    assert_eq!(placement_violations(&db), []);
}

#[test]
fn a_cell_outside_every_fence_is_legalized_out_of_one() {
    let mut db = design(&[(pt(10, 0), PlacementStatus::Placed, 20)]);
    fence_left_half(&mut db, &[]);

    legalize_zoned(&mut db).unwrap();

    assert!(
        db.cells[CellId(0)].location.x >= Dbu::new(50),
        "{:?}",
        db.cells[CellId(0)].location
    );
    assert_eq!(placement_violations(&db), []);
}

#[test]
fn a_full_fence_is_an_error_even_with_room_outside() {
    let mut db = design(&[(pt(0, 0), PlacementStatus::Placed, 50); 5]);
    fence_left_half(&mut db, &[0, 1, 2, 3, 4].map(CellId));

    assert!(matches!(
        legalize_zoned(&mut db),
        Err(LegalizeError::CellDoesNotFit(_))
    ));
}

#[test]
fn overlapping_cells_end_up_legal() {
    let mut db = design(&[(pt(33, 7), PlacementStatus::Placed, 20); 6]);

    legalize_zoned(&mut db).unwrap();

    assert!(
        placement_violations(&db).is_empty(),
        "{:?}",
        placement_violations(&db)
    );
}

#[test]
fn cells_take_their_row_orientation() {
    let mut db = design(&[(pt(40, 95), PlacementStatus::Placed, 20)]);

    legalize_zoned(&mut db).unwrap();

    assert_eq!(db.cells[CellId(0)].location, pt(40, 100));
    assert_eq!(db.cells[CellId(0)].orient, Orientation::FS);
}

#[test]
fn a_cell_already_on_a_site_does_not_move() {
    let mut db = design(&[(pt(30, 0), PlacementStatus::Placed, 20)]);

    legalize_zoned(&mut db).unwrap();

    assert_eq!(db.cells[CellId(0)].location, pt(30, 0));
}

#[test]
fn cells_flow_around_a_fixed_cell() {
    let mut db = design(&[
        (pt(40, 0), PlacementStatus::Fixed, 20),
        (pt(42, 0), PlacementStatus::Placed, 20),
    ]);

    legalize_zoned(&mut db).unwrap();

    assert_eq!(db.cells[CellId(0)].location, pt(40, 0));
    assert_eq!(placement_violations(&db), []);
}

#[test]
fn more_cells_than_sites_is_an_error() {
    let mut db = design(&[(pt(0, 0), PlacementStatus::Placed, 50); 5]);

    assert!(matches!(
        legalize_zoned(&mut db),
        Err(LegalizeError::CellDoesNotFit(_))
    ));
}

#[test]
fn cells_fill_the_row_from_the_right_edge_inward() {
    let mut db = design(&[(pt(95, 0), PlacementStatus::Placed, 30); 3]);

    legalize_zoned(&mut db).unwrap();

    let mut xs: Vec<Dbu> = db
        .cells
        .iter()
        .filter(|c| c.location.y == Dbu::new(0))
        .map(|c| c.location.x)
        .collect();
    xs.sort();
    assert_eq!(xs, [Dbu::new(10), Dbu::new(40), Dbu::new(70)]);
}
