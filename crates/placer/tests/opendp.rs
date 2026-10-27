//! Detailed placement on the tiny fixture design, against the positions
//! `OpenROAD`'s `detailed_placement`, `improve_placement -max_displacement
//! {5 1}` and `optimize_mirroring` give the same inputs: three rows of 0.2
//! um sites, 2 um high, a fixed NAND2 at (2, 0) and a 2 by 4 um RAM fixed at
//! (4, 0); the library has no one-site cell, so no one-site gaps are left.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_placer::legalize::opendp::improve::improve_placement;
use pare_placer::legalize::opendp::mirror::optimize_mirroring;
use pare_placer::legalize::opendp::{DetailedPlacementError, detailed_placement};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

const MOVABLE: [&str; 6] = ["u1", "u2", "u4", "u5", "u6", "u7"];

/// The tiny design with its movable cells at `places`.
fn tiny_with(places: &[(i64, i64)]) -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    let mut db = def::parse(&fixture("tiny.def"), library).unwrap();
    for (name, &(x, y)) in MOVABLE.iter().zip(places) {
        let id = db.cell_id(name).unwrap();
        let c = &mut db.cells[id.index()];
        c.location = Point::new(Dbu(x), Dbu(y));
        c.orient = Orientation::N;
        c.status = PlacementStatus::Placed;
    }
    db
}

fn places(db: &NetlistDB) -> Vec<(i64, i64, Orientation)> {
    MOVABLE
        .iter()
        .map(|name| {
            let c = &db.cells[db.cell_id(name).unwrap().index()];
            (c.location.x.get(), c.location.y.get(), c.orient)
        })
        .collect()
}

#[test]
fn cells_on_one_spot_spread_along_the_nearest_row() {
    let mut db = tiny_with(&[(1000, 1000); 6]);

    let summary = detailed_placement(&mut db).unwrap();

    assert_eq!(summary.cells, 6);
    assert_eq!(
        places(&db),
        vec![
            (1600, 2000, Orientation::FS),
            (0, 2000, Orientation::FS),
            (1000, 2000, Orientation::FS),
            (2000, 2000, Orientation::FS),
            (400, 2000, Orientation::FS),
            (2400, 2000, Orientation::FS),
        ]
    );
}

#[test]
fn cells_on_a_macro_move_off_it() {
    let mut db = tiny_with(&[
        (4500, 2000),
        (4500, 2100),
        (4500, 2200),
        (4500, 2300),
        (4500, 2400),
        (4500, 2500),
    ]);

    let _ = detailed_placement(&mut db).unwrap();

    assert_eq!(
        places(&db),
        vec![
            (1600, 4000, Orientation::N),
            (3600, 2000, Orientation::FS),
            (2800, 4000, Orientation::N),
            (2000, 4000, Orientation::N),
            (3400, 4000, Orientation::N),
            (2400, 4000, Orientation::N),
        ]
    );
}

#[test]
fn cells_past_the_core_come_back_inside() {
    let mut db = tiny_with(&[(5900, 5900); 6]);

    let _ = detailed_placement(&mut db).unwrap();

    assert_eq!(
        places(&db),
        vec![
            (2400, 4000, Orientation::N),
            (2000, 4000, Orientation::N),
            (3400, 4000, Orientation::N),
            (3600, 2000, Orientation::FS),
            (2800, 4000, Orientation::N),
            (1600, 4000, Orientation::N),
        ]
    );
}

#[test]
fn a_design_too_full_for_its_rows_fails_naming_the_cells_left_out() {
    let mut db = tiny_with(&[(1000, 1000); 6]);
    let inv = db.cells[db.cell_id("u1").unwrap().index()].macro_id;
    for i in 0..30 {
        let _ = db
            .add_cell(
                format!("extra{i}"),
                inv,
                Point::new(Dbu(1000), Dbu(1000)),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
    }

    let result = detailed_placement(&mut db);

    assert!(matches!(result, Err(DetailedPlacementError::Failed(names)) if !names.is_empty()));
}

/// The places `OpenROAD` starts improvement from in the tests below.
const SCATTERED: [(i64, i64); 6] = [
    (0, 4000),
    (3000, 0),
    (600, 2000),
    (2600, 4000),
    (200, 2000),
    (3400, 4000),
];

fn legalized(start: &[(i64, i64)]) -> NetlistDB {
    let mut db = tiny_with(start);
    let _ = detailed_placement(&mut db).unwrap();
    db
}

#[test]
fn improvement_pulls_cells_to_their_nets_as_openroad_does() {
    let mut db = legalized(&[(1000, 1000); 6]);

    let improvement = improve_placement(&mut db, (5, 1)).unwrap();

    assert!(improvement.hpwl_after < improvement.hpwl_before);
    assert_eq!(
        places(&db),
        vec![
            (1600, 2000, Orientation::FS),
            (2600, 0, Orientation::N),
            (3400, 0, Orientation::N),
            (3600, 4000, Orientation::N),
            (2400, 4000, Orientation::N),
            (3000, 0, Orientation::N),
        ]
    );
}

#[test]
fn improvement_from_scattered_cells_matches_openroad() {
    let mut db = legalized(&SCATTERED);

    let _ = improve_placement(&mut db, (5, 1)).unwrap();

    assert_eq!(
        places(&db),
        vec![
            (1200, 2000, Orientation::FS),
            (1600, 0, Orientation::N),
            (2600, 0, Orientation::N),
            (2400, 2000, Orientation::FS),
            (2400, 4000, Orientation::N),
            (3400, 4000, Orientation::N),
        ]
    );
}

#[test]
fn mirroring_turns_the_cells_openroad_turns() {
    let mut db = legalized(&SCATTERED);
    let _ = improve_placement(&mut db, (5, 1)).unwrap();

    let mirrored = optimize_mirroring(&mut db);

    assert_eq!(mirrored, 3);
    let orients: Vec<Orientation> = places(&db).iter().map(|p| p.2).collect();
    assert_eq!(
        orients,
        vec![
            Orientation::FS,
            Orientation::N,
            Orientation::N,
            Orientation::S,
            Orientation::FN,
            Orientation::FN,
        ]
    );
}
