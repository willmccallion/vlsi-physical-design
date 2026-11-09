//! Detailed placement of a design whose cells are fenced into a region,
//! pinned down whole.

#![allow(clippy::unwrap_used)]

mod common;

use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::testing;
use pare_placer::legalize::opendp::{DetailedPlacementError, detailed_placement};
use std::fmt::Write as _;
use std::path::Path;

/// The ring with its gates scattered over the core, a fenced region in
/// the middle, and every fifth gate in a group bound to it.
fn fenced_ring() -> NetlistDB {
    ring_fenced_by("( 20000 20000 ) ( 40000 40000 )")
}

/// The ring with its gates scattered over the core and every fifth gate in
/// a group bound to a fence of `rects`, given as DEF points.
fn ring_fenced_by(rects: &str) -> NetlistDB {
    let gates = 240;
    let mut members = String::new();
    for g in (0..gates).step_by(5) {
        let _ = write!(members, " g{g}");
    }
    let text = common::ring_text(gates)
        .replace(
            &format!("COMPONENTS {gates} ;"),
            &format!(
                "REGIONS 1 ;\n- fence {rects} + TYPE FENCE ;\n\
                 END REGIONS\nCOMPONENTS {gates} ;"
            ),
        )
        .replace(
            "END DESIGN",
            &format!("GROUPS 1 ;\n- g{members} + REGION fence ;\nEND GROUPS\nEND DESIGN"),
        );
    let mut db = common::parse(&text);
    for (i, cell) in db.cells.iter_mut().enumerate() {
        let i = i as i64;
        cell.location = Point::new(Dbu(i * 977 % 58_000), Dbu(i * 1931 % 56_000));
        cell.status = PlacementStatus::Placed;
    }
    db
}

#[test]
fn a_fenced_group_is_placed_inside_its_region_as_it_always_is() {
    let mut db = fenced_ring();

    let summary = detailed_placement(&mut db).unwrap();

    let mut text = format!("{summary:?}\n");
    for cell in &db.cells {
        let _ = writeln!(
            text,
            "{} {} {} {:?}",
            cell.name,
            cell.location.x.get(),
            cell.location.y.get(),
            cell.orient
        );
    }
    testing::assert_golden(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden"),
        "fenced_ring_240.txt",
        &text,
    );
}

#[test]
fn a_group_whose_region_misses_the_core_is_an_error() {
    let mut db = ring_fenced_by("( 70000 70000 ) ( 80000 80000 )");

    let result = detailed_placement(&mut db);

    assert_eq!(
        result,
        Err(DetailedPlacementError::RegionOutsideCore("fence".into()))
    );
}
