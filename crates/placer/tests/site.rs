//! Single-cell legalization on the tiny fixture design: three rows of 0.2
//! um sites, 2 um high, with a 2 by 4 um RAM fixed at (4, 0), against
//! where `OpenROAD`'s `legalCellPos` puts each cell.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::db::library::{Site, SiteClass};
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_placer::legalize::opendp::DetailedPlacementError;
use pare_placer::legalize::site::SiteGrid;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

fn tiny_design() -> NetlistDB {
    tiny_edited(|text| text)
}

/// The tiny design from its DEF as `edit` rewrites it.
fn tiny_edited(edit: impl Fn(String) -> String) -> NetlistDB {
    let text = edit(std::fs::read_to_string(fixture("tiny.def")).unwrap());
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse_str(&text, &fixture("tiny.def"), library).unwrap()
}

/// Puts the inverter `u1` of `db` at `(x, y)` in `orient` and returns
/// where legalization moves it.
fn legalized_in(mut db: NetlistDB, (x, y): (i64, i64), orient: Orientation) -> Option<Point<Dbu>> {
    let u1 = db.cell_id("u1").unwrap();
    db.cells[u1].location = Point::new(Dbu(x), Dbu(y));
    db.cells[u1].orient = orient;
    let grid = SiteGrid::new(&db).unwrap();
    grid.legal_position(&db, u1)
}

/// Puts the inverter `u1` at `(x, y)` and returns where legalization
/// moves it.
fn legalized(x: i64, y: i64) -> Option<Point<Dbu>> {
    let mut db = tiny_design();
    let u1 = db.cell_id("u1").unwrap();
    db.cells[u1].location = Point::new(Dbu(x), Dbu(y));
    let grid = SiteGrid::new(&db).unwrap();
    grid.legal_position(&db, u1)
}

#[test]
fn a_cell_on_a_site_and_row_stays() {
    assert_eq!(legalized(200, 0), None);
}

#[test]
fn a_cell_off_the_grid_goes_to_the_nearest_site_and_row() {
    assert_eq!(legalized(250, 900), Some(Point::new(Dbu(200), Dbu(0))));
}

#[test]
fn a_cell_on_a_macro_goes_past_its_nearest_edge() {
    assert_eq!(
        legalized(4500, 2000),
        Some(Point::new(Dbu(4600), Dbu(4000)))
    );
}

#[test]
fn a_cell_past_the_core_is_brought_inside() {
    assert_eq!(
        legalized(5900, 5500),
        Some(Point::new(Dbu(5600), Dbu(4000)))
    );
}

#[test]
fn a_cell_near_the_top_of_a_raised_core_stays_on_its_top_row() {
    let mut db = tiny_design();
    for row in &mut db.rows {
        row.origin.y += Dbu(2000);
    }
    let u1 = db.cell_id("u1").unwrap();
    db.cells[u1].location = Point::new(Dbu(1000), Dbu(7900));
    let grid = SiteGrid::new(&db).unwrap();

    let legal = grid.legal_position(&db, u1);

    assert_eq!(legal, Some(Point::new(Dbu(1000), Dbu(6000))));
}

#[test]
fn a_cell_on_the_top_row_of_a_macro_off_the_rows_moves_past_its_edge() {
    let mut db = tiny_design();
    let ram = db.cell_id("ram0").unwrap();
    db.cells[ram].location = Point::new(Dbu(4000), Dbu(1000));

    let legal = legalized_in(db, (4400, 4000), Orientation::N);

    assert_eq!(legal, Some(Point::new(Dbu(3600), Dbu(4000))));
}

#[test]
fn a_cell_snaps_to_the_rows_a_row_statement_repeats() {
    let db = tiny_edited(|text| {
        text.replace(
            "ROW ROW_0 core 0 0 N DO 30 BY 1 STEP 200 0 ;",
            "ROW ROW_0 core 0 0 N DO 30 BY 3 STEP 200 2000 ;",
        )
        .replace("ROW ROW_1 core 0 2000 FS DO 30 BY 1 STEP 200 0 ;", "")
        .replace("ROW ROW_2 core 0 4000 N DO 20 BY 1 STEP 200 0 ;", "")
    });

    let legal = legalized_in(db, (1000, 4100), Orientation::N);

    assert_eq!(legal, Some(Point::new(Dbu(1000), Dbu(4000))));
}

#[test]
fn a_turned_cell_is_kept_inside_the_core_by_its_turned_size() {
    let legal = legalized_in(tiny_design(), (5000, 4600), Orientation::W);

    assert_eq!(legal, Some(Point::new(Dbu(4000), Dbu(4000))));
}

#[test]
fn rows_of_sites_of_different_widths_are_an_error() {
    let mut db = tiny_design();
    let wide = db.library.sites.push(Site {
        name: "wide".into(),
        class: SiteClass::Core,
        size: Point::new(Dbu(400), Dbu(2000)),
        symmetry: db.library.sites[db.rows[0].site].symmetry,
    });
    db.rows[2].site = wide;

    let grid = SiteGrid::new(&db);

    assert_eq!(
        grid.map(|_| ()),
        Err(DetailedPlacementError::SiteWidths(Dbu(400), Dbu(200)))
    );
}
