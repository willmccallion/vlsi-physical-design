//! Single-cell legalization on the tiny fixture design: three rows of 0.2
//! um sites, 2 um high, with a 2 by 4 um RAM fixed at (4, 0).

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_placer::legalize::site::SiteGrid;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

fn tiny_design() -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse(&fixture("tiny.def"), library).unwrap()
}

/// Puts the inverter `u1` at `(x, y)` and returns where legalization
/// moves it.
fn legalized(x: i64, y: i64) -> Option<Point<Dbu>> {
    let mut db = tiny_design();
    let u1 = db.cell_id("u1").unwrap();
    db.cells[u1.index()].location = Point::new(Dbu(x), Dbu(y));
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
