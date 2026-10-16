//! The placement model of the shared fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_placer::model::{CellKind, PlacementModel};
use pare_placer::zones::Zones;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

fn tiny() -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse(&fixture("tiny.def"), library).unwrap()
}

fn kind_of(db: &NetlistDB, cell: &str) -> CellKind {
    let model = PlacementModel::from_db(db, &Zones::new(db).unwrap());
    model.kinds[db.cells.iter().position(|c| c.name == cell).unwrap()]
}

fn set_inv_height(db: &mut NetlistDB, height: Dbu) {
    let inv = db
        .library
        .macros
        .iter()
        .position(|m| m.name == "INV")
        .unwrap();
    db.library.macros[inv].size.y = height;
}

#[test]
fn a_block_is_a_macro_and_a_row_high_cell_is_standard() {
    let db = tiny();

    assert_eq!(kind_of(&db, "ram0"), CellKind::Macro);
    assert_eq!(kind_of(&db, "u1"), CellKind::Standard);
}

#[test]
fn a_core_cell_more_than_six_rows_high_is_a_macro() {
    let mut db = tiny();
    set_inv_height(&mut db, Dbu(12_200));

    assert_eq!(kind_of(&db, "u1"), CellKind::Macro);
}

#[test]
fn a_core_cell_six_rows_high_is_standard() {
    let mut db = tiny();
    set_inv_height(&mut db, Dbu(12_000));

    assert_eq!(kind_of(&db, "u1"), CellKind::Standard);
}
