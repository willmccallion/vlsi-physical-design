//! The placement model of the shared fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::{BlockageKind, NetlistDB, PlacementBlockage};
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
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
    let inv = db.library.macro_id("INV").unwrap();
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

#[test]
fn a_movable_cell_of_two_signal_pins_keeps_its_size_in_the_density() {
    let db = tiny();
    let model = PlacementModel::from_db(&db, &Zones::new(&db).unwrap());
    let inv = db.cells.iter().position(|c| c.name == "u1").unwrap();

    assert_eq!(model.density_sizes[inv], model.sizes[inv]);
}

#[test]
fn a_fixed_cell_keeps_its_size_in_the_density() {
    let db = tiny();
    let model = PlacementModel::from_db(&db, &Zones::new(&db).unwrap());
    let fixed = db.cells.iter().position(|c| c.name == "u3").unwrap();

    assert_eq!(model.density_sizes[fixed], model.sizes[fixed]);
}

#[test]
fn a_movable_cell_of_three_signal_pins_scales_toward_the_average_pin_density() {
    let mut db = tiny();
    let inv = db
        .library
        .macros
        .iter_mut()
        .find(|m| m.name == "INV")
        .unwrap();
    inv.size.x = inv.size.x * 2;
    let model = PlacementModel::from_db(&db, &Zones::new(&db).unwrap());
    let nand = db.cells.iter().position(|c| c.name == "u4").unwrap();

    let scale = model.density_sizes[nand].x / model.sizes[nand].x;

    assert!(scale > 1.0 && scale <= 1.2, "{scale}");
}

#[test]
fn global_placement_keeps_out_of_a_soft_blockage() {
    let open = tiny();
    let before = PlacementModel::from_db(&open, &Zones::new(&open).unwrap());
    let row = before.rows[0];
    let mut db = tiny();
    let dbu = |microns: f64| db.dbu().to_dbu_rounded(microns);
    let rect = Rect::new(
        Point::new(dbu(row.min.x), dbu(row.min.y)),
        Point::new(dbu(row.max.x), dbu(row.max.y)),
    );
    db.placement_blockages.push(PlacementBlockage {
        rect,
        kind: BlockageKind::Soft,
    });

    let after = PlacementModel::from_db(&db, &Zones::new(&db).unwrap());

    assert_eq!(after.zones[0].rects.len(), before.zones[0].rects.len() - 1);
    assert!(after.free_area() < before.free_area());
}
