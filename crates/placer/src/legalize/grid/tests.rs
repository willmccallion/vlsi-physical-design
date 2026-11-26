use super::*;
use pare_common::db::parser::{def, lef};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

/// The tiny design's three rows, 2 um high, raised 2 um off the die's
/// bottom, so the core runs from 2 to 8 um.
fn raised_rows() -> RowGrid {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    let mut db = def::parse(&fixture("tiny.def"), library).unwrap();
    for row in &mut db.rows {
        row.origin.y += Dbu::new(2000);
    }
    RowGrid::new(&db, &core_rows(&db)).unwrap()
}

#[test]
fn past_the_last_row_edge_lies_the_core_top_relative_to_the_core() {
    let rows = raised_rows();

    let past_last = rows.row_y(rows.row_count() + 1);

    assert_eq!(past_last, Dbu::new(6000));
}
