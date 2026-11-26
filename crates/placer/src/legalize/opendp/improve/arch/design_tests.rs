use super::super::{Network, Placer};
use super::*;
use pare_common::db::parser::{def, lef};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

/// The improver's rows of the tiny design with its top row as `row`.
fn arch_with_top_row(row: &str) -> Result<Arch> {
    let text = std::fs::read_to_string(fixture("tiny.def"))
        .unwrap()
        .replace("ROW ROW_2 core 0 4000 N DO 20 BY 1 STEP 200 0 ;", row);
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    let db = def::parse_str(&text, &fixture("tiny.def"), library).unwrap();
    let grid = Placer::new(&db).unwrap();
    let mut network = Network::new(&db, &grid).unwrap();
    Arch::new(&db, &grid, &mut network)
}

#[test]
fn a_one_site_row_without_a_step_spans_its_site() {
    let arch = arch_with_top_row("ROW ROW_2 core 0 4000 N DO 1 BY 1 STEP 0 0 ;").unwrap();

    let top = *arch.rows.iter().last().unwrap();

    assert_eq!((top.left, top.right()), (Dbu::new(0), Dbu::new(200)));
}

#[test]
fn a_row_of_sites_without_a_step_is_an_error() {
    let arch = arch_with_top_row("ROW ROW_2 core 0 4000 N DO 20 BY 1 STEP 0 0 ;");

    assert_eq!(
        arch.map(|_| ()),
        Err(ImprovementError::RowWithoutStep("ROW_2".into()))
    );
}
