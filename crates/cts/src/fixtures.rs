//! Designs over the test fixture cells, for the unit tests.

use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::DbuPerMicron;
use std::path::Path;

/// A design over the fixture cells and the block `RAM`, 200 microns a
/// side at 2000 database units to the micron, with the DEF `sections`
/// (components, nets) given.
pub(crate) fn design(sections: &str) -> NetlistDB {
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(
        &[fixtures.join("clock.lef"), fixtures.join("block.lef")],
        units,
    )
    .unwrap();
    let text = format!(
        "VERSION 5.8 ;\nDESIGN fixture ;\nUNITS DISTANCE MICRONS 2000 ;\n\
         DIEAREA ( 0 0 ) ( 400000 400000 ) ;\n{sections}END DESIGN\n"
    );
    def::parse_str(&text, Path::new("fixture.def"), library).unwrap()
}
