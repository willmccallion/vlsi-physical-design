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

/// The characterization of the fixture cells' `CLKBUF_X3` on clock wire
/// of 1.5 ohm and 0.11 fF per micron, and the buffer itself.
pub(crate) fn clock_tech() -> (
    crate::techchar::TechChar,
    pare_common::db::liberty::BufferCell,
) {
    use crate::techchar::{CharSetup, TechChar};
    use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
    use pare_resizer::{Link, WireRc};
    let db = design("");
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/clock.lib");
    let lib = pare_common::db::parser::liberty::parse_file(&path).unwrap();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let buffer = lib.buffer_cell(lib.cell_id("CLKBUF_X3").unwrap()).unwrap();
    let tech = TechChar::new(&CharSetup {
        link: &link,
        db: &db,
        masters: &[buffer],
        sink_buffer: buffer,
        clock_rc: WireRc {
            resistance: OhmsPerMeter::from_ohms_per_micron(1.5),
            capacitance: FaradsPerMeter::from_femtofarads_per_micron(0.11),
            layer: db.library.routing_layer_id("metal3").unwrap(),
        },
    })
    .unwrap();
    (tech, buffer)
}
