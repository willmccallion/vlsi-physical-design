//! The designs the placer's integration tests share.

use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{def, lef};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

/// The DEF of gates in a ring, every third an inverter and the rest NAND
/// gates, each NAND's second input fed by the gate halfway round, unplaced
/// in a 60 by 60 um core of thirty rows of the fixture's cells.
pub fn ring_text(gates: usize) -> String {
    let mut text = String::from(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN ring ;\n\
         UNITS DISTANCE MICRONS 1000 ;\nDIEAREA ( 0 0 ) ( 60000 60000 ) ;\n",
    );
    for r in 0..30 {
        let orient = if r % 2 == 0 { "N" } else { "FS" };
        let _ = writeln!(
            text,
            "ROW ROW_{r} core 0 {} {orient} DO 300 BY 1 STEP 200 0 ;",
            r * 2000
        );
    }
    let is_inverter = |g: usize| g.is_multiple_of(3);
    let _ = writeln!(text, "COMPONENTS {gates} ;");
    for g in 0..gates {
        let master = if is_inverter(g) { "INV" } else { "NAND2" };
        let _ = writeln!(text, "- g{g} {master} ;");
    }
    let _ = writeln!(text, "END COMPONENTS\nNETS {gates} ;");
    for g in 0..gates {
        let _ = write!(text, "- n{g} ( g{g} Y ) ( g{} A )", (g + 1) % gates);
        let fed = (g + gates / 2) % gates;
        if !is_inverter(fed) {
            let _ = write!(text, " ( g{fed} B )");
        }
        text.push_str(" ;\n");
    }
    text.push_str("END NETS\nEND DESIGN\n");
    text
}

/// The design of [`ring_text`].
#[allow(dead_code)] // not every test file places the plain ring
pub fn ring(gates: usize) -> NetlistDB {
    parse(&ring_text(gates))
}

/// A design over the fixture library from DEF `text`.
pub fn parse(text: &str) -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse_str(text, Path::new("ring.def"), library).unwrap()
}
