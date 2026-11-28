//! Designs over the clock test cells and timed repair sessions on them,
//! shared by the resizer's integration tests.

use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::{def, lef, liberty, sdc};
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
use pare_resizer::{
    ClockNetwork, Constants, Context, Link, Namer, Netlist, Resizer, SessionInputs, Sta, WireRc,
};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

/// The cells and nets of a design, as DEF component and net lines.
pub struct Work {
    pub cells: Vec<String>,
    pub nets: Vec<String>,
}

pub fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../cts/tests/fixtures")
        .join(name)
}

/// Where column `c` and row `r` of the site grid put a cell, as DEF text.
pub fn at(c: usize, r: usize) -> String {
    format!("( {} {} ) N", 2000 + c * 380, 2000 + r * 2800)
}

/// A die of 140 rows holding `work`, every flop clocked from the port
/// `clk`.
fn design_text(work: Work) -> String {
    let Work { cells, mut nets } = work;
    let mut clock = String::from("- clk ( PIN clk )");
    for cell in cells.iter().filter(|c| c.contains(" DFF_")) {
        let name = cell.split_whitespace().nth(1).unwrap();
        let _ = write!(clock, " ( {name} CK )");
    }
    nets.push(clock + " + USE CLOCK ;");

    let mut text = String::from(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN flow ;\n\
         UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 400000 400000 ) ;\n",
    );
    for r in 0..140 {
        let orient = if r % 2 == 0 { "N" } else { "FS" };
        let _ = writeln!(
            text,
            "ROW ROW_{r} core 2000 {} {orient} DO 1000 BY 1 STEP 380 0 ;",
            2000 + r * 2800
        );
    }
    let _ = writeln!(text, "COMPONENTS {} ;", cells.len());
    for cell in &cells {
        let _ = writeln!(text, "{cell}");
    }
    text.push_str(
        "END COMPONENTS\nPINS 2 ;\n\
         - clk + NET clk + DIRECTION INPUT + USE CLOCK\n  \
         + PORT + LAYER metal2 ( -35 0 ) ( 35 70 )\n  + PLACED ( 200000 0 ) N ;\n\
         - in0 + NET in0 + DIRECTION INPUT + USE SIGNAL\n  \
         + PORT + LAYER metal2 ( -35 0 ) ( 35 70 )\n  + PLACED ( 0 200000 ) N ;\n\
         END PINS\n",
    );
    let _ = writeln!(text, "NETS {} ;", nets.len());
    for net in &nets {
        let _ = writeln!(text, "{net}");
    }
    text.push_str("END NETS\nEND DESIGN\n");
    text
}

/// The clock test cells' library.
pub fn clock_cells() -> Liberty {
    liberty::parse_file(&fixture("clock.lib")).unwrap()
}

/// What `repair` returns from a timed session over the design of `work`
/// under the SDC `constraints`, with `lib` as its library, and the design
/// once the session ends.
pub fn repaired_in_session<R>(
    work: Work,
    constraints: &str,
    lib: &Liberty,
    repair: impl FnOnce(&mut Resizer<'_, Sta>) -> R,
) -> (R, NetlistDB) {
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("clock.lef")], units).unwrap();
    let mut db = def::parse_str(&design_text(work), Path::new("flow.def"), library).unwrap();
    let constraints = sdc::parse_str(constraints, Path::new("flow.sdc"), &db, lib).unwrap();
    let link = Link::new(&db, lib, &[]).unwrap();
    let buffers = link.find_buffers(&db).unwrap();
    let (constants, clocks) = {
        let netlist = Netlist::new(&db, &link);
        let constants = Constants::find(&netlist);
        let clocks = ClockNetwork::from_sdc(&netlist, &constraints, &constants);
        (constants, clocks)
    };
    let rc = WireRc {
        resistance: OhmsPerMeter::from_ohms_per_micron(3.5),
        capacitance: FaradsPerMeter::from_femtofarads_per_micron(0.1),
        layer: db.library.routing_layer_id("metal2").unwrap(),
    };
    let context = Context {
        link: &link,
        clocks: &clocks,
        constants: &constants,
        clock_rc: None,
    };
    let mut namer = Namer::new();
    let inputs = SessionInputs {
        db: &mut db,
        context,
        rc,
        buffers,
        namer: &mut namer,
    };
    let mut resizer = Resizer::timed(inputs, &constraints).unwrap();
    let reported = repair(&mut resizer);
    let _ = resizer.finish().unwrap();
    (reported, db)
}
