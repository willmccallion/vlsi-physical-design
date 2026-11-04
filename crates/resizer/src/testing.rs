//! Timed sessions over small designs, for tests.

use crate::library::Link;
use crate::netlist::{ClockNetwork, Constants, Namer, Netlist};
use crate::parasitics::WireRc;
use crate::resizer::{Context, Resizer, SessionInputs};
use crate::sta::Sta;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{LayerId, PinId};
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::{def, lef, liberty, sdc};
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../cts/tests/fixtures")
        .join(name)
}

/// The design a DEF `text` describes over the clock test cells.
pub(crate) fn design(text: &str) -> NetlistDB {
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("clock.lef")], units).unwrap();
    def::parse_str(text, Path::new("test.def"), library).unwrap()
}

/// The text of the clock test cells' library.
pub(crate) fn clock_cells_source() -> String {
    std::fs::read_to_string(fixture("clock.lib")).unwrap()
}

/// The clock test cells' library, from `source`.
pub(crate) fn cells(source: &str) -> Liberty {
    liberty::parse_str(source, Path::new("clock.lib")).unwrap()
}

/// The pin `cell/pin` of `db`.
pub(crate) fn pin(db: &NetlistDB, name: &str) -> PinId {
    let (cell, _) = name.split_once('/').unwrap();
    db.cells[db.cell_id(cell).unwrap().index()]
        .pins
        .iter()
        .copied()
        .find(|&p| db.pin_name(p) == name)
        .unwrap()
}

/// Runs `test` on a timed session over `db` with the cells of `lib`, under
/// `constraints`, with wires of no resistance or capacitance.
pub(crate) fn with_timed_session(
    db: &mut NetlistDB,
    lib: &Liberty,
    constraints: &str,
    test: impl FnOnce(&mut Resizer<'_, Sta>),
) {
    let constraints = sdc::parse_str(constraints, Path::new("test.sdc"), db, lib).unwrap();
    let link = Link::new(db, lib, &[]).unwrap();
    let buffers = link.find_buffers(db).unwrap();
    let (constants, clocks) = {
        let netlist = Netlist::new(db, &link);
        let constants = Constants::find(&netlist);
        let clocks = ClockNetwork::from_sdc(&netlist, &constraints, &constants);
        (constants, clocks)
    };
    let rc = WireRc {
        resistance: OhmsPerMeter::ZERO,
        capacitance: FaradsPerMeter::ZERO,
        layer: LayerId(1),
    };
    let context = Context {
        link: &link,
        clocks: &clocks,
        constants: &constants,
        clock_rc: None,
    };
    let mut namer = Namer::new();
    let inputs = SessionInputs {
        db,
        context,
        rc,
        buffers,
        namer: &mut namer,
    };
    let mut resizer = Resizer::timed(inputs, &constraints).unwrap();
    test(&mut resizer);
}
