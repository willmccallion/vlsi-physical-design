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
use pare_common::db::sdc::Sdc;
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
    design_with_copies(text, &[])
}

/// The design a DEF `text` describes over the clock test cells and, for
/// each `(master, name)` of `copies`, a copy of `master` called `name`.
pub(crate) fn design_with_copies(text: &str, copies: &[(&str, &str)]) -> NetlistDB {
    let units = DbuPerMicron::new(2000).unwrap();
    let mut library = lef::parse_files(&[fixture("clock.lef")], units).unwrap();
    for &(master, name) in copies {
        let mut copy = library
            .cell_macro(library.macro_id(master).unwrap())
            .clone();
        copy.name = name.to_string();
        let _ = library.add_macro(copy).unwrap();
    }
    def::parse_str(text, Path::new("test.def"), library).unwrap()
}

/// The DEF text of design `name` made of `sections`, on a die 400 microns
/// square.
pub(crate) fn def_text(name: &str, sections: &DefSections<'_>) -> String {
    let count = |section: &str| section.lines().filter(|l| l.starts_with('-')).count();
    let pins: Vec<String> = sections
        .inputs
        .iter()
        .map(|p| (p, "INPUT"))
        .chain(sections.outputs.iter().map(|p| (p, "OUTPUT")))
        .map(|(p, direction)| format!("- {p} + NET {p} + DIRECTION {direction} ;\n"))
        .collect();
    format!(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN {name} ;\n\
         UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 800000 800000 ) ;\n\
         COMPONENTS {} ;\n{}END COMPONENTS\nPINS {} ;\n{}END PINS\n\
         NETS {} ;\n{}END NETS\nEND DESIGN\n",
        count(sections.components),
        sections.components,
        pins.len(),
        pins.concat(),
        count(sections.nets),
        sections.nets,
    )
}

/// The sections of a design [`def_text`] writes.
pub(crate) struct DefSections<'s> {
    /// The `COMPONENTS` lines.
    pub components: &'s str,
    /// The `NETS` lines.
    pub nets: &'s str,
    /// The input pins, each on a net of its name.
    pub inputs: &'s [&'s str],
    /// The output pins, each on a net of its name.
    pub outputs: &'s [&'s str],
}

/// The text of the clock test cells' library.
pub(crate) fn clock_cells_source() -> String {
    std::fs::read_to_string(fixture("clock.lib")).unwrap()
}

/// The clock test cells' library, from `source`.
pub(crate) fn cells(source: &str) -> Liberty {
    liberty::parse_str(source, Path::new("clock.lib")).unwrap()
}

/// The clock test cells' library with the Liberty cell groups `extra`.
pub(crate) fn cells_with(extra: &str) -> Liberty {
    let source = clock_cells_source();
    let end = source.rfind('}').unwrap();
    cells(&format!("{}{extra}\n}}\n", &source[..end]))
}

/// The pin `cell/pin` of `db`.
pub(crate) fn pin(db: &NetlistDB, name: &str) -> PinId {
    let (cell, _) = name.split_once('/').unwrap();
    db.cells[db.cell_id(cell).unwrap()]
        .pins
        .iter()
        .copied()
        .find(|&p| db.pin_name(p) == name)
        .unwrap()
}

/// Wires of no resistance or capacitance.
const IDEAL_WIRES: WireRc = WireRc {
    resistance: OhmsPerMeter::ZERO,
    capacitance: FaradsPerMeter::ZERO,
    layer: LayerId(1),
};

/// Wires of 3.5 ohms and 0.1 fF a micron on metal2.
pub(crate) fn metal2_wires(db: &NetlistDB) -> WireRc {
    WireRc::per_micron(3.5, 0.1, db.library.routing_layer_id("metal2").unwrap())
}

/// Runs `test` on a timed session over `db` with the cells of `lib`, under
/// `constraints`, with wires of no resistance or capacitance.
pub(crate) fn with_timed_session(
    db: &mut NetlistDB,
    lib: &Liberty,
    constraints: &str,
    test: impl FnOnce(&mut Resizer<'_, Sta>),
) {
    with_session_inputs(db, lib, constraints, IDEAL_WIRES, |inputs, sdc| {
        test(&mut Resizer::timed(inputs, sdc).unwrap());
    });
}

/// What `start` makes of the inputs of a session over `db` with the cells
/// of `lib` and wires of `rc`, given `constraints` parsed.
pub(crate) fn with_session_inputs<R>(
    db: &mut NetlistDB,
    lib: &Liberty,
    constraints: &str,
    rc: WireRc,
    start: impl FnOnce(SessionInputs<'_>, &Sdc) -> R,
) -> R {
    let constraints = sdc::parse_str(constraints, Path::new("test.sdc"), db, lib).unwrap();
    let link = Link::new(db, lib, &[]).unwrap();
    let buffers = link.find_buffers(db).unwrap();
    let (constants, clocks) = {
        let netlist = Netlist::new(db, &link);
        let constants = Constants::find(&netlist);
        let clocks = ClockNetwork::from_sdc(&netlist, &constraints, &constants);
        (constants, clocks)
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
    start(inputs, &constraints)
}
