//! Small designs over the fixture library `tiny.lef`, written as DEF, that
//! the crates' property tests generate their cases from.

use crate::db::design::NetlistDB;
use crate::db::parser::{def, lef};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

/// The fixture `name` of this crate's tests.
#[must_use]
pub fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

/// The design of DEF `text` over the fixture library `tiny.lef`.
///
/// # Panics
///
/// When the fixtures or `text` do not parse.
#[must_use]
pub fn parse(text: &str) -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap_or_else(|e| panic!("tiny.def: {e}"));
    let library = lef::parse_files(&[fixture("tiny.lef")], units)
        .unwrap_or_else(|e| panic!("tiny.lef: {e}"));
    def::parse_str(text, Path::new("generated.def"), library)
        .unwrap_or_else(|e| panic!("generated DEF: {e}\n{text}"))
}

/// The DEF of an unplaced ring of `gates` gates.
///
/// Every third gate is an inverter and the rest NAND gates, each NAND's
/// second input fed by the gate halfway round, in a 60 by 60 um core of
/// thirty rows of the fixture's cells.
#[must_use]
pub fn ring_text(gates: usize) -> String {
    ring_def(gates, |_| None, &[])
}

/// [`ring_text`] with gate `g` placed with its lower-left corner at
/// `starts[g]`, and a fixed RAM block at each corner of `blocks`.
#[must_use]
pub fn placed_ring_text(starts: &[(i64, i64)], blocks: &[(i64, i64)]) -> String {
    ring_def(starts.len(), |g| Some(starts[g]), blocks)
}

fn ring_def(
    gates: usize,
    start: impl Fn(usize) -> Option<(i64, i64)>,
    blocks: &[(i64, i64)],
) -> String {
    let mut text = header("ring", 60_000, 60_000);
    for r in 0..30 {
        let orient = if r % 2 == 0 { "N" } else { "FS" };
        let _ = writeln!(
            text,
            "ROW ROW_{r} core 0 {} {orient} DO 300 BY 1 STEP 200 0 ;",
            r * 2000
        );
    }
    let is_inverter = |g: usize| g.is_multiple_of(3);
    let _ = writeln!(text, "COMPONENTS {} ;", gates + blocks.len());
    for g in 0..gates {
        let master = if is_inverter(g) { "INV" } else { "NAND2" };
        match start(g) {
            Some((x, y)) => {
                let _ = writeln!(text, "- g{g} {master} + PLACED ( {x} {y} ) N ;");
            }
            None => {
                let _ = writeln!(text, "- g{g} {master} ;");
            }
        }
    }
    for (b, &(x, y)) in blocks.iter().enumerate() {
        let _ = writeln!(text, "- ram{b} RAM + FIXED ( {x} {y} ) N ;");
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

/// Columns of cell slots a row of [`InverterNetwork`] holds, each slot six
/// sites wide.
pub const SLOT_COLUMNS: i64 = 16;
/// Rows of [`InverterNetwork`]'s core; cells stand on every other one.
pub const ROWS: i64 = 12;
/// The width of a slot, in database units: six sites of 0.2 um.
const SLOT: i64 = 1200;
/// The height of a row, in database units.
const ROW_HEIGHT: i64 = 2000;

/// A roomy design: inverters in some of the slots of every other row, every
/// inverter's output wired to the inputs of others.
// Inverters alone: the fixture's NAND2 has its inputs one M2 track apart, and
// pin access does not yet keep neighbouring pins' vias apart (issue #87).
#[derive(Clone, Debug)]
pub struct InverterNetwork {
    /// Each gate's slot, as a column below [`SLOT_COLUMNS`] and an even row
    /// below [`ROWS`].
    pub gates: Vec<(i64, i64)>,
    /// Each net's driving gate and the gates whose inputs it loads.
    pub nets: Vec<(usize, Vec<usize>)>,
}

impl InverterNetwork {
    /// The DEF of the design over the fixture library, with routing tracks;
    /// a load that would drive itself or whose input an earlier net took is
    /// left out, and so is a net left with no load.
    #[must_use]
    pub fn def_text(&self) -> String {
        let (w, h) = (SLOT_COLUMNS * SLOT, ROWS * ROW_HEIGHT);
        let mut text = header("props", w, h);
        for r in 0..ROWS {
            let orient = if r % 2 == 0 { "N" } else { "FS" };
            let _ = writeln!(
                text,
                "ROW ROW_{r} core 0 {} {orient} DO {} BY 1 STEP 200 0 ;",
                r * ROW_HEIGHT,
                w / 200
            );
        }
        let _ = writeln!(text, "TRACKS Y 100 DO {} STEP 200 LAYER M1 ;", h / 200);
        let _ = writeln!(text, "TRACKS X 100 DO {} STEP 200 LAYER M2 ;", w / 200);
        let _ = writeln!(text, "TRACKS Y 200 DO {} STEP 400 LAYER M3 ;", h / 400);
        let _ = writeln!(text, "COMPONENTS {} ;", self.gates.len());
        for (g, &(c, r)) in self.gates.iter().enumerate() {
            let orient = if r % 2 == 0 { "N" } else { "FS" };
            let _ = writeln!(
                text,
                "- g{g} INV + PLACED ( {} {} ) {orient} ;",
                c * SLOT,
                r * ROW_HEIGHT
            );
        }
        let nets = self.net_lines();
        let _ = writeln!(text, "END COMPONENTS\nNETS {} ;", nets.len());
        for net in &nets {
            let _ = writeln!(text, "{net}");
        }
        text.push_str("END NETS\nEND DESIGN\n");
        text
    }

    fn net_lines(&self) -> Vec<String> {
        let mut free = vec![true; self.gates.len()];
        let mut nets = Vec::new();
        for &(driver, ref loads) in &self.nets {
            let mut pins = vec![format!("( g{driver} Y )")];
            for &load in loads {
                if load != driver && std::mem::take(&mut free[load]) {
                    pins.push(format!("( g{load} A )"));
                }
            }
            if pins.len() > 1 {
                nets.push(format!("- n{driver} {} ;", pins.join(" ")));
            }
        }
        nets
    }
}

/// The opening lines of a DEF of design `name` on a `w` by `h` die.
fn header(name: &str, w: i64, h: i64) -> String {
    format!(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN {name} ;\n\
         UNITS DISTANCE MICRONS 1000 ;\nDIEAREA ( 0 0 ) ( {w} {h} ) ;\n"
    )
}
