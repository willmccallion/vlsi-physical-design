//! Design, setup and hold repair of a small generated design over the
//! clock test cells, pinned down whole.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::{def, lef, liberty, sdc};
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::testing;
use pare_common::units::Seconds;
use pare_resizer::parasitics::WireRc;
use pare_resizer::{
    ClockNetwork, Constants, Context, HoldOptions, Link, Namer, Netlist, Repairer, Resizer,
    SessionInputs, SetupOptions, repair_hold, repair_setup,
};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../cts/tests/fixtures")
        .join(name)
}

/// Where column `c` and row `r` of the site grid put a cell, as DEF text.
fn at(c: usize, r: usize) -> String {
    format!("( {} {} ) N", 2000 + c * 380, 2000 + r * 2800)
}

/// A die of 140 rows with three kinds of work for repair: eight flop to
/// flop paths through six gates spread across the die, a buffer fanning
/// out to 24 flops all over it, and four pairs of flops side by side.
fn design_text() -> String {
    let mut cells = Vec::new();
    let mut nets = Vec::new();
    for s in 0..8 {
        let row = 10 + s * 16;
        cells.push(format!("- ffa{s} DFF_X1 + PLACED {} ;", at(10, row)));
        let mut previous = format!("( ffa{s} Q )");
        for k in 0..6 {
            let master = if k % 2 == 0 { "BUF_X1" } else { "INV_X1" };
            let out = if k % 2 == 0 { "Z" } else { "ZN" };
            cells.push(format!(
                "- g{s}_{k} {master} + PLACED {} ;",
                at(60 + 140 * k, row)
            ));
            nets.push(format!("- p{s}_{k} {previous} ( g{s}_{k} A ) ;"));
            previous = format!("( g{s}_{k} {out} )");
        }
        cells.push(format!("- ffb{s} DFF_X1 + PLACED {} ;", at(950, row)));
        nets.push(format!("- p{s}_6 {previous} ( ffb{s} D ) ;"));
    }
    cells.push(format!("- fan BUF_X1 + PLACED {} ;", at(500, 70)));
    let mut fanout = String::from("- fo ( fan Z )");
    for i in 0..24 {
        cells.push(format!(
            "- fo{i} DFF_X1 + PLACED {} ;",
            at(40 * i, (i * 37) % 130)
        ));
        let _ = write!(fanout, " ( fo{i} D )");
    }
    nets.push(fanout + " ;");
    nets.push("- in0 ( PIN in0 ) ( fan A ) ;".to_string());
    for h in 0..4 {
        let row = 4 + h * 32;
        cells.push(format!("- ha{h} DFF_X1 + PLACED {} ;", at(500, row)));
        cells.push(format!("- hb{h} DFF_X1 + PLACED {} ;", at(512, row)));
        nets.push(format!("- h{h} ( ha{h} Q ) ( hb{h} D ) ;"));
    }
    let mut clock = String::from("- clk ( PIN clk )");
    for cell in &cells {
        let name = cell.split_whitespace().nth(1).unwrap();
        if name.starts_with("ff") || name.starts_with("fo") || name.starts_with('h') {
            let _ = write!(clock, " ( {name} CK )");
        }
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

fn load() -> (NetlistDB, Liberty) {
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("clock.lef")], units).unwrap();
    let db = def::parse_str(&design_text(), Path::new("flow.def"), library).unwrap();
    let lib = liberty::parse_file(&fixture("clock.lib")).unwrap();
    (db, lib)
}

/// Every cell with its master and place, as text.
fn cells_text(db: &NetlistDB) -> String {
    let mut text = String::new();
    for cell in db.cells.iter().filter(|c| !c.retired) {
        let _ = writeln!(
            text,
            "{} {} {} {}",
            cell.name,
            db.library.macros[cell.macro_id.index()].name,
            cell.location.x.get(),
            cell.location.y.get()
        );
    }
    text
}

/// Design, setup and hold repair of the design under a clock of
/// `period_ns`, as text: what each repair did and every cell after.
fn repaired(period_ns: f64) -> String {
    let (mut db, lib) = load();
    let constraints = sdc::parse_str(
        &format!(
            "create_clock -name core -period {period_ns} [get_ports clk]\n\
             set_input_delay 0.1 -clock core [get_ports in0]\n"
        ),
        Path::new("flow.sdc"),
        &db,
        &lib,
    )
    .unwrap();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let buffers = link.find_buffers(&db).unwrap();
    let (constants, clocks) = {
        let netlist = Netlist::new(&db, &link);
        let constants = Constants::find(&netlist);
        let clocks = ClockNetwork::from_sdc(&netlist, &constraints, &constants);
        (constants, clocks)
    };
    let rc = WireRc::per_micron(3.5, 0.1, db.library.routing_layer_id("metal2").unwrap());
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

    let design = Repairer::new(&mut resizer).repair_design().unwrap();
    let setup = repair_setup(&mut resizer, SetupOptions::default(), &mut |_| {}).unwrap();
    let hold = repair_hold(
        &mut resizer,
        &HoldOptions {
            hold_margin: Seconds::new(0.12e-9),
            ..HoldOptions::default()
        },
    )
    .unwrap();
    let _ = resizer.finish().unwrap();

    format!("{design:#?}\n{setup:#?}\n{hold:#?}\n{}", cells_text(&db))
}

fn assert_golden(name: &str, actual: &str) {
    testing::assert_golden(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden"),
        name,
        actual,
    );
}

#[test]
fn repairing_design_setup_and_hold_gives_what_it_always_has() {
    assert_golden("flow.txt", &repaired(0.25));
}

#[test]
fn setup_repair_short_of_its_target_gives_what_it_always_has() {
    assert_golden("flow_unclosed.txt", &repaired(0.18));
}
