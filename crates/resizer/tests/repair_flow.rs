//! Design, setup and hold repair of a small generated design over the
//! clock test cells, pinned down whole.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::{def, lef, liberty, sdc};
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::testing;
use pare_common::units::Seconds;
use pare_resizer::WireRc;
use pare_resizer::{
    ClockNetwork, Constants, Context, HoldOptions, Link, Namer, Netlist, Repairer, Resizer,
    SessionInputs, SetupOptions, Sta, repair_hold, repair_setup,
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

/// Three kinds of work for repair, as DEF cells and nets: eight flop to
/// flop paths through six gates spread across the die, a buffer fanning
/// out to 24 flops all over it, and four pairs of flops side by side.
fn timing_work() -> (Vec<String>, Vec<String>) {
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
    (cells, nets)
}

/// Nets design repair splits, as DEF cells and nets: a buffer driving 180
/// flops, past the load limit of any size of it, four flops each driving one flop across the
/// die, and one driving three flops at the die's far corners.
fn long_and_loaded_nets() -> (Vec<String>, Vec<String>) {
    let mut cells = Vec::new();
    let mut nets = Vec::new();
    cells.push(format!("- fan BUF_X1 + PLACED {} ;", at(500, 70)));
    let mut fanout = String::from("- fo ( fan Z )");
    for i in 0..180 {
        cells.push(format!(
            "- fo{i} DFF_X1 + PLACED {} ;",
            at(450 + (i % 10) * 10, 60 + i / 10 * 2)
        ));
        let _ = write!(fanout, " ( fo{i} D )");
    }
    nets.push(fanout + " ;");
    nets.push("- in0 ( PIN in0 ) ( fan A ) ;".to_string());
    for k in 0..4 {
        let row = 20 + k * 30;
        cells.push(format!("- ffl{k} DFF_X1 + PLACED {} ;", at(5, row)));
        cells.push(format!("- ffr{k} DFF_X1 + PLACED {} ;", at(990, row)));
        nets.push(format!("- l{k} ( ffl{k} Q ) ( ffr{k} D ) ;"));
    }
    cells.push(format!("- ffm DFF_X1 + PLACED {} ;", at(500, 135)));
    let corners = [(0, 0), (990, 0), (0, 139)];
    for (n, &(c, r)) in corners.iter().enumerate() {
        cells.push(format!("- ffc{n} DFF_X1 + PLACED {} ;", at(c, r)));
    }
    nets.push("- m ( ffm Q ) ( ffc0 D ) ( ffc1 D ) ( ffc2 D ) ;".to_string());
    (cells, nets)
}

/// A die of 140 rows holding `cells` and `nets`, every flop clocked from
/// the port `clk`.
fn design_text(cells: &[String], mut nets: Vec<String>) -> String {
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
    for cell in cells {
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

fn load((cells, nets): (Vec<String>, Vec<String>)) -> (NetlistDB, Liberty) {
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("clock.lef")], units).unwrap();
    let text = design_text(&cells, nets);
    let db = def::parse_str(&text, Path::new("flow.def"), library).unwrap();
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
            db.library.macros[cell.macro_id].name,
            cell.location.x.get(),
            cell.location.y.get()
        );
    }
    text
}

/// What `repair` reports of a timed session over the design of `work`
/// under a clock of `period_ns`, then every cell once the session ends.
fn repaired_text(
    work: (Vec<String>, Vec<String>),
    period_ns: f64,
    repair: impl FnOnce(&mut Resizer<'_, Sta>) -> String,
) -> String {
    let (mut db, lib) = load(work);
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
    let reported = repair(&mut resizer);
    let _ = resizer.finish().unwrap();
    format!("{reported}\n{}", cells_text(&db))
}

/// Design, setup and hold repair of the timing work under a clock of
/// `period_ns`, as text: what each repair did and every cell after.
fn repaired(period_ns: f64) -> String {
    repaired_text(timing_work(), period_ns, |resizer| {
        let design = Repairer::new(resizer).repair_design().unwrap();
        let mut reports = Vec::new();
        let setup = repair_setup(resizer, SetupOptions::default(), &mut |progress| {
            reports.push((progress.iteration, progress.failing));
        })
        .unwrap();
        let hold = repair_hold(
            resizer,
            &HoldOptions {
                hold_margin: Seconds::new(0.12e-9),
                ..HoldOptions::default()
            },
        )
        .unwrap();
        format!("{design:#?}\nreported (iteration, failing): {reports:?}\n{setup:#?}\n{hold:#?}")
    })
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

/// The longest wire design repair leaves, as `repair_design -max_wire_length
/// 100`.
const MAX_WIRE_LENGTH: Dbu = Dbu(200_000);

#[test]
fn design_repair_splits_long_and_overloaded_nets_as_it_always_has() {
    let repaired = repaired_text(long_and_loaded_nets(), 1.0, |resizer| {
        resizer.set_max_wire_length(Some(MAX_WIRE_LENGTH));
        let design = Repairer::new(resizer).repair_design().unwrap();
        format!("{design:#?}")
    });

    assert_golden("long_and_loaded_nets.txt", &repaired);
}
