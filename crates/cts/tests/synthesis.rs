//! Clock tree synthesis and clock inverter repair on small generated
//! designs over the fixture cells.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{NetId, PinId};
use pare_common::db::liberty::Liberty;
use pare_common::db::library::SignalUse;
use pare_common::db::parser::{def, lef, liberty, sdc};
use pare_common::db::sdc::Sdc;
use pare_common::geom::dbu::DbuPerMicron;
use pare_cts::{CtsInput, CtsOptions, synthesize};
use pare_resizer::parasitics::WireRc;
use pare_resizer::{ClockNetwork, Constants, Link, Namer, Netlist, repair_clock_inverters};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

/// A die with `columns` by `rows` flops on a grid, their clocks on one net
/// from port `clk`, optionally through one inverter.
fn design(columns: usize, rows: usize, inverter: bool) -> String {
    let mut text = String::from(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN grid ;\n\
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
    let flops = columns * rows;
    let _ = writeln!(text, "COMPONENTS {} ;", flops + usize::from(inverter));
    for i in 0..flops {
        let (c, r) = (i % columns, i / columns);
        let _ = writeln!(
            text,
            "- ff{i} DFF_X1 + PLACED ( {} {} ) N ;",
            4000 + c * (380_000 / columns.max(1)) / 380 * 380,
            2000 + r * (140 / rows.max(1)) * 2800
        );
    }
    if inverter {
        text.push_str("- clkinv INV_X1 + PLACED ( 2000 2000 ) N ;\n");
    }
    text.push_str("END COMPONENTS\nPINS 1 ;\n");
    text.push_str(
        "- clk + NET clk + DIRECTION INPUT + USE CLOCK\n  + PORT + LAYER metal2 ( -35 0 ) ( 35 70 )\n  + PLACED ( 200000 0 ) N ;\nEND PINS\n",
    );
    let nets = 1 + usize::from(inverter);
    let _ = writeln!(text, "NETS {nets} ;");
    let mut sinks = String::new();
    for i in 0..flops {
        let _ = write!(sinks, " ( ff{i} CK )");
    }
    if inverter {
        let _ = writeln!(text, "- clk ( PIN clk ) ( clkinv A ) + USE CLOCK ;");
        let _ = writeln!(text, "- clk_n ( clkinv ZN ){sinks} + USE CLOCK ;");
    } else {
        let _ = writeln!(text, "- clk ( PIN clk ){sinks} + USE CLOCK ;");
    }
    text.push_str("END NETS\nEND DESIGN\n");
    text
}

fn load(columns: usize, rows: usize, inverter: bool) -> (NetlistDB, Liberty, Sdc) {
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("clock.lef")], units).unwrap();
    let db = def::parse_str(
        &design(columns, rows, inverter),
        Path::new("grid.def"),
        library,
    )
    .unwrap();
    let lib = liberty::parse_file(&fixture("clock.lib")).unwrap();
    let constraints = sdc::parse_str(
        "create_clock -name core_clock -period 1.0 [get_ports clk]\n",
        Path::new("grid.sdc"),
        &db,
        &lib,
    )
    .unwrap();
    (db, lib, constraints)
}

fn wire(db: &NetlistDB, layer: &str, ohm: f64, ff: f64) -> WireRc {
    WireRc::per_micron(ohm, ff, db.library.routing_layer_id(layer).unwrap())
}

fn clock_network(db: &NetlistDB, link: &Link<'_>, constraints: &Sdc) -> ClockNetwork {
    let netlist = Netlist::new(db, link);
    let constants = Constants::find(&netlist);
    ClockNetwork::from_sdc(&netlist, constraints, &constants)
}

/// The cell driving `net`, if one does.
fn driver_cell(db: &NetlistDB, net: NetId) -> Option<String> {
    db.nets[net.index()]
        .pins
        .iter()
        .find_map(|&p| match db.pins[p.index()].owner {
            PinOwner::Cell { cell, pin } => {
                let macro_pin = db.cell_macro(cell).pin(pin);
                (macro_pin.name == "Z" || macro_pin.name == "ZN")
                    .then(|| db.cells[cell.index()].name.clone())
            }
            PinOwner::Io(_) => None,
        })
}

/// The chain of cells from a flop's clock pin back to the port, by name.
fn path_to_port(db: &NetlistDB, mut pin: PinId) -> Vec<String> {
    let mut path = Vec::new();
    for _ in 0..64 {
        let net = db.pins[pin.index()].net;
        let Some(name) = driver_cell(db, net) else {
            return path;
        };
        let cell = db.cell_id(&name).unwrap();
        path.push(name);
        pin = db.cells[cell.index()]
            .pins
            .iter()
            .copied()
            .find(|&p| matches!(db.pins[p.index()].owner, PinOwner::Cell { pin, .. } if db.cell_macro(cell).pin(pin).name == "A"))
            .unwrap();
    }
    panic!("the clock path does not reach the port")
}

fn flop_clock_pins(db: &NetlistDB) -> Vec<PinId> {
    db.pins
        .iter()
        .enumerate()
        .filter_map(|(i, p)| match p.owner {
            PinOwner::Cell { cell, pin } if db.cell_macro(cell).pin(pin).name == "CK" => {
                Some(PinId::new(i))
            }
            _ => None,
        })
        .collect()
}

#[test]
fn every_flop_is_clocked_through_a_tree_of_clock_buffers() {
    let (mut db, lib, constraints) = load(16, 16, false);
    let link = Link::new(&db, &lib, &[]).unwrap();
    let clocks = clock_network(&db, &link, &constraints);
    let input = CtsInput {
        link: &link,
        sdc: &constraints,
        clocks: &clocks,
        signal_rc: wire(&db, "metal2", 3.5, 0.1),
        clock_rc: wire(&db, "metal3", 1.5, 0.11),
        max_wire_length: 0,
        options: CtsOptions::default(),
    };

    let trees = synthesize(&mut db, &input).unwrap();

    assert_eq!(trees.summary.trees, 1);
    assert_eq!(trees.summary.sinks, 256);
    assert!(trees.summary.buffers > 1);
    for pin in flop_clock_pins(&db) {
        let path = path_to_port(&db, pin);
        assert_ne!(path.len(), 0);
        assert!(
            path.iter().all(|name| name.starts_with("clkbuf_")),
            "{path:?}"
        );
        assert_eq!(path.last().map(String::as_str), Some("clkbuf_0_clk"));
    }
}

#[test]
fn the_root_net_gets_the_clock_rule_of_doubled_spacing() {
    let (mut db, lib, constraints) = load(16, 16, false);
    let link = Link::new(&db, &lib, &[]).unwrap();
    let clocks = clock_network(&db, &link, &constraints);
    let input = CtsInput {
        link: &link,
        sdc: &constraints,
        clocks: &clocks,
        signal_rc: wire(&db, "metal2", 3.5, 0.1),
        clock_rc: wire(&db, "metal3", 1.5, 0.11),
        max_wire_length: 0,
        options: CtsOptions::default(),
    };

    let _ = synthesize(&mut db, &input).unwrap();

    let rule = db
        .non_default_rules
        .iter()
        .position(|r| r.name == "CTS_NDR_0")
        .unwrap();
    let ndr = &db.non_default_rules[rule];
    let metal1 = &ndr.layers[0];
    assert_eq!(
        metal1.spacing.map(pare_common::geom::dbu::Dbu::get),
        Some(260)
    );
    let clk = db.net_id("clk").unwrap();
    assert_eq!(
        db.nets[clk.index()]
            .non_default_rule
            .map(pare_common::db::indices::NonDefaultRuleId::index),
        Some(rule)
    );
    let root = db.net_id("clknet_0_clk").unwrap();
    assert_eq!(
        db.nets[root.index()]
            .non_default_rule
            .map(pare_common::db::indices::NonDefaultRuleId::index),
        Some(rule)
    );
    assert_eq!(db.nets[root.index()].signal_use, SignalUse::Clock);
}

#[test]
fn each_load_of_a_clock_inverter_gets_its_own_copy() {
    let (mut db, lib, constraints) = load(3, 1, true);
    let link = Link::new(&db, &lib, &[]).unwrap();
    let clocks = clock_network(&db, &link, &constraints);
    let mut namer = Namer::new();

    let done = repair_clock_inverters(&mut db, &link, &clocks, &mut namer).unwrap();

    assert_eq!(done.inverters, 1);
    assert_eq!(done.clones, 3);
    assert!(db.cell_id("clkinv").is_none());
    for pin in flop_clock_pins(&db) {
        let net = db.pins[pin.index()].net;
        assert_eq!(db.nets[net.index()].pins.len(), 2);
        assert!(driver_cell(&db, net).unwrap().starts_with("clkinv_"));
    }
}
