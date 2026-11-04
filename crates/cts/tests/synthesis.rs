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
use pare_cts::{CtsInput, CtsOptions, CtsSummary, synthesize};
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

/// The clock constraint of every generated design.
const CLOCK: &str = "create_clock -name core_clock -period 1.0 [get_ports clk]\n";

fn load(columns: usize, rows: usize, inverter: bool) -> (NetlistDB, Liberty, Sdc) {
    load_constrained(columns, rows, inverter, CLOCK)
}

/// The generated design under the constraints `sdc_text`.
fn load_constrained(
    columns: usize,
    rows: usize,
    inverter: bool,
    sdc_text: &str,
) -> (NetlistDB, Liberty, Sdc) {
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("clock.lef")], units).unwrap();
    let db = def::parse_str(
        &design(columns, rows, inverter),
        Path::new("grid.def"),
        library,
    )
    .unwrap();
    let lib = liberty::parse_file(&fixture("clock.lib")).unwrap();
    let constraints = sdc::parse_str(sdc_text, Path::new("grid.sdc"), &db, &lib).unwrap();
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

/// Checks `actual` against the golden file `name`, or writes it there when
/// `PARE_BLESS` is set, for pinning down whole results.
fn assert_golden(name: &str, actual: &str) {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/golden")
        .join(name);
    if std::env::var_os("PARE_BLESS").is_some() {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, actual).unwrap();
        return;
    }
    let expected = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!("{}: {e}; PARE_BLESS=1 writes it", path.display());
    });
    if let Some((line, (want, got))) = expected
        .lines()
        .zip(actual.lines())
        .enumerate()
        .find(|(_, (want, got))| want != got)
    {
        panic!(
            "{} differs at line {}:\n  golden: {want}\n  now:    {got}",
            path.display(),
            line + 1
        );
    }
    assert_eq!(
        expected.lines().count(),
        actual.lines().count(),
        "{} differs in length",
        path.display()
    );
}

/// Every cell clock tree synthesis added, where it put it, and every clock
/// net with its rule, driver and loads, as text to pin a run down.
fn clock_trees_text(db: &NetlistDB, first_added: usize, summary: &CtsSummary) -> String {
    let mut text = format!("{summary:#?}\n");
    for cell in &db.cells[first_added..] {
        let _ = writeln!(
            text,
            "cell {} {} {} {} {:?}",
            cell.name,
            db.library.macros[cell.macro_id.index()].name,
            cell.location.x.get(),
            cell.location.y.get(),
            cell.orient
        );
    }
    let mut nets: Vec<&pare_common::db::design::Net> = db
        .nets
        .iter()
        .filter(|n| n.signal_use == SignalUse::Clock)
        .collect();
    nets.sort_by(|a, b| a.name.cmp(&b.name));
    for net in nets {
        let rule = net
            .non_default_rule
            .map_or("-", |r| db.non_default_rules[r.index()].name.as_str());
        let mut pins: Vec<String> = net.pins.iter().map(|&p| db.pin_name(p)).collect();
        pins.sort();
        let _ = writeln!(text, "net {} {rule} {}", net.name, pins.join(" "));
    }
    text
}

/// Synthesizes the trees of a 24 by 24 grid of flops under `sdc_text`
/// with wires of at most `max_wire_length`, as text.
fn synthesized_grid(sdc_text: &str, max_wire_length: i64) -> String {
    let (mut db, lib, constraints) = load_constrained(24, 24, false, sdc_text);
    let link = Link::new(&db, &lib, &[]).unwrap();
    let clocks = clock_network(&db, &link, &constraints);
    let input = CtsInput {
        link: &link,
        sdc: &constraints,
        clocks: &clocks,
        signal_rc: wire(&db, "metal2", 3.5, 0.1),
        clock_rc: wire(&db, "metal3", 1.5, 0.11),
        max_wire_length,
        options: CtsOptions::default(),
    };
    let first_added = db.cells.len();

    let trees = synthesize(&mut db, &input).unwrap();

    clock_trees_text(&db, first_added, &trees.summary)
}

#[test]
fn a_grid_of_clustered_flops_gets_the_trees_it_always_has() {
    assert_golden("grid_24x24.txt", &synthesized_grid(CLOCK, 0));
}

#[test]
fn a_grid_under_a_fanout_limit_gets_the_trees_it_always_has() {
    let sdc = format!("{CLOCK}set_max_fanout 10 [current_design]\n");
    assert_golden("grid_24x24_fanout_10.txt", &synthesized_grid(&sdc, 0));
}

#[test]
fn a_grid_under_a_wire_length_limit_gets_the_trees_it_always_has() {
    assert_golden(
        "grid_24x24_wire_100um.txt",
        &synthesized_grid(CLOCK, 200_000),
    );
}
