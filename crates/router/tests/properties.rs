//! What routing keeps on a roomy design, whatever its cells and nets: every
//! net ends in one connected piece, and the independent checker of the
//! verify crate finds no violation.

#![allow(clippy::unwrap_used)]

use pare_common::config::RoutingConfig;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::NetId;
use pare_common::db::parser::{def, lef};
use pare_router::{RouteSetup, route};
use pare_verify::drc::check_drc;
use pare_verify::routing::net_components;
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

/// Columns of cell slots a row holds, each slot six sites wide.
const SLOT_COLUMNS: i64 = 16;
/// Rows of the core; cells stand on every other one.
const ROWS: i64 = 12;
/// The width of a slot, in database units: six sites of 0.2 um.
const SLOT: i64 = 1200;
/// The height of a row, in database units.
const ROW_HEIGHT: i64 = 2000;

/// A roomy design: inverters in some of the slots of every other row, every
/// inverter's output wired to the inputs of up to three others.
// Inverters alone: the fixture's NAND2 has its inputs one M2 track apart, and
// pin access does not yet keep neighbouring pins' vias apart (issue #87).
#[derive(Clone, Debug)]
struct Design {
    /// Each gate's slot, as a column and a row.
    gates: Vec<(i64, i64)>,
    /// Each net's driving gate and the gates whose inputs it loads.
    nets: Vec<(usize, Vec<usize>)>,
}

fn design() -> impl Strategy<Value = Design> {
    let slots: Vec<(i64, i64)> = (0..ROWS)
        .step_by(2)
        .flat_map(|r| (0..SLOT_COLUMNS).map(move |c| (c, r)))
        .collect();
    prop::sample::subsequence(slots, 4..24)
        .prop_flat_map(|slots| {
            let n = slots.len();
            let loads = prop::collection::vec(prop::collection::vec(0..n, 1..4), n);
            (Just(slots), loads)
        })
        .prop_map(|(gates, loads)| Design {
            gates,
            nets: loads.into_iter().enumerate().collect(),
        })
}

/// The DEF of `design` over the fixture library, a load that would drive
/// itself or whose input an earlier net took left out.
fn def_text(design: &Design) -> String {
    let (w, h) = (SLOT_COLUMNS * SLOT, ROWS * ROW_HEIGHT);
    let mut text = format!(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN props ;\n\
         UNITS DISTANCE MICRONS 1000 ;\nDIEAREA ( 0 0 ) ( {w} {h} ) ;\n"
    );
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
    let _ = writeln!(text, "COMPONENTS {} ;", design.gates.len());
    for (g, &(c, r)) in design.gates.iter().enumerate() {
        let orient = if r % 2 == 0 { "N" } else { "FS" };
        let _ = writeln!(
            text,
            "- g{g} INV + PLACED ( {} {} ) {orient} ;",
            c * SLOT,
            r * ROW_HEIGHT
        );
    }
    let mut free = vec![true; design.gates.len()];
    let mut nets = Vec::new();
    for &(driver, ref loads) in &design.nets {
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
    let _ = writeln!(text, "END COMPONENTS\nNETS {} ;", nets.len());
    for net in &nets {
        let _ = writeln!(text, "{net}");
    }
    text.push_str("END NETS\nEND DESIGN\n");
    text
}

fn parse(text: &str) -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse_str(text, Path::new("props.def"), library).unwrap()
}

fn config() -> Config {
    Config {
        cases: 16,
        rng_seed: RngSeed::Fixed(47),
        failure_persistence: None,
        ..Config::default()
    }
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn routing_a_roomy_design_connects_every_net_and_breaks_no_rule(design in design()) {
        let mut db = parse(&def_text(&design));

        let summary = route(&mut db, &RouteSetup::from(&RoutingConfig::default())).unwrap();

        prop_assert!(summary.incomplete.is_empty() && summary.inaccessible.is_empty(), "{summary:?}");
        for n in 0..db.nets.len() {
            let net = NetId::new(n);
            prop_assert_eq!(net_components(&db, net), 1, "net {}", db.nets[net].name);
        }
        let drc = check_drc(&db);
        prop_assert!(drc.violations.is_empty(), "{} violations, first {:?}", drc.violations.len(), drc.violations.first());
    }
}
