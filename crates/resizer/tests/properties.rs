//! What the resizer keeps, whatever its input: the delay calculator's
//! effective capacitance stays within the load and its delay grows with
//! the load; a Steiner tree spans its pins, no shorter than half their
//! bounding box's perimeter and no longer than the star from the driver;
//! design repair drives every load from its old driver through buffers
//! alone.

#![allow(clippy::unwrap_used)]

mod common;

use common::{Work, at, clock_cells, repaired_in_session};
use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{CellId, NetId, PinId};
use pare_common::db::liberty::{Liberty, TimingArc};
use pare_common::db::library::PinDirection;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::units::{Farads, Ohms, Seconds};
use pare_resizer::Repairer;
use pare_resizer::dmp::{PiModel, gate_delay};
use pare_resizer::steiner::{SteinerTree, prim_dijkstra};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};
use std::fmt::Write as _;

/// Every delay arc of the fixture library's buffers and inverter.
fn driver_arcs(lib: &Liberty) -> Vec<TimingArc> {
    ["BUF_X1", "CLKBUF_X3", "INV_X1"]
        .iter()
        .flat_map(|name| {
            let cell = lib.cell(lib.cell_id(name).unwrap());
            cell.arc_sets
                .iter()
                .flat_map(|s| s.arcs.clone())
                .collect::<Vec<_>>()
        })
        .collect()
}

/// A pi load as wires and pins make them: a near capacitance from almost
/// none to tens of femtofarads, a resistance from a short wire's to a long
/// one's and a far capacitance of one to a few hundred femtofarads.
fn pi_load() -> impl Strategy<Value = PiModel> {
    let near = prop_oneof![1e-18..1e-16_f64, 1e-16..50e-15_f64];
    (near, 10.0..5000.0_f64, 1e-15..300e-15_f64).prop_map(|(c2, rpi, c1)| PiModel {
        c2: Farads::new(c2),
        rpi: Ohms::new(rpi),
        c1: Farads::new(c1),
    })
}

fn scaled(pi: PiModel, factor: f64) -> PiModel {
    PiModel {
        c2: pi.c2 * factor,
        c1: pi.c1 * factor,
        ..pi
    }
}

/// Up to two dozen pins on a grid a few hundred microns across, the first
/// of them the driver.
fn pins() -> impl Strategy<Value = Vec<Point<Dbu>>> {
    prop::collection::vec((0..400_000_i64, 0..400_000_i64), 2..25).prop_map(|xy| {
        xy.into_iter()
            .map(|(x, y)| Point::new(Dbu::new(x), Dbu::new(y)))
            .collect()
    })
}

fn manhattan(a: Point<Dbu>, b: Point<Dbu>) -> Dbu {
    (a.x - b.x).abs() + (a.y - b.y).abs()
}

/// Whether every node of `tree` reaches its root through its parents.
fn reaches_root(tree: &SteinerTree) -> bool {
    (0..tree.nodes.len()).all(|start| {
        let mut node = start;
        for _ in 0..tree.nodes.len() {
            match tree.parent[node] {
                Some(parent) => node = parent,
                None => return node == tree.root,
            }
        }
        false
    })
}

/// One buffer the input port drives, loading flops spread over the die.
#[derive(Clone, Debug)]
struct FanoutNet {
    driver: (usize, usize),
    sinks: Vec<(usize, usize)>,
}

/// A buffer anywhere on the die loading one to eighty flops anywhere on
/// it, past what one buffer may drive in load or wire for many of them.
fn fanout_net() -> impl Strategy<Value = FanoutNet> {
    let place = (0..1000_usize, 0..140_usize);
    (place.clone(), prop::collection::vec(place, 1..80))
        .prop_map(|(driver, sinks)| FanoutNet { driver, sinks })
}

/// The flops of `net` are `s0`, `s1`, ...; its buffer is `fan`.
fn work_of(net: &FanoutNet) -> Work {
    let mut cells = vec![format!(
        "- fan BUF_X1 + PLACED {} ;",
        at(net.driver.0, net.driver.1)
    )];
    let mut fanout = String::from("- fo ( fan Z )");
    for (i, &(c, r)) in net.sinks.iter().enumerate() {
        cells.push(format!("- s{i} DFF_X1 + PLACED {} ;", at(c, r)));
        let _ = write!(fanout, " ( s{i} D )");
    }
    let nets = vec!["- in0 ( PIN in0 ) ( fan A ) ;".to_string(), fanout + " ;"];
    Work { cells, nets }
}

/// The design of `net` after design repair.
fn repaired(net: &FanoutNet, lib: &Liberty) -> NetlistDB {
    let constraints = "create_clock -name core -period 1.0 [get_ports clk]\n\
                       set_input_delay 0.1 -clock core [get_ports in0]\n";
    let (_, db) = repaired_in_session(work_of(net), constraints, lib, |resizer| {
        Repairer::new(resizer).repair_design().unwrap()
    });
    db
}

/// Whether `pin` drives its net: a cell's output or an input port.
fn drives(db: &NetlistDB, pin: PinId) -> bool {
    match db.pins[pin].owner {
        PinOwner::Cell { cell, pin } => {
            db.library.macros[db.cells[cell].macro_id].pins[pin].direction == PinDirection::Output
        }
        PinOwner::Io(io) => db.io_pins[io].direction == PinDirection::Input,
    }
}

fn driver_of(db: &NetlistDB, net: NetId) -> PinId {
    db.nets[net]
        .pins
        .iter()
        .copied()
        .find(|&p| drives(db, p))
        .unwrap()
}

fn cell_of(db: &NetlistDB, pin: PinId) -> CellId {
    match db.pins[pin].owner {
        PinOwner::Cell { cell, .. } => cell,
        PinOwner::Io(_) => panic!("{} is a port", db.pin_name(pin)),
    }
}

/// The pin of `cell` that does not drive.
fn input_of(db: &NetlistDB, cell: CellId) -> PinId {
    db.cells[cell]
        .pins
        .iter()
        .copied()
        .find(|&p| !drives(db, p))
        .unwrap()
}

fn is_buffer(db: &NetlistDB, lib: &Liberty, cell: CellId) -> bool {
    let name = &db.library.macros[db.cells[cell].macro_id].name;
    lib.cell(lib.cell_id(name).unwrap()).is_buffer()
}

fn config() -> Config {
    Config {
        cases: 256,
        rng_seed: RngSeed::Fixed(29),
        failure_persistence: None,
        ..Config::default()
    }
}

/// Fewer cases for properties that run a timed repair session each.
fn session_config() -> Config {
    Config {
        cases: 32,
        ..config()
    }
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn the_effective_capacitance_lies_within_the_load(
        arc in any::<prop::sample::Index>(),
        pi in pi_load(),
        in_slew in 5e-12..300e-12_f64,
    ) {
        let lib = clock_cells();
        let arcs = driver_arcs(&lib);
        let arc = arc.get(&arcs);

        let delay = gate_delay(arc, &lib.thresholds, Seconds::new(in_slew), pi);

        let total = (pi.c1 + pi.c2).get();
        prop_assert!(delay.ceff.get() >= 0.0 && delay.ceff.get() <= total * (1.0 + 1e-9),
            "ceff {} of {total}", delay.ceff.get());
        prop_assert!(delay.delay.get().is_finite() && delay.slew.get() > 0.0);
    }

    #[test]
    fn a_heavier_load_is_driven_no_faster(
        arc in any::<prop::sample::Index>(),
        pi in pi_load(),
        in_slew in 5e-12..300e-12_f64,
        factor in 1.01..3.0_f64,
    ) {
        let lib = clock_cells();
        let arcs = driver_arcs(&lib);
        let arc = arc.get(&arcs);
        let slew = Seconds::new(in_slew);

        let light = gate_delay(arc, &lib.thresholds, slew, pi);
        let heavy = gate_delay(arc, &lib.thresholds, slew, scaled(pi, factor));

        prop_assert!(heavy.delay >= light.delay, "{} then {}", light.delay.get(), heavy.delay.get());
    }

    #[test]
    fn a_steiner_tree_spans_its_pins_between_their_box_and_their_star(
        pins in pins(),
        alpha in prop::sample::select(vec![0.0, 0.3, 0.5, 0.8, 1.0]),
    ) {
        let tree = prim_dijkstra(&pins, 0, alpha);

        let half_perimeter = {
            let (xs, ys): (Vec<_>, Vec<_>) = pins.iter().map(|p| (p.x, p.y)).unzip();
            (*xs.iter().max().unwrap() - *xs.iter().min().unwrap())
                + (*ys.iter().max().unwrap() - *ys.iter().min().unwrap())
        };
        let star = pins.iter().map(|&p| manhattan(p, pins[0])).fold(Dbu::ZERO, |a, b| a + b);
        prop_assert_eq!(&tree.nodes[..tree.terminals], &pins[..]);
        prop_assert!(reaches_root(&tree));
        prop_assert!(tree.length() >= half_perimeter && tree.length() <= star,
            "{} not within {half_perimeter}..{star}", tree.length().get());
    }
}

proptest! {
    #![proptest_config(session_config())]

    #[test]
    fn design_repair_drives_every_flop_from_its_buffer_through_buffers_alone(net in fanout_net()) {
        let lib = clock_cells();

        let db = repaired(&net, &lib);

        let fan = db.cell_id("fan").unwrap();
        for i in 0..net.sinks.len() {
            let flop = db.cell_id(&format!("s{i}")).unwrap();
            let mut cell = cell_of(&db, driver_of(&db, db.pins[input_of(&db, flop)].net));
            for _ in 0..db.cells.len() {
                if cell == fan {
                    break;
                }
                prop_assert!(is_buffer(&db, &lib, cell), "s{i} is driven by {}", db.cells[cell].name);
                cell = cell_of(&db, driver_of(&db, db.pins[input_of(&db, cell)].net));
            }
            prop_assert_eq!(cell, fan, "s{} never reaches fan", i);
        }
    }
}
