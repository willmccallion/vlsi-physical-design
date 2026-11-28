use super::*;
use crate::resizer::GlobalRouting;
use crate::resizer::gate_delays;
use crate::route_parasitics::RouteRc;
use crate::testing::{
    DefSections, cells, clock_cells_source, def_text, design, pin, with_timed_session,
    with_wired_session,
};
use crate::testing::{metal2_wires, with_session_inputs};
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::{
    NetRoute, PinGridLocation, RoutePoint, RouteSegment, RouteSource,
};
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::liberty::{Edge, EdgeValues, Liberty, MinMax};
use pare_common::db::library::SignalUse;
use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
use std::fmt::Write;

/// A flop driving a flop at the far corner of the die.
fn across_the_die() -> pare_common::db::design::NetlistDB {
    design(&def_text(
        "across",
        &DefSections {
            components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 780000 780000 ) N ;\n",
            nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( f2 D ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// How far `pin`'s slower slew is past its limit; negative within it.
fn slew_excess<T: Timing>(resizer: &Resizer<'_, T>, pin: PinId) -> Seconds {
    let netlist = resizer.netlist();
    let (cell, port) = netlist.lib_cell_port(pin).unwrap();
    let slew = resizer.graph.slew(pin);
    slew[Edge::Rise].max(slew[Edge::Fall]) - resizer.max_input_slew(cell, port)
}

/// A flop driving a flop 300 microns away, whose input slew there
/// is under the library's default limit of 0.2 nanoseconds.
fn apart() -> pare_common::db::design::NetlistDB {
    design(&def_text(
        "apart",
        &DefSections {
            components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 602000 2800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( f2 D ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

/// Whether design repair under `constraints`, keeping `slew_margin`
/// percent of each limit in hand, finds a load of [`apart`]'s net over
/// its slew limit, and the load's slew.
fn load_slew_violated(constraints: &str, slew_margin: f64) -> (bool, Seconds) {
    let mut db = apart();
    let lib = cells(&clock_cells_source());
    let mut found = (false, Seconds::ZERO);
    with_wired_session(&mut db, &lib, constraints, |resizer| {
        let (driver, load) = (pin(resizer.db, "f1/Q"), pin(resizer.db, "f2/D"));
        resizer.options.slew_margin = slew_margin;
        let slew = resizer.graph.slew(load).max();
        found = (Repairer::new(resizer).loads_violate_slew(driver), slew);
    });
    found
}

const SLOW_CLOCK: &str = "create_clock -name clk -period 10.0 [get_ports clk]\n";

#[test]
fn a_load_within_the_library_default_breaks_a_tighter_design_limit() {
    let (unconstrained, slew) = load_slew_violated(SLOW_CLOCK, 0.0);
    assert!(!unconstrained && slew < Seconds::new(0.2e-9), "{slew:?}");
    let limit = slew.get() * 0.5e9;
    let constraints = format!("{SLOW_CLOCK}set_max_transition {limit} [current_design]\n");

    let (constrained, _) = load_slew_violated(&constraints, 0.0);

    assert!(constrained);
}

#[test]
fn the_slew_margin_applies_to_a_load_with_the_library_default_limit() {
    let (_, slew) = load_slew_violated(SLOW_CLOCK, 0.0);
    let margin = (1.0 - slew.get() / 0.2e-9).mul_add(100.0, 1.0);

    let (violated, _) = load_slew_violated(SLOW_CLOCK, margin);

    assert!(violated, "{slew:?} at {margin}%");
}

#[test]
fn the_wire_left_behind_a_repeater_is_measured_from_where_it_stands() {
    let (left, right) = (
        Point::new(Dbu::ZERO, Dbu::ZERO),
        Point::new(Dbu::new(100), Dbu::ZERO),
    );

    let toward_left = back_along(right, left, 30.5, Dbu::new(100));
    let toward_right = back_along(left, right, 30.5, Dbu::new(100));

    assert_eq!(
        toward_left,
        (Point::new(Dbu::new(69), Dbu::ZERO), Dbu::new(69))
    );
    assert_eq!(
        toward_right,
        (Point::new(Dbu::new(30), Dbu::ZERO), Dbu::new(70))
    );
}

#[test]
fn a_junction_isolates_no_branch_for_load_slew_behind_an_undriven_port() {
    let mut db = design(&def_text(
        "fork",
        &DefSections {
            components: "- b1 BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - b2 BUF_X1 + PLACED ( 2000 2800 ) N ;\n",
            nets: "- in ( PIN in ) ( b1 A ) ( b2 A ) ;\n",
            inputs: &["in"],
            outputs: &[],
        },
    ));
    let lib = cells(&clock_cells_source());

    with_wired_session(&mut db, &lib, "", |resizer| {
        let driver = resizer.db.io_pins[resizer.db.io_pin_id("in").unwrap()].pin;
        let at = Point::new(Dbu::new(2000), Dbu::new(2800));
        let load = |name: &str| {
            let pin = pin(resizer.db, name);
            bnet::Bnet::load(at, pin, bnet::LoadModel::load(&*resizer, pin))
        };
        let tree = bnet::Bnet::junction(at, load("b1/A"), load("b2/A"));
        resizer.options.slew_margin = 99.99;
        let mut repairer = Repairer::new(resizer);

        let _ = repairer
            .repair_tree(&tree, driver, Farads::INFINITY)
            .unwrap();

        assert_eq!(repairer.summary.inserted, 0);
    });
}

/// Global routes in which net `q` runs from its driver to its first
/// load and misses its second.
struct BrokenRoutes;

impl RouteSource for BrokenRoutes {
    fn update(&mut self, _: &NetlistDB, _: &[NetId]) {}

    fn begin_trial(&mut self) {}

    fn keep_trial(&mut self) {}

    fn restore_trial(&mut self) {}

    fn route(&self, db: &NetlistDB, net: NetId) -> Option<NetRoute> {
        if db.nets[net].name != "q" {
            return None;
        }
        let at = |pin: PinId| RoutePoint {
            at: db.pin_position(pin),
            layer: LayerId(0),
        };
        let pins = &db.nets[net].pins;
        Some(NetRoute {
            segments: vec![RouteSegment {
                from: at(pins[0]),
                to: at(pins[1]),
            }],
            pins: pins
                .iter()
                .map(|&pin| PinGridLocation {
                    pin,
                    at: db.pin_position(pin),
                    grid: db.pin_position(pin),
                    layer: LayerId(0),
                })
                .collect(),
        })
    }
}

#[test]
fn a_design_repair_that_fails_leaves_the_session_timed_and_unbatched() {
    let mut db = design(&def_text(
        "fork",
        &DefSections {
            components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 202000 2800 ) N ;\n\
                             - f3 DFF_X1 + PLACED ( 2000 202800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ( f3 CK ) ;\n\
                       - q ( f1 Q ) ( f2 D ) ( f3 D ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ));
    let lib = cells(&clock_cells_source());
    let rc = RouteRc::new(&db.library, &[], &[], LayerId(0));
    let wires = metal2_wires(&db);

    with_session_inputs(&mut db, &lib, SLOW_CLOCK, wires, |inputs, sdc| {
        let mut source = BrokenRoutes;
        let routes = GlobalRouting {
            source: &mut source,
            rc,
            legalize: &|_, _| {},
        };
        let mut resizer = Resizer::on_global_routes(inputs, sdc, routes).unwrap();

        let repaired = Repairer::new(&mut resizer).repair_design();

        assert!(
            matches!(repaired, Err(RepairError::RouteTree { .. })),
            "{repaired:?}"
        );
        let q = pin(resizer.db, "f1/Q");
        assert!(resizer.sta().slack(q, MinMax::Max).time().is_some());
    });
}

#[test]
fn a_long_clock_net_gets_repeaters_and_the_session_keeps_its_options() {
    let mut db = design(&def_text(
        "long_clock",
        &DefSections {
            components: "- cb CLKBUF_X3 + PLACED ( 2000 2800 ) N ;\n\
                             - f DFF_X1 + PLACED ( 602000 2800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( f CK ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ));
    let lib = cells(&clock_cells_source());
    let constraints = format!("{SLOW_CLOCK}set_propagated_clock [all_clocks]\n");

    with_wired_session(&mut db, &lib, &constraints, |resizer| {
        let options = resizer.options;
        let strongest = resizer.r_strongest_buffer;
        let clock_buffer = resizer
            .link
            .liberty
            .buffer_cell(resizer.link.liberty.cell_id("CLKBUF_X3").unwrap())
            .unwrap();

        let summary = Repairer::new(resizer)
            .repair_clock_nets(&[clock_buffer], Some(Dbu::new(200_000)))
            .unwrap();

        assert!(summary.inserted > 0, "{summary:?}");
        assert_eq!(summary.length_violations, 1);
        assert_eq!(resizer.options, options);
        assert_eq!(resizer.r_strongest_buffer, strongest);
    });
}

#[test]
fn clock_nets_without_clock_buffers_cannot_be_repaired() {
    let mut db = apart();
    let lib = cells(&clock_cells_source());

    with_wired_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let repaired = Repairer::new(resizer).repair_clock_nets(&[], None);

        assert_eq!(repaired, Err(RepairError::NoBuffers));
    });
}

#[test]
fn a_net_across_the_die_gets_repeaters_that_bring_every_slew_within_its_limit() {
    let mut db = across_the_die();
    let lib = cells(&clock_cells_source());
    let clock = "create_clock -name clk -period 10.0 [get_ports clk]\n";

    with_wired_session(&mut db, &lib, clock, |resizer| {
        let end = pin(resizer.db, "f2/D");
        assert!(slew_excess(resizer, end) > Seconds::ZERO);

        let summary = Repairer::new(resizer).repair_design().unwrap();

        assert!(summary.inserted > 0, "{summary:?}");
        let netlist = resizer.netlist();
        let data_loads: Vec<PinId> = resizer
            .db
            .pins
            .ids()
            .filter(|&p| netlist.is_load(p) && !resizer.clocks.is_clock(p))
            .collect();
        assert!(data_loads.len() > 1);
        for load in data_loads {
            let excess = slew_excess(resizer, load);
            assert!(
                excess <= Seconds::ZERO,
                "{}: {excess:?}",
                resizer.db.pin_name(load)
            );
        }
    });
}

/// Runs `test` on a repair of [`apart`], wired, with the slew shape factor
/// 2.0, so that the arithmetic of a test is by hand.
fn with_a_repairer(constraints: &str, test: impl FnOnce(&mut Repairer<'_, '_, crate::sta::Sta>)) {
    with_a_repairer_over(&cells(&clock_cells_source()), constraints, test);
}

/// [`with_a_repairer`] over the cells of `lib`.
fn with_a_repairer_over(
    lib: &Liberty,
    constraints: &str,
    test: impl FnOnce(&mut Repairer<'_, '_, crate::sta::Sta>),
) {
    let mut db = apart();
    with_wired_session(&mut db, lib, constraints, |resizer| {
        resizer.slew_rc_factor = 2.0;
        test(&mut Repairer::new(resizer));
    });
}

fn femtofarads(value: f64) -> Farads {
    Farads::new(value * 1e-15)
}

#[test]
fn the_elmore_slew_of_a_wire_is_the_factor_times_the_rc_of_driver_wire_and_load() {
    let wire = DriverWire {
        res: OhmsPerMeter::from_ohms_per_micron(1.0),
        cap: FaradsPerMeter::from_femtofarads_per_micron(1.0),
        r_drvr: Ohms::new(1000.0),
        slew_rc_factor: 2.0,
        dbu_per_micron: DbuPerMicron::new(1000).unwrap(),
    };

    let slew = wire.elmore_slew(Dbu::new(1000), femtofarads(2.0));

    // One micron of 1 ohm and 1 fF: 1000 * 3 fF + 1 * 2 fF + 1 * 1 fF / 2.
    assert!((slew.get() / 3.0025e-12 - 2.0).abs() < 1e-9, "{slew:?}");
}

/// A branch of `cap_ff` femtofarads, `wire_length` long, that tolerates the
/// load slew `max_slew_ps` picoseconds.
fn branch(cap_ff: f64, max_slew_ps: f64, wire_length: i64) -> Branch {
    Branch {
        cap: femtofarads(cap_ff),
        fanout: 1.0,
        max_load_slew: Seconds::new(max_slew_ps * 1e-12),
        wire_length: Dbu::new(wire_length),
        loads: Vec::new(),
    }
}

/// A driver of `ohms` that buffers down to `max_cap_ff` femtofarads.
fn tree_driver(ohms: f64, max_cap_ff: f64) -> TreeDriver {
    TreeDriver {
        max_cap: femtofarads(max_cap_ff),
        resistance: Ohms::new(ohms),
        wire_resistance: Ohms::new(ohms),
    }
}

fn repeaters(left: bool, right: bool, reason: &'static str) -> JunctionRepeaters {
    JunctionRepeaters {
        left,
        right,
        reason,
    }
}

#[test]
fn a_junction_isolates_the_branch_with_less_slew_slack_when_the_load_slew_breaks() {
    with_a_repairer("", |repairer| {
        let driver = tree_driver(2000.0, f64::INFINITY);
        let found = |left: f64, right: f64| {
            repairer.junction_repeaters(&branch(left, 100.0, 0), &branch(right, 100.0, 0), driver)
        };

        // 2000 ohms and a factor of 2 turn 40 fF into 160 ps over the 100.
        assert_eq!(found(10.0, 30.0), repeaters(false, true, "load_slew"));
        assert_eq!(found(30.0, 10.0), repeaters(true, false, "load_slew"));
        assert_eq!(found(20.0, 20.0), repeaters(false, true, "load_slew"));
    });
}

#[test]
fn a_junction_whose_slew_just_meets_its_limit_isolates_no_branch() {
    with_a_repairer("", |repairer| {
        let driver = tree_driver(2000.0, f64::INFINITY);
        let (left, right) = (femtofarads(10.0), femtofarads(10.0));
        let slew = driver.resistance * (left + right) * 2.0;
        let at_limit = |limit: Seconds| Branch {
            max_load_slew: limit,
            ..branch(10.0, 0.0, 0)
        };

        let met = repairer.junction_repeaters(&at_limit(slew), &at_limit(slew), driver);
        let broken = repairer.junction_repeaters(&at_limit(slew * 0.99), &at_limit(slew), driver);

        assert_eq!(met, repeaters(false, false, "load_slew"));
        assert_eq!(broken, repeaters(true, false, "load_slew"));
    });
}

#[test]
fn a_junction_isolates_the_heavier_branch_when_the_load_breaks() {
    with_a_repairer("", |repairer| {
        let driver = tree_driver(0.0, 25.0);
        let found = |left: f64, right: f64| {
            repairer.junction_repeaters(&branch(left, 100.0, 0), &branch(right, 100.0, 0), driver)
        };

        assert_eq!(found(10.0, 20.0), repeaters(false, true, "max_cap"));
        assert_eq!(found(20.0, 10.0), repeaters(true, false, "max_cap"));
        assert_eq!(found(15.0, 15.0), repeaters(false, true, "max_cap"));
    });
}

#[test]
fn a_junction_whose_load_is_exactly_the_limit_isolates_no_branch() {
    with_a_repairer("", |repairer| {
        let driver = TreeDriver {
            max_cap: femtofarads(10.0) + femtofarads(15.0),
            ..tree_driver(0.0, 0.0)
        };

        let found =
            repairer.junction_repeaters(&branch(10.0, 100.0, 0), &branch(15.0, 100.0, 0), driver);

        assert_eq!(found, repeaters(false, false, "load_slew"));
    });
}

#[test]
fn a_junction_isolates_the_longer_branch_when_the_wire_is_too_long() {
    with_a_repairer("", |repairer| {
        repairer.resizer.options.max_wire_length = Some(Dbu::new(100));
        let driver = tree_driver(0.0, f64::INFINITY);
        let found = |left: i64, right: i64| {
            repairer.junction_repeaters(
                &branch(1.0, 100.0, left),
                &branch(1.0, 100.0, right),
                driver,
            )
        };

        assert_eq!(found(60, 50), repeaters(true, false, "max_length"));
        assert_eq!(found(50, 60), repeaters(false, true, "max_length"));
        assert_eq!(found(55, 55), repeaters(false, true, "max_length"));
        assert_eq!(found(50, 50), repeaters(false, false, "load_slew"));
    });
}

#[test]
fn a_junction_branch_presents_the_sum_of_its_loads_and_the_tighter_slew_limit() {
    let source = clock_cells_source()
        .replace(
            "capacitance : 1.1 ;",
            "capacitance : 1.1 ; fanout_load : 2.0 ;",
        )
        .replace(
            "capacitance : 0.95 ;",
            "capacitance : 0.95 ; fanout_load : 3.0 ;",
        );
    with_a_repairer_over(&cells(&source), "", |repairer| {
        let at = Point::new(Dbu::new(2000), Dbu::new(2800));
        let load = |name: &str| {
            let pin = pin(repairer.resizer.db, name);
            bnet::Bnet::load(at, pin, bnet::LoadModel::load(&*repairer.resizer, pin))
        };
        let tree = bnet::Bnet::junction(at, load("f2/D"), load("f1/CK"));
        let (a, b) = (
            bnet::LoadModel::load(&*repairer.resizer, pin(repairer.resizer.db, "f2/D")),
            bnet::LoadModel::load(&*repairer.resizer, pin(repairer.resizer.db, "f1/CK")),
        );

        let branch = repairer
            .repair_branch(&tree, tree_driver(0.0, f64::INFINITY))
            .unwrap();

        assert!((branch.cap - (a.cap + b.cap)).abs() < femtofarads(1e-9));
        assert!((a.fanout - 2.0).abs() < 1e-12 && (b.fanout - 3.0).abs() < 1e-12);
        assert!((branch.fanout - 5.0).abs() < 1e-12);
        assert_eq!(branch.max_load_slew, a.max_slew.min(b.max_slew));
        assert_eq!(branch.loads.len(), 2);
    });
}

/// A split input over 100 microns of wire (200 000 units) that breaks no
/// limit, with the wire of 2 ohms and 0.1 fF a micron.
fn unbroken_wire() -> SplitInput {
    SplitInput {
        wire_res: OhmsPerMeter::from_ohms_per_micron(2.0),
        wire_cap: FaradsPerMeter::from_femtofarads_per_micron(0.1),
        length: Dbu::new(200_000),
        wire_length: Dbu::new(200_000),
        wire_length_ref: Dbu::ZERO,
        load_cap: femtofarads(15.0),
        ref_cap: femtofarads(5.0),
        max_cap: Farads::INFINITY,
        load_slew: Seconds::new(50e-12),
        max_load_slew_margined: Seconds::new(100e-12),
        r_drvr: Ohms::new(1000.0),
    }
}

#[test]
fn a_wire_over_the_length_limit_splits_at_the_limit_less_the_wire_beyond_and_at_most_half() {
    with_a_repairer("", |repairer| {
        repairer.resizer.options.max_wire_length = Some(Dbu::new(100_000));
        let split = |length: i64, wire_length: i64, beyond: i64| {
            repairer.split_length(SplitInput {
                length: Dbu::new(length),
                wire_length: Dbu::new(wire_length),
                wire_length_ref: Dbu::new(beyond),
                ..unbroken_wire()
            })
        };

        assert_eq!(split(150_000, 150_000, 20_000), Some(Dbu::new(75_000)));
        assert_eq!(split(150_000, 160_000, 60_000), Some(Dbu::new(40_000)));
        assert_eq!(split(150_000, 220_000, 120_000), Some(Dbu::ZERO));
    });
}

#[test]
fn a_wire_within_the_length_limit_is_not_split_for_it() {
    with_a_repairer("", |repairer| {
        repairer.resizer.options.max_wire_length = Some(Dbu::new(100_000));
        let split = |wire_length: i64| {
            repairer.split_length(SplitInput {
                wire_length: Dbu::new(wire_length),
                ..unbroken_wire()
            })
        };

        assert_eq!(split(100_000), None);
        assert_eq!(split(99_999), None);
        assert_eq!(split(100_001), Some(Dbu::new(100_000)));
    });
}

#[test]
fn a_load_over_the_limit_splits_the_wire_where_what_is_beyond_and_the_wire_fill_it() {
    with_a_repairer("", |repairer| {
        let split = |load_cap: Farads, max_cap: Farads, wire_cap: f64| {
            repairer.split_length(SplitInput {
                load_cap,
                max_cap,
                wire_cap: FaradsPerMeter::from_femtofarads_per_micron(wire_cap),
                ..unbroken_wire()
            })
        };

        // (15 - 5) fF at 0.1 fF a micron is 100 microns of 2000 units.
        assert_eq!(
            split(femtofarads(20.0), femtofarads(15.0), 0.1),
            Some(Dbu::new(200_000))
        );
        assert_eq!(split(femtofarads(15.0), femtofarads(15.0), 0.1), None);
        assert_eq!(split(femtofarads(20.0), femtofarads(30.0), 0.1), None);
        assert_eq!(split(femtofarads(20.0), femtofarads(15.0), 0.0), None);
    });
}

/// The meters of `units` database units of the session's grid of 2000 a micron.
fn within_one_unit(found: Option<Dbu>, expected_units: f64) -> bool {
    found.is_some_and(|dbu| (dbu.to_f64() - expected_units).abs() <= 3.0)
}

#[test]
fn a_load_slew_over_the_limit_splits_the_wire_where_the_slew_estimate_meets_it() {
    with_a_repairer("", |repairer| {
        repairer.resizer.slew_rc_factor = 1.0;
        let over = |wire_res: f64, wire_cap: f64| SplitInput {
            wire_res: OhmsPerMeter::from_ohms_per_micron(wire_res),
            wire_cap: FaradsPerMeter::from_femtofarads_per_micron(wire_cap),
            load_slew: Seconds::new(500e-12),
            max_load_slew_margined: Seconds::new(100e-12),
            ref_cap: femtofarads(5.0),
            ..unbroken_wire()
        };

        let quadratic = repairer.split_length(over(2.0, 0.1));
        let linear = repairer.split_length(over(0.0, 0.1));
        let flat = repairer.split_length(over(0.0, 0.0));

        // With r = wire ohms, c = wire farads a meter, R the driver, C the
        // load beyond and S the slew limit, the length l meets
        // r*c*l^2/2 + (R*c + r*C)*l + R*C = S.
        let (wire_ohms, wire_farads) = (2e6_f64, 1e-10_f64);
        let (driver_ohms, beyond, limit) = (1000.0_f64, 5e-15_f64, 100e-12_f64);
        let quad = wire_ohms * wire_farads / 2.0;
        let lin = wire_ohms.mul_add(beyond, driver_ohms * wire_farads);
        let constant = driver_ohms.mul_add(beyond, -limit);
        let root = (-lin + lin.mul_add(lin, -4.0 * quad * constant).sqrt()) / (2.0 * quad);
        assert!(
            within_one_unit(quadratic, root * 2000.0 * 1e6),
            "{quadratic:?} vs {root}"
        );
        let line = -constant / (driver_ohms * wire_farads);
        assert!(
            within_one_unit(linear, line * 2000.0 * 1e6),
            "{linear:?} vs {line}"
        );
        assert_eq!(flat, Some(Dbu::ZERO));
    });
}

#[test]
fn a_load_slew_exactly_at_the_limit_is_not_split() {
    with_a_repairer("", |repairer| {
        let at_limit = SplitInput {
            load_slew: Seconds::new(100e-12),
            max_load_slew_margined: Seconds::new(100e-12),
            ..unbroken_wire()
        };

        assert_eq!(repairer.split_length(at_limit), None);
    });
}

/// A wire of 0.1 fF a micron driven through 1000 ohms.
fn driver_wire(repairer: &Repairer<'_, '_, crate::sta::Sta>, farads_per_micron: f64) -> DriverWire {
    DriverWire {
        res: OhmsPerMeter::from_ohms_per_micron(2.0),
        cap: FaradsPerMeter::from_femtofarads_per_micron(farads_per_micron),
        r_drvr: Ohms::new(1000.0),
        slew_rc_factor: repairer.resizer.slew_rc_factor,
        dbu_per_micron: repairer.resizer.db.dbu(),
    }
}

/// The rest of a wire `length` long carrying `ref_cap_ff` that tolerates
/// slews far over any it could give.
fn lax_rest(length: i64, wire_length: i64, ref_cap_ff: f64) -> WireRest {
    WireRest {
        to: Point::new(Dbu::new(length), Dbu::ZERO),
        length: Dbu::new(length),
        wire_length: Dbu::new(wire_length),
        wire_length_ref: Dbu::ZERO,
        ref_cap: femtofarads(ref_cap_ff),
        max_load_slew: Seconds::new(1.0),
    }
}

#[test]
fn a_repeater_goes_short_of_the_split_by_a_twentieth_or_at_the_end_of_the_wire_where_it_all_must_split()
 {
    with_a_repairer("", |repairer| {
        let wire = driver_wire(repairer, 0.1);
        let rest = lax_rest(200_000, 200_000, 5.0);
        let distance = |max_cap: Farads| repairer.repeater_distance(&wire, &rest, max_cap);

        // 100 microns of 0.1 fF hold 10 fF; the load there is 15 fF.
        assert!((distance(femtofarads(10.0)) / 100_000.0 - 0.95).abs() < 1e-9);
        assert!((distance(femtofarads(14.999_99)) - 200_000.0).abs() < 1e-6);
        assert!((distance(Farads::INFINITY) - 200_000.0).abs() < 1e-6);
    });
}

#[test]
fn the_rest_of_a_wire_breaks_the_length_limit_only_past_it() {
    with_a_repairer("", |repairer| {
        repairer.resizer.options.max_wire_length = Some(Dbu::new(100));
        let wire = driver_wire(repairer, 0.1);
        let breaks = |wire_length: i64| {
            repairer.wire_breaks_limits(&wire, &lax_rest(50, wire_length, 0.0), Farads::INFINITY)
        };

        assert!(breaks(101));
        assert!(!breaks(100));
    });
}

#[test]
fn the_rest_of_a_wire_breaks_the_load_limit_only_where_the_wire_has_capacitance_and_the_limit_is_set()
 {
    with_a_repairer("", |repairer| {
        let rest = lax_rest(0, 0, 15.0);
        let breaks = |farads_per_micron: f64, max_cap: Farads| {
            let wire = driver_wire(repairer, farads_per_micron);
            repairer.wire_breaks_limits(&wire, &rest, max_cap)
        };

        assert!(breaks(0.1, femtofarads(10.0)));
        assert!(!breaks(0.1, rest.ref_cap));
        assert!(!breaks(0.0, femtofarads(10.0)));
        assert!(!breaks(0.1, Farads::ZERO));
    });
}

#[test]
fn the_rest_of_a_wire_breaks_the_slew_limit_beyond_it_only_past_it() {
    with_a_repairer("", |repairer| {
        let wire = driver_wire(repairer, 0.1);
        let ref_cap = femtofarads(2.0);
        let slew = wire.r_drvr * ref_cap * wire.slew_rc_factor;
        let breaks = |limit: Seconds| {
            let rest = WireRest {
                max_load_slew: limit,
                ref_cap,
                ..lax_rest(0, 0, 0.0)
            };
            repairer.wire_breaks_limits(&wire, &rest, Farads::INFINITY)
        };

        assert!(!breaks(slew));
        assert!(breaks(slew * 0.99));
        assert!(!breaks(slew * 1.01));
    });
}

/// One of each of the clock cells and a buffer of no drive resistance, in a
/// chain from the port `in`.
fn a_driver_of_each_kind() -> NetlistDB {
    crate::testing::design_with_copies(
        &def_text(
            "kinds",
            &DefSections {
                components: "- cb CLKBUF_X3 + PLACED ( 2000 2800 ) N ;\n\
                             - b BUF_X1 + PLACED ( 6000 2800 ) N ;\n\
                             - i INV_X1 + PLACED ( 10000 2800 ) N ;\n\
                             - g AND2_X1 + PLACED ( 14000 2800 ) N ;\n\
                             - f DFF_X1 + PLACED ( 18000 2800 ) N ;\n\
                             - s IDEAL + PLACED ( 24000 2800 ) N ;\n\
                             - sl SLOWSLEW + PLACED ( 30000 2800 ) N ;\n\
                             - fl FLATSLEW + PLACED ( 36000 2800 ) N ;\n",
                nets: "- in ( PIN in ) ( cb A ) ( g B ) ;\n- n1 ( cb Z ) ( b A ) ;\n\
                       - n2 ( b Z ) ( i A ) ;\n- n3 ( i ZN ) ( g A ) ;\n\
                       - n4 ( g Z ) ( f D ) ;\n- clk ( PIN clk ) ( f CK ) ;\n\
                       - n5 ( f Q ) ( s A ) ;\n- out ( s Z ) ( PIN out ) ;\n\
                       - slin ( PIN slin ) ( sl A ) ;\n- slout ( sl Z ) ( PIN slout ) ;\n\
                       - flin ( PIN flin ) ( fl A ) ;\n- flout ( fl Z ) ( PIN flout ) ;\n",
                inputs: &["in", "clk", "slin", "flin"],
                outputs: &["out", "slout", "flout"],
            },
        ),
        &[
            ("BUF_X1", "IDEAL"),
            ("BUF_X1", "SLOWSLEW"),
            ("BUF_X1", "FLATSLEW"),
        ],
    )
}

/// A buffer whose delay climbs steeply with its load and slew hardly at all,
/// so that the load for a slew is far past a guess made from the delay.
const SLOWSLEW_BUFFER: &str = "cell (SLOWSLEW) { area : 0.8 ;
        pin (A) { direction : input ; capacitance : 0.97 ; }
        pin (Z) { direction : output ; function : \"A\" ; max_capacitance : 6000.0 ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (delay_t) { values (\"0.1, 1.0\", \"0.1, 1.0\") ; }
            cell_fall (delay_t) { values (\"0.1, 1.0\", \"0.1, 1.0\") ; }
            rise_transition (delay_t) { values (\"0.01, 0.05\", \"0.01, 0.05\") ; }
            fall_transition (delay_t) { values (\"0.01, 0.05\", \"0.01, 0.05\") ; } } } }";

/// A buffer whose delay climbs with its load but whose slew is flat, so that
/// no load gives a slew over it.
const FLATSLEW_BUFFER: &str = "cell (FLATSLEW) { area : 0.8 ;
        pin (A) { direction : input ; capacitance : 0.97 ; }
        pin (Z) { direction : output ; function : \"A\" ; max_capacitance : 6000.0 ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (delay_t) { values (\"0.1, 1.0\", \"0.1, 1.0\") ; }
            cell_fall (delay_t) { values (\"0.1, 1.0\", \"0.1, 1.0\") ; }
            rise_transition (delay_t) { values (\"0.01, 0.01\", \"0.01, 0.01\") ; }
            fall_transition (delay_t) { values (\"0.01, 0.01\", \"0.01, 0.01\") ; } } } }";

/// A buffer whose delay and slew do not depend on its load.
const IDEAL_BUFFER: &str = "cell (IDEAL) { area : 0.8 ;
        pin (A) { direction : input ; capacitance : 0.97 ; }
        pin (Z) { direction : output ; function : \"A\" ; max_capacitance : 60.0 ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.05\") ; }
            cell_fall (scalar) { values (\"0.05\") ; }
            rise_transition (scalar) { values (\"0.02\") ; }
            fall_transition (scalar) { values (\"0.02\") ; } } } }";

#[test]
fn the_load_that_gives_a_driver_a_slew_is_as_found_for_each_kind_of_cell_and_slew() {
    let mut db = a_driver_of_each_kind();
    let drivers = ["cb/Z", "b/Z", "i/ZN", "g/Z", "f/Q", "sl/Z", "fl/Z"].map(|name| pin(&db, name));
    let ideal = pin(&db, "s/Z");
    let port = db.io_pins[db.io_pin_id("in").unwrap()].pin;
    let mut lines = String::new();

    with_wired_session(
        &mut db,
        &crate::testing::cells_with(&format!(
            "{IDEAL_BUFFER}\n{SLOWSLEW_BUFFER}\n{FLATSLEW_BUFFER}"
        )),
        "",
        |resizer| {
            let repairer = Repairer::new(resizer);
            for driver in drivers {
                for slew_ps in [5.0, 10.0, 20.0, 30.0, 40.0, 80.0, 150.0, 300.0] {
                    let slew = Seconds::new(slew_ps * 1e-12);
                    let found = repairer.slew_load_cap(driver, slew).unwrap();
                    let (lib, port) = repairer
                        .resizer
                        .link
                        .pin_port(repairer.resizer.db, driver)
                        .unwrap();
                    let cell = repairer.resizer.link.liberty.cell(lib);
                    let reached =
                        |cap: Farads| gate_slew(cell, port, &repairer.resizer.targets.slews, cap);
                    assert!(found >= Farads::ZERO);
                    // A slew no load reaches runs the search out to its cap.
                    if found > Farads::ZERO && found.get() < 1.0 {
                        assert!(reached(found) <= slew, "{slew_ps} ps");
                        assert!(reached(found * 1.011) >= slew, "{slew_ps} ps");
                    }
                    let _ = writeln!(
                        lines,
                        "{} {slew_ps} ps {:.4}",
                        repairer.resizer.db.pin_name(driver),
                        found.get() * 1e15
                    );
                }
            }
            assert_eq!(
                repairer.slew_load_cap(ideal, Seconds::new(50e-12)),
                Some(Farads::INFINITY)
            );
            assert_eq!(repairer.slew_load_cap(port, Seconds::new(50e-12)), None);
        },
    );

    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden");
    pare_common::testing::assert_golden(&dir, "b_slew_load_cap.txt", &lines);
}

/// The repeaters inserted by repairing `driver`'s net alone in `db`, wired
/// over the clock cells and `extra`, under `constraints`.
fn inserted_repairing_driver(
    mut db: NetlistDB,
    extra: &str,
    constraints: &str,
    driver: &str,
) -> RepairSummary {
    let driver = pin(&db, driver);
    let mut found = RepairSummary::default();
    with_wired_session(
        &mut db,
        &crate::testing::cells_with(extra),
        constraints,
        |resizer| {
            let mut repairer = Repairer::new(resizer);
            repairer.repair_driver(driver).unwrap();
            found = repairer.summary.clone();
        },
    );
    found
}

/// `driver_cell` placed at the origin driving a net of `master` placed at
/// the far corner of the die.
fn far_net(driver_cell: &str, driver_pins: (&str, &str), sink: &str, tie: bool) -> NetlistDB {
    let _ = tie;
    crate::testing::design_with_copies(
        &def_text(
            "far",
            &DefSections {
                components: &format!(
                    "- d {driver_cell} + PLACED ( 2000 2800 ) N ;\n\
                     - s {sink} + PLACED ( 780000 780000 ) N ;\n"
                ),
                nets: &format!("- n ( d {} ) ( s {} ) ;\n", driver_pins.0, driver_pins.1),
                inputs: &[],
                outputs: &[],
            },
        ),
        &[("BUF_X1", "TIEH")],
    )
}

#[test]
fn a_net_with_a_slew_over_its_limit_gets_repeaters_unless_it_is_a_supply_or_constant() {
    let tie = "cell (TIEH) { area : 0.5 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (Z) { direction : output ; function : \"1\" ; } }";
    let flop_to_flop = || far_net("DFF_X1", ("Q", "D"), "DFF_X1", false);
    let mut supply = flop_to_flop();
    let net = supply.net_id("n").unwrap();
    supply.nets[net].signal_use = SignalUse::Power;
    let tied = far_net("TIEH", ("Z", "D"), "DFF_X1", true);

    let plain = inserted_repairing_driver(flop_to_flop(), tie, LOAD_LIMIT, "d/Q");
    let of_a_supply = inserted_repairing_driver(supply, tie, LOAD_LIMIT, "d/Q");
    let of_a_constant = inserted_repairing_driver(tied, tie, LOAD_LIMIT, "d/Z");

    assert!(plain.inserted > 0 && plain.cap_violations > 0);
    assert_eq!(of_a_supply, RepairSummary::default());
    assert_eq!(of_a_constant, RepairSummary::default());
}

/// A design load limit that every long net breaks.
const LOAD_LIMIT: &str = "set_max_capacitance 1.0 [current_design]\n";

/// A clock buffer `cb` next to its flop and another flop `far` away that
/// only data reaches, from the first.
fn clock_near_data_far() -> NetlistDB {
    design(&def_text(
        "clock_near",
        &DefSections {
            components: "- cb BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                         - f DFF_X1 + PLACED ( 6000 2800 ) N ;\n\
                         - far DFF_X1 + PLACED ( 780000 780000 ) N ;\n",
            nets: "- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( f CK ) ;\n\
                   - q ( f Q ) ( far D ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

#[test]
fn a_long_clock_net_is_not_repaired_as_a_data_net_and_a_clock_net_is_repaired_by_length_alone() {
    let mut data = clock_near_data_far();
    let mut clock = across_the_die_clock();
    let constraints = "create_clock -name clk -period 1.0 [get_ports clk]\n";
    let lib = cells(&clock_cells_source());
    let mut found = (usize::MAX, usize::MAX);

    with_wired_session(&mut data, &lib, constraints, |resizer| {
        let cells_before = resizer.db.cells.len();
        let buffer = resizer.buffers.lowest_drive;
        let repaired = Repairer::new(resizer)
            .repair_clock_nets(&[buffer], Some(Dbu::new(100_000)))
            .unwrap();
        found.0 = repaired.inserted + (resizer.db.cells.len() - cells_before);
    });
    let propagated = format!("{constraints}set_propagated_clock [all_clocks]\n");
    with_wired_session(&mut clock, &lib, &propagated, |resizer| {
        let buffer = resizer.buffers.lowest_drive;
        let repaired = Repairer::new(resizer)
            .repair_clock_nets(&[buffer], Some(Dbu::new(100_000)))
            .unwrap();
        found.1 = repaired.inserted;
    });

    assert_eq!(found.0, 0);
    assert!(found.1 > 0);
}

/// A clock buffer driving a flop at the far corner.
fn across_the_die_clock() -> NetlistDB {
    design(&def_text(
        "clock_far",
        &DefSections {
            components: "- cb BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                         - f DFF_X1 + PLACED ( 780000 780000 ) N ;\n",
            nets: "- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( f CK ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ))
}

#[test]
fn an_input_port_on_no_instance_is_still_checked_against_the_slew_limit() {
    let mut db = design(&def_text(
        "dangling",
        &DefSections {
            components: "- f DFF_X1 + PLACED ( 2000 2800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( f CK ) ;\n",
            inputs: &["clk", "boot"],
            outputs: &[],
        },
    ));
    let boot = db.io_pins[db.io_pin_id("boot").unwrap()].pin;
    let constraints = "set_max_transition 0.1 [current_design]\n\
                       set_input_transition 0.5 [get_ports boot]\n";

    with_wired_session(
        &mut db,
        &cells(&clock_cells_source()),
        constraints,
        |resizer| {
            let mut repairer = Repairer::new(resizer);

            repairer.repair_driver(boot).unwrap();

            assert_eq!(repairer.summary.slew_violations, 1);
        },
    );
}

#[test]
fn a_net_whose_longest_wire_is_the_length_limit_is_not_too_long() {
    let mut db = apart();
    let constraints = "create_clock -name clk -period 10.0 [get_ports clk]\n";

    with_wired_session(
        &mut db,
        &cells(&clock_cells_source()),
        constraints,
        |resizer| {
            let driver = pin(resizer.db, "f1/Q");
            let longest = resizer
                .buffered_tree(driver)
                .unwrap()
                .unwrap()
                .max_load_wire_length();
            let violations = |resizer: &mut Resizer<'_, crate::sta::Sta>, limit: Dbu| {
                resizer.set_max_wire_length(Some(limit));
                let mut repairer = Repairer::new(resizer);
                repairer.repair_net(driver, Checks::LengthOnly).unwrap();
                repairer.summary.length_violations
            };

            assert!(longest > Dbu::ZERO);
            assert_eq!(violations(resizer, longest), 0);
            assert_eq!(violations(resizer, longest - Dbu::new(1)), 1);
        },
    );
}

/// What `check_limits` makes of `f1/Q` of [`apart`] under `constraints`
/// after `adjust`, and what it counted.
fn limits_checked(
    constraints: &str,
    adjust: impl FnOnce(&mut Resizer<'_, crate::sta::Sta>, PinId),
) -> (NetNeeds, RepairSummary) {
    let mut db = apart();
    let mut found = None;
    with_wired_session(
        &mut db,
        &cells(&clock_cells_source()),
        constraints,
        |resizer| {
            let driver = pin(resizer.db, "f1/Q");
            adjust(resizer, driver);
            let mut repairer = Repairer::new(resizer);
            let needs = repairer.check_limits(driver).unwrap();
            found = Some((needs, repairer.summary.clone()));
        },
    );
    found.unwrap()
}

#[test]
fn a_driver_whose_slew_is_exactly_its_limit_does_not_violate_it() {
    let (needs, summary) = limits_checked(SLOW_CLOCK, |resizer, driver| {
        let limit = resizer.driver_slew_limit(driver).unwrap();
        resizer
            .graph
            .annotate_slew(driver, resizer.margined_slew(limit));
    });

    assert_eq!(needs, NetNeeds::default());
    assert_eq!(summary.slew_violations, 0);
}

#[test]
fn a_driver_whose_slew_stays_over_its_limit_after_resizing_needs_its_net_buffered() {
    let tight = format!("{SLOW_CLOCK}set_max_transition 0.02 [current_design]\n");

    let (needs, summary) = limits_checked(&tight, |_, _| {});

    assert!(needs.max_cap.is_some(), "{needs:?}");
    assert_eq!((summary.slew_violations, summary.resized), (1, 0));
}

#[test]
fn a_load_exactly_at_its_driver_limit_does_not_violate_it() {
    let source = clock_cells_source();
    let start = source.find("cell (DFF_X1)").unwrap();
    let end = start + source[start..].find("cell (DFF_SLOW)").unwrap();
    let edited = source[start..end].replacen("max_capacitance : 60.0", "max_capacitance : 1.1", 1);
    let lib = cells(&format!("{}{edited}{}", &source[..start], &source[end..]));
    let mut db = apart();
    let mut found = None;

    with_timed_session(&mut db, &lib, SLOW_CLOCK, |resizer| {
        let driver = pin(resizer.db, "f1/Q");
        assert_eq!(resizer.cap_limit(driver), Some(resizer.load_cap(driver)));
        let mut repairer = Repairer::new(resizer);
        let needs = repairer.check_limits(driver).unwrap();
        found = Some((needs, repairer.summary.cap_violations));
    });

    assert_eq!(found, Some((NetNeeds::default(), 0)));
}

#[test]
fn a_load_whose_slew_is_exactly_its_limit_does_not_violate_it() {
    let mut db = apart();

    with_timed_session(
        &mut db,
        &cells(&clock_cells_source()),
        SLOW_CLOCK,
        |resizer| {
            let (driver, load) = (pin(resizer.db, "f1/Q"), pin(resizer.db, "f2/D"));
            let limit = resizer.margined_slew(resizer.load_slews(driver)[0].1);

            resizer.graph.annotate_slew(load, limit);
            let at_limit = Repairer::new(resizer).loads_violate_slew(driver);
            resizer.graph.annotate_slew(load, limit * 1.01);
            let over = Repairer::new(resizer).loads_violate_slew(driver);

            assert!(!at_limit);
            assert!(over);
        },
    );
}

/// An inverter that loads its driver with 500 fF, in a class of its own.
const HEAVY_INVERTER: &str = "cell (HEAVY_INVERTER) { area : 0.9 ;
        pin (A) { direction : input ; capacitance : 500.0 ; }
        pin (ZN) { direction : output ; function : \"!A\" ;
          timing () { related_pin : \"A\" ; timing_sense : negative_unate ;
            cell_rise (scalar) { values (\"0.05\") ; }
            cell_fall (scalar) { values (\"0.05\") ; } } } }";

/// A buffer of `area` square microns whose slew is `slew_ns` and limit
/// `limit_ns` whatever it drives.
fn sized_buffer(name: &str, area: f64, slew_ns: f64, limit_ns: f64) -> String {
    format!(
        "cell ({name}) {{ area : {area} ;
          pin (A) {{ direction : input ; capacitance : 0.97 ; }}
          pin (Z) {{ direction : output ; function : \"A\" ; max_capacitance : 600.0 ;
            max_transition : {limit_ns} ;
            timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
              cell_rise (scalar) {{ values (\"0.05\") ; }}
              cell_fall (scalar) {{ values (\"0.05\") ; }}
              rise_transition (scalar) {{ values (\"{slew_ns}\") ; }}
              fall_transition (scalar) {{ values (\"{slew_ns}\") ; }} }} }} }}"
    )
}

/// The size the driver `b` of a heavy load becomes under `limit_ns`, among
/// sizes of 0.3, 0.5 and 0.7 square microns that slew 0.04, 0.03 and 0.02 ns,
/// listed in the library from the largest.
fn size_repaired_for(limit_ns: f64) -> String {
    let mut db = crate::testing::design_with_copies(
        &def_text(
            "sizes",
            &DefSections {
                components: "- b BUF_L + PLACED ( 2000 2800 ) N ;\n\
                             - s HEAVY_INVERTER + PLACED ( 6000 2800 ) N ;\n",
                nets: "- in ( PIN in ) ( b A ) ;\n- n ( b Z ) ( s A ) ;\n",
                inputs: &["in"],
                outputs: &[],
            },
        ),
        &[
            ("BUF_X1", "BUF_L"),
            ("BUF_X1", "BUF_M"),
            ("BUF_X1", "BUF_S"),
            ("INV_X1", "HEAVY_INVERTER"),
        ],
    );
    let extra = format!(
        "{}\n{}\n{}\n{HEAVY_INVERTER}",
        sized_buffer("BUF_L", 0.7, 0.02, limit_ns),
        sized_buffer("BUF_M", 0.5, 0.03, limit_ns),
        sized_buffer("BUF_S", 0.3, 0.04, limit_ns),
    );
    let mut size = String::new();
    with_timed_session(
        &mut db,
        &crate::testing::cells_with(&extra),
        "",
        |resizer| {
            let driver = pin(resizer.db, "b/Z");
            let _ = Repairer::new(resizer).repair_driver_slew(driver).unwrap();
            let b = resizer.db.cell_id("b").unwrap();
            size = resizer.link.cell(resizer.db, b).unwrap().1.name.clone();
        },
    );
    size
}

#[test]
fn the_smallest_size_that_meets_the_slew_limit_is_chosen_and_where_none_does_the_least_violating() {
    assert_eq!(size_repaired_for(0.05), "BUF_S");
    assert_eq!(size_repaired_for(0.035), "BUF_M");
    assert_eq!(size_repaired_for(0.01), "BUF_L");
}

/// How far past the library's limit the slew out of `port` of the cell
/// `name` is, driving `load_ff` from `input_slews` under the session, as
/// `slew_violation` finds it, and as the slews of the cell's arcs say.
fn violation_of(
    extra: &str,
    name: &str,
    port: &str,
    input_slews: &[(&str, EdgeValues<Seconds>)],
    load_ff: f64,
) -> (Option<Seconds>, Seconds) {
    let mut db = apart();
    let slews: PortSlews<'_> = input_slews.iter().copied().collect();
    let mut found = None;
    with_timed_session(
        &mut db,
        &crate::testing::cells_with(extra),
        SLOW_CLOCK,
        |resizer| {
            let liberty = resizer.link.liberty;
            let id = liberty.cell_id(name).unwrap();
            let cell = liberty.cell(id);
            let port_id = cell.port(port).unwrap();
            let load = femtofarads(load_ff);
            let repairer = Repairer::new(resizer);
            let found_here = repairer.slew_violation(id, port, &slews, load);

            let given = EdgeValues {
                rise: input_slews
                    .iter()
                    .map(|(_, s)| s.rise)
                    .fold(Seconds::ZERO, Seconds::max),
                fall: input_slews
                    .iter()
                    .map(|(_, s)| s.fall)
                    .fold(Seconds::ZERO, Seconds::max),
            };
            let (_, out) = gate_delays(cell, port_id, load, &given, None);
            let limit = liberty.slew_limit(cell, &cell.ports[port_id]).unwrap();
            found = Some((
                found_here,
                (out.rise.max(out.fall) - limit).max(Seconds::ZERO),
            ));
        },
    );
    found.unwrap()
}

#[test]
fn a_registers_clock_to_output_slew_is_found_from_a_zero_input_slew_whatever_the_clock_slew() {
    let slews = [("CK", EdgeValues::both(Seconds::new(0.15e-9)))];

    let (found, expected) = violation_of("", "DFF_X1", "Q", &[], 80.0);
    let (with_clock_slew, _) = violation_of("", "DFF_X1", "Q", &slews, 80.0);

    assert!(expected > Seconds::new(0.05e-9), "{expected:?}");
    assert_eq!(found, Some(expected));
    assert_eq!(with_clock_slew, Some(expected));
}

#[test]
fn a_gates_output_slew_is_found_from_the_slew_at_its_input() {
    let slews = [(
        "A",
        EdgeValues {
            rise: Seconds::new(0.15e-9),
            fall: Seconds::new(0.05e-9),
        },
    )];

    let (found, expected) = violation_of("", "BUF_X1", "Z", &slews, 80.0);
    let (without, _) = violation_of("", "BUF_X1", "Z", &[], 80.0);

    assert_eq!(found, Some(expected));
    assert_ne!(found, without);
}

/// A cell of `name` with an input `A` and outputs `B` and `Z` whose slews
/// are `b_ns` and `z_ns`.
fn two_slewed_outputs(name: &str, b_ns: f64, z_ns: f64) -> String {
    format!(
        "cell ({name}) {{ area : 1.0 ;
          pin (A) {{ direction : input ; capacitance : 1.0 ; }}
          pin (B) {{ direction : output ; function : \"A\" ;
            timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
              cell_rise (scalar) {{ values (\"0.05\") ; }}
              cell_fall (scalar) {{ values (\"0.05\") ; }}
              rise_transition (scalar) {{ values (\"{b_ns}\") ; }}
              fall_transition (scalar) {{ values (\"{b_ns}\") ; }} }} }}
          pin (Z) {{ direction : output ; function : \"A\" ;
            timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
              cell_rise (scalar) {{ values (\"0.05\") ; }}
              cell_fall (scalar) {{ values (\"0.05\") ; }}
              rise_transition (scalar) {{ values (\"{z_ns}\") ; }}
              fall_transition (scalar) {{ values (\"{z_ns}\") ; }} }} }} }}"
    )
}

#[test]
fn an_outputs_slew_violation_counts_only_the_arcs_into_that_output() {
    let cell = two_slewed_outputs("TWO", 0.01, 0.30);

    let (calm, _) = violation_of(&cell, "TWO", "B", &[], 1.0);
    let (loud, _) = violation_of(&cell, "TWO", "Z", &[], 1.0);

    assert_eq!(calm, Some(Seconds::ZERO));
    assert!(
        loud.is_some_and(|v| (v.get() - 0.1e-9).abs() < 1e-18),
        "{loud:?}"
    );
}

/// The clock cells with their buffers protected from resizing, and `extra`.
fn untouchable_cells(extra: &str) -> Liberty {
    let source = clock_cells_source()
        .replacen("area : 1.33 ;", "area : 1.33 ; dont_touch : true ;", 1)
        .replacen("area : 0.8 ;", "area : 0.8 ; dont_touch : true ;", 1);
    let end = source.rfind('}').unwrap();
    cells(&format!("{}{extra}\n}}\n", &source[..end]))
}

/// An inverter, in a class of its own, whose input loads `ff` femtofarads.
fn inverter_of(name: &str, ff: f64) -> String {
    HEAVY_INVERTER
        .replace("HEAVY_INVERTER", name)
        .replace("capacitance : 500.0", &format!("capacitance : {ff}"))
}

/// A buffer `d` driving the two sinks `s1` and `s2` that load `a_ff` and `b_ff`
/// femtofarads; what a junction of the two repairs with a driver that can
/// only take `max_cap_ff`: the buffer put in and the ones the sum and the
/// difference of the loads call for.
fn junction_buffer(a_ff: f64, b_ff: f64, max_cap_ff: f64) -> (String, String, String) {
    let mut db = crate::testing::design_with_copies(
        &def_text(
            "junction",
            &DefSections {
                components: "- d BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - s1 SINK_A + PLACED ( 6000 2800 ) N ;\n\
                             - s2 SINK_B + PLACED ( 6000 20000 ) N ;\n",
                nets: "- in ( PIN in ) ( d A ) ;\n- n ( d Z ) ( s1 A ) ( s2 A ) ;\n",
                inputs: &["in"],
                outputs: &[],
            },
        ),
        &[("INV_X1", "SINK_A"), ("INV_X1", "SINK_B")],
    );
    let extra = format!(
        "{}\n{}",
        inverter_of("SINK_A", a_ff),
        inverter_of("SINK_B", b_ff)
    );
    let mut found = None;
    with_wired_session(&mut db, &untouchable_cells(&extra), "", |resizer| {
        let at = Point::new(Dbu::new(4000), Dbu::new(2800));
        let load = |name: &str| {
            let pin = pin(resizer.db, name);
            bnet::Bnet::load(at, pin, bnet::LoadModel::load(&*resizer, pin))
        };
        let tree = bnet::Bnet::junction(at, load("s1/A"), load("s2/A"));
        let lowest = resizer.buffers.lowest_drive;
        let name =
            |resizer: &Resizer<'_, crate::sta::Sta>, id| resizer.link.liberty.cell(id).name.clone();
        let sum = resizer
            .find_target_buffer(lowest, femtofarads(a_ff + b_ff))
            .unwrap();
        let diff = resizer
            .find_target_buffer(lowest, femtofarads((a_ff - b_ff).abs()))
            .unwrap();
        let (sum, diff) = (name(resizer, sum.id), name(resizer, diff.id));
        let mut repairer = Repairer::new(resizer);
        let driver = tree_driver(0.0, max_cap_ff);

        let _ = repairer.repair_branch(&tree, driver).unwrap();

        let inserted = repairer.resizer.db.cell_id("max_cap1").unwrap();
        let placed = repairer
            .resizer
            .link
            .cell(repairer.resizer.db, inserted)
            .unwrap()
            .1
            .name
            .clone();
        found = Some((placed, sum, diff));
    });
    found.unwrap()
}

#[test]
fn a_junctions_repeater_is_the_buffer_size_for_the_load_of_both_branches() {
    let (placed, for_the_sum, for_the_difference) = junction_buffer(30.0, 10.0, 1.0);

    assert_ne!(for_the_sum, for_the_difference);
    assert_eq!(placed, for_the_sum);
}

#[test]
fn a_via_lowers_the_load_slew_limit_by_the_slew_its_resistance_adds_to_the_load() {
    with_a_repairer("", |repairer| {
        let at = Point::new(Dbu::new(2000), Dbu::new(2800));
        let pin = pin(repairer.resizer.db, "f2/D");
        let model = bnet::LoadModel::load(&*repairer.resizer, pin);
        let load = bnet::Bnet::load(at, pin, model);

        let branch = repairer
            .repair_via(&load, Ohms::new(100.0), tree_driver(0.0, f64::INFINITY))
            .unwrap();

        // 100 ohms of the via, the load's capacitance and the factor of 2.
        let drop = Seconds::new(100.0 * model.cap.get() * 2.0);
        assert!((branch.max_load_slew - (model.max_slew - drop)).abs() < Seconds::new(1e-24));
        assert_eq!(branch.cap, model.cap);
    });
}
