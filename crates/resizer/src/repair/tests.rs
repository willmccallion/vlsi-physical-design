use super::*;
use crate::resizer::GlobalRouting;
use crate::route_parasitics::RouteRc;
use crate::testing::{
    DefSections, cells, clock_cells_source, def_text, design, pin, with_wired_session,
};
use crate::testing::{metal2_wires, with_session_inputs};
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::{
    NetRoute, PinGridLocation, RoutePoint, RouteSegment, RouteSource,
};
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::liberty::{Edge, MinMax};

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
