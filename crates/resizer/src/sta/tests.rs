use super::*;
use crate::graph::{Estimator, PortConditions};
use crate::library::Link;
use crate::parasitics::WireRc;
use crate::testing::{
    DefSections, cells_with, def_text, design, design_with_copies, with_timed_session,
};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::LayerId;
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::{def, lef, liberty, sdc};
use pare_common::db::sdc::Clock;
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

fn clock(name: &str, period: f64, rise: f64, fall: f64) -> Clock {
    Clock {
        name: name.into(),
        period: Seconds::new(period),
        rise: Seconds::new(rise),
        fall: Seconds::new(fall),
        sources: Vec::new(),
        uncertainty: SideValues::both(None),
        propagated: false,
    }
}

fn edge(clock: u32, edge: Edge) -> ClockEdge {
    ClockEdge {
        clock: ClockId(clock),
        edge,
    }
}

#[test]
fn same_clock_captures_a_period_later_for_setup_and_at_once_for_hold() {
    let sdc = Sdc {
        clocks: vec![clock("clk", 2e-9, 0.0, 1e-9)].into(),
        ..Sdc::default()
    };

    let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Rise));

    assert!((acct.setup_required.get() - 2e-9).abs() < 1e-20);
    assert!(acct.hold_required.get().abs() < 1e-20);
}

#[test]
fn a_gated_clocks_hold_check_is_the_trailing_edge_of_the_same_cycle() {
    let sdc = Sdc {
        clocks: vec![clock("clk", 2e-9, 0.0, 1e-9)].into(),
        ..Sdc::default()
    };

    let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Fall));

    assert!((acct.gated_hold_required.get() - 1e-9).abs() < 1e-20);
    assert!((acct.required(MinMax::Min, Check::GatedClock).get() - 1e-9).abs() < 1e-20);
    assert!((acct.required(MinMax::Min, Check::Ordinary).get() + 1e-9).abs() < 1e-20);
}

#[test]
fn a_falling_capture_of_a_rising_launch_is_half_a_period_away() {
    let sdc = Sdc {
        clocks: vec![clock("clk", 2e-9, 0.0, 1e-9)].into(),
        ..Sdc::default()
    };

    let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Fall));

    assert!((acct.setup_required.get() - 1e-9).abs() < 1e-20);
    assert!((acct.hold_required.get() + 1e-9).abs() < 1e-20);
}

#[test]
fn a_latch_opening_on_the_fall_is_required_at_the_fall_before_the_close() {
    let sdc = Sdc {
        clocks: vec![clock("clk", 2.2e-9, 0.0, 1.1e-9)].into(),
        ..Sdc::default()
    };

    let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Fall));

    assert!((acct.latch_setup_required.get() - 1.1e-9).abs() < 1e-20);
    assert_eq!(acct.latch_cycles, (0, 0));
}

#[test]
fn a_fast_clock_captures_a_slow_launch_at_its_next_edge() {
    let sdc = Sdc {
        clocks: vec![
            clock("slow", 4e-9, 0.0, 2e-9),
            clock("fast", 1e-9, 0.0, 0.5e-9),
        ]
        .into(),
        ..Sdc::default()
    };

    let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(1, Edge::Rise));

    assert!((acct.setup_required.get() - 1e-9).abs() < 1e-20);
    assert!(acct.hold_required.get().abs() < 1e-20);
}

#[test]
fn a_launch_without_a_period_is_captured_in_the_first_cycle() {
    let sdc = Sdc {
        clocks: vec![
            clock("virtual", 0.0, 0.0, 0.0),
            clock("clk", 2e-9, 0.0, 1e-9),
        ]
        .into(),
        ..Sdc::default()
    };

    let falling = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(1, Edge::Fall));
    let rising = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(1, Edge::Rise));

    assert!((falling.setup_required.get() - 1e-9).abs() < 1e-20);
    assert!((falling.hold_required.get() - 1e-9).abs() < 1e-20);
    assert!((rising.setup_required.get() - 2e-9).abs() < 1e-20);
    assert!(rising.hold_required.get().abs() < 1e-20);
    assert_eq!(rising.latch_cycles, (0, 1));
}

#[test]
fn clocks_without_a_common_period_are_flagged() {
    let sdc = Sdc {
        clocks: vec![
            clock("a", 1e-9, 0.0, 0.5e-9),
            clock("b", 2f64.sqrt() * 1e-9, 0.0, 0.5e-9),
        ]
        .into(),
        ..Sdc::default()
    };

    let unrelated = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(1, Edge::Rise));
    let same = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Rise));

    assert!(unrelated.max_cycles_exceeded);
    assert!(!same.max_cycles_exceeded);
}

fn fixture(crate_name: &str, name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join(crate_name)
        .join("tests/fixtures")
        .join(name)
}

/// A flop clocked and fed from ports, beside an input port on nothing.
fn dangling_port_design() -> NetlistDB {
    let text = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN dangle ;\n\
            UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\n\
            COMPONENTS 1 ;\n- f DFF_X1 + PLACED ( 2000 2800 ) N ;\nEND COMPONENTS\n\
            PINS 3 ;\n- clk + NET clk + DIRECTION INPUT ;\n- in + NET in + DIRECTION INPUT ;\n\
            - boot + NET boot + DIRECTION INPUT ;\nEND PINS\n\
            NETS 3 ;\n- clk ( PIN clk ) ( f CK ) ;\n- in ( PIN in ) ( f D ) ;\n\
            - boot ( PIN boot ) ;\nEND NETS\nEND DESIGN\n";
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("cts", "clock.lef")], units).unwrap();
    def::parse_str(text, Path::new("dangle.def"), library).unwrap()
}

/// Two flops on one clock, the first launching into the second.
fn flop_to_flop_design() -> NetlistDB {
    let text = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN pair ;\n\
            UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\n\
            COMPONENTS 2 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
            - f2 DFF_X1 + PLACED ( 12000 2800 ) N ;\nEND COMPONENTS\n\
            PINS 2 ;\n- clk + NET clk + DIRECTION INPUT ;\n- in + NET in + DIRECTION INPUT ;\n\
            END PINS\nNETS 3 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- in ( PIN in ) ( f1 D ) ;\n\
            - q ( f1 Q ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n";
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("cts", "clock.lef")], units).unwrap();
    def::parse_str(text, Path::new("pair.def"), library).unwrap()
}

fn cell_pin(db: &NetlistDB, name: &str) -> PinId {
    let (cell, _) = name.split_once('/').unwrap();
    db.cells[db.cell_id(cell).unwrap()]
        .pins
        .iter()
        .copied()
        .find(|&pin| db.pin_name(pin) == name)
        .unwrap()
}

/// The slack at `pin` on `side` with the design timed under `constraints`
/// and wires without resistance or capacitance.
fn slack_under(db: &NetlistDB, constraints: &str, pin: PinId, side: MinMax) -> f64 {
    timed(db, constraints, |_, sta| {
        sta.slack(pin, side).time().unwrap().get()
    })
}

/// Whether `constraints` constrain `pin` on `side`.
fn constrained_under(db: &NetlistDB, constraints: &str, pin: PinId, side: MinMax) -> bool {
    timed(db, constraints, |_, sta| {
        sta.slack(pin, side).time().is_some()
    })
}

/// What `inspect` reads from the design timed under `constraints` with
/// wires without resistance or capacitance.
fn timed<R>(db: &NetlistDB, constraints: &str, inspect: impl FnOnce(&Netlist<'_>, &Sta) -> R) -> R {
    let lib = liberty::parse_file(&fixture("cts", "clock.lib")).unwrap();
    timed_with(db, &lib, constraints, inspect)
}

/// What `inspect` reads from the design timed with the cells of `lib`
/// under `constraints` with wires without resistance or capacitance.
fn timed_with<R>(
    db: &NetlistDB,
    lib: &Liberty,
    constraints: &str,
    inspect: impl FnOnce(&Netlist<'_>, &Sta) -> R,
) -> R {
    timed_in_full(db, lib, constraints, |netlist, _, _, sta| {
        inspect(netlist, sta)
    })
}

/// What `inspect` reads from the design, its timing graph, clocks and
/// analysis, timed as [`timed_with`] does.
fn timed_in_full<R>(
    db: &NetlistDB,
    lib: &Liberty,
    constraints: &str,
    inspect: impl FnOnce(&Netlist<'_>, &TimingGraph, &ClockNetwork, &Sta) -> R,
) -> R {
    let sdc = sdc::parse_str(constraints, Path::new("test.sdc"), db, lib).unwrap();
    let link = Link::new(db, lib, &[]).unwrap();
    let netlist = Netlist::new(db, &link);
    let constants = Constants::find(&netlist);
    let clocks = ClockNetwork::from_sdc(&netlist, &sdc, &constants);
    let rc = WireRc {
        resistance: OhmsPerMeter::ZERO,
        capacitance: FaradsPerMeter::ZERO,
        layer: LayerId(1),
    };
    let graph = TimingGraph::new(
        &netlist,
        &clocks,
        &constants,
        rc,
        None,
        Estimator::Placement,
        PortConditions::from_sdc(&sdc),
    );
    let sta = Sta::new(&netlist, &graph, &clocks, &constants, &sdc);
    inspect(&netlist, &graph, &clocks, &sta)
}

#[test]
fn an_input_port_on_nothing_keeps_the_negative_of_its_input_delay_as_slack() {
    let db = dangling_port_design();
    let constraints = "create_clock -name clk -period 2.2 [get_ports clk]\n\
            set_input_delay 0.44 -clock clk [get_ports in]\n\
            set_input_delay 0.44 -clock clk [get_ports boot]\n";
    let boot = db.io_pins[db.io_pin_id("boot").unwrap()].pin;

    let slack = slack_under(&db, constraints, boot, MinMax::Max);

    assert!((slack + 0.44e-9).abs() < 1e-15, "{slack}");
}

#[test]
fn a_setup_multicycle_moves_the_hold_check_by_the_extra_cycles() {
    let db = flop_to_flop_design();
    let clock = "create_clock -name clk -period 2.2 [get_ports clk]\n";
    let multicycle = format!(
        "{clock}set_multicycle_path 3 -setup -from [get_clocks clk] -to [get_clocks clk]\n"
    );
    let d = cell_pin(&db, "f2/D");

    let single = slack_under(&db, clock, d, MinMax::Min);
    let tripled = slack_under(&db, &multicycle, d, MinMax::Min);

    assert!(
        (single - tripled - 4.4e-9).abs() < 1e-15,
        "{single} vs {tripled}"
    );
}

#[test]
fn a_clock_used_as_data_has_its_check_slack_at_the_pin_and_a_path() {
    let text = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN cad ;\n\
            UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\n\
            COMPONENTS 1 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\nEND COMPONENTS\n\
            PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
            NETS 1 ;\n- clk ( PIN clk ) ( f1 CK ) ( f1 D ) ;\nEND NETS\nEND DESIGN\n";
    let units = DbuPerMicron::new(2000).unwrap();
    let library = lef::parse_files(&[fixture("cts", "clock.lef")], units).unwrap();
    let db = def::parse_str(text, Path::new("cad.def"), library).unwrap();
    let d = cell_pin(&db, "f1/D");
    let constraints = "create_clock -name clk -period 2.2 [get_ports clk]\n";

    let (pin_slack, worst, path) = timed(&db, constraints, |netlist, sta| {
        (
            sta.slack(d, MinMax::Max),
            sta.worst_slack(MinMax::Max),
            sta.worst_slack_path(netlist, d, MinMax::Max),
        )
    });

    let pin_slack = pin_slack.time().unwrap();
    assert_eq!(
        worst,
        Some(EndpointSlack {
            pin: d,
            slack: pin_slack
        })
    );
    assert_eq!(path.map(|p| p.slack), Some(pin_slack));
}

/// A latch transparent while `CK` is high, whose setup before the
/// closing fall is negative for rising data and positive for falling
/// data, with a tenth of a nanosecond from `D` and from `CK` to `Q`.
const LATCH: &str = "cell (LAT) { area : 4.5 ;
        latch (\"IQ\", \"IQN\") { data_in : \"D\" ; enable : \"CK\" ; }
        pin (D) { direction : input ; capacitance : 1.0 ;
          timing () { related_pin : \"CK\" ; timing_type : setup_falling ;
            rise_constraint (scalar) { values (\"-0.05\") ; }
            fall_constraint (scalar) { values (\"0.08\") ; } } }
        pin (CK) { direction : input ; clock : true ; capacitance : 1.0 ; }
        pin (Q) { direction : output ; function : \"IQ\" ;
          timing () { related_pin : \"D\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } }
          timing () { related_pin : \"CK\" ; timing_type : rising_edge ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } } } }";

/// The latch `l` of [`LATCH`] clocked from `clk`, its data from the
/// port `in`, its output to the port `out`.
fn latch_design() -> NetlistDB {
    design_with_copies(
        &def_text(
            "latched",
            &DefSections {
                components: "- l LAT + PLACED ( 2000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( l CK ) ;\n- in ( PIN in ) ( l D ) ;\n\
                           - out ( l Q ) ( PIN out ) ;\n",
                inputs: &["clk", "in"],
                outputs: &["out"],
            },
        ),
        &[("DFF_X1", "LAT")],
    )
}

#[test]
fn data_borrows_up_to_the_setup_of_its_own_edge_before_the_latch_closes() {
    let db = latch_design();
    let q = cell_pin(&db, "l/Q");
    let constraints = "create_clock -name clk -period 2.0 [get_ports clk]\n\
                           set_input_delay 0.98 -clock clk [get_ports in]\n";

    let (rise, fall) = timed_with(&db, &cells_with(LATCH), constraints, |_, sta| {
        (
            sta.arrival(q, Edge::Rise, MinMax::Max).unwrap(),
            sta.arrival(q, Edge::Fall, MinMax::Max).unwrap(),
        )
    });

    assert!((rise.get() - 1.08e-9).abs() < 1e-15, "{rise:?}");
    assert!((fall.get() - 1.02e-9).abs() < 1e-15, "{fall:?}");
}

/// A buffer as `BUF_X1` but a tenth of a nanosecond slower.
const SLOW_BUFFER: &str = "cell (BUF_SLOW) { area : 0.8 ;
        pin (A) { direction : input ; capacitance : 0.97 ; }
        pin (Z) { direction : output ; function : \"A\" ; max_capacitance : 60.0 ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (delay_t) { values (\"0.13, 0.40\", \"0.14, 0.41\") ; }
            cell_fall (delay_t) { values (\"0.13, 0.40\", \"0.14, 0.41\") ; }
            rise_transition (delay_t) { values (\"0.01, 0.40\", \"0.02, 0.41\") ; }
            fall_transition (delay_t) { values (\"0.01, 0.40\", \"0.02, 0.41\") ; } } } }";

#[test]
fn a_borrowing_latch_requires_its_data_where_an_upstream_resize_moved_it() {
    let mut db = design_with_copies(
        &def_text(
            "borrow",
            &DefSections {
                components: "- b1 BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                                 - b2 BUF_X1 + PLACED ( 4000 2800 ) N ;\n\
                                 - l LAT + PLACED ( 6000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( l CK ) ;\n- in ( PIN in ) ( b1 A ) ;\n\
                           - n1 ( b1 Z ) ( b2 A ) ;\n- n2 ( b2 Z ) ( l D ) ;\n\
                           - out ( l Q ) ( PIN out ) ;\n",
                inputs: &["clk", "in"],
                outputs: &["out"],
            },
        ),
        &[("DFF_X1", "LAT"), ("BUF_X1", "BUF_SLOW")],
    );
    let d = cell_pin(&db, "l/D");
    let lib = cells_with(&format!("{LATCH}\n{SLOW_BUFFER}"));
    let constraints = "create_clock -name clk -period 2.0 [get_ports clk]\n\
                           set_input_delay 0.4 -clock clk [get_ports in]\n";

    with_timed_session(&mut db, &lib, constraints, |resizer| {
        let b1 = resizer.db.cell_id("b1").unwrap();
        let slow = resizer.link.liberty.cell_id("BUF_SLOW").unwrap();
        assert!(resizer.replace_cell(b1, slow).unwrap());

        let netlist = resizer.netlist();
        let sdc = resizer.sta().sdc();
        let fresh = Sta::new(
            &netlist,
            &resizer.graph,
            resizer.clocks,
            resizer.constants,
            sdc,
        );
        assert_eq!(
            resizer.sta().slack(d, MinMax::Max),
            fresh.slack(d, MinMax::Max)
        );
    });
}

#[test]
fn a_false_path_leaves_its_endpoint_unconstrained() {
    let db = flop_to_flop_design();
    let d = cell_pin(&db, "f2/D");
    let clock = "create_clock -name clk -period 2.2 [get_ports clk]\n";
    let false_path = format!("{clock}set_false_path -from [get_clocks clk] -to [get_clocks clk]\n");

    assert!(constrained_under(&db, clock, d, MinMax::Max));
    assert!(!constrained_under(&db, &false_path, d, MinMax::Max));
}

/// A flop clocked from `clk` driving the output port `out`.
fn flop_to_port_design() -> NetlistDB {
    design_with_copies(
        &def_text(
            "out",
            &DefSections {
                components: "- f DFF_X1 + PLACED ( 2000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f CK ) ;\n- out ( f Q ) ( PIN out ) ;\n",
                inputs: &["clk"],
                outputs: &["out"],
            },
        ),
        &[],
    )
}

#[test]
fn an_output_delay_takes_its_time_off_the_capturing_edge() {
    let db = flop_to_port_design();
    let out = db.io_pins[db.io_pin_id("out").unwrap()].pin;
    let constraints = "create_clock -name clk -period 2.2 [get_ports clk]\n\
                           set_output_delay 0.5 -clock clk [get_ports out]\n";

    let (slack, arrival) = timed(&db, constraints, |_, sta| {
        let latest = Edge::BOTH
            .iter()
            .filter_map(|&edge| sta.arrival(out, edge, MinMax::Max))
            .fold(Seconds::new(f64::NEG_INFINITY), Seconds::max);
        (sta.slack(out, MinMax::Max).time().unwrap(), latest)
    });

    assert!(arrival > Seconds::ZERO);
    assert!(
        ((slack + arrival).get() - 1.7e-9).abs() < 1e-15,
        "{slack:?} + {arrival:?}"
    );
}

/// A flop clocked from `clk` through buffer `cb`.
fn buffered_clock_design() -> NetlistDB {
    design_with_copies(
        &def_text(
            "cb",
            &DefSections {
                components: "- cb BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                                 - f DFF_X1 + PLACED ( 4000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( f CK ) ;\n",
                inputs: &["clk"],
                outputs: &[],
            },
        ),
        &[],
    )
}

#[test]
fn a_propagated_clock_arrives_through_its_buffers_and_an_ideal_one_at_once() {
    let db = buffered_clock_design();
    let ck = cell_pin(&db, "f/CK");
    let clock = "create_clock -name clk -period 2.2 [get_ports clk]\n";
    let propagated = format!("{clock}set_propagated_clock [all_clocks]\n");

    let ideal = timed(&db, clock, |_, sta| sta.clock_rise_arrival(ck));
    let late = timed(&db, &propagated, |_, sta| sta.clock_rise_arrival(ck));

    assert_eq!(ideal, Some(Seconds::ZERO));
    assert!(late.is_some_and(|t| t > Seconds::ZERO), "{late:?}");
}

/// The clock `clk` gated by the port `en` in and gate `g` on its way to
/// flop `f`.
fn gated_clock_design() -> NetlistDB {
    design_with_copies(
        &def_text(
            "gated",
            &DefSections {
                components: "- g GATE + PLACED ( 2000 2800 ) N ;\n\
                                 - f DFF_X1 + PLACED ( 4000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( g A ) ;\n- en ( PIN en ) ( g B ) ;\n\
                           - gck ( g Z ) ( f CK ) ;\n",
                inputs: &["clk", "en"],
                outputs: &[],
            },
        ),
        &[("AND2_X1", "GATE")],
    )
}

/// An and gate timed from both its inputs.
const GATE: &str = "cell (GATE) { area : 1.0 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (B) { direction : input ; capacitance : 1.0 ; }
        pin (Z) { direction : output ; function : \"(A & B)\" ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.05\") ; }
            cell_fall (scalar) { values (\"0.05\") ; } }
          timing () { related_pin : \"B\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.05\") ; }
            cell_fall (scalar) { values (\"0.05\") ; } } } }";

#[test]
fn a_gated_clock_enable_is_checked_against_its_clock() {
    let db = gated_clock_design();
    let enable = cell_pin(&db, "g/B");
    let constraints = "create_clock -name clk -period 2.2 [get_ports clk]\n\
                           set_input_delay 0.3 -clock clk [get_ports en]\n";

    let (setup, hold, ends) = timed_with(&db, &cells_with(GATE), constraints, |_, sta| {
        (
            sta.slack(enable, MinMax::Max),
            sta.slack(enable, MinMax::Min),
            sta.endpoints().contains(&enable),
        )
    });

    assert!(ends);
    assert!(
        setup.time().is_some() && hold.time().is_some(),
        "{setup:?} {hold:?}"
    );
}

#[test]
fn slack_is_infinite_without_an_arrival_or_a_bound() {
    assert!(!slack_of(None, Seconds::new(1.0), MinMax::Max).is_finite());
    assert!(
        !slack_of(
            Some(Seconds::new(1.0)),
            REQUIRED_INIT[MinMax::Max],
            MinMax::Max
        )
        .is_finite()
    );
    assert!(
        (slack_of(Some(Seconds::new(1.0)), Seconds::new(3.0), MinMax::Max).get() - 2.0).abs()
            < 1e-12
    );
    assert!(
        (slack_of(Some(Seconds::new(1.0)), Seconds::new(0.5), MinMax::Min).get() - 0.5).abs()
            < 1e-12
    );
}

const CLOCK_2NS: &str = "create_clock -name clk -period 2.2 [get_ports clk]\n";

/// Whether `value` is `nanoseconds` to within a femtosecond.
fn is_ns(value: Seconds, nanoseconds: f64) -> bool {
    (value - Seconds::new(nanoseconds * 1e-9)).abs() < Seconds::new(1e-15)
}

#[test]
fn a_slack_is_below_a_bound_only_when_constrained_and_under_it() {
    let bound = Seconds::new(1e-9);

    assert!(Slack::Constrained(Seconds::new(0.5e-9)).is_below(bound));
    assert!(!Slack::Constrained(bound).is_below(bound));
    assert!(!Slack::Constrained(Seconds::new(2e-9)).is_below(bound));
    assert!(!Slack::Unconstrained.is_below(bound));
}

#[test]
fn only_an_infinite_required_time_is_unconstrained() {
    assert!(unconstrained(Seconds::INFINITY));
    assert!(unconstrained(Seconds::new(f64::NEG_INFINITY)));
    assert!(!unconstrained(Seconds::ZERO));
    assert!(!unconstrained(Seconds::new(1e-9)));
}

#[test]
fn a_required_time_is_tighter_when_earlier_for_setup_and_later_for_hold() {
    let (early, late) = (Seconds::new(1e-9), Seconds::new(2e-9));

    assert!(tighter(MinMax::Max, early, late));
    assert!(!tighter(MinMax::Max, late, early));
    assert!(tighter(MinMax::Min, late, early));
    assert!(!tighter(MinMax::Min, early, late));
    assert!(!tighter(MinMax::Max, early, early));
    assert!(!tighter(MinMax::Min, late, late));
}

#[test]
fn a_check_captures_with_the_latency_of_the_side_opposite_to_it() {
    let capture = Capture {
        edge: edge(0, Edge::Rise),
        latency: SideValues {
            min: Seconds::new(1e-9),
            max: Seconds::new(3e-9),
        },
    };

    assert_eq!(capture.latency(MinMax::Max), Seconds::new(1e-9));
    assert_eq!(capture.latency(MinMax::Min), Seconds::new(3e-9));
}

#[test]
fn a_registers_data_input_ends_timing_and_its_clock_pin_and_output_do_not() {
    let db = flop_to_flop_design();
    let (d1, d2) = (cell_pin(&db, "f1/D"), cell_pin(&db, "f2/D"));
    let others = ["f1/CK", "f2/CK", "f1/Q"].map(|name| cell_pin(&db, name));

    let (ends, flags) = timed(&db, CLOCK_2NS, |_, sta| {
        let flags: Vec<bool> = others.iter().map(|&p| sta.is_endpoint(p)).collect();
        (sta.endpoints().to_vec(), flags)
    });

    assert_eq!(ends, vec![d1, d2]);
    assert_eq!(flags, [false, false, false]);
}

#[test]
fn an_output_port_ends_timing_only_with_an_output_delay() {
    let db = flop_to_port_design();
    let out = db.io_pins[db.io_pin_id("out").unwrap()].pin;
    let with_delay = format!("{CLOCK_2NS}set_output_delay 0.5 -clock clk [get_ports out]\n");

    let bare = timed(&db, CLOCK_2NS, |_, sta| sta.is_endpoint(out));
    let delayed = timed(&db, &with_delay, |_, sta| sta.is_endpoint(out));

    assert!(!bare);
    assert!(delayed);
}

/// A flop clocked from `clk` through the inverter `ci`.
fn inverted_clock_design() -> NetlistDB {
    design_with_copies(
        &def_text(
            "inv",
            &DefSections {
                components: "- ci INV_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f DFF_X1 + PLACED ( 4000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( ci A ) ;\n- ck ( ci ZN ) ( f CK ) ;\n\
                       - out ( f Q ) ( PIN out ) ;\n",
                inputs: &["clk"],
                outputs: &["out"],
            },
        ),
        &[],
    )
}

#[test]
fn the_rising_clock_arrival_is_the_rising_edge_not_the_falling_one_it_inverts_to() {
    let db = inverted_clock_design();
    let ck = cell_pin(&db, "f/CK");
    let clock = "create_clock -name clk -period 2.0 -waveform {0.2 1.2} [get_ports clk]\n";

    let rise = timed(&db, clock, |_, sta| sta.clock_rise_arrival(ck));

    assert!(rise.is_some_and(|t| is_ns(t, 0.2)), "{rise:?}");
}

#[test]
fn a_launched_register_output_is_data_not_the_clock() {
    let db = flop_to_port_design();
    let q = cell_pin(&db, "f/Q");
    let ck = cell_pin(&db, "f/CK");

    let (at_q, at_ck, q_tags) = timed(&db, CLOCK_2NS, |_, sta| {
        let tags: Vec<bool> = Edge::BOTH
            .iter()
            .flat_map(|&e| sta.entries(q, e).iter().map(|x| sta.tags[x.tag].is_clock))
            .collect();
        (sta.clock_rise_arrival(q), sta.clock_rise_arrival(ck), tags)
    });

    assert_eq!(at_q, None);
    assert_eq!(at_ck, Some(Seconds::ZERO));
    assert_eq!(q_tags, [false, false]);
}

#[test]
fn the_worst_slacks_are_remembered_once_found_again() {
    let db = flop_to_flop_design();
    let constraints = format!("{CLOCK_2NS}set_input_delay 0.3 -clock clk [get_ports in]\n");

    timed(&db, &constraints, |_, sta| {
        let mut stale = sta.clone();
        stale.worst = SideValues::both(WorstSlack::Stale);

        stale.cache_worst();

        for side in MinMax::BOTH {
            assert_eq!(stale.worst[side], WorstSlack::Found(sta.worst_slack(side)));
            assert!(sta.worst_slack(side).is_some());
        }
    });
}

#[test]
fn a_pin_with_arrivals_but_no_check_has_no_worst_slack_path() {
    let db = flop_to_port_design();
    let out = db.io_pins[db.io_pin_id("out").unwrap()].pin;

    let (slack, path) = timed(&db, CLOCK_2NS, |netlist, sta| {
        (
            sta.slack(out, MinMax::Max),
            sta.worst_slack_path(netlist, out, MinMax::Max),
        )
    });

    assert_eq!(slack, Slack::Unconstrained);
    assert_eq!(path, None);
}

#[test]
fn a_path_starts_its_data_at_the_register_output_it_launches_from() {
    let db = flop_to_flop_design();
    let d2 = cell_pin(&db, "f2/D");
    let launching = [cell_pin(&db, "f1/CK"), cell_pin(&db, "f1/Q"), d2];

    let path = timed(&db, CLOCK_2NS, |netlist, sta| {
        sta.worst_slack_path(netlist, d2, MinMax::Max).unwrap()
    });

    let pins: Vec<PinId> = path.points.iter().map(|p| p.pin).collect();
    assert_eq!(pins, launching);
    assert_eq!(path.start_index, 1);
}

/// A latch open while `CK` is low, with the setup of [`LATCH`] before the
/// closing rise.
const LATCH_OPEN_LOW: &str = "cell (LAT_N) { area : 4.5 ;
        latch (\"IQ\", \"IQN\") { data_in : \"D\" ; enable : \"!CK\" ; }
        pin (D) { direction : input ; capacitance : 1.0 ;
          timing () { related_pin : \"CK\" ; timing_type : setup_rising ;
            rise_constraint (scalar) { values (\"-0.05\") ; }
            fall_constraint (scalar) { values (\"0.08\") ; } } }
        pin (CK) { direction : input ; clock : true ; capacitance : 1.0 ; }
        pin (Q) { direction : output ; function : \"IQ\" ;
          timing () { related_pin : \"D\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } }
          timing () { related_pin : \"CK\" ; timing_type : falling_edge ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } } } }";

/// [`LATCH`] with a hold check on `D` listed before its setup check.
const LATCH_HOLD_FIRST: &str = "cell (LAT) { area : 4.5 ;
        latch (\"IQ\", \"IQN\") { data_in : \"D\" ; enable : \"CK\" ; }
        pin (D) { direction : input ; capacitance : 1.0 ;
          timing () { related_pin : \"CK\" ; timing_type : hold_falling ;
            rise_constraint (scalar) { values (\"0.02\") ; }
            fall_constraint (scalar) { values (\"0.03\") ; } }
          timing () { related_pin : \"CK\" ; timing_type : setup_falling ;
            rise_constraint (scalar) { values (\"-0.05\") ; }
            fall_constraint (scalar) { values (\"0.08\") ; } } }
        pin (CK) { direction : input ; clock : true ; capacitance : 1.0 ; }
        pin (Q) { direction : output ; function : \"IQ\" ;
          timing () { related_pin : \"D\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } }
          timing () { related_pin : \"CK\" ; timing_type : rising_edge ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } } } }";

/// Cells that hold their output high and low.
const TIE_CELLS: &str = "cell (TIEH) { area : 0.5 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (Z) { direction : output ; function : \"1\" ; } }
    cell (TIEL) { area : 0.5 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (Z) { direction : output ; function : \"0\" ; } }";

/// The latch `l` of cell `latch`, enabled by the port `clk` or, with a
/// `tie` cell, by its constant output.
fn enabled_latch_design(latch: &str, tie: Option<&str>) -> NetlistDB {
    let (tie_cell, enable_nets) = match tie {
        Some(cell) => (
            format!("- t {cell} + PLACED ( 2000 2800 ) N ;\n"),
            "- clk ( PIN clk ) ;\n- en ( t Z ) ( l CK ) ;\n",
        ),
        None => (String::new(), "- clk ( PIN clk ) ( l CK ) ;\n"),
    };
    design_with_copies(
        &def_text(
            "enabled",
            &DefSections {
                components: &format!("{tie_cell}- l {latch} + PLACED ( 6000 2800 ) N ;\n"),
                nets: &format!(
                    "{enable_nets}- in ( PIN in ) ( l D ) ;\n- out ( l Q ) ( PIN out ) ;\n"
                ),
                inputs: &["clk", "in"],
                outputs: &["out"],
            },
        ),
        &[("DFF_X1", latch), ("BUF_X1", "TIEH"), ("BUF_X1", "TIEL")],
    )
}

/// The cells of the latch tests.
fn latch_cells() -> Liberty {
    cells_with(&format!("{LATCH}\n{LATCH_OPEN_LOW}\n{TIE_CELLS}"))
}

#[test]
fn a_latch_is_enabled_by_a_clock_or_held_open_or_closed_by_a_constant() {
    let cases = [
        ("LAT", None, Edge::Rise, LatchState::Enabled),
        ("LAT_N", None, Edge::Fall, LatchState::Enabled),
        ("LAT", Some("TIEH"), Edge::Rise, LatchState::Open),
        ("LAT", Some("TIEL"), Edge::Rise, LatchState::Closed),
        ("LAT_N", Some("TIEH"), Edge::Fall, LatchState::Closed),
        ("LAT_N", Some("TIEL"), Edge::Fall, LatchState::Open),
    ];

    for (latch, tie, open_edge, state) in cases {
        let db = enabled_latch_design(latch, tie);
        let (data, enable) = (cell_pin(&db, "l/D"), cell_pin(&db, "l/CK"));

        let found = timed_with(&db, &latch_cells(), "", |_, sta| {
            sta.latches.values().copied().collect::<Vec<_>>()
        });

        let expected = Latch {
            enable,
            open_edge,
            data,
            state,
        };
        assert_eq!(found, [expected], "{latch} {tie:?}");
    }
}

/// The late arrivals at the output `q` of the latch design of `latch`
/// under `constraints`: rising then falling.
fn latch_output_arrivals(
    latch: &str,
    tie: Option<&str>,
    lib: &Liberty,
    constraints: &str,
) -> (Option<Seconds>, Option<Seconds>) {
    let db = enabled_latch_design(latch, tie);
    let q = cell_pin(&db, "l/Q");
    timed_with(&db, lib, constraints, |_, sta| {
        (
            sta.arrival(q, Edge::Rise, MinMax::Max),
            sta.arrival(q, Edge::Fall, MinMax::Max),
        )
    })
}

fn assert_arrivals(found: (Option<Seconds>, Option<Seconds>), rise: f64, fall: f64) {
    let (found_rise, found_fall) = found;
    assert!(
        found_rise.is_some_and(|t| is_ns(t, rise)) && found_fall.is_some_and(|t| is_ns(t, fall)),
        "{found:?} vs {rise} and {fall} ns"
    );
}

const LATCH_CLOCK: &str = "create_clock -name clk -period 2.0 [get_ports clk]\n";

#[test]
fn data_passes_an_open_latch_as_through_a_gate_and_not_a_closed_one() {
    let constraints = "create_clock -name v -period 2.0\n\
                       set_input_delay 0.5 -clock v [get_ports in]\n";

    let open = latch_output_arrivals("LAT", Some("TIEH"), &latch_cells(), constraints);
    let closed = latch_output_arrivals("LAT", Some("TIEL"), &latch_cells(), constraints);

    assert_arrivals(open, 0.6, 0.6);
    assert_eq!(closed, (None, None));
}

#[test]
fn a_latch_margin_is_the_setup_check_and_not_a_hold_check_listed_before_it() {
    let lib = cells_with(LATCH_HOLD_FIRST);
    let constraints = format!("{LATCH_CLOCK}set_input_delay 0.98 -clock clk [get_ports in]\n");

    let found = latch_output_arrivals("LAT", None, &lib, &constraints);

    assert_arrivals(found, 1.08, 1.02);
}

#[test]
fn a_clock_uncertainty_opens_the_latch_window_earlier_for_borrowing() {
    let constraints = format!(
        "{LATCH_CLOCK}set_clock_uncertainty 0.1 [get_clocks clk]\n\
         set_input_delay 0.98 -clock clk [get_ports in]\n"
    );

    let found = latch_output_arrivals("LAT", None, &latch_cells(), &constraints);

    assert_arrivals(found, 1.05, 0.92);
}

#[test]
fn a_setup_multicycle_keeps_data_from_borrowing_before_the_later_edge() {
    let constraints = format!(
        "{LATCH_CLOCK}set_input_delay 0.98 -clock clk [get_ports in]\n\
         set_multicycle_path 2 -setup -from [get_clocks clk] -to [get_clocks clk]\n"
    );

    let found = latch_output_arrivals("LAT", None, &latch_cells(), &constraints);

    assert_arrivals(found, 0.1, 0.1);
}

#[test]
fn a_latch_opening_on_the_fall_borrows_up_to_the_width_of_its_open_window_less_the_setup() {
    let constraints = "create_clock -name clk -period 2.0 -waveform {0 0.8} [get_ports clk]\n\
                       set_input_delay 1.95 -clock clk [get_ports in]\n";

    let found = latch_output_arrivals("LAT_N", None, &latch_cells(), constraints);

    // The window is 1.2 wide from the fall at 0.8; rising data borrows
    // 1.15 of the 1.25 it may, falling data 1.12 of its 1.12.
    assert_arrivals(found, 2.05, 2.02);
}

#[test]
fn data_from_a_faster_clock_borrows_in_the_frame_of_the_latchs_cycle() {
    let constraints = "create_clock -name a -period 2.0 -waveform {0 1.0}\n\
                       create_clock -name b -period 3.0 -waveform {0 1.5} [get_ports clk]\n\
                       set_input_delay 0.5 -clock a [get_ports in]\n";

    let found = latch_output_arrivals("LAT", None, &latch_cells(), constraints);

    // The window opens 1.0 before the launch's edge; the data's time moves
    // from two cycles of `a` to one of `b`, 1.0 later.
    assert_arrivals(found, 1.6, 1.52);
}

/// The arc group from `from` into `to` the graph of a timed design follows.
fn arc_set_from(graph: &TimingGraph, from: PinId, to: PinId) -> ArcSetId {
    graph
        .in_arcs(to)
        .iter()
        .find(|arc| arc.from == from)
        .unwrap()
        .set
}

#[test]
fn a_path_starts_at_a_registers_clock_arc_and_an_enabled_latchs_data_arc_only() {
    let clocked = enabled_latch_design("LAT", None);
    let open = enabled_latch_design("LAT", Some("TIEH"));
    let gate_path = flop_to_flop_design();

    let starts = |db: &NetlistDB, from: &str, to: &str| {
        timed_in_full(db, &latch_cells(), LATCH_CLOCK, |netlist, graph, _, sta| {
            let (from, to) = (cell_pin(db, from), cell_pin(db, to));
            sta.starts_data(netlist, to, arc_set_from(graph, from, to))
        })
    };

    assert!(starts(&clocked, "l/CK", "l/Q"));
    assert!(starts(&clocked, "l/D", "l/Q"));
    assert!(!starts(&open, "l/D", "l/Q"));
    let flop = timed_in_full(
        &gate_path,
        &latch_cells(),
        CLOCK_2NS,
        |netlist, graph, _, sta| {
            let (ck, q) = (cell_pin(&gate_path, "f1/CK"), cell_pin(&gate_path, "f1/Q"));
            sta.starts_data(netlist, q, arc_set_from(graph, ck, q))
        },
    );
    assert!(flop);
}

#[test]
fn a_slower_arrival_at_a_latch_input_is_the_only_one_the_required_search_revisits() {
    let db = two_registers_behind_buffers();
    let (flop_d, latch_d) = (cell_pin(&db, "f/D"), cell_pin(&db, "l/D"));
    let constraints = "create_clock -name clk -period 2.0 [get_ports clk]\n\
                       set_input_delay 0.98 -clock clk [get_ports in1]\n\
                       set_input_delay 0.98 -clock clk [get_ports in2]\n";
    let lib = latch_cells();

    let requiring = timed_in_full(&db, &lib, constraints, |netlist, graph, clocks, sta| {
        let (to_flop, to_latch) = (cell_pin(&db, "b1/Z"), cell_pin(&db, "b2/Z"));
        let mut slower = graph.clone();
        for input in ["b1/A", "b2/A"] {
            slower.annotate_slew(cell_pin(&db, input), Seconds::new(0.15e-9));
        }
        let _ = slower.propagate_from(netlist, clocks, &[to_flop, to_latch]);
        let seeds = [to_flop, flop_d, to_latch, latch_d];

        sta.clone().find_arrivals(netlist, &slower, clocks, &seeds)
    });

    assert_eq!(requiring, [latch_d]);
}

/// A flop `f` and a latch `l`, each with its data from a port through a
/// buffer, and each with its output on a port.
fn two_registers_behind_buffers() -> NetlistDB {
    design_with_copies(
        &def_text(
            "pair",
            &DefSections {
                components: "- b1 BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - b2 BUF_X1 + PLACED ( 2000 20000 ) N ;\n\
                             - f DFF_X1 + PLACED ( 20000 2800 ) N ;\n\
                             - l LAT + PLACED ( 20000 20000 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f CK ) ( l CK ) ;\n- in1 ( PIN in1 ) ( b1 A ) ;\n\
                       - in2 ( PIN in2 ) ( b2 A ) ;\n- d1 ( b1 Z ) ( f D ) ;\n\
                       - d2 ( b2 Z ) ( l D ) ;\n- o1 ( f Q ) ( PIN o1 ) ;\n\
                       - o2 ( l Q ) ( PIN o2 ) ;\n",
                inputs: &["clk", "in1", "in2"],
                outputs: &["o1", "o2"],
            },
        ),
        &[("DFF_X1", "LAT")],
    )
}

#[test]
fn arrivals_found_again_unchanged_leave_nothing_to_require_even_where_a_side_is_absent() {
    let db = enabled_latch_design("LAT", None);
    let constraints = format!("{LATCH_CLOCK}set_input_delay 0.5 -max -clock clk [get_ports in]\n");

    let requiring = timed_in_full(
        &db,
        &latch_cells(),
        &constraints,
        |netlist, graph, clocks, sta| {
            let all: Vec<PinId> = netlist.db.pins.ids().collect();
            sta.clone().find_arrivals(netlist, graph, clocks, &all)
        },
    );

    assert_eq!(requiring, []);
}

#[test]
fn a_propagated_clock_pin_is_read_at_its_own_slew_and_an_ideal_one_at_zero() {
    let db = buffered_clock_design();
    let ck = cell_pin(&db, "f/CK");
    let propagated = format!("{CLOCK_2NS}set_propagated_clock [all_clocks]\n");

    let slews = |constraints: &str| {
        timed_in_full(&db, &cells_with(""), constraints, |_, graph, _, sta| {
            (
                sta.clock_slew(graph, ck, Edge::Rise, MinMax::Max),
                graph.slew_side(ck, MinMax::Max).rise,
            )
        })
    };

    let (ideal, graph_slew) = slews(CLOCK_2NS);
    let (read, same_pin_slew) = slews(&propagated);

    assert_eq!(ideal, Seconds::ZERO);
    assert!(same_pin_slew > Seconds::ZERO);
    assert_eq!(read, same_pin_slew);
    assert_eq!(graph_slew, same_pin_slew);
}

#[test]
fn a_cell_delay_derate_scales_the_delays_through_the_cells() {
    let db = flop_to_flop_design();
    let q = cell_pin(&db, "f1/Q");
    let derated = format!("{CLOCK_2NS}set_timing_derate -cell_delay 2.0\n");
    let launch = |constraints: &str| {
        timed(&db, constraints, |_, sta| {
            sta.arrival(q, Edge::Rise, MinMax::Max).unwrap()
        })
    };

    let (plain, doubled) = (launch(CLOCK_2NS), launch(&derated));

    assert!(plain > Seconds::ZERO);
    assert!(
        ((doubled / plain) - 2.0).abs() < 1e-9,
        "{plain:?} {doubled:?}"
    );
}

#[test]
fn timing_into_a_latch_output_comes_from_its_enable_alone() {
    let db = enabled_latch_design("LAT", None);
    let (enable, q) = (cell_pin(&db, "l/CK"), cell_pin(&db, "l/Q"));

    let from = timed_in_full(&db, &latch_cells(), LATCH_CLOCK, |netlist, graph, _, _| {
        predecessors(netlist, graph, q)
    });

    assert_eq!(from, [enable]);
}

#[test]
fn timing_into_a_gate_in_a_ring_does_not_come_back_round_the_loop() {
    let db = design(&def_text(
        "ring",
        &DefSections {
            components: "- i1 INV_X1 + PLACED ( 2000 2800 ) N ;\n\
                         - i2 INV_X1 + PLACED ( 4000 2800 ) N ;\n",
            nets: "- a ( i1 ZN ) ( i2 A ) ;\n- b ( i2 ZN ) ( i1 A ) ;\n",
            inputs: &[],
            outputs: &[],
        },
    ));
    let arcs =
        [("i1/A", "i1/ZN"), ("i2/A", "i2/ZN")].map(|(a, z)| (cell_pin(&db, a), cell_pin(&db, z)));

    let from = timed_in_full(&db, &cells_with(""), "", |netlist, graph, _, _| {
        let cut = arcs
            .into_iter()
            .find(|&(a, z)| graph.is_loop_arc(a, z))
            .unwrap();
        predecessors(netlist, graph, cut.1)
    });

    assert_eq!(from, []);
}

#[test]
fn a_retired_cells_pins_lose_their_arrivals_when_timed_again() {
    let mut db = design_with_copies(
        &def_text(
            "through",
            &DefSections {
                components: "- b BUF_X1 + PLACED ( 2000 2800 ) N ;\n",
                nets: "- in ( PIN in ) ( b A ) ;\n- out ( b Z ) ( PIN out ) ;\n",
                inputs: &["in"],
                outputs: &["out"],
            },
        ),
        &[],
    );
    let buffer = db.cell_id("b").unwrap();
    let pins = [cell_pin(&db, "b/A"), cell_pin(&db, "b/Z")];
    let lib = cells_with("");
    let constraints = "create_clock -name v -period 2.0\n\
                       set_input_delay 0.5 -clock v [get_ports in]\n";
    let sdc = sdc::parse_str(constraints, Path::new("test.sdc"), &db, &lib).unwrap();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let (clocks, graph, mut sta) = {
        let netlist = Netlist::new(&db, &link);
        let constants = Constants::find(&netlist);
        let clocks = ClockNetwork::from_sdc(&netlist, &sdc, &constants);
        let rc = WireRc {
            resistance: OhmsPerMeter::ZERO,
            capacitance: FaradsPerMeter::ZERO,
            layer: LayerId(1),
        };
        let graph = TimingGraph::new(
            &netlist,
            &clocks,
            &constants,
            rc,
            None,
            Estimator::Placement,
            PortConditions::from_sdc(&sdc),
        );
        let sta = Sta::new(&netlist, &graph, &clocks, &constants, &sdc);
        (clocks, graph, sta)
    };
    let timed_before = pins.map(|p| sta.arrival(p, Edge::Rise, MinMax::Max).is_some());

    let _ = db.retire_cell(buffer);
    let netlist = Netlist::new(&db, &link);
    let _ = sta.find_arrivals(&netlist, &graph, &clocks, &pins);

    assert_eq!(timed_before, [true, true]);
    assert_eq!(
        pins.map(|p| sta.arrival(p, Edge::Rise, MinMax::Max)),
        [None, None]
    );
}

/// A nand gate whose two inputs and two edges have delays of their own.
const SKEWED_NAND: &str = "cell (SKEW) { area : 1.0 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (B) { direction : input ; capacitance : 1.0 ; }
        pin (Z) { direction : output ; function : \"!(A & B)\" ;
          timing () { related_pin : \"A\" ; timing_sense : negative_unate ;
            cell_rise (scalar) { values (\"0.10\") ; }
            cell_fall (scalar) { values (\"0.30\") ; } }
          timing () { related_pin : \"B\" ; timing_sense : negative_unate ;
            cell_rise (scalar) { values (\"0.20\") ; }
            cell_fall (scalar) { values (\"0.50\") ; } } } }";

/// The gate `g` of [`SKEWED_NAND`] fed from two ports and feeding flop `f`.
fn skewed_gate_design() -> NetlistDB {
    design_with_copies(
        &def_text(
            "skew",
            &DefSections {
                components: "- g SKEW + PLACED ( 2000 2800 ) N ;\n\
                             - f DFF_X1 + PLACED ( 4000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f CK ) ;\n- in1 ( PIN in1 ) ( g A ) ;\n\
                       - in2 ( PIN in2 ) ( g B ) ;\n- z ( g Z ) ( f D ) ;\n",
                inputs: &["clk", "in1", "in2"],
                outputs: &[],
            },
        ),
        &[("AND2_X1", "SKEW")],
    )
}

#[test]
fn a_pins_required_time_is_found_back_through_the_arc_from_it_on_its_own_edge() {
    let db = skewed_gate_design();
    let (a, b) = (cell_pin(&db, "g/A"), cell_pin(&db, "g/B"));
    let constraints = format!(
        "{CLOCK_2NS}set_input_delay 0 -clock clk [get_ports in1]\n\
         set_input_delay 0 -clock clk [get_ports in2]\n"
    );

    let slack = |pin, edge| {
        timed_with(&db, &cells_with(SKEWED_NAND), &constraints, |_, sta| {
            sta.slack_edge(pin, edge, MinMax::Max).time().unwrap()
        })
    };

    // A rising input makes the output fall, so the falling-output delay is
    // the one it loses to; the other pin loses a different delay.
    assert!(is_ns(slack(a, Edge::Fall) - slack(a, Edge::Rise), 0.2));
    assert!(is_ns(slack(b, Edge::Rise) - slack(a, Edge::Rise), -0.2));
    assert!(is_ns(slack(b, Edge::Fall) - slack(a, Edge::Fall), -0.1));
}

/// A register with two data inputs, `A` with a short setup and `Z` with a
/// long one, both checked against its clock `B`.
const TWO_INPUT_REGISTER: &str = "cell (REG2) { area : 4.5 ;
        ff (\"IQ\", \"IQN\") { next_state : \"(A & Z)\" ; clocked_on : \"B\" ; }
        pin (A) { direction : input ; capacitance : 1.0 ;
          timing () { related_pin : \"B\" ; timing_type : setup_rising ;
            rise_constraint (scalar) { values (\"0.03\") ; }
            fall_constraint (scalar) { values (\"0.03\") ; } } }
        pin (Z) { direction : input ; capacitance : 1.0 ;
          timing () { related_pin : \"B\" ; timing_type : setup_rising ;
            rise_constraint (scalar) { values (\"0.20\") ; }
            fall_constraint (scalar) { values (\"0.20\") ; } } }
        pin (B) { direction : input ; clock : true ; capacitance : 1.0 ; } }";

#[test]
fn an_inputs_setup_is_checked_alone_and_not_with_those_of_the_cells_other_inputs() {
    let db = design_with_copies(
        &def_text(
            "two",
            &DefSections {
                components: "- r REG2 + PLACED ( 2000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( r B ) ;\n- in1 ( PIN in1 ) ( r A ) ;\n\
                       - in2 ( PIN in2 ) ( r Z ) ;\n",
                inputs: &["clk", "in1", "in2"],
                outputs: &[],
            },
        ),
        &[("AND2_X1", "REG2")],
    );
    let (a, z) = (cell_pin(&db, "r/A"), cell_pin(&db, "r/Z"));
    let constraints = "create_clock -name clk -period 2.0 [get_ports clk]\n\
                       set_input_delay 0.1 -clock clk [get_ports in1]\n\
                       set_input_delay 0.1 -clock clk [get_ports in2]\n";

    let (short, long) = timed_with(
        &db,
        &cells_with(TWO_INPUT_REGISTER),
        constraints,
        |_, sta| {
            (
                sta.slack(a, MinMax::Max).time().unwrap(),
                sta.slack(z, MinMax::Max).time().unwrap(),
            )
        },
    );

    assert!(is_ns(short, 1.87), "{short:?}");
    assert!(is_ns(long, 1.70), "{long:?}");
}

#[test]
fn a_latch_input_borrows_where_enabled_and_is_checked_as_a_register_for_its_hold() {
    let db = enabled_latch_design("LAT", None);
    let d = cell_pin(&db, "l/D");
    let constraints = format!("{LATCH_CLOCK}set_input_delay 0.98 -clock clk [get_ports in]\n");

    let slack = |edge, side| {
        timed_with(
            &db,
            &cells_with(LATCH_HOLD_FIRST),
            &constraints,
            |_, sta| sta.slack_edge(d, edge, side).time().unwrap(),
        )
    };

    // Rising data borrows what it needs and falling data more than it may;
    // the hold is against the closing fall of the cycle before.
    assert!(is_ns(slack(Edge::Rise, MinMax::Max), 0.0));
    assert!(is_ns(slack(Edge::Fall, MinMax::Max), -0.06));
    assert!(is_ns(slack(Edge::Rise, MinMax::Min), 1.96));
    assert!(is_ns(slack(Edge::Fall, MinMax::Min), 1.95));
}

/// A flop whose checks and delays do not depend on slews.
const SCALAR_REGISTER: &str = "cell (REGS) { area : 4.5 ;
        ff (\"IQ\", \"IQN\") { next_state : \"D\" ; clocked_on : \"CK\" ; }
        pin (D) { direction : input ; capacitance : 1.0 ;
          timing () { related_pin : \"CK\" ; timing_type : setup_rising ;
            rise_constraint (scalar) { values (\"0.04\") ; }
            fall_constraint (scalar) { values (\"0.04\") ; } }
          timing () { related_pin : \"CK\" ; timing_type : hold_rising ;
            rise_constraint (scalar) { values (\"0.02\") ; }
            fall_constraint (scalar) { values (\"0.02\") ; } } }
        pin (CK) { direction : input ; clock : true ; capacitance : 1.0 ; }
        pin (Q) { direction : output ; function : \"IQ\" ;
          timing () { related_pin : \"CK\" ; timing_type : rising_edge ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } } } }";

/// The flop `f` of [`SCALAR_REGISTER`] clocked from `clk` through `cb`, its
/// data from `in`.
fn scalar_flop_behind_a_clock_buffer() -> NetlistDB {
    design_with_copies(
        &def_text(
            "late",
            &DefSections {
                components: "- cb BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - f REGS + PLACED ( 4000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( f CK ) ;\n\
                       - in ( PIN in ) ( f D ) ;\n- out ( f Q ) ( PIN out ) ;\n",
                inputs: &["clk", "in"],
                outputs: &["out"],
            },
        ),
        &[("DFF_X1", "REGS")],
    )
}

#[test]
fn a_propagated_clock_moves_the_checks_by_the_latency_it_reaches_the_register_with() {
    let db = scalar_flop_behind_a_clock_buffer();
    let (d, ck) = (cell_pin(&db, "f/D"), cell_pin(&db, "f/CK"));
    let constraints = "create_clock -name clk -period 2.0 [get_ports clk]\n\
                       set_propagated_clock [get_clocks clk]\n\
                       set_input_delay 0.3 -clock clk [get_ports in]\n";

    let (setup, hold, latency) =
        timed_with(&db, &cells_with(SCALAR_REGISTER), constraints, |_, sta| {
            (
                sta.slack(d, MinMax::Max).time().unwrap(),
                sta.slack(d, MinMax::Min).time().unwrap(),
                sta.arrival(ck, Edge::Rise, MinMax::Max).unwrap(),
            )
        });

    assert!(latency > Seconds::new(1e-12));
    assert!(is_ns(setup - latency, 2.0 - 0.04 - 0.3), "{setup:?}");
    assert!(is_ns(hold + latency, 0.3 - 0.02), "{hold:?}");
}

#[test]
fn a_clock_uncertainty_takes_its_time_from_setup_and_hold_slacks_alike() {
    let db = flop_to_flop_design();
    let d = cell_pin(&db, "f2/D");
    let uncertain = format!("{CLOCK_2NS}set_clock_uncertainty 0.1 [get_clocks clk]\n");

    let slacks = |constraints: &str| {
        timed(&db, constraints, |_, sta| {
            (
                sta.slack(d, MinMax::Max).time().unwrap(),
                sta.slack(d, MinMax::Min).time().unwrap(),
            )
        })
    };
    let ((setup, hold), (uncertain_setup, uncertain_hold)) =
        (slacks(CLOCK_2NS), slacks(&uncertain));

    assert!(is_ns(setup - uncertain_setup, 0.1));
    assert!(is_ns(hold - uncertain_hold, 0.1));
}

#[test]
fn a_hold_multicycle_gives_back_its_cycles_to_the_setup_ones() {
    let db = flop_to_flop_design();
    let d = cell_pin(&db, "f2/D");
    let setup_only = format!(
        "{CLOCK_2NS}set_multicycle_path 3 -setup -from [get_clocks clk] -to [get_clocks clk]\n"
    );
    let with_hold = format!(
        "{setup_only}set_multicycle_path 1 -hold -from [get_clocks clk] -to [get_clocks clk]\n"
    );
    let hold_slack = |constraints: &str| {
        timed(&db, constraints, |_, sta| {
            sta.slack(d, MinMax::Min).time().unwrap()
        })
    };

    let (plain, shifted) = (hold_slack(CLOCK_2NS), hold_slack(&with_hold));

    // Two extra setup cycles, one of them taken back by the hold.
    assert!(is_ns(plain - shifted, 2.2), "{plain:?} {shifted:?}");
}

#[test]
fn clocks_without_a_common_period_are_remembered_once_as_unaligned() {
    let db = flop_to_flop_design();
    let constraints = "create_clock -name a -period 1.0 [get_ports clk]\n\
                       create_clock -name b -period 1.4142135623731 [get_ports in]\n";

    let unaligned = timed(&db, constraints, |_, sta| {
        let mut again = sta.clone();
        again.accountings.clear();
        again.unaligned_clocks.clear();
        let _ = again.accounting(edge(0, Edge::Rise), edge(1, Edge::Rise));
        let _ = again.accounting(edge(0, Edge::Rise), edge(0, Edge::Fall));
        again.unaligned_clocks
    });

    assert_eq!(unaligned.len(), 1);
    assert!(unaligned.contains(&(ClockId(0), ClockId(1))));
}

/// The clock `clk` through buffer `cb` to the gate `g` that gates it with
/// the port `en`, on its way to flop `f`.
fn buffered_gated_clock_design() -> NetlistDB {
    design_with_copies(
        &def_text(
            "gated",
            &DefSections {
                components: "- cb BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - g GATE + PLACED ( 4000 2800 ) N ;\n\
                             - f DFF_X1 + PLACED ( 6000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( g A ) ;\n\
                       - en ( PIN en ) ( g B ) ;\n- gck ( g Z ) ( f CK ) ;\n",
                inputs: &["clk", "en"],
                outputs: &[],
            },
        ),
        &[("AND2_X1", "GATE")],
    )
}

#[test]
fn a_gated_clocks_enable_is_checked_against_the_gate_clock_as_it_arrives() {
    let db = buffered_gated_clock_design();
    let (enable, gate_clock) = (cell_pin(&db, "g/B"), cell_pin(&db, "g/A"));
    let ideal = "create_clock -name clk -period 2.2 [get_ports clk]\n\
                 set_input_delay 0.3 -clock clk [get_ports en]\n";
    let propagated = format!("{ideal}set_propagated_clock [get_clocks clk]\n");
    let read = |constraints: &str| {
        timed_with(&db, &cells_with(GATE), constraints, |_, sta| {
            (
                sta.slack(enable, MinMax::Max).time().unwrap(),
                sta.arrival(gate_clock, Edge::Rise, MinMax::Max).unwrap(),
            )
        })
    };

    let ((ideal_slack, _), (late_slack, latency)) = (read(ideal), read(&propagated));

    assert!(latency > Seconds::new(1e-12));
    assert!(is_ns(late_slack - ideal_slack, latency.get() * 1e9));
}

/// A clock of `period`, `rise` and `fall`, in nanoseconds.
fn clock_ns(name: &str, period: f64, rise: f64, fall: f64) -> Clock {
    clock(name, period * 1e-9, rise * 1e-9, fall * 1e-9)
}

/// Clocks as `(period, rise, fall)` in nanoseconds, with edges before, at
/// and after the period, equal edges, and periods no other divides.
const CYCLE_CLOCKS: [(f64, f64, f64); 14] = [
    (1.0, 0.0, 0.5),
    (2.0, 0.0, 1.0),
    (2.0, 0.5, 1.5),
    (2.0, -0.3, 0.7),
    (2.0, 0.4, 2.0),
    (2.0, 2.0, 3.0),
    (2.0, 2.6, 3.2),
    (3.0, 0.0, 1.5),
    (2.0, 0.5, 0.5),
    (4.0, 1.0, 3.0),
    (2.5, 0.25, 1.75),
    (0.0, 0.0, 0.0),
    (std::f64::consts::SQRT_2, 0.0, 0.7),
    (2.0, 1.0, 0.0),
];

/// A time in nanoseconds to the picosecond, with no sign on a zero.
fn ps(time: Seconds) -> String {
    let nanoseconds = time.get() * 1e9;
    let rounded = if nanoseconds.abs() < 5e-4 {
        0.0
    } else {
        nanoseconds
    };
    format!("{rounded:.3}")
}

#[test]
fn the_cycle_accounting_between_every_pair_of_clock_shapes_is_as_found() {
    let mut lines = String::new();
    for (i, &(p1, r1, f1)) in CYCLE_CLOCKS.iter().enumerate() {
        for (j, &(p2, r2, f2)) in CYCLE_CLOCKS.iter().enumerate() {
            let sdc = Sdc {
                clocks: vec![clock_ns("a", p1, r1, f1), clock_ns("b", p2, r2, f2)].into(),
                ..Sdc::default()
            };
            for (launch, target) in [
                (Edge::Rise, Edge::Rise),
                (Edge::Rise, Edge::Fall),
                (Edge::Fall, Edge::Rise),
                (Edge::Fall, Edge::Fall),
            ] {
                let (from, to) = (edge(0, launch), edge(1, target));
                let found = cycle_accounting(&sdc, from, to);
                assert_cycle_accounting_makes_sense(&sdc, from, to, found, (p1, p2));
                let _ = writeln!(
                    lines,
                    "{i}:{j} {launch:?}>{target:?} setup {} hold {} gated {} latch {} {:?} exceeded {}",
                    ps(found.setup_required),
                    ps(found.hold_required),
                    ps(found.gated_hold_required),
                    ps(found.latch_setup_required),
                    found.latch_cycles,
                    found.max_cycles_exceeded,
                );
            }
        }
    }

    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden");
    pare_common::testing::assert_golden(&dir, "b_cycle_accounting.txt", &lines);
}

/// A capture that follows its launch is after it, one at or before it is no
/// later, wherever the clocks have a period and realign.
fn assert_cycle_accounting_makes_sense(
    sdc: &Sdc,
    launch: ClockEdge,
    target: ClockEdge,
    found: Accounting,
    periods: (f64, f64),
) {
    if periods.0 <= 0.0 || periods.1 <= 0.0 || found.max_cycles_exceeded {
        return;
    }
    let launch_time = sdc.clock(launch.clock).edge_time(launch.edge);
    let slack = 1e-15;
    assert!(
        found.setup_required > launch_time - Seconds::new(slack),
        "{launch:?} {target:?}"
    );
    assert!(
        found.hold_required <= launch_time + Seconds::new(slack),
        "{launch:?} {target:?}"
    );
    assert!(
        found.setup_required >= found.hold_required,
        "{launch:?} {target:?}"
    );
}
