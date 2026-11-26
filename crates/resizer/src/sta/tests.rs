use super::*;
use crate::graph::{Estimator, PortConditions};
use crate::library::Link;
use crate::parasitics::WireRc;
use crate::testing::{DefSections, cells_with, def_text, design_with_copies, with_timed_session};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::LayerId;
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::{def, lef, liberty, sdc};
use pare_common::db::sdc::Clock;
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
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
    inspect(
        &netlist,
        &Sta::new(&netlist, &graph, &clocks, &constants, &sdc),
    )
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
