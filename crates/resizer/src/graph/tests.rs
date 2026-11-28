use super::{Estimator, PerSide, PortConditions, TimingGraph, is_non_leaf_clock, same_slews};
use crate::bnet::LoadModel;
use crate::library::Link;
use crate::netlist::Netlist;
use crate::parasitics::WireRc;
use crate::testing::{
    DefSections, cells, cells_with, clock_cells_source, def_text, design, design_with_copies,
    metal2_wires, pin, with_session_inputs, with_timed_session, with_wired_session,
};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::PinId;
use pare_common::db::liberty::{ArcRole, Edge, EdgeValues, MinMax, SideValues};
use pare_common::db::library::SignalUse;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::units::{Farads, FaradsPerMeter, OhmsPerMeter, Seconds};

/// Whether `slew` is `nanoseconds` to within a femtosecond.
fn is_about(slew: Seconds, nanoseconds: f64) -> bool {
    (slew - Seconds::new(nanoseconds * 1e-9)).abs() < Seconds::new(1e-15)
}

/// An input port `in` through buffer `b` to an output port `out`.
fn port_to_port() -> NetlistDB {
    design(&def_text(
        "ports",
        &DefSections {
            components: "- b BUF_X1 + PLACED ( 40000 2800 ) N ;\n",
            nets: "- in ( PIN in ) ( b A ) ;\n- out ( b Z ) ( PIN out ) ;\n",
            inputs: &["in"],
            outputs: &["out"],
        },
    ))
}

/// The load buffer `b` of [`port_to_port`] drives under `constraints`.
fn buffer_load(constraints: &str) -> Farads {
    let mut db = port_to_port();
    let driver = pin(&db, "b/Z");
    let mut load = Farads::ZERO;
    with_timed_session(&mut db, &cells(&clock_cells_source()), constraints, |r| {
        load = r.load_cap(driver);
    });
    load
}

#[test]
fn a_load_set_on_an_output_port_loads_its_driver() {
    let bare = buffer_load("");

    let loaded = buffer_load("set_load 50 [get_ports out]\n");

    assert!(
        ((loaded - bare).get() - 50e-15).abs() < 1e-20,
        "{bare:?} -> {loaded:?}"
    );
}

#[test]
fn a_buffering_tree_sees_the_load_set_on_an_output_port() {
    let mut db = port_to_port();
    let out = db.io_pins[db.io_pin_id("out").unwrap()].pin;
    let constraints = "set_load 50 [get_ports out]\n";

    with_timed_session(&mut db, &cells(&clock_cells_source()), constraints, |r| {
        let cap = LoadModel::load(r, out).cap;

        assert!((cap.get() - 50e-15).abs() < 1e-20, "{cap:?}");
    });
}

#[test]
fn an_input_transition_is_the_slew_an_input_port_drives_its_loads_with() {
    let mut db = port_to_port();
    let load = pin(&db, "b/A");
    let constraints = "set_input_transition -rise 0.3 [get_ports in]\n\
                           set_input_transition -fall -max 0.2 [get_ports in]\n";

    with_timed_session(&mut db, &cells(&clock_cells_source()), constraints, |r| {
        let late = r.graph.slew_side(load, MinMax::Max);
        let early = r.graph.slew_side(load, MinMax::Min);

        assert!(is_about(late.rise, 0.3), "{late:?}");
        assert!(is_about(late.fall, 0.2), "{late:?}");
        assert!(is_about(early.rise, 0.3), "{early:?}");
        assert!(is_about(early.fall, 0.0), "{early:?}");
    });
}

#[test]
fn an_input_ports_wire_adds_its_elmore_slew_to_the_input_transition() {
    let mut db = port_to_port();
    let load = pin(&db, "b/A");
    let lib = cells(&clock_cells_source());

    let mut bare = Seconds::ZERO;
    with_wired_session(&mut db, &lib, "", |r| bare = r.graph.slew(load).rise);
    let mut driven = Seconds::ZERO;
    let constraints = "set_input_transition 0.1 [get_ports in]\n";
    with_wired_session(&mut db, &lib, constraints, |r| {
        driven = r.graph.slew(load).rise;
    });

    assert!(bare > Seconds::new(1e-15));
    assert!(
        ((driven - bare).get() - 0.1e-9).abs() < 1e-18,
        "{bare:?} -> {driven:?}"
    );
}

#[test]
fn a_loop_of_two_inverters_is_broken_at_one_arc() {
    let mut db = design(&def_text(
        "ring",
        &DefSections {
            components: "- i1 INV_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - i2 INV_X1 + PLACED ( 4000 2800 ) N ;\n",
            nets: "- a ( i1 ZN ) ( i2 A ) ;\n- b ( i2 ZN ) ( i1 A ) ;\n",
            inputs: &[],
            outputs: &[],
        },
    ));
    let arcs = [("i1/A", "i1/ZN"), ("i2/A", "i2/ZN")].map(|(a, z)| (pin(&db, a), pin(&db, z)));

    with_timed_session(&mut db, &cells(&clock_cells_source()), "", |r| {
        let broken: Vec<bool> = arcs
            .iter()
            .map(|&(a, z)| r.graph.is_loop_arc(a, z))
            .collect();

        assert_eq!(broken.iter().filter(|&&b| b).count(), 1, "{broken:?}");
    });
}

#[test]
fn a_clock_net_driving_only_buffers_is_non_leaf_and_one_reaching_a_register_is_not() {
    let mut db = design(&def_text(
        "tree",
        &DefSections {
            components: "- c1 BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - c2 BUF_X1 + PLACED ( 4000 2800 ) N ;\n\
                             - f DFF_X1 + PLACED ( 6000 2800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( c1 A ) ;\n- trunk ( c1 Z ) ( c2 A ) ;\n\
                       - leaf ( c2 Z ) ( f CK ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ));
    let (trunk, leaf) = (db.net_id("trunk").unwrap(), db.net_id("leaf").unwrap());
    for net in [trunk, leaf] {
        db.nets[net].signal_use = SignalUse::Clock;
    }
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);

    assert!(is_non_leaf_clock(&netlist, trunk));
    assert!(!is_non_leaf_clock(&netlist, leaf));
}

/// A ring of an inverter `i2` and a gate `g`, whose other input `src` drives;
/// `src` is the last pin of the design to be listed, `in` and its buffer
/// the first.
fn ring_fed_by_a_source() -> NetlistDB {
    design_with_copies(
        &def_text(
            "fed_ring",
            &DefSections {
                components: "- b BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - i2 INV_X1 + PLACED ( 4000 2800 ) N ;\n\
                             - g GATE + PLACED ( 6000 2800 ) N ;\n\
                             - s SRC + PLACED ( 8000 2800 ) N ;\n",
                nets: "- in ( PIN in ) ( b A ) ;\n- n2 ( i2 ZN ) ( g A ) ;\n\
                       - n1 ( g Z ) ( i2 A ) ;\n- en ( s Z ) ( g B ) ;\n\
                       - out ( b Z ) ( PIN out ) ;\n",
                inputs: &["in"],
                outputs: &["out"],
            },
        ),
        &[("BUF_X1", "SRC"), ("AND2_X1", "GATE")],
    )
}

/// A cell of the buffer's pins that drives with no arc into its output,
/// and an and gate timed from both of its inputs.
const SOURCE_AND_GATE: &str = "cell (SRC) { area : 0.8 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (Z) { direction : output ; function : \"A\" ; } }
    cell (GATE) { area : 1.0 ;
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
fn a_ring_is_broken_where_the_search_from_its_source_comes_back_to_its_path() {
    let mut db = ring_fed_by_a_source();
    let arcs = [("g/A", "g/Z"), ("g/B", "g/Z"), ("i2/A", "i2/ZN")]
        .map(|(a, z)| (pin(&db, a), pin(&db, z)));
    let wires = [("g/Z", "i2/A"), ("i2/ZN", "g/A")].map(|(a, z)| (pin(&db, a), pin(&db, z)));

    with_timed_session(&mut db, &cells_with(SOURCE_AND_GATE), "", |r| {
        let broken_arcs: Vec<bool> = arcs
            .iter()
            .map(|&(a, z)| r.graph.is_loop_arc(a, z))
            .collect();
        let broken_wires: Vec<bool> = wires
            .iter()
            .map(|&(a, z)| r.graph.is_loop_arc(a, z))
            .collect();

        assert_eq!(broken_arcs, [true, false, false]);
        assert_eq!(broken_wires, [false, false]);
    });
}

/// The slew `driver` ends with after the graph of `db` propagates from it,
/// with `hold` first annotated, if it is a pin, at the slew it has.
fn slew_after_propagating(db: &mut NetlistDB, driver: PinId, hold: Option<PinId>) -> Seconds {
    let mut slew = Seconds::ZERO;
    with_timed_session(db, &cells(&clock_cells_source()), "", |r| {
        if let Some(held) = hold {
            let at = r.graph.slew(held).rise;
            r.graph.annotate_slew(held, at);
        }
        let netlist = Netlist::new(r.db, r.link);
        let _ = r.graph.propagate_from(&netlist, r.clocks, &[driver]);
        slew = r.graph.slew(driver).rise;
    });
    slew
}

#[test]
fn propagating_from_a_driver_does_not_chase_its_slew_round_a_loop() {
    let mut db = design(&def_text(
        "ring",
        &DefSections {
            components: "- i1 INV_X1 + PLACED ( 2000 2800 ) N ;\n\
                         - i2 INV_X1 + PLACED ( 4000 2800 ) N ;\n",
            nets: "- a ( i1 ZN ) ( i2 A ) ;\n- b ( i2 ZN ) ( i1 A ) ;\n",
            inputs: &[],
            outputs: &[],
        },
    ));
    let cells_of_ring =
        [("i1/A", "i1/ZN"), ("i2/A", "i2/ZN")].map(|(a, z)| (pin(&db, a), pin(&db, z)));
    let mut cut = None;
    with_timed_session(&mut db, &cells(&clock_cells_source()), "", |r| {
        cut = cells_of_ring
            .into_iter()
            .find(|&(a, z)| r.graph.is_loop_arc(a, z));
    });
    let (input, driver) = cut.unwrap();

    let propagated = slew_after_propagating(&mut db, driver, None);
    let chain = slew_after_propagating(&mut db, driver, Some(input));

    assert!(
        (propagated - chain).abs() < Seconds::new(1e-18),
        "{propagated:?} vs {chain:?}"
    );
}

/// A latch `l`, its data from `b`, which `in` drives.
fn latch_behind_a_buffer() -> NetlistDB {
    design_with_copies(
        &def_text(
            "latched",
            &DefSections {
                components: "- b BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - l LAT + PLACED ( 6000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( l CK ) ;\n- in ( PIN in ) ( b A ) ;\n\
                       - d ( b Z ) ( l D ) ;\n- out ( l Q ) ( PIN out ) ;\n",
                inputs: &["clk", "in"],
                outputs: &["out"],
            },
        ),
        &[("DFF_X1", "LAT")],
    )
}

/// A latch of the clock cells' flop, open while `CK` is high.
const LATCH_CELL: &str = "cell (LAT) { area : 4.5 ;
        latch (\"IQ\", \"IQN\") { data_in : \"D\" ; enable : \"CK\" ; }
        pin (D) { direction : input ; capacitance : 1.0 ; }
        pin (CK) { direction : input ; clock : true ; capacitance : 1.0 ; }
        pin (Q) { direction : output ; function : \"IQ\" ;
          timing () { related_pin : \"D\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } }
          timing () { related_pin : \"CK\" ; timing_type : rising_edge ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } } } }";

#[test]
fn propagating_reaches_a_latchs_output_through_its_data_input() {
    let mut db = latch_behind_a_buffer();
    let (input, driver) = (pin(&db, "b/A"), pin(&db, "b/Z"));
    let q = pin(&db, "l/Q");
    let lib = cells_with(LATCH_CELL);

    with_timed_session(&mut db, &lib, "", |r| {
        let netlist = Netlist::new(r.db, r.link);
        r.graph.annotate_slew(input, Seconds::new(1e-10));

        let visited = r.graph.propagate_from(&netlist, r.clocks, &[driver]);

        assert_eq!(visited, [driver, q]);
    });
}

#[test]
fn a_load_whose_slew_is_at_its_limit_is_not_annotated() {
    let mut db = port_to_port();
    let out = db.io_pins[db.io_pin_id("out").unwrap()].pin;

    with_timed_session(&mut db, &cells(&clock_cells_source()), "", |r| {
        let netlist = Netlist::new(r.db, r.link);
        let at_limit = r.graph.slew(out).max();
        assert!(at_limit > Seconds::ZERO);

        let (held, _) = r
            .graph
            .sweep_annotating(&netlist, r.clocks, &|p| (p == out).then_some(at_limit));
        let (held_below, _) = r.graph.sweep_annotating(&netlist, r.clocks, &|p| {
            (p == out).then_some(at_limit * 0.5)
        });

        assert!(held.is_empty(), "{held:?}");
        assert_eq!(held_below, [(out, at_limit * 0.5)]);
    });
}

/// A flop `f` clocked through buffer `b`, with data from `in` and output to `out`.
fn buffered_flop() -> NetlistDB {
    design(&def_text(
        "flop",
        &DefSections {
            components: "- b BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                         - f DFF_X1 + PLACED ( 6000 2800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( b A ) ;\n- ck ( b Z ) ( f CK ) ;\n\
                   - in ( PIN in ) ( f D ) ;\n- out ( f Q ) ( PIN out ) ;\n",
            inputs: &["clk", "in"],
            outputs: &["out"],
        },
    ))
}

#[test]
fn a_loads_cell_drivers_fed_by_it_are_those_an_arc_from_it_reaches() {
    let mut db = buffered_flop();
    let pins = ["b/A", "f/CK", "f/D"].map(|name| pin(&db, name));
    let (buffer_out, flop_q) = (pin(&db, "b/Z"), pin(&db, "f/Q"));

    with_timed_session(&mut db, &cells(&clock_cells_source()), "", |r| {
        let netlist = Netlist::new(r.db, r.link);
        let fed: Vec<Vec<PinId>> = pins
            .iter()
            .map(|&p| r.graph.drivers_fed_by(&netlist, p))
            .collect();

        assert_eq!(fed, [vec![buffer_out], vec![flop_q], vec![]]);
    });
}

#[test]
fn an_inserted_buffer_sits_a_level_above_the_pin_it_loads_from() {
    let mut db = port_to_port();
    let net = db.net_id("in").unwrap();
    let load = pin(&db, "b/A");

    with_timed_session(&mut db, &cells(&clock_cells_source()), "", |r| {
        let source = r.netlist().drivers(net).next().unwrap();
        let buffer = r.buffers.lowest_drive;
        let at = Point::new(Dbu::new(20_000), Dbu::new(2800));

        let put = r
            .insert_buffer_before_loads(net, &[load], buffer, at, "split")
            .unwrap();

        assert_eq!(r.graph.level(put.input), r.graph.level(source) + 1);
        assert_eq!(r.graph.level(put.output), r.graph.level(put.input) + 1);
    });
}

#[test]
fn same_slews_tell_a_change_of_either_edge_on_either_side() {
    let base = SideValues::both(EdgeValues {
        rise: Seconds::new(1e-10),
        fall: Seconds::new(2e-10),
    });
    let changed = |side: MinMax, edge: Edge, by: f64| {
        let mut other: PerSide = base;
        let slew = other[side].get(edge);
        other[side].set(edge, slew + Seconds::new(by));
        other
    };

    assert!(same_slews(&base, &base));
    assert!(same_slews(&base, &changed(MinMax::Max, Edge::Rise, 1e-20)));
    for side in MinMax::BOTH {
        for edge in Edge::BOTH {
            assert!(
                !same_slews(&base, &changed(side, edge, 1e-11)),
                "{side:?} {edge:?}"
            );
        }
    }
}

/// A wire of `ohms` and `femtofarads` a micron on metal2.
fn wires_of(db: &NetlistDB, ohms: f64, femtofarads: f64) -> WireRc {
    WireRc {
        resistance: OhmsPerMeter::from_ohms_per_micron(ohms),
        capacitance: FaradsPerMeter::from_femtofarads_per_micron(femtofarads),
        ..metal2_wires(db)
    }
}

/// The load of the driver of each of `nets` under a graph with signal
/// wires `signal` and, if any, clock wires `clock`, in a design where
/// `clock_nets` are marked clocks.
fn loads_under(
    db: &mut NetlistDB,
    signal: WireRc,
    clock: Option<WireRc>,
    drivers: &[PinId],
) -> Vec<Farads> {
    let lib = cells(&clock_cells_source());
    with_session_inputs(db, &lib, "", signal, |inputs, _| {
        let netlist = Netlist::new(inputs.db, inputs.context.link);
        let graph = TimingGraph::new(
            &netlist,
            inputs.context.clocks,
            inputs.context.constants,
            signal,
            clock,
            Estimator::Placement,
            PortConditions::default(),
        );
        drivers
            .iter()
            .map(|&d| graph.load_cap(&netlist, d))
            .collect()
    })
}

#[test]
fn a_non_leaf_clock_net_takes_the_clock_wire_and_a_leaf_one_the_signal_wire() {
    let mut db = design(&def_text(
        "tree",
        &DefSections {
            components: "- c1 BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                         - c2 BUF_X1 + PLACED ( 60000 2800 ) N ;\n\
                         - f DFF_X1 + PLACED ( 120000 2800 ) N ;\n",
            nets: "- clk ( PIN clk ) ( c1 A ) ;\n- trunk ( c1 Z ) ( c2 A ) ;\n\
                   - leaf ( c2 Z ) ( f CK ) ;\n",
            inputs: &["clk"],
            outputs: &[],
        },
    ));
    for name in ["trunk", "leaf"] {
        let net = db.net_id(name).unwrap();
        db.nets[net].signal_use = SignalUse::Clock;
    }
    let drivers = [pin(&db, "c1/Z"), pin(&db, "c2/Z")];
    let (signal, clock) = (wires_of(&db, 3.5, 0.1), wires_of(&db, 3.5, 0.4));

    let with_clock_wire = loads_under(&mut db, signal, Some(clock), &drivers);
    let signal_only = loads_under(&mut db, signal, None, &drivers);
    let clock_everywhere = loads_under(&mut db, clock, None, &drivers);

    assert_eq!(with_clock_wire, [clock_everywhere[0], signal_only[1]]);
    assert!(clock_everywhere[0] > signal_only[0]);
}

#[test]
fn a_power_net_and_a_net_behind_an_ideal_clock_have_no_parasitics() {
    let mut db = buffered_flop();
    let (ck, data) = (db.net_id("ck").unwrap(), db.net_id("in").unwrap());
    let (power, clk) = (db.net_id("out").unwrap(), db.net_id("clk").unwrap());
    db.nets[power].signal_use = SignalUse::Power;
    let constraints = "create_clock -name clk -period 1.0 [get_ports clk]\n";

    with_wired_session(&mut db, &cells(&clock_cells_source()), constraints, |r| {
        let has = |net| r.graph.parasitics(net).is_some();

        assert!(!has(power));
        assert!(!has(clk));
        assert!(!has(ck));
        assert!(has(data));
    });
}

/// A latch whose output slew follows its data slew, over the clock cells'
/// tables.
const SLEW_FOLLOWING_LATCH: &str = "cell (LAT) { area : 4.5 ;
        latch (\"IQ\", \"IQN\") { data_in : \"D\" ; enable : \"CK\" ; }
        pin (D) { direction : input ; capacitance : 1.0 ; }
        pin (CK) { direction : input ; clock : true ; capacitance : 1.0 ; }
        pin (Q) { direction : output ; function : \"IQ\" ;
          timing () { related_pin : \"D\" ; timing_sense : positive_unate ;
            cell_rise (delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }
            cell_fall (delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }
            rise_transition (delay_t) { values (\"0.006, 0.12\", \"0.02, 0.13\") ; }
            fall_transition (delay_t) { values (\"0.006, 0.12\", \"0.02, 0.13\") ; } }
          timing () { related_pin : \"CK\" ; timing_type : rising_edge ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } } } }";

/// The slew of the output of the latch of [`latch_feeding_itself`] after
/// propagating from it, with its data input held at the slew it has first
/// when `hold_data`.
fn latch_output_slew(db: &mut NetlistDB, hold_data: bool) -> Seconds {
    let (data, q) = (pin(db, "l/D"), pin(db, "l/Q"));
    let lib = cells_with(SLEW_FOLLOWING_LATCH);
    let mut slew = Seconds::ZERO;
    with_timed_session(db, &lib, "", |r| {
        if hold_data {
            let at = r.graph.slew(data).rise;
            r.graph.annotate_slew(data, at);
        }
        let netlist = Netlist::new(r.db, r.link);
        let _ = r.graph.propagate_from(&netlist, r.clocks, &[q]);
        slew = r.graph.slew(q).rise;
    });
    slew
}

#[test]
fn a_latch_output_is_not_timed_again_for_the_data_that_its_own_slew_made() {
    let mut db = design_with_copies(
        &def_text(
            "loop",
            &DefSections {
                components: "- b BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - l LAT + PLACED ( 6000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( l CK ) ;\n- fed ( l Q ) ( b A ) ;\n\
                       - d ( b Z ) ( l D ) ;\n",
                inputs: &["clk"],
                outputs: &[],
            },
        ),
        &[("DFF_X1", "LAT")],
    );

    let propagated = latch_output_slew(&mut db, false);
    let from_held_data = latch_output_slew(&mut db, true);

    assert!(
        (propagated - from_held_data).abs() < Seconds::new(1e-18),
        "{propagated:?} vs {from_held_data:?}"
    );
}

/// The slew at the load of the input port `in` of [`port_to_port`] under
/// wires, with the library's slew derate `derate`.
fn input_slew_with_derate(derate: &str) -> Seconds {
    let source = clock_cells_source().replace(
        "slew_derate_from_library : 1.0",
        &format!("slew_derate_from_library : {derate}"),
    );
    let mut db = port_to_port();
    let load = pin(&db, "b/A");
    let mut slew = Seconds::ZERO;
    with_wired_session(&mut db, &cells(&source), "", |r| {
        slew = r.graph.slew(load).rise;
    });
    slew
}

#[test]
fn an_input_ports_wire_slew_grows_as_the_library_derate_shrinks() {
    let full = input_slew_with_derate("1.0");
    let half = input_slew_with_derate("0.5");

    assert!(full > Seconds::new(1e-15));
    assert!(((half / full) - 2.0).abs() < 1e-9, "{full:?} {half:?}");
}

/// The delay of the arc from clock pin to output of flop `f` of
/// [`buffered_flop`] when timed under `constraints`.
fn clock_to_output_delay(constraints: &str) -> Seconds {
    let mut db = buffered_flop();
    let q = pin(&db, "f/Q");
    let mut delay = Seconds::ZERO;
    with_timed_session(&mut db, &cells(&clock_cells_source()), constraints, |r| {
        delay = r.graph.arc_delays(q)[0].delay[MinMax::Max];
    });
    delay
}

#[test]
fn an_ideal_clock_launches_a_register_as_if_its_slew_were_zero() {
    let ideal = clock_to_output_delay("create_clock -name clk -period 1.0 [get_ports clk]\n");
    let propagated = clock_to_output_delay(
        "create_clock -name clk -period 1.0 [get_ports clk]\n\
         set_propagated_clock [get_clocks clk]\n",
    );
    let unclocked = clock_to_output_delay("");

    assert!(propagated > ideal, "{propagated:?} vs {ideal:?}");
    assert!(unclocked > ideal, "{unclocked:?} vs {ideal:?}");
}

/// An and cell reshaped as a three-pin one whose input `A` reaches outputs `B` and `Z`.
const MULTI_OUTPUT_CELL: &str = "cell (MULTI) { area : 1.0 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (B) { direction : output ; function : \"A\" ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.05\") ; }
            cell_fall (scalar) { values (\"0.05\") ; } } }
        pin (Z) { direction : output ; function : \"A\" ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.05\") ; }
            cell_fall (scalar) { values (\"0.05\") ; } }
          timing () { related_pin : \"A\" ; timing_type : clear ;
            timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.07\") ; }
            cell_fall (scalar) { values (\"0.07\") ; } }
          timing () { related_pin : \"A\" ; timing_sense : negative_unate ; } } }";

#[test]
fn only_the_searched_arcs_with_timing_into_an_output_are_followed_into_it() {
    let mut db = design_with_copies(
        &def_text(
            "multi",
            &DefSections {
                components: "- m MULTI + PLACED ( 2000 2800 ) N ;\n",
                nets: "- in ( PIN in ) ( m A ) ;\n- o1 ( m B ) ( PIN o1 ) ;\n\
                       - o2 ( m Z ) ( PIN o2 ) ;\n",
                inputs: &["in"],
                outputs: &["o1", "o2"],
            },
        ),
        &[("AND2_X1", "MULTI")],
    );
    let (a, b, z) = (pin(&db, "m/A"), pin(&db, "m/B"), pin(&db, "m/Z"));

    with_timed_session(&mut db, &cells_with(MULTI_OUTPUT_CELL), "", |r| {
        let multi = r
            .link
            .liberty
            .cell(r.link.liberty.cell_id("MULTI").unwrap());
        let followed = |output| {
            r.graph
                .in_arcs(output)
                .iter()
                .map(|arc| {
                    (
                        arc.from,
                        multi.arc_sets[arc.set].role,
                        multi.arc_sets[arc.set].arcs.len(),
                    )
                })
                .collect::<Vec<_>>()
        };

        assert_eq!(followed(b), [(a, ArcRole::Combinational, 2)]);
        assert_eq!(followed(z), [(a, ArcRole::Combinational, 2)]);
    });
}

#[test]
fn a_latchs_data_arc_is_the_one_levelization_does_not_follow() {
    let mut db = latch_behind_a_buffer();
    let (data, enable, q) = (pin(&db, "l/D"), pin(&db, "l/CK"), pin(&db, "l/Q"));
    let lib = cells_with(LATCH_CELL);

    with_timed_session(&mut db, &lib, "", |r| {
        let through_latch: Vec<(PinId, bool)> = r
            .graph
            .in_arcs(q)
            .iter()
            .map(|arc| (arc.from, arc.through_latch))
            .collect();

        assert_eq!(through_latch, [(data, true), (enable, false)]);
    });
}
