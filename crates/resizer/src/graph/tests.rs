use super::is_non_leaf_clock;
use crate::bnet::LoadModel;
use crate::library::Link;
use crate::netlist::Netlist;
use crate::testing::{
    DefSections, cells, clock_cells_source, def_text, design, pin, with_timed_session,
    with_wired_session,
};
use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::MinMax;
use pare_common::db::library::SignalUse;
use pare_common::units::{Farads, Seconds};

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
