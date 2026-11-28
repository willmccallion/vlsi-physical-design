use super::*;
use crate::testing::{
    DefSections, cells, cells_with, clock_cells_source, def_text, design, design_with_copies, pin,
};
use pare_common::db::liberty::Liberty;
use pare_common::db::library::SignalUse;
use pare_common::db::parser::sdc;
use std::path::Path;

/// Buffers to remove: `b0` and `b1` in a chain from port `in0` to flop
/// `f1`, `b2` from port `in1` to flop `f2`, `b3` from `f1` to port
/// `out0`, and `b4` between nets inside and outside a hierarchy; and a
/// fixed buffer `bf`, the only load of port `in2`. Port `out1` is driven
/// by `f2` and port `out2` by nothing.
fn buffered() -> NetlistDB {
    design(&def_text(
        "buffered",
        &DefSections {
            components: "- b0 BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - b1 BUF_X1 + PLACED ( 4000 2800 ) N ;\n\
                             - b2 BUF_X1 + PLACED ( 6000 2800 ) N ;\n\
                             - b3 BUF_X1 + PLACED ( 8000 2800 ) N ;\n\
                             - b4 BUF_X1 + PLACED ( 10000 2800 ) N ;\n\
                             - bf BUF_X1 + FIXED ( 12000 2800 ) N ;\n\
                             - f1 DFF_X1 + PLACED ( 20000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 30000 2800 ) N ;\n\
                             - f3 DFF_X1 + PLACED ( 40000 2800 ) N ;\n\
                             - f4 DFF_X1 + PLACED ( 50000 2800 ) N ;\n",
            nets: "- in0 ( PIN in0 ) ( b0 A ) ;\n- n0 ( b0 Z ) ( b1 A ) ;\n\
                       - n1 ( b1 Z ) ( f1 D ) ;\n\
                       - in1 ( PIN in1 ) ( b2 A ) ;\n- n2 ( b2 Z ) ( f2 D ) ;\n\
                       - q ( f1 Q ) ( b3 A ) ;\n- out0 ( PIN out0 ) ( b3 Z ) ;\n\
                       - u1/u2/x ( f3 Q ) ( b4 A ) ;\n- y ( b4 Z ) ( f4 D ) ;\n\
                       - in2 ( PIN in2 ) ( bf A ) ;\n- nf ( bf Z ) ( f3 D ) ;\n\
                       - out1 ( PIN out1 ) ( f2 Q ) ;\n- out2 ( PIN out2 ) ;\n\
                       - clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ( f3 CK ) ( f4 CK ) ;\n",
            inputs: &["in0", "in1", "in2", "clk"],
            outputs: &["out0", "out1", "out2"],
        },
    ))
}

#[test]
fn every_buffer_that_is_not_fixed_goes() {
    let mut db = buffered();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();

    let removed = remove_buffers(&mut db, &link, None).unwrap();

    assert_eq!(removed, 5);
    let left: Vec<&str> = db.cells.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(left, ["bf", "f1", "f2", "f3", "f4"]);
    let in0 = db.io_pins[db.io_pin_id("in0").unwrap()].net;
    assert_eq!(db.pins[pin(&db, "f1/D")].net, in0);
}

#[test]
fn a_buffer_with_its_own_derate_stays() {
    let mut db = buffered();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let constraints = "set_timing_derate -late 1.1 [get_cells b2]\n";
    let sdc = sdc::parse_str(constraints, Path::new("test.sdc"), &db, &lib).unwrap();

    let removed = remove_buffers(&mut db, &link, Some(&sdc)).unwrap();

    assert_eq!(removed, 4);
    assert!(db.cell_id("b2").is_some());
}

#[test]
fn the_net_with_a_port_survives_and_a_deeper_survivor_takes_the_shallower_name() {
    let mut db = buffered();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();

    let _ = remove_buffers(&mut db, &link, None).unwrap();

    let net_name = |name: &str| db.nets[db.pins[pin(&db, name)].net].name.as_str();
    assert_eq!(net_name("f1/Q"), "out0");
    assert_eq!(net_name("f4/D"), "y");
    assert_eq!(net_name("f3/Q"), "y");
    assert!(db.net_id("u1/u2/x").is_none() && db.net_id("q").is_none());
}

#[test]
fn ports_driving_or_driven_by_logic_get_a_buffer_once_the_buffers_are_gone() {
    let mut db = buffered();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let _ = remove_buffers(&mut db, &link, None).unwrap();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let buffers = link.find_buffers(&db).unwrap();
    let (clocks, constants) = {
        let netlist = Netlist::new(&db, &link);
        let constants = Constants::find(&netlist);
        let clocks = ClockNetwork::from_ports(&netlist, &["clk".to_string()], &constants).unwrap();
        (clocks, constants)
    };

    let buffered = buffer_ports(
        &mut db,
        &link,
        &buffers,
        &clocks,
        &constants,
        &mut Namer::new(),
    )
    .unwrap();

    assert_eq!(
        buffered,
        PortBuffers {
            inputs: 2,
            outputs: 2
        }
    );
    let port_net = |port: &str| db.io_pins[db.io_pin_id(port).unwrap()].net;
    assert_eq!(db.nets[port_net("in0")].name, "in0");
    assert_eq!(db.nets[port_net("in0")].pins.len(), 2);
}

#[test]
fn a_buffer_joins_the_net_it_survives_by_its_ports() {
    let db = buffered();
    let lib = cells(&clock_cells_source());
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let net = |name: &str| db.net_id(name).unwrap();

    let merged = |in_net: &str, out_net: &str| {
        let merge = merged_nets(&netlist, net(in_net), net(out_net));
        (merge.survivor, merge.removed)
    };

    // `in0` and `out0` reach ports; `n0` and `n1` do not.
    assert_eq!(merged("n0", "in0"), (net("in0"), net("n0")));
    assert_eq!(merged("in0", "n0"), (net("in0"), net("n0")));
    assert_eq!(merged("in0", "out0"), (net("in0"), net("out0")));
    assert_eq!(merged("n0", "n1"), (net("n0"), net("n1")));
}

/// An inverter `inv` from input port `a` to output port `z`, with the
/// cells `extra`.
fn port_chain(extra_cells: &str, z_net: &str, copies: &[(&str, &str)]) -> NetlistDB {
    design_with_copies(
        &def_text(
            "ports",
            &DefSections {
                components: &format!("- inv INV_X1 + PLACED ( 2000 2800 ) N ;\n{extra_cells}"),
                nets: &format!("- a ( PIN a ) ( inv A ) ;\n{z_net}"),
                inputs: &["a"],
                outputs: &["z"],
            },
        ),
        copies,
    )
}

const INV_TO_Z: &str = "- z ( inv ZN ) ( PIN z ) ;\n";

/// What `buffer_ports` adds to `db` over `lib`, by the lowest-drive buffer
/// of its library.
fn buffer_ports_of(db: &mut NetlistDB, lib: &Liberty) -> PortBuffers {
    let link = Link::new(db, lib, &[]).unwrap();
    let buffers = link.find_buffers(db).unwrap();
    let (clocks, constants) = {
        let netlist = Netlist::new(db, &link);
        let constants = Constants::find(&netlist);
        let clocks = ClockNetwork::from_ports(&netlist, &[], &constants).unwrap();
        (clocks, constants)
    };
    buffer_ports(db, &link, &buffers, &clocks, &constants, &mut Namer::new()).unwrap()
}

fn net_of(db: &NetlistDB, pin_name: &str) -> String {
    db.nets[db.pins[pin(db, pin_name)].net].name.clone()
}

#[test]
fn a_buffer_sits_between_each_port_and_the_logic_it_serves() {
    let mut db = port_chain("", INV_TO_Z, &[]);
    let lib = cells(&clock_cells_source());

    let added = buffer_ports_of(&mut db, &lib);

    assert_eq!(
        added,
        PortBuffers {
            inputs: 1,
            outputs: 1
        }
    );
    assert_eq!(net_of(&db, "input1/A"), "a");
    assert_eq!(net_of(&db, "input1/Z"), net_of(&db, "inv/A"));
    assert_eq!(net_of(&db, "output2/A"), net_of(&db, "inv/ZN"));
    assert_eq!(net_of(&db, "output2/Z"), "z");
    assert_ne!(net_of(&db, "inv/ZN"), "z");
}

#[test]
fn an_input_port_on_a_power_net_gets_no_buffer() {
    let mut db = port_chain("", INV_TO_Z, &[]);
    let net = db.net_id("a").unwrap();
    db.nets[net].signal_use = SignalUse::Power;
    let lib = cells(&clock_cells_source());

    let added = buffer_ports_of(&mut db, &lib);

    assert_eq!(
        added,
        PortBuffers {
            inputs: 0,
            outputs: 1
        }
    );
}

#[test]
fn an_output_port_on_a_ground_net_gets_no_buffer() {
    let mut db = port_chain("", INV_TO_Z, &[]);
    let net = db.net_id("z").unwrap();
    db.nets[net].signal_use = SignalUse::Ground;
    let lib = cells(&clock_cells_source());

    let added = buffer_ports_of(&mut db, &lib);

    assert_eq!(
        added,
        PortBuffers {
            inputs: 1,
            outputs: 0
        }
    );
}

#[test]
fn an_output_port_a_tie_cell_holds_gets_no_buffer() {
    let mut db = port_chain(
        "- lo TIELO + PLACED ( 6000 2800 ) N ;\n",
        "- z ( lo ZN ) ( PIN z ) ;\n- w ( inv ZN ) ;\n",
        &[("INV_X1", "TIELO")],
    );
    let lib = cells_with(
        "cell (TIELO) { area : 0.53 ; pin (ZN) { direction : output ; function : \"0\" ; } }",
    );

    let added = buffer_ports_of(&mut db, &lib);

    assert_eq!(added.outputs, 0);
    assert_eq!(added.inputs, 1);
}

#[test]
fn an_output_port_a_tristate_driver_feeds_gets_no_buffer() {
    let mut db = port_chain(
        "- tb TBUF + PLACED ( 6000 2800 ) N ;\n",
        "- z ( tb Z ) ( PIN z ) ;\n- w ( inv ZN ) ( tb A ) ( tb B ) ;\n",
        &[("AND2_X1", "TBUF")],
    );
    let lib = cells_with(
        "cell (TBUF) { area : 0.8 ; \
         pin (A) { direction : input ; capacitance : 1.0 ; } \
         pin (B) { direction : input ; capacitance : 1.0 ; } \
         pin (Z) { direction : output ; function : \"A\" ; three_state : \"B\" ; } }",
    );

    let added = buffer_ports_of(&mut db, &lib);

    assert_eq!(added.outputs, 0);
}

#[test]
fn an_output_port_a_protected_cell_drives_gets_no_buffer() {
    let mut db = port_chain(
        "- keep KEEP + PLACED ( 6000 2800 ) N ;\n",
        "- z ( keep ZN ) ( PIN z ) ;\n- w ( inv ZN ) ( keep A ) ;\n",
        &[("INV_X1", "KEEP")],
    );
    let lib = cells_with(
        "cell (KEEP) { area : 0.53 ; dont_touch : true ; \
         pin (A) { direction : input ; capacitance : 1.6 ; } \
         pin (ZN) { direction : output ; function : \"!A\" ; } }",
    );

    let added = buffer_ports_of(&mut db, &lib);

    assert_eq!(added.outputs, 0);
}

#[test]
fn an_input_port_that_feeds_a_protected_cell_gets_no_buffer() {
    let mut db = design_with_copies(
        &def_text(
            "ports",
            &DefSections {
                components: "- inv INV_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - keep KEEP + PLACED ( 6000 2800 ) N ;\n",
                nets: "- a ( PIN a ) ( inv A ) ( keep A ) ;\n\
                       - z ( inv ZN ) ( PIN z ) ;\n- w ( keep ZN ) ;\n",
                inputs: &["a"],
                outputs: &["z"],
            },
        ),
        &[("INV_X1", "KEEP")],
    );
    let lib = cells_with(
        "cell (KEEP) { area : 0.53 ; dont_touch : true ; \
         pin (A) { direction : input ; capacitance : 1.6 ; } \
         pin (ZN) { direction : output ; function : \"!A\" ; } }",
    );

    let added = buffer_ports_of(&mut db, &lib);

    assert_eq!(added.inputs, 0);
}
