use super::*;
use crate::testing::{DefSections, cells_with, def_text, design_with_copies, pin};
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::sdc;
use std::path::Path;

/// A tie-low cell on the inverter's footprint.
const TIE_LOW: &str = "cell (TIELO) { area : 0.53 ; \
                           pin (ZN) { direction : output ; function : \"0\" ; } }";

/// A tie-low cell `t` driving an inverter `i` and one input of an and
/// gate `a` whose other input is the port `in0`; and the clock port
/// `clk` reaching flop `f1` through buffer `b` and flop `f2` through an
/// and gate `g` the tie-low cell holds shut.
fn tied() -> (NetlistDB, Liberty) {
    let db = design_with_copies(
        &def_text(
            "tied",
            &DefSections {
                components: "- t TIELO + PLACED ( 2000 2800 ) N ;\n\
                                 - i INV_X1 + PLACED ( 4000 2800 ) N ;\n\
                                 - a AND2_X1 + PLACED ( 6000 2800 ) N ;\n\
                                 - b BUF_X1 + PLACED ( 8000 2800 ) N ;\n\
                                 - g AND2_X1 + PLACED ( 10000 2800 ) N ;\n\
                                 - f1 DFF_X1 + PLACED ( 12000 2800 ) N ;\n\
                                 - f2 DFF_X1 + PLACED ( 20000 2800 ) N ;\n",
                nets: "- lo ( t ZN ) ( i A ) ( a B ) ( g B ) ;\n\
                           - in0 ( PIN in0 ) ( a A ) ( f1 D ) ( f2 D ) ;\n\
                           - clk ( PIN clk ) ( b A ) ( g A ) ;\n\
                           - ck1 ( b Z ) ( f1 CK ) ;\n- ck2 ( g Z ) ( f2 CK ) ;\n\
                           - hi ( i ZN ) ;\n- x ( a Z ) ;\n",
                inputs: &["in0", "clk"],
                outputs: &[],
            },
        ),
        &[("INV_X1", "TIELO")],
    );
    (db, cells_with(TIE_LOW))
}

#[test]
fn a_tie_cell_holds_what_it_drives_through_the_gates_it_decides() {
    let (db, lib) = tied();
    let link = Link::new(&db, &lib, &[]).unwrap();

    let constants = Constants::find(&Netlist::new(&db, &link));

    assert_eq!(constants.level(pin(&db, "a/B")), Logic::Zero);
    assert_eq!(constants.level(pin(&db, "i/ZN")), Logic::One);
    assert_eq!(constants.level(pin(&db, "a/Z")), Logic::Zero);
    assert_eq!(constants.level(pin(&db, "a/A")), Logic::Unknown);
    assert!(!constants.is_constant(pin(&db, "f1/D")));
}

#[test]
fn a_clock_port_reaches_flops_through_buffers_but_not_through_a_gate_held_shut() {
    let (db, lib) = tied();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let constants = Constants::find(&netlist);

    let clocks = ClockNetwork::from_ports(&netlist, &["clk".to_string()], &constants).unwrap();

    assert!(clocks.is_clock(pin(&db, "f1/CK")));
    assert_eq!(clocks.clocks_of(pin(&db, "f1/CK")), [ClockId::new(0)]);
    assert!(clocks.is_clock(pin(&db, "g/A")));
    assert!(!clocks.is_clock(pin(&db, "f2/CK")));
    assert!(!clocks.is_clock(pin(&db, "f1/D")));
    assert_eq!(
        clocks.sources(),
        [db.io_pins[db.io_pin_id("clk").unwrap()].pin]
    );
}

#[test]
fn a_clock_port_the_design_lacks_is_an_error() {
    let (db, lib) = tied();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let constants = Constants::find(&netlist);

    let clocks = ClockNetwork::from_ports(&netlist, &["nope".to_string()], &constants);

    assert_eq!(
        clocks,
        Err(NetlistError::UnknownClockPort("nope".to_string()))
    );
}

/// The clock network of [`tied`] under the SDC `constraints`.
fn sdc_clocks(constraints: &str) -> (NetlistDB, ClockNetwork) {
    let (db, lib) = tied();
    let sdc = sdc::parse_str(constraints, Path::new("test.sdc"), &db, &lib).unwrap();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let clocks = ClockNetwork::from_sdc(&netlist, &sdc, &Constants::find(&netlist));
    (db, clocks)
}

#[test]
fn an_sdc_clock_is_ideal_unless_propagated() {
    let create = "create_clock -name core -period 1.0 [get_ports clk]\n";
    let (db, ideal) = sdc_clocks(create);
    let (_, propagated) = sdc_clocks(&format!("{create}set_propagated_clock [all_clocks]\n"));

    let ck = pin(&db, "f1/CK");
    assert!(ideal.is_ideal(ck) && !ideal.is_propagated(ck));
    assert!(propagated.is_propagated(ck) && !propagated.is_ideal(ck));
}
