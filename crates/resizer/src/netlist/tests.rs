use super::*;
use crate::testing::{DefSections, cells_with, def_text, design_with_copies, pin};
use pare_common::db::liberty::Liberty;
use pare_common::db::library::{LayerRef, Shape};
use pare_common::db::parser::sdc;
use pare_common::geom::rect::Rect;
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

/// Tie-high and tristate cells on the clock cells' footprints, and a clock
/// gate that has no function on its output.
const LEVEL_CELLS: &str = "\
    cell (TIEHI) { area : 0.53 ; \
      pin (ZN) { direction : output ; function : \"1\" ; } } \
    cell (TBUF) { area : 0.8 ; \
      pin (A) { direction : input ; capacitance : 1.0 ; } \
      pin (B) { direction : input ; capacitance : 1.0 ; } \
      pin (Z) { direction : output ; function : \"A\" ; three_state : \"B\" ; \
        timing () { related_pin : \"A\" ; timing_sense : positive_unate ; \
          cell_rise (delay_t) { values (\"0.03, 0.30\", \"0.04, 0.31\") ; } } } } \
    cell (ICG) { area : 0.8 ; \
      pin (A) { direction : input ; capacitance : 1.0 ; clock_gate_clock_pin : true ; } \
      pin (B) { direction : input ; capacitance : 1.0 ; clock_gate_enable_pin : true ; } \
      pin (Z) { direction : output ; clock_gate_out_pin : true ; } }";

/// Tie cells `hi` and `lo`, tristate buffers `tb1` to `tb5` and clock
/// gates `icg1` to `icg3`, wired as the test cases of `levels` read.
fn levels() -> (NetlistDB, Liberty) {
    let db = design_with_copies(
        &def_text(
            "levels",
            &DefSections {
                components: "- hi TIEHI + PLACED ( 2000 2800 ) N ;\n\
                             - lo TIELO + PLACED ( 4000 2800 ) N ;\n\
                             - tb1 TBUF + PLACED ( 6000 2800 ) N ;\n\
                             - tb2 TBUF + PLACED ( 8000 2800 ) N ;\n\
                             - tb3 TBUF + PLACED ( 10000 2800 ) N ;\n\
                             - tb4 TBUF + PLACED ( 12000 2800 ) N ;\n\
                             - tb5 TBUF + PLACED ( 14000 2800 ) N ;\n\
                             - icg1 ICG + PLACED ( 16000 2800 ) N ;\n\
                             - icg2 ICG + PLACED ( 18000 2800 ) N ;\n\
                             - icg3 ICG + PLACED ( 22000 2800 ) N ;\n",
                nets: "- h ( hi ZN ) ( tb1 A ) ( tb1 B ) ( tb2 A ) ( tb3 A ) ( tb5 B ) ( icg3 B ) ;\n\
                       - l ( lo ZN ) ( tb3 B ) ( tb4 B ) ( icg1 B ) ;\n\
                       - d ( PIN d ) ( tb4 A ) ( tb5 A ) ;\n\
                       - en ( PIN en ) ( tb2 B ) ( icg2 B ) ;\n\
                       - clk ( PIN clk ) ( icg1 A ) ( icg2 A ) ( icg3 A ) ;\n\
                       - z1 ( tb1 Z ) ;\n- z2 ( tb2 Z ) ;\n- z3 ( tb3 Z ) ;\n\
                       - z4 ( tb4 Z ) ;\n- z5 ( tb5 Z ) ;\n\
                       - g1 ( icg1 Z ) ;\n- g2 ( icg2 Z ) ;\n- g3 ( icg3 Z ) ;\n",
                inputs: &["d", "en", "clk"],
                outputs: &[],
            },
        ),
        &[
            ("INV_X1", "TIEHI"),
            ("INV_X1", "TIELO"),
            ("AND2_X1", "TBUF"),
            ("AND2_X1", "ICG"),
        ],
    );
    (db, cells_with(&format!("{TIE_LOW}{LEVEL_CELLS}")))
}

#[test]
fn a_tie_high_cell_holds_its_output_high() {
    let (db, lib) = levels();
    let link = Link::new(&db, &lib, &[]).unwrap();

    let constants = Constants::find(&Netlist::new(&db, &link));

    assert_eq!(constants.level(pin(&db, "hi/ZN")), Logic::One);
    assert_eq!(constants.level(pin(&db, "tb1/A")), Logic::One);
    assert!(constants.is_constant(pin(&db, "hi/ZN")));
    assert!(constants.is_constant(pin(&db, "lo/ZN")));
}

#[test]
fn a_tristate_output_is_known_only_while_its_enable_is_known_high() {
    let (db, lib) = levels();
    let link = Link::new(&db, &lib, &[]).unwrap();

    let constants = Constants::find(&Netlist::new(&db, &link));

    assert_eq!(constants.level(pin(&db, "tb1/Z")), Logic::One);
    assert_eq!(constants.level(pin(&db, "tb2/Z")), Logic::Unknown);
    assert_eq!(constants.level(pin(&db, "tb3/Z")), Logic::Unknown);
}

#[test]
fn a_clock_gate_output_is_low_while_its_enable_is_low_and_unknown_otherwise() {
    let (db, lib) = levels();
    let link = Link::new(&db, &lib, &[]).unwrap();

    let constants = Constants::find(&Netlist::new(&db, &link));

    assert_eq!(constants.level(pin(&db, "icg1/Z")), Logic::Zero);
    assert_eq!(constants.level(pin(&db, "icg2/Z")), Logic::Unknown);
    // Its enable is known high but its clock is not.
    assert_eq!(constants.level(pin(&db, "icg3/Z")), Logic::Unknown);
}

#[test]
fn a_tristate_arc_is_disabled_while_its_enable_is_held_low() {
    let (db, lib) = levels();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let constants = Constants::find(&netlist);
    let (_, tbuf) = link.cell(&db, db.cell_id("tb1").unwrap()).unwrap();
    let (a, z) = (tbuf.port("A").unwrap(), tbuf.port("Z").unwrap());
    let enabled =
        |instance: &str| constants.arc_enabled(&netlist, db.cell_id(instance).unwrap(), a, z);

    assert!(!enabled("tb4"), "enable held low");
    assert!(enabled("tb5"), "enable held high");
    assert!(!enabled("tb1"), "data held");
}

#[test]
fn a_power_or_ground_net_is_special_and_a_signal_net_is_not() {
    let (mut db, _) = levels();
    let net = db.net_id("h").unwrap();
    let other = db.net_id("l").unwrap();
    db.nets[net].signal_use = SignalUse::Power;
    db.nets[other].signal_use = SignalUse::Signal;
    let lib = cells_with("");
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);

    assert!(netlist.is_special(net));
    assert!(!netlist.is_special(other));
}

#[test]
fn a_pin_of_several_shapes_sits_at_the_mean_of_their_centers() {
    let (mut db, lib) = levels();
    let shape = |x0, y0, x1, y1| Shape {
        layer: LayerRef::Routing(pare_common::db::indices::LayerId::new(0)),
        rect: Rect::new(
            Point::new(Dbu::new(x0), Dbu::new(y0)),
            Point::new(Dbu::new(x1), Dbu::new(y1)),
        ),
    };
    let master = db.library.macro_id("TBUF").unwrap();
    let macro_pin = db.library.macros[master]
        .pins
        .iter_mut()
        .find(|p| p.name == "A")
        .unwrap();
    macro_pin.shapes = vec![shape(0, 0, 100, 100), shape(300, 0, 500, 200)];
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);

    let at = netlist.pin_location(pin(&db, "tb1/A"));

    // The centers (50, 50) and (400, 100) average to (225, 75) from the
    // origin of the instance at (6000, 2800).
    assert_eq!(at, Point::new(Dbu::new(6225), Dbu::new(2875)));
}

#[test]
fn an_ideal_clock_reaches_the_pins_it_clocks_and_a_propagated_one_does_not() {
    let create = "create_clock -name core -period 1.0 [get_ports clk]\n";
    let (db, ideal) = sdc_clocks(create);
    let (_, propagated) = sdc_clocks(&format!("{create}set_propagated_clock [all_clocks]\n"));

    let ck = pin(&db, "f1/CK");
    assert!(ideal.carries_ideal_clock(ck));
    assert!(!propagated.carries_ideal_clock(ck));
    assert!(!ideal.carries_ideal_clock(pin(&db, "f1/D")));
}
