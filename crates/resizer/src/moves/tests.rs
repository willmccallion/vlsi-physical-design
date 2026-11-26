use super::*;
use crate::testing::{
    cells, cells_with, clock_cells_source, design, design_with_copies, with_timed_session,
};
use pare_common::db::parser::liberty::parse_str;
use std::path::Path;

/// An AND gate whose inputs reach `ZN` with the given rise and fall
/// delays, in nanoseconds.
fn and2(a1: (f64, f64), a2: (f64, f64)) -> LibCell {
    let timing = |pin: &str, (rise, fall): (f64, f64)| {
        format!(
            "timing () {{ related_pin : \"{pin}\"; timing_sense : positive_unate;
                   cell_rise(scalar) {{ values (\"{rise}\"); }}
                   cell_fall(scalar) {{ values (\"{fall}\"); }} }}"
        )
    };
    let src = format!(
        "library (t) {{ time_unit : \"1ns\"; capacitive_load_unit (1,ff);
               cell (AND2_X1) {{
                 pin (A1) {{ direction : input; capacitance : 1.0; }}
                 pin (A2) {{ direction : input; capacitance : 1.0; }}
                 pin (ZN) {{ direction : output; function : \"(A1 & A2)\"; {} {} }}
               }} }}",
        timing("A1", a1),
        timing("A2", a2)
    );
    let lib = parse_str(&src, Path::new("t.lib")).unwrap();
    lib.cells[LibCellId(0)].clone()
}

fn candidate(cell: &LibCell) -> Option<LibPortId> {
    let slews = EdgeValues {
        rise: Seconds::ZERO,
        fall: Seconds::ZERO,
    };
    swap_candidate(
        cell,
        LibPortId::new(0),
        LibPortId::new(2),
        &[LibPortId::new(1)],
        Farads::ZERO,
        &PortSlews::default(),
        &slews,
    )
}

/// A buffer `name` on `BUF_X1`'s footprint that may drive `max_cap_ff`
/// femtofarads and slews to `slew` nanoseconds at 100 fF.
fn sized_buffer(name: &str, max_cap_ff: f64, slew: f64) -> String {
    format!(
        "cell ({name}) {{ area : 0.8 ;
               pin (A) {{ direction : input ; capacitance : 0.97 ; }}
               pin (Z) {{ direction : output ; function : \"A\" ; max_capacitance : {max_cap_ff} ;
                 timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ;
                   cell_rise (delay_t) {{ values (\"0.03, 0.30\", \"0.04, 0.31\") ; }}
                   rise_transition (delay_t) {{
                     values (\"0.0, {slew}\", \"0.0, {slew}\") ; }} }} }} }}\n"
    )
}

/// The size `half_driving_power_cell` picks for a `BUF_X1` under
/// `constraints`, among buffers `A` driving up to 30.6 fF, `B` up to
/// 30 fF and `C` up to 10 fF, `A` the weakest.
fn half_driver_under(constraints: &str) -> String {
    let sizes = [("A", 30.6, 0.9), ("B", 30.0, 0.5), ("C", 10.0, 0.45)];
    let copies: Vec<(&str, &str)> = sizes.iter().map(|&(n, _, _)| ("BUF_X1", n)).collect();
    let mut db = design_with_copies(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN sizes ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\nEND DESIGN\n",
        &copies,
    );
    let extra: String = sizes
        .iter()
        .map(|&(name, cap, slew)| sized_buffer(name, cap, slew))
        .collect();
    let lib = cells_with(&extra);
    let mut picked = String::new();
    with_timed_session(&mut db, &lib, constraints, |resizer| {
        let buf = resizer.link.liberty.cell_id("BUF_X1").unwrap();
        let half = half_driving_power_cell(resizer, buf).unwrap();
        picked.clone_from(&resizer.link.liberty.cell(half).name);
    });
    picked
}

#[test]
fn the_half_driving_size_matches_its_target_exactly_or_comes_closest() {
    assert_eq!(half_driver_under(""), "B");
}

#[test]
fn a_design_load_limit_bounds_every_size_the_half_driver_is_chosen_by() {
    assert_eq!(
        half_driver_under("set_max_capacitance 20 [current_design]\n"),
        "C"
    );
}

#[test]
fn an_isolation_cell_has_no_equivalent_inputs() {
    let gate = and2((0.05, 0.05), (0.05, 0.05));
    let src = "library (t) {
               cell (ISO) { is_isolation_cell : true ;
                 pin (A1) { direction : input; capacitance : 1.0; }
                 pin (A2) { direction : input; capacitance : 1.0; }
                 pin (ZN) { direction : output; function : \"(A1 & A2)\"; } } }";
    let isolation = parse_str(src, Path::new("t.lib")).unwrap().cells[LibCellId(0)].clone();

    assert_eq!(
        equivalent_pins(&gate, LibPortId::new(0)),
        [LibPortId::new(1)]
    );
    assert_eq!(equivalent_pins(&isolation, LibPortId::new(0)), []);
}

#[test]
fn another_input_is_judged_by_its_slowest_arc_not_its_first() {
    let cell = and2((0.05, 0.05), (0.03, 0.08));

    assert_eq!(candidate(&cell), None);
}

#[test]
fn another_input_faster_on_every_arc_is_taken() {
    let cell = and2((0.05, 0.05), (0.04, 0.03));

    assert_eq!(candidate(&cell), Some(LibPortId::new(1)));
}

/// A flop launching through an inverter into another flop.
fn inverted_pair() -> pare_common::db::design::NetlistDB {
    design(
        "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN inv ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 3 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - i1 INV_X1 + PLACED ( 20000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 40000 2800 ) N ;\n\
             END COMPONENTS\nPINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 3 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- a ( f1 Q ) ( i1 A ) ;\n\
             - b ( i1 ZN ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n",
    )
}

/// The inverter's input slews unchanged and its inputs arriving
/// `adjust_ns` later, judged by `rule`.
fn later_by(adjust_ns: f64, rule: SlackRule) -> SlewImpact {
    let slew = EdgeValues::both(Seconds::new(0.02e-9));
    SlewImpact {
        old: slew,
        new: slew,
        adjust: Seconds::new(adjust_ns * 1e-9),
        margin: Seconds::ZERO,
        rule,
    }
}

#[test]
fn an_unchanged_arrival_keeps_the_no_worse_rule_under_a_margin() {
    let impact = SlewImpact {
        margin: Seconds::new(0.05e-9),
        ..later_by(0.0, SlackRule::NoWorse)
    };

    assert_eq!(inverter_keeps(&[impact]), [true]);
}

/// Whether the inverter's output keeps its slack under each impact,
/// with a clock of 1 ns.
fn inverter_keeps(impacts: &[SlewImpact]) -> Vec<bool> {
    let mut db = inverted_pair();
    let mut kept = Vec::new();
    let constraints = "create_clock -name c -period 1 [get_ports clk]\n";
    with_timed_session(
        &mut db,
        &cells(&clock_cells_source()),
        constraints,
        |resizer| {
            let inverter = resizer.db.cell_id("i1").unwrap();
            kept = impacts
                .iter()
                .map(|impact| input_slew_impact_ok(resizer, inverter, impact))
                .collect();
        },
    );
    kept
}

#[test]
fn a_later_arrival_breaks_the_no_worse_rule_but_not_a_positive_slack() {
    let impacts = [
        later_by(0.01, SlackRule::NoWorse),
        later_by(0.01, SlackRule::NonNegative),
    ];

    assert_eq!(inverter_keeps(&impacts), [false, true]);
}

#[test]
fn an_arrival_past_the_slack_breaks_the_non_negative_rule() {
    assert_eq!(
        inverter_keeps(&[later_by(10.0, SlackRule::NonNegative)]),
        [false]
    );
}

#[test]
fn an_earlier_arrival_keeps_both_rules() {
    let impacts = [
        later_by(-0.01, SlackRule::NoWorse),
        later_by(-0.01, SlackRule::NonNegative),
    ];

    assert_eq!(inverter_keeps(&impacts), [true, true]);
}

#[test]
fn the_critical_input_is_judged_by_its_slowest_arc() {
    let cell = and2((0.02, 0.06), (0.05, 0.05));

    assert_eq!(candidate(&cell), Some(LibPortId::new(1)));
}
