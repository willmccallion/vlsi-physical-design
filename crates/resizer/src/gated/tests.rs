use super::*;
use crate::library::Link;
use crate::netlist::Constants;
use crate::testing::{DefSections, cells_with, def_text, design_with_copies};
use pare_common::db::parser::sdc;
use std::fmt::Write;
use std::path::Path;

fn f(text: &str) -> FuncExpr {
    FuncExpr::parse(text).unwrap()
}

#[test]
fn an_and_gates_its_clock_while_the_enable_is_high() {
    assert_eq!(gating(&f("(CK & EN)"), "EN", "CK"), Some(ActiveValue::One));
}

#[test]
fn an_or_gates_its_clock_while_the_enable_is_low() {
    assert_eq!(gating(&f("(CK | EN)"), "EN", "CK"), Some(ActiveValue::Zero));
}

#[test]
fn a_nand_still_gates_and_an_inverted_clock_flips_the_active_value() {
    assert_eq!(gating(&f("!(CK & EN)"), "EN", "CK"), Some(ActiveValue::One));
    assert_eq!(
        gating(&f("(!CK & EN)"), "EN", "CK"),
        Some(ActiveValue::Zero)
    );
}

#[test]
fn a_wider_and_gates_with_any_of_its_other_inputs() {
    assert_eq!(
        gating(&f("((A1 & A2) & A3)"), "A3", "A1"),
        Some(ActiveValue::One)
    );
}

#[test]
fn an_xor_or_a_mux_does_not_gate() {
    assert_eq!(gating(&f("(CK ^ EN)"), "EN", "CK"), None);
    assert_eq!(gating(&f("((S & B) | (!S & A))"), "S", "A"), None);
}

#[test]
fn setup_checks_the_leading_edge_and_hold_the_trailing_one() {
    assert_eq!(ActiveValue::One.check_edge(MinMax::Max), Edge::Rise);
    assert_eq!(ActiveValue::One.check_edge(MinMax::Min), Edge::Fall);
    assert_eq!(ActiveValue::Zero.check_edge(MinMax::Max), Edge::Fall);
    assert_eq!(ActiveValue::Zero.check_edge(MinMax::Min), Edge::Rise);
}

#[test]
fn a_clock_is_not_gated_by_an_operand_that_is_not_the_clock() {
    assert_eq!(gating(&f("(A & EN)"), "EN", "CK"), None);
    assert_eq!(gating(&f("(!EN & B)"), "EN", "CK"), None);
}

#[test]
fn an_inverted_enable_leaves_the_active_value_of_the_clock_alone() {
    assert_eq!(gating(&f("(CK & !EN)"), "EN", "CK"), Some(ActiveValue::One));
}

#[test]
fn a_clock_needs_an_operand_that_reads_the_enable() {
    assert_eq!(gating(&f("(CK & OTHER)"), "EN", "CK"), None);
}

#[test]
fn an_enable_may_be_read_deep_inside_an_operand() {
    assert_eq!(
        gating(&f("(CK & (A | EN))"), "EN", "CK"),
        Some(ActiveValue::One)
    );
    assert_eq!(
        gating(&f("(CK & !(A | EN))"), "EN", "CK"),
        Some(ActiveValue::One)
    );
    assert_eq!(
        gating(&f("(CK & (A ^ EN))"), "EN", "CK"),
        Some(ActiveValue::One)
    );
}

/// A cell of `AND2_X1`'s footprint computing `(A & B)` with an arc to its
/// output from each of the inputs `related`.
fn gate_with_arcs_from(name: &str, related: &[&str]) -> String {
    let mut arcs = String::new();
    for pin in related {
        write!(
            arcs,
            "timing () {{ related_pin : \"{pin}\" ; timing_sense : positive_unate ; \
             cell_rise (delay_t) {{ values (\"0.03, 0.30\", \"0.04, 0.31\") ; }} \
             cell_fall (delay_t) {{ values (\"0.03, 0.30\", \"0.04, 0.31\") ; }} \
             rise_transition (delay_t) {{ values (\"0.01, 0.40\", \"0.02, 0.41\") ; }} \
             fall_transition (delay_t) {{ values (\"0.01, 0.40\", \"0.02, 0.41\") ; }} }} "
        )
        .unwrap();
    }
    format!(
        "cell ({name}) {{ area : 0.8 ; \
         pin (A) {{ direction : input ; capacitance : 0.97 ; }} \
         pin (B) {{ direction : input ; capacitance : 0.97 ; }} \
         pin (Z) {{ direction : output ; function : \"(A & B)\" ; max_capacitance : 60.0 ; \
         {arcs} }} }}"
    )
}

/// The gated enables, as `(enable, clock, active value)` pin names, of a
/// gate `g` of cell `gate` with arcs from `related`, driving flop `f`'s clock, with input `a` on
/// `g/A` and input `b` on `g/B`, under `constraints`.
fn enables_of_gate(
    gate: &str,
    related: &[&str],
    constraints: &str,
) -> Vec<(String, String, ActiveValue)> {
    let db = design_with_copies(
        &def_text(
            "gate",
            &DefSections {
                components: &format!(
                    "- g {gate} + PLACED ( 2000 2800 ) N ;\n\
                     - f DFF_X1 + PLACED ( 10000 2800 ) N ;\n"
                ),
                nets: "- a ( PIN a ) ( g A ) ;\n- b ( PIN b ) ( g B ) ;\n- ck ( g Z ) ( f CK ) ;\n",
                inputs: &["a", "b"],
                outputs: &[],
            },
        ),
        &[("AND2_X1", gate)],
    );
    let lib = cells_with(&gate_with_arcs_from(gate, related));
    let sdc = sdc::parse_str(constraints, Path::new("test.sdc"), &db, &lib).unwrap();
    let link = Link::new(&db, &lib, &[]).unwrap();
    let netlist = Netlist::new(&db, &link);
    let clocks = ClockNetwork::from_sdc(&netlist, &sdc, &Constants::find(&netlist));

    let mut found: Vec<_> = gated_enables(&netlist, &clocks)
        .into_iter()
        .map(|(enable, gated)| (db.pin_name(enable), db.pin_name(gated.clock), gated.active))
        .collect();
    found.sort_by(|x, y| x.0.cmp(&y.0));
    found
}

const CLOCK_ON_A: &str = "create_clock -name c -period 1.0 [get_ports a]\n";

#[test]
fn a_gate_with_a_combinational_arc_from_its_enable_is_a_gated_enable() {
    let found = enables_of_gate("GATE_BOTH", &["A", "B"], CLOCK_ON_A);

    assert_eq!(
        found,
        [("g/B".to_string(), "g/A".to_string(), ActiveValue::One)]
    );
}

#[test]
fn an_input_without_an_arc_to_the_gated_output_is_not_an_enable() {
    assert_eq!(enables_of_gate("GATE_A_ONLY", &["A"], CLOCK_ON_A), []);
}

#[test]
fn an_input_that_carries_a_clock_is_not_an_enable() {
    let two_clocks = "create_clock -name c1 -period 1.0 [get_ports a]\n\
                      create_clock -name c2 -period 1.0 [get_ports b]\n";

    assert_eq!(enables_of_gate("GATE_BOTH", &["A", "B"], two_clocks), []);
}
