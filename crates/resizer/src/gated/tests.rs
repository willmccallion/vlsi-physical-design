use super::*;

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
