use super::*;

#[test]
fn a_cell_is_centered_in_a_box_rounding_down() {
    assert_eq!(
        centered(Interval::new(Dbu::new(0), Dbu::new(11)), Dbu::new(4)),
        Dbu::new(3)
    );
}

#[test]
fn a_cell_centered_left_of_the_origin_rounds_toward_minus_infinity() {
    assert_eq!(
        centered(Interval::new(Dbu::new(-10), Dbu::new(-1)), Dbu::new(4)),
        Dbu::new(-8)
    );
}
