use super::*;

#[test]
fn a_length_rounds_up_to_the_next_grid_step_unless_on_it() {
    assert_eq!(round_up(Dbu::new(101), Dbu::new(5)), Dbu::new(105));
    assert_eq!(round_up(Dbu::new(100), Dbu::new(5)), Dbu::new(100));
    assert_eq!(round_up(Dbu::new(-3), Dbu::new(5)), Dbu::new(0));
}
