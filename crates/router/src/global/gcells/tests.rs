use super::*;

#[test]
fn a_point_below_the_origin_lies_in_a_gcell_before_the_first() {
    assert_eq!(Gcells::holding(Dbu::new(-1), Dbu::new(100)), Gcells(-1));
    assert_eq!(Gcells::holding(Dbu::new(100), Dbu::new(100)), Gcells(1));
}

#[test]
fn covering_an_extent_takes_a_gcell_for_any_part_of_one() {
    assert_eq!(Gcells::covering(Dbu::new(200), Dbu::new(100)), Gcells(2));
    assert_eq!(Gcells::covering(Dbu::new(201), Dbu::new(100)), Gcells(3));
}
