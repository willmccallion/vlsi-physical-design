use super::*;

#[test]
fn nearby_times_are_equal_and_distant_ones_are_not() {
    assert!(equal(1e-9, 1e-9 + 1e-16));
    assert!(!equal(1e-9, 1.1e-9));
    assert!(equal(0.0, 1e-16));
    assert!(!less(1e-9, 1e-9 + 1e-16));
    assert!(less(1e-9, 2e-9));
    assert!(greater_equal(1e-9, 1e-9 + 1e-16));
}

#[test]
fn a_value_is_equal_to_zero_only_below_the_zero_tolerance() {
    assert!(equal(1e-16, 0.0));
    assert!(equal(0.0, 1e-16));
    assert!(!equal(1e-15, 0.0));
    assert!(!equal(0.0, 1e-15));
}

#[test]
fn zero_accepts_values_of_either_sign_below_the_tolerance_and_nothing_larger() {
    assert!(zero(0.0));
    assert!(zero(1e-16));
    assert!(zero(-1e-16));
    assert!(!zero(1e-15));
    assert!(!zero(-1e-14));
    assert!(!zero(5.0));
}

#[test]
fn ordering_comparisons_treat_values_within_tolerance_as_tied() {
    let (a, b) = (1.0, 1.0 + 1e-9);
    assert!(!less(a, b) && !greater(b, a));
    assert!(less_equal(a, b) && less_equal(b, a));
    assert!(greater_equal(a, b) && greater_equal(b, a));
    assert!(less(1.0, 2.0) && !less(2.0, 1.0));
    assert!(greater(2.0, 1.0) && !greater(1.0, 2.0));
    assert!(less_equal(1.0, 2.0) && !less_equal(2.0, 1.0));
    assert!(greater_equal(2.0, 1.0) && !greater_equal(1.0, 2.0));
}
