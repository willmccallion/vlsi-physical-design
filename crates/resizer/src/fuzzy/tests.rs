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
