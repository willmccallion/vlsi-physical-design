//! Comparisons of times with the tolerance `OpenSTA` uses, so that two
//! delays computed by different routes count as equal.

/// Values closer to zero than this are zero.
const ZERO_TOLERANCE: f64 = 1e-15;

/// Values closer than this fraction of the larger are equal.
const RELATIVE_TOLERANCE: f64 = 1e-6;

/// Whether two times are equal within tolerance.
// Exact equality is the first test, as OpenSTA's; the tolerances follow.
#[allow(clippy::float_cmp)]
#[must_use]
pub fn equal(a: f64, b: f64) -> bool {
    if a == b {
        return true;
    }
    if a == 0.0 {
        return b.abs() < ZERO_TOLERANCE;
    }
    if b == 0.0 {
        return a.abs() < ZERO_TOLERANCE;
    }
    (a - b).abs() < RELATIVE_TOLERANCE * a.abs().max(b.abs())
}

/// Whether a time is zero within tolerance.
#[must_use]
pub fn zero(a: f64) -> bool {
    a == 0.0 || a.abs() < ZERO_TOLERANCE
}

/// Whether `a` is less than `b` by more than the tolerance.
#[must_use]
pub fn less(a: f64, b: f64) -> bool {
    a < b && !equal(a, b)
}

/// Whether `a` is less than or within tolerance of `b`.
#[must_use]
pub fn less_equal(a: f64, b: f64) -> bool {
    a < b || equal(a, b)
}

/// Whether `a` is greater than `b` by more than the tolerance.
#[must_use]
pub fn greater(a: f64, b: f64) -> bool {
    a > b && !equal(a, b)
}

/// Whether `a` is greater than or within tolerance of `b`.
#[must_use]
pub fn greater_equal(a: f64, b: f64) -> bool {
    a > b || equal(a, b)
}

#[cfg(test)]
mod tests {
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
}
