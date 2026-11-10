//! Comparisons with the tolerance of `OpenSTA`'s `Fuzzy.cc`.
//!
//! Two delays computed by different routes count as equal. The tolerances
//! are absolute in base units, made for seconds; `OpenROAD` applies the
//! same comparisons to capacitances and areas in places, and PARE does
//! where it follows it.

use pare_common::units::Quantity;

/// Values closer to zero than this are zero.
const ZERO_TOLERANCE: f64 = 1e-15;

/// Values closer than this fraction of the larger are equal.
const RELATIVE_TOLERANCE: f64 = 1e-6;

/// Whether two times are equal within tolerance.
#[must_use]
pub fn equal<Q: Quantity>(a: Q, b: Q) -> bool {
    equal_base(a.base(), b.base())
}

// Exact equality is the first test, as OpenSTA's; the tolerances follow.
#[allow(clippy::float_cmp)]
fn equal_base(a: f64, b: f64) -> bool {
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
pub fn zero<Q: Quantity>(a: Q) -> bool {
    let a = a.base();
    a == 0.0 || a.abs() < ZERO_TOLERANCE
}

/// Whether `a` is less than `b` by more than the tolerance.
#[must_use]
pub fn less<Q: Quantity>(a: Q, b: Q) -> bool {
    let (a, b) = (a.base(), b.base());
    a < b && !equal_base(a, b)
}

/// Whether `a` is less than or within tolerance of `b`.
#[must_use]
pub(crate) fn less_equal<Q: Quantity>(a: Q, b: Q) -> bool {
    let (a, b) = (a.base(), b.base());
    a < b || equal_base(a, b)
}

/// Whether `a` is greater than `b` by more than the tolerance.
#[must_use]
pub(crate) fn greater<Q: Quantity>(a: Q, b: Q) -> bool {
    let (a, b) = (a.base(), b.base());
    a > b && !equal_base(a, b)
}

/// Whether `a` is greater than or within tolerance of `b`.
#[must_use]
pub(crate) fn greater_equal<Q: Quantity>(a: Q, b: Q) -> bool {
    let (a, b) = (a.base(), b.base());
    a > b || equal_base(a, b)
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
