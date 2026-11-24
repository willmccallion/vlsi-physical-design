//! Conversions between numeric types that have no lossless `From`, each
//! named for what it does with a value the target type cannot hold.

/// A float made an integer of `Self` toward zero, saturating at the bounds
/// of `Self`, NaN made zero: what Rust's `as` does, named at the call.
pub trait SaturatingFrom<F> {
    /// `value` toward zero, clamped to the range of `Self`.
    fn saturating_from(value: F) -> Self;
}

/// A float narrowed to `Self` at the nearest value it holds, as C++'s
/// conversion of a `double` to a `float` rounds.
pub trait NearestFrom<F> {
    /// `value` rounded to the nearest `Self`.
    fn nearest_from(value: F) -> Self;
}

macro_rules! saturating_from_floats {
    ($($int:ty),+) => {
        $(
            impl SaturatingFrom<f64> for $int {
                // The one place the saturating float-to-integer `as` is used.
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                fn saturating_from(value: f64) -> Self {
                    value as Self
                }
            }

            impl SaturatingFrom<f32> for $int {
                // The one place the saturating float-to-integer `as` is used.
                #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
                fn saturating_from(value: f32) -> Self {
                    value as Self
                }
            }
        )+
    };
}

saturating_from_floats!(i8, i16, i32, i64, isize, u8, u16, u32, u64, usize);

impl NearestFrom<f64> for f32 {
    // The one place a double is narrowed: `as` rounds to the nearest float.
    #[allow(clippy::cast_possible_truncation)]
    fn nearest_from(value: f64) -> Self {
        value as Self
    }
}

/// A count of things held in memory, such as a slice's length, as an
/// `i64`: such counts never exceed `isize::MAX`, which `i64` holds on every
/// target.
#[must_use]
pub fn count_i64(count: usize) -> i64 {
    i64::try_from(count).unwrap_or(i64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_float_goes_toward_zero_and_saturates_at_the_bounds() {
        assert_eq!(
            [
                i64::saturating_from(2.9_f64),
                i64::saturating_from(-2.9_f64),
                i64::saturating_from(f64::NAN),
                i64::saturating_from(1e300_f64),
            ],
            [2, -2, 0, i64::MAX]
        );
        assert_eq!(
            [
                u32::saturating_from(-1.0_f64),
                u32::saturating_from(5e9_f32)
            ],
            [0, u32::MAX]
        );
    }

    #[test]
    fn a_double_narrows_to_the_nearest_float() {
        assert_eq!(f32::nearest_from(0.1_f64).to_bits(), 0.1_f32.to_bits());
    }

    #[test]
    fn a_count_converts_as_it_is() {
        assert_eq!(count_i64(42), 42);
    }
}
