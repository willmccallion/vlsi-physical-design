//! Conversions between numeric types that have no lossless `From`, each
//! named for what it does with a value the target type cannot hold.

/// A number made a `Self`, clamped to the range of `Self`.
///
/// From a float it goes toward zero, NaN made zero: what Rust's `as` does,
/// named at the call. From an integer it is the value itself whenever it
/// fits, where `as` would wrap it.
pub trait SaturatingFrom<F> {
    /// `value` clamped to the range of `Self`.
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

macro_rules! saturating_from_unsigned {
    ($from:ty => $($int:ty),+) => {
        $(
            impl SaturatingFrom<$from> for $int {
                fn saturating_from(value: $from) -> Self {
                    Self::try_from(value).unwrap_or(Self::MAX)
                }
            }
        )+
    };
}

macro_rules! saturating_from_signed {
    ($from:ty => $($int:ty),+) => {
        $(
            impl SaturatingFrom<$from> for $int {
                fn saturating_from(value: $from) -> Self {
                    Self::try_from(value).unwrap_or(if value < 0 { Self::MIN } else { Self::MAX })
                }
            }
        )+
    };
}

saturating_from_unsigned!(u32 => u8, u16, i8, i16, i32, usize, isize);
saturating_from_unsigned!(u64 => u8, u16, u32, usize, i8, i16, i32, i64, isize);
saturating_from_unsigned!(usize => u8, u16, u32, u64, i8, i16, i32, i64, isize);
saturating_from_signed!(i32 => u8, u16, u32, u64, usize, i8, i16);
saturating_from_signed!(i64 => u8, u16, u32, u64, usize, i8, i16, i32, isize);
saturating_from_signed!(isize => u8, u16, u32, u64, usize, i8, i16, i32);

impl NearestFrom<f64> for f32 {
    // The one place a double is narrowed: `as` rounds to the nearest float.
    #[allow(clippy::cast_possible_truncation)]
    fn nearest_from(value: f64) -> Self {
        value as Self
    }
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
    fn an_integer_that_fits_converts_as_it_is_and_one_that_does_not_clamps() {
        assert_eq!(
            [
                i64::saturating_from(42_usize),
                i64::saturating_from(usize::MAX)
            ],
            [42, i64::MAX]
        );
        assert_eq!(
            [
                usize::saturating_from(7_i64),
                usize::saturating_from(-3_i64)
            ],
            [7, 0]
        );
        assert_eq!(u16::saturating_from(70_000_u32), u16::MAX);
        assert_eq!(i8::saturating_from(-500_i32), i8::MIN);
    }
}
