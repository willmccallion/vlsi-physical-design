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

/// An integer narrowed to `Self` by keeping its low bits, as C++'s
/// conversion to a narrower unsigned type does.
pub trait WrappingFrom<F> {
    /// The low bits of `value` that `Self` holds.
    fn wrapping_from(value: F) -> Self;
}

impl WrappingFrom<u32> for u8 {
    fn wrapping_from(value: u32) -> Self {
        let [low, ..] = value.to_le_bytes();
        low
    }
}

impl WrappingFrom<i64> for u8 {
    fn wrapping_from(value: i64) -> Self {
        let [low, ..] = value.to_le_bytes();
        low
    }
}

impl NearestFrom<f64> for f32 {
    // The one place a double is narrowed: `as` rounds to the nearest float.
    #[allow(clippy::cast_possible_truncation)]
    fn nearest_from(value: f64) -> Self {
        value as Self
    }
}

#[cfg(test)]
mod tests;
