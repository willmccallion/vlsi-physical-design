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
fn an_integer_narrows_to_its_low_bits() {
    assert_eq!(
        [u8::wrapping_from(200_u32), u8::wrapping_from(300_u32)],
        [200, 44]
    );
    assert_eq!(
        [u8::wrapping_from(300_i64), u8::wrapping_from(-1_i64)],
        [44, 255]
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
