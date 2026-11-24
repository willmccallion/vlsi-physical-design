//! Integer database units (DBU), the exact coordinate type of a design.

use crate::convert::SaturatingFrom;
use crate::geom::point::Point;
use std::fmt;
use std::ops::{Div, Mul, Rem};

/// A length or coordinate in database units.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Dbu(i64);

impl Dbu {
    /// Zero length.
    pub const ZERO: Self = Self(0);

    /// The length or coordinate of `value` database units.
    #[must_use]
    pub const fn new(value: i64) -> Self {
        Self(value)
    }

    /// Returns the raw integer value.
    #[must_use]
    pub const fn get(self) -> i64 {
        self.0
    }

    /// Returns the absolute value.
    #[must_use]
    pub const fn abs(self) -> Self {
        Self(self.0.abs())
    }

    /// Converts to `f64` without scaling.
    #[must_use]
    pub const fn to_f64(self) -> f64 {
        self.0 as f64
    }

    /// Returns the midpoint, rounding toward negative infinity.
    #[must_use]
    pub const fn midpoint(self, other: Self) -> Self {
        Self((self.0 + other.0).div_euclid(2))
    }

    /// How many whole `step`s fit, rounding toward zero as C++ `/` does.
    #[must_use]
    pub const fn div_trunc(self, step: Self) -> i64 {
        self.0 / step.0
    }

    /// How many whole `step`s fit, rounding toward negative infinity: the
    /// index of the `step`-wide slot holding `self` for a positive `step`.
    #[must_use]
    pub const fn div_floor(self, step: Self) -> i64 {
        self.0.div_euclid(step.0)
    }

    /// How many `step`s it takes to cover `self`, rounding up, for a
    /// positive `step`.
    #[must_use]
    pub const fn div_ceil(self, step: Self) -> i64 {
        -(-self.0).div_euclid(step.0)
    }

    /// Whether `self` is a whole number of `step`s.
    #[must_use]
    pub const fn is_multiple_of(self, step: Self) -> bool {
        self.0 % step.0 == 0
    }
}

impl fmt::Display for Dbu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

crate::newtype_ops!(Dbu: Add, AddAssign, Sub, SubAssign, Neg, Mul<i64>, Sum);

impl Mul for Dbu {
    type Output = DbuArea;
    fn mul(self, rhs: Self) -> DbuArea {
        DbuArea(self.0 * rhs.0)
    }
}

/// Truncates toward zero, as C++ `/` does.
impl Div<i64> for Dbu {
    type Output = Self;
    fn div(self, rhs: i64) -> Self {
        Self(self.0 / rhs)
    }
}

/// The remainder of [`Div`]'s truncating division, as C++ `%`, so that
/// `a / b * b + a % b == a`.
impl Rem<i64> for Dbu {
    type Output = Self;
    fn rem(self, rhs: i64) -> Self {
        Self(self.0 % rhs)
    }
}

/// An area in square database units.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DbuArea(pub i64);

impl DbuArea {
    /// No area.
    pub const ZERO: Self = Self(0);

    /// Returns the raw integer value.
    #[must_use]
    pub const fn get(self) -> i64 {
        self.0
    }

    /// Converts to `f64` without scaling.
    #[must_use]
    pub const fn to_f64(self) -> f64 {
        self.0 as f64
    }

    /// The length a strip `side` wide must run to cover the area, rounded
    /// up, for a positive `side`.
    #[must_use]
    pub const fn div_ceil(self, side: Dbu) -> Dbu {
        Dbu(-(-self.0).div_euclid(side.0))
    }
}

crate::newtype_ops!(DbuArea: Add, AddAssign, Sub, Sum);

impl fmt::Display for DbuArea {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

/// How far, in database units, a converted value may sit from a grid point
/// and still count as on it: far below a unit, far above the float error of
/// a decimal value times the resolution.
const ON_GRID_NOISE: f64 = 1e-6;

/// A value converted to the nearest database unit, and whether it lay off
/// the grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Snapped<T> {
    /// The nearest grid value.
    pub value: T,
    /// Whether the value was further than float noise from it.
    pub off_grid: bool,
}

/// `scaled`, a value in database units, at its nearest whole unit, rounded
/// half away from zero, and whether it was off the grid.
fn snap_scaled(scaled: f64) -> (i64, bool) {
    // NOTE: lefin's dbdist and dbarea lround as this does, silently; PARE's
    // readers warn of the values they round.
    let rounded = scaled.round();
    (
        i64::saturating_from(rounded),
        (scaled - rounded).abs() > ON_GRID_NOISE,
    )
}

/// The number of database units per micron of a design.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct DbuPerMicron(u32);

impl DbuPerMicron {
    /// Returns `None` for a zero resolution.
    #[must_use]
    pub const fn new(value: u32) -> Option<Self> {
        if value == 0 { None } else { Some(Self(value)) }
    }

    /// Returns the raw resolution.
    #[must_use]
    pub const fn get(self) -> u32 {
        self.0
    }

    /// `microns` at the nearest DBU, as `lefin`'s `dbdist` rounds it, and
    /// whether it was off the grid.
    pub(crate) fn snap(self, microns: f64) -> Snapped<Dbu> {
        let (value, off_grid) = snap_scaled(microns * f64::from(self.0));
        Snapped {
            value: Dbu(value),
            off_grid,
        }
    }

    /// `square_microns` at the nearest square DBU, as `lefin`'s `dbarea`
    /// rounds it, and whether it was off the grid.
    pub(crate) fn snap_area(self, square_microns: f64) -> Snapped<DbuArea> {
        let per = f64::from(self.0);
        let (value, off_grid) = snap_scaled(square_microns * per * per);
        Snapped {
            value: DbuArea(value),
            off_grid,
        }
    }

    /// Converts microns to the nearest DBU, halves away from zero.
    #[must_use]
    pub fn to_dbu_rounded(self, microns: f64) -> Dbu {
        self.snap(microns).value
    }

    /// Converts DBU to microns.
    #[must_use]
    pub fn to_microns(self, dbu: Dbu) -> f64 {
        dbu.to_f64() / f64::from(self.0)
    }

    /// `p` in microns.
    #[must_use]
    pub fn point_in_microns(self, p: Point<Dbu>) -> Point<f64> {
        Point::new(self.to_microns(p.x), self.to_microns(p.y))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn truncating_division_rounds_toward_zero_and_floor_division_down() {
        assert_eq!(Dbu(7).div_trunc(Dbu(2)), 3);
        assert_eq!(Dbu(-7).div_trunc(Dbu(2)), -3);
        assert_eq!(Dbu(7).div_floor(Dbu(2)), 3);
        assert_eq!(Dbu(-7).div_floor(Dbu(2)), -4);
    }

    #[test]
    fn ceiling_division_rounds_up_unless_exact() {
        assert_eq!(Dbu(7).div_ceil(Dbu(2)), 4);
        assert_eq!(Dbu(8).div_ceil(Dbu(2)), 4);
        assert_eq!(Dbu(0).div_ceil(Dbu(2)), 0);
        assert_eq!(Dbu(-7).div_ceil(Dbu(2)), -3);
    }

    #[test]
    fn a_multiple_of_a_step_has_no_remainder_on_either_side_of_zero() {
        assert!(Dbu(-6).is_multiple_of(Dbu(3)));
        assert!(!Dbu(7).is_multiple_of(Dbu(3)));
    }

    #[test]
    fn a_strip_runs_long_enough_to_cover_the_area_rounded_up() {
        assert_eq!(DbuArea(1400).div_ceil(Dbu(70)), Dbu(20));
        assert_eq!(DbuArea(1401).div_ceil(Dbu(70)), Dbu(21));
        assert_eq!(DbuArea(0).div_ceil(Dbu(70)), Dbu(0));
    }

    const fn on_grid<T>(value: T) -> Snapped<T> {
        Snapped {
            value,
            off_grid: false,
        }
    }

    const fn off_grid<T>(value: T) -> Snapped<T> {
        Snapped {
            value,
            off_grid: true,
        }
    }

    #[test]
    fn decimal_micron_values_on_the_grid_convert_exactly() {
        let units = DbuPerMicron::new(2000).unwrap();

        assert_eq!(units.snap(0.095), on_grid(Dbu(190)));
        assert_eq!(units.snap(-0.035), on_grid(Dbu(-70)));
        assert_eq!(units.snap(296.2965), on_grid(Dbu(592_593)));
    }

    #[test]
    fn small_and_large_values_off_the_grid_round_to_the_nearest_unit() {
        let units = DbuPerMicron::new(1000).unwrap();

        assert_eq!(units.snap(0.0004), off_grid(Dbu(0)));
        assert_eq!(units.snap(0.0006), off_grid(Dbu(1)));
        assert_eq!(units.snap(296.2961), off_grid(Dbu(296_296)));
        assert_eq!(units.snap(1000.0004), off_grid(Dbu(1_000_000)));
    }

    #[test]
    fn a_value_half_way_between_units_rounds_away_from_zero() {
        let units = DbuPerMicron::new(2).unwrap();

        assert_eq!(units.snap(0.25), off_grid(Dbu(1)));
        assert_eq!(units.snap(-0.25), off_grid(Dbu(-1)));
    }

    #[test]
    fn square_microns_round_to_the_nearest_square_unit() {
        let units = DbuPerMicron::new(2000).unwrap();

        assert_eq!(units.snap_area(0.0115), on_grid(DbuArea(46_000)));
        assert_eq!(units.snap_area(0.125_000_01), off_grid(DbuArea(500_000)));
    }

    #[test]
    fn zero_resolution_is_rejected() {
        assert!(DbuPerMicron::new(0).is_none());
    }

    #[test]
    fn remainder_pairs_with_truncating_division() {
        let (quotient, remainder) = (Dbu(-3) / 2, Dbu(-3) % 2);

        assert_eq!((quotient, remainder), (Dbu(-1), Dbu(-1)));
        assert_eq!(quotient * 2 + remainder, Dbu(-3));
    }

    #[test]
    fn midpoint_rounds_down() {
        assert_eq!(Dbu(-3).midpoint(Dbu(0)), Dbu(-2));
        assert_eq!(Dbu(3).midpoint(Dbu(4)), Dbu(3));
    }
}
