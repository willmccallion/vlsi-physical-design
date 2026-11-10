//! Integer database units (DBU), the exact coordinate type of a design.

use crate::geom::point::Point;
use std::fmt;
use std::iter::Sum;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Rem, Sub, SubAssign};

/// A length or coordinate in database units.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Dbu(pub i64);

impl Dbu {
    /// Zero length.
    pub const ZERO: Self = Self(0);

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
}

impl fmt::Display for Dbu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl Add for Dbu {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign for Dbu {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Sub for Dbu {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl SubAssign for Dbu {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl Neg for Dbu {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

impl Mul<i64> for Dbu {
    type Output = Self;
    fn mul(self, rhs: i64) -> Self {
        Self(self.0 * rhs)
    }
}

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

impl Sum for Dbu {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        Self(iter.map(|d| d.0).sum())
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
}

impl Add for DbuArea {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign for DbuArea {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Sum for DbuArea {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        Self(iter.map(|a| a.0).sum())
    }
}

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
    (rounded as i64, (scaled - rounded).abs() > ON_GRID_NOISE)
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
