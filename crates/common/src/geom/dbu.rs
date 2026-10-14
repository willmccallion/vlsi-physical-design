//! Integer database units (DBU), the exact coordinate type of a design.

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
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self(self.0 * rhs.0)
    }
}

impl Div<i64> for Dbu {
    type Output = Self;
    fn div(self, rhs: i64) -> Self {
        Self(self.0 / rhs)
    }
}

impl Rem<i64> for Dbu {
    type Output = Self;
    fn rem(self, rhs: i64) -> Self {
        Self(self.0.rem_euclid(rhs))
    }
}

impl Sum for Dbu {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        Self(iter.map(|d| d.0).sum())
    }
}

/// A micron value that is not a whole number of database units.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
#[error("{microns} um is not a multiple of 1/{dbu_per_micron} um")]
pub struct OffGrid {
    /// The value that failed to convert.
    pub microns: f64,
    /// The database resolution it was converted at.
    pub dbu_per_micron: u32,
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

    /// Converts microns to DBU, failing if the value is not on the DBU grid.
    pub fn to_dbu(self, microns: f64) -> Result<Dbu, OffGrid> {
        let scaled = microns * f64::from(self.0);
        let rounded = scaled.round();
        if (scaled - rounded).abs() > 1e-6 * scaled.abs().max(1.0) {
            return Err(OffGrid {
                microns,
                dbu_per_micron: self.0,
            });
        }
        Ok(Dbu(rounded as i64))
    }

    /// Converts microns to the nearest DBU.
    #[must_use]
    pub fn to_dbu_rounded(self, microns: f64) -> Dbu {
        Dbu((microns * f64::from(self.0)).round() as i64)
    }

    /// Converts DBU to microns.
    #[must_use]
    pub fn to_microns(self, dbu: Dbu) -> f64 {
        dbu.to_f64() / f64::from(self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_micron_values_convert_to_dbu() {
        let units = DbuPerMicron::new(2000).unwrap();

        assert_eq!(units.to_dbu(0.095), Ok(Dbu(190)));
        assert_eq!(units.to_dbu(-0.035), Ok(Dbu(-70)));
    }

    #[test]
    fn values_finer_than_a_dbu_are_rejected() {
        let units = DbuPerMicron::new(1000).unwrap();

        assert!(units.to_dbu(0.0005).is_err());
    }

    #[test]
    fn zero_resolution_is_rejected() {
        assert!(DbuPerMicron::new(0).is_none());
    }

    #[test]
    fn midpoint_rounds_down() {
        assert_eq!(Dbu(-3).midpoint(Dbu(0)), Dbu(-2));
        assert_eq!(Dbu(3).midpoint(Dbu(4)), Dbu(3));
    }
}
