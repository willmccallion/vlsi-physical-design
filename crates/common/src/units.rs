//! Electrical quantities in SI units, typed so that a time cannot be passed
//! where a capacitance or a resistance is expected.

use crate::geom::dbu::{Dbu, DbuPerMicron};
use std::iter::Sum;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

/// A quantity in base units, or a plain number, for comparisons that work
/// on any one of them.
pub trait Quantity: Copy {
    /// The value in base units.
    fn base(self) -> f64;
}

impl Quantity for f64 {
    fn base(self) -> f64 {
        self
    }
}

macro_rules! define_quantity {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
        pub struct $name(f64);

        impl $name {
            /// None of it.
            pub const ZERO: Self = Self(0.0);

            /// More than any amount, which marks the absence of a limit.
            pub const INFINITY: Self = Self(f64::INFINITY);

            /// The quantity of `si` base units.
            #[must_use]
            pub const fn new(si: f64) -> Self {
                Self(si)
            }

            /// The quantity in base units.
            #[must_use]
            pub const fn get(self) -> f64 {
                self.0
            }

            /// The larger of the two, as `f64::max` picks it.
            #[must_use]
            pub const fn max(self, other: Self) -> Self {
                Self(self.0.max(other.0))
            }

            /// The smaller of the two, as `f64::min` picks it.
            #[must_use]
            pub const fn min(self, other: Self) -> Self {
                Self(self.0.min(other.0))
            }

            /// Whether it is neither infinite nor NaN.
            #[must_use]
            pub const fn is_finite(self) -> bool {
                self.0.is_finite()
            }

            /// The same quantity without its sign.
            #[must_use]
            pub const fn abs(self) -> Self {
                Self(self.0.abs())
            }

            /// `self * factor + addend` with one rounding, as `f64::mul_add`.
            #[must_use]
            pub const fn mul_add(self, factor: f64, addend: Self) -> Self {
                Self(self.0.mul_add(factor, addend.0))
            }

            /// The order of `f64::total_cmp`.
            #[must_use]
            pub fn total_cmp(&self, other: &Self) -> std::cmp::Ordering {
                self.0.total_cmp(&other.0)
            }
        }

        impl Quantity for $name {
            fn base(self) -> f64 {
                self.0
            }
        }

        impl Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                Self(self.0 + rhs.0)
            }
        }

        impl AddAssign for $name {
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }

        impl Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self {
                Self(self.0 - rhs.0)
            }
        }

        impl SubAssign for $name {
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0;
            }
        }

        impl Neg for $name {
            type Output = Self;
            fn neg(self) -> Self {
                Self(-self.0)
            }
        }

        impl Mul<f64> for $name {
            type Output = Self;
            fn mul(self, factor: f64) -> Self {
                Self(self.0 * factor)
            }
        }

        impl Mul<$name> for f64 {
            type Output = $name;
            fn mul(self, quantity: $name) -> $name {
                $name(self * quantity.0)
            }
        }

        impl Div<f64> for $name {
            type Output = Self;
            fn div(self, divisor: f64) -> Self {
                Self(self.0 / divisor)
            }
        }

        impl Div for $name {
            type Output = f64;
            fn div(self, rhs: Self) -> f64 {
                self.0 / rhs.0
            }
        }

        impl Sum for $name {
            fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
                Self(iter.map(|q| q.0).sum())
            }
        }
    };
}

macro_rules! define_product {
    ($a:ident * $b:ident = $product:ident) => {
        impl Mul<$b> for $a {
            type Output = $product;
            fn mul(self, rhs: $b) -> $product {
                $product(self.0 * rhs.0)
            }
        }

        impl Mul<$a> for $b {
            type Output = $product;
            fn mul(self, rhs: $a) -> $product {
                $product(self.0 * rhs.0)
            }
        }

        impl Div<$a> for $product {
            type Output = $b;
            fn div(self, rhs: $a) -> $b {
                $b(self.0 / rhs.0)
            }
        }

        impl Div<$b> for $product {
            type Output = $a;
            fn div(self, rhs: $b) -> $a {
                $a(self.0 / rhs.0)
            }
        }
    };
}

define_quantity!(
    /// A time, in seconds.
    Seconds
);
define_quantity!(
    /// A capacitance, in farads.
    Farads
);
define_quantity!(
    /// A resistance, in ohms.
    Ohms
);
define_quantity!(
    /// A power, in watts.
    Watts
);

define_quantity!(
    /// A length, in meters.
    Meters
);
define_quantity!(
    /// A wire's resistance per length, in ohms per meter.
    OhmsPerMeter
);
define_quantity!(
    /// A wire's capacitance per length, in farads per meter.
    FaradsPerMeter
);

define_quantity!(
    /// A wire's resistance per length, in ohms per database unit.
    OhmsPerDbu
);
define_quantity!(
    /// A wire's capacitance per length, in farads per database unit.
    FaradsPerDbu
);

define_product!(Ohms * Farads = Seconds);
define_product!(OhmsPerMeter * Meters = Ohms);
define_product!(FaradsPerMeter * Meters = Farads);

const NANOS_PER_SECOND: f64 = 1e9;

/// Meters in a micron, the length a design's database units count in.
pub const METERS_PER_MICRON: f64 = 1e-6;

/// Microns in a meter.
pub const MICRONS_PER_METER: f64 = 1e6;

/// Farads in a picofarad.
pub const FARADS_PER_PICOFARAD: f64 = 1e-12;

/// Farads in a femtofarad.
pub const FARADS_PER_FEMTOFARAD: f64 = 1e-15;

impl Seconds {
    /// The time in nanoseconds, as reports give it.
    #[must_use]
    pub const fn nanos(self) -> f64 {
        self.0 * NANOS_PER_SECOND
    }
}

impl OhmsPerMeter {
    /// Ohms per database unit, at `dbu_per_micron` of them to a micron.
    #[must_use]
    pub fn per_dbu(self, dbu_per_micron: DbuPerMicron) -> OhmsPerDbu {
        OhmsPerDbu(self.0 * METERS_PER_MICRON / f64::from(dbu_per_micron.get()))
    }
}

impl FaradsPerMeter {
    /// Farads per database unit, at `dbu_per_micron` of them to a micron.
    #[must_use]
    pub fn per_dbu(self, dbu_per_micron: DbuPerMicron) -> FaradsPerDbu {
        FaradsPerDbu(self.0 * METERS_PER_MICRON / f64::from(dbu_per_micron.get()))
    }

    /// Farads per micron, for arithmetic kept in microns.
    #[must_use]
    pub fn per_micron(self) -> f64 {
        self.0 * METERS_PER_MICRON
    }
}

impl Meters {
    /// `length` at `dbu_per_micron` database units to a micron.
    #[must_use]
    pub fn from_dbu(length: Dbu, dbu_per_micron: DbuPerMicron) -> Self {
        Self(length.to_f64() / f64::from(dbu_per_micron.get()) * METERS_PER_MICRON)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_resistance_driving_a_capacitance_gives_a_time() {
        let tau = Ohms::new(1e3) * Farads::new(2e-15);

        assert!((tau.nanos() - 2e-3).abs() < 1e-15);
    }

    #[test]
    fn a_time_over_a_capacitance_gives_a_resistance() {
        let resistance = Seconds::new(1e-9) / Farads::new(10e-15);

        assert!((resistance.get() - 1e5).abs() < 1e-6);
    }

    #[test]
    fn a_wire_per_meter_converts_to_per_database_unit() {
        let dbu = DbuPerMicron::new(2000).unwrap();

        let res = OhmsPerMeter::new(4e6).per_dbu(dbu);
        let cap = FaradsPerMeter::new(2e-10).per_dbu(dbu);

        assert!((res.get() - 2e-3).abs() < 1e-15);
        assert!((cap.get() - 1e-19).abs() < 1e-30);
    }

    #[test]
    fn a_length_in_database_units_converts_to_meters() {
        let length = Meters::from_dbu(Dbu(4000), DbuPerMicron::new(2000).unwrap());

        assert!((length.get() - 2e-6).abs() < 1e-18);
    }

    #[test]
    fn two_quantities_of_one_kind_divide_to_a_plain_ratio() {
        let ratio = Seconds::new(3e-9) / Seconds::new(2e-9);

        assert!((ratio - 1.5).abs() < 1e-12);
    }
}
