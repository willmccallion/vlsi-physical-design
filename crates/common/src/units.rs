//! Electrical quantities in SI units, typed so that a time cannot be passed
//! where a capacitance or a resistance is expected.

use std::iter::Sum;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

macro_rules! define_quantity {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
        pub struct $name(f64);

        impl $name {
            /// None of it.
            pub const ZERO: Self = Self(0.0);

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

            /// Its magnitude.
            #[must_use]
            pub const fn abs(self) -> Self {
                Self(self.0.abs())
            }

            /// The order of `f64::total_cmp`.
            #[must_use]
            pub fn total_cmp(&self, other: &Self) -> std::cmp::Ordering {
                self.0.total_cmp(&other.0)
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

define_product!(Ohms * Farads = Seconds);
define_product!(OhmsPerMeter * Meters = Ohms);
define_product!(FaradsPerMeter * Meters = Farads);

const NANOS_PER_SECOND: f64 = 1e9;
const PICOS_PER_UNIT: f64 = 1e12;
const FEMTOS_PER_UNIT: f64 = 1e15;

impl Seconds {
    /// `ns` nanoseconds.
    #[must_use]
    pub const fn from_nanos(ns: f64) -> Self {
        Self(ns * 1e-9)
    }

    /// The time in nanoseconds, as reports give it.
    #[must_use]
    pub const fn nanos(self) -> f64 {
        self.0 * NANOS_PER_SECOND
    }
}

impl Farads {
    /// `ff` femtofarads.
    #[must_use]
    pub const fn from_femto(ff: f64) -> Self {
        Self(ff * 1e-15)
    }

    /// The capacitance in femtofarads, as reports give it.
    #[must_use]
    pub const fn femto(self) -> f64 {
        self.0 * FEMTOS_PER_UNIT
    }

    /// The capacitance in picofarads, as reports give it.
    #[must_use]
    pub const fn pico(self) -> f64 {
        self.0 * PICOS_PER_UNIT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_resistance_driving_a_capacitance_gives_a_time() {
        let tau = Ohms::new(1e3) * Farads::from_femto(2.0);

        assert!((tau.nanos() - 2e-3).abs() < 1e-15);
    }

    #[test]
    fn a_time_over_a_capacitance_gives_a_resistance() {
        let resistance = Seconds::from_nanos(1.0) / Farads::from_femto(10.0);

        assert!((resistance.get() - 1e5).abs() < 1e-6);
    }

    #[test]
    fn two_quantities_of_one_kind_divide_to_a_plain_ratio() {
        let ratio = Seconds::from_nanos(3.0) / Seconds::from_nanos(2.0);

        assert!((ratio - 1.5).abs() < 1e-12);
    }
}
