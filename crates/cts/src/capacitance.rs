//! Capacitance in the single precision `TritonCTS` sums loads in, so that
//! balancing a tree's loads rounds as it does.

use pare_common::units::Farads;
use std::iter::Sum;
use std::ops::{Add, AddAssign, Sub};

/// A capacitance in farads, held in single precision.
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub(crate) struct Farads32(f32);

impl Farads32 {
    /// No capacitance.
    pub(crate) const ZERO: Self = Self(0.0);

    /// The least positive capacitance, which a search for the largest load
    /// starts from as `std::numeric_limits<float>::min()` does.
    pub(crate) const MIN_POSITIVE: Self = Self(f32::MIN_POSITIVE);

    /// `farads` rounded to single precision.
    #[must_use]
    pub(crate) const fn of(farads: Farads) -> Self {
        Self(farads.get() as f32)
    }

    /// The capacitance in farads.
    #[must_use]
    pub(crate) const fn get(self) -> f32 {
        self.0
    }

    /// The size of the capacitance, whatever its sign.
    #[must_use]
    pub(crate) const fn abs(self) -> Self {
        Self(self.0.abs())
    }

    /// The total order of `f32::total_cmp`.
    #[must_use]
    pub(crate) fn total_cmp(self, other: Self) -> std::cmp::Ordering {
        self.0.total_cmp(&other.0)
    }
}

impl Add for Farads32 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign for Farads32 {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Sub for Farads32 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl Sum for Farads32 {
    fn sum<I: Iterator<Item = Self>>(loads: I) -> Self {
        Self(loads.map(|c| c.0).sum())
    }
}
