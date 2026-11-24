//! Capacitance in the single precision `TritonCTS` sums loads in, so that
//! balancing a tree's loads rounds as it does.

use pare_common::convert::NearestFrom;
use pare_common::units::Farads;

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
    pub(crate) fn of(farads: Farads) -> Self {
        Self(f32::nearest_from(farads.get()))
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

pare_common::newtype_ops!(Farads32: Add, AddAssign, Sub, Sum);
