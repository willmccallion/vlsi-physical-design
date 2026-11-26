//! Fractional tracks of a gcell edge, in fixed point so that adding and
//! removing the same demand always cancels exactly.

use pare_common::convert::SaturatingFrom;

/// A number of tracks, possibly fractional.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Tracks(i64);

impl Tracks {
    const SCALE: i64 = 1 << 16;

    /// No tracks.
    pub const ZERO: Self = Self(0);

    /// One whole track.
    pub(crate) const ONE: Self = Self(Self::SCALE);

    /// `n` whole tracks.
    pub const fn whole(n: u32) -> Self {
        Self(n as i64 * Self::SCALE)
    }

    /// The nearest representable count to `tracks`.
    pub(crate) fn from_f64(tracks: f64) -> Self {
        Self(i64::saturating_from((tracks * Self::SCALE as f64).round()))
    }

    /// The count as a float.
    pub fn to_f64(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }

    /// The count rounded to whole tracks, for reporting.
    pub const fn rounded(self) -> i64 {
        (self.0 + Self::SCALE / 2).div_euclid(Self::SCALE)
    }

    /// The fixed-point count, for storing in an atomic.
    pub(crate) const fn raw(self) -> i64 {
        self.0
    }

    /// The count a fixed-point value from [`Self::raw`] stands for.
    pub(crate) const fn from_raw(raw: i64) -> Self {
        Self(raw)
    }

    /// `self - other`, or zero when `other` is the larger.
    #[must_use]
    pub const fn saturating_sub(self, other: Self) -> Self {
        let d = self.0 - other.0;
        Self(if d < 0 { 0 } else { d })
    }
}

pare_common::newtype_ops!(Tracks: Add, AddAssign, Sub, SubAssign, Sum);

#[cfg(test)]
mod tests;
