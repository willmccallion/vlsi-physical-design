//! Fractional tracks of a gcell edge, in fixed point so that adding and
//! removing the same demand always cancels exactly.

use std::iter::Sum;
use std::ops::{Add, AddAssign, Sub, SubAssign};

/// A number of tracks, possibly fractional.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Tracks(i64);

impl Tracks {
    const SCALE: i64 = 1 << 16;

    /// No tracks.
    pub const ZERO: Self = Self(0);

    /// One whole track.
    pub const ONE: Self = Self(Self::SCALE);

    /// `n` whole tracks.
    pub const fn whole(n: u32) -> Self {
        Self(n as i64 * Self::SCALE)
    }

    /// The nearest representable count to `tracks`.
    pub fn from_f64(tracks: f64) -> Self {
        Self((tracks * Self::SCALE as f64).round() as i64)
    }

    /// The count as a float.
    pub fn to_f64(self) -> f64 {
        self.0 as f64 / Self::SCALE as f64
    }

    /// The count rounded to whole tracks, for reporting.
    pub fn rounded(self) -> u64 {
        u64::try_from((self.0 + Self::SCALE / 2).div_euclid(Self::SCALE)).unwrap_or(0)
    }

    /// `self - other`, or zero when `other` is the larger.
    #[must_use]
    pub const fn saturating_sub(self, other: Self) -> Self {
        let d = self.0 - other.0;
        Self(if d < 0 { 0 } else { d })
    }
}

impl Add for Tracks {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign for Tracks {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Sub for Tracks {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl SubAssign for Tracks {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl Sum for Tracks {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        iter.fold(Self::ZERO, |a, b| a + b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adding_and_removing_a_fraction_cancels_exactly() {
        let loss = Tracks::from_f64(0.175);
        let mut usage = Tracks::whole(3);

        for _ in 0..1000 {
            usage += loss;
        }
        for _ in 0..1000 {
            usage -= loss;
        }

        assert_eq!(usage, Tracks::whole(3));
    }

    #[test]
    fn rounding_reports_the_nearest_whole_track() {
        assert_eq!(Tracks::from_f64(2.49).rounded(), 2);
        assert_eq!(Tracks::from_f64(2.5).rounded(), 3);
        assert_eq!(Tracks::ZERO.rounded(), 0);
    }

    #[test]
    fn saturating_sub_stops_at_zero() {
        assert_eq!(
            Tracks::whole(2).saturating_sub(Tracks::whole(5)),
            Tracks::ZERO
        );
        assert_eq!(
            Tracks::whole(5).saturating_sub(Tracks::whole(2)),
            Tracks::whole(3)
        );
    }
}
