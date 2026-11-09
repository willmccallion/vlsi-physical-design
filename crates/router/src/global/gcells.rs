//! Columns, rows and spans of the gcell grid.

use pare_common::geom::dbu::Dbu;
use std::ops::{Add, AddAssign, Div, Sub, SubAssign};

/// A column, row or span of the gcell grid, in gcells.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Gcells(pub(crate) i64);

impl Gcells {
    /// The gcell of side `gcell` holding a point `offset` from the grid's
    /// origin along one axis, each gcell holding its lower edge.
    pub(crate) const fn holding(offset: Dbu, gcell: Dbu) -> Self {
        Self(offset.get().div_euclid(gcell.get()))
    }

    /// How many gcells of side `gcell` it takes to cover `extent`.
    pub(crate) const fn covering(extent: Dbu, gcell: Dbu) -> Self {
        Self((extent.get() + gcell.get() - 1) / gcell.get())
    }

    /// The length of this many gcells of side `gcell`.
    pub(crate) fn length(self, gcell: Dbu) -> Dbu {
        gcell * self.0
    }

    /// Every column or row from this one to `last`, both included.
    pub(crate) fn through(self, last: Self) -> impl Iterator<Item = Self> {
        (self.0..=last.0).map(Self)
    }
}

impl From<u16> for Gcells {
    fn from(index: u16) -> Self {
        Self(i64::from(index))
    }
}

impl Add for Gcells {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign for Gcells {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Sub for Gcells {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl SubAssign for Gcells {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl Div<i64> for Gcells {
    type Output = Self;

    fn div(self, rhs: i64) -> Self {
        Self(self.0 / rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_point_below_the_origin_lies_in_a_gcell_before_the_first() {
        assert_eq!(Gcells::holding(Dbu(-1), Dbu(100)), Gcells(-1));
        assert_eq!(Gcells::holding(Dbu(100), Dbu(100)), Gcells(1));
    }

    #[test]
    fn covering_an_extent_takes_a_gcell_for_any_part_of_one() {
        assert_eq!(Gcells::covering(Dbu(200), Dbu(100)), Gcells(2));
        assert_eq!(Gcells::covering(Dbu(201), Dbu(100)), Gcells(3));
    }
}
