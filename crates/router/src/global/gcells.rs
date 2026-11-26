//! Columns, rows and spans of the gcell grid.

use pare_common::geom::dbu::Dbu;
use std::fmt;

/// A column, row or span of the gcell grid, in gcells.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Gcells(pub(crate) i64);

impl Gcells {
    /// The gcell of side `gcell` holding a point `offset` from the grid's
    /// origin along one axis, each gcell holding its lower edge.
    pub(crate) const fn holding(offset: Dbu, gcell: Dbu) -> Self {
        Self(offset.div_floor(gcell))
    }

    /// How many gcells of side `gcell` it takes to cover `extent`.
    pub(crate) const fn covering(extent: Dbu, gcell: Dbu) -> Self {
        Self(extent.div_ceil(gcell))
    }

    /// The length of this many gcells of side `gcell`.
    pub(crate) fn length(self, gcell: Dbu) -> Dbu {
        gcell * self.0
    }

    /// The raw count.
    pub(crate) const fn get(self) -> i64 {
        self.0
    }

    /// Every column or row from this one to `last`, both included.
    pub(crate) fn through(self, last: Self) -> impl Iterator<Item = Self> {
        (self.0..=last.0).map(Self)
    }
}

impl fmt::Display for Gcells {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl From<u16> for Gcells {
    fn from(index: u16) -> Self {
        Self(i64::from(index))
    }
}

pare_common::newtype_ops!(Gcells: Add, AddAssign, Sub, SubAssign, Neg, Mul<i64>, Div<i64>);

#[cfg(test)]
mod tests;
