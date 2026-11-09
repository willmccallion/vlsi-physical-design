//! The coordinates of `OpenDP`'s grid, as dpl's `Coordinates.h` types them:
//! a site column, a row, and points and boxes of the two.
//!
//! As in dpl, a coordinate and a span along one axis share a type.
//! Converting to and from database units goes through the methods that
//! name the rounding, never arithmetic on the raw values.

use pare_common::geom::dbu::Dbu;
use std::ops::{Add, Mul, Sub};

/// Defines one axis of the grid, as an instance of dpl's `TypedCoordinate`.
macro_rules! grid_axis {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
        pub struct $name(i64);

        impl $name {
            /// The grid's first line along this axis.
            pub const ZERO: Self = Self(0);

            /// The coordinate `value` lines from the grid's first.
            pub const fn new(value: i64) -> Self {
                Self(value)
            }

            /// The raw number of lines.
            pub const fn get(self) -> i64 {
                self.0
            }

            /// The span's length, whichever way it runs.
            pub const fn abs(self) -> Self {
                Self(self.0.abs())
            }

            /// The coordinates from `self` up to `end`, `end` excluded.
            pub fn until(self, end: Self) -> impl DoubleEndedIterator<Item = Self> {
                (self.0..end.0).map(Self)
            }

            /// The coordinates from `self` up to `end`, `end` included.
            pub fn through(self, end: Self) -> impl Iterator<Item = Self> {
                (self.0..=end.0).map(Self)
            }
        }

        impl Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                Self(self.0 + rhs.0)
            }
        }

        impl Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self {
                Self(self.0 - rhs.0)
            }
        }

        /// A step of whole lines, as dpl adds an `int`.
        impl Add<i64> for $name {
            type Output = Self;
            fn add(self, rhs: i64) -> Self {
                Self(self.0 + rhs)
            }
        }

        /// A step of whole lines, as dpl subtracts an `int`.
        impl Sub<i64> for $name {
            type Output = Self;
            fn sub(self, rhs: i64) -> Self {
                Self(self.0 - rhs)
            }
        }
    };
}

grid_axis!(
    /// A site column of the grid, or a width in sites, as dpl's `GridX`.
    GridX
);

grid_axis!(
    /// A row of the grid, the index of the row edge at its bottom, or a
    /// height in rows, as dpl's `GridY`.
    GridY
);

impl GridX {
    /// The column holding `x`, truncating toward zero, as `Grid::gridX`
    /// and dpl's `dbuToGridFloor`, whose `divFloor` truncates.
    pub const fn from_dbu_trunc(x: Dbu, site_width: Dbu) -> Self {
        Self(x.get() / site_width.get())
    }

    /// The column holding `x`, rounding toward negative infinity.
    pub const fn from_dbu_floor(x: Dbu, site_width: Dbu) -> Self {
        Self(x.get().div_euclid(site_width.get()))
    }

    /// The first column at or right of `x`, in doubles as dpl's
    /// `dbuToGridCeil` and `Grid::gridEndX` divide.
    pub fn from_dbu_ceil(x: Dbu, site_width: Dbu) -> Self {
        Self((x.to_f64() / site_width.to_f64()).ceil() as i64)
    }

    /// The column nearest `x`, halves away from zero, in doubles as dpl's
    /// `divRound` divides.
    pub fn from_dbu_round(x: Dbu, site_width: Dbu) -> Self {
        Self((x.to_f64() / site_width.to_f64()).round() as i64)
    }

    /// The column's left edge, or the span's width, as dpl's `gridToDbu`.
    pub const fn to_dbu(self, site_width: Dbu) -> Dbu {
        Dbu(self.0 * site_width.get())
    }
}

/// A width scaled by a whole factor, as dpl scales a `GridX`'s value.
impl Mul<i64> for GridX {
    type Output = Self;
    fn mul(self, rhs: i64) -> Self {
        Self(self.0 * rhs)
    }
}

/// A point of the grid, as dpl's `GridPt`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct GridPt {
    pub x: GridX,
    pub y: GridY,
}

impl GridPt {
    pub const fn new(x: GridX, y: GridY) -> Self {
        Self { x, y }
    }
}

/// A box of grid points, `max` excluded, as dpl's `GridRect`.
#[derive(Clone, Copy, Debug)]
pub struct GridRect {
    pub min: GridPt,
    pub max: GridPt,
}

impl GridRect {
    /// Whether `p` lies in the box.
    pub fn contains(&self, p: GridPt) -> bool {
        (self.min.x..self.max.x).contains(&p.x) && (self.min.y..self.max.y).contains(&p.y)
    }
}
