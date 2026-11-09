//! Columns and rows of the square buckets a spatial hash files things in.

use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use std::ops::{Add, AddAssign, Sub};

/// A column or row of a spatial hash's square buckets, counted from the
/// origin the hash measures from, or an offset between two.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct Bucket(pub(crate) i64);

impl Bucket {
    /// The bucket of side `size` holding `at` along one axis, each bucket
    /// holding its lower edge.
    const fn holding(at: Dbu, size: Dbu) -> Self {
        Self(at.get().div_euclid(size.get()))
    }

    /// The size of an offset between columns or rows, whichever way it runs.
    pub(crate) const fn abs(self) -> Self {
        Self(self.0.abs())
    }

    /// 0 for an even column or row, 1 for an odd one.
    pub(crate) const fn parity(self) -> i64 {
        self.0.rem_euclid(2)
    }

    /// The length of this many buckets of side `size`.
    pub(crate) fn length(self, size: Dbu) -> Dbu {
        size * self.0
    }

    /// Every column or row from this one to `last`, both included.
    pub(crate) fn through(self, last: Self) -> impl Iterator<Item = Self> {
        (self.0..=last.0).map(Self)
    }
}

impl Add for Bucket {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign for Bucket {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Sub for Bucket {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

/// The bucket of side `size` holding `p`.
pub(crate) const fn bucket_at(p: Point<Dbu>, size: Dbu) -> Point<Bucket> {
    Point::new(Bucket::holding(p.x, size), Bucket::holding(p.y, size))
}

/// Every bucket from `min` to `max`, both included, column by column.
pub(crate) fn buckets_between(
    min: Point<Bucket>,
    max: Point<Bucket>,
) -> impl Iterator<Item = Point<Bucket>> {
    min.x
        .through(max.x)
        .flat_map(move |x| min.y.through(max.y).map(move |y| Point::new(x, y)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_point_below_the_origin_lies_in_a_bucket_before_the_first() {
        let p = Point::new(Dbu(-1), Dbu(100));

        assert_eq!(bucket_at(p, Dbu(100)), Point::new(Bucket(-1), Bucket(1)));
    }

    #[test]
    fn buckets_between_two_corners_come_column_by_column() {
        let all: Vec<Point<Bucket>> = buckets_between(
            Point::new(Bucket(0), Bucket(0)),
            Point::new(Bucket(1), Bucket(1)),
        )
        .collect();

        let at = |x, y| Point::new(Bucket(x), Bucket(y));
        assert_eq!(all, [at(0, 0), at(0, 1), at(1, 0), at(1, 1)]);
    }
}
