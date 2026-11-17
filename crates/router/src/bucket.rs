//! Columns and rows of the square buckets a spatial hash files things in.

use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use rustc_hash::FxHashMap;
use std::ops::{Add, AddAssign, Sub};

/// A column or row of a spatial hash's square buckets, counted from the
/// origin the hash measures from, or an offset between two.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct Bucket(pub(crate) i64);

impl Bucket {
    /// The bucket of side `size` holding `at` along one axis, each bucket
    /// holding its lower edge.
    const fn holding(at: Dbu, size: Dbu) -> Self {
        Self(at.div_floor(size))
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

/// Values filed by the square bucket they lie in: buckets of side `side`,
/// counted from `origin`.
#[derive(Clone, Debug)]
pub(crate) struct SpatialHash<V> {
    origin: Point<Dbu>,
    side: Dbu,
    map: FxHashMap<Point<Bucket>, Vec<V>>,
}

impl<V> SpatialHash<V> {
    /// An empty hash of buckets of the positive side `side` from `origin`.
    pub(crate) fn new(origin: Point<Dbu>, side: Dbu) -> Self {
        Self {
            origin,
            side,
            map: FxHashMap::default(),
        }
    }

    /// The bucket holding `p`.
    pub(crate) fn bucket(&self, p: Point<Dbu>) -> Point<Bucket> {
        bucket_at(p - self.origin, self.side)
    }

    /// Files `value` in bucket `key`.
    pub(crate) fn insert(&mut self, key: Point<Bucket>, value: V) {
        self.map.entry(key).or_default().push(value);
    }

    /// The values filed in bucket `key`.
    pub(crate) fn in_bucket(&self, key: Point<Bucket>) -> &[V] {
        self.map.get(&key).map_or(&[], Vec::as_slice)
    }

    /// The values filed in every bucket from `min` to `max`, both
    /// included, column by column.
    pub(crate) fn between(
        &self,
        min: Point<Bucket>,
        max: Point<Bucket>,
    ) -> impl Iterator<Item = &V> {
        buckets_between(min, max).flat_map(|key| self.in_bucket(key))
    }

    /// Keeps only the values `keep` accepts, in every bucket.
    pub(crate) fn retain(&mut self, mut keep: impl FnMut(&V) -> bool) {
        for list in self.map.values_mut() {
            list.retain(&mut keep);
        }
    }

    /// The bytes its buckets' lists hold room for.
    pub(crate) fn list_bytes(&self) -> usize {
        self.map
            .values()
            .map(|list| list.capacity() * size_of::<V>())
            .sum()
    }

    /// Frees the room each bucket's list has grown beyond what it holds.
    pub(crate) fn shrink_to_fit(&mut self) {
        for list in self.map.values_mut() {
            list.shrink_to_fit();
        }
    }

    /// Every bucket with the values filed in it, in no particular order.
    #[cfg(test)]
    pub(crate) fn buckets(&self) -> impl Iterator<Item = (Point<Bucket>, &[V])> {
        self.map.iter().map(|(&key, list)| (key, list.as_slice()))
    }

    /// Keeps only the values in bucket `key` that `keep` accepts.
    pub(crate) fn retain_in(&mut self, key: Point<Bucket>, keep: impl FnMut(&V) -> bool) {
        if let Some(list) = self.map.get_mut(&key) {
            list.retain(keep);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn values_are_found_in_the_buckets_they_were_filed_in() {
        let mut hash = SpatialHash::new(Point::new(Dbu(100), Dbu(0)), Dbu(10));
        let key = hash.bucket(Point::new(Dbu(125), Dbu(5)));
        hash.insert(key, 'a');
        hash.insert(hash.bucket(Point::new(Dbu(145), Dbu(5))), 'b');

        assert_eq!(key, Point::new(Bucket(2), Bucket(0)));
        assert_eq!(hash.in_bucket(key), ['a']);
        assert_eq!(
            hash.between(key, Point::new(Bucket(4), Bucket(0))).count(),
            2
        );
        assert!(hash.in_bucket(Point::new(Bucket(3), Bucket(0))).is_empty());
    }

    #[test]
    fn retaining_keeps_only_the_accepted_values_of_one_bucket() {
        let mut hash = SpatialHash::new(Point::new(Dbu(0), Dbu(0)), Dbu(10));
        let key = Point::new(Bucket(0), Bucket(0));
        hash.insert(key, 1);
        hash.insert(key, 2);

        hash.retain_in(key, |&v| v == 2);

        assert_eq!(hash.in_bucket(key), [2]);
    }

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
