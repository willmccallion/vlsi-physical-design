//! Axis-aligned rectangle type.

use super::dbu::Dbu;
use super::interval::Interval;
use super::point::Point;
use std::ops::{Mul, Sub};

/// An axis-aligned rectangle given by its lower-left and upper-right corners.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Rect<T> {
    /// Lower-left corner.
    pub min: Point<T>,
    /// Upper-right corner.
    pub max: Point<T>,
}

impl<T> Rect<T> {
    /// Creates a rectangle from corners the caller has already ordered.
    pub const fn new(min: Point<T>, max: Point<T>) -> Self {
        Self { min, max }
    }
}

impl<T: Copy + PartialOrd + Sub<Output = T>> Rect<T> {
    /// Width along X.
    pub fn width(&self) -> T {
        self.max.x - self.min.x
    }

    /// Height along Y.
    pub fn height(&self) -> T {
        self.max.y - self.min.y
    }

    /// Width times height.
    pub fn area(&self) -> <T as Mul>::Output
    where
        T: Mul,
    {
        self.width() * self.height()
    }

    /// Whether the interiors of the two rectangles intersect.
    pub fn overlaps(&self, other: &Self) -> bool {
        self.min.x < other.max.x
            && self.max.x > other.min.x
            && self.min.y < other.max.y
            && self.max.y > other.min.y
    }

    /// Whether the closed rectangles share at least one point.
    pub fn touches(&self, other: &Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    /// Whether the point lies inside or on the boundary.
    pub fn contains(&self, p: Point<T>) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }

    /// Whether `other` lies entirely inside this rectangle.
    pub fn encloses(&self, other: &Self) -> bool {
        self.contains(other.min) && self.contains(other.max)
    }
}

impl Rect<Dbu> {
    /// Its stretch along X.
    #[must_use]
    pub const fn x_span(&self) -> Interval {
        Interval::new(self.min.x, self.max.x)
    }

    /// Its stretch along Y.
    #[must_use]
    pub const fn y_span(&self) -> Interval {
        Interval::new(self.min.y, self.max.y)
    }

    /// Creates a rectangle from two opposite corners in any order.
    pub fn from_corners(a: Point<Dbu>, b: Point<Dbu>) -> Self {
        Self::new(
            Point::new(a.x.min(b.x), a.y.min(b.y)),
            Point::new(a.x.max(b.x), a.y.max(b.y)),
        )
    }

    /// Moves the rectangle by `offset`.
    #[must_use]
    pub fn translate(&self, offset: Point<Dbu>) -> Self {
        Self::new(self.min + offset, self.max + offset)
    }

    /// Grows every side by `amount`.
    #[must_use]
    pub fn expand(&self, amount: Dbu) -> Self {
        Self::new(
            Point::new(self.min.x - amount, self.min.y - amount),
            Point::new(self.max.x + amount, self.max.y + amount),
        )
    }

    /// The smallest rectangle containing both.
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        Self::new(
            Point::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            Point::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        )
    }

    /// The overlap of the closed rectangles, if they touch.
    pub fn intersection(&self, other: &Self) -> Option<Self> {
        self.touches(other).then(|| {
            Self::new(
                Point::new(self.min.x.max(other.min.x), self.min.y.max(other.min.y)),
                Point::new(self.max.x.min(other.max.x), self.max.y.min(other.max.y)),
            )
        })
    }

    /// The center, rounded toward the lower-left.
    pub const fn center(&self) -> Point<Dbu> {
        Point::new(
            self.min.x.midpoint(self.max.x),
            self.min.y.midpoint(self.max.y),
        )
    }

    /// Whether the union of `cover`, whose rectangles may overlap, contains
    /// this rectangle, all taken as closed point sets: a rectangle without
    /// area is covered when every point of it, ends included, lies in some
    /// cover rectangle, edges included.
    pub fn covered_by(&self, cover: &[Self]) -> bool {
        let (x, y) = (self.min.x, self.min.y);
        if self.width() == Dbu::ZERO {
            let crossing = cover.iter().filter(|c| c.min.x <= x && x <= c.max.x);
            return union_covers(crossing.map(Self::y_span), self.y_span());
        }
        if self.height() == Dbu::ZERO {
            let crossing = cover.iter().filter(|c| c.min.y <= y && y <= c.max.y);
            return union_covers(crossing.map(Self::x_span), self.x_span());
        }
        let pieces: Vec<Self> = cover
            .iter()
            .filter(|c| c.overlaps(self))
            .filter_map(|c| c.intersection(self))
            .collect();
        let mut xs: Vec<Dbu> = pieces
            .iter()
            .flat_map(|p| [p.min.x, p.max.x])
            .chain([self.min.x, self.max.x])
            .collect();
        xs.sort_unstable();
        xs.dedup();
        xs.windows(2)
            .all(|slab| self.slab_covered(&pieces, slab[0], slab[1]))
    }

    /// Whether `pieces` spanning the whole slab `[x0, x1]` cover this
    /// rectangle's height there.
    fn slab_covered(&self, pieces: &[Self], x0: Dbu, x1: Dbu) -> bool {
        let spanning = pieces
            .iter()
            .filter(|p| p.min.x <= x0 && x1 <= p.max.x)
            .map(Self::y_span);
        union_covers(spanning, self.y_span())
    }
}

/// Whether the closed intervals `spans` together hold all of the closed
/// interval `whole`.
fn union_covers(spans: impl Iterator<Item = Interval>, whole: Interval) -> bool {
    let mut spans: Vec<Interval> = spans.filter(|span| span.hi >= whole.lo).collect();
    spans.sort_unstable();
    let mut reached: Option<Dbu> = None;
    for span in spans {
        if span.lo > reached.unwrap_or(whole.lo) {
            break;
        }
        reached = Some(reached.map_or(span.hi, |r| r.max(span.hi)));
    }
    reached.is_some_and(|r| r >= whole.hi)
}

#[cfg(test)]
mod tests;
