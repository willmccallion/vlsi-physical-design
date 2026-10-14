//! Axis-aligned rectangle type.

use super::dbu::Dbu;
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

impl<T: Copy + PartialOrd + Sub<Output = T> + Mul<Output = T>> Rect<T> {
    /// Width along X.
    pub fn width(&self) -> T {
        self.max.x - self.min.x
    }

    /// Height along Y.
    pub fn height(&self) -> T {
        self.max.y - self.min.y
    }

    /// Width times height.
    pub fn area(&self) -> T {
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
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::from_corners(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    #[test]
    fn abutting_rectangles_touch_but_do_not_overlap() {
        let a = rect(0, 0, 10, 10);
        let b = rect(10, 0, 20, 10);

        assert!(a.touches(&b));
        assert!(!a.overlaps(&b));
    }

    #[test]
    fn corners_are_normalized() {
        assert_eq!(rect(10, 20, 0, 5), rect(0, 5, 10, 20));
    }

    #[test]
    fn intersection_of_disjoint_rectangles_is_none() {
        assert_eq!(rect(0, 0, 1, 1).intersection(&rect(2, 2, 3, 3)), None);
    }
}
