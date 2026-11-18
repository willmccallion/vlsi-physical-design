//! Two-dimensional point type.

use crate::geom::dbu::Dbu;
use std::ops::{Add, AddAssign, Div, Mul, Sub};

/// A two-dimensional point, generic over its coordinate type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Point<T> {
    /// X coordinate.
    pub x: T,
    /// Y coordinate.
    pub y: T,
}

impl<T> Point<T> {
    /// Creates a point.
    pub const fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
}

impl Point<Dbu> {
    /// The Manhattan distance to `other`.
    #[must_use]
    pub const fn manhattan(self, other: Self) -> Dbu {
        Dbu::new((self.x.get() - other.x.get()).abs() + (self.y.get() - other.y.get()).abs())
    }
}

impl<T: Add<Output = T>> Add for Point<T> {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl<T: AddAssign> AddAssign for Point<T> {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl<T: Sub<Output = T>> Sub for Point<T> {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f64> for Point<f64> {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl Div<f64> for Point<f64> {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self::new(self.x / rhs, self.y / rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_manhattan_distance_adds_both_sides_whichever_way_they_run() {
        let a = Point::new(Dbu::new(10), Dbu::new(-4));
        let b = Point::new(Dbu::new(-2), Dbu::new(5));

        assert_eq!(a.manhattan(b), Dbu::new(21));
        assert_eq!(b.manhattan(a), Dbu::new(21));
        assert_eq!(a.manhattan(a), Dbu::new(0));
    }

    #[test]
    fn points_order_by_x_then_y() {
        let mut points = vec![
            Point::new(Dbu::new(2), Dbu::new(0)),
            Point::new(Dbu::new(1), Dbu::new(9)),
            Point::new(Dbu::new(1), Dbu::new(3)),
        ];

        points.sort();

        assert_eq!(
            points,
            vec![
                Point::new(Dbu::new(1), Dbu::new(3)),
                Point::new(Dbu::new(1), Dbu::new(9)),
                Point::new(Dbu::new(2), Dbu::new(0)),
            ]
        );
    }
}
