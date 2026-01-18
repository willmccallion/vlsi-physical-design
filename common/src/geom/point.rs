//! Two-Dimensional Point Type.
//!
//! Provides a generic two-dimensional point type with arithmetic operations
//! for vector math. Used throughout the codebase to represent physical
//! coordinates, offsets, and spatial relationships.

use std::ops::{Add, AddAssign, Div, Mul, Sub};

/// A two-dimensional point with generic coordinate type.
///
/// Represents a point in 2D space with X and Y coordinates. The type parameter
/// T allows the point to work with different numeric types (f64 for physical
/// coordinates, i32 for integer coordinates, etc.). The C representation
/// ensures compatibility with external libraries that expect struct layout.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Point<T> {
    /// X coordinate of the point.
    pub x: T,
    /// Y coordinate of the point.
    pub y: T,
}

impl<T> Point<T> {
    /// Creates a new point with the specified X and Y coordinates.
    pub fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
}

impl Add for Point<f64> {
    /// The resulting point type after addition.
    type Output = Self;
    /// Adds two points component-wise to produce a new point.
    ///
    /// Performs vector addition: (x1, y1) + (x2, y2) = (x1+x2, y1+y2).
    /// Used for translating points by offsets and combining vectors.
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl AddAssign for Point<f64> {
    /// Adds another point to this point in-place.
    ///
    /// Modifies this point by adding the components of the other point.
    /// More efficient than Add when the result can overwrite the original.
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Point<f64> {
    /// The resulting point type after subtraction.
    type Output = Self;
    /// Subtracts two points component-wise to produce a new point.
    ///
    /// Performs vector subtraction: (x1, y1) - (x2, y2) = (x1-x2, y1-y2).
    /// Used for computing offsets and displacement vectors.
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Mul<f64> for Point<f64> {
    /// The resulting point type after multiplication.
    type Output = Self;
    /// Multiplies a point by a scalar to scale the point.
    ///
    /// Performs scalar multiplication: (x, y) * s = (x*s, y*s).
    /// Used for scaling vectors and applying scaling factors to coordinates.
    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Div<f64> for Point<f64> {
    /// The resulting point type after division.
    type Output = Self;
    /// Divides a point by a scalar to scale the point.
    ///
    /// Performs scalar division: (x, y) / s = (x/s, y/s).
    /// Used for normalizing vectors and applying inverse scaling factors.
    fn div(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}
