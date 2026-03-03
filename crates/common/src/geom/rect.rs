//! Axis-Aligned Rectangle Type.
//!
//! Provides a rectangle type defined by minimum and maximum corner points.
//! Used extensively for representing cell bounding boxes, die areas, and
//! spatial regions in placement and routing algorithms.

use super::point::Point;

/// An axis-aligned rectangle defined by minimum and maximum corner points.
///
/// Represents a rectangular region in 2D space. The min point is the
/// bottom-left corner and the max point is the top-right corner. This
/// structure is used for die areas, cell bounding boxes, and spatial
/// queries throughout the placement and routing algorithms.
#[derive(Clone, Copy, Debug, Default)]
pub struct Rect {
    /// Minimum corner (bottom-left) of the rectangle.
    pub min: Point<f64>,
    /// Maximum corner (top-right) of the rectangle.
    pub max: Point<f64>,
}

impl Rect {
    /// Creates a new rectangle from minimum and maximum corner points.
    ///
    /// The caller must ensure that min.x <= max.x and min.y <= max.y
    /// for the rectangle to be valid.
    pub fn new(min: Point<f64>, max: Point<f64>) -> Self {
        Self { min, max }
    }

    /// Returns the width of the rectangle (difference in X coordinates).
    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }
    /// Returns the height of the rectangle (difference in Y coordinates).
    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }
    /// Returns the area of the rectangle (width * height).
    pub fn area(&self) -> f64 {
        self.width() * self.height()
    }

    /// Checks whether this rectangle overlaps with another rectangle.
    ///
    /// Two rectangles overlap if they share any interior points. This is
    /// used for collision detection during placement to ensure cells do
    /// not overlap after legalization.
    pub fn overlaps(&self, other: &Rect) -> bool {
        self.min.x < other.max.x
            && self.max.x > other.min.x
            && self.min.y < other.max.y
            && self.max.y > other.min.y
    }

    /// Checks whether a point lies inside or on the boundary of this rectangle.
    ///
    /// Returns true if the point's coordinates are within the rectangle's
    /// bounds (inclusive of boundaries). Used for containment tests and
    /// spatial queries.
    pub fn contains(&self, p: Point<f64>) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }
}
