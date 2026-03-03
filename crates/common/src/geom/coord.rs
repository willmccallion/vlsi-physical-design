//! Grid Coordinate System for Routing.
//!
//! Defines the discrete coordinate system used by routing algorithms to
//! represent positions on the routing grid. Grid coordinates are integer-based
//! and correspond to specific routing tracks and layers.

/// A discrete coordinate on the routing grid.
///
/// Represents a position in the three-dimensional routing space: X and Y
/// specify the grid cell location, and Z specifies the routing layer.
/// Grid coordinates are used by routing algorithms to index into the
/// routing grid data structures and represent wire paths.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GridCoord {
    /// X coordinate on the routing grid.
    pub x: u32,
    /// Y coordinate on the routing grid.
    pub y: u32,
    /// Layer index (Z coordinate) in the metal stack.
    pub z: u8,
}

impl GridCoord {
    /// Creates a new grid coordinate with the specified X, Y, and layer.
    pub fn new(x: u32, y: u32, z: u8) -> Self {
        Self { x, y, z }
    }
}
