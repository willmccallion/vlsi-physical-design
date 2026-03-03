//! Coordinate Conversion Between Physical and Grid Space.
//!
//! Provides conversion between physical coordinates (floating-point microns)
//! and discrete grid coordinates (integer indices). Handles coordinate
//! system transformations including origin offsets and scaling factors.

use pare_common::geom::coord::GridCoord;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// Converter between physical coordinates and grid coordinates.
///
/// Maintains scaling factors and origin offsets to transform between
/// the continuous physical coordinate space and the discrete grid space
/// used by routing algorithms. The converter accounts for die area
/// boundaries and grid step sizes.
#[derive(Clone, Debug)]
pub struct GridConverter {
    scale_x: f64,
    scale_y: f64,
    origin_x: f64,
    origin_y: f64,
    grid_w: u32,
    grid_h: u32,
}

impl GridConverter {
    /// Creates a converter from die area and grid dimensions.
    ///
    /// Computes scaling factors based on the ratio of grid size to die area
    /// size. Sets the origin to the die area's minimum corner so that
    /// physical coordinates are properly offset before scaling.
    pub fn new(die_area: Rect, grid_w: u32, grid_h: u32) -> Self {
        Self {
            scale_x: grid_w as f64 / die_area.width(),
            scale_y: grid_h as f64 / die_area.height(),
            origin_x: die_area.min.x,
            origin_y: die_area.min.y,
            grid_w,
            grid_h,
        }
    }

    /// Creates a converter from explicit step sizes and origin.
    ///
    /// Uses the inverse of step sizes as scaling factors, allowing direct
    /// specification of grid resolution. The origin parameters specify
    /// the physical coordinate corresponding to grid (0, 0).
    pub fn from_steps(
        step_x: f64,
        step_y: f64,
        origin_x: f64,
        origin_y: f64,
        grid_w: u32,
        grid_h: u32,
    ) -> Self {
        Self {
            scale_x: 1.0 / step_x,
            scale_y: 1.0 / step_y,
            origin_x,
            origin_y,
            grid_w,
            grid_h,
        }
    }

    /// Converts a physical coordinate to a grid coordinate.
    ///
    /// Subtracts the origin offset, scales by the grid resolution, and
    /// clamps to valid grid bounds. The layer is preserved from the input.
    /// Returns a grid coordinate that can be used to index into routing
    /// grid data structures.
    pub fn to_grid(&self, p: Point<f64>, layer: u8) -> GridCoord {
        let raw_x = (p.x - self.origin_x) * self.scale_x;
        let raw_y = (p.y - self.origin_y) * self.scale_y;

        let x = raw_x.floor().max(0.0).min((self.grid_w - 1) as f64) as u32;
        let y = raw_y.floor().max(0.0).min((self.grid_h - 1) as f64) as u32;

        GridCoord::new(x, y, layer)
    }

    /// Converts a grid coordinate to a physical coordinate.
    ///
    /// Scales the grid coordinates by the inverse scaling factors and
    /// adds the origin offset to obtain the physical position. This is
    /// used to convert routed paths back to physical coordinates for
    /// DEF file output.
    pub fn to_world(&self, g: GridCoord) -> Point<f64> {
        Point::new(
            (g.x as f64 / self.scale_x) + self.origin_x,
            (g.y as f64 / self.scale_y) + self.origin_y,
        )
    }
}
