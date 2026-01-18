//! Routing Grid Data Structures.
//!
//! Defines the interface and implementation for routing grids that track
//! wire occupancy, obstacles, and congestion. The grid is used by routing
//! algorithms to check feasibility and compute routing costs.

pub mod dense;

pub use dense::DenseGrid;

use eda_common::geom::coord::GridCoord;

/// Trait defining the interface for routing grid implementations.
///
/// Provides methods for querying and modifying grid state including
/// obstacles, wire occupancy, congestion, and routing costs. Implementations
/// must be thread-safe to support parallel routing.
pub trait RoutingGrid: Sync + Send {
    /// Returns the width of the routing grid in grid cells.
    fn width(&self) -> u32;
    /// Returns the height of the routing grid in grid cells.
    fn height(&self) -> u32;
    /// Returns the number of routing layers in the grid.
    fn layers(&self) -> u8;

    /// Marks a grid coordinate as an obstacle that cannot be routed through.
    fn set_obstacle(&mut self, coord: GridCoord);
    /// Removes the obstacle status from a grid coordinate.
    fn clear_obstacle(&mut self, coord: GridCoord);
    /// Checks whether a grid coordinate is marked as an obstacle.
    fn is_obstacle(&self, coord: GridCoord) -> bool;

    /// Increments the wire occupancy count at the specified coordinate.
    fn add_wire(&mut self, coord: GridCoord);
    /// Decrements the wire occupancy count at the specified coordinate.
    fn remove_wire(&mut self, coord: GridCoord);

    /// Computes the routing cost for a grid coordinate given the collision penalty.
    ///
    /// The cost increases with congestion and history violations. Used by A*
    /// pathfinding to prefer less congested routes.
    fn get_cost(&self, coord: GridCoord, collision_penalty: f64) -> f64;
    /// Updates the congestion history for all overflowing grid cells.
    ///
    /// Increments history values for cells that exceed capacity, discouraging
    /// future routing through persistently congested regions.
    fn update_history(&mut self, history_increment: f64);
    /// Decays all history values by the specified factor.
    ///
    /// Reduces history costs over time to allow previously congested regions
    /// to become available again as the routing solution evolves.
    fn decay_history(&mut self, decay_factor: f64);

    /// Returns the maximum wire occupancy across all grid cells.
    fn max_occupancy(&self) -> u32;
    /// Checks whether a grid coordinate exceeds its routing capacity.
    fn is_congested(&self, coord: GridCoord) -> bool;
    /// Returns the total number of grid cells that exceed their capacity.
    fn total_conflicts(&self) -> usize;

    /// Returns the routing capacity for a grid coordinate's layer.
    fn capacity(&self, coord: GridCoord) -> u32;
    /// Sets the collision penalty multiplier for cost computation.
    fn set_penalty(&mut self, penalty: f64);
}
