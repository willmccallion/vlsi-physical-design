//! Routing Grid Data Structures.
//!
//! Defines the interface and implementation for routing grids that track
//! edge-based wire usage and congestion. The grid uses gcells (global routing
//! cells) where capacity and usage are tracked on edges between adjacent
//! gcells, not on the gcells themselves. This models real physical routing
//! tracks: horizontal edges carry horizontal wires, vertical edges carry
//! vertical wires.

pub mod dense;

pub use dense::GCellGrid;


/// Trait defining the interface for edge-based routing grid implementations.
///
/// Routing capacity and usage are tracked on **edges** between adjacent gcells,
/// not on the gcells themselves. A horizontal edge between (x,y) and (x+1,y)
/// on layer L has capacity equal to the number of horizontal tracks crossing
/// that boundary. Direction is structurally enforced: horizontal layers only
/// have horizontal edge capacity, vertical layers only have vertical edge
/// capacity.
pub trait RoutingGrid: Sync + Send {
    /// Returns the width of the routing grid in gcells.
    fn width(&self) -> u32;
    /// Returns the height of the routing grid in gcells.
    fn height(&self) -> u32;
    /// Returns the number of routing layers in the grid.
    fn layers(&self) -> u8;

    /// Returns the cost of using the horizontal edge from (x,y) to (x+1,y) on a layer.
    fn h_edge_cost(&self, x: u32, y: u32, layer: u8, collision_penalty: f64) -> f64;
    /// Returns the cost of using the vertical edge from (x,y) to (x,y+1) on a layer.
    fn v_edge_cost(&self, x: u32, y: u32, layer: u8, collision_penalty: f64) -> f64;
    /// Returns the cost of a via at gcell (x,y) between layers.
    fn via_cost(&self, x: u32, y: u32, layer: u8) -> f64;

    /// Increments horizontal edge usage from (x,y) to (x+1,y) on a layer.
    fn add_h_wire(&mut self, x: u32, y: u32, layer: u8);
    /// Decrements horizontal edge usage from (x,y) to (x+1,y) on a layer.
    fn remove_h_wire(&mut self, x: u32, y: u32, layer: u8);
    /// Increments vertical edge usage from (x,y) to (x,y+1) on a layer.
    fn add_v_wire(&mut self, x: u32, y: u32, layer: u8);
    /// Decrements vertical edge usage from (x,y) to (x,y+1) on a layer.
    fn remove_v_wire(&mut self, x: u32, y: u32, layer: u8);

    /// Checks if horizontal edge from (x,y) to (x+1,y) exceeds capacity.
    fn is_h_congested(&self, x: u32, y: u32, layer: u8) -> bool;
    /// Checks if vertical edge from (x,y) to (x,y+1) exceeds capacity.
    fn is_v_congested(&self, x: u32, y: u32, layer: u8) -> bool;

    /// Returns the total number of edges exceeding capacity across all layers.
    fn total_overflow(&self) -> usize;

    /// Updates history costs based on current congestion.
    fn update_history(&mut self, history_increment: f64);
    /// Decays all history values by the specified factor.
    fn decay_history(&mut self, decay_factor: f64);

    /// Sets the collision penalty multiplier.
    fn set_penalty(&mut self, penalty: f64);

    /// Returns the horizontal edge capacity for (x,y) to (x+1,y) on a layer.
    fn h_edge_cap(&self, x: u32, y: u32, layer: u8) -> u16;
    /// Returns the vertical edge capacity for (x,y) to (x,y+1) on a layer.
    fn v_edge_cap(&self, x: u32, y: u32, layer: u8) -> u16;
    /// Returns the horizontal edge usage for (x,y) to (x+1,y) on a layer.
    fn h_edge_usage(&self, x: u32, y: u32, layer: u8) -> u16;
    /// Returns the vertical edge usage for (x,y) to (x,y+1) on a layer.
    fn v_edge_usage(&self, x: u32, y: u32, layer: u8) -> u16;
}
