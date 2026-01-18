//! Dense Grid Implementation for Routing.
//!
//! Implements a dense 3D grid that tracks occupancy, history costs, and
//! obstacles for each grid cell. Uses packed data structures for cache
//! efficiency and provides congestion visualization support.

use super::RoutingGrid;
use eda_common::geom::coord::GridCoord;
use eda_common::util::visualization::CongestionProvider;

/// Packed grid node structure for efficient memory usage.
///
/// Stores occupancy (number of wires), quantized history cost, and cached
/// routing cost in a compact 8-byte structure. The history cost accumulates
/// over iterations to discourage repeated congestion in the same locations.
#[derive(Clone, Copy)]
struct GridNode {
    occupancy: u16,
    history: u16,
    cost_cache: f32,
}

impl Default for GridNode {
    /// Creates a default grid node with zero occupancy and history.
    ///
    /// Initializes the cost cache to 1.0 (base routing cost) since an empty
    /// cell has no congestion or history penalties.
    fn default() -> Self {
        Self {
            occupancy: 0,
            history: 0,
            cost_cache: 1.0,
        }
    }
}

/// Dense 3D routing grid implementation.
///
/// Maintains a flat array of grid nodes indexed by (layer, y, x) coordinates.
/// Tracks wire occupancy, obstacles, and history costs for congestion-aware
/// routing. Supports per-layer capacity settings and provides efficient
/// lookup and update operations.
pub struct DenseGrid {
    width: u32,
    height: u32,
    layers: u8,
    nodes: Vec<GridNode>,
    obstacles: Vec<bool>,
    current_penalty: f32,
    capacities: Vec<u32>,
}

impl DenseGrid {
    /// Creates a new dense grid with the specified dimensions.
    ///
    /// Allocates arrays for grid nodes and obstacles sized to width * height * layers.
    /// Initializes all nodes with zero occupancy and default capacity. Warns if
    /// the allocation size exceeds 2GB to alert users of potential memory issues.
    pub fn new(width: u32, height: u32, layers: u8, default_capacity: u32) -> Self {
        let size = (width as usize) * (height as usize) * (layers as usize);

        if size > 2_000_000_000 {
            log::warn!(
                "Allocating large DenseGrid: {} elements. Ensure sufficient RAM.",
                size
            );
        }

        Self {
            width,
            height,
            layers,
            nodes: vec![GridNode::default(); size],
            obstacles: vec![false; size],
            current_penalty: 1.0,
            capacities: vec![default_capacity; layers as usize],
        }
    }

    /// Sets the routing capacity for a specific layer.
    ///
    /// Overrides the default capacity for the given layer. This is used to
    /// set infinite capacity for layer 0 (M1) in global routing to allow
    /// pin access without congestion penalties.
    pub fn set_layer_capacity(&mut self, layer: u8, capacity: u32) {
        if (layer as usize) < self.capacities.len() {
            self.capacities[layer as usize] = capacity;
        }
    }

    /// Returns the history cost value for a grid node at the given index.
    ///
    /// Used for debugging and analysis of congestion patterns. The history
    /// value accumulates when cells exceed capacity and decays over time.
    pub fn get_history_at(&self, idx: usize) -> u16 {
        self.nodes[idx].history
    }

    /// Computes the linear index for a grid coordinate in the flat node array.
    ///
    /// The indexing order is (layer, y, x), allowing efficient cache-friendly
    /// access patterns when iterating over layers or rows.
    #[inline(always)]
    fn index(&self, coord: GridCoord) -> usize {
        (coord.z as usize) * (self.width as usize) * (self.height as usize)
            + (coord.y as usize) * (self.width as usize)
            + (coord.x as usize)
    }

    /// Extracts the layer index from a linear array index.
    ///
    /// Divides the linear index by the plane size (width * height) to obtain
    /// the layer number. Used when iterating over all nodes to determine
    /// which layer's capacity to check.
    #[inline(always)]
    fn get_layer_from_index(&self, idx: usize) -> usize {
        idx / ((self.width as usize) * (self.height as usize))
    }

    /// Updates the cached routing cost for a grid node.
    ///
    /// Recomputes the cost based on current occupancy, history, and penalty
    /// settings. The cached value is used by get_cost to avoid repeated
    /// calculations during pathfinding.
    #[inline(always)]
    fn update_cache_at(&mut self, idx: usize, layer: usize) {
        let cap = unsafe { *self.capacities.get_unchecked(layer) } as f32;
        let node = unsafe { self.nodes.get_unchecked_mut(idx) };

        let base_cost = 1.0 + (node.history as f32 * 0.2);

        let occ = node.occupancy as f32;
        let congestion_cost = if occ >= cap {
            (occ - cap + 1.0) * self.current_penalty
        } else {
            0.0
        };

        node.cost_cache = base_cost + congestion_cost;
    }
}

impl CongestionProvider for DenseGrid {
    /// Returns the grid dimensions (width, height) for visualization.
    ///
    /// Provides the 2D dimensions of the routing grid, which are used
    /// to size congestion heatmap images.
    fn get_dims(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    /// Computes the congestion ratio for a grid coordinate across all layers.
    ///
    /// Aggregates occupancy and capacity across all routing layers at the
    /// given X,Y position. Layers with infinite capacity (>= 1000) are excluded
    /// from the calculation. Returns the ratio of total occupancy to total
    /// capacity, or 2.0 if there's occupancy but no finite capacity.
    fn get_congestion_ratio(&self, x: u32, y: u32) -> f32 {
        let mut total_occupancy = 0.0;
        let mut total_capacity = 0.0;

        for z in 0..self.layers {
            let idx = (z as u32 * self.width * self.height + y * self.width + x) as usize;
            let cap = self.capacities[z as usize] as f32;

            if cap < 1000.0 {
                total_occupancy += self.nodes[idx].occupancy as f32;
                total_capacity += cap;
            }
        }

        if total_capacity > 0.0 {
            total_occupancy / total_capacity
        } else if total_occupancy > 0.0 {
            2.0
        } else {
            0.0
        }
    }
}

impl RoutingGrid for DenseGrid {
    /// Returns the width of the grid.
    fn width(&self) -> u32 {
        self.width
    }
    /// Returns the height of the grid.
    fn height(&self) -> u32 {
        self.height
    }
    /// Returns the number of layers in the grid.
    fn layers(&self) -> u8 {
        self.layers
    }

    /// Marks a specific grid coordinate as an obstacle.
    fn set_obstacle(&mut self, coord: GridCoord) {
        let idx = self.index(coord);
        self.obstacles[idx] = true;
    }

    /// Clears the obstacle flag for a specific grid coordinate.
    fn clear_obstacle(&mut self, coord: GridCoord) {
        let idx = self.index(coord);
        self.obstacles[idx] = false;
    }

    /// Checks if a coordinate is occupied by an obstacle or out of bounds.
    fn is_obstacle(&self, coord: GridCoord) -> bool {
        if coord.x >= self.width || coord.y >= self.height || coord.z >= self.layers {
            return true;
        }
        self.obstacles[self.index(coord)]
    }

    /// Adds a wire usage to the grid cell, increasing occupancy.
    fn add_wire(&mut self, coord: GridCoord) {
        let idx = self.index(coord);
        unsafe {
            self.nodes.get_unchecked_mut(idx).occupancy += 1;
        }
        self.update_cache_at(idx, coord.z as usize);
    }

    /// Removes a wire usage from the grid cell, decreasing occupancy.
    fn remove_wire(&mut self, coord: GridCoord) {
        let idx = self.index(coord);
        unsafe {
            let node = self.nodes.get_unchecked_mut(idx);
            if node.occupancy > 0 {
                node.occupancy -= 1;
            }
        }
        self.update_cache_at(idx, coord.z as usize);
    }

    /// Calculates the routing cost for a grid cell.
    #[inline(always)]
    fn get_cost(&self, coord: GridCoord, _collision_penalty: f64) -> f64 {
        let idx = self.index(coord);
        let cost = unsafe { self.nodes.get_unchecked(idx).cost_cache };

        if coord.z == 0 {
            return (cost as f64) * 100.0;
        }

        cost as f64
    }

    /// Updates history costs based on current congestion.
    fn update_history(&mut self, history_increment: f64) {
        for i in 0..self.nodes.len() {
            let layer = self.get_layer_from_index(i);
            let cap = self.capacities[layer];
            let node = &mut self.nodes[i];

            if node.occupancy as u32 > cap {
                let overflow = (node.occupancy as u32 - cap) as f32;
                let inc = (overflow * history_increment as f32 * 10.0) as u16;
                node.history = node.history.saturating_add(inc.max(1));
            }
        }
    }

    /// Decays history costs to allow recovery from past congestion.
    fn decay_history(&mut self, decay_factor: f64) {
        let factor = (decay_factor * 1024.0) as u32;
        for node in &mut self.nodes {
            if node.history > 0 {
                let h = (node.history as u32 * factor) >> 10;
                node.history = h as u16;
            }
        }
        self.set_penalty(self.current_penalty as f64);
    }

    /// Returns the maximum occupancy observed in any grid cell.
    fn max_occupancy(&self) -> u32 {
        self.nodes
            .iter()
            .map(|n| n.occupancy as u32)
            .max()
            .unwrap_or(0)
    }

    /// Checks if a specific grid cell is congested (occupancy > capacity).
    fn is_congested(&self, coord: GridCoord) -> bool {
        let idx = self.index(coord);
        let layer = coord.z as usize;
        self.nodes[idx].occupancy as u32 > self.capacities[layer]
    }

    /// Counts the total number of congested nodes in the grid.
    fn total_conflicts(&self) -> usize {
        let mut conflicts = 0;
        for i in 0..self.nodes.len() {
            let layer = self.get_layer_from_index(i);
            if self.nodes[i].occupancy as u32 > self.capacities[layer] {
                conflicts += 1;
            }
        }
        conflicts
    }

    /// Returns the capacity of a specific grid cell.
    fn capacity(&self, coord: GridCoord) -> u32 {
        self.capacities[coord.z as usize]
    }

    /// Sets the current congestion penalty factor.
    fn set_penalty(&mut self, penalty: f64) {
        self.current_penalty = penalty as f32;
        for i in 0..self.nodes.len() {
            let layer = self.get_layer_from_index(i);
            self.update_cache_at(i, layer);
        }
    }
}
