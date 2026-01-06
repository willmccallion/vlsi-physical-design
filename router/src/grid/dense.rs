use super::RoutingGrid;
use eda_common::geom::coord::GridCoord;

// Packed struct for better cache locality (8 bytes)
#[derive(Clone, Copy)]
struct GridNode {
    occupancy: u16,  // Max 65535 wires per GCell
    history: u16,    // Quantized history cost
    cost_cache: f32, // Pre-calculated cost for A*
}

impl Default for GridNode {
    fn default() -> Self {
        Self {
            occupancy: 0,
            history: 0,
            cost_cache: 1.0,
        }
    }
}

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

    pub fn set_layer_capacity(&mut self, layer: u8, capacity: u32) {
        if (layer as usize) < self.capacities.len() {
            self.capacities[layer as usize] = capacity;
        }
    }

    #[inline(always)]
    fn index(&self, coord: GridCoord) -> usize {
        (coord.z as usize) * (self.width as usize) * (self.height as usize)
            + (coord.y as usize) * (self.width as usize)
            + (coord.x as usize)
    }

    #[inline(always)]
    fn get_layer_from_index(&self, idx: usize) -> usize {
        idx / ((self.width as usize) * (self.height as usize))
    }

    #[inline(always)]
    fn update_cache_at(&mut self, idx: usize, layer: usize) {
        let cap = unsafe { *self.capacities.get_unchecked(layer) } as f32;
        let node = unsafe { self.nodes.get_unchecked_mut(idx) };

        let base_cost = 1.0 + (node.history as f32 * 0.1);

        let occ = node.occupancy as f32;
        let congestion_cost = if occ >= cap {
            (occ - cap + 1.0) * self.current_penalty
        } else {
            0.0
        };

        node.cost_cache = base_cost + congestion_cost;
    }
}

impl RoutingGrid for DenseGrid {
    fn width(&self) -> u32 {
        self.width
    }
    fn height(&self) -> u32 {
        self.height
    }
    fn layers(&self) -> u8 {
        self.layers
    }

    fn set_obstacle(&mut self, coord: GridCoord) {
        let idx = self.index(coord);
        self.obstacles[idx] = true;
    }

    fn clear_obstacle(&mut self, coord: GridCoord) {
        let idx = self.index(coord);
        self.obstacles[idx] = false;
    }

    fn is_obstacle(&self, coord: GridCoord) -> bool {
        if coord.x >= self.width || coord.y >= self.height || coord.z >= self.layers {
            return true;
        }
        self.obstacles[self.index(coord)]
    }

    fn add_wire(&mut self, coord: GridCoord) {
        let idx = self.index(coord);
        unsafe {
            self.nodes.get_unchecked_mut(idx).occupancy += 1;
        }
        // Pass layer explicitly to avoid division
        self.update_cache_at(idx, coord.z as usize);
    }

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

    #[inline(always)]
    fn get_cost(&self, coord: GridCoord, _collision_penalty: f64) -> f64 {
        let idx = self.index(coord);
        let cost = unsafe { self.nodes.get_unchecked(idx).cost_cache };

        if coord.z == 0 {
            return (cost as f64) * 100.0;
        }

        cost as f64
    }

    fn update_history(&mut self, history_increment: f64) {
        for i in 0..self.nodes.len() {
            // We still need division here, but this is only called once per iteration
            let layer = self.get_layer_from_index(i);
            let cap = self.capacities[layer];
            let node = &mut self.nodes[i];

            if node.occupancy as u32 > cap {
                let overflow = (node.occupancy as u32 - cap) as f32;
                let inc = (overflow * history_increment as f32 * 10.0) as u16;
                node.history = node.history.saturating_add(inc);
            }
        }
    }

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

    fn max_occupancy(&self) -> u32 {
        self.nodes
            .iter()
            .map(|n| n.occupancy as u32)
            .max()
            .unwrap_or(0)
    }

    fn is_congested(&self, coord: GridCoord) -> bool {
        let idx = self.index(coord);
        let layer = coord.z as usize;
        self.nodes[idx].occupancy as u32 > self.capacities[layer]
    }

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

    fn capacity(&self, coord: GridCoord) -> u32 {
        self.capacities[coord.z as usize]
    }

    fn set_penalty(&mut self, penalty: f64) {
        self.current_penalty = penalty as f32;
        // This loop is unavoidable, but it's linear scan
        for i in 0..self.nodes.len() {
            let layer = self.get_layer_from_index(i);
            self.update_cache_at(i, layer);
        }
    }
}
