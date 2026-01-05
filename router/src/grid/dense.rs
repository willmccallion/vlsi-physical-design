use super::RoutingGrid;
use eda_common::geom::coord::GridCoord;

pub struct DenseGrid {
    width: u32,
    height: u32,
    layers: u8,
    occupancy: Vec<u16>,
    history: Vec<f64>,
    obstacles: Vec<bool>,
    cost_cache: Vec<f64>,
    current_penalty: f64,
    default_capacity: u32,
}

impl DenseGrid {
    pub fn new(width: u32, height: u32, layers: u8, default_capacity: u32) -> Self {
        let size = (width * height * (layers as u32)) as usize;
        Self {
            width,
            height,
            layers,
            occupancy: vec![0; size],
            history: vec![1.0; size],
            obstacles: vec![false; size],
            cost_cache: vec![1.0; size],
            current_penalty: 1.0,
            default_capacity,
        }
    }

    #[inline(always)]
    fn index(&self, coord: GridCoord) -> usize {
        (coord.z as u32 * self.width * self.height + coord.y * self.width + coord.x) as usize
    }

    #[inline(always)]
    fn update_cache_at(&mut self, idx: usize) {
        let base_cost = 1.0;
        let hist_cost = self.history[idx];
        let cap = self.default_capacity as f64;
        let occ = self.occupancy[idx] as f64;

        let congestion_cost = if occ >= cap {
            (occ - cap + 1.0) * self.current_penalty
        } else {
            0.0
        };

        self.cost_cache[idx] = base_cost + hist_cost + congestion_cost;
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
        self.occupancy[idx] += 1;
        self.update_cache_at(idx);
    }

    fn remove_wire(&mut self, coord: GridCoord) {
        let idx = self.index(coord);
        if self.occupancy[idx] > 0 {
            self.occupancy[idx] -= 1;
            self.update_cache_at(idx);
        }
    }

    #[inline(always)]
    fn get_cost(&self, coord: GridCoord, _collision_penalty: f64) -> f64 {
        let idx = self.index(coord);
        let base_cost = unsafe { *self.cost_cache.get_unchecked(idx) };

        if coord.z == 0 {
            return base_cost * 100.0;
        }

        base_cost
    }

    fn update_history(&mut self, history_increment: f64) {
        for i in 0..self.occupancy.len() {
            if self.occupancy[i] as u32 > self.default_capacity {
                let overflow = (self.occupancy[i] as u32 - self.default_capacity) as f64;
                self.history[i] += overflow * history_increment;
            }
        }
    }

    fn decay_history(&mut self, decay_factor: f64) {
        for i in 0..self.history.len() {
            if self.history[i] > 1.0 {
                self.history[i] *= decay_factor;
                if self.history[i] < 1.0 {
                    self.history[i] = 1.0;
                }
            }
        }
        self.set_penalty(self.current_penalty);
    }

    fn max_occupancy(&self) -> u32 {
        self.occupancy.iter().map(|&x| x as u32).max().unwrap_or(0)
    }

    fn is_congested(&self, coord: GridCoord) -> bool {
        let idx = self.index(coord);
        self.occupancy[idx] as u32 > self.default_capacity
    }

    fn total_conflicts(&self) -> usize {
        self.occupancy
            .iter()
            .filter(|&&x| x as u32 > self.default_capacity)
            .count()
    }

    fn capacity(&self, _coord: GridCoord) -> u32 {
        self.default_capacity
    }

    fn set_penalty(&mut self, penalty: f64) {
        self.current_penalty = penalty;
        for i in 0..self.cost_cache.len() {
            self.update_cache_at(i);
        }
    }
}
