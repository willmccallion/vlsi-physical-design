//! GCell Grid Implementation for Routing.
//!
//! Implements an edge-based gcell grid where routing capacity is tracked on
//! edges between adjacent gcells. Each gcell covers a physical region of the
//! die, and edges have per-layer capacity equal to the number of routing tracks
//! crossing that boundary. Direction is structurally enforced: horizontal layers
//! only have horizontal edge capacity, vertical layers only have vertical edge
//! capacity.

use super::RoutingGrid;
use pare_common::db::core::{LayerDirection, NetlistDB};
use pare_common::util::visualization::CongestionProvider;

/// Edge-based gcell routing grid.
///
/// Routing capacity and usage are tracked on edges between adjacent gcells.
/// Horizontal edges connect (x,y) to (x+1,y) and carry horizontal wires.
/// Vertical edges connect (x,y) to (x,y+1) and carry vertical wires.
/// Each edge has per-layer capacity, usage, and history cost.
pub struct GCellGrid {
    width: u32,
    height: u32,
    layers: u8,
    gcell_w: f64,
    gcell_h: f64,
    // Horizontal edges: between (x,y) and (x+1,y)
    // Layout: [layer * height * (width-1) + y * (width-1) + x]
    h_cap: Vec<u16>,
    h_usage: Vec<u16>,
    h_history: Vec<u16>,
    // Vertical edges: between (x,y) and (x,y+1)
    // Layout: [layer * (height-1) * width + y * width + x]
    v_cap: Vec<u16>,
    v_usage: Vec<u16>,
    v_history: Vec<u16>,
    current_penalty: f64,
}

impl GCellGrid {
    /// Creates a new GCellGrid from the design database with auto-computed capacities.
    ///
    /// The gcell_size parameter specifies the physical size of each gcell in microns.
    /// Capacities are computed from the track definitions in the database: for each
    /// layer, the number of tracks crossing each gcell boundary determines the edge
    /// capacity. Horizontal layers get horizontal edge capacity, vertical layers get
    /// vertical edge capacity.
    pub fn new(db: &NetlistDB, gcell_size: f64) -> Self {
        let die_w = db.die_area.width();
        let die_h = db.die_area.height();

        // Auto-scale gcell_size to keep grid dimensions manageable.
        // Target max ~500 gcells per dimension to avoid excessive memory
        // and A* search times on large designs.
        let max_dim = 500u32;
        let gcell_size = if die_w / gcell_size > max_dim as f64
            || die_h / gcell_size > max_dim as f64
        {
            let needed = (die_w.max(die_h) / max_dim as f64).ceil();
            // Also align to layer pitch if available
            let min_pitch = db.layers.iter()
                .filter(|l| l.pitch > 0.001)
                .map(|l| l.pitch)
                .fold(f64::MAX, f64::min);
            let scaled = if min_pitch < f64::MAX && min_pitch > 0.001 {
                // Round up to multiple of pitch
                let n = (needed / min_pitch).ceil();
                n * min_pitch
            } else {
                needed
            };
            log::info!(
                "Auto-scaling gcell_size: {:.2} -> {:.2} (die: {:.0}x{:.0})",
                gcell_size, scaled, die_w, die_h
            );
            scaled
        } else {
            gcell_size
        };

        let grid_w = (die_w / gcell_size).ceil() as u32;
        let grid_h = (die_h / gcell_size).ceil() as u32;
        let grid_w = grid_w.max(2);
        let grid_h = grid_h.max(2);

        let layers = if db.layers.is_empty() {
            2
        } else {
            db.layers.len() as u8
        };

        let h_edge_count = (layers as usize) * (grid_h as usize) * ((grid_w - 1) as usize);
        let v_edge_count = (layers as usize) * ((grid_h - 1) as usize) * (grid_w as usize);

        let mut grid = Self {
            width: grid_w,
            height: grid_h,
            layers,
            gcell_w: gcell_size,
            gcell_h: gcell_size,
            h_cap: vec![0; h_edge_count],
            h_usage: vec![0; h_edge_count],
            h_history: vec![0; h_edge_count],
            v_cap: vec![0; v_edge_count],
            v_usage: vec![0; v_edge_count],
            v_history: vec![0; v_edge_count],
            current_penalty: 1.0,
        };

        // Compute per-layer, per-edge capacities from track definitions
        grid.init_capacities(db);

        log::info!(
            "GCellGrid: {}x{} gcells ({:.2}um), {} layers",
            grid_w,
            grid_h,
            gcell_size,
            layers,
        );

        for li in 0..layers {
            if (li as usize) < db.layers.len() {
                let layer = &db.layers[li as usize];
                // Sample capacity from a middle edge
                let sample_h = if grid_w > 2 {
                    grid.h_cap[grid.h_idx(grid_w / 2, grid_h / 2, li)]
                } else {
                    0
                };
                let sample_v = if grid_h > 2 {
                    grid.v_cap[grid.v_idx(grid_w / 2, grid_h / 2, li)]
                } else {
                    0
                };
                log::info!(
                    "  Layer {} ({}): dir={:?}, pitch={:.3}, h_cap={}, v_cap={}",
                    li,
                    layer.name,
                    layer.direction,
                    layer.pitch,
                    sample_h,
                    sample_v,
                );
            }
        }

        grid
    }

    /// Initialize edge capacities from DEF track definitions.
    fn init_capacities(&mut self, db: &NetlistDB) {
        let w = self.width;
        let h = self.height;
        let w1 = (w - 1) as usize;
        let h1 = (h - 1) as usize;
        let gcell_w = self.gcell_w;

        for li in 0..self.layers {
            let li_usize = li as usize;
            if li_usize >= db.layers.len() {
                continue;
            }
            let layer = &db.layers[li_usize];
            let h_base = li_usize * (h as usize) * w1;
            let v_base = li_usize * h1 * (w as usize);

            if li == 0 {
                // M1: high capacity for pin access on all edges
                for y in 0..h as usize {
                    for x in 0..w1 {
                        self.h_cap[h_base + y * w1 + x] = 1000;
                    }
                }
                for y in 0..h1 {
                    for x in 0..w as usize {
                        self.v_cap[v_base + y * (w as usize) + x] = 1000;
                    }
                }
                continue;
            }

            let pitch = if layer.pitch > 0.001 {
                layer.pitch
            } else {
                db.tracks
                    .iter()
                    .find(|t| t.layer == layer.name && t.step > 0.001)
                    .map(|t| t.step)
                    .unwrap_or(0.28)
            };

            let tracks_per_gcell = (gcell_w / pitch).floor().max(1.0) as u16;

            match layer.direction {
                LayerDirection::Horizontal => {
                    for y in 0..h as usize {
                        for x in 0..w1 {
                            self.h_cap[h_base + y * w1 + x] = tracks_per_gcell;
                        }
                    }
                }
                LayerDirection::Vertical => {
                    for y in 0..h1 {
                        for x in 0..w as usize {
                            self.v_cap[v_base + y * (w as usize) + x] = tracks_per_gcell;
                        }
                    }
                }
                LayerDirection::Unknown => {
                    let half = (tracks_per_gcell / 2).max(1);
                    for y in 0..h as usize {
                        for x in 0..w1 {
                            self.h_cap[h_base + y * w1 + x] = half;
                        }
                    }
                    for y in 0..h1 {
                        for x in 0..w as usize {
                            self.v_cap[v_base + y * (w as usize) + x] = half;
                        }
                    }
                }
            }
        }
    }

    /// Returns the physical width of each gcell.
    pub fn gcell_w(&self) -> f64 {
        self.gcell_w
    }

    /// Returns the physical height of each gcell.
    pub fn gcell_h(&self) -> f64 {
        self.gcell_h
    }

    /// Computes index into horizontal edge arrays.
    /// Edge from (x,y) to (x+1,y) on layer.
    #[inline(always)]
    fn h_idx(&self, x: u32, y: u32, layer: u8) -> usize {
        let w_minus_1 = (self.width - 1) as usize;
        (layer as usize) * (self.height as usize) * w_minus_1
            + (y as usize) * w_minus_1
            + (x as usize)
    }

    /// Computes index into vertical edge arrays.
    /// Edge from (x,y) to (x,y+1) on layer.
    #[inline(always)]
    fn v_idx(&self, x: u32, y: u32, layer: u8) -> usize {
        let h_minus_1 = (self.height - 1) as usize;
        (layer as usize) * h_minus_1 * (self.width as usize)
            + (y as usize) * (self.width as usize)
            + (x as usize)
    }

    /// Computes cost for an edge given usage, capacity, and history.
    #[inline(always)]
    fn edge_cost(usage: u16, cap: u16, history: u16, collision_penalty: f64) -> f64 {
        let base = 1.0 + history as f64 * 0.5;
        let overflow = if usage >= cap {
            (usage as u32 - cap as u32 + 1) as f64 * collision_penalty
        } else {
            0.0
        };
        base + overflow
    }
}

impl CongestionProvider for GCellGrid {
    fn get_dims(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn get_congestion_ratio(&self, x: u32, y: u32) -> f32 {
        let mut total_usage = 0.0f32;
        let mut total_cap = 0.0f32;

        for z in 0..self.layers {
            // Check horizontal edges touching this gcell
            if x > 0 {
                let idx = self.h_idx(x - 1, y, z);
                total_usage += self.h_usage[idx] as f32;
                total_cap += self.h_cap[idx] as f32;
            }
            if x < self.width - 1 {
                let idx = self.h_idx(x, y, z);
                total_usage += self.h_usage[idx] as f32;
                total_cap += self.h_cap[idx] as f32;
            }
            // Check vertical edges touching this gcell
            if y > 0 {
                let idx = self.v_idx(x, y - 1, z);
                total_usage += self.v_usage[idx] as f32;
                total_cap += self.v_cap[idx] as f32;
            }
            if y < self.height - 1 {
                let idx = self.v_idx(x, y, z);
                total_usage += self.v_usage[idx] as f32;
                total_cap += self.v_cap[idx] as f32;
            }
        }

        if total_cap > 0.0 {
            total_usage / total_cap
        } else if total_usage > 0.0 {
            2.0
        } else {
            0.0
        }
    }
}

impl RoutingGrid for GCellGrid {
    fn width(&self) -> u32 {
        self.width
    }
    fn height(&self) -> u32 {
        self.height
    }
    fn layers(&self) -> u8 {
        self.layers
    }

    fn h_edge_cost(&self, x: u32, y: u32, layer: u8, collision_penalty: f64) -> f64 {
        if x >= self.width - 1 {
            return f64::MAX;
        }
        let idx = self.h_idx(x, y, layer);
        let cap = self.h_cap[idx];
        if cap == 0 {
            return f64::MAX;
        }
        Self::edge_cost(self.h_usage[idx], cap, self.h_history[idx], collision_penalty)
    }

    fn v_edge_cost(&self, x: u32, y: u32, layer: u8, collision_penalty: f64) -> f64 {
        if y >= self.height - 1 {
            return f64::MAX;
        }
        let idx = self.v_idx(x, y, layer);
        let cap = self.v_cap[idx];
        if cap == 0 {
            return f64::MAX;
        }
        Self::edge_cost(self.v_usage[idx], cap, self.v_history[idx], collision_penalty)
    }

    fn via_cost(&self, _x: u32, _y: u32, _layer: u8) -> f64 {
        // Vias have a small fixed cost to mildly discourage unnecessary layer changes
        1.0
    }

    fn add_h_wire(&mut self, x: u32, y: u32, layer: u8) {
        if x < self.width - 1 {
            let idx = self.h_idx(x, y, layer);
            self.h_usage[idx] = self.h_usage[idx].saturating_add(1);
        }
    }

    fn remove_h_wire(&mut self, x: u32, y: u32, layer: u8) {
        if x < self.width - 1 {
            let idx = self.h_idx(x, y, layer);
            self.h_usage[idx] = self.h_usage[idx].saturating_sub(1);
        }
    }

    fn add_v_wire(&mut self, x: u32, y: u32, layer: u8) {
        if y < self.height - 1 {
            let idx = self.v_idx(x, y, layer);
            self.v_usage[idx] = self.v_usage[idx].saturating_add(1);
        }
    }

    fn remove_v_wire(&mut self, x: u32, y: u32, layer: u8) {
        if y < self.height - 1 {
            let idx = self.v_idx(x, y, layer);
            self.v_usage[idx] = self.v_usage[idx].saturating_sub(1);
        }
    }

    fn is_h_congested(&self, x: u32, y: u32, layer: u8) -> bool {
        if x >= self.width - 1 {
            return false;
        }
        let idx = self.h_idx(x, y, layer);
        self.h_usage[idx] > self.h_cap[idx]
    }

    fn is_v_congested(&self, x: u32, y: u32, layer: u8) -> bool {
        if y >= self.height - 1 {
            return false;
        }
        let idx = self.v_idx(x, y, layer);
        self.v_usage[idx] > self.v_cap[idx]
    }

    fn total_overflow(&self) -> usize {
        let mut overflow = 0usize;
        #[allow(clippy::needless_range_loop)]
        for i in 0..self.h_usage.len() {
            if self.h_usage[i] > self.h_cap[i] {
                overflow += (self.h_usage[i] - self.h_cap[i]) as usize;
            }
        }
        #[allow(clippy::needless_range_loop)]
        for i in 0..self.v_usage.len() {
            if self.v_usage[i] > self.v_cap[i] {
                overflow += (self.v_usage[i] - self.v_cap[i]) as usize;
            }
        }
        overflow
    }

    fn update_history(&mut self, history_increment: f64) {
        #[allow(clippy::needless_range_loop)]
        for i in 0..self.h_usage.len() {
            if self.h_usage[i] > self.h_cap[i] {
                let overflow = (self.h_usage[i] - self.h_cap[i]) as f64;
                let inc = (overflow * history_increment).max(1.0) as u16;
                self.h_history[i] = self.h_history[i].saturating_add(inc);
            }
        }
        #[allow(clippy::needless_range_loop)]
        for i in 0..self.v_usage.len() {
            if self.v_usage[i] > self.v_cap[i] {
                let overflow = (self.v_usage[i] - self.v_cap[i]) as f64;
                let inc = (overflow * history_increment).max(1.0) as u16;
                self.v_history[i] = self.v_history[i].saturating_add(inc);
            }
        }
    }

    fn decay_history(&mut self, decay_factor: f64) {
        let factor = (decay_factor * 1024.0) as u32;
        for h in &mut self.h_history {
            if *h > 0 {
                *h = ((*h as u32 * factor) >> 10) as u16;
            }
        }
        for h in &mut self.v_history {
            if *h > 0 {
                *h = ((*h as u32 * factor) >> 10) as u16;
            }
        }
    }

    fn set_penalty(&mut self, penalty: f64) {
        self.current_penalty = penalty;
    }

    fn h_edge_cap(&self, x: u32, y: u32, layer: u8) -> u16 {
        if x >= self.width - 1 {
            return 0;
        }
        self.h_cap[self.h_idx(x, y, layer)]
    }

    fn v_edge_cap(&self, x: u32, y: u32, layer: u8) -> u16 {
        if y >= self.height - 1 {
            return 0;
        }
        self.v_cap[self.v_idx(x, y, layer)]
    }

    fn h_edge_usage(&self, x: u32, y: u32, layer: u8) -> u16 {
        if x >= self.width - 1 {
            return 0;
        }
        self.h_usage[self.h_idx(x, y, layer)]
    }

    fn v_edge_usage(&self, x: u32, y: u32, layer: u8) -> u16 {
        if y >= self.height - 1 {
            return 0;
        }
        self.v_usage[self.v_idx(x, y, layer)]
    }
}
