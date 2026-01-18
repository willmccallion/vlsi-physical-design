//! Fast Guide Oracle Implementation.
//!
//! Provides efficient guide constraint checking for detailed routing using
//! precomputed coordinate mapping tables and tag-based tracking.

use crate::algo::astar::GuideOracle;
use crate::utils::conversion::GridConverter;
use eda_common::geom::coord::GridCoord;
use std::collections::HashSet;

/// Fast guide oracle implementation using lookup tables.
///
/// Maintains precomputed mapping tables from fine grid coordinates to
/// coarse grid coordinates, allowing O(1) guide checking. Expands guides
/// by a small radius to provide routing flexibility while maintaining
/// guide adherence. Uses tag-based tracking to support multiple nets
/// without clearing the entire grid.
#[derive(Clone)]
pub struct FastGuideOracle {
    x_map: Vec<u32>,
    y_map: Vec<u32>,
    grid: Vec<u32>,
    coarse_w: u32,
    coarse_h: u32,
    current_net_id: u32,
    allow_all: bool,
}

impl FastGuideOracle {
    /// Creates a new fast guide oracle with coordinate mapping tables.
    ///
    /// Precomputes lookup tables that map fine grid coordinates to coarse
    /// grid coordinates by converting through physical space. This allows
    /// efficient guide checking without repeated coordinate conversions
    /// during pathfinding.
    pub fn new(
        fine_w: u32,
        fine_h: u32,
        coarse_w: u32,
        coarse_h: u32,
        layers: u8,
        fine_conv: &GridConverter,
        coarse_conv: &GridConverter,
    ) -> Self {
        let mut x_map = vec![0; fine_w as usize];
        let mut y_map = vec![0; fine_h as usize];

        for x in 0..fine_w {
            let fine_coord = GridCoord::new(x, 0, 0);
            let world_pos = fine_conv.to_world(fine_coord);
            let coarse_coord = coarse_conv.to_grid(world_pos, 0);
            x_map[x as usize] = coarse_coord.x.min(coarse_w - 1);
        }

        for y in 0..fine_h {
            let fine_coord = GridCoord::new(0, y, 0);
            let world_pos = fine_conv.to_world(fine_coord);
            let coarse_coord = coarse_conv.to_grid(world_pos, 0);
            y_map[y as usize] = coarse_coord.y.min(coarse_h - 1);
        }

        let size = (coarse_w * coarse_h * layers as u32) as usize;
        Self {
            x_map,
            y_map,
            grid: vec![0; size],
            coarse_w,
            coarse_h,
            current_net_id: 0,
            allow_all: false,
        }
    }

    /// Prepares the oracle for routing a specific net with its guides.
    ///
    /// Marks all guide coordinates (and their neighbors within expansion
    /// radius) as valid for the current net. Uses a tag-based system where
    /// each net gets a unique tag, allowing multiple nets to be routed
    /// without clearing the entire grid between queries.
    pub fn prepare(&mut self, net_id: usize, guides: &HashSet<GridCoord>) {
        if guides.is_empty() {
            self.allow_all = true;
            return;
        }
        self.allow_all = false;

        self.current_net_id = (net_id as u32) + 1;
        if self.current_net_id == 0 {
            self.grid.fill(0);
            self.current_net_id = 1;
        }

        let w = self.coarse_w as i32;
        let h = self.coarse_h as i32;

        let expansion = 2i32;

        for &g in guides {
            for dy in -expansion..=expansion {
                for dx in -expansion..=expansion {
                    let nx = g.x as i32 + dx;
                    let ny = g.y as i32 + dy;

                    if nx >= 0 && nx < w && ny >= 0 && ny < h {
                        let idx = (g.z as u32 * self.coarse_w * self.coarse_h
                            + (ny as u32) * self.coarse_w
                            + (nx as u32)) as usize;
                        if idx < self.grid.len() {
                            self.grid[idx] = self.current_net_id;
                        }
                    }
                }
            }
        }
    }
}

impl GuideOracle for FastGuideOracle {
    /// Checks if a grid coordinate is within the routing guide using lookup tables.
    ///
    /// Uses precomputed coordinate mapping tables to convert fine grid coordinates
    /// to coarse grid coordinates in O(1) time. If allow_all is set, returns true
    /// for all coordinates to bypass guide constraints.
    #[inline(always)]
    fn is_in_guide(&self, c: GridCoord) -> bool {
        if self.allow_all {
            return true;
        }
        let cx = unsafe { *self.x_map.get_unchecked(c.x as usize) };
        let cy = unsafe { *self.y_map.get_unchecked(c.y as usize) };
        let idx = (c.z as u32 * self.coarse_w * self.coarse_h + cy * self.coarse_w + cx) as usize;
        unsafe { *self.grid.get_unchecked(idx) == self.current_net_id }
    }
}
