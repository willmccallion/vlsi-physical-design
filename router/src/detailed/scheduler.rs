//! Spatial Data Structure for Batching Routing Operations.
//!
//! Maintains a grid of bins to track which regions have active routing
//! operations. Used to schedule routing batches that don't spatially
//! overlap, enabling parallel execution without conflicts. Uses tag-based
//! tracking to reset state efficiently between batches.

/// Spatial data structure for batching routing operations.
///
/// Maintains a grid of bins to track which regions have active routing
/// operations. Used to schedule routing batches that don't spatially
/// overlap, enabling parallel execution without conflicts. Uses tag-based
/// tracking to reset state efficiently between batches.
pub struct SpatialSet {
    grid: Vec<u32>,
    w: u32,
    h: u32,
    bin_size: u32,
    current_batch: u32,
}

impl SpatialSet {
    /// Creates a new spatial set with the specified grid dimensions and bin size.
    ///
    /// Allocates a grid of bins sized to cover the routing area. The bin_size
    /// parameter controls the granularity of spatial conflict detection.
    pub fn new(width: u32, height: u32, bin_size: u32) -> Self {
        let w = width.div_ceil(bin_size);
        let h = height.div_ceil(bin_size);
        Self {
            grid: vec![0; (w * h) as usize],
            w,
            h,
            bin_size,
            current_batch: 0,
        }
    }

    /// Resets the spatial set for a new batch of operations.
    ///
    /// Increments the batch tag to invalidate all previous entries without
    /// clearing the entire array. This provides O(1) reset performance.
    pub fn reset(&mut self) {
        self.current_batch += 1;
        if self.current_batch == 0 {
            self.grid.fill(0);
            self.current_batch = 1;
        }
    }

    /// Attempts to insert a bounding box into the spatial set.
    ///
    /// Checks if the bounding box overlaps with any previously inserted
    /// boxes in the current batch. If no overlap exists, marks all bins
    /// covered by the box and returns true. Otherwise returns false to
    /// indicate a conflict. This is used to schedule non-overlapping
    /// routing operations for parallel execution.
    pub fn try_insert(&mut self, bbox: &(i32, i32, i32, i32)) -> bool {
        let min_x = (bbox.0.max(0) as u32) / self.bin_size;
        let max_x = (bbox.1.max(0) as u32) / self.bin_size;
        let min_y = (bbox.2.max(0) as u32) / self.bin_size;
        let max_y = (bbox.3.max(0) as u32) / self.bin_size;

        let limit_x = self.w - 1;
        let limit_y = self.h - 1;

        for y in min_y..=max_y.min(limit_y) {
            for x in min_x..=max_x.min(limit_x) {
                if self.grid[(y * self.w + x) as usize] == self.current_batch {
                    return false;
                }
            }
        }

        for y in min_y..=max_y.min(limit_y) {
            for x in min_x..=max_x.min(limit_x) {
                self.grid[(y * self.w + x) as usize] = self.current_batch;
            }
        }
        true
    }
}
