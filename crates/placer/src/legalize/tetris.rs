//! Tetris Legalization Algorithm.
//!
//! Implements a simpler legalization algorithm that places cells one at a time
//! in the nearest available slot, similar to the Tetris game. Cells are sorted
//! by X coordinate and placed in the best available position near their ideal
//! location, searching nearby rows if the ideal row is full.

use pare_common::db::core::NetlistDB;
use std::cmp::Ordering;

/// Tetris-style legalization algorithm implementation.
///
/// Places cells sequentially by finding the nearest available slot to their
/// ideal position. Simpler than Abacus but may produce less optimal results
/// for designs with high utilization.
#[derive(Debug)]
pub struct TetrisLegalizer;

/// Tracks occupancy intervals for a placement row in Tetris legalization.
///
/// Maintains a list of occupied intervals (start, end) and provides
/// methods to find available slots for cell placement. The intervals
/// are merged automatically to maintain a sorted, non-overlapping list.
#[derive(Clone)]
struct RowIntervals {
    blockages: Vec<(f64, f64)>,
    row_y: f64,
    die_max_x: f64,
}

impl RowIntervals {
    /// Creates a new row intervals structure for a placement row.
    ///
    /// Initializes with a single blockage at the die minimum X to represent
    /// the left boundary. The `row_y` parameter stores the Y coordinate of this
    /// row for later use in cell placement.
    fn new(row_y: f64, die_min_x: f64, die_max_x: f64) -> Self {
        Self {
            blockages: vec![(die_min_x, die_min_x)],
            row_y,
            die_max_x,
        }
    }

    /// Adds an occupied interval and merges overlapping intervals.
    ///
    /// Inserts a new occupancy interval and automatically merges it with
    /// existing intervals if they overlap or are adjacent. Maintains the
    /// intervals in sorted order for efficient slot finding.
    fn add_occupancy(&mut self, start: f64, end: f64) {
        self.blockages.push((start, end));
        self.blockages
            .sort_by(|a, b| a.0.total_cmp(&b.0));

        let mut merged = Vec::new();
        if let Some(first) = self.blockages.first() {
            let mut current_start = first.0;
            let mut current_end = first.1;

            for &(next_start, next_end) in self.blockages.iter().skip(1) {
                if next_start < current_end + 1e-6 {
                    current_end = current_end.max(next_end);
                } else {
                    merged.push((current_start, current_end));
                    current_start = next_start;
                    current_end = next_end;
                }
            }
            merged.push((current_start, current_end));
        }
        self.blockages = merged;
    }

    /// Finds the best available slot for a cell of the given width.
    ///
    /// Searches all gaps between occupied intervals to find the slot closest
    /// to the target X position that can accommodate the cell width. Returns
    /// the slot position and distance from target, or None if no suitable
    /// slot exists in this row.
    fn find_best_slot(&self, target_x: f64, width: f64) -> Option<(f64, f64)> {
        let mut best_x = None;
        let mut min_dist = f64::INFINITY;

        for i in 0..self.blockages.len() {
            let (_, occupied_end) = self.blockages[i];

            let next_start = if i + 1 < self.blockages.len() {
                self.blockages[i + 1].0
            } else {
                self.die_max_x
            };

            let gap_size = next_start - occupied_end;

            if gap_size >= width {
                let valid_min = occupied_end;
                let valid_max = next_start - width;

                let candidate_x = target_x.clamp(valid_min, valid_max);
                let dist = (candidate_x - target_x).abs();

                if dist < min_dist {
                    min_dist = dist;
                    best_x = Some(candidate_x);
                }
            }
        }

        best_x.map(|x| (x, min_dist))
    }
}

impl Default for TetrisLegalizer {
    fn default() -> Self {
        Self
    }
}

impl TetrisLegalizer {
    /// Creates a new Tetris legalizer instance.
    pub const fn new() -> Self {
        Self
    }

    /// Legalizes the placement by placing cells in the nearest available slots.
    ///
    /// Sorts cells by X coordinate, then for each cell searches nearby rows
    /// (starting from the ideal row) to find the best available slot. The
    /// algorithm maintains occupancy intervals for each row to track available
    /// space. If no slot is found within the search radius, the cell is placed
    /// at the die boundary as a fallback.
    pub fn legalize(&self, db: &mut NetlistDB) {
        let row_height = db
            .cells
            .iter()
            .filter(|c| !c.is_macro && c.height > 0.0)
            .map(|c| c.height)
            .next()
            .unwrap_or(1.0);

        let die_min_y = db.die_area.min.y;
        let die_height = db.die_area.height();
        let num_rows = (die_height / row_height).ceil() as usize;

        let mut rows: Vec<RowIntervals> = (0..num_rows)
            .map(|i| {
                let y = (i as f64).mul_add(row_height, die_min_y);
                RowIntervals::new(y, db.die_area.min.x, db.die_area.max.x)
            })
            .collect();

        for i in 0..db.num_cells() {
            if db.cells[i].is_fixed {
                let pos = db.positions[i];
                let cell = &db.cells[i];

                let start_row = ((pos.y - die_min_y) / row_height).floor() as isize;
                let end_row =
                    ((pos.y + cell.height - 1e-6 - die_min_y) / row_height).floor() as isize;

                for r in start_row..=end_row {
                    if r >= 0 && (r as usize) < num_rows {
                        rows[r as usize].add_occupancy(pos.x, pos.x + cell.width);
                    }
                }
            }
        }

        let mut indices: Vec<usize> = (0..db.num_cells())
            .filter(|&i| !db.cells[i].is_fixed)
            .collect();

        indices.sort_by(|&a, &b| {
            db.positions[a]
                .x
                .partial_cmp(&db.positions[b].x)
                .unwrap_or(Ordering::Equal)
        });

        for &cell_idx in &indices {
            let cell = &db.cells[cell_idx];
            let original_pos = db.positions[cell_idx];

            let ideal_row_idx = ((original_pos.y - die_min_y) / row_height).round() as isize;

            let search_radius = 20;

            let mut best_pos = None;
            let mut best_cost = f64::INFINITY;
            let mut best_row_idx = 0;

            for offset in 0..=search_radius {
                let candidates = if offset == 0 {
                    vec![0]
                } else {
                    vec![offset, -offset]
                };

                for dir in candidates {
                    let r_idx = ideal_row_idx + dir;
                    if r_idx < 0 || r_idx >= num_rows as isize {
                        continue;
                    }
                    let r_idx = r_idx as usize;

                    if let Some((x, x_cost)) =
                        rows[r_idx].find_best_slot(original_pos.x, cell.width)
                    {
                        let y_cost = (rows[r_idx].row_y - original_pos.y).abs();

                        let total_cost = y_cost.mul_add(2.0, x_cost);

                        if total_cost < best_cost {
                            best_cost = total_cost;
                            best_pos = Some(x);
                            best_row_idx = r_idx;
                        }
                    }
                }

                if best_cost < row_height * 0.1 {
                    break;
                }
            }

            if let Some(x) = best_pos {
                db.positions[cell_idx].x = x;
                db.positions[cell_idx].y = rows[best_row_idx].row_y;

                rows[best_row_idx].add_occupancy(x, x + cell.width);
            } else {
                log::warn!(
                    "Tetris: Could not find legal spot for cell '{}'. Placing at die boundary.",
                    cell.name
                );
                let r_idx = ideal_row_idx.clamp(0, num_rows as isize - 1) as usize;
                db.positions[cell_idx].x = db.die_area.max.x - cell.width;
                db.positions[cell_idx].y = rows[r_idx].row_y;
            }
        }
    }
}
