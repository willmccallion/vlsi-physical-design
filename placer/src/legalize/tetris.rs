use eda_common::db::core::NetlistDB;
use std::cmp::Ordering;

pub struct TetrisLegalizer;

#[derive(Clone)]
struct RowIntervals {
    blockages: Vec<(f64, f64)>,
    row_y: f64,
    die_max_x: f64,
}

impl RowIntervals {
    fn new(row_y: f64, die_min_x: f64, die_max_x: f64) -> Self {
        Self {
            blockages: vec![(die_min_x, die_min_x)],
            row_y,
            die_max_x,
        }
    }

    fn add_occupancy(&mut self, start: f64, end: f64) {
        self.blockages.push((start, end));
        self.blockages
            .sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

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

impl TetrisLegalizer {
    pub fn new() -> Self {
        Self
    }

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
                let y = die_min_y + (i as f64) * row_height;
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

                        let total_cost = x_cost + (y_cost * 2.0);

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
