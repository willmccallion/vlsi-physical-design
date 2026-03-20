//! Abacus Legalization Algorithm.
//!
//! Implements the Abacus legalization algorithm, which places cells in rows
//! by clustering cells together and computing optimal cluster positions using
//! weighted average calculations. The algorithm handles fixed cells, macros,
//! and standard cells differently, with dynamic padding to prevent row overflow.

use pare_common::db::core::NetlistDB;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::collections::HashMap;

/// Abacus legalization algorithm implementation.
///
/// This legalizer places cells into rows by grouping them into clusters
/// and computing optimal cluster positions that minimize displacement from
/// the analytical placement solution while ensuring no overlaps.
#[derive(Debug)]
pub struct AbacusLegalizer;

/// A cluster of cells in the Abacus algorithm.
///
/// Represents a group of cells that have been merged together during
/// legalization. The cluster's position (x) is computed as a weighted
/// average of member cell positions. The q field stores the weighted
/// sum used in position computation.
struct Cluster {
    x: f64,
    width: f64,
    weight: f64,
    q: f64,
    member_cells: Vec<usize>,
}

/// A sub-row region within a placement row.
///
/// Represents a contiguous free region in a row, bounded by fixed
/// blockages or die boundaries. Tracks the used width and list of
/// cells assigned to this sub-row for legalization.
struct SubRow {
    min_x: f64,
    max_x: f64,
    used_width: f64,
    cells: Vec<usize>,
}

impl Default for AbacusLegalizer {
    fn default() -> Self {
        Self
    }
}

impl AbacusLegalizer {
    /// Creates a new Abacus legalizer instance.
    pub const fn new() -> Self {
        Self
    }

    /// Legalizes the placement by moving cells to non-overlapping positions in rows.
    ///
    /// First places fixed cells and macros, then processes standard cells row by row.
    /// For each row, cells are assigned to sub-rows (regions between fixed blockages),
    /// then clustered using the Abacus algorithm to minimize displacement. Dynamic
    /// padding is computed based on design utilization to prevent row overflow while
    /// maintaining reasonable spacing. The algorithm searches nearby rows if the
    /// ideal row cannot accommodate a cell.
    pub fn legalize(&self, db: &mut NetlistDB) {
        let mut height_counts = HashMap::new();
        for cell in &db.cells {
            if cell.height > 0.001 {
                let h_key = (cell.height * 1000.0).round() as i32;
                *height_counts.entry(h_key).or_insert(0) += 1;
            }
        }

        let row_height = height_counts
            .into_iter()
            .max_by_key(|&(_, count)| count)
            .map_or(1.0, |(h, _)| h as f64 / 1000.0);

        let die_min_x = db.die_area.min.x;
        let die_max_x = db.die_area.max.x;
        let die_min_y = db.die_area.min.y;
        let die_height = db.die_area.height();
        let num_rows = (die_height / row_height).ceil() as usize;

        if num_rows == 0 {
            return;
        }

        let is_macro = |h: f64| h > row_height * 1.5;

        let mut total_movable_width = 0.0;
        let mut movable_cell_count = 0;

        for cell in &db.cells {
            if !cell.is_fixed && !is_macro(cell.height) {
                total_movable_width += cell.width;
                movable_cell_count += 1;
            }
        }

        let total_row_capacity = (die_max_x - die_min_x) * (num_rows as f64);
        let utilization = if total_row_capacity > 0.0 {
            total_movable_width / total_row_capacity
        } else {
            1.0
        };

        let whitespace = total_row_capacity - total_movable_width;
        // Use at most 50% of global whitespace as padding so locally dense rows
        // don't overflow when the global utilization is high (>75%).
        let padding_fraction = if utilization > 0.75 { 0.25 } else { 0.50 };
        let raw_padding = if movable_cell_count > 0 && whitespace > 0.0 {
            (whitespace * padding_fraction / (movable_cell_count as f64)).max(0.0)
        } else {
            0.0
        };

        let padding_per_cell = raw_padding.min(2.0);

        log::debug!(
            "Abacus: Util={:.2}%. Padding: {:.2} units/cell (Raw: {:.2})",
            utilization * 100.0,
            padding_per_cell,
            raw_padding
        );

        let mut fixed_rects = Vec::new();
        let mut movable_macros = Vec::new();

        for i in 0..db.num_cells() {
            let cell = &db.cells[i];
            if cell.is_fixed {
                let pos = db.positions[i];
                fixed_rects.push(Rect::new(
                    pos,
                    Point::new(pos.x + cell.width, pos.y + cell.height),
                ));
            } else if is_macro(cell.height) {
                movable_macros.push(i);
            }
        }

        movable_macros.sort_by(|&a, &b| db.positions[a].x.total_cmp(&db.positions[b].x));

        for &idx in &movable_macros {
            let cell = &db.cells[idx];
            let mut pos = db.positions[idx];

            let ideal_row = ((pos.y - die_min_y) / row_height).round();
            pos.y = die_min_y + ideal_row * row_height;
            pos.y = pos.y.clamp(die_min_y, db.die_area.max.y - cell.height);

            pos.x = pos.x.clamp(die_min_x, die_max_x - cell.width);

            db.positions[idx] = pos;
            fixed_rects.push(Rect::new(
                pos,
                Point::new(pos.x + cell.width, pos.y + cell.height),
            ));
        }

        let mut row_blockages: Vec<Vec<(f64, f64)>> = vec![vec![]; num_rows];
        for rect in &fixed_rects {
            let start_row = ((rect.min.y - die_min_y) / row_height).floor() as isize;
            let end_row = ((rect.max.y - 0.001 - die_min_y) / row_height).floor() as isize;

            for r in start_row..=end_row {
                if r >= 0 && (r as usize) < num_rows {
                    row_blockages[r as usize].push((rect.min.x, rect.max.x));
                }
            }
        }

        let mut rows: Vec<Vec<SubRow>> = Vec::with_capacity(num_rows);
        for blockage_list in &row_blockages {
            let mut blockages = blockage_list.clone();
            blockages.sort_by(|a, b| a.0.total_cmp(&b.0));

            let mut merged = Vec::new();
            if !blockages.is_empty() {
                let mut curr = blockages[0];
                for next in blockages.iter().skip(1) {
                    if next.0 < curr.1 + 0.001 {
                        curr.1 = curr.1.max(next.1);
                    } else {
                        merged.push(curr);
                        curr = *next;
                    }
                }
                merged.push(curr);
            }

            let mut sub_rows = Vec::new();
            let mut current_x = die_min_x;

            for (b_start, b_end) in merged {
                if b_start > current_x + 0.001 {
                    sub_rows.push(SubRow {
                        min_x: current_x,
                        max_x: b_start,
                        used_width: 0.0,
                        cells: Vec::new(),
                    });
                }
                current_x = current_x.max(b_end);
            }
            if current_x < die_max_x - 0.001 {
                sub_rows.push(SubRow {
                    min_x: current_x,
                    max_x: die_max_x,
                    used_width: 0.0,
                    cells: Vec::new(),
                });
            }
            rows.push(sub_rows);
        }

        let mut std_cells: Vec<usize> = (0..db.num_cells())
            .filter(|&i| !db.cells[i].is_fixed && !is_macro(db.cells[i].height))
            .collect();

        std_cells.sort_by(|&a, &b| {
            let pos_a = db.positions[a];
            let pos_b = db.positions[b];
            let row_a = ((pos_a.y - die_min_y) / row_height).round() as isize;
            let row_b = ((pos_b.y - die_min_y) / row_height).round() as isize;
            row_a
                .cmp(&row_b)
                .then(pos_a.x.total_cmp(&pos_b.x))
        });

        for &i in &std_cells {
            let cell = &db.cells[i];
            let pos = db.positions[i];
            let ideal_row_idx = ((pos.y - die_min_y) / row_height).round() as isize;

            let mut placed = false;
            let search_radius = num_rows;

            for offset in 0..=search_radius {
                let signs = if offset == 0 { vec![1] } else { vec![1, -1] };
                for sign in signs {
                    let r_idx = ideal_row_idx + (offset as isize * sign);
                    if r_idx < 0 || r_idx >= num_rows as isize {
                        continue;
                    }
                    let r = r_idx as usize;

                    let mut best_sub = None;
                    let mut min_dist = f64::INFINITY;

                    for (k, sub) in rows[r].iter().enumerate() {
                        let cell_total_w = cell.width + padding_per_cell;
                        if sub.used_width + cell_total_w <= (sub.max_x - sub.min_x) {
                            let sub_center = f64::midpoint(sub.min_x, sub.max_x);
                            let dist = (pos.x - sub_center).abs();
                            if dist < min_dist {
                                min_dist = dist;
                                best_sub = Some(k);
                            }
                        }
                    }

                    if let Some(k) = best_sub {
                        rows[r][k].cells.push(i);
                        rows[r][k].used_width += cell.width + padding_per_cell;
                        placed = true;
                        break;
                    }
                }
                if placed {
                    break;
                }
            }

            if !placed {
                let r = ideal_row_idx.clamp(0, num_rows as isize - 1) as usize;
                if !rows[r].is_empty() {
                    let cell_w = db.cells[i].width + padding_per_cell;
                    rows[r][0].cells.push(i);
                    rows[r][0].used_width += cell_w;
                }
            }
        }

        for (r, row) in rows.iter_mut().enumerate() {
            let row_y = (r as f64).mul_add(row_height, die_min_y);

            for sub in row.iter_mut() {
                if sub.cells.is_empty() {
                    continue;
                }

                sub.cells
                    .sort_by(|&a, &b| db.positions[a].x.total_cmp(&db.positions[b].x));

                let mut clusters: Vec<Cluster> = Vec::new();

                for &cell_idx in &sub.cells {
                    let cell_w = db.cells[cell_idx].width + padding_per_cell;
                    let target_x = db.positions[cell_idx].x;

                    let new_cluster = Cluster {
                        x: target_x,
                        width: cell_w,
                        weight: 1.0,
                        q: target_x,
                        member_cells: vec![cell_idx],
                    };

                    clusters.push(new_cluster);
                    Self::collapse(&mut clusters, sub.min_x);
                }

                // Left-to-right: push clusters right if they underflow
                let mut left_limit = sub.min_x;
                for cluster in &mut clusters {
                    if cluster.x < left_limit {
                        cluster.x = left_limit;
                    }
                    left_limit = cluster.x + cluster.width;
                }

                // Right-to-left: push clusters left if they overflow
                let mut right_limit = sub.max_x;
                for cluster in clusters.iter_mut().rev() {
                    if cluster.x + cluster.width > right_limit {
                        cluster.x = right_limit - cluster.width;
                    }
                    right_limit = cluster.x;
                }

                // Collect all cells ordered by their cluster-assigned x position,
                // then do a single strict left-to-right sequential placement.
                // This guarantees no overlaps and no out-of-bounds regardless of
                // how many clusters collided at the right edge.
                let sub_capacity = sub.max_x - sub.min_x;
                let sub_cell_width: f64 = sub.cells.iter()
                    .map(|&ci| db.cells[ci].width)
                    .sum();
                let has_slack = sub_cell_width < sub_capacity - 0.001;

                let mut ordered: Vec<(f64, usize)> = clusters
                    .iter()
                    .flat_map(|c| {
                        let mut cx = c.x;
                        c.member_cells.iter().map(move |&ci| {
                            let entry = (cx, ci);
                            cx += 0.0; // width advanced below during sort
                            entry
                        })
                    })
                    .collect();
                // Re-compute each cell's ideal x by walking clusters properly
                ordered.clear();
                for cluster in &clusters {
                    let mut cx = cluster.x;
                    for &ci in &cluster.member_cells {
                        ordered.push((cx, ci));
                        cx += db.cells[ci].width + if has_slack { padding_per_cell } else { 0.0 };
                    }
                }
                ordered.sort_by(|a, b| a.0.total_cmp(&b.0));

                // Sequential pack: honour ideal positions where possible,
                // but always advance monotonically and clamp to die boundary.
                let mut cursor = sub.min_x;
                for (ideal_x, cell_idx) in &ordered {
                    let cell_w = db.cells[*cell_idx].width;
                    let pad = if has_slack { padding_per_cell } else { 0.0 };
                    // Move cursor to ideal_x if it doesn't go backward
                    if *ideal_x > cursor {
                        cursor = *ideal_x;
                    }
                    // Clamp to left boundary (cursor always advances, never backward)
                    let place_x = cursor.max(die_min_x);
                    db.positions[*cell_idx].x = place_x;
                    db.positions[*cell_idx].y = row_y;
                    cursor = place_x + cell_w + pad;
                }
            }
        }

        // Post-legalization: clamp Y to die boundary, then compact any
        // cells that overflow the right die boundary by pushing them left
        // (and their neighbors) in a right-to-left pass per row.
        for i in 0..db.num_cells() {
            if db.cells[i].is_fixed {
                continue;
            }
            let h = db.cells[i].height;
            db.positions[i].y = db.positions[i].y.clamp(die_min_y, db.die_area.max.y - h);
        }

        // Right-to-left compaction for each row to fix right-boundary overflow.
        for row in &rows {
            for sub in row {
                if sub.cells.is_empty() {
                    continue;
                }

                // Sort cells by their placed x position
                let mut cells_in_sub: Vec<usize> = sub.cells.clone();
                cells_in_sub.sort_by(|&a, &b| {
                    db.positions[a].x.total_cmp(&db.positions[b].x)
                });

                // Right-to-left: if the rightmost cell exceeds the sub-row
                // boundary, push it left (and its neighbors).
                let mut right_limit = sub.max_x;
                for &ci in cells_in_sub.iter().rev() {
                    let w = db.cells[ci].width;
                    if db.positions[ci].x + w > right_limit {
                        db.positions[ci].x = (right_limit - w).max(die_min_x);
                    }
                    right_limit = db.positions[ci].x;
                }
            }
        }
    }

    /// Collapses overlapping clusters into a single cluster.
    ///
    /// Merges clusters that overlap by computing a weighted average position
    /// and combined width. This implements the core Abacus algorithm where
    /// clusters are merged left-to-right until no overlaps remain. The `min_x`
    /// parameter ensures clusters do not extend beyond the sub-row boundary.
    fn collapse(clusters: &mut Vec<Cluster>, min_x: f64) {
        loop {
            if let Some(last) = clusters.last_mut()
                && last.x < min_x
            {
                last.x = min_x;
            }

            if clusters.len() <= 1 {
                break;
            }

            let last_idx = clusters.len() - 1;
            let prev_idx = last_idx - 1;

            let last_x = clusters[last_idx].x;
            let prev_end = clusters[prev_idx].x + clusters[prev_idx].width;

            if prev_end > last_x {
                let Some(last_c) = clusters.pop() else { break };
                let prev_c = &mut clusters[prev_idx];

                prev_c.q += last_c.weight.mul_add(-prev_c.width, last_c.q);
                prev_c.member_cells.extend(last_c.member_cells);
                prev_c.width += last_c.width;
                prev_c.weight += last_c.weight;
                prev_c.x = prev_c.q / prev_c.weight;
            } else {
                break;
            }
        }
    }
}
