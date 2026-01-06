use eda_common::db::core::NetlistDB;
use eda_common::geom::point::Point;
use eda_common::geom::rect::Rect;
use std::collections::HashMap;

pub struct AbacusLegalizer;

struct Cluster {
    x: f64,
    width: f64,
    weight: f64,
    q: f64,
    member_cells: Vec<usize>,
}

struct SubRow {
    min_x: f64,
    max_x: f64,
    used_width: f64,
    cells: Vec<usize>,
}

impl AbacusLegalizer {
    pub fn new() -> Self {
        Self
    }

    pub fn legalize(&self, db: &mut NetlistDB) {
        const CELL_PADDING: f64 = 0.05;

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
            .map(|(h, _)| h as f64 / 1000.0)
            .unwrap_or(1.0);

        let die_min_x = db.die_area.min.x;
        let die_max_x = db.die_area.max.x;
        let die_min_y = db.die_area.min.y;
        let die_height = db.die_area.height();
        let num_rows = (die_height / row_height).ceil() as usize;

        if num_rows == 0 {
            return;
        }

        let is_macro = |h: f64| h > row_height * 1.5;

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

        movable_macros.sort_by(|&a, &b| db.positions[a].x.partial_cmp(&db.positions[b].x).unwrap());

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
        for r in 0..num_rows {
            let mut blockages = row_blockages[r].clone();
            blockages.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

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
                .then(pos_a.x.partial_cmp(&pos_b.x).unwrap())
        });

        for &i in &std_cells {
            let cell = &db.cells[i];
            let pos = db.positions[i];
            let ideal_row_idx = ((pos.y - die_min_y) / row_height).round() as isize;

            let mut placed = false;
            let search_radius = 50;

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
                        let cell_total_w = cell.width + CELL_PADDING;
                        if sub.used_width + cell_total_w <= (sub.max_x - sub.min_x) {
                            let sub_center = (sub.min_x + sub.max_x) / 2.0;
                            let dist = (pos.x - sub_center).abs();
                            if dist < min_dist {
                                min_dist = dist;
                                best_sub = Some(k);
                            }
                        }
                    }

                    if let Some(k) = best_sub {
                        rows[r][k].cells.push(i);
                        rows[r][k].used_width += cell.width + CELL_PADDING;
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
                    rows[r][0].cells.push(i);
                }
            }
        }

        for r in 0..num_rows {
            let row_y = die_min_y + (r as f64) * row_height;

            for sub in rows[r].iter_mut() {
                if sub.cells.is_empty() {
                    continue;
                }

                sub.cells
                    .sort_by(|&a, &b| db.positions[a].x.partial_cmp(&db.positions[b].x).unwrap());

                let mut clusters: Vec<Cluster> = Vec::new();

                for &cell_idx in &sub.cells {
                    let cell_w = db.cells[cell_idx].width + CELL_PADDING;
                    let target_x = db.positions[cell_idx].x;

                    let new_cluster = Cluster {
                        x: target_x,
                        width: cell_w,
                        weight: 1.0,
                        q: target_x,
                        member_cells: vec![cell_idx],
                    };

                    clusters.push(new_cluster);
                    self.collapse(&mut clusters, sub.min_x);
                }

                let mut left_limit = sub.min_x;
                for cluster in clusters.iter_mut() {
                    if cluster.x < left_limit {
                        cluster.x = left_limit;
                    }
                    left_limit = cluster.x + cluster.width;
                }

                let mut right_limit = sub.max_x;
                for cluster in clusters.iter_mut().rev() {
                    if cluster.x + cluster.width > right_limit {
                        cluster.x = right_limit - cluster.width;
                    }
                    right_limit = cluster.x;
                }

                for cluster in &clusters {
                    let mut current_x = cluster.x;
                    for &cell_idx in &cluster.member_cells {
                        current_x = current_x.max(sub.min_x);
                        let safe_x = current_x;

                        db.positions[cell_idx].x = safe_x;
                        db.positions[cell_idx].y = row_y;
                        current_x += db.cells[cell_idx].width + CELL_PADDING;
                    }
                }
            }
        }
    }

    fn collapse(&self, clusters: &mut Vec<Cluster>, min_x: f64) {
        loop {
            if let Some(last) = clusters.last_mut() {
                if last.x < min_x {
                    last.x = min_x;
                }
            }

            if clusters.len() <= 1 {
                break;
            }

            let last_idx = clusters.len() - 1;
            let prev_idx = last_idx - 1;

            let last_x = clusters[last_idx].x;
            let prev_end = clusters[prev_idx].x + clusters[prev_idx].width;

            if prev_end > last_x {
                let last_c = clusters.pop().unwrap();
                let prev_c = &mut clusters[prev_idx];

                prev_c.q += last_c.q - (last_c.weight * prev_c.width);
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
