use eda_common::db::core::NetlistDB;

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
    cells: Vec<usize>,
}

impl AbacusLegalizer {
    pub fn new() -> Self {
        Self
    }

    pub fn legalize(&self, db: &mut NetlistDB) {
        const CELL_PADDING: f64 = 0.38;

        let row_height = db
            .cells
            .iter()
            .map(|c| c.height)
            .find(|&h| h > 0.001)
            .unwrap_or(1.0);
        let num_rows = (db.die_area.height() / row_height).floor() as usize;
        if num_rows == 0 {
            return;
        }

        let die_min_x = db.die_area.min.x;
        let die_max_x = db.die_area.max.x;
        let mut row_blockages: Vec<Vec<(f64, f64)>> = vec![vec![]; num_rows];

        for i in 0..db.num_cells() {
            if db.cells[i].is_fixed {
                let pos = db.positions[i];
                let h = db.cells[i].height;
                let w = db.cells[i].width;
                let start_row = ((pos.y - db.die_area.min.y) / row_height).floor() as isize;
                let end_row =
                    ((pos.y + h - 0.001 - db.die_area.min.y) / row_height).floor() as isize;
                for r in start_row..=end_row {
                    if r >= 0 && r < num_rows as isize {
                        row_blockages[r as usize].push((pos.x, pos.x + w));
                    }
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
                    if next.0 < curr.1 {
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
                        cells: Vec::new(),
                    });
                }
                current_x = current_x.max(b_end);
            }
            if current_x < die_max_x - 0.001 {
                sub_rows.push(SubRow {
                    min_x: current_x,
                    max_x: die_max_x,
                    cells: Vec::new(),
                });
            }
            rows.push(sub_rows);
        }

        for i in 0..db.num_cells() {
            if db.cells[i].is_fixed {
                continue;
            }
            let pos = db.positions[i];
            let y_clamped = pos
                .y
                .clamp(db.die_area.min.y, db.die_area.max.y - db.cells[i].height);

            let r_idx = ((y_clamped - db.die_area.min.y) / row_height).floor() as usize;
            let r_idx = r_idx.min(num_rows - 1);
            let cell_center_x = pos.x + db.cells[i].width / 2.0;
            let mut best_sub = 0;
            let mut min_dist = f64::INFINITY;
            let mut found = false;

            if !rows[r_idx].is_empty() {
                for (k, sub) in rows[r_idx].iter().enumerate() {
                    if sub.max_x - sub.min_x >= db.cells[i].width {
                        let sub_center = (sub.min_x + sub.max_x) / 2.0;
                        let dist = (sub_center - cell_center_x).abs();
                        if dist < min_dist {
                            min_dist = dist;
                            best_sub = k;
                            found = true;
                        }
                    }
                }
                if found {
                    rows[r_idx][best_sub].cells.push(i);
                } else {
                    rows[r_idx][0].cells.push(i);
                }
            }
        }

        for r in 0..num_rows {
            let row_y = db.die_area.min.y + (r as f64) * row_height;
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
                        let safe_x =
                            current_x.clamp(sub.min_x, sub.max_x - db.cells[cell_idx].width);
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
