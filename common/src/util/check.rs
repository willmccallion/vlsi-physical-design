use crate::db::core::NetlistDB;
use crate::db::indices::NetId;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use rayon::prelude::*;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

const CHECK_TOLERANCE: f64 = 0.005; // 5nm tolerance
const BIN_SIZE: f64 = 10.0;

pub fn run_placement_check(db: &NetlistDB) -> Result<(), String> {
    log::info!("Starting Placement Verification...");
    let valid = AtomicBool::new(true);

    db.cells.par_iter().enumerate().for_each(|(i, cell)| {
        if cell.name == "IO_VIRTUAL_CELL" {
            return;
        }

        let pos = db.positions[i];
        let cell_rect = Rect::new(pos, Point::new(pos.x + cell.width, pos.y + cell.height));

        if cell_rect.min.x < db.die_area.min.x - CHECK_TOLERANCE
            || cell_rect.min.y < db.die_area.min.y - CHECK_TOLERANCE
            || cell_rect.max.x > db.die_area.max.x + CHECK_TOLERANCE
            || cell_rect.max.y > db.die_area.max.y + CHECK_TOLERANCE
        {
            log::error!("FAIL: Cell '{}' out of bounds.", cell.name);
            valid.store(false, Ordering::Relaxed);
        }
    });

    let has_overlap = (0..db.num_cells()).into_par_iter().any(|i| {
        if db.cells[i].name == "IO_VIRTUAL_CELL" {
            return false;
        }

        let r1 = Rect::new(
            db.positions[i],
            Point::new(
                db.positions[i].x + db.cells[i].width,
                db.positions[i].y + db.cells[i].height,
            ),
        );
        let r1_shrink = Rect::new(
            Point::new(r1.min.x + CHECK_TOLERANCE, r1.min.y + CHECK_TOLERANCE),
            Point::new(r1.max.x - CHECK_TOLERANCE, r1.max.y - CHECK_TOLERANCE),
        );

        for j in (i + 1)..db.num_cells() {
            if db.cells[j].name == "IO_VIRTUAL_CELL" {
                continue;
            }

            let r2 = Rect::new(
                db.positions[j],
                Point::new(
                    db.positions[j].x + db.cells[j].width,
                    db.positions[j].y + db.cells[j].height,
                ),
            );

            if r1_shrink.overlaps(&r2) {
                log::error!(
                    "FAIL: Cell Overlap '{}' and '{}'",
                    db.cells[i].name,
                    db.cells[j].name
                );
                return true;
            }
        }
        false
    });

    if has_overlap {
        valid.store(false, Ordering::Relaxed);
    }

    if valid.load(Ordering::Relaxed) {
        log::info!("\x1b[32mPASS\x1b[0m: Placement is valid.");
        Ok(())
    } else {
        Err("Placement verification failed.".to_string())
    }
}

pub fn run(db: &NetlistDB) -> Result<(), String> {
    log::info!("Starting Design Verification (DRC/LVS) [High Precision Mode]");

    let (shorts_result, opens_result) =
        rayon::join(|| check_shorts_and_loops(db), || check_opens(db));

    let mut valid = true;
    let mut msgs = Vec::new();

    match shorts_result {
        Err(e) => {
            log::error!("\x1b[31mFAIL\x1b[0m: Short Circuits / Loops Detected");
            log::error!("{}", e);
            msgs.push(e);
            valid = false;
        }
        Ok(_) => log::info!("\x1b[32mPASS\x1b[0m: No Shorts or Illegal Loops found."),
    }

    match opens_result {
        Err(e) => {
            log::error!("\x1b[31mFAIL\x1b[0m: Open Net (Disconnected) Detected");
            log::error!("{}", e);
            msgs.push(e);
            valid = false;
        }
        Ok(_) => log::info!("\x1b[32mPASS\x1b[0m: All nets are fully connected."),
    }

    if valid {
        log::info!("\x1b[32mSUCCESS\x1b[0m: VALID CHIP");
        Ok(())
    } else {
        log::error!(
            "\x1b[31mFAILURE\x1b[0m: INVALID CHIP ({} Errors)",
            msgs.len()
        );
        Err(msgs.join("; "))
    }
}

#[derive(Clone, Copy, Debug)]
struct Segment {
    p1: Point<f64>,
    p2: Point<f64>,
    layer: u8,
    net_id: NetId,
    seg_idx: usize,
}

impl Segment {
    fn intersects(&self, other: &Segment) -> bool {
        if self.layer != other.layer {
            return false;
        }

        let min_x1 = self.p1.x.min(self.p2.x) - CHECK_TOLERANCE;
        let max_x1 = self.p1.x.max(self.p2.x) + CHECK_TOLERANCE;
        let min_y1 = self.p1.y.min(self.p2.y) - CHECK_TOLERANCE;
        let max_y1 = self.p1.y.max(self.p2.y) + CHECK_TOLERANCE;

        let min_x2 = other.p1.x.min(other.p2.x) - CHECK_TOLERANCE;
        let max_x2 = other.p1.x.max(other.p2.x) + CHECK_TOLERANCE;
        let min_y2 = other.p1.y.min(other.p2.y) - CHECK_TOLERANCE;
        let max_y2 = other.p1.y.max(other.p2.y) + CHECK_TOLERANCE;

        if max_x1 < min_x2 || min_x1 > max_x2 || max_y1 < min_y2 || min_y1 > max_y2 {
            return false;
        }

        fn on_segment(p: Point<f64>, a: Point<f64>, b: Point<f64>) -> bool {
            p.x >= a.x.min(b.x) - CHECK_TOLERANCE
                && p.x <= a.x.max(b.x) + CHECK_TOLERANCE
                && p.y >= a.y.min(b.y) - CHECK_TOLERANCE
                && p.y <= a.y.max(b.y) + CHECK_TOLERANCE
        }

        let o1 = orientation(self.p1, self.p2, other.p1);
        let o2 = orientation(self.p1, self.p2, other.p2);
        let o3 = orientation(other.p1, other.p2, self.p1);
        let o4 = orientation(other.p1, other.p2, self.p2);

        if o1 != o2 && o3 != o4 {
            return true;
        }

        if o1 == 0 && on_segment(other.p1, self.p1, self.p2) {
            return true;
        }
        if o2 == 0 && on_segment(other.p2, self.p1, self.p2) {
            return true;
        }
        if o3 == 0 && on_segment(self.p1, other.p1, other.p2) {
            return true;
        }
        if o4 == 0 && on_segment(self.p2, other.p1, other.p2) {
            return true;
        }

        false
    }

    fn shares_endpoint(&self, other: &Segment) -> bool {
        let dist_sq = |a: Point<f64>, b: Point<f64>| (a.x - b.x).powi(2) + (a.y - b.y).powi(2);
        let tol_sq = CHECK_TOLERANCE * CHECK_TOLERANCE;

        dist_sq(self.p1, other.p1) < tol_sq
            || dist_sq(self.p1, other.p2) < tol_sq
            || dist_sq(self.p2, other.p1) < tol_sq
            || dist_sq(self.p2, other.p2) < tol_sq
    }
}

fn orientation(p: Point<f64>, q: Point<f64>, r: Point<f64>) -> i32 {
    let val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if val.abs() < CHECK_TOLERANCE {
        return 0;
    }
    if val > 0.0 { 1 } else { 2 }
}

#[derive(Hash, Eq, PartialEq, PartialOrd, Ord, Clone, Copy, Debug)]
struct BinKey {
    layer: u8,
    bx: i32,
    by: i32,
}

fn check_shorts_and_loops(db: &NetlistDB) -> Result<(), String> {
    let mut all_bin_entries: Vec<(BinKey, Segment)> = db
        .nets
        .par_iter()
        .enumerate()
        .flat_map(|(net_idx, net)| {
            let net_id = NetId::new(net_idx);
            let mut entries = Vec::new();
            for (seg_idx, seg) in net.route_segments.iter().enumerate() {
                let s = Segment {
                    p1: seg.p1,
                    p2: seg.p2,
                    layer: seg.layer,
                    net_id,
                    seg_idx,
                };

                let min_x = s.p1.x.min(s.p2.x);
                let max_x = s.p1.x.max(s.p2.x);
                let min_y = s.p1.y.min(s.p2.y);
                let max_y = s.p1.y.max(s.p2.y);

                let start_bx = (min_x / BIN_SIZE).floor() as i32;
                let end_bx = (max_x / BIN_SIZE).floor() as i32;
                let start_by = (min_y / BIN_SIZE).floor() as i32;
                let end_by = (max_y / BIN_SIZE).floor() as i32;

                for bx in start_bx..=end_bx {
                    for by in start_by..=end_by {
                        entries.push((
                            BinKey {
                                layer: s.layer,
                                bx,
                                by,
                            },
                            s,
                        ));
                    }
                }
            }
            entries
        })
        .collect();

    all_bin_entries.par_sort_unstable_by(|a, b| a.0.cmp(&b.0));

    let mut chunks = Vec::new();
    if !all_bin_entries.is_empty() {
        let mut start = 0;
        for i in 1..all_bin_entries.len() {
            if all_bin_entries[i].0 != all_bin_entries[i - 1].0 {
                chunks.push((start, i));
                start = i;
            }
        }
        chunks.push((start, all_bin_entries.len()));
    }

    let error_found = AtomicBool::new(false);
    let error_msg = Arc::new(Mutex::new(String::new()));

    chunks.par_iter().for_each(|&(start, end)| {
        if error_found.load(Ordering::Relaxed) {
            return;
        }

        let slice = &all_bin_entries[start..end];

        for i in 0..slice.len() {
            for j in (i + 1)..slice.len() {
                let s1 = &slice[i].1;
                let s2 = &slice[j].1;

                if s1.net_id == s2.net_id && s1.seg_idx == s2.seg_idx {
                    continue;
                }

                if s1.intersects(s2) {
                    if s1.net_id != s2.net_id {
                        let n1 = &db.nets[s1.net_id.index()].name;
                        let n2 = &db.nets[s2.net_id.index()].name;
                        let msg = format!("SHORT: '{}' vs '{}' on Layer {}", n1, n2, s1.layer);

                        if !error_found.swap(true, Ordering::Relaxed) {
                            *error_msg.lock().unwrap() = msg;
                        }
                        return;
                    } else {
                        if !s1.shares_endpoint(s2) {
                            let is_via1 = (s1.p1.x - s1.p2.x).abs() < 1e-6 && (s1.p1.y - s1.p2.y).abs() < 1e-6;
                            let is_via2 = (s2.p1.x - s2.p2.x).abs() < 1e-6 && (s2.p1.y - s2.p2.y).abs() < 1e-6;

                            if is_via1 || is_via2 {
                                continue;
                            }

                            let n1 = &db.nets[s1.net_id.index()].name;
                            let msg = format!(
                                "SELF-SHORT/LOOP: Net '{}' intersects itself on Layer {} near ({:.3},{:.3})",
                                n1, s1.layer, s1.p1.x, s1.p1.y
                            );

                            if !error_found.swap(true, Ordering::Relaxed) {
                                *error_msg.lock().unwrap() = msg;
                            }
                            return;
                        }
                    }
                }
            }
        }
    });

    if error_found.load(Ordering::Relaxed) {
        Err(error_msg.lock().unwrap().clone())
    } else {
        Ok(())
    }
}

fn check_opens(db: &NetlistDB) -> Result<(), String> {
    let error_found = AtomicBool::new(false);
    let error_msg = Arc::new(Mutex::new(String::new()));

    db.nets.par_iter().enumerate().for_each(|(net_idx, net)| {
        if error_found.load(Ordering::Relaxed) {
            return;
        }
        if net.pins.len() < 2 {
            return;
        }

        let segments: Vec<Segment> = net
            .route_segments
            .iter()
            .enumerate()
            .map(|(i, s)| Segment {
                p1: s.p1,
                p2: s.p2,
                layer: s.layer,
                net_id: NetId::new(net_idx),
                seg_idx: i,
            })
            .collect();

        let n = segments.len();
        if n == 0 {
            if !error_found.swap(true, Ordering::Relaxed) {
                *error_msg.lock().unwrap() = format!("Net '{}': Unrouted (No segments)", net.name);
            }
            return;
        }

        let mut adj = vec![Vec::new(); n];

        for i in 0..n {
            for j in (i + 1)..n {
                let s1 = &segments[i];
                let s2 = &segments[j];

                let same_layer = s1.layer == s2.layer;
                let adj_layer = (s1.layer as i32 - s2.layer as i32).abs() == 1;

                if same_layer {
                    if s1.intersects(s2) || s1.shares_endpoint(s2) {
                        adj[i].push(j);
                        adj[j].push(i);
                    }
                } else if adj_layer && segments_overlap_2d(s1, s2) {
                    adj[i].push(j);
                    adj[j].push(i);
                }
            }
        }

        let mut pin_segment_indices = Vec::new();
        for (pin_idx_in_net, &pin_id) in net.pins.iter().enumerate() {
            let cell_id = db.pin_to_cell[pin_id.index()];
            let pos = db.positions[cell_id.index()];
            let pin_pos = db.get_pin_position(pin_id, &pos);

            let mut found = false;
            for (seg_i, seg) in segments.iter().enumerate() {
                if seg.layer == 0
                    && point_to_segment_dist(pin_pos, seg.p1, seg.p2) < CHECK_TOLERANCE
                {
                    pin_segment_indices.push(seg_i);
                    found = true;
                    break;
                }
            }
            if !found {
                if !error_found.swap(true, Ordering::Relaxed) {
                    *error_msg.lock().unwrap() = format!(
                        "Net '{}': Pin {} at ({:.3},{:.3}) not connected to any wire.",
                        net.name, pin_idx_in_net, pin_pos.x, pin_pos.y
                    );
                }
                return;
            }
        }

        if pin_segment_indices.is_empty() {
            return;
        }

        let start_node = pin_segment_indices[0];
        let mut visited = vec![false; n];
        let mut queue = VecDeque::new();

        visited[start_node] = true;
        queue.push_back(start_node);

        while let Some(u) = queue.pop_front() {
            for &v in &adj[u] {
                if !visited[v] {
                    visited[v] = true;
                    queue.push_back(v);
                }
            }
        }

        for &seg_idx in &pin_segment_indices {
            if !visited[seg_idx] {
                if !error_found.swap(true, Ordering::Relaxed) {
                    *error_msg.lock().unwrap() =
                        format!("Net '{}': Broken connectivity (Split net).", net.name);
                }
                return;
            }
        }
    });

    if error_found.load(Ordering::Relaxed) {
        Err(error_msg.lock().unwrap().clone())
    } else {
        Ok(())
    }
}

fn segments_overlap_2d(s1: &Segment, s2: &Segment) -> bool {
    let mut s1_2d = *s1;
    s1_2d.layer = 0;
    let mut s2_2d = *s2;
    s2_2d.layer = 0;
    s1_2d.intersects(&s2_2d)
}

fn point_to_segment_dist(p: Point<f64>, a: Point<f64>, b: Point<f64>) -> f64 {
    let l2 = (a.x - b.x).powi(2) + (a.y - b.y).powi(2);
    if l2 == 0.0 {
        return ((p.x - a.x).powi(2) + (p.y - a.y).powi(2)).sqrt();
    }

    let t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
    let t = t.clamp(0.0, 1.0);

    let proj_x = a.x + t * (b.x - a.x);
    let proj_y = a.y + t * (b.y - a.y);

    ((p.x - proj_x).powi(2) + (p.y - proj_y).powi(2)).sqrt()
}
