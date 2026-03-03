//! Design Rule Checking and Layout Versus Schematic Verification.
//!
//! This module implements verification algorithms to ensure the placed and routed
//! design meets physical design rules and maintains electrical connectivity.
//! It checks for cell overlaps, die boundary violations, short circuits between
//! different nets, and open nets (disconnected pins).

use crate::db::core::NetlistDB;
use crate::db::indices::NetId;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use crate::util::ui;
use rayon::prelude::*;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// Geometric tolerance for floating point comparisons and overlap checks.
const CHECK_TOLERANCE: f64 = 0.001;
/// Maximum squared distance for two points to be considered connected.
const CONNECTIVITY_TOLERANCE: f64 = 0.5;
/// Size of spatial bins used for accelerating intersection tests.
const BIN_SIZE: f64 = 10.0;

/// Verifies that the placement is legal (no overlaps, all cells within die area).
///
/// Checks that all cells are positioned within the die boundaries and that
/// no two cells overlap. Uses parallel iteration for efficiency on large designs.
/// Returns an error if any violations are detected, otherwise returns Ok(()).
pub fn run_placement_check(db: &NetlistDB) -> Result<(), String> {
    log::debug!("Starting Placement Verification...");
    let valid = AtomicBool::new(true);

    db.cells.par_iter().enumerate().for_each(|(i, cell)| {
        if cell.name == "IO_VIRTUAL_CELL" || cell.is_fixed {
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
        if db.cells[i].name == "IO_VIRTUAL_CELL" || db.cells[i].is_fixed {
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
            if db.cells[j].name == "IO_VIRTUAL_CELL" || db.cells[j].is_fixed {
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
        ui::check("Placement is valid.");
        Ok(())
    } else {
        ui::fail("Placement verification failed.");
        Err("Placement verification failed.".to_string())
    }
}

/// Performs complete design verification including shorts and opens detection.
///
/// Executes parallel checks for short circuits (wires from different nets
/// intersecting) and open nets (nets with disconnected pins). Uses spatial
/// binning to accelerate intersection tests. Returns an error if any violations
/// are found, otherwise returns Ok(()). This is the main verification entry point
/// called after routing completes.
pub fn run(db: &NetlistDB) -> Result<(), String> {
    log::debug!("Starting Design Verification (DRC/LVS)");

    let (shorts_result, opens_result) =
        rayon::join(|| check_shorts_and_loops(db), || check_opens(db));

    let mut valid = true;
    let mut msgs = Vec::new();

    match shorts_result {
        Err(e) => {
            ui::fail("Short Circuits / Loops Detected");
            log::error!("{}", e);
            msgs.push(e);
            valid = false;
        }
        Ok(_) => ui::check("DRC passed -- no shorts or illegal loops."),
    }

    match opens_result {
        Err(e) => {
            ui::fail("Open Net (Disconnected) Detected");
            log::error!("{}", e);
            msgs.push(e);
            valid = false;
        }
        Ok(_) => ui::check("LVS passed -- all nets fully connected."),
    }

    if valid {
        Ok(())
    } else {
        ui::fail(&format!("INVALID CHIP ({} Errors)", msgs.len()));
        Err(msgs.join("; "))
    }
}

/// Internal representation of a routing segment for verification.
///
/// Wraps a route segment with additional metadata (net ID, segment index)
/// needed for verification algorithms. Used to track which segments belong
/// to which nets for short circuit detection.
#[derive(Clone, Copy, Debug)]
struct Segment {
    p1: Point<f64>,
    p2: Point<f64>,
    layer: u8,
    net_id: NetId,
    seg_idx: usize,
}

impl Segment {
    /// Checks if this segment intersects with another segment on the same layer.
    ///
    /// Uses the orientation test to detect crossing segments and collinear
    /// overlap detection for segments that lie on the same line. Returns
    /// false immediately if segments are on different layers. Uses tolerance
    /// to handle numerical precision issues while avoiding false positives
    /// from segments that only touch at endpoints.
    fn intersects(&self, other: &Segment) -> bool {
        if self.layer != other.layer {
            return false;
        }

        let min_x1 = self.p1.x.min(self.p2.x);
        let max_x1 = self.p1.x.max(self.p2.x);
        let min_y1 = self.p1.y.min(self.p2.y);
        let max_y1 = self.p1.y.max(self.p2.y);

        let min_x2 = other.p1.x.min(other.p2.x);
        let max_x2 = other.p1.x.max(other.p2.x);
        let min_y2 = other.p1.y.min(other.p2.y);
        let max_y2 = other.p1.y.max(other.p2.y);

        if max_x1 <= min_x2 + CHECK_TOLERANCE
            || min_x1 >= max_x2 - CHECK_TOLERANCE
            || max_y1 <= min_y2 + CHECK_TOLERANCE
            || min_y1 >= max_y2 - CHECK_TOLERANCE
        {
            return false;
        }

        /// Checks if a point lies on a line segment within tolerance.
        ///
        /// Determines whether point p is collinear with and between points a and b,
        /// accounting for floating-point precision using CHECK_TOLERANCE. Used in
        /// segment intersection tests to detect when segments share endpoints.
        fn on_segment(p: Point<f64>, a: Point<f64>, b: Point<f64>) -> bool {
            p.x >= a.x.min(b.x) + CHECK_TOLERANCE
                && p.x <= a.x.max(b.x) - CHECK_TOLERANCE
                && p.y >= a.y.min(b.y) + CHECK_TOLERANCE
                && p.y <= a.y.max(b.y) - CHECK_TOLERANCE
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

    /// Checks if this segment shares an endpoint with another segment.
    ///
    /// Compares all four endpoint combinations and returns true if any pair
    /// is within the connectivity tolerance. This is used to determine if
    /// segments are connected (which is legal) versus intersecting (which
    /// may indicate a short circuit). Uses a larger tolerance than intersection
    /// tests to account for grid quantization effects.
    fn shares_endpoint(&self, other: &Segment) -> bool {
        let dist_sq = |a: Point<f64>, b: Point<f64>| (a.x - b.x).powi(2) + (a.y - b.y).powi(2);
        let tol_sq = CONNECTIVITY_TOLERANCE * CONNECTIVITY_TOLERANCE;

        dist_sq(self.p1, other.p1) < tol_sq
            || dist_sq(self.p1, other.p2) < tol_sq
            || dist_sq(self.p2, other.p1) < tol_sq
            || dist_sq(self.p2, other.p2) < tol_sq
    }
}

/// Computes the orientation of three points (collinear test).
///
/// Returns 0 if points are collinear, 1 if clockwise, 2 if counterclockwise.
/// Used in segment intersection tests to determine if line segments cross.
/// Implements the cross product test with tolerance for numerical stability.
fn orientation(p: Point<f64>, q: Point<f64>, r: Point<f64>) -> i32 {
    let val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if val.abs() < CHECK_TOLERANCE {
        return 0;
    }
    if val > 0.0 { 1 } else { 2 }
}

/// Key for spatial binning of segments for efficient intersection testing.
///
/// Groups segments by layer and spatial bin to reduce the number of
/// intersection tests needed. Segments in different bins or layers
/// cannot intersect, allowing early pruning of comparisons.
#[derive(Hash, Eq, PartialEq, PartialOrd, Ord, Clone, Copy, Debug)]
struct BinKey {
    layer: u8,
    bx: i32,
    by: i32,
}

/// Checks for short circuits and illegal routing loops.
///
/// Uses spatial binning to group segments by location, then tests segments
/// within the same bin for intersections. Detects shorts (intersections
/// between different nets) and self-loops (intersections within the same
/// net that don't share endpoints and aren't collinear). Returns an error
/// if any violations are found.
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
                    let msg = format!(
                        "SHORT: '{}' vs '{}' on Layer {}\n  seg1: ({:.3},{:.3})->({:.3},{:.3})\n  seg2: ({:.3},{:.3})->({:.3},{:.3})",
                        n1, n2, s1.layer,
                        s1.p1.x, s1.p1.y, s1.p2.x, s1.p2.y,
                        s2.p1.x, s2.p1.y, s2.p2.x, s2.p2.y
                    );

                    if !error_found.swap(true, Ordering::Relaxed) {
                        *error_msg.lock().unwrap() = msg;
                    }
                    return;
                } else if !s1.shares_endpoint(s2) {
                    let is_via1 = (s1.p1.x - s1.p2.x).abs() < 1e-6 && (s1.p1.y - s1.p2.y).abs() < 1e-6;
                    let is_via2 = (s2.p1.x - s2.p2.x).abs() < 1e-6 && (s2.p1.y - s2.p2.y).abs() < 1e-6;

                    if is_via1 || is_via2 {
                        continue;
                    }

                    let is_collinear = {
                        let dx1 = s1.p2.x - s1.p1.x;
                        let dy1 = s1.p2.y - s1.p1.y;
                        let dx2 = s2.p2.x - s2.p1.x;
                        let dy2 = s2.p2.y - s2.p1.y;
                        (dx1 * dy2 - dy1 * dx2).abs() < CHECK_TOLERANCE * 100.0
                    };

                    if is_collinear {
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
    });

    if error_found.load(Ordering::Relaxed) {
        Err(error_msg.lock().unwrap().clone())
    } else {
        Ok(())
    }
}

/// Checks for open nets (disconnected pins).
///
/// For each net, builds a connectivity graph of segments and verifies that
/// all pins are reachable from each other. Uses breadth-first search to
/// traverse the segment graph starting from the first pin. Detects both
/// completely unrouted nets and nets with disconnected components. Returns
/// an error if any opens are found.
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
                if seg.layer <= 1
                    && point_to_segment_dist(pin_pos, seg.p1, seg.p2) < CONNECTIVITY_TOLERANCE
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

/// Checks if two segments overlap when projected to 2D (ignoring layer).
///
/// Used to detect via connections where segments on adjacent layers overlap
/// at the same X,Y coordinates. This allows vias to connect segments across
/// layers even if they don't share exact endpoints.
fn segments_overlap_2d(s1: &Segment, s2: &Segment) -> bool {
    let mut s1_2d = *s1;
    s1_2d.layer = 0;
    let mut s2_2d = *s2;
    s2_2d.layer = 0;
    s1_2d.intersects(&s2_2d) || s1_2d.shares_endpoint(&s2_2d)
}

/// Computes the distance from a point to a line segment.
///
/// Projects the point onto the line containing the segment, clamps to the
/// segment endpoints, and returns the Euclidean distance. Used to check
/// if pins are close enough to wire segments to be considered connected.
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
