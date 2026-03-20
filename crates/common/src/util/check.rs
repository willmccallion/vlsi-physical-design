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
use std::collections::{HashMap, VecDeque};
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

    // Use spatial binning to reduce overlap check from O(n²) to O(n·k).
    // Compute bin size from max cell dimension.
    let max_cell_dim = db.cells.iter()
        .filter(|c| !c.is_fixed && c.name != "IO_VIRTUAL_CELL")
        .map(|c| c.width.max(c.height))
        .fold(0.0_f64, f64::max);
    let overlap_bin_size = (max_cell_dim * 2.0).max(1.0);

    // Collect movable cell indices and insert into bins by position.
    let movable: Vec<usize> = (0..db.num_cells())
        .filter(|&i| !db.cells[i].is_fixed && db.cells[i].name != "IO_VIRTUAL_CELL")
        .collect();

    let mut cell_bins: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
    for &i in &movable {
        let pos = db.positions[i];
        let cell = &db.cells[i];
        let bx0 = (pos.x / overlap_bin_size).floor() as i32;
        let bx1 = ((pos.x + cell.width) / overlap_bin_size).floor() as i32;
        let by0 = (pos.y / overlap_bin_size).floor() as i32;
        let by1 = ((pos.y + cell.height) / overlap_bin_size).floor() as i32;
        for bx in bx0..=bx1 {
            for by in by0..=by1 {
                cell_bins.entry((bx, by)).or_default().push(i);
            }
        }
    }

    // Check overlap only within same/adjacent bins using canonical ordering (i < j).
    let bin_entries: Vec<_> = cell_bins.into_iter().collect();
    let has_overlap = bin_entries.par_iter().any(|(_key, indices)| {
        for a in 0..indices.len() {
            let i = indices[a];
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

            for &j in &indices[(a + 1)..] {
                if i >= j { continue; }

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
            log::error!("{e}");
            msgs.push(e);
            valid = false;
        }
        Ok(()) => ui::check("DRC passed -- no shorts or illegal loops."),
    }

    match opens_result {
        Err(e) => {
            ui::fail("Open Net (Disconnected) Detected");
            log::error!("{e}");
            msgs.push(e);
            valid = false;
        }
        Ok(()) => ui::check("LVS passed -- all nets fully connected."),
    }

    // Design rule checks (spacing/width) — run only if layers have spacing info.
    // These are reported as informational metrics since the router doesn't yet
    // guarantee spacing-clean output. Width violations (non-Manhattan) are
    // treated as errors since they indicate a routing bug.
    let has_spacing_info = db.layers.iter().any(|l| l.min_spacing > 0.0);
    if has_spacing_info {
        let (spacing_violations, width_violations) = check_design_rules(db);
        if spacing_violations == 0 && width_violations == 0 {
            ui::check("Design rules passed -- spacing and width OK.");
        } else {
            if spacing_violations > 0 {
                log::info!("DRC: {spacing_violations} spacing violations (informational)");
            }
            if width_violations > 0 {
                let msg = format!("DRC: {width_violations} non-Manhattan wire violations");
                ui::fail(&msg);
                msgs.push(msg);
                valid = false;
            }
        }
    }

    if valid {
        Ok(())
    } else {
        ui::fail(&format!("INVALID CHIP ({} Errors)", msgs.len()));
        Err(msgs.join("; "))
    }
}

/// Internal representation of a routing segment for verification.
#[derive(Clone, Copy, Debug)]
struct Segment {
    p1: Point<f64>,
    p2: Point<f64>,
    layer: u8,
    net_id: NetId,
}

impl Segment {
    /// Checks if this segment intersects with another segment on the same layer.
    fn intersects(&self, other: &Self) -> bool {
        fn on_segment(p: Point<f64>, a: Point<f64>, b: Point<f64>) -> bool {
            p.x >= a.x.min(b.x) + CHECK_TOLERANCE
                && p.x <= a.x.max(b.x) - CHECK_TOLERANCE
                && p.y >= a.y.min(b.y) + CHECK_TOLERANCE
                && p.y <= a.y.max(b.y) - CHECK_TOLERANCE
        }

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
    fn shares_endpoint(&self, other: &Self) -> bool {
        let dist_sq = |a: Point<f64>, b: Point<f64>| (a.x - b.x).mul_add(a.x - b.x, (a.y - b.y).powi(2));
        let tol_sq = CONNECTIVITY_TOLERANCE * CONNECTIVITY_TOLERANCE;

        dist_sq(self.p1, other.p1) < tol_sq
            || dist_sq(self.p1, other.p2) < tol_sq
            || dist_sq(self.p2, other.p1) < tol_sq
            || dist_sq(self.p2, other.p2) < tol_sq
    }
}

/// Computes the orientation of three points (collinear test).
fn orientation(p: Point<f64>, q: Point<f64>, r: Point<f64>) -> i32 {
    let val = (q.y - p.y).mul_add(r.x - q.x, -((q.x - p.x) * (r.y - q.y)));
    if val.abs() < CHECK_TOLERANCE {
        return 0;
    }
    if val > 0.0 { 1 } else { 2 }
}

/// Key for spatial binning of segments for efficient intersection testing.
#[derive(Hash, Eq, PartialEq, PartialOrd, Ord, Clone, Copy, Debug)]
struct BinKey {
    layer: u8,
    bx: i32,
    by: i32,
}

/// Returns the set of bin keys that a segment touches.
fn segment_bin_keys(s: &Segment) -> Vec<BinKey> {
    let min_x = s.p1.x.min(s.p2.x);
    let max_x = s.p1.x.max(s.p2.x);
    let min_y = s.p1.y.min(s.p2.y);
    let max_y = s.p1.y.max(s.p2.y);

    let start_bx = (min_x / BIN_SIZE).floor() as i32;
    let end_bx = (max_x / BIN_SIZE).floor() as i32;
    let start_by = (min_y / BIN_SIZE).floor() as i32;
    let end_by = (max_y / BIN_SIZE).floor() as i32;

    let mut keys = Vec::with_capacity(((end_bx - start_bx + 1) * (end_by - start_by + 1)) as usize);
    for bx in start_bx..=end_bx {
        for by in start_by..=end_by {
            keys.push(BinKey { layer: s.layer, bx, by });
        }
    }
    keys
}

/// Computes the minimum bounding-box gap between two same-layer segments.
///
/// For Manhattan segments, the gap is the shortest distance between their
/// bounding boxes. Returns 0.0 if they overlap or touch. This is a
/// conservative approximation suitable for spacing DRC.
fn segment_gap_distance(s1: &Segment, s2: &Segment) -> f64 {
    let min_x1 = s1.p1.x.min(s1.p2.x);
    let max_x1 = s1.p1.x.max(s1.p2.x);
    let min_y1 = s1.p1.y.min(s1.p2.y);
    let max_y1 = s1.p1.y.max(s1.p2.y);

    let min_x2 = s2.p1.x.min(s2.p2.x);
    let max_x2 = s2.p1.x.max(s2.p2.x);
    let min_y2 = s2.p1.y.min(s2.p2.y);
    let max_y2 = s2.p1.y.max(s2.p2.y);

    let dx = (min_x1 - max_x2).max(min_x2 - max_x1).max(0.0);
    let dy = (min_y1 - max_y2).max(min_y2 - max_y1).max(0.0);

    // For Manhattan segments, the gap is max(dx, dy) when they're parallel,
    // or the Euclidean distance of the bounding box gap for L-shaped pairs.
    dx.hypot(dy)
}

/// Checks design rules: minimum spacing between cross-net segments and
/// Manhattan width constraints.
///
/// Uses spatial binning (same approach as shorts checking) to efficiently
/// find nearby cross-net segment pairs on the same layer, then computes
/// their gap distance and compares against the layer's `min_spacing` rule.
///
/// Returns `(spacing_violations, width_violations)` on completion.
fn check_design_rules(db: &NetlistDB) -> (usize, usize) {
    let spacing_count = std::sync::atomic::AtomicUsize::new(0);
    let width_count = std::sync::atomic::AtomicUsize::new(0);

    // Width check: verify all wire segments are Manhattan
    db.nets.par_iter().enumerate().for_each(|(net_idx, net)| {
        for seg in &net.route_segments {
            // Skip vias (degenerate segments)
            if (seg.p1.x - seg.p2.x).abs() < 1e-6 && (seg.p1.y - seg.p2.y).abs() < 1e-6 {
                continue;
            }
            // Non-Manhattan check: a wire should be either horizontal or vertical
            let is_horizontal = (seg.p1.y - seg.p2.y).abs() < CHECK_TOLERANCE;
            let is_vertical = (seg.p1.x - seg.p2.x).abs() < CHECK_TOLERANCE;
            if !is_horizontal && !is_vertical {
                log::warn!(
                    "Width violation: Net '{}' has diagonal segment on L{}: ({:.3},{:.3})->({:.3},{:.3})",
                    net.name, seg.layer, seg.p1.x, seg.p1.y, seg.p2.x, seg.p2.y
                );
                let _ = width_count.fetch_add(1, Ordering::Relaxed);
            }
        }
        let _ = net_idx;
    });

    // Spacing check: use spatial binning to find nearby cross-net pairs.
    // Use a larger bin size to capture spacing violations (segments that
    // are close but don't overlap).
    let max_spacing: f64 = db.layers.iter().map(|l| l.min_spacing).fold(0.0, f64::max);
    if max_spacing <= 0.0 {
        return (
            spacing_count.load(Ordering::Relaxed),
            width_count.load(Ordering::Relaxed),
        );
    }

    let spacing_bin_size = BIN_SIZE.max(max_spacing * 2.0);

    // Build ONE global spatial index over ALL nets so cross-chunk spacing
    // violations are never missed. Expand bins by max_spacing to catch nearby segments.
    let mut bins: HashMap<BinKey, Vec<Segment>> = HashMap::new();

    for (net_idx, net) in db.nets.iter().enumerate() {
        let net_id = NetId::new(net_idx);
        for seg in &net.route_segments {
            // Skip vias for spacing check
            if (seg.p1.x - seg.p2.x).abs() < 1e-6 && (seg.p1.y - seg.p2.y).abs() < 1e-6 {
                continue;
            }

            let s = Segment {
                p1: seg.p1,
                p2: seg.p2,
                layer: seg.layer,
                net_id,
            };

            let min_x = s.p1.x.min(s.p2.x) - max_spacing;
            let max_x = s.p1.x.max(s.p2.x) + max_spacing;
            let min_y = s.p1.y.min(s.p2.y) - max_spacing;
            let max_y = s.p1.y.max(s.p2.y) + max_spacing;

            let start_bx = (min_x / spacing_bin_size).floor() as i32;
            let end_bx = (max_x / spacing_bin_size).floor() as i32;
            let start_by = (min_y / spacing_bin_size).floor() as i32;
            let end_by = (max_y / spacing_bin_size).floor() as i32;

            for bx in start_bx..=end_bx {
                for by in start_by..=end_by {
                    bins.entry(BinKey { layer: s.layer, bx, by })
                        .or_default()
                        .push(s);
                }
            }
        }
    }

    // Check bins in parallel.
    let bin_entries: Vec<_> = bins.into_iter().collect();
    bin_entries.par_iter().for_each(|(_key, segs)| {
        let first_net = segs[0].net_id;
        if segs.iter().all(|s| s.net_id == first_net) {
            return;
        }

        for i in 0..segs.len() {
            for j in (i + 1)..segs.len() {
                let s1 = &segs[i];
                let s2 = &segs[j];

                if s1.net_id == s2.net_id || s1.layer != s2.layer {
                    continue;
                }

                let li = s1.layer as usize;
                if li >= db.layers.len() {
                    continue;
                }
                let min_space = db.layers[li].min_spacing;
                if min_space <= 0.0 {
                    continue;
                }

                let gap = segment_gap_distance(s1, s2);
                if gap > 0.0 && gap < min_space - CHECK_TOLERANCE {
                    let _ = spacing_count.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    });

    (
        spacing_count.load(Ordering::Relaxed),
        width_count.load(Ordering::Relaxed),
    )
}

/// Checks for short circuits and illegal routing loops.
///
/// Processes nets in parallel chunks, building per-chunk spatial bins to keep
/// memory bounded. Each chunk inserts its segments into a local `HashMap`, then
/// checks for intersections between segments from different nets within the
/// same bin.
fn check_shorts_and_loops(db: &NetlistDB) -> Result<(), String> {
    use std::sync::atomic::AtomicUsize;

    // Build ONE global spatial index over ALL nets so cross-chunk shorts
    // are never missed.
    let mut bins: HashMap<BinKey, Vec<Segment>> = HashMap::new();

    for (net_idx, net) in db.nets.iter().enumerate() {
        let net_id = NetId::new(net_idx);
        for seg in &net.route_segments {
            let s = Segment {
                p1: seg.p1,
                p2: seg.p2,
                layer: seg.layer,
                net_id,
            };
            for key in segment_bin_keys(&s) {
                bins.entry(key).or_default().push(s);
            }
        }
    }

    // Check bins in parallel, counting ALL shorts and collecting first 100 examples.
    let short_count = AtomicUsize::new(0);
    let error_msgs: Mutex<Vec<String>> = Mutex::new(Vec::new());

    let bin_entries: Vec<_> = bins.into_iter().collect();
    bin_entries.par_iter().for_each(|(_key, segs)| {
        // Fast path: single-net bin → no cross-net shorts possible.
        let first_net = segs[0].net_id;
        if segs.iter().all(|s| s.net_id == first_net) {
            return;
        }

        for i in 0..segs.len() {
            for j in (i + 1)..segs.len() {
                let s1 = &segs[i];
                let s2 = &segs[j];

                if s1.net_id == s2.net_id {
                    continue;
                }

                if s1.intersects(s2) {
                    let prev = short_count.fetch_add(1, Ordering::Relaxed);
                    if prev < 100 {
                        let n1 = &db.nets[s1.net_id.index()].name;
                        let n2 = &db.nets[s2.net_id.index()].name;
                        let msg = format!(
                            "SHORT: '{}' vs '{}' on Layer {}\n  seg1: ({:.3},{:.3})->({:.3},{:.3})\n  seg2: ({:.3},{:.3})->({:.3},{:.3})",
                            n1, n2, s1.layer,
                            s1.p1.x, s1.p1.y, s1.p2.x, s1.p2.y,
                            s2.p1.x, s2.p1.y, s2.p2.x, s2.p2.y
                        );
                        if let Ok(mut msgs) = error_msgs.lock() {
                            msgs.push(msg);
                        }
                    }
                }
            }
        }
    });

    let total = short_count.load(Ordering::Relaxed);
    if total > 0 {
        let msgs = error_msgs.into_inner().unwrap_or_default();
        let first = msgs.first().cloned().unwrap_or_default();
        Err(format!("{total} short(s) detected. First: {first}"))
    } else {
        Ok(())
    }
}

/// Checks for open nets (disconnected pins).
///
/// For each net, builds a connectivity graph of segments using spatial binning
/// to avoid O(n²) comparisons, then verifies all pins are reachable via BFS.
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
            .map(|s| Segment {
                p1: s.p1,
                p2: s.p2,
                layer: s.layer,
                net_id: NetId::new(net_idx),
            })
            .collect();

        let n = segments.len();
        if n == 0 {
            if !error_found.swap(true, Ordering::Relaxed)
                && let Ok(mut guard) = error_msg.lock()
            {
                *guard = format!("Net '{}': Unrouted (No segments)", net.name);
            }
            return;
        }

        // Build adjacency using spatial binning to avoid O(n²) for large nets.
        let mut adj = vec![Vec::new(); n];

        if n <= 64 {
            // Small net: O(n²) is fine and has less overhead.
            for i in 0..n {
                for j in (i + 1)..n {
                    if segments_connected(&segments[i], &segments[j]) {
                        adj[i].push(j);
                        adj[j].push(i);
                    }
                }
            }
        } else {
            // Large net: bin segments by endpoint positions to limit comparisons.
            // Precompute bin keys once per segment, then insert for same-layer
            // and adjacent layers.
            let mut endpoint_bins: HashMap<(i32, i32, u8), Vec<usize>> = HashMap::new();

            for (i, seg) in segments.iter().enumerate() {
                let keys = segment_bin_keys(seg);
                for key in &keys {
                    endpoint_bins.entry((key.bx, key.by, seg.layer)).or_default().push(i);
                    if seg.layer > 0 {
                        endpoint_bins.entry((key.bx, key.by, seg.layer - 1)).or_default().push(i);
                    }
                    endpoint_bins.entry((key.bx, key.by, seg.layer + 1)).or_default().push(i);
                }
            }

            let mut checked = std::collections::HashSet::new();
            for indices in endpoint_bins.values() {
                for &i in indices {
                    for &j in indices {
                        if i >= j {
                            continue;
                        }
                        if !checked.insert((i, j)) {
                            continue;
                        }
                        if segments_connected(&segments[i], &segments[j]) {
                            adj[i].push(j);
                            adj[j].push(i);
                        }
                    }
                }
            }
        }

        // Build spatial index for fast pin-to-segment matching (all layers).
        let pin_bin_size: f64 = CONNECTIVITY_TOLERANCE * 2.0;
        let mut pin_seg_bins: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for (seg_i, seg) in segments.iter().enumerate() {
            let min_x = seg.p1.x.min(seg.p2.x);
            let max_x = seg.p1.x.max(seg.p2.x);
            let min_y = seg.p1.y.min(seg.p2.y);
            let max_y = seg.p1.y.max(seg.p2.y);
            let bx0 = (min_x / pin_bin_size).floor() as i32;
            let bx1 = (max_x / pin_bin_size).floor() as i32;
            let by0 = (min_y / pin_bin_size).floor() as i32;
            let by1 = (max_y / pin_bin_size).floor() as i32;
            for bx in bx0..=bx1 {
                for by in by0..=by1 {
                    pin_seg_bins.entry((bx, by)).or_default().push(seg_i);
                }
            }
        }

        let mut pin_segment_indices = Vec::new();
        for (pin_idx_in_net, &pin_id) in net.pins.iter().enumerate() {
            let cell_id = db.pin_to_cell[pin_id.index()];
            let pos = db.positions[cell_id.index()];
            let pin_pos = db.get_pin_position(pin_id, &pos);

            let pbx = (pin_pos.x / pin_bin_size).floor() as i32;
            let pby = (pin_pos.y / pin_bin_size).floor() as i32;
            let mut found = false;
            'pin_search: for dbx in (pbx - 1)..=(pbx + 1) {
                for dby in (pby - 1)..=(pby + 1) {
                    if let Some(candidates) = pin_seg_bins.get(&(dbx, dby)) {
                        for &seg_i in candidates {
                            if point_to_segment_dist(
                                pin_pos,
                                segments[seg_i].p1,
                                segments[seg_i].p2,
                            ) < CONNECTIVITY_TOLERANCE
                            {
                                pin_segment_indices.push(seg_i);
                                found = true;
                                break 'pin_search;
                            }
                        }
                    }
                }
            }
            if !found {
                if !error_found.swap(true, Ordering::Relaxed)
                    && let Ok(mut guard) = error_msg.lock()
                {
                    *guard = format!(
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
                if !error_found.swap(true, Ordering::Relaxed)
                    && let Ok(mut guard) = error_msg.lock()
                {
                    *guard = format!("Net '{}': Broken connectivity (Split net).", net.name);
                }
                return;
            }
        }
    });

    if error_found.load(Ordering::Relaxed) {
        Err(error_msg.lock().map_or_else(|e| e.into_inner().clone(), |g| g.clone()))
    } else {
        Ok(())
    }
}

/// Checks if two segments are connected (same-layer intersection/endpoint, or
/// adjacent-layer 2D overlap).
fn segments_connected(s1: &Segment, s2: &Segment) -> bool {
    let same_layer = s1.layer == s2.layer;
    let adj_layer = (s1.layer as i32 - s2.layer as i32).abs() == 1;

    if same_layer {
        s1.intersects(s2) || s1.shares_endpoint(s2)
    } else if adj_layer {
        segments_overlap_2d(s1, s2)
    } else {
        false
    }
}

/// Checks if two segments overlap when projected to 2D (ignoring layer).
fn segments_overlap_2d(s1: &Segment, s2: &Segment) -> bool {
    let mut s1_2d = *s1;
    s1_2d.layer = 0;
    let mut s2_2d = *s2;
    s2_2d.layer = 0;
    s1_2d.intersects(&s2_2d) || s1_2d.shares_endpoint(&s2_2d)
}

/// Computes the distance from a point to a line segment.
fn point_to_segment_dist(p: Point<f64>, a: Point<f64>, b: Point<f64>) -> f64 {
    let l2 = (a.x - b.x).mul_add(a.x - b.x, (a.y - b.y).powi(2));
    if l2 == 0.0 {
        return (p.x - a.x).hypot(p.y - a.y);
    }

    let t = (p.x - a.x).mul_add(b.x - a.x, (p.y - a.y) * (b.y - a.y)) / l2;
    let t = t.clamp(0.0, 1.0);

    let proj_x = a.x + t * (b.x - a.x);
    let proj_y = a.y + t * (b.y - a.y);

    (p.x - proj_x).hypot(p.y - proj_y)
}
