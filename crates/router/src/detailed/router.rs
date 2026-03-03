use crate::algo::astar::{AStar, GuideOracle, NoGuide};
use crate::algo::pattern;
use crate::grid::GCellGrid;
use crate::grid::RoutingGrid;
use crate::utils::conversion::GridConverter;
use pare_common::db::core::{NetData, NetlistDB, RouteSegment};
use pare_common::geom::coord::GridCoord;
use pare_common::geom::point::Point;
use pare_common::util::config::DetailedRoutingConfig;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicUsize, Ordering};

use super::oracle::FastGuideOracle;

/// Threshold for dispatching to parallel routing.
pub const LARGE_NET_PIN_THRESHOLD: usize = 5_000;

/// Routes a single net using A* pathfinding with guide constraints.
#[allow(clippy::too_many_arguments)]
pub fn route_net_dr_pure<O: GuideOracle>(
    net: &NetData,
    grid: &GCellGrid,
    solver: &mut AStar,
    converter: &GridConverter,
    db: &NetlistDB,
    penalty: f64,
    oracle: &O,
    config: &DetailedRoutingConfig,
    ripup_count: u32,
    strict_mode: bool,
    stat_pattern_hits: &AtomicUsize,
    stat_astar_calls: &AtomicUsize,
    stat_astar_fallback: &AtomicUsize,
    stat_astar_expansions: &AtomicUsize,
    stat_max_expansions: &AtomicUsize,
) -> Option<Vec<Vec<GridCoord>>> {
    let pin_coords: Vec<GridCoord> = net
        .pins
        .iter()
        .map(|&pid| {
            let cell_id = db.pin_to_cell[pid.index()];
            let pos = db.get_pin_position(pid, &db.positions[cell_id.index()]);
            let die_w = db.die_area.width();
            let die_h = db.die_area.height();
            let is_io = pos.x <= 0.001
                || pos.x >= die_w - 0.001
                || pos.y <= 0.001
                || pos.y >= die_h - 0.001;
            let layer = if is_io { 2 } else { 1 };
            let safe_layer = layer.min(grid.layers() - 1);
            converter.to_grid(pos, safe_layer)
        })
        .collect();

    let mut pin_indices: Vec<usize> = (0..net.pins.len()).collect();
    let mut sorted_indices = Vec::with_capacity(net.pins.len());
    let mut current_idx = pin_indices.remove(0);
    sorted_indices.push(current_idx);

    while !pin_indices.is_empty() {
        let curr_pos = pin_coords[current_idx];
        let mut best_dist = u32::MAX;
        let mut best_k = 0;
        for (k, &idx) in pin_indices.iter().enumerate() {
            let target_pos = pin_coords[idx];
            let dist = (curr_pos.x as i32 - target_pos.x as i32).abs()
                + (curr_pos.y as i32 - target_pos.y as i32).abs();
            if (dist as u32) < best_dist {
                best_dist = dist as u32;
                best_k = k;
            }
        }
        current_idx = pin_indices.remove(best_k);
        sorted_indices.push(current_idx);
    }

    let start_idx = sorted_indices[0];
    let mut tree_nodes = vec![pin_coords[start_idx]];
    let mut paths = Vec::new();
    let margin_multiplier = 1.0 + (ripup_count as f64 * 0.25);

    let net_hpwl = {
        let mut min_x = u32::MAX;
        let mut max_x = 0u32;
        let mut min_y = u32::MAX;
        let mut max_y = 0u32;
        for &coord in &pin_coords {
            min_x = min_x.min(coord.x);
            max_x = max_x.max(coord.x);
            min_y = min_y.min(coord.y);
            max_y = max_y.max(coord.y);
        }
        (max_x.saturating_sub(min_x)) + (max_y.saturating_sub(min_y))
    };
    let hpwl_expansions = (net_hpwl * 50).clamp(5_000, 1_000_000);
    let max_expansions = hpwl_expansions
        .saturating_add(ripup_count.saturating_mul(50_000))
        .min(1_000_000);

    for &next_pin_idx in &sorted_indices[1..] {
        let target = pin_coords[next_pin_idx];
        let nearest_start = tree_nodes
            .iter()
            .min_by_key(|&&s| {
                (s.x as i32 - target.x as i32).unsigned_abs()
                    + (s.y as i32 - target.y as i32).unsigned_abs()
            })
            .copied()
            .unwrap();

        if let Some(path) = pattern::try_pattern_route(grid, nearest_start, target, db) {
            stat_pattern_hits.fetch_add(1, Ordering::Relaxed);
            tree_nodes.extend_from_slice(&path);
            paths.push(path);
            continue;
        }

        stat_astar_calls.fetch_add(1, Ordering::Relaxed);
        let path_opt = solver.find_path(
            grid, db, &tree_nodes, target, penalty,
            config.astar_heuristic_weight, config.astar_window_margin_base,
            margin_multiplier, oracle, &pin_coords, max_expansions, strict_mode,
        );
        stat_astar_expansions.fetch_add(solver.last_expansions as usize, Ordering::Relaxed);
        let _ = stat_max_expansions.fetch_max(solver.last_expansions as usize, Ordering::Relaxed);
        let path_opt = if path_opt.is_some() {
            path_opt
        } else {
            stat_astar_fallback.fetch_add(1, Ordering::Relaxed);
            let p = solver.find_path(
                grid, db, &tree_nodes, target, penalty,
                config.astar_heuristic_weight, config.astar_window_margin_max,
                margin_multiplier, &NoGuide, &pin_coords, max_expansions * 2, false,
            );
            stat_astar_expansions.fetch_add(solver.last_expansions as usize, Ordering::Relaxed);
            let _ = stat_max_expansions.fetch_max(solver.last_expansions as usize, Ordering::Relaxed);
            p
        };

        if let Some(path) = path_opt {
            tree_nodes.extend_from_slice(&path);
            paths.push(path);
        } else {
            return None;
        }
    }

    Some(paths)
}

// ---------------------------------------------------------------------------
// Parallel routing for large nets
// ---------------------------------------------------------------------------

/// A spatial cluster of pins for parallel routing.
struct PinCluster {
    /// Indices into the pin_coords array.
    pin_indices: Vec<usize>,
    /// Index into pin_coords for the cluster representative.
    representative_idx: usize,
    /// Bounding box (min_x, max_x, min_y, max_y).
    bbox: (u32, u32, u32, u32),
}

/// Spatially clusters pins into grid buckets for parallel routing.
///
/// Divides the pin bounding box into a grid of buckets. Each non-empty bucket
/// becomes a cluster. The representative is the pin closest to the bucket centroid.
fn cluster_pins(pin_coords: &[GridCoord], target_count: usize) -> Vec<PinCluster> {
    if pin_coords.is_empty() {
        return Vec::new();
    }

    // Compute pin bounding box
    let mut min_x = u32::MAX;
    let mut max_x = 0u32;
    let mut min_y = u32::MAX;
    let mut max_y = 0u32;
    for &c in pin_coords {
        min_x = min_x.min(c.x);
        max_x = max_x.max(c.x);
        min_y = min_y.min(c.y);
        max_y = max_y.max(c.y);
    }

    let width = (max_x - min_x + 1) as f64;
    let height = (max_y - min_y + 1) as f64;
    let area = width * height;
    let bucket_side = (area / target_count as f64).sqrt().ceil().max(1.0) as u32;

    // Assign each pin to a bucket
    let mut buckets: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, &c) in pin_coords.iter().enumerate() {
        let bx = (c.x - min_x) / bucket_side;
        let by = (c.y - min_y) / bucket_side;
        buckets.entry((bx, by)).or_default().push(i);
    }

    // Convert buckets to clusters
    let mut clusters = Vec::with_capacity(buckets.len());
    for ((bx, by), indices) in buckets {
        if indices.is_empty() {
            continue;
        }

        // Bucket centroid
        let cx = min_x as f64 + (bx as f64 + 0.5) * bucket_side as f64;
        let cy = min_y as f64 + (by as f64 + 0.5) * bucket_side as f64;

        // Find representative (pin nearest centroid)
        let rep = *indices
            .iter()
            .min_by_key(|&&i| {
                let dx = pin_coords[i].x as f64 - cx;
                let dy = pin_coords[i].y as f64 - cy;
                (dx * dx + dy * dy) as u64
            })
            .unwrap();

        // Compute cluster bbox
        let mut cmin_x = u32::MAX;
        let mut cmax_x = 0u32;
        let mut cmin_y = u32::MAX;
        let mut cmax_y = 0u32;
        for &i in &indices {
            cmin_x = cmin_x.min(pin_coords[i].x);
            cmax_x = cmax_x.max(pin_coords[i].x);
            cmin_y = cmin_y.min(pin_coords[i].y);
            cmax_y = cmax_y.max(pin_coords[i].y);
        }

        clusters.push(PinCluster {
            pin_indices: indices,
            representative_idx: rep,
            bbox: (cmin_x, cmax_x, cmin_y, cmax_y),
        });
    }

    clusters
}

/// Routes the skeleton: connects cluster representatives using the existing
/// incremental Steiner algorithm.
#[allow(clippy::too_many_arguments)]
fn route_skeleton<O: GuideOracle>(
    clusters: &[PinCluster],
    pin_coords: &[GridCoord],
    grid: &GCellGrid,
    solver: &mut AStar,
    db: &NetlistDB,
    penalty: f64,
    oracle: &O,
    config: &DetailedRoutingConfig,
    ripup_count: u32,
    max_expansions: u32,
    stat_pattern_hits: &AtomicUsize,
    stat_astar_calls: &AtomicUsize,
    stat_astar_fallback: &AtomicUsize,
    stat_astar_expansions: &AtomicUsize,
    stat_max_expansions: &AtomicUsize,
) -> Option<(Vec<Vec<GridCoord>>, Vec<GridCoord>)> {
    let reps: Vec<GridCoord> = clusters
        .iter()
        .map(|c| pin_coords[c.representative_idx])
        .collect();

    if reps.len() < 2 {
        return Some((Vec::new(), reps));
    }

    // Nearest-neighbor sort on representatives
    let mut remaining: Vec<usize> = (0..reps.len()).collect();
    let mut sorted = Vec::with_capacity(reps.len());
    let mut current = remaining.remove(0);
    sorted.push(current);

    while !remaining.is_empty() {
        let curr_pos = reps[current];
        let mut best_dist = u32::MAX;
        let mut best_k = 0;
        for (k, &idx) in remaining.iter().enumerate() {
            let dist = (curr_pos.x as i32 - reps[idx].x as i32).unsigned_abs()
                + (curr_pos.y as i32 - reps[idx].y as i32).unsigned_abs();
            if dist < best_dist {
                best_dist = dist;
                best_k = k;
            }
        }
        current = remaining.remove(best_k);
        sorted.push(current);
    }

    let margin_multiplier = 1.0 + (ripup_count as f64 * 0.25);
    let mut tree_nodes = vec![reps[sorted[0]]];
    let mut paths = Vec::new();

    for &rep_idx in &sorted[1..] {
        let target = reps[rep_idx];

        let nearest_start = tree_nodes
            .iter()
            .min_by_key(|&&s| {
                (s.x as i32 - target.x as i32).unsigned_abs()
                    + (s.y as i32 - target.y as i32).unsigned_abs()
            })
            .copied()
            .unwrap();

        if let Some(path) = pattern::try_pattern_route(grid, nearest_start, target, db) {
            stat_pattern_hits.fetch_add(1, Ordering::Relaxed);
            tree_nodes.extend_from_slice(&path);
            paths.push(path);
            continue;
        }

        stat_astar_calls.fetch_add(1, Ordering::Relaxed);
        let path_opt = solver.find_path(
            grid,
            db,
            &tree_nodes,
            target,
            penalty,
            config.astar_heuristic_weight,
            config.astar_window_margin_base,
            margin_multiplier,
            oracle,
            &reps,
            max_expansions,
            false,
        );
        stat_astar_expansions.fetch_add(solver.last_expansions as usize, Ordering::Relaxed);
        let _ = stat_max_expansions.fetch_max(solver.last_expansions as usize, Ordering::Relaxed);

        let path_opt = if path_opt.is_some() {
            path_opt
        } else {
            stat_astar_fallback.fetch_add(1, Ordering::Relaxed);
            let p = solver.find_path(
                grid,
                db,
                &tree_nodes,
                target,
                penalty,
                config.astar_heuristic_weight,
                config.astar_window_margin_max,
                margin_multiplier,
                &NoGuide,
                &reps,
                max_expansions * 2,
                false,
            );
            stat_astar_expansions.fetch_add(solver.last_expansions as usize, Ordering::Relaxed);
            let _ =
                stat_max_expansions.fetch_max(solver.last_expansions as usize, Ordering::Relaxed);
            p
        };

        if let Some(path) = path_opt {
            tree_nodes.extend_from_slice(&path);
            paths.push(path);
        } else {
            return None;
        }
    }

    Some((paths, tree_nodes))
}

/// Routes pins within a single cluster to nearby skeleton anchor nodes.
#[allow(clippy::too_many_arguments)]
fn route_cluster<O: GuideOracle>(
    cluster: &PinCluster,
    pin_coords: &[GridCoord],
    skeleton_tree_nodes: &[GridCoord],
    grid: &GCellGrid,
    solver: &mut AStar,
    db: &NetlistDB,
    penalty: f64,
    oracle: &O,
    config: &DetailedRoutingConfig,
    ripup_count: u32,
    max_expansions: u32,
    strict_mode: bool,
    stat_pattern_hits: &AtomicUsize,
    stat_astar_calls: &AtomicUsize,
    stat_astar_fallback: &AtomicUsize,
    stat_astar_expansions: &AtomicUsize,
    stat_max_expansions: &AtomicUsize,
) -> Option<Vec<Vec<GridCoord>>> {
    let (cmin_x, cmax_x, cmin_y, cmax_y) = cluster.bbox;
    let margin = 5u32;

    // Select skeleton anchors within the cluster bbox + margin
    let anchor_min_x = cmin_x.saturating_sub(margin);
    let anchor_max_x = cmax_x + margin;
    let anchor_min_y = cmin_y.saturating_sub(margin);
    let anchor_max_y = cmax_y + margin;

    let mut anchors: Vec<GridCoord> = skeleton_tree_nodes
        .iter()
        .filter(|&&n| {
            n.x >= anchor_min_x && n.x <= anchor_max_x && n.y >= anchor_min_y && n.y <= anchor_max_y
        })
        .copied()
        .collect();

    // Fallback: if no anchors in range, use nearest skeleton node
    if anchors.is_empty() {
        let rep = pin_coords[cluster.representative_idx];
        if let Some(&nearest) = skeleton_tree_nodes.iter().min_by_key(|&&n| {
            (n.x as i32 - rep.x as i32).unsigned_abs()
                + (n.y as i32 - rep.y as i32).unsigned_abs()
        }) {
            anchors.push(nearest);
        }
    }

    // Cap anchors to avoid excessive tree_nodes
    if anchors.len() > 100 {
        let rep = pin_coords[cluster.representative_idx];
        anchors.sort_by_key(|n| {
            (n.x as i32 - rep.x as i32).unsigned_abs()
                + (n.y as i32 - rep.y as i32).unsigned_abs()
        });
        anchors.truncate(100);
    }

    if anchors.is_empty() {
        return Some(Vec::new());
    }

    // Collect local pins (exclude the representative if it's already in skeleton)
    let local_pins: Vec<usize> = cluster.pin_indices.clone();
    if local_pins.is_empty() {
        return Some(Vec::new());
    }

    // Nearest-neighbor sort on local pins
    let local_coords: Vec<GridCoord> = local_pins.iter().map(|&i| pin_coords[i]).collect();
    let mut remaining: Vec<usize> = (0..local_coords.len()).collect();
    let mut sorted = Vec::with_capacity(local_coords.len());
    let mut current = remaining.remove(0);
    sorted.push(current);

    while !remaining.is_empty() {
        let curr_pos = local_coords[current];
        let mut best_dist = u32::MAX;
        let mut best_k = 0;
        for (k, &idx) in remaining.iter().enumerate() {
            let dist = (curr_pos.x as i32 - local_coords[idx].x as i32).unsigned_abs()
                + (curr_pos.y as i32 - local_coords[idx].y as i32).unsigned_abs();
            if dist < best_dist {
                best_dist = dist;
                best_k = k;
            }
        }
        current = remaining.remove(best_k);
        sorted.push(current);
    }

    // Seed tree with anchors
    let mut tree_nodes: Vec<GridCoord> = anchors;
    let mut paths = Vec::new();
    let margin_multiplier = 1.0 + (ripup_count as f64 * 0.25);

    for &local_idx in &sorted {
        let target = local_coords[local_idx];

        // Check if target is already in tree
        if tree_nodes.contains(&target) {
            continue;
        }

        let nearest_start = tree_nodes
            .iter()
            .min_by_key(|&&s| {
                (s.x as i32 - target.x as i32).unsigned_abs()
                    + (s.y as i32 - target.y as i32).unsigned_abs()
            })
            .copied()
            .unwrap();

        if let Some(path) = pattern::try_pattern_route(grid, nearest_start, target, db) {
            stat_pattern_hits.fetch_add(1, Ordering::Relaxed);
            tree_nodes.extend_from_slice(&path);
            paths.push(path);
            continue;
        }

        stat_astar_calls.fetch_add(1, Ordering::Relaxed);
        let path_opt = solver.find_path(
            grid,
            db,
            &tree_nodes,
            target,
            penalty,
            config.astar_heuristic_weight,
            config.astar_window_margin_base,
            margin_multiplier,
            oracle,
            &local_coords,
            max_expansions,
            strict_mode,
        );
        stat_astar_expansions.fetch_add(solver.last_expansions as usize, Ordering::Relaxed);
        let _ = stat_max_expansions.fetch_max(solver.last_expansions as usize, Ordering::Relaxed);

        let path_opt = if path_opt.is_some() {
            path_opt
        } else {
            stat_astar_fallback.fetch_add(1, Ordering::Relaxed);
            let p = solver.find_path(
                grid,
                db,
                &tree_nodes,
                target,
                penalty,
                config.astar_heuristic_weight,
                config.astar_window_margin_max,
                margin_multiplier,
                &NoGuide,
                &local_coords,
                max_expansions * 2,
                false,
            );
            stat_astar_expansions.fetch_add(solver.last_expansions as usize, Ordering::Relaxed);
            let _ =
                stat_max_expansions.fetch_max(solver.last_expansions as usize, Ordering::Relaxed);
            p
        };

        if let Some(path) = path_opt {
            tree_nodes.extend_from_slice(&path);
            paths.push(path);
        } else {
            return None;
        }
    }

    Some(paths)
}

/// Routes a large net in parallel using spatial clustering.
///
/// 1. Clusters pins spatially into ~(n/200) groups
/// 2. Routes cluster representatives sequentially (skeleton)
/// 3. Routes each cluster's pins to skeleton anchors in parallel
#[allow(clippy::too_many_arguments)]
pub fn route_net_parallel<O: GuideOracle>(
    net: &NetData,
    grid: &GCellGrid,
    solver: &mut AStar,
    converter: &GridConverter,
    db: &NetlistDB,
    penalty: f64,
    oracle: &O,
    config: &DetailedRoutingConfig,
    ripup_count: u32,
    strict_mode: bool,
    // Oracle constructor args for parallel workers:
    oracle_grid_w: u32,
    oracle_grid_h: u32,
    oracle_coarse_w: u32,
    oracle_coarse_h: u32,
    oracle_layers: u8,
    oracle_fine_converter: &GridConverter,
    oracle_coarse_converter: &GridConverter,
    net_guides: &HashSet<GridCoord>,
    net_id: usize,
    // Stats:
    stat_pattern_hits: &AtomicUsize,
    stat_astar_calls: &AtomicUsize,
    stat_astar_fallback: &AtomicUsize,
    stat_astar_expansions: &AtomicUsize,
    stat_max_expansions: &AtomicUsize,
) -> Option<Vec<Vec<GridCoord>>> {
    // Compute pin coordinates (same as route_net_dr_pure)
    let pin_coords: Vec<GridCoord> = net
        .pins
        .iter()
        .map(|&pid| {
            let cell_id = db.pin_to_cell[pid.index()];
            let pos = db.get_pin_position(pid, &db.positions[cell_id.index()]);
            let die_w = db.die_area.width();
            let die_h = db.die_area.height();
            let is_io = pos.x <= 0.001
                || pos.x >= die_w - 0.001
                || pos.y <= 0.001
                || pos.y >= die_h - 0.001;
            let layer = if is_io { 2 } else { 1 };
            let safe_layer = layer.min(grid.layers() - 1);
            converter.to_grid(pos, safe_layer)
        })
        .collect();

    // Phase 1: Cluster
    let num_pins = pin_coords.len();
    let target_count = (num_pins / 200).clamp(50, num_pins / 10);
    let clusters = cluster_pins(&pin_coords, target_count);

    // Compute HPWL for max_expansions
    let net_hpwl = {
        let mut min_x = u32::MAX;
        let mut max_x = 0u32;
        let mut min_y = u32::MAX;
        let mut max_y = 0u32;
        for &coord in &pin_coords {
            min_x = min_x.min(coord.x);
            max_x = max_x.max(coord.x);
            min_y = min_y.min(coord.y);
            max_y = max_y.max(coord.y);
        }
        (max_x.saturating_sub(min_x)) + (max_y.saturating_sub(min_y))
    };
    let hpwl_expansions = (net_hpwl * 50).clamp(5_000, 1_000_000);
    let max_expansions = hpwl_expansions
        .saturating_add(ripup_count.saturating_mul(50_000))
        .min(1_000_000);

    // Phase 2: Skeleton (sequential)
    let (skeleton_paths, skeleton_tree_nodes) = route_skeleton(
        &clusters,
        &pin_coords,
        grid,
        solver,
        db,
        penalty,
        oracle,
        config,
        ripup_count,
        max_expansions,
        stat_pattern_hits,
        stat_astar_calls,
        stat_astar_fallback,
        stat_astar_expansions,
        stat_max_expansions,
    )?;

    // Phase 3: Parallel cluster routing
    let cluster_results: Vec<Option<Vec<Vec<GridCoord>>>> = clusters
        .par_iter()
        .map_with(
            (
                AStar::new(),
                FastGuideOracle::new(
                    oracle_grid_w,
                    oracle_grid_h,
                    oracle_coarse_w,
                    oracle_coarse_h,
                    oracle_layers,
                    oracle_fine_converter,
                    oracle_coarse_converter,
                ),
            ),
            |(local_solver, local_oracle), cluster| {
                local_oracle.prepare(net_id, net_guides);
                route_cluster(
                    cluster,
                    &pin_coords,
                    &skeleton_tree_nodes,
                    grid,
                    local_solver,
                    db,
                    penalty,
                    local_oracle,
                    config,
                    ripup_count,
                    max_expansions,
                    strict_mode,
                    stat_pattern_hits,
                    stat_astar_calls,
                    stat_astar_fallback,
                    stat_astar_expansions,
                    stat_max_expansions,
                )
            },
        )
        .collect();
    // Phase 4: Merge
    let mut all_paths = skeleton_paths;
    for cluster_result in cluster_results {
        let cluster_paths = cluster_result?;
        all_paths.extend(cluster_paths);
    }

    Some(all_paths)
}

/// Converts grid path topology to physical routing segments.
///
/// Uses gcell centers for all internal node positions to guarantee
/// connectivity at via transitions and junctions. Pin access uses
/// short M1 wire + via stack from pin to the gcell center.
#[allow(clippy::too_many_arguments)]
pub fn generate_segments_from_topology(
    topology: &[Vec<GridCoord>],
    pin_locations: &HashMap<(u32, u32, u8), Vec<Point<f64>>>,
    gcell_w: f64,
    gcell_h: f64,
    origin_x: f64,
    origin_y: f64,
) -> Vec<RouteSegment> {
    let mut segments = Vec::new();

    let gcell_center = |node: GridCoord| -> Point<f64> {
        Point::new(
            origin_x + (node.x as f64 + 0.5) * gcell_w,
            origin_y + (node.y as f64 + 0.5) * gcell_h,
        )
    };

    // Collect all unique nodes from topology (including single-node paths
    // for same-gcell pin connections)
    let mut nodes = HashSet::new();
    for path in topology {
        for &coord in path {
            nodes.insert(coord);
        }
    }
    // Also add pin gcell nodes so that pin access is generated even
    // when the topology has no edges (e.g., all pins in the same gcell)
    for &(x, y, z) in pin_locations.keys() {
        nodes.insert(GridCoord::new(x, y, z));
    }
    if nodes.is_empty() { return segments; }

    // For each edge on the same layer, emit a wire segment
    // between the gcell centers of the two endpoints.
    let mut emitted_edges: HashSet<(GridCoord, GridCoord)> = HashSet::new();
    for path in topology {
        for i in 0..path.len().saturating_sub(1) {
            let u = path[i];
            let v = path[i + 1];
            if u == v { continue; }
            let edge = if (u.x, u.y, u.z) < (v.x, v.y, v.z) { (u, v) } else { (v, u) };
            if !emitted_edges.insert(edge) { continue; }

            if u.z == v.z {
                // Same-layer wire segment
                let p1 = gcell_center(u);
                let p2 = gcell_center(v);
                segments.push(RouteSegment { layer: u.z, p1, p2 });
            } else {
                // Via: zero-length segment on the lower layer
                let lo = u.z.min(v.z);
                let p = gcell_center(u); // u and v have same (x,y) for via
                segments.push(RouteSegment { layer: lo, p1: p, p2: p });
            }
        }
    }

    // Pin access: connect each physical pin position to gcell center via M1.
    // Use two Manhattan segments (L-shape) to avoid diagonal crossings.
    // pin_locations stores Vec<Point> per gcell to handle multiple pins
    // mapping to the same gcell.
    for (&(x, y, z), positions) in pin_locations {
        let grid_coord = GridCoord::new(x, y, z);
        if !nodes.contains(&grid_coord) { continue; }

        let center = gcell_center(grid_coord);

        // Generate access for every pin at this gcell
        for &exact_pos in positions {
            let dx = (center.x - exact_pos.x).abs();
            let dy = (center.y - exact_pos.y).abs();
            if dx > 1e-6 && dy > 1e-6 {
                // L-shape: horizontal from pin, then vertical to center
                let mid = Point::new(center.x, exact_pos.y);
                segments.push(RouteSegment { layer: 0, p1: exact_pos, p2: mid });
                segments.push(RouteSegment { layer: 0, p1: mid, p2: center });
            } else if dx > 1e-6 || dy > 1e-6 {
                // Straight horizontal or vertical
                segments.push(RouteSegment { layer: 0, p1: exact_pos, p2: center });
            }
        }

        // Via stack at gcell center from M1 (layer 0) to wire layer z
        // (only once per gcell, not per pin)
        for l in 0..z {
            segments.push(RouteSegment {
                layer: l,
                p1: center,
                p2: center,
            });
        }
    }

    segments
}
