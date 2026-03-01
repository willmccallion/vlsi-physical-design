use crate::algo::astar::{AStar, GuideOracle, NoGuide};
use crate::algo::pattern;
use crate::grid::GCellGrid;
use crate::grid::RoutingGrid;
use crate::utils::conversion::GridConverter;
use eda_common::db::core::{NetData, NetlistDB, RouteSegment};
use eda_common::geom::coord::GridCoord;
use eda_common::geom::point::Point;
use eda_common::util::config::DetailedRoutingConfig;
use std::collections::{HashMap, HashSet};

/// Routes a single net using A* pathfinding with guide constraints.
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
    let margin_multiplier = 1.0 + (ripup_count as f64 * 0.15);

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
    let hpwl_expansions = (net_hpwl * 50).clamp(5_000, 500_000);
    let max_expansions = hpwl_expansions
        .saturating_add(ripup_count.saturating_mul(30_000))
        .min(500_000);

    for i in 1..sorted_indices.len() {
        let next_pin_idx = sorted_indices[i];
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
            tree_nodes.extend_from_slice(&path);
            paths.push(path);
            continue;
        }

        let path_opt = solver.find_path(
            grid, db, &tree_nodes, target, penalty,
            config.astar_heuristic_weight, config.astar_window_margin_base,
            margin_multiplier, oracle, &pin_coords, max_expansions, strict_mode,
        );
        let path_opt = if path_opt.is_some() {
            path_opt
        } else {
            solver.find_path(
                grid, db, &tree_nodes, target, penalty,
                config.astar_heuristic_weight, config.astar_window_margin_max,
                1.0, &NoGuide, &pin_coords, max_expansions * 2, false,
            )
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

/// Track info for a specific layer.
#[derive(Clone, Debug)]
pub struct LayerTrackInfo {
    pub start: f64,
    pub pitch: f64,
}

fn snap_to_track(coord: f64, ti: &LayerTrackInfo) -> f64 {
    if ti.pitch < 0.001 { return coord; }
    let idx = ((coord - ti.start) / ti.pitch).round();
    ti.start + idx * ti.pitch
}

/// Converts grid path topology to physical routing segments.
///
/// Uses gcell centers for all internal node positions to guarantee
/// connectivity at via transitions and junctions. Pin access uses
/// short M1 wire + via stack from pin to the gcell center.
pub fn generate_segments_from_topology(
    topology: &[Vec<GridCoord>],
    pin_locations: &HashMap<(u32, u32, u8), Vec<Point<f64>>>,
    _track_info: &[LayerTrackInfo],
    gcell_w: f64,
    gcell_h: f64,
    origin_x: f64,
    origin_y: f64,
    _edge_slots: &HashMap<(u32, u32, u8, bool), u16>,
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
