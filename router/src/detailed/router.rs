use crate::algo::astar::{AStar, GuideOracle, NoGuide};
use crate::grid::RoutingGrid;
use crate::grid::dense::DenseGrid;
use crate::utils::conversion::GridConverter;
use eda_common::db::core::{NetData, NetlistDB, RouteSegment};
use eda_common::geom::coord::GridCoord;
use eda_common::util::config::DetailedRoutingConfig;
use std::collections::{HashMap, HashSet};

/// Routes a single net using A* pathfinding with guide constraints.
///
/// Converts pin positions to grid coordinates, sorts pins by proximity to
/// build a minimum spanning tree topology, then routes paths sequentially
/// from the tree root to each remaining pin. Uses A* with guide constraints
/// and falls back to unconstrained routing if guide-constrained routing fails.
/// Returns the set of occupied grid coordinates and the path topology, or
/// None if routing fails.
pub fn route_net_dr_pure<O: GuideOracle>(
    net: &NetData,
    grid: &DenseGrid,
    solver: &mut AStar,
    converter: &GridConverter,
    db: &NetlistDB,
    penalty: f64,
    oracle: &O,
    config: &DetailedRoutingConfig,
    ripup_count: u32,
    strict_mode: bool,
) -> Option<(Vec<GridCoord>, Vec<Vec<GridCoord>>)> {
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
    let mut occupied_set = HashSet::new();
    occupied_set.insert(pin_coords[start_idx]);

    let margin_multiplier = 1.0 + (ripup_count as f64 * 0.15);
    let base_expansions = 20_000u32;
    let max_expansions = base_expansions
        .saturating_add(ripup_count.saturating_mul(15_000))
        .min(150_000);

    for i in 1..sorted_indices.len() {
        let next_pin_idx = sorted_indices[i];
        let target = pin_coords[next_pin_idx];

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
            &pin_coords,
            max_expansions,
            strict_mode,
        );

        let path_opt = if path_opt.is_some() {
            path_opt
        } else {
            solver.find_path(
                grid,
                db,
                &tree_nodes,
                target,
                penalty,
                config.astar_heuristic_weight,
                config.astar_window_margin_max,
                1.0,
                &NoGuide,
                &pin_coords,
                if ripup_count > 3 {
                    max_expansions * 2
                } else {
                    max_expansions
                },
                false,
            )
        };

        if let Some(path) = path_opt {
            tree_nodes.extend_from_slice(&path);
            for &node in &path {
                occupied_set.insert(node);
            }
            paths.push(path);
        } else {
            return None;
        }
    }

    let occupied_vec: Vec<GridCoord> = occupied_set.into_iter().collect();
    Some((occupied_vec, paths))
}

/// Converts grid path topology to physical routing segments.
///
/// Takes the grid coordinate paths and converts them to RouteSegment structures
/// in physical coordinates. Identifies stop points (pins, layer changes, direction
/// changes) and traces straight segments between them. Handles vias for layer
/// transitions and connects segments to exact pin positions when grid coordinates
/// don't align perfectly.
pub fn generate_segments_from_topology(
    topology: &[Vec<GridCoord>],
    pin_locations: &HashMap<(u32, u32, u8), eda_common::geom::point::Point<f64>>,
    converter: &GridConverter,
) -> Vec<RouteSegment> {
    let mut segments = Vec::new();
    let mut adj: HashMap<GridCoord, HashSet<GridCoord>> = HashMap::new();
    let mut nodes = HashSet::new();

    for path in topology {
        for i in 0..path.len().saturating_sub(1) {
            let u = path[i];
            let v = path[i + 1];
            if u != v {
                adj.entry(u).or_default().insert(v);
                adj.entry(v).or_default().insert(u);
                nodes.insert(u);
                nodes.insert(v);
            }
        }
    }

    let mut stop_points = HashSet::new();
    for &u in &nodes {
        if pin_locations.contains_key(&(u.x, u.y, u.z)) {
            stop_points.insert(u);
        }

        let neighbors = if let Some(n) = adj.get(&u) {
            n
        } else {
            continue;
        };

        if neighbors.len() != 2 {
            stop_points.insert(u);
        } else {
            let ns: Vec<&GridCoord> = neighbors.iter().collect();
            let n1 = ns[0];
            let n2 = ns[1];

            if n1.z != u.z || n2.z != u.z {
                stop_points.insert(u);
            } else if (n1.x != n2.x) && (n1.y != n2.y) {
                stop_points.insert(u);
            }
        }

        for &v in neighbors {
            if v.z != u.z {
                stop_points.insert(u);
            }
        }
    }

    let mut visited_edges = HashSet::new();
    for &start_node in &stop_points {
        if let Some(neighbors) = adj.get(&start_node) {
            for &next_node in neighbors {
                if next_node.z != start_node.z {
                    continue;
                }

                let edge_key = if start_node.x < next_node.x
                    || (start_node.x == next_node.x && start_node.y < next_node.y)
                {
                    (start_node, next_node)
                } else {
                    (next_node, start_node)
                };

                if visited_edges.contains(&edge_key) {
                    continue;
                }

                let mut curr = next_node;
                let mut prev = start_node;

                while !stop_points.contains(&curr) {
                    let n_neighbors = adj.get(&curr).unwrap();
                    let mut found_next = false;
                    for &n in n_neighbors {
                        if n != prev && n.z == curr.z {
                            prev = curr;
                            curr = n;
                            found_next = true;
                            break;
                        }
                    }
                    if !found_next {
                        break;
                    }
                }

                let mut w_prev = start_node;
                let mut w_curr = next_node;
                loop {
                    let key =
                        if w_prev.x < w_curr.x || (w_prev.x == w_curr.x && w_prev.y < w_curr.y) {
                            (w_prev, w_curr)
                        } else {
                            (w_curr, w_prev)
                        };
                    visited_edges.insert(key);

                    if w_curr == curr {
                        break;
                    }

                    let n_neighbors = adj.get(&w_curr).unwrap();
                    for &n in n_neighbors {
                        if n != w_prev && n.z == w_curr.z {
                            w_prev = w_curr;
                            w_curr = n;
                            break;
                        }
                    }
                }

                let p1 = converter.to_world(start_node);
                let p2 = converter.to_world(curr);
                segments.push(RouteSegment {
                    layer: start_node.z,
                    p1,
                    p2,
                });
            }
        }
    }

    for &u in &nodes {
        if let Some(neighbors) = adj.get(&u) {
            for &v in neighbors {
                if v.z > u.z {
                    let p = converter.to_world(u);
                    segments.push(RouteSegment {
                        layer: u.z,
                        p1: p,
                        p2: p,
                    });
                }
            }
        }
    }

    for (&(x, y, z), &exact_pos) in pin_locations {
        let grid_coord = GridCoord::new(x, y, z);
        if nodes.contains(&grid_coord) {
            let grid_pos = converter.to_world(grid_coord);
            if (grid_pos.x - exact_pos.x).abs() > 1e-6 || (grid_pos.y - exact_pos.y).abs() > 1e-6 {
                segments.push(RouteSegment {
                    layer: z,
                    p1: grid_pos,
                    p2: exact_pos,
                });
            }
            for l in 0..z {
                segments.push(RouteSegment {
                    layer: l,
                    p1: exact_pos,
                    p2: exact_pos,
                });
            }
        }
    }

    segments
}
