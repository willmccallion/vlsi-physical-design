use crate::algo::astar::{AStar, NoGuide};
use crate::grid::GCellGrid;
use crate::grid::RoutingGrid;
use crate::utils::conversion::GridConverter;
use pare_common::db::core::NetlistDB;
use pare_common::geom::coord::GridCoord;
use pare_common::util::config::GlobalRoutingConfig;
use pare_common::util::ui;
use rand::seq::SliceRandom;
use rand::thread_rng;
use rayon::prelude::*;
use std::collections::HashSet;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

/// Adds edge usage for a path on the grid.
fn add_path_to_grid(grid: &mut GCellGrid, path: &[GridCoord]) {
    for i in 0..path.len().saturating_sub(1) {
        let a = path[i];
        let b = path[i + 1];
        if a.z == b.z {
            if a.x != b.x {
                let min_x = a.x.min(b.x);
                grid.add_h_wire(min_x, a.y, a.z);
            } else if a.y != b.y {
                let min_y = a.y.min(b.y);
                grid.add_v_wire(a.x, min_y, a.z);
            }
        }
    }
}

/// Removes edge usage for a path from the grid.
fn remove_path_from_grid(grid: &mut GCellGrid, path: &[GridCoord]) {
    for i in 0..path.len().saturating_sub(1) {
        let a = path[i];
        let b = path[i + 1];
        if a.z == b.z {
            if a.x != b.x {
                let min_x = a.x.min(b.x);
                grid.remove_h_wire(min_x, a.y, a.z);
            } else if a.y != b.y {
                let min_y = a.y.min(b.y);
                grid.remove_v_wire(a.x, min_y, a.z);
            }
        }
    }
}

/// Checks if any edge along a path is congested.
fn path_is_congested(grid: &GCellGrid, path: &[GridCoord]) -> bool {
    for i in 0..path.len().saturating_sub(1) {
        let a = path[i];
        let b = path[i + 1];
        if a.z == b.z {
            if a.x != b.x {
                let min_x = a.x.min(b.x);
                if grid.is_h_congested(min_x, a.y, a.z) {
                    return true;
                }
            } else if a.y != b.y {
                let min_y = a.y.min(b.y);
                if grid.is_v_congested(a.x, min_y, a.z) {
                    return true;
                }
            }
        }
    }
    false
}

/// Executes global routing on a gcell grid to generate routing guides.
pub fn run(
    db: &NetlistDB,
    config: &GlobalRoutingConfig,
) -> Result<(Vec<HashSet<GridCoord>>, GridConverter), String> {
    log::debug!("Starting Global Routing...");

    let bin_width = config.gcell_size as f64;
    let grid = GCellGrid::new(db, bin_width);

    let grid_w = grid.width();
    let grid_h = grid.height();
    let layers = grid.layers();

    let converter = GridConverter::new(db.die_area, grid_w, grid_h);

    // Wrap in mutable for routing loop
    let mut grid = grid;

    let mut net_paths: Vec<Vec<GridCoord>> = vec![Vec::new(); db.nets.len()];
    let mut collision_penalty = config.initial_penalty;
    let history_increment = config.history_increment;
    let total_nets = db.nets.len();

    log::debug!("GR: Starting Initial Route for {} nets...", total_nets);
    let start_time = Instant::now();

    let batch_size = 500;
    let net_indices: Vec<usize> = (0..total_nets).collect();

    let progress = AtomicUsize::new(0);

    for chunk in net_indices.chunks(batch_size) {
        let results: Vec<(usize, Vec<GridCoord>)> = chunk
            .par_iter()
            .map_with(AStar::new(), |solver, &net_id| {
                let net = &db.nets[net_id];
                if net.pins.len() < 2 {
                    return (net_id, Vec::new());
                }
                let path = compute_net_path_gr(
                    net,
                    &grid,
                    solver,
                    &converter,
                    db,
                    collision_penalty,
                    config,
                );

                let p = progress.fetch_add(1, Ordering::Relaxed) + 1;

                if p.is_multiple_of(100) || p == total_nets {
                    ui::progress("[GR Init]", p, total_nets);
                }
                (net_id, path)
            })
            .collect();

        for (net_id, path) in results {
            add_path_to_grid(&mut grid, &path);
            net_paths[net_id] = path;
        }
    }
    ui::progress_clear();
    ui::phase("Global Routing");
    log::info!("Initial route: {:.2}s", start_time.elapsed().as_secs_f32());

    let mut last_overflow = usize::MAX;
    let mut stagnation_counter = 0;
    let mut gr_table_printed = false;

    for iter in 0..config.max_iterations {
        let start = Instant::now();
        grid.set_penalty(collision_penalty);
        let overflow = grid.total_overflow();

        if overflow == 0 {
            ui::check(&format!("Global routing converged at iter {}", iter));
            break;
        }

        if !gr_table_printed {
            ui::routing_table_header("GR");
            gr_table_printed = true;
        }

        let improvement = last_overflow.saturating_sub(overflow);
        if improvement < (last_overflow / 100).max(5) {
            stagnation_counter += 1;
        } else {
            stagnation_counter = 0;
        }
        last_overflow = overflow;

        if stagnation_counter > 10 {
            log::warn!(
                "GR Stagnation detected ({} iters). Dumping heatmap.",
                stagnation_counter
            );
            if stagnation_counter % 5 == 0 {
                grid.decay_history(0.9);
            }
        }

        grid.update_history(history_increment);

        let mut nets_to_reroute = Vec::new();
        #[allow(clippy::needless_range_loop)]
        for net_id in 0..total_nets {
            let path = &net_paths[net_id];
            if path.is_empty() {
                continue;
            }

            // Check if internal edges are congested
            if path_is_congested(&grid, path) {
                remove_path_from_grid(&mut grid, path);
                net_paths[net_id].clear();
                nets_to_reroute.push(net_id);
            }
        }

        let ripped = nets_to_reroute.len();
        let mut rng = thread_rng();
        nets_to_reroute.shuffle(&mut rng);

        if ripped < 500 {
            for (i, &net_id) in nets_to_reroute.iter().enumerate() {
                let path = compute_net_path_gr(
                    &db.nets[net_id],
                    &grid,
                    &mut AStar::new(),
                    &converter,
                    db,
                    collision_penalty,
                    config,
                );
                add_path_to_grid(&mut grid, &path);
                net_paths[net_id] = path;

                if i % 50 == 0 || i == ripped - 1 {
                    ui::progress(&format!("[GR Iter {}]", iter), i + 1, ripped);
                }
            }
        } else {
            let progress = AtomicUsize::new(0);
            for chunk in nets_to_reroute.chunks(batch_size) {
                let results: Vec<(usize, Vec<GridCoord>)> = chunk
                    .par_iter()
                    .map_with(AStar::new(), |solver, &net_id| {
                        let path = compute_net_path_gr(
                            &db.nets[net_id],
                            &grid,
                            solver,
                            &converter,
                            db,
                            collision_penalty,
                            config,
                        );

                        let p = progress.fetch_add(1, Ordering::Relaxed) + 1;
                        if p.is_multiple_of(100) || p == ripped {
                            ui::progress(&format!("[GR Iter {}]", iter), p, ripped);
                        }
                        (net_id, path)
                    })
                    .collect();

                for (net_id, path) in results {
                    add_path_to_grid(&mut grid, &path);
                    net_paths[net_id] = path;
                }
            }
        }
        ui::progress_clear();

        ui::routing_iter("GR", iter, overflow, ripped, collision_penalty, start.elapsed().as_millis());

        if ripped == 0 {
            ui::check("Global routing converged (endpoint congestion only)");
            break;
        }

        if iter > 50 && ripped < 20 {
            log::warn!(
                "GR: Stopping early. Remaining {} ripped nets are likely unresolvable.",
                ripped
            );
            break;
        }

        if stagnation_counter > 20 {
            log::warn!(
                "GR: Stopping due to stagnation ({} iters without improvement).",
                stagnation_counter
            );
            break;
        }

        collision_penalty = (collision_penalty * config.penalty_multiplier).min(10_000.0);
    }

    let mut net_guides: Vec<HashSet<GridCoord>> = vec![HashSet::new(); db.nets.len()];
    for (net_id, path) in net_paths.iter().enumerate() {
        for &coord in path {
            for z in 0..layers {
                net_guides[net_id].insert(GridCoord::new(coord.x, coord.y, z));
                for n in get_neighbors_2d(coord, grid_w, grid_h) {
                    net_guides[net_id].insert(GridCoord::new(n.x, n.y, z));
                }
            }
        }
    }

    Ok((net_guides, converter))
}

/// Computes the routing path for a single net in global routing.
fn compute_net_path_gr(
    net: &pare_common::db::core::NetData,
    grid: &GCellGrid,
    solver: &mut AStar,
    converter: &GridConverter,
    db: &NetlistDB,
    penalty: f64,
    config: &GlobalRoutingConfig,
) -> Vec<GridCoord> {
    let pin_coords: Vec<GridCoord> = net
        .pins
        .iter()
        .map(|&pid| {
            let cell_id = db.pin_to_cell[pid.index()];
            let pos = db.get_pin_position(pid, &db.positions[cell_id.index()]);
            converter.to_grid(pos, 0)
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

    let mut tree_nodes = vec![pin_coords[sorted_indices[0]]];
    let mut occupied = HashSet::new();
    occupied.insert(pin_coords[sorted_indices[0]]);

    for i in 1..sorted_indices.len() {
        let target = pin_coords[sorted_indices[i]];

        if let Some(path) = solver.find_path(
            grid,
            db,
            &tree_nodes,
            target,
            penalty,
            config.heuristic_weight,
            config.margin,
            1.0,
            &NoGuide,
            &[],
            50_000,
            false,
        ) {
            for &c in &path {
                occupied.insert(c);
            }
            tree_nodes.extend_from_slice(&path);
        }
    }
    occupied.into_iter().collect()
}

/// Returns the 2D neighbors of a grid coordinate (same layer).
fn get_neighbors_2d(c: GridCoord, w: u32, h: u32) -> Vec<GridCoord> {
    let mut n = Vec::new();
    if c.x > 0 {
        n.push(GridCoord::new(c.x - 1, c.y, c.z));
    }
    if c.x < w - 1 {
        n.push(GridCoord::new(c.x + 1, c.y, c.z));
    }
    if c.y > 0 {
        n.push(GridCoord::new(c.x, c.y - 1, c.z));
    }
    if c.y < h - 1 {
        n.push(GridCoord::new(c.x, c.y + 1, c.z));
    }
    n
}
