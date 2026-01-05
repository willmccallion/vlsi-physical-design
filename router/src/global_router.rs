use crate::algo::astar::{AStar, NoGuide};
use crate::grid::RoutingGrid;
use crate::grid::dense::DenseGrid;
use crate::utils::conversion::GridConverter;
use eda_common::db::core::NetlistDB;
use eda_common::geom::coord::GridCoord;
use eda_common::util::config::GlobalRoutingConfig;
use rand::seq::SliceRandom;
use rand::thread_rng;
use rayon::prelude::*;
use std::collections::HashSet;
use std::io::Write;
use std::sync::Mutex;
use std::time::Instant;

pub fn run(
    db: &NetlistDB,
    config: &GlobalRoutingConfig,
) -> Result<(Vec<HashSet<GridCoord>>, GridConverter), String> {
    log::info!("Starting Global Routing...");

    let grid_w = config.gcell_size as u32;
    let grid_h = config.gcell_size as u32;
    let layers = if db.layers.is_empty() {
        2
    } else {
        db.layers.len() as u8
    };

    let default_capacity = config.capacity;

    let mut grid = DenseGrid::new(grid_w, grid_h, layers, default_capacity);
    let converter = GridConverter::new(db.die_area.width(), db.die_area.height(), grid_w, grid_h);

    let mut net_paths: Vec<Vec<GridCoord>> = vec![Vec::new(); db.nets.len()];
    let mut collision_penalty = config.initial_penalty;
    let history_increment = config.history_increment;
    let total_nets = db.nets.len();

    log::info!("GR: Starting Initial Route for {} nets...", total_nets);
    let start_time = Instant::now();

    let batch_size = 500;
    let net_indices: Vec<usize> = (0..total_nets).collect();
    let progress = Mutex::new(0);

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

                let mut p: usize = *progress.lock().unwrap();
                p += 1;
                if p.is_multiple_of(50) || p == total_nets {
                    let pct = (p as f64 / total_nets as f64) * 100.0;
                    eprint!(
                        "\r\x1b[36m[GR Init] Progress: {:>3.0}% ({}/{}) Time: {:.1}s\x1b[0m",
                        pct,
                        p,
                        total_nets,
                        start_time.elapsed().as_secs_f32()
                    );
                    let _ = std::io::stderr().flush();
                }
                (net_id, path)
            })
            .collect();

        for (net_id, path) in results {
            for &c in &path {
                grid.add_wire(c);
            }
            net_paths[net_id] = path;
        }
    }
    eprint!("\r\x1b[2K");

    for iter in 0..config.max_iterations {
        let start = Instant::now();
        let conflicts = grid.total_conflicts();

        if conflicts == 0 {
            log::info!("Global Routing Converged at iter {}!", iter);
            break;
        }

        grid.update_history(history_increment);

        // Identify and rip-up congested nets
        let mut nets_to_reroute = Vec::new();
        for net_id in 0..total_nets {
            let path = &net_paths[net_id];
            if path.is_empty() {
                continue;
            }

            let mut internal_congestion = false;
            let len = path.len();

            for (i, &coord) in path.iter().enumerate() {
                if grid.is_congested(coord) {
                    // We only rip up if there is "internal" congestion along the wire.
                    if i > 0 && i < len - 1 {
                        internal_congestion = true;
                        break;
                    }
                }
            }

            // If the path is just 2 nodes (start and end) and both are congested,
            // we treat it as endpoint congestion and do not rip up.
            if internal_congestion {
                for &coord in path {
                    grid.remove_wire(coord);
                }
                net_paths[net_id].clear();
                nets_to_reroute.push(net_id);
            }
        }

        let ripped = nets_to_reroute.len();

        // Shuffle to prevent livelock
        let mut rng = thread_rng();
        nets_to_reroute.shuffle(&mut rng);

        // Hybrid Execution Strategy
        if ripped < 500 {
            // Sequential: For precision when conflicts are few.
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
                for &c in &path {
                    grid.add_wire(c);
                }
                net_paths[net_id] = path;

                if i % 10 == 0 && i > 0 {
                    eprint!(
                        "\r\x1b[36m[GR Iter {}] Seq Reroute: {}/{} | Time: {:.1}s\x1b[0m",
                        iter,
                        i,
                        ripped,
                        start.elapsed().as_secs_f32()
                    );
                    let _ = std::io::stderr().flush();
                }
            }
        } else {
            // Parallel: For speed when conflicts are many.
            let progress = Mutex::new(0);
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

                        let mut p: usize = *progress.lock().unwrap();
                        p += 1;
                        if p.is_multiple_of(50) || p == ripped {
                            eprint!(
                                "\r\x1b[36m[GR Iter {}] Par Reroute: {}/{} | Time: {:.1}s\x1b[0m",
                                iter,
                                p,
                                ripped,
                                start.elapsed().as_secs_f32()
                            );
                            let _ = std::io::stderr().flush();
                        }
                        (net_id, path)
                    })
                    .collect();

                for (net_id, path) in results {
                    for &c in &path {
                        grid.add_wire(c);
                    }
                    net_paths[net_id] = path;
                }
            }
        }
        eprint!("\r\x1b[2K");

        log::info!(
            "GR Iter {}: Conflicts: {}, Ripped: {}, Penalty: {:.2}, Time: {}ms",
            iter,
            conflicts,
            ripped,
            collision_penalty,
            start.elapsed().as_millis()
        );

        if ripped == 0 {
            log::info!(
                "Global Routing Converged (Remaining conflicts are unavoidable endpoint congestion)."
            );
            break;
        }

        if iter > 100 && ripped < 50 {
            log::warn!(
                "GR: Stopping early. Remaining {} ripped nets are likely unresolvable at global level.",
                ripped
            );
            break;
        }

        collision_penalty *= config.penalty_multiplier;
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

fn compute_net_path_gr(
    net: &eda_common::db::core::NetData,
    grid: &DenseGrid,
    solver: &mut AStar,
    converter: &GridConverter,
    db: &NetlistDB,
    penalty: f64,
    config: &GlobalRoutingConfig,
) -> Vec<GridCoord> {
    let mut pin_indices: Vec<usize> = (0..net.pins.len()).collect();
    let mut sorted_indices = Vec::with_capacity(net.pins.len());
    let mut current_idx = pin_indices.remove(0);
    sorted_indices.push(current_idx);

    let pin_coords: Vec<GridCoord> = net
        .pins
        .iter()
        .map(|&pid| {
            let cell_id = db.pin_to_cell[pid.index()];
            let pos = db.get_pin_position(pid, &db.positions[cell_id.index()]);
            converter.to_grid(pos, 0)
        })
        .collect();

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

    let start_pin = net.pins[sorted_indices[0]];
    let c1 = db.pin_to_cell[start_pin.index()];
    let start_pos = db.get_pin_position(start_pin, &db.positions[c1.index()]);
    let start = converter.to_grid(start_pos, 0);

    let mut full_path = Vec::new();
    let mut curr = start;

    for i in 1..sorted_indices.len() {
        let next_pin = net.pins[sorted_indices[i]];
        let c2 = db.pin_to_cell[next_pin.index()];
        let end_pos = db.get_pin_position(next_pin, &db.positions[c2.index()]);
        let end = converter.to_grid(end_pos, 0);

        if let Some(path) = solver.find_path(
            grid,
            db,
            &[curr],
            end,
            penalty,
            config.heuristic_weight,
            config.margin,
            1.0, // No dynamic margin in GR
            &NoGuide,
            &[],
            500_000,
            false,
        ) {
            if !full_path.is_empty() {
                full_path.extend(path.into_iter().skip(1));
            } else {
                full_path.extend(path);
            }
            curr = end;
        }
    }
    full_path
}

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
