use crate::algo::astar::{AStar, NoGuide};
use crate::grid::RoutingGrid;
use crate::grid::dense::DenseGrid;
use crate::utils::conversion::GridConverter;
use eda_common::db::core::NetlistDB;
use eda_common::geom::coord::GridCoord;
use eda_common::util::config::GlobalRoutingConfig;
use eda_common::util::visualization::draw_congestion_heatmap;
use rand::seq::SliceRandom;
use rand::thread_rng;
use rayon::prelude::*;
use std::collections::HashSet;
use std::io::Write;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

/// Executes global routing on a coarse grid to generate routing guides.
///
/// Routes all nets on a coarse grid (typically 100-200x coarser than detailed
/// routing) to determine preferred routing regions. Uses A* pathfinding with
/// congestion-aware costs and rip-up-and-reroute to resolve conflicts. The
/// resulting paths are expanded into guides that constrain detailed routing.
/// Returns the guide sets for each net and a grid converter for coordinate
/// transformation, or an error if routing fails.
pub fn run(
    db: &NetlistDB,
    config: &GlobalRoutingConfig,
) -> Result<(Vec<HashSet<GridCoord>>, GridConverter), String> {
    log::info!("Starting Global Routing...");

    let bin_width = config.gcell_size as f64;
    let die_w = db.die_area.width();
    let die_h = db.die_area.height();

    let grid_w = (die_w / bin_width).ceil() as u32;
    let grid_h = (die_h / bin_width).ceil() as u32;

    let grid_w = grid_w.max(1);
    let grid_h = grid_h.max(1);

    let layers = if db.layers.is_empty() {
        2
    } else {
        db.layers.len() as u8
    };

    let avg_pitch = if !db.layers.is_empty() {
        let sum: f64 = db.layers.iter().map(|l| l.pitch).sum();
        sum / db.layers.len() as f64
    } else {
        1.0
    };

    let physical_tracks = (bin_width / avg_pitch).floor() as u32;
    let default_capacity = config.capacity;

    log::info!(
        "GR Grid: {}x{} (Bin Size: {:.1}). Pitch={:.2} -> Physical Tracks/Bin={}. Config Cap={}",
        grid_w,
        grid_h,
        bin_width,
        avg_pitch,
        physical_tracks,
        default_capacity
    );

    let mut grid = DenseGrid::new(grid_w, grid_h, layers, default_capacity);

    let converter = GridConverter::new(db.die_area, grid_w, grid_h);

    if layers > 0 {
        log::info!("Setting Layer 0 (M1) capacity to INFINITE for pin access.");
        grid.set_layer_capacity(0, 999_999);
    }

    let mut net_paths: Vec<Vec<GridCoord>> = vec![Vec::new(); db.nets.len()];
    let mut collision_penalty = config.initial_penalty;
    let history_increment = config.history_increment;
    let total_nets = db.nets.len();

    log::info!("GR: Starting Initial Route for {} nets...", total_nets);
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

                if p % 100 == 0 || p == total_nets {
                    eprint!("\r\x1b[36m[GR Init] {}/{}\x1b[0m\x1b[K", p, total_nets);
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
    eprint!("\r\x1b[K");
    log::info!("Initial Route: {:.2}s", start_time.elapsed().as_secs_f32());

    draw_congestion_heatmap(&grid, "output/gr_initial_congestion.png");

    let mut last_conflicts = usize::MAX;
    let mut stagnation_counter = 0;

    for iter in 0..config.max_iterations {
        let start = Instant::now();
        let conflicts = grid.total_conflicts();

        if conflicts == 0 {
            log::info!("Global Routing Converged at iter {}!", iter);
            break;
        }

        let improvement = last_conflicts.saturating_sub(conflicts);
        if improvement < (last_conflicts / 100).max(5) {
            stagnation_counter += 1;
        } else {
            stagnation_counter = 0;
        }
        last_conflicts = conflicts;

        if stagnation_counter > 10 {
            log::warn!(
                "GR Stagnation detected ({} iters). Dumping heatmap.",
                stagnation_counter
            );
            draw_congestion_heatmap(&grid, &format!("output/gr_congestion_iter_{}.png", iter));
            if stagnation_counter % 5 == 0 {
                grid.decay_history(0.9);
            }
        }

        grid.update_history(history_increment);

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
                    if i > 0 && i < len - 1 {
                        internal_congestion = true;
                        break;
                    }
                }
            }

            if internal_congestion {
                for &coord in path {
                    grid.remove_wire(coord);
                }
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
                for &c in &path {
                    grid.add_wire(c);
                }
                net_paths[net_id] = path;

                if i % 50 == 0 || i == ripped - 1 {
                    eprint!(
                        "\r\x1b[36m[GR Iter {}] {}/{}\x1b[0m\x1b[K",
                        iter,
                        i + 1,
                        ripped
                    );
                    let _ = std::io::stderr().flush();
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
                        if p % 100 == 0 || p == ripped {
                            eprint!("\r\x1b[36m[GR Iter {}] {}/{}\x1b[0m\x1b[K", iter, p, ripped);
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
        eprint!("\r\x1b[K");

        log::info!(
            "GR Iter {}: Conflicts: {}, Ripped: {}, Penalty: {:.2}, Time: {}ms",
            iter,
            conflicts,
            ripped,
            collision_penalty,
            start.elapsed().as_millis()
        );

        if ripped == 0 {
            log::info!("Global Routing Converged (Endpoint congestion only).");
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

    draw_congestion_heatmap(&grid, "output/gr_congestion.png");

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
///
/// Converts pin positions to grid coordinates, sorts pins by proximity
/// to build a routing order, then routes paths sequentially using A*
/// pathfinding. Connects paths end-to-end to form a complete tree
/// connecting all pins. Returns the complete path as a sequence of grid
/// coordinates.
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
            1.0,
            &NoGuide,
            &[],
            50_000,
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

/// Returns the 2D neighbors of a grid coordinate (same layer).
///
/// Generates the four adjacent coordinates (north, south, east, west)
/// on the same layer, excluding coordinates outside the grid boundaries.
/// Used for guide expansion in global routing to create routing regions.
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
