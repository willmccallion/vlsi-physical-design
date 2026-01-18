/// Fast guide oracle for efficient guide constraint checking.
///
/// Provides efficient guide constraint checking for detailed routing using
/// precomputed coordinate mapping tables and tag-based tracking. Maintains
/// precomputed mapping tables from fine grid coordinates to coarse grid
/// coordinates, allowing O(1) guide checking. Expands guides by a small radius
/// to provide routing flexibility while maintaining guide adherence. Uses tag-based
/// tracking to support multiple nets without clearing the entire grid.
mod oracle;
/// A* pathfinding router with guide constraints and topology generation.
///
/// Routes individual nets using A* pathfinding with guide constraints. Converts
/// pin positions to grid coordinates, sorts pins by proximity to build a minimum
/// spanning tree topology, then routes paths sequentially from the tree root to
/// each remaining pin. Uses A* with guide constraints and falls back to
/// unconstrained routing if guide-constrained routing fails. Generates routing
/// segments from grid paths for database storage.
mod router;
/// Spatial data structure for batching routing operations.
///
/// Maintains a grid of bins to track which regions have active routing operations.
/// Used to schedule routing batches that don't spatially overlap, enabling parallel
/// execution without conflicts. Uses tag-based tracking to reset state efficiently
/// between batches, providing O(1) reset performance without clearing the entire
/// array. The bin size controls the granularity of spatial conflict detection.
mod scheduler;

use self::oracle::FastGuideOracle;
use self::router::{generate_segments_from_topology, route_net_dr_pure};
use self::scheduler::SpatialSet;

use crate::algo::astar::AStar;
use crate::grid::RoutingGrid;
use crate::grid::dense::DenseGrid;
use crate::utils::conversion::GridConverter;
use eda_common::db::core::NetlistDB;
use eda_common::geom::coord::GridCoord;
use eda_common::util::config::DetailedRoutingConfig;
use eda_common::util::visualization::draw_congestion_heatmap;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use std::io::Write;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

/// Executes detailed routing on a fine grid using global routing guides.
///
/// First performs initial routing of all nets in batches, then iteratively
/// reroutes congested nets using rip-up-and-reroute. Uses spatial batching
/// to parallelize routing while avoiding conflicts. Tracks congestion history
/// and adaptively increases penalties for persistent congestion. Generates
/// routing segments from the grid paths and updates the database. Returns
/// an error if routing fails to converge.
pub fn run(
    db: &mut NetlistDB,
    config: &DetailedRoutingConfig,
    guides: &[HashSet<GridCoord>],
    coarse_converter: &GridConverter,
) -> Result<(), String> {
    log::info!("Starting Detailed Routing...");

    let (mut step_x, off_x) = db
        .tracks
        .iter()
        .find(|t| t.direction == "X" && t.step > 2.0)
        .map(|t| (t.step, t.start))
        .unwrap_or_else(|| {
            let p = db.layers.first().map(|l| l.pitch).unwrap_or(0.2);
            (p, 0.0)
        });

    let (mut step_y, off_y) = db
        .tracks
        .iter()
        .find(|t| t.direction == "Y" && t.step > 2.0)
        .map(|t| (t.step, t.start))
        .unwrap_or_else(|| {
            let p = db.layers.first().map(|l| l.pitch).unwrap_or(0.2);
            (p, 0.0)
        });

    let layers = if db.layers.is_empty() {
        2
    } else {
        db.layers.len() as u8
    };
    let raw_grid_w = ((db.die_area.width() - off_x) / step_x).ceil() as u64;
    let raw_grid_h = ((db.die_area.height() - off_y) / step_y).ceil() as u64;
    let total_points = raw_grid_w * raw_grid_h * (layers as u64);

    if total_points > 50_000_000 {
        let scale_factor = (total_points as f64 / 50_000_000.0).sqrt().ceil();
        log::warn!(
            "Grid too large ({:.1}B points). Scaling step by {:.0}x.",
            total_points as f64 / 1e9,
            scale_factor
        );
        step_x *= scale_factor;
        step_y *= scale_factor;
    }

    let grid_w = ((db.die_area.width() - off_x) / step_x).ceil() as u32;
    let grid_h = ((db.die_area.height() - off_y) / step_y).ceil() as u32;

    log::info!(
        "Detailed Grid: {}x{} (Step X: {:.3}, Y: {:.3})",
        grid_w,
        grid_h,
        step_x,
        step_y
    );

    let mut grid = DenseGrid::new(grid_w, grid_h, layers, config.capacity);
    let converter = GridConverter::from_steps(
        step_x,
        step_y,
        db.die_area.min.x,
        db.die_area.min.y,
        grid_w,
        grid_h,
    );
    for i in 0..db.num_cells() {
        let cell = &db.cells[i];
        let pos = db.positions[i];
        let min = converter.to_grid(pos, 0);
        let max_p = eda_common::geom::point::Point::new(pos.x + cell.width, pos.y + cell.height);
        let max = converter.to_grid(max_p, 0);

        for x in min.x.min(grid_w - 1)..=max.x.min(grid_w - 1) {
            for y in min.y.min(grid_h - 1)..=max.y.min(grid_h - 1) {
                grid.set_obstacle(GridCoord::new(x, y, 0));
            }
        }
    }

    let mut net_paths: Vec<Vec<GridCoord>> = vec![Vec::new(); db.nets.len()];
    let mut net_topologies: Vec<Vec<Vec<GridCoord>>> = vec![Vec::new(); db.nets.len()];
    let mut ripup_counts: Vec<u32> = vec![0; db.nets.len()];

    let total_nets = db.nets.len();

    let mut net_indices: Vec<usize> = (0..total_nets).collect();
    net_indices.sort_by_key(|&id| {
        if let Some(pin) = db.nets[id].pins.first() {
            let cid = db.pin_to_cell[pin.index()];
            let pos = db.positions[cid.index()];
            ((pos.x as i32) / 1000, (pos.y as i32) / 1000)
        } else {
            (0, 0)
        }
    });

    let batch_count = (total_nets / 1000).clamp(8, 128);
    let batch_size = (total_nets + batch_count - 1) / batch_count;

    log::info!("DR: Batched Initial Route (Windowed Order)...");
    let start_time = Instant::now();

    let progress = AtomicUsize::new(0);

    for (b_idx, chunk) in net_indices.chunks(batch_size).enumerate() {
        let batch_penalty = if b_idx == 0 { 1.0 } else { 2.0 };
        grid.set_penalty(batch_penalty);

        let coarse_max = coarse_converter.to_grid(
            eda_common::geom::point::Point::new(db.die_area.width(), db.die_area.height()),
            0,
        );
        let coarse_w = coarse_max.x + 1;
        let coarse_h = coarse_max.y + 1;

        let results: Vec<(usize, Option<(Vec<GridCoord>, Vec<Vec<GridCoord>>)>)> = chunk
            .par_iter()
            .map_with(
                (
                    AStar::new(),
                    FastGuideOracle::new(
                        grid_w,
                        grid_h,
                        coarse_w,
                        coarse_h,
                        layers,
                        &converter,
                        coarse_converter,
                    ),
                ),
                |(solver, oracle), &net_id| {
                    let net = &db.nets[net_id];
                    if net.pins.len() < 2 {
                        return (net_id, None);
                    }
                    oracle.prepare(net_id, &guides[net_id]);
                    let res = route_net_dr_pure(
                        net,
                        &grid,
                        solver,
                        &converter,
                        db,
                        batch_penalty,
                        oracle,
                        config,
                        0,
                        b_idx > 0,
                    );

                    let p = progress.fetch_add(1, Ordering::Relaxed) + 1;
                    if p.is_multiple_of(100) || p == total_nets {
                        eprint!("\r\x1b[36m[DR Init] {}/{}\x1b[0m\x1b[K", p, total_nets);
                        let _ = std::io::stderr().flush();
                    }

                    (net_id, res)
                },
            )
            .collect();

        for (net_id, res) in results {
            if let Some((occupied, topology)) = res {
                for &c in &occupied {
                    grid.add_wire(c);
                }
                net_paths[net_id] = occupied;
                net_topologies[net_id] = topology;
            } else {
                net_paths[net_id].clear();
                net_topologies[net_id].clear();
            }
        }
    }
    eprint!("\r\x1b[K");

    log::info!("Initial Route: {:.2}s", start_time.elapsed().as_secs_f32());

    draw_congestion_heatmap(&grid, "output/dr_initial_congestion.png");

    let mut collision_penalty = config.initial_penalty.max(2.5);
    let history_increment = config.history_increment.max(1.0);
    let mut last_conflicts = usize::MAX;
    let mut stagnation_counter = 0;
    let mut spatial_set = SpatialSet::new(grid_w, grid_h, 200);

    for iter in 0..config.max_iterations {
        let start = Instant::now();
        let mut conflicts = grid.total_conflicts();

        let failed_nets: Vec<usize> = net_paths
            .iter()
            .enumerate()
            .filter(|(id, p)| p.is_empty() && db.nets[*id].pins.len() >= 2)
            .map(|(id, _)| id)
            .collect();
        conflicts += failed_nets.len();

        if conflicts == 0 {
            log::info!("Converged at iter {}!", iter);
            break;
        }

        let improvement = last_conflicts.saturating_sub(conflicts);
        let threshold = (last_conflicts / 20).max(5);

        if improvement < threshold {
            stagnation_counter += 1;
        } else {
            stagnation_counter = 0;
        }
        last_conflicts = conflicts;

        if stagnation_counter > config.stagnation_threshold {
            log::warn!(
                "Stagnation detected ({} iters). Dumping congestion heatmap...",
                stagnation_counter
            );
            draw_congestion_heatmap(&grid, &format!("output/dr_congestion_iter_{}.png", iter));

            if stagnation_counter == config.stagnation_threshold + 1 {
                log::warn!("Dumping first 5 failing nets for debug:");
                for &net_id in failed_nets.iter().take(5) {
                    let net = &db.nets[net_id];
                    log::warn!(" - Net {}: {} pins", net.name, net.pins.len());
                    for &pin in &net.pins {
                        let cell_id = db.pin_to_cell[pin.index()];
                        let pos = db.positions[cell_id.index()];
                        log::warn!(
                            "   - Pin on cell {} at ({:.2}, {:.2})",
                            db.cells[cell_id.index()].name,
                            pos.x,
                            pos.y
                        );
                    }
                }
            }
        }

        if stagnation_counter > 2 * config.stagnation_threshold {
            log::error!("Routing stagnated. Stopping early.");
            break;
        }

        let mut force_ripup = false;
        let effective_history_inc = if stagnation_counter > config.stagnation_threshold {
            force_ripup = true;
            history_increment * config.history_increment
        } else {
            history_increment
        };

        grid.update_history(effective_history_inc);

        if force_ripup && stagnation_counter == config.stagnation_threshold + 1 {
            grid.decay_history(0.5);
        }

        grid.set_penalty(collision_penalty);

        let mut nets_to_reroute = HashSet::new();
        for &net_id in &failed_nets {
            nets_to_reroute.insert(net_id);
        }

        let mut congested_nodes = HashSet::new();
        for net_id in 0..db.nets.len() {
            let occupied = &net_paths[net_id];
            if !occupied.is_empty() {
                let mut congested = false;
                for &c in occupied {
                    if grid.is_congested(c) {
                        congested = true;
                        if force_ripup {
                            congested_nodes.insert(c);
                        }
                    }
                }
                if congested {
                    nets_to_reroute.insert(net_id);
                }
            }
        }

        if force_ripup && !congested_nodes.is_empty() {
            let dynamic_radius = config.ripup_radius + (stagnation_counter as i32 / 5);
            let mut kill_zone = HashSet::new();
            for &c in &congested_nodes {
                for dz in 0..layers {
                    for dy in -dynamic_radius..=dynamic_radius {
                        for dx in -dynamic_radius..=dynamic_radius {
                            let nx = c.x as i32 + dx;
                            let ny = c.y as i32 + dy;
                            if nx >= 0 && nx < grid_w as i32 && ny >= 0 && ny < grid_h as i32 {
                                kill_zone.insert(GridCoord::new(nx as u32, ny as u32, dz));
                            }
                        }
                    }
                }
            }
            for net_id in 0..db.nets.len() {
                if nets_to_reroute.contains(&net_id) {
                    continue;
                }
                let occupied = &net_paths[net_id];
                for &c in occupied {
                    if kill_zone.contains(&c) {
                        nets_to_reroute.insert(net_id);
                        break;
                    }
                }
            }
        }

        let mut nets_with_priority: Vec<(usize, u32)> = nets_to_reroute
            .into_iter()
            .map(|net_id| {
                let occupied = &net_paths[net_id];
                let congestion_score: u32 =
                    occupied.iter().filter(|&&c| grid.is_congested(c)).count() as u32;
                (net_id, congestion_score + ripup_counts[net_id])
            })
            .collect();

        nets_with_priority.sort_by(|a, b| b.1.cmp(&a.1));
        let mut nets_vec: Vec<usize> = nets_with_priority.into_iter().map(|(id, _)| id).collect();

        let max_reroute = if iter == 0 {
            (nets_vec.len() * 15 / 100).max(500).min(nets_vec.len())
        } else if iter < 3 {
            (nets_vec.len() * 30 / 100).max(1500).min(nets_vec.len())
        } else if iter < 8 {
            (nets_vec.len() * 50 / 100).max(2500).min(nets_vec.len())
        } else {
            nets_vec.len()
        };

        nets_vec.truncate(max_reroute);

        for &net_id in &nets_vec {
            ripup_counts[net_id] += 1;
        }

        let mut candidates: Vec<(usize, (i32, i32, i32, i32))> = nets_vec
            .iter()
            .map(|&net_id| {
                let net = &db.nets[net_id];
                let occupied = &net_paths[net_id];
                let mut min_x = i32::MAX;
                let mut max_x = i32::MIN;
                let mut min_y = i32::MAX;
                let mut max_y = i32::MIN;

                let iter_points = if !occupied.is_empty() {
                    Box::new(occupied.iter().map(|c| (c.x, c.y)))
                        as Box<dyn Iterator<Item = (u32, u32)>>
                } else {
                    Box::new(net.pins.iter().map(|&pid| {
                        let cid = db.pin_to_cell[pid.index()];
                        let pos = db.get_pin_position(pid, &db.positions[cid.index()]);
                        let g = converter.to_grid(pos, 0);
                        (g.x, g.y)
                    }))
                };

                for (x, y) in iter_points {
                    min_x = min_x.min(x as i32);
                    max_x = max_x.max(x as i32);
                    min_y = min_y.min(y as i32);
                    max_y = max_y.max(y as i32);
                }
                (net_id, (min_x - 5, max_x + 5, min_y - 5, max_y + 5))
            })
            .collect();

        let total_ripped = nets_vec.len();
        let progress = AtomicUsize::new(0);

        while !candidates.is_empty() {
            let mut batch = Vec::new();
            let mut remaining = Vec::new();
            spatial_set.reset();

            for item in candidates {
                if spatial_set.try_insert(&item.1) {
                    batch.push(item.0);
                } else {
                    remaining.push(item);
                }
            }

            for &net_id in &batch {
                for &coord in &net_paths[net_id] {
                    grid.remove_wire(coord);
                }
            }

            let coarse_max = coarse_converter.to_grid(
                eda_common::geom::point::Point::new(db.die_area.width(), db.die_area.height()),
                0,
            );
            let coarse_w = coarse_max.x + 1;
            let coarse_h = coarse_max.y + 1;

            let results: Vec<(usize, Option<(Vec<GridCoord>, Vec<Vec<GridCoord>>)>)> = batch
                .par_iter()
                .map_with(
                    (
                        AStar::new(),
                        FastGuideOracle::new(
                            grid_w,
                            grid_h,
                            coarse_w,
                            coarse_h,
                            layers,
                            &converter,
                            coarse_converter,
                        ),
                    ),
                    |(solver, oracle), &net_id| {
                        oracle.prepare(net_id, &guides[net_id]);
                        let use_strict = iter > 2 && ripup_counts[net_id] < 3;
                        let res = route_net_dr_pure(
                            &db.nets[net_id],
                            &grid,
                            solver,
                            &converter,
                            db,
                            collision_penalty,
                            oracle,
                            config,
                            ripup_counts[net_id],
                            use_strict,
                        );

                        let p = progress.fetch_add(1, Ordering::Relaxed) + 1;
                        if p % 100 == 0 || p == total_ripped {
                            eprint!(
                                "\r\x1b[36m[DR Iter {}] {}/{}\x1b[0m\x1b[K",
                                iter, p, total_ripped
                            );
                            let _ = std::io::stderr().flush();
                        }

                        (net_id, res)
                    },
                )
                .collect();

            for (net_id, res) in results {
                if let Some((occupied, topology)) = res {
                    for &c in &occupied {
                        grid.add_wire(c);
                    }
                    net_paths[net_id] = occupied;
                    net_topologies[net_id] = topology;
                } else {
                    net_paths[net_id].clear();
                    net_topologies[net_id].clear();
                }
            }
            candidates = remaining;
        }
        eprint!("\r\x1b[K");

        log::info!(
            "Iter {}: Conflicts: {}, Ripped: {}, Pen: {:.1}, Time: {}ms",
            iter,
            conflicts,
            nets_vec.len(),
            collision_penalty,
            start.elapsed().as_millis()
        );

        if nets_vec.is_empty() {
            break;
        }
        collision_penalty = (collision_penalty * config.penalty_multiplier).min(20000.0);
    }

    let mut all_segments = Vec::with_capacity(db.nets.len());
    for (net_id, topology) in net_topologies.iter().enumerate() {
        let mut segments = Vec::new();
        let net = &db.nets[net_id];
        let mut pin_locations = HashMap::new();

        for &pin_id in &net.pins {
            let cell_id = db.pin_to_cell[pin_id.index()];
            let exact_pos = db.get_pin_position(pin_id, &db.positions[cell_id.index()]);

            let die_w = db.die_area.width();
            let die_h = db.die_area.height();
            let is_io = exact_pos.x <= 0.001
                || exact_pos.x >= die_w - 0.001
                || exact_pos.y <= 0.001
                || exact_pos.y >= die_h - 0.001;
            let layer = if is_io { 2 } else { 1 };
            let safe_layer = layer.min(grid.layers() - 1);
            let grid_pos = converter.to_grid(exact_pos, safe_layer);

            pin_locations.insert((grid_pos.x, grid_pos.y, grid_pos.z), exact_pos);
        }

        if !topology.is_empty() {
            segments = generate_segments_from_topology(topology, &pin_locations, &converter);
        }
        all_segments.push(segments);
    }

    for (net_id, segments) in all_segments.into_iter().enumerate() {
        if !segments.is_empty() {
            db.nets[net_id].route_segments = segments;
        }
    }

    Ok(())
}
