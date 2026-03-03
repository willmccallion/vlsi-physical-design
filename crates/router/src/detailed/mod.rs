/// Fast guide oracle for efficient guide constraint checking.
mod oracle;
/// A* pathfinding router with guide constraints and topology generation.
mod router;
/// Spatial data structure for batching routing operations.
mod scheduler;

use self::oracle::FastGuideOracle;
use self::router::{
    LARGE_NET_PIN_THRESHOLD, generate_segments_from_topology, route_net_dr_pure, route_net_parallel,
};
use self::scheduler::SpatialSet;

use crate::algo::astar::AStar;
use crate::grid::GCellGrid;
use crate::grid::RoutingGrid;
use crate::utils::conversion::GridConverter;
use pare_common::db::core::NetlistDB;
use pare_common::geom::coord::GridCoord;
use pare_common::geom::point::Point;
use pare_common::util::config::DetailedRoutingConfig;
use pare_common::util::ui;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

/// Adds edge usage for a single sub-path on the grid.
fn add_subpath_to_grid(grid: &mut GCellGrid, path: &[GridCoord]) {
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

/// Adds edge usage for all sub-paths of a topology.
fn add_topology_to_grid(grid: &mut GCellGrid, topology: &[Vec<GridCoord>]) {
    for path in topology {
        add_subpath_to_grid(grid, path);
    }
}

/// Removes edge usage for a single sub-path from the grid.
fn remove_subpath_from_grid(grid: &mut GCellGrid, path: &[GridCoord]) {
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

/// Removes edge usage for all sub-paths of a topology.
fn remove_topology_from_grid(grid: &mut GCellGrid, topology: &[Vec<GridCoord>]) {
    for path in topology {
        remove_subpath_from_grid(grid, path);
    }
}

/// Checks if any edge along a topology is congested.
fn topology_is_congested(grid: &GCellGrid, topology: &[Vec<GridCoord>]) -> bool {
    for path in topology {
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
    }
    false
}

/// Executes detailed routing on a gcell grid using global routing guides.
pub fn run(
    db: &mut NetlistDB,
    config: &DetailedRoutingConfig,
    guides: &[HashSet<GridCoord>],
    coarse_converter: &GridConverter,
) -> Result<(), String> {
    log::debug!("Starting Detailed Routing (GCell-based)...");

    let gcell_size = config.gcell_size;
    let mut grid = GCellGrid::new(db, gcell_size);

    let grid_w = grid.width();
    let grid_h = grid.height();
    let layers = grid.layers();

    // Build converter: gcell coordinates ↔ physical coordinates
    // Each gcell center maps to its physical position
    let converter = GridConverter::from_steps(
        grid.gcell_w(),
        grid.gcell_h(),
        db.die_area.min.x,
        db.die_area.min.y,
        grid_w,
        grid_h,
    );

    let mut net_topologies: Vec<Vec<Vec<GridCoord>>> = vec![Vec::new(); db.nets.len()];
    let mut ripup_counts: Vec<u32> = vec![0; db.nets.len()];

    let total_nets = db.nets.len();

    let mut net_indices: Vec<usize> = (0..total_nets).collect();
    // Sort by HPWL ascending: short nets first for better routability
    net_indices.sort_by_key(|&id| {
        let net = &db.nets[id];
        if net.pins.len() < 2 {
            return 0i64;
        }
        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        let mut min_y = f64::MAX;
        let mut max_y = f64::MIN;
        for &pid in &net.pins {
            let cid = db.pin_to_cell[pid.index()];
            let pos = db.get_pin_position(pid, &db.positions[cid.index()]);
            min_x = min_x.min(pos.x);
            max_x = max_x.max(pos.x);
            min_y = min_y.min(pos.y);
            max_y = max_y.max(pos.y);
        }
        ((max_x - min_x) + (max_y - min_y)) as i64
    });

    log::debug!("DR: Sequential Initial Route ({} nets)...", total_nets);
    let start_time = Instant::now();

    let coarse_max = coarse_converter.to_grid(
        pare_common::geom::point::Point::new(db.die_area.width(), db.die_area.height()),
        0,
    );
    let coarse_w = coarse_max.x + 1;
    let coarse_h = coarse_max.y + 1;

    let mut solver = AStar::new();
    let mut oracle = FastGuideOracle::new(
        grid_w,
        grid_h,
        coarse_w,
        coarse_h,
        layers,
        &converter,
        coarse_converter,
    );

    let init_penalty = config.initial_penalty.max(0.5);

    // Sequential initial routing
    let dummy_stat = AtomicUsize::new(0);
    for (progress, &net_id) in net_indices.iter().enumerate() {
        let net = &db.nets[net_id];
        if net.pins.len() < 2 {
            continue;
        }
        oracle.prepare(net_id, &guides[net_id]);
        let res = if net.pins.len() >= LARGE_NET_PIN_THRESHOLD {
            route_net_parallel(
                net,
                &grid,
                &mut solver,
                &converter,
                db,
                init_penalty,
                &oracle,
                config,
                0,
                false,
                grid_w,
                grid_h,
                coarse_w,
                coarse_h,
                layers,
                &converter,
                coarse_converter,
                &guides[net_id],
                net_id,
                &dummy_stat,
                &dummy_stat,
                &dummy_stat,
                &dummy_stat,
                &dummy_stat,
            )
        } else {
            route_net_dr_pure(
                net,
                &grid,
                &mut solver,
                &converter,
                db,
                init_penalty,
                &oracle,
                config,
                0,
                false,
                &dummy_stat,
                &dummy_stat,
                &dummy_stat,
                &dummy_stat,
                &dummy_stat,
            )
        };

        if let Some(topology) = res {
            add_topology_to_grid(&mut grid, &topology);
            net_topologies[net_id] = topology;
        }

        if (progress + 1) % 100 == 0 || progress + 1 == total_nets {
            ui::progress("[DR Init]", progress + 1, total_nets);
        }
    }
    ui::progress_clear();

    ui::phase("Detailed Routing");
    log::info!("Initial route: {:.2}s", start_time.elapsed().as_secs_f32());

    // Debug: per-layer edge overflow breakdown
    log_layer_stats(&grid, db, layers);

    let failed_init = net_topologies
        .iter()
        .enumerate()
        .filter(|(id, t)| t.is_empty() && db.nets[*id].pins.len() >= 2)
        .count();
    if failed_init > 0 {
        log::info!("Failed nets (no path found): {}", failed_init);
    }

    ui::routing_table_header("DR");

    let mut collision_penalty = init_penalty;
    let history_increment = config.history_increment;
    let mut last_overflow = usize::MAX;
    let mut stagnation_counter = 0;
    let mut spatial_set = SpatialSet::new(grid_w, grid_h, 10);

    for iter in 0..config.max_iterations {
        let start = Instant::now();
        let mut overflow = grid.total_overflow();

        let failed_nets: Vec<usize> = net_topologies
            .iter()
            .enumerate()
            .filter(|(id, t)| t.is_empty() && db.nets[*id].pins.len() >= 2)
            .map(|(id, _)| id)
            .collect();
        overflow += failed_nets.len();

        if overflow == 0 {
            ui::check(&format!("Detailed routing converged at iter {}", iter));
            break;
        }

        let improvement = last_overflow.saturating_sub(overflow);
        let threshold = (last_overflow / 20).max(5);

        if improvement < threshold {
            stagnation_counter += 1;
        } else {
            stagnation_counter = 0;
        }
        last_overflow = overflow;

        if stagnation_counter > config.stagnation_threshold {
            log::warn!(
                "Stagnation detected ({} iters). Dumping congestion heatmap...",
                stagnation_counter
            );
        }

        if stagnation_counter > 2 * config.stagnation_threshold {
            log::error!("Routing stagnated. Stopping early.");
            break;
        }

        let mut force_ripup = false;
        let effective_history_inc = if stagnation_counter > config.stagnation_threshold {
            force_ripup = true;
            history_increment * 2.0
        } else {
            history_increment
        };

        grid.update_history(effective_history_inc);

        if force_ripup && stagnation_counter == config.stagnation_threshold + 1 {
            // Full history reset + penalty reduction for small overflow.
            // When only a few edges are congested, accumulated history
            // makes them prohibitively expensive, trapping the router in
            // a cycle where nets keep avoiding the same edges.
            if overflow <= 10 {
                grid.decay_history(0.0);
                collision_penalty = init_penalty;
                log::info!(
                    "Reset history and penalty for small overflow ({})",
                    overflow
                );
            } else {
                grid.decay_history(0.5);
            }
        }

        grid.set_penalty(collision_penalty);

        let mut nets_to_reroute = HashSet::new();
        for &net_id in &failed_nets {
            nets_to_reroute.insert(net_id);
        }

        // Find nets using congested edges
        #[allow(clippy::needless_range_loop)]
        for net_id in 0..db.nets.len() {
            let topo = &net_topologies[net_id];
            if !topo.is_empty() && topology_is_congested(&grid, topo) {
                nets_to_reroute.insert(net_id);
            }
        }

        // Force ripup: also reroute neighbors of congested nets
        if force_ripup {
            let congested_gcells: HashSet<(u32, u32)> = nets_to_reroute
                .iter()
                .flat_map(|&nid| net_topologies[nid].iter().flatten().map(|c| (c.x, c.y)))
                .collect();

            let dynamic_radius = config.ripup_radius + (stagnation_counter as i32 / 5);
            let mut kill_zone = HashSet::new();
            for &(cx, cy) in &congested_gcells {
                for dy in -dynamic_radius..=dynamic_radius {
                    for dx in -dynamic_radius..=dynamic_radius {
                        let nx = cx as i32 + dx;
                        let ny = cy as i32 + dy;
                        if nx >= 0 && nx < grid_w as i32 && ny >= 0 && ny < grid_h as i32 {
                            kill_zone.insert((nx as u32, ny as u32));
                        }
                    }
                }
            }
            #[allow(clippy::needless_range_loop)]
            for net_id in 0..db.nets.len() {
                if nets_to_reroute.contains(&net_id) {
                    continue;
                }
                let topo = &net_topologies[net_id];
                let mut found = false;
                for path in topo {
                    for c in path {
                        if kill_zone.contains(&(c.x, c.y)) {
                            found = true;
                            break;
                        }
                    }
                    if found {
                        break;
                    }
                }
                if found {
                    nets_to_reroute.insert(net_id);
                }
            }
        }

        let mut nets_with_priority: Vec<(usize, u32)> = nets_to_reroute
            .into_iter()
            .map(|net_id| {
                let topo = &net_topologies[net_id];
                let congestion_score = if topo.is_empty() {
                    10u32
                } else {
                    let mut score = 0u32;
                    for path in topo {
                        for i in 0..path.len().saturating_sub(1) {
                            let a = path[i];
                            let b = path[i + 1];
                            if a.z == b.z {
                                if a.x != b.x {
                                    let min_x = a.x.min(b.x);
                                    if grid.is_h_congested(min_x, a.y, a.z) {
                                        score += 1;
                                    }
                                } else if a.y != b.y {
                                    let min_y = a.y.min(b.y);
                                    if grid.is_v_congested(a.x, min_y, a.z) {
                                        score += 1;
                                    }
                                }
                            }
                        }
                    }
                    score
                };
                (net_id, congestion_score + ripup_counts[net_id])
            })
            .collect();

        nets_with_priority.sort_by(|a, b| b.1.cmp(&a.1));
        let nets_vec: Vec<usize> = nets_with_priority.into_iter().map(|(id, _)| id).collect();

        for &net_id in &nets_vec {
            ripup_counts[net_id] += 1;
        }

        let mut candidates: Vec<(usize, (i32, i32, i32, i32))> = nets_vec
            .iter()
            .map(|&net_id| {
                let net = &db.nets[net_id];
                let topo = &net_topologies[net_id];
                let mut min_x = i32::MAX;
                let mut max_x = i32::MIN;
                let mut min_y = i32::MAX;
                let mut max_y = i32::MIN;

                if !topo.is_empty() {
                    for path in topo {
                        for c in path {
                            min_x = min_x.min(c.x as i32);
                            max_x = max_x.max(c.x as i32);
                            min_y = min_y.min(c.y as i32);
                            max_y = max_y.max(c.y as i32);
                        }
                    }
                } else {
                    for &pid in &net.pins {
                        let cid = db.pin_to_cell[pid.index()];
                        let pos = db.get_pin_position(pid, &db.positions[cid.index()]);
                        let g = converter.to_grid(pos, 0);
                        min_x = min_x.min(g.x as i32);
                        max_x = max_x.max(g.x as i32);
                        min_y = min_y.min(g.y as i32);
                        max_y = max_y.max(g.y as i32);
                    }
                }
                (net_id, (min_x - 2, max_x + 2, min_y - 2, max_y + 2))
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

            // Rip up all nets in the batch
            for &net_id in &batch {
                remove_topology_from_grid(&mut grid, &net_topologies[net_id]);
            }

            let coarse_max = coarse_converter.to_grid(
                pare_common::geom::point::Point::new(db.die_area.width(), db.die_area.height()),
                0,
            );
            let coarse_w = coarse_max.x + 1;
            let coarse_h = coarse_max.y + 1;

            let stat_pattern_hits = AtomicUsize::new(0);
            let stat_astar_calls = AtomicUsize::new(0);
            let stat_astar_fallback = AtomicUsize::new(0);
            let stat_astar_expansions = AtomicUsize::new(0);
            let stat_max_expansions = AtomicUsize::new(0);
            let results: Vec<(usize, Option<Vec<Vec<GridCoord>>>)> = batch
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
                        let net = &db.nets[net_id];
                        let res = if net.pins.len() >= LARGE_NET_PIN_THRESHOLD {
                            route_net_parallel(
                                net,
                                &grid,
                                solver,
                                &converter,
                                db,
                                collision_penalty,
                                oracle,
                                config,
                                ripup_counts[net_id],
                                use_strict,
                                grid_w,
                                grid_h,
                                coarse_w,
                                coarse_h,
                                layers,
                                &converter,
                                coarse_converter,
                                &guides[net_id],
                                net_id,
                                &stat_pattern_hits,
                                &stat_astar_calls,
                                &stat_astar_fallback,
                                &stat_astar_expansions,
                                &stat_max_expansions,
                            )
                        } else {
                            route_net_dr_pure(
                                net,
                                &grid,
                                solver,
                                &converter,
                                db,
                                collision_penalty,
                                oracle,
                                config,
                                ripup_counts[net_id],
                                use_strict,
                                &stat_pattern_hits,
                                &stat_astar_calls,
                                &stat_astar_fallback,
                                &stat_astar_expansions,
                                &stat_max_expansions,
                            )
                        };

                        let p = progress.fetch_add(1, Ordering::Relaxed) + 1;
                        if p.is_multiple_of(100) || p == total_ripped {
                            ui::progress(&format!("[DR Iter {}]", iter), p, total_ripped);
                        }

                        (net_id, res)
                    },
                )
                .collect();

            for (net_id, res) in results {
                if let Some(topology) = res {
                    add_topology_to_grid(&mut grid, &topology);
                    net_topologies[net_id] = topology;
                } else {
                    net_topologies[net_id].clear();
                }
            }
            candidates = remaining;
        }
        ui::progress_clear();

        // Per-layer breakdown periodically
        if iter % 5 == 0 || iter < 3 {
            log_layer_stats_compact(&grid, db, layers);
        }

        ui::routing_iter(
            "DR",
            iter,
            overflow,
            nets_vec.len(),
            collision_penalty,
            start.elapsed().as_millis(),
        );

        if nets_vec.is_empty() {
            break;
        }
        collision_penalty = (collision_penalty * config.penalty_multiplier).min(20000.0);
    }

    let origin_x = db.die_area.min.x;
    let origin_y = db.die_area.min.y;
    let gw = grid.gcell_w();
    let gh = grid.gcell_h();

    // Build track grids for snapping wire coordinates to legal tracks
    let track_grids = db.build_track_grids();

    // Generate segments from topology
    let mut all_segments = Vec::with_capacity(db.nets.len());
    for (net_id, topology) in net_topologies.iter().enumerate() {
        let mut segments = Vec::new();
        let net = &db.nets[net_id];
        let mut pin_locations: HashMap<(u32, u32, u8), Vec<Point<f64>>> = HashMap::new();

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

            pin_locations
                .entry((grid_pos.x, grid_pos.y, grid_pos.z))
                .or_default()
                .push(exact_pos);
        }

        if !topology.is_empty() {
            segments = generate_segments_from_topology(
                topology,
                &pin_locations,
                gw,
                gh,
                origin_x,
                origin_y,
                &track_grids,
                &db.layers,
            );
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

/// Log per-layer edge overflow statistics.
fn log_layer_stats(grid: &GCellGrid, db: &NetlistDB, layers: u8) {
    let w = grid.width();
    let h = grid.height();
    for z in 0..layers {
        let mut h_overflow = 0u32;
        let mut h_used = 0u32;
        let mut v_overflow = 0u32;
        let mut v_used = 0u32;
        let mut total_h_cap = 0u32;
        let mut total_v_cap = 0u32;

        for y in 0..h {
            for x in 0..(w - 1) {
                let usage = grid.h_edge_usage(x, y, z);
                let cap = grid.h_edge_cap(x, y, z);
                if usage > 0 {
                    h_used += 1;
                }
                if usage > cap {
                    h_overflow += (usage - cap) as u32;
                }
                total_h_cap += cap as u32;
            }
        }
        for y in 0..(h - 1) {
            for x in 0..w {
                let usage = grid.v_edge_usage(x, y, z);
                let cap = grid.v_edge_cap(x, y, z);
                if usage > 0 {
                    v_used += 1;
                }
                if usage > cap {
                    v_overflow += (usage - cap) as u32;
                }
                total_v_cap += cap as u32;
            }
        }

        if h_used > 0 || v_used > 0 || h_overflow > 0 || v_overflow > 0 {
            let name = if (z as usize) < db.layers.len() {
                db.layers[z as usize].name.as_str()
            } else {
                "?"
            };
            log::debug!(
                "  Layer {} ({}): h_overflow={}, v_overflow={}, h_used={}/{}, v_used={}/{}",
                z,
                name,
                h_overflow,
                v_overflow,
                h_used,
                total_h_cap,
                v_used,
                total_v_cap,
            );
        }
    }
}

/// Compact per-layer stats for iteration logging.
fn log_layer_stats_compact(grid: &GCellGrid, db: &NetlistDB, layers: u8) {
    let w = grid.width();
    let h = grid.height();
    let mut info = String::new();
    for z in 0..layers {
        let mut overflow = 0u32;
        let mut used = 0u32;

        for y in 0..h {
            for x in 0..(w - 1) {
                let u = grid.h_edge_usage(x, y, z);
                let c = grid.h_edge_cap(x, y, z);
                if u > c {
                    overflow += (u - c) as u32;
                }
                if u > 0 {
                    used += 1;
                }
            }
        }
        for y in 0..(h - 1) {
            for x in 0..w {
                let u = grid.v_edge_usage(x, y, z);
                let c = grid.v_edge_cap(x, y, z);
                if u > c {
                    overflow += (u - c) as u32;
                }
                if u > 0 {
                    used += 1;
                }
            }
        }

        if used > 0 || overflow > 0 {
            let name = if (z as usize) < db.layers.len() {
                &db.layers[z as usize].name
            } else {
                "?"
            };
            info.push_str(&format!(" {}:{}/{}", name, overflow, used));
        }
    }
    if !info.is_empty() {
        log::debug!("  Layers:{}", info);
    }
}
