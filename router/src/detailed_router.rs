use crate::algo::astar::{AStar, GuideOracle, NoGuide};
use crate::grid::RoutingGrid;
use crate::grid::dense::DenseGrid;
use crate::utils::conversion::GridConverter;
use eda_common::db::core::{NetlistDB, RouteSegment};
use eda_common::geom::coord::GridCoord;
use eda_common::util::config::DetailedRoutingConfig;
use rand::seq::SliceRandom;
use rand::thread_rng;
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};
use std::io::Write;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

#[derive(Clone)]
struct FastGuideOracle {
    x_map: Vec<u32>,
    y_map: Vec<u32>,
    grid: Vec<u32>,
    coarse_w: u32,
    coarse_h: u32,
    current_net_id: u32,
    allow_all: bool,
}

impl FastGuideOracle {
    fn new(
        fine_w: u32,
        fine_h: u32,
        coarse_w: u32,
        coarse_h: u32,
        layers: u8,
        fine_conv: &GridConverter,
        coarse_conv: &GridConverter,
    ) -> Self {
        let mut x_map = vec![0; fine_w as usize];
        let mut y_map = vec![0; fine_h as usize];

        for x in 0..fine_w {
            let fine_coord = GridCoord::new(x, 0, 0);
            let world_pos = fine_conv.to_world(fine_coord);
            let coarse_coord = coarse_conv.to_grid(world_pos, 0);
            x_map[x as usize] = coarse_coord.x.min(coarse_w - 1);
        }

        for y in 0..fine_h {
            let fine_coord = GridCoord::new(0, y, 0);
            let world_pos = fine_conv.to_world(fine_coord);
            let coarse_coord = coarse_conv.to_grid(world_pos, 0);
            y_map[y as usize] = coarse_coord.y.min(coarse_h - 1);
        }

        let size = (coarse_w * coarse_h * layers as u32) as usize;
        Self {
            x_map,
            y_map,
            grid: vec![0; size],
            coarse_w,
            coarse_h,
            current_net_id: 0,
            allow_all: false,
        }
    }

    fn prepare(&mut self, net_id: usize, guides: &HashSet<GridCoord>) {
        if guides.is_empty() {
            self.allow_all = true;
            return;
        }
        self.allow_all = false;

        self.current_net_id = (net_id as u32) + 1;
        if self.current_net_id == 0 {
            self.grid.fill(0);
            self.current_net_id = 1;
        }

        let w = self.coarse_w as i32;
        let h = self.coarse_h as i32;

        for &g in guides {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let nx = g.x as i32 + dx;
                    let ny = g.y as i32 + dy;

                    if nx >= 0 && nx < w && ny >= 0 && ny < h {
                        let idx = (g.z as u32 * self.coarse_w * self.coarse_h
                            + (ny as u32) * self.coarse_w
                            + (nx as u32)) as usize;
                        if idx < self.grid.len() {
                            self.grid[idx] = self.current_net_id;
                        }
                    }
                }
            }
        }
    }
}

impl GuideOracle for FastGuideOracle {
    #[inline(always)]
    fn is_in_guide(&self, c: GridCoord) -> bool {
        if self.allow_all {
            return true;
        }
        let cx = unsafe { *self.x_map.get_unchecked(c.x as usize) };
        let cy = unsafe { *self.y_map.get_unchecked(c.y as usize) };
        let idx = (c.z as u32 * self.coarse_w * self.coarse_h + cy * self.coarse_w + cx) as usize;
        unsafe { *self.grid.get_unchecked(idx) == self.current_net_id }
    }
}

struct SpatialSet {
    grid: Vec<u32>,
    w: u32,
    h: u32,
    bin_size: u32,
    current_batch: u32,
}

impl SpatialSet {
    fn new(width: u32, height: u32, bin_size: u32) -> Self {
        let w = (width + bin_size - 1) / bin_size;
        let h = (height + bin_size - 1) / bin_size;
        Self {
            grid: vec![0; (w * h) as usize],
            w,
            h,
            bin_size,
            current_batch: 0,
        }
    }

    fn reset(&mut self) {
        self.current_batch += 1;
        if self.current_batch == 0 {
            self.grid.fill(0);
            self.current_batch = 1;
        }
    }

    fn try_insert(&mut self, bbox: &(i32, i32, i32, i32)) -> bool {
        let min_x = (bbox.0.max(0) as u32) / self.bin_size;
        let max_x = (bbox.1.max(0) as u32) / self.bin_size;
        let min_y = (bbox.2.max(0) as u32) / self.bin_size;
        let max_y = (bbox.3.max(0) as u32) / self.bin_size;

        let limit_x = self.w - 1;
        let limit_y = self.h - 1;

        for y in min_y..=max_y.min(limit_y) {
            for x in min_x..=max_x.min(limit_x) {
                if self.grid[(y * self.w + x) as usize] == self.current_batch {
                    return false;
                }
            }
        }

        for y in min_y..=max_y.min(limit_y) {
            for x in min_x..=max_x.min(limit_x) {
                self.grid[(y * self.w + x) as usize] = self.current_batch;
            }
        }
        true
    }
}

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
    let converter = GridConverter::from_steps(step_x, step_y, off_x, off_y, grid_w, grid_h);

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
        let batch_penalty = if b_idx == 0 { 0.5 } else { 1.5 };
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
                        100.0,
                        config,
                        0,
                        true,
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

    let mut collision_penalty = config.initial_penalty.max(2.5);
    let history_increment = config.history_increment.max(1.0);
    let mut last_conflicts = usize::MAX;
    let mut stagnation_counter = 0;
    let mut spatial_set = SpatialSet::new(grid_w, grid_h, 60);

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

        let mut nets_vec: Vec<usize> = nets_to_reroute.into_iter().collect();
        nets_vec.shuffle(&mut thread_rng());

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
                        let res = route_net_dr_pure(
                            &db.nets[net_id],
                            &grid,
                            solver,
                            &converter,
                            db,
                            collision_penalty,
                            oracle,
                            1.0,
                            config,
                            ripup_counts[net_id],
                            false, // STRICT MODE DISABLED for Rip-up
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

fn generate_segments_from_topology(
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

fn route_net_dr_pure<O: GuideOracle>(
    net: &eda_common::db::core::NetData,
    grid: &DenseGrid,
    solver: &mut AStar,
    converter: &GridConverter,
    db: &NetlistDB,
    penalty: f64,
    oracle: &O,
    _pattern_threshold: f64,
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

    let margin_multiplier = 1.0 + (ripup_count as f64 * 0.2);
    let max_expansions = 300_000 + (ripup_count * 10_000);

    for i in 1..sorted_indices.len() {
        let next_pin_idx = sorted_indices[i];
        let target = pin_coords[next_pin_idx];

        let path_opt = solver
            .find_path(
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
            )
            .or_else(|| {
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
                    max_expansions * 2,
                    strict_mode,
                )
            });

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
