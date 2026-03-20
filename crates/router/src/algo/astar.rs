//! A* Pathfinding Algorithm for Edge-Based `GCell` Routing.
//!
//! Implements A* on an edge-based gcell graph where direction is structurally
//! enforced. Horizontal layers only allow horizontal moves (using `h_edge_cost`),
//! vertical layers only allow vertical moves (using `v_edge_cost`), and vias
//! allow layer transitions at the same gcell.

use crate::grid::RoutingGrid;
use pare_common::db::core::{LayerDirection, NetlistDB};
use pare_common::geom::coord::GridCoord;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// State node in the A* search priority queue.
#[derive(Copy, Clone, Eq, PartialEq)]
struct State {
    f_score: i64,
    g_score: i64,
    index: u32,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .cmp(&self.f_score)
            .then_with(|| self.g_score.cmp(&other.g_score))
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Trait for guide constraint checking during routing.
pub trait GuideOracle {
    /// Returns true if the given coordinate is within the routing guide.
    fn is_in_guide(&self, c: GridCoord) -> bool;
}

/// Dummy guide oracle that allows routing anywhere.
#[derive(Clone, Copy, Debug)]
pub struct NoGuide;
impl GuideOracle for NoGuide {
    fn is_in_guide(&self, _c: GridCoord) -> bool {
        true
    }
}

/// A 3D bounding box that limits the A* search space on the gcell grid.
#[derive(Clone, Copy)]
struct RoutingWindow {
    min_x: u32,
    max_x: u32,
    min_y: u32,
    max_y: u32,
    width: u32,
    height: u32,
    layers: u8,
}

impl RoutingWindow {
    fn new(
        starts: &[GridCoord],
        end: GridCoord,
        margin: u32,
        grid_w: u32,
        grid_h: u32,
        layers: u8,
    ) -> Self {
        let mut min_x = end.x;
        let mut max_x = end.x;
        let mut min_y = end.y;
        let mut max_y = end.y;
        for s in starts {
            min_x = min_x.min(s.x);
            max_x = max_x.max(s.x);
            min_y = min_y.min(s.y);
            max_y = max_y.max(s.y);
        }
        let min_x = min_x.saturating_sub(margin);
        let max_x = (max_x + margin).min(grid_w - 1);
        let min_y = min_y.saturating_sub(margin);
        let max_y = (max_y + margin).min(grid_h - 1);
        Self {
            min_x,
            max_x,
            min_y,
            max_y,
            width: max_x - min_x + 1,
            height: max_y - min_y + 1,
            layers,
        }
    }

    #[inline(always)]
    const fn contains(&self, c: GridCoord) -> bool {
        c.x >= self.min_x && c.x <= self.max_x && c.y >= self.min_y && c.y <= self.max_y
    }

    #[inline(always)]
    const fn get_local_idx(&self, c: GridCoord) -> usize {
        let lx = c.x - self.min_x;
        let ly = c.y - self.min_y;
        let lz = c.z as u32;
        (lz * self.width * self.height + ly * self.width + lx) as usize
    }

    #[inline(always)]
    const fn get_coord(&self, idx: u32) -> GridCoord {
        let plane_size = self.width * self.height;
        let z = (idx / plane_size) as u8;
        let rem = idx % plane_size;
        let y = rem / self.width + self.min_y;
        let x = rem % self.width + self.min_x;
        GridCoord::new(x, y, z)
    }
}

/// A* pathfinding solver for edge-based gcell routing.
#[derive(Clone, Debug)]
pub struct AStar {
    parents: Vec<u32>,
    g_score: Vec<i64>,
    visited_tag: Vec<u32>,
    current_tag: u32,
    capacity: usize,
    /// Number of expansions in the last `find_path` call.
    pub last_expansions: u32,
}

impl Default for AStar {
    fn default() -> Self {
        Self::new()
    }
}

impl AStar {
    /// Creates a new A* solver with a default internal capacity.
    pub fn new() -> Self {
        let cap = 100_000;
        Self {
            parents: vec![u32::MAX; cap],
            g_score: vec![i64::MAX; cap],
            visited_tag: vec![0; cap],
            current_tag: 1,
            capacity: cap,
            last_expansions: 0,
        }
    }

    fn ensure_capacity(&mut self, size: usize) {
        if size > self.capacity {
            self.capacity = size.max(self.capacity * 2);
            self.parents.resize(self.capacity, u32::MAX);
            self.g_score.resize(self.capacity, i64::MAX);
            self.visited_tag.resize(self.capacity, 0);
        }
    }

    fn reset_window(&mut self) {
        self.current_tag += 1;
        if self.current_tag == 0 {
            self.visited_tag.fill(0);
            self.current_tag = 1;
        }
    }

    /// Finds a path on the edge-based gcell graph.
    ///
    /// Direction is structurally enforced:
    /// - Horizontal layers allow moves in ±X (using `h_edge_cost`)
    /// - Vertical layers allow moves in ±Y (using `v_edge_cost`)
    /// - Vias allow layer changes at the same gcell
    /// - Near pins (within 1 gcell), any direction is allowed for pin access
    #[allow(clippy::too_many_arguments)]
    pub fn find_path<G: RoutingGrid + ?Sized, O: GuideOracle>(
        &mut self,
        grid: &G,
        db: &NetlistDB,
        starts: &[GridCoord],
        end: GridCoord,
        collision_penalty: f64,
        heuristic_weight: f64,
        base_margin: u32,
        margin_multiplier: f64,
        oracle: &O,
        _allowed_pins: &[GridCoord],
        max_expansions: u32,
        strict_mode: bool,
    ) -> Option<Vec<GridCoord>> {
        self.last_expansions = 0;
        if starts.is_empty() {
            return None;
        }

        let window = RoutingWindow::new(
            starts,
            end,
            (base_margin as f64 * margin_multiplier) as u32,
            grid.width(),
            grid.height(),
            grid.layers(),
        );
        self.ensure_capacity((window.width * window.height * window.layers as u32) as usize);
        self.reset_window();

        let mut heap = BinaryHeap::new();
        let end_x = end.x as i32;
        let end_y = end.y as i32;
        let end_z = end.z as i32;
        let scale = 100.0;

        for &start in starts {
            if !window.contains(start) {
                continue;
            }
            let start_local = window.get_local_idx(start);
            self.g_score[start_local] = 0;
            self.visited_tag[start_local] = self.current_tag;
            self.parents[start_local] = u32::MAX;
            let h = self.heuristic(start, end_x, end_y, end_z, heuristic_weight);
            heap.push(State {
                f_score: (h * scale) as i64,
                g_score: 0,
                index: start_local as u32,
            });
        }

        let guide_penalty = collision_penalty.mul_add(0.5, 10.0) * scale;
        let mut expansions = 0u32;

        while let Some(State { f_score, index, .. }) = heap.pop() {
            let curr_local = index as usize;
            if f_score
                > self.g_score[curr_local]
                    + (self.heuristic_fast(index, &window, end_x, end_y, end_z, heuristic_weight)
                        * scale) as i64
            {
                continue;
            }
            let position = window.get_coord(index);
            if position == end {
                self.last_expansions = expansions;
                return Some(self.reconstruct_path(end, &window));
            }

            expansions += 1;
            if expansions > max_expansions {
                self.last_expansions = expansions;
                return None;
            }

            let current_g = self.g_score[curr_local];

            // Determine what moves are allowed on this layer
            let on_layer_0 = position.z == 0;
            let near_pin = {
                let dx = (position.x as i32 - end_x).abs();
                let dy = (position.y as i32 - end_y).abs();
                dx + dy <= 1
            } || starts.iter().any(|s| {
                (position.x as i32 - s.x as i32).abs() + (position.y as i32 - s.y as i32).abs()
                    <= 1
            });

            // Structurally enforce direction:
            // - Horizontal layers allow X movement
            // - Vertical layers allow Y movement
            // - Near pins or on layer 0: allow both for pin access
            let layer_dir = if on_layer_0 || near_pin {
                None // allow both
            } else if (position.z as usize) < db.layers.len() {
                Some(db.layers[position.z as usize].direction)
            } else {
                None
            };

            let allow_x = match layer_dir {
                None => true,
                Some(LayerDirection::Horizontal | LayerDirection::Unknown) => true,
                Some(LayerDirection::Vertical) => false,
            };

            let allow_y = match layer_dir {
                None => true,
                Some(LayerDirection::Vertical | LayerDirection::Unknown) => true,
                Some(LayerDirection::Horizontal) => false,
            };

            // Horizontal neighbors (±X) - uses h_edge_cost
            if allow_x {
                // Move left: edge from (x-1, y) to (x, y)
                if position.x > 0 {
                    let neighbor = GridCoord::new(position.x - 1, position.y, position.z);
                    if window.contains(neighbor) {
                        let edge_cost =
                            grid.h_edge_cost(position.x - 1, position.y, position.z, collision_penalty);
                        if edge_cost < f64::MAX / 2.0 {
                            self.try_push(
                                neighbor, current_g, edge_cost, scale, guide_penalty,
                                oracle, strict_mode, end, &window, end_x, end_y, end_z,
                                heuristic_weight, curr_local, &mut heap,
                            );
                        }
                    }
                }
                // Move right: edge from (x, y) to (x+1, y)
                if position.x < grid.width() - 1 {
                    let neighbor = GridCoord::new(position.x + 1, position.y, position.z);
                    if window.contains(neighbor) {
                        let edge_cost =
                            grid.h_edge_cost(position.x, position.y, position.z, collision_penalty);
                        if edge_cost < f64::MAX / 2.0 {
                            self.try_push(
                                neighbor, current_g, edge_cost, scale, guide_penalty,
                                oracle, strict_mode, end, &window, end_x, end_y, end_z,
                                heuristic_weight, curr_local, &mut heap,
                            );
                        }
                    }
                }
            }

            // Vertical neighbors (±Y) - uses v_edge_cost
            if allow_y {
                // Move up: edge from (x, y-1) to (x, y)
                if position.y > 0 {
                    let neighbor = GridCoord::new(position.x, position.y - 1, position.z);
                    if window.contains(neighbor) {
                        let edge_cost =
                            grid.v_edge_cost(position.x, position.y - 1, position.z, collision_penalty);
                        if edge_cost < f64::MAX / 2.0 {
                            self.try_push(
                                neighbor, current_g, edge_cost, scale, guide_penalty,
                                oracle, strict_mode, end, &window, end_x, end_y, end_z,
                                heuristic_weight, curr_local, &mut heap,
                            );
                        }
                    }
                }
                // Move down: edge from (x, y) to (x, y+1)
                if position.y < grid.height() - 1 {
                    let neighbor = GridCoord::new(position.x, position.y + 1, position.z);
                    if window.contains(neighbor) {
                        let edge_cost =
                            grid.v_edge_cost(position.x, position.y, position.z, collision_penalty);
                        if edge_cost < f64::MAX / 2.0 {
                            self.try_push(
                                neighbor, current_g, edge_cost, scale, guide_penalty,
                                oracle, strict_mode, end, &window, end_x, end_y, end_z,
                                heuristic_weight, curr_local, &mut heap,
                            );
                        }
                    }
                }
            }

            // Via neighbors (±Z) - always allowed
            if position.z > 0 {
                let neighbor = GridCoord::new(position.x, position.y, position.z - 1);
                let via_c = grid.via_cost(position.x, position.y, position.z - 1);
                self.try_push(
                    neighbor, current_g, via_c, scale, guide_penalty,
                    oracle, strict_mode, end, &window, end_x, end_y, end_z,
                    heuristic_weight, curr_local, &mut heap,
                );
            }
            if position.z < grid.layers() - 1 {
                let neighbor = GridCoord::new(position.x, position.y, position.z + 1);
                let via_c = grid.via_cost(position.x, position.y, position.z);
                self.try_push(
                    neighbor, current_g, via_c, scale, guide_penalty,
                    oracle, strict_mode, end, &window, end_x, end_y, end_z,
                    heuristic_weight, curr_local, &mut heap,
                );
            }
        }
        self.last_expansions = expansions;
        None
    }

    /// Try to push a neighbor into the priority queue.
    #[inline(always)]
    #[allow(clippy::too_many_arguments)]
    fn try_push<O: GuideOracle>(
        &mut self,
        neighbor: GridCoord,
        current_g: i64,
        edge_cost: f64,
        scale: f64,
        guide_penalty: f64,
        oracle: &O,
        strict_mode: bool,
        end: GridCoord,
        window: &RoutingWindow,
        end_x: i32,
        end_y: i32,
        end_z: i32,
        heuristic_weight: f64,
        curr_local: usize,
        heap: &mut BinaryHeap<State>,
    ) {
        if !window.contains(neighbor) {
            return;
        }

        let is_pin = neighbor == end;

        if strict_mode && !oracle.is_in_guide(neighbor) && !is_pin {
            return;
        }

        let step_guide_cost = if is_pin || oracle.is_in_guide(neighbor) {
            0.0
        } else {
            guide_penalty
        };

        let total_cost = edge_cost.mul_add(scale, step_guide_cost);
        let tentative_g = current_g + total_cost as i64;
        let neighbor_local = window.get_local_idx(neighbor);

        if self.visited_tag[neighbor_local] != self.current_tag
            || tentative_g < self.g_score[neighbor_local]
        {
            self.parents[neighbor_local] = curr_local as u32;
            self.g_score[neighbor_local] = tentative_g;
            self.visited_tag[neighbor_local] = self.current_tag;
            let h = self.heuristic(neighbor, end_x, end_y, end_z, heuristic_weight);
            heap.push(State {
                f_score: tentative_g + (h * scale) as i64,
                g_score: tentative_g,
                index: neighbor_local as u32,
            });
        }
    }

    #[inline(always)]
    fn heuristic(&self, a: GridCoord, ex: i32, ey: i32, ez: i32, weight: f64) -> f64 {
        let _ = self;
        ((a.x as i32 - ex).abs() as f64
            + (a.y as i32 - ey).abs() as f64
            + (a.z as i32 - ez).abs() as f64)
            * weight
    }

    #[inline(always)]
    fn heuristic_fast(
        &self,
        idx: u32,
        w: &RoutingWindow,
        ex: i32,
        ey: i32,
        ez: i32,
        weight: f64,
    ) -> f64 {
        self.heuristic(w.get_coord(idx), ex, ey, ez, weight)
    }

    fn reconstruct_path(&self, end: GridCoord, window: &RoutingWindow) -> Vec<GridCoord> {
        let mut path = Vec::new();
        let mut curr_local = window.get_local_idx(end);
        loop {
            path.push(window.get_coord(curr_local as u32));
            let parent = self.parents[curr_local];
            if parent == u32::MAX {
                break;
            }
            curr_local = parent as usize;
        }
        path.reverse();
        path
    }
}
