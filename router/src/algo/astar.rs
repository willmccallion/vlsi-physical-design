use crate::grid::RoutingGrid;
use eda_common::db::core::{LayerDirection, NetlistDB};
use eda_common::geom::coord::GridCoord;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

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

pub trait GuideOracle {
    fn is_in_guide(&self, c: GridCoord) -> bool;
}

pub struct NoGuide;
impl GuideOracle for NoGuide {
    fn is_in_guide(&self, _c: GridCoord) -> bool {
        true
    }
}

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
    fn contains(&self, c: GridCoord) -> bool {
        c.x >= self.min_x && c.x <= self.max_x && c.y >= self.min_y && c.y <= self.max_y
    }
    #[inline(always)]
    fn get_local_idx(&self, c: GridCoord) -> usize {
        let lx = c.x - self.min_x;
        let ly = c.y - self.min_y;
        let lz = c.z as u32;
        (lz * self.width * self.height + ly * self.width + lx) as usize
    }
    #[inline(always)]
    fn get_coord(&self, idx: u32) -> GridCoord {
        let plane_size = self.width * self.height;
        let z = (idx / plane_size) as u8;
        let rem = idx % plane_size;
        let y = rem / self.width + self.min_y;
        let x = rem % self.width + self.min_x;
        GridCoord::new(x, y, z)
    }
}

#[derive(Clone)]
pub struct AStar {
    parents: Vec<u32>,
    g_score: Vec<i64>,
    visited_tag: Vec<u32>,
    current_tag: u32,
    capacity: usize,
}

impl AStar {
    pub fn new() -> Self {
        let cap = 100_000;
        Self {
            parents: vec![u32::MAX; cap],
            g_score: vec![i64::MAX; cap],
            visited_tag: vec![0; cap],
            current_tag: 1,
            capacity: cap,
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
        allowed_pins: &[GridCoord],
        max_expansions: u32,
        strict_mode: bool,
    ) -> Option<Vec<GridCoord>> {
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

        let guide_penalty = (500.0 + (collision_penalty * 2.0)) * scale;
        let layer_change_cost = 10.0 * scale;
        let wrong_dir_cost = 25.0 * scale;
        let base_move_cost = 1.0 * scale;
        let mut expansions = 0;

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
                return Some(self.reconstruct_path(end, &window));
            }

            expansions += 1;
            if expansions > max_expansions {
                return None;
            }

            let current_g = self.g_score[curr_local];
            let mut neighbors = [GridCoord { x: 0, y: 0, z: 0 }; 6];
            let mut n_count = 0;
            if position.x > 0 {
                neighbors[n_count] = GridCoord::new(position.x - 1, position.y, position.z);
                n_count += 1;
            }
            if position.x < grid.width() - 1 {
                neighbors[n_count] = GridCoord::new(position.x + 1, position.y, position.z);
                n_count += 1;
            }
            if position.y > 0 {
                neighbors[n_count] = GridCoord::new(position.x, position.y - 1, position.z);
                n_count += 1;
            }
            if position.y < grid.height() - 1 {
                neighbors[n_count] = GridCoord::new(position.x, position.y + 1, position.z);
                n_count += 1;
            }
            if position.z > 0 {
                neighbors[n_count] = GridCoord::new(position.x, position.y, position.z - 1);
                n_count += 1;
            }
            if position.z < grid.layers() - 1 {
                neighbors[n_count] = GridCoord::new(position.x, position.y, position.z + 1);
                n_count += 1;
            }

            for i in 0..n_count {
                let neighbor = neighbors[i];
                if !window.contains(neighbor) {
                    continue;
                }

                if strict_mode && !oracle.is_in_guide(neighbor) && neighbor != end {
                    continue;
                }

                if grid.is_obstacle(neighbor) && neighbor != end {
                    let is_allowed = allowed_pins.iter().any(|&p| {
                        p == neighbor
                            || (p.z == neighbor.z
                                && ((p.x as i32 - neighbor.x as i32).abs()
                                    + (p.y as i32 - neighbor.y as i32).abs()
                                    <= 1))
                    });

                    if !is_allowed {
                        continue;
                    }
                }

                let is_pin = neighbor == end;
                let step_guide_cost = if is_pin || oracle.is_in_guide(neighbor) {
                    0.0
                } else {
                    guide_penalty
                };

                // Check if we are very close to the target pin (Manhattan distance < 2).
                let dist_to_end =
                    (neighbor.x as i32 - end_x).abs() + (neighbor.y as i32 - end_y).abs();
                let is_near_pin = dist_to_end < 2;

                let mut step_move_cost = base_move_cost;
                if position.z != neighbor.z {
                    step_move_cost = layer_change_cost;
                    if position.z == 0 || neighbor.z == 0 {
                        step_move_cost = 1.0;
                    }
                } else if (position.z as usize) < db.layers.len() {
                    match db.layers[position.z as usize].direction {
                        LayerDirection::Vertical if position.x != neighbor.x => {
                            // If we are right next to the pin, allow wrong-way routing cheaply
                            step_move_cost = if is_near_pin {
                                base_move_cost * 2.0
                            } else {
                                wrong_dir_cost
                            };
                        }
                        LayerDirection::Horizontal if position.y != neighbor.y => {
                            // If we are right next to the pin, allow wrong-way routing cheaply
                            step_move_cost = if is_near_pin {
                                base_move_cost * 2.0
                            } else {
                                wrong_dir_cost
                            };
                        }
                        _ => {}
                    }
                }

                if position.z == 0 && neighbor.z == 0 {
                    step_move_cost += 1000.0;
                }

                let node_cost = if neighbor == end {
                    0.0
                } else {
                    (grid.get_cost(neighbor, collision_penalty) - 1.0) * scale
                };
                let total_cost = step_move_cost + step_guide_cost + node_cost;
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
        }
        None
    }

    #[inline(always)]
    fn heuristic(&self, a: GridCoord, ex: i32, ey: i32, ez: i32, weight: f64) -> f64 {
        ((a.x as i32 - ex).abs() as f64
            + (a.y as i32 - ey).abs() as f64
            + (a.z as i32 - ez).abs() as f64 * 5.0)
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
