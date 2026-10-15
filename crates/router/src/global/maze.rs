//! Maze routing of one net through the 3D gcell grid: each pin joins the
//! tree built so far by an A* search confined to a window around the two
//! pins it connects, widened only when the window holds no path.

use super::{GCell, GlobalCosts, GlobalGrid, window};
use crate::tree::connection_order;
use pare_common::db::indices::LayerId;
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BinaryHeap;

/// Edge, in gcells, of the buckets used to find tree cells near a connection.
const BUCKET: u16 = 4;

/// A tree through every terminal, as the paths that build it. Each
/// connection searches the box around its two ends grown by `margin`, and
/// a larger box only when that holds no path.
pub(super) fn route_tree(
    grid: &GlobalGrid,
    terminals: &[GCell],
    margin: u16,
    costs: GlobalCosts,
) -> Vec<Vec<GCell>> {
    let mut terminals: Vec<GCell> = terminals.to_vec();
    terminals.sort_unstable();
    terminals.dedup();
    if terminals.is_empty() {
        return Vec::new();
    }
    let points: Vec<Rect<Dbu>> = terminals
        .iter()
        .map(|c| {
            let p = Point::new(Dbu(i64::from(c.x)), Dbu(i64::from(c.y)));
            Rect::new(p, p)
        })
        .collect();
    let (first, joins) = connection_order(&points);

    let mut tree: FxHashSet<GCell> = FxHashSet::default();
    let mut buckets: FxHashMap<(u16, u16), Vec<GCell>> = FxHashMap::default();
    add_cell(&mut tree, &mut buckets, terminals[first]);
    let mut paths = vec![vec![terminals[first]]];
    let full = (0, grid.nx - 1, 0, grid.ny - 1);

    for join in joins {
        let target = terminals[join.pin];
        if tree.contains(&target) {
            continue;
        }
        let mut margin = margin;
        let path = loop {
            let w = window(&[target, terminals[join.parent]], margin, grid.nx, grid.ny);
            let sources = cells_in(&buckets, w);
            if !sources.is_empty()
                && let Some(p) = search(grid, &sources, target, w, costs)
            {
                break p;
            }
            if w == full {
                break vec![target];
            }
            margin = margin.saturating_mul(4).max(1);
        };
        for &c in &path {
            add_cell(&mut tree, &mut buckets, c);
        }
        paths.push(path);
    }
    paths
}

fn add_cell(
    tree: &mut FxHashSet<GCell>,
    buckets: &mut FxHashMap<(u16, u16), Vec<GCell>>,
    c: GCell,
) {
    if tree.insert(c) {
        buckets
            .entry((c.x / BUCKET, c.y / BUCKET))
            .or_default()
            .push(c);
    }
}

/// Tree cells inside a window.
fn cells_in(buckets: &FxHashMap<(u16, u16), Vec<GCell>>, w: (u16, u16, u16, u16)) -> Vec<GCell> {
    let mut out = Vec::new();
    for bx in w.0 / BUCKET..=w.1 / BUCKET {
        for by in w.2 / BUCKET..=w.3 / BUCKET {
            for &c in buckets.get(&(bx, by)).into_iter().flatten() {
                if c.x >= w.0 && c.x <= w.1 && c.y >= w.2 && c.y <= w.3 {
                    out.push(c);
                }
            }
        }
    }
    out
}

#[derive(Clone, Copy, PartialEq)]
struct Entry {
    f: f64,
    g: f64,
    cell: GCell,
}

impl Eq for Entry {}

impl Ord for Entry {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .f
            .total_cmp(&self.f)
            .then_with(|| self.cell.cmp(&other.cell))
    }
}

impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// A* state over every gcell of a grid, kept per thread between searches
/// and cleared by bumping a generation stamp.
#[derive(Debug, Default)]
struct SearchState {
    cost: Vec<f64>,
    parent: Vec<u32>,
    stamp: Vec<u32>,
    generation: u32,
}

/// Marks a node without a parent.
const NO_PARENT: u32 = u32::MAX;

impl SearchState {
    /// Forgets every node, sized for `nodes` nodes.
    fn reset(&mut self, nodes: usize) {
        if self.stamp.len() < nodes {
            self.cost.resize(nodes, 0.0);
            self.parent.resize(nodes, NO_PARENT);
            self.stamp.resize(nodes, 0);
        }
        self.generation = self.generation.wrapping_add(1);
        if self.generation == 0 {
            self.stamp.fill(0);
            self.generation = 1;
        }
    }

    fn cost(&self, node: usize) -> Option<f64> {
        (self.stamp[node] == self.generation).then(|| self.cost[node])
    }

    fn set(&mut self, node: usize, cost: f64, parent: u32) {
        self.stamp[node] = self.generation;
        self.cost[node] = cost;
        self.parent[node] = parent;
    }
}

thread_local! {
    static SEARCH: std::cell::RefCell<SearchState> = std::cell::RefCell::new(SearchState::default());
}

/// Cheapest path from any source to `target` inside the window.
fn search(
    grid: &GlobalGrid,
    sources: &[GCell],
    target: GCell,
    win: (u16, u16, u16, u16),
    costs: GlobalCosts,
) -> Option<Vec<GCell>> {
    SEARCH.with_borrow_mut(|state| search_with(state, grid, sources, target, win, costs))
}

fn search_with(
    state: &mut SearchState,
    grid: &GlobalGrid,
    sources: &[GCell],
    target: GCell,
    win: (u16, u16, u16, u16),
    costs: GlobalCosts,
) -> Option<Vec<GCell>> {
    let h = |c: GCell| {
        let layers = c.layer.index().abs_diff(target.layer.index()) as f64;
        let plane = f64::from(c.x.abs_diff(target.x) + c.y.abs_diff(target.y));
        costs.via.mul_add(layers, plane)
    };
    state.reset(grid.usage.len());
    let mut heap = BinaryHeap::new();
    for &c in sources {
        state.set(grid.edge(c), 0.0, NO_PARENT);
        heap.push(Entry {
            f: h(c),
            g: 0.0,
            cell: c,
        });
    }
    while let Some(Entry { g, cell, .. }) = heap.pop() {
        let here = grid.edge(cell);
        if state.cost(here).is_some_and(|b| g > b) {
            continue;
        }
        if cell == target {
            let mut path = vec![cell];
            let mut node = state.parent[here];
            while node != NO_PARENT {
                path.push(grid.cell_of(node as usize));
                node = state.parent[node as usize];
            }
            path.reverse();
            return Some(path);
        }
        let mut push = |next: GCell, step: f64| {
            let ng = g + step;
            let index = grid.edge(next);
            if state.cost(index).is_none_or(|b| ng < b) {
                state.set(index, ng, crate::tech::to_u32(here));
                heap.push(Entry {
                    f: ng + h(next),
                    g: ng,
                    cell: next,
                });
            }
        };
        let (x, y) = (cell.x, cell.y);
        match grid.direction(cell.layer) {
            Direction::Horizontal => {
                if x > win.0 {
                    let next = GCell { x: x - 1, ..cell };
                    push(next, grid.edge_cost(grid.edge(next), costs));
                }
                if x < win.1 {
                    push(
                        GCell { x: x + 1, ..cell },
                        grid.edge_cost(grid.edge(cell), costs),
                    );
                }
            }
            Direction::Vertical => {
                if y > win.2 {
                    let next = GCell { y: y - 1, ..cell };
                    push(next, grid.edge_cost(grid.edge(next), costs));
                }
                if y < win.3 {
                    push(
                        GCell { y: y + 1, ..cell },
                        grid.edge_cost(grid.edge(cell), costs),
                    );
                }
            }
        }
        let l = cell.layer.index();
        if l > *grid.layers().start() {
            push(
                GCell {
                    layer: LayerId::new(l - 1),
                    ..cell
                },
                costs.via,
            );
        }
        if l < *grid.layers().end() {
            push(
                GCell {
                    layer: LayerId::new(l + 1),
                    ..cell
                },
                costs.via,
            );
        }
    }
    None
}
