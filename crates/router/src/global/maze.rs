//! Maze routing of one net through the 3D gcell grid: each pin joins the
//! tree built so far by an A* search confined to a window around the two
//! pins it connects, widened only when the window holds no path. Edges that
//! fixed metal blocks outright are never crossed and cells with no room for
//! a via pad are never climbed into, so a net detours around a macro
//! however far that takes it; only a pin no detour reaches crosses them,
//! and then as few of them as possible.

use super::{BLOCKED_COST, GCell, GlobalCosts, GlobalGrid, GlobalNet, window};
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

/// A tree through every terminal of `net`, as the paths that build it.
/// Each connection searches the box around its two ends grown by `margin`,
/// and a larger box only when that holds no path.
pub(super) fn route_tree(
    grid: &GlobalGrid,
    net: &GlobalNet,
    margin: u16,
    costs: GlobalCosts,
) -> Vec<Vec<GCell>> {
    let mut terminals: Vec<GCell> = net.terminals.clone();
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
            let avoid = Query {
                net,
                costs,
                blocked: Blocked::Avoid,
            };
            if !sources.is_empty()
                && let Some(p) = search(grid, avoid, &sources, target, w)
            {
                break p;
            }
            if w == full {
                let all = cells_in(&buckets, full);
                let cross = Query {
                    blocked: Blocked::Cross,
                    ..avoid
                };
                break search(grid, cross, &all, target, full).unwrap_or_else(|| vec![target]);
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

/// Whether a search may cross edges and climb into cells that fixed metal
/// blocks outright.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Blocked {
    Avoid,
    Cross,
}

/// What one search is for and how it prices the grid.
#[derive(Clone, Copy)]
struct Query<'a> {
    net: &'a GlobalNet,
    costs: GlobalCosts,
    blocked: Blocked,
}

/// Cheapest path from any source to `target` inside the window.
fn search(
    grid: &GlobalGrid,
    query: Query<'_>,
    sources: &[GCell],
    target: GCell,
    win: (u16, u16, u16, u16),
) -> Option<Vec<GCell>> {
    SEARCH.with_borrow_mut(|state| search_with(state, grid, query, sources, target, win))
}

fn search_with(
    state: &mut SearchState,
    grid: &GlobalGrid,
    query: Query<'_>,
    sources: &[GCell],
    target: GCell,
    win: (u16, u16, u16, u16),
) -> Option<Vec<GCell>> {
    let Query {
        net,
        costs,
        blocked,
    } = query;
    let passable = |e: usize| blocked == Blocked::Cross || !grid.blocked[e];
    let climbable = |c: GCell| blocked == Blocked::Cross || !grid.via_blocked(c);
    let edge_cost = |e: usize| {
        let demand = net.demand(grid.layer_offset(grid.cell_of(e).layer));
        let crossing = if grid.blocked[e] { BLOCKED_COST } else { 0.0 };
        grid.edge_cost(e, demand, costs) + crossing
    };
    let climb_cost = |c: GCell| {
        let crossing = if grid.via_blocked(c) {
            BLOCKED_COST
        } else {
            0.0
        };
        costs.via + crossing
    };
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
                    let e = grid.edge(next);
                    if passable(e) {
                        push(next, edge_cost(e));
                    }
                }
                if x < win.1 && passable(here) {
                    push(GCell { x: x + 1, ..cell }, edge_cost(here));
                }
            }
            Direction::Vertical => {
                if y > win.2 {
                    let next = GCell { y: y - 1, ..cell };
                    let e = grid.edge(next);
                    if passable(e) {
                        push(next, edge_cost(e));
                    }
                }
                if y < win.3 && passable(here) {
                    push(GCell { y: y + 1, ..cell }, edge_cost(here));
                }
            }
        }
        let l = cell.layer.index();
        if l > *grid.layers().start() {
            let next = GCell {
                layer: LayerId::new(l - 1),
                ..cell
            };
            if climbable(next) {
                push(next, climb_cost(next));
            }
        }
        if l < *grid.layers().end() {
            let next = GCell {
                layer: LayerId::new(l + 1),
                ..cell
            };
            if climbable(next) {
                push(next, climb_cost(next));
            }
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::super::Tracks;
    use super::*;
    use pare_common::db::indices::NetId;

    const COSTS: GlobalCosts = GlobalCosts {
        via: 1.0,
        congestion: 1.0,
        penalty: 1.0,
    };

    /// A 10x10 grid with a horizontal layer 1 over a vertical layer 2,
    /// every edge holding 4 tracks.
    fn grid() -> GlobalGrid {
        let (nx, ny) = (10, 10);
        let edges = 2 * usize::from(nx) * usize::from(ny);
        GlobalGrid {
            origin: Point::new(Dbu(0), Dbu(0)),
            gcell: Dbu(100),
            nx,
            ny,
            min_layer: LayerId::new(1),
            directions: vec![Direction::Horizontal, Direction::Vertical],
            capacity: vec![Tracks::whole(4); edges],
            blocked: vec![false; edges],
            via_blocked: vec![false; edges],
            via_loss: vec![Tracks::ZERO; 2],
            usage: vec![Tracks::ZERO; edges],
            history: vec![0.0; edges],
        }
    }

    fn cell(layer: usize, x: u16, y: u16) -> GCell {
        GCell {
            layer: LayerId::new(layer),
            x,
            y,
        }
    }

    fn net(terminals: &[GCell]) -> GlobalNet {
        GlobalNet {
            net: NetId::new(0),
            terminals: terminals.to_vec(),
            demand: vec![Tracks::ONE; 2],
        }
    }

    /// Blocks the horizontal edges from column `x` to `x + 1` in rows `rows`.
    fn wall(grid: &mut GlobalGrid, x: u16, rows: std::ops::RangeInclusive<u16>) {
        for y in rows {
            let e = grid.edge(cell(1, x, y));
            grid.capacity[e] = Tracks::ZERO;
            grid.blocked[e] = true;
        }
    }

    fn crossed_blocked(grid: &GlobalGrid, paths: &[Vec<GCell>]) -> usize {
        paths
            .iter()
            .flat_map(|p| grid.path_edges(p))
            .filter(|&e| grid.blocked[e])
            .count()
    }

    fn reaches(paths: &[Vec<GCell>], target: GCell) -> bool {
        paths.iter().flatten().any(|&c| c == target)
    }

    #[test]
    fn a_net_detours_around_a_blocked_wall_beyond_its_margin() {
        let mut grid = grid();
        wall(&mut grid, 4, 0..=8);
        let terminals = [cell(1, 2, 4), cell(1, 7, 4)];

        let paths = route_tree(&grid, &net(&terminals), 1, COSTS);

        assert!(reaches(&paths, terminals[1]));
        assert_eq!(crossed_blocked(&grid, &paths), 0);
    }

    #[test]
    fn a_pin_no_detour_reaches_is_joined_across_blocked_edges() {
        let mut grid = grid();
        wall(&mut grid, 4, 0..=9);
        let terminals = [cell(1, 2, 4), cell(1, 7, 4)];

        let paths = route_tree(&grid, &net(&terminals), 1, COSTS);

        assert!(reaches(&paths, terminals[1]));
        assert_eq!(crossed_blocked(&grid, &paths), 1);
    }

    #[test]
    fn a_net_never_climbs_into_a_cell_with_no_room_for_a_via() {
        let mut grid = grid();
        wall(&mut grid, 4, 0..=8);
        for x in 0..=3 {
            for y in 0..10 {
                let i = grid.edge(cell(2, x, y));
                grid.via_blocked[i] = true;
            }
        }
        let terminals = [cell(1, 2, 4), cell(1, 7, 4)];

        let paths = route_tree(&grid, &net(&terminals), 1, COSTS);

        assert!(reaches(&paths, terminals[1]));
        assert_eq!(crossed_blocked(&grid, &paths), 0);
        assert!(paths.iter().flatten().all(|&c| !grid.via_blocked(c)));
    }

    #[test]
    fn a_wider_net_pays_for_the_tracks_it_takes() {
        let mut grid = grid();
        for x in 2..7 {
            let e = grid.edge(cell(1, x, 4));
            grid.usage[e] = Tracks::whole(3);
        }
        let terminals = [cell(1, 2, 4), cell(1, 7, 4)];
        let wide = GlobalNet {
            demand: vec![Tracks::whole(2); 2],
            ..net(&terminals)
        };

        let narrow_paths = route_tree(&grid, &net(&terminals), 2, COSTS);
        let wide_paths = route_tree(&grid, &wide, 2, COSTS);

        let on_row = |paths: &[Vec<GCell>]| {
            paths
                .iter()
                .flatten()
                .filter(|c| c.layer == LayerId::new(1) && c.y == 4)
                .count()
        };
        assert!(on_row(&narrow_paths) > on_row(&wide_paths));
    }
}
