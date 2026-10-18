//! Maze routing of one net through the 3D gcell grid: each pin joins the
//! tree built so far by an A* search confined to a window around the two
//! pins it connects, extended by the net's coarse corridor when it has one
//! (`coarse`), and widened only when that holds no path. Edges that
//! fixed metal blocks outright are never crossed and cells with no room for
//! a via pad are never climbed into, except to leave or reach a terminal
//! that sits in such a cell, so a net detours around a macro however far
//! that takes it; only a pin no detour reaches crosses them elsewhere, and
//! then as few of them as possible.

use super::coarse::{Corridor, Plan, SCALE};
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
/// Each connection searches the box around its two ends grown by `margin`
/// together with the path between them along the coarse tree of `plan`, if
/// given, and a larger box only when that holds no path.
pub(super) fn route_tree(
    grid: &GlobalGrid,
    net: &GlobalNet,
    margin: u16,
    costs: GlobalCosts,
    plan: Option<&Plan>,
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
    let avoid = Query {
        net,
        costs,
        blocked: Blocked::Avoid,
        corridor: None,
    };

    for join in joins {
        let target = terminals[join.pin];
        if tree.contains(&target) {
            continue;
        }
        let ends = [target, terminals[join.parent]];
        let path = plan
            .and_then(|p| p.between(ends[0], ends[1], grid))
            .and_then(|c| search_extended(grid, avoid, &c, &buckets, ends, margin))
            .unwrap_or_else(|| search_widening(grid, avoid, &buckets, ends, margin));
        for &c in &path {
            add_cell(&mut tree, &mut buckets, c);
        }
        paths.push(path);
    }
    paths
}

/// Cheapest path from the tree to the first of `ends` inside the box around
/// both grown by `margin` or inside `corridor`, if they hold one.
fn search_extended(
    grid: &GlobalGrid,
    query: Query<'_>,
    corridor: &Corridor,
    buckets: &FxHashMap<(u16, u16), Vec<GCell>>,
    ends: [GCell; 2],
    margin: u16,
) -> Option<Vec<GCell>> {
    let w = window(&ends, margin, grid.nx, grid.ny);
    let c = corridor.window();
    let span = (w.0.min(c.0), w.1.max(c.1), w.2.min(c.2), w.3.max(c.3));
    let inside = |cell: &GCell| {
        (w.0..=w.1).contains(&cell.x) && (w.2..=w.3).contains(&cell.y)
            || corridor.allows(cell.x, cell.y)
    };
    let sources: Vec<GCell> = cells_in(buckets, span).into_iter().filter(inside).collect();
    if sources.is_empty() {
        return None;
    }
    let extended = Query {
        corridor: Some(corridor),
        ..query
    };
    search(grid, extended, &sources, ends[0], w)
}

/// Cheapest path from the tree to the first of `ends` inside the box around
/// both grown by `margin`, growing the box fourfold while it holds no path,
/// and across blocked edges once the whole grid holds none.
fn search_widening(
    grid: &GlobalGrid,
    avoid: Query<'_>,
    buckets: &FxHashMap<(u16, u16), Vec<GCell>>,
    ends: [GCell; 2],
    margin: u16,
) -> Vec<GCell> {
    let target = ends[0];
    let full = (0, grid.nx - 1, 0, grid.ny - 1);
    let mut margin = margin;
    loop {
        let w = window(&ends, margin, grid.nx, grid.ny);
        let sources = cells_in(buckets, w);
        if !sources.is_empty()
            && let Some(p) = search(grid, avoid, &sources, target, w)
        {
            return p;
        }
        if w == full {
            let all = cells_in(buckets, full);
            let cross = Query {
                blocked: Blocked::Cross,
                ..avoid
            };
            return search(grid, cross, &all, target, full).unwrap_or_else(|| vec![target]);
        }
        margin = margin.saturating_mul(4).max(1);
    }
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

/// A* state over the gcells of a search window, kept per thread between
/// searches and cleared by bumping a generation stamp; nodes are numbered
/// within the window, see [`Local`].
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

    /// Lets go of the state when it holds more than `nodes` nodes.
    fn release_beyond(&mut self, nodes: usize) {
        if self.stamp.len() > nodes {
            *self = Self::default();
        }
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

/// What one search is for, how it prices the grid and where it may go.
#[derive(Clone, Copy)]
struct Query<'a> {
    net: &'a GlobalNet,
    costs: GlobalCosts,
    blocked: Blocked,
    /// Coarse cells the search may use beyond its window, if any.
    corridor: Option<&'a Corridor>,
}

/// Cheapest path from any source to `target` inside the window, or inside
/// the query's corridor if it has one.
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
        corridor,
    } = query;
    let terminals: FxHashSet<usize> = sources
        .iter()
        .chain(std::iter::once(&target))
        .map(|&c| grid.edge(c))
        .collect();
    let at_terminal =
        |a: GCell, b: GCell| terminals.contains(&grid.edge(a)) || terminals.contains(&grid.edge(b));
    let passable = |e: usize, from: GCell, to: GCell| {
        blocked == Blocked::Cross || !grid.blocked[e] || at_terminal(from, to)
    };
    let climbable = |from: GCell, to: GCell| {
        blocked == Blocked::Cross || !grid.via_blocked(to) || at_terminal(from, to)
    };
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
    let local = Local::new(grid, win, corridor);
    state.reset(local.nodes());
    let mut heap = BinaryHeap::new();
    for &c in sources {
        state.set(local.index(c), 0.0, NO_PARENT);
        heap.push(Entry {
            f: h(c),
            g: 0.0,
            cell: c,
        });
    }
    let found = loop {
        let Some(Entry { g, cell, .. }) = heap.pop() else {
            break None;
        };
        let here = grid.edge(cell);
        let at = local.index(cell);
        if state.cost(at).is_some_and(|b| g > b) {
            continue;
        }
        if cell == target {
            let mut path = vec![cell];
            let mut node = state.parent[at];
            while node != NO_PARENT {
                path.push(local.cell(node as usize));
                node = state.parent[node as usize];
            }
            path.reverse();
            break Some(path);
        }
        let mut push = |next: GCell, step: f64| {
            let ng = g + step;
            let index = local.index(next);
            if state.cost(index).is_none_or(|b| ng < b) {
                state.set(index, ng, crate::tech::to_u32(at));
                heap.push(Entry {
                    f: ng + h(next),
                    g: ng,
                    cell: next,
                });
            }
        };
        let (x, y) = (cell.x, cell.y);
        let (behind, ahead) = match grid.direction(cell.layer) {
            Direction::Horizontal => (
                x.checked_sub(1).map(|x| GCell { x, ..cell }),
                (x + 1 < grid.nx).then(|| GCell { x: x + 1, ..cell }),
            ),
            Direction::Vertical => (
                y.checked_sub(1).map(|y| GCell { y, ..cell }),
                (y + 1 < grid.ny).then(|| GCell { y: y + 1, ..cell }),
            ),
        };
        if let Some(next) = behind.filter(|&n| local.contains(n)) {
            let e = grid.edge(next);
            if passable(e, cell, next) {
                push(next, edge_cost(e));
            }
        }
        if let Some(next) = ahead.filter(|&n| local.contains(n))
            && passable(here, cell, next)
        {
            push(next, edge_cost(here));
        }
        let l = cell.layer.index();
        if l > *grid.layers().start() {
            let next = GCell {
                layer: LayerId::new(l - 1),
                ..cell
            };
            if climbable(cell, next) {
                push(next, climb_cost(next));
            }
        }
        if l < *grid.layers().end() {
            let next = GCell {
                layer: LayerId::new(l + 1),
                ..cell
            };
            if climbable(cell, next) {
                push(next, climb_cost(next));
            }
        }
    };
    state.release_beyond(KEPT_SEARCH_NODES);
    found
}

/// Nodes of search state a thread keeps between searches; a larger state,
/// grown for a search over most of the grid, is let go after it.
const KEPT_SEARCH_NODES: usize = 1 << 20;

/// The cells a search may visit on every layer, numbered from zero: the
/// gcells of a window, then those of the coarse cells of a corridor that
/// extends it, numbered by whole coarse cells so that a long corridor needs
/// no state for the gcells of its bounding box it leaves out.
#[derive(Clone, Copy, Debug)]
struct Local<'a> {
    x0: u16,
    y0: u16,
    x1: u16,
    y1: u16,
    corridor: Option<&'a Corridor>,
    layers: usize,
    first_layer: usize,
}

/// Gcells in one coarse cell.
const COARSE_CELLS: usize = SCALE as usize * SCALE as usize;

impl<'a> Local<'a> {
    fn new(grid: &GlobalGrid, win: (u16, u16, u16, u16), corridor: Option<&'a Corridor>) -> Self {
        Self {
            x0: win.0,
            y0: win.2,
            x1: win.1,
            y1: win.3,
            corridor,
            layers: grid.layers().count(),
            first_layer: *grid.layers().start(),
        }
    }

    const fn width(&self) -> usize {
        (self.x1 - self.x0) as usize + 1
    }

    const fn window_cells(&self) -> usize {
        self.width() * ((self.y1 - self.y0) as usize + 1)
    }

    fn in_window(&self, c: GCell) -> bool {
        (self.x0..=self.x1).contains(&c.x) && (self.y0..=self.y1).contains(&c.y)
    }

    fn nodes(&self) -> usize {
        let beyond = self.corridor.map_or(0, |c| c.cells().len() * COARSE_CELLS);
        (self.window_cells() + beyond) * self.layers
    }

    fn contains(&self, c: GCell) -> bool {
        self.in_window(c)
            || self
                .corridor
                .is_some_and(|corridor| corridor.allows(c.x, c.y))
    }

    fn index(&self, c: GCell) -> usize {
        let l = c.layer.index() - self.first_layer;
        let plane = if self.in_window(c) {
            usize::from(c.y - self.y0) * self.width() + usize::from(c.x - self.x0)
        } else {
            let slot = self
                .corridor
                .and_then(|corridor| corridor.slot(c.x, c.y))
                .unwrap_or(0);
            let within = usize::from(c.y % SCALE) * usize::from(SCALE) + usize::from(c.x % SCALE);
            self.window_cells() + slot * COARSE_CELLS + within
        };
        plane * self.layers + l
    }

    fn cell(&self, index: usize) -> GCell {
        let layer = LayerId::new(index % self.layers + self.first_layer);
        let plane = index / self.layers;
        let (x, y) = match plane.checked_sub(self.window_cells()) {
            None => (
                self.x0 + to_u16(plane % self.width()),
                self.y0 + to_u16(plane / self.width()),
            ),
            Some(beyond) => {
                let (cx, cy) = self
                    .corridor
                    .map_or((0, 0), |corridor| corridor.cells()[beyond / COARSE_CELLS]);
                let within = beyond % COARSE_CELLS;
                let s = usize::from(SCALE);
                (
                    cx * SCALE + to_u16(within % s),
                    cy * SCALE + to_u16(within / s),
                )
            }
        };
        GCell { layer, x, y }
    }
}

fn to_u16(v: usize) -> u16 {
    u16::try_from(v).unwrap_or(u16::MAX)
}

#[cfg(test)]
mod tests {
    use super::super::coarse::CoarseGrid;
    use super::super::{Tracks, Usage};
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
        sized_grid(10, 10)
    }

    /// A `nx` by `ny` grid like [`grid`].
    fn sized_grid(nx: u16, ny: u16) -> GlobalGrid {
        let edges = 2 * usize::from(nx) * usize::from(ny);
        GlobalGrid {
            origin: Point::new(Dbu(0), Dbu(0)),
            gcell: Dbu(100),
            nx,
            ny,
            min_layer: LayerId::new(1),
            directions: vec![Direction::Horizontal, Direction::Vertical],
            reserve: 0.0,
            capacity: vec![Tracks::whole(4); edges],
            blocked: vec![false; edges],
            via_blocked: vec![false; edges],
            via_loss: vec![Tracks::ZERO; 2],
            usage: Usage::zeroed(edges),
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

        let paths = route_tree(&grid, &net(&terminals), 1, COSTS, None);

        assert!(reaches(&paths, terminals[1]));
        assert_eq!(crossed_blocked(&grid, &paths), 0);
    }

    #[test]
    fn a_pin_no_detour_reaches_is_joined_across_blocked_edges() {
        let mut grid = grid();
        wall(&mut grid, 4, 0..=9);
        let terminals = [cell(1, 2, 4), cell(1, 7, 4)];

        let paths = route_tree(&grid, &net(&terminals), 1, COSTS, None);

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

        let paths = route_tree(&grid, &net(&terminals), 1, COSTS, None);

        assert!(reaches(&paths, terminals[1]));
        assert_eq!(crossed_blocked(&grid, &paths), 0);
        assert!(paths.iter().flatten().all(|&c| !grid.via_blocked(c)));
    }

    #[test]
    fn a_net_takes_the_detour_its_corridor_plans_beyond_its_margin() {
        let mut grid = sized_grid(30, 30);
        for x in 10..20 {
            for y in 0..20 {
                grid.usage.set(grid.edge(cell(1, x, y)), Tracks::whole(40));
            }
        }
        let terminals = [cell(1, 2, 2), cell(1, 27, 2)];
        let plan = CoarseGrid::new(&grid)
            .plan(&grid, &net(&terminals))
            .unwrap();

        let planned = route_tree(&grid, &net(&terminals), 1, COSTS, Some(&plan));
        let windowed = route_tree(&grid, &net(&terminals), 1, COSTS, None);

        let highest = |paths: &[Vec<GCell>]| paths.iter().flatten().map(|c| c.y).max();
        assert!(reaches(&planned, terminals[1]));
        assert!(highest(&planned) >= Some(20));
        assert!(highest(&windowed) < Some(20));
    }

    #[test]
    fn a_net_whose_corridor_holds_no_path_is_still_routed() {
        let mut grid = grid();
        wall(&mut grid, 6, 0..=4);
        let terminals = [cell(1, 1, 1), cell(1, 8, 1)];
        let plan = CoarseGrid::new(&grid)
            .plan(&grid, &net(&terminals))
            .unwrap();

        let paths = route_tree(&grid, &net(&terminals), 1, COSTS, Some(&plan));

        assert!(!plan.corridor().allows(6, 6));
        assert!(reaches(&paths, terminals[1]));
        assert_eq!(crossed_blocked(&grid, &paths), 0);
    }

    #[test]
    fn a_wider_net_pays_for_the_tracks_it_takes() {
        let mut grid = grid();
        for x in 2..7 {
            let e = grid.edge(cell(1, x, 4));
            grid.usage.set(e, Tracks::whole(3));
        }
        let terminals = [cell(1, 2, 4), cell(1, 7, 4)];
        let wide = GlobalNet {
            demand: vec![Tracks::whole(2); 2],
            ..net(&terminals)
        };

        let narrow_paths = route_tree(&grid, &net(&terminals), 2, COSTS, None);
        let wide_paths = route_tree(&grid, &wide, 2, COSTS, None);

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
