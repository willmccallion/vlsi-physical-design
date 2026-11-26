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
use super::grid_index;
use super::stamped::StampedTable;
use super::{BLOCKED_COST, GCell, GcellWindow, GlobalCosts, GlobalGrid, GlobalNet};
use crate::tree::connection_order;
use pare_common::db::indices::LayerId;
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BinaryHeap;
use std::convert::Infallible;

/// Edge, in gcells, of the buckets used to find tree cells near a connection.
const BUCKET: u16 = 4;

/// The cells of a tree by the bucket of [`BUCKET`] gcells holding them.
type Buckets = FxHashMap<(u16, u16), Vec<GCell>>;

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
    let Ok(paths) = grow_tree(grid, net, margin, costs, plan, |avoid, buckets, ends| {
        Ok::<_, Infallible>(search_widening(grid, avoid, buckets, ends, margin))
    });
    paths
}

/// A connection of a net had no path inside the box around its ends grown
/// by the net's margin, nor along its planned coarse path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct OutgrowsArea;

/// As [`route_tree`], but every search stays inside the box around its
/// connection's ends grown by `margin` and the coarse path planned for it,
/// so it reads the grid nowhere else; fails when a connection has no path
/// there.
pub(super) fn route_tree_within(
    grid: &GlobalGrid,
    net: &GlobalNet,
    margin: u16,
    costs: GlobalCosts,
    plan: Option<&Plan>,
) -> Result<Vec<Vec<GCell>>, OutgrowsArea> {
    grow_tree(grid, net, margin, costs, plan, |avoid, buckets, ends| {
        search_window(grid, avoid, buckets, ends, margin).ok_or(OutgrowsArea)
    })
}

/// Joins each terminal of `net` to the tree built so far, along the coarse
/// path `plan` gives the connection when that holds one, else by `connect`.
fn grow_tree<E>(
    grid: &GlobalGrid,
    net: &GlobalNet,
    margin: u16,
    costs: GlobalCosts,
    plan: Option<&Plan>,
    connect: impl Fn(Query<'_>, &Buckets, [GCell; 2]) -> Result<Vec<GCell>, E>,
) -> Result<Vec<Vec<GCell>>, E> {
    let mut terminals: Vec<GCell> = net.terminals.clone();
    terminals.sort_unstable();
    terminals.dedup();
    if terminals.is_empty() {
        return Ok(Vec::new());
    }
    let points: Vec<Rect<Dbu>> = terminals
        .iter()
        .map(|c| {
            let p = Point::new(Dbu::new(i64::from(c.x)), Dbu::new(i64::from(c.y)));
            Rect::new(p, p)
        })
        .collect();
    let (first, joins) = connection_order(&points);

    let mut tree: FxHashSet<GCell> = FxHashSet::default();
    let mut buckets: Buckets = FxHashMap::default();
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
        let planned = plan
            .and_then(|p| p.between(ends[0], ends[1], grid))
            .and_then(|c| search_extended(grid, avoid, &c, &buckets, ends, margin));
        let path = match planned {
            Some(path) => path,
            None => connect(avoid, &buckets, ends)?,
        };
        for &c in &path {
            add_cell(&mut tree, &mut buckets, c);
        }
        paths.push(path);
    }
    Ok(paths)
}

/// Cheapest path from the tree to the first of `ends` inside the box around
/// both grown by `margin` or inside `corridor`, if they hold one.
fn search_extended(
    grid: &GlobalGrid,
    query: Query<'_>,
    corridor: &Corridor,
    buckets: &Buckets,
    ends: [GCell; 2],
    margin: u16,
) -> Option<Vec<GCell>> {
    let w = GcellWindow::spanning(ends[0], ends[1]).grown(margin, grid.nx, grid.ny);
    let c = corridor.window();
    let span = w.union(c);
    let inside = |cell: &GCell| w.contains(cell.x, cell.y) || corridor.allows(cell.x, cell.y);
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
    buckets: &Buckets,
    ends: [GCell; 2],
    margin: u16,
) -> Vec<GCell> {
    let target = ends[0];
    let full = GcellWindow::whole(grid.nx, grid.ny);
    let mut margin = margin;
    loop {
        if let Some(p) = search_window(grid, avoid, buckets, ends, margin) {
            return p;
        }
        if GcellWindow::spanning(ends[0], ends[1]).grown(margin, grid.nx, grid.ny) == full {
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

/// Cheapest path from the tree to the first of `ends` inside the box around
/// both grown by `margin`, if it holds one.
fn search_window(
    grid: &GlobalGrid,
    query: Query<'_>,
    buckets: &Buckets,
    ends: [GCell; 2],
    margin: u16,
) -> Option<Vec<GCell>> {
    let w = GcellWindow::spanning(ends[0], ends[1]).grown(margin, grid.nx, grid.ny);
    let sources = cells_in(buckets, w);
    if sources.is_empty() {
        return None;
    }
    search(grid, query, &sources, ends[0], w)
}

fn add_cell(tree: &mut FxHashSet<GCell>, buckets: &mut Buckets, c: GCell) {
    if tree.insert(c) {
        buckets
            .entry((c.x / BUCKET, c.y / BUCKET))
            .or_default()
            .push(c);
    }
}

/// Tree cells inside a window.
fn cells_in(buckets: &Buckets, w: GcellWindow) -> Vec<GCell> {
    let mut out = Vec::new();
    for bx in w.x0 / BUCKET..=w.x1 / BUCKET {
        for by in w.y0 / BUCKET..=w.y1 / BUCKET {
            for &c in buckets.get(&(bx, by)).into_iter().flatten() {
                if w.contains(c.x, c.y) {
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
    /// The cell's node in the search, see [`Local`].
    node: usize,
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

thread_local! {
    static SEARCH: std::cell::RefCell<StampedTable> = std::cell::RefCell::new(StampedTable::default());
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
    win: GcellWindow,
) -> Option<Vec<GCell>> {
    SEARCH.with_borrow_mut(|state| search_with(state, grid, query, sources, target, win))
}

fn search_with(
    state: &mut StampedTable,
    grid: &GlobalGrid,
    query: Query<'_>,
    sources: &[GCell],
    target: GCell,
    win: GcellWindow,
) -> Option<Vec<GCell>> {
    let rules = MoveRules::new(grid, query, sources, target);
    let local = Local::new(grid, win, query.corridor);
    state.reset(local.nodes());
    let mut heap = BinaryHeap::new();
    for (c, node) in sources.iter().filter_map(|&c| Some((c, local.index(c)?))) {
        state.set(node, 0.0, None);
        heap.push(Entry {
            f: rules.estimate(c),
            g: 0.0,
            cell: c,
            node,
        });
    }
    let found = loop {
        let Some(Entry { g, cell, node, .. }) = heap.pop() else {
            break None;
        };
        if state.cost(node).is_some_and(|b| g > b) {
            continue;
        }
        if cell == target {
            break Some(traced_path(state, &local, node, cell));
        }
        for step in rules.moves_from(cell, &local).into_iter().flatten() {
            let ng = g + step.cost;
            if state.cost(step.node).is_none_or(|b| ng < b) {
                state.set(step.node, ng, Some(node));
                heap.push(Entry {
                    f: ng + rules.estimate(step.cell),
                    g: ng,
                    cell: step.cell,
                    node: step.node,
                });
            }
        }
    };
    state.release_beyond(KEPT_SEARCH_NODES);
    found
}

/// The path to `cell`, at node `at`, back through its parents to a source.
fn traced_path(state: &StampedTable, local: &Local<'_>, at: usize, cell: GCell) -> Vec<GCell> {
    let mut path = vec![cell];
    let mut node = state.parent(at);
    while let Some(at) = node {
        path.push(local.cell(at));
        node = state.parent(at);
    }
    path.reverse();
    path
}

/// Which moves one search may make and what each costs.
struct MoveRules<'a> {
    grid: &'a GlobalGrid,
    net: &'a GlobalNet,
    costs: GlobalCosts,
    blocked: Blocked,
    /// The edges of the sources and the target, which blocked metal never
    /// shuts.
    terminals: FxHashSet<usize>,
    target: GCell,
}

impl<'a> MoveRules<'a> {
    fn new(grid: &'a GlobalGrid, query: Query<'a>, sources: &[GCell], target: GCell) -> Self {
        let terminals = sources
            .iter()
            .chain(std::iter::once(&target))
            .map(|&c| grid.edge(c))
            .collect();
        Self {
            grid,
            net: query.net,
            costs: query.costs,
            blocked: query.blocked,
            terminals,
            target,
        }
    }

    fn at_terminal(&self, a: GCell, b: GCell) -> bool {
        self.terminals.contains(&self.grid.edge(a)) || self.terminals.contains(&self.grid.edge(b))
    }

    fn passable(&self, e: usize, from: GCell, to: GCell) -> bool {
        self.blocked == Blocked::Cross || !self.grid.blocked[e] || self.at_terminal(from, to)
    }

    fn climbable(&self, from: GCell, to: GCell) -> bool {
        self.blocked == Blocked::Cross || !self.grid.via_blocked(to) || self.at_terminal(from, to)
    }

    fn edge_cost(&self, e: usize) -> f64 {
        let grid = self.grid;
        let demand = self.net.demand(grid.layer_offset(grid.cell_of(e).layer));
        let crossing = if grid.blocked[e] { BLOCKED_COST } else { 0.0 };
        grid.edge_cost(e, demand, self.costs) + crossing
    }

    fn climb_cost(&self, c: GCell) -> f64 {
        let crossing = if self.grid.via_blocked(c) {
            BLOCKED_COST
        } else {
            0.0
        };
        self.costs.via + crossing
    }

    /// The least cost from `c` to the target: its distance in the plane
    /// and a via per layer between.
    fn estimate(&self, c: GCell) -> f64 {
        let layers = c.layer.index().abs_diff(self.target.layer.index()) as f64;
        let along = |a: u16, b: u16| f64::from(a.abs_diff(b));
        let plane = along(c.x, self.target.x) + along(c.y, self.target.y);
        self.costs.via.mul_add(layers, plane)
    }

    /// The cells one move from `cell` within `local` with what each move
    /// costs: back and ahead along the layer's direction, where the net's
    /// wires may run on it, then down and up.
    fn moves_from(&self, cell: GCell, local: &Local<'_>) -> [Option<Move>; 4] {
        let grid = self.grid;
        let (x, y) = (cell.x, cell.y);
        let (behind, ahead) = match grid.direction(cell.layer) {
            _ if !grid.allows_wires(self.net.kind, cell.layer) => (None, None),
            Direction::Horizontal => (
                x.checked_sub(1).map(|x| GCell { x, ..cell }),
                (x + 1 < grid.nx).then(|| GCell { x: x + 1, ..cell }),
            ),
            Direction::Vertical => (
                y.checked_sub(1).map(|y| GCell { y, ..cell }),
                (y + 1 < grid.ny).then(|| GCell { y: y + 1, ..cell }),
            ),
        };
        let visit = |next: GCell, cost: f64| {
            local.index(next).map(|node| Move {
                cell: next,
                node,
                cost,
            })
        };
        let back = behind.and_then(|next| {
            let e = grid.edge(next);
            self.passable(e, cell, next)
                .then(|| visit(next, self.edge_cost(e)))
                .flatten()
        });
        let here = grid.edge(cell);
        let forth = ahead
            .filter(|&n| self.passable(here, cell, n))
            .and_then(|next| visit(next, self.edge_cost(here)));
        let l = cell.layer.index();
        let climb = |layer: usize| {
            let next = GCell {
                layer: LayerId::new(layer),
                ..cell
            };
            self.climbable(cell, next)
                .then(|| visit(next, self.climb_cost(next)))
                .flatten()
        };
        let down = (l > *grid.layers().start()).then(|| climb(l - 1)).flatten();
        let up = (l < *grid.layers().end()).then(|| climb(l + 1)).flatten();
        [back, forth, down, up]
    }
}

/// One move of a search: the cell it reaches, that cell's node and what the
/// move costs.
#[derive(Clone, Copy, Debug)]
struct Move {
    cell: GCell,
    node: usize,
    cost: f64,
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
    window: GcellWindow,
    corridor: Option<&'a Corridor>,
    /// The corridor's coarse cells, none without one.
    corridor_cells: &'a [(u16, u16)],
    layers: usize,
    first_layer: usize,
}

/// Gcells in one coarse cell.
const COARSE_CELLS: usize = SCALE as usize * SCALE as usize;

impl<'a> Local<'a> {
    fn new(grid: &GlobalGrid, window: GcellWindow, corridor: Option<&'a Corridor>) -> Self {
        Self {
            window,
            corridor,
            corridor_cells: corridor.map_or(&[], Corridor::cells),
            layers: grid.layers().count(),
            first_layer: *grid.layers().start(),
        }
    }

    const fn width(&self) -> usize {
        (self.window.x1 - self.window.x0) as usize + 1
    }

    const fn window_cells(&self) -> usize {
        self.width() * ((self.window.y1 - self.window.y0) as usize + 1)
    }

    fn in_window(&self, c: GCell) -> bool {
        self.window.contains(c.x, c.y)
    }

    const fn nodes(&self) -> usize {
        let beyond = self.corridor_cells.len() * COARSE_CELLS;
        (self.window_cells() + beyond) * self.layers
    }

    /// The node of `c`, if the search may visit it.
    fn index(&self, c: GCell) -> Option<usize> {
        let l = c.layer.index() - self.first_layer;
        let plane = if self.in_window(c) {
            usize::from(c.y - self.window.y0) * self.width() + usize::from(c.x - self.window.x0)
        } else {
            let slot = self.corridor?.slot(c.x, c.y)?;
            let within = usize::from(c.y % SCALE) * usize::from(SCALE) + usize::from(c.x % SCALE);
            self.window_cells() + slot * COARSE_CELLS + within
        };
        Some(plane * self.layers + l)
    }

    fn cell(&self, index: usize) -> GCell {
        let layer = LayerId::new(index % self.layers + self.first_layer);
        let plane = index / self.layers;
        let (x, y) = match plane.checked_sub(self.window_cells()) {
            None => (
                self.window.x0 + grid_index(plane % self.width()),
                self.window.y0 + grid_index(plane / self.width()),
            ),
            Some(beyond) => {
                let (cx, cy) = self.corridor_cells[beyond / COARSE_CELLS];
                let within = beyond % COARSE_CELLS;
                let s = usize::from(SCALE);
                (
                    cx * SCALE + grid_index(within % s),
                    cy * SCALE + grid_index(within / s),
                )
            }
        };
        GCell { layer, x, y }
    }
}

#[cfg(test)]
mod tests;
