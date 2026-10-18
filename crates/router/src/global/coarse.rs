//! Coarse planning of rip-up and reroute, as CUGR's multi-level maze
//! routing: before a ripped net is maze-routed on the gcell grid, a search
//! over the whole chip on cells of [`SCALE`] by [`SCALE`] gcells, priced by
//! how few free tracks they have left, picks the corridor its route keeps
//! to. A net can then detour as far as the free tracks call for, while the
//! fine search stays inside a band a coarse cell wide.

use super::{GCell, GlobalGrid, GlobalNet};
use pare_common::db::indices::LayerId;
use pare_common::db::library::Direction;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::cell::RefCell;
use std::collections::BinaryHeap;

/// Gcells along each side of a coarse cell, as in CUGR.
pub(super) const SCALE: u16 = 5;

/// Free tracks a coarse cell is priced at when it has fewer, as in CUGR, so
/// a full cell costs a bounded amount rather than an infinite one.
const MIN_FREE: f64 = 0.1;

/// The gcell grid coarsened by [`SCALE`]: per layer and coarse cell, the
/// mean free tracks of its gcells, and where fixed metal walls it off.
#[derive(Debug)]
pub(super) struct CoarseGrid {
    nx: u16,
    ny: u16,
    first_layer: usize,
    directions: Vec<Direction>,
    /// Mean free tracks of each node's gcells, by node.
    free: Vec<f64>,
    /// Whether some gcell edge crosses from each node to the next coarse
    /// cell along its layer without fixed metal blocking it outright.
    open: Vec<bool>,
    /// Whether some gcell of each node has room for a via pad.
    climbable: Vec<bool>,
}

impl CoarseGrid {
    /// The coarse grid over `grid`, with the free tracks its usage leaves.
    pub(super) fn new(grid: &GlobalGrid) -> Self {
        let nx = grid.nx.div_ceil(SCALE);
        let ny = grid.ny.div_ceil(SCALE);
        let first_layer = *grid.layers().start();
        let directions: Vec<Direction> = grid
            .layers()
            .map(|l| grid.direction(LayerId::new(l)))
            .collect();
        let mut coarse = Self {
            nx,
            ny,
            first_layer,
            directions,
            free: Vec::new(),
            open: Vec::new(),
            climbable: Vec::new(),
        };
        let nodes = coarse.nodes();
        coarse.open = (0..nodes)
            .into_par_iter()
            .map(|n| coarse.crossable(grid, coarse.position(n)))
            .collect();
        coarse.climbable = (0..nodes)
            .into_par_iter()
            .map(|n| {
                coarse
                    .gcells(grid, coarse.position(n))
                    .any(|c| !grid.via_blocked(c))
            })
            .collect();
        coarse.free = vec![0.0; nodes];
        coarse.refresh(grid);
        coarse
    }

    /// Takes the free tracks from the grid's current usage.
    pub(super) fn refresh(&mut self, grid: &GlobalGrid) {
        let mut free = std::mem::take(&mut self.free);
        free.par_iter_mut()
            .enumerate()
            .for_each(|(n, f)| *f = self.mean_free(grid, n));
        self.free = free;
    }

    /// Takes the free tracks of the coarse cells `cells`, on every layer,
    /// from the grid's current usage.
    pub(super) fn refresh_cells(&mut self, grid: &GlobalGrid, cells: &[(u16, u16)]) {
        let shared: &Self = self;
        let fresh: Vec<(usize, f64)> = cells
            .par_iter()
            .flat_map_iter(|&(x, y)| {
                (0..shared.directions.len()).map(move |l| {
                    let node = shared.index(l, x, y);
                    (node, shared.mean_free(grid, node))
                })
            })
            .collect();
        for (node, free) in fresh {
            self.free[node] = free;
        }
    }

    fn mean_free(&self, grid: &GlobalGrid, node: usize) -> f64 {
        let (sum, count) = self
            .gcells(grid, self.position(node))
            .fold((0.0, 0_u32), |(s, k), c| (s + free_tracks(grid, c), k + 1));
        if count == 0 {
            0.0
        } else {
            sum / f64::from(count)
        }
    }

    /// The plan of a reroute of `net`: the cheapest coarse tree joining its
    /// terminals, found by CUGR's multi-source Dijkstra search, which grows
    /// the tree from one terminal and restarts from the whole tree at cost
    /// zero each time it reaches another. None when its terminals share one
    /// coarse cell, which the windowed search covers, or when fixed metal
    /// cuts one off.
    pub(super) fn plan(&self, grid: &GlobalGrid, net: &GlobalNet) -> Option<Plan> {
        let mut pins: Vec<usize> = net.terminals.iter().map(|&c| self.node_of(c)).collect();
        pins.sort_unstable();
        pins.dedup();
        let footprint: FxHashSet<(u16, u16)> = pins.iter().map(|&n| self.plane(n)).collect();
        if footprint.len() <= 1 {
            return None;
        }
        let nodes = PLAN.with_borrow_mut(|state| {
            let tree = self.search(state, &pins)?;
            Some(
                tree.into_iter()
                    .map(|n| (n, state.parent[n]))
                    .collect::<Vec<(usize, u32)>>(),
            )
        })?;
        let tree = CoarseTree::new(self, &nodes);
        let cells: FxHashSet<(u16, u16)> = tree.cells.iter().copied().collect();
        Some(Plan {
            corridor: Corridor::new(cells.into_iter().collect(), grid),
            tree,
        })
    }

    fn search(&self, state: &mut PlanState, pins: &[usize]) -> Option<Vec<usize>> {
        state.reset(self.nodes());
        let is_pin = |n: usize| pins.binary_search(&n).is_ok();
        let mut tree: FxHashSet<usize> = FxHashSet::default();
        let mut heap = BinaryHeap::new();
        let first = pins[0];
        let _ = tree.insert(first);
        state.set(first, 0.0, NO_PARENT);
        heap.push(Visit {
            cost: 0.0,
            node: first,
        });
        let mut remaining = pins.len() - 1;
        while remaining > 0 {
            let reached = loop {
                let Visit { cost, node } = heap.pop()?;
                if state.cost(node).is_some_and(|best| cost > best) {
                    continue;
                }
                if is_pin(node) && !tree.contains(&node) {
                    break node;
                }
                self.relax(state, &mut heap, node, cost, is_pin);
            };
            let mut node = reached;
            while !tree.contains(&node) {
                let _ = tree.insert(node);
                if is_pin(node) {
                    remaining -= 1;
                }
                let parent = state.parent[node];
                state.set(node, 0.0, parent);
                heap.push(Visit { cost: 0.0, node });
                if parent == NO_PARENT {
                    break;
                }
                node = parent as usize;
            }
        }
        Some(tree.into_iter().collect())
    }

    fn relax(
        &self,
        state: &mut PlanState,
        heap: &mut BinaryHeap<Visit>,
        node: usize,
        cost: f64,
        is_pin: impl Fn(usize) -> bool,
    ) {
        let mut push = |next: usize, step: f64| {
            let reach = cost + step;
            if state.cost(next).is_none_or(|best| reach < best) {
                state.set(next, reach, crate::tech::to_u32(node));
                heap.push(Visit {
                    cost: reach,
                    node: next,
                });
            }
        };
        let (l, x, y) = self.position(node);
        let along = |a: usize, b: usize| f64::from(SCALE) * (self.price(a) + self.price(b));
        let neighbours = match self.directions[l] {
            Direction::Horizontal => [
                x.checked_sub(1).map(|x| (x, y)),
                (x + 1 < self.nx).then_some((x + 1, y)),
            ],
            Direction::Vertical => [
                y.checked_sub(1).map(|y| (x, y)),
                (y + 1 < self.ny).then_some((x, y + 1)),
            ],
        };
        for (nx, ny) in neighbours.into_iter().flatten() {
            let next = self.index(l, nx, ny);
            let behind = if next < node { next } else { node };
            if self.open[behind] || is_pin(node) || is_pin(next) {
                push(next, along(node, next));
            }
        }
        let below = l.checked_sub(1);
        let above = (l + 1 < self.directions.len()).then_some(l + 1);
        for other in [below, above].into_iter().flatten() {
            let next = self.index(other, x, y);
            if self.climbable[next] || is_pin(next) {
                push(next, self.price(node) + self.price(next));
            }
        }
    }

    /// Cost of passing through a node, CUGR's inverse of its free tracks.
    fn price(&self, node: usize) -> f64 {
        1.0 / self.free[node].max(MIN_FREE)
    }

    /// Whether a wire can cross from `node` to the next coarse cell along
    /// its layer on some gcell edge fixed metal does not block outright.
    fn crossable(&self, grid: &GlobalGrid, (l, x, y): (usize, u16, u16)) -> bool {
        let layer = LayerId::new(self.first_layer + l);
        let last = |c: u16, n: u16| (c * SCALE + SCALE - 1).min(n - 1);
        match self.directions[l] {
            Direction::Horizontal => {
                let gx = last(x, grid.nx);
                gx + 1 < grid.nx
                    && rows(grid, y).any(|gy| {
                        !grid.blocked[grid.edge(GCell {
                            layer,
                            x: gx,
                            y: gy,
                        })]
                    })
            }
            Direction::Vertical => {
                let gy = last(y, grid.ny);
                gy + 1 < grid.ny
                    && columns(grid, x).any(|gx| {
                        !grid.blocked[grid.edge(GCell {
                            layer,
                            x: gx,
                            y: gy,
                        })]
                    })
            }
        }
    }

    fn gcells<'a>(
        &'a self,
        grid: &'a GlobalGrid,
        (l, x, y): (usize, u16, u16),
    ) -> impl Iterator<Item = GCell> + 'a {
        let layer = LayerId::new(self.first_layer + l);
        rows(grid, y).flat_map(move |gy| {
            columns(grid, x).map(move |gx| GCell {
                layer,
                x: gx,
                y: gy,
            })
        })
    }

    const fn nodes(&self) -> usize {
        self.directions.len() * self.nx as usize * self.ny as usize
    }

    const fn index(&self, l: usize, x: u16, y: u16) -> usize {
        (l * self.ny as usize + y as usize) * self.nx as usize + x as usize
    }

    const fn node_of(&self, c: GCell) -> usize {
        self.index(c.layer.index() - self.first_layer, c.x / SCALE, c.y / SCALE)
    }

    fn position(&self, node: usize) -> (usize, u16, u16) {
        let (nx, ny) = (usize::from(self.nx), usize::from(self.ny));
        let x = node % nx;
        let rest = node / nx;
        (
            rest / ny,
            u16::try_from(x).unwrap_or(u16::MAX),
            u16::try_from(rest % ny).unwrap_or(u16::MAX),
        )
    }

    fn plane(&self, node: usize) -> (u16, u16) {
        let (_, x, y) = self.position(node);
        (x, y)
    }
}

/// The gcell columns of coarse column `x`.
fn columns(grid: &GlobalGrid, x: u16) -> std::ops::Range<u16> {
    x * SCALE..(x * SCALE + SCALE).min(grid.nx)
}

/// The gcell rows of coarse row `y`.
fn rows(grid: &GlobalGrid, y: u16) -> std::ops::Range<u16> {
    y * SCALE..(y * SCALE + SCALE).min(grid.ny)
}

/// A net's coarse plan: the corridor of every coarse cell its coarse tree
/// passes, which bounds what its reroute may touch, and the tree, whose
/// path between two terminals is the detour planned for the connection
/// between them.
#[derive(Clone, Debug)]
pub(super) struct Plan {
    corridor: Corridor,
    tree: CoarseTree,
}

impl Plan {
    /// The coarse cells of the whole tree.
    pub(super) const fn corridor(&self) -> &Corridor {
        &self.corridor
    }

    /// The coarse cells on the tree's path between the coarse cells of
    /// gcells `a` and `b`, if the tree holds both.
    pub(super) fn between(&self, a: GCell, b: GCell, grid: &GlobalGrid) -> Option<Corridor> {
        let mut x = *self.tree.index.get(&self.tree.node_of(a))? as usize;
        let mut y = *self.tree.index.get(&self.tree.node_of(b))? as usize;
        let (parent, depth) = (&self.tree.parent, &self.tree.depth);
        let mut cells = Vec::new();
        while depth[x] > depth[y] {
            cells.push(self.tree.cells[x]);
            x = parent[x] as usize;
        }
        while depth[y] > depth[x] {
            cells.push(self.tree.cells[y]);
            y = parent[y] as usize;
        }
        while x != y {
            cells.push(self.tree.cells[x]);
            cells.push(self.tree.cells[y]);
            x = parent[x] as usize;
            y = parent[y] as usize;
        }
        cells.push(self.tree.cells[x]);
        cells.sort_unstable();
        cells.dedup();
        Some(Corridor::new(cells, grid))
    }
}

/// A coarse tree as its nodes, each with its coarse cell, parent and depth.
#[derive(Clone, Debug)]
struct CoarseTree {
    cells: Vec<(u16, u16)>,
    /// Position of each node's parent, the root's being its own.
    parent: Vec<u32>,
    depth: Vec<u32>,
    /// Position in the tree of each coarse grid node it holds.
    index: FxHashMap<usize, u32>,
    first_layer: usize,
    nx: u16,
    ny: u16,
}

impl CoarseTree {
    /// The tree of `nodes`, coarse grid nodes of `coarse` each with its
    /// parent among them or [`NO_PARENT`] at the root.
    fn new(coarse: &CoarseGrid, nodes: &[(usize, u32)]) -> Self {
        let index: FxHashMap<usize, u32> = nodes
            .iter()
            .enumerate()
            .map(|(k, &(n, _))| (n, crate::tech::to_u32(k)))
            .collect();
        let parent: Vec<u32> = nodes
            .iter()
            .enumerate()
            .map(|(k, &(_, p))| {
                index
                    .get(&(p as usize))
                    .copied()
                    .filter(|_| p != NO_PARENT)
                    .unwrap_or_else(|| crate::tech::to_u32(k))
            })
            .collect();
        let mut depth = vec![u32::MAX; nodes.len()];
        for start in 0..nodes.len() {
            let mut chain = Vec::new();
            let mut k = start;
            while depth[k] == u32::MAX && parent[k] as usize != k {
                chain.push(k);
                k = parent[k] as usize;
            }
            if depth[k] == u32::MAX {
                depth[k] = 0;
            }
            let mut d = depth[k];
            for &c in chain.iter().rev() {
                d += 1;
                depth[c] = d;
            }
        }
        Self {
            cells: nodes.iter().map(|&(n, _)| coarse.plane(n)).collect(),
            parent,
            depth,
            index,
            first_layer: coarse.first_layer,
            nx: coarse.nx,
            ny: coarse.ny,
        }
    }

    fn node_of(&self, c: GCell) -> usize {
        let l = c.layer.index() - self.first_layer;
        (l * usize::from(self.ny) + usize::from(c.y / SCALE)) * usize::from(self.nx)
            + usize::from(c.x / SCALE)
    }
}

/// Free tracks a gcell has left on its layer: the mean over the edges it
/// lies between of their capacity less their usage, as CUGR's cell resource.
fn free_tracks(grid: &GlobalGrid, c: GCell) -> f64 {
    let mut sum = 0.0;
    let mut count = 0_u32;
    for e in [grid.backward_edge(c), grid.forward_edge(c)]
        .into_iter()
        .flatten()
    {
        sum += (grid.capacity[e] - grid.usage.get(e)).to_f64();
        count += 1;
    }
    if count == 0 {
        0.0
    } else {
        sum / f64::from(count)
    }
}

/// The coarse cells a net's reroute keeps to, on every layer.
#[derive(Clone, Debug)]
pub(super) struct Corridor {
    x0: u16,
    y0: u16,
    width: u16,
    /// Position among `cells` of each coarse cell of the bounding box, or
    /// [`NO_SLOT`].
    slots: Vec<u32>,
    cells: Vec<(u16, u16)>,
    /// The gcells of the bounding box, as (x0, x1, y0, y1).
    window: (u16, u16, u16, u16),
    nx: u16,
    ny: u16,
}

/// Marks a coarse cell of a corridor's bounding box outside the corridor.
const NO_SLOT: u32 = u32::MAX;

impl Corridor {
    fn new(mut cells: Vec<(u16, u16)>, grid: &GlobalGrid) -> Self {
        cells.sort_unstable();
        let x0 = cells.iter().map(|c| c.0).min().unwrap_or(0);
        let x1 = cells.iter().map(|c| c.0).max().unwrap_or(0);
        let y0 = cells.iter().map(|c| c.1).min().unwrap_or(0);
        let y1 = cells.iter().map(|c| c.1).max().unwrap_or(0);
        let width = x1 - x0 + 1;
        let height = y1 - y0 + 1;
        let mut slots = vec![NO_SLOT; usize::from(width) * usize::from(height)];
        for (k, &(x, y)) in cells.iter().enumerate() {
            slots[usize::from(y - y0) * usize::from(width) + usize::from(x - x0)] =
                crate::tech::to_u32(k);
        }
        let window = (
            x0 * SCALE,
            (x1 * SCALE + SCALE - 1).min(grid.nx - 1),
            y0 * SCALE,
            (y1 * SCALE + SCALE - 1).min(grid.ny - 1),
        );
        Self {
            x0,
            y0,
            width,
            slots,
            cells,
            window,
            nx: grid.nx,
            ny: grid.ny,
        }
    }

    /// The gcells of the corridor's bounding box, as (x0, x1, y0, y1).
    pub(super) const fn window(&self) -> (u16, u16, u16, u16) {
        self.window
    }

    /// The corridor's coarse cells.
    pub(super) fn cells(&self) -> &[(u16, u16)] {
        &self.cells
    }

    /// Position among the corridor's cells of the coarse cell holding
    /// gcell (`x`, `y`), if the corridor holds it.
    pub(super) fn slot(&self, x: u16, y: u16) -> Option<usize> {
        if x >= self.nx || y >= self.ny {
            return None;
        }
        let (cx, cy) = (x / SCALE, y / SCALE);
        if cx < self.x0 || cy < self.y0 || cx - self.x0 >= self.width {
            return None;
        }
        let i = usize::from(cy - self.y0) * usize::from(self.width) + usize::from(cx - self.x0);
        self.slots
            .get(i)
            .copied()
            .filter(|&s| s != NO_SLOT)
            .map(|s| s as usize)
    }

    /// Whether the corridor holds gcell (`x`, `y`).
    pub(super) fn allows(&self, x: u16, y: u16) -> bool {
        self.slot(x, y).is_some()
    }
}

#[derive(Clone, Copy, PartialEq)]
struct Visit {
    cost: f64,
    node: usize,
}

impl Eq for Visit {}

impl Ord for Visit {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| self.node.cmp(&other.node))
    }
}

impl PartialOrd for Visit {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// Dijkstra state over the coarse grid, kept per thread between plans and
/// cleared by bumping a generation stamp.
#[derive(Debug, Default)]
struct PlanState {
    cost: Vec<f64>,
    parent: Vec<u32>,
    stamp: Vec<u32>,
    generation: u32,
}

/// Marks a node without a parent.
const NO_PARENT: u32 = u32::MAX;

impl PlanState {
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
    static PLAN: RefCell<PlanState> = RefCell::new(PlanState::default());
}

#[cfg(test)]
mod tests {
    use super::super::{GlobalGrid, Tracks, Usage};
    use super::*;
    use pare_common::db::indices::NetId;
    use pare_common::geom::dbu::Dbu;
    use pare_common::geom::point::Point;

    /// A `nx` by `ny` grid with a horizontal layer 1 over a vertical layer
    /// 2, every edge holding 4 tracks.
    fn grid(nx: u16, ny: u16) -> GlobalGrid {
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

    const fn cell(layer: usize, x: u16, y: u16) -> GCell {
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

    fn block(grid: &mut GlobalGrid, c: GCell) {
        let e = grid.edge(c);
        grid.capacity[e] = Tracks::ZERO;
        grid.blocked[e] = true;
    }

    #[test]
    fn a_gcell_has_the_mean_of_what_its_edges_leave_free() {
        let mut grid = grid(10, 10);
        grid.usage.set(grid.edge(cell(1, 3, 2)), Tracks::whole(3));

        let between = free_tracks(&grid, cell(1, 4, 2));
        let at_the_edge = free_tracks(&grid, cell(1, 9, 2));

        assert!((between - 2.5).abs() < 1e-9);
        assert!((at_the_edge - 4.0).abs() < 1e-9);
    }

    #[test]
    fn a_coarse_edge_is_a_wall_only_when_every_gcell_edge_across_it_is_blocked() {
        let mut grid = grid(10, 10);
        for y in 0..5 {
            block(&mut grid, cell(1, 4, y));
        }
        for y in 5..9 {
            block(&mut grid, cell(1, 4, y));
        }

        let coarse = CoarseGrid::new(&grid);

        assert!(!coarse.open[coarse.index(0, 0, 0)]);
        assert!(coarse.open[coarse.index(0, 0, 1)]);
    }

    #[test]
    fn a_plan_joins_every_terminal() {
        let grid = grid(30, 30);
        let terminals = [cell(1, 2, 2), cell(1, 27, 3), cell(2, 14, 28)];

        let plan = CoarseGrid::new(&grid)
            .plan(&grid, &net(&terminals))
            .unwrap();

        assert!(terminals.iter().all(|c| plan.corridor().allows(c.x, c.y)));
    }

    #[test]
    fn a_plan_detours_around_a_band_with_no_free_tracks() {
        let grid = grid(40, 30);
        for x in 10..30 {
            for y in 0..25 {
                grid.usage.add(grid.edge(cell(1, x, y)), Tracks::whole(20));
            }
        }
        let terminals = [cell(1, 2, 2), cell(1, 37, 2)];

        let plan = CoarseGrid::new(&grid)
            .plan(&grid, &net(&terminals))
            .unwrap();

        assert!(plan.corridor().allows(20, 27));
        assert!(!plan.corridor().allows(20, 2));
    }

    #[test]
    fn a_plan_never_crosses_a_wall_of_blocked_edges() {
        let mut grid = grid(40, 30);
        for y in 0..25 {
            block(&mut grid, cell(1, 19, y));
        }
        let terminals = [cell(1, 2, 2), cell(1, 37, 2)];

        let plan = CoarseGrid::new(&grid)
            .plan(&grid, &net(&terminals))
            .unwrap();

        assert!(plan.corridor().allows(19, 27));
        assert!(!plan.corridor().allows(19, 2));
    }

    #[test]
    fn the_path_between_two_terminals_leaves_out_the_branch_to_a_third() {
        let grid = grid(40, 40);
        let terminals = [cell(1, 2, 2), cell(1, 37, 2), cell(2, 20, 37)];

        let plan = CoarseGrid::new(&grid)
            .plan(&grid, &net(&terminals))
            .unwrap();
        let path = plan.between(terminals[0], terminals[1], &grid).unwrap();

        assert!(path.allows(2, 2) && path.allows(37, 2));
        assert!(plan.corridor().allows(20, 37));
        assert!(!path.allows(20, 37));
    }

    #[test]
    fn terminals_in_one_coarse_cell_need_no_plan() {
        let grid = grid(30, 30);
        let terminals = [cell(1, 1, 1), cell(2, 3, 4)];

        let corridor = CoarseGrid::new(&grid).plan(&grid, &net(&terminals));

        assert!(corridor.is_none());
    }
}
