//! Global routing on a grid of gcells.
//!
//! Each routable layer gets a capacity on every gcell edge in its preferred
//! direction: the number of its tracks crossing that edge that no obstacle
//! blocks. Nets are routed as trees of A* paths through the 3D grid with a
//! cost that grows with overflow and with each edge's overflow history; nets
//! crossing overflowed edges are ripped up and rerouted. The cells each net
//! passes through become its guide for detailed routing.

use crate::occupancy::{Occupancy, Owner, TrackRef};
use crate::tech::RouteTech;
use crate::tree::connection_order;
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BinaryHeap;

/// A gcell on a layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct GCell {
    /// Routing layer.
    pub layer: LayerId,
    /// Column.
    pub x: u16,
    /// Row.
    pub y: u16,
}

/// The gcell grid with edge capacities, usage and history.
#[derive(Debug)]
pub struct GlobalGrid {
    origin: Point<Dbu>,
    gcell: Dbu,
    nx: u16,
    ny: u16,
    min_layer: LayerId,
    directions: Vec<Direction>,
    capacity: Vec<u16>,
    usage: Vec<u16>,
    history: Vec<f32>,
}

impl GlobalGrid {
    /// A grid of `gcell`-sized cells over `die` whose edges offer the free
    /// tracks crossing them less the `reserve` fraction.
    pub fn new(
        tech: &RouteTech,
        occupancy: &Occupancy,
        die: Rect<Dbu>,
        gcell: Dbu,
        reserve: f64,
    ) -> Self {
        let span = |extent: Dbu| {
            u16::try_from((extent.get() + gcell.get() - 1) / gcell.get())
                .unwrap_or(u16::MAX)
                .max(1)
        };
        let (nx, ny) = (span(die.width()), span(die.height()));
        let layers: Vec<LayerId> = (tech.min_layer.index()..=tech.max_layer.index())
            .map(LayerId::new)
            .collect();
        let per_layer = usize::from(nx) * usize::from(ny);
        let mut grid = Self {
            origin: die.min,
            gcell,
            nx,
            ny,
            min_layer: tech.min_layer,
            directions: layers.iter().map(|&l| tech.layer(l).direction).collect(),
            capacity: Vec::new(),
            usage: vec![0; per_layer * layers.len()],
            history: vec![0.0; per_layer * layers.len()],
        };
        grid.capacity = layers
            .par_iter()
            .flat_map_iter(|&l| grid.layer_capacity(tech, occupancy, l))
            .map(|tracks| (f64::from(tracks) * (1.0 - reserve)).floor() as u16)
            .collect();
        grid
    }

    /// Free-track counts across each preferred-direction edge of one layer.
    fn layer_capacity(&self, tech: &RouteTech, occupancy: &Occupancy, layer: LayerId) -> Vec<u16> {
        let lt = tech.layer(layer);
        let (nx, ny) = (usize::from(self.nx), usize::from(self.ny));
        let mut cap = vec![0u16; nx * ny];
        let (along_origin, across_origin, along_cells) = match lt.direction {
            Direction::Horizontal => (self.origin.x, self.origin.y, nx),
            Direction::Vertical => (self.origin.y, self.origin.x, ny),
        };
        let probe = Owner::Net(u32::MAX);
        for (t, &across) in lt.tracks.iter().enumerate() {
            let row = (across - across_origin).get().div_euclid(self.gcell.get());
            let Ok(row) = usize::try_from(row) else {
                continue;
            };
            let track = TrackRef {
                layer,
                track: crate::tech::to_u32(t),
            };
            for i in 0..along_cells.saturating_sub(1) {
                let boundary = along_origin + self.gcell * (i as i64 + 1);
                let half = lt.width / 2;
                let blocked = occupancy
                    .conflicts(
                        track,
                        boundary - half,
                        boundary + half,
                        lt.spacing,
                        probe,
                        true,
                    )
                    .any(|o| o.owner == Owner::Obstacle);
                if blocked {
                    continue;
                }
                let index = match lt.direction {
                    Direction::Horizontal if row < ny => row * nx + i,
                    Direction::Vertical if row < nx => i * nx + row,
                    _ => continue,
                };
                cap[index] = cap[index].saturating_add(1);
            }
        }
        cap
    }

    /// Number of grid columns and rows.
    pub const fn size(&self) -> (u16, u16) {
        (self.nx, self.ny)
    }

    /// Edge of one gcell.
    pub const fn gcell(&self) -> Dbu {
        self.gcell
    }

    /// The gcell containing a point on a layer.
    pub fn cell_at(&self, layer: LayerId, p: Point<Dbu>) -> GCell {
        let coord = |v: Dbu, o: Dbu, n: u16| {
            let c = (v - o)
                .get()
                .div_euclid(self.gcell.get())
                .clamp(0, i64::from(n) - 1);
            u16::try_from(c).unwrap_or(0)
        };
        GCell {
            layer,
            x: coord(p.x, self.origin.x, self.nx),
            y: coord(p.y, self.origin.y, self.ny),
        }
    }

    /// The area of a gcell.
    pub fn cell_rect(&self, x: u16, y: u16) -> Rect<Dbu> {
        let min = self.origin + Point::new(self.gcell * i64::from(x), self.gcell * i64::from(y));
        Rect::new(min, min + Point::new(self.gcell, self.gcell))
    }

    /// Index of the edge leaving `c` forward in its layer's direction.
    fn edge(&self, c: GCell) -> usize {
        let l = c.layer.index() - self.min_layer.index();
        let per_layer = usize::from(self.nx) * usize::from(self.ny);
        l * per_layer + usize::from(c.y) * usize::from(self.nx) + usize::from(c.x)
    }

    fn direction(&self, layer: LayerId) -> Direction {
        self.directions[layer.index() - self.min_layer.index()]
    }

    const fn layers(&self) -> std::ops::RangeInclusive<usize> {
        self.min_layer.index()..=self.min_layer.index() + self.directions.len() - 1
    }

    /// Edges a path uses: each step within a layer uses the edge leaving its
    /// lower-coordinate end.
    fn path_edges<'a>(&'a self, path: &'a [GCell]) -> impl Iterator<Item = usize> + 'a {
        path.windows(2).filter_map(move |w| {
            let (a, b) = (w[0], w[1]);
            let moved = a.x != b.x || a.y != b.y;
            (a.layer == b.layer && moved).then(|| {
                let lower = if (a.x, a.y) < (b.x, b.y) { a } else { b };
                self.edge(lower)
            })
        })
    }

    fn add_tree(&mut self, paths: &[Vec<GCell>], delta: i32) {
        let edges: Vec<usize> = paths.iter().flat_map(|p| self.path_edges(p)).collect();
        for e in edges {
            let u = i32::from(self.usage[e]) + delta;
            self.usage[e] = u16::try_from(u.max(0)).unwrap_or(u16::MAX);
        }
    }

    fn overflowed(&self, e: usize) -> bool {
        self.usage[e] > self.capacity[e]
    }

    /// Total usage above capacity over all edges.
    pub fn overflow(&self) -> u64 {
        self.usage
            .iter()
            .zip(&self.capacity)
            .map(|(&u, &c)| u64::from(u.saturating_sub(c)))
            .sum()
    }

    fn edge_cost(&self, e: usize, penalty: f64) -> f64 {
        let over = (f64::from(self.usage[e]) + 1.0 - f64::from(self.capacity[e])).max(0.0);
        penalty.mul_add(over, 1.0 + f64::from(self.history[e]))
    }
}

/// Search costs of global routing, in gcells.
#[derive(Clone, Copy, Debug)]
pub struct GlobalCosts {
    /// Cost of changing layer.
    pub via: f64,
    /// Cost per unit of overflow an edge would reach.
    pub penalty: f64,
}

/// Global routes of all nets.
#[derive(Debug, Default)]
pub struct GlobalRoutes {
    /// The paths of each net's tree; each path starts on the tree built so far.
    pub paths: FxHashMap<NetId, Vec<Vec<GCell>>>,
}

/// A net's global route as a tree over its gcells, for finding the cells
/// that join two of them.
#[derive(Clone, Debug, Default)]
pub struct RouteTree {
    index: FxHashMap<GCell, usize>,
    cells: Vec<GCell>,
    parent: Vec<usize>,
    depth: Vec<usize>,
}

impl RouteTree {
    /// Builds the tree from paths that each start on the paths before them;
    /// a path that does not becomes a separate tree.
    pub fn new(paths: &[Vec<GCell>]) -> Self {
        let mut tree = Self::default();
        for path in paths {
            let mut previous = None;
            for &c in path {
                let id = match tree.index.get(&c) {
                    Some(&id) => id,
                    None => tree.push(c, previous),
                };
                previous = Some(id);
            }
        }
        tree
    }

    fn push(&mut self, c: GCell, parent: Option<usize>) -> usize {
        let id = self.cells.len();
        let _ = self.index.insert(c, id);
        self.cells.push(c);
        self.parent.push(parent.unwrap_or(id));
        self.depth.push(parent.map_or(0, |p| self.depth[p] + 1));
        id
    }

    /// Every gcell of the route, once.
    pub fn cells(&self) -> &[GCell] {
        &self.cells
    }

    /// The cells of the branch joining `from` to `to`, in order, or `None`
    /// when either is off the route or they are not joined.
    pub fn branch(&self, from: GCell, to: GCell) -> Option<Vec<GCell>> {
        let mut left = *self.index.get(&from)?;
        let mut right = *self.index.get(&to)?;
        let mut head = vec![from];
        let mut tail = vec![to];
        while left != right {
            let (side, cells) = if self.depth[left] >= self.depth[right] {
                (&mut left, &mut head)
            } else {
                (&mut right, &mut tail)
            };
            let up = self.parent[*side];
            if up == *side {
                return None;
            }
            *side = up;
            cells.push(self.cells[up]);
        }
        let _ = tail.pop();
        head.extend(tail.into_iter().rev());
        Some(head)
    }
}

/// A net to route globally: its pins' gcells.
#[derive(Clone, Debug)]
pub struct GlobalNet {
    /// The net.
    pub net: NetId,
    /// One gcell per pin.
    pub terminals: Vec<GCell>,
}

/// Routes `nets` and negotiates overflow for `rounds` rounds.
pub fn route_global(grid: &mut GlobalGrid, nets: &[GlobalNet], rounds: usize) -> GlobalRoutes {
    let mut costs = GlobalCosts {
        via: 1.0,
        penalty: 1.0,
    };
    let mut routes = GlobalRoutes::default();
    let all: Vec<usize> = (0..nets.len()).collect();
    route_batched(grid, nets, &all, costs, &mut routes);
    for _ in 0..rounds {
        if grid.overflow() == 0 {
            break;
        }
        for e in 0..grid.usage.len() {
            if grid.overflowed(e) {
                grid.history[e] += 1.0;
            }
        }
        let ripped: Vec<usize> = (0..nets.len())
            .filter(|&i| {
                routes.paths.get(&nets[i].net).is_some_and(|tree| {
                    tree.iter()
                        .any(|p| grid.path_edges(p).any(|e| grid.overflowed(e)))
                })
            })
            .collect();
        for &i in &ripped {
            if let Some(tree) = routes.paths.remove(&nets[i].net) {
                grid.add_tree(&tree, -1);
            }
        }
        costs.penalty *= 1.5;
        route_disjoint(grid, nets, &ripped, costs, &mut routes);
    }
    routes
}

/// Reroutes nets in parallel batches whose windows share no gcell, so every
/// net sees the usage of every other net that could compete with it.
fn route_disjoint(
    grid: &mut GlobalGrid,
    nets: &[GlobalNet],
    which: &[usize],
    costs: GlobalCosts,
    routes: &mut GlobalRoutes,
) {
    let (nx, ny) = (usize::from(grid.nx), usize::from(grid.ny));
    let mut pending = which.to_vec();
    let mut stamp = vec![0u32; nx * ny];
    let mut round = 0u32;
    while !pending.is_empty() {
        round += 1;
        let mut batch = Vec::new();
        let mut rest = Vec::new();
        for i in pending {
            let w = window(&nets[i].terminals, 3, grid.nx, grid.ny);
            let cells = || {
                (usize::from(w.2)..=usize::from(w.3)).flat_map(move |y| {
                    (usize::from(w.0)..=usize::from(w.1)).map(move |x| y * nx + x)
                })
            };
            if cells().all(|c| stamp[c] != round) {
                for c in cells() {
                    stamp[c] = round;
                }
                batch.push(i);
            } else {
                rest.push(i);
            }
        }
        let shared: &GlobalGrid = grid;
        let found: Vec<(usize, Vec<Vec<GCell>>)> = batch
            .par_iter()
            .map(|&i| (i, route_tree(shared, &nets[i].terminals, costs)))
            .collect();
        for (i, tree) in found {
            grid.add_tree(&tree, 1);
            let _ = routes.paths.insert(nets[i].net, tree);
        }
        pending = rest;
    }
}

/// Routes the given nets in parallel chunks, each against the usage left by
/// the chunks before it. Nets in one chunk do not see each other; the
/// rip-up rounds settle the overflow that causes.
fn route_batched(
    grid: &mut GlobalGrid,
    nets: &[GlobalNet],
    which: &[usize],
    costs: GlobalCosts,
    routes: &mut GlobalRoutes,
) {
    const CHUNK: usize = 4096;
    for chunk in which.chunks(CHUNK) {
        let shared: &GlobalGrid = grid;
        let found: Vec<(usize, Vec<Vec<GCell>>)> = chunk
            .par_iter()
            .map(|&i| (i, route_tree(shared, &nets[i].terminals, costs)))
            .collect();
        for (i, tree) in found {
            grid.add_tree(&tree, 1);
            let _ = routes.paths.insert(nets[i].net, tree);
        }
    }
}

/// Bounding box of cells, grown by `margin` and clamped to the grid, as
/// (x0, x1, y0, y1).
fn window(cells: &[GCell], margin: u16, nx: u16, ny: u16) -> (u16, u16, u16, u16) {
    let x0 = cells.iter().map(|c| c.x).min().unwrap_or(0);
    let x1 = cells.iter().map(|c| c.x).max().unwrap_or(0);
    let y0 = cells.iter().map(|c| c.y).min().unwrap_or(0);
    let y1 = cells.iter().map(|c| c.y).max().unwrap_or(0);
    (
        x0.saturating_sub(margin),
        (x1 + margin).min(nx - 1),
        y0.saturating_sub(margin),
        (y1 + margin).min(ny - 1),
    )
}

/// Edge, in gcells, of the buckets used to find tree cells near a connection.
const BUCKET: u16 = 4;

/// A tree through every terminal, as the paths that build it.
fn route_tree(grid: &GlobalGrid, terminals: &[GCell], costs: GlobalCosts) -> Vec<Vec<GCell>> {
    let mut terminals: Vec<GCell> = terminals.to_vec();
    terminals.sort_unstable();
    terminals.dedup();
    if terminals.is_empty() {
        return Vec::new();
    }
    let points: Vec<Point<Dbu>> = terminals
        .iter()
        .map(|c| Point::new(Dbu(i64::from(c.x)), Dbu(i64::from(c.y))))
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
        let mut margin = 2;
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
            margin = margin.saturating_mul(4);
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

/// Cheapest path from any source to `target` inside the window.
fn search(
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
    let mut best: FxHashMap<GCell, (f64, Option<GCell>)> = FxHashMap::default();
    let mut heap = BinaryHeap::new();
    for &c in sources {
        let _ = best.insert(c, (0.0, None));
        heap.push(Entry {
            f: h(c),
            g: 0.0,
            cell: c,
        });
    }
    while let Some(Entry { g, cell, .. }) = heap.pop() {
        if best.get(&cell).is_some_and(|&(b, _)| g > b) {
            continue;
        }
        if cell == target {
            let mut path = vec![cell];
            let mut cur = cell;
            while let Some(&(_, Some(prev))) = best.get(&cur) {
                path.push(prev);
                cur = prev;
            }
            path.reverse();
            return Some(path);
        }
        let mut push = |next: GCell, step: f64| {
            let ng = g + step;
            if best.get(&next).is_none_or(|&(b, _)| ng < b) {
                let _ = best.insert(next, (ng, Some(cell)));
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
                    push(next, grid.edge_cost(grid.edge(next), costs.penalty));
                }
                if x < win.1 {
                    push(
                        GCell { x: x + 1, ..cell },
                        grid.edge_cost(grid.edge(cell), costs.penalty),
                    );
                }
            }
            Direction::Vertical => {
                if y > win.2 {
                    let next = GCell { y: y - 1, ..cell };
                    push(next, grid.edge_cost(grid.edge(next), costs.penalty));
                }
                if y < win.3 {
                    push(
                        GCell { y: y + 1, ..cell },
                        grid.edge_cost(grid.edge(cell), costs.penalty),
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

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(x: u16, y: u16) -> GCell {
        GCell {
            layer: LayerId::new(1),
            x,
            y,
        }
    }

    #[test]
    fn branch_walks_from_one_cell_to_the_other_through_their_junction() {
        let paths = vec![
            vec![cell(0, 0)],
            vec![cell(0, 0), cell(0, 1), cell(0, 2), cell(1, 2)],
            vec![cell(0, 2), cell(0, 3), cell(0, 4)],
            vec![cell(0, 0), cell(1, 0), cell(2, 0)],
        ];

        let tree = RouteTree::new(&paths);

        assert_eq!(
            tree.branch(cell(1, 2), cell(0, 4)),
            Some(vec![cell(1, 2), cell(0, 2), cell(0, 3), cell(0, 4)])
        );
        assert_eq!(
            tree.branch(cell(2, 0), cell(0, 1)),
            Some(vec![cell(2, 0), cell(1, 0), cell(0, 0), cell(0, 1)])
        );
    }

    #[test]
    fn branch_is_none_for_cells_on_separate_trees() {
        let paths = vec![vec![cell(0, 0), cell(1, 0)], vec![cell(5, 5)]];

        let tree = RouteTree::new(&paths);

        assert_eq!(tree.branch(cell(1, 0), cell(5, 5)), None);
        assert_eq!(tree.branch(cell(1, 0), cell(9, 9)), None);
    }
}
