//! Global routing on a grid of gcells.
//!
//! Each routable layer gets a capacity on every gcell edge in its preferred
//! direction: the number of its tracks crossing that edge that no obstacle
//! blocks. Nets are routed as trees of A* paths through the 3D grid. An
//! edge costs its length, a logistic congestion term that rises as its
//! usage nears capacity, its accumulated overflow history, and a penalty per
//! unit of overflow that grows every round, in the manner of `PathFinder`.
//! Nets crossing overflowed edges are ripped up and rerouted, each time
//! with a search area grown around their pins so they can detour further
//! around congestion. The cells each net passes through become its guide
//! for detailed routing.

mod maze;

use crate::occupancy::{Occupancy, Owner, TrackRef};
use crate::tech::RouteTech;
use maze::route_tree;
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use rustc_hash::FxHashMap;

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

    /// The gcell an index from [`Self::edge`] names.
    fn cell_of(&self, index: usize) -> GCell {
        let per_layer = usize::from(self.nx) * usize::from(self.ny);
        let rest = index % per_layer;
        GCell {
            layer: LayerId::new(self.min_layer.index() + index / per_layer),
            x: u16::try_from(rest % usize::from(self.nx)).unwrap_or(u16::MAX),
            y: u16::try_from(rest / usize::from(self.nx)).unwrap_or(u16::MAX),
        }
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

    /// Cost of one more net on edge `e`.
    fn edge_cost(&self, e: usize, costs: GlobalCosts) -> f64 {
        let excess = f64::from(self.usage[e]) + 1.0 - f64::from(self.capacity[e]);
        let congestion = costs.congestion / (1.0 + (-CONGESTION_SLOPE * excess).exp());
        let overflow = costs.penalty * excess.max(0.0);
        1.0 + f64::from(self.history[e]) + congestion + overflow
    }
}

/// Search costs of global routing, in gcells.
#[derive(Clone, Copy, Debug)]
pub struct GlobalCosts {
    /// Cost of changing layer.
    pub via: f64,
    /// Cost of an edge used to capacity, approached smoothly from below.
    pub congestion: f64,
    /// Cost per unit of overflow an edge would reach.
    pub penalty: f64,
}

/// Steepness, per track, of the logistic congestion cost around capacity.
const CONGESTION_SLOPE: f64 = 1.0;

/// Search margin, in gcells, around a net's first route's connections.
const FIRST_MARGIN: u16 = 2;

/// Gcells the search margin grows by each time a net is ripped up.
const MARGIN_STEP: u16 = 2;

/// Largest search margin, in gcells; wider margins leave too few nets with
/// disjoint search areas to reroute in parallel.
const MAX_MARGIN: u16 = 12;

/// Growth of the overflow penalty per round.
const PENALTY_GROWTH: f64 = 1.5;

/// Global routes of all nets.
#[derive(Clone, Debug, Default)]
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

/// Routes `nets` and negotiates overflow for at most `rounds` rounds.
///
/// Negotiation stops early once [`STALL_ROUNDS`] pass without a new lowest
/// overflow. Returns the routes of the round with the lowest overflow, and
/// leaves the grid's usage at theirs.
pub fn route_global(grid: &mut GlobalGrid, nets: &[GlobalNet], rounds: usize) -> GlobalRoutes {
    let mut costs = GlobalCosts {
        via: 1.0,
        congestion: 1.0,
        penalty: 1.0,
    };
    let mut routes = GlobalRoutes::default();
    let mut margins = vec![FIRST_MARGIN; nets.len()];
    let all: Vec<usize> = (0..nets.len()).collect();
    route_batched(grid, nets, &all, costs, &mut routes);
    let mut best = Snapshot::of(grid, &routes);
    let mut stalled = 0;
    for _ in 0..rounds {
        if best.overflow == 0 || stalled >= STALL_ROUNDS {
            break;
        }
        for e in 0..grid.usage.len() {
            let over = grid.usage[e].saturating_sub(grid.capacity[e]);
            grid.history[e] += f32::from(over);
        }
        let mut ripped: Vec<usize> = (0..nets.len())
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
            margins[i] = margins[i].saturating_add(MARGIN_STEP).min(MAX_MARGIN);
        }
        ripped.sort_by_key(|&i| (span(&nets[i].terminals), i));
        costs.penalty *= PENALTY_GROWTH;
        let start = std::time::Instant::now();
        route_disjoint(grid, nets, &ripped, &margins, costs, &mut routes);
        log::debug!(
            "global round: {} nets ripped, overflow {}, {:.1}s",
            ripped.len(),
            grid.overflow(),
            start.elapsed().as_secs_f64()
        );
        if grid.overflow() < best.overflow {
            best = Snapshot::of(grid, &routes);
            stalled = 0;
        } else {
            stalled += 1;
        }
    }
    if grid.overflow() > best.overflow {
        grid.usage = best.usage;
        routes = best.routes;
    }
    routes
}

/// Rounds without a new lowest overflow after which negotiation stops.
const STALL_ROUNDS: usize = 8;

/// The routes and edge usage of one round.
struct Snapshot {
    overflow: u64,
    usage: Vec<u16>,
    routes: GlobalRoutes,
}

impl Snapshot {
    fn of(grid: &GlobalGrid, routes: &GlobalRoutes) -> Self {
        Self {
            overflow: grid.overflow(),
            usage: grid.usage.clone(),
            routes: routes.clone(),
        }
    }
}

/// Half-perimeter of a net's terminals' bounding box, in gcells.
fn span(cells: &[GCell]) -> u32 {
    let (x0, x1, y0, y1) = window(cells, 0, u16::MAX, u16::MAX);
    u32::from(x1 - x0) + u32::from(y1 - y0)
}

/// Reroutes nets in parallel batches whose search areas share no block of
/// [`BATCH_BLOCK`] gcells, so every net sees the usage of every other net
/// that could compete with it.
fn route_disjoint(
    grid: &mut GlobalGrid,
    nets: &[GlobalNet],
    which: &[usize],
    margins: &[u16],
    costs: GlobalCosts,
    routes: &mut GlobalRoutes,
) {
    let blocks_x = usize::from(grid.nx.div_ceil(BATCH_BLOCK));
    let blocks_y = usize::from(grid.ny.div_ceil(BATCH_BLOCK));
    let areas: Vec<(usize, usize, usize, usize)> = which
        .iter()
        .map(|&i| {
            let w = window(&nets[i].terminals, margins[i], grid.nx, grid.ny);
            let block = |v: u16| usize::from(v / BATCH_BLOCK);
            (block(w.0), block(w.1), block(w.2), block(w.3))
        })
        .collect();
    let mut pending: Vec<usize> = (0..which.len()).collect();
    let mut stamp = vec![0u32; blocks_x * blocks_y];
    let mut round = 0u32;
    while !pending.is_empty() {
        round += 1;
        let mut batch = Vec::new();
        let mut rest = Vec::new();
        for k in pending {
            let (x0, x1, y0, y1) = areas[k];
            let free = (y0..=y1).all(|y| (x0..=x1).all(|x| stamp[y * blocks_x + x] != round));
            if free {
                for y in y0..=y1 {
                    stamp[y * blocks_x + x0..=y * blocks_x + x1].fill(round);
                }
                batch.push(which[k]);
            } else {
                rest.push(k);
            }
        }
        let shared: &GlobalGrid = grid;
        let found: Vec<(usize, Vec<Vec<GCell>>)> = batch
            .par_iter()
            .map(|&i| (i, route_tree(shared, &nets[i].terminals, margins[i], costs)))
            .collect();
        for (i, tree) in found {
            grid.add_tree(&tree, 1);
            let _ = routes.paths.insert(nets[i].net, tree);
        }
        pending = rest;
    }
    log::debug!("global reroute of {} nets in {round} batches", which.len());
}

/// Edge, in gcells, of the blocks that keep the search areas of nets
/// rerouted together apart.
const BATCH_BLOCK: u16 = 4;

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
            .map(|&i| {
                (
                    i,
                    route_tree(shared, &nets[i].terminals, FIRST_MARGIN, costs),
                )
            })
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
