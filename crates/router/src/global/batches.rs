//! Routing nets in parallel batches whose search windows do not meet.

use super::coarse::{Corridor, Plan, SCALE};
use super::maze::{OutgrowsArea, route_tree, route_tree_within};
use super::pattern::route_pattern;
use super::{GCell, GlobalCosts, GlobalGrid, GlobalNet, GlobalRoutes, NetTree, PlacedTree};
use crate::tech::{NetKind, index_u32};
use pare_common::convert::SaturatingFrom;
use rayon::prelude::*;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU32, Ordering};

/// Half-perimeter of a net's terminals' bounding box, in gcells.
pub(super) fn span(cells: &[GCell]) -> u32 {
    window(cells, 0, u16::MAX, u16::MAX)
        .map_or(0, |w| u32::from(w.x1 - w.x0) + u32::from(w.y1 - w.y0))
}

/// Nets to reroute in one batch: their indices in the order to route them,
/// with each net's search margin and the coarse plan made for each.
pub(super) struct Reroute<'a> {
    pub(super) which: &'a [usize],
    pub(super) margins: &'a [u16],
    /// Aligned with `which`.
    pub(super) plans: &'a [Option<Plan>],
}

/// Reroutes nets with the result of routing them one after another in the
/// order of `which`, routing at once those whose search areas, grown by a
/// gcell, share no block of [`BATCH_BLOCK`] gcells: each net waits only for
/// the nets before it whose areas meet its own, so it sees the usage of
/// every earlier net that could compete with it, while nets elsewhere keep
/// the threads busy. A net's area is the window around its pins together
/// with its corridor's cells. A net with a connection that has no path
/// there is routed again after the others, in order, with its search free
/// to widen, so no search reads usage that a net routed at the same time
/// changes and every run gives the same routes.
pub(super) fn route_disjoint(
    grid: &GlobalGrid,
    nets: &[GlobalNet],
    reroute: &Reroute<'_>,
    costs: GlobalCosts,
    routes: &mut GlobalRoutes,
) {
    let Reroute {
        which,
        margins,
        plans,
    } = *reroute;
    let (waits, next) = dependencies(grid, nets, reroute);
    let ready: Vec<usize> = (0..which.len())
        .filter(|&k| waits[k].load(Ordering::Relaxed) == 0)
        .collect();
    let schedule = Schedule {
        grid,
        nets,
        which,
        margins,
        plans,
        costs,
        waits,
        next,
        trees: (0..which.len()).map(|_| OnceLock::new()).collect(),
    };
    let shared = &schedule;
    rayon::scope(|scope| {
        for k in ready {
            scope.spawn(move |scope| shared.run(scope, k));
        }
    });
    for (k, tree) in schedule.trees.into_iter().enumerate() {
        let i = which[k];
        let placed = if let Some(Ok(placed)) = tree.into_inner() {
            placed
        } else {
            let paths = route_tree(grid, &nets[i], margins[i], costs, plans[k].as_ref());
            grid.place(NetTree {
                net: nets[i].clone(),
                paths,
            })
        };
        let _ = routes.trees.insert(nets[i].net, placed);
    }
}

/// For each net of `reroute`, how many earlier nets its search area meets,
/// and for each, the later nets whose areas meet its own.
fn dependencies(
    grid: &GlobalGrid,
    nets: &[GlobalNet],
    reroute: &Reroute<'_>,
) -> (Vec<AtomicU32>, Vec<Vec<u32>>) {
    let Reroute {
        which,
        margins,
        plans,
    } = *reroute;
    let blocks_x = usize::from(grid.nx.div_ceil(BATCH_BLOCK));
    let blocks_y = usize::from(grid.ny.div_ceil(BATCH_BLOCK));
    let mut last: Vec<Option<u32>> = vec![None; blocks_x * blocks_y];
    let mut waits: Vec<AtomicU32> = Vec::with_capacity(which.len());
    let mut next: Vec<Vec<u32>> = vec![Vec::new(); which.len()];
    for (k, &i) in which.iter().enumerate() {
        let area = window(
            &nets[i].terminals,
            margins[i].saturating_add(1),
            grid.nx,
            grid.ny,
        );
        let mut blocks = area.map_or_else(Vec::new, |a| blocks_of(a, blocks_x));
        if let Some(plan) = &plans[k] {
            blocks.extend(corridor_blocks(plan.corridor(), grid, blocks_x));
        }
        blocks.sort_unstable();
        blocks.dedup();
        let mut before: Vec<u32> = Vec::new();
        for b in blocks {
            before.extend(last[b].replace(index_u32(k)));
        }
        before.sort_unstable();
        before.dedup();
        for &p in &before {
            next[p as usize].push(index_u32(k));
        }
        waits.push(AtomicU32::new(index_u32(before.len())));
    }
    (waits, next)
}

/// The blocks of [`BATCH_BLOCK`] gcells a window of gcells meets.
fn blocks_of(w: GcellWindow, blocks_x: usize) -> Vec<usize> {
    let block = |v: u16| usize::from(v / BATCH_BLOCK);
    let mut out = Vec::new();
    for y in block(w.y0)..=block(w.y1) {
        for x in block(w.x0)..=block(w.x1) {
            out.push(y * blocks_x + x);
        }
    }
    out
}

/// The blocks of [`BATCH_BLOCK`] gcells a corridor's cells meet, each cell
/// grown by a gcell for the edges beside the gcells a route takes.
fn corridor_blocks(corridor: &Corridor, grid: &GlobalGrid, blocks_x: usize) -> Vec<usize> {
    corridor
        .cells()
        .iter()
        .flat_map(|&(cx, cy)| {
            let cell = GcellWindow {
                x0: (cx * SCALE).saturating_sub(1),
                x1: (cx * SCALE + SCALE).min(grid.nx - 1),
                y0: (cy * SCALE).saturating_sub(1),
                y1: (cy * SCALE + SCALE).min(grid.ny - 1),
            };
            blocks_of(cell, blocks_x)
        })
        .collect()
}

/// The nets [`route_disjoint`] reroutes, what each waits for and what it
/// found.
struct Schedule<'a> {
    pub(super) grid: &'a GlobalGrid,
    pub(super) nets: &'a [GlobalNet],
    pub(super) which: &'a [usize],
    pub(super) margins: &'a [u16],
    pub(super) plans: &'a [Option<Plan>],
    pub(super) costs: GlobalCosts,
    /// Earlier nets each one still waits for.
    waits: Vec<AtomicU32>,
    /// The later nets waiting for each one.
    pub(super) next: Vec<Vec<u32>>,
    pub(super) trees: Vec<OnceLock<Result<PlacedTree, OutgrowsArea>>>,
}

impl Schedule<'_> {
    /// Routes the `k`th net inside its area and places its demand, or leaves
    /// it for later when its area holds no route, then starts every later net
    /// that was waiting only for it.
    pub(super) fn run<'s>(&'s self, scope: &rayon::Scope<'s>, k: usize) {
        let i = self.which[k];
        let tree = route_tree_within(
            self.grid,
            &self.nets[i],
            self.margins[i],
            self.costs,
            self.plans[k].as_ref(),
        );
        let placed = tree.map(|paths| {
            self.grid.place(NetTree {
                net: self.nets[i].clone(),
                paths,
            })
        });
        let _ = self.trees[k].set(placed);
        for &later in &self.next[k] {
            let later = later as usize;
            if self.waits[later].fetch_sub(1, Ordering::AcqRel) == 1 {
                scope.spawn(move |scope| self.run(scope, later));
            }
        }
    }
}

/// `v` as a 16-bit grid coordinate; `RouteTech::new` refuses a die whose
/// grid would span more.
pub(crate) fn grid_index(v: usize) -> u16 {
    debug_assert!(u16::try_from(v).is_ok(), "grid coordinate {v} past 16 bits");
    u16::saturating_from(v)
}

/// Seconds one rip-up round spends in each of its steps.
#[derive(Debug, Default)]
pub(super) struct RoundTimes {
    pub(super) planning: f64,
    pub(super) routing: f64,
    pub(super) refreshing: f64,
}

/// Edge, in gcells, of the blocks in which the pin boxes and search areas
/// of nets rerouted together are compared.
const BATCH_BLOCK: u16 = 4;

/// Splits `ripped` into levels, keeping its order within each, whose nets'
/// pin boxes share no block of [`BATCH_BLOCK`] gcells: each net goes one
/// level above the highest net before it that meets it. As CUGR's batches
/// of nets whose boxes do not overlap, a level's nets are planned at once
/// against the coarse grid the levels before it left, so nets planned
/// together start apart rather than all taking the same free corridor.
pub(super) fn disjoint_levels(
    grid: &GlobalGrid,
    nets: &[GlobalNet],
    ripped: &[usize],
) -> Vec<Vec<usize>> {
    let blocks_x = usize::from(grid.nx.div_ceil(BATCH_BLOCK));
    let blocks_y = usize::from(grid.ny.div_ceil(BATCH_BLOCK));
    let mut height = vec![0_usize; blocks_x * blocks_y];
    let mut levels: Vec<Vec<usize>> = Vec::new();
    for &i in ripped {
        let area = window(&nets[i].terminals, 0, grid.nx, grid.ny);
        let blocks = area.map_or_else(Vec::new, |a| blocks_of(a, blocks_x));
        let level = blocks.iter().map(|&b| height[b]).max().unwrap_or(0);
        for &b in &blocks {
            height[b] = level + 1;
        }
        if levels.len() <= level {
            levels.resize_with(level + 1, Vec::new);
        }
        levels[level].push(i);
    }
    levels
}

/// The coarse cells whose free tracks routes through `trees` change: those
/// of every gcell they pass and of the gcells beside it, whose edges they
/// share.
pub(super) fn touched_coarse_cells<'a>(
    grid: &GlobalGrid,
    trees: impl Iterator<Item = &'a [Vec<GCell>]>,
) -> Vec<(u16, u16)> {
    let mut cells: Vec<(u16, u16)> = trees
        .flatten()
        .flatten()
        .flat_map(|c| {
            let xs = c.x.saturating_sub(1)..=(c.x + 1).min(grid.nx - 1);
            let ys = c.y.saturating_sub(1)..=(c.y + 1).min(grid.ny - 1);
            ys.flat_map(move |y| xs.clone().map(move |x| (x / SCALE, y / SCALE)))
        })
        .collect();
    cells.sort_unstable();
    cells.dedup();
    cells
}

/// Pattern-routes every net in [`initial_order`], in parallel chunks, each
/// against the usage left by the chunks before it. Nets in one chunk do not
/// see each other; the rip-up rounds settle the overflow that causes.
pub(super) fn route_initial(
    grid: &GlobalGrid,
    nets: &[GlobalNet],
    costs: GlobalCosts,
    routes: &mut GlobalRoutes,
) {
    for batch in initial_order(nets).chunks(INITIAL_CHUNK) {
        let shared: &GlobalGrid = grid;
        let found: Vec<(usize, Vec<Vec<GCell>>)> = batch
            .par_iter()
            .map(|&i| (i, route_pattern(shared, &nets[i], costs)))
            .collect();
        for (i, paths) in found {
            let placed = grid.place(NetTree {
                net: nets[i].clone(),
                paths,
            });
            let _ = routes.trees.insert(nets[i].net, placed);
        }
    }
}

/// The order nets are first routed in: clock trunks, then the rest, each
/// shortest first.
// NOTE: GlobalRouter::findNets puts the clock trunks first too, but
// orders each group by name; PARE keeps its shortest-first order within each.
pub(super) fn initial_order(nets: &[GlobalNet]) -> Vec<usize> {
    let mut order: Vec<usize> = (0..nets.len()).collect();
    order.sort_by_key(|&i| {
        let is_trunk = nets[i].kind == NetKind::ClockTrunk;
        (!is_trunk, span(&nets[i].terminals), i)
    });
    order
}

/// Nets pattern-routed together in one chunk of the initial routing; rayon
/// spreads each chunk over the threads. Fixed rather than scaled by the
/// thread count, so that the routes do not depend on the machine: 28
/// threads times 32 nets each, which keeps a 28-thread machine's routes and
/// wall time. PARE tunable, no `OpenROAD` counterpart.
const INITIAL_CHUNK: usize = 896;

/// An inclusive box of gcell columns `x0..=x1` and rows `y0..=y1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct GcellWindow {
    pub(crate) x0: u16,
    pub(crate) x1: u16,
    pub(crate) y0: u16,
    pub(crate) y1: u16,
}

impl GcellWindow {
    /// Every gcell of a grid of `nx` columns and `ny` rows.
    pub(crate) const fn whole(nx: u16, ny: u16) -> Self {
        Self {
            x0: 0,
            x1: nx - 1,
            y0: 0,
            y1: ny - 1,
        }
    }

    /// Whether the gcell at column `x` and row `y` lies inside.
    pub(crate) fn contains(self, x: u16, y: u16) -> bool {
        (self.x0..=self.x1).contains(&x) && (self.y0..=self.y1).contains(&y)
    }

    /// The smallest window holding both.
    pub(crate) fn union(self, other: Self) -> Self {
        Self {
            x0: self.x0.min(other.x0),
            x1: self.x1.max(other.x1),
            y0: self.y0.min(other.y0),
            y1: self.y1.max(other.y1),
        }
    }

    /// The window of the gcell `c` alone.
    pub(super) const fn of(c: GCell) -> Self {
        Self {
            x0: c.x,
            x1: c.x,
            y0: c.y,
            y1: c.y,
        }
    }

    /// The window of the gcells `a` and `b` and those between.
    pub(crate) fn spanning(a: GCell, b: GCell) -> Self {
        Self::of(a).union(Self::of(b))
    }

    /// This window grown by `margin` on every side and clamped to a grid of
    /// `nx` columns and `ny` rows; the growth saturates, which clamps it
    /// exactly, as the grid ends below `u16::MAX`.
    pub(crate) fn grown(self, margin: u16, nx: u16, ny: u16) -> Self {
        Self {
            x0: self.x0.saturating_sub(margin),
            x1: self.x1.saturating_add(margin).min(nx - 1),
            y0: self.y0.saturating_sub(margin),
            y1: self.y1.saturating_add(margin).min(ny - 1),
        }
    }
}

/// Bounding box of `cells`, grown by `margin` and clamped to the grid; none
/// for no cells.
pub(super) fn window(cells: &[GCell], margin: u16, nx: u16, ny: u16) -> Option<GcellWindow> {
    cells
        .iter()
        .map(|&c| GcellWindow::of(c))
        .reduce(GcellWindow::union)
        .map(|w| w.grown(margin, nx, ny))
}
