//! Negotiated rip-up and reroute rounds over the whole grid, as in `PathFinder`.

use super::coarse::{CoarseGrid, Plan};
use super::tracks::Tracks;
use super::{
    FIRST_MARGIN, GlobalCosts, GlobalGrid, GlobalNet, GlobalRoutes, MARGIN_STEP, MAX_MARGIN,
    NetTree, PlacedTree, Reroute, RoundTimes, disjoint_levels, route_disjoint, route_initial, span,
    touched_coarse_cells,
};
use pare_common::convert::NearestFrom;
use pare_common::db::indices::NetId;
use pare_common::geom::point::Point;
use rayon::prelude::*;
use rustc_hash::FxHashMap;

/// Routes `nets` and negotiates overflow for at most `rounds` rounds.
///
/// The history of overflow on each edge grows every round. The overflow
/// penalty grows by `PENALTY_STEP` after a round that lowers the overflow
/// below the best round's and shrinks by it after any other: more pressure
/// clears the last overflow of a routable design, while pressure that keeps
/// growing on overflow that cannot be cleared drives nets into detours that
/// overflow more than they relieve. Negotiation stops early once
/// `STALL_ROUNDS` pass without lowering the overflow by
/// `MIN_PROGRESS` of what it was when it last did. Returns the routes of
/// the round with the lowest overflow, and leaves the grid's usage at
/// theirs.
pub fn route_global(grid: &mut GlobalGrid, nets: &[GlobalNet], rounds: usize) -> GlobalRoutes {
    let mut routes = GlobalRoutes::default();
    route_initial(grid, nets, FIRST_COSTS, &mut routes);
    negotiate(grid, nets, &mut routes, rounds);
    log_zero_capacity_crossings(grid, &routes);
    log_overflow_regions(grid);
    routes
}

/// Routes `nets` against the usage `grid` already holds for every other
/// net, as an incremental global router does after edits.
///
/// Pattern routing comes first; then, if any of them crosses an overflowed
/// edge, they alone are ripped up and rerouted for at most `rounds` rounds,
/// as [`route_global`] negotiates.
pub fn reroute(grid: &mut GlobalGrid, nets: &[GlobalNet], rounds: usize) -> GlobalRoutes {
    let mut routes = GlobalRoutes::default();
    route_initial(grid, nets, FIRST_COSTS, &mut routes);
    let crosses_overflow = nets.iter().any(|n| {
        routes
            .trees
            .get(&n.net)
            .is_some_and(|t| grid.crosses_overflow(t))
    });
    if crosses_overflow {
        negotiate(grid, nets, &mut routes, rounds);
    }
    routes
}

/// The costs of the first routing of a net. PARE tunables: grt cugr weighs
/// a via at `weight_via_number` (4) wire units, and has no overflow penalty.
pub(super) const FIRST_COSTS: GlobalCosts = GlobalCosts {
    via: 1.0,
    congestion: 1.0,
    penalty: 1.0,
};

/// Rips up and reroutes the nets of `nets` that cross overflowed edges, for
/// at most `rounds` rounds, as [`route_global`] describes; leaves the routes
/// and the grid's usage at the round with the lowest overflow.
// NOTE: CUGR::route runs pattern, detoured pattern and sparse maze stages and
// keeps no history; PARE negotiates PathFinder-style, its own scheme.
pub(super) fn negotiate(
    grid: &mut GlobalGrid,
    nets: &[GlobalNet],
    routes: &mut GlobalRoutes,
    rounds: usize,
) {
    let mut margins = vec![FIRST_MARGIN; nets.len()];
    let mut coarse = CoarseGrid::new(grid);
    let mut negotiation = Negotiation::new(grid);
    for _ in 0..rounds {
        if negotiation.is_over() {
            break;
        }
        accrue_history(grid);
        let ripped = rip_up_overflowed(grid, nets, routes, &mut negotiation.best, &mut margins);
        let start = std::time::Instant::now();
        coarse.refresh(grid);
        let levels = disjoint_levels(grid, nets, &ripped);
        let round = RerouteRound {
            nets,
            margins: &margins,
            costs: negotiation.costs,
        };
        let spent = reroute_levels(grid, &mut coarse, &round, &levels, routes);
        log::debug!(
            "global round: {} nets ripped in {} levels (largest {}), overflow {}, {:.1}s: \
             {:.1}s planning, {:.1}s routing, {:.1}s refreshing",
            ripped.len(),
            levels.len(),
            levels.iter().map(Vec::len).max().unwrap_or(0),
            grid.overflow().rounded(),
            start.elapsed().as_secs_f64(),
            spent.planning,
            spent.routing,
            spent.refreshing
        );
        negotiation.after_round(grid);
    }
    negotiation.finish(grid, routes);
}

/// Adds each edge's overflow to its history.
fn accrue_history(grid: &mut GlobalGrid) {
    for e in 0..grid.usage.len() {
        let over = grid.usage.get(e).saturating_sub(grid.capacity[e]);
        grid.history[e] += f32::nearest_from(over.to_f64());
    }
}

/// Rips up the nets of `nets` whose routes cross an overflowed edge,
/// shelving each route with `best` and widening its search margin, and
/// returns them, the smallest spans first.
fn rip_up_overflowed(
    grid: &GlobalGrid,
    nets: &[GlobalNet],
    routes: &mut GlobalRoutes,
    best: &mut Best,
    margins: &mut [u16],
) -> Vec<usize> {
    let mut ripped: Vec<usize> = (0..nets.len())
        .filter(|&i| {
            routes
                .trees
                .get(&nets[i].net)
                .is_some_and(|t| grid.crosses_overflow(t))
        })
        .collect();
    for &i in &ripped {
        if let Some(placed) = routes.trees.remove(&nets[i].net) {
            best.shelve(grid.lift(placed));
        }
        margins[i] = margins[i].saturating_add(MARGIN_STEP).min(MAX_MARGIN);
    }
    ripped.sort_by_key(|&i| (span(&nets[i].terminals), i));
    ripped
}

/// What every level of a round's rerouting shares: the nets, their search
/// margins and the round's costs.
struct RerouteRound<'a> {
    pub(super) nets: &'a [GlobalNet],
    pub(super) margins: &'a [u16],
    pub(super) costs: GlobalCosts,
}

/// Plans and reroutes each of `levels` in turn, its nets at once, keeping
/// the coarse grid up to date; returns where the time went.
fn reroute_levels(
    grid: &GlobalGrid,
    coarse: &mut CoarseGrid,
    round: &RerouteRound<'_>,
    levels: &[Vec<usize>],
    routes: &mut GlobalRoutes,
) -> RoundTimes {
    let nets = round.nets;
    let mut spent = RoundTimes::default();
    for level in levels {
        let planning = std::time::Instant::now();
        let plans: Vec<Option<Plan>> = level
            .par_iter()
            .map(|&i| coarse.plan(grid, &nets[i]))
            .collect();
        spent.planning += planning.elapsed().as_secs_f64();
        let reroute = Reroute {
            which: level,
            margins: round.margins,
            plans: &plans,
        };
        let routing = std::time::Instant::now();
        route_disjoint(grid, nets, &reroute, round.costs, routes);
        spent.routing += routing.elapsed().as_secs_f64();
        let refreshing = std::time::Instant::now();
        let touched = touched_coarse_cells(
            grid,
            level
                .iter()
                .filter_map(|&i| routes.trees.get(&nets[i].net))
                .map(PlacedTree::paths),
        );
        coarse.refresh_cells(grid, &touched);
        spent.refreshing += refreshing.elapsed().as_secs_f64();
    }
    spent
}

/// Where negotiation stands between rounds: the round with the lowest
/// overflow, the costs the next round routes with, and the overflow when
/// it last made progress, with the rounds since; `docs/global-routing.md`
/// draws its states.
struct Negotiation {
    pub(super) best: Best,
    pub(super) costs: GlobalCosts,
    progress_from: Tracks,
    stalled: usize,
}

impl Negotiation {
    pub(super) fn new(grid: &GlobalGrid) -> Self {
        let best = Best::of(grid);
        Self {
            progress_from: best.overflow,
            best,
            costs: FIRST_COSTS,
            stalled: 0,
        }
    }

    /// Whether negotiation is over: the best round has no overflow, or
    /// [`STALL_ROUNDS`] rounds have passed without enough progress.
    fn is_over(&self) -> bool {
        self.best.overflow == Tracks::ZERO || self.stalled >= STALL_ROUNDS
    }

    /// Takes in the overflow a round left on `grid`: keeps the round if it
    /// is the best yet and raises the penalty, else lowers it, and counts
    /// the round as progress or as stalled.
    fn after_round(&mut self, grid: &GlobalGrid) {
        let overflow = grid.overflow();
        if overflow < self.best.overflow {
            self.costs.penalty *= PENALTY_STEP;
            self.best = Best::of(grid);
        } else {
            self.costs.penalty /= PENALTY_STEP;
        }
        if overflow.to_f64() <= self.progress_from.to_f64() * (1.0 - MIN_PROGRESS) {
            self.progress_from = overflow;
            self.stalled = 0;
        } else {
            self.stalled += 1;
        }
    }

    /// Leaves `grid` and `routes` at the best round, where the last was
    /// worse.
    fn finish(self, grid: &mut GlobalGrid, routes: &mut GlobalRoutes) {
        if grid.overflow() > self.best.overflow {
            self.best.restore(grid, routes);
        }
    }
}

/// Logs, at debug level, every net whose route uses edges without capacity.
fn log_zero_capacity_crossings(grid: &GlobalGrid, routes: &GlobalRoutes) {
    if !log::log_enabled!(log::Level::Debug) {
        return;
    }
    for (net, tree) in &routes.trees {
        let blocked = tree
            .paths()
            .iter()
            .flat_map(|p| grid.path_edges(p))
            .filter(|&e| grid.capacity[e] == Tracks::ZERO)
            .count();
        if blocked > 0 {
            log::debug!("net {} crosses {blocked} zero-capacity edges", net.index());
        }
    }
}

/// Logs, at debug level, the regions of [`OVERFLOW_REGION`] by
/// [`OVERFLOW_REGION`] gcells holding the most overflow, with their share of
/// the whole.
fn log_overflow_regions(grid: &GlobalGrid) {
    if !log::log_enabled!(log::Level::Debug) {
        return;
    }
    let total = grid.overflow().to_f64();
    if total <= 0.0 {
        return;
    }
    let mut regions: FxHashMap<(u16, u16), f64> = FxHashMap::default();
    for (e, (used, &cap)) in grid.usage.iter().zip(&grid.capacity).enumerate() {
        let over = used.saturating_sub(cap).to_f64();
        if over > 0.0 {
            let c = grid.cell_of(e);
            *regions
                .entry((c.x / OVERFLOW_REGION, c.y / OVERFLOW_REGION))
                .or_default() += over;
        }
    }
    let mut worst: Vec<((u16, u16), f64)> = regions.into_iter().collect();
    worst.sort_by(|a, b| b.1.total_cmp(&a.1));
    let side = grid.gcell * i64::from(OVERFLOW_REGION);
    for ((x, y), over) in worst.into_iter().take(LOGGED_OVERFLOW_REGIONS) {
        let at = grid.origin + Point::new(side * i64::from(x), side * i64::from(y));
        log::debug!(
            "GR overflow {over:.0} ({:.1}%) in the region at ({}, {})",
            100.0 * over / total,
            at.x.get(),
            at.y.get()
        );
    }
}

/// Edge, in gcells, of the regions [`log_overflow_regions`] sums overflow
/// over.
const OVERFLOW_REGION: u16 = 20;

/// How many of the regions with the most overflow are logged.
const LOGGED_OVERFLOW_REGIONS: usize = 12;

/// Rounds without enough progress after which negotiation stops. PARE
/// tunable, no `OpenROAD` counterpart.
const STALL_ROUNDS: usize = 3;

/// Share of the overflow a round must remove, against the overflow when
/// it last made progress, to count as progress. PARE tunable, no
/// `OpenROAD` counterpart.
const MIN_PROGRESS: f64 = 0.005;

/// Factor the overflow penalty grows by after a round that lowers the
/// overflow below the best round's, and shrinks by after any other. PARE
/// tunable, no `OpenROAD` counterpart.
const PENALTY_STEP: f64 = 1.5;

/// The round with the lowest overflow so far: its edge usage, and the
/// routes of that round that later rounds have since replaced. Every other
/// net still has the route it had then.
struct Best {
    pub(super) overflow: Tracks,
    pub(super) usage: Vec<Tracks>,
    shelved: FxHashMap<NetId, NetTree>,
}

impl Best {
    /// Marks the grid's current routes as the best.
    pub(super) fn of(grid: &GlobalGrid) -> Self {
        Self {
            overflow: grid.overflow(),
            usage: grid.usage.snapshot(),
            shelved: FxHashMap::default(),
        }
    }

    /// Keeps the route a net had in the best round as it is ripped up, the
    /// first time since that round.
    fn shelve(&mut self, tree: NetTree) {
        let _ = self.shelved.entry(tree.net.net).or_insert(tree);
    }

    /// Puts the best round's routes and usage back.
    pub(super) fn restore(self, grid: &mut GlobalGrid, routes: &mut GlobalRoutes) {
        grid.usage.restore(&self.usage);
        for (net, tree) in self.shelved {
            // No place: the restored usage already holds the demand of every
            // route of the best round, these among them.
            let _ = routes.trees.insert(net, PlacedTree(tree));
        }
    }
}
