//! Global routing on a grid of gcells, in the manner of cu-gr (CUGR 1), not
//! of `OpenROAD`'s `grt/src/cugr` (CUGR 2).
//!
//! Each routable layer gets a capacity on every gcell edge in its preferred
//! direction: its tracks crossing that edge, each counted for the share of
//! the edge that fixed metal leaves usable. A route demands the tracks its
//! wires take on every edge they cross, more for a net with a wider
//! non-default rule, and on the edges beside each of its vias the share of
//! a track the via's pad is expected to take from wires passing it. An edge
//! costs its length, a logistic congestion term that rises as its usage
//! nears capacity, its accumulated overflow history, and a penalty per unit
//! of overflow that grows every round, as in `PathFinder`. Every net is
//! first routed along its Steiner tree by 3D pattern routing (`pattern`);
//! then nets crossing overflowed edges are ripped up and rerouted by maze
//! search (`maze`) in a search area grown around their pins and extended
//! by a corridor that a search over the whole chip on coarse cells picks
//! first (`coarse`), as in cu-gr, so a net can detour as far as the free
//! tracks call for. As in cu-gr, nets are planned in parallel batches whose
//! pin boxes do not meet, then rerouted in order, each seeing every earlier
//! net it could meet. The cells each net passes through become its guide
//! for detailed routing.

// NOTE: which parts follow which router. From cu-gr (CUGR 1): 3D pattern
// routing, and the coarse grid of `coarse` with its 1/free price. From
// OpenROAD's grt/src/cugr: the net order by span (CUGR::sortNetIndices,
// without slack) and the capacity of usable_fraction (GridGraph's
// constructor). PARE's own: edge_cost, via_loss, the PathFinder history and
// overflow penalty across rounds, and the Steiner trees of `steiner`;
// FastRoute's costs are not followed either.

mod batches;
mod coarse;
mod gcells;
mod grid;
mod maze;
mod negotiate;
mod pattern;
mod stamped;
mod steiner;
mod tracks;

use crate::tech::{NetKind, RouteTech, index_u32};
pub(crate) use batches::{GcellWindow, grid_index};
use batches::{
    Reroute, RoundTimes, disjoint_levels, route_disjoint, route_initial, span, touched_coarse_cells,
};
pub use gcells::Gcells;
pub use grid::{CapacityReduction, GlobalGrid, rudy_wire_width};
pub use negotiate::{reroute, route_global};
use pare_common::convert::SaturatingFrom;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use rustc_hash::{FxHashMap, FxHashSet};
pub use tracks::Tracks;

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

/// Search costs of global routing, in gcells.
#[derive(Clone, Copy, Debug)]
pub(crate) struct GlobalCosts {
    /// Cost of changing layer.
    pub via: f64,
    /// Cost of an edge used to capacity, approached smoothly from below.
    pub congestion: f64,
    /// Cost per unit of overflow an edge would reach.
    pub penalty: f64,
}

/// Steepness, per track, of the logistic congestion cost around capacity,
/// as grt cugr's `cost_logistic_slope` default.
const CONGESTION_SLOPE: f64 = 1.0;

/// Cost of an edge or via pad fixed metal blocks outright, in pattern
/// routing: far above any detour, so a plan takes one only when no plan
/// avoids it.
const BLOCKED_COST: f64 = 1e12;

/// Search margin, in gcells, around a net's first route's connections.
/// PARE tunable, no `OpenROAD` counterpart.
const FIRST_MARGIN: u16 = 2;

/// Gcells the search margin grows by each time a net is ripped up. PARE
/// tunable, no `OpenROAD` counterpart.
const MARGIN_STEP: u16 = 2;

/// Largest search margin, in gcells; wider margins leave too few nets with
/// disjoint search areas to reroute in parallel. PARE tunable, no
/// `OpenROAD` counterpart.
const MAX_MARGIN: u16 = 12;

/// Global routes of all nets, each placed on the grid that routed it.
#[derive(Debug, Default, PartialEq, Eq)]
pub struct GlobalRoutes {
    /// Each net's tree, its demand on the grid.
    pub trees: FxHashMap<NetId, PlacedTree>,
}

/// A net and the paths of its global route; each path starts on the paths
/// before it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NetTree {
    /// The net, with the terminals and track demand its paths were found
    /// for.
    pub net: GlobalNet,
    /// The paths.
    pub paths: Vec<Vec<GCell>>,
}

/// A net's tree whose demand the grid holds. Only
/// [`GlobalGrid::place`] makes one, and [`GlobalGrid::lift`] takes it off
/// the grid with the very net it was placed with, so a tree's demand is
/// lifted once, exactly as it was placed; `docs/global-routing.md` draws
/// its states.
#[derive(Debug, PartialEq, Eq)]
pub struct PlacedTree(NetTree);

impl PlacedTree {
    /// Its paths.
    #[must_use]
    pub fn paths(&self) -> &[Vec<GCell>] {
        &self.0.paths
    }

    /// Follows its net to the number `net` after a renumbering.
    pub(crate) const fn renumber(&mut self, net: NetId) {
        self.0.net.net = net;
    }
}

/// A net's global route as a tree over its gcells, for finding the cells
/// that join two of them.
#[derive(Clone, Debug, Default)]
pub(crate) struct RouteTree {
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

    /// The position of a cell in [`Self::cells`], if the route has it.
    pub fn position(&self, c: GCell) -> Option<usize> {
        self.index.get(&c).copied()
    }

    /// Every link of the tree as (cell, parent) positions in
    /// [`Self::cells`]; the root of each separate tree is left out.
    pub(crate) fn links(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.parent
            .iter()
            .enumerate()
            .filter(|&(c, &p)| c != p)
            .map(|(c, &p)| (c, p))
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

/// A net to route globally: its pins' gcells and what its wires take.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GlobalNet {
    /// The net to route.
    pub net: NetId,
    /// One gcell per pin.
    pub terminals: Vec<GCell>,
    /// Tracks one of its wires takes on each routable layer, lowest first.
    pub demand: Vec<Tracks>,
    /// Which layers its wires take.
    pub kind: NetKind,
}

impl GlobalNet {
    /// `net` of `db` with its pins at `terminals`.
    #[must_use]
    pub fn of(db: &NetlistDB, tech: &RouteTech, net: NetId, terminals: Vec<GCell>) -> Self {
        Self {
            net,
            terminals,
            demand: track_demand(db, tech, net),
            kind: tech.net_kind(db, net),
        }
    }

    /// Tracks a wire of the net takes on the layer at `offset` among the
    /// routable layers.
    fn demand(&self, offset: usize) -> Tracks {
        self.demand.get(offset).copied().unwrap_or(Tracks::ONE)
    }
}

/// Tracks a wire of `net` takes on each routable layer: one, or as many
/// pitches as its non-default rule's width and spacing span; one on a layer
/// of a single track, which has no pitch.
// NOTE: GlobalRouter::computeTrackConsumption takes ceil((w/2 + s + w_default/2)
// / pitch); PARE ceil((w + s) / pitch), room for a whole wire and its spacing.
pub fn track_demand(db: &NetlistDB, tech: &RouteTech, net: NetId) -> Vec<Tracks> {
    let rule = db.nets[net]
        .non_default_rule
        .map(|r| &db.non_default_rules[r]);
    tech.routable_layers()
        .map(|layer| {
            let lt = tech.layer(layer);
            let Some(ndr) = rule.and_then(|r| r.layers.iter().find(|x| x.layer == layer)) else {
                return Tracks::ONE;
            };
            let Some(pitch) = lt.pitch().map(Dbu::get) else {
                return Tracks::ONE;
            };
            let span = (ndr.width + ndr.spacing.unwrap_or(lt.spacing)).get();
            let tracks = (span + pitch - 1) / pitch;
            Tracks::whole(index_u32(usize::saturating_from(tracks.max(1))))
        })
        .collect()
}

/// Why saved global routes cannot be put back on a grid.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RestoreError {
    /// A net to route has no saved route.
    #[error("no saved global route for net {}", .0.index())]
    Missing(NetId),
    /// A saved route is for a net that is not routed.
    #[error("net {} has a saved global route but is not routed", .0.index())]
    Unrouted(NetId),
    /// A saved path leaves the grid or steps between gcells that are not
    /// neighbours along a layer or one layer apart.
    #[error("the saved global route of net {} has a step off the grid", .0.index())]
    OffGrid(NetId),
    /// A saved path after the first does not start on the paths before it.
    #[error("the saved global route of net {} is not one tree", .0.index())]
    Disconnected(NetId),
    /// A pin of the net lies off its saved route, as when the placement or
    /// pin access has changed since it was saved.
    #[error("the saved global route of net {} misses one of its pins", .0.index())]
    Unreached(NetId),
}

impl GlobalGrid {
    /// Whether a saved path is a walk on this grid: every gcell inside it
    /// and every step to the next gcell along the layer or to the same gcell
    /// one layer up or down.
    fn is_walk(&self, path: &[GCell]) -> bool {
        let inside =
            |c: &GCell| self.layers().contains(&c.layer.index()) && c.x < self.nx && c.y < self.ny;
        path.iter().all(inside)
            && path.windows(2).all(|w| {
                let (a, b) = (w[0], w[1]);
                if a.layer == b.layer {
                    let (dx, dy) = (a.x.abs_diff(b.x), a.y.abs_diff(b.y));
                    match self.direction(a.layer) {
                        Direction::Horizontal => dx == 1 && dy == 0,
                        Direction::Vertical => dx == 0 && dy == 1,
                    }
                } else {
                    a.x == b.x && a.y == b.y && a.layer.index().abs_diff(b.layer.index()) == 1
                }
            })
    }
}

/// Puts saved global routes back on `grid` for `nets`, which must be
/// exactly the nets saved, placing their demand as routing them would have.
///
/// # Errors
///
/// When a net lacks a saved route or has one it should not, or a saved
/// route is not a tree on the grid through every one of the net's pins.
pub(crate) fn restore_global(
    grid: &GlobalGrid,
    nets: &[GlobalNet],
    saved: Vec<(NetId, Vec<Vec<GCell>>)>,
) -> Result<GlobalRoutes, RestoreError> {
    let wanted: FxHashSet<NetId> = nets.iter().map(|n| n.net).collect();
    if let Some(&(net, _)) = saved.iter().find(|(n, _)| !wanted.contains(n)) {
        return Err(RestoreError::Unrouted(net));
    }
    let mut saved: FxHashMap<NetId, Vec<Vec<GCell>>> = saved.into_iter().collect();
    let mut routes = GlobalRoutes::default();
    for net in nets {
        let tree = saved
            .remove(&net.net)
            .ok_or(RestoreError::Missing(net.net))?;
        if !tree.iter().all(|p| grid.is_walk(p)) {
            return Err(RestoreError::OffGrid(net.net));
        }
        let mut cells: FxHashSet<GCell> = FxHashSet::default();
        for (k, path) in tree.iter().enumerate() {
            if k > 0 && !path.first().is_some_and(|c| cells.contains(c)) {
                return Err(RestoreError::Disconnected(net.net));
            }
            cells.extend(path.iter().copied());
        }
        if !net.terminals.iter().all(|t| cells.contains(t)) {
            return Err(RestoreError::Unreached(net.net));
        }
        let placed = grid.place(NetTree {
            net: net.clone(),
            paths: tree,
        });
        let _ = routes.trees.insert(net.net, placed);
    }
    Ok(routes)
}

#[cfg(test)]
mod tests {
    use super::batches::{initial_order, window};
    use super::grid::{Sign, Usage, adjusted_capacity};
    use super::maze::route_tree;
    use super::negotiate::FIRST_COSTS;
    use super::pattern::route_pattern;
    use super::*;
    use pare_common::config::CapacityReserve;
    use pare_common::geom::point::Point;

    #[test]
    fn an_adjusted_edge_keeps_at_least_one_of_its_tracks() {
        assert!((adjusted_capacity(1.0, 0.5) - 1.0).abs() < 1e-12);
        assert!((adjusted_capacity(7.0, 0.5) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn an_edge_held_back_entirely_keeps_none() {
        assert!(adjusted_capacity(7.0, 1.0).abs() < 1e-12);
    }

    #[test]
    fn no_adjustment_keeps_every_unblocked_track() {
        assert!((adjusted_capacity(6.5, 0.0) - 6.5).abs() < 1e-12);
    }

    fn cell(x: u16, y: u16) -> GCell {
        GCell {
            layer: LayerId::new(1),
            x,
            y,
        }
    }

    pub(super) fn at(layer: usize, x: u16, y: u16) -> GCell {
        GCell {
            layer: LayerId::new(layer),
            x,
            y,
        }
    }

    /// A 10x10 grid with a horizontal layer 1 over a vertical layer 2,
    /// every edge holding 4 tracks and every via taking a tenth of one.
    pub(super) fn grid() -> GlobalGrid {
        let (nx, ny) = (10, 10);
        let edges = 2 * usize::from(nx) * usize::from(ny);
        GlobalGrid {
            origin: Point::new(Dbu::new(0), Dbu::new(0)),
            gcell: Dbu::new(100),
            nx,
            ny,
            min_layer: LayerId::new(1),
            directions: vec![Direction::Horizontal, Direction::Vertical],
            clock_layers: LayerId::new(1)..=LayerId::new(2),
            reserve: CapacityReserve::NONE,
            capacity: vec![Tracks::whole(4); edges],
            blocked: vec![false; edges],
            via_blocked: vec![false; edges],
            via_loss: vec![Tracks::from_f64(0.1); 2],
            usage: Usage::zeroed(edges),
            history: vec![0.0; edges],
        }
    }

    pub(super) fn net(terminals: Vec<GCell>) -> GlobalNet {
        GlobalNet {
            net: NetId::new(0),
            terminals,
            demand: vec![Tracks::ONE; 2],
            kind: NetKind::Signal,
        }
    }

    pub(super) const COSTS: GlobalCosts = GlobalCosts {
        via: 1.0,
        congestion: 1.0,
        penalty: 1.0,
    };

    /// A 10x10 grid like [`grid`] with a second horizontal layer 3 above,
    /// whose clock trunks take layers 2 and 3.
    fn three_layer_grid() -> GlobalGrid {
        let edges = 3 * 10 * 10;
        GlobalGrid {
            directions: vec![
                Direction::Horizontal,
                Direction::Vertical,
                Direction::Horizontal,
            ],
            clock_layers: LayerId::new(2)..=LayerId::new(3),
            capacity: vec![Tracks::whole(4); edges],
            blocked: vec![false; edges],
            via_blocked: vec![false; edges],
            via_loss: vec![Tracks::from_f64(0.1); 3],
            usage: Usage::zeroed(edges),
            history: vec![0.0; edges],
            ..grid()
        }
    }

    fn trunk(terminals: Vec<GCell>) -> GlobalNet {
        GlobalNet {
            kind: NetKind::ClockTrunk,
            ..net(terminals)
        }
    }

    /// The layers a tree runs along, without its vias.
    fn wire_layers(paths: &[Vec<GCell>]) -> Vec<usize> {
        let mut layers: Vec<usize> = paths
            .iter()
            .flat_map(|p| p.windows(2))
            .filter(|w| w[0].layer == w[1].layer)
            .map(|w| w[0].layer.index())
            .collect();
        layers.sort_unstable();
        layers.dedup();
        layers
    }

    #[test]
    fn pattern_routing_keeps_a_clock_trunks_wires_on_the_clock_layers() {
        let grid = three_layer_grid();
        let terminals = vec![at(1, 2, 4), at(1, 7, 4)];

        let signal = route_pattern(&grid, &net(terminals.clone()), COSTS);
        let clock = route_pattern(&grid, &trunk(terminals), COSTS);

        assert_eq!(wire_layers(&signal), vec![1]);
        assert_eq!(wire_layers(&clock), vec![3]);
    }

    #[test]
    fn maze_routing_keeps_a_clock_trunks_wires_on_the_clock_layers() {
        let grid = three_layer_grid();
        let terminals = vec![at(1, 2, 4), at(1, 7, 6)];

        let signal = route_tree(&grid, &net(terminals.clone()), 2, COSTS, None);
        let clock = route_tree(&grid, &trunk(terminals), 2, COSTS, None);

        assert_eq!(wire_layers(&signal), vec![1, 2]);
        assert_eq!(wire_layers(&clock), vec![2, 3]);
    }

    #[test]
    fn clock_trunks_are_routed_first_then_the_shortest_nets() {
        let nets = vec![
            net(vec![cell(0, 0), cell(9, 9)]),
            net(vec![cell(0, 0), cell(1, 0)]),
            trunk(vec![cell(0, 0), cell(9, 0)]),
        ];

        assert_eq!(initial_order(&nets), vec![2, 1, 0]);
    }

    /// Net `id` through `terminals`, one track wide.
    fn numbered(id: usize, terminals: Vec<GCell>) -> GlobalNet {
        GlobalNet {
            net: NetId::new(id),
            ..net(terminals)
        }
    }

    /// Blocks the horizontal edges from column `x` to `x + 1` in rows `rows`.
    fn wall(grid: &mut GlobalGrid, x: u16, rows: std::ops::RangeInclusive<u16>) {
        for y in rows {
            let e = grid.edge(cell(x, y));
            grid.capacity[e] = Tracks::ZERO;
            grid.blocked[e] = true;
        }
    }

    /// Routes `order` one net after another, each seeing all before it.
    fn route_in_order(grid: &GlobalGrid, nets: &[GlobalNet], order: &[usize]) -> GlobalRoutes {
        let mut routes = GlobalRoutes::default();
        for &i in order {
            let paths = route_tree(grid, &nets[i], 1, COSTS, None);
            let placed = grid.place(NetTree {
                net: nets[i].clone(),
                paths,
            });
            let _ = routes.trees.insert(nets[i].net, placed);
        }
        routes
    }

    /// Routes `which` as one level with [`route_disjoint`].
    fn route_level(grid: &GlobalGrid, nets: &[GlobalNet], which: &[usize]) -> GlobalRoutes {
        let margins = vec![1; nets.len()];
        let plans = vec![None; which.len()];
        let reroute = Reroute {
            which,
            margins: &margins,
            plans: &plans,
        };
        let mut routes = GlobalRoutes::default();
        route_disjoint(grid, nets, &reroute, COSTS, &mut routes);
        routes
    }

    /// Nets crossing the grid in neighbouring rows, so that every one
    /// meets the area of the ones before it.
    fn crowded_nets() -> Vec<GlobalNet> {
        (0..6)
            .map(|k| {
                numbered(
                    k,
                    vec![cell(0, 3 + u16::try_from(k % 3).unwrap()), cell(9, 4)],
                )
            })
            .collect()
    }

    #[test]
    fn nets_rerouted_together_take_the_routes_of_rerouting_them_in_order() {
        let mut serial = grid();
        serial.capacity.fill(Tracks::whole(2));
        let mut parallel = grid();
        parallel.capacity.fill(Tracks::whole(2));
        let nets = crowded_nets();
        let order: Vec<usize> = (0..nets.len()).collect();

        let expected = route_in_order(&serial, &nets, &order);
        let routes = route_level(&parallel, &nets, &order);

        assert_eq!(routes.trees, expected.trees);
        assert!(serial.usage.iter().eq(parallel.usage.iter()));
    }

    /// Many short nets along a 40 by 40 grid, in chains of nets that meet
    /// and between chains that do not.
    fn chained_nets() -> (GlobalGrid, Vec<GlobalNet>) {
        let n = 40;
        let edges = 2 * usize::from(n) * usize::from(n);
        let grid = GlobalGrid {
            nx: n,
            ny: n,
            capacity: vec![Tracks::whole(2); edges],
            blocked: vec![false; edges],
            via_blocked: vec![false; edges],
            usage: Usage::zeroed(edges),
            history: vec![0.0; edges],
            ..grid()
        };
        let mut nets = Vec::new();
        for row in (0..n).step_by(10) {
            for k in 0..30 {
                let x = k % 10;
                let id = nets.len();
                nets.push(numbered(id, vec![cell(x, row + 1), cell(x + 20, row + 2)]));
            }
        }
        (grid, nets)
    }

    /// Restores `saved` as the routes of net 0 from (1, 2) to (4, 2) on
    /// layer 1 of a fresh grid, and whether that worked.
    fn restore_route_of_net_0(saved: Vec<(NetId, Vec<Vec<GCell>>)>) -> Result<(), RestoreError> {
        let nets = [net(vec![at(1, 1, 2), at(1, 4, 2)])];
        restore_global(&grid(), &nets, saved).map(|_| ())
    }

    fn row_2(from: u16, to: u16) -> Vec<GCell> {
        (from..=to).map(|x| at(1, x, 2)).collect()
    }

    #[test]
    fn a_saved_tree_through_every_pin_is_restored() {
        let saved = vec![(NetId::new(0), vec![row_2(1, 4)])];

        assert_eq!(restore_route_of_net_0(saved), Ok(()));
    }

    #[test]
    fn a_net_without_a_saved_route_is_refused() {
        assert_eq!(
            restore_route_of_net_0(Vec::new()),
            Err(RestoreError::Missing(NetId::new(0)))
        );
    }

    #[test]
    fn a_saved_route_of_a_net_not_routed_is_refused() {
        let saved = vec![
            (NetId::new(0), vec![row_2(1, 4)]),
            (NetId::new(7), vec![row_2(5, 6)]),
        ];

        assert_eq!(
            restore_route_of_net_0(saved),
            Err(RestoreError::Unrouted(NetId::new(7)))
        );
    }

    #[test]
    fn a_saved_step_against_the_layer_s_direction_is_refused() {
        let saved = vec![(
            NetId::new(0),
            vec![row_2(1, 4), vec![at(1, 2, 2), at(1, 2, 3)]],
        )];

        assert_eq!(
            restore_route_of_net_0(saved),
            Err(RestoreError::OffGrid(NetId::new(0)))
        );
    }

    #[test]
    fn a_saved_path_off_the_grid_is_refused() {
        let saved = vec![(NetId::new(0), vec![row_2(1, 4), row_2(9, 10)])];

        assert_eq!(
            restore_route_of_net_0(saved),
            Err(RestoreError::OffGrid(NetId::new(0)))
        );
    }

    #[test]
    fn a_saved_path_that_starts_off_the_tree_is_refused() {
        let saved = vec![(NetId::new(0), vec![row_2(1, 2), row_2(3, 4)])];

        assert_eq!(
            restore_route_of_net_0(saved),
            Err(RestoreError::Disconnected(NetId::new(0)))
        );
    }

    #[test]
    fn a_saved_route_that_misses_a_pin_is_refused() {
        let saved = vec![(NetId::new(0), vec![row_2(1, 3)])];

        assert_eq!(
            restore_route_of_net_0(saved),
            Err(RestoreError::Unreached(NetId::new(0)))
        );
    }

    #[test]
    fn nets_without_terminals_have_no_window_and_no_span() {
        assert_eq!(window(&[], 2, 10, 10), None);
        assert_eq!(span(&[]), 0);
    }

    #[test]
    fn a_window_grows_by_its_margin_within_the_grid() {
        let w = window(&[cell(1, 8), cell(4, 2)], 2, 10, 10);

        let expected = GcellWindow {
            x0: 0,
            x1: 6,
            y0: 0,
            y1: 9,
        };
        assert_eq!(w, Some(expected));
    }

    #[test]
    fn initial_routes_do_not_depend_on_the_thread_count() {
        let route_with = |threads: usize| {
            let pool = rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap();
            pool.install(|| {
                let (grid, nets) = chained_nets();
                let mut routes = GlobalRoutes::default();
                route_initial(&grid, &nets, FIRST_COSTS, &mut routes);
                (routes.trees, grid.usage.snapshot())
            })
        };

        let one = route_with(1);
        let four = route_with(4);

        assert_eq!(one, four);
    }

    #[test]
    fn every_net_rerouted_together_is_routed_once_as_in_order() {
        let (serial, nets) = chained_nets();
        let order: Vec<usize> = (0..nets.len()).collect();
        let expected = route_in_order(&serial, &nets, &order);

        for _ in 0..20 {
            let (parallel, _) = chained_nets();
            let routes = route_level(&parallel, &nets, &order);

            assert_eq!(routes.trees, expected.trees);
            assert!(serial.usage.iter().eq(parallel.usage.iter()));
        }
    }

    /// A grid walled across at column 4 but for rows 0 and 9, the gap in
    /// row 0 holding one track.
    fn two_gaps() -> GlobalGrid {
        let mut grid = grid();
        wall(&mut grid, 4, 1..=8);
        let gap = grid.edge(cell(4, 0));
        grid.capacity[gap] = Tracks::ONE;
        grid
    }

    #[test]
    fn a_net_whose_area_holds_no_route_is_rerouted_after_the_others() {
        let serial = two_gaps();
        let parallel = two_gaps();
        let wide = GlobalNet {
            demand: vec![Tracks::whole(2); 2],
            ..numbered(1, vec![cell(1, 0), cell(8, 0)])
        };
        let nets = vec![numbered(0, vec![cell(2, 3), cell(7, 5)]), wide];

        let expected = route_in_order(&serial, &nets, &[1, 0]);
        let routes = route_level(&parallel, &nets, &[0, 1]);

        assert_eq!(routes.trees, expected.trees);
        assert!(serial.usage.iter().eq(parallel.usage.iter()));
    }

    #[test]
    fn nets_whose_pin_boxes_are_apart_are_rerouted_together() {
        let nets = [
            net(vec![cell(0, 0), cell(1, 1)]),
            net(vec![cell(8, 8), cell(9, 9)]),
        ];

        let levels = disjoint_levels(&grid(), &nets, &[0, 1]);

        assert_eq!(levels, vec![vec![0, 1]]);
    }

    #[test]
    fn nets_whose_pin_boxes_share_a_block_are_rerouted_one_after_the_other() {
        let nets = [
            net(vec![cell(0, 0), cell(2, 2)]),
            net(vec![cell(1, 1), cell(3, 3)]),
        ];

        let levels = disjoint_levels(&grid(), &nets, &[0, 1]);

        assert_eq!(levels, vec![vec![0], vec![1]]);
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

    #[test]
    fn a_wire_takes_one_track_on_each_edge_it_crosses() {
        let grid = grid();
        let net = net(vec![at(1, 2, 5), at(1, 5, 5)]);
        let paths = vec![vec![at(1, 2, 5), at(1, 3, 5), at(1, 4, 5), at(1, 5, 5)]];

        grid.add_tree(&net, &paths, Sign::Add);

        for x in 2..5 {
            assert_eq!(grid.usage.get(grid.edge(at(1, x, 5))), Tracks::ONE);
        }
        assert_eq!(grid.usage.get(grid.edge(at(1, 1, 5))), Tracks::ZERO);
        assert_eq!(grid.usage.get(grid.edge(at(1, 5, 5))), Tracks::ZERO);
    }

    #[test]
    fn a_via_costs_the_edges_beside_it_that_its_own_wire_does_not_cross() {
        let grid = grid();
        let net = net(vec![at(1, 2, 5), at(2, 5, 7)]);
        let paths = vec![vec![
            at(1, 2, 5),
            at(1, 3, 5),
            at(1, 4, 5),
            at(1, 5, 5),
            at(2, 5, 5),
            at(2, 5, 6),
            at(2, 5, 7),
        ]];
        let loss = Tracks::from_f64(0.1);

        grid.add_tree(&net, &paths, Sign::Add);

        assert_eq!(grid.usage.get(grid.edge(at(1, 4, 5))), Tracks::ONE);
        assert_eq!(grid.usage.get(grid.edge(at(1, 5, 5))), loss);
        assert_eq!(grid.usage.get(grid.edge(at(2, 5, 5))), Tracks::ONE);
        assert_eq!(grid.usage.get(grid.edge(at(2, 5, 4))), loss);
    }

    #[test]
    fn a_stack_through_a_layer_costs_both_edges_beside_it() {
        let mut grid = grid();
        grid.directions.push(Direction::Horizontal);
        let edges = grid.usage.len() / 2;
        grid.capacity.extend(vec![Tracks::whole(4); edges]);
        grid.blocked.extend(vec![false; edges]);
        grid.via_blocked.extend(vec![false; edges]);
        grid.via_loss.push(Tracks::from_f64(0.1));
        grid.usage.extend_zeroed(edges);
        grid.history.extend(vec![0.0; edges]);
        let net = net(vec![at(1, 5, 5), at(3, 7, 5)]);
        let paths = vec![vec![
            at(1, 5, 5),
            at(2, 5, 5),
            at(3, 5, 5),
            at(3, 6, 5),
            at(3, 7, 5),
        ]];
        let loss = Tracks::from_f64(0.1);

        grid.add_tree(&net, &paths, Sign::Add);

        assert_eq!(grid.usage.get(grid.edge(at(2, 5, 4))), loss);
        assert_eq!(grid.usage.get(grid.edge(at(2, 5, 5))), loss);
        assert_eq!(grid.usage.get(grid.edge(at(3, 4, 5))), loss);
        assert_eq!(grid.usage.get(grid.edge(at(3, 5, 5))), Tracks::ONE);
    }

    #[test]
    fn a_terminal_pad_is_not_charged_again() {
        let grid = grid();
        let net = net(vec![at(1, 5, 5), at(2, 5, 7)]);
        let paths = vec![vec![at(1, 5, 5), at(2, 5, 5), at(2, 5, 6), at(2, 5, 7)]];

        grid.add_tree(&net, &paths, Sign::Add);

        assert_eq!(grid.usage.get(grid.edge(at(1, 4, 5))), Tracks::ZERO);
        assert_eq!(grid.usage.get(grid.edge(at(1, 5, 5))), Tracks::ZERO);
    }

    #[test]
    fn removing_a_tree_restores_the_usage_exactly() {
        let grid = grid();
        let net = net(vec![at(1, 2, 5), at(2, 5, 7)]);
        let paths = vec![vec![
            at(1, 2, 5),
            at(1, 3, 5),
            at(1, 4, 5),
            at(1, 5, 5),
            at(2, 5, 5),
            at(2, 5, 6),
            at(2, 5, 7),
        ]];

        grid.add_tree(&net, &paths, Sign::Add);
        grid.add_tree(&net, &paths, Sign::Remove);

        assert!(grid.usage.iter().all(|u| u == Tracks::ZERO));
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "more usage lifted")]
    fn removing_demand_that_was_never_added_is_caught() {
        let grid = grid();
        let net = net(vec![at(1, 2, 5), at(1, 4, 5)]);
        let paths = vec![vec![at(1, 2, 5), at(1, 3, 5), at(1, 4, 5)]];

        grid.add_tree(&net, &paths, Sign::Remove);
    }

    #[test]
    fn a_wider_rule_demands_more_tracks_per_edge() {
        let grid = grid();
        let net = GlobalNet {
            demand: vec![Tracks::whole(2); 2],
            ..net(vec![at(1, 2, 5), at(1, 4, 5)])
        };
        let paths = vec![vec![at(1, 2, 5), at(1, 3, 5), at(1, 4, 5)]];

        grid.add_tree(&net, &paths, Sign::Add);

        assert_eq!(grid.usage.get(grid.edge(at(1, 2, 5))), Tracks::whole(2));
        assert_eq!(grid.usage.get(grid.edge(at(1, 3, 5))), Tracks::whole(2));
        assert_eq!(grid.overflow(), Tracks::ZERO);
    }
}
