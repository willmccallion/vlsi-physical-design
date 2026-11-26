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
mod tests;
