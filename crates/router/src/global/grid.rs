//! The gcell grid: edge capacities from the tracks and what blocks them, usage and history.

use super::gcells::Gcells;
use super::tracks::Tracks;
use super::{CONGESTION_SLOPE, GCell, GlobalCosts, GlobalNet, NetTree, PlacedTree, grid_index};
use crate::occupancy::{FixedMetal, TrackRef};
use crate::tech::{NetKind, RouteTech};
use pare_common::config::{CapacityReserve, Share};
use pare_common::db::indices::{IdVec, LayerId};
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::Interval;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use std::ops::RangeInclusive;
use std::sync::atomic::{AtomicI64, Ordering};

/// Share of a gcell's tracks on a layer that fixed metal must take for the
/// layer's congestion there to be left out of [`GlobalGrid::congestion`],
/// since no placement changes what crosses it; `OpenROAD`'s
/// `ignoreEdgeRatio`.
const IGNORED_BLOCKAGE: f64 = 0.8;

/// The gcell grid with edge capacities, usage and history.
#[derive(Debug)]
pub struct GlobalGrid {
    pub(super) origin: Point<Dbu>,
    pub(super) gcell: Dbu,
    pub(super) nx: u16,
    pub(super) ny: u16,
    pub(super) min_layer: LayerId,
    pub(super) directions: Vec<Direction>,
    /// The layers clock trunks' wires may use.
    pub(super) clock_layers: RangeInclusive<LayerId>,
    /// Share of each edge's usable tracks held back from `capacity`.
    pub(super) reserve: CapacityReserve,
    pub(super) capacity: Vec<Tracks>,
    /// Edges no track crosses at all, which fixed metal blocks outright.
    pub(super) blocked: Vec<bool>,
    /// Cells where fixed metal leaves no track of the layer room for a via pad.
    pub(super) via_blocked: Vec<bool>,
    /// Per layer, the share of a track a via somewhere in a gcell is expected
    /// to take from each edge beside it.
    pub(super) via_loss: Vec<Tracks>,
    pub(super) usage: Usage,
    pub(super) history: Vec<f32>,
}

/// Each edge's usage, which nets routed at the same time in disjoint areas
/// add to and take from at once; whole adds and removes keep it exact.
#[derive(Debug, Default)]
pub(crate) struct Usage(Vec<AtomicI64>);

impl Usage {
    pub(crate) fn zeroed(edges: usize) -> Self {
        Self((0..edges).map(|_| AtomicI64::new(0)).collect())
    }

    pub(crate) fn get(&self, e: usize) -> Tracks {
        Tracks::from_raw(self.0[e].load(Ordering::Relaxed))
    }

    pub(super) fn add(&self, e: usize, t: Tracks) {
        let _ = self.0[e].fetch_add(t.raw(), Ordering::Relaxed);
    }

    /// Takes `t` from edge `e`, which an add of at least `t` put there.
    pub(super) fn remove(&self, e: usize, t: Tracks) {
        let before = self.0[e].fetch_sub(t.raw(), Ordering::Relaxed);
        debug_assert!(
            before >= t.raw(),
            "more usage lifted from edge {e} than was placed: {before} < {}",
            t.raw()
        );
    }

    #[cfg(test)]
    pub(crate) fn set(&mut self, e: usize, t: Tracks) {
        *self.0[e].get_mut() = t.raw();
    }

    fn clear(&mut self) {
        for u in &mut self.0 {
            *u.get_mut() = 0;
        }
    }

    pub(crate) const fn len(&self) -> usize {
        self.0.len()
    }

    pub(super) fn iter(&self) -> impl Iterator<Item = Tracks> + '_ {
        (0..self.len()).map(|e| self.get(e))
    }

    pub(super) fn snapshot(&self) -> Vec<Tracks> {
        self.iter().collect()
    }

    pub(super) fn restore(&mut self, from: &[Tracks]) {
        for (u, t) in self.0.iter_mut().zip(from) {
            *u.get_mut() = t.raw();
        }
    }

    #[cfg(test)]
    pub(super) fn extend_zeroed(&mut self, edges: usize) {
        self.0.extend((0..edges).map(|_| AtomicI64::new(0)));
    }
}

/// One layer's routing resources and their use, summed over its edges.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct LayerTotals {
    /// The routing layer the totals are for.
    pub layer: LayerId,
    /// Its preferred direction, which its edges run along.
    pub direction: Direction,
    /// Tracks the edges offer, after the reserve and fixed metal.
    pub capacity: Tracks,
    /// Tracks the routes take.
    pub usage: Tracks,
    /// Usage above capacity.
    pub overflow: Tracks,
}

/// Whether a route's demand is being placed on the grid or lifted from it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Sign {
    Add,
    Remove,
}

/// What one layer's tracks and fixed metal give the grid.
struct LayerResources {
    /// Usable tracks across each edge.
    pub(super) tracks: Vec<f64>,
    /// Cells with no room for a via pad on any track.
    pub(super) via_blocked: Vec<bool>,
}

impl GlobalGrid {
    /// A grid of `gcell`-sized cells over `die` whose edges offer the free
    /// tracks crossing them less the `reserve` fraction.
    pub fn new(
        tech: &RouteTech,
        fixed: &(impl FixedMetal + Sync),
        die: Rect<Dbu>,
        gcell: Dbu,
        reserve: CapacityReserve,
    ) -> Self {
        // NOTE: GlobalRouter::initGrid takes floor(die / gcell) gcells and
        // stretches the last; PARE covers the die by ceil, every gcell equal.
        let span = |extent: Dbu| {
            let cells = extent.div_ceil(gcell);
            grid_index(cells as usize).max(1)
        };
        let (nx, ny) = (span(die.width()), span(die.height()));
        let layers: Vec<LayerId> = tech.routable_layers().collect();
        let per_layer = usize::from(nx) * usize::from(ny);
        let mut grid = Self {
            origin: die.min,
            gcell,
            nx,
            ny,
            min_layer: tech.min_layer,
            directions: layers.iter().map(|&l| tech.layer(l).direction).collect(),
            clock_layers: tech.clock_layers.clone(),
            reserve,
            capacity: Vec::new(),
            blocked: Vec::new(),
            via_blocked: Vec::new(),
            via_loss: layers.iter().map(|&l| via_loss(tech, l, gcell)).collect(),
            usage: Usage::zeroed(per_layer * layers.len()),
            history: vec![0.0; per_layer * layers.len()],
        };
        let resources: Vec<LayerResources> = layers
            .par_iter()
            .map(|&l| grid.layer_resources(tech, fixed, die, l))
            .collect();
        grid.capacity = resources
            .iter()
            .flat_map(|r| r.tracks.iter())
            .map(|&t| Tracks::from_f64(t * reserve.left()))
            .collect();
        grid.blocked = resources
            .iter()
            .flat_map(|r| r.tracks.iter())
            .map(|&t| t <= 0.0)
            .collect();
        grid.via_blocked = resources
            .iter()
            .flat_map(|r| r.via_blocked.iter().copied())
            .collect();
        grid
    }

    /// Usable tracks across each preferred-direction edge of one layer, as
    /// in CUGR: each track counts for the fraction of the stretch between
    /// the centres of the edge's two gcells that fixed metal leaves usable,
    /// and nothing when fixed metal crosses the boundary between them. Also
    /// which of the layer's cells leave no track a gap long enough for a
    /// via pad.
    fn layer_resources(
        &self,
        tech: &RouteTech,
        fixed: &impl FixedMetal,
        die: Rect<Dbu>,
        layer: LayerId,
    ) -> LayerResources {
        let lt = tech.layer(layer);
        let (nx, ny) = (usize::from(self.nx), usize::from(self.ny));
        let mut cap = vec![0.0; nx * ny];
        let mut room = vec![false; nx * ny];
        let pad = via_pad_along(tech, layer);
        let (along_origin, along_end, across_origin, along_cells) = match lt.direction {
            Direction::Horizontal => (self.origin.x, die.max.x, self.origin.y, nx),
            Direction::Vertical => (self.origin.y, die.max.y, self.origin.x, ny),
        };
        let extent = |i: usize| {
            let start = along_origin + self.gcell * i as i64;
            let end = (start + self.gcell).min(along_end).max(start);
            Interval::new(start, end)
        };
        let centre = |i: usize| {
            let Interval { lo, hi } = extent(i);
            lo + (hi - lo) / 2
        };
        for (t, &across) in lt.tracks.iter_enumerated() {
            let row = (across - across_origin).div_floor(self.gcell);
            let Ok(row) = usize::try_from(row) else {
                continue;
            };
            let track = TrackRef { layer, track: t };
            for i in 0..along_cells {
                let index = match lt.direction {
                    Direction::Horizontal if row < ny => row * nx + i,
                    Direction::Vertical if row < nx => i * nx + row,
                    _ => continue,
                };
                if has_gap(fixed, track, extent(i), pad) {
                    room[index] = true;
                }
                if i + 1 < along_cells {
                    let boundary = along_origin + self.gcell * (i as i64 + 1);
                    let stretch = Interval::new(centre(i), centre(i + 1));
                    cap[index] += usable_fraction(fixed, track, boundary, stretch);
                }
            }
        }
        LayerResources {
            tracks: cap,
            via_blocked: room.into_iter().map(|r| !r).collect(),
        }
    }

    /// Forgets every route's demand and the negotiation history, keeping
    /// the capacities.
    pub fn clear(&mut self) {
        self.usage.clear();
        self.history.fill(0.0);
    }

    /// Each gcell's congestion, row by row: the largest share of its
    /// capacity the routes take on any routable layer, over both edges of
    /// the gcell along that layer, as `OpenROAD`'s placer reads its global
    /// router (`getUsageCapacityRatio` in gpl's `RouteBase::updateRoute`). A
    /// layer does not count in a gcell where fixed metal takes
    /// `IGNORED_BLOCKAGE` of the tracks or more, or every one of them.
    // NOTE: gpl reads (usage + blockage) / (capacity + blockage) and lets the
    // last layer past minInflationRatio set the tile; PARE takes routed usage
    // over what is left, the largest over layers.
    pub fn congestion(&self, tech: &RouteTech) -> Vec<f64> {
        let (nx, ny) = (usize::from(self.nx), usize::from(self.ny));
        let mut congestion = vec![0.0; nx * ny];
        for l in self.layers() {
            let layer = LayerId::new(l);
            let direction = self.direction(layer);
            let tracks = self.tracks_per_line(tech, layer);
            for y in 0..ny {
                for x in 0..nx {
                    let (line, back, forward) = match direction {
                        Direction::Horizontal => (
                            y,
                            x.checked_sub(1).map(|px| self.cell_index(layer, px, y)),
                            (x + 1 < nx).then(|| self.cell_index(layer, x, y)),
                        ),
                        Direction::Vertical => (
                            x,
                            y.checked_sub(1).map(|py| self.cell_index(layer, x, py)),
                            (y + 1 < ny).then(|| self.cell_index(layer, x, y)),
                        ),
                    };
                    let ratio = [back, forward]
                        .into_iter()
                        .flatten()
                        .filter_map(|e| self.edge_ratio(e, tracks[line]))
                        .fold(0.0, f64::max);
                    let at = &mut congestion[y * nx + x];
                    *at = f64::max(*at, ratio);
                }
            }
        }
        congestion
    }

    /// Usage over capacity of edge `e`, or none when fixed metal leaves it
    /// fewer than `1 - IGNORED_BLOCKAGE` of the `tracks` it would have.
    fn edge_ratio(&self, e: usize, tracks: f64) -> Option<f64> {
        let capacity = self.capacity[e].to_f64();
        let available = tracks * self.reserve.left();
        if capacity <= 0.0 || available <= 0.0 {
            return None;
        }
        if (available - capacity) / available >= IGNORED_BLOCKAGE {
            return None;
        }
        Some(self.usage.get(e).to_f64() / capacity)
    }

    /// Every gcell's [`CapacityReduction`] in the terms `FastRoute` reports
    /// it to `OpenROAD`'s RUDY: each routing layer adds the tracks of the
    /// edge leaving the gcell along its direction to the gcell's capacity,
    /// and to its reduction the tracks fixed metal takes of that edge plus
    /// the `adjustment` share of what is left, rounded as `OpenROAD`'s
    /// `computeUserLayerAdjustments` rounds it. The last gcell of a row or
    /// column takes the edge before it; `FastRoute` takes its capacity so
    /// but reads its reduction from an edge that does not exist, which is
    /// none.
    pub fn capacity_reduction(
        &self,
        tech: &RouteTech,
        adjustments: &IdVec<LayerId, Share>,
        wire_width: Dbu,
    ) -> CapacityReduction {
        let (nx, ny) = (usize::from(self.nx), usize::from(self.ny));
        let mut capacity = vec![0.0; nx * ny];
        let mut reduction = vec![0.0; nx * ny];
        for l in self.layers() {
            let layer = LayerId::new(l);
            let direction = self.direction(layer);
            let along = match direction {
                Direction::Horizontal => nx,
                Direction::Vertical => ny,
            };
            if along < 2 {
                continue;
            }
            let tracks = self.tracks_per_line(tech, layer);
            let adjustment = adjustments.get(layer).copied().map_or(0.0, Share::get);
            for y in 0..ny {
                for x in 0..nx {
                    let (line, edge) = match direction {
                        Direction::Horizontal => (y, self.cell_index(layer, x.min(along - 2), y)),
                        Direction::Vertical => (x, self.cell_index(layer, x, y.min(along - 2))),
                    };
                    let unblocked = self.capacity[edge].to_f64() / self.reserve.left();
                    let left = adjusted_capacity(unblocked, adjustment);
                    capacity[y * nx + x] += tracks[line];
                    reduction[y * nx + x] += (tracks[line] - left).max(0.0);
                }
            }
        }
        CapacityReduction {
            origin: self.origin,
            gcell: self.gcell,
            nx,
            ny,
            capacity,
            reduction,
            wire_width,
        }
    }

    /// How many of `layer`'s tracks run through each row of gcells when it
    /// is horizontal, or each column when it is vertical.
    fn tracks_per_line(&self, tech: &RouteTech, layer: LayerId) -> Vec<f64> {
        let lt = tech.layer(layer);
        let (origin, lines) = match lt.direction {
            Direction::Horizontal => (self.origin.y, usize::from(self.ny)),
            Direction::Vertical => (self.origin.x, usize::from(self.nx)),
        };
        let mut count = vec![0.0; lines];
        for &across in &lt.tracks {
            let line = (across - origin).div_floor(self.gcell);
            if let Ok(line) = usize::try_from(line)
                && line < lines
            {
                count[line] += 1.0;
            }
        }
        count
    }

    /// Index of the edge leaving gcell (`x`, `y`) of `layer`.
    fn cell_index(&self, layer: LayerId, x: usize, y: usize) -> usize {
        self.edge(GCell {
            layer,
            x: grid_index(x),
            y: grid_index(y),
        })
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
            let c = (v - o).div_floor(self.gcell).clamp(0, i64::from(n) - 1);
            grid_index(c as usize)
        };
        GCell {
            layer,
            x: coord(p.x, self.origin.x, self.nx),
            y: coord(p.y, self.origin.y, self.ny),
        }
    }

    /// The center of a gcell, as `OpenROAD`'s global router places route
    /// points (`grt::Grid::getPositionOnGrid`): half a gcell into it,
    /// whatever part of it the die holds.
    pub(crate) fn cell_center(&self, c: GCell) -> Point<Dbu> {
        let half = Dbu::new(self.gcell.get() / 2);
        self.origin
            + Point::new(
                Gcells::from(c.x).length(self.gcell) + half,
                Gcells::from(c.y).length(self.gcell) + half,
            )
    }

    /// The area of a gcell.
    pub(crate) fn cell_rect(&self, x: u16, y: u16) -> Rect<Dbu> {
        let min = self.origin
            + Point::new(
                Gcells::from(x).length(self.gcell),
                Gcells::from(y).length(self.gcell),
            );
        Rect::new(min, min + Point::new(self.gcell, self.gcell))
    }

    /// Index of the edge leaving `c` forward in its layer's direction, and
    /// of `c` itself among the layer's cells.
    pub(super) fn edge(&self, c: GCell) -> usize {
        let l = c.layer.index() - self.min_layer.index();
        let per_layer = usize::from(self.nx) * usize::from(self.ny);
        l * per_layer + usize::from(c.y) * usize::from(self.nx) + usize::from(c.x)
    }

    /// The gcell an index from [`Self::edge`] names.
    pub(super) fn cell_of(&self, index: usize) -> GCell {
        let per_layer = usize::from(self.nx) * usize::from(self.ny);
        let rest = index % per_layer;
        GCell {
            layer: LayerId::new(self.min_layer.index() + index / per_layer),
            x: grid_index(rest % usize::from(self.nx)),
            y: grid_index(rest / usize::from(self.nx)),
        }
    }

    pub(super) fn direction(&self, layer: LayerId) -> Direction {
        self.directions[layer.index() - self.min_layer.index()]
    }

    /// The position of `layer` among the routable layers.
    pub(super) const fn layer_offset(&self, layer: LayerId) -> usize {
        layer.index() - self.min_layer.index()
    }

    pub(super) const fn layers(&self) -> RangeInclusive<usize> {
        self.min_layer.index()..=self.min_layer.index() + self.directions.len() - 1
    }

    /// Whether wires of a net of `kind` may run on `layer`; vias may climb
    /// through any layer, to reach pins below a clock trunk's layers.
    pub(super) fn allows_wires(&self, kind: NetKind, layer: LayerId) -> bool {
        match kind {
            NetKind::Signal => true,
            NetKind::ClockTrunk => self.clock_layers.contains(&layer),
        }
    }

    /// Whether no via pad fits in cell `c` on its layer.
    pub(super) fn via_blocked(&self, c: GCell) -> bool {
        self.via_blocked[self.edge(c)]
    }

    /// Whether a wire of `c`'s layer can leave `c` forward, and the edge it
    /// takes.
    pub(super) fn forward_edge(&self, c: GCell) -> Option<usize> {
        let last = match self.direction(c.layer) {
            Direction::Horizontal => c.x + 1 >= self.nx,
            Direction::Vertical => c.y + 1 >= self.ny,
        };
        (!last).then(|| self.edge(c))
    }

    /// The edge a wire of `c`'s layer enters `c` by from behind, if any.
    pub(super) fn backward_edge(&self, c: GCell) -> Option<usize> {
        let behind = match self.direction(c.layer) {
            Direction::Horizontal => GCell {
                x: c.x.checked_sub(1)?,
                ..c
            },
            Direction::Vertical => GCell {
                y: c.y.checked_sub(1)?,
                ..c
            },
        };
        Some(self.edge(behind))
    }

    /// Edges a path uses: each step within a layer uses the edge leaving its
    /// lower-coordinate end.
    pub(super) fn path_edges<'a>(&'a self, path: &'a [GCell]) -> impl Iterator<Item = usize> + 'a {
        path.windows(2).filter_map(move |w| {
            let (a, b) = (w[0], w[1]);
            let moved = a.x != b.x || a.y != b.y;
            (a.layer == b.layer && moved).then(|| {
                let lower = if (a.x, a.y) < (b.x, b.y) { a } else { b };
                self.edge(lower)
            })
        })
    }

    /// Places or lifts the demand of a net's paths: its track demand on
    /// every edge they cross, counting an edge the paths share once, and
    /// the via loss of its layer on each edge beside a via that no wire of
    /// the net on that layer crosses. The pad a terminal already holds as
    /// fixed metal is not counted again.
    pub(super) fn add_tree(&self, net: &GlobalNet, paths: &[Vec<GCell>], sign: Sign) {
        let mut edges: Vec<usize> = paths.iter().flat_map(|p| self.path_edges(p)).collect();
        edges.sort_unstable();
        edges.dedup();
        let mut demands: Vec<(usize, Tracks)> = edges
            .iter()
            .map(|&e| (e, net.demand(self.layer_offset(self.cell_of(e).layer))))
            .collect();
        for stack in via_stacks(paths) {
            for c in stack {
                if net.terminals.contains(&c) {
                    continue;
                }
                let loss = self.via_loss[self.layer_offset(c.layer)];
                for e in [self.backward_edge(c), self.forward_edge(c)]
                    .into_iter()
                    .flatten()
                {
                    if edges.binary_search(&e).is_err() {
                        demands.push((e, loss));
                    }
                }
            }
        }
        for (e, t) in demands {
            match sign {
                Sign::Add => self.usage.add(e, t),
                Sign::Remove => self.usage.remove(e, t),
            }
        }
    }

    /// Places the demand of `tree` on the grid.
    pub(crate) fn place(&self, tree: NetTree) -> PlacedTree {
        self.add_tree(&tree.net, &tree.paths, Sign::Add);
        PlacedTree(tree)
    }

    /// Lifts the demand of `placed` off the grid, as [`Self::place`] put
    /// it there.
    pub(crate) fn lift(&self, placed: PlacedTree) -> NetTree {
        let PlacedTree(tree) = placed;
        self.add_tree(&tree.net, &tree.paths, Sign::Remove);
        tree
    }

    /// Each edge's usage, for tests to compare whole grids.
    #[cfg(test)]
    pub(crate) fn usage_by_edge(&self) -> Vec<Tracks> {
        (0..self.capacity.len())
            .map(|e| self.usage.get(e))
            .collect()
    }

    pub(super) fn overflowed(&self, e: usize) -> bool {
        self.usage.get(e) > self.capacity[e]
    }

    /// Whether `tree` crosses an overflowed edge.
    pub(super) fn crosses_overflow(&self, tree: &PlacedTree) -> bool {
        tree.paths()
            .iter()
            .any(|p| self.path_edges(p).any(|e| self.overflowed(e)))
    }

    /// Total usage above capacity over all edges.
    pub fn overflow(&self) -> Tracks {
        self.usage
            .iter()
            .zip(&self.capacity)
            .map(|(u, &c)| u.saturating_sub(c))
            .sum()
    }

    /// Capacity, usage and overflow summed over each layer's edges, lowest
    /// routing layer first.
    pub(crate) fn layer_totals(&self) -> Vec<LayerTotals> {
        let per_layer = usize::from(self.nx) * usize::from(self.ny);
        self.directions
            .iter()
            .enumerate()
            .map(|(l, &direction)| {
                let edges = l * per_layer..(l + 1) * per_layer;
                let mut totals = LayerTotals {
                    layer: LayerId::new(self.min_layer.index() + l),
                    direction,
                    capacity: Tracks::ZERO,
                    usage: Tracks::ZERO,
                    overflow: Tracks::ZERO,
                };
                for e in edges {
                    totals.capacity += self.capacity[e];
                    totals.usage += self.usage.get(e);
                    totals.overflow += self.usage.get(e).saturating_sub(self.capacity[e]);
                }
                totals
            })
            .collect()
    }

    /// Cost of `demand` more tracks on edge `e`.
    // NOTE: grt cugr's GridGraph::getWireCost weighs demand by edge length and
    // a logistic of free capacity; PARE prices each edge alike, with history.
    pub(super) fn edge_cost(&self, e: usize, demand: Tracks, costs: GlobalCosts) -> f64 {
        let excess = (self.usage.get(e) + demand - self.capacity[e]).to_f64();
        let congestion = costs.congestion / (1.0 + (-CONGESTION_SLOPE * excess).exp());
        let overflow = costs.penalty * excess.max(0.0);
        1.0 + f64::from(self.history[e]) + congestion + overflow
    }
}

/// What `OpenROAD`'s RUDY reads from its global router.
///
/// Each gcell's routing capacity and the part of it taken before placement,
/// in tracks summed over the routing layers, with the width RUDY gives a
/// wire.
#[derive(Clone, Debug, PartialEq)]
pub struct CapacityReduction {
    /// Lower-left corner of the gcell grid.
    pub origin: Point<Dbu>,
    /// Edge of one gcell.
    pub gcell: Dbu,
    /// Columns.
    pub nx: usize,
    /// Rows.
    pub ny: usize,
    /// Tracks through each gcell, row by row.
    pub capacity: Vec<f64>,
    /// Tracks of `capacity` that fixed metal and the layer adjustment take.
    pub reduction: Vec<f64>,
    /// Width of a net's wire in RUDY; see [`rudy_wire_width`].
    pub wire_width: Dbu,
}

/// The tracks an edge with `unblocked` tracks free of fixed metal keeps
/// once `adjustment` of them is held back, as `OpenROAD`'s
/// `computeUserLayerAdjustments`: rounded down, but at least one where
/// any was and not all are held back.
pub(super) fn adjusted_capacity(unblocked: f64, adjustment: f64) -> f64 {
    if adjustment == 0.0 {
        return unblocked;
    }
    let kept = (unblocked * (1.0 - adjustment)).floor();
    if unblocked > 0.0 && adjustment < 1.0 {
        kept.max(1.0)
    } else {
        kept
    }
}

/// The width `OpenROAD`'s RUDY gives a net's wire over routing layers of
/// `pitches`: their harmonic mean pitch over their number, times 1.25,
/// truncated to a whole unit as `grt::Rudy` stores it.
pub fn rudy_wire_width(pitches: &[Dbu]) -> Dbu {
    let inverse: f64 = pitches
        .iter()
        .filter(|p| p.get() > 0)
        .map(|p| 1.0 / p.to_f64())
        .sum();
    if inverse > 0.0 {
        Dbu::new((1.25 / inverse).trunc() as i64)
    } else {
        RUDY_DEFAULT_WIRE_WIDTH
    }
}

/// The wire width `grt::Rudy` keeps when no layer has a pitch.
const RUDY_DEFAULT_WIRE_WIDTH: Dbu = Dbu::new(100);

/// The fraction of `stretch` along `track` that a wire crossing `boundary`
/// can use between the fixed metal on either side of it.
pub(super) fn usable_fraction(
    fixed: &impl FixedMetal,
    track: TrackRef,
    boundary: Dbu,
    stretch: Interval,
) -> f64 {
    if stretch.is_empty() {
        return 0.0;
    }
    let mut usable = stretch;
    for taken in fixed.fixed_stretches(track, stretch) {
        if taken.lo <= boundary && boundary <= taken.hi {
            return 0.0;
        }
        if taken.hi < boundary {
            usable.lo = usable.lo.max(taken.hi);
        } else {
            usable.hi = usable.hi.min(taken.lo);
        }
    }
    usable.len().to_f64() / stretch.len().to_f64()
}

/// Whether fixed metal leaves a gap of at least `length` on `track` within
/// `extent`.
fn has_gap(fixed: &impl FixedMetal, track: TrackRef, extent: Interval, length: Dbu) -> bool {
    let mut taken: Vec<Interval> = fixed.fixed_stretches(track, extent).collect();
    taken.sort_unstable();
    let mut free_from = extent.lo;
    for stretch in taken {
        if stretch.lo - free_from >= length {
            return true;
        }
        free_from = free_from.max(stretch.hi);
    }
    extent.hi - free_from >= length
}

/// How far the pads of the vias touching `layer` reach along its tracks.
fn via_pad_along(tech: &RouteTech, layer: LayerId) -> Dbu {
    let lt = tech.layer(layer);
    let along = |rect: &Rect<Dbu>| {
        let (_, Interval { lo: b0, hi: b1 }) = lt.across_along_rect(rect);
        b1 - b0
    };
    let from_below = layer
        .cut_below()
        .and_then(|cut| tech.vias.get(cut))
        .map(|v| along(&v.top));
    let from_above = tech.vias.get(layer.cut_above()).map(|v| along(&v.bottom));
    from_below
        .into_iter()
        .chain(from_above)
        .max()
        .unwrap_or(lt.width)
        .max(lt.width)
}

/// The share of a track a via somewhere in a gcell of `layer` is expected
/// to take from each edge beside the gcell.
///
/// A via at a uniformly random position along the gcell blocks its pad and
/// the spacing around it, and a wire crossing an edge can use the stretch
/// between the two gcell centres only up to that blockage, as
/// [`usable_fraction`] counts fixed metal. Averaged over the position, each
/// adjacent edge loses an eighth of a track plus the blocked length over
/// the gcell size.
// NOTE: grt cugr's GridGraph::commitVia adds min-area wire demand to the
// neighbouring edges; PARE models the pad's expected blockage instead.
pub(super) fn via_loss(tech: &RouteTech, layer: LayerId, gcell: Dbu) -> Tracks {
    let lt = tech.layer(layer);
    let blocked = via_pad_along(tech, layer) / 2 + lt.spacing;
    let loss = VIA_LOSS_BASE + blocked.to_f64() / gcell.to_f64();
    Tracks::from_f64(loss.min(MAX_VIA_LOSS))
}

/// The share of a track a via takes from each edge beside its gcell on
/// average whatever its pad: an eighth, from averaging its position along
/// the gcell. PARE tunable, no `OpenROAD` counterpart.
const VIA_LOSS_BASE: f64 = 0.125;

/// The most of a track a via takes from an edge beside its gcell. PARE
/// tunable, no `OpenROAD` counterpart.
const MAX_VIA_LOSS: f64 = 0.5;

/// The via stacks of a tree: for every column of cells on consecutive
/// layers at one position, the cells a via pad lands on.
fn via_stacks(paths: &[Vec<GCell>]) -> Vec<Vec<GCell>> {
    let mut cells: Vec<GCell> = paths.iter().flatten().copied().collect();
    cells.sort_unstable_by_key(|c| (c.x, c.y, c.layer));
    cells.dedup();
    let mut stacks = Vec::new();
    let mut run: Vec<GCell> = Vec::new();
    let mut flush = |run: &mut Vec<GCell>| {
        if run.len() > 1 {
            stacks.push(std::mem::take(run));
        } else {
            run.clear();
        }
    };
    for c in cells {
        let continues = run
            .last()
            .is_some_and(|p| p.x == c.x && p.y == c.y && p.layer.index() + 1 == c.layer.index());
        if !continues {
            flush(&mut run);
        }
        run.push(c);
    }
    flush(&mut run);
    stacks
}
