//! Global routing on a grid of gcells, in the manner of CUGR.
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
//! first (`coarse`), as in CUGR, so a net can detour as far as the free
//! tracks call for. As in CUGR, nets are planned in parallel batches whose
//! pin boxes do not meet, then rerouted in order, each seeing every earlier
//! net it could meet. The cells each net passes through become its guide
//! for detailed routing.

mod coarse;
mod maze;
mod pattern;
mod steiner;
mod tracks;

use crate::occupancy::{FixedMetal, TrackRef};
use crate::tech::RouteTech;
use coarse::{CoarseGrid, Corridor, Plan, SCALE};
use maze::{OutgrowsArea, route_tree, route_tree_within};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pattern::route_pattern;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::sync::OnceLock;
use std::sync::atomic::{AtomicI64, AtomicU32, Ordering};
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

/// The gcell grid with edge capacities, usage and history.
#[derive(Debug)]
pub struct GlobalGrid {
    origin: Point<Dbu>,
    gcell: Dbu,
    nx: u16,
    ny: u16,
    min_layer: LayerId,
    directions: Vec<Direction>,
    /// Fraction of each edge's usable tracks held back from `capacity`.
    reserve: f64,
    capacity: Vec<Tracks>,
    /// Edges no track crosses at all, which fixed metal blocks outright.
    blocked: Vec<bool>,
    /// Cells where fixed metal leaves no track of the layer room for a via pad.
    via_blocked: Vec<bool>,
    /// Per layer, the share of a track a via somewhere in a gcell is expected
    /// to take from each edge beside it.
    via_loss: Vec<Tracks>,
    usage: Usage,
    history: Vec<f32>,
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

    fn add(&self, e: usize, t: Tracks) {
        let _ = self.0[e].fetch_add(t.raw(), Ordering::Relaxed);
    }

    /// Takes `t` from edge `e`, down to no usage at most.
    fn remove(&self, e: usize, t: Tracks) {
        let _ = self.0[e].try_update(Ordering::Relaxed, Ordering::Relaxed, |v| {
            Some((v - t.raw()).max(0))
        });
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

    fn iter(&self) -> impl Iterator<Item = Tracks> + '_ {
        (0..self.len()).map(|e| self.get(e))
    }

    fn snapshot(&self) -> Vec<Tracks> {
        self.iter().collect()
    }

    fn restore(&mut self, from: &[Tracks]) {
        for (u, t) in self.0.iter_mut().zip(from) {
            *u.get_mut() = t.raw();
        }
    }

    #[cfg(test)]
    fn extend_zeroed(&mut self, edges: usize) {
        self.0.extend((0..edges).map(|_| AtomicI64::new(0)));
    }
}

/// One layer's routing resources and their use, summed over its edges.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LayerTotals {
    /// The layer.
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
enum Sign {
    Add,
    Remove,
}

/// What one layer's tracks and fixed metal give the grid.
struct LayerResources {
    /// Usable tracks across each edge.
    tracks: Vec<f64>,
    /// Cells with no room for a via pad on any track.
    via_blocked: Vec<bool>,
}

impl GlobalGrid {
    /// A grid of `gcell`-sized cells over `die` whose edges offer the free
    /// tracks crossing them less the `reserve` fraction.
    pub fn new(
        tech: &RouteTech,
        fixed: &(impl FixedMetal + Sync),
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
            .map(|&t| Tracks::from_f64(t * (1.0 - reserve)))
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
            (start, end)
        };
        let centre = |i: usize| {
            let (start, end) = extent(i);
            start + (end - start) / 2
        };
        for (t, &across) in lt.tracks.iter().enumerate() {
            let row = (across - across_origin).get().div_euclid(self.gcell.get());
            let Ok(row) = usize::try_from(row) else {
                continue;
            };
            let track = TrackRef {
                layer,
                track: crate::tech::to_u32(t),
            };
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
                    let stretch = (centre(i), centre(i + 1));
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
    /// router. A layer does not count in a gcell where fixed metal takes
    /// `IGNORED_BLOCKAGE` of the tracks or more, or every one of them.
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
        let available = tracks * (1.0 - self.reserve);
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
        adjustment: f64,
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
            for y in 0..ny {
                for x in 0..nx {
                    let (line, edge) = match direction {
                        Direction::Horizontal => (y, self.cell_index(layer, x.min(along - 2), y)),
                        Direction::Vertical => (x, self.cell_index(layer, x, y.min(along - 2))),
                    };
                    let unblocked = self.capacity[edge].to_f64() / (1.0 - self.reserve);
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
            let line = (across - origin).get().div_euclid(self.gcell.get());
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
            x: u16::try_from(x).unwrap_or(u16::MAX),
            y: u16::try_from(y).unwrap_or(u16::MAX),
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

    /// Index of the edge leaving `c` forward in its layer's direction, and
    /// of `c` itself among the layer's cells.
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

    /// The position of `layer` among the routable layers.
    const fn layer_offset(&self, layer: LayerId) -> usize {
        layer.index() - self.min_layer.index()
    }

    const fn layers(&self) -> std::ops::RangeInclusive<usize> {
        self.min_layer.index()..=self.min_layer.index() + self.directions.len() - 1
    }

    /// Whether no via pad fits in cell `c` on its layer.
    fn via_blocked(&self, c: GCell) -> bool {
        self.via_blocked[self.edge(c)]
    }

    /// Whether a wire of `c`'s layer can leave `c` forward, and the edge it
    /// takes.
    fn forward_edge(&self, c: GCell) -> Option<usize> {
        let last = match self.direction(c.layer) {
            Direction::Horizontal => c.x + 1 >= self.nx,
            Direction::Vertical => c.y + 1 >= self.ny,
        };
        (!last).then(|| self.edge(c))
    }

    /// The edge a wire of `c`'s layer enters `c` by from behind, if any.
    fn backward_edge(&self, c: GCell) -> Option<usize> {
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

    /// Places or lifts the demand of a net's paths: its track demand on
    /// every edge they cross, counting an edge the paths share once, and
    /// the via loss of its layer on each edge beside a via that no wire of
    /// the net on that layer crosses. The pad a terminal already holds as
    /// fixed metal is not counted again.
    fn add_tree(&self, net: &GlobalNet, paths: &[Vec<GCell>], sign: Sign) {
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

    fn overflowed(&self, e: usize) -> bool {
        self.usage.get(e) > self.capacity[e]
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
    pub fn layer_totals(&self) -> Vec<LayerTotals> {
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
    fn edge_cost(&self, e: usize, demand: Tracks, costs: GlobalCosts) -> f64 {
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
fn adjusted_capacity(unblocked: f64, adjustment: f64) -> f64 {
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
        Dbu((1.25 / inverse).trunc() as i64)
    } else {
        Dbu(RUDY_DEFAULT_WIRE_WIDTH)
    }
}

/// The wire width `grt::Rudy` keeps when no layer has a pitch.
const RUDY_DEFAULT_WIRE_WIDTH: i64 = 100;

/// The fraction of `stretch` along `track` that a wire crossing `boundary`
/// can use between the fixed metal on either side of it.
fn usable_fraction(
    fixed: &impl FixedMetal,
    track: TrackRef,
    boundary: Dbu,
    stretch: (Dbu, Dbu),
) -> f64 {
    let (lo, hi) = stretch;
    if hi <= lo {
        return 0.0;
    }
    let mut usable = (lo, hi);
    for (start, end) in fixed.fixed_stretches(track, lo, hi) {
        if start <= boundary && boundary <= end {
            return 0.0;
        }
        if end < boundary {
            usable.0 = usable.0.max(end);
        } else {
            usable.1 = usable.1.min(start);
        }
    }
    (usable.1 - usable.0).to_f64() / (hi - lo).to_f64()
}

/// Whether fixed metal leaves a gap of at least `length` on `track` within
/// `extent`.
fn has_gap(fixed: &impl FixedMetal, track: TrackRef, extent: (Dbu, Dbu), length: Dbu) -> bool {
    let (lo, hi) = extent;
    let mut taken: Vec<(Dbu, Dbu)> = fixed.fixed_stretches(track, lo, hi).collect();
    taken.sort_unstable();
    let mut free_from = lo;
    for (start, end) in taken {
        if start - free_from >= length {
            return true;
        }
        free_from = free_from.max(end);
    }
    hi - free_from >= length
}

/// How far the pads of the vias touching `layer` reach along its tracks.
fn via_pad_along(tech: &RouteTech, layer: LayerId) -> Dbu {
    let lt = tech.layer(layer);
    let along = |rect: &Rect<Dbu>| {
        let (_, (b0, b1)) = lt.across_along_rect(rect);
        b1 - b0
    };
    let from_below = layer
        .index()
        .checked_sub(1)
        .and_then(|below| tech.vias.get(below))
        .map(|v| along(&v.top));
    let from_above = tech.vias.get(layer.index()).map(|v| along(&v.bottom));
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
fn via_loss(tech: &RouteTech, layer: LayerId, gcell: Dbu) -> Tracks {
    let lt = tech.layer(layer);
    let blocked = via_pad_along(tech, layer) / 2 + lt.spacing;
    let loss = 0.125 + blocked.to_f64() / gcell.to_f64();
    Tracks::from_f64(loss.min(0.5))
}

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

/// Cost of an edge or via pad fixed metal blocks outright, in pattern
/// routing: far above any detour, so a plan takes one only when no plan
/// avoids it.
const BLOCKED_COST: f64 = 1e12;

/// Search margin, in gcells, around a net's first route's connections.
const FIRST_MARGIN: u16 = 2;

/// Gcells the search margin grows by each time a net is ripped up.
const MARGIN_STEP: u16 = 2;

/// Largest search margin, in gcells; wider margins leave too few nets with
/// disjoint search areas to reroute in parallel.
const MAX_MARGIN: u16 = 12;

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

    /// The position of a cell in [`Self::cells`], if the route has it.
    pub fn position(&self, c: GCell) -> Option<usize> {
        self.index.get(&c).copied()
    }

    /// Every link of the tree as (cell, parent) positions in
    /// [`Self::cells`]; the root of each separate tree is left out.
    pub fn links(&self) -> impl Iterator<Item = (usize, usize)> + '_ {
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
#[derive(Clone, Debug)]
pub struct GlobalNet {
    /// The net.
    pub net: NetId,
    /// One gcell per pin.
    pub terminals: Vec<GCell>,
    /// Tracks one of its wires takes on each routable layer, lowest first.
    pub demand: Vec<Tracks>,
}

impl GlobalNet {
    /// Tracks a wire of the net takes on the layer at `offset` among the
    /// routable layers.
    fn demand(&self, offset: usize) -> Tracks {
        self.demand.get(offset).copied().unwrap_or(Tracks::ONE)
    }
}

/// Tracks a wire of `net` takes on each routable layer: one, or as many
/// pitches as its non-default rule's width and spacing span.
pub fn track_demand(db: &NetlistDB, tech: &RouteTech, net: NetId) -> Vec<Tracks> {
    let rule = db.nets[net.index()]
        .non_default_rule
        .map(|r| &db.non_default_rules[r.index()]);
    (tech.min_layer.index()..=tech.max_layer.index())
        .map(|l| {
            let layer = LayerId::new(l);
            let lt = tech.layer(layer);
            let Some(ndr) = rule.and_then(|r| r.layers.iter().find(|x| x.layer == layer)) else {
                return Tracks::ONE;
            };
            let pitch = lt.pitch().get();
            if pitch <= 0 {
                return Tracks::ONE;
            }
            let span = (ndr.width + ndr.spacing.unwrap_or(lt.spacing)).get();
            let tracks = (span + pitch - 1) / pitch;
            Tracks::whole(u32::try_from(tracks.max(1)).unwrap_or(1))
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
pub fn restore_global(
    grid: &mut GlobalGrid,
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
        grid.add_tree(net, &tree, Sign::Add);
        let _ = routes.paths.insert(net.net, tree);
    }
    Ok(routes)
}

/// Routes `nets` and negotiates overflow for at most `rounds` rounds.
///
/// The history of overflow on each edge grows every round. The overflow
/// penalty grows by `PENALTY_STEP` after a round that lowers the overflow
/// and shrinks by it after one that raises it: more pressure clears the
/// last overflow of a routable design, while pressure that keeps growing
/// on overflow that cannot be cleared drives nets into detours that
/// overflow more than they relieve. Negotiation stops early once
/// `STALL_ROUNDS` pass without lowering the overflow by
/// `MIN_PROGRESS` of what it was when it last did. Returns the routes of
/// the round with the lowest overflow, and leaves the grid's usage at
/// theirs.
pub fn route_global(grid: &mut GlobalGrid, nets: &[GlobalNet], rounds: usize) -> GlobalRoutes {
    let mut costs = GlobalCosts {
        via: 1.0,
        congestion: 1.0,
        penalty: 1.0,
    };
    let mut routes = GlobalRoutes::default();
    let mut margins = vec![FIRST_MARGIN; nets.len()];
    route_initial(grid, nets, costs, &mut routes);
    let mut coarse = CoarseGrid::new(grid);
    let mut best = Best::of(grid);
    let mut progress_from = best.overflow;
    let mut stalled = 0;
    for _ in 0..rounds {
        if best.overflow == Tracks::ZERO || stalled >= STALL_ROUNDS {
            break;
        }
        for e in 0..grid.usage.len() {
            let over = grid.usage.get(e).saturating_sub(grid.capacity[e]);
            grid.history[e] += over.to_f64() as f32;
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
                grid.add_tree(&nets[i], &tree, Sign::Remove);
                best.shelve(nets[i].net, tree);
            }
            margins[i] = margins[i].saturating_add(MARGIN_STEP).min(MAX_MARGIN);
        }
        ripped.sort_by_key(|&i| (span(&nets[i].terminals), i));
        let start = std::time::Instant::now();
        coarse.refresh(grid);
        let levels = disjoint_levels(grid, nets, &ripped);
        let mut spent = RoundTimes::default();
        for level in &levels {
            let shared: &GlobalGrid = grid;
            let planning = std::time::Instant::now();
            let plans: Vec<Option<Plan>> = level
                .par_iter()
                .map(|&i| coarse.plan(shared, &nets[i]))
                .collect();
            spent.planning += planning.elapsed().as_secs_f64();
            let reroute = Reroute {
                which: level,
                margins: &margins,
                plans: &plans,
            };
            let routing = std::time::Instant::now();
            route_disjoint(grid, nets, &reroute, costs, &mut routes);
            spent.routing += routing.elapsed().as_secs_f64();
            let refreshing = std::time::Instant::now();
            let touched = touched_coarse_cells(
                grid,
                level.iter().filter_map(|&i| routes.paths.get(&nets[i].net)),
            );
            coarse.refresh_cells(grid, &touched);
            spent.refreshing += refreshing.elapsed().as_secs_f64();
        }
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
        let overflow = grid.overflow();
        if overflow < best.overflow {
            costs.penalty *= PENALTY_STEP;
            best = Best::of(grid);
        } else {
            costs.penalty /= PENALTY_STEP;
        }
        if overflow.to_f64() <= progress_from.to_f64() * (1.0 - MIN_PROGRESS) {
            progress_from = overflow;
            stalled = 0;
        } else {
            stalled += 1;
        }
    }
    if grid.overflow() > best.overflow {
        best.restore(grid, &mut routes);
    }
    log_zero_capacity_crossings(grid, &routes);
    log_overflow_regions(grid);
    routes
}

/// Logs, at debug level, every net whose route uses edges without capacity.
fn log_zero_capacity_crossings(grid: &GlobalGrid, routes: &GlobalRoutes) {
    if !log::log_enabled!(log::Level::Debug) {
        return;
    }
    for (net, tree) in &routes.paths {
        let blocked = tree
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
    for ((x, y), over) in worst.into_iter().take(12) {
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

/// Rounds without enough progress after which negotiation stops.
const STALL_ROUNDS: usize = 3;

/// Share of the overflow a round must remove, against the overflow when
/// it last made progress, to count as progress.
const MIN_PROGRESS: f64 = 0.005;

/// Factor the overflow penalty grows by after a round that lowers the
/// overflow, and shrinks by after one that does not.
const PENALTY_STEP: f64 = 1.5;

/// Share of a gcell's tracks on a layer that fixed metal must take for the
/// layer's congestion there to be left out of [`GlobalGrid::congestion`],
/// since no placement changes what crosses it; `OpenROAD`'s
/// `ignoreEdgeRatio`.
const IGNORED_BLOCKAGE: f64 = 0.8;

/// The round with the lowest overflow so far: its edge usage, and the
/// routes of that round that later rounds have since replaced. Every other
/// net still has the route it had then.
struct Best {
    overflow: Tracks,
    usage: Vec<Tracks>,
    shelved: FxHashMap<NetId, Vec<Vec<GCell>>>,
}

impl Best {
    /// Marks the grid's current routes as the best.
    fn of(grid: &GlobalGrid) -> Self {
        Self {
            overflow: grid.overflow(),
            usage: grid.usage.snapshot(),
            shelved: FxHashMap::default(),
        }
    }

    /// Keeps the route `net` had in the best round as it is ripped up, the
    /// first time since that round.
    fn shelve(&mut self, net: NetId, tree: Vec<Vec<GCell>>) {
        let _ = self.shelved.entry(net).or_insert(tree);
    }

    /// Puts the best round's routes and usage back.
    fn restore(self, grid: &mut GlobalGrid, routes: &mut GlobalRoutes) {
        grid.usage.restore(&self.usage);
        for (net, tree) in self.shelved {
            let _ = routes.paths.insert(net, tree);
        }
    }
}

/// Half-perimeter of a net's terminals' bounding box, in gcells.
fn span(cells: &[GCell]) -> u32 {
    let (x0, x1, y0, y1) = window(cells, 0, u16::MAX, u16::MAX);
    u32::from(x1 - x0) + u32::from(y1 - y0)
}

/// Nets to reroute in one batch: their indices in the order to route them,
/// with each net's search margin and the coarse plan made for each.
struct Reroute<'a> {
    which: &'a [usize],
    margins: &'a [u16],
    /// Aligned with `which`.
    plans: &'a [Option<Plan>],
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
fn route_disjoint(
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
        let tree = if let Some(Ok(tree)) = tree.into_inner() {
            tree
        } else {
            let tree = route_tree(grid, &nets[i], margins[i], costs, plans[k].as_ref());
            grid.add_tree(&nets[i], &tree, Sign::Add);
            tree
        };
        let _ = routes.paths.insert(nets[i].net, tree);
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
    let mut last = vec![u32::MAX; blocks_x * blocks_y];
    let mut waits: Vec<AtomicU32> = Vec::with_capacity(which.len());
    let mut next: Vec<Vec<u32>> = vec![Vec::new(); which.len()];
    for (k, &i) in which.iter().enumerate() {
        let area = window(
            &nets[i].terminals,
            margins[i].saturating_add(1),
            grid.nx,
            grid.ny,
        );
        let mut blocks = blocks_of(area, blocks_x);
        if let Some(plan) = &plans[k] {
            blocks.extend(corridor_blocks(plan.corridor(), grid, blocks_x));
        }
        blocks.sort_unstable();
        blocks.dedup();
        let mut before: Vec<u32> = Vec::new();
        for b in blocks {
            let slot = &mut last[b];
            if *slot != u32::MAX {
                before.push(*slot);
            }
            *slot = to_u32(k);
        }
        before.sort_unstable();
        before.dedup();
        for &p in &before {
            next[p as usize].push(to_u32(k));
        }
        waits.push(AtomicU32::new(to_u32(before.len())));
    }
    (waits, next)
}

/// The blocks of [`BATCH_BLOCK`] gcells a window of gcells meets.
fn blocks_of((x0, x1, y0, y1): (u16, u16, u16, u16), blocks_x: usize) -> Vec<usize> {
    let block = |v: u16| usize::from(v / BATCH_BLOCK);
    let mut out = Vec::new();
    for y in block(y0)..=block(y1) {
        for x in block(x0)..=block(x1) {
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
            let cell = (
                (cx * SCALE).saturating_sub(1),
                (cx * SCALE + SCALE).min(grid.nx - 1),
                (cy * SCALE).saturating_sub(1),
                (cy * SCALE + SCALE).min(grid.ny - 1),
            );
            blocks_of(cell, blocks_x)
        })
        .collect()
}

/// The nets [`route_disjoint`] reroutes, what each waits for and what it
/// found.
struct Schedule<'a> {
    grid: &'a GlobalGrid,
    nets: &'a [GlobalNet],
    which: &'a [usize],
    margins: &'a [u16],
    plans: &'a [Option<Plan>],
    costs: GlobalCosts,
    /// Earlier nets each one still waits for.
    waits: Vec<AtomicU32>,
    /// The later nets waiting for each one.
    next: Vec<Vec<u32>>,
    trees: Vec<OnceLock<Result<Vec<Vec<GCell>>, OutgrowsArea>>>,
}

impl Schedule<'_> {
    /// Routes the `k`th net inside its area and places its demand, or leaves
    /// it for later when its area holds no route, then starts every later net
    /// that was waiting only for it.
    fn run<'s>(&'s self, scope: &rayon::Scope<'s>, k: usize) {
        let i = self.which[k];
        let tree = route_tree_within(
            self.grid,
            &self.nets[i],
            self.margins[i],
            self.costs,
            self.plans[k].as_ref(),
        );
        if let Ok(tree) = &tree {
            self.grid.add_tree(&self.nets[i], tree, Sign::Add);
        }
        let _ = self.trees[k].set(tree);
        for &later in &self.next[k] {
            let later = later as usize;
            if self.waits[later].fetch_sub(1, Ordering::AcqRel) == 1 {
                scope.spawn(move |scope| self.run(scope, later));
            }
        }
    }
}

fn to_u32(v: usize) -> u32 {
    u32::try_from(v).unwrap_or(u32::MAX)
}

/// Seconds one rip-up round spends in each of its steps.
#[derive(Debug, Default)]
struct RoundTimes {
    planning: f64,
    routing: f64,
    refreshing: f64,
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
fn disjoint_levels(grid: &GlobalGrid, nets: &[GlobalNet], ripped: &[usize]) -> Vec<Vec<usize>> {
    let blocks_x = usize::from(grid.nx.div_ceil(BATCH_BLOCK));
    let blocks_y = usize::from(grid.ny.div_ceil(BATCH_BLOCK));
    let mut height = vec![0_usize; blocks_x * blocks_y];
    let mut levels: Vec<Vec<usize>> = Vec::new();
    for &i in ripped {
        let area = window(&nets[i].terminals, 0, grid.nx, grid.ny);
        let blocks = blocks_of(area, blocks_x);
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
fn touched_coarse_cells<'a>(
    grid: &GlobalGrid,
    trees: impl Iterator<Item = &'a Vec<Vec<GCell>>>,
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

/// Pattern-routes every net, shortest first, in parallel chunks, each
/// against the usage left by the chunks before it. Nets in one chunk do not
/// see each other; the rip-up rounds settle the overflow that causes.
fn route_initial(
    grid: &GlobalGrid,
    nets: &[GlobalNet],
    costs: GlobalCosts,
    routes: &mut GlobalRoutes,
) {
    let chunk = rayon::current_num_threads().max(1) * INITIAL_PER_THREAD;
    let mut order: Vec<usize> = (0..nets.len()).collect();
    order.sort_by_key(|&i| (span(&nets[i].terminals), i));
    for batch in order.chunks(chunk) {
        let shared: &GlobalGrid = grid;
        let found: Vec<(usize, Vec<Vec<GCell>>)> = batch
            .par_iter()
            .map(|&i| (i, route_pattern(shared, &nets[i], costs)))
            .collect();
        for (i, tree) in found {
            grid.add_tree(&nets[i], &tree, Sign::Add);
            let _ = routes.paths.insert(nets[i].net, tree);
        }
    }
}

/// Nets per thread pattern-routed in one chunk of the initial routing.
const INITIAL_PER_THREAD: usize = 32;

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

    fn at(layer: usize, x: u16, y: u16) -> GCell {
        GCell {
            layer: LayerId::new(layer),
            x,
            y,
        }
    }

    /// A 10x10 grid with a horizontal layer 1 over a vertical layer 2,
    /// every edge holding 4 tracks and every via taking a tenth of one.
    fn grid() -> GlobalGrid {
        let (nx, ny) = (10, 10);
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
            via_loss: vec![Tracks::from_f64(0.1); 2],
            usage: Usage::zeroed(edges),
            history: vec![0.0; edges],
        }
    }

    fn net(terminals: Vec<GCell>) -> GlobalNet {
        GlobalNet {
            net: NetId::new(0),
            terminals,
            demand: vec![Tracks::ONE; 2],
        }
    }

    const COSTS: GlobalCosts = GlobalCosts {
        via: 1.0,
        congestion: 1.0,
        penalty: 1.0,
    };

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
            let tree = route_tree(grid, &nets[i], 1, COSTS, None);
            grid.add_tree(&nets[i], &tree, Sign::Add);
            let _ = routes.paths.insert(nets[i].net, tree);
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
            .map(|k| numbered(k, vec![cell(0, 3 + k as u16 % 3), cell(9, 4)]))
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

        assert_eq!(routes.paths, expected.paths);
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

    #[test]
    fn every_net_rerouted_together_is_routed_once_as_in_order() {
        let (serial, nets) = chained_nets();
        let order: Vec<usize> = (0..nets.len()).collect();
        let expected = route_in_order(&serial, &nets, &order);

        for _ in 0..20 {
            let (parallel, _) = chained_nets();
            let routes = route_level(&parallel, &nets, &order);

            assert_eq!(routes.paths, expected.paths);
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

        assert_eq!(routes.paths, expected.paths);
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
