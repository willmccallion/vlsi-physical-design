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
//! search (`maze`), each time in a search area grown around their pins so
//! they can detour further, in parallel batches whose areas do not meet.
//! The cells each net passes through become its guide for detailed routing.

mod maze;
mod pattern;
mod steiner;
mod tracks;

use crate::occupancy::{FixedMetal, TrackRef};
use crate::tech::RouteTech;
use maze::route_tree;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::library::Direction;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pattern::route_pattern;
use rayon::prelude::*;
use rustc_hash::FxHashMap;
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
    usage: Vec<Tracks>,
    history: Vec<f32>,
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
            usage: vec![Tracks::ZERO; per_layer * layers.len()],
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
        self.usage.fill(Tracks::ZERO);
        self.history.fill(0.0);
    }

    /// Each gcell's congestion, row by row: the largest share of its
    /// capacity the routes take on any routable layer, over both edges of
    /// the gcell along that layer, as `OpenROAD`'s placer reads its global
    /// router. A layer does not count in a gcell where fixed metal takes
    /// [`IGNORED_BLOCKAGE`] of the tracks or more, or every one of them.
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
        Some(self.usage[e].to_f64() / capacity)
    }

    /// Every gcell's [`CapacityReduction`] in the terms `FastRoute` reports
    /// it to RUDY: each reachable layer adds the tracks of the edge leaving
    /// the gcell along its direction and the tracks fixed metal costs that
    /// edge, before the reserve, to the totals of its direction; the last
    /// gcell of a row or column takes those of the edge before it.
    pub fn capacity_reduction(&self, tech: &RouteTech) -> CapacityReduction {
        let (nx, ny) = (usize::from(self.nx), usize::from(self.ny));
        let mut horizontal = TileTracks::zeroed(nx * ny);
        let mut vertical = TileTracks::zeroed(nx * ny);
        let reachable = self.reachable_layers();
        for (l, &reached) in self.layers().zip(&reachable) {
            if !reached {
                continue;
            }
            let layer = LayerId::new(l);
            let direction = self.direction(layer);
            let along = match direction {
                Direction::Horizontal => nx,
                Direction::Vertical => ny,
            };
            if along < 2 {
                continue;
            }
            let totals = match direction {
                Direction::Horizontal => &mut horizontal,
                Direction::Vertical => &mut vertical,
            };
            let tracks = self.tracks_per_line(tech, layer);
            for y in 0..ny {
                for x in 0..nx {
                    let (line, edge) = match direction {
                        Direction::Horizontal => (y, self.cell_index(layer, x.min(along - 2), y)),
                        Direction::Vertical => (x, self.cell_index(layer, x, y.min(along - 2))),
                    };
                    let usable = self.capacity[edge].to_f64() / (1.0 - self.reserve);
                    totals.capacity[y * nx + x] += tracks[line];
                    totals.reduction[y * nx + x] += (tracks[line] - usable).max(0.0);
                }
            }
        }
        CapacityReduction {
            origin: self.origin,
            gcell: self.gcell,
            nx,
            ny,
            horizontal,
            vertical,
        }
    }

    /// Whether a route can reach each layer, lowest first: the lowest can
    /// be, and each one above it when the layer below it can be and some
    /// gcell leaves room for a via pad on both, since a via up needs one.
    /// A layer that fixed metal blocks everywhere, such as a die-wide
    /// routing blockage, cuts off every layer above it.
    fn reachable_layers(&self) -> Vec<bool> {
        let per_layer = usize::from(self.nx) * usize::from(self.ny);
        let has_room = |offset: usize, cell: usize| !self.via_blocked[offset * per_layer + cell];
        let mut reachable = Vec::with_capacity(self.directions.len());
        for offset in 0..self.directions.len() {
            let reached = offset == 0
                || (reachable[offset - 1]
                    && (0..per_layer).any(|c| has_room(offset - 1, c) && has_room(offset, c)));
            reachable.push(reached);
        }
        reachable
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
    fn add_tree(&mut self, net: &GlobalNet, paths: &[Vec<GCell>], sign: Sign) {
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
            self.usage[e] = match sign {
                Sign::Add => self.usage[e] + t,
                Sign::Remove => self.usage[e].saturating_sub(t),
            };
        }
    }

    fn overflowed(&self, e: usize) -> bool {
        self.usage[e] > self.capacity[e]
    }

    /// Total usage above capacity over all edges.
    pub fn overflow(&self) -> Tracks {
        self.usage
            .iter()
            .zip(&self.capacity)
            .map(|(&u, &c)| u.saturating_sub(c))
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
                    totals.usage += self.usage[e];
                    totals.overflow += self.usage[e].saturating_sub(self.capacity[e]);
                }
                totals
            })
            .collect()
    }

    /// Cost of `demand` more tracks on edge `e`.
    fn edge_cost(&self, e: usize, demand: Tracks, costs: GlobalCosts) -> f64 {
        let excess = (self.usage[e] + demand - self.capacity[e]).to_f64();
        let congestion = costs.congestion / (1.0 + (-CONGESTION_SLOPE * excess).exp());
        let overflow = costs.penalty * excess.max(0.0);
        1.0 + f64::from(self.history[e]) + congestion + overflow
    }
}

/// Each gcell's routing capacity in each direction and the part of it fixed
/// metal takes, in tracks summed over the layers a route can reach.
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
    /// Tracks of the horizontal layers.
    pub horizontal: TileTracks,
    /// Tracks of the vertical layers.
    pub vertical: TileTracks,
}

/// Tracks of one direction through each gcell, row by row.
#[derive(Clone, Debug, PartialEq)]
pub struct TileTracks {
    /// Tracks before any reduction.
    pub capacity: Vec<f64>,
    /// Tracks fixed metal takes.
    pub reduction: Vec<f64>,
}

impl TileTracks {
    fn zeroed(gcells: usize) -> Self {
        Self {
            capacity: vec![0.0; gcells],
            reduction: vec![0.0; gcells],
        }
    }
}

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
    route_initial(grid, nets, costs, &mut routes);
    let mut best = Best::of(grid);
    let mut stalled = 0;
    for _ in 0..rounds {
        if best.overflow == Tracks::ZERO || stalled >= STALL_ROUNDS {
            break;
        }
        for e in 0..grid.usage.len() {
            let over = grid.usage[e].saturating_sub(grid.capacity[e]);
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
        costs.penalty *= PENALTY_GROWTH;
        let start = std::time::Instant::now();
        route_disjoint(grid, nets, &ripped, &margins, costs, &mut routes);
        log::debug!(
            "global round: {} nets ripped, overflow {}, {:.1}s",
            ripped.len(),
            grid.overflow().rounded(),
            start.elapsed().as_secs_f64()
        );
        if grid.overflow() < best.overflow {
            best = Best::of(grid);
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
    for (e, (&used, &cap)) in grid.usage.iter().zip(&grid.capacity).enumerate() {
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

/// Rounds without a new lowest overflow after which negotiation stops.
const STALL_ROUNDS: usize = 8;

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
            usage: grid.usage.clone(),
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
        grid.usage = self.usage;
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
            .map(|&i| (i, route_tree(shared, &nets[i], margins[i], costs)))
            .collect();
        for (i, tree) in found {
            grid.add_tree(&nets[i], &tree, Sign::Add);
            let _ = routes.paths.insert(nets[i].net, tree);
        }
        pending = rest;
    }
    log::debug!("global reroute of {} nets in {round} batches", which.len());
}

/// Edge, in gcells, of the blocks that keep the search areas of nets
/// rerouted together apart.
const BATCH_BLOCK: u16 = 4;

/// Pattern-routes every net, shortest first, in parallel chunks, each
/// against the usage left by the chunks before it. Nets in one chunk do not
/// see each other; the rip-up rounds settle the overflow that causes.
fn route_initial(
    grid: &mut GlobalGrid,
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
            usage: vec![Tracks::ZERO; edges],
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
        let mut grid = grid();
        let net = net(vec![at(1, 2, 5), at(1, 5, 5)]);
        let paths = vec![vec![at(1, 2, 5), at(1, 3, 5), at(1, 4, 5), at(1, 5, 5)]];

        grid.add_tree(&net, &paths, Sign::Add);

        for x in 2..5 {
            assert_eq!(grid.usage[grid.edge(at(1, x, 5))], Tracks::ONE);
        }
        assert_eq!(grid.usage[grid.edge(at(1, 1, 5))], Tracks::ZERO);
        assert_eq!(grid.usage[grid.edge(at(1, 5, 5))], Tracks::ZERO);
    }

    #[test]
    fn a_via_costs_the_edges_beside_it_that_its_own_wire_does_not_cross() {
        let mut grid = grid();
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

        assert_eq!(grid.usage[grid.edge(at(1, 4, 5))], Tracks::ONE);
        assert_eq!(grid.usage[grid.edge(at(1, 5, 5))], loss);
        assert_eq!(grid.usage[grid.edge(at(2, 5, 5))], Tracks::ONE);
        assert_eq!(grid.usage[grid.edge(at(2, 5, 4))], loss);
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
        grid.usage.extend(vec![Tracks::ZERO; edges]);
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

        assert_eq!(grid.usage[grid.edge(at(2, 5, 4))], loss);
        assert_eq!(grid.usage[grid.edge(at(2, 5, 5))], loss);
        assert_eq!(grid.usage[grid.edge(at(3, 4, 5))], loss);
        assert_eq!(grid.usage[grid.edge(at(3, 5, 5))], Tracks::ONE);
    }

    #[test]
    fn a_terminal_pad_is_not_charged_again() {
        let mut grid = grid();
        let net = net(vec![at(1, 5, 5), at(2, 5, 7)]);
        let paths = vec![vec![at(1, 5, 5), at(2, 5, 5), at(2, 5, 6), at(2, 5, 7)]];

        grid.add_tree(&net, &paths, Sign::Add);

        assert_eq!(grid.usage[grid.edge(at(1, 4, 5))], Tracks::ZERO);
        assert_eq!(grid.usage[grid.edge(at(1, 5, 5))], Tracks::ZERO);
    }

    #[test]
    fn removing_a_tree_restores_the_usage_exactly() {
        let mut grid = grid();
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

        assert!(grid.usage.iter().all(|&u| u == Tracks::ZERO));
    }

    #[test]
    fn a_wider_rule_demands_more_tracks_per_edge() {
        let mut grid = grid();
        let net = GlobalNet {
            demand: vec![Tracks::whole(2); 2],
            ..net(vec![at(1, 2, 5), at(1, 4, 5)])
        };
        let paths = vec![vec![at(1, 2, 5), at(1, 3, 5), at(1, 4, 5)]];

        grid.add_tree(&net, &paths, Sign::Add);

        assert_eq!(grid.usage[grid.edge(at(1, 2, 5))], Tracks::whole(2));
        assert_eq!(grid.usage[grid.edge(at(1, 3, 5))], Tracks::whole(2));
        assert_eq!(grid.overflow(), Tracks::ZERO);
    }
}
