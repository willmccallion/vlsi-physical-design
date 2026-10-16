//! The exact design rule checker as the judge of detailed routing.
//!
//! The search prices steps with the occupancy's approximate conflict model,
//! but whether a region's result is kept, which of its pieces are routed
//! again, and which nets a round repairs are decided by the same checker
//! that judges the finished design, so every rule it knows is enforced
//! here. A region is judged from the layout of its window: the fixed shapes
//! near it, the wiring of every net present there, and the candidate wiring
//! of its pieces.

use super::{DetailedRouter, Marker};
use crate::wiring::{Edge, NetRoute, Wiring, gap_fills, tidy_patches};
use pare_common::db::design::RouteElem;
use pare_common::db::indices::{CutLayerId, LayerId, NetId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::drc::{
    Findings, Layout, Marker as DrcMarker, Owner, Violation, check_tiled, check_window,
};
use rustc_hash::FxHashSet;

/// How many halos wide a tile of the whole-design check is.
const TILE_HALOS: i64 = 32;

/// Where the fixed shapes of the design lie, by gcell, so that the layout
/// of a window is built from the shapes near it alone.
#[derive(Debug)]
pub(super) struct ShapeTiles {
    origin: Point<Dbu>,
    cell: Dbu,
    nx: usize,
    ny: usize,
    buckets: Vec<Vec<(LayerRef, u32)>>,
}

impl ShapeTiles {
    /// Buckets every shape of `layout` by the `cell`-sized tiles over `die`.
    pub(super) fn new(layout: &Layout, die: &Rect<Dbu>, cell: Dbu) -> Self {
        let cell = cell.max(Dbu(1));
        let count = |span: Dbu| usize::try_from(span.get() / cell.get() + 1).unwrap_or(1);
        let (nx, ny) = (count(die.width()), count(die.height()));
        let mut tiles = Self {
            origin: die.min,
            cell,
            nx,
            ny,
            buckets: vec![Vec::new(); nx * ny],
        };
        for l in 0..layout.metal_layers() {
            for (i, s) in layout.metal(l).iter().enumerate() {
                tiles.insert(LayerRef::Routing(LayerId::new(l)), i, &s.rect);
            }
        }
        for c in 0..layout.cut_layers() {
            for (i, s) in layout.cut(c).iter().enumerate() {
                tiles.insert(LayerRef::Cut(CutLayerId::new(c)), i, &s.rect);
            }
        }
        tiles
    }

    fn insert(&mut self, layer: LayerRef, index: usize, rect: &Rect<Dbu>) {
        let index = u32::try_from(index).unwrap_or(u32::MAX);
        let (x0, x1, y0, y1) = self.span(rect);
        for j in y0..=y1 {
            for i in x0..=x1 {
                self.buckets[j * self.nx + i].push((layer, index));
            }
        }
    }

    /// The columns and rows of tiles `rect` meets, clamped to the grid.
    fn span(&self, rect: &Rect<Dbu>) -> (usize, usize, usize, usize) {
        let index = |v: Dbu, origin: Dbu, count: usize| {
            usize::try_from((v - origin).get().div_euclid(self.cell.get()))
                .unwrap_or(0)
                .min(count - 1)
        };
        (
            index(rect.min.x, self.origin.x, self.nx),
            index(rect.max.x, self.origin.x, self.nx),
            index(rect.min.y, self.origin.y, self.ny),
            index(rect.max.y, self.origin.y, self.ny),
        )
    }

    /// The shapes, as (layer, position on the layer), whose tiles meet
    /// `window`; some may lie outside it.
    pub(super) fn near(&self, window: &Rect<Dbu>) -> FxHashSet<(LayerRef, u32)> {
        let (x0, x1, y0, y1) = self.span(window);
        let mut found = FxHashSet::default();
        for j in y0..=y1 {
            for i in x0..=x1 {
                found.extend(self.buckets[j * self.nx + i].iter().copied());
            }
        }
        found
    }
}

/// What judging a region found: the markers inside it and the nets with
/// a violation.
#[derive(Debug, Default)]
pub(super) struct Judgment {
    pub findings: Findings,
    pub nets: FxHashSet<NetId>,
}

/// A net's wiring as a worker proposes it: its committed wiring without
/// what the worker took out, with what it routed.
#[derive(Clone, Copy, Debug)]
pub(super) struct Candidate<'e> {
    pub net: NetId,
    pub without: &'e Wiring,
    pub with: &'e Wiring,
}

impl DetailedRouter<'_> {
    /// The window a region is judged from: the region grown by the halo.
    pub(super) fn judge_window(&self, area: &Rect<Dbu>) -> Rect<Dbu> {
        area.expand(self.halo)
    }

    /// Judges `area` with each candidate net's wiring as proposed.
    pub(super) fn judge(&self, area: &Rect<Dbu>, candidates: &[Candidate<'_>]) -> Judgment {
        let window = self.judge_window(area);
        let layout = self.window_layout(&window, candidates);
        let findings = check_window(&layout, &self.rules, *area, window);
        let nets = findings
            .violations
            .iter()
            .flat_map(|m| [m.owners.0, m.owners.1])
            .filter_map(|o| match layout.owners()[o.index()] {
                Owner::Net(n) => Some(n),
                _ => None,
            })
            .collect();
        Judgment { findings, nets }
    }

    /// The layout of everything meeting `window`: the fixed shapes there,
    /// the wiring of every net with metal or cuts in it, which brings along
    /// via pads on layers below the routable ones, and for each candidate
    /// net its kept wiring together with its candidate edges.
    fn window_layout(&self, window: &Rect<Dbu>, candidates: &[Candidate<'_>]) -> Layout {
        let mut layout = Layout::with_owners_of(&self.fixed);
        for (layer, i) in self.fixed_tiles.near(window) {
            let shape = match layer {
                LayerRef::Routing(l) => &self.fixed.metal(l.index())[i as usize],
                LayerRef::Cut(c) => &self.fixed.cut(c.index())[i as usize],
            };
            if shape.rect.touches(window) {
                layout.add_shape(layer, shape);
            }
        }
        let candidate_nets: FxHashSet<NetId> = candidates.iter().map(|c| c.net).collect();
        let mut present: FxHashSet<NetId> = FxHashSet::default();
        let mut note = |owner: crate::occupancy::Owner| {
            if let crate::occupancy::Owner::Net(n) = owner {
                let _ = present.insert(NetId(n));
            }
        };
        for l in self.tech.min_layer.index()..=self.tech.max_layer.index() {
            for owner in self.occupancy.owners_in(self.tech, LayerId::new(l), window) {
                note(owner);
            }
        }
        for c in 0..self.tech.cuts.len() {
            let cut = CutLayerId::new(c);
            let probe = crate::occupancy::Owner::Obstacle;
            for occupant in self.occupancy.cut_conflicts(cut, window, probe) {
                note(occupant.owner);
            }
        }
        for net in present {
            if !candidate_nets.contains(&net) {
                layout.add_wiring(self.db, net, &self.elems[net.index()], Some(window));
            }
        }
        for c in candidates {
            let elems: Vec<RouteElem> = self.as_committed(c).elems(self.tech);
            layout.add_wiring(self.db, c.net, &elems, Some(window));
        }
        layout
    }

    /// Every violation of the current wiring, as markers for repair, with
    /// how many there are of each kind.
    pub(super) fn exact_markers(&self) -> (Vec<Marker>, Vec<(Violation, usize)>) {
        let mut layout = self.fixed.clone();
        for (n, elems) in self.elems.iter().enumerate() {
            layout.add_wiring(self.db, NetId::new(n), elems, None);
        }
        let report = check_tiled(&layout, &self.rules, self.halo * TILE_HALOS, self.halo);
        let counts = report.counts();
        let markers = report
            .violations
            .iter()
            .map(|m| marker_of(&report.owners, m))
            .collect();
        (markers, counts)
    }

    /// The route a candidate stands for, with the gaps along its tracks
    /// filled as committing it does, so that it is judged as it will be kept.
    pub(super) fn as_committed(&self, c: &Candidate<'_>) -> NetRoute {
        let mut route = self.routes[c.net.index()].clone();
        if !c.without.is_empty() {
            let gone: FxHashSet<Edge> = c.without.edges.iter().copied().collect();
            route.edges.retain(|e| !gone.contains(e));
            route.patches.retain(|p| !c.without.patches.contains(p));
        }
        route.edges.extend_from_slice(&c.with.edges);
        route.patches.extend_from_slice(&c.with.patches);
        route.edges.extend(gap_fills(&route, self.db, self.tech));
        route.edges.sort_unstable();
        route.edges.dedup();
        tidy_patches(&mut route.patches);
        route
    }
}

/// The repair marker of a checker marker: its center, layer and nets.
fn marker_of(owners: &[Owner], m: &DrcMarker) -> Marker {
    let net = |o: pare_common::verify::drc::OwnerIndex| match owners[o.index()] {
        Owner::Net(n) => Some(n),
        _ => None,
    };
    Marker {
        at: m.bbox.center(),
        layer: m.layer,
        nets: [net(m.owners.0), net(m.owners.1)],
    }
}
