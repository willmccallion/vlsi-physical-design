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
use crate::runs::EdgeSet;
use crate::wiring::{NetRoute, Wiring, gap_fills, tidy_patches};
use pare_common::db::design::{NetlistDB, RouteElem};
use pare_common::db::indices::{IdVec, LayerId, NetId};
use pare_common::db::library::LayerRef;
use pare_common::db::shapes::route_elem_shapes;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::drc::{
    Findings, Layout, Marker as DrcMarker, Origin, Owner, OwnerIndex, Rules, Violation,
    check_tiles_lazily, check_window,
};
use rayon::prelude::*;
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
    /// Buckets every shape of `layout` by the `cell`-sized tiles over `die`;
    /// `cell` is positive.
    pub(super) fn new(layout: &Layout, die: &Rect<Dbu>, cell: Dbu) -> Self {
        let count = |span: Dbu| (span.get() / cell.get() + 1) as usize;
        let (nx, ny) = (count(die.width()), count(die.height()));
        let mut tiles = Self {
            origin: die.min,
            cell,
            nx,
            ny,
            buckets: vec![Vec::new(); nx * ny],
        };
        for l in layout.metal_layer_ids() {
            for (i, s) in layout.metal(l).iter().enumerate() {
                tiles.insert(LayerRef::Routing(l), i, &s.rect);
            }
        }
        for c in layout.cut_layer_ids() {
            for (i, s) in layout.cut(c).iter().enumerate() {
                tiles.insert(LayerRef::Cut(c), i, &s.rect);
            }
        }
        tiles
    }

    fn insert(&mut self, layer: LayerRef, index: usize, rect: &Rect<Dbu>) {
        let index = crate::tech::index_u32(index);
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
            let at = (v - origin).get().div_euclid(self.cell.get());
            at.clamp(0, count as i64 - 1) as usize
        };
        (
            index(rect.min.x, self.origin.x, self.nx),
            index(rect.max.x, self.origin.x, self.nx),
            index(rect.min.y, self.origin.y, self.ny),
            index(rect.max.y, self.origin.y, self.ny),
        )
    }

    /// The shapes, as (layer, position on the layer), whose tiles meet
    /// `window`, in order and each once; some may lie outside it.
    pub(super) fn near(&self, window: &Rect<Dbu>) -> Vec<(LayerRef, u32)> {
        let (x0, x1, y0, y1) = self.span(window);
        let mut found = Vec::new();
        for j in y0..=y1 {
            for i in x0..=x1 {
                found.extend(self.buckets[j * self.nx + i].iter().copied());
            }
        }
        found.sort_unstable();
        found.dedup();
        found
    }
}

/// Whether wiring of `net` covering `metal` on `layer`, among the fixed
/// shapes of `fixed` that `tiles` finds near it, breaks no design rule.
pub(super) fn wiring_is_clean(
    fixed: &Layout,
    tiles: &ShapeTiles,
    rules: &Rules,
    net: NetId,
    layer: LayerId,
    metal: &Rect<Dbu>,
) -> bool {
    let core = metal.expand(rules.max_reach());
    let window = core.expand(rules.max_reach());
    let mut layout = Layout::with_owners_of(fixed);
    for (layer, i) in tiles.near(&window) {
        let shape = match layer {
            LayerRef::Routing(l) => &fixed.metal(l)[i as usize],
            LayerRef::Cut(c) => &fixed.cut(c)[i as usize],
        };
        if shape.rect.touches(&window) {
            layout.add_shape(layer, shape);
        }
    }
    let owner = layout.owner(Owner::Net(net), false);
    layout.add(LayerRef::Routing(layer), owner, *metal, Origin::Routing);
    check_window(&layout, rules, core, window)
        .violations
        .is_empty()
}

/// Whether `metal` of `net` on `layer` keeps `spacing` from every fixed
/// shape of another owner there, as pin access keeps a via's pads clear of
/// its cell's other shapes: the shapes of neighbouring cells count too.
pub(super) fn clear_of_fixed(
    fixed: &Layout,
    tiles: &ShapeTiles,
    net: NetId,
    layer: LayerId,
    metal: &Rect<Dbu>,
    spacing: Dbu,
) -> bool {
    let own = fixed.owner_index(Owner::Net(net));
    let reach = metal.expand(spacing);
    tiles.near(&reach).into_iter().all(|(l, i)| {
        if l != LayerRef::Routing(layer) {
            return true;
        }
        let shape = &fixed.metal(layer)[i as usize];
        Some(shape.owner) == own || !shape.rect.overlaps(&reach)
    })
}

/// What judging a region found: the markers inside it, and for each
/// marker its two owners as nets, none for an owner that is not a net.
#[derive(Debug, Default)]
pub(super) struct Judgment {
    pub findings: Findings,
    pub owners: Vec<[Option<NetId>; 2]>,
}

/// A net's wiring as a worker proposes it: its committed wiring without
/// what the worker took out, with what it routed.
#[derive(Clone, Copy, Debug)]
pub(super) struct Candidate<'e> {
    pub net: NetId,
    pub without: &'e Wiring,
    pub with: &'e Wiring,
}

/// The fixed shapes of the design, with every net an owner in net order
/// as adding all wiring would make them, so that the layouts of windows
/// share its owners instead of each extending a copy.
pub(super) fn fixed_layout(db: &NetlistDB) -> Layout {
    let mut layout = Layout::fixed_from_db(db);
    for (n, net) in db.nets.iter().enumerate() {
        let _ = layout.owner(Owner::Net(NetId::new(n)), net.signal_use.is_supply());
    }
    layout
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
        let net_of = |o: OwnerIndex| match layout.owners()[o] {
            Owner::Net(n) => Some(n),
            _ => None,
        };
        let owners: Vec<[Option<NetId>; 2]> = findings
            .violations
            .iter()
            .map(|m| [net_of(m.owners.0), net_of(m.owners.1)])
            .collect();
        Judgment { findings, owners }
    }

    /// The layout of everything meeting `window`: the fixed shapes there,
    /// the wiring of every net with metal or cuts in it, which brings along
    /// via pads on layers below the routable ones, and for each candidate
    /// net its kept wiring together with its candidate edges.
    fn window_layout(&self, window: &Rect<Dbu>, candidates: &[Candidate<'_>]) -> Layout {
        let mut layout = Layout::with_owners_of(&self.fixed);
        for (layer, i) in self.fixed_tiles.near(window) {
            let shape = match layer {
                LayerRef::Routing(l) => &self.fixed.metal(l)[i as usize],
                LayerRef::Cut(c) => &self.fixed.cut(c)[i as usize],
            };
            if shape.rect.touches(window) {
                layout.add_shape(layer, shape);
            }
        }
        let candidate_nets: FxHashSet<NetId> = candidates.iter().map(|c| c.net).collect();
        let mut present: Vec<NetId> = Vec::new();
        let mut note = |owner: crate::occupancy::Owner| {
            if let crate::occupancy::Owner::Net(n) = owner {
                present.push(n);
            }
        };
        for layer in self.tech.routable_layers() {
            for owner in self.occupancy.owners_in(self.tech, layer, window) {
                note(owner);
            }
        }
        for cut in self.tech.cuts.ids() {
            let probe = crate::occupancy::Owner::Obstacle;
            for occupant in self.occupancy.cut_conflicts(cut, window, probe) {
                note(occupant.owner);
            }
        }
        present.sort_unstable();
        present.dedup();
        for net in present {
            if !candidate_nets.contains(&net) {
                layout.add_wiring(self.db, net, &self.elems[net], Some(window));
            }
        }
        for c in candidates {
            let elems: Vec<RouteElem> = self.as_committed(c, window).elems(self.tech);
            layout.add_wiring(self.db, c.net, &elems, Some(window));
        }
        layout
    }

    /// Every violation of the current wiring, as markers for repair, with
    /// how many there are of each kind; as `TritonRoute`'s checker, each tile
    /// is built from the shapes near it only when it is checked.
    pub(super) fn exact_markers(&self) -> (Vec<Marker>, Vec<(Violation, usize)>) {
        let Some(bounds) = self
            .wiring_bounds()
            .into_iter()
            .chain(self.fixed.bounds())
            .reduce(|a, b| a.union(&b))
        else {
            return (Vec::new(), Vec::new());
        };
        let report = check_tiles_lazily(
            bounds,
            self.fixed.owners().clone(),
            &self.rules,
            (self.halo * TILE_HALOS, self.halo),
            |window| self.window_layout(window, &[]),
        );
        let counts = report.counts();
        let markers: Vec<Marker> = report
            .violations
            .iter()
            .map(|m| marker_of(&report.owners, m))
            .collect();
        (markers, counts)
    }

    /// The box around every shape of every net's wiring.
    fn wiring_bounds(&self) -> Option<Rect<Dbu>> {
        self.elems
            .par_iter_enumerated()
            .filter_map(|(n, elems)| {
                elems
                    .iter()
                    .flat_map(|e| route_elem_shapes(self.db, n, e))
                    .map(|s| s.rect)
                    .reduce(|a, b| a.union(&b))
            })
            .reduce_with(|a, b| a.union(&b))
    }

    /// The route a candidate stands for near `window`, with the gaps along
    /// its tracks filled as committing it does, so that it is judged there as
    /// it will be kept; only its wiring within a halo of `window` is built.
    pub(super) fn as_committed(&self, c: &Candidate<'_>, window: &Rect<Dbu>) -> NetRoute {
        let mut route = self.routes[c.net].near(self.tech, &window.expand(self.halo));
        if !c.without.is_empty() {
            let gone: EdgeSet = c.without.edges.iter().copied().collect();
            route.edges.subtract(&gone);
            route.patches.retain(|p| !c.without.patches.contains(p));
        }
        route.edges.extend(c.with.edges.iter().copied());
        route.patches.extend_from_slice(&c.with.patches);
        route
            .edges
            .extend(gap_fills(&route, self.db, self.tech, c.net));
        tidy_patches(&mut route.patches);
        route
    }
}

/// The repair marker of a checker marker: its center, extent, layer and
/// nets.
fn marker_of(owners: &IdVec<OwnerIndex, Owner>, m: &DrcMarker) -> Marker {
    let net = |o: OwnerIndex| match owners[o] {
        Owner::Net(n) => Some(n),
        _ => None,
    };
    Marker {
        at: m.bbox.center(),
        bbox: m.bbox,
        layer: m.layer,
        nets: [net(m.owners.0), net(m.owners.1)],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::indices::CutLayerId;

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    /// A 1000 by 1000 die in tiles of 100 with metal at two corners, one
    /// across four tiles and past the die's upper right corner, and a cut.
    fn cornered() -> (Layout, ShapeTiles) {
        let mut layout = Layout::new(1, 1);
        let owner = layout.owner(Owner::Net(NetId::new(0)), false);
        let metal = LayerRef::Routing(LayerId::new(0));
        layout.add(metal, owner, rect(850, 850, 1100, 1100), Origin::Input);
        layout.add(metal, owner, rect(0, 0, 50, 50), Origin::Input);
        let cut = LayerRef::Cut(CutLayerId::new(0));
        layout.add(cut, owner, rect(960, 960, 990, 990), Origin::Input);
        let tiles = ShapeTiles::new(&layout, &rect(0, 0, 1000, 1000), Dbu(100));
        (layout, tiles)
    }

    #[test]
    fn shapes_past_the_die_edge_are_found_from_its_edge_tiles_in_order_once() {
        let (_, tiles) = cornered();

        let near = tiles.near(&rect(900, 900, 2000, 2000));

        let metal = LayerRef::Routing(LayerId::new(0));
        assert_eq!(near, [(metal, 0), (LayerRef::Cut(CutLayerId::new(0)), 0)]);
    }

    #[test]
    fn a_window_beyond_the_die_finds_the_shapes_of_the_nearest_tiles() {
        let (_, tiles) = cornered();

        let near = tiles.near(&rect(-500, -500, -400, -400));

        assert_eq!(near, [(LayerRef::Routing(LayerId::new(0)), 1)]);
    }
}
