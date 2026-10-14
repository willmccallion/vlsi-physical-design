//! Routing checks on exact geometry: shorts, spacing, opens and wiring stats.
//!
//! Only pairs involving routed wiring are checked for shorts and spacing;
//! cell masters and pre-existing special wiring are assumed clean. Shapes
//! of different owners need the spacing of the layer's parallel run table,
//! for the wider shape's width and the length they face each other. Spacing
//! also applies between two shapes of the same net that do not touch,
//! unless another shape of the net fills the gap between them, and between
//! two cuts of the same net. The metal of two vias of one net must keep the
//! spacing even where wiring fills the gap, as `OpenROAD`'s checker flags
//! it.

use super::shapes::{
    OwnedShape, Owner, Source, fixed_shapes, is_signal_net, layer_slot, route_elem_shapes,
    wiring_shapes,
};
use crate::db::design::{NetlistDB, RouteElem, TrackAxis};
use crate::db::indices::NetId;
use crate::db::library::{Direction, LayerRef, Shape};
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use rayon::prelude::*;
use std::collections::HashSet;

/// Two shapes of different nets, or a net and an obstacle, that touch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Short {
    /// Layer of the contact.
    pub layer: LayerRef,
    /// The routed shape's net.
    pub net: NetId,
    /// What it touches.
    pub other: Owner,
    /// Where the two shapes meet.
    pub at: Rect<Dbu>,
}

/// Two shapes of different owners closer than the layer's minimum spacing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SpacingViolation {
    /// Layer of the two shapes.
    pub layer: LayerRef,
    /// The routed shape's net.
    pub net: NetId,
    /// The neighbouring shape's owner.
    pub other: Owner,
    /// Squared distance between the shapes.
    pub distance_sq: i128,
    /// The space between the shapes.
    pub at: Rect<Dbu>,
}

/// A signal net whose pins are not all connected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Open {
    /// The net.
    pub net: NetId,
    /// How many separate groups its pins fall into.
    pub components: usize,
}

/// Wiring totals.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct WiringStats {
    /// Centerline wirelength per routing layer.
    pub wirelength: Vec<Dbu>,
    /// Number of via instances.
    pub vias: usize,
    /// Wires that are neither horizontal nor vertical.
    pub non_manhattan: usize,
    /// Wires running against their layer's preferred direction.
    pub wrong_way: usize,
    /// Preferred-direction wires whose centerline is not on a track.
    pub off_track: usize,
}

/// Everything the routing checker found.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RoutingReport {
    /// Shorts, one per touching shape pair.
    pub shorts: Vec<Short>,
    /// Spacing violations, one per shape pair.
    pub spacing: Vec<SpacingViolation>,
    /// Nets with disconnected pins.
    pub opens: Vec<Open>,
    /// Wiring totals.
    pub stats: WiringStats,
}

impl RoutingReport {
    /// Whether every signal net is connected and nothing is shorted.
    pub const fn is_connected_without_shorts(&self) -> bool {
        self.shorts.is_empty() && self.opens.is_empty()
    }
}

/// Checks all routed wiring of `db`.
pub fn check_routing(db: &NetlistDB) -> RoutingReport {
    let (shorts, spacing) = shorts_and_spacing(db);
    RoutingReport {
        shorts,
        spacing,
        opens: opens(db),
        stats: wiring_stats(db),
    }
}

fn min_spacing(db: &NetlistDB, layer: LayerRef) -> Dbu {
    match layer {
        LayerRef::Routing(l) => db.library.layer(l).min_spacing,
        LayerRef::Cut(c) => db.library.cut_layers[c.index()]
            .spacing
            .unwrap_or(Dbu::ZERO),
    }
}

/// The largest spacing any two shapes on `layer` can need.
fn widest_spacing(db: &NetlistDB, layer: LayerRef) -> Dbu {
    let min = min_spacing(db, layer);
    match layer {
        LayerRef::Routing(l) => db
            .library
            .layer(l)
            .spacing_table
            .as_ref()
            .map_or(min, |t| t.widest_for(Dbu(i64::MAX)).max(min)),
        LayerRef::Cut(_) => min,
    }
}

/// The spacing two shapes of different owners on `layer` need: from the
/// layer's spacing table for the wider shape's width and the length they
/// run side by side, or its minimum spacing.
fn required_spacing(db: &NetlistDB, layer: LayerRef, a: &Rect<Dbu>, b: &Rect<Dbu>) -> Dbu {
    let LayerRef::Routing(l) = layer else {
        return min_spacing(db, layer);
    };
    let Some(table) = &db.library.layer(l).spacing_table else {
        return min_spacing(db, layer);
    };
    let width = |r: &Rect<Dbu>| r.width().min(r.height());
    table.spacing(width(a).max(width(b)), parallel_run(a, b))
}

/// The length over which two disjoint rectangles face each other across a
/// gap; zero or less when they are apart in both directions.
fn parallel_run(a: &Rect<Dbu>, b: &Rect<Dbu>) -> Dbu {
    let x = a.max.x.min(b.max.x) - a.min.x.max(b.min.x);
    let y = a.max.y.min(b.max.y) - a.min.y.max(b.min.y);
    match (x > Dbu::ZERO, y > Dbu::ZERO) {
        (true, false) => x,
        (false, true) => y,
        _ => Dbu::ZERO,
    }
}

/// Squared Euclidean distance between two rectangles; zero if they touch.
fn distance_sq(a: &Rect<Dbu>, b: &Rect<Dbu>) -> i128 {
    let gap = |lo_a: Dbu, hi_a: Dbu, lo_b: Dbu, hi_b: Dbu| {
        i128::from((lo_a - hi_b).max(lo_b - hi_a).max(Dbu::ZERO).get())
    };
    let dx = gap(a.min.x, a.max.x, b.min.x, b.max.x);
    let dy = gap(a.min.y, a.max.y, b.min.y, b.max.y);
    dx * dx + dy * dy
}

/// Placement of a uniform grid of square bins; points outside it clamp to
/// the edge bins.
#[derive(Clone, Copy)]
struct Grid {
    origin: Point<Dbu>,
    bin: i64,
    nx: i64,
    ny: i64,
}

impl Grid {
    fn coord(&self, v: Dbu, origin: Dbu, n: i64) -> i64 {
        (v - origin).get().div_euclid(self.bin).clamp(0, n - 1)
    }

    fn bin_of(&self, p: Point<Dbu>) -> usize {
        let bx = self.coord(p.x, self.origin.x, self.nx);
        let by = self.coord(p.y, self.origin.y, self.ny);
        usize::try_from(by * self.nx + bx).unwrap_or(0)
    }

    fn for_each_bin(&self, r: &Rect<Dbu>, mut f: impl FnMut(usize)) {
        let x0 = self.coord(r.min.x, self.origin.x, self.nx);
        let x1 = self.coord(r.max.x, self.origin.x, self.nx);
        let y0 = self.coord(r.min.y, self.origin.y, self.ny);
        let y1 = self.coord(r.max.y, self.origin.y, self.ny);
        for by in y0..=y1 {
            for bx in x0..=x1 {
                f(usize::try_from(by * self.nx + bx).unwrap_or(0));
            }
        }
    }
}

/// The shapes of one layer, bucketed by bin in compressed-row form.
struct BinIndex {
    grid: Grid,
    start: Vec<u32>,
    items: Vec<u32>,
}

impl BinIndex {
    fn new(grid: Grid, members: &[(u32, Rect<Dbu>)]) -> Self {
        let bins = usize::try_from(grid.nx * grid.ny).unwrap_or(0);
        let mut start = vec![0u32; bins + 1];
        for (_, r) in members {
            grid.for_each_bin(r, |b| start[b + 1] += 1);
        }
        for b in 0..bins {
            start[b + 1] += start[b];
        }
        let mut cursor = start.clone();
        let mut items = vec![0u32; start[bins] as usize];
        for &(id, r) in members {
            grid.for_each_bin(&r, |b| {
                items[cursor[b] as usize] = id;
                cursor[b] += 1;
            });
        }
        Self { grid, start, items }
    }

    fn members(&self, bin: usize) -> &[u32] {
        &self.items[self.start[bin] as usize..self.start[bin + 1] as usize]
    }
}

/// Bin edge for a layer: eight track pitches, but coarse enough that the
/// region has at most about a million bins.
fn bin_size(db: &NetlistDB, slot: usize, bounds: &Rect<Dbu>) -> i64 {
    let layers = db.library.routing_layers.len();
    let pitch_layer = if slot < layers { slot } else { slot - layers };
    let pitch = db.library.routing_layers[pitch_layer].track_pitch().get();
    let span = bounds.width().max(bounds.height()).get();
    (8 * pitch).max(span / 1000 + 1).max(1)
}

fn shorts_and_spacing(db: &NetlistDB) -> (Vec<Short>, Vec<SpacingViolation>) {
    let mut shapes = fixed_shapes(db);
    let first_wiring = shapes.len();
    shapes.extend(wiring_shapes(db));
    if first_wiring == shapes.len() {
        return (Vec::new(), Vec::new());
    }

    let bounds = shapes
        .iter()
        .map(|s| s.shape.rect)
        .fold(db.die_area, |a, b| a.union(&b));
    let slots = db.library.routing_layers.len() + db.library.cut_layers.len();
    let mut per_slot: Vec<Vec<(u32, Rect<Dbu>)>> = vec![Vec::new(); slots];
    for (i, s) in shapes.iter().enumerate() {
        let id = u32::try_from(i).unwrap_or(u32::MAX);
        per_slot[layer_slot(db, s.shape.layer)].push((id, s.shape.rect));
    }
    let indexes: Vec<BinIndex> = per_slot
        .par_iter()
        .enumerate()
        .map(|(slot, members)| {
            let bin = bin_size(db, slot, &bounds);
            let grid = Grid {
                origin: bounds.min,
                bin,
                nx: bounds.width().get() / bin + 1,
                ny: bounds.height().get() / bin + 1,
            };
            BinIndex::new(grid, members)
        })
        .collect();

    (first_wiring..shapes.len())
        .into_par_iter()
        .fold(
            || (Vec::new(), Vec::new()),
            |(mut shorts, mut spacing), i| {
                check_wiring_shape(db, &shapes, &indexes, i, &mut shorts, &mut spacing);
                (shorts, spacing)
            },
        )
        .reduce(
            || (Vec::new(), Vec::new()),
            |(mut a, mut b), (c, d)| {
                a.extend(c);
                b.extend(d);
                (a, b)
            },
        )
}

fn check_wiring_shape(
    db: &NetlistDB,
    shapes: &[OwnedShape],
    indexes: &[BinIndex],
    this: usize,
    shorts: &mut Vec<Short>,
    spacing: &mut Vec<SpacingViolation>,
) {
    let wiring = shapes[this];
    let Owner::Net(net) = wiring.owner else {
        return;
    };
    let layer = wiring.shape.layer;
    let index = &indexes[layer_slot(db, layer)];
    let sp = min_spacing(db, layer);
    let sp_sq = i128::from(sp.get()) * i128::from(sp.get());
    let reach = wiring.shape.rect.expand(widest_spacing(db, layer));
    index.grid.for_each_bin(&reach, |bin| {
        for &member in index.members(bin) {
            let near = member as usize;
            let other = &shapes[near];
            let same_net = other.owner == wiring.owner;
            let cut = matches!(layer, LayerRef::Cut(_));
            if near == this
                || (other.is_wiring() && near < this)
                || (same_net && cut && !other.is_wiring())
            {
                continue;
            }
            let r = other.shape.rect;
            if !reach.touches(&r) {
                continue;
            }
            let owner_point = Point::new(reach.min.x.max(r.min.x), reach.min.y.max(r.min.y));
            if index.grid.bin_of(owner_point) != bin {
                continue;
            }
            if same_net {
                let d = distance_sq(&wiring.shape.rect, &r);
                let both_vias = wiring.source == Source::Via && other.source == Source::Via;
                let bridged = || {
                    !cut && !both_vias
                        && bridged(shapes, index, wiring.owner, &gap(&wiring.shape.rect, &r))
                };
                if d > 0 && d < sp_sq && !bridged() {
                    spacing.push(SpacingViolation {
                        layer,
                        net,
                        other: other.owner,
                        distance_sq: d,
                        at: gap(&wiring.shape.rect, &r),
                    });
                }
                continue;
            }
            if let Some(at) = wiring.shape.rect.intersection(&r) {
                shorts.push(Short {
                    layer,
                    net,
                    other: other.owner,
                    at,
                });
            } else {
                let d = distance_sq(&wiring.shape.rect, &r);
                let need = i128::from(required_spacing(db, layer, &wiring.shape.rect, &r).get());
                if d < need * need {
                    spacing.push(SpacingViolation {
                        layer,
                        net,
                        other: other.owner,
                        distance_sq: d,
                        at: gap(&wiring.shape.rect, &r),
                    });
                }
            }
        }
    });
}

/// The rectangle between two disjoint rectangles: their overlap along an
/// axis where they overlap, the space between them along one where they
/// do not.
fn gap(a: &Rect<Dbu>, b: &Rect<Dbu>) -> Rect<Dbu> {
    let span = |lo_a: Dbu, hi_a: Dbu, lo_b: Dbu, hi_b: Dbu| {
        if hi_a < lo_b {
            (hi_a, lo_b)
        } else if hi_b < lo_a {
            (hi_b, lo_a)
        } else {
            (lo_a.max(lo_b), hi_a.min(hi_b))
        }
    };
    let (x0, x1) = span(a.min.x, a.max.x, b.min.x, b.max.x);
    let (y0, y1) = span(a.min.y, a.max.y, b.min.y, b.max.y);
    Rect::new(Point::new(x0, y0), Point::new(x1, y1))
}

/// Whether the shapes of `owner` in `index` together cover all of `gap`,
/// so that the two shapes on either side of it are one piece of metal.
fn bridged(shapes: &[OwnedShape], index: &BinIndex, owner: Owner, gap: &Rect<Dbu>) -> bool {
    let mut pieces: Vec<Rect<Dbu>> = Vec::new();
    index.grid.for_each_bin(gap, |bin| {
        for &m in index.members(bin) {
            let s = &shapes[m as usize];
            if s.owner == owner
                && let Some(part) = s.shape.rect.intersection(gap)
            {
                pieces.push(part);
            }
        }
    });
    covers(gap, &pieces)
}

/// Whether the union of `pieces`, each inside `area`, is all of `area`.
fn covers(area: &Rect<Dbu>, pieces: &[Rect<Dbu>]) -> bool {
    let mut xs: Vec<Dbu> = pieces
        .iter()
        .flat_map(|r| [r.min.x, r.max.x])
        .chain([area.min.x, area.max.x])
        .collect();
    xs.sort_unstable();
    xs.dedup();
    let column_covered = |x0: Dbu, x1: Dbu| {
        let mut spans: Vec<(Dbu, Dbu)> = pieces
            .iter()
            .filter(|r| r.min.x <= x0 && x1 <= r.max.x)
            .map(|r| (r.min.y, r.max.y))
            .collect();
        spans.sort_unstable();
        let mut reached = area.min.y;
        for (lo, hi) in spans {
            if lo > reached {
                return false;
            }
            reached = reached.max(hi);
        }
        reached >= area.max.y
    };
    xs.windows(2).all(|w| column_covered(w[0], w[1]))
}

/// Whether metal or cut shapes on these layers can conduct into each other.
fn layers_connect(a: LayerRef, b: LayerRef) -> bool {
    match (a, b) {
        (LayerRef::Routing(x), LayerRef::Routing(y)) => x == y,
        (LayerRef::Cut(x), LayerRef::Cut(y)) => x == y,
        (LayerRef::Cut(c), LayerRef::Routing(m)) | (LayerRef::Routing(m), LayerRef::Cut(c)) => {
            m == c.below() || m == c.above()
        }
    }
}

const fn find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

const fn union(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    if ra != rb {
        parent[ra] = rb;
    }
}

/// How many separately connected groups the pins of `net` form, counting
/// its wiring and the shapes of its pins.
pub fn net_components(db: &NetlistDB, net: NetId) -> usize {
    let n = &db.nets[net.index()];
    let mut shapes: Vec<(Shape, Option<usize>)> = n
        .routing
        .iter()
        .flat_map(|e| route_elem_shapes(db, e))
        .map(|s| (s, None))
        .collect();
    let mut shapeless_pins = 0;
    for (k, &pin) in n.pins.iter().enumerate() {
        let pin_shapes = db.pin_shapes(pin);
        if pin_shapes.is_empty() {
            shapeless_pins += 1;
        }
        shapes.extend(pin_shapes.into_iter().map(|s| (s, Some(k))));
    }

    let mut parent: Vec<usize> = (0..shapes.len()).collect();
    let mut first_shape_of_pin: Vec<Option<usize>> = vec![None; n.pins.len()];
    for (i, (_, pin)) in shapes.iter().enumerate() {
        if let Some(k) = *pin {
            match first_shape_of_pin[k] {
                Some(first) => union(&mut parent, first, i),
                None => first_shape_of_pin[k] = Some(i),
            }
        }
    }

    let mut order: Vec<usize> = (0..shapes.len()).collect();
    order.sort_by_key(|&i| shapes[i].0.rect.min.x);
    for (pos, &a) in order.iter().enumerate() {
        let ra = shapes[a].0;
        for &b in &order[pos + 1..] {
            let rb = shapes[b].0;
            if rb.rect.min.x > ra.rect.max.x {
                break;
            }
            if layers_connect(ra.layer, rb.layer) && ra.rect.touches(&rb.rect) {
                union(&mut parent, a, b);
            }
        }
    }

    let roots: HashSet<usize> = first_shape_of_pin
        .iter()
        .flatten()
        .map(|&i| find(&mut parent, i))
        .collect();
    roots.len() + shapeless_pins
}

fn opens(db: &NetlistDB) -> Vec<Open> {
    (0..db.nets.len())
        .into_par_iter()
        .map(NetId::new)
        .filter(|&net| is_signal_net(db, net))
        .filter_map(|net| {
            let components = net_components(db, net);
            (components > 1).then_some(Open { net, components })
        })
        .collect()
}

fn wiring_stats(db: &NetlistDB) -> WiringStats {
    let layers = &db.library.routing_layers;
    let mut tracks: HashSet<(usize, Dbu)> = HashSet::new();
    let mut has_tracks = vec![false; layers.len()];
    for t in &db.tracks {
        for &l in &t.layers {
            let preferred = match layers[l.index()].direction {
                Direction::Horizontal => TrackAxis::Y,
                Direction::Vertical => TrackAxis::X,
            };
            if t.axis == preferred {
                has_tracks[l.index()] = true;
                tracks.extend(t.coords().map(|c| (l.index(), c)));
            }
        }
    }

    let mut stats = WiringStats {
        wirelength: vec![Dbu::ZERO; layers.len()],
        ..WiringStats::default()
    };
    for net in &db.nets {
        for elem in &net.routing {
            match *elem {
                RouteElem::Via { .. } => stats.vias += 1,
                RouteElem::Wire { layer, from, to } => {
                    let l = layer.index();
                    let (dx, dy) = ((to.x - from.x).abs(), (to.y - from.y).abs());
                    stats.wirelength[l] += dx + dy;
                    if dx != Dbu::ZERO && dy != Dbu::ZERO {
                        stats.non_manhattan += 1;
                        continue;
                    }
                    let (along, across) = match layers[l].direction {
                        Direction::Horizontal => (dx, from.y),
                        Direction::Vertical => (dy, from.x),
                    };
                    let length = dx + dy;
                    if length > Dbu::ZERO && along == Dbu::ZERO {
                        stats.wrong_way += 1;
                    } else if has_tracks[l] && !tracks.contains(&(l, across)) {
                        stats.off_track += 1;
                    }
                }
            }
        }
    }
    stats
}
