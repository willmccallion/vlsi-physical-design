//! Each owner's merged metal on every layer, and indexes to find it.
//!
//! On a metal layer an owner's shapes merge into polygons, described by
//! their maximal rectangles and boundary edges. A rectangle or edge is
//! fixed when the owner's fixed shapes alone have it too. Cuts are kept one
//! by one; a cut is fixed when a fixed cut of its owner has its rectangle.

use super::layout::{Layout, LayoutShape, Origin, OwnerIndex};
use crate::geom::dbu::{Dbu, DbuArea};
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use crate::geom::region::{Edge, Region};
use rayon::prelude::*;
use rstar::{AABB, RTree, RTreeObject};
use std::collections::{HashMap, HashSet};

/// A maximal rectangle of one owner's metal on a layer, or one cut.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct GcRect {
    pub rect: Rect<Dbu>,
    pub owner: OwnerIndex,
    pub fixed: bool,
}

impl GcRect {
    /// The smaller of the rectangle's sides.
    pub fn width(&self) -> Dbu {
        self.rect.width().min(self.rect.height())
    }
}

/// A boundary edge of one owner's metal on a layer, with its place in the
/// boundary: the piece it bounds and the edges before and after it, as
/// positions in the layer's edge list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct GcEdge {
    pub edge: Edge,
    pub owner: OwnerIndex,
    pub fixed: bool,
    pub piece: u32,
    pub prev: u32,
    pub next: u32,
}

/// One connected piece of an owner's metal on a layer that holds routed
/// metal. Pieces of fixed metal alone are not kept: every `FlexGC` shape
/// rule exempts them.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct GcPolygon {
    pub owner: OwnerIndex,
    pub region: Region,
    pub bbox: Rect<Dbu>,
    pub area: DbuArea,
    pub has_fixed_edge: bool,
}

/// The maximal rectangles, edges and pieces of one owner's metal on a
/// layer.
#[derive(Debug, Default)]
struct OwnerGeometry {
    rects: Vec<GcRect>,
    edges: Vec<GcEdge>,
    polygons: Vec<GcPolygon>,
    /// How many connected pieces the owner's metal falls into.
    pieces: u32,
}

/// A box of the index and the position of what it bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Entry {
    lo: [i64; 2],
    hi: [i64; 2],
    index: usize,
}

impl RTreeObject for Entry {
    type Envelope = AABB<[i64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.lo, self.hi)
    }
}

/// Positions of boxes, found by area.
#[derive(Debug)]
struct Index {
    tree: RTree<Entry>,
}

impl Index {
    fn new(boxes: impl Iterator<Item = Rect<Dbu>>) -> Self {
        let entries = boxes
            .enumerate()
            .map(|(i, r)| Entry {
                lo: [r.min.x.get(), r.min.y.get()],
                hi: [r.max.x.get(), r.max.y.get()],
                index: i,
            })
            .collect();
        Self {
            tree: RTree::bulk_load(entries),
        }
    }

    /// Positions of the boxes that meet `area`, edges included.
    fn near(&self, area: &Rect<Dbu>) -> impl Iterator<Item = usize> + '_ {
        let envelope = AABB::from_corners(
            [area.min.x.get(), area.min.y.get()],
            [area.max.x.get(), area.max.y.get()],
        );
        self.tree
            .locate_in_envelope_intersecting(envelope)
            .map(|e| e.index)
    }
}

/// The merged metal of every owner on one routing layer.
#[derive(Debug)]
pub(super) struct Metal<'a> {
    pub rects: Vec<GcRect>,
    pub edges: Vec<GcEdge>,
    pub polygons: Vec<GcPolygon>,
    shapes: &'a [LayoutShape],
    rect_index: Index,
    edge_index: Index,
    shape_index: Index,
}

impl<'a> Metal<'a> {
    fn new(shapes: &'a [LayoutShape]) -> Self {
        let mut order: Vec<usize> = (0..shapes.len()).collect();
        order.sort_by_key(|&i| shapes[i].owner);
        let groups: Vec<&[usize]> = order
            .chunk_by(|&a, &b| shapes[a].owner == shapes[b].owner)
            .collect();
        let owners: Vec<OwnerGeometry> = groups
            .par_iter()
            .map(|group| owner_geometry(shapes, group))
            .collect();
        let mut rects = Vec::new();
        let mut edges: Vec<GcEdge> = Vec::new();
        let mut polygons = Vec::new();
        let mut pieces = 0;
        for geometry in owners {
            let edge_base = u32::try_from(edges.len()).unwrap_or(u32::MAX);
            rects.extend(geometry.rects);
            edges.extend(geometry.edges.into_iter().map(|e| GcEdge {
                piece: e.piece + pieces,
                prev: e.prev + edge_base,
                next: e.next + edge_base,
                ..e
            }));
            polygons.extend(geometry.polygons);
            pieces += geometry.pieces;
        }
        Self {
            rect_index: Index::new(rects.iter().map(|r| r.rect)),
            edge_index: Index::new(
                edges
                    .iter()
                    .map(|e| Rect::from_corners(e.edge.from, e.edge.to)),
            ),
            shape_index: Index::new(shapes.iter().map(|s| s.rect)),
            rects,
            edges,
            polygons,
            shapes,
        }
    }

    /// Positions of the maximal rectangles meeting `area`.
    pub fn rects_near(&self, area: &Rect<Dbu>) -> impl Iterator<Item = usize> + '_ {
        self.rect_index.near(area)
    }

    /// The edges meeting `area`.
    pub fn edges_near(&self, area: &Rect<Dbu>) -> impl Iterator<Item = &GcEdge> + '_ {
        self.edge_indices_near(area).map(|i| &self.edges[i])
    }

    /// Positions of the edges meeting `area`.
    pub fn edge_indices_near(&self, area: &Rect<Dbu>) -> impl Iterator<Item = usize> + '_ {
        self.edge_index.near(area)
    }

    /// Whether any owner's metal lies strictly inside `area`.
    pub fn has_metal_in(&self, area: &Rect<Dbu>) -> bool {
        self.rects_near(area)
            .any(|i| self.rects[i].rect.overlaps(area))
    }

    /// The part of `window` the fixed metal of `owner` covers.
    pub fn fixed_within(&self, owner: OwnerIndex, window: &Rect<Dbu>) -> Region {
        Region::from_rects(
            self.shape_index
                .near(window)
                .map(|i| self.shapes[i])
                .filter(|s| s.owner == owner && s.is_fixed())
                .filter_map(|s| s.rect.intersection(window)),
        )
    }

    /// Whether routed metal of `owner` shares area with `area`.
    pub fn has_route_in(&self, owner: OwnerIndex, area: &Rect<Dbu>) -> bool {
        self.shape_index
            .near(area)
            .map(|i| self.shapes[i])
            .any(|s| s.owner == owner && !s.is_fixed() && s.rect.overlaps(area))
    }

    /// Whether metal that placement put there, of either of `owners`, meets
    /// `area`.
    pub fn placed_metal_meets(&self, owners: (OwnerIndex, OwnerIndex), area: &Rect<Dbu>) -> bool {
        placed_shape_meets(self.shapes, self.shape_index.near(area), owners)
    }
}

fn placed_shape_meets(
    shapes: &[LayoutShape],
    near: impl Iterator<Item = usize>,
    owners: (OwnerIndex, OwnerIndex),
) -> bool {
    near.map(|i| shapes[i])
        .any(|s| s.origin == Origin::Placement && (s.owner == owners.0 || s.owner == owners.1))
}

/// The geometry of the metal made of `group`, shapes of one owner.
fn owner_geometry(shapes: &[LayoutShape], group: &[usize]) -> OwnerGeometry {
    let owner = shapes[group[0]].owner;
    let all = Region::from_rects(group.iter().map(|&i| shapes[i].rect));
    let fixed = Region::from_rects(
        group
            .iter()
            .map(|&i| shapes[i])
            .filter(LayoutShape::is_fixed)
            .map(|s| s.rect),
    );
    let routed: Vec<Rect<Dbu>> = group
        .iter()
        .map(|&i| shapes[i])
        .filter(|s| !s.is_fixed())
        .map(|s| s.rect)
        .collect();
    let fixed_rects: HashSet<Rect<Dbu>> = fixed.max_rectangles().into_iter().collect();
    let fixed_edges: HashSet<Edge> = fixed.edges().into_iter().collect();
    let mut geometry = OwnerGeometry {
        rects: all
            .max_rectangles()
            .into_iter()
            .map(|rect| GcRect {
                rect,
                owner,
                fixed: fixed_rects.contains(&rect),
            })
            .collect(),
        ..OwnerGeometry::default()
    };
    for piece in all.components() {
        let first = geometry.edges.len();
        let boundary = piece.edges();
        let starting_at: HashMap<Point<Dbu>, usize> = boundary
            .iter()
            .enumerate()
            .map(|(k, e)| (e.from, first + k))
            .collect();
        let ending_at: HashMap<Point<Dbu>, usize> = boundary
            .iter()
            .enumerate()
            .map(|(k, e)| (e.to, first + k))
            .collect();
        let position = |i: usize| u32::try_from(i).unwrap_or(u32::MAX);
        geometry
            .edges
            .extend(boundary.iter().enumerate().map(|(k, &edge)| GcEdge {
                edge,
                owner,
                fixed: fixed_edges.contains(&edge),
                piece: geometry.pieces,
                prev: position(*ending_at.get(&edge.from).unwrap_or(&(first + k))),
                next: position(*starting_at.get(&edge.to).unwrap_or(&(first + k))),
            }));
        geometry.pieces += 1;
        let Some(bbox) = piece.bounding_box() else {
            continue;
        };
        let holds_route = routed
            .iter()
            .any(|r| r.overlaps(&bbox) && piece.intersects_rect(r));
        if holds_route {
            geometry.polygons.push(GcPolygon {
                owner,
                bbox,
                area: piece.area(),
                has_fixed_edge: geometry.edges[first..].iter().any(|e| e.fixed),
                region: piece,
            });
        }
    }
    geometry
}

/// The cuts of every owner on one cut layer.
#[derive(Debug)]
pub(super) struct Cut<'a> {
    pub rects: Vec<GcRect>,
    shapes: &'a [LayoutShape],
    index: Index,
}

impl<'a> Cut<'a> {
    fn new(shapes: &'a [LayoutShape]) -> Self {
        let fixed_cuts: HashSet<(OwnerIndex, Rect<Dbu>)> = shapes
            .iter()
            .filter(|s| s.is_fixed())
            .map(|s| (s.owner, s.rect))
            .collect();
        let rects: Vec<GcRect> = shapes
            .iter()
            .map(|s| GcRect {
                rect: s.rect,
                owner: s.owner,
                fixed: fixed_cuts.contains(&(s.owner, s.rect)),
            })
            .collect();
        Self {
            index: Index::new(rects.iter().map(|r| r.rect)),
            rects,
            shapes,
        }
    }

    /// Positions of the cuts meeting `area`.
    pub fn near(&self, area: &Rect<Dbu>) -> impl Iterator<Item = usize> + '_ {
        self.index.near(area)
    }

    /// Whether a cut that placement put there, of either of `owners`, meets
    /// `area`.
    pub fn placed_cut_meets(&self, owners: (OwnerIndex, OwnerIndex), area: &Rect<Dbu>) -> bool {
        placed_shape_meets(self.shapes, self.index.near(area), owners)
    }
}

/// The geometry of every layer of a layout.
#[derive(Debug)]
pub(super) struct Model<'a> {
    pub layout: &'a Layout,
    pub metal: Vec<Metal<'a>>,
    pub cut: Vec<Cut<'a>>,
}

impl<'a> Model<'a> {
    pub fn new(layout: &'a Layout) -> Self {
        Self {
            metal: (0..layout.metal_layers())
                .map(|l| Metal::new(layout.metal(l)))
                .collect(),
            cut: (0..layout.cut_layers())
                .map(|c| Cut::new(layout.cut(c)))
                .collect(),
            layout,
        }
    }
}

/// The overlap of two intervals, or the gap between them when apart.
pub(super) fn between(a: (Dbu, Dbu), b: (Dbu, Dbu)) -> (Dbu, Dbu) {
    if a.1 < b.0 {
        (a.1, b.0)
    } else if b.1 < a.0 {
        (b.1, a.0)
    } else {
        (a.0.max(b.0), a.1.min(b.1))
    }
}

/// How far apart two intervals are; zero when they meet.
pub(super) fn apart(a: (Dbu, Dbu), b: (Dbu, Dbu)) -> Dbu {
    (b.0 - a.1).max(a.0 - b.1).max(Dbu::ZERO)
}

/// Where two rectangles relate: the rectangle between them (their overlap
/// along an axis where they overlap, the gap where they are apart), the
/// gaps along X and Y, and the run along X and Y, negative where apart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Facing {
    pub marker: Rect<Dbu>,
    pub dist_x: Dbu,
    pub dist_y: Dbu,
    pub prl_x: Dbu,
    pub prl_y: Dbu,
}

impl Facing {
    pub fn new(a: &Rect<Dbu>, b: &Rect<Dbu>) -> Self {
        let (x0, x1) = between((a.min.x, a.max.x), (b.min.x, b.max.x));
        let (y0, y1) = between((a.min.y, a.max.y), (b.min.y, b.max.y));
        let dist_x = apart((a.min.x, a.max.x), (b.min.x, b.max.x));
        let dist_y = apart((a.min.y, a.max.y), (b.min.y, b.max.y));
        let signed = |run: Dbu, dist: Dbu| if dist > Dbu::ZERO { -run } else { run };
        Self {
            marker: Rect::new(Point::new(x0, y0), Point::new(x1, y1)),
            dist_x,
            dist_y,
            prl_x: signed(x1 - x0, dist_x),
            prl_y: signed(y1 - y0, dist_y),
        }
    }

    /// The longer run.
    pub fn prl(&self) -> Dbu {
        self.prl_x.max(self.prl_y)
    }
}
