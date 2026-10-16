//! Rectilinear regions: finite unions of axis-aligned rectangles.
//!
//! A region is kept as vertical slabs: between consecutive X breakpoints,
//! one sorted list of disjoint Y spans. Spans that touch are merged and
//! neighbouring slabs that are equal are joined, so two regions covering the
//! same area have the same representation. Rectangles without area add
//! nothing.

use super::dbu::{Dbu, DbuArea};
use super::point::Point;
use super::rect::Rect;
use std::collections::HashMap;

/// Disjoint, non-touching Y intervals in increasing order.
type Spans = Vec<(Dbu, Dbu)>;

/// One vertical slab of a [`Region`].
#[derive(Clone, Debug, PartialEq, Eq)]
struct Slab {
    x0: Dbu,
    x1: Dbu,
    spans: Spans,
}

/// A finite union of axis-aligned rectangles.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Region {
    slabs: Vec<Slab>,
}

/// Which way a boundary edge runs. Edges run with the region on their
/// left: bottom edges east, top edges west, left edges south and right
/// edges north.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Heading {
    /// Toward increasing X.
    East,
    /// Toward decreasing X.
    West,
    /// Toward increasing Y.
    North,
    /// Toward decreasing Y.
    South,
}

/// A maximal straight piece of a region's boundary, run with the region on
/// its left.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Edge {
    /// Where the edge starts.
    pub from: Point<Dbu>,
    /// Where the edge ends.
    pub to: Point<Dbu>,
}

impl Edge {
    /// The way the edge runs.
    pub fn heading(&self) -> Heading {
        if self.to.x > self.from.x {
            Heading::East
        } else if self.to.x < self.from.x {
            Heading::West
        } else if self.to.y > self.from.y {
            Heading::North
        } else {
            Heading::South
        }
    }

    /// The edge's length.
    pub fn length(&self) -> Dbu {
        (self.to.x - self.from.x).abs() + (self.to.y - self.from.y).abs()
    }
}

impl Region {
    /// The empty region.
    pub const fn new() -> Self {
        Self { slabs: Vec::new() }
    }

    /// The union of `rects`.
    pub fn from_rects(rects: impl IntoIterator<Item = Rect<Dbu>>) -> Self {
        let mut rects: Vec<Rect<Dbu>> = rects
            .into_iter()
            .filter(|r| r.width() > Dbu::ZERO && r.height() > Dbu::ZERO)
            .collect();
        if rects.is_empty() {
            return Self::new();
        }
        rects.sort_unstable_by_key(|r| r.min.x);
        let mut xs: Vec<Dbu> = rects.iter().flat_map(|r| [r.min.x, r.max.x]).collect();
        xs.sort_unstable();
        xs.dedup();
        let mut region = Self::new();
        let mut active: Vec<Rect<Dbu>> = Vec::new();
        let mut next = 0;
        for w in xs.windows(2) {
            let (x0, x1) = (w[0], w[1]);
            active.retain(|r| r.max.x > x0);
            while next < rects.len() && rects[next].min.x <= x0 {
                active.push(rects[next]);
                next += 1;
            }
            let spans = merged(active.iter().map(|r| (r.min.y, r.max.y)).collect());
            region.push_slab(x0, x1, spans);
        }
        region
    }

    /// Appends a slab to the right of every slab so far, joining it to the
    /// last one when they abut with equal spans.
    fn push_slab(&mut self, x0: Dbu, x1: Dbu, spans: Spans) {
        if spans.is_empty() || x1 <= x0 {
            return;
        }
        if let Some(last) = self.slabs.last_mut()
            && last.x1 == x0
            && last.spans == spans
        {
            last.x1 = x1;
            return;
        }
        self.slabs.push(Slab { x0, x1, spans });
    }

    /// Whether the region has no area.
    pub const fn is_empty(&self) -> bool {
        self.slabs.is_empty()
    }

    /// The region's area.
    pub fn area(&self) -> DbuArea {
        DbuArea(
            self.slabs
                .iter()
                .map(|s| {
                    let height: i64 = s.spans.iter().map(|&(lo, hi)| (hi - lo).get()).sum();
                    (s.x1 - s.x0).get() * height
                })
                .sum(),
        )
    }

    /// The smallest rectangle containing the region.
    pub fn bounding_box(&self) -> Option<Rect<Dbu>> {
        let first = self.slabs.first()?;
        let last = self.slabs.last()?;
        let lo = self
            .slabs
            .iter()
            .filter_map(|s| s.spans.first())
            .map(|s| s.0)
            .min()?;
        let hi = self
            .slabs
            .iter()
            .filter_map(|s| s.spans.last())
            .map(|s| s.1)
            .max()?;
        Some(Rect::new(Point::new(first.x0, lo), Point::new(last.x1, hi)))
    }

    /// The rectangles the region is made of, one per span of each slab.
    pub fn rectangles(&self) -> impl Iterator<Item = Rect<Dbu>> + '_ {
        self.slabs.iter().flat_map(|s| {
            s.spans
                .iter()
                .map(move |&(lo, hi)| Rect::new(Point::new(s.x0, lo), Point::new(s.x1, hi)))
        })
    }

    /// Points in either region.
    #[must_use]
    pub fn union(&self, other: &Self) -> Self {
        self.combine(other, |a, b| a || b)
    }

    /// Points in both regions.
    #[must_use]
    pub fn intersection(&self, other: &Self) -> Self {
        self.combine(other, |a, b| a && b)
    }

    /// Points in this region and not in `other`.
    #[must_use]
    pub fn difference(&self, other: &Self) -> Self {
        self.combine(other, |a, b| a && !b)
    }

    /// The region `keep` selects slab by slab from this region and `other`.
    fn combine(&self, other: &Self, keep: fn(bool, bool) -> bool) -> Self {
        let mut xs: Vec<Dbu> = self
            .slabs
            .iter()
            .chain(&other.slabs)
            .flat_map(|s| [s.x0, s.x1])
            .collect();
        xs.sort_unstable();
        xs.dedup();
        let mut out = Self::new();
        let (mut i, mut j) = (0, 0);
        for w in xs.windows(2) {
            let (x0, x1) = (w[0], w[1]);
            let a = covering_spans(&self.slabs, &mut i, x0, x1);
            let b = covering_spans(&other.slabs, &mut j, x0, x1);
            out.push_slab(x0, x1, combine_spans(a, b, keep));
        }
        out
    }

    /// Whether the region and `rect` share area.
    pub fn intersects_rect(&self, rect: &Rect<Dbu>) -> bool {
        self.slabs
            .iter()
            .filter(|s| s.x0 < rect.max.x && rect.min.x < s.x1)
            .any(|s| {
                s.spans
                    .iter()
                    .any(|&(lo, hi)| lo < rect.max.y && rect.min.y < hi)
            })
    }

    /// Whether the region contains all of `rect`, which must have area.
    pub fn covers_rect(&self, rect: &Rect<Dbu>) -> bool {
        if rect.width() <= Dbu::ZERO || rect.height() <= Dbu::ZERO {
            return false;
        }
        let mut reached = rect.min.x;
        for s in self
            .slabs
            .iter()
            .filter(|s| s.x1 > rect.min.x && s.x0 < rect.max.x)
        {
            if s.x0 > reached {
                return false;
            }
            if !spans_cover(&s.spans, rect.min.y, rect.max.y) {
                return false;
            }
            reached = s.x1;
        }
        reached >= rect.max.x
    }

    /// The region's connected pieces. Pieces meeting only at a corner are
    /// separate.
    pub fn components(&self) -> Vec<Self> {
        let offsets: Vec<usize> = self
            .slabs
            .iter()
            .scan(0, |next, s| {
                let start = *next;
                *next += s.spans.len();
                Some(start)
            })
            .collect();
        let total = offsets
            .last()
            .map_or(0, |&o| o + self.slabs.last().map_or(0, |s| s.spans.len()));
        let mut parent: Vec<usize> = (0..total).collect();
        for k in 1..self.slabs.len() {
            let (left, right) = (&self.slabs[k - 1], &self.slabs[k]);
            if left.x1 != right.x0 {
                continue;
            }
            let (mut a, mut b) = (0, 0);
            while a < left.spans.len() && b < right.spans.len() {
                let (la, ha) = left.spans[a];
                let (lb, hb) = right.spans[b];
                if la < hb && lb < ha {
                    union(&mut parent, offsets[k - 1] + a, offsets[k] + b);
                }
                if ha < hb {
                    a += 1;
                } else {
                    b += 1;
                }
            }
        }
        let mut index_of_root: HashMap<usize, usize> = HashMap::new();
        let mut pieces: Vec<Self> = Vec::new();
        for (k, s) in self.slabs.iter().enumerate() {
            let mut by_piece: Vec<(usize, Spans)> = Vec::new();
            for (n, &span) in s.spans.iter().enumerate() {
                let root = find(&mut parent, offsets[k] + n);
                let piece = *index_of_root.entry(root).or_insert_with(|| {
                    pieces.push(Self::new());
                    pieces.len() - 1
                });
                match by_piece.iter_mut().find(|(p, _)| *p == piece) {
                    Some((_, spans)) => spans.push(span),
                    None => by_piece.push((piece, vec![span])),
                }
            }
            for (piece, spans) in by_piece {
                pieces[piece].push_slab(s.x0, s.x1, spans);
            }
        }
        pieces
    }

    /// The boundary of the region as maximal straight edges, each run with
    /// the region on its left.
    pub fn edges(&self) -> Vec<Edge> {
        let mut edges = Vec::new();
        self.vertical_edges(&mut edges);
        self.horizontal_edges(&mut edges);
        edges
    }

    fn vertical_edges(&self, edges: &mut Vec<Edge>) {
        let empty: Spans = Vec::new();
        let mut xs: Vec<Dbu> = self.slabs.iter().flat_map(|s| [s.x0, s.x1]).collect();
        xs.dedup();
        let mut k = 0;
        for x in xs {
            while k < self.slabs.len() && self.slabs[k].x1 < x {
                k += 1;
            }
            let left = self.slabs[k..]
                .iter()
                .take(2)
                .find(|s| s.x1 == x)
                .map_or(&empty, |s| &s.spans);
            let right = self.slabs[k..]
                .iter()
                .take(2)
                .find(|s| s.x0 == x)
                .map_or(&empty, |s| &s.spans);
            for (lo, hi) in combine_spans(right, left, |r, l| r && !l) {
                edges.push(Edge {
                    from: Point::new(x, hi),
                    to: Point::new(x, lo),
                });
            }
            for (lo, hi) in combine_spans(left, right, |l, r| l && !r) {
                edges.push(Edge {
                    from: Point::new(x, lo),
                    to: Point::new(x, hi),
                });
            }
        }
    }

    fn horizontal_edges(&self, edges: &mut Vec<Edge>) {
        let mut open_bottom: HashMap<Dbu, usize> = HashMap::new();
        let mut open_top: HashMap<Dbu, usize> = HashMap::new();
        let mut previous_end: Option<Dbu> = None;
        for s in &self.slabs {
            if previous_end != Some(s.x0) {
                open_bottom.clear();
                open_top.clear();
            }
            let mut bottom = HashMap::new();
            let mut top = HashMap::new();
            for &(lo, hi) in &s.spans {
                let b = if let Some(&e) = open_bottom.get(&lo) {
                    edges[e].to.x = s.x1;
                    e
                } else {
                    edges.push(Edge {
                        from: Point::new(s.x0, lo),
                        to: Point::new(s.x1, lo),
                    });
                    edges.len() - 1
                };
                let t = if let Some(&e) = open_top.get(&hi) {
                    edges[e].from.x = s.x1;
                    e
                } else {
                    edges.push(Edge {
                        from: Point::new(s.x1, hi),
                        to: Point::new(s.x0, hi),
                    });
                    edges.len() - 1
                };
                let _ = bottom.insert(lo, b);
                let _ = top.insert(hi, t);
            }
            open_bottom = bottom;
            open_top = top;
            previous_end = Some(s.x1);
        }
    }

    /// Every rectangle inside the region that no larger rectangle inside
    /// the region contains.
    pub fn max_rectangles(&self) -> Vec<Rect<Dbu>> {
        let n = self.slabs.len();
        let mut out = Vec::new();
        for i in 0..n {
            let mut common = self.slabs[i].spans.clone();
            for j in i..n {
                if j > i {
                    if self.slabs[j].x0 != self.slabs[j - 1].x1 {
                        break;
                    }
                    common = combine_spans(&common, &self.slabs[j].spans, |a, b| a && b);
                    if common.is_empty() {
                        break;
                    }
                }
                for &(lo, hi) in &common {
                    let grows_left = i > 0
                        && self.slabs[i - 1].x1 == self.slabs[i].x0
                        && spans_cover(&self.slabs[i - 1].spans, lo, hi);
                    let grows_right = j + 1 < n
                        && self.slabs[j + 1].x0 == self.slabs[j].x1
                        && spans_cover(&self.slabs[j + 1].spans, lo, hi);
                    if !grows_left && !grows_right {
                        out.push(Rect::new(
                            Point::new(self.slabs[i].x0, lo),
                            Point::new(self.slabs[j].x1, hi),
                        ));
                    }
                }
            }
        }
        out
    }
}

/// The spans of the slab of `slabs` covering `[x0, x1]`, advancing `cursor`
/// past slabs that end before it; empty if no slab covers it.
fn covering_spans<'a>(slabs: &'a [Slab], cursor: &mut usize, x0: Dbu, x1: Dbu) -> &'a [(Dbu, Dbu)] {
    while *cursor < slabs.len() && slabs[*cursor].x1 <= x0 {
        *cursor += 1;
    }
    match slabs.get(*cursor) {
        Some(s) if s.x0 <= x0 && x1 <= s.x1 => &s.spans,
        _ => &[],
    }
}

/// `spans` sorted, with overlapping or touching spans merged and empty ones
/// dropped.
fn merged(mut spans: Spans) -> Spans {
    spans.retain(|&(lo, hi)| lo < hi);
    spans.sort_unstable();
    let mut out: Spans = Vec::with_capacity(spans.len());
    for (lo, hi) in spans {
        match out.last_mut() {
            Some(last) if lo <= last.1 => last.1 = last.1.max(hi),
            _ => out.push((lo, hi)),
        }
    }
    out
}

/// The spans where `keep` holds of membership in `a` and in `b`.
fn combine_spans(a: &[(Dbu, Dbu)], b: &[(Dbu, Dbu)], keep: fn(bool, bool) -> bool) -> Spans {
    let mut cuts: Vec<Dbu> = a
        .iter()
        .chain(b)
        .flat_map(|&span| <[Dbu; 2]>::from(span))
        .collect();
    cuts.sort_unstable();
    cuts.dedup();
    let mut out: Spans = Vec::new();
    let (mut i, mut j) = (0, 0);
    for w in cuts.windows(2) {
        let (y0, y1) = (w[0], w[1]);
        while i < a.len() && a[i].1 <= y0 {
            i += 1;
        }
        while j < b.len() && b[j].1 <= y0 {
            j += 1;
        }
        let in_a = a.get(i).is_some_and(|&(lo, _)| lo <= y0);
        let in_b = b.get(j).is_some_and(|&(lo, _)| lo <= y0);
        if !keep(in_a, in_b) {
            continue;
        }
        match out.last_mut() {
            Some(last) if last.1 == y0 => last.1 = y1,
            _ => out.push((y0, y1)),
        }
    }
    out
}

/// Whether `spans` contain all of `[lo, hi]`.
fn spans_cover(spans: &[(Dbu, Dbu)], lo: Dbu, hi: Dbu) -> bool {
    spans.iter().any(|&(a, b)| a <= lo && hi <= b)
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::from_corners(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    fn edge(x0: i64, y0: i64, x1: i64, y1: i64) -> Edge {
        Edge {
            from: Point::new(Dbu(x0), Dbu(y0)),
            to: Point::new(Dbu(x1), Dbu(y1)),
        }
    }

    fn sorted<T: Ord>(mut v: Vec<T>) -> Vec<T> {
        v.sort();
        v
    }

    fn key(r: &Rect<Dbu>) -> (i64, i64, i64, i64) {
        (r.min.x.get(), r.min.y.get(), r.max.x.get(), r.max.y.get())
    }

    fn edge_key(e: &Edge) -> (i64, i64, i64, i64) {
        (e.from.x.get(), e.from.y.get(), e.to.x.get(), e.to.y.get())
    }

    /// An L of a 10x2 bar along the bottom and a 2x10 bar up the left side.
    fn ell() -> Region {
        Region::from_rects([rect(0, 0, 10, 2), rect(0, 0, 2, 10)])
    }

    #[test]
    fn regions_covering_the_same_area_are_equal() {
        let one = Region::from_rects([rect(0, 0, 10, 10)]);
        let pieces = Region::from_rects([rect(0, 0, 5, 10), rect(5, 0, 10, 4), rect(5, 3, 10, 10)]);

        assert_eq!(one, pieces);
    }

    #[test]
    fn rectangles_without_area_add_nothing() {
        let region = Region::from_rects([rect(0, 0, 10, 0), rect(3, 3, 3, 8)]);

        assert!(region.is_empty());
    }

    #[test]
    fn overlapping_rectangles_count_their_shared_area_once() {
        let region = Region::from_rects([rect(0, 0, 10, 10), rect(5, 5, 15, 15)]);

        assert_eq!(region.area(), DbuArea(175));
    }

    #[test]
    fn an_ell_has_six_edges_with_the_region_on_their_left() {
        let edges = sorted(ell().edges().iter().map(edge_key).collect());

        let expected = sorted(
            [
                edge(0, 0, 10, 0),
                edge(10, 0, 10, 2),
                edge(10, 2, 2, 2),
                edge(2, 2, 2, 10),
                edge(2, 10, 0, 10),
                edge(0, 10, 0, 0),
            ]
            .iter()
            .map(edge_key)
            .collect(),
        );
        assert_eq!(edges, expected);
    }

    #[test]
    fn edge_headings_follow_their_direction() {
        assert_eq!(edge(0, 0, 10, 0).heading(), Heading::East);
        assert_eq!(edge(10, 2, 2, 2).heading(), Heading::West);
        assert_eq!(edge(10, 0, 10, 2).heading(), Heading::North);
        assert_eq!(edge(0, 10, 0, 0).heading(), Heading::South);
    }

    #[test]
    fn a_hole_is_bounded_by_edges_with_the_metal_on_their_left() {
        let ring = Region::from_rects([
            rect(0, 0, 10, 2),
            rect(0, 8, 10, 10),
            rect(0, 0, 2, 10),
            rect(8, 0, 10, 10),
        ]);

        let edges: HashSet<(i64, i64, i64, i64)> = ring.edges().iter().map(edge_key).collect();

        assert_eq!(edges.len(), 8);
        assert!(edges.contains(&edge_key(&edge(2, 2, 2, 8))));
        assert!(edges.contains(&edge_key(&edge(8, 8, 8, 2))));
        assert!(edges.contains(&edge_key(&edge(2, 8, 8, 8))));
        assert!(edges.contains(&edge_key(&edge(8, 2, 2, 2))));
    }

    #[test]
    fn an_ell_has_two_maximal_rectangles() {
        let rects = sorted(ell().max_rectangles().iter().map(key).collect());

        assert_eq!(rects, vec![(0, 0, 2, 10), (0, 0, 10, 2)]);
    }

    #[test]
    fn a_ring_has_four_maximal_rectangles() {
        let ring = Region::from_rects([
            rect(0, 0, 10, 2),
            rect(0, 8, 10, 10),
            rect(0, 0, 2, 10),
            rect(8, 0, 10, 10),
        ]);

        let rects = sorted(ring.max_rectangles().iter().map(key).collect());

        assert_eq!(
            rects,
            vec![(0, 0, 2, 10), (0, 0, 10, 2), (0, 8, 10, 10), (8, 0, 10, 10)]
        );
    }

    #[test]
    fn rectangles_meeting_at_a_corner_are_separate_pieces() {
        let region = Region::from_rects([rect(0, 0, 5, 5), rect(5, 5, 10, 10)]);

        assert_eq!(region.components().len(), 2);
    }

    #[test]
    fn rectangles_sharing_an_edge_are_one_piece() {
        let region = Region::from_rects([rect(0, 0, 5, 5), rect(5, 2, 10, 4), rect(20, 0, 25, 5)]);

        let mut areas: Vec<i64> = region.components().iter().map(|c| c.area().get()).collect();
        areas.sort_unstable();

        assert_eq!(areas, vec![25, 35]);
    }

    #[test]
    fn covering_needs_every_point_of_the_rectangle() {
        let region = ell();

        assert!(region.covers_rect(&rect(0, 0, 10, 2)));
        assert!(region.covers_rect(&rect(0, 0, 2, 10)));
        assert!(!region.covers_rect(&rect(0, 0, 3, 3)));
        assert!(region.intersects_rect(&rect(1, 1, 5, 5)));
        assert!(!region.intersects_rect(&rect(2, 2, 5, 5)));
    }

    /// A small deterministic generator for the randomised comparisons.
    struct Lcg(u64);

    impl Lcg {
        fn below(&mut self, n: i64) -> i64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            i64::try_from((self.0 >> 33) % u64::try_from(n).unwrap()).unwrap()
        }

        fn rects(&mut self, count: usize, side: i64) -> Vec<Rect<Dbu>> {
            (0..count)
                .map(|_| {
                    let (x, y) = (self.below(side), self.below(side));
                    rect(
                        x,
                        y,
                        x + 1 + self.below(side - x),
                        y + 1 + self.below(side - y),
                    )
                })
                .collect()
        }
    }

    /// Whether the unit cell with lower-left corner `(x, y)` lies in `rects`.
    fn cell_in(rects: &[Rect<Dbu>], x: i64, y: i64) -> bool {
        rects.iter().any(|r| r.encloses(&rect(x, y, x + 1, y + 1)))
    }

    fn cells(region: &Region, side: i64) -> HashSet<(i64, i64)> {
        let rects: Vec<Rect<Dbu>> = region.rectangles().collect();
        (0..=side)
            .flat_map(|x| (0..=side).map(move |y| (x, y)))
            .filter(|&(x, y)| cell_in(&rects, x, y))
            .collect()
    }

    #[test]
    fn boolean_operations_match_cell_by_cell_membership() {
        let mut lcg = Lcg(7);
        for _ in 0..200 {
            let (ra, rb) = (lcg.rects(4, 12), lcg.rects(4, 12));
            let (a, b) = (
                Region::from_rects(ra.clone()),
                Region::from_rects(rb.clone()),
            );
            let (ca, cb) = (cells(&a, 12), cells(&b, 12));

            assert_eq!(cells(&a.union(&b), 12), &ca | &cb);
            assert_eq!(cells(&a.intersection(&b), 12), &ca & &cb);
            assert_eq!(cells(&a.difference(&b), 12), &ca - &cb);
            assert_eq!(a.area().get(), i64::try_from(ca.len()).unwrap());
        }
    }

    /// Maximal rectangles of the unit-cell set `inside`, by trying every
    /// rectangle on the grid; on a grid a rectangle is maximal when no
    /// one-cell extension of it fits.
    fn brute_max_rectangles(inside: &HashSet<(i64, i64)>, side: i64) -> Vec<(i64, i64, i64, i64)> {
        let fits = |x0: i64, y0: i64, x1: i64, y1: i64| {
            (x0..x1).all(|x| (y0..y1).all(|y| inside.contains(&(x, y))))
        };
        let mut maximal = Vec::new();
        for x0 in 0..=side {
            for y0 in 0..=side {
                for x1 in x0 + 1..=side + 1 {
                    for y1 in y0 + 1..=side + 1 {
                        let grows = fits(x0 - 1, y0, x1, y1)
                            || fits(x0, y0 - 1, x1, y1)
                            || fits(x0, y0, x1 + 1, y1)
                            || fits(x0, y0, x1, y1 + 1);
                        if fits(x0, y0, x1, y1) && !grows {
                            maximal.push((x0, y0, x1, y1));
                        }
                    }
                }
            }
        }
        sorted(maximal)
    }

    #[test]
    fn maximal_rectangles_match_an_exhaustive_search() {
        let mut lcg = Lcg(11);
        for _ in 0..60 {
            let region = Region::from_rects(lcg.rects(3, 8));

            let got = sorted(region.max_rectangles().iter().map(key).collect());

            assert_eq!(got, brute_max_rectangles(&cells(&region, 8), 8));
        }
    }

    #[test]
    fn edges_add_up_to_the_boundary_of_each_cell_set() {
        let mut lcg = Lcg(13);
        for _ in 0..100 {
            let region = Region::from_rects(lcg.rects(4, 10));
            let inside = cells(&region, 10);
            let boundary: i64 = inside
                .iter()
                .map(|&(x, y)| {
                    [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
                        .iter()
                        .filter(|c| !inside.contains(c))
                        .count()
                })
                .sum::<usize>()
                .try_into()
                .unwrap();

            let length: i64 = region.edges().iter().map(|e| e.length().get()).sum();

            assert_eq!(length, boundary);
        }
    }

    #[test]
    fn every_edge_ends_where_another_starts() {
        let mut lcg = Lcg(17);
        for _ in 0..100 {
            let region = Region::from_rects(lcg.rects(4, 10));
            let edges = region.edges();
            let mut starts: Vec<(i64, i64)> = edges
                .iter()
                .map(|e| (e.from.x.get(), e.from.y.get()))
                .collect();
            let mut ends: Vec<(i64, i64)> =
                edges.iter().map(|e| (e.to.x.get(), e.to.y.get())).collect();
            starts.sort_unstable();
            ends.sort_unstable();

            assert_eq!(starts, ends);
            assert!(edges.iter().all(|e| e.length() > Dbu::ZERO));
        }
    }

    fn continues_straight(first: &Edge, second: &Edge) -> bool {
        let meets = first.to == second.from;
        meets && first.heading() == second.heading()
    }

    #[test]
    fn no_edge_continues_straight_into_another() {
        let mut lcg = Lcg(23);
        for _ in 0..100 {
            let edges = Region::from_rects(lcg.rects(4, 10)).edges();

            let joins_straight = edges
                .iter()
                .any(|a| edges.iter().any(|b| a != b && continues_straight(a, b)));

            assert!(!joins_straight);
        }
    }

    #[test]
    fn components_partition_the_region() {
        let mut lcg = Lcg(19);
        for _ in 0..100 {
            let region = Region::from_rects(lcg.rects(5, 12));

            let pieces = region.components();
            let joined = pieces.iter().fold(Region::new(), |acc, p| acc.union(p));
            let total: i64 = pieces.iter().map(|p| p.area().get()).sum();

            assert_eq!(joined, region);
            assert_eq!(total, region.area().get());
        }
    }
}
