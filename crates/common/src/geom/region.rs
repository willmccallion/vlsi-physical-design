//! Rectilinear regions: finite unions of axis-aligned rectangles.
//!
//! A region is kept as vertical slabs: between consecutive X breakpoints,
//! one sorted list of disjoint Y spans. Spans that touch are merged and
//! neighbouring slabs that are equal are joined, so two regions covering the
//! same area have the same representation. Rectangles without area add
//! nothing.

use super::dbu::{Dbu, DbuArea};
use super::interval::{Interval, merge_touching};
use super::point::Point;
use super::rect::Rect;
use std::cmp::Ordering;
use std::collections::{BTreeMap, HashMap};
use std::ops::Bound;

/// Disjoint, non-touching Y intervals in increasing order.
type Spans = Vec<Interval>;

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

impl Heading {
    /// The heading a quarter turn counterclockwise.
    #[must_use]
    pub const fn turned_left(self) -> Self {
        match self {
            Self::East => Self::North,
            Self::North => Self::West,
            Self::West => Self::South,
            Self::South => Self::East,
        }
    }

    /// The heading a quarter turn clockwise.
    #[must_use]
    pub const fn turned_right(self) -> Self {
        match self {
            Self::East => Self::South,
            Self::South => Self::West,
            Self::West => Self::North,
            Self::North => Self::East,
        }
    }
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
        match (self.to.x.cmp(&self.from.x), self.to.y.cmp(&self.from.y)) {
            (Ordering::Greater, _) => Heading::East,
            (Ordering::Less, _) => Heading::West,
            (Ordering::Equal, Ordering::Greater) => Heading::North,
            (Ordering::Equal, _) => Heading::South,
        }
    }

    /// The edge's length.
    pub const fn length(&self) -> Dbu {
        self.from.manhattan(self.to)
    }
}

impl Region {
    /// The empty region.
    pub const fn new() -> Self {
        Self { slabs: Vec::new() }
    }

    /// The union of `rects`.
    pub fn from_rects(rects: impl IntoIterator<Item = Rect<Dbu>>) -> Self {
        // A rectangle without width would stretch across the slab at its x;
        // one without height adds only empty spans, which merging drops.
        let mut rects: Vec<Rect<Dbu>> = rects
            .into_iter()
            .filter(|r| r.width() > Dbu::ZERO)
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
            let spans = merged(active.iter().map(Rect::y_span).collect());
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
        self.slabs
            .iter()
            .map(|s| (s.x1 - s.x0) * s.spans.iter().map(|span| span.len()).sum::<Dbu>())
            .sum()
    }

    /// The smallest rectangle containing the region.
    pub fn bounding_box(&self) -> Option<Rect<Dbu>> {
        let first = self.slabs.first()?;
        let last = self.slabs.last()?;
        let lo = self
            .slabs
            .iter()
            .filter_map(|s| s.spans.first())
            .map(|s| s.lo)
            .min()?;
        let hi = self
            .slabs
            .iter()
            .filter_map(|s| s.spans.last())
            .map(|s| s.hi)
            .max()?;
        Some(Rect::new(Point::new(first.x0, lo), Point::new(last.x1, hi)))
    }

    /// The rectangles the region is made of, one per span of each slab.
    pub fn rectangles(&self) -> impl Iterator<Item = Rect<Dbu>> + '_ {
        self.slabs.iter().flat_map(|s| {
            s.spans
                .iter()
                .map(move |span| Rect::new(Point::new(s.x0, span.lo), Point::new(s.x1, span.hi)))
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
                    .any(|span| span.lo < rect.max.y && rect.min.y < span.hi)
            })
    }

    /// Whether the region, taken with its boundary, contains all of `rect`,
    /// by the rule of [`Rect::covered_by`].
    pub fn covers_rect(&self, rect: &Rect<Dbu>) -> bool {
        // The sweep finds no slab inside a rectangle without width; one
        // without height it covers by each slab's closed spans.
        if rect.width() == Dbu::ZERO {
            return rect.covered_by(&self.rectangles().collect::<Vec<_>>());
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
        self.connected(Corners::Separate)
    }

    /// The parts of the plane the region encloses, as Boost.Polygon gives
    /// the holes of a polygon: empty space meeting the outside even at a
    /// single corner is outside.
    pub fn holes(&self) -> Vec<Self> {
        let Some(bbox) = self.bounding_box() else {
            return Vec::new();
        };
        let frame = bbox.expand(Dbu::new(1));
        Self::from_rects([frame])
            .difference(self)
            .connected(Corners::Join)
            .into_iter()
            .filter(|space| space.bounding_box() != Some(frame))
            .collect()
    }

    fn connected(&self, corners: Corners) -> Vec<Self> {
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
            for (a, b) in meeting_spans(&left.spans, &right.spans, corners) {
                union(&mut parent, offsets[k - 1] + a, offsets[k] + b);
            }
        }
        let mut index_of_root: HashMap<usize, usize> = HashMap::new();
        let mut pieces: Vec<Self> = Vec::new();
        for (k, s) in self.slabs.iter().enumerate() {
            let mut by_piece: Vec<(usize, Interval)> = Vec::with_capacity(s.spans.len());
            for (n, &span) in s.spans.iter().enumerate() {
                let root = find(&mut parent, offsets[k] + n);
                let piece = *index_of_root.entry(root).or_insert_with(|| {
                    pieces.push(Self::new());
                    pieces.len() - 1
                });
                by_piece.push((piece, span));
            }
            by_piece.sort_by_key(|&(piece, _)| piece);
            for group in by_piece.chunk_by(|a, b| a.0 == b.0) {
                let spans = group.iter().map(|&(_, span)| span).collect();
                pieces[group[0].0].push_slab(s.x0, s.x1, spans);
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
            k += self.slabs[k..].iter().take_while(|s| s.x1 < x).count();
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
            for Interval { lo, hi } in combine_spans(right, left, |r, l| r && !l) {
                edges.push(Edge {
                    from: Point::new(x, hi),
                    to: Point::new(x, lo),
                });
            }
            for Interval { lo, hi } in combine_spans(left, right, |l, r| l && !r) {
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
            for &Interval { lo, hi } in &s.spans {
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
    ///
    /// A maximal rectangle starting at a slab cannot lie in a span the slab
    /// to its left covers, so such spans are dropped as the rectangles grow
    /// right: a long piece of metal is not walked again from every slab it
    /// crosses.
    pub fn max_rectangles(&self) -> Vec<Rect<Dbu>> {
        let n = self.slabs.len();
        let mut out = Vec::new();
        for i in 0..n {
            let left: &[Interval] = if i > 0 && self.slabs[i - 1].x1 == self.slabs[i].x0 {
                &self.slabs[i - 1].spans
            } else {
                &[]
            };
            let mut common: Spans = self.slabs[i]
                .spans
                .iter()
                .copied()
                .filter(|span| !spans_cover(left, span.lo, span.hi))
                .collect();
            for j in i..n {
                if j > i {
                    if self.slabs[j].x0 != self.slabs[j - 1].x1 {
                        break;
                    }
                    common = combine_spans(&common, &self.slabs[j].spans, |a, b| a && b);
                    common.retain(|span| !spans_cover(left, span.lo, span.hi));
                }
                if common.is_empty() {
                    break;
                }
                for &Interval { lo, hi } in &common {
                    let grows_right = j + 1 < n
                        && self.slabs[j + 1].x0 == self.slabs[j].x1
                        && spans_cover(&self.slabs[j + 1].spans, lo, hi);
                    if !grows_right {
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

    /// The region cut into rectangles as Boost.Polygon's `get_rectangles`
    /// cuts it with horizontal slicing: a sweep upward opens a rectangle
    /// over each horizontal run and closes it where a boundary edge next
    /// touches it.
    pub fn horizontal_slices(&self) -> Vec<Rect<Dbu>> {
        let events = self
            .edges()
            .into_iter()
            .filter(|e| matches!(e.heading(), Heading::East | Heading::West))
            .map(|e| (e.from.y, e.from.x.min(e.to.x), e.from.x.max(e.to.x)));
        Sweep::slices(events)
            .into_iter()
            .map(|s| Rect::new(Point::new(s.lo, s.start), Point::new(s.hi, s.end)))
            .collect()
    }

    /// The region cut into rectangles as Boost.Polygon's `get_rectangles`
    /// cuts it with vertical slicing: [`Self::horizontal_slices`] with the
    /// axes swapped.
    pub fn vertical_slices(&self) -> Vec<Rect<Dbu>> {
        let events = self
            .edges()
            .into_iter()
            .filter(|e| matches!(e.heading(), Heading::North | Heading::South))
            .map(|e| (e.from.x, e.from.y.min(e.to.y), e.from.y.max(e.to.y)));
        Sweep::slices(events)
            .into_iter()
            .map(|s| Rect::new(Point::new(s.start, s.lo), Point::new(s.end, s.hi)))
            .collect()
    }
}

/// Whether pieces of a region meeting only at a corner are one piece.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Corners {
    Separate,
    Join,
}

/// A rectangle of a slicing sweep: `[lo, hi]` across the sweep, from
/// `start` to `end` along it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Slice {
    lo: Dbu,
    hi: Dbu,
    start: Dbu,
    end: Dbu,
}

/// Boost.Polygon's `ScanLineToRects`: the rectangles open at the sweep
/// position, keyed by their low end, and the one being widened by edges
/// at the current position.
#[derive(Debug, Default)]
struct Sweep {
    open: BTreeMap<Dbu, Slice>,
    current: Option<Slice>,
    at: Dbu,
    out: Vec<Slice>,
}

impl Sweep {
    /// The slices made by `events`, boundary edges across the sweep as
    /// `(position, lo, hi)`.
    fn slices(events: impl Iterator<Item = (Dbu, Dbu, Dbu)>) -> Vec<Slice> {
        let mut events: Vec<(Dbu, Dbu, Dbu)> = events.collect();
        events.sort_unstable();
        let mut sweep = Self::default();
        let mut position = None;
        for (at, lo, hi) in events {
            if position != Some(at) {
                sweep.advance(at);
                position = Some(at);
            }
            sweep.edge(lo, hi);
        }
        sweep.out
    }

    fn advance(&mut self, at: Dbu) {
        if let Some(current) = self.current.take() {
            let _ = self.open.insert(current.lo, current);
        }
        self.at = at;
    }

    const fn opened(&self, lo: Dbu, hi: Dbu) -> Slice {
        Slice {
            lo,
            hi,
            start: self.at,
            end: self.at,
        }
    }

    fn close(&mut self, slice: Slice) {
        if slice.start < self.at {
            self.out.push(Slice {
                end: self.at,
                ..slice
            });
        }
    }

    /// `processEdge_`: an edge inside an open rectangle closes that part of
    /// it; an edge only touching open rectangles closes them and starts a
    /// wider one with them.
    fn edge(&mut self, lo: Dbu, hi: Dbu) {
        let mut processed = false;
        let mut cursor = self
            .open
            .range(..=lo)
            .next_back()
            .or_else(|| self.open.iter().next())
            .map(|(&k, _)| k);
        while let Some(key) = cursor {
            let slice = self.open[&key];
            if slice.lo > hi {
                break;
            }
            let next = self
                .open
                .range((Bound::Excluded(key), Bound::Unbounded))
                .next()
                .map(|(&k, _)| k);
            cursor = next;
            if slice.hi < lo {
                continue;
            }
            self.close(slice);
            let _ = self.open.remove(&key);
            if slice.lo <= lo && hi <= slice.hi {
                if slice.lo < lo {
                    let _ = self.open.insert(slice.lo, self.opened(slice.lo, lo));
                }
                if slice.hi > hi {
                    let _ = self.open.insert(hi, self.opened(hi, slice.hi));
                }
                return;
            }
            self.widen_touching(slice, lo, hi, processed);
            processed = true;
        }
        if !processed {
            self.open_alone(lo, hi);
        }
    }

    /// The edge `[lo, hi]` touches the open rectangle `slice`, now closed.
    fn widen_touching(&mut self, slice: Slice, lo: Dbu, hi: Dbu, processed: bool) {
        let joined = self.opened(slice.lo.min(lo), slice.hi.max(hi));
        let Some(mut current) = self.current else {
            self.current = Some(joined);
            return;
        };
        if current.hi < lo {
            let _ = self.open.insert(current.lo, current);
            self.current = Some(joined);
            return;
        }
        if !processed && current.hi > lo {
            let _ = self.open.insert(current.lo, Slice { hi: lo, ..current });
            if current.hi > hi {
                current.lo = hi;
                self.current = Some(current);
            } else {
                self.current = None;
            }
        } else {
            current.hi = hi.max(slice.hi);
            self.current = Some(current);
        }
    }

    /// The edge `[lo, hi]` touches no open rectangle.
    fn open_alone(&mut self, lo: Dbu, hi: Dbu) {
        if let Some(mut current) = self.current.take() {
            if current.hi >= lo {
                if current.hi > lo {
                    let _ = self.open.insert(current.lo, Slice { hi: lo, ..current });
                    if current.hi > hi {
                        current.lo = hi;
                        self.current = Some(current);
                    }
                } else {
                    current.hi = hi;
                    self.current = Some(current);
                }
                return;
            }
            let _ = self.open.insert(current.lo, current);
        }
        let _ = self.open.insert(lo, self.opened(lo, hi));
    }
}

/// The spans of the slab of `slabs` covering `[x0, x1]`, advancing `cursor`
/// past slabs that end before it; empty if no slab covers it.
fn covering_spans<'a>(slabs: &'a [Slab], cursor: &mut usize, x0: Dbu, x1: Dbu) -> &'a [Interval] {
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
    spans.retain(|span| span.lo < span.hi);
    spans.sort_unstable();
    merge_touching(spans)
}

/// The spans where `keep` holds of membership in `a` and in `b`.
fn combine_spans(a: &[Interval], b: &[Interval], keep: fn(bool, bool) -> bool) -> Spans {
    let mut cuts: Vec<Dbu> = a
        .iter()
        .chain(b)
        .flat_map(|span| [span.lo, span.hi])
        .collect();
    cuts.sort_unstable();
    cuts.dedup();
    let mut out: Spans = Vec::new();
    let (mut i, mut j) = (0, 0);
    for w in cuts.windows(2) {
        let (y0, y1) = (w[0], w[1]);
        while i < a.len() && a[i].hi <= y0 {
            i += 1;
        }
        while j < b.len() && b[j].hi <= y0 {
            j += 1;
        }
        let in_a = a.get(i).is_some_and(|s| s.lo <= y0);
        let in_b = b.get(j).is_some_and(|s| s.lo <= y0);
        if !keep(in_a, in_b) {
            continue;
        }
        match out.last_mut() {
            Some(last) if last.hi == y0 => last.hi = y1,
            _ => out.push(Interval::new(y0, y1)),
        }
    }
    out
}

/// Whether `spans` contain all of `[lo, hi]`.
/// The pairs of spans of two abutting slabs that meet, by their indices
/// in each.
fn meeting_spans(left: &[Interval], right: &[Interval], corners: Corners) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    let (mut a, mut b) = (0, 0);
    while let (Some(&l), Some(&r)) = (left.get(a), right.get(b)) {
        if spans_meet(l, r, corners) {
            pairs.push((a, b));
        }
        if l.hi < r.hi {
            a += 1;
        } else {
            b += 1;
        }
    }
    pairs
}

/// Whether spans of abutting slabs meet, sharing an edge or, where corners
/// join, a point.
fn spans_meet(l: Interval, r: Interval, corners: Corners) -> bool {
    match corners {
        Corners::Separate => l.lo < r.hi && r.lo < l.hi,
        Corners::Join => l.lo <= r.hi && r.lo <= l.hi,
    }
}

fn spans_cover(spans: &[Interval], lo: Dbu, hi: Dbu) -> bool {
    let k = spans.partition_point(|s| s.hi < hi);
    spans.get(k).is_some_and(|s| s.lo <= lo && hi <= s.hi)
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
mod tests;
