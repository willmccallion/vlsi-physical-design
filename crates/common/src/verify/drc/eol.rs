//! End-of-line spacing between the boundary edges of a layer, as `FlexGC`
//! checks a `SPACING ... ENDOFLINE` rule.
//!
//! A line end is a convex edge shorter than the rule's end-of-line width.
//! Across from it, within the rule's spacing and extended sideways by its
//! `WITHIN`, an edge of another piece running the opposite way is a
//! violation when the gap between them holds no metal and routed metal has
//! a part in it. A `PARALLELEDGE` clause arms the line end only when an edge
//! running along the wire, that is across the line end, lies beside one of
//! its corners within the clause's distances, or beside both with `TWOEDGES`.

use super::model::{Facing, GcEdge, Metal, Model};
use super::rules::Rules;
use super::{Marker, Markers, Violation};
use crate::db::indices::LayerId;
use crate::db::library::LayerRef;
use crate::db::rules::{EndOfLineSpacing, ParallelEdge};
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use crate::geom::region::{Edge, Heading};

/// Checks every boundary edge on every routing layer with end-of-line rules.
pub(super) fn check(model: &Model<'_>, rules: &Rules) -> Markers {
    model
        .metal
        .iter()
        .enumerate()
        .filter(|(l, _)| !rules.metal[*l].end_of_line.is_empty())
        .map(|(l, metal)| {
            let check = LayerCheck {
                metal,
                layer: LayerId::new(l),
                rules: &rules.metal[l].end_of_line,
            };
            (0..metal.edges.len()).fold(Markers::default(), |mut found, i| {
                check.check_edge(i, &mut found);
                found
            })
        })
        .fold(Markers::default(), Markers::merge)
}

/// Which end of an edge a parallel edge is looked for beside.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum End {
    /// The end the edge starts from.
    Low,
    /// The end the edge runs to.
    High,
}

/// The end-of-line checks of one routing layer.
struct LayerCheck<'m> {
    metal: &'m Metal<'m>,
    layer: LayerId,
    rules: &'m [EndOfLineSpacing],
}

impl LayerCheck<'_> {
    fn check_edge(&self, i: usize, found: &mut Markers) {
        let edge = &self.metal.edges[i];
        for rule in self.rules {
            if !self.is_line_end(edge, rule) {
                continue;
            }
            let mut routed = !edge.fixed
                && self
                    .metal
                    .has_route_in(edge.owner, &inside_strip(&edge.edge));
            if !self.has_parallel_edge(edge, rule, &mut routed) {
                continue;
            }
            self.check_across(edge, rule, routed, found);
        }
    }

    /// Whether the edge is a line end for `rule`: shorter than the rule's
    /// width and convex at both corners.
    fn is_line_end(&self, edge: &GcEdge, rule: &EndOfLineSpacing) -> bool {
        let prev = &self.metal.edges[edge.prev as usize].edge;
        let next = &self.metal.edges[edge.next as usize].edge;
        edge.edge.length() < rule.eol_width
            && turns_left(prev, &edge.edge)
            && turns_left(&edge.edge, next)
    }

    /// Whether the rule's parallel-edge clause, if any, arms the line end;
    /// `routed` learns whether a parallel edge found is routed metal.
    fn has_parallel_edge(&self, edge: &GcEdge, rule: &EndOfLineSpacing, routed: &mut bool) -> bool {
        let Some(parallel) = rule.parallel_edge else {
            return true;
        };
        let low = self.parallel_edge_beside(edge, rule, &parallel, End::Low, routed);
        let high = self.parallel_edge_beside(edge, rule, &parallel, End::High, routed);
        if parallel.two_edges {
            low && high
        } else {
            low || high
        }
    }

    fn parallel_edge_beside(
        &self,
        edge: &GcEdge,
        rule: &EndOfLineSpacing,
        parallel: &ParallelEdge,
        end: End,
        routed: &mut bool,
    ) -> bool {
        let query = parallel_edge_query(&edge.edge, rule.within, parallel, end);
        let mut found = false;
        for j in self.metal.edge_indices_near(&query) {
            let other = &self.metal.edges[j];
            let beside = match end {
                End::Low => turn(&other.edge, &edge.edge),
                End::High => turn(&edge.edge, &other.edge),
            };
            if beside != Turn::Right {
                continue;
            }
            let strip = inside_strip(&other.edge);
            if !query.overlaps(&strip) {
                continue;
            }
            found = true;
            if !*routed
                && !other.fixed
                && let Some(part) = query.intersection(&strip)
                && self.metal.has_route_in(other.owner, &part)
            {
                *routed = true;
                break;
            }
        }
        found
    }

    /// Checks the line end against every edge of another piece running the
    /// opposite way within the rule's spacing.
    fn check_across(
        &self,
        edge: &GcEdge,
        rule: &EndOfLineSpacing,
        routed: bool,
        found: &mut Markers,
    ) {
        let query = line_end_query(&edge.edge, rule.within, rule.spacing);
        for j in self.metal.edge_indices_near(&query) {
            let other = &self.metal.edges[j];
            if other.piece == edge.piece
                || (edge.fixed && other.fixed)
                || !opposite(edge.edge.heading(), other.edge.heading())
            {
                continue;
            }
            let strip = inside_strip(&other.edge);
            if !query.overlaps(&strip) {
                continue;
            }
            let routed = routed
                || (!other.fixed
                    && query
                        .intersection(&strip)
                        .is_some_and(|part| self.metal.has_route_in(other.owner, &part)));
            if routed {
                self.mark(edge, other, found);
            }
        }
    }

    /// Reports the gap between the line end and `other` unless metal lies in it.
    fn mark(&self, edge: &GcEdge, other: &GcEdge, found: &mut Markers) {
        let between = Facing::new(&inside_strip(&edge.edge), &inside_strip(&other.edge)).marker;
        let probe = if between.width() <= Dbu::ZERO || between.height() <= Dbu::ZERO {
            match edge.edge.heading() {
                Heading::East | Heading::West => Rect::new(
                    Point::new(between.min.x - Dbu(1), between.min.y),
                    Point::new(between.max.x + Dbu(1), between.max.y),
                ),
                Heading::North | Heading::South => Rect::new(
                    Point::new(between.min.x, between.min.y - Dbu(1)),
                    Point::new(between.max.x, between.max.y + Dbu(1)),
                ),
            }
        } else {
            between
        };
        if self.metal.has_metal_in(&probe) {
            return;
        }
        let marker = Marker::new(
            Violation::EndOfLineSpacing,
            0,
            LayerRef::Routing(self.layer),
            between,
            edge.owner,
            other.owner,
        );
        found.add(marker, false);
    }
}

/// The turn from the direction of one segment to that of another.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Turn {
    Left,
    Right,
    Straight,
}

/// The turn from `a`'s direction to `b`'s, as Boost.Polygon orients two
/// segments: parallel segments turn neither way.
fn turn(a: &Edge, b: &Edge) -> Turn {
    let dir = |e: &Edge| {
        (
            i128::from((e.to.x - e.from.x).get()),
            i128::from((e.to.y - e.from.y).get()),
        )
    };
    let ((ax, ay), (bx, by)) = (dir(a), dir(b));
    match (ax * by - ay * bx).signum() {
        1 => Turn::Left,
        -1 => Turn::Right,
        _ => Turn::Straight,
    }
}

/// Whether the boundary turns left, toward the metal, from `a` to `b`.
fn turns_left(a: &Edge, b: &Edge) -> bool {
    turn(a, b) == Turn::Left
}

const fn opposite(a: Heading, b: Heading) -> bool {
    matches!(
        (a, b),
        (Heading::East, Heading::West)
            | (Heading::West, Heading::East)
            | (Heading::North, Heading::South)
            | (Heading::South, Heading::North)
    )
}

/// The strip one unit deep just inside the metal along `edge`.
fn inside_strip(edge: &Edge) -> Rect<Dbu> {
    let (from, to) = (edge.from, edge.to);
    let one = Dbu(1);
    match edge.heading() {
        Heading::East => Rect::new(from, Point::new(to.x, to.y + one)),
        Heading::West => Rect::new(Point::new(to.x, to.y - one), from),
        Heading::North => Rect::new(Point::new(from.x - one, from.y), to),
        Heading::South => Rect::new(to, Point::new(from.x + one, from.y)),
    }
}

/// The region across from a line end that a facing edge must reach into:
/// `spacing` deep outside the edge, `within` past each end.
fn line_end_query(edge: &Edge, within: Dbu, spacing: Dbu) -> Rect<Dbu> {
    let (from, to) = (edge.from, edge.to);
    match edge.heading() {
        Heading::East => Rect::new(
            Point::new(from.x - within, from.y - spacing),
            Point::new(to.x + within, to.y),
        ),
        Heading::West => Rect::new(
            Point::new(to.x - within, to.y),
            Point::new(from.x + within, from.y + spacing),
        ),
        Heading::North => Rect::new(
            Point::new(from.x, from.y - within),
            Point::new(to.x + spacing, to.y + within),
        ),
        Heading::South => Rect::new(
            Point::new(to.x - spacing, to.y - within),
            Point::new(from.x, from.y + within),
        ),
    }
}

/// The region beside one `end` of a line end where a parallel edge arms the
/// rule: `parallel.spacing` past the end along the edge, from `within`
/// outside the edge to `parallel.within` inside.
fn parallel_edge_query(edge: &Edge, within: Dbu, parallel: &ParallelEdge, end: End) -> Rect<Dbu> {
    let p = match end {
        End::Low => edge.from,
        End::High => edge.to,
    };
    let (space, par_within) = (parallel.spacing, parallel.within);
    match (edge.heading(), end) {
        (Heading::East, End::Low) => Rect::new(
            Point::new(p.x - space, p.y - within),
            Point::new(p.x, p.y + par_within),
        ),
        (Heading::East, End::High) => Rect::new(
            Point::new(p.x, p.y - within),
            Point::new(p.x + space, p.y + par_within),
        ),
        (Heading::West, End::Low) => Rect::new(
            Point::new(p.x, p.y - par_within),
            Point::new(p.x + space, p.y + within),
        ),
        (Heading::West, End::High) => Rect::new(
            Point::new(p.x - space, p.y - par_within),
            Point::new(p.x, p.y + within),
        ),
        (Heading::North, End::Low) => Rect::new(
            Point::new(p.x - par_within, p.y - space),
            Point::new(p.x + within, p.y),
        ),
        (Heading::North, End::High) => Rect::new(
            Point::new(p.x - par_within, p.y),
            Point::new(p.x + within, p.y + space),
        ),
        (Heading::South, End::Low) => Rect::new(
            Point::new(p.x - within, p.y),
            Point::new(p.x + par_within, p.y + space),
        ),
        (Heading::South, End::High) => Rect::new(
            Point::new(p.x - within, p.y - space),
            Point::new(p.x + par_within, p.y),
        ),
    }
}
