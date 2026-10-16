//! Minimum step length along the boundary of a piece of metal, as `FlexGC`
//! checks a `MINSTEP` rule.
//!
//! Walking each boundary loop from an edge at least the step length long,
//! every run of shorter edges between two long edges is a step. A run
//! breaks the rule when it has more edges than `MAXEDGES` allows, or when it
//! exists at all for a rule without `MAXEDGES`, provided routed metal takes
//! part in it. A loop with fewer than two long edges is left alone, as
//! `FlexGC` never closes a run on the edge it started from.

use super::model::{GcEdge, Metal, Model};
use super::rules::Rules;
use super::{Marker, Markers, Violation};
use crate::db::indices::LayerId;
use crate::db::library::LayerRef;
use crate::db::rules::MinStep;
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;
use crate::geom::rect::Rect;

/// Checks every boundary loop on every routing layer with minimum step rules.
pub(super) fn check(model: &Model<'_>, rules: &Rules) -> Markers {
    model
        .metal
        .iter()
        .enumerate()
        .filter(|(l, _)| !rules.metal[*l].min_steps.is_empty())
        .map(|(l, metal)| {
            let check = LayerCheck {
                metal,
                layer: LayerId::new(l),
                rules: &rules.metal[l].min_steps,
            };
            piece_ranges(&metal.edges)
                .into_iter()
                .fold(Markers::default(), |mut found, range| {
                    check.check_piece(range, &mut found);
                    found
                })
        })
        .fold(Markers::default(), Markers::merge)
}

/// The index ranges of each piece's edges, which lie together.
fn piece_ranges(edges: &[GcEdge]) -> Vec<std::ops::Range<usize>> {
    let mut ranges = Vec::new();
    let mut start = 0;
    for i in 1..=edges.len() {
        if i == edges.len() || edges[i].piece != edges[start].piece {
            ranges.push(start..i);
            start = i;
        }
    }
    ranges
}

/// The minimum step checks of one routing layer.
struct LayerCheck<'m> {
    metal: &'m Metal<'m>,
    layer: LayerId,
    rules: &'m [MinStep],
}

impl LayerCheck<'_> {
    fn check_piece(&self, range: std::ops::Range<usize>, found: &mut Markers) {
        let mut visited = vec![false; range.len()];
        for start in range.clone() {
            if visited[start - range.start] {
                continue;
            }
            let mut loop_edges = Vec::new();
            let mut i = start;
            while !visited[i - range.start] {
                visited[i - range.start] = true;
                loop_edges.push(i);
                i = self.metal.edges[i].next as usize;
                if !range.contains(&i) {
                    break;
                }
            }
            for rule in self.rules {
                self.check_loop(&loop_edges, rule, found);
            }
        }
    }

    /// Checks the runs of short edges of one boundary loop, given in order.
    fn check_loop(&self, loop_edges: &[usize], rule: &MinStep, found: &mut Markers) {
        let edge = |k: usize| &self.metal.edges[loop_edges[k % loop_edges.len()]];
        let is_long = |e: &GcEdge| e.edge.length() >= rule.length;
        let Some(first) = (0..loop_edges.len()).find(|&k| is_long(edge(k))) else {
            return;
        };
        if !(first + 1..loop_edges.len()).any(|k| is_long(edge(k))) {
            return;
        }
        let mut run = Run::starting_at(edge(first));
        for k in first + 1..=first + loop_edges.len() {
            let e = edge(k);
            if is_long(e) {
                run.routed |= !e.fixed;
                self.check_run(&run, rule, found);
                run = Run::starting_at(e);
            } else {
                run.extend(e);
            }
        }
    }

    fn check_run(&self, run: &Run, rule: &MinStep, found: &mut Markers) {
        if run.edges == 0 || !run.routed {
            return;
        }
        if let Some(max_edges) = rule.max_edges
            && run.edges <= max_edges
        {
            return;
        }
        let marker = Marker::new(
            Violation::MinStep,
            0,
            LayerRef::Routing(self.layer),
            run.bbox,
            run.owner,
            run.owner,
        );
        found.add(marker, false);
    }
}

/// A run of short edges after a long one: how many, whether routed metal
/// takes part, and the box around its corners.
struct Run {
    owner: super::layout::OwnerIndex,
    edges: u32,
    routed: bool,
    bbox: Rect<Dbu>,
}

impl Run {
    const fn starting_at(long_edge: &GcEdge) -> Self {
        Self {
            owner: long_edge.owner,
            edges: 0,
            routed: !long_edge.fixed,
            bbox: Rect::new(long_edge.edge.to, long_edge.edge.to),
        }
    }

    fn extend(&mut self, short_edge: &GcEdge) {
        self.edges += 1;
        self.routed |= !short_edge.fixed;
        let p: Point<Dbu> = short_edge.edge.to;
        self.bbox = Rect::new(
            Point::new(self.bbox.min.x.min(p.x), self.bbox.min.y.min(p.y)),
            Point::new(self.bbox.max.x.max(p.x), self.bbox.max.y.max(p.y)),
        );
    }
}
