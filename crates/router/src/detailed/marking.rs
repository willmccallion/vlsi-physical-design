//! Where a violation marks the track graph, as `TritonRoute` adds marker
//! cost (`initMazeCost_marker_route_queue_addHistoryCost` in `OpenROAD`'s
//! `src/drt/src/dr/FlexDR_init.cpp`). The marks go on the metal that takes
//! part in the violation, not on the empty space around it: on each wire
//! of a violating net that meets the violation, the crossings within the
//! violation grown by the wire's width, grown again until at least two are
//! taken in; one via of each violating net that meets it; and the
//! crossings under a patch that meets it, with the vias above and below
//! them. Only wiring inside the worker's region is marked, since only that
//! can move.

use super::DetailedRouter;
use super::region::contains_half_open;
use crate::access::Node;
use crate::search::Marks;
use crate::tech::LayerTech;
use crate::wiring::{Edge, NetRoute};
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::rect::Rect;
use pare_common::verify::shapes::wire_rect;

/// Widths a violation is grown by, at most, to take in two crossings of a
/// wire: `TritonRoute` grows it once and then up to five times more.
const MAX_BLOATS: i64 = 6;

impl DetailedRouter<'_> {
    /// Marks in `marks` the places inside `area` where the wiring of
    /// `routes`, each net's route as it stands, takes part in a violation
    /// on `layer` at `bbox`.
    pub(super) fn mark_violation(
        &self,
        marks: &mut Marks,
        area: &Rect<Dbu>,
        layer: LayerRef,
        bbox: &Rect<Dbu>,
        routes: &[(NetId, &NetRoute)],
    ) {
        for &(net, route) in routes {
            if let LayerRef::Routing(l) = layer
                && self.tech.routable(l)
            {
                self.mark_wires(marks, area, l, bbox, net, route);
                self.mark_patches(marks, area, l, bbox, route);
            }
            self.mark_one_via(marks, area, layer, bbox, route);
        }
    }

    /// Marks the crossings of `net`'s wires on `layer` near a violation at
    /// `bbox`.
    fn mark_wires(
        &self,
        marks: &mut Marks,
        area: &Rect<Dbu>,
        layer: LayerId,
        bbox: &Rect<Dbu>,
        net: NetId,
        route: &NetRoute,
    ) {
        let lt = self.tech.layer(layer);
        let width = self.tech.wire_width(self.tech.net_rule(net.index()), layer);
        let along = lt.across_along_rect(bbox).1;
        let inside = |n: Node| contains_half_open(area, self.point(n));
        let mut steps: Vec<(u32, u32)> = route
            .edges
            .iter()
            .filter_map(|e| match e {
                Edge::Wire(n) if n.layer == layer => Some(n),
                _ => None,
            })
            .filter(|&n| {
                inside(n)
                    && inside(Node {
                        cross: n.cross + 1,
                        ..n
                    })
            })
            .map(|n| (n.track, n.cross))
            .collect();
        steps.sort_unstable();
        for run in steps.chunk_by(|a, b| a.0 == b.0 && a.1 + 1 == b.1) {
            let (track, first) = run[0];
            let last = run[run.len() - 1].1 + 1;
            let metal = wire_rect(lt.point(track, first), lt.point(track, last), width);
            if !metal.is_some_and(|m| m.touches(bbox)) {
                continue;
            }
            if let Some((from, to)) = marked_crossings(lt, (first, last), along, width) {
                for cross in from..=to {
                    marks.mark_planar(Node {
                        layer,
                        track,
                        cross,
                    });
                }
            }
        }
    }

    /// Marks the crossings under each patch of the route on `layer` that
    /// meets a violation at `bbox`, and the vias above and below them.
    fn mark_patches(
        &self,
        marks: &mut Marks,
        area: &Rect<Dbu>,
        layer: LayerId,
        bbox: &Rect<Dbu>,
        route: &NetRoute,
    ) {
        let lt = self.tech.layer(layer);
        for patch in &route.patches {
            if patch.layer != layer
                || !patch.rect.touches(bbox)
                || !contains_half_open(area, patch.rect.center())
            {
                continue;
            }
            let ((a0, a1), (b0, b1)) = lt.across_along_rect(&patch.rect);
            let first = lt.cross.partition_point(|c| c.at < b0);
            let end = lt.cross.partition_point(|c| c.at <= b1);
            for track in lt.tracks_within(a0, a1) {
                for cross in first..end {
                    let node = Node {
                        layer,
                        track,
                        cross: crate::tech::to_u32(cross),
                    };
                    marks.mark_planar(node);
                    marks.mark_via(node);
                    if let Some(below) = self.node_below(node) {
                        marks.mark_via(below);
                    }
                }
            }
        }
    }

    /// Marks the first via of the route inside `area` whose metal or cut on
    /// `layer` meets a violation at `bbox`.
    fn mark_one_via(
        &self,
        marks: &mut Marks,
        area: &Rect<Dbu>,
        layer: LayerRef,
        bbox: &Rect<Dbu>,
        route: &NetRoute,
    ) {
        let meets = |lower: Node| {
            let at = self.point(lower);
            let via = self.tech.via_above(lower.layer);
            match layer {
                LayerRef::Routing(l) if l == lower.layer => via.bottom.translate(at).touches(bbox),
                LayerRef::Routing(l) if l.index() == lower.layer.index() + 1 => {
                    via.top.translate(at).touches(bbox)
                }
                LayerRef::Routing(_) => false,
                LayerRef::Cut(c) => {
                    c == lower.layer.cut_above()
                        && via.cuts.iter().any(|r| r.translate(at).touches(bbox))
                }
            }
        };
        let first = route.edges.iter().find_map(|e| match e {
            Edge::Via { lower, .. }
                if contains_half_open(area, self.point(lower)) && meets(lower) =>
            {
                Some(lower)
            }
            _ => None,
        });
        if let Some(lower) = first {
            marks.mark_via(lower);
        }
    }

    /// The node on the layer below `node` at the same point, if that point
    /// is a crossing there.
    fn node_below(&self, node: Node) -> Option<Node> {
        if node.layer <= self.tech.min_layer {
            return None;
        }
        let lt = self.tech.layer(node.layer);
        let track = lt.cross[node.cross as usize].down?;
        let cross = lt.track_down[node.track as usize]?;
        Some(Node {
            layer: LayerId::new(node.layer.index() - 1),
            track,
            cross,
        })
    }
}

/// The crossings from `wire.0` to `wire.1` of a wire along `lt`'s track
/// that a violation spanning `along` marks: those within the violation
/// grown by `width`, grown again by `width` until at least two lie within
/// it or it has grown [`MAX_BLOATS`] times.
fn marked_crossings(
    lt: &LayerTech,
    wire: (u32, u32),
    along: (Dbu, Dbu),
    width: Dbu,
) -> Option<(u32, u32)> {
    let mut within = None;
    for bloat in 1..=MAX_BLOATS {
        let (lo, hi) = (along.0 - width * bloat, along.1 + width * bloat);
        let first = crate::tech::to_u32(lt.cross.partition_point(|c| c.at < lo)).max(wire.0);
        let end = crate::tech::to_u32(lt.cross.partition_point(|c| c.at <= hi)).min(wire.1 + 1);
        within = (first < end).then(|| (first, end - 1));
        if end >= first + 2 {
            break;
        }
    }
    within
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tech::RouteTech;
    use pare_common::db::parser::{def, lef};
    use pare_common::util::config::RoutingConfig;
    use std::path::{Path, PathBuf};

    fn fixture(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../common/tests/fixtures")
            .join(name)
    }

    fn tiny_tech() -> RouteTech {
        let units = def::read_units(&fixture("tiny.def")).unwrap();
        let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
        let db = def::parse(&fixture("tiny.def"), library).unwrap();
        RouteTech::new(&db, &RoutingConfig::default()).unwrap()
    }

    #[test]
    fn a_violation_between_two_crossings_marks_both() {
        let tech = tiny_tech();
        let lt = tech.layer(tech.min_layer);
        let (a, b) = (lt.cross[3].at, lt.cross[4].at);
        let gap = (a + Dbu(1), b - Dbu(1));

        let marked = marked_crossings(lt, (0, 8), gap, Dbu(1));

        assert_eq!(marked, Some((3, 4)));
    }

    /// The distance from crossing `i` to its nearer neighbour along `lt`.
    fn nearer_gap(lt: &LayerTech, i: usize) -> Dbu {
        (lt.cross[i].at - lt.cross[i - 1].at).min(lt.cross[i + 1].at - lt.cross[i].at)
    }

    #[test]
    fn a_violation_on_one_crossing_is_grown_until_it_takes_in_a_neighbour() {
        let tech = tiny_tech();
        let lt = tech.layer(tech.min_layer);
        let at = lt.cross[5].at;
        let width = nearer_gap(lt, 5);

        let (from, to) = marked_crossings(lt, (0, 8), (at, at), width).unwrap();

        assert!(from <= 5 && 5 <= to && to > from, "{from}..={to}");
        for i in from..=to {
            assert!((lt.cross[i as usize].at - at).abs() <= width, "{i}");
        }
    }

    #[test]
    fn a_violation_with_no_neighbour_within_reach_marks_its_one_crossing() {
        let tech = tiny_tech();
        let lt = tech.layer(tech.min_layer);
        let at = lt.cross[5].at;

        let marked = marked_crossings(lt, (0, 8), (at, at), Dbu(1));

        assert_eq!(marked, Some((5, 5)));
    }

    #[test]
    fn only_the_wire_s_own_crossings_are_marked() {
        let tech = tiny_tech();
        let lt = tech.layer(tech.min_layer);
        let at = lt.cross[5].at;
        let width = lt.cross[6].at - lt.cross[4].at;

        let (from, _) = marked_crossings(lt, (5, 8), (at, at), width).unwrap();

        assert_eq!(from, 5);
    }
}
