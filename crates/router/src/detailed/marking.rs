//! Where a violation marks the track graph, as `TritonRoute` adds marker
//! cost (`initMazeCost_marker_route_queue_addHistoryCost` in `OpenROAD`'s
//! `src/drt/src/dr/FlexDR_init.cpp`). The marks go on the metal that takes
//! part in the violation, not on the empty space around it: on each wire
//! of a violating net that meets the violation, the crossings within the
//! violation grown by the wire's width, grown again until at least two are
//! taken in; both ends of each jog of a violating net that meets it; one
//! via of each violating net that meets it; and the
//! crossings under a patch that meets it, with the vias above and below
//! them. Only wiring inside the worker's region is marked, since only that
//! can move.

use super::DetailedRouter;
use super::region::contains_half_open;
use crate::access::Node;
use crate::search::Marks;
use crate::tech::LayerTech;
use crate::tech::{CrossIdx, IndexRange, TrackIdx};
use crate::wiring::{Edge, NetRoute};
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::library::LayerRef;
use pare_common::db::shapes::wire_rect;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::Interval;
use pare_common::geom::rect::Rect;

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
                self.mark_jogs(marks, area, l, bbox, net, route);
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
        let width = self.tech.wire_width(self.tech.net_rule(net), layer);
        let along = lt.across_along_rect(bbox).1;
        let inside = |n: Node| contains_half_open(area, self.point(n));
        let mut steps: Vec<(TrackIdx, CrossIdx)> = route
            .edges
            .iter()
            .filter_map(|e| match e {
                Edge::Wire(n) if n.layer == layer => Some(n),
                _ => None,
            })
            .filter(|&n| {
                inside(n)
                    && inside(Node {
                        cross: n.cross.next(),
                        ..n
                    })
            })
            .map(|n| (n.track, n.cross))
            .collect();
        steps.sort_unstable();
        for run in steps.chunk_by(|a, b| a.0 == b.0 && a.1.next() == b.1) {
            let (track, first) = run[0];
            let last = run[run.len() - 1].1.next();
            let metal = wire_rect(lt.point(track, first), lt.point(track, last), width);
            if !metal.is_some_and(|m| m.touches(bbox)) {
                continue;
            }
            if let Some((from, to)) = marked_crossings(lt, (first, last), along, width) {
                for cross in IndexRange::through(from, to) {
                    marks.mark_planar(Node {
                        layer,
                        track,
                        cross,
                    });
                }
            }
        }
    }

    /// Marks both ends of each of `net`'s jogs on `layer` inside `area`
    /// whose metal meets a violation at `bbox`.
    fn mark_jogs(
        &self,
        marks: &mut Marks,
        area: &Rect<Dbu>,
        layer: LayerId,
        bbox: &Rect<Dbu>,
        net: NetId,
        route: &NetRoute,
    ) {
        let width = self.tech.wire_width(self.tech.net_rule(net), layer);
        for &lower in route.edges.jogs().iter().filter(|n| n.layer == layer) {
            let (a, b) = Edge::Jog(lower).ends();
            let (pa, pb) = (self.point(a), self.point(b));
            let inside = contains_half_open(area, pa) && contains_half_open(area, pb);
            if inside && wire_rect(pa, pb, width).is_some_and(|m| m.touches(bbox)) {
                marks.mark_planar(a);
                marks.mark_planar(b);
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
            let (Interval { lo: a0, hi: a1 }, Interval { lo: b0, hi: b1 }) =
                lt.across_along_rect(&patch.rect);
            let crosses = lt.cross_within(b0, b1);
            for track in lt.tracks_within(a0, a1) {
                for cross in crosses.clone() {
                    let node = Node {
                        layer,
                        track,
                        cross,
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
        let track = lt.cross[node.cross].down?;
        let cross = lt.track_down[node.track]?;
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
    wire: (CrossIdx, CrossIdx),
    along: Interval,
    width: Dbu,
) -> Option<(CrossIdx, CrossIdx)> {
    let mut within = None;
    for bloat in 1..=MAX_BLOATS {
        let near = lt.cross_within(along.lo - width * bloat, along.hi + width * bloat);
        let first = near.start.max(wire.0);
        let end = near.end.min(wire.1.next());
        within = end
            .prev()
            .filter(|&last| first <= last)
            .map(|last| (first, last));
        if end.steps_from(first).is_some_and(|steps| steps >= 2) {
            break;
        }
    }
    within
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tech::RouteSetup;
    use crate::tech::RouteTech;
    use crate::tests::tiny;
    use pare_common::config::RoutingConfig;
    use pare_common::geom::point::Point;

    #[test]
    fn a_violation_on_a_jog_marks_both_its_ends() {
        let (db, config) = crate::tests::placed_tiny();
        let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
        let dr = DetailedRouter::new(&db, &tech, crate::gcell_size(&tech, &config));
        let layer = tech.min_layer;
        let cross = tech
            .layer(layer)
            .cross
            .iter_enumerated()
            .find(|(_, c)| c.up.is_some());
        let lower = Node {
            layer,
            track: TrackIdx(1),
            cross: cross.unwrap().0,
        };
        let route = NetRoute {
            edges: std::iter::once(Edge::Jog(lower)).collect(),
            ..NetRoute::default()
        };
        let (a, b) = Edge::Jog(lower).ends();
        let (pa, pb) = (dr.point(a), dr.point(b));
        let middle = Point::new(pa.x.midpoint(pb.x), pa.y.midpoint(pb.y));
        let mut marks = Marks::default();

        dr.mark_violation(
            &mut marks,
            &db.die_area,
            LayerRef::Routing(layer),
            &Rect::new(middle, middle),
            &[(NetId(0), &route)],
        );

        assert!(marks.planar(a) && marks.planar(b));
    }

    /// A wire over the crossings the tests look at.
    const WHOLE_WIRE: (CrossIdx, CrossIdx) = (CrossIdx(0), CrossIdx(8));

    fn tiny_tech() -> RouteTech {
        RouteTech::new(&tiny(), &RouteSetup::from(&RoutingConfig::default())).unwrap()
    }

    #[test]
    fn a_violation_between_two_crossings_marks_both() {
        let tech = tiny_tech();
        let lt = tech.layer(tech.min_layer);
        let (a, b) = (lt.cross[CrossIdx(3)].at, lt.cross[CrossIdx(4)].at);
        let gap = Interval::new(a + Dbu::new(1), b - Dbu::new(1));

        let marked = marked_crossings(lt, WHOLE_WIRE, gap, Dbu::new(1));

        assert_eq!(marked, Some((CrossIdx(3), CrossIdx(4))));
    }

    /// The distance from crossing `i` to its nearer neighbour along `lt`.
    fn nearer_gap(lt: &LayerTech, i: CrossIdx) -> Dbu {
        let at = |c: CrossIdx| lt.cross[c].at;
        let before = i.prev().expect("a crossing before");
        (at(i) - at(before)).min(at(i.next()) - at(i))
    }

    #[test]
    fn a_violation_on_one_crossing_is_grown_until_it_takes_in_a_neighbour() {
        let tech = tiny_tech();
        let lt = tech.layer(tech.min_layer);
        let at = lt.cross[CrossIdx(5)].at;
        let width = nearer_gap(lt, CrossIdx(5));

        let (from, to) = marked_crossings(lt, WHOLE_WIRE, Interval::new(at, at), width).unwrap();

        assert!(from.0 <= 5 && 5 <= to.0 && to > from, "{from:?}..={to:?}");
        for i in IndexRange::through(from, to) {
            assert!((lt.cross[i].at - at).abs() <= width, "{i:?}");
        }
    }

    #[test]
    fn a_violation_with_no_neighbour_within_reach_marks_its_one_crossing() {
        let tech = tiny_tech();
        let lt = tech.layer(tech.min_layer);
        let at = lt.cross[CrossIdx(5)].at;

        let marked = marked_crossings(lt, WHOLE_WIRE, Interval::new(at, at), Dbu::new(1));

        assert_eq!(marked, Some((CrossIdx(5), CrossIdx(5))));
    }

    #[test]
    fn only_the_wire_s_own_crossings_are_marked() {
        let tech = tiny_tech();
        let lt = tech.layer(tech.min_layer);
        let at = lt.cross[CrossIdx(5)].at;
        let width = lt.cross[CrossIdx(6)].at - lt.cross[CrossIdx(4)].at;
        let wire = (CrossIdx(5), CrossIdx(8));

        let (from, _) = marked_crossings(lt, wire, Interval::new(at, at), width).unwrap();

        assert_eq!(from, CrossIdx(5));
    }
}
