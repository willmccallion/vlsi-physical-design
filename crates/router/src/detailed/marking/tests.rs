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
