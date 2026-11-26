use super::*;
use crate::access::Ways;
use crate::tech::RouteSetup;
use pare_common::config::RoutingConfig;
use pare_common::geom::point::Point;

fn node(layer: u8, track: u32, cross: u32) -> Node {
    Node {
        layer: LayerId(layer),
        track: TrackIdx(track),
        cross: CrossIdx(cross),
    }
}

#[test]
fn neighbours_on_a_track_across_tracks_or_a_layer_apart_are_joined_by_one_edge() {
    assert_eq!(
        Edge::between(node(1, 4, 7), node(1, 4, 6)),
        Some(Edge::Wire(node(1, 4, 6)))
    );
    assert_eq!(
        Edge::between(node(2, 9, 3), node(1, 4, 7)),
        Some(Edge::Via {
            lower: node(1, 4, 7),
            upper: node(2, 9, 3)
        })
    );
    assert_eq!(
        Edge::between(node(1, 5, 7), node(1, 4, 7)),
        Some(Edge::Jog(node(1, 4, 7)))
    );
    assert_eq!(Edge::between(node(1, 4, 7), node(1, 6, 7)), None);
    assert_eq!(Edge::between(node(1, 4, 7), node(1, 5, 8)), None);
}

#[test]
fn a_jog_joins_its_crossing_to_the_same_crossing_of_the_next_track() {
    assert_eq!(
        Edge::Jog(node(1, 4, 7)).ends(),
        (node(1, 4, 7), node(1, 5, 7))
    );
}

#[test]
fn jogs_across_consecutive_tracks_at_one_crossing_make_one_run() {
    let jogs = [node(1, 5, 7), node(1, 4, 7), node(1, 7, 7), node(1, 4, 8)];

    assert_eq!(
        jog_runs(&jogs),
        [
            (node(1, 4, 7), node(1, 5, 7)),
            (node(1, 7, 7), node(1, 7, 7)),
            (node(1, 4, 8), node(1, 4, 8)),
        ]
    );
}

#[test]
fn only_gaps_narrower_than_the_spacing_are_reported() {
    let span = |lo, hi| Interval::new(Dbu::new(lo), Dbu::new(hi));
    let spans = vec![span(0, 100), span(150, 300), span(280, 400), span(600, 700)];

    assert_eq!(narrow_gaps(spans, Dbu::new(70)), [span(100, 150)]);
}

#[test]
fn joined_crossings_share_a_root() {
    let mut sets = DisjointSets::default();

    sets.union(node(1, 4, 1), node(1, 4, 2));
    sets.union(node(1, 4, 3), node(1, 4, 2));

    assert_eq!(sets.find(node(1, 4, 3)), sets.find(node(1, 4, 1)));
    assert_ne!(sets.find(node(1, 4, 4)), sets.find(node(1, 4, 1)));
}

/// A route of the tiny fixture running along one second-layer track
/// from crossing 7 to crossing 9, reaching a pin through `access`.
fn run_to_access(tech: &RouteTech, access: AccessPoint) -> Vec<RouteElem> {
    let run = Run {
        layer: LayerId(1),
        track: TrackIdx(5),
        from: CrossIdx(7),
        to: CrossIdx(9),
    };
    let route = NetRoute {
        edges: EdgeSet::from_runs([run]),
        access: vec![access],
        ..NetRoute::default()
    };
    route.elems(tech)
}

#[test]
fn a_wire_ending_where_an_access_point_touches_its_pin_stops_at_the_node() {
    let db = crate::tests::tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();
    let point = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
    let (on_pin, far) = (node(1, 5, 7), node(1, 5, 9));
    let access = AccessPoint {
        node: on_pin,
        at: point(on_pin),
        pin_layer: LayerId(1),
        vias: Vec::new(),
        ways: Ways::ALL,
    };

    let elems = run_to_access(&tech, access);

    let wire = RouteElem::Wire {
        layer: LayerId(1),
        from: point(on_pin),
        to: point(far),
        extensions: (Some(Dbu::ZERO), None),
    };
    assert!(elems.contains(&wire), "{elems:?}");
}

#[test]
fn a_wire_ending_at_an_access_point_with_a_stub_keeps_its_extension() {
    let db = crate::tests::tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();
    let point = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
    let (node_end, far) = (node(1, 5, 7), node(1, 5, 9));
    let access = AccessPoint {
        node: node_end,
        at: point(node_end) - Point::new(Dbu::ZERO, Dbu::new(50)),
        pin_layer: LayerId(1),
        vias: Vec::new(),
        ways: Ways::ALL,
    };

    let elems = run_to_access(&tech, access);

    let wire = RouteElem::Wire {
        layer: LayerId(1),
        from: point(node_end),
        to: point(far),
        extensions: (None, None),
    };
    assert!(elems.contains(&wire), "{elems:?}");
}

#[test]
fn a_path_that_doubles_back_uses_each_step_once() {
    let path = [node(1, 4, 5), node(1, 4, 6), node(1, 4, 7), node(1, 4, 6)];

    let mut edges: Vec<Edge> = path_edges(&path).collect();
    edges.sort_unstable();
    edges.dedup();

    assert_eq!(
        edges,
        [Edge::Wire(node(1, 4, 5)), Edge::Wire(node(1, 4, 6))]
    );
}
