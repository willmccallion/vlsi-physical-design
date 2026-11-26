use super::*;
use crate::runs::{EdgeSet, Run};
use crate::tech::RouteSetup;
use crate::tech::RouteTech;
use crate::tech::{CrossIdx, TrackIdx};
use pare_common::db::design::{NetlistDB, NonDefaultLayer, NonDefaultRule};
use pare_common::db::indices::LayerId;

/// The placed fixture with net 1 under a wide non-default rule.
fn tiny_with_a_rule_on_net_1() -> NetlistDB {
    let (mut db, _) = crate::tests::placed_tiny();
    let rule = db.non_default_rules.push(NonDefaultRule {
        name: "DW".to_string(),
        hard_spacing: true,
        layers: vec![NonDefaultLayer {
            layer: LayerId(1),
            width: Dbu::new(200),
            spacing: Some(Dbu::new(300)),
            wire_extension: None,
        }],
        vias: Vec::new(),
        via_rules: Vec::new(),
        min_cuts: Vec::new(),
    });
    db.nets[NetId::new(1)].non_default_rule = Some(rule);
    db
}

/// A judgment of one marker between `a` and `b`.
fn marker_between(a: NetId, b: NetId) -> Judgment {
    Judgment {
        owners: vec![[Some(a), Some(b)]],
        ..Judgment::default()
    }
}

#[test]
fn a_net_under_a_rule_is_spared_for_three_rounds_then_rerouted() {
    let db = tiny_with_a_rule_on_net_1();
    let (_, config) = crate::tests::placed_tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
    let dr = DetailedRouter::new(&db, &tech, crate::gcell_size(&tech, &config));
    let (ruled, plain) = (NetId::new(1), NetId::new(2));
    let judgment = marker_between(ruled, plain);
    let mut avoided = FxHashMap::default();

    let spared: Vec<FxHashSet<NetId>> = (0..NDR_RIPUP_HARDINESS)
        .map(|_| dr.to_reroute(&judgment, &mut avoided))
        .collect();
    let then = dr.to_reroute(&judgment, &mut avoided);

    assert!(spared.iter().all(|r| *r == FxHashSet::from_iter([plain])));
    assert_eq!(then, FxHashSet::from_iter([ruled, plain]));
    assert!(avoided.is_empty());
}

#[test]
fn a_net_under_a_rule_alone_in_a_marker_is_rerouted() {
    let db = tiny_with_a_rule_on_net_1();
    let (_, config) = crate::tests::placed_tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
    let dr = DetailedRouter::new(&db, &tech, crate::gcell_size(&tech, &config));
    let ruled = NetId::new(1);

    let reroute = dr.to_reroute(&marker_between(ruled, ruled), &mut FxHashMap::default());

    assert_eq!(reroute, FxHashSet::from_iter([ruled]));
}

/// The placed fixture's router with net 0 routed along M2 track 5 from
/// crossing 0 to `to`, and the area from crossing 4 to crossing 8.
fn router_with_a_run_to(to: u32, f: impl FnOnce(&DetailedRouter<'_>, &Rect<Dbu>)) {
    let (db, config) = crate::tests::placed_tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
    let mut dr = DetailedRouter::new(&db, &tech, crate::gcell_size(&tech, &config));
    let layer = LayerId(1);
    let run = Run {
        layer,
        track: TrackIdx(5),
        from: CrossIdx(0),
        to: CrossIdx(to),
    };
    dr.routes[NetId::new(0)] = NetRoute {
        edges: EdgeSet::from_runs([run]),
        ..NetRoute::default()
    };
    let lt = tech.layer(layer);
    let area = Rect::new(
        Point::new(Dbu::new(0), lt.cross[CrossIdx(4)].at),
        Point::new(db.die_area.max.x, lt.cross[CrossIdx(8)].at),
    );
    f(&dr, &area);
}

fn m2_track_5(cross: u32) -> Node {
    Node {
        layer: LayerId(1),
        track: TrackIdx(5),
        cross: CrossIdx(cross),
    }
}

#[test]
fn carving_a_run_through_an_area_leaves_a_piece_joining_both_sides() {
    router_with_a_run_to(12, |dr, area| {
        let carved = dr.carve(NetId::new(0), std::slice::from_ref(area));

        let (removed, piece) = &carved[0];
        let groups = piece.as_ref().map(|p| p.groups.clone());
        assert_eq!(groups, Some(vec![vec![m2_track_5(4)], vec![m2_track_5(7)]]));
        assert_eq!(removed.edges.len(), 3);
    });
}

#[test]
fn carving_a_branch_that_ends_in_an_area_leaves_nothing_to_route() {
    router_with_a_run_to(6, |dr, area| {
        let carved = dr.carve(NetId::new(0), std::slice::from_ref(area));

        let (removed, piece) = &carved[0];
        assert!(piece.is_none());
        assert_eq!(removed.edges.len(), 2);
    });
}

#[test]
fn a_nets_own_vias_leave_out_those_the_worker_ripped_up() {
    let (db, config) = crate::tests::placed_tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
    let mut dr = DetailedRouter::new(&db, &tech, crate::gcell_size(&tech, &config));
    let net = NetId::new(0);
    let lt = tech.layer(LayerId(1));
    let lower = (4..8)
        .map(m2_track_5)
        .find(|n| lt.cross[n.cross].up.is_some())
        .unwrap();
    let upper = Node {
        layer: LayerId(2),
        track: lt.cross[lower.cross].up.unwrap(),
        cross: lt.track_up[lower.track].unwrap(),
    };
    let edge = Edge::Via { lower, upper };
    let mut edges = EdgeSet::default();
    edges.extend([edge]);
    dr.commit(
        net,
        &NetRoute {
            edges,
            ..NetRoute::default()
        },
    );
    let area = Rect::new(
        Point::new(Dbu::new(0), lt.cross[CrossIdx(4)].at),
        Point::new(db.die_area.max.x, lt.cross[CrossIdx(8)].at),
    );
    let (via, at) = dr.via_of(edge).unwrap();
    let widest = db
        .library
        .via(via)
        .shapes
        .iter()
        .map(|s| s.rect.width())
        .max()
        .unwrap();
    let beside = at + Point::new(widest + Dbu::new(1), Dbu::ZERO);
    let (removed, _) = dr.carve(net, std::slice::from_ref(&area)).pop().unwrap();
    let crowded = |taken: &Wiring| {
        dr.vias_near(net, &area, taken)
            .crowded(&db, &tech, via, beside)
    };

    let (kept, ripped) = (crowded(&Wiring::default()), crowded(&removed));

    assert_eq!((kept, ripped), (1, 0));
}

#[test]
fn a_region_holds_its_lower_left_edges_but_not_its_upper_right_ones() {
    let r = Rect::new(
        Point::new(Dbu::new(0), Dbu::new(0)),
        Point::new(Dbu::new(10), Dbu::new(10)),
    );

    assert!(contains_half_open(&r, Point::new(Dbu::new(0), Dbu::new(9))));
    assert!(!contains_half_open(
        &r,
        Point::new(Dbu::new(10), Dbu::new(5))
    ));
}
