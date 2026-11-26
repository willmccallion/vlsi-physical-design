use super::edge_rules::NO_RULES;
use super::*;
use crate::tech::RouteSetup;
use crate::tech::{Cross, CrossIdx, LayerTech, TrackIdx};
use crate::tests::tiny;
use pare_common::config::RoutingConfig;
use pare_common::db::indices::NetId;
use pare_common::db::library::Direction;

/// Runs on one track between two vias through the same cut, as the run's
/// length and the reach of that cut.
fn returns_through_a_cut(tech: &RouteTech, path: &[Node]) -> Vec<(Dbu, Dbu)> {
    let mut out = Vec::new();
    let mut i = 1;
    while i < path.len() {
        let (from, onto) = (path[i - 1], path[i]);
        if from.layer == onto.layer {
            i += 1;
            continue;
        }
        let mut j = i;
        while j + 1 < path.len()
            && path[j + 1].layer == onto.layer
            && path[j + 1].track == onto.track
        {
            j += 1;
        }
        if path.get(j + 1).is_some_and(|n| n.layer == from.layer) {
            let lt = tech.layer(onto.layer);
            let run = (lt.cross[path[j].cross].at - lt.cross[onto.cross].at).abs();
            out.push((run, tech.via_above(from.layer.min(onto.layer)).cut_reach));
        }
        i = j + 1;
    }
    out
}

#[test]
fn a_global_route_of_no_cells_gives_no_guide() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&RoutingConfig::default())).unwrap();

    let guide = Guide::new(&[], db.die_area.min, Dbu::new(1000), &tech);

    assert!(guide.is_none());
}

#[test]
fn a_path_leaves_a_cut_crowded_by_its_last_via_before_coming_back_through_it() {
    let db = tiny();
    let config = RoutingConfig {
        min_layer: Some("M1".to_string()),
        ..RoutingConfig::default()
    };
    let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let search = Search {
        db: &db,
        tech: &tech,
        occupancy: &occupancy,
        overlay: None,
        marks: &marks,
        owner: Owner::Net(NetId(0)),
        costs: COSTS,
        guide: None,
        own_vias: None,
        rules: &NO_RULES,
        ripped: &[],
    };
    let m1 = LayerId(0);
    let lt = tech.layer(m1);
    let node = |y: i64| Node {
        layer: m1,
        track: TrackIdx::new(lt.tracks.as_slice().binary_search(&Dbu::new(y)).unwrap()),
        cross: CrossIdx::new(
            lt.cross
                .as_slice()
                .partition_point(|c| c.at < Dbu::new(1100)),
        ),
    };
    let targets: FxHashMap<Node, (usize, f64)> = std::iter::once((node(300), (0, 0.0))).collect();

    let (path, _) = search
        .find(&[(node(100), 0.0)], &targets, &db.die_area)
        .unwrap();

    assert_eq!(path.last(), Some(&node(300)));
    for (run, reach) in returns_through_a_cut(&tech, &path) {
        assert!(
            run >= reach,
            "a via returns after {run} of a {reach} reach: {path:?}"
        );
    }
}

/// A search for net 0 of the tiny design on M1 and up, with nothing
/// routed, at `costs`.
fn open_search<'a>(
    db: &'a NetlistDB,
    tech: &'a RouteTech,
    occupancy: &'a Occupancy,
    marks: &'a Marks,
    costs: Costs,
) -> Search<'a> {
    Search {
        db,
        tech,
        occupancy,
        overlay: None,
        marks,
        owner: Owner::Net(NetId(0)),
        costs,
        guide: None,
        own_vias: None,
        rules: &NO_RULES,
        ripped: &[],
    }
}

fn m1_tech(db: &NetlistDB) -> RouteTech {
    let config = RoutingConfig {
        min_layer: Some("M1".to_string()),
        ..RoutingConfig::default()
    };
    RouteTech::new(db, &RouteSetup::from(&config)).unwrap()
}

/// The first crossing of `layer` that `wanted` accepts, on track 1.
fn crossing_where(tech: &RouteTech, layer: LayerId, wanted: impl Fn(&Cross) -> bool) -> Node {
    let cross = tech
        .layer(layer)
        .cross
        .iter_enumerated()
        .find(|(_, c)| wanted(c));
    Node {
        layer,
        track: TrackIdx(1),
        cross: cross.unwrap().0,
    }
}

#[test]
fn a_path_to_the_next_track_jogs_across_when_vias_cost_more() {
    let db = tiny();
    let tech = m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let costs = Costs {
        via: 10_000.0,
        ..COSTS
    };
    let search = open_search(&db, &tech, &occupancy, &marks, costs);
    let from = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
    let to = Node {
        track: TrackIdx(2),
        ..from
    };
    let targets: FxHashMap<Node, (usize, f64)> = std::iter::once((to, (0, 0.0))).collect();

    let (path, _) = search.find(&[(from, 0.0)], &targets, &db.die_area).unwrap();

    assert_eq!(path, [from, to]);
}

#[test]
fn no_jog_leaves_a_crossing_the_layer_above_does_not_cross() {
    let db = tiny();
    let tech = m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
    let n = crossing_where(&tech, LayerId(1), |c| c.up.is_none());
    let mut out = Vec::new();

    search.neighbours(State::start(n), &db.die_area, &mut FreshCosts, &mut out);

    assert!(
        out.iter()
            .all(|(s, _)| s.node.track == n.track || s.node.layer != n.layer)
    );
}

#[test]
fn a_jog_keeps_a_recent_via_only_while_it_lies_within_reach_across() {
    let db = tiny();
    let tech = m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
    let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
    let recent = Some(RecentVia {
        from: LayerId(1),
        track: n.track,
        cross: n.cross,
    });
    let far = tech.layer(n.layer).tracks.len() - 1;

    assert_eq!(search.after_jog(n.layer, recent, n.track), recent);
    assert_eq!(search.after_jog(n.layer, recent, TrackIdx::new(far)), None);
}

const COSTS: Costs = Costs {
    via: 50.0,
    conflict: 8.0,
    marker: 32.0,
    decay: 0.95,
};

/// The tiny design on M1 and up, with another net's wire along M1's
/// track 1 from crossing 1 to crossing 3.
fn beside_a_wire(db: &NetlistDB) -> (RouteTech, Occupancy) {
    let config = RoutingConfig {
        min_layer: Some("M1".to_string()),
        ..RoutingConfig::default()
    };
    let tech = RouteTech::new(db, &RouteSetup::from(&config)).unwrap();
    let lt = tech.layer(LayerId(0));
    let other = wire_rect(
        lt.point(TrackIdx(1), CrossIdx(1)),
        lt.point(TrackIdx(1), CrossIdx(3)),
        lt.width,
    )
    .unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu::new(1000));
    occupancy.add_rect(
        &tech,
        LayerId(0),
        &other,
        Owner::Net(NetId(1)),
        Origin::Wiring,
    );
    (tech, occupancy)
}

fn step(track: u32, cross: u32) -> Node {
    Node {
        layer: LayerId(0),
        track: TrackIdx(track),
        cross: CrossIdx(cross),
    }
}

fn step_length(tech: &RouteTech, lower: Node) -> f64 {
    let lt = tech.layer(lower.layer);
    let (a, b) = (
        lt.point(lower.track, lower.cross),
        lt.point(lower.track, lower.cross.next()),
    );
    a.manhattan(b).to_f64()
}

#[test]
fn a_step_over_another_nets_wire_pays_the_conflict_cost_per_unit_of_length() {
    let db = tiny();
    let (tech, occupancy) = beside_a_wire(&db);
    let marks = Marks::default();
    let search = Search {
        db: &db,
        tech: &tech,
        occupancy: &occupancy,
        overlay: None,
        marks: &marks,
        owner: Owner::Net(NetId(0)),
        costs: COSTS,
        guide: None,
        own_vias: None,
        rules: &NO_RULES,
        ripped: &[],
    };
    let lower = step(1, 1);
    let length = step_length(&tech, lower);

    let price = search.step_price(lower, &mut Total);

    let (conflict, total) = (COSTS.conflict * length, (1.0 + COSTS.conflict) * length);
    assert!((price.conflict - conflict).abs() < 1e-9, "{price:?}");
    assert!((price.total() - total).abs() < 1e-9);
}

#[test]
fn a_free_jog_costs_its_length_and_the_grid_cost_per_unit_of_it() {
    let db = tiny();
    let (tech, _) = beside_a_wire(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let search = Search {
        db: &db,
        tech: &tech,
        occupancy: &occupancy,
        overlay: None,
        marks: &marks,
        owner: Owner::Net(NetId(0)),
        costs: COSTS,
        guide: None,
        own_vias: None,
        rules: &NO_RULES,
        ripped: &[],
    };
    let lower = step(1, 4);
    let lt = tech.layer(lower.layer);
    let length = (lt.tracks[TrackIdx(2)] - lt.tracks[TrackIdx(1)]).to_f64();

    let price = search.jog_price(lower, &mut Total);

    let total = (1.0 + GRID_COST) * length;
    assert!((price.total() - total).abs() < 1e-9, "{price:?}");
}

/// The point `along` the track at `across` of `lt`.
fn across_along_point(lt: &LayerTech, across: Dbu, along: Dbu) -> Point<Dbu> {
    match lt.direction {
        Direction::Horizontal => Point::new(along, across),
        Direction::Vertical => Point::new(across, along),
    }
}

/// The net's own vias: one up from M1 on the track of `jog`, whose pad
/// ends `gap` short of the jog's metal along that track.
fn own_via_short_of(db: &NetlistDB, tech: &RouteTech, jog: Node, gap: Dbu) -> OwnVias {
    let lt = tech.layer(jog.layer);
    let via = tech.via_above(jog.layer);
    let (_, pad) = lt.across_along_rect(&via.bottom);
    let jog_end = lt.cross[jog.cross].at + lt.width / 2;
    let at = across_along_point(lt, lt.tracks[jog.track], jog_end + gap - pad.lo);
    let mut own = OwnVias::new(db, tech, None);
    own.add(via.via, at);
    own
}

#[test]
fn a_jog_whose_metal_crowds_the_nets_own_via_pays_a_conflict_over_its_length() {
    let db = tiny();
    let tech = m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let jog = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
    let own = own_via_short_of(&db, &tech, jog, tech.layer(jog.layer).spacing / 2);
    let search = Search {
        own_vias: Some(&own),
        ..open_search(&db, &tech, &occupancy, &marks, COSTS)
    };
    let lt = tech.layer(jog.layer);
    let jogged = Node {
        track: jog.track.plus(1),
        ..jog
    };
    let length = (lt.tracks[jogged.track] - lt.tracks[jog.track]).to_f64();
    let mut out = Vec::new();

    search.neighbours(State::start(jog), &db.die_area, &mut FreshCosts, &mut out);

    let (_, cost) = out.iter().find(|(s, _)| s.node == jogged).unwrap();
    let expected = (1.0 + GRID_COST + COSTS.conflict) * length;
    assert!((cost - expected).abs() < 1e-9, "{cost} against {expected}");
}

#[test]
fn a_jog_whose_metal_overlaps_the_nets_own_via_pays_nothing_for_it() {
    let db = tiny();
    let tech = m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let jog = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
    let overlap = tech.layer(jog.layer).width;
    let own = own_via_short_of(&db, &tech, jog, -overlap);
    let search = Search {
        own_vias: Some(&own),
        ..open_search(&db, &tech, &occupancy, &marks, COSTS)
    };

    assert!(search.own_jog_cost(jog).abs() < 1e-9);
}

#[test]
fn a_jog_stopping_a_track_short_of_the_nets_own_via_pays_nothing_for_it() {
    let db = tiny();
    let tech = m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let jog = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
    let lt = tech.layer(jog.layer);
    let via = tech.via_above(jog.layer);
    let at = lt.point(jog.track.plus(2), jog.cross);
    let mut own = OwnVias::new(&db, &tech, None);
    own.add(via.via, at);
    let (a, b) = (
        lt.point(jog.track, jog.cross),
        lt.point(jog.track.plus(1), jog.cross),
    );
    let metal = wire_rect(a, b, lt.width).unwrap();
    assert!(too_close(lt.spacing, &metal, &via.bottom.translate(at)));
    let search = Search {
        own_vias: Some(&own),
        ..open_search(&db, &tech, &occupancy, &marks, COSTS)
    };

    assert!(search.own_jog_cost(jog).abs() < 1e-9);
}

/// The jogs `search` offers from `node` just after a via from M2 landed
/// at `via_cross` on its track.
fn jogs_after_a_via(search: &Search<'_>, node: Node, via_cross: CrossIdx) -> Vec<Node> {
    let state = State {
        node,
        heading: Heading::Still,
        recent_via: Some(RecentVia {
            from: LayerId(1),
            track: node.track,
            cross: via_cross,
        }),
        recent_jog: None,
    };
    let mut out = Vec::new();
    search.neighbours(state, &search.db.die_area, &mut FreshCosts, &mut out);
    out.into_iter()
        .map(|(s, _)| s.node)
        .filter(|m| m.layer == node.layer && m.track != node.track)
        .collect()
}

#[test]
fn no_jog_leaves_where_its_metal_would_end_short_of_the_last_via_along_the_track() {
    let db = tiny();
    let tech = wide_m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
    let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
    let lt = tech.layer(n.layer);
    let pad = search.pad(n.layer, LayerId(1));
    let via_metal = pad.translate(lt.point(n.track, n.cross.next()));
    let jog_metal = search.jog_metal(n).unwrap();
    assert!(too_close_along(lt, lt.spacing, &jog_metal, &via_metal));

    let jogs = jogs_after_a_via(&search, n, n.cross.next());

    assert_eq!(jogs, []);
}

#[test]
fn a_jog_leaves_from_the_crossing_the_last_via_landed_on() {
    let db = tiny();
    let tech = m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
    let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());

    let jogs = jogs_after_a_via(&search, n, n.cross);

    assert!(jogs.contains(&Node {
        track: n.track.plus(1),
        ..n
    }));
}

/// The tiny design on M1 and up, M1 spacing half as wide again, so that
/// metal one crossing along a track ends short of the spacing.
fn wide_m1_tech(db: &NetlistDB) -> RouteTech {
    let mut tech = m1_tech(db);
    let m1 = &mut tech.layers[LayerId(0)];
    m1.spacing = m1.spacing * 3 / 2;
    tech
}

/// Whether `search` offers a via up from `node`, reached by a step from
/// the jog the path took at crossing `jog` of its track.
fn via_up_after_a_jog(search: &Search<'_>, node: Node, jog: CrossIdx) -> bool {
    let state = State {
        node,
        heading: Heading::Down,
        recent_via: None,
        recent_jog: Some(jog),
    };
    search.via_move(state, Climb::Up, &mut FreshCosts).is_some()
}

#[test]
fn no_via_leaves_where_its_metal_would_end_short_of_the_last_jog_along_the_track() {
    let db = tiny();
    let tech = wide_m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
    let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
    let lt = tech.layer(n.layer);
    let (jog_along, via_along) = (lt.cross[n.cross.next()].at, lt.cross[n.cross].at);
    assert!(search.via_crowds_jog(n.layer, LayerId(1), jog_along, via_along));

    assert!(!via_up_after_a_jog(&search, n, n.cross.next()));
}

#[test]
fn a_via_leaves_from_the_crossing_the_last_jog_ran_across() {
    let db = tiny();
    let tech = wide_m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let search = open_search(&db, &tech, &occupancy, &marks, COSTS);
    let n = crossing_where(&tech, LayerId(0), |c| c.up.is_some());

    assert!(via_up_after_a_jog(&search, n, n.cross));
}

#[test]
fn a_jog_is_remembered_only_where_a_via_ahead_could_crowd_it() {
    let db = tiny();
    let (plain, wide) = (m1_tech(&db), wide_m1_tech(&db));
    let occupancy = Occupancy::new(&plain, Dbu::new(1000));
    let marks = Marks::default();
    let n = crossing_where(&plain, LayerId(0), |c| c.up.is_some());
    let held = |tech: &RouteTech| {
        let search = open_search(&db, tech, &occupancy, &marks, COSTS);
        search.hold_jog(n.layer, n.cross, n.cross)
    };

    assert_eq!((held(&plain), held(&wide)), (None, Some(n.cross)));
}

#[test]
fn a_hashed_visit_is_found_again_whatever_recent_jog_it_carried() {
    let node = step(1, 3);
    let carrying = |recent_jog| State {
        node,
        heading: Heading::Up,
        recent_via: None,
        recent_jog,
    };
    let mut visits = HashVisits::default();

    visits.record(carrying(None), 5.0, None);

    assert_eq!(visits.best(carrying(Some(CrossIdx(2)))), Some(5.0));
}

#[test]
fn explaining_a_jog_counts_its_crowding_the_nets_own_via() {
    let db = tiny();
    let tech = m1_tech(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let marks = Marks::default();
    let jog = crossing_where(&tech, LayerId(0), |c| c.up.is_some());
    let own = own_via_short_of(&db, &tech, jog, tech.layer(jog.layer).spacing / 2);
    let search = Search {
        own_vias: Some(&own),
        ..open_search(&db, &tech, &occupancy, &marks, COSTS)
    };
    let paid = search.jog_price(jog, &mut Total).total() + search.own_jog_cost(jog);

    let explanation = search.explain(&[Edge::Jog(jog)]);

    assert!(explanation.own_vias > 0.0, "{explanation:?}");
    assert!((explanation.total() - paid).abs() < 1e-9);
}

#[test]
fn a_step_over_wiring_being_routed_again_costs_only_its_length() {
    let db = tiny();
    let (tech, occupancy) = beside_a_wire(&db);
    let marks = Marks::default();
    let search = Search {
        db: &db,
        tech: &tech,
        occupancy: &occupancy,
        overlay: None,
        marks: &marks,
        owner: Owner::Net(NetId(0)),
        costs: COSTS,
        guide: None,
        own_vias: None,
        rules: &NO_RULES,
        ripped: &[Owner::Net(NetId(1))],
    };
    let lower = step(1, 1);

    let price = search.step_price(lower, &mut Total);

    assert!(
        (price.total() - step_length(&tech, lower)).abs() < 1e-9,
        "{price:?}"
    );
}

#[test]
fn a_step_from_a_marked_crossing_pays_the_marker_cost_per_unit_of_length() {
    let db = tiny();
    let (tech, _) = beside_a_wire(&db);
    let occupancy = Occupancy::new(&tech, Dbu::new(1000));
    let mut marks = Marks::default();
    marks.mark_planar(step(4, 2));
    let search = Search {
        db: &db,
        tech: &tech,
        occupancy: &occupancy,
        overlay: None,
        marks: &marks,
        owner: Owner::Net(NetId(0)),
        costs: COSTS,
        guide: None,
        own_vias: None,
        rules: &NO_RULES,
        ripped: &[],
    };
    let (into, away) = (step(4, 1), step(4, 3));

    let (into_mark, beyond) = (
        search.step_price(into, &mut Total),
        search.step_price(away, &mut Total),
    );

    let marker = COSTS.marker * step_length(&tech, into);
    assert!((into_mark.marker - marker).abs() < 1e-9, "{into_mark:?}");
    assert!(beyond.marker.abs() < 1e-9, "{beyond:?}");
}

#[test]
fn explaining_a_route_names_whose_wiring_it_meets_and_adds_up_to_its_cost() {
    let db = tiny();
    let (tech, occupancy) = beside_a_wire(&db);
    let marks = Marks::default();
    let search = Search {
        db: &db,
        tech: &tech,
        occupancy: &occupancy,
        overlay: None,
        marks: &marks,
        owner: Owner::Net(NetId(0)),
        costs: COSTS,
        guide: None,
        own_vias: None,
        rules: &NO_RULES,
        ripped: &[],
    };
    let edges = [Edge::Wire(step(1, 0)), Edge::Wire(step(1, 1))];
    let paid: f64 = edges
        .iter()
        .map(|e| match *e {
            Edge::Wire(n) => FreshCosts.wire(&search, n),
            Edge::Via { lower, upper } => FreshCosts.via(&search, lower, upper),
            Edge::Jog(lower) => search.jog_price(lower, &mut Total).total(),
        })
        .sum();

    let explanation = search.explain(&edges);

    assert_eq!(explanation.met, vec![Owner::Net(NetId(1))]);
    assert!(
        (explanation.total() - paid).abs() < 1e-9,
        "{explanation:?} against {paid}"
    );
}
