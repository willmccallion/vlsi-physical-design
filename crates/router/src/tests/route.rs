//! Routing entry checks on the shared fixture design.

use super::{TINY_ROUNDS, placed_tiny, tiny};
use crate::access::Node;
use crate::detailed::RestoreRoutesError;
use crate::global::{GCell, GlobalGrid, GlobalNet, route_global, track_demand};
use crate::occupancy::{FixedTracks, Occupancy, Origin, Owner};
use crate::runs::{EdgeSet, Run};
use crate::tech::RouteTech;
use crate::wiring::{Edge, NetRoute};
use crate::{
    GlobalRouteEstimator, RouteError, RouteState, RunOptions, Save, SaveError, SavePoint,
    capacity_reduction, route, route_with, rudy_wire_width,
};
use pare_common::config::{CapacityReserve, RoutingConfig, Share};
use pare_common::db::design::{
    NetlistDB, NonDefaultLayer, NonDefaultRule, PlacementStatus, RouteElem,
};
use pare_common::db::indices::{CellId, CutLayerId, IdVec, LayerId, NetId, NonDefaultRuleId};
use pare_common::db::library::{Direction, LayerRef, Shape};
use pare_common::db::shapes::fixed_shapes;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::routing::net_components;
use std::num::NonZeroU32;

#[test]
fn routing_refuses_a_design_with_unplaced_cells() {
    let mut db = tiny();

    let result = route(&mut db, &RoutingConfig::default());

    assert_eq!(
        result,
        Err(RouteError::UnplacedCells {
            count: 2,
            first: "u5".to_string(),
        })
    );
}

#[test]
fn routing_refuses_layers_that_all_run_one_way() {
    let (mut db, placed) = placed_tiny();
    let config = RoutingConfig {
        min_layer: Some("M2".to_string()),
        max_layer: Some("M2".to_string()),
        ..placed
    };

    let result = route(&mut db, &config).map(|_| ());

    assert_eq!(result, Err(RouteError::OneDirection(Direction::Vertical)));
}

#[test]
fn routing_refuses_a_lowest_layer_of_a_single_track() {
    let (mut db, config) = placed_tiny();
    let m2 = db.library.routing_layer_id("M2").unwrap();
    for t in db.tracks.iter_mut().filter(|t| t.layers.contains(&m2)) {
        t.count = 1;
    }

    let result = route(&mut db, &config).map(|_| ());

    assert_eq!(result, Err(RouteError::OneTrack("M2".to_string())));
}

#[test]
fn nets_crossing_many_regions_are_routed_connected() {
    let (mut db, config) = placed_tiny();

    let summary = route(&mut db, &config).unwrap();

    assert!(summary.incomplete.is_empty(), "{:?}", summary.incomplete);
    for n in 0..db.nets.len() {
        assert_eq!(net_components(&db, NetId::new(n)), 1, "net {n}");
    }
}

/// Routes the placed fixture, saving its state after `point`.
fn route_saving(point: SavePoint) -> (Vec<Vec<RouteElem>>, RouteState) {
    let (mut db, config) = placed_tiny();
    let mut saved = None;
    let mut write = |state: &RouteState| {
        saved = Some(state.clone());
        Ok(())
    };
    let run = RunOptions {
        save: Some(Save {
            after: point,
            write: &mut write,
        }),
        ..RunOptions::default()
    };
    let _ = route_with(&mut db, &config, run).unwrap();
    let wiring = db.nets.iter().map(|n| n.routing.clone()).collect();
    (wiring, saved.unwrap())
}

/// Routes the placed fixture from `state`.
fn route_resuming(state: &RouteState) -> Vec<Vec<RouteElem>> {
    let (mut db, config) = placed_tiny();
    let run = RunOptions {
        resume: Some(state),
        ..RunOptions::default()
    };
    let _ = route_with(&mut db, &config, run).unwrap();
    db.nets.iter().map(|n| n.routing.clone()).collect()
}

#[test]
fn a_run_resumed_after_global_routing_routes_as_the_run_that_saved_it() {
    let (straight, state) = route_saving(SavePoint::Global);

    let resumed = route_resuming(&state);

    assert!(!state.has_detailed());
    assert_eq!(resumed, straight);
}

#[test]
fn a_run_resumed_after_the_first_pass_routes_as_the_run_that_saved_it() {
    let (straight, state) = route_saving(SavePoint::Round(0));

    let resumed = route_resuming(&state);

    assert_eq!(state.round(), Some(0));
    assert_eq!(resumed, straight);
}

#[test]
fn a_run_resumed_after_a_stubborn_round_routes_as_the_run_that_saved_it() {
    let (straight, state) = route_saving(SavePoint::Round(4));

    let resumed = route_resuming(&state);

    assert_eq!(state.round(), Some(4));
    assert_eq!(resumed, straight);
}

#[test]
fn a_run_resumed_after_its_last_round_counts_the_rounds_before_it() {
    let (_, state) = route_saving(SavePoint::Round(TINY_ROUNDS));
    let (mut db, config) = placed_tiny();

    let summary = route_with(
        &mut db,
        &config,
        RunOptions {
            resume: Some(&state),
            ..RunOptions::default()
        },
    )
    .unwrap();

    assert_eq!(summary.rounds, TINY_ROUNDS);
}

#[test]
fn a_save_that_fails_stops_the_run_with_its_error() {
    let (mut db, config) = placed_tiny();
    let mut write = |_: &RouteState| Err(SaveError("disk full".to_string()));
    let run = RunOptions {
        save: Some(Save {
            after: SavePoint::Global,
            write: &mut write,
        }),
        ..RunOptions::default()
    };

    let result = route_with(&mut db, &config, run);

    assert_eq!(
        result.map(|_| ()),
        Err(RouteError::Save(SaveError("disk full".to_string())))
    );
}

#[test]
fn a_save_point_the_run_never_reaches_saves_nothing_and_says_so() {
    let (mut db, config) = placed_tiny();
    let mut saved = false;
    let mut write = |_: &RouteState| {
        saved = true;
        Ok(())
    };
    let run = RunOptions {
        save: Some(Save {
            after: SavePoint::Round(TINY_ROUNDS + 1),
            write: &mut write,
        }),
        ..RunOptions::default()
    };

    let summary = route_with(&mut db, &config, run).unwrap();

    assert!(!saved);
    assert!(!summary.state_saved);
}

/// The state saved after the first pass with its first line of `kind`
/// rewritten by `edit` from its words.
fn tampered_state(kind: &str, edit: impl Fn(&[&str]) -> String) -> RouteState {
    let (_, state) = route_saving(SavePoint::Round(0));
    let mut text = Vec::new();
    state.write(&mut text).unwrap();
    let text = String::from_utf8(text).unwrap();
    let mut tampered = false;
    let lines: Vec<String> = text
        .lines()
        .map(|line| {
            let words: Vec<&str> = line.split(' ').collect();
            if tampered || words[0] != kind {
                return line.to_string();
            }
            tampered = true;
            edit(&words)
        })
        .collect();
    assert!(tampered, "no `{kind}` line to tamper with");
    RouteState::parse(&(lines.join("\n") + "\n")).unwrap()
}

/// Routes the placed fixture resuming from `state`.
fn resume_tiny(state: &RouteState) -> Result<crate::RoutingSummary, RouteError> {
    let (mut db, config) = placed_tiny();
    let run = RunOptions {
        resume: Some(state),
        ..RunOptions::default()
    };
    route_with(&mut db, &config, run)
}

#[test]
fn a_state_with_a_wire_past_the_end_of_its_track_is_refused() {
    let (db, config) = placed_tiny();
    let tech = RouteTech::new(&db, &config).unwrap();
    let bad = tampered_state("s", |words| {
        let layer = LayerId::new(words[1].parse().unwrap());
        let past_end = tech.layer(layer).cross.len();
        format!("s {} {} {} {past_end}", words[1], words[2], words[3])
    });

    let result = resume_tiny(&bad);

    assert!(
        matches!(
            result,
            Err(RouteError::DetailedRestore(RestoreRoutesError::OffTracks(
                _
            )))
        ),
        "{result:?}"
    );
}

#[test]
fn a_state_with_a_via_between_nodes_of_one_layer_is_refused() {
    let bad = tampered_state("v", |words| {
        format!(
            "v {} {} {} {} {} {}",
            words[1], words[2], words[3], words[1], words[2], words[3]
        )
    });

    let result = resume_tiny(&bad);

    assert!(
        matches!(
            result,
            Err(RouteError::DetailedRestore(RestoreRoutesError::Via(_)))
        ),
        "{result:?}"
    );
}

#[test]
fn a_state_of_another_design_is_refused() {
    let (_, state) = route_saving(SavePoint::Global);
    let (mut db, config) = placed_tiny();
    db.name = "other".to_string();

    let result = route_with(
        &mut db,
        &config,
        RunOptions {
            resume: Some(&state),
            ..RunOptions::default()
        },
    );

    assert!(matches!(result, Err(RouteError::StateDesign(_))));
}

#[test]
fn removing_a_nets_wiring_keeps_its_fixed_metal() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu(1000));
    let layer = LayerId(1);
    let around = |p: Point<Dbu>| Rect::new(p, p).expand(Dbu(50));
    let pin = around(tech.layer(layer).point(0, 0));
    let wire = around(tech.layer(layer).point(0, 20));
    let owner = Owner::Net(NetId(0));
    occupancy.add_rect(&tech, layer, &pin, owner, Origin::Fixed);
    occupancy.add_rect(&tech, layer, &wire, owner, Origin::Wiring);
    let slots = occupancy.slots_of(&tech, owner, &[(layer, pin), (layer, wire)], &[]);

    occupancy.remove_wiring(&slots, owner);

    let other = Owner::Net(NetId(1));
    assert_eq!(
        occupancy.rect_conflicts(&tech, layer, &pin, other).count(),
        1
    );
    assert_eq!(
        occupancy.rect_conflicts(&tech, layer, &wire, other).count(),
        0
    );
}

#[test]
fn cuts_of_different_nets_conflict_within_the_cut_spacing() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu(1000));
    let v1 = CutLayerId::new(0);
    let cut = |x: i64, y: i64| {
        Rect::new(
            Point::new(Dbu(x), Dbu(y)),
            Point::new(Dbu(x + 100), Dbu(y + 100)),
        )
    };
    occupancy.add_cut(v1, &cut(0, 0), Owner::Net(NetId(0)), Origin::Wiring);

    let close = occupancy
        .cut_conflicts(v1, &cut(210, 0), Owner::Net(NetId(1)))
        .count();
    let apart = occupancy
        .cut_conflicts(v1, &cut(220, 0), Owner::Net(NetId(1)))
        .count();
    let diagonal = occupancy
        .cut_conflicts(v1, &cut(190, 190), Owner::Net(NetId(1)))
        .count();
    let same_net = occupancy
        .cut_conflicts(v1, &cut(150, 0), Owner::Net(NetId(0)))
        .count();

    assert_eq!((close, apart, diagonal, same_net), (1, 0, 0, 0));
}

#[test]
fn removing_a_nets_wiring_keeps_its_fixed_cuts() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu(1000));
    let v1 = CutLayerId::new(0);
    let cut = |x: i64| {
        Rect::new(
            Point::new(Dbu(x), Dbu(0)),
            Point::new(Dbu(x + 100), Dbu(100)),
        )
    };
    let owner = Owner::Net(NetId(0));
    occupancy.add_cut(v1, &cut(0), owner, Origin::Fixed);
    occupancy.add_cut(v1, &cut(5000), owner, Origin::Wiring);
    let slots = occupancy.slots_of(&tech, owner, &[], &[(v1, cut(0)), (v1, cut(5000))]);

    occupancy.remove_wiring(&slots, owner);

    let other = Owner::Net(NetId(1));
    assert_eq!(occupancy.cut_conflicts(v1, &cut(150), other).count(), 1);
    assert_eq!(occupancy.cut_conflicts(v1, &cut(5150), other).count(), 0);
}

#[test]
fn wires_keep_the_table_spacing_from_a_long_wide_shape_and_the_minimum_from_a_wire() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let m2 = tech.layer(LayerId(1));
    let stripe = Rect::new(Point::new(Dbu(0), Dbu(0)), Point::new(Dbu(400), Dbu(2000)));
    let wire = Rect::new(Point::new(Dbu(0), Dbu(0)), Point::new(Dbu(100), Dbu(2000)));

    assert_eq!(m2.spacing_around(&stripe), (Dbu(200), Dbu(110)));
    assert_eq!(m2.spacing_around(&wire), (Dbu(110), Dbu(110)));
}

#[test]
fn routing_refuses_pins_of_different_nets_that_overlap() {
    let mut db = tiny();
    for cell in &mut db.cells {
        cell.status = PlacementStatus::Placed;
    }
    let nand = db.library.macro_id("NAND2").unwrap();
    let master = &mut db.library.macros[nand];
    let (a, b) = (master.pin_id("A").unwrap(), master.pin_id("B").unwrap());
    master.pins[b].shapes = master.pins[a].shapes.clone();

    let result = route(&mut db, &RoutingConfig::default());

    assert_eq!(
        result,
        Err(RouteError::OverlappingPins {
            count: 2,
            cell: "u3".to_string(),
            nets: ("n1".to_string(), "n2".to_string()),
        })
    );
}

#[test]
fn consecutive_wire_edges_become_one_wire() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let node = |cross| Node {
        layer: LayerId(1),
        track: 0,
        cross,
    };
    let route = NetRoute {
        edges: [
            Edge::Wire(node(2)),
            Edge::Wire(node(0)),
            Edge::Wire(node(1)),
        ]
        .into_iter()
        .collect(),
        ..NetRoute::default()
    };

    let elems = route.elems(&tech);

    let lt = tech.layer(LayerId(1));
    assert_eq!(
        elems,
        [RouteElem::Wire {
            layer: LayerId(1),
            from: lt.point(0, 0),
            to: lt.point(0, 3),
            extensions: (None, None),
        }]
    );
}

/// Gcell pitches that cut the tiny design's 6000-unit die into two by two
/// gcells: its gcell pitch is the 400-unit track spacing of M3, its highest
/// routing layer.
const TWO_BY_TWO: NonZeroU32 = NonZeroU32::new(8).unwrap();

const fn share(value: f64) -> Share {
    Share::new(value).unwrap()
}

const fn reserve(value: f64) -> CapacityReserve {
    CapacityReserve::new(value).unwrap()
}

fn no_reserve() -> RoutingConfig {
    RoutingConfig {
        global_capacity_reserve: CapacityReserve::NONE,
        gcell_tracks: TWO_BY_TWO,
        ..RoutingConfig::default()
    }
}

#[test]
fn every_track_through_a_gcell_counts_toward_its_capacity() {
    let db = tiny();

    let reduction = capacity_reduction(&db, &no_reserve(), &IdVec::new()).unwrap();

    assert_eq!((reduction.nx, reduction.ny), (2, 2));
    assert_eq!(reduction.capacity, vec![24.0, 22.0, 23.0, 21.0]);
    assert!(reduction.reduction[0].abs() < 1e-12);
    assert!(reduction.reduction[2].abs() < 1e-12);
}

#[test]
fn the_capacity_reserve_does_not_count_as_reduction() {
    let db = tiny();
    let config = RoutingConfig {
        global_capacity_reserve: reserve(0.5),
        gcell_tracks: TWO_BY_TWO,
        ..RoutingConfig::default()
    };

    let reduction = capacity_reduction(&db, &config, &IdVec::new()).unwrap();

    assert_eq!(reduction.capacity, vec![24.0, 22.0, 23.0, 21.0]);
    assert!(reduction.reduction[0].abs() < 1e-9);
    assert!(reduction.reduction[2].abs() < 1e-9);
}

#[test]
fn the_layer_adjustment_counts_as_reduction_rounded_as_openroad_rounds_it() {
    let db = tiny();

    let reduction = capacity_reduction(&db, &no_reserve(), &IdVec::filled(share(0.5), 10)).unwrap();

    assert!(
        (reduction.reduction[0] - (4.0 + 8.0)).abs() < 1e-9,
        "{reduction:?}"
    );
    assert!(
        (reduction.reduction[2] - (4.0 + 8.0)).abs() < 1e-9,
        "{reduction:?}"
    );
}

#[test]
fn each_layer_counts_only_its_own_adjustment() {
    let db = tiny();
    let all = capacity_reduction(&db, &no_reserve(), &IdVec::filled(share(0.5), 10)).unwrap();

    let one_layer =
        capacity_reduction(&db, &no_reserve(), &vec![share(0.0), share(0.5)].into()).unwrap();

    assert!(one_layer.reduction[0] > 0.0, "{one_layer:?}");
    assert!(one_layer.reduction[0] < all.reduction[0], "{one_layer:?}");
}

#[test]
fn a_fixed_block_takes_the_tracks_its_obstruction_crosses() {
    let db = tiny();

    let reduction = capacity_reduction(&db, &no_reserve(), &IdVec::new()).unwrap();

    assert!(reduction.reduction[1] >= 10.0, "{reduction:?}");
    assert!(reduction.reduction[3] >= 10.0, "{reduction:?}");
}

#[test]
fn a_layer_blocked_everywhere_counts_as_taken() {
    let mut db = tiny();
    let _ = db.routing_blockages.push(Shape {
        layer: LayerRef::Routing(LayerId(1)),
        rect: db.die_area,
    });

    let reduction = capacity_reduction(&db, &no_reserve(), &IdVec::new()).unwrap();

    assert_eq!(reduction.capacity, vec![24.0, 22.0, 23.0, 21.0]);
    assert!(
        (reduction.reduction[0] - 16.0).abs() < 1e-9,
        "{reduction:?}"
    );
    assert!(
        (reduction.reduction[2] - 16.0).abs() < 1e-9,
        "{reduction:?}"
    );
}

#[test]
fn rudy_wires_are_as_wide_as_openroad_makes_them() {
    let nangate45 = [380, 280, 560, 560, 560, 1600, 1600, 3200, 3200].map(Dbu);

    let width = rudy_wire_width(&nangate45);

    assert_eq!(width, Dbu(93));
}

#[test]
fn merged_fixed_stretches_give_the_capacity_the_occupancy_gives() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let gcell = tech.min_layer_pitch * 15;
    let mut occupancy = Occupancy::new(&tech, gcell);
    occupancy.add_fixed(&tech, fixed_shapes(&db));
    let fixed = FixedTracks::new(&tech, fixed_shapes(&db));

    let from_occupancy = GlobalGrid::new(&tech, &occupancy, db.die_area, gcell, reserve(0.3));
    let from_stretches = GlobalGrid::new(&tech, &fixed, db.die_area, gcell, reserve(0.3));

    assert_eq!(
        from_occupancy.capacity_reduction(&tech, &IdVec::filled(share(0.5), 10), Dbu(100)),
        from_stretches.capacity_reduction(&tech, &IdVec::filled(share(0.5), 10), Dbu(100))
    );
}

#[test]
fn gcell_congestion_follows_the_routed_edges_of_the_grid() {
    let db = tiny();
    let tech = RouteTech::new(&db, &no_reserve()).unwrap();
    let gcell = tech.min_layer_pitch * 5;
    let fixed = FixedTracks::new(&tech, fixed_shapes(&db));
    let mut grid = GlobalGrid::new(&tech, &fixed, db.die_area, gcell, CapacityReserve::NONE);
    let (nx, ny) = grid.size();
    let layer = tech.min_layer;
    let row = ny - 1;
    let along = |x: u16| GCell { layer, x, y: row };
    let net = GlobalNet {
        net: NetId::new(0),
        terminals: vec![along(0), along(2)],
        demand: track_demand(&db, &tech, NetId::new(0)),
    };

    let _ = route_global(&mut grid, &[net], 0);
    let congestion = grid.congestion(&tech);

    assert_eq!(congestion.len(), usize::from(nx) * usize::from(ny));
    let at = |x: u16| congestion[usize::from(row) * usize::from(nx) + usize::from(x)];
    assert!(at(0) > 0.0 && at(1) > 0.0 && at(2) > 0.0, "{congestion:?}");
    let on_row = |i: usize| i / usize::from(nx) == usize::from(row);
    assert!(
        congestion
            .iter()
            .enumerate()
            .all(|(i, &c)| c == 0.0 || on_row(i)),
        "{congestion:?}"
    );
}

#[test]
fn the_estimator_measures_congestion_where_the_given_locations_put_the_nets() {
    let db = tiny();
    let config = RoutingConfig {
        gcell_tracks: NonZeroU32::new(5).unwrap(),
        ..no_reserve()
    };
    let mut estimator = GlobalRouteEstimator::new(&db, &config).unwrap();
    let tiles = estimator.capacity_reduction(&IdVec::new());
    let packed = IdVec::filled(Point::new(Dbu(0), Dbu(0)), db.num_cells());
    let spread: IdVec<CellId, Point<Dbu>> = db
        .cells
        .ids()
        .map(|id| {
            let far = Dbu(5000 * i64::from(u8::from(id.index() % 2 == 1)));
            Point::new(far, far)
        })
        .collect();

    let packed_congestion = estimator.congestion(&packed);
    let spread_congestion = estimator.congestion(&spread);

    assert_eq!(packed_congestion.len(), tiles.nx * tiles.ny);
    let congested = |map: &[f64]| map.iter().filter(|&&c| c > 0.0).count();
    assert!(
        congested(&spread_congestion) > congested(&packed_congestion),
        "{packed_congestion:?} vs {spread_congestion:?}"
    );
}

/// The tiny design with net 0 under a rule doubling its width on M1 and
/// asking 0.3 um of spacing there.
fn tiny_with_wide_rule() -> NetlistDB {
    let mut db = tiny();
    let rule = db.non_default_rules.push(NonDefaultRule {
        name: "DW".to_string(),
        hard_spacing: true,
        layers: vec![NonDefaultLayer {
            layer: LayerId(0),
            width: Dbu(200),
            spacing: Some(Dbu(300)),
            wire_extension: None,
        }],
        vias: Vec::new(),
        via_rules: Vec::new(),
        min_cuts: Vec::new(),
    });
    db.nets[NetId(0)].non_default_rule = Some(rule);
    db
}

#[test]
fn a_rule_sets_the_width_and_spacing_of_its_nets_wires() {
    let db = tiny_with_wide_rule();

    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();

    let rule = tech.net_rule(NetId(0));
    assert_eq!(rule, Some(NonDefaultRuleId::new(0)));
    assert_eq!(tech.wire_width(rule, LayerId(0)), Dbu(200));
    assert_eq!(tech.wire_spacing(rule, LayerId(0)), Dbu(300));
    assert_eq!(tech.wire_width(rule, LayerId(1)), Dbu(100));
    assert_eq!(tech.wire_width(None, LayerId(0)), Dbu(100));
    assert_eq!(tech.wire_spacing(None, LayerId(0)), Dbu(100));
}

#[test]
fn a_wire_under_a_rule_keeps_other_tracks_at_the_rules_spacing() {
    let db = tiny_with_wide_rule();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu(1000));
    let m1 = LayerId(0);
    let wide = Rect::new(
        Point::new(Dbu(1000), Dbu(0)),
        Point::new(Dbu(2000), Dbu(200)),
    );
    occupancy.add_rect(&tech, m1, &wide, Owner::Net(NetId(0)), Origin::Wiring);
    let two_tracks_up = Rect::new(
        Point::new(Dbu(1000), Dbu(450)),
        Point::new(Dbu(2000), Dbu(550)),
    );

    let conflicts = occupancy
        .rect_conflicts(&tech, m1, &two_tracks_up, Owner::Net(NetId(1)))
        .count();

    assert_eq!(conflicts, 1);
}

#[test]
fn a_wire_without_a_rule_leaves_tracks_beyond_the_layer_spacing_alone() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu(1000));
    let m1 = LayerId(0);
    let wire = Rect::new(
        Point::new(Dbu(1000), Dbu(50)),
        Point::new(Dbu(2000), Dbu(150)),
    );
    occupancy.add_rect(&tech, m1, &wire, Owner::Net(NetId(0)), Origin::Wiring);
    let two_tracks_up = Rect::new(
        Point::new(Dbu(1000), Dbu(450)),
        Point::new(Dbu(2000), Dbu(550)),
    );

    let conflicts = occupancy
        .rect_conflicts(&tech, m1, &two_tracks_up, Owner::Net(NetId(1)))
        .count();

    assert_eq!(conflicts, 0);
}

#[test]
fn a_wire_under_a_rule_keeps_its_track_clear_for_the_rules_spacing() {
    let db = tiny_with_wide_rule();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let mut occupancy = Occupancy::new(&tech, Dbu(1000));
    let m1 = LayerId(0);
    let wide = Rect::new(
        Point::new(Dbu(1000), Dbu(0)),
        Point::new(Dbu(2000), Dbu(200)),
    );
    occupancy.add_rect(&tech, m1, &wide, Owner::Net(NetId(0)), Origin::Wiring);
    let further_along = Rect::new(
        Point::new(Dbu(2150), Dbu(50)),
        Point::new(Dbu(3000), Dbu(150)),
    );

    let conflicts = occupancy
        .rect_conflicts(&tech, m1, &further_along, Owner::Net(NetId(1)))
        .count();

    assert_eq!(conflicts, 1);
}

/// A route of one straight run of `crosses` crossings along track 0 of
/// `layer`, with a via up from its first crossing.
fn straight_route(tech: &RouteTech, layer: LayerId, crosses: u32) -> NetRoute {
    let node = |cross| Node {
        layer,
        track: 0,
        cross,
    };
    let above = Node {
        layer: LayerId(layer.0 + 1),
        ..node(0)
    };
    let mut edges = EdgeSet::from_runs([Run {
        layer,
        track: 0,
        from: 0,
        to: crosses,
    }]);
    edges.extend([Edge::Via {
        lower: node(0),
        upper: above,
    }]);
    assert!(tech.layer(layer).cross.len() > crosses as usize);
    NetRoute {
        edges,
        ..NetRoute::default()
    }
}

#[test]
fn a_route_near_an_area_keeps_the_stretch_of_a_long_run_meeting_it() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let layer = LayerId(1);
    let lt = tech.layer(layer);
    let route = straight_route(&tech, layer, 20);
    let area = Rect::new(lt.point(0, 8), lt.point(0, 11));

    let near = route.near(&tech, &area);

    let expected = Run {
        layer,
        track: 0,
        from: 7,
        to: 12,
    };
    assert_eq!(near.edges.runs(), &[expected]);
}

#[test]
fn a_route_near_an_area_drops_the_vias_outside_it() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let layer = LayerId(1);
    let lt = tech.layer(layer);
    let route = straight_route(&tech, layer, 20);
    assert_eq!(route.edges.vias().len(), 1);

    let near = route.near(&tech, &Rect::new(lt.point(0, 8), lt.point(0, 11)));

    assert_eq!(near.edges.vias(), []);
}
