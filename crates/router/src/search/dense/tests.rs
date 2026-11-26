use super::*;

use crate::occupancy::{Occupancy, Owner};
use crate::search::edge_rules::NO_RULES;
use crate::search::{Costs, Marks};
use crate::tech::{RouteSetup, RouteTech};
use pare_common::config::RoutingConfig;
use pare_common::db::indices::NetId;

/// The numbering of the tiny design's whole die, on M1 and up.
fn tiny_numbering() -> Numbering {
    let db = crate::tests::tiny();
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
        costs: Costs {
            via: 50.0,
            conflict: 8.0,
            marker: 32.0,
            decay: 0.95,
        },
        guide: None,
        own_vias: None,
        rules: &NO_RULES,
        ripped: &[],
    };
    Numbering::new(&search, &db.die_area)
}

/// Every state the search can hold on `node`, by the reaches of its
/// layer's span.
fn states_on(numbering: &Numbering, node: Node) -> Vec<State> {
    let span = numbering.span(node.layer).unwrap();
    let still = [Heading::Still, Heading::Up, Heading::Down].map(|heading| State {
        node,
        heading,
        recent_via: None,
        recent_jog: None,
    });
    let (reach, across) = (i64::from(span.reach), i64::from(span.across_reach));
    let sides = [node.layer.below(), node.layer.above()];
    let after_vias = sides.into_iter().flatten().flat_map(|from| {
        (-reach..=reach).flat_map(move |back| {
            (-across..=across).filter_map(move |over| {
                let cross = u32::try_from(i64::from(node.cross.0) - back).ok()?;
                let track = u32::try_from(i64::from(node.track.0) - over).ok()?;
                Some(State {
                    node,
                    heading: Heading::Still,
                    recent_via: Some(RecentVia {
                        from,
                        track: TrackIdx(track),
                        cross: CrossIdx(cross),
                    }),
                    recent_jog: None,
                })
            })
        })
    });
    still.into_iter().chain(after_vias).collect()
}

#[test]
fn every_state_in_the_window_gets_its_own_slot_on_its_own_node() {
    let numbering = tiny_numbering();
    let nodes: Vec<Node> = numbering
        .spans
        .iter()
        .enumerate()
        .flat_map(|(l, span)| {
            let layer = LayerId::new(numbering.min_layer.index() + l);
            (0..span.nodes()).map(move |i| span.node_at(layer, i))
        })
        .collect();

    let mut slots = Vec::new();
    for &node in &nodes {
        for state in states_on(&numbering, node) {
            let slot = numbering.slot(state).unwrap();
            assert_eq!(numbering.node_of_slot(slot), node);
            slots.push(slot);
        }
    }

    let count = slots.len();
    slots.sort_unstable();
    slots.dedup();
    assert_eq!(slots.len(), count);
    assert!(slots.iter().all(|&s| s < numbering.slots));
}

#[test]
fn a_recent_via_a_jog_left_on_the_next_track_gets_its_own_slot() {
    let numbering = tiny_numbering();
    let node = Node {
        layer: LayerId::new(0),
        track: TrackIdx(2),
        cross: CrossIdx(3),
    };
    let landed_on = |track: TrackIdx| State {
        node,
        heading: Heading::Still,
        recent_via: Some(RecentVia {
            from: LayerId::new(1),
            track,
            cross: node.cross,
        }),
        recent_jog: None,
    };

    let here = numbering.slot(landed_on(TrackIdx(2)));
    let beside = numbering.slot(landed_on(TrackIdx(1)));

    assert!(here.is_some() && beside.is_some() && here != beside);
}

#[test]
fn a_state_s_recent_jog_leaves_its_slot_alone() {
    let numbering = tiny_numbering();
    let past = |recent_jog| State {
        node: Node {
            layer: LayerId::new(0),
            track: TrackIdx(2),
            cross: CrossIdx(3),
        },
        heading: Heading::Up,
        recent_via: None,
        recent_jog,
    };

    let slots = [None, Some(CrossIdx(2))].map(|j| numbering.slot(past(j)));

    assert!(slots[0].is_some() && slots[0] == slots[1]);
}

#[test]
fn steps_within_counts_the_longest_run_closer_than_the_reach() {
    let at = [
        Dbu::new(0),
        Dbu::new(10),
        Dbu::new(20),
        Dbu::new(50),
        Dbu::new(55),
    ];

    assert_eq!(steps_within(&at, Dbu::new(21)), 2);
    assert_eq!(steps_within(&at, Dbu::new(20)), 1);
    assert_eq!(steps_within(&at, Dbu::new(1)), 0);
}
