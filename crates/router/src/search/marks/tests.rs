use super::*;
use crate::tech::{CrossIdx, TrackIdx};
use pare_common::db::indices::LayerId;

fn node(cross: u32) -> Node {
    Node {
        layer: LayerId::new(1),
        track: TrackIdx(4),
        cross: CrossIdx(cross),
    }
}

#[test]
fn one_violation_fades_after_ten_decays_at_the_early_rate() {
    let mut marks = Marks::default();
    marks.mark_planar(node(3));

    for _ in 0..9 {
        marks.decay(0.95);
    }
    let after_nine = marks.planar(node(3));
    marks.decay(0.95);

    assert!(after_nine);
    assert!(!marks.planar(node(3)));
}

#[test]
fn two_violations_on_one_place_last_longer_than_one() {
    let mut once = Marks::default();
    once.mark_via(node(3));
    let mut twice = once.clone();
    twice.mark_via(node(3));

    for _ in 0..10 {
        once.decay(0.95);
        twice.decay(0.95);
    }

    assert!(!once.via(node(3)));
    assert!(twice.via(node(3)));
}

#[test]
fn a_counter_stops_at_its_largest_value() {
    let mut marks = Marks::default();

    for _ in 0..40 {
        marks.mark_planar(node(0));
    }

    assert_eq!(marks.planar.get(&node(0)), Some(&u8::MAX));
}

#[test]
fn crossings_and_vias_are_marked_apart() {
    let mut marks = Marks::default();

    marks.mark_planar(node(2));

    assert!(marks.planar(node(2)));
    assert!(!marks.via(node(2)));
}
