use super::*;
use pare_common::config::RoutingConfig;

#[test]
fn the_first_pass_has_no_marker_cost() {
    let costs = round_costs(0, 1.0);

    assert!((costs.conflict - 8.0).abs() < f64::EPSILON);
    assert!(costs.marker.abs() < f64::EPSILON);
}

#[test]
fn conflict_and_marker_costs_grow_together() {
    let costs = round_costs(34, 1.0);

    assert!((costs.conflict - 128.0).abs() < f64::EPSILON);
    assert!((costs.marker - 128.0).abs() < f64::EPSILON);
}

#[test]
fn rounds_past_the_schedule_keep_its_last_costs() {
    assert_eq!(round_costs(200, 1.0), round_costs(64, 1.0));
}

#[test]
fn rounds_that_rip_up_every_net_go_back_to_the_base_costs() {
    let strategy = round_strategy(49);

    assert_eq!(strategy.ripup, RipUp::All);
    assert!((strategy.conflict - 8.0).abs() < f64::EPSILON);
}

#[test]
fn rounds_with_equal_costs_but_a_new_fixed_shape_cost_differ() {
    let (fourteenth, fifteenth) = (round_strategy(14), round_strategy(15));

    assert_ne!(fourteenth, fifteenth);
}

#[test]
fn rounds_differing_only_in_tiling_are_the_same_strategy() {
    assert_eq!(round_strategy(4), round_strategy(5));
}

#[test]
fn routing_ends_by_default_at_the_last_round_of_the_schedule() {
    let rounds = RoutingConfig::default().detailed_iterations;

    assert_eq!(rounds, SCHEDULE.len() - 1);
}
