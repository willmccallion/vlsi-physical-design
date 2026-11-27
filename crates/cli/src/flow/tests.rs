use super::*;
use crate::testing::{scratch_dir, timed, timed_config, tiny, tiny_config};

#[test]
fn a_design_placed_by_its_input_leaves_global_routing_to_the_detailed_router() {
    let design = Design {
        db: tiny(),
        stage: Loaded,
    };
    let mut report = Report::default();

    let routed = design
        .placed_by_input()
        .route_globally(&tiny_config(""), &mut report)
        .unwrap();

    assert!(matches!(routed.stage.start, RouteStart::FromScratch(_)));
    assert!(report.phases.is_empty());
}

#[test]
fn a_placed_design_takes_its_leaf_clock_pins_from_the_liberty_its_config_names() {
    let config = timed_config("", &scratch_dir("flow-leaves"));
    let design = Design {
        db: timed(),
        stage: Loaded,
    }
    .placed_by_input();

    let leaves = design.leaf_clock_pins(&config).unwrap();

    assert_ne!(leaves, LeafClockPins::default());
}
