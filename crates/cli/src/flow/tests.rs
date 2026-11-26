use super::*;
use crate::testing::{tiny, tiny_config};

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
