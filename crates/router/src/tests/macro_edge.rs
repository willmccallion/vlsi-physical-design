//! Reaching the pins of a hard block that sit between its obstruction
//! strips.

use super::macro_edge;
use crate::route;
use crate::tech::RouteSetup;
use pare_common::config::RoutingConfig;
use pare_verify::drc::check_drc;

/// Detailed rounds the fixture is routed for after its first pass: none,
/// since the first pass alone must reach every pin cleanly.
const ROUNDS: usize = 0;

#[test]
fn pins_between_obstruction_strips_are_reached_without_touching_them() {
    let mut db = macro_edge();
    let config = RoutingConfig {
        detailed_iterations: ROUNDS,
        ..RoutingConfig::default()
    };

    let _ = route(&mut db, &RouteSetup::from(&config)).unwrap();

    let drc = check_drc(&db);
    assert!(
        drc.violations.is_empty(),
        "{} violations, first {:?}",
        drc.violations.len(),
        drc.violations.first()
    );
}
