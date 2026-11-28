//! What routing keeps on a roomy design, whatever its cells and nets: every
//! net ends in one connected piece, and the independent checker of the
//! verify crate finds no violation.

#![allow(clippy::unwrap_used)]

use pare_common::config::RoutingConfig;
use pare_common::db::indices::NetId;
use pare_common::testing::designs::{self, InverterNetwork, ROWS, SLOT_COLUMNS};
use pare_router::{RouteSetup, route};
use pare_verify::drc::check_drc;
use pare_verify::routing::net_components;
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

/// A roomy network of 4 to 23 inverters, each driving up to three others.
fn design() -> impl Strategy<Value = InverterNetwork> {
    let slots: Vec<(i64, i64)> = (0..ROWS)
        .step_by(2)
        .flat_map(|r| (0..SLOT_COLUMNS).map(move |c| (c, r)))
        .collect();
    prop::sample::subsequence(slots, 4..24)
        .prop_flat_map(|slots| {
            let n = slots.len();
            let loads = prop::collection::vec(prop::collection::vec(0..n, 1..4), n);
            (Just(slots), loads)
        })
        .prop_map(|(gates, loads)| InverterNetwork {
            gates,
            nets: loads.into_iter().enumerate().collect(),
        })
}

fn config() -> Config {
    Config {
        cases: 16,
        rng_seed: RngSeed::Fixed(47),
        failure_persistence: None,
        ..Config::default()
    }
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn routing_a_roomy_design_connects_every_net_and_breaks_no_rule(design in design()) {
        let mut db = designs::parse(&design.def_text());

        let summary = route(&mut db, &RouteSetup::from(&RoutingConfig::default())).unwrap();

        prop_assert!(summary.incomplete.is_empty() && summary.inaccessible.is_empty(), "{summary:?}");
        for n in 0..db.nets.len() {
            let net = NetId::new(n);
            prop_assert_eq!(net_components(&db, net), 1, "net {}", db.nets[net].name);
        }
        let drc = check_drc(&db);
        prop_assert!(drc.violations.is_empty(), "{} violations, first {:?}", drc.violations.len(), drc.violations.first());
    }
}
