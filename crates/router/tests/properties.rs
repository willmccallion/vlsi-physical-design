//! What routing keeps on a roomy design, whatever its cells and nets: every
//! net ends in one connected piece, the independent checker of the verify
//! crate finds no violation, and the routes do not depend on how many
//! threads found them.

#![allow(clippy::unwrap_used)]

use pare_common::config::RoutingConfig;
use pare_common::db::indices::NetId;
use pare_common::db::writer::def;
use pare_common::testing::designs::{self, InverterNetwork, inverter_networks};
use pare_router::{RouteSetup, route};
use pare_verify::drc::check_drc;
use pare_verify::routing::net_components;
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

/// The DEF of `design` routed on a pool of `threads` threads.
fn routed_on(design: &InverterNetwork, threads: usize) -> String {
    let mut db = designs::parse(&design.def_text());
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .unwrap();
    let _ = pool
        .install(|| route(&mut db, &RouteSetup::from(&RoutingConfig::default())))
        .unwrap();
    let mut text = Vec::new();
    def::write(&db, &mut text).unwrap();
    String::from_utf8(text).unwrap()
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
    fn routing_a_roomy_design_connects_every_net_and_breaks_no_rule(design in inverter_networks()) {
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

    #[test]
    fn one_thread_and_many_route_a_design_alike(design in inverter_networks()) {
        prop_assert_eq!(routed_on(&design, 1), routed_on(&design, 4));
    }
}
