//! What routing keeps on a roomy design, whatever its cells and nets: every
//! net ends in one connected piece, the independent checker of the verify
//! crate finds no violation, the routes do not depend on how many threads
//! found them, and every net's global route reaches each of its pins.

#![allow(clippy::unwrap_used)]

use pare_common::config::RoutingConfig;
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::{NetRoute, RoutePoint, RouteSource};
use pare_common::db::indices::NetId;
use pare_common::db::writer::def;
use pare_common::testing::designs::{self, InverterNetwork, inverter_networks};
use pare_router::{GlobalSession, RouteSetup, route};
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

/// Whether `point` lies on one of `route`'s segments, ends included.
fn on_route(route: &NetRoute, point: RoutePoint) -> bool {
    route.segments.iter().any(|s| {
        let (a, b) = (s.from, s.to);
        let xs = a.at.x.min(b.at.x)..=a.at.x.max(b.at.x);
        let ys = a.at.y.min(b.at.y)..=a.at.y.max(b.at.y);
        let layers = a.layer.min(b.layer)..=a.layer.max(b.layer);
        xs.contains(&point.at.x) && ys.contains(&point.at.y) && layers.contains(&point.layer)
    })
}

/// Why `net`'s global route misses one of its pins, if it does.
fn missed_pin(db: &NetlistDB, net: NetId, route: &NetRoute) -> Option<String> {
    let mut pins: Vec<_> = route.pins.iter().map(|p| p.pin).collect();
    let mut wanted = db.nets[net].pins.clone();
    pins.sort_unstable();
    wanted.sort_unstable();
    if pins != wanted {
        return Some(format!("route meets pins {pins:?}, net has {wanted:?}"));
    }
    let single_gcell = route.pins.windows(2).all(|w| w[0].grid == w[1].grid);
    if route.segments.is_empty() && single_gcell {
        return None;
    }
    route
        .pins
        .iter()
        .find(|p| {
            !on_route(
                route,
                RoutePoint {
                    at: p.grid,
                    layer: p.layer,
                },
            )
        })
        .map(|p| {
            format!(
                "pin {:?} at {:?} on {:?} is off {route:?}",
                p.pin, p.grid, p.layer
            )
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

    #[test]
    fn every_global_route_reaches_each_pin_of_its_net(design in inverter_networks()) {
        let db = designs::parse(&design.def_text());

        let session = GlobalSession::new(&db, &RouteSetup::from(&RoutingConfig::default()), &mut |_| {}).unwrap();

        for net in db.nets.ids() {
            let route = session.route(&db, net);
            prop_assert!(route.is_some(), "net {} has no route", db.nets[net].name);
            if let Some(why) = route.and_then(|r| missed_pin(&db, net, &r)) {
                prop_assert!(false, "net {}: {why}", db.nets[net].name);
            }
        }
    }
}
