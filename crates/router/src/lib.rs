//! Global and detailed routing on the design's tracks.
//!
//! Global routing finds a corridor of gcells for every net; detailed routing
//! then draws wires on tracks and vias at track crossings inside those
//! corridors, negotiating with other nets until no two touch or come closer
//! than the layer spacing.

pub mod access;
pub mod detailed;
pub mod occupancy;
pub mod tech;

use detailed::{Costs, DetailedRouter, NetRoute, routable_nets};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::NetId;
use pare_common::util::config::RoutingConfig;
use pare_common::util::ui;
use rustc_hash::FxHashMap;
use std::time::Instant;
use tech::RouteTech;

/// What routing achieved.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RoutingSummary {
    /// Signal nets routed.
    pub nets: usize,
    /// Nets with a pin no access point reaches.
    pub inaccessible: Vec<NetId>,
    /// Nets whose search failed to connect every pin.
    pub incomplete: Vec<NetId>,
    /// Negotiation rounds run.
    pub rounds: usize,
    /// Nets still in conflict when negotiation stopped.
    pub conflicted: Vec<NetId>,
}

/// Routes every signal net of `db` and stores the wiring in the design.
pub fn route(db: &mut NetlistDB, config: &RoutingConfig) -> Result<RoutingSummary, RouteError> {
    let tech = RouteTech::new(db, config)?;
    let pitch = tech
        .layer(tech.min_layer)
        .tracks
        .windows(2)
        .map(|w| w[1] - w[0])
        .min()
        .unwrap_or_default();
    let gcell = pitch * i64::from(config.gcell_tracks.max(1));
    let pitch = pitch.to_f64();
    let mut costs = Costs {
        via: 4.0 * pitch,
        penalty: 8.0 * pitch,
        history: 4.0 * pitch,
    };

    let (routes, summary) = {
        let mut dr = DetailedRouter::new(db, &tech, gcell);
        let nets = routable_nets(db);
        let mut summary = RoutingSummary {
            nets: nets.len(),
            inaccessible: dr.inaccessible_nets(),
            ..RoutingSummary::default()
        };
        let start = Instant::now();
        let mut routes: FxHashMap<NetId, NetRoute> =
            dr.route_nets(&nets, costs).into_iter().collect();
        ui::routing_table_header("DR");
        let mut conflicted = dr.conflicted_nets(1.0);
        ui::routing_iter(
            "DR",
            0,
            conflicted.len(),
            nets.len(),
            costs.penalty / pitch,
            start.elapsed().as_millis(),
        );
        for round in 1..=config.detailed_iterations {
            if conflicted.is_empty() {
                break;
            }
            let start = Instant::now();
            conflicted.extend(dr.access_blockers(&conflicted, 1.0));
            conflicted.sort_unstable();
            conflicted.dedup();
            for &n in &conflicted {
                dr.rip_up(n);
            }
            costs.penalty *= 1.5;
            routes.extend(dr.route_nets(&conflicted, costs));
            let ripped = conflicted.len();
            conflicted = dr.conflicted_nets(1.0);
            summary.rounds = round;
            ui::routing_iter(
                "DR",
                round,
                conflicted.len(),
                ripped,
                costs.penalty / pitch,
                start.elapsed().as_millis(),
            );
        }
        summary.conflicted = conflicted;
        summary.incomplete = routes
            .iter()
            .filter(|(_, r)| !r.complete)
            .map(|(&n, _)| n)
            .collect();
        summary.incomplete.sort_unstable();
        (routes, summary)
    };
    for net in &mut db.nets {
        net.routing.clear();
    }
    for (n, r) in routes {
        db.nets[n.index()].routing = r.elems;
    }
    Ok(summary)
}

/// Why routing could not run.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RouteError {
    /// A configured layer name is not a routing layer.
    #[error("unknown routing layer '{0}'")]
    UnknownLayer(String),
    /// The minimum routing layer is above the maximum.
    #[error("the minimum routing layer is above the maximum")]
    EmptyLayerRange,
    /// A routable layer has no tracks and no pitch to derive them from.
    #[error("routing layer '{0}' has no tracks")]
    NoTracks(String),
    /// No usable via is defined through a cut layer.
    #[error("no via is defined through cut layer '{0}'")]
    NoVia(String),
}
