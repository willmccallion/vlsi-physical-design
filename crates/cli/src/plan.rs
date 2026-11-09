//! The run's config resolved against the design's library before any step
//! runs, so that a mistake in it fails at once rather than after hours of
//! placement.

use anyhow::Context;
use pare_common::config::{
    Config, CongestionEstimate, GlobalPlacementConfig, LayerRcConfig, NetlistConfig, Purpose, Share,
};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{IdVec, LayerId};
use pare_common::units::Ohms;
use pare_resizer::{RouteRc, WireRc};
use std::path::PathBuf;

/// What the config asks of placement and repair, its names resolved
/// against the design's library and its rules checked.
#[derive(Debug)]
pub(crate) struct FlowPlan {
    /// The share of each routing layer's capacity routability-driven
    /// placement counts as taken.
    pub(crate) layer_adjustments: IdVec<LayerId, Share>,
    /// Netlist repair, when the config names Liberty files.
    pub(crate) repair: Option<RepairPlan>,
}

impl FlowPlan {
    /// The plan `config` makes for `db`.
    pub(crate) fn resolve(config: &Config, db: &NetlistDB) -> anyhow::Result<Self> {
        Ok(Self {
            layer_adjustments: layer_adjustments(&config.global_placement, db)?,
            repair: RepairPlan::resolve(config, db)?,
        })
    }
}

/// How the netlist is repaired.
#[derive(Debug)]
pub(crate) struct RepairPlan {
    /// The wires parasitics are estimated with.
    pub(crate) wires: Wires,
    /// Where the clocks come from.
    pub(crate) clocks: ClockSource,
    /// The wire clock trees are built on, when they are built: with clock
    /// trees on and an SDC file to time them.
    pub(crate) clock_tree: Option<WireRc>,
    /// The layers' parasitics for repair on the global routes, when the
    /// design is routed and the config asks for it.
    pub(crate) on_global_routes: Option<RouteRc>,
    /// Cells repair neither inserts nor swaps to, beyond those the library
    /// marks.
    pub(crate) dont_use: Vec<String>,
}

impl RepairPlan {
    /// The repair `config` asks of `db`, when it names Liberty files.
    fn resolve(config: &Config, db: &NetlistDB) -> anyhow::Result<Option<Self>> {
        if config.input.liberty_files.is_empty() {
            return Ok(None);
        }
        let netlist = &config.netlist;
        let wires = Wires::from_config(netlist, db)?;
        let clocks = ClockSource::of(config)?;
        let timed = matches!(clocks, ClockSource::Sdc(_));
        if timed && config.global_placement.timing_driven {
            check_congestion_follows_timing(&config.global_placement)?;
        }
        let clock_tree = (timed && netlist.clock_tree)
            .then(|| wires.clock.context("[netlist] clock trees need clock_wire"))
            .transpose()?;
        let routed = config.purpose == Purpose::PlaceAndRoute;
        let on_global_routes = (routed && netlist.repair_on_global_routes)
            .then(|| route_rc(config, db))
            .transpose()?;
        Ok(Some(Self {
            wires,
            clocks,
            clock_tree,
            on_global_routes,
            dont_use: netlist.dont_use.clone(),
        }))
    }
}

/// Where the clocks repair leaves alone come from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ClockSource {
    /// The SDC file at this path, whose clocks also time the design.
    Sdc(PathBuf),
    /// The I/O pins `[netlist] clock_ports` names, without an SDC file.
    Ports(Vec<String>),
}

impl ClockSource {
    /// The clocks of `config`: its SDC file's, else its clock ports, which
    /// it may not name beside an SDC file.
    fn of(config: &Config) -> anyhow::Result<Self> {
        let ports = &config.netlist.clock_ports;
        match &config.input.sdc_file {
            Some(_) if !ports.is_empty() => {
                anyhow::bail!("[netlist] clock_ports must be empty when [input] sdc_file is set")
            }
            Some(path) => Ok(Self::Sdc(path.clone())),
            None => Ok(Self::Ports(ports.clone())),
        }
    }
}

/// The wires repair estimates parasitics with, as `set_wire_rc` sets them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Wires {
    /// The signal wire, as `set_wire_rc -signal`.
    pub(crate) signal: WireRc,
    /// The clock wire, as `set_wire_rc -clock`, when the config has one.
    pub(crate) clock: Option<WireRc>,
}

impl Wires {
    /// The wires the netlist config names, on their layers of `db`.
    fn from_config(config: &NetlistConfig, db: &NetlistDB) -> anyhow::Result<Self> {
        let signal = config
            .signal_wire
            .as_ref()
            .context("[netlist] repair needs signal_wire")?;
        Ok(Self {
            signal: layer_wire_rc(signal, db).context("[netlist] signal_wire")?,
            clock: config
                .clock_wire
                .as_ref()
                .map(|wire| layer_wire_rc(wire, db).context("[netlist] clock_wire"))
                .transpose()?,
        })
    }
}

fn layer_wire_rc(wire: &LayerRcConfig, db: &NetlistDB) -> anyhow::Result<WireRc> {
    let layer = db
        .library
        .routing_layer_id(&wire.layer)
        .with_context(|| format!("unknown routing layer '{}'", wire.layer))?;
    Ok(WireRc::per_micron(
        wire.resistance_ohm_per_um,
        wire.capacitance_ff_per_um,
        layer,
    ))
}

/// Each routing layer's and cut layer's resistance and capacitance from
/// the netlist config, for the global routes' parasitics.
fn route_rc(config: &Config, db: &NetlistDB) -> anyhow::Result<RouteRc> {
    let lib = &db.library;
    let layers = config
        .netlist
        .layer_rc
        .iter()
        .map(|l| {
            let id = lib
                .routing_layer_id(&l.layer)
                .with_context(|| format!("[netlist] layer_rc names unknown layer '{}'", l.layer))?;
            let rc = WireRc::per_micron(l.resistance_ohm_per_um, l.capacitance_ff_per_um, id);
            Ok((id, rc.resistance, rc.capacitance))
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    let vias = config
        .netlist
        .via_rc
        .iter()
        .map(|v| {
            let cut = lib.cut_layer_id(&v.layer).with_context(|| {
                format!("[netlist] via_rc names unknown cut layer '{}'", v.layer)
            })?;
            Ok((cut, Ohms::new(v.resistance_ohm)))
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    let min_layer = *config.routing.layer_range(lib)?.start();
    Ok(RouteRc::new(lib, &layers, &vias, min_layer))
}

/// The share of each routing layer's capacity held back, by layer index.
fn layer_adjustments(
    gp: &GlobalPlacementConfig,
    db: &NetlistDB,
) -> anyhow::Result<IdVec<LayerId, Share>> {
    let layers = &db.library.routing_layers;
    let mut adjustments = IdVec::filled(gp.routing_layer_adjustment, layers.len());
    for entry in &gp.layer_adjustments {
        let layer = db
            .library
            .routing_layer_id(&entry.layer)
            .with_context(|| format!("no routing layer named {}", entry.layer))?;
        adjustments[layer] = entry.adjustment;
    }
    Ok(adjustments)
}

/// Fails when timing-driven placement would run beside the global-router
/// congestion estimate, which cannot follow the netlist's changes.
fn check_congestion_follows_timing(gp: &GlobalPlacementConfig) -> anyhow::Result<()> {
    if gp.routability_driven && gp.congestion_estimate == CongestionEstimate::GlobalRouter {
        anyhow::bail!(
            "timing-driven placement changes the netlist as it goes, which the global-router \
             congestion estimate cannot follow: set congestion_estimate = \"rudy\" or \
             timing_driven = false"
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{tiny, tiny_config};

    fn wire(layer: &str) -> LayerRcConfig {
        LayerRcConfig {
            layer: layer.to_string(),
            resistance_ohm_per_um: 2.0,
            capacitance_ff_per_um: 0.5,
        }
    }

    #[test]
    fn wires_take_their_layers_and_rc_from_the_config() {
        let db = tiny();
        let config = NetlistConfig {
            signal_wire: Some(wire("M1")),
            clock_wire: Some(wire("M1")),
            ..NetlistConfig::default()
        };

        let wires = Wires::from_config(&config, &db).unwrap();

        let m1 = WireRc::per_micron(2.0, 0.5, db.library.routing_layer_id("M1").unwrap());
        assert_eq!(
            wires,
            Wires {
                signal: m1,
                clock: Some(m1)
            }
        );
    }

    #[test]
    fn wires_without_a_clock_wire_estimate_clocks_as_signals() {
        let config = NetlistConfig {
            signal_wire: Some(wire("M1")),
            ..NetlistConfig::default()
        };

        let wires = Wires::from_config(&config, &tiny()).unwrap();

        assert_eq!(wires.clock, None);
    }

    #[test]
    fn wires_need_a_signal_wire() {
        let error = Wires::from_config(&NetlistConfig::default(), &tiny()).unwrap_err();

        assert_eq!(error.to_string(), "[netlist] repair needs signal_wire");
    }

    #[test]
    fn a_wire_on_an_unknown_layer_names_the_layer() {
        let config = NetlistConfig {
            signal_wire: Some(wire("M1")),
            clock_wire: Some(wire("M9")),
            ..NetlistConfig::default()
        };

        let error = Wires::from_config(&config, &tiny()).unwrap_err();

        assert_eq!(
            format!("{error:#}"),
            "[netlist] clock_wire: unknown routing layer 'M9'"
        );
    }

    /// The plan of a config for the tiny design with Liberty files, an SDC
    /// file and a signal wire, and the TOML `netlist` keys and `tables`.
    fn timed_plan(netlist: &str, tables: &str) -> anyhow::Result<FlowPlan> {
        let config = tiny_config(&format!(
            "liberty_files = [\"clock.lib\"]\n\
             sdc_file = \"tiny.sdc\"\n\
             [netlist]\n\
             signal_wire = {{ layer = \"M2\", resistance_ohm_per_um = 2.0, capacitance_ff_per_um = 0.5 }}\n\
             {netlist}\n\
             {tables}"
        ));
        FlowPlan::resolve(&config, &tiny())
    }

    #[test]
    fn clock_trees_on_a_timed_design_need_a_clock_wire() {
        let error = timed_plan("", "").unwrap_err();

        assert_eq!(error.to_string(), "[netlist] clock trees need clock_wire");
    }

    #[test]
    fn clock_trees_are_built_on_the_clock_wire() {
        let plan = timed_plan(
            "clock_wire = { layer = \"M3\", resistance_ohm_per_um = 1.0, capacitance_ff_per_um = 0.25 }",
            "",
        )
        .unwrap();

        let repair = plan.repair.unwrap();
        assert_eq!(repair.clock_tree, repair.wires.clock);
        assert!(repair.clock_tree.is_some());
    }

    #[test]
    fn without_clock_trees_no_clock_wire_is_needed() {
        let plan = timed_plan("clock_tree = false", "").unwrap();

        assert_eq!(plan.repair.unwrap().clock_tree, None);
    }

    #[test]
    fn clock_ports_beside_an_sdc_file_are_an_error() {
        let error = timed_plan("clock_tree = false\nclock_ports = [\"in\"]", "").unwrap_err();

        assert_eq!(
            error.to_string(),
            "[netlist] clock_ports must be empty when [input] sdc_file is set"
        );
    }

    #[test]
    fn without_an_sdc_file_the_clocks_are_the_clock_ports() {
        let config = tiny_config(
            "liberty_files = [\"clock.lib\"]\n\
             [netlist]\n\
             signal_wire = { layer = \"M2\", resistance_ohm_per_um = 2.0, capacitance_ff_per_um = 0.5 }\n\
             clock_ports = [\"in\"]",
        );

        let plan = FlowPlan::resolve(&config, &tiny()).unwrap();

        let repair = plan.repair.unwrap();
        assert_eq!(repair.clocks, ClockSource::Ports(vec!["in".to_string()]));
        assert_eq!(repair.clock_tree, None);
    }

    #[test]
    fn without_an_sdc_file_the_design_is_still_repaired_on_global_routes() {
        let config = tiny_config(
            "liberty_files = [\"clock.lib\"]\n\
             [netlist]\n\
             signal_wire = { layer = \"M2\", resistance_ohm_per_um = 2.0, capacitance_ff_per_um = 0.5 }",
        );

        let plan = FlowPlan::resolve(&config, &tiny()).unwrap();

        assert!(plan.repair.unwrap().on_global_routes.is_some());
    }

    #[test]
    fn a_layer_rc_on_an_unknown_layer_fails_the_plan() {
        let error = timed_plan(
            "clock_tree = false\n\
             layer_rc = [{ layer = \"M9\", resistance_ohm_per_um = 1.0, capacitance_ff_per_um = 0.1 }]",
            "",
        )
        .unwrap_err();

        assert_eq!(
            error.to_string(),
            "[netlist] layer_rc names unknown layer 'M9'"
        );
    }

    #[test]
    fn timing_driven_placement_beside_the_global_router_estimate_fails_the_plan() {
        let error = timed_plan(
            "clock_tree = false",
            "[global_placement]\ncongestion_estimate = \"global_router\"",
        )
        .unwrap_err();

        assert!(
            error
                .to_string()
                .starts_with("timing-driven placement changes the netlist"),
            "{error}"
        );
    }

    #[test]
    fn without_liberty_files_nothing_is_repaired() {
        let plan = FlowPlan::resolve(&tiny_config(""), &tiny()).unwrap();

        assert!(plan.repair.is_none());
    }

    fn share(value: f64) -> Share {
        Share::new(value).unwrap()
    }

    #[test]
    fn layers_without_their_own_adjustment_take_the_routing_adjustment() {
        let config = tiny_config(
            "[global_placement]\n\
             routing_layer_adjustment = 0.5\n\
             layer_adjustments = [{ layer = \"M2\", adjustment = 0.25 }]",
        );

        let plan = FlowPlan::resolve(&config, &tiny()).unwrap();

        assert_eq!(
            plan.layer_adjustments,
            IdVec::from(vec![share(0.5), share(0.25), share(0.5)])
        );
    }

    #[test]
    fn an_adjustment_of_an_unknown_layer_names_the_layer() {
        let config = tiny_config(
            "[global_placement]\nlayer_adjustments = [{ layer = \"M9\", adjustment = 0.25 }]",
        );

        let error = FlowPlan::resolve(&config, &tiny()).unwrap_err();

        assert_eq!(error.to_string(), "no routing layer named M9");
    }
}
