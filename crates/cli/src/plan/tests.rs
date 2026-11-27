use super::*;
use crate::testing::{tiny, tiny_config};
use pare_common::units::{FaradsPerMeter, OhmsPerMeter};

fn wire(layer: &str) -> LayerRcConfig {
    LayerRcConfig {
        layer: layer.to_string(),
        resistance: OhmsPerMeter::from_ohms_per_micron(2.0),
        capacitance: FaradsPerMeter::from_femtofarads_per_micron(0.5),
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

    let m1 = WireRc {
        resistance: OhmsPerMeter::from_ohms_per_micron(2.0),
        capacitance: FaradsPerMeter::from_femtofarads_per_micron(0.5),
        layer: db.library.routing_layer_id("M1").unwrap(),
    };
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
fn placement_that_is_not_timing_driven_may_use_the_global_router_estimate() {
    let plan = timed_plan(
        "clock_tree = false",
        "[global_placement]\ntiming_driven = false\ncongestion_estimate = \"global_router\"",
    )
    .unwrap();

    assert!(plan.repair.is_some());
}

#[test]
fn a_design_that_is_only_placed_is_not_repaired_on_global_routes() {
    let mut config = tiny_config(
        "liberty_files = [\"clock.lib\"]\n\
             [netlist]\n\
             signal_wire = { layer = \"M2\", resistance_ohm_per_um = 2.0, capacitance_ff_per_um = 0.5 }",
    );
    config.purpose = Purpose::Placement;

    let plan = FlowPlan::resolve(&config, &tiny()).unwrap();

    assert!(plan.repair.unwrap().on_global_routes.is_none());
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
