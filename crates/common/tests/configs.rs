//! Every shipped run configuration must parse, and malformed ones must not.

#![allow(clippy::unwrap_used)]

use pare_common::util::config::Config;
use std::path::{Path, PathBuf};

/// Every `.toml` file under `dir`, in its subfolders too.
fn toml_files(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.is_dir() {
            out.extend(toml_files(&path));
        } else if path.extension().is_some_and(|e| e == "toml") {
            out.push(path);
        }
    }
    out
}

#[test]
fn every_design_config_parses() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs");

    let files = toml_files(&dir);

    assert!(!files.is_empty(), "no configs found in {}", dir.display());
    for path in files {
        let text = std::fs::read_to_string(&path).unwrap();
        let parsed: Result<Config, _> = toml::from_str(&text);
        assert!(parsed.is_ok(), "{}: {:?}", path.display(), parsed.err());
    }
}

/// `body` after the one key every config needs.
fn parse(body: &str) -> Result<Config, toml::de::Error> {
    toml::from_str(&format!("purpose = \"place_and_route\"\n{body}"))
}

#[test]
fn a_config_of_only_its_purpose_parses() {
    let parsed = parse("");

    assert!(parsed.is_ok(), "{:?}", parsed.err());
}

#[test]
fn every_step_runs_unless_the_config_turns_it_off() {
    let config = parse("").unwrap();

    let netlist = &config.netlist;
    let gp = &config.global_placement;
    assert!(netlist.remove_buffers && netlist.buffer_ports && netlist.clock_tree);
    assert!(netlist.repair_design && netlist.repair_timing && netlist.repair_on_global_routes);
    assert!(gp.routability_driven && gp.timing_driven);
    assert!(config.io_placement.enabled);
}

#[test]
fn unknown_keys_are_rejected() {
    let parsed = parse("[global_placement]\ntarget_densty = 0.7\n");

    assert!(parsed.is_err());
}

#[test]
fn a_wire_is_read_as_its_layer_and_rc() {
    let parsed = parse(
        "[netlist]\nsignal_wire = { layer = \"metal3\", resistance_ohm_per_um = 3.5, capacitance_ff_per_um = 0.1 }\n",
    )
    .unwrap();

    let wire = parsed.netlist.signal_wire.unwrap();
    assert_eq!(
        (
            wire.layer.as_str(),
            wire.resistance_ohm_per_um,
            wire.capacitance_ff_per_um
        ),
        ("metal3", 3.5, 0.1)
    );
}

#[test]
fn a_wire_without_its_capacitance_is_rejected() {
    let parsed =
        parse("[netlist]\nclock_wire = { layer = \"metal5\", resistance_ohm_per_um = 1.5 }\n");

    assert!(parsed.is_err());
}

#[test]
fn values_outside_their_range_are_rejected() {
    let out_of_range = [
        "[global_placement]\ntarget_density = 0.0",
        "[global_placement]\ntarget_density = 1.5",
        "[global_placement]\nbin_dimension = 0",
        "[global_placement]\ntarget_overflow = -0.1",
        "[global_placement]\nrouting_layer_adjustment = 1.5",
        "[global_placement]\nlayer_adjustments = [{ layer = \"metal2\", adjustment = 1.5 }]",
        "[global_placement]\ntiming_driven_overflows = [0.64, 1.2]",
        "[routing]\nglobal_capacity_reserve = 1.0",
        "[routing]\nglobal_capacity_reserve = nan",
        "[routing]\ngcell_tracks = 0",
    ];

    let accepted: Vec<&str> = out_of_range
        .into_iter()
        .filter(|body| parse(body).is_ok())
        .collect();

    assert!(accepted.is_empty(), "accepted: {accepted:?}");
}

#[test]
fn values_at_the_edges_of_their_range_are_read() {
    let at_edges = [
        "[global_placement]\ntarget_density = 1.0",
        "[global_placement]\nbin_dimension = 1",
        "[global_placement]\ntarget_overflow = 0.0",
        "[global_placement]\nrouting_layer_adjustment = 1.0",
        "[global_placement]\nlayer_adjustments = [{ layer = \"metal2\", adjustment = 0.0 }]",
        "[routing]\nglobal_capacity_reserve = 0.0",
        "[routing]\ngcell_tracks = 1",
    ];

    let rejected: Vec<&str> = at_edges
        .into_iter()
        .filter(|body| parse(body).is_err())
        .collect();

    assert!(rejected.is_empty(), "rejected: {rejected:?}");
}

#[test]
fn an_out_of_range_value_is_reported_with_its_range() {
    let error = parse("[routing]\nglobal_capacity_reserve = 1.0").unwrap_err();

    assert!(error.to_string().contains("1 is outside [0, 1)"), "{error}");
}
