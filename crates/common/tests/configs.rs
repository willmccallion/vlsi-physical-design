//! Every shipped run configuration must parse, and malformed ones must not.

#![allow(clippy::unwrap_used)]

use pare_common::config::{Config, DesignInput, InputConfig, InputError, IoLayerNames, OneIoLayer};
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

/// `body` between the purpose and the design every config needs.
fn parse(body: &str) -> Result<Config, toml::de::Error> {
    toml::from_str(&format!(
        "purpose = \"place_and_route\"\n{body}\n[input]\nlef_files = [\"a.lef\"]\ndef_file = \"a.def\"\n"
    ))
}

/// A config whose `[input]` table is `input`.
fn parse_input(input: &str) -> Result<InputConfig, String> {
    toml::from_str::<Config>(&format!("purpose = \"place_and_route\"\n[input]\n{input}"))
        .map(|c| c.input)
        .map_err(|e| e.message().to_string())
}

#[test]
fn a_config_of_its_purpose_and_design_parses() {
    let parsed = parse("");

    assert!(parsed.is_ok(), "{:?}", parsed.err());
}

#[test]
fn a_config_without_a_design_is_rejected() {
    let parsed = toml::from_str::<Config>("purpose = \"place_and_route\"\n");

    assert!(parsed.is_err());
}

#[test]
fn a_def_design_is_read_with_its_library() {
    let input = parse_input("lef_files = [\"t.lef\", \"c.lef\"]\ndef_file = \"d.def\"\n").unwrap();

    assert_eq!(
        input.design,
        DesignInput::LefDef {
            lef_files: vec![PathBuf::from("t.lef"), PathBuf::from("c.lef")],
            def_file: PathBuf::from("d.def"),
        }
    );
}

#[test]
fn a_bookshelf_design_takes_six_layers_unless_told() {
    let plain = parse_input("bookshelf_aux_file = \"d.aux\"\n").unwrap();
    let ten = parse_input("bookshelf_aux_file = \"d.aux\"\nnum_layers = 10\n").unwrap();

    let layers = |input: &InputConfig| match input.design {
        DesignInput::Bookshelf { num_layers, .. } => Some(num_layers),
        DesignInput::LefDef { .. } => None,
    };
    assert_eq!((layers(&plain), layers(&ten)), (Some(6), Some(10)));
}

#[test]
fn an_input_naming_no_single_design_is_rejected_with_why() {
    let cases = [
        (
            "def_file = \"d.def\"\nlef_files = [\"t.lef\"]\nbookshelf_aux_file = \"d.aux\"\n",
            InputError::TwoDesigns,
        ),
        ("sdc_file = \"d.sdc\"\n", InputError::NoDesign),
        ("def_file = \"d.def\"\n", InputError::NoLefFiles),
        (
            "bookshelf_aux_file = \"d.aux\"\nlef_files = [\"t.lef\"]\n",
            InputError::LefFilesWithBookshelf,
        ),
        (
            "def_file = \"d.def\"\nlef_files = [\"t.lef\"]\nnum_layers = 4\n",
            InputError::NumLayersWithDef,
        ),
    ];

    for (input, error) in cases {
        assert_eq!(
            parse_input(input).unwrap_err(),
            error.to_string(),
            "{input}"
        );
    }
}

#[test]
fn every_step_runs_unless_the_config_turns_it_off() {
    let config = parse("").unwrap();

    let netlist = &config.netlist;
    let gp = &config.global_placement;
    assert!(netlist.remove_buffers && netlist.buffer_ports && netlist.clock_tree);
    assert!(netlist.repair_design && netlist.repair_timing && netlist.repair_on_global_routes);
    assert!(gp.routability_driven && gp.timing_driven);
}

#[test]
fn pins_are_placed_on_the_two_layers_named() {
    let config =
        parse("[io_placement]\nhorizontal_layer = \"metal5\"\nvertical_layer = \"metal6\"\n")
            .unwrap();

    assert_eq!(
        config.io_placement.layers,
        Some(IoLayerNames {
            horizontal: "metal5".to_string(),
            vertical: "metal6".to_string(),
        })
    );
}

#[test]
fn pins_keep_their_places_unless_layers_are_named() {
    let config = parse("").unwrap();

    assert_eq!(config.io_placement.layers, None);
}

#[test]
fn one_pin_layer_without_the_other_is_rejected() {
    let error = parse("[io_placement]\nvertical_layer = \"metal6\"\n").unwrap_err();

    assert_eq!(error.message(), OneIoLayer.to_string());
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
