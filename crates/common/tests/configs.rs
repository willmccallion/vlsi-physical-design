//! Every shipped run configuration must parse.

#![allow(clippy::unwrap_used)]

use pare_common::util::config::Config;
use std::path::Path;

#[test]
fn every_place_config_parses() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../configs/place");
    let mut count = 0;

    for entry in std::fs::read_dir(&dir).unwrap() {
        let path = entry.unwrap().path();
        let text = std::fs::read_to_string(&path).unwrap();
        let parsed: Result<Config, _> = toml::from_str(&text);
        assert!(parsed.is_ok(), "{}: {:?}", path.display(), parsed.err());
        count += 1;
    }

    assert!(count > 0, "no configs found in {}", dir.display());
}

#[test]
fn unknown_keys_are_rejected() {
    let parsed: Result<Config, _> = toml::from_str("[global_placement]\ntarget_densty = 0.7\n");

    assert!(parsed.is_err());
}
