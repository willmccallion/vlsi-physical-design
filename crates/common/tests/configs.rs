//! Every shipped run configuration must parse.

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

#[test]
fn unknown_keys_are_rejected() {
    let parsed: Result<Config, _> = toml::from_str("[global_placement]\ntarget_densty = 0.7\n");

    assert!(parsed.is_err());
}
