//! Designs, configs and files the CLI's tests share.

use pare_common::config::Config;
use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{def, lef};
use std::path::{Path, PathBuf};

/// A file of the common crate's test fixtures.
pub(crate) fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

/// The shared tiny fixture design.
pub(crate) fn tiny() -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse(&fixture("tiny.def"), library).unwrap()
}

/// A config for the tiny design with the TOML `tables` beside its input.
pub(crate) fn tiny_config(tables: &str) -> Config {
    let quoted = |p: &Path| format!("{:?}", p.display().to_string());
    toml::from_str(&format!(
        "purpose = \"place_and_route\"\n\
         [input]\n\
         lef_files = [{}]\n\
         def_file = {}\n\
         output_def = \"output/tiny/placed.def\"\n\
         {tables}",
        quoted(&fixture("tiny.lef")),
        quoted(&fixture("tiny.def")),
    ))
    .unwrap()
}

/// An empty directory of this test process's own, named for `tag`.
pub(crate) fn scratch_dir(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("pare-cli-{tag}-{}", std::process::id()));
    if dir.exists() {
        std::fs::remove_dir_all(&dir).unwrap();
    }
    std::fs::create_dir_all(&dir).unwrap();
    dir
}
