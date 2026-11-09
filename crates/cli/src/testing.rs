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

/// A file of the CTS crate's test fixtures, whose Liberty library a
/// repair links to.
pub(crate) fn cts_fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../cts/tests/fixtures")
        .join(name)
}

/// The shared tiny fixture design.
pub(crate) fn tiny() -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse(&fixture("tiny.def"), library).unwrap()
}

/// The tiny design on a die half as tall as it is wide.
pub(crate) fn half_height_tiny() -> NetlistDB {
    let mut db = tiny();
    db.die_area.max.y = db.die_area.min.y + db.die_area.height() / 2;
    db
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

/// A file of the CLI's own test fixtures.
fn cli_fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures")
        .join(name)
}

/// The timed fixture design: four flops over the clock tree synthesis
/// tests' cells.
pub(crate) fn timed() -> NetlistDB {
    let units = def::read_units(&cli_fixture("timed.def")).unwrap();
    let library = lef::parse_files(&[cts_fixture("clock.lef")], units).unwrap();
    def::parse(&cli_fixture("timed.def"), library).unwrap()
}

/// A config for the timed design, with its Liberty library, constraints
/// and wires and the TOML `netlist` keys, writing its results into `out`.
pub(crate) fn timed_config(netlist: &str, out: &Path) -> Config {
    let quoted = |p: &Path| format!("{:?}", p.display().to_string());
    let wire = |layer: &str| {
        format!(
            "{{ layer = \"{layer}\", resistance_ohm_per_um = 3.0, capacitance_ff_per_um = 0.1 }}"
        )
    };
    toml::from_str(&format!(
        "purpose = \"place_and_route\"\n\
         [global_placement]\n\
         placer_max_iterations = 100\n\
         [routing]\n\
         min_layer = \"metal1\"\n\
         [netlist]\n\
         signal_wire = {}\n\
         clock_wire = {}\n\
         {netlist}\n\
         [input]\n\
         lef_files = [{}]\n\
         def_file = {}\n\
         liberty_files = [{}]\n\
         sdc_file = {}\n\
         output_def = {}\n",
        wire("metal2"),
        wire("metal3"),
        quoted(&cts_fixture("clock.lef")),
        quoted(&cli_fixture("timed.def")),
        quoted(&cts_fixture("clock.lib")),
        quoted(&cli_fixture("timed.sdc")),
        quoted(&out.join("placed.def")),
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
