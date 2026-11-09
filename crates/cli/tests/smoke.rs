//! Runs the `pare` binary on the tiny fixture design: placing it, routing
//! it globally and checking it, as a user would.

#![allow(clippy::unwrap_used)]

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

/// Global placement iterations, few enough to keep the run quick; the
/// fixture's RAM keeps overflow above the target however long it runs.
const PLACER_ITERATIONS: usize = 100;

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

/// An empty directory for one test's files.
fn work_dir(tag: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("smoke-{tag}"));
    if dir.exists() {
        std::fs::remove_dir_all(&dir).unwrap();
    }
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Writes a config for the tiny design of `purpose` into `dir`, writing
/// its results there too, and returns its path.
fn write_config(dir: &Path, purpose: &str) -> PathBuf {
    let quoted = |p: &Path| format!("{:?}", p.display().to_string());
    let config = dir.join("tiny.toml");
    std::fs::write(
        &config,
        format!(
            "purpose = \"{purpose}\"\n\
             [global_placement]\n\
             placer_max_iterations = {PLACER_ITERATIONS}\n\
             [input]\n\
             lef_files = [{}]\n\
             def_file = {}\n\
             output_def = {}\n",
            quoted(&fixture("tiny.lef")),
            quoted(&fixture("tiny.def")),
            quoted(&dir.join("placed.def")),
        ),
    )
    .unwrap();
    config
}

fn pare(args: &[&Path]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_pare"))
        .args(args)
        .output()
        .unwrap()
}

/// Everything the run printed.
fn printed(output: &Output) -> String {
    format!(
        "{}{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    )
}

/// Places the tiny design with a config in `dir`, returning the config
/// and the placed DEF.
fn place_tiny(dir: &Path) -> (PathBuf, PathBuf) {
    let config = write_config(dir, "place_and_route");
    let output = pare(&[Path::new("place"), &config]);
    assert!(output.status.success(), "{}", printed(&output));
    (config, dir.join("placed.def"))
}

#[test]
fn placing_the_tiny_design_writes_a_legal_placed_def() {
    let dir = work_dir("place");
    let config = write_config(&dir, "place_and_route");

    let output = pare(&[Path::new("place"), &config]);

    assert!(output.status.success(), "{}", printed(&output));
    assert!(printed(&output).contains("Placement is legal."));
    let placed = std::fs::read_to_string(dir.join("placed.def")).unwrap();
    assert!(placed.contains("DESIGN tiny ;"));
    assert!(!placed.contains("UNPLACED"), "{placed}");
}

#[test]
fn routing_a_placed_design_globally_writes_a_guide_for_each_net() {
    let dir = work_dir("route");
    let (config, placed) = place_tiny(&dir);
    let guides = dir.join("tiny.guide");

    let output = pare(&[
        Path::new("route"),
        &config,
        Path::new("--placement"),
        &placed,
        Path::new("--global-only"),
        Path::new("--guides"),
        &guides,
    ]);

    assert!(output.status.success(), "{}", printed(&output));
    let text = std::fs::read_to_string(&guides).unwrap();
    for net in ["in", "n1", "n2", "n3", "out"] {
        assert!(
            text.lines().any(|line| line == net),
            "no guide for {net}:\n{text}"
        );
    }
}

#[test]
fn checking_an_unrouted_placement_passes_the_placement_and_fails_the_routing() {
    let dir = work_dir("check");
    let (config, placed) = place_tiny(&dir);
    let report = dir.join("report.json");

    let output = pare(&[
        Path::new("check"),
        &config,
        Path::new("--placement"),
        &placed,
        Path::new("--report"),
        &report,
    ]);

    assert!(!output.status.success());
    let printed = printed(&output);
    assert!(printed.contains("PASS Placement is legal."), "{printed}");
    assert!(printed.contains("tiny failed its checks"), "{printed}");
    let report: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&report).unwrap()).unwrap();
    assert_eq!(report["design"], "tiny");
    assert_eq!(report["passed"], false);
}

#[test]
fn placing_a_routing_benchmark_is_refused() {
    let dir = work_dir("refuse");
    let config = write_config(&dir, "routing");

    let output = pare(&[Path::new("place"), &config]);

    assert!(!output.status.success());
    assert!(
        printed(&output).contains("is a routing benchmark: its placement is part of the input"),
        "{}",
        printed(&output)
    );
}
