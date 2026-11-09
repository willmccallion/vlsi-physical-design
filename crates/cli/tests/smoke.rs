//! Runs the `pare` binary on small fixture designs, as a user would:
//! placing, routing, checking and viewing them, with and without timing.

#![allow(clippy::unwrap_used)]

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

/// Global placement iterations, few enough to keep the run quick; the
/// fixture's RAM keeps overflow above the target however long it runs.
const PLACER_ITERATIONS: usize = 100;

/// One detailed routing round keeps routing quick; the tiny design never
/// routes clean, since its input already breaks rules.
const QUICK_ROUTING: &str = "[routing]\ndetailed_iterations = 1\n";

/// A file of the test fixtures of the workspace crate in `dir`.
fn fixture_of(dir: &str, name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join(dir)
        .join("tests/fixtures")
        .join(name)
}

fn fixture(name: &str) -> PathBuf {
    fixture_of("common", name)
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

/// `path` as a TOML string.
fn quoted(path: &Path) -> String {
    format!("{:?}", path.display().to_string())
}

/// Writes a config of `purpose` reading the design `input`, a TOML
/// `[input]` table's keys, into `dir`, with the TOML `tables` after it, and
/// returns its path. The run writes its results into `dir` too.
fn write_config_for(dir: &Path, purpose: &str, input: &str, tables: &str) -> PathBuf {
    let config = dir.join("design.toml");
    std::fs::write(
        &config,
        format!(
            "purpose = \"{purpose}\"\n\
             [global_placement]\n\
             placer_max_iterations = {PLACER_ITERATIONS}\n\
             [input]\n\
             {input}\
             output_def = {}\n\
             {tables}",
            quoted(&dir.join("placed.def")),
        ),
    )
    .unwrap();
    config
}

/// Writes a config for the tiny design of `purpose` into `dir`, with the
/// TOML `tables` after its input, and returns its path.
fn write_config(dir: &Path, purpose: &str, tables: &str) -> PathBuf {
    let input = format!(
        "lef_files = [{}]\ndef_file = {}\n",
        quoted(&fixture("tiny.lef")),
        quoted(&fixture("tiny.def")),
    );
    write_config_for(dir, purpose, &input, tables)
}

/// Writes a config for the timed fixture design, with its Liberty library
/// and constraints, into `dir`, and returns its path.
fn write_timed_config(dir: &Path) -> PathBuf {
    let input = format!(
        "lef_files = [{}]\ndef_file = {}\nliberty_files = [{}]\nsdc_file = {}\n",
        quoted(&fixture_of("cts", "clock.lef")),
        quoted(&fixture_of("cli", "timed.def")),
        quoted(&fixture_of("cts", "clock.lib")),
        quoted(&fixture_of("cli", "timed.sdc")),
    );
    let wire = |layer: &str| {
        format!(
            "{{ layer = \"{layer}\", resistance_ohm_per_um = 3.0, capacitance_ff_per_um = 0.1 }}"
        )
    };
    let tables = format!(
        "[netlist]\nsignal_wire = {}\nclock_wire = {}\n\
         [routing]\nmin_layer = \"metal1\"\ndetailed_iterations = 1\n",
        wire("metal2"),
        wire("metal3"),
    );
    write_config_for(dir, "place_and_route", &input, &tables)
}

/// Writes a config for the Bookshelf fixture design, a placement
/// benchmark, into `dir`, and returns its path.
fn write_bookshelf_config(dir: &Path) -> PathBuf {
    let input = format!(
        "bookshelf_aux_file = {}\n",
        quoted(&fixture("bookshelf/tiny.aux"))
    );
    write_config_for(dir, "placement", &input, "")
}

fn pare(args: &[&Path]) -> Output {
    pare_in(Path::new(env!("CARGO_MANIFEST_DIR")), args)
}

/// Runs `pare` with `args` from the directory `cwd`.
fn pare_in(cwd: &Path, args: &[&Path]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_pare"))
        .current_dir(cwd)
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

/// The JSON report at `path`.
fn read_report(path: &Path) -> serde_json::Value {
    serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap()
}

/// The names of the phases `report` timed, in order.
fn phase_names(report: &serde_json::Value) -> Vec<String> {
    report["phases"]
        .as_array()
        .unwrap()
        .iter()
        .map(|phase| phase["name"].as_str().unwrap().to_string())
        .collect()
}

/// Places the tiny design with a config in `dir` that adds the TOML
/// `tables`, returning the config and the placed DEF.
fn place_tiny(dir: &Path, tables: &str) -> (PathBuf, PathBuf) {
    let config = write_config(dir, "place_and_route", tables);
    let output = pare(&[Path::new("place"), &config]);
    assert!(output.status.success(), "{}", printed(&output));
    (config, dir.join("placed.def"))
}

#[test]
fn placing_the_tiny_design_writes_a_legal_placed_def() {
    let dir = work_dir("place");
    let config = write_config(&dir, "place_and_route", "");

    let output = pare(&[Path::new("place"), &config]);

    assert!(output.status.success(), "{}", printed(&output));
    assert!(printed(&output).contains("Placement is legal."));
    let placed = std::fs::read_to_string(dir.join("placed.def")).unwrap();
    assert!(placed.contains("DESIGN tiny ;"));
    assert!(!placed.contains("UNPLACED"), "{placed}");
}

#[test]
fn refining_a_given_placement_writes_a_legal_placed_def() {
    let dir = work_dir("place-from");
    let (config, placed) = place_tiny(&dir, "");
    let given = dir.join("given.def");
    std::fs::rename(&placed, &given).unwrap();

    let output = pare(&[Path::new("place"), &config, Path::new("--from"), &given]);

    assert!(output.status.success(), "{}", printed(&output));
    assert!(printed(&output).contains("Starting from the given placement"));
    assert!(printed(&output).contains("Placement is legal."));
    assert!(placed.exists());
}

#[test]
fn placing_a_bookshelf_design_writes_its_placement_as_pl() {
    let dir = work_dir("bookshelf");
    let config = write_bookshelf_config(&dir);

    let output = pare(&[Path::new("place"), &config]);

    assert!(output.status.success(), "{}", printed(&output));
    let placed = std::fs::read_to_string(dir.join("placed.pl")).unwrap();
    assert!(placed.starts_with("UCLA pl 1.0"), "{placed}");
}

#[test]
fn routing_a_placed_design_globally_writes_a_guide_for_each_net() {
    let dir = work_dir("route");
    let (config, placed) = place_tiny(&dir, "");
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
fn routing_a_placed_design_in_detail_writes_and_checks_the_routed_def() {
    let dir = work_dir("route-detailed");
    let (config, placed) = place_tiny(&dir, QUICK_ROUTING);
    let report = dir.join("report.json");

    let output = pare(&[
        Path::new("route"),
        &config,
        Path::new("--placement"),
        &placed,
        Path::new("--report"),
        &report,
    ]);

    let printed = printed(&output);
    assert!(
        printed.contains("Error: routing has opens or design rule violations"),
        "{printed}"
    );
    let routed = std::fs::read_to_string(dir.join("routed.def")).unwrap();
    assert!(routed.contains("DESIGN tiny ;"));
    assert!(dir.join("routed.png").exists());
    let report = read_report(&report);
    assert_eq!(report["routing"]["nets"], 5);
    assert_eq!(
        phase_names(&report),
        ["routing", "verification", "routing output"]
    );
}

#[test]
fn a_saved_routing_state_is_resumed_from() {
    let dir = work_dir("resume");
    let (config, placed) = place_tiny(&dir, QUICK_ROUTING);
    let state = dir.join("tiny.state");
    let route = |extra: &[&Path]| {
        let mut args = vec![
            Path::new("route"),
            &config,
            Path::new("--placement"),
            &placed,
        ];
        args.extend_from_slice(extra);
        printed(&pare(&args))
    };

    let saved = route(&[
        Path::new("--save-state"),
        &state,
        Path::new("--save-after"),
        Path::new("global"),
    ]);
    let resumed = route(&[Path::new("--resume"), &state]);

    assert!(state.exists(), "{saved}");
    assert!(resumed.contains("Global restored:"), "{resumed}");
}

#[test]
fn running_the_tiny_design_places_routes_and_reports_both() {
    let dir = work_dir("run");
    let config = write_config(&dir, "place_and_route", QUICK_ROUTING);
    let report = dir.join("report.json");

    let output = pare(&[Path::new("run"), &config, Path::new("--report"), &report]);

    assert!(!output.status.success());
    assert!(dir.join("placed.def").exists());
    assert!(dir.join("routed.def").exists());
    let report = read_report(&report);
    assert_eq!(report["placement"]["cells"], 6);
    assert_eq!(report["routing"]["nets"], 5);
    assert_eq!(report["passed"], false);
}

#[test]
fn placing_a_timed_design_repairs_it_and_builds_its_clock_tree() {
    let dir = work_dir("timed-place");
    let config = write_timed_config(&dir);
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(output.status.success(), "{}", printed(&output));
    let report = read_report(&report);
    assert_eq!(
        phase_names(&report),
        [
            "buffer removal",
            "port buffering",
            "global placement",
            "design repair",
            "legalization",
            "detailed placement",
            "clock tree synthesis",
            "legalization after clock tree synthesis",
            "timing repair",
            "legalization after timing repair",
            "placement output",
        ]
    );
    assert_eq!(report["netlist"]["buffers_removed"], 1);
    assert_eq!(report["clock_tree"]["sinks"], 4);
    assert!(report["timing"]["wns_after_ns"].is_number());
}

#[test]
fn running_a_timed_design_repairs_it_on_its_global_routes() {
    let dir = work_dir("timed-run");
    let config = write_timed_config(&dir);
    let report = dir.join("report.json");

    let _ = pare(&[Path::new("run"), &config, Path::new("--report"), &report]);

    let report = read_report(&report);
    let phases = phase_names(&report);
    let after_placement: Vec<&str> = phases
        .iter()
        .map(String::as_str)
        .skip_while(|&name| name != "placement output")
        .collect();
    assert_eq!(
        after_placement,
        [
            "placement output",
            "global routing",
            "design repair on global routes",
            "legalization on global routes",
            "timing repair on global routes",
            "legalization on global routes",
            "routing",
            "verification",
            "routing output",
        ]
    );
    assert!(report["global_route_repair"]["timing"].is_object());
}

#[test]
fn viewing_an_area_draws_it_and_lists_its_violations() {
    let dir = work_dir("view");
    let config = write_config(&dir, "place_and_route", "");
    let image = dir.join("area.png");

    let output = pare(&[
        Path::new("view"),
        &config,
        Path::new("--box"),
        Path::new("0"),
        Path::new("0"),
        Path::new("6"),
        Path::new("6"),
        Path::new("--out"),
        &image,
        Path::new("--size"),
        Path::new("100"),
    ]);

    assert!(output.status.success(), "{}", printed(&output));
    let listed = String::from_utf8_lossy(&output.stdout);
    assert!(
        listed
            .lines()
            .any(|line| line
                == "Short on M3 at (4.8000, 3.4000) - (5.1000, 3.6000) between VDD and n3"),
        "{listed}"
    );
    assert_eq!(image::image_dimensions(&image).unwrap(), (100, 100));
}

#[test]
fn checking_an_unrouted_placement_passes_the_placement_and_fails_the_routing() {
    let dir = work_dir("check");
    let (config, placed) = place_tiny(&dir, "");
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
    let report = read_report(&report);
    assert_eq!(report["design"], "tiny");
    assert_eq!(report["passed"], false);
}

#[test]
fn checking_a_placement_benchmark_checks_only_its_placement() {
    let dir = work_dir("check-placement");
    let config = write_bookshelf_config(&dir);
    let placed = pare(&[Path::new("place"), &config]);
    assert!(placed.status.success(), "{}", printed(&placed));

    let output = pare(&[
        Path::new("check"),
        &config,
        Path::new("--placement"),
        &dir.join("placed.pl"),
    ]);

    assert!(output.status.success(), "{}", printed(&output));
    assert!(!printed(&output).contains("Opens:"), "{}", printed(&output));
}

#[test]
fn placing_a_routing_benchmark_is_refused() {
    let dir = work_dir("refuse-place");
    let config = write_config(&dir, "routing", "");

    let output = pare(&[Path::new("place"), &config]);

    assert!(!output.status.success());
    assert!(
        printed(&output).contains("is a routing benchmark: its placement is part of the input"),
        "{}",
        printed(&output)
    );
}

#[test]
fn routing_a_placement_benchmark_is_refused() {
    let dir = work_dir("refuse-route");
    let config = write_config(&dir, "placement", "");

    let output = pare(&[Path::new("route"), &config]);

    assert!(!output.status.success());
    assert!(
        printed(&output).contains("is a placement benchmark and is not routed"),
        "{}",
        printed(&output)
    );
}

#[test]
fn an_unknown_design_says_where_to_get_its_config() {
    let dir = work_dir("unknown");
    std::fs::create_dir(dir.join("configs")).unwrap();

    let output = pare_in(&dir, &[Path::new("check"), Path::new("no-such-design")]);

    assert!(!output.status.success());
    assert!(
        printed(&output).contains(
            "no config for 'no-such-design' in configs/<suite>/; download it with \
             scripts/download.py"
        ),
        "{}",
        printed(&output)
    );
}

#[test]
fn a_report_that_cannot_be_written_is_named_without_hiding_the_run_error() {
    let dir = work_dir("report-unwritable");
    std::fs::create_dir(dir.join("configs")).unwrap();
    let report = dir.join("missing").join("report.json");

    let output = pare_in(
        &dir,
        &[
            Path::new("check"),
            Path::new("no-such-design"),
            Path::new("--report"),
            &report,
        ],
    );

    assert!(!output.status.success());
    let printed = printed(&output);
    assert!(
        printed.contains("Error: no config for 'no-such-design'"),
        "{printed}"
    );
    assert!(
        printed.contains(&format!("cannot write report {}", report.display())),
        "{printed}"
    );
}

#[test]
fn a_report_that_cannot_be_written_fails_a_run_that_passed() {
    let dir = work_dir("report-unwritable-pass");
    let config = write_config(&dir, "place_and_route", "");
    let report = dir.join("missing").join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(!output.status.success());
    assert!(
        printed(&output).contains(&format!("Error: cannot write report {}", report.display())),
        "{}",
        printed(&output)
    );
}

#[test]
fn an_image_that_cannot_be_written_fails_the_run() {
    let dir = work_dir("image-unwritable");
    let config = write_config(&dir, "place_and_route", "");
    let image = dir.join("nesterov_placer.png");
    std::fs::create_dir(&image).unwrap();

    let output = pare(&[Path::new("place"), &config]);

    assert!(!output.status.success());
    assert!(
        printed(&output).contains(&format!("Error: cannot write {}", image.display())),
        "{}",
        printed(&output)
    );
}
