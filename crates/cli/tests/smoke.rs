//! Runs the `pare` binary on small fixture designs, as a user would:
//! placing, routing, checking and viewing them, with and without timing.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{def, lef};
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

/// The parts of a test config that vary between tests.
#[derive(Default)]
struct ConfigParts<'a> {
    /// What the design is for: `placement`, `routing` or `place_and_route`.
    purpose: &'a str,
    /// Top-level keys after the purpose.
    top_level: &'a str,
    /// `[global_placement]` keys besides the iteration limit.
    global_placement: &'a str,
    /// `[input]` keys naming the design, without the output DEF.
    input: String,
    /// The TOML tables after `[input]`.
    tables: String,
}

/// Writes a config of `parts` into `dir` and returns its path. The run
/// writes its results into `dir` too.
fn write_config_parts(dir: &Path, parts: &ConfigParts<'_>) -> PathBuf {
    let config = dir.join("design.toml");
    std::fs::write(
        &config,
        format!(
            "purpose = \"{}\"\n\
             {}\n\
             [global_placement]\n\
             placer_max_iterations = {PLACER_ITERATIONS}\n\
             {}\n\
             [input]\n\
             {}\
             output_def = {}\n\
             {}",
            parts.purpose,
            parts.top_level,
            parts.global_placement,
            parts.input,
            quoted(&dir.join("placed.def")),
            parts.tables,
        ),
    )
    .unwrap();
    config
}

/// The `[input]` keys of the tiny design read from the DEF at `def`.
fn tiny_input(def: &Path) -> String {
    format!(
        "lef_files = [{}]\ndef_file = {}\n",
        quoted(&fixture("tiny.lef")),
        quoted(def),
    )
}

/// Writes a config for the tiny design of `purpose` into `dir`, with the
/// TOML `tables` after its input, and returns its path.
fn write_config(dir: &Path, purpose: &str, tables: &str) -> PathBuf {
    write_config_parts(
        dir,
        &ConfigParts {
            purpose,
            input: tiny_input(&fixture("tiny.def")),
            tables: tables.to_string(),
            ..ConfigParts::default()
        },
    )
}

/// A run of a timed design over the clock tree synthesis tests' cells.
struct TimedRun<'a> {
    /// The design.
    def: PathBuf,
    /// Its constraints.
    sdc: PathBuf,
    /// The signal wire's resistance in ohms and capacitance in femtofarads,
    /// per micron.
    signal_rc: (f64, f64),
    /// `[netlist]` keys besides the wires.
    netlist: &'a str,
    /// `[global_placement]` keys besides the iteration limit.
    global_placement: &'a str,
    /// TOML tables after `[routing]`.
    tables: &'a str,
}

impl Default for TimedRun<'_> {
    fn default() -> Self {
        Self {
            def: fixture_of("cli", "timed.def"),
            sdc: fixture_of("cli", "timed.sdc"),
            signal_rc: (3.0, 0.1),
            netlist: "",
            global_placement: "",
            tables: "",
        }
    }
}

/// Writes a config for `run`, with the Liberty library of its cells, into
/// `dir`, and returns its path.
fn write_timed_run(dir: &Path, run: &TimedRun<'_>) -> PathBuf {
    let input = format!(
        "lef_files = [{}]\ndef_file = {}\nliberty_files = [{}]\nsdc_file = {}\n",
        quoted(&fixture_of("cts", "clock.lef")),
        quoted(&run.def),
        quoted(&fixture_of("cts", "clock.lib")),
        quoted(&run.sdc),
    );
    let wire = |layer: &str, (resistance, capacitance): (f64, f64)| {
        format!(
            "{{ layer = \"{layer}\", resistance_ohm_per_um = {resistance:?}, \
             capacitance_ff_per_um = {capacitance:?} }}"
        )
    };
    let tables = format!(
        "[netlist]\n{}\nsignal_wire = {}\nclock_wire = {}\n\
         [routing]\nmin_layer = \"metal1\"\ndetailed_iterations = 1\n{}",
        run.netlist,
        wire("metal2", run.signal_rc),
        wire("metal3", (3.0, 0.1)),
        run.tables,
    );
    write_config_parts(
        dir,
        &ConfigParts {
            purpose: "place_and_route",
            global_placement: run.global_placement,
            input,
            tables,
            ..ConfigParts::default()
        },
    )
}

/// Writes a config for the timed fixture design, with its Liberty library
/// and constraints and the TOML `netlist` keys, into `dir`, and returns its
/// path.
fn write_timed_config(dir: &Path, netlist: &str) -> PathBuf {
    write_timed_run(
        dir,
        &TimedRun {
            netlist,
            ..TimedRun::default()
        },
    )
}

/// Writes a config for the Bookshelf fixture design, a placement
/// benchmark, with the top-level keys `top_level`, into `dir`, and returns
/// its path.
fn write_bookshelf_config(dir: &Path, top_level: &str) -> PathBuf {
    write_config_parts(
        dir,
        &ConfigParts {
            purpose: "placement",
            top_level,
            input: format!(
                "bookshelf_aux_file = {}\n",
                quoted(&fixture("bookshelf/tiny.aux"))
            ),
            ..ConfigParts::default()
        },
    )
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
fn refining_a_given_placement_reports_it_without_a_global_placement() {
    let dir = work_dir("place-from-report");
    let (config, placed) = place_tiny(&dir, "");
    let given = dir.join("given.def");
    std::fs::rename(&placed, &given).unwrap();
    let report = dir.join("report.json");

    let output = pare(&[
        Path::new("place"),
        &config,
        Path::new("--from"),
        &given,
        Path::new("--report"),
        &report,
    ]);

    assert!(output.status.success(), "{}", printed(&output));
    let placement = &read_report(&report)["placement"];
    assert!(placement["global"].is_null(), "{placement}");
    assert!(placement["detailed_moves"].is_number(), "{placement}");
    assert!(placement["final_hpwl_um"].is_number(), "{placement}");
}

/// The first word after `label` in `line`.
fn word_after<'a>(line: &'a str, label: &str) -> &'a str {
    let after = &line[line.find(label).unwrap() + label.len()..];
    after.split_whitespace().next().unwrap()
}

/// The last half-perimeter wirelength the run printed.
fn last_printed_hpwl(output: &Output) -> String {
    const LABEL: &str = "HPWL (um):";
    let printed = printed(output);
    let line = printed.lines().rfind(|line| line.contains(LABEL)).unwrap();
    word_after(line, LABEL).to_string()
}

/// The value the run printed after `label`, on the first line with it.
fn printed_stat(printed: &str, label: &str) -> String {
    let line = printed
        .lines()
        .find(|line| line.contains(label))
        .unwrap_or_else(|| panic!("no {label} in:\n{printed}"));
    word_after(line, label).to_string()
}

/// The design of the DEF at `def` over the cells of the LEF at `lef`.
fn read_design(lef: &Path, def: &Path) -> NetlistDB {
    let units = def::read_units(def).unwrap();
    let library = lef::parse_files(&[lef.to_path_buf()], units).unwrap();
    def::parse(def, library).unwrap()
}

#[test]
fn the_final_wirelength_is_that_of_the_placement_written_out() {
    let dir = work_dir("final-hpwl");
    let config = write_timed_config(&dir, "repair_timing = false\n");
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(output.status.success(), "{}", printed(&output));
    let placement = &read_report(&report)["placement"];
    let final_hpwl = placement["final_hpwl_um"].as_f64().unwrap();
    assert_eq!(format!("{final_hpwl:.0}"), last_printed_hpwl(&output));
    let written = read_design(&fixture_of("cts", "clock.lef"), &dir.join("placed.def"));
    let written_hpwl = written
        .dbu()
        .to_microns(pare_placer::legalize::opendp::mirror::hpwl(&written));
    assert!(
        (final_hpwl - written_hpwl).abs() < 1e-9,
        "{final_hpwl} {written_hpwl}"
    );
}

#[test]
fn placing_a_bookshelf_design_writes_its_placement_as_pl() {
    let dir = work_dir("bookshelf");
    let config = write_bookshelf_config(&dir, "");

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
    let written = read_design(&fixture("tiny.lef"), &dir.join("routed.def"));
    let units = written.dbu();
    let written_um: f64 = pare_verify::routing::check_routing(&written)
        .stats
        .wirelength
        .iter()
        .map(|&length| units.to_microns(length))
        .sum();
    let reported_um = report["checks"]["wirelength_um"].as_f64().unwrap();
    assert!(written_um > 0.0);
    assert!(
        (reported_um - written_um).abs() < 1e-9,
        "{reported_um} {written_um}"
    );
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
    assert!(!saved.contains("no state was saved"), "{saved}");
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
    assert_eq!(report["placement"]["global"]["cells"], 6);
    assert_eq!(report["routing"]["nets"], 5);
    assert_eq!(report["passed"], false);
}

#[test]
fn placing_a_timed_design_repairs_it_and_builds_its_clock_tree() {
    let dir = work_dir("timed-place");
    let config = write_timed_config(&dir, "");
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
    assert_eq!(report["netlist"]["input_buffers"], 1);
    assert_eq!(report["netlist"]["output_buffers"], 1);
    assert_eq!(report["clock_tree"]["sinks"], 4);
    assert_eq!(report["clock_tree"]["cloned_inverters"], 0);
    assert_eq!(report["clock_tree"]["clock_net_repeaters"], 0);
    assert!(report["timing"]["wns_after_ns"].is_number());
    let placed = std::fs::read_to_string(dir.join("placed.def")).unwrap();
    assert!(
        !placed.contains("- b0 "),
        "the synthesis buffer is left:\n{placed}"
    );
}

#[test]
fn a_timed_design_without_a_clock_wire_fails_before_placing() {
    let dir = work_dir("timed-no-clock-wire");
    let config = write_timed_config(&dir, "");
    let text = std::fs::read_to_string(&config).unwrap();
    let without: Vec<&str> = text
        .lines()
        .filter(|line| !line.starts_with("clock_wire"))
        .collect();
    std::fs::write(&config, without.join("\n")).unwrap();
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(!output.status.success());
    assert!(
        printed(&output).contains("Error: [netlist] clock trees need clock_wire"),
        "{}",
        printed(&output)
    );
    assert_eq!(phase_names(&read_report(&report)), Vec::<String>::new());
}

#[test]
fn running_a_timed_design_repairs_it_on_its_global_routes() {
    let dir = work_dir("timed-run");
    let config = write_timed_config(&dir, "");
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
    let config = write_bookshelf_config(&dir, "");
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

#[test]
fn placing_shows_how_global_placement_ended_and_the_cells_legalization_moved() {
    let dir = work_dir("place-shown");
    let config = write_config(&dir, "place_and_route", "");

    let output = pare(&[Path::new("place"), &config]);

    let printed = printed(&output);
    assert!(output.status.success(), "{printed}");
    assert!(printed.contains("Final congestion:"), "{printed}");
    assert!(
        printed.contains("Stopped at the iteration limit"),
        "{printed}"
    );
    assert!(printed.contains("Legalized 6 cells"), "{printed}");
    assert!(!printed.contains("Zone "), "{printed}");
}

/// Writes the tiny design with its cells `u1` and `u2` fenced into the
/// region `left` into `dir`, returning the DEF's path.
fn write_fenced_tiny(dir: &Path) -> PathBuf {
    let text = std::fs::read_to_string(fixture("tiny.def")).unwrap();
    let fenced = text
        .replace(
            "COMPONENTS 8 ;",
            "REGIONS 1 ;\n- left ( 0 0 ) ( 2000 4000 ) + TYPE FENCE ;\nEND REGIONS\n\nCOMPONENTS 8 ;",
        )
        .replace(
            "END NETS\n",
            "END NETS\n\nGROUPS 1 ;\n- left_cells u1 u2 + REGION left ;\nEND GROUPS\n",
        );
    let def = dir.join("fenced.def");
    std::fs::write(&def, fenced).unwrap();
    def
}

#[test]
fn placing_a_fenced_design_shows_each_zone_of_global_placement() {
    let dir = work_dir("fenced");
    let config = write_config_parts(
        &dir,
        &ConfigParts {
            purpose: "place_and_route",
            input: tiny_input(&write_fenced_tiny(&dir)),
            ..ConfigParts::default()
        },
    );

    let output = pare(&[Path::new("place"), &config]);

    let printed = printed(&output);
    assert!(output.status.success(), "{printed}");
    assert!(printed.contains("Zone left:"), "{printed}");
    assert!(printed.contains("Zone outside fences:"), "{printed}");
}

#[test]
fn detailed_routing_shows_the_counts_its_report_records() {
    let dir = work_dir("route-shown");
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
    let report = read_report(&report);
    let checks = &report["checks"];
    assert_eq!(
        printed_stat(&printed, "Opens:"),
        checks["opens"].to_string()
    );
    assert_eq!(printed_stat(&printed, "Vias:"), checks["vias"].to_string());
    let violations: u64 = checks["violations"]
        .as_object()
        .unwrap()
        .values()
        .map(|count| count.as_u64().unwrap())
        .sum();
    assert_eq!(
        printed_stat(&printed, "DRC violations:"),
        violations.to_string()
    );
    assert_eq!(
        printed_stat(&printed, "Rounds:"),
        report["routing"]["rounds"].to_string()
    );
    assert_eq!(printed.matches("DR Iter").count(), 1, "{printed}");
}

#[test]
fn checking_a_design_with_nothing_to_route_passes() {
    let dir = work_dir("check-unconnected");
    let config = write_config(&dir, "place_and_route", "");

    let output = pare(&[
        Path::new("check"),
        &config,
        Path::new("--placement"),
        &fixture_of("cli", "unconnected_tiny.def"),
    ]);

    let printed = printed(&output);
    assert!(output.status.success(), "{printed}");
    assert!(
        printed.contains("PASS Routing is connected and clean."),
        "{printed}"
    );
}

#[test]
fn placing_a_timed_design_shows_what_each_repair_did() {
    let dir = work_dir("timed-shown");
    let config = write_timed_config(&dir, "");
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    let printed = printed(&output);
    assert!(output.status.success(), "{printed}");
    let report = read_report(&report);
    assert_eq!(
        printed_stat(&printed, "Slew violations:"),
        report["netlist"]["slew_violations"].to_string()
    );
    assert_eq!(
        printed_stat(&printed, "Sinks:"),
        report["clock_tree"]["sinks"].to_string()
    );
    assert_eq!(
        printed_stat(&printed, "Iterations:"),
        report["timing"]["iterations"].to_string()
    );
    assert_eq!(
        printed_stat(&printed, "Hold buffers:"),
        report["timing"]["hold_buffers"].to_string()
    );
}

#[test]
fn design_repair_fixes_the_slew_and_load_a_resistive_wire_breaks() {
    let dir = work_dir("resistive-repair");
    let config = write_timed_run(
        &dir,
        &TimedRun {
            signal_rc: (20_000.0, 50.0),
            netlist: "clock_tree = false\nrepair_timing = false",
            ..TimedRun::default()
        },
    );
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(output.status.success(), "{}", printed(&output));
    let netlist = &read_report(&report)["netlist"];
    assert!(
        netlist["slew_violations"].as_u64().unwrap() > 0,
        "{netlist}"
    );
    assert!(netlist["repeaters"].as_u64().unwrap() > 0, "{netlist}");
}

#[test]
fn clock_nets_longer_than_a_buffer_drives_are_split_by_repeaters() {
    let dir = work_dir("clock-repeaters");
    let config = write_timed_run(
        &dir,
        &TimedRun {
            signal_rc: (3_000.0, 10.0),
            ..TimedRun::default()
        },
    );
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(output.status.success(), "{}", printed(&output));
    let clock_tree = &read_report(&report)["clock_tree"];
    assert!(
        clock_tree["clock_net_repeaters"].as_u64().unwrap() > 0,
        "{clock_tree}"
    );
}

#[test]
fn a_clock_inverter_is_cloned_before_the_clock_tree_is_built() {
    let dir = work_dir("clock-inverter");
    let config = write_timed_run(
        &dir,
        &TimedRun {
            def: fixture_of("cli", "timed_chain.def"),
            ..TimedRun::default()
        },
    );
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(output.status.success(), "{}", printed(&output));
    assert_eq!(read_report(&report)["clock_tree"]["cloned_inverters"], 1);
}

#[test]
fn timing_driven_placement_repairs_and_reweights_at_each_overflow_it_passes() {
    let dir = work_dir("timing-driven");
    let sdc = dir.join("tight.sdc");
    std::fs::write(
        &sdc,
        "create_clock -name clk -period 0.05 [get_ports clk]\n",
    )
    .unwrap();
    let config = write_timed_run(
        &dir,
        &TimedRun {
            def: fixture_of("cli", "timed_chain.def"),
            sdc,
            global_placement: "timing_driven_overflows = [0.95]",
            ..TimedRun::default()
        },
    );
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(output.status.success(), "{}", printed(&output));
    assert_eq!(
        read_report(&report)["placement"]["global"]["timing_rounds"],
        1
    );
}

/// Writes the timed fixture design with its I/O pins unplaced into `dir`,
/// returning the DEF's path.
fn write_unplaced_io_timed(dir: &Path) -> PathBuf {
    let text = std::fs::read_to_string(fixture_of("cli", "timed.def")).unwrap();
    let unplaced: Vec<String> = text
        .lines()
        .map(|line| match line.find(" + FIXED (") {
            Some(at) if line.trim_start().starts_with("+ PORT") => format!("{} ;", &line[..at]),
            _ => line.to_string(),
        })
        .collect();
    let def = dir.join("unplaced_io.def");
    std::fs::write(&def, unplaced.join("\n")).unwrap();
    def
}

#[test]
fn unplaced_io_pins_are_placed_after_a_placement_without_them() {
    let dir = work_dir("io-placement");
    let config = write_timed_run(
        &dir,
        &TimedRun {
            def: write_unplaced_io_timed(&dir),
            tables: "[io_placement]\nhorizontal_layer = \"metal3\"\nvertical_layer = \"metal2\"\n",
            ..TimedRun::default()
        },
    );
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(output.status.success(), "{}", printed(&output));
    let phases = phase_names(&read_report(&report));
    assert_eq!(
        phases[..2],
        ["global placement without I/O", "I/O pin placement"]
    );
    let placed = std::fs::read_to_string(dir.join("placed.def")).unwrap();
    let pins = &placed[placed.find("PINS 3 ;").unwrap()..placed.find("END PINS").unwrap()];
    assert_eq!(pins.matches("+ PLACED").count(), 3, "{pins}");
}

#[test]
fn the_report_records_the_worker_threads_the_config_sets() {
    let dir = work_dir("threads");
    let config = write_bookshelf_config(&dir, "threads = 1");
    let report = dir.join("report.json");

    let output = pare(&[Path::new("place"), &config, Path::new("--report"), &report]);

    assert!(output.status.success(), "{}", printed(&output));
    assert_eq!(read_report(&report)["threads"], 1);
}
