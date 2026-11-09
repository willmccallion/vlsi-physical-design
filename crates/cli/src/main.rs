//! PARE -- Placement And Routing Engine.
//!
//! Loads a LEF/DEF or Bookshelf design, places and routes it, checks the
//! result and writes it out.

mod checks;
mod cts;
mod logger;
mod phase;
mod placement;
mod repair;
mod report;
mod routing;
#[cfg(test)]
mod testing;
mod ui;
mod view;
mod visualization;

use crate::checks::{report_placement, report_routing};
use crate::placement::{PlacementStart, run_placement};
use crate::routing::{
    StateFiles, global_route_with_repair, parse_save_point, route_detailed, route_global,
};
use crate::view::view_area;
use anyhow::Context;
use clap::{Parser, Subcommand};
use pare_common::config::{Config, DesignInput, Purpose};
use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{bookshelf, def, lef};
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use report::Report;
use std::path::{Path, PathBuf};
use std::time::Instant;

#[derive(Parser)]
#[command(author, version, about = "PARE -- Placement And Routing Engine", long_about = None)]
struct Args {
    #[command(subcommand)]
    command: Commands,
    /// Write a JSON report of the run (phases, results, checks, peak memory) here.
    #[arg(long, global = true)]
    report: Option<PathBuf>,
}

#[derive(Subcommand)]
enum Commands {
    /// Place a design.
    Place {
        /// Design name (e.g. 'ibex') or path to a TOML config.
        config: PathBuf,
        /// Placed DEF, or Bookshelf `.pl`, to legalize and refine instead of
        /// placing from scratch; global placement is skipped.
        #[arg(long)]
        from: Option<PathBuf>,
    },
    /// Route a placed design.
    Route(RouteArgs),
    /// Place, route, or place and route a design, as its purpose says.
    Run {
        /// Design name (e.g. 'ibex') or path to a TOML config.
        config: PathBuf,
    },
    /// Check the placement and routing of a design.
    Check {
        /// Design name (e.g. 'ibex') or path to a TOML config.
        config: PathBuf,
        /// DEF, or Bookshelf `.pl`, to check instead of the config's input.
        #[arg(long)]
        placement: Option<PathBuf>,
    },
    /// Draw one area of a design at its true geometry, with the design rule
    /// violations there outlined and listed.
    View {
        /// Design name (e.g. 'ibex') or path to a TOML config.
        config: PathBuf,
        /// DEF to draw instead of the config's input, such as a routed one.
        #[arg(long)]
        placement: Option<PathBuf>,
        /// The area to draw, in microns.
        #[arg(long = "box", num_args = 4, value_names = ["X0", "Y0", "X1", "Y1"], required = true)]
        area: Vec<f64>,
        /// A net to draw bright, the others dimmed. Repeatable.
        #[arg(long = "net")]
        nets: Vec<String>,
        /// Routing layers to draw, by name, with the cuts between them; all
        /// of them when not given. Repeatable.
        #[arg(long = "layer")]
        layers: Vec<String>,
        /// The PNG to write.
        #[arg(long)]
        out: PathBuf,
        /// Pixels along the area's longer side.
        #[arg(long, default_value_t = 1600)]
        size: u32,
    },
}

/// What the `route` command takes.
#[derive(clap::Args)]
struct RouteArgs {
    /// Design name (e.g. 'gcd') or path to a TOML config.
    config: PathBuf,
    /// Placed DEF, or Bookshelf `.pl`, to route instead of the config's input.
    #[arg(long)]
    placement: Option<PathBuf>,
    /// Stop after global routing and track assignment, reporting their
    /// result without routing in detail or writing a DEF.
    #[arg(long)]
    global_only: bool,
    /// With --global-only, write each net's global route to this file as
    /// route guides in the ISPD 2018 format.
    #[arg(long, requires = "global_only")]
    guides: Option<PathBuf>,
    /// Save the run's state to this file, to resume from with --resume.
    #[arg(long, conflicts_with = "global_only")]
    save_state: Option<PathBuf>,
    /// When to save the state: `global` (after global routing, the
    /// default) or a detailed round, 0 being the first pass.
    #[arg(long, requires = "save_state", value_parser = parse_save_point)]
    save_after: Option<pare_router::SavePoint>,
    /// Resume from a state saved by --save-state on the same design and
    /// placement, skipping what it holds.
    #[arg(long, conflicts_with = "global_only")]
    resume: Option<PathBuf>,
    /// Log this net's detailed routing in detail: each route found with
    /// what it cost, each region judgment and its violations. Repeatable.
    #[arg(long = "trace-net", conflicts_with = "global_only")]
    trace_nets: Vec<String>,
    /// Log detailed routing in detail in every region meeting this box,
    /// given in microns as X0 Y0 X1 Y1.
    #[arg(long, num_args = 4, value_names = ["X0", "Y0", "X1", "Y1"], conflicts_with = "global_only")]
    trace_box: Option<Vec<f64>>,
}

/// The directory of suites whose configs a design name finds.
const CONFIGS_DIR: &str = "configs";

/// Resolves a config argument to a file path.
///
/// An existing path, or anything containing '/' or ending in '.toml', is used
/// as is; otherwise it names a design whose config is
/// `<configs>/<suite>/<design>.toml`.
fn resolve_config(name: &Path, configs: &Path) -> anyhow::Result<PathBuf> {
    let text = name.to_str().context("config path is not valid UTF-8")?;
    if name.exists()
        || text.contains('/')
        || name
            .extension()
            .is_some_and(|ext| ext.eq_ignore_ascii_case("toml"))
    {
        return Ok(name.to_path_buf());
    }
    let file = format!("{text}.toml");
    let mut found = Vec::new();
    let suites = std::fs::read_dir(configs)
        .with_context(|| format!("cannot read {}/", configs.display()))?;
    for suite in suites {
        let candidate = suite?.path().join(&file);
        if candidate.is_file() {
            found.push(candidate);
        }
    }
    match found.as_slice() {
        [one] => Ok(one.clone()),
        [] => anyhow::bail!(
            "no config for '{text}' in {}/<suite>/; download it with scripts/download.py",
            configs.display()
        ),
        _ => anyhow::bail!("'{text}' names several configs: {found:?}"),
    }
}

/// Reads the config, records it in the report and sizes the thread pool as
/// it asks.
fn load_config(name: &Path, report: &mut Report) -> anyhow::Result<Config> {
    let path = resolve_config(name, Path::new(CONFIGS_DIR))?;
    ui::header("Config", &path.display().to_string());
    report.config = path.display().to_string();
    let text = std::fs::read_to_string(&path)
        .with_context(|| format!("cannot read config {}", path.display()))?;
    let config: Config =
        toml::from_str(&text).with_context(|| format!("invalid config {}", path.display()))?;
    if let Some(threads) = config.threads {
        rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build_global()
            .context("cannot size the thread pool")?;
    }
    Ok(config)
}

fn main() -> anyhow::Result<()> {
    logger::init();
    let args = Args::parse();
    let start_time = Instant::now();
    ui::banner();

    let mut report = Report::default();
    let outcome = run(args.command, &mut report);
    report.passed = outcome.is_ok();
    report.wall = start_time.elapsed();
    report.peak_rss_mb = report::peak_rss_mb();
    let written = args
        .report
        .as_deref()
        .map_or(Ok(()), |path| write_report(&report, path));
    if let (Err(_), Err(unwritten)) = (&outcome, &written) {
        log::error!("{unwritten:#}");
    }
    outcome?;
    written?;

    ui::done(report.wall);
    Ok(())
}

/// Writes `report` as JSON to `path` and says so.
fn write_report(report: &Report, path: &Path) -> anyhow::Result<()> {
    report.write(path)?;
    ui::wrote(path);
    Ok(())
}

/// Runs one command, recording what it did in `report`.
fn run(command: Commands, report: &mut Report) -> anyhow::Result<()> {
    match command {
        Commands::Place { config, from } => place(&config, from.as_deref(), report),
        Commands::Route(args) => route(args, report),
        Commands::Run { config } => place_and_route(&config, report),
        Commands::Check { config, placement } => check(&config, placement.as_deref(), report),
        Commands::View {
            config,
            placement,
            area,
            nets,
            layers,
            out,
            size,
        } => {
            let config = load_config(&config, report)?;
            let db = load_design(&config, placement.as_deref())?;
            view_area(
                &db,
                MicronBox::from_corners(&area)?.in_dbu(db.dbu()),
                &nets,
                &layers,
                &out,
                size,
            )
        }
    }
}

/// The `place` command: places the design `name` from scratch, or refines
/// the placement `from`.
fn place(name: &Path, from: Option<&Path>, report: &mut Report) -> anyhow::Result<()> {
    let config = load_config(name, report)?;
    if config.purpose == Purpose::Routing {
        anyhow::bail!(
            "{} is a routing benchmark: its placement is part of the input",
            name.display()
        );
    }
    let mut db = load_design(&config, from)?;
    report.design.clone_from(&db.name);
    let start = if from.is_some() {
        PlacementStart::Given
    } else {
        PlacementStart::Scratch
    };
    let _ = run_placement(
        &config,
        &mut db,
        &prepare_output_dir(&config)?,
        start,
        report,
    )?;
    Ok(())
}

/// The `route` command.
fn route(args: RouteArgs, report: &mut Report) -> anyhow::Result<()> {
    let config = load_config(&args.config, report)?;
    if config.purpose == Purpose::Placement {
        anyhow::bail!(
            "{} is a placement benchmark and is not routed",
            args.config.display()
        );
    }
    let mut db = load_design(&config, args.placement.as_deref())?;
    report.design.clone_from(&db.name);
    if args.global_only {
        return route_global(&config, &db, args.guides.as_deref(), report);
    }
    let files = state_files(args, db.dbu())?;
    route_detailed(&config, &mut db, &files, None, report)
}

/// A box the command line gives by two opposite corners in microns, in
/// either order.
#[derive(Clone, Copy, Debug, PartialEq)]
struct MicronBox {
    corner: Point<f64>,
    opposite: Point<f64>,
}

impl MicronBox {
    /// The box whose corners `corners` gives as x0, y0, x1, y1.
    fn from_corners(corners: &[f64]) -> anyhow::Result<Self> {
        let &[x0, y0, x1, y1] = corners else {
            anyhow::bail!("a box takes four corner coordinates, not {}", corners.len());
        };
        Ok(Self {
            corner: Point::new(x0, y0),
            opposite: Point::new(x1, y1),
        })
    }

    /// The box rounded onto the database grid of `units`.
    fn in_dbu(self, units: DbuPerMicron) -> Rect<Dbu> {
        let at = |p: Point<f64>| Point::new(units.to_dbu_rounded(p.x), units.to_dbu_rounded(p.y));
        Rect::from_corners(at(self.corner), at(self.opposite))
    }
}

/// The state files `args` name, with the trace box in database `units`.
fn state_files(args: RouteArgs, units: DbuPerMicron) -> anyhow::Result<StateFiles> {
    let area = args
        .trace_box
        .map(|corners| MicronBox::from_corners(&corners).map(|b| b.in_dbu(units)))
        .transpose()?;
    Ok(StateFiles {
        save: args.save_state.map(|path| {
            (
                path,
                args.save_after.unwrap_or(pare_router::SavePoint::Global),
            )
        }),
        resume: args.resume,
        trace: pare_router::TraceRequest {
            nets: args.trace_nets,
            area,
        },
    })
}

/// The `run` command: places and routes the design `name` as its purpose
/// says.
fn place_and_route(name: &Path, report: &mut Report) -> anyhow::Result<()> {
    let config = load_config(name, report)?;
    let mut db = load_design(&config, None)?;
    report.design.clone_from(&db.name);
    let output_dir = prepare_output_dir(&config)?;
    let repair = if config.purpose == Purpose::Routing {
        None
    } else {
        run_placement(
            &config,
            &mut db,
            &output_dir,
            PlacementStart::Scratch,
            report,
        )?
    };
    if config.purpose == Purpose::Placement {
        return Ok(());
    }
    let global = match repair {
        Some(mut repair) => global_route_with_repair(&config, &mut db, &mut repair, report)?,
        None => None,
    };
    route_detailed(&config, &mut db, &StateFiles::default(), global, report)
}

/// The `check` command: checks the placement, and the routing too unless
/// the design is a placement benchmark, which is never routed.
fn check(name: &Path, placement: Option<&Path>, report: &mut Report) -> anyhow::Result<()> {
    let config = load_config(name, report)?;
    let db = load_design(&config, placement)?;
    report.design.clone_from(&db.name);
    let placement_ok = report_placement(&db, report);
    let routing_ok = config.purpose == Purpose::Placement || report_routing(&db, report);
    if !(placement_ok && routing_ok) {
        anyhow::bail!("{} failed its checks", db.name);
    }
    Ok(())
}

/// Reads the design named by the `[input]` section. `placement` replaces the
/// input DEF, or for a Bookshelf design supplies a `.pl` to apply.
fn load_design(config: &Config, placement: Option<&Path>) -> anyhow::Result<NetlistDB> {
    match &config.input.design {
        DesignInput::Bookshelf {
            aux_file,
            num_layers,
        } => {
            ui::header("Input", &aux_file.display().to_string());
            let mut db = bookshelf::parse(aux_file, *num_layers)?;
            if let Some(pl) = placement {
                ui::header("Input", &pl.display().to_string());
                bookshelf::read_placement(&mut db, pl)?;
            }
            Ok(db)
        }
        DesignInput::LefDef {
            lef_files,
            def_file,
        } => {
            let def_path = placement.unwrap_or(def_file);
            for lef in lef_files {
                ui::header("Input", &lef.display().to_string());
            }
            ui::header("Input", &def_path.display().to_string());
            let units = def::read_units(def_path)?;
            let library = lef::parse_files(lef_files, units)?;
            Ok(def::parse(def_path, library)?)
        }
    }
}

/// Creates the directory for images and results, as [`output_dir`] names
/// it, and returns it.
pub(crate) fn prepare_output_dir(config: &Config) -> anyhow::Result<PathBuf> {
    let dir = output_dir(config);
    std::fs::create_dir_all(&dir).with_context(|| format!("cannot create {}", dir.display()))?;
    Ok(dir)
}

/// Directory for images and results: `output_dir`, else `output_def`'s parent.
fn output_dir(config: &Config) -> PathBuf {
    if let Some(dir) = &config.input.output_dir {
        return dir.clone();
    }
    config
        .input
        .output_def
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .map_or_else(|| PathBuf::from("output"), Path::to_path_buf)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::scratch_dir;

    /// A configs directory holding an empty config at each of `files`.
    fn configs_with(tag: &str, files: &[&str]) -> PathBuf {
        let configs = scratch_dir(tag);
        for file in files {
            let path = configs.join(file);
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            std::fs::write(path, "").unwrap();
        }
        configs
    }

    fn config_with_input(input: &str) -> Config {
        toml::from_str(&format!(
            "purpose = \"placement\"\n[input]\nlef_files = [\"a.lef\"]\ndef_file = \"a.def\"\n{input}"
        ))
        .unwrap()
    }

    #[test]
    fn a_config_path_is_taken_as_it_is() {
        let configs = configs_with("resolve-path", &[]);

        for name in ["suite/design", "design.toml", "design.TOML"] {
            assert_eq!(
                resolve_config(Path::new(name), &configs).unwrap(),
                PathBuf::from(name)
            );
        }
    }

    #[test]
    fn a_design_name_finds_its_config_in_whichever_suite_has_it() {
        let configs = configs_with("resolve-one", &["ibm/ibm01.toml", "openroad/aes.toml"]);

        let path = resolve_config(Path::new("aes"), &configs).unwrap();

        assert_eq!(path, configs.join("openroad/aes.toml"));
    }

    #[test]
    fn a_design_name_without_a_config_says_where_to_get_one() {
        let configs = configs_with("resolve-none", &["openroad/aes.toml"]);

        let error = resolve_config(Path::new("ibex"), &configs).unwrap_err();

        assert_eq!(
            error.to_string(),
            format!(
                "no config for 'ibex' in {}/<suite>/; download it with scripts/download.py",
                configs.display()
            )
        );
    }

    #[test]
    fn a_design_name_in_two_suites_is_ambiguous() {
        let configs = configs_with("resolve-two", &["a/gcd.toml", "b/gcd.toml"]);

        let error = resolve_config(Path::new("gcd"), &configs).unwrap_err();

        assert!(
            error.to_string().starts_with("'gcd' names several configs"),
            "{error}"
        );
    }

    #[test]
    fn a_design_name_without_a_configs_directory_names_the_directory() {
        let missing = scratch_dir("resolve-missing").join("configs");

        let error = resolve_config(Path::new("aes"), &missing).unwrap_err();

        assert_eq!(
            error.to_string(),
            format!("cannot read {}/", missing.display())
        );
    }

    #[test]
    fn the_output_directory_is_the_one_the_config_names() {
        let config = config_with_input("output_def = \"out/placed.def\"\noutput_dir = \"images\"");

        assert_eq!(output_dir(&config), PathBuf::from("images"));
    }

    #[test]
    fn without_an_output_directory_results_go_beside_the_output_def() {
        let config = config_with_input("output_def = \"out/aes/placed.def\"");

        assert_eq!(output_dir(&config), PathBuf::from("out/aes"));
    }

    #[test]
    fn an_output_def_without_a_directory_puts_results_in_output() {
        let config = config_with_input("output_def = \"placed.def\"");

        assert_eq!(output_dir(&config), PathBuf::from("output"));
    }

    #[test]
    fn a_box_needs_four_corner_coordinates() {
        assert!(MicronBox::from_corners(&[1.0, 2.0, 3.0]).is_err());
    }

    #[test]
    fn a_box_in_microns_is_rounded_onto_the_database_grid() {
        let units = DbuPerMicron::new(2000).unwrap();

        let area = MicronBox::from_corners(&[1.0, 2.5, 3.00025, 4.0])
            .unwrap()
            .in_dbu(units);

        assert_eq!(
            area,
            Rect::new(
                Point::new(Dbu(2000), Dbu(5000)),
                Point::new(Dbu(6001), Dbu(8000))
            )
        );
    }

    #[test]
    fn a_box_given_high_corner_first_is_the_same_box() {
        let units = DbuPerMicron::new(1000).unwrap();
        let at = |corners: &[f64]| MicronBox::from_corners(corners).unwrap().in_dbu(units);

        assert_eq!(at(&[6.0, 6.0, 0.0, 0.0]), at(&[0.0, 0.0, 6.0, 6.0]));
        assert_eq!(at(&[6.0, 0.0, 0.0, 6.0]), at(&[0.0, 0.0, 6.0, 6.0]));
    }
}
