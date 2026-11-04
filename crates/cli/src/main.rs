//! PARE -- Placement And Routing Engine.
//!
//! Loads a LEF/DEF or Bookshelf design, places and routes it, checks the
//! result and writes it out.

mod checks;
mod cts;
mod placement;
mod repair;
mod report;
mod routing;
mod view;

use crate::checks::{report_placement, report_routing};
use crate::placement::{PlacementStart, run_placement};
use crate::routing::{
    RouteStage, StateFiles, global_route_with_repair, parse_save_point, run_routing,
};
use crate::view::view_area;
use anyhow::Context;
use clap::{Parser, Subcommand};
use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{bookshelf, def, lef};
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::geom::point::Point;
use pare_common::util::config::{Config, DesignInput, Purpose};
use pare_common::util::{logger, ui};
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

/// Resolves a config argument to a file path.
///
/// An existing path, or anything containing '/' or ending in '.toml', is used
/// as is; otherwise it names a design whose config is
/// `configs/<suite>/<design>.toml`.
fn resolve_config(name: &Path) -> anyhow::Result<PathBuf> {
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
    for suite in std::fs::read_dir("configs").context("cannot read configs/")? {
        let candidate = suite?.path().join(&file);
        if candidate.is_file() {
            found.push(candidate);
        }
    }
    match found.as_slice() {
        [one] => Ok(one.clone()),
        [] => anyhow::bail!(
            "no config for '{text}' in configs/<suite>/; download it with scripts/download.py"
        ),
        _ => anyhow::bail!("'{text}' names several configs: {found:?}"),
    }
}

/// Reads the config, records it in the report and sizes the thread pool as
/// it asks.
fn load_config(name: &Path, report: &mut Report) -> anyhow::Result<Config> {
    let path = resolve_config(name)?;
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
    report.wall_seconds = start_time.elapsed().as_secs_f64();
    report.peak_rss_mb = report::peak_rss_mb();
    if let Some(path) = &args.report {
        report.write(path)?;
        ui::wrote(path);
    }
    outcome?;

    ui::done(report.wall_seconds);
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
            view_area(&db, &area, &nets, &layers, &out, size)
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
    let stage = route_stage(args, db.dbu());
    run_routing(
        &config,
        &mut db,
        &prepare_output_dir(&config)?,
        stage,
        None,
        report,
    )
}

/// The routing `args` ask for, with the trace box in database `units`.
fn route_stage(args: RouteArgs, units: DbuPerMicron) -> RouteStage {
    if args.global_only {
        return RouteStage::Global {
            guides: args.guides,
        };
    }
    let area = args.trace_box.map(|b| {
        let at = |x: f64, y: f64| Point::new(units.to_dbu_rounded(x), units.to_dbu_rounded(y));
        pare_common::geom::rect::Rect::new(at(b[0], b[1]), at(b[2], b[3]))
    });
    RouteStage::Detailed(StateFiles {
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
    let stage = RouteStage::Detailed(StateFiles::default());
    run_routing(&config, &mut db, &output_dir, stage, global, report)
}

/// The `check` command.
fn check(name: &Path, placement: Option<&Path>, report: &mut Report) -> anyhow::Result<()> {
    let config = load_config(name, report)?;
    let db = load_design(&config, placement)?;
    report.design.clone_from(&db.name);
    let placement_ok = report_placement(&db, report);
    let routing_ok = report_routing(&db, report);
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

fn prepare_output_dir(config: &Config) -> anyhow::Result<PathBuf> {
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
