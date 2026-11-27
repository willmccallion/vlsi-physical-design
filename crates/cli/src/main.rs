//! PARE -- Placement And Routing Engine.
//!
//! Loads a LEF/DEF or Bookshelf design, places and routes it, checks the
//! result and writes it out.

mod checks;
mod cts;
mod flow;
mod logger;
mod phase;
mod placement;
mod plan;
mod repair;
mod report;
mod routing;
#[cfg(test)]
mod testing;
mod ui;
mod view;
mod visualization;

use crate::checks::{report_placement, report_routing};
use crate::flow::{Design, read_design};
use crate::placement::PlacementStart;
use crate::plan::FlowPlan;
use crate::routing::{RouterRun, SaveRequest, parse_save_point};
use crate::view::{ViewRequest, view_area};
use anyhow::Context;
use clap::{Parser, Subcommand};
use pare_common::config::{Config, Purpose};
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use report::Report;
use std::num::NonZeroUsize;
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

/// Resolves the config `name`, records it in the report, reads it and
/// sizes the thread pool as it asks.
fn load_config(name: &Path, report: &mut Report) -> anyhow::Result<Config> {
    let path = resolve_config(name, Path::new(CONFIGS_DIR))?;
    ui::header("Config", &path.display().to_string());
    report.config = path.display().to_string();
    let config = read_config(&path)?;
    if let Some(threads) = config.threads {
        size_thread_pool(threads)?;
    }
    Ok(config)
}

/// The config in the TOML file at `path`.
fn read_config(path: &Path) -> anyhow::Result<Config> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("cannot read config {}", path.display()))?;
    toml::from_str(&text).with_context(|| format!("invalid config {}", path.display()))
}

/// Gives every parallel step `threads` workers.
fn size_thread_pool(threads: NonZeroUsize) -> anyhow::Result<()> {
    rayon::ThreadPoolBuilder::new()
        .num_threads(threads.get())
        .build_global()
        .context("cannot size the thread pool")
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
    report.peak_rss_mb = pare_common::memory::peak_rss_mb();
    report.threads = rayon::current_num_threads();
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
            let db = read_design(&config, placement.as_deref())?;
            let request = ViewRequest {
                area: MicronBox::from_corners(&area)?.in_dbu(db.dbu()),
                nets,
                layers,
                out,
                size,
            };
            view_area(&db, &request)
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
    let design = Design::load(&config, from, report)?;
    let plan = FlowPlan::resolve(&config, design.db())?;
    let start = if from.is_some() {
        PlacementStart::Given
    } else {
        PlacementStart::Scratch
    };
    let _ = design.place(&config, &plan, start, report)?;
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
    let design = Design::load(&config, args.placement.as_deref(), report)?.placed_by_input();
    if args.global_only {
        return design.route_globally_only(&config, args.guides.as_deref(), report);
    }
    let run = router_run(args, design.db().dbu())?;
    design
        .route_globally(&config, report)?
        .route(&config, &run, report)
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

/// The router run `args` ask for, with the trace box in database `units`.
fn router_run(args: RouteArgs, units: DbuPerMicron) -> anyhow::Result<RouterRun> {
    let area = args
        .trace_box
        .map(|corners| MicronBox::from_corners(&corners).map(|b| b.in_dbu(units)))
        .transpose()?;
    Ok(RouterRun {
        save: args.save_state.map(|path| SaveRequest {
            path,
            after: args.save_after.unwrap_or(pare_router::SavePoint::Global),
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
    let design = Design::load(&config, None, report)?;
    let plan;
    let placed = match config.purpose {
        Purpose::Routing => design.placed_by_input(),
        Purpose::Placement | Purpose::PlaceAndRoute => {
            plan = FlowPlan::resolve(&config, design.db())?;
            design.place(&config, &plan, PlacementStart::Scratch, report)?
        }
    };
    if config.purpose == Purpose::Placement {
        return Ok(());
    }
    placed
        .route_globally(&config, report)?
        .route(&config, &RouterRun::default(), report)
}

/// The `check` command: checks the placement, and the routing too unless
/// the design is a placement benchmark, which is never routed.
fn check(name: &Path, placement: Option<&Path>, report: &mut Report) -> anyhow::Result<()> {
    let config = load_config(name, report)?;
    let design = Design::load(&config, placement, report)?;
    let db = design.db();
    let placement_ok = report_placement(db, report);
    let routing_ok = config.purpose == Purpose::Placement || report_routing(db, report);
    if !(placement_ok && routing_ok) {
        anyhow::bail!("{} failed its checks", db.name);
    }
    Ok(())
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
mod tests;
