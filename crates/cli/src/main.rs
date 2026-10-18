//! PARE -- Placement And Routing Engine.
//!
//! Loads a LEF/DEF or Bookshelf design, places and routes it, checks the
//! result and writes it out.

mod report;

use anyhow::Context;
use clap::{Parser, Subcommand};
use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::indices::CellId;
use pare_common::db::library::LayerRef;
use pare_common::db::parser::{bookshelf, def, lef};
use pare_common::db::writer;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::util::config::{Config, CongestionEstimate, Purpose};
use pare_common::util::{logger, ui, visualization};
use pare_common::verify::drc::{DrcReport, Marker, check_drc};
use pare_common::verify::pin_access::PinAccess;
use pare_common::verify::placement::{PlacementViolation, placement_violations};
use pare_common::verify::routing::check_routing;
use pare_placer::detailed;
use pare_placer::global::{
    self, CongestionEstimator, GlobalParams, Outcome, RoutabilityParams, RoutingTiles, TileTracks,
};
use pare_placer::io::{self, IoLayers};
use pare_placer::legalize::abacus;
use pare_placer::model::PlacementModel;
use pare_placer::zones::{Zone, Zones};
use pare_router::{CapacityReduction, GlobalRouteEstimator};
use report::{PlacementSummary, Report, RoutingSummary};
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
    Route {
        /// Design name (e.g. 'gcd') or path to a TOML config.
        config: PathBuf,
        /// Placed DEF, or Bookshelf `.pl`, to route instead of the config's input.
        #[arg(long)]
        placement: Option<PathBuf>,
        /// Stop after global routing and track assignment, reporting their
        /// result without routing in detail or writing a DEF.
        #[arg(long)]
        global_only: bool,
    },
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
        ui::wrote(&path_str(path)?);
    }
    outcome?;

    ui::done(report.wall_seconds);
    Ok(())
}

/// Runs one command, recording what it did in `report`.
fn run(command: Commands, report: &mut Report) -> anyhow::Result<()> {
    match command {
        Commands::Place { config: name, from } => {
            let config = load_config(&name, report)?;
            if config.purpose == Purpose::Routing {
                anyhow::bail!(
                    "{} is a routing benchmark: its placement is part of the input",
                    name.display()
                );
            }
            let mut db = load_design(&config, from.as_deref())?;
            report.design.clone_from(&db.name);
            let start = if from.is_some() {
                PlacementStart::Given
            } else {
                PlacementStart::Scratch
            };
            run_placement(
                &config,
                &mut db,
                &prepare_output_dir(&config)?,
                start,
                report,
            )
        }
        Commands::Route {
            config: name,
            placement,
            global_only,
        } => {
            let config = load_config(&name, report)?;
            if config.purpose == Purpose::Placement {
                anyhow::bail!(
                    "{} is a placement benchmark and is not routed",
                    name.display()
                );
            }
            let mut db = load_design(&config, placement.as_deref())?;
            report.design.clone_from(&db.name);
            let stage = if global_only {
                RouteStage::Global
            } else {
                RouteStage::Detailed
            };
            run_routing(
                &config,
                &mut db,
                &prepare_output_dir(&config)?,
                stage,
                report,
            )
        }
        Commands::Run { config } => {
            let config = load_config(&config, report)?;
            let mut db = load_design(&config, None)?;
            report.design.clone_from(&db.name);
            let output_dir = prepare_output_dir(&config)?;
            if config.purpose != Purpose::Routing {
                run_placement(
                    &config,
                    &mut db,
                    &output_dir,
                    PlacementStart::Scratch,
                    report,
                )?;
            }
            if config.purpose != Purpose::Placement {
                run_routing(&config, &mut db, &output_dir, RouteStage::Detailed, report)?;
            }
            Ok(())
        }
        Commands::Check { config, placement } => {
            let config = load_config(&config, report)?;
            let db = load_design(&config, placement.as_deref())?;
            report.design.clone_from(&db.name);
            let placement_ok = report_placement(&db, report);
            let routing_ok = report_routing(&db, report);
            if !(placement_ok && routing_ok) {
                anyhow::bail!("{} failed its checks", db.name);
            }
            Ok(())
        }
    }
}

/// Reads the design named by the `[input]` section. `placement` replaces the
/// input DEF, or for a Bookshelf design supplies a `.pl` to apply.
fn load_design(config: &Config, placement: Option<&Path>) -> anyhow::Result<NetlistDB> {
    if let Some(aux) = &config.input.bookshelf_aux_file {
        ui::header("Input", aux);
        let mut db = bookshelf::parse(Path::new(aux), config.input.num_layers)?;
        if let Some(pl) = placement {
            ui::header("Input", &pl.display().to_string());
            bookshelf::read_placement(&mut db, pl)?;
        }
        return Ok(db);
    }
    let def_path = placement.unwrap_or_else(|| Path::new(&config.input.def_file));
    let lefs: Vec<PathBuf> = config.input.lef_files.iter().map(PathBuf::from).collect();
    for lef in &lefs {
        ui::header("Input", &lef.display().to_string());
    }
    ui::header("Input", &def_path.display().to_string());
    let units = def::read_units(def_path)?;
    let library = lef::parse_files(&lefs, units)?;
    Ok(def::parse(def_path, library)?)
}

fn prepare_output_dir(config: &Config) -> anyhow::Result<PathBuf> {
    let dir = output_dir(config);
    std::fs::create_dir_all(&dir).with_context(|| format!("cannot create {}", dir.display()))?;
    Ok(dir)
}

/// Directory for images and results: `output_dir`, else `output_def`'s parent.
fn output_dir(config: &Config) -> PathBuf {
    if let Some(dir) = &config.input.output_dir {
        return PathBuf::from(dir);
    }
    Path::new(&config.input.output_def)
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .map_or_else(|| PathBuf::from("output"), Path::to_path_buf)
}

/// Where placement starts from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PlacementStart {
    /// Global placement from a random start.
    Scratch,
    /// The positions the design already has, which are only legalized and
    /// refined.
    Given,
}

/// How far routing goes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RouteStage {
    /// Global routing and track assignment only.
    Global,
    /// Through detailed routing, verification and output.
    Detailed,
}

/// Global placement, I/O pin placement, legalization, checking and output.
fn run_placement(
    config: &Config,
    db: &mut NetlistDB,
    output_dir: &Path,
    start: PlacementStart,
    report: &mut Report,
) -> anyhow::Result<()> {
    let movable_io = io::movable_io_pins(db);
    let io_layers = if movable_io.is_empty() || start == PlacementStart::Given {
        None
    } else if config.io_placement.enabled {
        Some(IoLayers::from_config(&config.io_placement, &db.library)?)
    } else {
        let unplaced = movable_io
            .iter()
            .filter(|&&p| db.io_pins[p.index()].status == PlacementStatus::Unplaced)
            .count();
        if unplaced > 0 {
            anyhow::bail!("I/O pin placement is disabled but {unplaced} I/O pins are unplaced");
        }
        None
    };

    let zones = Zones::new(db)?;
    ui::section("Placement");
    let model = if start == PlacementStart::Given {
        let unplaced = db
            .cells
            .iter()
            .filter(|c| c.status == PlacementStatus::Unplaced)
            .count();
        if unplaced > 0 {
            anyhow::bail!("{unplaced} cells of the given placement are unplaced");
        }
        ui::check("Starting from the given placement");
        PlacementModel::from_db(db, &zones)
    } else if let Some(layers) = io_layers {
        place_cells_and_io_pins(config, db, &zones, layers, report)?
    } else {
        let phase_start = Instant::now();
        let model = global_placement(
            config,
            db,
            PlacementModel::from_db(db, &zones),
            Pass::Full,
            report,
        )?;
        report.phase("global placement", phase_start);
        model
    };
    let movable_cells = model.movable.iter().filter(|&&m| m).count();
    model.write_back(db);
    let global_png = output_dir.join("nesterov_placer.png");
    visualization::draw_placement(db, &path_str(&global_png)?, 1000, 1000);

    let phase_start = Instant::now();
    ui::phase("Legalization");
    abacus::legalize(db, &zones)?;
    ui::check(&format!("Legalized {movable_cells} cells"));
    let access = config
        .routing
        .layer_range(&db.library)
        .ok()
        .and_then(|layers| PinAccess::new(db, *layers.start()));
    let blocked_pins = access.as_ref().map_or(0, |access| {
        let freed = detailed::free_blocked_pins(db, &zones, access);
        ui::stat_row(
            "Blocked pins:",
            &format!("{} -> {}", freed.blocked_before, freed.blocked_after),
            "Cells moved:",
            &freed.moved.to_string(),
        );
        freed.blocked_after
    });
    ui::stat_row(
        "HPWL (um):",
        &format!("{:.0}", PlacementModel::from_db(db, &zones).hpwl()),
        "",
        "",
    );
    report.phase("legalization", phase_start);

    let phase_start = Instant::now();
    ui::phase("Detailed Placement");
    let result = detailed::improve(db, &zones, 3, access.as_ref());
    let hpwl = PlacementModel::from_db(db, &zones).hpwl();
    ui::stat_row(
        "Moves:",
        &result.moves.to_string(),
        "HPWL (um):",
        &format!("{hpwl:.0}"),
    );
    report.phase("detailed placement", phase_start);
    if let Some(summary) = &mut report.placement {
        summary.hpwl_um = hpwl;
        summary.detailed_moves = result.moves;
        summary.blocked_pins = blocked_pins;
    }

    if !report_placement(db, report) {
        anyhow::bail!("placement is not legal");
    }

    let phase_start = Instant::now();
    ui::phase("Output");
    ui::wrote(&path_str(&global_png)?);
    let placed_png = output_dir.join("placed.png");
    visualization::draw_placement(db, &path_str(&placed_png)?, 1000, 1000);
    ui::wrote(&path_str(&placed_png)?);
    let placed = if config.input.bookshelf_aux_file.is_some() {
        let pl = output_dir.join("placed.pl");
        writer::bookshelf::write_pl(db, &pl)
            .with_context(|| format!("cannot write {}", pl.display()))?;
        pl
    } else {
        let out = PathBuf::from(&config.input.output_def);
        writer::def::write_file(db, &out)
            .with_context(|| format!("cannot write {}", out.display()))?;
        out
    };
    ui::wrote(&path_str(&placed)?);
    report.phase("placement output", phase_start);
    Ok(())
}

/// Global placement without the movable I/O pins, I/O pin placement for
/// the cells' positions, then global placement again with the pins, as
/// `OpenROAD`'s flow does.
fn place_cells_and_io_pins(
    config: &Config,
    db: &mut NetlistDB,
    zones: &Zones,
    layers: IoLayers,
    report: &mut Report,
) -> anyhow::Result<PlacementModel> {
    let phase_start = Instant::now();
    ui::phase("Global Placement Without I/O");
    let model = global_placement(
        config,
        db,
        PlacementModel::from_db_without_movable_io(db, zones),
        Pass::SkipIo,
        report,
    )?;
    model.write_back(db);
    report.phase("global placement without I/O", phase_start);

    let phase_start = Instant::now();
    ui::phase("I/O Pin Placement");
    let placed = io::place_io_pins(db, layers)?;
    ui::stat_row(
        "Pins:",
        &placed.pins.to_string(),
        "Free slots:",
        &placed.slots.to_string(),
    );
    report.phase("I/O pin placement", phase_start);

    let phase_start = Instant::now();
    ui::phase("Global Placement");
    let model = global_placement(
        config,
        db,
        PlacementModel::from_db(db, zones),
        Pass::Full,
        report,
    )?;
    report.phase("global placement", phase_start);
    Ok(model)
}

/// Whether a global placement pass sees every connection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pass {
    /// Before the I/O pins are placed: like `OpenROAD`'s skip-I/O mode,
    /// without routability.
    SkipIo,
    /// With every connection in place.
    Full,
}

/// Runs global placement on `model` and reports it; `db` supplies the
/// routing resources for routability-driven placement.
fn global_placement(
    config: &Config,
    db: &NetlistDB,
    mut model: PlacementModel,
    pass: Pass,
    report: &mut Report,
) -> anyhow::Result<PlacementModel> {
    let gp = &config.global_placement;

    let movable_area: f64 = model
        .sizes
        .iter()
        .zip(&model.movable)
        .filter(|(_, m)| **m)
        .map(|(s, _)| s.x * s.y)
        .sum();
    let movable_cells = model.movable.iter().filter(|&&m| m).count();
    let utilization = movable_area / model.free_area();
    let bins = gp
        .bin_dimension
        .unwrap_or_else(|| global::default_bins(&model));
    let target_density = gp
        .target_density
        .unwrap_or_else(|| (utilization + 0.05).clamp(0.80, 1.0));

    ui::stat_row(
        "Cells:",
        &movable_cells.to_string(),
        "Nets:",
        &model.nets.len().to_string(),
    );
    ui::stat_row(
        "Utilization:",
        &format!("{:.2}%", utilization * 100.0),
        "Target Density:",
        &format!("{target_density:.2}"),
    );
    ui::stat_row("Bin Grid:", &format!("{bins}x{bins}"), "", "");

    let (routability, mut estimator) = if gp.routability_driven && pass == Pass::Full {
        let (params, estimator) = routability_setup(config, db)?;
        (Some(params), estimator)
    } else {
        (None, None)
    };
    let params = GlobalParams {
        target_density,
        target_overflow: gp.target_overflow,
        max_iterations: gp.placer_max_iterations,
        bins,
        seed: gp.seed,
        routability,
    };
    ui::placement_table_header();
    let result = global::place(
        &mut model,
        &params,
        estimator
            .as_mut()
            .map(|e| e as &mut dyn CongestionEstimator),
        |p| {
            if p.iteration % 50 == 0 {
                ui::placement_iter(p.iteration, p.hpwl, p.overflow, p.penalty, p.step);
            }
        },
    );
    eprintln!();
    for (k, round) in result.congestion.iter().enumerate() {
        ui::stat_row(
            &format!("Congestion {}:", k + 1),
            &format!("{:.4} @ {}", round.congestion, round.iteration),
            "Inflated:",
            &format!(
                "{} cells, area x{:.2}, density {:.2}",
                round.inflated, round.area_ratio, round.target_density
            ),
        );
    }
    if let Some(congestion) = result.final_congestion {
        ui::stat_row("Final congestion:", &format!("{congestion:.4}"), "", "");
    }
    if result.zones.len() > 1 {
        for z in &result.zones {
            let name = match z.zone {
                Zone::Fence(r) => db.regions[r.index()].name.as_str(),
                Zone::Outside => "outside fences",
            };
            ui::stat_row(
                &format!("Zone {name}:"),
                &format!("{} cells, util {:.3}", z.cells, z.utilization),
                "Target / ovfl:",
                &format!("{:.3} / {:.3}", z.target, z.overflow),
            );
        }
    }
    match result.outcome {
        Outcome::Converged => ui::check(&format!(
            "Converged at iteration {} (overflow {:.3})",
            result.iterations, result.overflow
        )),
        Outcome::IterationLimit => log::warn!(
            "Stopped at the iteration limit with overflow {:.3}",
            result.overflow
        ),
        Outcome::Diverged => anyhow::bail!(
            "global placement diverged after {} iterations",
            result.iterations
        ),
    }
    let hpwl = model.hpwl();
    ui::stat_row("HPWL (um):", &format!("{hpwl:.0}"), "", "");
    report.placement = Some(PlacementSummary {
        cells: movable_cells,
        nets: model.nets.len(),
        iterations: result.iterations,
        overflow: result.overflow,
        hpwl_um: hpwl,
        detailed_moves: 0,
        blocked_pins: 0,
    });
    Ok(model)
}

/// What routability-driven placement takes from the router: its gcells
/// with the capacity they lose before placement, and the width RUDY gives a
/// wire on its layers.
/// The routability settings of `config`, and the global-router estimator
/// when congestion is to be measured that way.
fn routability_setup<'a>(
    config: &Config,
    db: &'a NetlistDB,
) -> anyhow::Result<(RoutabilityParams, Option<GlobalRouteCongestion<'a>>)> {
    match config.global_placement.congestion_estimate {
        CongestionEstimate::GlobalRouter => {
            let estimator = GlobalRouteCongestion::new(db, config)?;
            let params = routability_params(config, db, estimator.capacity_reduction());
            Ok((params, Some(estimator)))
        }
        CongestionEstimate::Rudy => {
            let reduction = pare_router::capacity_reduction(db, &config.routing)?;
            Ok((routability_params(config, db, reduction), None))
        }
    }
}

/// Routing congestion from `pare_router`'s global router, measured where
/// global placement has the cells.
struct GlobalRouteCongestion<'a> {
    db: &'a NetlistDB,
    estimator: GlobalRouteEstimator<'a>,
    /// Width and height of each cell, in microns.
    sizes: Vec<Point<f64>>,
}

impl<'a> GlobalRouteCongestion<'a> {
    fn new(db: &'a NetlistDB, config: &Config) -> anyhow::Result<Self> {
        let units = db.dbu();
        Ok(Self {
            db,
            estimator: GlobalRouteEstimator::new(db, &config.routing)?,
            sizes: (0..db.num_cells())
                .map(|i| {
                    let size = db.cell_size(CellId::new(i));
                    Point::new(units.to_microns(size.x), units.to_microns(size.y))
                })
                .collect(),
        })
    }

    fn capacity_reduction(&self) -> CapacityReduction {
        self.estimator.capacity_reduction()
    }
}

impl CongestionEstimator for GlobalRouteCongestion<'_> {
    fn congestion(&mut self, centers: &[Point<f64>]) -> Vec<f64> {
        let units = self.db.dbu();
        let locations: Vec<Point<Dbu>> = centers
            .iter()
            .zip(&self.sizes)
            .map(|(c, size)| {
                Point::new(
                    units.to_dbu_rounded(size.x.mul_add(-0.5, c.x)),
                    units.to_dbu_rounded(size.y.mul_add(-0.5, c.y)),
                )
            })
            .collect();
        self.estimator.congestion(&locations)
    }
}

fn routability_params(
    config: &Config,
    db: &NetlistDB,
    reduction: CapacityReduction,
) -> RoutabilityParams {
    let units = db.dbu();
    let tracks = |t: pare_router::TileTracks| TileTracks {
        capacity: t.capacity,
        reduction: t.reduction,
    };
    RoutabilityParams {
        tiles: RoutingTiles {
            origin: Point::new(
                units.to_microns(reduction.origin.x),
                units.to_microns(reduction.origin.y),
            ),
            size: units.to_microns(reduction.gcell),
            nx: reduction.nx,
            ny: reduction.ny,
            horizontal: tracks(reduction.horizontal),
            vertical: tracks(reduction.vertical),
        },
        target_congestion: config.global_placement.target_congestion,
    }
}

fn path_str(path: &Path) -> anyhow::Result<String> {
    path.to_str()
        .map(str::to_owned)
        .with_context(|| format!("path is not valid UTF-8: {}", path.display()))
}

/// Routes every signal net, checks the result and writes it out.
fn run_routing(
    config: &Config,
    db: &mut NetlistDB,
    output_dir: &Path,
    stage: RouteStage,
    report: &mut Report,
) -> anyhow::Result<()> {
    let phase_start = Instant::now();
    ui::section("Routing");
    if stage == RouteStage::Global {
        let summary = pare_router::route_global_only(db, &config.routing)?;
        ui::stat_row("Nets:", &summary.nets.to_string(), "", "");
        report.phase("global routing", phase_start);
        return Ok(());
    }
    let summary = pare_router::route(db, &config.routing)?;
    ui::stat_row(
        "Nets:",
        &summary.nets.to_string(),
        "Rounds:",
        &summary.rounds.to_string(),
    );
    ui::stat_row(
        "No access:",
        &summary.inaccessible.len().to_string(),
        "Incomplete:",
        &summary.incomplete.len().to_string(),
    );
    ui::stat_row(
        "In conflict:",
        &summary.conflicted.len().to_string(),
        "",
        "",
    );
    report.phase("routing", phase_start);
    report.routing = Some(RoutingSummary {
        nets: summary.nets,
        rounds: summary.rounds,
        inaccessible: summary.inaccessible.len(),
        incomplete: summary.incomplete.len(),
        conflicted: summary.conflicted.len(),
    });

    let phase_start = Instant::now();
    ui::phase("Verification");
    let routing_ok = report_routing(db, report);
    report.phase("verification", phase_start);

    let phase_start = Instant::now();
    ui::phase("Output");
    let png = output_dir.join("routed.png");
    visualization::draw_routed_design(db, &path_str(&png)?, 2000, 2000);
    ui::wrote(&path_str(&png)?);
    let out = output_dir.join("routed.def");
    writer::def::write_file(db, &out).with_context(|| format!("cannot write {}", out.display()))?;
    ui::wrote(&path_str(&out)?);
    report.phase("routing output", phase_start);

    if !routing_ok {
        anyhow::bail!("routing has opens or design rule violations");
    }
    Ok(())
}

/// Prints placement violations by kind; returns whether there were none.
fn report_placement(db: &NetlistDB, report: &mut Report) -> bool {
    let violations = placement_violations(db);
    report.checks.placement_violations = violations.len();
    if violations.is_empty() {
        ui::check("Placement is legal.");
        return true;
    }
    if log::log_enabled!(log::Level::Debug) {
        for v in &violations {
            log::debug!("placement violation: {}", describe_violation(db, v));
        }
    }
    let count = |f: fn(&PlacementViolation) -> bool| violations.iter().filter(|v| f(v)).count();
    ui::fail(&format!("Placement has {} violations", violations.len()));
    ui::stat_row(
        "Outside die:",
        &count(|v| matches!(v, PlacementViolation::OutsideDie(_))).to_string(),
        "Overlaps:",
        &count(|v| matches!(v, PlacementViolation::Overlap(..))).to_string(),
    );
    ui::stat_row(
        "Not in a row:",
        &count(|v| matches!(v, PlacementViolation::NotInRow(_))).to_string(),
        "Off site:",
        &count(|v| matches!(v, PlacementViolation::OffSite(_))).to_string(),
    );
    ui::stat_row(
        "Wrong orient:",
        &count(|v| matches!(v, PlacementViolation::WrongOrientation(_))).to_string(),
        "In blockage:",
        &count(|v| matches!(v, PlacementViolation::InBlockage(_))).to_string(),
    );
    ui::stat_row(
        "Outside region:",
        &count(|v| matches!(v, PlacementViolation::OutsideRegion(_))).to_string(),
        "In a fence:",
        &count(|v| matches!(v, PlacementViolation::InFence(_))).to_string(),
    );
    ui::stat_row(
        "Unplaced:",
        &count(|v| matches!(v, PlacementViolation::Unplaced(_))).to_string(),
        "",
        "",
    );
    false
}

/// A violation with the names, locations and sizes of the cells in it.
fn describe_violation(db: &NetlistDB, v: &PlacementViolation) -> String {
    let cell = |c: CellId| {
        let bbox = db.cell_bbox(c);
        format!(
            "{} {:?} at ({}, {})-({}, {})",
            db.cells[c.index()].name,
            db.cells[c.index()]
                .region
                .map(|r| db.regions[r.index()].name.as_str()),
            bbox.min.x.get(),
            bbox.min.y.get(),
            bbox.max.x.get(),
            bbox.max.y.get()
        )
    };
    match *v {
        PlacementViolation::Overlap(a, b) => format!("overlap {} with {}", cell(a), cell(b)),
        PlacementViolation::Unplaced(c) => format!("unplaced {}", cell(c)),
        PlacementViolation::OutsideDie(c) => format!("outside die {}", cell(c)),
        PlacementViolation::InBlockage(c) => format!("in blockage {}", cell(c)),
        PlacementViolation::NotInRow(c) => format!("not in a row {}", cell(c)),
        PlacementViolation::OffSite(c) => format!("off site {}", cell(c)),
        PlacementViolation::WrongOrientation(c) => format!("wrong orientation {}", cell(c)),
        PlacementViolation::OutsideRegion(c) => format!("outside region {}", cell(c)),
        PlacementViolation::InFence(c) => format!("in a fence {}", cell(c)),
    }
}

/// Prints the connectivity and design rule check results; returns whether
/// every net is connected and nothing routed breaks a rule.
fn report_routing(db: &NetlistDB, run: &mut Report) -> bool {
    let report = check_routing(db);
    for open in &report.opens {
        log::debug!(
            "open: net {} '{}' in {} pieces",
            open.net.index(),
            db.nets[open.net.index()].name,
            open.components
        );
    }
    let units = db.dbu();
    let wirelength: f64 = report
        .stats
        .wirelength
        .iter()
        .map(|&l| units.to_microns(l))
        .sum();
    ui::stat_row(
        "Opens:",
        &report.opens.len().to_string(),
        "Wirelength (um):",
        &format!("{wirelength:.0}"),
    );
    ui::stat_row(
        "Vias:",
        &report.stats.vias.to_string(),
        "Off track:",
        &report.stats.off_track.to_string(),
    );
    ui::stat_row(
        "Wrong way:",
        &report.stats.wrong_way.to_string(),
        "Non-Manhattan:",
        &report.stats.non_manhattan.to_string(),
    );
    let drc = check_drc(db);
    for m in &drc.violations {
        log::debug!("drc: {}", describe_marker(db, &drc, m));
    }
    for m in &drc.advisory {
        log::debug!("drc advisory: {}", describe_marker(db, &drc, m));
    }
    for m in &drc.pin_obstruction {
        log::debug!("drc pin in obstruction: {}", describe_marker(db, &drc, m));
    }
    for m in &drc.placement {
        log::debug!("drc from placement: {}", describe_marker(db, &drc, m));
    }
    for m in &drc.input {
        log::debug!("drc in the input: {}", describe_marker(db, &drc, m));
    }
    ui::stat_row(
        "DRC violations:",
        &drc.violations.len().to_string(),
        "From placement:",
        &drc.placement.len().to_string(),
    );
    ui::stat_row(
        "In the input:",
        &drc.input.len().to_string(),
        "Advisory:",
        &drc.advisory.len().to_string(),
    );
    ui::stat_row(
        "Pins in obstructions:",
        &drc.pin_obstruction.len().to_string(),
        "",
        "",
    );
    for (kind, count) in drc.counts() {
        ui::stat_row(&format!("  {}:", kind.name()), &count.to_string(), "", "");
    }
    for (kind, count) in drc.advisory_counts() {
        ui::stat_row(
            &format!("  {} (advisory):", kind.name()),
            &count.to_string(),
            "",
            "",
        );
    }
    let by_kind = |counts: Vec<(pare_common::verify::drc::Violation, usize)>| {
        counts
            .into_iter()
            .map(|(kind, n)| (kind.name().to_string(), n))
            .collect()
    };
    run.checks.opens = report.opens.len();
    run.checks.wirelength_um = wirelength;
    run.checks.vias = report.stats.vias;
    run.checks.violations = by_kind(drc.counts());
    run.checks.advisory = by_kind(drc.advisory_counts());
    run.checks.pin_obstructions = drc.pin_obstruction.len();
    run.checks.from_placement = drc.placement.len();
    run.checks.in_input = drc.input.len();
    if report.is_connected() && drc.violations.is_empty() {
        ui::check("Routing is connected and clean.");
        true
    } else {
        ui::fail("Routing has opens or design rule violations.");
        false
    }
}

/// A violation, where it is and what it involves, for the debug log.
fn describe_marker(db: &NetlistDB, drc: &DrcReport, m: &Marker) -> String {
    let units = db.dbu();
    let layer = match m.layer {
        LayerRef::Routing(l) => db.library.layer(l).name.as_str(),
        LayerRef::Cut(c) => db.library.cut_layers[c.index()].name.as_str(),
    };
    let name = |i: usize| drc.owners[i].name(db);
    format!(
        "{} on {layer} at ({:.4}, {:.4}) - ({:.4}, {:.4}) between {} and {}",
        m.violation.name(),
        units.to_microns(m.bbox.min.x),
        units.to_microns(m.bbox.min.y),
        units.to_microns(m.bbox.max.x),
        units.to_microns(m.bbox.max.y),
        name(m.owners.0.index()),
        name(m.owners.1.index()),
    )
}
