//! PARE -- Placement And Routing Engine.
//!
//! Loads a LEF/DEF or Bookshelf design, places and routes it, checks the
//! result and writes it out.

use anyhow::Context;
use clap::{Parser, Subcommand};
use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::parser::{bookshelf, def, lef};
use pare_common::db::writer;
use pare_common::util::config::{Config, Purpose};
use pare_common::util::{logger, ui, visualization};
use pare_common::verify::placement::{PlacementViolation, placement_violations};
use pare_common::verify::routing::check_routing;
use pare_placer::detailed;
use pare_placer::global::{self, GlobalParams, Outcome, RoutabilityParams, TrackSupply};
use pare_placer::io::{self, IoLayers};
use pare_placer::legalize::abacus;
use pare_placer::model::PlacementModel;
use pare_placer::zones::Zones;
use std::path::{Path, PathBuf};
use std::time::Instant;

#[derive(Parser)]
#[command(author, version, about = "PARE -- Placement And Routing Engine", long_about = None)]
struct Args {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Place a design.
    Place {
        /// Design name (e.g. 'ibex') or path to a TOML config.
        config: PathBuf,
    },
    /// Route a placed design.
    Route {
        /// Design name (e.g. 'gcd') or path to a TOML config.
        config: PathBuf,
        /// Placed DEF, or Bookshelf `.pl`, to route instead of the config's input.
        #[arg(long)]
        placement: Option<PathBuf>,
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

fn load_config(name: &Path) -> anyhow::Result<Config> {
    let path = resolve_config(name)?;
    ui::header("Config", &path.display().to_string());
    let text = std::fs::read_to_string(&path)
        .with_context(|| format!("cannot read config {}", path.display()))?;
    toml::from_str(&text).with_context(|| format!("invalid config {}", path.display()))
}

fn main() -> anyhow::Result<()> {
    logger::init();
    let args = Args::parse();
    let start_time = Instant::now();
    ui::banner();

    match args.command {
        Commands::Place { config: name } => {
            let config = load_config(&name)?;
            if config.purpose == Purpose::Routing {
                anyhow::bail!(
                    "{} is a routing benchmark: its placement is part of the input",
                    name.display()
                );
            }
            let mut db = load_design(&config, None)?;
            run_placement(&config, &mut db, &prepare_output_dir(&config)?)?;
        }
        Commands::Route {
            config: name,
            placement,
        } => {
            let config = load_config(&name)?;
            if config.purpose == Purpose::Placement {
                anyhow::bail!(
                    "{} is a placement benchmark and is not routed",
                    name.display()
                );
            }
            let mut db = load_design(&config, placement.as_deref())?;
            run_routing(&config, &mut db, &prepare_output_dir(&config)?)?;
        }
        Commands::Run { config } => {
            let config = load_config(&config)?;
            let mut db = load_design(&config, None)?;
            let output_dir = prepare_output_dir(&config)?;
            if config.purpose != Purpose::Routing {
                run_placement(&config, &mut db, &output_dir)?;
            }
            if config.purpose != Purpose::Placement {
                run_routing(&config, &mut db, &output_dir)?;
            }
        }
        Commands::Check { config, placement } => {
            let config = load_config(&config)?;
            let db = load_design(&config, placement.as_deref())?;
            let placement_ok = report_placement(&db);
            let routing_ok = report_routing(&db);
            if !(placement_ok && routing_ok) {
                anyhow::bail!("{} failed its checks", db.name);
            }
        }
    }

    ui::done(start_time.elapsed().as_secs_f64());
    Ok(())
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

/// Global placement, I/O pin placement, legalization, checking and output.
fn run_placement(config: &Config, db: &mut NetlistDB, output_dir: &Path) -> anyhow::Result<()> {
    let movable_io = io::movable_io_pins(db);
    let io_layers = if movable_io.is_empty() {
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

    ui::section("Placement");
    let model = if let Some(layers) = io_layers {
        place_cells_and_io_pins(config, db, layers)?
    } else {
        let phase_start = Instant::now();
        let model = global_placement(config, db, PlacementModel::from_db(db))?;
        ui::phase_time(phase_start.elapsed().as_secs_f64());
        model
    };
    let movable_cells = model.movable.iter().filter(|&&m| m).count();
    model.write_back(db);
    let global_png = output_dir.join("nesterov_placer.png");
    visualization::draw_placement(db, &path_str(&global_png)?, 1000, 1000);

    let zones = Zones::new(db)?;
    let phase_start = Instant::now();
    ui::phase("Legalization");
    abacus::legalize(db, &zones)?;
    ui::check(&format!("Legalized {movable_cells} cells"));
    ui::stat_row(
        "HPWL (um):",
        &format!("{:.0}", PlacementModel::from_db(db).hpwl()),
        "",
        "",
    );
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    let phase_start = Instant::now();
    ui::phase("Detailed Placement");
    let result = detailed::improve(db, &zones, 3);
    ui::stat_row(
        "Moves:",
        &result.moves.to_string(),
        "HPWL (um):",
        &format!("{:.0}", PlacementModel::from_db(db).hpwl()),
    );
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    if !report_placement(db) {
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
    ui::phase_time(phase_start.elapsed().as_secs_f64());
    Ok(())
}

/// Global placement without the movable I/O pins, I/O pin placement for
/// the cells' positions, then global placement again with the pins, as
/// `OpenROAD`'s flow does.
fn place_cells_and_io_pins(
    config: &Config,
    db: &mut NetlistDB,
    layers: IoLayers,
) -> anyhow::Result<PlacementModel> {
    let phase_start = Instant::now();
    ui::phase("Global Placement Without I/O");
    let model = global_placement(config, db, PlacementModel::from_db_without_movable_io(db))?;
    model.write_back(db);
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    let phase_start = Instant::now();
    ui::phase("I/O Pin Placement");
    let placed = io::place_io_pins(db, layers)?;
    ui::stat_row(
        "Pins:",
        &placed.pins.to_string(),
        "Free slots:",
        &placed.slots.to_string(),
    );
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    let phase_start = Instant::now();
    ui::phase("Global Placement");
    let model = global_placement(config, db, PlacementModel::from_db(db))?;
    ui::phase_time(phase_start.elapsed().as_secs_f64());
    Ok(model)
}

/// Runs global placement on `model` and reports it; `db` supplies the
/// routing resources for routability-driven placement.
fn global_placement(
    config: &Config,
    db: &NetlistDB,
    mut model: PlacementModel,
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

    let routability = if gp.routability_driven {
        let layers = config.routing.layer_range(&db.library)?;
        Some(RoutabilityParams {
            supply: TrackSupply::from_db(db, layers, config.routing.global_capacity_reserve),
            target_congestion: gp.target_congestion,
        })
    } else {
        None
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
    let result = global::place(&mut model, &params, |p| {
        if p.iteration % 50 == 0 {
            ui::placement_iter(p.iteration, p.hpwl, p.overflow, p.penalty, p.step);
        }
    });
    eprintln!();
    for (k, round) in result.congestion.iter().enumerate() {
        ui::stat_row(
            &format!("Congestion {}:", k + 1),
            &format!("{:.2} @ {}", round.congestion, round.iteration),
            "Inflated:",
            &format!(
                "{} cells, area x{:.2}, density {:.2}",
                round.inflated, round.area_ratio, round.target_density
            ),
        );
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
    ui::stat_row("HPWL (um):", &format!("{:.0}", model.hpwl()), "", "");
    Ok(model)
}

fn path_str(path: &Path) -> anyhow::Result<String> {
    path.to_str()
        .map(str::to_owned)
        .with_context(|| format!("path is not valid UTF-8: {}", path.display()))
}

/// Routes every signal net, checks the result and writes it out.
fn run_routing(config: &Config, db: &mut NetlistDB, output_dir: &Path) -> anyhow::Result<()> {
    let phase_start = Instant::now();
    ui::section("Routing");
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
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    let phase_start = Instant::now();
    ui::phase("Verification");
    let routing_ok = report_routing(db);
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    let phase_start = Instant::now();
    ui::phase("Output");
    let png = output_dir.join("routed.png");
    visualization::draw_routed_design(db, &path_str(&png)?, 2000, 2000);
    ui::wrote(&path_str(&png)?);
    let out = output_dir.join("routed.def");
    writer::def::write_file(db, &out).with_context(|| format!("cannot write {}", out.display()))?;
    ui::wrote(&path_str(&out)?);
    ui::phase_time(phase_start.elapsed().as_secs_f64());

    if !routing_ok {
        anyhow::bail!("routing has shorts or opens");
    }
    Ok(())
}

/// Prints placement violations by kind; returns whether there were none.
fn report_placement(db: &NetlistDB) -> bool {
    let violations = placement_violations(db);
    if violations.is_empty() {
        ui::check("Placement is legal.");
        return true;
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

/// Prints routing check results; returns whether there are no shorts or opens.
fn report_routing(db: &NetlistDB) -> bool {
    let report = check_routing(db);
    for short in &report.shorts {
        log::debug!("short: {short:?}");
    }
    for v in &report.spacing {
        log::debug!("spacing: {v:?}");
    }
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
        "Shorts:",
        &report.shorts.len().to_string(),
        "Opens:",
        &report.opens.len().to_string(),
    );
    ui::stat_row(
        "Spacing:",
        &report.spacing.len().to_string(),
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
    if report.is_connected_without_shorts() {
        ui::check("Routing has no shorts or opens.");
        true
    } else {
        ui::fail("Routing has shorts or opens.");
        false
    }
}
