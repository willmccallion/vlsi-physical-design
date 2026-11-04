//! Placement: global placement with its timing and routability drivers,
//! legalization and detailed placement.

use crate::checks::report_placement;
use crate::cts::clock_tree_synthesis;
use crate::path_str;
use crate::repair::{NetlistRepair, TimedRepair, repair_design, repair_netlist, repair_timing};
use crate::report::{PlacementSummary, Report};
use anyhow::Context;
use pare_common::db::design::{NetlistDB, PlacementStatus, Renumbering};
use pare_common::db::indices::{CellId, NetId};
use pare_common::db::liberty::MinMax;
use pare_common::db::writer;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::units::Seconds;
use pare_common::util::config::{
    Config, CongestionEstimate, DesignInput, GlobalPlacementConfig, Share, TargetDensity,
};
use pare_common::util::{ui, visualization};
use pare_common::verify::pin_access::PinAccess;
use pare_placer::detailed;
use pare_placer::global::{
    self, CongestionEstimator, GlobalParams, GlobalResult, Outcome, Reweighted, RoutabilityParams,
    RoutingTiles, TimingDriver, TimingParams,
};
use pare_placer::io::{self, IoLayers};
use pare_placer::legalize::Legalization;
use pare_placer::legalize::opendp::improve::improve_placement;
use pare_placer::legalize::opendp::mirror::optimize_mirroring;
use pare_placer::model::PlacementModel;
use pare_placer::zones::{Zone, Zones};
use pare_resizer::{Rebuffer, Repairer, Resizer, area_change_percent, timing_net_weights};
use pare_router::{CapacityReduction, GlobalRouteEstimator};
use std::num::NonZeroUsize;
use std::path::Path;
use std::time::{Duration, Instant};

/// Iterations between rows of global placement's progress table.
const PROGRESS_INTERVAL: usize = 50;

/// How far `improve_placement` may move a cell, in row heights across and
/// rows up or down: OpenROAD-flow-scripts' `DPO_MAX_DISPLACEMENT`.
const IMPROVE_MAX_DISPLACEMENT: (i64, i64) = (5, 1);

/// Where placement starts from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PlacementStart {
    /// Global placement from a random start.
    Scratch,
    /// The positions the design already has, which are only legalized and
    /// refined.
    Given,
}

/// Global placement, I/O pin placement, legalization, checking and output.
pub(crate) fn run_placement(
    config: &Config,
    db: &mut NetlistDB,
    output_dir: &Path,
    start: PlacementStart,
    report: &mut Report,
) -> anyhow::Result<Option<NetlistRepair>> {
    if let Some(layers) = io_layers_to_place(config, db, start)? {
        ui::section("Pin Placement");
        place_io_pins_by_cells(config, db, &Zones::new(db)?, layers, report)?;
    }
    let mut repair = if start == PlacementStart::Scratch {
        repair_netlist(config, db, report)?
    } else {
        None
    };
    ui::section("Placement");
    let model = starting_placement(config, db, start, repair.as_mut(), report)?;
    model.write_back(db);
    if let Some(repair) = &mut repair {
        repair_design(config, db, repair, report)?;
    }
    let global_png = output_dir.join("nesterov_placer.png");
    visualization::draw_placement(db, &path_str(&global_png)?, 1000, 1000);
    let (legalization, access, blocked_pins) = legalize_with_pin_access(config, db, report)?;
    detailed_placement(db, legalization, access.as_ref(), blocked_pins, report)?;
    if let Some(repair) = &mut repair
        && clock_tree_synthesis(config, db, repair, report)?
    {
        let _ = legalize_again(db, report, "legalization after clock tree synthesis")?;
    }
    if let Some(repair) = &mut repair
        && repair_timing(config, db, repair, report)?
    {
        let hpwl = legalize_again(db, report, "legalization after timing repair")?;
        if let Some(summary) = &mut report.placement {
            summary.hpwl_um = hpwl;
        }
    }
    if !report_placement(db, report) {
        anyhow::bail!("placement is not legal");
    }
    write_placement(config, db, output_dir, &global_png, report)?;
    Ok(repair)
}

/// The layers to place the movable I/O pins on, when they are placed;
/// none when there are none, the placement is given, or the config names no
/// layers and every one already has a place.
fn io_layers_to_place(
    config: &Config,
    db: &NetlistDB,
    start: PlacementStart,
) -> anyhow::Result<Option<IoLayers>> {
    let movable_io = io::movable_io_pins(db);
    if movable_io.is_empty() || start == PlacementStart::Given {
        return Ok(None);
    }
    if let Some(names) = &config.io_placement.layers {
        return Ok(Some(IoLayers::from_names(names, &db.library)?));
    }
    let unplaced = movable_io
        .iter()
        .filter(|&&p| db.io_pins[p.index()].status == PlacementStatus::Unplaced)
        .count();
    if unplaced > 0 {
        anyhow::bail!(
            "{unplaced} I/O pins are unplaced; [io_placement] horizontal_layer and \
             vertical_layer name the layers to place them on"
        );
    }
    Ok(None)
}

/// The placement to legalize: the given one, which must place every cell,
/// or a global placement from scratch.
fn starting_placement(
    config: &Config,
    db: &mut NetlistDB,
    start: PlacementStart,
    repair: Option<&mut NetlistRepair>,
    report: &mut Report,
) -> anyhow::Result<PlacementModel> {
    let zones = Zones::new(db)?;
    if start == PlacementStart::Given {
        let unplaced = db
            .cells
            .iter()
            .filter(|c| c.status == PlacementStatus::Unplaced)
            .count();
        if unplaced > 0 {
            anyhow::bail!("{unplaced} cells of the given placement are unplaced");
        }
        ui::check("Starting from the given placement");
        return Ok(PlacementModel::from_db(db, &zones));
    }
    let phase_start = Instant::now();
    let model = global_placement(
        config,
        db,
        PlacementModel::from_db(db, &zones),
        Pass::Full,
        repair,
        report,
    )?;
    report.phase("global placement", phase_start);
    Ok(model)
}

/// Legalizes the global placement and frees the pins it left blocked;
/// returns how it legalized, the pin access it used and the pins still
/// blocked.
fn legalize_with_pin_access(
    config: &Config,
    db: &mut NetlistDB,
    report: &mut Report,
) -> anyhow::Result<(Legalization, Option<PinAccess>, usize)> {
    let zones = Zones::new(db)?;
    let movable_cells = db
        .cells
        .iter()
        .filter(|c| !matches!(c.status, PlacementStatus::Fixed | PlacementStatus::Cover))
        .count();
    let phase_start = Instant::now();
    ui::phase("Legalization");
    let legalization = legalize(db, &zones)?;
    ui::check(&format!("Legalized {movable_cells} cells"));
    let access = config
        .routing
        .lowest_signal_layer(&db.library)?
        .and_then(|layer| PinAccess::new(db, layer));
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
    Ok((legalization, access, blocked_pins))
}

/// Improves the legal placement, as OpenROAD-flow-scripts'
/// `improve_placement` and `optimize_mirroring` do after `OpenDP`, or with
/// PARE's own detailed placer otherwise.
fn detailed_placement(
    db: &mut NetlistDB,
    legalization: Legalization,
    access: Option<&PinAccess>,
    blocked_pins: usize,
    report: &mut Report,
) -> anyhow::Result<()> {
    let zones = Zones::new(db)?;
    let phase_start = Instant::now();
    ui::phase("Detailed Placement");
    let before: Vec<_> = db.cells.iter().map(|c| (c.location, c.orient)).collect();
    if let Legalization::DetailedPlacement(_) = legalization {
        let _ = improve_placement(db, IMPROVE_MAX_DISPLACEMENT)?;
        let mirrored = optimize_mirroring(db);
        ui::stat_row("Mirrored:", &mirrored.to_string(), "", "");
    } else {
        let _ = detailed::improve(db, &zones, 3, access);
    }
    let changed = db
        .cells
        .iter()
        .zip(&before)
        .filter(|&(c, &b)| (c.location, c.orient) != b)
        .count();
    let hpwl = PlacementModel::from_db(db, &zones).hpwl();
    ui::stat_row(
        "Cells changed:",
        &changed.to_string(),
        "HPWL (um):",
        &format!("{hpwl:.0}"),
    );
    report.phase("detailed placement", phase_start);
    if let Some(summary) = &mut report.placement {
        summary.hpwl_um = hpwl;
        summary.detailed_moves = changed;
        summary.blocked_pins = blocked_pins;
    }
    Ok(())
}

/// Legalizes the placement again after a step that added or moved cells,
/// reporting it as `phase`; returns its half-perimeter wirelength.
fn legalize_again(db: &mut NetlistDB, report: &mut Report, phase: &str) -> anyhow::Result<f64> {
    let zones = Zones::new(db)?;
    let phase_start = Instant::now();
    ui::phase("Legalization");
    let _ = legalize(db, &zones)?;
    let hpwl = PlacementModel::from_db(db, &zones).hpwl();
    ui::stat_row("HPWL (um):", &format!("{hpwl:.0}"), "", "");
    report.phase(phase, phase_start);
    Ok(hpwl)
}

/// Draws the placement and writes it where the config says, as DEF or, for
/// a Bookshelf input, as a `.pl` file.
fn write_placement(
    config: &Config,
    db: &NetlistDB,
    output_dir: &Path,
    global_png: &Path,
    report: &mut Report,
) -> anyhow::Result<()> {
    let phase_start = Instant::now();
    ui::phase("Output");
    ui::wrote(&path_str(global_png)?);
    let placed_png = output_dir.join("placed.png");
    visualization::draw_placement(db, &path_str(&placed_png)?, 1000, 1000);
    ui::wrote(&path_str(&placed_png)?);
    let placed = if matches!(config.input.design, DesignInput::Bookshelf { .. }) {
        let pl = output_dir.join("placed.pl");
        writer::bookshelf::write_pl(db, &pl)
            .with_context(|| format!("cannot write {}", pl.display()))?;
        pl
    } else {
        let out = config.input.output_def.clone();
        writer::def::write_file(db, &out)
            .with_context(|| format!("cannot write {}", out.display()))?;
        out
    };
    ui::wrote(&path_str(&placed)?);
    report.phase("placement output", phase_start);
    Ok(())
}

/// Legalizes the placement as OpenROAD-flow-scripts' `detailed_placement`
/// does, showing how far the cells moved.
pub(crate) fn legalize(db: &mut NetlistDB, zones: &Zones) -> anyhow::Result<Legalization> {
    let legalization = pare_placer::legalize::legalize(db, zones)?;
    if let Legalization::DetailedPlacement(done) = legalization {
        let um = |d: Dbu| d.get() as f64 / f64::from(db.dbu().get());
        ui::stat_row(
            "Displacement (um):",
            &format!("{:.0}", um(done.total_displacement)),
            "Max (um):",
            &format!("{:.1}", um(done.max_displacement)),
        );
    }
    Ok(legalization)
}

/// Global placement without the movable I/O pins, then I/O pin placement
/// for the cells' positions, as OpenROAD-flow-scripts does before it touches
/// the netlist.
fn place_io_pins_by_cells(
    config: &Config,
    db: &mut NetlistDB,
    zones: &Zones,
    layers: IoLayers,
    report: &mut Report,
) -> anyhow::Result<()> {
    let phase_start = Instant::now();
    ui::phase("Global Placement Without I/O");
    let model = global_placement(
        config,
        db,
        PlacementModel::from_db_without_movable_io(db, zones),
        Pass::SkipIo,
        None,
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
    Ok(())
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
/// routing resources for routability-driven placement and, with `repair`
/// and an SDC file, is repaired along the way for timing-driven placement.
fn global_placement(
    config: &Config,
    db: &mut NetlistDB,
    mut model: PlacementModel,
    pass: Pass,
    repair: Option<&mut NetlistRepair>,
    report: &mut Report,
) -> anyhow::Result<PlacementModel> {
    let gp = &config.global_placement;
    let timing = repair
        .filter(|_| pass == Pass::Full && gp.timing_driven)
        .and_then(NetlistRepair::timed);
    if timing.is_some() {
        check_congestion_follows_timing(gp)?;
    }
    let density = Density::of(&model, gp);
    let (routability, mut estimator) = if gp.routability_driven && pass == Pass::Full {
        let (params, estimator) = routability_setup(config, db)?;
        (Some(params), estimator)
    } else {
        (None, None)
    };
    let params = GlobalParams {
        target_density: density.target,
        target_overflow: gp.target_overflow.get(),
        max_iterations: gp.placer_max_iterations,
        initial_rounds: match pass {
            Pass::SkipIo => 0,
            Pass::Full => global::INITIAL_PLACE_ROUNDS,
        },
        bins: density.bins,
        seed: gp.seed,
        routability,
        timing: timing.as_ref().map(|_| timing_params(gp)),
    };
    ui::placement_table_header();
    let result = if let Some(repair) = timing {
        drop(estimator);
        let mut driver = TimingPlacement {
            config,
            db,
            repair,
            rounds: 0,
        };
        global::place(
            &mut model,
            &params,
            None,
            Some(&mut driver),
            show_global_progress,
        )?
    } else {
        let estimator = estimator
            .as_mut()
            .map(|e| e as &mut dyn CongestionEstimator);
        let Ok(result) = global::place::<global::NoTiming>(
            &mut model,
            &params,
            estimator,
            None,
            show_global_progress,
        );
        result
    };
    eprintln!();
    show_global_result(db, &result)?;
    report.placement = Some(placement_summary(&model, &result, density.movable_cells));
    Ok(model)
}

/// Fails when timing-driven placement would run beside the global-router
/// congestion estimate, which cannot follow the netlist's changes.
fn check_congestion_follows_timing(gp: &GlobalPlacementConfig) -> anyhow::Result<()> {
    if gp.routability_driven && gp.congestion_estimate == CongestionEstimate::GlobalRouter {
        anyhow::bail!(
            "timing-driven placement changes the netlist as it goes, which the global-router \
             congestion estimate cannot follow: set congestion_estimate = \"rudy\" or \
             timing_driven = false"
        );
    }
    Ok(())
}

/// The overflows timing-driven placement repairs the design at.
fn timing_params(gp: &GlobalPlacementConfig) -> TimingParams {
    TimingParams {
        overflows: gp
            .timing_driven_overflows
            .iter()
            .copied()
            .map(Share::get)
            .collect(),
    }
}

/// Shows global placement's progress every [`PROGRESS_INTERVAL`]
/// iterations.
fn show_global_progress(p: &global::Progress) {
    if p.iteration.is_multiple_of(PROGRESS_INTERVAL) {
        ui::placement_iter(p.iteration, p.hpwl, p.overflow, p.penalty, p.step);
    }
}

/// What global placement of `cells` movable cells reached, for the report;
/// shows its wirelength.
fn placement_summary(
    model: &PlacementModel,
    result: &GlobalResult,
    cells: usize,
) -> PlacementSummary {
    let hpwl = model.hpwl();
    ui::stat_row("HPWL (um):", &format!("{hpwl:.0}"), "", "");
    PlacementSummary {
        cells,
        nets: model.nets.len(),
        iterations: result.iterations,
        overflow: result.overflow,
        hpwl_um: hpwl,
        detailed_moves: 0,
        blocked_pins: 0,
        timing_rounds: result.timing_rounds,
    }
}

/// How densely global placement packs `model`: its movable cells, their
/// utilization of the free area, the target density and the bins.
struct Density {
    movable_cells: usize,
    target: f64,
    bins: usize,
}

impl Density {
    /// The density `gp` sets or derives for `model`, shown as it is found.
    fn of(model: &PlacementModel, gp: &GlobalPlacementConfig) -> Self {
        let movable_area: f64 = model
            .density_sizes
            .iter()
            .zip(&model.movable)
            .filter(|(_, m)| **m)
            .map(|(s, _)| s.x * s.y)
            .sum();
        let movable_cells = model.movable.iter().filter(|&&m| m).count();
        let utilization = movable_area / model.free_area();
        let target = gp.target_density.map_or_else(
            || global::default_target_density(utilization),
            TargetDensity::get,
        );
        let bins = gp
            .bin_dimension
            .map_or_else(|| global::default_bins(model, target), NonZeroUsize::get);
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
            &format!("{target:.2}"),
        );
        ui::stat_row("Bin Grid:", &format!("{bins}x{bins}"), "", "");
        Self {
            movable_cells,
            target,
            bins,
        }
    }
}

/// Shows the congestion rounds, the zones and how global placement ended;
/// an error when it diverged.
fn show_global_result(db: &NetlistDB, result: &GlobalResult) -> anyhow::Result<()> {
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
    Ok(())
}

/// Repairs the design between iterations of timing-driven global
/// placement, as `OpenROAD`'s resizer does for its placer: `repair_design`
/// with the buffers' wire length limit, every net rebuffered for timing,
/// and the worst tenth of the nets weighted by their slack.
struct TimingPlacement<'a> {
    config: &'a Config,
    db: &'a mut NetlistDB,
    repair: TimedRepair<'a>,
    rounds: usize,
}

impl TimingDriver for TimingPlacement<'_> {
    type Error = anyhow::Error;

    fn repair(&mut self, model: &PlacementModel) -> anyhow::Result<Option<Reweighted>> {
        let round_start = Instant::now();
        self.rounds += 1;
        model.write_back(self.db);
        let (round, renumbering) = self.repair_round()?;
        show_timing_round(self.rounds, &round, round_start.elapsed());
        if round.weights.is_empty() {
            log::warn!("timing-driven placement found no net slacks; it stops here");
            return Ok(None);
        }
        Ok(Some(reweighted_model(
            self.db,
            &renumbering,
            round.weights,
        )?))
    }
}

impl TimingPlacement<'_> {
    /// Repairs the design and rebuffers every net for timing, as one round
    /// of `OpenROAD`'s timing-driven placement does, and weights the nets by
    /// their slack; returns the round and how the design was renumbered.
    fn repair_round(&mut self) -> anyhow::Result<(TimingRound, Renumbering)> {
        let stage = self.repair.stage(self.config, self.db)?;
        let cells_before = self.db.cells.len();
        let inputs = stage.session(self.db, self.repair.wires, self.repair.namer);
        let mut resizer = Resizer::timed(inputs, self.repair.sdc)?;
        let area_before = resizer.design_area();
        let wns_before = resizer.sta().worst_slack(MinMax::Max).0.time();
        resizer.set_max_wire_length(pare_resizer::rebuffer::max_wire_length(&resizer));
        let done = Repairer::new(&mut resizer).repair_design()?;
        let mut rebuffer = Rebuffer::new(&resizer)?;
        rebuffer.fully_rebuffer(&mut resizer)?;
        let slacks = resizer.driver_net_slacks();
        let weights =
            timing_net_weights(&slacks, self.config.global_placement.timing_net_weight_max);
        log::debug!(
            "timing weights: worst net slack {:.4} ns, largest weight {:.4}",
            slacks.first().map_or(f64::INFINITY, |s| s.1.time().nanos()),
            weights.iter().map(|w| w.1).fold(1.0, f64::max)
        );
        let wns_after = resizer.sta().worst_slack(MinMax::Max).0.time();
        let area_after = resizer.design_area();
        let renumbering = resizer.finish()?;
        let round = TimingRound {
            wns_before,
            wns_after,
            area_change_percent: area_change_percent(area_before, area_after),
            resized: done.resized,
            repeaters: done.inserted,
            rebuffer_removed: rebuffer.removed,
            rebuffer_inserted: rebuffer.inserted,
            nets: slacks.len(),
            weights,
            cells_before,
            cells_after: self.db.cells.len(),
        };
        Ok((round, renumbering))
    }
}

/// What one round of timing-driven repair did.
struct TimingRound {
    wns_before: Seconds,
    wns_after: Seconds,
    area_change_percent: f64,
    resized: usize,
    repeaters: usize,
    rebuffer_removed: usize,
    rebuffer_inserted: usize,
    /// The nets with a slack.
    nets: usize,
    /// The nets weighted for timing, with their weights.
    weights: Vec<(NetId, f64)>,
    cells_before: usize,
    cells_after: usize,
}

/// Shows what timing round `number` did in `elapsed`.
fn show_timing_round(number: usize, round: &TimingRound, elapsed: Duration) {
    ui::stat_row(
        &format!("Timing round {number}:"),
        &format!(
            "WNS {:.3} -> {:.3} ns",
            round.wns_before.nanos(),
            round.wns_after.nanos()
        ),
        "Area:",
        &format!("{:+.2}%", round.area_change_percent),
    );
    ui::stat_row(
        "Repair:",
        &format!("{} resized, {} repeaters", round.resized, round.repeaters),
        "Rebuffer:",
        &format!(
            "{} removed, {} inserted",
            round.rebuffer_removed, round.rebuffer_inserted
        ),
    );
    ui::stat_row(
        "Weighted nets:",
        &format!("{} of {}", round.weights.len(), round.nets),
        "Cells:",
        &format!("{} -> {}", round.cells_before, round.cells_after),
    );
    ui::stat_row(
        "Round time:",
        &format!("{:.1}s", elapsed.as_secs_f64()),
        "",
        "",
    );
}

/// The placement model of `db` after a round's repair, its nets weighted by
/// `weights` under their old numbers, and where each old cell went.
fn reweighted_model(
    db: &NetlistDB,
    renumbering: &Renumbering,
    weights: Vec<(NetId, f64)>,
) -> anyhow::Result<Reweighted> {
    let zones = Zones::new(db)?;
    let mut model = PlacementModel::from_db(db, &zones);
    let weight_of: std::collections::HashMap<NetId, f64> = weights
        .into_iter()
        .filter_map(|(net, weight)| renumbering.nets[net.index()].map(|new| (new, weight)))
        .collect();
    for net in &mut model.nets {
        if let Some(weight) = weight_of.get(&net.net) {
            net.weight *= weight;
        }
    }
    let cell_map = renumbering
        .cells
        .iter()
        .map(|c| c.map(CellId::index))
        .collect();
    Ok(Reweighted { model, cell_map })
}

/// The routability settings of `config`, and the global-router estimator
/// when congestion is to be measured that way.
fn routability_setup<'a>(
    config: &Config,
    db: &'a NetlistDB,
) -> anyhow::Result<(RoutabilityParams, Option<GlobalRouteCongestion<'a>>)> {
    let adjustment = layer_adjustments(config, db)?;
    match config.global_placement.congestion_estimate {
        CongestionEstimate::GlobalRouter => {
            let estimator = GlobalRouteCongestion::new(db, config)?;
            let reduction = estimator.capacity_reduction(&adjustment);
            Ok((routability_params(config, db, reduction), Some(estimator)))
        }
        CongestionEstimate::Rudy => {
            let reduction = pare_router::capacity_reduction(db, &config.routing, &adjustment)?;
            Ok((routability_params(config, db, reduction), None))
        }
    }
}

/// The share of each routing layer's capacity held back, by layer index.
fn layer_adjustments(config: &Config, db: &NetlistDB) -> anyhow::Result<Vec<Share>> {
    let gp = &config.global_placement;
    let layers = &db.library.routing_layers;
    let mut adjustments = vec![gp.routing_layer_adjustment; layers.len()];
    for entry in &gp.layer_adjustments {
        let index = layers
            .iter()
            .position(|l| l.name == entry.layer)
            .with_context(|| format!("no routing layer named {}", entry.layer))?;
        adjustments[index] = entry.adjustment;
    }
    Ok(adjustments)
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

    fn capacity_reduction(&self, adjustments: &[Share]) -> CapacityReduction {
        self.estimator.capacity_reduction(adjustments)
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

/// What routability-driven placement takes from the router: its gcells
/// with the capacity taken before placement, and the width RUDY gives a
/// wire on its layers.
fn routability_params(
    config: &Config,
    db: &NetlistDB,
    reduction: CapacityReduction,
) -> RoutabilityParams {
    let units = db.dbu();
    RoutabilityParams {
        tiles: RoutingTiles {
            origin: Point::new(
                units.to_microns(reduction.origin.x),
                units.to_microns(reduction.origin.y),
            ),
            size: units.to_microns(reduction.gcell),
            nx: reduction.nx,
            ny: reduction.ny,
            capacity: reduction.capacity,
            reduction: reduction.reduction,
        },
        wire_width: units.to_microns(reduction.wire_width),
        target_congestion: config.global_placement.target_congestion,
    }
}
