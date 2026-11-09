//! Placement: global placement with its timing and routability drivers,
//! legalization and detailed placement.

use crate::checks::report_placement;
use crate::cts::clock_tree_synthesis;
use crate::phase::{Phase, PhaseTimer};
use crate::plan::FlowPlan;
use crate::prepare_output_dir;
use crate::repair::{NetlistRepair, TimedRepair, repair_design, repair_netlist, repair_timing};
use crate::report::{GlobalPlacementSummary, PlacementSummary, Report};
use crate::{ui, visualization};
use anyhow::Context;
use pare_common::config::{
    Config, CongestionEstimate, DesignInput, GlobalPlacementConfig, Share, TargetDensity,
};
use pare_common::db::design::{NetlistDB, PlacementStatus, Renumbering};
use pare_common::db::indices::{CellId, IdVec, LayerId, NetId};
use pare_common::db::liberty::MinMax;
use pare_common::db::writer;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::units::Seconds;
use pare_common::verify::pin_access::PinAccess;
use pare_placer::detailed;
use pare_placer::global::{
    self, CongestionEstimator, GlobalParams, GlobalPlacementError, GlobalResult, Outcome,
    Reweighted, RoutabilityParams, RoutingTiles, TimingDriver, TimingParams,
};
use pare_placer::io::{self, IoLayers};
use pare_placer::legalize::Legalization;
use pare_placer::legalize::opendp::improve::{MaxDisplacement, RowHeights, improve_placement};
use pare_placer::legalize::opendp::mirror::{hpwl, optimize_mirroring};
use pare_placer::model::PlacementModel;
use pare_placer::zones::{Zone, Zones};
use pare_resizer::{Rebuffer, Repairer, Resizer, area_change_percent, timing_net_weights};
use pare_router::{CapacityReduction, GlobalRouteEstimator};
use std::num::NonZeroUsize;
use std::path::Path;
use std::time::{Duration, Instant};

/// Rounds of swaps and reordering detailed placement runs on a design that
/// does not follow `OpenROAD`'s flow.
const DETAILED_PASSES: usize = 3;

/// Pixels along the die's longer side in a placement image.
const PLACEMENT_IMAGE_PIXELS: u32 = 1000;

/// Iterations between rows of global placement's progress table.
const PROGRESS_INTERVAL: usize = 50;

/// How far `improve_placement` may move a cell, in row heights across and
/// up or down: OpenROAD-flow-scripts' `DPO_MAX_DISPLACEMENT` of `5 1`.
const IMPROVE_MAX_DISPLACEMENT: MaxDisplacement = MaxDisplacement {
    x: Some(RowHeights(5)),
    y: Some(RowHeights(1)),
};

/// Where placement starts from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PlacementStart {
    /// Global placement from a random start.
    Scratch,
    /// The positions the design already has, which are only legalized and
    /// refined.
    Given,
}

/// Global placement, I/O pin placement, legalization, checking and output,
/// as `plan` resolves the config; returns the netlist repair, when the run
/// repaired the netlist, for the repair after global routing.
pub(crate) fn run_placement<'p>(
    config: &Config,
    plan: &'p FlowPlan,
    db: &mut NetlistDB,
    start: PlacementStart,
    report: &mut Report,
) -> anyhow::Result<Option<NetlistRepair<'p>>> {
    let output_dir = prepare_output_dir(config)?;
    if let Some(layers) = io_layers_to_place(config, db, start)? {
        ui::section("Pin Placement");
        place_io_pins_by_cells(config, db, &Zones::new(db)?, layers, report)?;
    }
    let mut repair = match (start, &plan.repair) {
        (PlacementStart::Scratch, Some(repair_plan)) => {
            Some(repair_netlist(config, repair_plan, db, report)?)
        }
        _ => None,
    };
    ui::section("Placement");
    let global = match start {
        PlacementStart::Given => {
            keep_given_placement(db)?;
            None
        }
        PlacementStart::Scratch => {
            let full = Pass::Full {
                adjustments: &plan.layer_adjustments,
                repair: repair.as_mut(),
            };
            Some(place_from_scratch(config, db, full, report)?)
        }
    };
    if let Some(repair) = &mut repair {
        repair_design(config, db, repair, report)?;
    }
    let global_png = output_dir.join("nesterov_placer.png");
    draw_placement(db, &global_png)?;
    let legalized = legalize_with_pin_access(config, db, report)?;
    let detailed = detailed_placement(db, &legalized, report)?;
    if let Some(repair) = &mut repair {
        clock_trees_and_timing(config, db, repair, report)?;
    }
    report.placement = Some(PlacementSummary {
        global,
        blocked_pins: legalized.pin_access.map(|checked| checked.blocked),
        detailed_moves: detailed.moves,
        detailed_hpwl_um: detailed.hpwl_um,
        final_hpwl_um: hpwl_um(db),
    });
    if !report_placement(db, report) {
        anyhow::bail!("placement is not legal");
    }
    write_placement(config, db, &output_dir, &global_png, report)?;
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
        .filter(|&&p| db.io_pins[p].status == PlacementStatus::Unplaced)
        .count();
    if unplaced > 0 {
        anyhow::bail!(
            "{unplaced} I/O pins are unplaced; [io_placement] horizontal_layer and \
             vertical_layer name the layers to place them on"
        );
    }
    Ok(None)
}

/// Starts from the given placement, which must place every cell.
fn keep_given_placement(db: &NetlistDB) -> anyhow::Result<()> {
    let unplaced = db
        .cells
        .iter()
        .filter(|c| c.status == PlacementStatus::Unplaced)
        .count();
    if unplaced > 0 {
        anyhow::bail!("{unplaced} cells of the given placement are unplaced");
    }
    ui::check("Starting from the given placement");
    Ok(())
}

/// Places the design globally from scratch with every connection, as
/// `full` steers it, returning what global placement did.
fn place_from_scratch(
    config: &Config,
    db: &mut NetlistDB,
    full: Pass<'_, '_>,
    report: &mut Report,
) -> anyhow::Result<GlobalPlacementSummary> {
    let timer = PhaseTimer::start(Phase::GlobalPlacement);
    let model = PlacementModel::from_db(db, &Zones::new(db)?);
    let summary = global_placement(config, db, model, full)?;
    timer.finish(report);
    Ok(summary)
}

/// How the placement was legalized, and the pin access checked on it.
struct Legalized {
    legalization: Legalization,
    /// The pin access of the lowest signal layer, when vias reach it, with
    /// the pins legalization left blocked.
    pin_access: Option<CheckedAccess>,
}

/// Pin access and the signal pins no via can reach.
struct CheckedAccess {
    access: PinAccess,
    blocked: usize,
}

/// Legalizes the global placement and frees the pins it left blocked.
fn legalize_with_pin_access(
    config: &Config,
    db: &mut NetlistDB,
    report: &mut Report,
) -> anyhow::Result<Legalized> {
    let zones = Zones::new(db)?;
    let movable_cells = db
        .cells
        .iter()
        .filter(|c| !matches!(c.status, PlacementStatus::Fixed | PlacementStatus::Cover))
        .count();
    let timer = PhaseTimer::start(Phase::Legalization);
    let legalization = legalize(db, &zones)?;
    ui::check(&format!("Legalized {movable_cells} cells"));
    let access = config
        .routing
        .lowest_signal_layer(&db.library)?
        .and_then(|layer| PinAccess::new(db, layer));
    let pin_access = access.map(|access| {
        let freed = detailed::free_blocked_pins(db, &zones, &access);
        ui::stat_row(
            "Blocked pins:",
            &format!("{} -> {}", freed.blocked_before, freed.blocked_after),
            "Cells moved:",
            &freed.moved.to_string(),
        );
        CheckedAccess {
            access,
            blocked: freed.blocked_after,
        }
    });
    show_hpwl(hpwl_um(db));
    timer.finish(report);
    Ok(Legalized {
        legalization,
        pin_access,
    })
}

/// What detailed placement did.
struct Detailed {
    /// Cells it moved or turned.
    moves: usize,
    /// The half-perimeter wirelength it left, in microns.
    hpwl_um: f64,
}

/// Improves the legal placement, as OpenROAD-flow-scripts'
/// `improve_placement` and `optimize_mirroring` do after `OpenDP`, or with
/// PARE's own detailed placer otherwise.
fn detailed_placement(
    db: &mut NetlistDB,
    legalized: &Legalized,
    report: &mut Report,
) -> anyhow::Result<Detailed> {
    let zones = Zones::new(db)?;
    let timer = PhaseTimer::start(Phase::DetailedPlacement);
    let before = CellPlaces::record(db);
    if let Legalization::DetailedPlacement(_) = legalized.legalization {
        let _ = improve_placement(db, IMPROVE_MAX_DISPLACEMENT)?;
        let mirrored = optimize_mirroring(db);
        ui::stat_row("Mirrored:", &mirrored.to_string(), "", "");
    } else {
        let access = legalized.pin_access.as_ref().map(|checked| &checked.access);
        let _ = detailed::improve(db, &zones, DETAILED_PASSES, access);
    }
    let detailed = Detailed {
        moves: before.moved(db).len(),
        hpwl_um: hpwl_um(db),
    };
    ui::stat_row(
        "Cells changed:",
        &detailed.moves.to_string(),
        "HPWL (um):",
        &format!("{:.0}", detailed.hpwl_um),
    );
    timer.finish(report);
    Ok(detailed)
}

/// Builds the clock trees and then repairs timing on the detailed
/// placement, legalizing after each, as OpenROAD-flow-scripts' clock tree
/// stage does.
fn clock_trees_and_timing(
    config: &Config,
    db: &mut NetlistDB,
    repair: &mut NetlistRepair<'_>,
    report: &mut Report,
) -> anyhow::Result<()> {
    if clock_tree_synthesis(db, repair, report)? {
        legalize_again(db, report, Phase::LegalizationAfterClockTrees)?;
    }
    if repair_timing(config, db, repair, report)? {
        legalize_again(db, report, Phase::LegalizationAfterTimingRepair)?;
    }
    Ok(())
}

/// Legalizes the placement again after a step that added or moved cells,
/// timing it as `phase`.
fn legalize_again(db: &mut NetlistDB, report: &mut Report, phase: Phase) -> anyhow::Result<()> {
    let zones = Zones::new(db)?;
    let timer = PhaseTimer::start(phase);
    let _ = legalize(db, &zones)?;
    show_hpwl(hpwl_um(db));
    timer.finish(report);
    Ok(())
}

/// The half-perimeter wirelength of the placed signal nets, in microns.
fn hpwl_um(db: &NetlistDB) -> f64 {
    db.dbu().to_microns(hpwl(db))
}

/// Shows the half-perimeter wirelength `um`, in microns.
fn show_hpwl(um: f64) {
    ui::stat_row("HPWL (um):", &format!("{um:.0}"), "", "");
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
    let timer = PhaseTimer::start(Phase::PlacementOutput);
    ui::wrote(global_png);
    let placed_png = output_dir.join("placed.png");
    draw_placement(db, &placed_png)?;
    ui::wrote(&placed_png);
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
    ui::wrote(&placed);
    timer.finish(report);
    Ok(())
}

/// Legalizes the placement as OpenROAD-flow-scripts' `detailed_placement`
/// does, showing how far the cells moved.
pub(crate) fn legalize(db: &mut NetlistDB, zones: &Zones) -> anyhow::Result<Legalization> {
    let legalization = pare_placer::legalize::legalize(db, zones)?;
    if let Legalization::DetailedPlacement(done) = legalization {
        let um = |d: Dbu| db.dbu().to_microns(d);
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
    let timer = PhaseTimer::start(Phase::GlobalPlacementWithoutIo);
    let model = PlacementModel::from_db_without_movable_io(db, zones);
    let _ = global_placement(config, db, model, Pass::SkipIo)?;
    timer.finish(report);

    let timer = PhaseTimer::start(Phase::IoPinPlacement);
    let placed = io::place_io_pins(db, layers)?;
    ui::stat_row(
        "Pins:",
        &placed.pins.to_string(),
        "Free slots:",
        &placed.slots.to_string(),
    );
    timer.finish(report);
    Ok(())
}

/// Which global placement pass runs, and what may steer it.
enum Pass<'a, 'p> {
    /// Before the I/O pins are placed: like `OpenROAD`'s skip-I/O mode,
    /// neither routability- nor timing-driven.
    SkipIo,
    /// With every connection in place.
    Full {
        /// The share of each layer's capacity routability-driven placement
        /// counts as taken.
        adjustments: &'a IdVec<LayerId, Share>,
        /// The netlist repair timing-driven placement makes, when there is
        /// one.
        repair: Option<&'a mut NetlistRepair<'p>>,
    },
}

impl Pass<'_, '_> {
    /// Rounds of initial placement before the Nesterov iterations.
    const fn initial_rounds(&self) -> usize {
        match self {
            Self::SkipIo => 0,
            Self::Full { .. } => global::INITIAL_PLACE_ROUNDS,
        }
    }
}

/// Runs global placement on `model`, writes it into `db` and shows it,
/// returning what it did; `db` supplies the routing resources for
/// routability-driven placement and, with a repair that has constraints,
/// is repaired along the way for timing-driven placement.
fn global_placement(
    config: &Config,
    db: &mut NetlistDB,
    mut model: PlacementModel,
    pass: Pass<'_, '_>,
) -> anyhow::Result<GlobalPlacementSummary> {
    let gp = &config.global_placement;
    let density = Density::of(&model, gp)?;
    let initial_rounds = pass.initial_rounds();
    let (adjustments, repair) = match pass {
        Pass::SkipIo => (None, None),
        Pass::Full {
            adjustments,
            repair,
        } => (Some(adjustments), repair),
    };
    let timing = repair
        .filter(|_| gp.timing_driven)
        .and_then(NetlistRepair::timed);
    let (routability, estimator) = match adjustments.filter(|_| gp.routability_driven) {
        Some(adjustments) => {
            let (params, estimator) = routability_setup(config, db, adjustments)?;
            (Some(params), estimator)
        }
        None => (None, None),
    };
    let params = GlobalParams {
        initial_rounds,
        routability,
        timing: timing.as_ref().map(|_| timing_params(gp)),
        ..global_params(gp, &density)
    };
    let steering = match timing {
        Some(repair) => {
            drop(estimator);
            Steering::Timing(TimingPlacement {
                db,
                repair,
                net_weight_max: gp.timing_net_weight_max,
                rounds: 0,
            })
        }
        None => Steering::Congestion(estimator.map(Box::new)),
    };
    ui::placement_table_header();
    let result = place_globally(&mut model, &params, steering)?;
    eprintln!();
    show_global_result(db, &result)?;
    model.write_back(db);
    let wirelength = hpwl_um(db);
    show_hpwl(wirelength);
    Ok(GlobalPlacementSummary {
        cells: density.movable_cells,
        nets: model.nets.len(),
        iterations: result.iterations,
        overflow: result.overflow,
        hpwl_um: wirelength,
        timing_rounds: result.timing_rounds,
    })
}

/// What steers global placement besides wirelength and density: repairs
/// for timing, or else the congestion the global router measures, when it
/// is the estimate.
enum Steering<'a> {
    Timing(TimingPlacement<'a>),
    Congestion(Option<Box<GlobalRouteCongestion<'a>>>),
}

/// The global placer's parameters for the configured placement, without
/// initial rounds and neither routability- nor timing-driven.
const fn global_params(gp: &GlobalPlacementConfig, density: &Density) -> GlobalParams {
    GlobalParams {
        target_density: density.target,
        target_overflow: gp.target_overflow,
        max_iterations: gp.placer_max_iterations,
        initial_rounds: 0,
        bins: density.bins,
        seed: gp.seed,
        routability: None,
        timing: None,
    }
}

/// Runs the global placer on `model` under `steering`.
fn place_globally(
    model: &mut PlacementModel,
    params: &GlobalParams,
    steering: Steering<'_>,
) -> anyhow::Result<GlobalResult> {
    match steering {
        Steering::Timing(mut driver) => {
            global::place(model, params, None, Some(&mut driver), show_global_progress).map_err(
                |e| match e {
                    GlobalPlacementError::Repair(repair) => repair,
                    other => other.into(),
                },
            )
        }
        Steering::Congestion(mut estimator) => {
            let estimator = estimator
                .as_deref_mut()
                .map(|e| e as &mut dyn CongestionEstimator);
            Ok(global::place::<global::NoTiming>(
                model,
                params,
                estimator,
                None,
                show_global_progress,
            )?)
        }
    }
}

/// The overflows timing-driven placement repairs the design at.
fn timing_params(gp: &GlobalPlacementConfig) -> TimingParams {
    TimingParams {
        overflows: gp.timing_driven_overflows.clone(),
    }
}

/// Shows global placement's progress every [`PROGRESS_INTERVAL`]
/// iterations.
fn show_global_progress(p: &global::Progress) {
    if p.iteration.is_multiple_of(PROGRESS_INTERVAL) {
        ui::placement_iter(p);
    }
}

/// How densely global placement packs `model`: its movable cells, their
/// utilization of the free area, the target density and the bins.
struct Density {
    movable_cells: usize,
    target: TargetDensity,
    bins: usize,
}

impl Density {
    /// The density `gp` sets or derives for `model`, shown as it is found.
    fn of(model: &PlacementModel, gp: &GlobalPlacementConfig) -> anyhow::Result<Self> {
        let movable_area: f64 = model
            .density_sizes
            .iter()
            .zip(&model.movable)
            .filter(|(_, m)| **m)
            .map(|(s, _)| s.x * s.y)
            .sum();
        let movable_cells = model.movable.iter().filter(|&&m| m).count();
        let utilization = movable_area / model.free_area();
        let target = if let Some(target) = gp.target_density {
            target
        } else {
            global::default_target_density(utilization).with_context(|| {
                format!("utilization {utilization} gives no target density in (0, 1]")
            })?
        };
        let bins = gp.bin_dimension.map_or_else(
            || global::default_bins(model, target.get()),
            NonZeroUsize::get,
        );
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
            &format!("{:.2}", target.get()),
        );
        ui::stat_row("Bin Grid:", &format!("{bins}x{bins}"), "", "");
        Ok(Self {
            movable_cells,
            target,
            bins,
        })
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
                Zone::Fence(r) => db.regions[r].name.as_str(),
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
    db: &'a mut NetlistDB,
    repair: TimedRepair<'a>,
    /// The weight the net of worst slack gets.
    net_weight_max: f64,
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
        let stage = self.repair.stage(self.db)?;
        let cells_before = self.db.cells.len();
        let inputs = stage.session(self.db, self.repair.plan.wires, self.repair.namer);
        let mut resizer = Resizer::timed(inputs, self.repair.sdc)?;
        let area_before = resizer.design_area();
        let wns_before = resizer.sta().worst_slack(MinMax::Max).0.time();
        resizer.set_max_wire_length(pare_resizer::max_wire_length(&resizer));
        let done = Repairer::new(&mut resizer).repair_design()?;
        let mut rebuffer = Rebuffer::new(&resizer)?;
        rebuffer.fully_rebuffer(&mut resizer)?;
        let slacks = resizer.driver_net_slacks();
        let weights = timing_net_weights(&slacks, self.net_weight_max);
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
    ui::stat_row("Round time:", &ui::seconds(elapsed), "", "");
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
        .filter_map(|(net, weight)| renumbering.nets[net].map(|new| (new, weight)))
        .collect();
    for net in &mut model.nets {
        if let Some(weight) = weight_of.get(&net.net) {
            net.weight *= weight;
        }
    }
    Ok(Reweighted {
        model,
        cell_map: renumbering.cells.clone(),
    })
}

/// Draws the placement of `db` into the image at `path`.
fn draw_placement(db: &NetlistDB, path: &Path) -> anyhow::Result<()> {
    visualization::draw_placement(db, path, PLACEMENT_IMAGE_PIXELS)
        .with_context(|| format!("cannot write {}", path.display()))
}

/// Where each cell of a design sits and how it is turned, to tell later
/// which cells a step moved.
pub(crate) struct CellPlaces(Vec<(Point<Dbu>, Orientation)>);

impl CellPlaces {
    /// Where the cells of `db` sit now.
    pub(crate) fn record(db: &NetlistDB) -> Self {
        Self(db.cells.iter().map(|c| (c.location, c.orient)).collect())
    }

    /// The cells of `db` placed elsewhere or turned since the record.
    pub(crate) fn moved(&self, db: &NetlistDB) -> Vec<CellId> {
        db.cells
            .iter()
            .zip(&self.0)
            .enumerate()
            .filter(|(_, (c, was))| (c.location, c.orient) != **was)
            .map(|(i, _)| CellId::new(i))
            .collect()
    }
}

/// The routability settings of `config` with the layers' capacity held
/// back by `adjustment`, and the global-router estimator when congestion is
/// to be measured that way.
fn routability_setup<'a>(
    config: &Config,
    db: &'a NetlistDB,
    adjustment: &IdVec<LayerId, Share>,
) -> anyhow::Result<(RoutabilityParams, Option<GlobalRouteCongestion<'a>>)> {
    match config.global_placement.congestion_estimate {
        CongestionEstimate::GlobalRouter => {
            let estimator = GlobalRouteCongestion::new(db, config)?;
            let reduction = estimator.capacity_reduction(adjustment);
            Ok((routability_params(config, db, reduction), Some(estimator)))
        }
        CongestionEstimate::Rudy => {
            let reduction = pare_router::capacity_reduction(db, &config.routing, adjustment)?;
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
    sizes: IdVec<CellId, Point<f64>>,
}

impl<'a> GlobalRouteCongestion<'a> {
    fn new(db: &'a NetlistDB, config: &Config) -> anyhow::Result<Self> {
        let units = db.dbu();
        Ok(Self {
            db,
            estimator: GlobalRouteEstimator::new(db, &config.routing)?,
            sizes: db
                .cells
                .ids()
                .map(|id| {
                    let size = db.cell_size(id);
                    Point::new(units.to_microns(size.x), units.to_microns(size.y))
                })
                .collect(),
        })
    }

    fn capacity_reduction(&self, adjustments: &IdVec<LayerId, Share>) -> CapacityReduction {
        self.estimator.capacity_reduction(adjustments)
    }
}

impl CongestionEstimator for GlobalRouteCongestion<'_> {
    fn congestion(&mut self, centers: &IdVec<CellId, Point<f64>>) -> Vec<f64> {
        let units = self.db.dbu();
        let locations: IdVec<CellId, Point<Dbu>> = centers
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{half_height_tiny, scratch_dir, tiny, tiny_config};

    #[test]
    fn cells_moved_or_turned_since_the_record_are_the_moved_ones() {
        let mut db = tiny();
        let before = CellPlaces::record(&db);
        let moved = db.cell_id("u1").unwrap();
        let turned = db.cell_id("u2").unwrap();

        db.cells[moved].location.x += Dbu(200);
        db.cells[turned].orient = Orientation::N;

        assert_eq!(before.moved(&db), vec![moved, turned]);
    }

    #[test]
    fn a_placement_image_keeps_the_die_in_proportion() {
        let path = scratch_dir("placement-image").join("placed.png");

        draw_placement(&half_height_tiny(), &path).unwrap();

        assert_eq!(
            image::image_dimensions(&path).unwrap(),
            (PLACEMENT_IMAGE_PIXELS, PLACEMENT_IMAGE_PIXELS / 2)
        );
    }

    #[test]
    fn nothing_moved_since_the_record_leaves_nothing_moved() {
        let db = tiny();

        assert_eq!(CellPlaces::record(&db).moved(&db), Vec::new());
    }

    fn model(db: &NetlistDB) -> PlacementModel {
        PlacementModel::from_db(db, &Zones::new(db).unwrap())
    }

    #[test]
    fn the_density_takes_the_target_and_bins_the_config_sets() {
        let db = tiny();
        let config = tiny_config("[global_placement]\ntarget_density = 0.7\nbin_dimension = 8");

        let density = Density::of(&model(&db), &config.global_placement).unwrap();

        assert_eq!(density.target, TargetDensity::new(0.7).unwrap());
        assert_eq!(density.bins, 8);
    }

    #[test]
    fn the_density_counts_the_cells_that_are_not_fixed() {
        let db = tiny();
        let config = tiny_config("");

        let density = Density::of(&model(&db), &config.global_placement).unwrap();

        assert_eq!(density.movable_cells, 6);
    }

    #[test]
    fn without_a_target_or_bins_the_density_derives_them_from_the_utilization() {
        let db = tiny();
        let model = model(&db);
        let config = tiny_config("");

        let density = Density::of(&model, &config.global_placement).unwrap();

        let movable_area: f64 = model
            .density_sizes
            .iter()
            .zip(&model.movable)
            .filter(|(_, movable)| **movable)
            .map(|(size, _)| size.x * size.y)
            .sum();
        let target = global::default_target_density(movable_area / model.free_area()).unwrap();
        assert_eq!(density.target, target);
        assert_eq!(density.bins, global::default_bins(&model, target.get()));
    }

    fn set_io_status(db: &mut NetlistDB, name: &str, status: PlacementStatus) {
        let pin = db.io_pin_id(name).unwrap();
        db.io_pins[pin].status = status;
    }

    #[test]
    fn placed_io_pins_keep_their_places_without_layers_to_move_them_to() {
        let config = tiny_config("");

        let layers = io_layers_to_place(&config, &tiny(), PlacementStart::Scratch).unwrap();

        assert_eq!(layers, None);
    }

    #[test]
    fn io_pins_are_placed_on_the_layers_the_config_names() {
        let db = tiny();
        let config =
            tiny_config("[io_placement]\nhorizontal_layer = \"M3\"\nvertical_layer = \"M2\"");

        let layers = io_layers_to_place(&config, &db, PlacementStart::Scratch).unwrap();

        let id = |name: &str| db.library.routing_layer_id(name).unwrap();
        assert_eq!(
            layers,
            Some(IoLayers {
                horizontal: id("M3"),
                vertical: id("M2"),
            })
        );
    }

    #[test]
    fn a_given_placement_places_no_io_pins() {
        let mut db = tiny();
        set_io_status(&mut db, "out", PlacementStatus::Unplaced);
        let config =
            tiny_config("[io_placement]\nhorizontal_layer = \"M3\"\nvertical_layer = \"M2\"");

        let layers = io_layers_to_place(&config, &db, PlacementStart::Given).unwrap();

        assert_eq!(layers, None);
    }

    #[test]
    fn fixed_io_pins_need_no_layers() {
        let mut db = tiny();
        set_io_status(&mut db, "out", PlacementStatus::Fixed);
        let config =
            tiny_config("[io_placement]\nhorizontal_layer = \"M3\"\nvertical_layer = \"M2\"");

        let layers = io_layers_to_place(&config, &db, PlacementStart::Scratch).unwrap();

        assert_eq!(layers, None);
    }

    #[test]
    fn unplaced_io_pins_without_layers_to_place_them_on_are_an_error() {
        let mut db = tiny();
        set_io_status(&mut db, "out", PlacementStatus::Unplaced);

        let error = io_layers_to_place(&tiny_config(""), &db, PlacementStart::Scratch).unwrap_err();

        assert_eq!(
            error.to_string(),
            "1 I/O pins are unplaced; [io_placement] horizontal_layer and vertical_layer \
             name the layers to place them on"
        );
    }
}
