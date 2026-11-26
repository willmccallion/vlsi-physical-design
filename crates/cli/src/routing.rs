//! Global routing with repair on its parasitics, and detailed routing.

use crate::checks::report_routing;
use crate::phase::{Phase, PhaseTimer};
use crate::placement::{CellPlaces, legalize};
use crate::prepare_output_dir;
use crate::repair::{
    Clocks, NetlistRepair, Stage as RepairStage, TimedRepair, load_liberty, propagate_clocks,
    repair_design_on, timing_repair_on,
};
use crate::report::{
    DesignRepairSummary, GlobalRouteRepairSummary, Report, RoutingSummary, TimingSummary,
};
use crate::{ui, visualization};
use anyhow::Context;
use pare_common::config::Config;
use pare_common::db::design::{NetlistDB, Renumbering};
use pare_common::db::global_route::RouteSource;
use pare_common::db::indices::{CellId, NetId};
use pare_common::db::liberty::Liberty;
use pare_common::db::library::Library;
use pare_common::db::writer;
use pare_common::memory;
use pare_placer::legalize::site::SiteGrid;
use pare_placer::zones::Zones;
use pare_resizer::GlobalRouting;
use pare_resizer::{Resizer, RouteRc};
use pare_router::{GlobalSession, LeafClockPins, Progress, RouteSetup, Stage};
use std::path::{Path, PathBuf};
use std::time::Duration;

/// Pixels along the die's longer side in the routed design's image.
const ROUTED_IMAGE_PIXELS: u32 = 2000;

/// How the router runs: where it saves its state and when, the state it
/// resumes from, and what it traces.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct RouterRun {
    pub(crate) save: Option<SaveRequest>,
    pub(crate) resume: Option<PathBuf>,
    pub(crate) trace: pare_router::TraceRequest,
}

/// Where to save the router's state, and after which stage.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct SaveRequest {
    pub(crate) path: PathBuf,
    pub(crate) after: pare_router::SavePoint,
}

/// A `--save-after` value: `global` or a detailed round number.
pub(crate) fn parse_save_point(text: &str) -> Result<pare_router::SavePoint, String> {
    if text == "global" {
        return Ok(pare_router::SavePoint::Global);
    }
    text.parse()
        .map(pare_router::SavePoint::Round)
        .map_err(|_| format!("expected `global` or a round number, got `{text}`"))
}

/// Reads a route state saved by `--save-state`.
fn read_state(path: &Path) -> anyhow::Result<pare_router::RouteState> {
    let text =
        std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    pare_router::RouteState::parse(&text).with_context(|| format!("reading {}", path.display()))
}

/// Writes a route state where `--save-state` asked.
fn write_state(path: &Path, state: &pare_router::RouteState) -> anyhow::Result<()> {
    use std::io::Write as _;
    let file =
        std::fs::File::create(path).with_context(|| format!("writing {}", path.display()))?;
    let mut out = std::io::BufWriter::new(file);
    state
        .write(&mut out)
        .and_then(|()| out.flush())
        .with_context(|| format!("writing {}", path.display()))
}

/// The pins of `library`'s masters that make a clock net a leaf, as
/// `liberty` tells them: those that clock a register or open a latch, and
/// every pin of a pad cell.
pub(crate) fn leaf_clock_pins(library: &Library, liberty: &Liberty) -> LeafClockPins {
    let mut leaves = Vec::new();
    for (master_id, master) in library.macros.iter_enumerated() {
        let Some(cell) = liberty.cell_id(&master.name).map(|c| liberty.cell(c)) else {
            continue;
        };
        for (pin_id, pin) in master.pins.iter_enumerated() {
            let Some(port) = cell.port(&pin.name) else {
                continue;
            };
            if cell.is_pad || cell.is_register_clock(port) {
                leaves.push((master_id, pin_id));
            }
        }
    }
    leaves.into_iter().collect()
}

/// The leaf clock pins of `db`: by the Liberty `repair` read when given,
/// else by the Liberty files `config` names; none without any, which makes
/// every clock net a trunk, as `OpenROAD` finds them without Liberty.
pub(crate) fn design_leaf_clock_pins(
    config: &Config,
    db: &NetlistDB,
    repair: Option<&NetlistRepair<'_>>,
) -> anyhow::Result<LeafClockPins> {
    if let Some(repair) = repair {
        return Ok(leaf_clock_pins(&db.library, &repair.liberty));
    }
    if config.input.liberty_files.is_empty() {
        return Ok(LeafClockPins::default());
    }
    let liberty = load_liberty(&config.input.liberty_files)?;
    Ok(leaf_clock_pins(&db.library, &liberty))
}

/// Where detailed routing starts.
pub(crate) enum RouteStart {
    /// From the global routes repair ran on.
    OnGlobalRoutes(Box<GlobalSession>),
    /// From global routing of its own, which takes a clock net for a leaf
    /// where it reaches one of these pins.
    FromScratch(LeafClockPins),
}

/// Global routing with design and timing repair on its parasitics, as
/// OpenROAD-flow-scripts' global route stage does it.
///
/// Every clock is propagated, with or without clock trees, as
/// `set_propagated_clock [all_clocks]` there. After `estimate_parasitics
/// -global_routing` come `repair_design` and detailed placement, then
/// `repair_timing` and detailed placement again; the nets of every cell an
/// edit or detailed placement moves are routed again. None when the run has
/// no constraints or the repair is off.
pub(crate) fn global_route_with_repair(
    config: &Config,
    db: &mut NetlistDB,
    repair: &mut NetlistRepair<'_>,
    report: &mut Report,
) -> anyhow::Result<Option<GlobalSession>> {
    let plan = repair.plan;
    let Some(rc) = &plan.on_global_routes else {
        return Ok(None);
    };
    ui::section("Global Routing");
    let timer = PhaseTimer::start(Phase::GlobalRouting);
    let mut tables = RouteTables::default();
    let setup = RouteSetup {
        config: &config.routing,
        leaf_clock_pins: leaf_clock_pins(&db.library, &repair.liberty),
    };
    let mut session = GlobalSession::new(db, &setup, &mut |p| tables.show(p))?;
    timer.finish(report);
    if let Some(timed) = repair.timed() {
        propagate_clocks(timed.sdc);
    }
    let sites = SiteGrid::new(db).context("cannot lay out the core's site grid to legalize on")?;
    let legalize = |db: &mut NetlistDB, cell: CellId| sites.legalize(db, cell);
    let mut routes = RepairRoutes {
        session: &mut session,
        rc,
        legalize: &legalize,
    };
    let mut summary = GlobalRouteRepairSummary::default();
    if config.netlist.repair_design {
        summary.design = repair_design_on_routes(db, repair, &mut routes, report)?;
    }
    if config.netlist.repair_timing
        && let Some(mut timed) = repair.timed()
    {
        summary.timing = Some(repair_timing_on_routes(
            db,
            &mut timed,
            &mut routes,
            report,
        )?);
    }
    report.global_route_repair = Some(summary);
    Ok(Some(session))
}

/// The global routes a repair follows, and how it legalizes the cells it
/// edits.
struct RepairRoutes<'s> {
    session: &'s mut GlobalSession,
    rc: &'s RouteRc,
    legalize: &'s dyn Fn(&mut NetlistDB, CellId),
}

impl RepairRoutes<'_> {
    /// The routes as a resizer session follows them.
    fn followed(&mut self) -> GlobalRouting<'_> {
        GlobalRouting {
            source: &mut *self.session,
            rc: self.rc.clone(),
            legalize: self.legalize,
        }
    }

    /// Takes in how a repair renumbered the design, then legalizes it and
    /// routes again the nets of the cells that moved.
    fn settle(
        &mut self,
        db: &mut NetlistDB,
        renumbering: &Renumbering,
        report: &mut Report,
    ) -> anyhow::Result<()> {
        self.session.renumber(renumbering);
        legalize_and_reroute(db, self.session, report)
    }
}

/// `repair_design` on the global routes' parasitics, timed when the input
/// has constraints, as OpenROAD-flow-scripts' `repair_design_helper` runs
/// it with or without them.
fn repair_design_on_routes(
    db: &mut NetlistDB,
    repair: &mut NetlistRepair<'_>,
    routes: &mut RepairRoutes<'_>,
    report: &mut Report,
) -> anyhow::Result<DesignRepairSummary> {
    let timer = PhaseTimer::start(Phase::DesignRepairOnGlobalRoutes);
    let clocks = repair.constraints.clocks();
    let stage = RepairStage::find(db, repair.plan, &repair.liberty, clocks)?;
    let inputs = stage.session(db, repair.plan.wires, &mut repair.namer);
    let followed = routes.followed();
    let (done, renumbering) = match clocks {
        Clocks::Sdc(sdc) => repair_design_on(Resizer::on_global_routes(inputs, sdc, followed)?)?,
        Clocks::Ports(_) => repair_design_on(Resizer::untimed_on_global_routes(inputs, followed)?)?,
    };
    timer.finish(report);
    routes.settle(db, &renumbering, report)?;
    Ok(DesignRepairSummary::of(&done))
}

/// `repair_timing` on the global routes' parasitics.
fn repair_timing_on_routes(
    db: &mut NetlistDB,
    timed: &mut TimedRepair<'_>,
    routes: &mut RepairRoutes<'_>,
    report: &mut Report,
) -> anyhow::Result<TimingSummary> {
    let timer = PhaseTimer::start(Phase::TimingRepairOnGlobalRoutes);
    let stage = timed.stage(db)?;
    let inputs = stage.session(db, timed.plan.wires, timed.namer);
    let mut resizer = Resizer::on_global_routes(inputs, timed.sdc, routes.followed())?;
    let summary = timing_repair_on(&mut resizer, timed.liberty)?;
    let renumbering = resizer.finish()?;
    timer.finish(report);
    routes.settle(db, &renumbering, report)?;
    Ok(summary)
}

/// Detailed placement after a repair, as OpenROAD-flow-scripts runs
/// `detailed_placement` between `global_route -start_incremental` and
/// `-end_incremental`: the nets of every cell it moves are routed again.
fn legalize_and_reroute(
    db: &mut NetlistDB,
    session: &mut GlobalSession,
    report: &mut Report,
) -> anyhow::Result<()> {
    let timer = PhaseTimer::start(Phase::LegalizationOnGlobalRoutes);
    let before = CellPlaces::record(db);
    let _ = legalize(db, &Zones::new(db)?)?;
    let moved = before.moved(db);
    let mut moved_nets: Vec<NetId> = moved
        .iter()
        .flat_map(|&c| db.cells[c].pins.iter().map(|&p| db.pins[p].net))
        .collect();
    moved_nets.sort_unstable();
    moved_nets.dedup();
    session.update(db, &moved_nets);
    ui::stat_pair(
        ("Cells moved:", &moved.len().to_string()),
        ("Nets updated:", &moved_nets.len().to_string()),
    );
    timer.finish(report);
    Ok(())
}

/// Writes `guides` to `path` in the ISPD 2018 route guide format: each net's
/// name, then its boxes between parentheses, one `x0 y0 x1 y1 layer` a line.
fn write_guides(
    path: &Path,
    db: &NetlistDB,
    guides: &[pare_router::NetGuide],
) -> anyhow::Result<()> {
    use std::fmt::Write as _;
    let mut out = String::new();
    for guide in guides {
        writeln!(out, "{}\n(", db.nets[guide.net].name)?;
        for (layer, r) in &guide.boxes {
            writeln!(
                out,
                "{} {} {} {} {}",
                r.min.x.get(),
                r.min.y.get(),
                r.max.x.get(),
                r.max.y.get(),
                db.library.layer(*layer).name
            )?;
        }
        writeln!(out, ")")?;
    }
    std::fs::write(path, out).with_context(|| format!("writing {}", path.display()))
}

/// Routes every signal net globally and assigns tracks, without detailed
/// routing, taking a clock net for a leaf where it reaches one of
/// `leaf_clock_pins`, and writes the global routes as guides to `guides`
/// when given.
pub(crate) fn route_global(
    config: &Config,
    db: &NetlistDB,
    leaf_clock_pins: LeafClockPins,
    guides: Option<&Path>,
    report: &mut Report,
) -> anyhow::Result<()> {
    ui::section("Routing");
    let timer = PhaseTimer::start(Phase::GlobalRouting);
    let mut tables = RouteTables::default();
    let mut show = |p: &Progress<'_>| tables.show(p);
    let setup = RouteSetup {
        config: &config.routing,
        leaf_clock_pins,
    };
    let global = pare_router::route_global_only(db, &setup, &mut show)?;
    ui::stat("Nets:", &global.summary.nets.to_string());
    if let Some(path) = guides {
        write_guides(path, db, &global.guides)?;
        ui::wrote(path);
    }
    timer.finish(report);
    Ok(())
}

/// Routes every signal net in detail from `start`, saving and resuming
/// state as `run` says, then checks the result and writes it to the output
/// directory.
pub(crate) fn route_detailed(
    config: &Config,
    db: &mut NetlistDB,
    run: &RouterRun,
    start: RouteStart,
    report: &mut Report,
) -> anyhow::Result<()> {
    ui::section("Routing");
    let timer = PhaseTimer::start(Phase::Routing);
    let summary = run_router(config, db, run, start)?;
    show_routing(&summary);
    timer.finish(report);
    report.routing = Some(RoutingSummary {
        nets: summary.nets,
        rounds: summary.rounds,
        inaccessible: summary.inaccessible.len(),
        incomplete: summary.incomplete.len(),
        conflicted: summary.conflicted.len(),
    });

    let timer = PhaseTimer::start(Phase::Verification);
    let routing_ok = report_routing(db, report);
    timer.finish(report);

    let timer = PhaseTimer::start(Phase::RoutingOutput);
    write_routed(db, &prepare_output_dir(config)?)?;
    timer.finish(report);

    if !routing_ok {
        anyhow::bail!("routing has opens or design rule violations");
    }
    Ok(())
}

/// Runs the detailed router over `db` from `start`, saving and resuming
/// its state as `run` says.
fn run_router(
    config: &Config,
    db: &mut NetlistDB,
    run: &RouterRun,
    start: RouteStart,
) -> anyhow::Result<pare_router::RoutingSummary> {
    let resume = run.resume.as_deref().map(read_state).transpose()?;
    let mut write = |state: &pare_router::RouteState| {
        let Some(save) = &run.save else {
            return Ok(());
        };
        write_state(&save.path, state).map_err(|e| pare_router::SaveError(format!("{e:#}")))?;
        ui::wrote(&save.path);
        Ok(())
    };
    let mut tables = RouteTables::default();
    let mut show = |p: &Progress<'_>| tables.show(p);
    let options = pare_router::RunOptions {
        resume: resume.as_ref(),
        save: run.save.as_ref().map(|save| pare_router::Save {
            after: save.after,
            write: &mut write,
        }),
        trace: run.trace.clone(),
        report: Some(&mut show),
    };
    let summary = match start {
        RouteStart::OnGlobalRoutes(global) => {
            pare_router::route_on(db, &config.routing, *global, options)?
        }
        RouteStart::FromScratch(leaf_clock_pins) => {
            let setup = RouteSetup {
                config: &config.routing,
                leaf_clock_pins,
            };
            pare_router::route_with(db, &setup, options)?
        }
    };
    if let Some(save) = &run.save
        && !summary.state_saved
    {
        log::warn!(
            "routing ended before {:?}, so no state was saved to {}",
            save.after,
            save.path.display()
        );
    }
    Ok(summary)
}

/// Shows what detailed routing left: nets, rounds and unfinished nets.
fn show_routing(summary: &pare_router::RoutingSummary) {
    ui::stat_pair(
        ("Nets:", &summary.nets.to_string()),
        ("Rounds:", &summary.rounds.to_string()),
    );
    ui::stat_pair(
        ("No access:", &summary.inaccessible.len().to_string()),
        ("Incomplete:", &summary.incomplete.len().to_string()),
    );
    ui::stat("In conflict:", &summary.conflicted.len().to_string());
}

/// Shows a routing run's progress as `pare`'s tables, the detailed table's
/// header before its first round.
#[derive(Debug, Default)]
struct RouteTables {
    detailed_started: bool,
}

impl RouteTables {
    fn show(&mut self, progress: &Progress<'_>) {
        match progress {
            Progress::Stage { stage, elapsed } => show_stage(*stage, *elapsed),
            Progress::GlobalRouted {
                gcells: (nx, ny),
                overflow,
                layers,
            } => {
                ui::stat_pair(
                    ("GCells:", &format!("{nx}x{ny}")),
                    ("GR overflow:", &overflow.to_string()),
                );
                ui::gr_layer_table_header();
                for layer in *layers {
                    ui::gr_layer_row(layer);
                }
            }
            Progress::GlobalSettled { kept, rerouted } => ui::stat_pair(
                ("Routes kept:", &kept.to_string()),
                ("Routed again:", &rerouted.to_string()),
            ),
            Progress::DetailedRound(round) => {
                if !self.detailed_started {
                    ui::dr_table_header();
                    self.detailed_started = true;
                }
                ui::dr_iter(round);
            }
        }
    }
}

/// Shows how long `stage` took and how much memory the process holds now.
fn show_stage(stage: Stage, elapsed: Duration) {
    let label = match stage {
        Stage::PinAccess => "Pin access:",
        Stage::GCellGrid => "GCell grid:",
        Stage::GlobalRoute => "Global route:",
        Stage::GlobalRestore => "Global restored:",
        Stage::GlobalSettle => "Global settled:",
        Stage::TrackAssignment => "Track assign:",
    };
    let resident = memory::current_rss_mb().map_or_else(String::new, |mb| format!("{mb} MB"));
    ui::stat_pair((label, &ui::seconds(elapsed)), ("Resident:", &resident));
}

/// Writes the routed design's image and DEF into `output_dir`.
fn write_routed(db: &NetlistDB, output_dir: &Path) -> anyhow::Result<()> {
    let png = output_dir.join("routed.png");
    visualization::draw_routed_design(db, &png, ROUTED_IMAGE_PIXELS)
        .with_context(|| format!("cannot write {}", png.display()))?;
    ui::wrote(&png);
    let out = output_dir.join("routed.def");
    writer::def::write_file(db, &out).with_context(|| format!("cannot write {}", out.display()))?;
    ui::wrote(&out);
    Ok(())
}

#[cfg(test)]
mod tests;
