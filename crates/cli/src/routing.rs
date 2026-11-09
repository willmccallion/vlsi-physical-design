//! Global routing with repair on its parasitics, and detailed routing.

use crate::checks::report_routing;
use crate::phase::{Phase, PhaseTimer};
use crate::placement::{CellPlaces, legalize};
use crate::prepare_output_dir;
use crate::repair::{NetlistRepair, propagate_clocks, show_design_repair, timing_repair_on};
use crate::report::{DesignRepairSummary, GlobalRouteRepairSummary, Report, RoutingSummary};
use crate::{ui, visualization};
use anyhow::Context;
use pare_common::config::Config;
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::RouteSource;
use pare_common::db::indices::{CellId, NetId};
use pare_common::db::writer;
use pare_common::memory;
use pare_placer::legalize::site::SiteGrid;
use pare_placer::zones::Zones;
use pare_resizer::GlobalRouting;
use pare_resizer::{Repairer, Resizer};
use pare_router::{GlobalSession, Progress, Stage};
use std::path::{Path, PathBuf};
use std::time::Duration;

/// Pixels along the die's longer side in the routed design's image.
const ROUTED_IMAGE_PIXELS: u32 = 2000;

/// Where a run saves its state and when, and the state it resumes from.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct StateFiles {
    pub(crate) save: Option<(PathBuf, pare_router::SavePoint)>,
    pub(crate) resume: Option<PathBuf>,
    pub(crate) trace: pare_router::TraceRequest,
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
    let Some(route_rc) = &plan.on_global_routes else {
        return Ok(None);
    };
    let Some(timed) = repair.timed() else {
        return Ok(None);
    };
    ui::section("Global Routing");
    let timer = PhaseTimer::start(Phase::GlobalRouting);
    let mut tables = RouteTables::default();
    let mut session = GlobalSession::new(db, &config.routing, &mut |p| tables.show(p))?;
    timer.finish(report);
    propagate_clocks(timed.sdc);
    let sites = SiteGrid::new(db).context("cannot lay out the core's site grid to legalize on")?;
    let legalize = |db: &mut NetlistDB, cell: CellId| sites.legalize(db, cell);
    let mut summary = GlobalRouteRepairSummary::default();
    for step in [RepairStep::Design, RepairStep::Timing] {
        let enabled = match step {
            RepairStep::Design => config.netlist.repair_design,
            RepairStep::Timing => config.netlist.repair_timing,
        };
        if !enabled {
            continue;
        }
        let timer = PhaseTimer::start(step.phase());
        let stage = timed.stage(db)?;
        let routes = GlobalRouting {
            source: &mut session,
            rc: route_rc.clone(),
            legalize: &legalize,
        };
        let inputs = stage.session(db, plan.wires, timed.namer);
        let mut resizer = Resizer::on_global_routes(inputs, timed.sdc, routes)?;
        match step {
            RepairStep::Design => {
                let done = Repairer::new(&mut resizer).repair_design()?;
                show_design_repair(&done, resizer.netlist().db.cells.len());
                summary.design = DesignRepairSummary::of(&done);
            }
            RepairStep::Timing => {
                summary.timing = Some(timing_repair_on(&mut resizer, timed.liberty)?);
            }
        }
        let renumbering = resizer.finish()?;
        session.renumber(&renumbering);
        timer.finish(report);
        legalize_and_reroute(db, &mut session, report)?;
    }
    report.global_route_repair = Some(summary);
    Ok(Some(session))
}

/// The repairs the global route stage makes, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RepairStep {
    Design,
    Timing,
}

impl RepairStep {
    const fn phase(self) -> Phase {
        match self {
            Self::Design => Phase::DesignRepairOnGlobalRoutes,
            Self::Timing => Phase::TimingRepairOnGlobalRoutes,
        }
    }
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
    ui::stat_row(
        "Cells moved:",
        &moved.len().to_string(),
        "Nets updated:",
        &moved_nets.len().to_string(),
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
/// routing, writing the global routes as guides to `guides` when given.
pub(crate) fn route_global(
    config: &Config,
    db: &NetlistDB,
    guides: Option<&Path>,
    report: &mut Report,
) -> anyhow::Result<()> {
    ui::section("Routing");
    let timer = PhaseTimer::start(Phase::GlobalRouting);
    let mut tables = RouteTables::default();
    let global = pare_router::route_global_only(db, &config.routing, &mut |p| tables.show(p))?;
    ui::stat_row("Nets:", &global.summary.nets.to_string(), "", "");
    if let Some(path) = guides {
        write_guides(path, db, &global.guides)?;
        ui::wrote(path);
    }
    timer.finish(report);
    Ok(())
}

/// Routes every signal net in detail, from the global routes of `global`
/// when given, saving and resuming state as `files` say, then checks the
/// result and writes it to the output directory.
pub(crate) fn route_detailed(
    config: &Config,
    db: &mut NetlistDB,
    files: &StateFiles,
    global: Option<GlobalSession>,
    report: &mut Report,
) -> anyhow::Result<()> {
    ui::section("Routing");
    let timer = PhaseTimer::start(Phase::Routing);
    let summary = run_router(config, db, files, global)?;
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

/// Runs the detailed router over `db`, saving and resuming its state as
/// `files` say.
fn run_router(
    config: &Config,
    db: &mut NetlistDB,
    files: &StateFiles,
    global: Option<GlobalSession>,
) -> anyhow::Result<pare_router::RoutingSummary> {
    let resume = files.resume.as_deref().map(read_state).transpose()?;
    let mut write = |state: &pare_router::RouteState| {
        let Some((path, _)) = &files.save else {
            return Ok(());
        };
        write_state(path, state).map_err(|e| pare_router::SaveError(format!("{e:#}")))?;
        ui::wrote(path);
        Ok(())
    };
    let mut tables = RouteTables::default();
    let mut show = |p: &Progress<'_>| tables.show(p);
    let run = pare_router::RunOptions {
        resume: resume.as_ref(),
        save: files.save.as_ref().map(|&(_, after)| pare_router::Save {
            after,
            write: &mut write,
        }),
        trace: files.trace.clone(),
        report: Some(&mut show),
    };
    let summary = match global {
        Some(global) => pare_router::route_on(db, &config.routing, global, run)?,
        None => pare_router::route_with(db, &config.routing, run)?,
    };
    if let Some((path, after)) = &files.save
        && !summary.state_saved
    {
        log::warn!(
            "routing ended before {after:?}, so no state was saved to {}",
            path.display()
        );
    }
    Ok(summary)
}

/// Shows what detailed routing left: nets, rounds and unfinished nets.
fn show_routing(summary: &pare_router::RoutingSummary) {
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
                ui::stat_row(
                    "GCells:",
                    &format!("{nx}x{ny}"),
                    "GR overflow:",
                    &overflow.to_string(),
                );
                ui::gr_layer_table_header();
                for layer in *layers {
                    ui::gr_layer_row(
                        layer.name,
                        layer.direction,
                        layer.capacity,
                        layer.usage,
                        layer.overflow,
                    );
                }
            }
            Progress::GlobalSettled { kept, rerouted } => ui::stat_row(
                "Routes kept:",
                &kept.to_string(),
                "Routed again:",
                &rerouted.to_string(),
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
    ui::stat_row(label, &ui::seconds(elapsed), "Resident:", &resident);
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
mod tests {
    use super::*;
    use crate::placement::{PlacementStart, run_placement};
    use crate::plan::FlowPlan;
    use crate::testing::{half_height_tiny, scratch_dir, timed, timed_config, tiny};
    use pare_common::geom::dbu::Dbu;
    use pare_common::geom::point::Point;
    use pare_common::geom::rect::Rect;
    use pare_router::{NetGuide, SavePoint};

    #[test]
    fn a_save_point_is_global_or_a_round_number() {
        assert_eq!(parse_save_point("global"), Ok(SavePoint::Global));
        assert_eq!(parse_save_point("0"), Ok(SavePoint::Round(0)));
        assert_eq!(parse_save_point("12"), Ok(SavePoint::Round(12)));
    }

    #[test]
    fn a_save_point_that_is_neither_names_what_it_takes() {
        let error = parse_save_point("first").unwrap_err();

        assert!(
            error.contains("first") && error.contains("global"),
            "{error}"
        );
    }

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    #[test]
    fn guides_are_written_net_by_net_in_the_ispd_format() {
        let db = tiny();
        let layer = |name: &str| db.library.routing_layer_id(name).unwrap();
        let guides = [
            NetGuide {
                net: db.net_id("n3").unwrap(),
                boxes: vec![
                    (layer("M1"), rect(2000, 0, 4000, 2000)),
                    (layer("M3"), rect(2000, 2000, 6000, 4000)),
                ],
            },
            NetGuide {
                net: db.net_id("in").unwrap(),
                boxes: vec![(layer("M2"), rect(0, 0, 2000, 6000))],
            },
        ];
        let path = scratch_dir("guides").join("tiny.guide");

        write_guides(&path, &db, &guides).unwrap();

        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            "n3\n(\n2000 0 4000 2000 M1\n2000 2000 6000 4000 M3\n)\n\
             in\n(\n0 0 2000 6000 M2\n)\n"
        );
    }

    #[test]
    fn no_guides_write_an_empty_file() {
        let path = scratch_dir("no-guides").join("tiny.guide");

        write_guides(&path, &tiny(), &[]).unwrap();

        assert_eq!(std::fs::read_to_string(&path).unwrap(), "");
    }

    #[test]
    fn a_routed_image_keeps_the_die_in_proportion() {
        let dir = scratch_dir("routed-image");

        write_routed(&half_height_tiny(), &dir).unwrap();

        assert_eq!(
            image::image_dimensions(dir.join("routed.png")).unwrap(),
            (ROUTED_IMAGE_PIXELS, ROUTED_IMAGE_PIXELS / 2)
        );
    }

    #[test]
    fn repair_on_global_routes_propagates_the_clocks_without_clock_trees() {
        let config = timed_config("clock_tree = false", &scratch_dir("propagated"));
        let mut db = timed();
        let plan = FlowPlan::resolve(&config, &db).unwrap();
        let mut report = Report::default();
        let mut repair = run_placement(
            &config,
            &plan,
            &mut db,
            PlacementStart::Scratch,
            &mut report,
        )
        .unwrap()
        .unwrap();

        let _ = global_route_with_repair(&config, &mut db, &mut repair, &mut report).unwrap();

        let timed = repair.timed().unwrap();
        assert!(timed.sdc.clocks.iter().all(|clock| clock.propagated));
    }
}
