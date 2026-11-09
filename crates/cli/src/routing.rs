//! Global routing with repair on its parasitics, and detailed routing.

use crate::checks::report_routing;
use crate::placement::legalize;
use crate::prepare_output_dir;
use crate::repair::{NetlistRepair, show_design_repair, timing_repair_on};
use crate::report::{GlobalRouteRepairSummary, Report, RoutingSummary};
use anyhow::Context;
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::RouteSource;
use pare_common::db::indices::{CellId, NetId};
use pare_common::db::writer;
use pare_common::units::Ohms;
use pare_common::util::config::Config;
use pare_common::util::{memory, ui, visualization};
use pare_placer::legalize::site::SiteGrid;
use pare_placer::zones::Zones;
use pare_resizer::parasitics::WireRc;
use pare_resizer::resizer::GlobalRouting;
use pare_resizer::route_parasitics::RouteRc;
use pare_resizer::{Repairer, Resizer};
use pare_router::{GlobalSession, Progress, Stage};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// Pixels across each side of the routed design's image.
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
/// After `estimate_parasitics -global_routing` come `repair_design` and
/// detailed placement, then `repair_timing` and detailed placement again;
/// the nets of every cell an edit or detailed placement moves are routed
/// again. None when the run has no constraints or the repair is off.
pub(crate) fn global_route_with_repair(
    config: &Config,
    db: &mut NetlistDB,
    repair: &mut NetlistRepair,
    report: &mut Report,
) -> anyhow::Result<Option<GlobalSession>> {
    if !config.netlist.repair_on_global_routes {
        return Ok(None);
    }
    let Some(timed) = repair.timed() else {
        return Ok(None);
    };
    ui::section("Global Routing");
    let phase_start = Instant::now();
    ui::phase("Global Route");
    let mut tables = RouteTables::default();
    let mut session = GlobalSession::new(db, &config.routing, &mut |p| tables.show(p))?;
    report.phase("global routing", phase_start);
    let route_rc = route_rc(config, db)?;
    let sites = SiteGrid::new(db).context("the design has no core rows to legalize on")?;
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
        let phase_start = Instant::now();
        let stage = timed.stage(config, db)?;
        let routes = GlobalRouting {
            source: &mut session,
            rc: route_rc.clone(),
            legalize: &legalize,
        };
        let inputs = stage.session(db, timed.wires, timed.namer);
        let mut resizer = Resizer::on_global_routes(inputs, timed.sdc, routes)?;
        match step {
            RepairStep::Design => {
                ui::phase("Design Repair");
                let done = Repairer::new(&mut resizer).repair_design()?;
                show_design_repair(&done, resizer.netlist().db.cells.len());
                summary.slew_violations = done.slew_violations;
                summary.cap_violations = done.cap_violations;
                summary.repaired_nets = done.repaired_nets;
                summary.resized = done.resized;
                summary.repeaters = done.inserted;
            }
            RepairStep::Timing => {
                ui::phase("Timing Repair");
                summary.timing = Some(timing_repair_on(&mut resizer, timed.liberty)?);
            }
        }
        let renumbering = resizer.finish()?;
        session.renumber(&renumbering);
        report.phase(step.phase_name(), phase_start);
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
    const fn phase_name(self) -> &'static str {
        match self {
            Self::Design => "design repair on global routes",
            Self::Timing => "timing repair on global routes",
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
    let phase_start = Instant::now();
    ui::phase("Legalization");
    let before: Vec<_> = db.cells.iter().map(|c| (c.location, c.orient)).collect();
    let _ = legalize(db, &Zones::new(db)?)?;
    let mut moved_nets: Vec<NetId> = db
        .cells
        .iter()
        .zip(&before)
        .filter(|(c, was)| (c.location, c.orient) != **was)
        .flat_map(|(c, _)| c.pins.iter().map(|&p| db.pins[p.index()].net))
        .collect();
    moved_nets.sort_unstable();
    moved_nets.dedup();
    let moved = db
        .cells
        .iter()
        .zip(&before)
        .filter(|(c, was)| (c.location, c.orient) != **was)
        .count();
    session.update(db, &moved_nets);
    ui::stat_row(
        "Cells moved:",
        &moved.to_string(),
        "Nets updated:",
        &moved_nets.len().to_string(),
    );
    report.phase("legalization on global routes", phase_start);
    Ok(())
}

/// Each routing layer's and cut layer's resistance and capacitance from
/// the netlist config, for the global routes' parasitics.
fn route_rc(config: &Config, db: &NetlistDB) -> anyhow::Result<RouteRc> {
    let lib = &db.library;
    let layers = config
        .netlist
        .layer_rc
        .iter()
        .map(|l| {
            let id = lib
                .routing_layer_id(&l.layer)
                .with_context(|| format!("[netlist] layer_rc names unknown layer '{}'", l.layer))?;
            let rc = WireRc::per_micron(l.resistance_ohm_per_um, l.capacitance_ff_per_um, id);
            Ok((id, rc.resistance, rc.capacitance))
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    let vias = config
        .netlist
        .via_rc
        .iter()
        .map(|v| {
            let cut = lib
                .cut_layers
                .iter()
                .position(|c| c.name == v.layer)
                .with_context(|| {
                    format!("[netlist] via_rc names unknown cut layer '{}'", v.layer)
                })?;
            Ok((cut, Ohms::new(v.resistance_ohm)))
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
    let min_layer = *config.routing.layer_range(lib)?.start();
    Ok(RouteRc::new(lib, &layers, &vias, min_layer))
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
        writeln!(out, "{}\n(", db.nets[guide.net.index()].name)?;
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
    let phase_start = Instant::now();
    ui::section("Routing");
    let mut tables = RouteTables::default();
    let global = pare_router::route_global_only(db, &config.routing, &mut |p| tables.show(p))?;
    ui::stat_row("Nets:", &global.summary.nets.to_string(), "", "");
    if let Some(path) = guides {
        write_guides(path, db, &global.guides)?;
        ui::wrote(path);
    }
    report.phase("global routing", phase_start);
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
    let phase_start = Instant::now();
    ui::section("Routing");
    let summary = run_router(config, db, files, global)?;
    show_routing(&summary);
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
    write_routed(db, &prepare_output_dir(config)?)?;
    report.phase("routing output", phase_start);

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
                ui::dr_iter(
                    round.round,
                    round.markers,
                    round.conflicted,
                    round.repaired,
                    round.conflict_cost,
                    round.flow,
                    round.elapsed.as_millis(),
                );
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
    ui::stat_row(
        label,
        &format!("{:.1}s", elapsed.as_secs_f64()),
        "Resident:",
        &resident,
    );
}

/// Writes the routed design's image and DEF into `output_dir`.
fn write_routed(db: &NetlistDB, output_dir: &Path) -> anyhow::Result<()> {
    let png = output_dir.join("routed.png");
    visualization::draw_routed_design(db, &png, ROUTED_IMAGE_PIXELS, ROUTED_IMAGE_PIXELS)
        .with_context(|| format!("cannot write {}", png.display()))?;
    ui::wrote(&png);
    let out = output_dir.join("routed.def");
    writer::def::write_file(db, &out).with_context(|| format!("cannot write {}", out.display()))?;
    ui::wrote(&out);
    Ok(())
}
