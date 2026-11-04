//! Setup repair as `OpenROAD`'s `repair_timing` does it.
//!
//! The failing endpoints are taken from the worst up, each with passes of
//! moves along its worst path, every pass kept only when the worst slack
//! improves, and a last gasp of sizing over whatever still fails.

use crate::fuzzy;
use crate::moves::{MoveKind, Moves};
use crate::resizer::{RepairError, Resizer};
use crate::sta::{Path, Sta, Via};
use pare_common::db::design::PinOwner;
use pare_common::db::indices::PinId;
use pare_common::db::liberty::MinMax;
use pare_common::units::{Farads, Seconds};

/// Passes an endpoint may go on failing to improve before its best state
/// is restored.
const DECREASING_SLACK_MAX_PASSES: usize = 50;

/// Iterations between checks of the rate of repair.
const SMALL_INTERVAL: usize = 100;

/// Iterations between doublings of the rate the repair must keep.
const LARGE_INTERVAL: usize = 1000;

/// Iterations the repair runs before its rate is judged.
const INITIAL_ITERATIONS: usize = 1000;

/// The share of the initial violation each hundred iterations must fix.
const FIX_RATE_THRESHOLD: f64 = 0.0001;

/// Passes per endpoint in the last gasp.
const LAST_GASP_PASSES: usize = 10;

/// The moves the last gasp tries: sizing only.
const LAST_GASP_SEQUENCE: [MoveKind; 3] =
    [MoveKind::SizeUpMatch, MoveKind::SizeUp, MoveKind::SwapPins];

/// Iterations between progress reports.
const PRINT_INTERVAL: usize = 10;

/// How setup repair runs.
#[derive(Clone, Debug, PartialEq)]
pub struct SetupOptions {
    /// Slack every endpoint should reach.
    pub margin: Seconds,
    /// Share of the failing endpoints to work on, from the worst.
    pub repair_tns_share: f64,
    /// Passes an endpoint may take.
    pub max_passes: usize,
    /// Moves a pass may make on the worst path.
    pub max_repairs_per_pass: usize,
    /// The moves tried on each driver, in order.
    pub(crate) sequence: Vec<MoveKind>,
    /// Whether the last gasp of sizing is skipped.
    pub skip_last_gasp: bool,
}

impl Default for SetupOptions {
    /// `OpenROAD`'s defaults with `OpenROAD-flow-scripts`' full-TNS
    /// repair and move sequence.
    fn default() -> Self {
        Self {
            margin: Seconds::ZERO,
            repair_tns_share: 1.0,
            max_passes: 10_000,
            max_repairs_per_pass: 1,
            sequence: vec![
                MoveKind::Unbuffer,
                MoveKind::SizeUp,
                MoveKind::SwapPins,
                MoveKind::Buffer,
                MoveKind::Clone,
                MoveKind::SplitLoad,
            ],
            skip_last_gasp: false,
        }
    }
}

/// What a report of progress says.
#[derive(Clone, Debug, PartialEq)]
pub struct SetupProgress {
    /// Iterations so far, or none for the final line.
    pub iteration: Option<usize>,
    /// Buffers removed.
    pub removed: usize,
    /// Gates resized.
    pub resized: usize,
    /// Buffers inserted.
    pub inserted: usize,
    /// Gates cloned.
    pub cloned: usize,
    /// Pins swapped.
    pub swapped: usize,
    /// Area now over area at the start, as a percentage change.
    pub area_change_percent: f64,
    /// Worst slack.
    pub wns: Seconds,
    /// Total negative slack.
    pub tns: Seconds,
    /// Endpoints still failing.
    pub failing: usize,
    /// The worst endpoint's name.
    pub worst: Option<String>,
}

/// What setup repair did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SetupSummary {
    /// Endpoints failing at the start.
    pub failing_before: usize,
    /// Endpoints failing at the end.
    pub failing_after: usize,
    /// Worst slack at the start.
    pub wns_before: Seconds,
    /// Worst slack at the end.
    pub wns_after: Seconds,
    /// Total negative slack at the start.
    pub tns_before: Seconds,
    /// Total negative slack at the end.
    pub tns_after: Seconds,
    /// Buffers removed.
    pub removed: usize,
    /// Gates resized.
    pub resized: usize,
    /// Buffers inserted.
    pub inserted: usize,
    /// Of them, to split loads.
    pub split: usize,
    /// Gates cloned.
    pub cloned: usize,
    /// Pins swapped.
    pub swapped: usize,
    /// Iterations run.
    pub iterations: usize,
    /// Area change, as a percentage.
    pub area_change_percent: f64,
}

/// The repair in progress.
struct Repair<'r, 'a> {
    resizer: &'r mut Resizer<'a, Sta>,
    moves: Moves,
    options: SetupOptions,
    report: &'r mut dyn FnMut(&SetupProgress),
    initial_area: f64,
    iteration: usize,
    min_violation: Seconds,
    max_violation: Seconds,
    fallback: bool,
}

/// What a phase of the repair runs with, and the rate of repair it is held
/// to, as `terminateProgress` tracks it.
struct Phase {
    /// The moves tried on each driver, in order.
    sequence: Vec<MoveKind>,
    /// Passes an endpoint may take.
    max_passes: usize,
    /// Total negative slack before the repair.
    initial_tns: Seconds,
    /// Total negative slack at the last check of the rate.
    prev_tns: Seconds,
    /// The share of the initial violation each check must have fixed.
    threshold: f64,
    /// Whether the last check of the rate ended an endpoint's passes.
    just_terminated: bool,
}

impl Phase {
    const fn new(
        sequence: Vec<MoveKind>,
        max_passes: usize,
        initial_tns: Seconds,
        tns: Seconds,
    ) -> Self {
        Self {
            sequence,
            max_passes,
            initial_tns,
            prev_tns: tns,
            threshold: FIX_RATE_THRESHOLD,
            just_terminated: false,
        }
    }

    /// Notes that the rate ended an endpoint's passes; whether the last
    /// check did too, which ends the phase.
    const fn terminated(&mut self) -> EndpointEnd {
        let twice = self.just_terminated;
        self.just_terminated = true;
        if twice {
            EndpointEnd::Stop
        } else {
            EndpointEnd::Next
        }
    }
}

/// What becomes of a phase once an endpoint's passes end.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EndpointEnd {
    /// It goes on to the next endpoint.
    Next,
    /// It stops: the repair has stopped paying off twice running.
    Stop,
}

/// Repairs the setup violations of the session's design, reporting
/// progress through `report`.
pub fn repair_setup(
    resizer: &mut Resizer<'_, Sta>,
    options: SetupOptions,
    report: &mut dyn FnMut(&SetupProgress),
) -> Result<SetupSummary, RepairError> {
    let moves = Moves::new(resizer)?;
    let initial_area = resizer.design_area();
    let mut repair = Repair {
        resizer,
        moves,
        options,
        report,
        initial_area,
        iteration: 0,
        min_violation: Seconds::ZERO,
        max_violation: Seconds::ZERO,
        fallback: false,
    };
    let violating = repair.violating_endpoints();
    let (wns_before, _) = repair.worst();
    let tns_before = repair.tns();
    let mut summary = SetupSummary {
        failing_before: violating.len(),
        wns_before,
        tns_before,
        ..SetupSummary::default()
    };
    if violating.is_empty() {
        summary.failing_after = 0;
        summary.wns_after = wns_before;
        summary.tns_after = tns_before;
        return Ok(summary);
    }
    repair.legacy_phase(&violating, tns_before)?;
    if !repair.options.skip_last_gasp {
        repair.last_gasp(tns_before)?;
    }
    let progress = repair.progress(None);
    (repair.report)(&progress);
    let log = &repair.moves.log;
    summary.removed = log.committed(MoveKind::Unbuffer);
    summary.resized = log.committed(MoveKind::SizeUp) + log.committed(MoveKind::SizeUpMatch);
    summary.inserted = log.committed(MoveKind::Buffer) + log.committed(MoveKind::SplitLoad);
    summary.split = log.committed(MoveKind::SplitLoad);
    summary.cloned = log.committed(MoveKind::Clone);
    summary.swapped = log.committed(MoveKind::SwapPins);
    summary.iterations = repair.iteration;
    summary.area_change_percent = progress.area_change_percent;
    summary.failing_after = progress.failing;
    summary.wns_after = progress.wns;
    summary.tns_after = progress.tns;
    Ok(summary)
}

impl Repair<'_, '_> {
    /// The endpoints below the margin, worst first, as
    /// `ViolatorCollector::collectViolatingEndpoints`.
    fn violating_endpoints(&self) -> Vec<(PinId, Seconds)> {
        let sta = self.resizer.sta();
        let mut ends: Vec<(PinId, Seconds)> = sta
            .endpoints()
            .iter()
            .map(|&end| (end, sta.slack(end, MinMax::Max).time()))
            .filter(|&(_, slack)| fuzzy::less(slack, self.options.margin))
            .collect();
        ends.sort_by(|a, b| a.1.total_cmp(&b.1));
        ends
    }

    fn worst(&self) -> (Seconds, Option<PinId>) {
        let (slack, end) = self.resizer.sta().worst_slack(MinMax::Max);
        (slack.time(), end)
    }

    fn tns(&self) -> Seconds {
        self.resizer.sta().total_negative_slack(MinMax::Max)
    }

    fn progress(&self, iteration: Option<usize>) -> SetupProgress {
        let (wns, worst) = self.worst();
        let area = self.resizer.design_area();
        let change = if self.initial_area > 0.0 {
            (area - self.initial_area) / self.initial_area * 100.0
        } else {
            0.0
        };
        let log = &self.moves.log;
        SetupProgress {
            iteration,
            removed: log.made(MoveKind::Unbuffer),
            resized: log.made(MoveKind::SizeUp) + log.made(MoveKind::SizeUpMatch),
            inserted: log.made(MoveKind::Buffer) + log.made(MoveKind::SplitLoad),
            cloned: log.made(MoveKind::Clone),
            swapped: log.made(MoveKind::SwapPins),
            area_change_percent: change,
            wns,
            tns: self.tns(),
            failing: self.violating_endpoints().len(),
            worst: worst.map(|pin| self.resizer.db.pin_name(pin)),
        }
    }

    fn journal_begin(&mut self) {
        self.resizer.journal_begin();
    }

    fn journal_end(&mut self) {
        self.resizer.journal_end();
        self.moves.log.commit();
    }

    fn journal_restore(&mut self) -> Result<(), RepairError> {
        self.resizer.journal_restore()?;
        self.moves.log.undo();
        Ok(())
    }

    /// Reports progress every [`PRINT_INTERVAL`] iterations and on the
    /// first.
    fn report_progress(&mut self) {
        if self.iteration.is_multiple_of(PRINT_INTERVAL) || self.iteration == 1 {
            let progress = self.progress(Some(self.iteration));
            (self.report)(&progress);
        }
    }

    fn end_slack(&self, end: PinId) -> Seconds {
        self.resizer.sta().slack(end, MinMax::Max).time()
    }

    /// Whether the repair has stopped paying off, as `terminateProgress`.
    fn terminate(&self, phase: &mut Phase) -> bool {
        if self.iteration.is_multiple_of(LARGE_INTERVAL) {
            phase.threshold *= 2.0;
        }
        if self.iteration.is_multiple_of(SMALL_INTERVAL) {
            let current = self.tns();
            let rate = if phase.initial_tns == Seconds::ZERO {
                0.0
            } else {
                (phase.prev_tns - current) / phase.initial_tns
            };
            phase.prev_tns = current;
            if self.iteration > INITIAL_ITERATIONS && rate < phase.threshold {
                return true;
            }
        }
        false
    }

    /// `repairSetup_Legacy`: passes over each failing endpoint from the
    /// worst, up to the share `repair_tns_share` of them.
    fn legacy_phase(
        &mut self,
        violating: &[(PinId, Seconds)],
        initial_tns: Seconds,
    ) -> Result<(), RepairError> {
        self.min_violation = -violating[violating.len() - 1].1;
        self.max_violation = -violating[0].1;
        let max_end_repairs =
            ((violating.len() as f64 * self.options.repair_tns_share) as usize).max(1);
        let mut phase = Phase::new(
            self.options.sequence.clone(),
            self.options.max_passes,
            initial_tns,
            initial_tns,
        );
        for (end_index, &(end, _)) in violating.iter().enumerate().take(max_end_repairs) {
            if self.legacy_endpoint(end_index + 1, end, &mut phase)? == EndpointEnd::Stop {
                break;
            }
        }
        Ok(())
    }

    /// The passes over the endpoint ranked `end_index` from the worst,
    /// each kept when the worst slack improves, or for any but the worst
    /// endpoint when its own slack does; the worst endpoint's passes follow
    /// the worst path wherever it goes.
    fn legacy_endpoint(
        &mut self,
        end_index: usize,
        first_end: PinId,
        phase: &mut Phase,
    ) -> Result<EndpointEnd, RepairError> {
        let margin = self.options.margin;
        self.fallback = false;
        let mut end = first_end;
        let mut end_slack = self.end_slack(end);
        let (mut prev_worst_slack, _) = self.worst();
        let mut prev_end_slack = end_slack;
        let mut decreasing = 0;
        self.journal_begin();
        for pass in 1..=phase.max_passes {
            self.iteration += 1;
            self.report_progress();
            if self.terminate(phase) {
                self.journal_restore()?;
                return Ok(phase.terminated());
            }
            if self.iteration.is_multiple_of(SMALL_INTERVAL) {
                phase.just_terminated = false;
            }
            if fuzzy::greater_equal(end_slack, margin)
                || !self.repair_endpoint(end, end_slack, &phase.sequence)?
            {
                self.end_passes(pass)?;
                return Ok(EndpointEnd::Next);
            }
            end_slack = self.end_slack(end);
            let (worst_slack, worst_vertex) = self.worst();
            let better = fuzzy::greater(worst_slack, prev_worst_slack)
                || (end_index != 1
                    && fuzzy::equal(worst_slack, prev_worst_slack)
                    && fuzzy::greater(end_slack, prev_end_slack));
            if better {
                prev_end_slack = end_slack;
                prev_worst_slack = worst_slack;
                decreasing = 0;
                self.journal_end();
                if pass == phase.max_passes {
                    return Ok(EndpointEnd::Next);
                }
                self.journal_begin();
            } else {
                self.fallback = true;
                decreasing += 1;
                if decreasing > DECREASING_SLACK_MAX_PASSES {
                    self.journal_restore()?;
                    return Ok(EndpointEnd::Next);
                }
            }
            if end_index == 1
                && let Some(worst) = worst_vertex
            {
                end = worst;
            }
        }
        self.journal_end();
        Ok(EndpointEnd::Next)
    }

    /// Closes the journal of passes that ended at `pass` with nothing more
    /// to try: kept after the first, undone after a later one, whose moves
    /// since the last improvement did not pay.
    fn end_passes(&mut self, pass: usize) -> Result<(), RepairError> {
        if pass == 1 {
            self.journal_end();
            Ok(())
        } else {
            self.journal_restore()
        }
    }

    /// `repairSetup_LastGasp`: sizing only, kept only when both worst and
    /// total slack improve. The rate of repair is judged from the last kept
    /// move's total, so the phase ends soon after moves stop landing often.
    fn last_gasp(&mut self, initial_tns: Seconds) -> Result<(), RepairError> {
        let violating = self.violating_endpoints();
        let current_tns = self.tns();
        if fuzzy::greater_equal(current_tns, Seconds::ZERO) || violating.is_empty() {
            return Ok(());
        }
        let mut phase = Phase::new(
            LAST_GASP_SEQUENCE.to_vec(),
            LAST_GASP_PASSES,
            initial_tns,
            current_tns,
        );
        let (mut prev_worst, _) = self.worst();
        for (end_index, &(end, _)) in violating.iter().enumerate() {
            let ended = self.last_gasp_endpoint(end_index + 1, end, &mut phase, &mut prev_worst)?;
            if ended == EndpointEnd::Stop {
                break;
            }
        }
        Ok(())
    }

    /// The last gasp's passes over the endpoint ranked `end_index` from the
    /// worst, each kept only when neither the worst slack `prev_worst` nor
    /// the total slack gets worse; the first that does ends them.
    fn last_gasp_endpoint(
        &mut self,
        end_index: usize,
        first_end: PinId,
        phase: &mut Phase,
        prev_worst: &mut Seconds,
    ) -> Result<EndpointEnd, RepairError> {
        let margin = self.options.margin;
        self.fallback = false;
        let mut end = first_end;
        let mut end_slack = self.end_slack(end);
        for pass in 1..=phase.max_passes {
            self.journal_begin();
            self.iteration += 1;
            if self.terminate(phase) {
                self.journal_end();
                return Ok(phase.terminated());
            }
            if self.iteration.is_multiple_of(SMALL_INTERVAL) {
                phase.just_terminated = false;
            }
            self.report_progress();
            if fuzzy::greater_equal(end_slack, margin) {
                self.journal_end();
                return Ok(EndpointEnd::Next);
            }
            if !self.repair_endpoint(end, end_slack, &phase.sequence)? {
                self.end_passes(pass)?;
                return Ok(EndpointEnd::Next);
            }
            end_slack = self.end_slack(end);
            let (current_worst, worst_vertex) = self.worst();
            let current_tns = self.tns();
            if !(fuzzy::greater_equal(current_worst, *prev_worst)
                && fuzzy::greater_equal(current_tns, phase.prev_tns))
            {
                self.fallback = true;
                self.journal_restore()?;
                return Ok(EndpointEnd::Next);
            }
            *prev_worst = current_worst;
            phase.prev_tns = current_tns;
            self.journal_end();
            if end_index == 1
                && let Some(worst) = worst_vertex
            {
                end = worst;
            }
        }
        Ok(EndpointEnd::Next)
    }

    /// One pass over the worst path into `end`: the drivers with the most
    /// load delay first, each tried with the move sequence until enough
    /// moves land, as `RepairSetup::repairEndpoint`.
    fn repair_endpoint(
        &mut self,
        end: PinId,
        path_slack: Seconds,
        sequence: &[MoveKind],
    ) -> Result<bool, RepairError> {
        let margin = self.options.margin;
        let path = {
            let netlist = self.resizer.netlist();
            self.resizer
                .sta()
                .worst_slack_path(&netlist, end, MinMax::Max)
        };
        let Some(path) = path else {
            return Ok(false);
        };
        if path.points.len() <= 1 {
            return Ok(false);
        }
        let mut load_delays: Vec<(usize, Seconds)> = Vec::new();
        for i in path.start_index..path.points.len() {
            if i == 0 {
                continue;
            }
            let point = &path.points[i];
            if !matches!(
                self.resizer.db.pins[point.pin.index()].owner,
                PinOwner::Cell { .. }
            ) {
                continue;
            }
            if !self.resizer.netlist().is_driver(point.pin) {
                continue;
            }
            let Some(load_delay) = self.load_delay(&path, i) else {
                continue;
            };
            load_delays.push((i, load_delay));
        }
        load_delays.sort_by(|a, b| b.1.total_cmp(&a.1).then_with(|| b.0.cmp(&a.0)));
        let mut repairs_per_pass = 1;
        if self.max_violation - self.min_violation != Seconds::ZERO
            && self.options.max_repairs_per_pass > 1
        {
            let share =
                (-path_slack - self.min_violation) / (self.max_violation - self.min_violation);
            repairs_per_pass +=
                ((self.options.max_repairs_per_pass - 1) as f64 * share).round() as usize;
        }
        if self.fallback {
            repairs_per_pass = 1;
        }
        let mut changed = 0;
        for (index, _) in load_delays {
            if changed >= repairs_per_pass {
                break;
            }
            for &kind in sequence {
                if self
                    .moves
                    .try_move(self.resizer, kind, &path, index, margin)?
                {
                    changed += if kind == MoveKind::Unbuffer {
                        repairs_per_pass
                    } else {
                        1
                    };
                    break;
                }
            }
        }
        Ok(changed > 0)
    }

    /// The delay the load adds to the arc into the driver at `index`: the
    /// arc's delay less its intrinsic delay, as `repairEndpoint` ranks
    /// drivers by.
    fn load_delay(&self, path: &Path, index: usize) -> Option<Seconds> {
        let point = &path.points[index];
        let prev = &path.points[index - 1];
        let Via::Arc { set } = point.via else {
            return None;
        };
        let (lib, _) = self.resizer.link.pin_port(self.resizer.db, point.pin)?;
        let cell = self.resizer.link.liberty.cell(lib);
        let arc_set = cell.arc_sets.get(set)?;
        let intrinsic = arc_set
            .arcs
            .iter()
            .filter(|a| a.from_edge == prev.edge && a.to_edge == point.edge)
            .map(|a| a.gate_delay(Seconds::ZERO, Farads::ZERO).0)
            .fold(Seconds::ZERO, Seconds::max);
        let delay = self
            .resizer
            .graph
            .arc_delays(point.pin)
            .iter()
            .find(|a| {
                a.arc.from == prev.pin
                    && a.arc.set == set
                    && a.from_edge == prev.edge
                    && a.to_edge == point.edge
            })
            .map(|a| a.delay[MinMax::Max.index()])?;
        Some(delay - intrinsic)
    }
}
