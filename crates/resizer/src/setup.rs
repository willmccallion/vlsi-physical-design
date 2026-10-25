//! Setup repair as `OpenROAD`'s `repair_timing` does it.
//!
//! The failing endpoints are taken from the worst up, each with passes of
//! moves along its worst path, every pass kept only when the worst slack
//! improves, and a last gasp of sizing over whatever still fails.

use crate::fuzzy;
use crate::moves::{MoveKind, Moves};
use crate::resizer::{RepairError, Resizer};
use crate::sta::{Path, Via};
use pare_common::db::design::PinOwner;
use pare_common::db::indices::PinId;
use pare_common::db::liberty::MinMax;

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

/// Iterations between progress reports.
const PRINT_INTERVAL: usize = 10;

/// How setup repair runs.
#[derive(Clone, Debug, PartialEq)]
pub struct SetupOptions {
    /// Slack every endpoint should reach, in seconds.
    pub margin: f64,
    /// Share of the failing endpoints to work on, from the worst.
    pub repair_tns_share: f64,
    /// Passes an endpoint may take.
    pub max_passes: usize,
    /// Moves a pass may make on the worst path.
    pub max_repairs_per_pass: usize,
    /// The moves tried on each driver, in order.
    pub sequence: Vec<MoveKind>,
    /// Whether the last gasp of sizing is skipped.
    pub skip_last_gasp: bool,
}

impl Default for SetupOptions {
    /// `OpenROAD`'s defaults with `OpenROAD-flow-scripts`' full-TNS
    /// repair and move sequence.
    fn default() -> Self {
        Self {
            margin: 0.0,
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
    /// Worst slack, in seconds.
    pub wns: f64,
    /// Total negative slack, in seconds.
    pub tns: f64,
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
    /// Worst slack at the start, in seconds.
    pub wns_before: f64,
    /// Worst slack at the end.
    pub wns_after: f64,
    /// Total negative slack at the start.
    pub tns_before: f64,
    /// Total negative slack at the end.
    pub tns_after: f64,
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

/// Why setup repair could not run.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum SetupError {
    /// The session has no timing analysis to repair against.
    #[error("setup repair needs timing constraints")]
    NoTiming,
    /// An edit failed.
    #[error(transparent)]
    Repair(#[from] RepairError),
}

/// The repair in progress.
struct Repair<'r, 'a> {
    resizer: &'r mut Resizer<'a>,
    moves: Moves,
    options: SetupOptions,
    initial_area: f64,
    iteration: usize,
    min_violation: f64,
    max_violation: f64,
    fallback: bool,
}

/// Repairs the setup violations of the session's design, reporting
/// progress through `report`.
pub fn repair_setup(
    resizer: &mut Resizer<'_>,
    options: SetupOptions,
    report: &mut dyn FnMut(&SetupProgress),
) -> Result<SetupSummary, SetupError> {
    if resizer.sta().is_none() {
        return Err(SetupError::NoTiming);
    }
    let moves = Moves::new(resizer)?;
    let initial_area = resizer.design_area();
    let mut repair = Repair {
        resizer,
        moves,
        options,
        initial_area,
        iteration: 0,
        min_violation: 0.0,
        max_violation: 0.0,
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
    let max_end_repairs =
        ((violating.len() as f64 * repair.options.repair_tns_share) as usize).max(1);
    let mut prev_tns = tns_before;
    repair.legacy_phase(
        &violating,
        max_end_repairs,
        tns_before,
        &mut prev_tns,
        report,
    )?;
    if !repair.options.skip_last_gasp {
        repair.last_gasp(tns_before, &mut prev_tns, report)?;
    }
    let progress = repair.progress(None);
    report(&progress);
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
    fn sta(&self) -> &crate::sta::Sta {
        // Checked once at entry; the analysis stays for the session.
        self.resizer
            .sta()
            .unwrap_or_else(|| unreachable!("setup repair runs with timing"))
    }

    /// The endpoints below the margin, worst first, as
    /// `ViolatorCollector::collectViolatingEndpoints`.
    fn violating_endpoints(&self) -> Vec<(PinId, f64)> {
        let sta = self.sta();
        let mut ends: Vec<(PinId, f64)> = sta
            .endpoints()
            .iter()
            .map(|&end| (end, sta.slack(end, MinMax::Max)))
            .filter(|&(_, slack)| fuzzy::less(slack, self.options.margin))
            .collect();
        ends.sort_by(|a, b| a.1.total_cmp(&b.1));
        ends
    }

    fn worst(&self) -> (f64, Option<PinId>) {
        self.sta().worst_slack(MinMax::Max)
    }

    fn tns(&self) -> f64 {
        self.sta().total_negative_slack(MinMax::Max)
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

    /// Whether the repair has stopped paying off, as `terminateProgress`.
    fn terminate(&self, initial_tns: f64, prev_tns: &mut f64, threshold: &mut f64) -> bool {
        if self.iteration.is_multiple_of(LARGE_INTERVAL) {
            *threshold *= 2.0;
        }
        if self.iteration.is_multiple_of(SMALL_INTERVAL) {
            let current = self.tns();
            let rate = if initial_tns == 0.0 {
                0.0
            } else {
                (*prev_tns - current) / initial_tns
            };
            *prev_tns = current;
            if self.iteration > INITIAL_ITERATIONS && rate < *threshold {
                return true;
            }
        }
        false
    }

    /// `repairSetup_Legacy`.
    fn legacy_phase(
        &mut self,
        violating: &[(PinId, f64)],
        max_end_repairs: usize,
        initial_tns: f64,
        prev_tns: &mut f64,
        report: &mut dyn FnMut(&SetupProgress),
    ) -> Result<(), SetupError> {
        self.min_violation = -violating[violating.len() - 1].1;
        self.max_violation = -violating[0].1;
        let mut threshold = FIX_RATE_THRESHOLD;
        let mut prev_termination = false;
        let margin = self.options.margin;
        for (end_index, &(first_end, _)) in violating.iter().enumerate() {
            let end_index = end_index + 1;
            if end_index > max_end_repairs {
                break;
            }
            self.fallback = false;
            let mut end = first_end;
            let mut end_slack = self.sta().slack(end, MinMax::Max);
            let (mut worst_slack, _) = self.worst();
            let mut prev_end_slack = end_slack;
            let mut prev_worst_slack = worst_slack;
            let mut decreasing = 0;
            let mut two_terminations = false;
            self.journal_begin();
            let mut journal_open = true;
            let mut pass = 1;
            while pass <= self.options.max_passes {
                self.iteration += 1;
                if self.iteration.is_multiple_of(PRINT_INTERVAL) || self.iteration == 1 {
                    report(&self.progress(Some(self.iteration)));
                }
                if self.terminate(initial_tns, prev_tns, &mut threshold) {
                    if prev_termination {
                        two_terminations = true;
                    } else {
                        prev_termination = true;
                    }
                    self.journal_restore()?;
                    journal_open = false;
                    break;
                }
                if self.iteration.is_multiple_of(SMALL_INTERVAL) {
                    prev_termination = false;
                }
                if fuzzy::greater_equal(end_slack, margin) {
                    if pass == 1 {
                        self.journal_end();
                    } else {
                        self.journal_restore()?;
                    }
                    journal_open = false;
                    break;
                }
                let changed = self.repair_endpoint(end, end_slack, margin)?;
                if !changed {
                    if pass == 1 {
                        self.journal_end();
                    } else {
                        self.journal_restore()?;
                    }
                    journal_open = false;
                    break;
                }
                end_slack = self.sta().slack(end, MinMax::Max);
                let (new_worst, worst_vertex) = self.worst();
                worst_slack = new_worst;
                let better = fuzzy::greater(worst_slack, prev_worst_slack)
                    || (end_index != 1
                        && fuzzy::equal(worst_slack, prev_worst_slack)
                        && fuzzy::greater(end_slack, prev_end_slack));
                if better {
                    prev_end_slack = end_slack;
                    prev_worst_slack = worst_slack;
                    decreasing = 0;
                    self.journal_end();
                    if pass < self.options.max_passes {
                        self.journal_begin();
                    } else {
                        journal_open = false;
                    }
                } else {
                    self.fallback = true;
                    decreasing += 1;
                    if decreasing > DECREASING_SLACK_MAX_PASSES {
                        self.journal_restore()?;
                        journal_open = false;
                        break;
                    }
                }
                if end_index == 1
                    && let Some(worst) = worst_vertex
                {
                    end = worst;
                }
                pass += 1;
            }
            if journal_open {
                self.journal_end();
            }
            if two_terminations {
                break;
            }
        }
        Ok(())
    }

    /// `repairSetup_LastGasp`: sizing only, kept only when both worst and
    /// total slack improve.
    fn last_gasp(
        &mut self,
        initial_tns: f64,
        prev_tns: &mut f64,
        report: &mut dyn FnMut(&SetupProgress),
    ) -> Result<(), SetupError> {
        let sequence = std::mem::replace(
            &mut self.options.sequence,
            vec![MoveKind::SizeUpMatch, MoveKind::SizeUp, MoveKind::SwapPins],
        );
        let margin = self.options.margin;
        let violating = self.violating_endpoints();
        let mut current_tns = self.tns();
        if fuzzy::greater_equal(current_tns, 0.0) || violating.is_empty() {
            self.options.sequence = sequence;
            return Ok(());
        }
        let mut threshold = FIX_RATE_THRESHOLD;
        let mut prev_termination = false;
        let (mut prev_worst, _) = self.worst();
        let mut local_prev_tns = current_tns;
        for (end_index, &(first_end, _)) in violating.iter().enumerate() {
            let end_index = end_index + 1;
            self.fallback = false;
            let mut end = first_end;
            let mut end_slack = self.sta().slack(end, MinMax::Max);
            let mut two_terminations = false;
            self.journal_begin();
            let mut journal_open = true;
            let mut pass = 1;
            while pass <= LAST_GASP_PASSES {
                self.iteration += 1;
                if self.terminate(initial_tns, prev_tns, &mut threshold) {
                    if prev_termination {
                        two_terminations = true;
                    } else {
                        prev_termination = true;
                    }
                    self.journal_end();
                    journal_open = false;
                    break;
                }
                if self.iteration.is_multiple_of(SMALL_INTERVAL) {
                    prev_termination = false;
                }
                if self.iteration.is_multiple_of(PRINT_INTERVAL) || self.iteration == 1 {
                    report(&self.progress(Some(self.iteration)));
                }
                if fuzzy::greater_equal(end_slack, margin) {
                    self.journal_end();
                    journal_open = false;
                    break;
                }
                let changed = self.repair_endpoint(end, end_slack, margin)?;
                if !changed {
                    if pass == 1 {
                        self.journal_end();
                    } else {
                        self.journal_restore()?;
                    }
                    journal_open = false;
                    break;
                }
                end_slack = self.sta().slack(end, MinMax::Max);
                let (current_worst, worst_vertex) = self.worst();
                current_tns = self.tns();
                let wns_improved = fuzzy::greater_equal(current_worst, prev_worst);
                let tns_improved = fuzzy::greater_equal(current_tns, local_prev_tns);
                if wns_improved && tns_improved {
                    prev_worst = current_worst;
                    local_prev_tns = current_tns;
                    self.journal_end();
                    if pass < LAST_GASP_PASSES {
                        self.journal_begin();
                    } else {
                        journal_open = false;
                    }
                } else {
                    self.fallback = true;
                    self.journal_restore()?;
                    journal_open = false;
                    break;
                }
                if end_index == 1
                    && let Some(worst) = worst_vertex
                {
                    end = worst;
                }
                pass += 1;
            }
            if journal_open {
                self.journal_end();
            }
            if two_terminations {
                break;
            }
        }
        self.options.sequence = sequence;
        Ok(())
    }

    /// One pass over the worst path into `end`: the drivers with the most
    /// load delay first, each tried with the move sequence until enough
    /// moves land, as `RepairSetup::repairEndpoint`.
    fn repair_endpoint(
        &mut self,
        end: PinId,
        path_slack: f64,
        margin: f64,
    ) -> Result<bool, RepairError> {
        let path = {
            let netlist = self.resizer.netlist();
            self.sta().worst_slack_path(&netlist, end, MinMax::Max)
        };
        let Some(path) = path else {
            return Ok(false);
        };
        if path.points.len() <= 1 {
            return Ok(false);
        }
        let mut load_delays: Vec<(usize, f64)> = Vec::new();
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
        if self.max_violation - self.min_violation != 0.0 && self.options.max_repairs_per_pass > 1 {
            let share =
                (-path_slack - self.min_violation) / (self.max_violation - self.min_violation);
            repairs_per_pass +=
                ((self.options.max_repairs_per_pass - 1) as f64 * share).round() as usize;
        }
        if self.fallback {
            repairs_per_pass = 1;
        }
        let mut changed = 0;
        let sequence = self.options.sequence.clone();
        for (index, _) in load_delays {
            if changed >= repairs_per_pass {
                break;
            }
            for &kind in &sequence {
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
    fn load_delay(&self, path: &Path, index: usize) -> Option<f64> {
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
            .map(|a| a.gate_delay(0.0, 0.0).0)
            .fold(0.0, f64::max);
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
