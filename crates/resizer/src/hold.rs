//! Hold repair, as `OpenROAD`'s `RepairHold`.
//!
//! The endpoints failing hold are taken from the worst up, and along each
//! one's worst early path a buffer goes in at each driver ahead of its
//! loads that fail hold, delaying them, as long as the loads' setup slack
//! can take the buffer's delay. An insertion is kept only when it leaves
//! the driver's slew and the design's worst setup slack about as they were.

use crate::fuzzy;
use crate::resizer::{RepairError, Resizer, buffer_delays};
use pare_common::db::indices::PinId;
use pare_common::db::liberty::{Edge, LibCellId, MinMax};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;

/// How close a hold buffer's delay per area may come to the best's for the
/// smaller of the two to be chosen.
const BUFFER_METRIC_MARGIN: f64 = 0.95;

/// How much a hold buffer may raise its driver's slew.
const MAX_SLEW_GROWTH: f64 = 1.20;

/// The share of a limit a hold buffer's slew and load must keep in hand
/// for it to stay at the smallest size.
const SLACK_LIMIT_RATIO: f64 = 0.2;

/// The least number of hold buffers the repair may insert, however small
/// the design.
const MIN_BUFFER_BUDGET: usize = 100;

/// How hold repair runs, as `repair_timing -hold` takes it.
#[derive(Clone, Debug, PartialEq)]
pub struct HoldOptions {
    /// Setup slack every endpoint should keep, in seconds.
    pub setup_margin: f64,
    /// Hold slack every endpoint should reach, in seconds.
    pub hold_margin: f64,
    /// Whether a hold fix may break setup.
    pub allow_setup_violations: bool,
    /// The most hold buffers, as a share of the instances.
    pub max_buffer_share: f64,
    /// The most endpoint repairs.
    pub max_passes: usize,
}

impl Default for HoldOptions {
    /// `OpenROAD`'s defaults.
    fn default() -> Self {
        Self {
            setup_margin: 0.0,
            hold_margin: 0.0,
            allow_setup_violations: false,
            max_buffer_share: 0.2,
            max_passes: 10_000,
        }
    }
}

/// What hold repair did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HoldSummary {
    /// Endpoints failing hold at the start.
    pub failing_before: usize,
    /// Endpoints failing hold at the end.
    pub failing_after: usize,
    /// Worst hold slack at the start, in seconds.
    pub wns_before: f64,
    /// Worst hold slack at the end.
    pub wns_after: f64,
    /// Worst setup slack at the start.
    pub setup_wns_before: f64,
    /// Worst setup slack at the end.
    pub setup_wns_after: f64,
    /// Hold buffers inserted.
    pub inserted: usize,
    /// Of them, resized to their load.
    pub resized: usize,
    /// The buffer cell used.
    pub buffer: Option<LibCellId>,
}

/// Repairs the hold violations of the session's design, as `OpenROAD`'s
/// `repair_timing -hold`. Needs timing.
pub fn repair_hold(
    resizer: &mut Resizer<'_>,
    options: &HoldOptions,
) -> Result<HoldSummary, RepairError> {
    let Some(sta) = resizer.sta() else {
        return Ok(HoldSummary::default());
    };
    let mut ends: Vec<PinId> = sta.endpoints().to_vec();
    ends.sort_unstable();
    let (setup_wns_before, _) = sta.worst_slack(MinMax::Max);
    let mut summary = HoldSummary {
        setup_wns_before,
        ..HoldSummary::default()
    };
    let (mut failures, mut worst) = hold_violations(resizer, &ends, options.hold_margin);
    summary.failing_before = failures.len();
    summary.wns_before = sta.worst_slack(MinMax::Min).0;
    let Some(buffer) = hold_buffer(resizer) else {
        summary.failing_after = failures.len();
        summary.wns_after = summary.wns_before;
        summary.setup_wns_after = setup_wns_before;
        return Ok(summary);
    };
    summary.buffer = Some(buffer);
    let cells = resizer.netlist().db.cells.len();
    let budget = ((options.max_buffer_share * cells as f64) as usize).max(MIN_BUFFER_BUDGET);
    let mut repair = HoldRepair {
        buffer,
        options,
        inserted: 0,
        resized: 0,
    };
    let mut pass = 1;
    let mut progress = true;
    while !failures.is_empty()
        && fuzzy::less(worst, options.hold_margin)
        && progress
        && repair.inserted <= budget
        && pass <= options.max_passes
    {
        let before = repair.inserted;
        let sta = resizer.sta().ok_or(RepairError::NoTiming)?;
        failures.sort_by(|&a, &b| {
            sta.slack(a, MinMax::Min)
                .total_cmp(&sta.slack(b, MinMax::Min))
                .then(a.cmp(&b))
        });
        for &end in &failures {
            repair.repair_end(resizer, end)?;
            pass += 1;
            if repair.inserted > budget || pass > options.max_passes {
                break;
            }
        }
        (failures, worst) = hold_violations(resizer, &ends, options.hold_margin);
        progress = repair.inserted > before;
    }
    let sta = resizer.sta().ok_or(RepairError::NoTiming)?;
    summary.failing_after = failures.len();
    summary.wns_after = sta.worst_slack(MinMax::Min).0;
    summary.setup_wns_after = sta.worst_slack(MinMax::Max).0;
    summary.inserted = repair.inserted;
    summary.resized = repair.resized;
    Ok(summary)
}

/// The endpoints whose hold slack is below `margin`, clock pins aside, and
/// the worst of their slacks, as `findHoldViolations`.
fn hold_violations(resizer: &Resizer<'_>, ends: &[PinId], margin: f64) -> (Vec<PinId>, f64) {
    let Some(sta) = resizer.sta() else {
        return (Vec::new(), f64::INFINITY);
    };
    let mut worst = f64::INFINITY;
    let mut failing = Vec::new();
    for &end in ends {
        let slack = sta.slack(end, MinMax::Min);
        if !resizer.clocks.is_clock(end) && slack < margin {
            worst = worst.min(slack);
            failing.push(end);
        }
    }
    (failing, worst)
}

/// The buffer with the most hold delay per area, or a smaller one within
/// [`BUFFER_METRIC_MARGIN`] of it, as `findHoldBuffer`; the library here
/// has no delay cells, sites or threshold voltages to prefer among.
fn hold_buffer(resizer: &Resizer<'_>) -> Option<LibCellId> {
    let liberty = resizer.link.liberty;
    let mut scored: Vec<(f64, LibCellId)> = resizer
        .buffers
        .cells
        .iter()
        .filter_map(|&cell| {
            let lib = liberty.cell(cell);
            (lib.area != 0.0).then(|| (hold_delay(resizer, cell) / lib.area, cell))
        })
        .collect();
    scored.sort_by(|a, b| a.0.total_cmp(&b.0));
    let &(best_metric, mut best) = scored.last()?;
    for &(metric, cell) in scored.iter().rev().skip(1) {
        if metric >= BUFFER_METRIC_MARGIN * best_metric
            && liberty.cell(cell).area < liberty.cell(best).area
        {
            best = cell;
        }
    }
    Some(best)
}

/// A buffer's delay driving its own input, the lesser of its edges, as
/// `bufferHoldDelay`.
fn hold_delay(resizer: &Resizer<'_>, cell: LibCellId) -> f64 {
    let lib = resizer.link.liberty.cell(cell);
    let Some((input, _)) = lib.buffer_ports() else {
        return 0.0;
    };
    let delays = buffer_delays(lib, &resizer.targets.slews, lib.ports[input].capacitance());
    delays.rise.min(delays.fall)
}

/// Slacks per edge and side, merged over loads as `RepairHold` merges
/// them: the least early slack and the greatest late one.
#[derive(Clone, Copy, Debug)]
struct Slacks {
    early: [f64; 2],
    late: [f64; 2],
}

impl Slacks {
    const fn new() -> Self {
        Self {
            early: [f64::INFINITY; 2],
            late: [f64::NEG_INFINITY; 2],
        }
    }

    fn merge(&mut self, resizer: &Resizer<'_>, pin: PinId) {
        let Some(sta) = resizer.sta() else {
            return;
        };
        for edge in Edge::BOTH {
            let i = edge.index();
            self.early[i] = self.early[i].min(sta.slack_edge(pin, edge, MinMax::Min));
            self.late[i] = self.late[i].max(sta.slack_edge(pin, edge, MinMax::Max));
        }
    }
}

/// The repair in progress.
struct HoldRepair<'o> {
    buffer: LibCellId,
    options: &'o HoldOptions,
    inserted: usize,
    resized: usize,
}

impl HoldRepair<'_> {
    /// Buffers the loads failing hold of each driver on the worst early
    /// path into `end`, as `repairEndHold`.
    fn repair_end(&mut self, resizer: &mut Resizer<'_>, end: PinId) -> Result<(), RepairError> {
        let path = {
            let netlist = resizer.netlist();
            let sta = resizer.sta().ok_or(RepairError::NoTiming)?;
            sta.worst_slack_path(&netlist, end, MinMax::Min)
        };
        let Some(path) = path else {
            return Ok(());
        };
        let pins: Vec<PinId> = path.points[path.start_index..]
            .iter()
            .map(|p| p.pin)
            .collect();
        if pins.len() <= 1 {
            return Ok(());
        }
        for i in 0..pins.len() - 1 {
            let driver = pins[i];
            if !resizer.netlist().is_driver(driver) || !resizer.ok_to_buffer_net(driver) {
                continue;
            }
            let Some(plan) = self.plan(resizer, driver)? else {
                continue;
            };
            let at = midpoint(
                resizer.netlist().pin_location(driver),
                resizer.netlist().pin_location(pins[i + 1]),
            );
            self.try_insert(resizer, driver, &plan, at)?;
        }
        Ok(())
    }

    /// The loads of `driver` to put behind a hold buffer, if the setup
    /// slack of the loads can take its delay.
    fn plan(
        &self,
        resizer: &Resizer<'_>,
        driver: PinId,
    ) -> Result<Option<Vec<PinId>>, RepairError> {
        let netlist = resizer.netlist();
        let sta = resizer.sta().ok_or(RepairError::NoTiming)?;
        let net = netlist.db.pins[driver.index()].net;
        let mut loads = Vec::new();
        let mut slacks = Slacks::new();
        let mut excluded_cap = 0.0;
        for load in netlist.loads(net).filter(|&l| l != driver) {
            if netlist.is_retired(load) {
                continue;
            }
            if sta.slack(load, MinMax::Min) < self.options.hold_margin {
                loads.push(load);
                slacks.merge(resizer, load);
            } else if let Some(port) = netlist.lib_port(load) {
                excluded_cap += port.capacitance();
            }
        }
        if loads.is_empty() {
            return Ok(None);
        }
        let load_cap = (resizer.load_cap(driver) - excluded_cap).max(0.0);
        let lib = resizer.link.liberty.cell(self.buffer);
        let delays = buffer_delays(lib, &resizer.targets.slews, load_cap);
        let o = self.options;
        let fits = o.allow_setup_violations
            || Edge::BOTH.iter().all(|&edge| {
                let i = edge.index();
                let setup = slacks.late[i] - o.setup_margin;
                setup > -(slacks.early[i] - o.hold_margin) && setup > delays.get(edge)
            });
        Ok(fits.then_some(loads))
    }

    /// Puts the hold buffer in and keeps it unless it raises the driver's
    /// slew by more than [`MAX_SLEW_GROWTH`] or worsens the worst setup
    /// slack below the margin, as `repairEndHold` and `makeHoldDelay`.
    fn try_insert(
        &mut self,
        resizer: &mut Resizer<'_>,
        driver: PinId,
        loads: &[PinId],
        at: Point<Dbu>,
    ) -> Result<(), RepairError> {
        resizer.journal_begin();
        let setup_before = resizer
            .sta()
            .ok_or(RepairError::NoTiming)?
            .worst_slack(MinMax::Max)
            .0;
        let slew_before = resizer.graph().slew(driver).max();
        let net = resizer.netlist().db.pins[driver.index()].net;
        let inserted = resizer.insert_buffer_before_loads(net, loads, self.buffer, at, "hold")?;
        let resized = !Self::load_in_hand(resizer, inserted.output)
            && resizer.resize_to_target_slew(inserted.output)?;
        let slew_after = resizer.graph().slew(driver).max();
        let setup_after = resizer
            .sta()
            .ok_or(RepairError::NoTiming)?
            .worst_slack(MinMax::Max)
            .0;
        let growth = if slew_before > 0.0 {
            slew_after / slew_before
        } else {
            1.0
        };
        let breaks_setup = !self.options.allow_setup_violations
            && fuzzy::less(setup_after, setup_before)
            && setup_after < self.options.setup_margin;
        if growth > MAX_SLEW_GROWTH || breaks_setup {
            resizer.journal_restore()?;
        } else {
            resizer.journal_end();
            self.inserted += 1;
            if resized {
                self.resized += 1;
            }
        }
        Ok(())
    }

    /// Whether a buffer's output keeps [`SLACK_LIMIT_RATIO`] of its load,
    /// slew and load slew limits in hand, so it can stay at the smallest
    /// size, as `checkMaxSlewCap`.
    fn load_in_hand(resizer: &Resizer<'_>, output: PinId) -> bool {
        let in_hand = |value: f64, limit: f64| (limit - value) / limit >= SLACK_LIMIT_RATIO;
        if let Some(limit) = resizer.cap_limit(output)
            && limit > 0.0
            && !in_hand(resizer.load_cap(output), limit)
        {
            return false;
        }
        if let Some(limit) = resizer.driver_slew_limit(output)
            && limit > 0.0
            && !in_hand(resizer.graph().slew(output).max(), limit)
        {
            return false;
        }
        let netlist = resizer.netlist();
        let net = netlist.db.pins[output.index()].net;
        netlist.loads(net).filter(|&l| l != output).all(|load| {
            netlist.lib_cell_port(load).is_none_or(|(cell, port)| {
                let limit = resizer.max_input_slew(cell, port);
                !limit.is_finite() || in_hand(resizer.graph().slew(load).max(), limit)
            })
        })
    }
}

const fn midpoint(a: Point<Dbu>, b: Point<Dbu>) -> Point<Dbu> {
    Point::new(
        Dbu(i64::midpoint(a.x.get(), b.x.get())),
        Dbu(i64::midpoint(a.y.get(), b.y.get())),
    )
}
