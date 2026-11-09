//! Hold repair, as `OpenROAD`'s `RepairHold`.
//!
//! The endpoints failing hold are taken from the worst up, and along each
//! one's worst early path a buffer goes in at each driver ahead of its
//! loads that fail hold, delaying them, as long as the loads' setup slack
//! can take the buffer's delay. An insertion is kept only when it leaves
//! the driver's slew and the design's worst setup slack about as they were.

use crate::fuzzy;
use crate::resizer::{RepairError, Resizer, buffer_delays};
use crate::sta::Sta;
use pare_common::config::Share;
use pare_common::db::indices::PinId;
use pare_common::db::liberty::{BufferCell, Edge, EdgeValues, LibCellId, MinMax};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::units::{Farads, Quantity, Seconds};

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

/// The most hold buffers as a share of the instances, `OpenROAD`'s
/// `-max_buffer_percent 20`.
const MAX_BUFFER_SHARE: Share = match Share::new(0.2) {
    Some(share) => share,
    None => panic!("0.2 is a share"),
};

/// How hold repair runs, as `repair_timing -hold` takes it.
#[derive(Clone, Debug, PartialEq)]
pub struct HoldOptions {
    /// Setup slack every endpoint should keep.
    pub setup_margin: Seconds,
    /// Hold slack every endpoint should reach.
    pub hold_margin: Seconds,
    /// Whether a hold fix may break setup.
    pub allow_setup_violations: bool,
    /// The most hold buffers, as a share of the instances.
    pub max_buffer_share: Share,
    /// The most endpoint repairs.
    pub max_passes: usize,
}

impl Default for HoldOptions {
    /// `OpenROAD`'s defaults.
    fn default() -> Self {
        Self {
            setup_margin: Seconds::ZERO,
            hold_margin: Seconds::ZERO,
            allow_setup_violations: false,
            max_buffer_share: MAX_BUFFER_SHARE,
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
    /// Worst hold slack at the start.
    pub wns_before: Seconds,
    /// Worst hold slack at the end.
    pub wns_after: Seconds,
    /// Worst setup slack at the start.
    pub setup_wns_before: Seconds,
    /// Worst setup slack at the end.
    pub setup_wns_after: Seconds,
    /// Hold buffers inserted.
    pub inserted: usize,
    /// Of them, resized to their load.
    pub resized: usize,
    /// The buffer cell used.
    pub buffer: Option<LibCellId>,
}

/// Repairs the hold violations of the session's design, as `OpenROAD`'s
/// `repair_timing -hold`.
pub fn repair_hold(
    resizer: &mut Resizer<'_, Sta>,
    options: &HoldOptions,
) -> Result<HoldSummary, RepairError> {
    let sta = resizer.sta();
    let mut ends: Vec<PinId> = sta.endpoints().to_vec();
    ends.sort_unstable();
    let setup_wns_before = sta.worst_slack(MinMax::Max).0.time();
    let mut summary = HoldSummary {
        setup_wns_before,
        ..HoldSummary::default()
    };
    let (mut failures, mut worst) = hold_violations(resizer, &ends, options.hold_margin);
    summary.failing_before = failures.len();
    summary.wns_before = sta.worst_slack(MinMax::Min).0.time();
    let Some(buffer) = hold_buffer(resizer) else {
        summary.failing_after = failures.len();
        summary.wns_after = summary.wns_before;
        summary.setup_wns_after = setup_wns_before;
        return Ok(summary);
    };
    summary.buffer = Some(buffer.id);
    let cells = resizer.netlist().db.cells.len();
    let budget = ((options.max_buffer_share.get() * cells as f64) as usize).max(MIN_BUFFER_BUDGET);
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
        let sta = resizer.sta();
        failures.sort_by(|&a, &b| {
            sta.slack(a, MinMax::Min)
                .time()
                .total_cmp(&sta.slack(b, MinMax::Min).time())
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
    let sta = resizer.sta();
    summary.failing_after = failures.len();
    summary.wns_after = sta.worst_slack(MinMax::Min).0.time();
    summary.setup_wns_after = sta.worst_slack(MinMax::Max).0.time();
    summary.inserted = repair.inserted;
    summary.resized = repair.resized;
    Ok(summary)
}

/// The endpoints whose hold slack is below `margin`, clock pins aside, and
/// the worst of their slacks, as `findHoldViolations`.
fn hold_violations(
    resizer: &Resizer<'_, Sta>,
    ends: &[PinId],
    margin: Seconds,
) -> (Vec<PinId>, Seconds) {
    let sta = resizer.sta();
    let mut worst = Seconds::INFINITY;
    let mut failing = Vec::new();
    for &end in ends {
        let slack = sta.slack(end, MinMax::Min).time();
        if !resizer.clocks.is_clock(end) && slack < margin {
            worst = worst.min(slack);
            failing.push(end);
        }
    }
    (failing, worst)
}

/// The buffer with the most hold delay per area, or a smaller one within
/// [`BUFFER_METRIC_MARGIN`] of it, as `findHoldBuffer`.
fn hold_buffer(resizer: &Resizer<'_, Sta>) -> Option<BufferCell> {
    let liberty = resizer.link.liberty;
    let mut scored: Vec<(Seconds, BufferCell)> = hold_buffer_candidates(resizer)
        .iter()
        .filter_map(|&buffer| {
            let lib = liberty.cell(buffer.id);
            (lib.area != 0.0).then(|| (hold_delay(resizer, buffer) / lib.area, buffer))
        })
        .collect();
    scored.sort_by(|a, b| a.0.total_cmp(&b.0));
    let &(best_metric, mut best) = scored.last()?;
    for &(metric, buffer) in scored.iter().rev().skip(1) {
        if metric >= BUFFER_METRIC_MARGIN * best_metric
            && liberty.cell(buffer.id).area < liberty.cell(best.id).area
        {
            best = buffer;
        }
    }
    Some(best)
}

/// The buffers a hold buffer is chosen from, as `filterHoldBuffers` over
/// `getBufferList`: every usable buffer, clock buffers too, since
/// `Resizer::repairHold` lets them in for libraries without delay cells;
/// those on the shortest site, and delay cells alone when the library has
/// any, each preference dropped when nothing meets it. The library model
/// has no threshold voltages to match.
fn hold_buffer_candidates(resizer: &Resizer<'_, Sta>) -> Vec<BufferCell> {
    let link = resizer.link;
    let library = &resizer.db.library;
    let buffers: Vec<BufferCell> = (0..link.liberty.cells.len())
        .map(|i| LibCellId(i as u32))
        .filter(|&id| {
            let cell = link.liberty.cell(id);
            cell.is_buffer() && !cell.is_power_cell && !link.dont_use(id) && link.is_link_cell(id)
        })
        .filter_map(|id| link.liberty.buffer_cell(id))
        .collect();
    let site_of = |b: BufferCell| link.macro_of(b.id).and_then(|m| library.cell_macro(m).site);
    let shortest_site = buffers
        .iter()
        .filter_map(|&b| site_of(b))
        .min_by_key(|&site| library.site(site).size.y);
    let delay_footprint = |b: BufferCell| {
        link.liberty
            .cell(b.id)
            .footprint
            .as_deref()
            .is_some_and(is_delay_cell)
    };
    let has_delay_cells = buffers.iter().any(|&b| delay_footprint(b));
    let is_delay = |b: BufferCell| !has_delay_cells || delay_footprint(b);
    let on_site_delay: Vec<BufferCell> = buffers
        .iter()
        .copied()
        .filter(|&b| site_of(b) == shortest_site && is_delay(b))
        .collect();
    if !on_site_delay.is_empty() {
        return on_site_delay;
    }
    let delay: Vec<BufferCell> = buffers.iter().copied().filter(|&b| is_delay(b)).collect();
    if delay.is_empty() { buffers } else { delay }
}

/// Whether a footprint names a delay cell, as `RepairHold`'s
/// `isDelayCell` reads the ones current libraries use.
fn is_delay_cell(footprint: &str) -> bool {
    ["DEL", "DLY", "dlygate"]
        .iter()
        .any(|mark| footprint.contains(mark))
}

/// A buffer's delay driving its own input, the lesser of its edges, as
/// `bufferHoldDelay`.
fn hold_delay(resizer: &Resizer<'_, Sta>, buffer: BufferCell) -> Seconds {
    let liberty = resizer.link.liberty;
    let input_cap = liberty.cell(buffer.id).ports[buffer.input].capacitance();
    let delays = buffer_delays(liberty, buffer, &resizer.targets.slews, input_cap);
    delays.rise.min(delays.fall)
}

/// Slacks per edge and side, merged over loads as `RepairHold` merges
/// them: the least early slack and the greatest late one.
#[derive(Clone, Copy, Debug)]
struct Slacks {
    early: EdgeValues<Seconds>,
    late: EdgeValues<Seconds>,
}

impl Slacks {
    const fn new() -> Self {
        Self {
            early: EdgeValues::both(Seconds::INFINITY),
            late: EdgeValues::both(Seconds::new(f64::NEG_INFINITY)),
        }
    }

    fn merge(&mut self, resizer: &Resizer<'_, Sta>, pin: PinId) {
        let sta = resizer.sta();
        for edge in Edge::BOTH {
            self.early[edge] = self.early[edge].min(sta.slack_edge(pin, edge, MinMax::Min).time());
            self.late[edge] = self.late[edge].max(sta.slack_edge(pin, edge, MinMax::Max).time());
        }
    }
}

/// The repair in progress.
struct HoldRepair<'o> {
    buffer: BufferCell,
    options: &'o HoldOptions,
    inserted: usize,
    resized: usize,
}

impl HoldRepair<'_> {
    /// Buffers the loads failing hold of each driver on the worst early
    /// path into `end`, as `repairEndHold`.
    fn repair_end(
        &mut self,
        resizer: &mut Resizer<'_, Sta>,
        end: PinId,
    ) -> Result<(), RepairError> {
        let path = {
            let netlist = resizer.netlist();
            let sta = resizer.sta();
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
            let Some(plan) = self.plan(resizer, driver) else {
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
    fn plan(&self, resizer: &Resizer<'_, Sta>, driver: PinId) -> Option<Vec<PinId>> {
        let netlist = resizer.netlist();
        let sta = resizer.sta();
        let net = netlist.db.pins[driver].net;
        let mut loads = Vec::new();
        let mut slacks = Slacks::new();
        let mut excluded_cap = Farads::ZERO;
        for load in netlist.loads(net).filter(|&l| l != driver) {
            if netlist.is_retired(load) {
                continue;
            }
            if sta.slack(load, MinMax::Min).time() < self.options.hold_margin {
                loads.push(load);
                slacks.merge(resizer, load);
            } else if let Some(port) = netlist.lib_port(load) {
                excluded_cap += port.capacitance();
            }
        }
        if loads.is_empty() {
            return None;
        }
        // NOTE: repairEndHold takes the endpoint's load here, not the driver's.
        let load_cap = (resizer.load_cap(driver) - excluded_cap).max(Farads::ZERO);
        let delays = buffer_delays(
            resizer.link.liberty,
            self.buffer,
            &resizer.targets.slews,
            load_cap,
        );
        let o = self.options;
        let fits = o.allow_setup_violations
            || Edge::BOTH.iter().all(|&edge| {
                let setup = slacks.late[edge] - o.setup_margin;
                setup > -(slacks.early[edge] - o.hold_margin) && setup > delays.get(edge)
            });
        fits.then_some(loads)
    }

    /// Puts the hold buffer in and keeps it unless it raises the driver's
    /// slew by more than [`MAX_SLEW_GROWTH`] or worsens the worst setup
    /// slack below the margin, as `repairEndHold` and `makeHoldDelay`.
    fn try_insert(
        &mut self,
        resizer: &mut Resizer<'_, Sta>,
        driver: PinId,
        loads: &[PinId],
        at: Point<Dbu>,
    ) -> Result<(), RepairError> {
        resizer.journal_begin();
        let setup_before = resizer.sta().worst_slack(MinMax::Max).0.time();
        let slew_before = resizer.graph().slew(driver).max();
        let net = resizer.netlist().db.pins[driver].net;
        let inserted = resizer.insert_buffer_before_loads(net, loads, self.buffer, at, "hold")?;
        let resized = !Self::load_in_hand(resizer, inserted.output)
            && resizer.resize_to_target_slew(inserted.output)?;
        let slew_after = resizer.graph().slew(driver).max();
        let setup_after = resizer.sta().worst_slack(MinMax::Max).0.time();
        let growth = if slew_before > Seconds::ZERO {
            slew_after / slew_before
        } else {
            1.0
        };
        let breaks_setup = !self.options.allow_setup_violations
            && fuzzy::less(setup_after, setup_before)
            && setup_after < self.options.setup_margin;
        if growth > MAX_SLEW_GROWTH || breaks_setup {
            // NOTE: repairEndHold zeroes its whole inserted count here.
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
    fn load_in_hand(resizer: &Resizer<'_, Sta>, output: PinId) -> bool {
        if let Some(limit) = resizer.cap_limit(output)
            && limit > Farads::ZERO
            && !in_hand(resizer.load_cap(output), limit)
        {
            return false;
        }
        if let Some(limit) = resizer.driver_slew_limit(output)
            && limit > Seconds::ZERO
            && !in_hand(resizer.graph().slew(output).max(), limit)
        {
            return false;
        }
        let netlist = resizer.netlist();
        let net = netlist.db.pins[output].net;
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

/// Whether `value` leaves [`SLACK_LIMIT_RATIO`] of `limit` in hand.
fn in_hand<Q: Quantity>(value: Q, limit: Q) -> bool {
    (limit.base() - value.base()) / limit.base() >= SLACK_LIMIT_RATIO
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{cells, clock_cells_source, design, with_timed_session};

    #[test]
    fn a_clock_buffer_with_the_most_hold_delay_per_area_is_the_hold_buffer() {
        let mut db = design(
            "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN pair ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\n\
             COMPONENTS 2 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - f2 DFF_X1 + PLACED ( 12000 2800 ) N ;\nEND COMPONENTS\n\
             PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 2 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( f2 D ) ;\n\
             END NETS\nEND DESIGN\n",
        );
        let slow_clock_buffer = clock_cells_source().replace(
            "values (\"0.02, 0.08\", \"0.03, 0.09\")",
            "values (\"0.5, 0.6\", \"0.5, 0.6\")",
        );
        let lib = cells(&slow_clock_buffer);
        let clock = "create_clock -name clk -period 1.0 [get_ports clk]\n";

        with_timed_session(&mut db, &lib, clock, |resizer| {
            let chosen = hold_buffer(resizer).map(|b| resizer.link.liberty.cell(b.id).name.clone());

            assert_eq!(chosen.as_deref(), Some("CLKBUF_X3"));
        });
    }
}
