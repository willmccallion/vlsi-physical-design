//! `OpenSTA`'s cycle accounting between a launching and a capturing clock edge.

use super::{Accounting, opposite};
use crate::fuzzy;
use pare_common::db::sdc::{ClockEdge, Sdc};
use pare_common::units::Seconds;

/// `OpenSTA`'s `CycleAccting::findDelays` for setup, hold and latch setup:
/// the first capturing edge after the launching one, the one at or before
/// it, and for a latch the opening edge before the closing edge the data
/// must beat, searched over the cycles until the two clocks realign.
pub(super) fn cycle_accounting(sdc: &Sdc, launch: ClockEdge, target: ClockEdge) -> Accounting {
    let src_clock = sdc.clock(launch.clock);
    let tgt_clock = sdc.clock(target.clock);
    let src_period = src_clock.period;
    let tgt_period = tgt_clock.period;
    let src_edge_time = src_clock.edge_time(launch.edge);
    let tgt_edge_time = tgt_clock.edge_time(target.edge);
    let tgt_opposite_time = tgt_clock.edge_time(opposite(target.edge));
    let mut search = CycleSearch::new();
    if tgt_period <= Seconds::ZERO {
        return search.accounting;
    }
    if src_period <= Seconds::ZERO {
        return default_arrival_accounting(tgt_edge_time, tgt_period);
    }
    let tgt_max_cycle: i64 = if tgt_period > src_period {
        SLOWER_TARGET_CYCLES
    } else {
        ((src_period / tgt_period).ceil() as i64).max(FASTER_TARGET_MIN_CYCLES)
    };
    let mut tgt_cycle = first_cycle(tgt_edge_time, tgt_period);
    while tgt_cycle <= tgt_max_cycle {
        let start = tgt_cycle as f64 * tgt_period;
        let tgt = TargetCycle {
            cycle: tgt_cycle,
            start,
            time: start + tgt_edge_time,
            opposite: start + tgt_opposite_time,
            period: tgt_period,
        };
        if search.record_sources(&tgt, src_edge_time, src_period) == SourceScan::Realigned {
            return search.accounting;
        }
        tgt_cycle += 1;
    }
    Accounting {
        max_cycles_exceeded: true,
        ..search.accounting
    }
}

/// The accounting of a launch without a period against a capturing edge at
/// `tgt_time` of a clock of `tgt_period`, as `CycleAccting`'s
/// `findDefaultArrivalSrcDelays`: setup in the first cycle the edge falls
/// in, hold at the edge itself.
fn default_arrival_accounting(tgt_time: Seconds, tgt_period: Seconds) -> Accounting {
    let (setup_required, tgt_cycle) = if tgt_time > tgt_period {
        (tgt_time - tgt_period, 0)
    } else if tgt_time > Seconds::ZERO {
        (tgt_time, 0)
    } else {
        (tgt_period, 1)
    };
    Accounting {
        setup_required,
        hold_required: tgt_time,
        gated_hold_required: Seconds::ZERO,
        latch_setup_required: setup_required,
        latch_cycles: (0, tgt_cycle),
        max_cycles_exceeded: false,
    }
}

/// The target cycles `CycleAccting::findDelays` searches when the target
/// clock is the slower.
const SLOWER_TARGET_CYCLES: i64 = 100;

/// The fewest target cycles `CycleAccting::findDelays` searches when the
/// target clock is no slower.
const FASTER_TARGET_MIN_CYCLES: i64 = 1000;

/// One cycle of the capturing clock: its number, when it starts, when its
/// edge and the opposite edge happen in it, and its period.
#[derive(Clone, Copy, Debug)]
struct TargetCycle {
    cycle: i64,
    start: Seconds,
    time: Seconds,
    opposite: Seconds,
    period: Seconds,
}

/// One cycle of the launching clock: its number, when it starts and when
/// its edge happens in it.
#[derive(Clone, Copy, Debug)]
struct SourceCycle {
    cycle: i64,
    start: Seconds,
    time: Seconds,
}

/// Where `CycleAccting::findDelays` stands: the tightest delays found so
/// far for each check, the requireds they give, and whether either clock's
/// edge has passed the other's.
struct CycleSearch {
    accounting: Accounting,
    setup_delay: Seconds,
    hold_delay: Seconds,
    gated_hold_delay: Seconds,
    latch_delay: Seconds,
    tgt_past_src: bool,
    src_past_tgt: bool,
}

/// The cycle a search of a clock's edges starts at, so that the edge at
/// `time` into a cycle lands in the first period.
fn first_cycle(time: Seconds, period: Seconds) -> i64 {
    if time < Seconds::ZERO {
        1
    } else if time < period {
        0
    } else {
        -1
    }
}

/// How a scan of the launching clock's cycles against one target cycle
/// ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SourceScan {
    /// The two clocks realigned, so the search is done.
    Realigned,
    /// The launching clock passed the target cycle.
    PassedTarget,
}

impl CycleSearch {
    /// Records each cycle of the launching clock, its edge `src_edge_time`
    /// into a period of `src_period`, against target cycle `tgt` until it
    /// passes `tgt` or the clocks realign.
    fn record_sources(
        &mut self,
        tgt: &TargetCycle,
        src_edge_time: Seconds,
        src_period: Seconds,
    ) -> SourceScan {
        let mut src_cycle = first_cycle(src_edge_time, src_period);
        loop {
            let src_start = src_cycle as f64 * src_period;
            if self.tgt_past_src && self.src_past_tgt && fuzzy::equal(src_start, tgt.start) {
                return SourceScan::Realigned;
            }
            if fuzzy::greater(src_start, tgt.start + tgt.period) && self.src_past_tgt {
                return SourceScan::PassedTarget;
            }
            self.record(
                tgt,
                SourceCycle {
                    cycle: src_cycle,
                    start: src_start,
                    time: src_start + src_edge_time,
                },
            );
            src_cycle += 1;
        }
    }

    const fn new() -> Self {
        Self {
            accounting: Accounting {
                setup_required: Seconds::ZERO,
                hold_required: Seconds::ZERO,
                gated_hold_required: Seconds::ZERO,
                latch_setup_required: Seconds::ZERO,
                latch_cycles: (0, 0),
                max_cycles_exceeded: false,
            },
            setup_delay: Seconds::INFINITY,
            hold_delay: Seconds::INFINITY,
            gated_hold_delay: Seconds::INFINITY,
            latch_delay: Seconds::INFINITY,
            tgt_past_src: false,
            src_past_tgt: false,
        }
    }

    /// Takes the launching edge of `src` against the capturing edges of
    /// `tgt`, keeping each check's tighter delay.
    fn record(&mut self, tgt: &TargetCycle, src: SourceCycle) {
        if fuzzy::greater(tgt.time, src.time) {
            self.tgt_past_src = true;
            let delay = tgt.time - src.time;
            if fuzzy::less(delay, self.setup_delay) {
                self.setup_delay = delay;
                self.accounting.setup_required = tgt.time - src.start;
            }
        }
        if fuzzy::greater(tgt.opposite, src.time) {
            let delay = tgt.opposite - src.time;
            if fuzzy::less(delay, self.latch_delay) {
                self.latch_delay = delay;
                let (mut latch_time, mut latch_cycle) = (tgt.time, tgt.cycle);
                if tgt.time > tgt.opposite {
                    latch_time -= tgt.period;
                    latch_cycle -= 1;
                }
                self.accounting.latch_setup_required = latch_time - src.start;
                self.accounting.latch_cycles = (src.cycle, latch_cycle);
            }
        }
        if fuzzy::less_equal(tgt.time, src.time) {
            let delay = src.time - tgt.time;
            self.src_past_tgt = true;
            if fuzzy::less(delay, self.hold_delay) {
                self.hold_delay = delay;
                self.accounting.hold_required = tgt.time - src.start;
            }
        }
        if fuzzy::less_equal(tgt.opposite, src.time) {
            let delay = src.time - tgt.time;
            if fuzzy::less(delay, self.gated_hold_delay) {
                self.gated_hold_delay = delay;
                self.accounting.gated_hold_required = tgt.time - src.start;
            }
        }
    }
}
