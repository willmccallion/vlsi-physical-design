//! Clock tree synthesis and the repairs that follow it.

use crate::repair::{NetlistRepair, TimedRepair};
use crate::report::{ClockTreeSummary, Report};
use anyhow::Context;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::PinId;
use pare_common::units::Seconds;
use pare_common::util::config::Config;
use pare_common::util::ui;
use pare_cts::{ClockTrees, CtsInput, CtsOptions, balance_latencies};
use pare_resizer::parasitics::WireRc;
use pare_resizer::rebuffer::max_wire_length_of;
use pare_resizer::{Link, Repairer, Resizer, repair_clock_inverters};
use rustc_hash::FxHashMap;
use std::time::Instant;

/// Builds the clock trees on the detailed placement and propagates the
/// clocks through them, as `OpenROAD-flow-scripts`' clock tree stage runs
/// `repair_clock_inverters` and `clock_tree_synthesis -sink_clustering_enable
/// -repair_clock_nets`, when the input has constraints; returns whether it
/// ran.
pub(crate) fn clock_tree_synthesis(
    config: &Config,
    db: &mut NetlistDB,
    repair: &mut NetlistRepair,
    report: &mut Report,
) -> anyhow::Result<bool> {
    if !config.netlist.clock_tree {
        return Ok(false);
    }
    let Some(mut timed) = repair.timed() else {
        return Ok(false);
    };
    let clock_rc = timed
        .wires
        .clock
        .context("[netlist] clock trees need clock_wire")?;
    let phase_start = Instant::now();
    ui::phase("Clock Tree Synthesis");
    let cloned_inverters = clone_clock_inverters(config, db, &mut timed)?;
    let trees = build_trees(config, db, &timed, clock_rc)?;
    for clock in &mut timed.sdc.clocks {
        clock.propagated = true;
    }
    let clock_net_repeaters = repair_clock_nets(config, db, &mut timed, &trees)?;
    let arrivals = clock_arrivals(config, db, &mut timed)?;
    let link = Link::new(db, timed.liberty, &config.netlist.dont_use)?;
    let delay_buffers = balance_latencies(db, &link, &trees, &|p| arrivals.get(&p).copied())?;
    let cts = &trees.summary;
    let summary = ClockTreeSummary {
        trees: cts.trees,
        sinks: cts.sinks,
        buffers: cts.buffers,
        dummies: cts.dummies,
        cloned_gates: cts.clones,
        cloned_inverters,
        clock_net_repeaters,
        delay_buffers,
    };
    show_clock_trees(&summary, &cts.root_buffer);
    report.clock_tree = Some(summary);
    report.phase("clock tree synthesis", phase_start);
    Ok(true)
}

/// Clones each clock inverter at its loads, as `repair_clock_inverters`;
/// returns how many were cloned.
fn clone_clock_inverters(
    config: &Config,
    db: &mut NetlistDB,
    timed: &mut TimedRepair<'_>,
) -> anyhow::Result<usize> {
    let stage = timed.stage(config, db)?;
    let done = repair_clock_inverters(db, &stage.link, &stage.clocks, timed.namer)?;
    Ok(done.inverters)
}

/// Builds a tree for every clock net on wires of `clock_rc`, no longer
/// than the resizer's buffers drive whole.
fn build_trees(
    config: &Config,
    db: &mut NetlistDB,
    timed: &TimedRepair<'_>,
    clock_rc: WireRc,
) -> anyhow::Result<ClockTrees> {
    let stage = timed.stage(config, db)?;
    let signal_rc = timed.wires.signal;
    let input = CtsInput {
        link: &stage.link,
        sdc: timed.sdc,
        clocks: &stage.clocks,
        signal_rc,
        clock_rc,
        max_wire_length: max_wire_length_of(&stage.link, &stage.buffers, signal_rc, db.dbu()),
        options: CtsOptions::default(),
    };
    Ok(pare_cts::synthesize(db, &input)?)
}

/// Splits the clock nets longer than the trees' wire limit with repeaters,
/// as `-repair_clock_nets`; returns how many were inserted.
fn repair_clock_nets(
    config: &Config,
    db: &mut NetlistDB,
    timed: &mut TimedRepair<'_>,
    trees: &ClockTrees,
) -> anyhow::Result<usize> {
    let stage = timed.stage(config, db)?;
    let mut resizer = Resizer::new(stage.session(db, timed.wires, timed.namer))?;
    let done = Repairer::new(&mut resizer)
        .repair_clock_nets(trees.clock_buffers(), trees.max_wire_length())?;
    let _ = resizer.finish()?;
    Ok(done.inserted)
}

/// Every pin's rising clock arrival through the propagated trees.
fn clock_arrivals(
    config: &Config,
    db: &mut NetlistDB,
    timed: &mut TimedRepair<'_>,
) -> anyhow::Result<FxHashMap<PinId, Seconds>> {
    let stage = timed.stage(config, db)?;
    let pins = db.pins.len();
    let resizer = Resizer::timed(stage.session(db, timed.wires, timed.namer), timed.sdc)?;
    let sta = resizer.sta();
    Ok((0..pins)
        .map(PinId::new)
        .filter_map(|p| sta.clock_rise_arrival(p).map(|a| (p, a)))
        .collect())
}

/// Shows what clock tree synthesis built, `root_buffer` naming the buffer
/// at each tree's root.
fn show_clock_trees(summary: &ClockTreeSummary, root_buffer: &str) {
    ui::stat_row(
        "Clock trees:",
        &summary.trees.to_string(),
        "Sinks:",
        &summary.sinks.to_string(),
    );
    ui::stat_row(
        "Buffers:",
        &format!("{} {root_buffer}", summary.buffers),
        "Dummy loads:",
        &summary.dummies.to_string(),
    );
    ui::stat_row(
        "Clock net repeaters:",
        &summary.clock_net_repeaters.to_string(),
        "Delay buffers:",
        &summary.delay_buffers.to_string(),
    );
}
