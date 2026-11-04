//! Clock tree synthesis and the repairs that follow it.

use crate::repair::{NetlistRepair, Networks, clock_wire_rc, wire_rc};
use crate::report::{ClockTreeSummary, Report};
use anyhow::Context;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::PinId;
use pare_common::units::Seconds;
use pare_common::util::config::Config;
use pare_common::util::ui;
use pare_cts::{CtsInput, CtsOptions, balance_latencies};
use pare_resizer::{Link, Repairer, Resizer, SessionInputs, repair_clock_inverters};
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
    if repair.sdc.is_none() || !config.netlist.clock_tree {
        return Ok(false);
    }
    let rc = wire_rc(config, db)?;
    let clock_rc = clock_wire_rc(config, db)?.context("[netlist] clock trees need clock_wire")?;
    let phase_start = Instant::now();
    ui::phase("Clock Tree Synthesis");
    let link = Link::new(db, &repair.liberty, &config.netlist.dont_use)?;
    let mut summary = ClockTreeSummary::default();
    {
        let networks = Networks::find(db, &link, config, repair.sdc.as_ref())?;
        let inverters = repair_clock_inverters(db, &link, &networks.clocks, &mut repair.namer)?;
        summary.cloned_inverters = inverters.inverters;
    }
    let max_wire_length =
        pare_resizer::rebuffer::max_wire_length_of(&link, &link.find_buffers(db)?, rc, db.dbu());
    let trees = {
        let sdc = repair.sdc.as_ref().context("constraints")?;
        let networks = Networks::find(db, &link, config, Some(sdc))?;
        let input = CtsInput {
            link: &link,
            sdc,
            clocks: &networks.clocks,
            signal_rc: rc,
            clock_rc,
            max_wire_length,
            options: CtsOptions::default(),
        };
        pare_cts::synthesize(db, &input)?
    };
    let sdc = repair.sdc.as_mut().context("constraints")?;
    for clock in &mut sdc.clocks {
        clock.propagated = true;
    }
    let sdc = repair.sdc.as_ref().context("constraints")?;
    {
        let networks = Networks::find(db, &link, config, Some(sdc))?;
        let mut resizer = Resizer::new(SessionInputs {
            db,
            context: networks.context(&link, Some(clock_rc)),
            rc,
            buffers: networks.buffers.clone(),
            namer: &mut repair.namer,
        })?;
        let done = Repairer::new(&mut resizer)
            .repair_clock_nets(trees.clock_buffers(), max_wire_length)?;
        summary.clock_net_repeaters = done.inserted;
        let _ = resizer.finish()?;
    }
    let arrivals: FxHashMap<PinId, Seconds> = {
        let networks = Networks::find(db, &link, config, Some(sdc))?;
        let pins = db.pins.len();
        let inputs = SessionInputs {
            db,
            context: networks.context(&link, Some(clock_rc)),
            rc,
            buffers: networks.buffers.clone(),
            namer: &mut repair.namer,
        };
        let resizer = Resizer::timed(inputs, sdc)?;
        let sta = resizer.sta();
        (0..pins)
            .map(PinId::new)
            .filter_map(|p| sta.clock_rise_arrival(p).map(|a| (p, a)))
            .collect()
    };
    summary.delay_buffers = balance_latencies(db, &link, &trees, &|p| arrivals.get(&p).copied())?;
    let cts = &trees.summary;
    summary.trees = cts.trees;
    summary.sinks = cts.sinks;
    summary.buffers = cts.buffers;
    summary.dummies = cts.dummies;
    summary.cloned_gates = cts.clones;
    ui::stat_row(
        "Clock trees:",
        &summary.trees.to_string(),
        "Sinks:",
        &summary.sinks.to_string(),
    );
    ui::stat_row(
        "Buffers:",
        &format!("{} {}", summary.buffers, cts.root_buffer),
        "Dummy loads:",
        &summary.dummies.to_string(),
    );
    ui::stat_row(
        "Clock net repeaters:",
        &summary.clock_net_repeaters.to_string(),
        "Delay buffers:",
        &summary.delay_buffers.to_string(),
    );
    report.clock_tree = Some(summary);
    report.phase("clock tree synthesis", phase_start);
    Ok(true)
}
