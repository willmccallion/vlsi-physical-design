//! Netlist repair around placement: buffer removal, port buffering, design
//! repair and timing repair, and the wire parasitics they estimate with.

use anyhow::Context;
use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::{liberty, sdc};
use pare_common::db::sdc::Sdc;
use pare_common::util::config::{Config, LayerRcConfig, NetlistConfig};
use pare_common::util::ui;

use crate::report::{NetlistSummary, Report, TimingSummary};
use pare_resizer::parasitics::WireRc;
use pare_resizer::resizer::Context as RepairContext;
use pare_resizer::{
    BufferSet, ClockNetwork, Constants, HoldOptions, Link, Namer, Netlist, Repairer, Resizer,
    SessionInputs, SetupOptions, Sta, Timing, repair_hold, repair_setup,
};
use std::path::Path;
use std::time::Instant;

/// Reads the Liberty files, the first one's thresholds and defaults
/// standing for all.
fn load_liberty(paths: &[String]) -> anyhow::Result<Liberty> {
    let mut merged: Option<Liberty> = None;
    for path in paths {
        ui::header("Input", path);
        let lib = liberty::parse_file(Path::new(path))?;
        match &mut merged {
            None => merged = Some(lib),
            Some(first) => first
                .absorb(lib)
                .map_err(|e| anyhow::anyhow!("cell '{}' is in two Liberty files", e.name))?,
        }
    }
    merged.context("no Liberty files")
}

/// What the netlist steps before placement leave for the repair after it.
pub(crate) struct NetlistRepair {
    pub(crate) liberty: Liberty,
    pub(crate) wires: Wires,
    pub(crate) sdc: Option<Sdc>,
    pub(crate) namer: Namer,
    pub(crate) summary: NetlistSummary,
}

/// The pins the clocks reach: those the SDC file's clocks enter at when
/// the input names one, else those `[netlist] clock_ports` names.
fn clock_network(
    netlist: &Netlist<'_>,
    config: &Config,
    sdc: Option<&Sdc>,
    constants: &Constants,
) -> anyhow::Result<ClockNetwork> {
    match sdc {
        Some(sdc) => {
            if !config.netlist.clock_ports.is_empty() {
                anyhow::bail!("[netlist] clock_ports must be empty when [input] sdc_file is set");
            }
            Ok(ClockNetwork::from_sdc(netlist, sdc, constants))
        }
        None => Ok(ClockNetwork::from_ports(
            netlist,
            &config.netlist.clock_ports,
            constants,
        )?),
    }
}

/// What a repair works around: the design's constant pins and clock
/// network, and the buffers it may insert.
pub(crate) struct Networks {
    pub(crate) constants: Constants,
    pub(crate) clocks: ClockNetwork,
    pub(crate) buffers: BufferSet,
}

impl Networks {
    /// The networks of `db` as it stands, linked by `link`.
    pub(crate) fn find(
        db: &NetlistDB,
        link: &Link<'_>,
        config: &Config,
        sdc: Option<&Sdc>,
    ) -> anyhow::Result<Self> {
        let netlist = Netlist::new(db, link);
        let constants = Constants::find(&netlist);
        let clocks = clock_network(&netlist, config, sdc, &constants)?;
        Ok(Self {
            constants,
            clocks,
            buffers: link.find_buffers(db)?,
        })
    }

    /// The context a resizer session over these networks takes.
    pub(crate) const fn context<'a>(
        &'a self,
        link: &'a Link<'a>,
        clock_rc: Option<WireRc>,
    ) -> RepairContext<'a> {
        RepairContext {
            link,
            clocks: &self.clocks,
            constants: &self.constants,
            clock_rc,
        }
    }
}

/// Strips the buffers synthesis left and buffers the ports, as
/// OpenROAD-flow-scripts does before placing, when the config names Liberty
/// files.
pub(crate) fn repair_netlist(
    config: &Config,
    db: &mut NetlistDB,
    report: &mut Report,
) -> anyhow::Result<Option<NetlistRepair>> {
    if config.input.liberty_files.is_empty() {
        return Ok(None);
    }
    let wires = Wires::from_config(&config.netlist, db)?;
    let liberty = load_liberty(&config.input.liberty_files)?;
    let sdc = match &config.input.sdc_file {
        Some(path) => {
            ui::header("Input", path);
            Some(sdc::parse_file(Path::new(path), db, &liberty)?)
        }
        None => None,
    };
    let link = Link::new(db, &liberty, &config.netlist.dont_use)?;
    ui::section("Netlist");
    let mut namer = Namer::new();
    let mut summary = NetlistSummary::default();
    if config.netlist.remove_buffers {
        let phase_start = Instant::now();
        ui::phase("Buffer Removal");
        let before = db.cells.len();
        summary.buffers_removed = pare_resizer::remove_buffers(db, &link)?;
        ui::stat_row(
            "Removed:",
            &summary.buffers_removed.to_string(),
            "Cells:",
            &format!("{before} -> {}", db.cells.len()),
        );
        report.phase("buffer removal", phase_start);
    }
    if config.netlist.buffer_ports {
        let phase_start = Instant::now();
        ui::phase("Port Buffering");
        let networks = Networks::find(db, &link, config, sdc.as_ref())?;
        let done = pare_resizer::buffer_ports(
            db,
            &link,
            &networks.buffers,
            &networks.clocks,
            &networks.constants,
            &mut namer,
        )?;
        summary.input_buffers = done.inputs;
        summary.output_buffers = done.outputs;
        summary
            .buffer_cell
            .clone_from(&liberty.cell(networks.buffers.lowest_drive).name);
        ui::stat_row(
            "Input buffers:",
            &done.inputs.to_string(),
            "Output buffers:",
            &done.outputs.to_string(),
        );
        ui::stat_row(
            "Buffer:",
            &summary.buffer_cell,
            "Clock pins:",
            &networks.clocks.sources().len().to_string(),
        );
        report.phase("port buffering", phase_start);
    }
    report.netlist = Some(NetlistSummary {
        buffers_removed: summary.buffers_removed,
        input_buffers: summary.input_buffers,
        output_buffers: summary.output_buffers,
        buffer_cell: summary.buffer_cell.clone(),
        ..NetlistSummary::default()
    });
    Ok(Some(NetlistRepair {
        liberty,
        wires,
        sdc,
        namer,
        summary,
    }))
}

/// Resizes drivers and inserts repeaters where the placed netlist breaks
/// the library's slew or load limits, as OpenROAD-flow-scripts'
/// `repair_design` does after global placement.
pub(crate) fn repair_design(
    config: &Config,
    db: &mut NetlistDB,
    repair: &mut NetlistRepair,
    report: &mut Report,
) -> anyhow::Result<()> {
    if !config.netlist.repair_design {
        return Ok(());
    }
    let phase_start = Instant::now();
    ui::phase("Design Repair");
    let link = Link::new(db, &repair.liberty, &config.netlist.dont_use)?;
    let networks = Networks::find(db, &link, config, repair.sdc.as_ref())?;
    let inputs = SessionInputs {
        db,
        context: networks.context(&link, repair.wires.clock),
        rc: repair.wires.signal,
        buffers: networks.buffers.clone(),
        namer: &mut repair.namer,
    };
    let done = match repair.sdc.as_ref() {
        Some(sdc) => repair_design_on(Resizer::timed(inputs, sdc)?)?,
        None => repair_design_on(Resizer::new(inputs)?)?,
    };
    repair.summary.slew_violations = done.slew_violations;
    repair.summary.cap_violations = done.cap_violations;
    repair.summary.repaired_nets = done.repaired_nets;
    repair.summary.resized = done.resized;
    repair.summary.repeaters = done.inserted;
    report.netlist = Some(NetlistSummary {
        buffers_removed: repair.summary.buffers_removed,
        input_buffers: repair.summary.input_buffers,
        output_buffers: repair.summary.output_buffers,
        buffer_cell: repair.summary.buffer_cell.clone(),
        slew_violations: done.slew_violations,
        cap_violations: done.cap_violations,
        repaired_nets: done.repaired_nets,
        resized: done.resized,
        repeaters: done.inserted,
    });
    report.phase("design repair", phase_start);
    Ok(())
}

/// Runs design repair over the session and ends it.
fn repair_design_on<T: Timing>(
    mut resizer: Resizer<'_, T>,
) -> anyhow::Result<pare_resizer::RepairSummary> {
    let done = Repairer::new(&mut resizer).repair_design()?;
    show_design_repair(&done, resizer.netlist().db.cells.len());
    let _ = resizer.finish()?;
    Ok(done)
}

/// Shows what design repair found and did.
pub(crate) fn show_design_repair(done: &pare_resizer::RepairSummary, cells: usize) {
    ui::stat_row(
        "Slew violations:",
        &done.slew_violations.to_string(),
        "Cap violations:",
        &done.cap_violations.to_string(),
    );
    ui::stat_row(
        "Resized:",
        &done.resized.to_string(),
        "Repeaters:",
        &done.inserted.to_string(),
    );
    ui::stat_row(
        "Nets repaired:",
        &done.repaired_nets.to_string(),
        "Cells:",
        &cells.to_string(),
    );
}

/// Repairs the paths failing setup and then hold on the legalized placement, as
/// `OpenROAD-flow-scripts` runs `repair_timing` after detailed placement,
/// when the input has constraints; returns whether it ran.
pub(crate) fn repair_timing(
    config: &Config,
    db: &mut NetlistDB,
    repair: &mut NetlistRepair,
    report: &mut Report,
) -> anyhow::Result<bool> {
    let Some(sdc) = repair.sdc.as_ref() else {
        return Ok(false);
    };
    if !config.netlist.repair_timing {
        return Ok(false);
    }
    let phase_start = Instant::now();
    ui::phase("Timing Repair");
    let link = Link::new(db, &repair.liberty, &config.netlist.dont_use)?;
    let networks = Networks::find(db, &link, config, Some(sdc))?;
    let inputs = SessionInputs {
        db,
        context: networks.context(&link, repair.wires.clock),
        rc: repair.wires.signal,
        buffers: networks.buffers.clone(),
        namer: &mut repair.namer,
    };
    let mut resizer = Resizer::timed(inputs, sdc)?;
    report.timing = Some(timing_repair_on(&mut resizer, &repair.liberty)?);
    let _ = resizer.finish()?;
    report.phase("timing repair", phase_start);
    Ok(true)
}

/// Repairs setup and then hold, as `repair_timing` does without
/// `-setup` or `-hold`, showing each iteration and what changed.
pub(crate) fn timing_repair_on(
    resizer: &mut Resizer<'_, Sta>,
    liberty: &Liberty,
) -> anyhow::Result<TimingSummary> {
    ui::timing_table_header();
    let summary = {
        let mut show = |progress: &pare_resizer::SetupProgress| {
            let iter = progress
                .iteration
                .map_or_else(|| "final".to_string(), |i| i.to_string());
            let worst = progress.worst.as_deref().unwrap_or("");
            ui::timing_iter(&ui::TimingRow {
                iter: &iter,
                removed: progress.removed,
                resized: progress.resized,
                inserted: progress.inserted,
                cloned: progress.cloned,
                swaps: progress.swapped,
                area_change_percent: progress.area_change_percent,
                wns_ns: progress.wns.nanos(),
                tns_ns: progress.tns.nanos(),
                failing: progress.failing,
                worst,
            });
        };
        repair_setup(resizer, SetupOptions::default(), &mut show)?
    };
    let hold = repair_hold(resizer, &HoldOptions::default())?;
    eprintln!();
    ui::stat_row(
        "WNS (ns):",
        &format!(
            "{:.3} -> {:.3}",
            summary.wns_before.nanos(),
            summary.wns_after.nanos()
        ),
        "TNS (ns):",
        &format!(
            "{:.1} -> {:.1}",
            summary.tns_before.nanos(),
            summary.tns_after.nanos()
        ),
    );
    ui::stat_row(
        "Failing endpoints:",
        &format!("{} -> {}", summary.failing_before, summary.failing_after),
        "Iterations:",
        &summary.iterations.to_string(),
    );
    ui::stat_row(
        "Removed:",
        &summary.removed.to_string(),
        "Resized:",
        &summary.resized.to_string(),
    );
    ui::stat_row(
        "Inserted:",
        &summary.inserted.to_string(),
        "Cloned:",
        &summary.cloned.to_string(),
    );
    ui::stat_row(
        "Pin swaps:",
        &summary.swapped.to_string(),
        "Area:",
        &format!("{:+.1}%", summary.area_change_percent),
    );
    ui::stat_row(
        "Hold WNS (ns):",
        &format!(
            "{:.3} -> {:.3}",
            hold.wns_before.nanos(),
            hold.wns_after.nanos()
        ),
        "Hold failing:",
        &format!("{} -> {}", hold.failing_before, hold.failing_after),
    );
    ui::stat_row(
        "Hold buffers:",
        &hold.inserted.to_string(),
        "Hold buffer:",
        hold.buffer
            .map_or("none", |b| liberty.cell(b).name.as_str()),
    );
    Ok(TimingSummary {
        failing_before: summary.failing_before,
        failing_after: summary.failing_after,
        wns_before_ns: summary.wns_before.nanos(),
        wns_after_ns: summary.wns_after.nanos(),
        tns_before_ns: summary.tns_before.nanos(),
        tns_after_ns: summary.tns_after.nanos(),
        removed: summary.removed,
        resized: summary.resized,
        inserted: summary.inserted,
        cloned: summary.cloned,
        swapped: summary.swapped,
        iterations: summary.iterations,
        area_change_percent: summary.area_change_percent,
        hold_failing_before: hold.failing_before,
        hold_failing_after: hold.failing_after,
        hold_wns_before_ns: hold.wns_before.nanos(),
        hold_wns_after_ns: hold.wns_after.nanos(),
        hold_buffers: hold.inserted,
    })
}

/// The wires repair estimates parasitics with, as `set_wire_rc` sets them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Wires {
    /// The signal wire, as `set_wire_rc -signal`.
    pub(crate) signal: WireRc,
    /// The clock wire, as `set_wire_rc -clock`, when the config has one.
    pub(crate) clock: Option<WireRc>,
}

impl Wires {
    /// The wires the netlist config names, on their layers of `db`.
    fn from_config(config: &NetlistConfig, db: &NetlistDB) -> anyhow::Result<Self> {
        let signal = config
            .signal_wire
            .as_ref()
            .context("[netlist] repair needs signal_wire")?;
        Ok(Self {
            signal: layer_wire_rc(signal, db).context("[netlist] signal_wire")?,
            clock: config
                .clock_wire
                .as_ref()
                .map(|wire| layer_wire_rc(wire, db).context("[netlist] clock_wire"))
                .transpose()?,
        })
    }
}

fn layer_wire_rc(wire: &LayerRcConfig, db: &NetlistDB) -> anyhow::Result<WireRc> {
    let layer = db
        .library
        .routing_layer_id(&wire.layer)
        .with_context(|| format!("unknown routing layer '{}'", wire.layer))?;
    Ok(WireRc::per_micron(
        wire.resistance_ohm_per_um,
        wire.capacitance_ff_per_um,
        layer,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::parser::{def, lef};
    use std::path::PathBuf;

    fn tiny() -> NetlistDB {
        let fixture = |name: &str| {
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../common/tests/fixtures")
                .join(name)
        };
        let units = def::read_units(&fixture("tiny.def")).unwrap();
        let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
        def::parse(&fixture("tiny.def"), library).unwrap()
    }

    fn wire(layer: &str) -> LayerRcConfig {
        LayerRcConfig {
            layer: layer.to_string(),
            resistance_ohm_per_um: 2.0,
            capacitance_ff_per_um: 0.5,
        }
    }

    #[test]
    fn wires_take_their_layers_and_rc_from_the_config() {
        let db = tiny();
        let config = NetlistConfig {
            signal_wire: Some(wire("M1")),
            clock_wire: Some(wire("M1")),
            ..NetlistConfig::default()
        };

        let wires = Wires::from_config(&config, &db).unwrap();

        let m1 = WireRc::per_micron(2.0, 0.5, db.library.routing_layer_id("M1").unwrap());
        assert_eq!(
            wires,
            Wires {
                signal: m1,
                clock: Some(m1)
            }
        );
    }

    #[test]
    fn wires_without_a_clock_wire_estimate_clocks_as_signals() {
        let config = NetlistConfig {
            signal_wire: Some(wire("M1")),
            ..NetlistConfig::default()
        };

        let wires = Wires::from_config(&config, &tiny()).unwrap();

        assert_eq!(wires.clock, None);
    }

    #[test]
    fn wires_need_a_signal_wire() {
        let error = Wires::from_config(&NetlistConfig::default(), &tiny()).unwrap_err();

        assert_eq!(error.to_string(), "[netlist] repair needs signal_wire");
    }

    #[test]
    fn a_wire_on_an_unknown_layer_names_the_layer() {
        let config = NetlistConfig {
            signal_wire: Some(wire("M1")),
            clock_wire: Some(wire("M9")),
            ..NetlistConfig::default()
        };

        let error = Wires::from_config(&config, &tiny()).unwrap_err();

        assert_eq!(
            format!("{error:#}"),
            "[netlist] clock_wire: unknown routing layer 'M9'"
        );
    }
}
