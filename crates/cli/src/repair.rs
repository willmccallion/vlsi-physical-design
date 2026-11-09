//! Netlist repair around placement: buffer removal, port buffering, design
//! repair and timing repair, and the wire parasitics they estimate with.

use crate::ui;
use anyhow::Context;
use pare_common::config::{Config, LayerRcConfig, NetlistConfig};
use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::{liberty, sdc};
use pare_common::db::sdc::Sdc;

use crate::report::{DesignRepairSummary, NetlistSummary, Report, TimingSummary};
use pare_resizer::Context as RepairContext;
use pare_resizer::WireRc;
use pare_resizer::{
    BufferSet, ClockNetwork, Constants, HoldOptions, HoldSummary, Link, Namer, Netlist, Repairer,
    Resizer, SessionInputs, SetupOptions, SetupProgress, SetupSummary, Sta, Timing, repair_hold,
    repair_setup,
};
use std::path::{Path, PathBuf};
use std::time::Instant;

/// Reads the Liberty files, the first one's thresholds and defaults
/// standing for all.
fn load_liberty(paths: &[PathBuf]) -> anyhow::Result<Liberty> {
    let mut merged: Option<Liberty> = None;
    for path in paths {
        ui::header("Input", &path.display().to_string());
        let lib = liberty::parse_file(path)?;
        match &mut merged {
            None => merged = Some(lib),
            Some(first) => first
                .absorb(lib)
                .map_err(|e| duplicate_cell(first, paths, path, &e.name))?,
        }
    }
    merged.context("no Liberty files")
}

/// The error for cell `name` of the Liberty file `path`, which `merged`,
/// read from the files `paths` before it, already has.
fn duplicate_cell(merged: &Liberty, paths: &[PathBuf], path: &Path, name: &str) -> anyhow::Error {
    let earlier = merged
        .cell_id(name)
        .and_then(|id| paths.get(merged.cell(id).library))
        .map_or_else(
            || "an earlier Liberty file".to_string(),
            |earlier| earlier.display().to_string(),
        );
    anyhow::anyhow!(
        "cell '{name}' of {} is already in {earlier}",
        path.display()
    )
}

/// What the netlist steps before placement leave for the repair after it.
pub(crate) struct NetlistRepair {
    pub(crate) liberty: Liberty,
    pub(crate) wires: Wires,
    pub(crate) sdc: Option<Sdc>,
    pub(crate) namer: Namer,
    pub(crate) summary: NetlistSummary,
}

impl NetlistRepair {
    /// The repair as the timed stages work on it, when the input has
    /// constraints.
    pub(crate) fn timed(&mut self) -> Option<TimedRepair<'_>> {
        let Self {
            liberty,
            wires,
            sdc,
            namer,
            ..
        } = self;
        Some(TimedRepair {
            liberty,
            wires: *wires,
            sdc: sdc.as_mut()?,
            namer,
        })
    }
}

/// A netlist repair whose input has constraints.
pub(crate) struct TimedRepair<'r> {
    pub(crate) liberty: &'r Liberty,
    pub(crate) wires: Wires,
    pub(crate) sdc: &'r mut Sdc,
    pub(crate) namer: &'r mut Namer,
}

impl<'r> TimedRepair<'r> {
    /// The stage `db` is at, its clocks those of the constraints.
    pub(crate) fn stage(&self, config: &Config, db: &NetlistDB) -> anyhow::Result<Stage<'r>> {
        Stage::find(config, db, self.liberty, Some(self.sdc))
    }
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

/// The design as a repair stage finds it: its masters linked to the
/// library, the constant pins and clock network repair leaves alone, and the
/// buffers it may insert.
pub(crate) struct Stage<'l> {
    pub(crate) link: Link<'l>,
    pub(crate) constants: Constants,
    pub(crate) clocks: ClockNetwork,
    pub(crate) buffers: BufferSet,
}

impl<'l> Stage<'l> {
    /// Links `db` as it stands to `liberty` and finds its networks.
    pub(crate) fn find(
        config: &Config,
        db: &NetlistDB,
        liberty: &'l Liberty,
        sdc: Option<&Sdc>,
    ) -> anyhow::Result<Self> {
        let link = Link::new(db, liberty, &config.netlist.dont_use)?;
        let netlist = Netlist::new(db, &link);
        let constants = Constants::find(&netlist);
        let clocks = clock_network(&netlist, config, sdc, &constants)?;
        let buffers = link.find_buffers(db)?;
        Ok(Self {
            link,
            constants,
            clocks,
            buffers,
        })
    }

    /// The inputs of a resizer session over `db` at this stage.
    pub(crate) fn session<'a>(
        &'a self,
        db: &'a mut NetlistDB,
        wires: Wires,
        namer: &'a mut Namer,
    ) -> SessionInputs<'a> {
        SessionInputs {
            db,
            context: RepairContext {
                link: &self.link,
                clocks: &self.clocks,
                constants: &self.constants,
                clock_rc: wires.clock,
            },
            rc: wires.signal,
            buffers: self.buffers.clone(),
            namer,
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
            ui::header("Input", &path.display().to_string());
            Some(sdc::parse_file(path, db, &liberty)?)
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
        summary.buffers_removed = pare_resizer::remove_buffers(db, &link, sdc.as_ref())?;
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
        let stage = Stage::find(config, db, &liberty, sdc.as_ref())?;
        let done = pare_resizer::buffer_ports(
            db,
            &stage.link,
            &stage.buffers,
            &stage.clocks,
            &stage.constants,
            &mut namer,
        )?;
        summary.input_buffers = done.inputs;
        summary.output_buffers = done.outputs;
        summary
            .buffer_cell
            .clone_from(&liberty.cell(stage.buffers.lowest_drive.id).name);
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
            &stage.clocks.sources().len().to_string(),
        );
        report.phase("port buffering", phase_start);
    }
    report.netlist = Some(summary.clone());
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
    let stage = Stage::find(config, db, &repair.liberty, repair.sdc.as_ref())?;
    let inputs = stage.session(db, repair.wires, &mut repair.namer);
    let done = match repair.sdc.as_ref() {
        Some(sdc) => repair_design_on(Resizer::timed(inputs, sdc)?)?,
        None => repair_design_on(Resizer::new(inputs)?)?,
    };
    repair.summary.design = DesignRepairSummary::of(&done);
    report.netlist = Some(repair.summary.clone());
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
    if !config.netlist.repair_timing {
        return Ok(false);
    }
    let Some(timed) = repair.timed() else {
        return Ok(false);
    };
    let phase_start = Instant::now();
    ui::phase("Timing Repair");
    let stage = timed.stage(config, db)?;
    let mut resizer = Resizer::timed(stage.session(db, timed.wires, timed.namer), timed.sdc)?;
    report.timing = Some(timing_repair_on(&mut resizer, timed.liberty)?);
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
    let setup = repair_setup(resizer, SetupOptions::default(), &mut show_setup_progress)?;
    let hold = repair_hold(resizer, &HoldOptions::default())?;
    eprintln!();
    show_timing_repair(&setup, &hold, liberty);
    Ok(TimingSummary::of(&setup, &hold))
}

/// Shows one row of setup repair's progress table.
fn show_setup_progress(progress: &SetupProgress) {
    let iter = progress
        .iteration
        .map_or_else(|| "final".to_string(), |i| i.to_string());
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
        worst: progress.worst.as_deref().unwrap_or(""),
    });
}

/// Shows what setup and hold repair found and did, naming hold's buffer
/// from `liberty`.
fn show_timing_repair(setup: &SetupSummary, hold: &HoldSummary, liberty: &Liberty) {
    ui::stat_row(
        "WNS (ns):",
        &format!(
            "{:.3} -> {:.3}",
            setup.wns_before.nanos(),
            setup.wns_after.nanos()
        ),
        "TNS (ns):",
        &format!(
            "{:.1} -> {:.1}",
            setup.tns_before.nanos(),
            setup.tns_after.nanos()
        ),
    );
    ui::stat_row(
        "Failing endpoints:",
        &format!("{} -> {}", setup.failing_before, setup.failing_after),
        "Iterations:",
        &setup.iterations.to_string(),
    );
    ui::stat_row(
        "Removed:",
        &setup.removed.to_string(),
        "Resized:",
        &setup.resized.to_string(),
    );
    ui::stat_row(
        "Inserted:",
        &setup.inserted.to_string(),
        "Cloned:",
        &setup.cloned.to_string(),
    );
    ui::stat_row(
        "Pin swaps:",
        &setup.swapped.to_string(),
        "Area:",
        &format!("{:+.1}%", setup.area_change_percent),
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
    use crate::testing::{cts_fixture, scratch_dir, tiny, tiny_config};
    use pare_common::db::design::PinOwner;
    use pare_common::db::indices::PinId;

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

    fn clock_lib() -> PathBuf {
        cts_fixture("clock.lib")
    }

    #[test]
    fn one_liberty_file_gives_its_cells() {
        let liberty = load_liberty(&[clock_lib()]).unwrap();

        assert!(liberty.cell_id("DFF_X1").is_some());
    }

    #[test]
    fn liberty_files_merge_their_cells() {
        let renamed = scratch_dir("liberty-merge").join("renamed.lib");
        let text = std::fs::read_to_string(clock_lib()).unwrap();
        std::fs::write(&renamed, text.replace("  cell (", "  cell (B_")).unwrap();

        let liberty = load_liberty(&[clock_lib(), renamed]).unwrap();

        assert!(liberty.cell_id("DFF_X1").is_some());
        assert!(liberty.cell_id("B_DFF_X1").is_some());
    }

    #[test]
    fn a_cell_in_two_liberty_files_names_both() {
        let dir = scratch_dir("liberty-twice");
        let text = std::fs::read_to_string(clock_lib()).unwrap();
        let renamed = text.replace("  cell (", "  cell (B_");
        let (second, third) = (dir.join("b.lib"), dir.join("c.lib"));
        std::fs::write(&second, &renamed).unwrap();
        std::fs::write(&third, &renamed).unwrap();

        let error = load_liberty(&[clock_lib(), second.clone(), third.clone()]).unwrap_err();

        assert_eq!(
            error.to_string(),
            format!(
                "cell 'B_CLKBUF_X3' of {} is already in {}",
                third.display(),
                second.display()
            )
        );
    }

    #[test]
    fn a_cell_of_the_first_liberty_file_again_names_the_first() {
        let error = load_liberty(&[clock_lib(), clock_lib()]).unwrap_err();

        assert_eq!(
            error.to_string(),
            format!(
                "cell 'CLKBUF_X3' of {} is already in {}",
                clock_lib().display(),
                clock_lib().display()
            )
        );
    }

    /// The clock network `clock_network` finds in the tiny design, linked to
    /// the clock library, for `config` and the SDC `constraints`.
    fn tiny_clock_network(
        config: &Config,
        constraints: Option<&str>,
    ) -> (NetlistDB, anyhow::Result<ClockNetwork>) {
        let db = tiny();
        let liberty = load_liberty(&[clock_lib()]).unwrap();
        let sdc = constraints
            .map(|text| sdc::parse_str(text, Path::new("tiny.sdc"), &db, &liberty).unwrap());
        let link = Link::new(&db, &liberty, &[]).unwrap();
        let netlist = Netlist::new(&db, &link);
        let constants = Constants::find(&netlist);
        let network = clock_network(&netlist, config, sdc.as_ref(), &constants);
        (db, network)
    }

    /// The input pin of `u1`, which the net of port `in` drives.
    fn u1_input(db: &NetlistDB) -> PinId {
        let u1 = db.cell_id("u1").unwrap();
        let net = db.net_id("in").unwrap();
        db.nets[net]
            .pins
            .iter()
            .copied()
            .find(|&p| matches!(db.pins[p].owner, PinOwner::Cell { cell, .. } if cell == u1))
            .unwrap()
    }

    const CLOCK_ON_IN: &str = "create_clock -name clk -period 10 [get_ports in]";

    #[test]
    fn without_an_sdc_file_the_clock_ports_are_the_clocks() {
        let config = tiny_config("[netlist]\nclock_ports = [\"in\"]");

        let (db, network) = tiny_clock_network(&config, None);

        assert!(network.unwrap().is_clock(u1_input(&db)));
    }

    #[test]
    fn without_clock_ports_or_an_sdc_file_nothing_is_a_clock() {
        let (db, network) = tiny_clock_network(&tiny_config(""), None);

        assert!(!network.unwrap().is_clock(u1_input(&db)));
    }

    #[test]
    fn an_unknown_clock_port_is_an_error() {
        let config = tiny_config("[netlist]\nclock_ports = [\"clk\"]");

        let (_, network) = tiny_clock_network(&config, None);

        assert_eq!(
            network.unwrap_err().to_string(),
            "clock port 'clk' is not an I/O pin of the design"
        );
    }

    #[test]
    fn with_an_sdc_file_its_clocks_are_the_clocks() {
        let (db, network) = tiny_clock_network(&tiny_config(""), Some(CLOCK_ON_IN));

        assert!(network.unwrap().is_clock(u1_input(&db)));
    }

    #[test]
    fn clock_ports_beside_an_sdc_file_are_an_error() {
        let config = tiny_config("[netlist]\nclock_ports = [\"in\"]");

        let (_, network) = tiny_clock_network(&config, Some(CLOCK_ON_IN));

        assert_eq!(
            network.unwrap_err().to_string(),
            "[netlist] clock_ports must be empty when [input] sdc_file is set"
        );
    }

    #[test]
    fn no_liberty_files_is_an_error() {
        let error = load_liberty(&[]).unwrap_err();

        assert_eq!(error.to_string(), "no Liberty files");
    }
}
