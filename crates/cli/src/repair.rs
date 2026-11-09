//! Netlist repair around placement: buffer removal, port buffering, design
//! repair and timing repair, and the wire parasitics they estimate with.

use crate::phase::{Phase, PhaseTimer};
use crate::plan::{ClockSource, RepairPlan, Wires};
use crate::ui;
use anyhow::Context;
use pare_common::config::Config;
use pare_common::db::design::NetlistDB;
use pare_common::db::liberty::Liberty;
use pare_common::db::parser::{liberty, sdc};
use pare_common::db::sdc::Sdc;

use crate::report::{DesignRepairSummary, NetlistSummary, Report, TimingSummary};
use pare_resizer::Context as RepairContext;
use pare_resizer::{
    BufferSet, ClockNetwork, Constants, HoldOptions, HoldSummary, Link, Namer, Netlist, Repairer,
    Resizer, SessionInputs, SetupOptions, SetupProgress, SetupSummary, Sta, Timing, repair_hold,
    repair_setup,
};
use std::path::{Path, PathBuf};

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
pub(crate) struct NetlistRepair<'p> {
    pub(crate) plan: &'p RepairPlan,
    pub(crate) liberty: Liberty,
    pub(crate) constraints: Constraints<'p>,
    pub(crate) namer: Namer,
    pub(crate) summary: NetlistSummary,
}

impl NetlistRepair<'_> {
    /// The repair as the timed stages work on it, when the input has
    /// constraints.
    pub(crate) const fn timed(&mut self) -> Option<TimedRepair<'_>> {
        let Self {
            plan,
            liberty,
            constraints,
            namer,
            ..
        } = self;
        let Constraints::Sdc(sdc) = constraints else {
            return None;
        };
        Some(TimedRepair {
            plan,
            liberty,
            sdc,
            namer,
        })
    }
}

/// The constraints a netlist repair works under.
pub(crate) enum Constraints<'p> {
    /// The SDC file's, which time the design.
    Sdc(Box<Sdc>),
    /// No timing, the clocks entering at these I/O pins.
    Ports(&'p [String]),
}

impl Constraints<'_> {
    /// Where the clocks come from.
    const fn clocks(&self) -> Clocks<'_> {
        match self {
            Self::Sdc(sdc) => Clocks::Sdc(sdc),
            Self::Ports(ports) => Clocks::Ports(ports),
        }
    }
}

/// A netlist repair whose input has constraints.
pub(crate) struct TimedRepair<'r> {
    pub(crate) plan: &'r RepairPlan,
    pub(crate) liberty: &'r Liberty,
    pub(crate) sdc: &'r mut Sdc,
    pub(crate) namer: &'r mut Namer,
}

impl<'r> TimedRepair<'r> {
    /// The stage `db` is at, its clocks those of the constraints.
    pub(crate) fn stage(&self, db: &NetlistDB) -> anyhow::Result<Stage<'r>> {
        Stage::find(db, self.plan, self.liberty, Clocks::Sdc(self.sdc))
    }
}

/// Where a repair stage finds the clocks.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Clocks<'a> {
    /// Where the SDC file's clocks enter.
    Sdc(&'a Sdc),
    /// At these I/O pins.
    Ports(&'a [String]),
}

/// The pins the clocks reach from where they enter.
fn clock_network(
    netlist: &Netlist<'_>,
    clocks: Clocks<'_>,
    constants: &Constants,
) -> anyhow::Result<ClockNetwork> {
    match clocks {
        Clocks::Sdc(sdc) => Ok(ClockNetwork::from_sdc(netlist, sdc, constants)),
        Clocks::Ports(ports) => Ok(ClockNetwork::from_ports(netlist, ports, constants)?),
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
    /// Links `db` as it stands to `liberty`, without the cells `plan` keeps
    /// repair from, and finds its networks.
    pub(crate) fn find(
        db: &NetlistDB,
        plan: &RepairPlan,
        liberty: &'l Liberty,
        clocks: Clocks<'_>,
    ) -> anyhow::Result<Self> {
        let link = Link::new(db, liberty, &plan.dont_use)?;
        let netlist = Netlist::new(db, &link);
        let constants = Constants::find(&netlist);
        let clocks = clock_network(&netlist, clocks, &constants)?;
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

/// Reads the Liberty files and the constraints `plan` names, then strips
/// the buffers synthesis left and buffers the ports, as
/// OpenROAD-flow-scripts does before placing.
pub(crate) fn repair_netlist<'p>(
    config: &Config,
    plan: &'p RepairPlan,
    db: &mut NetlistDB,
    report: &mut Report,
) -> anyhow::Result<NetlistRepair<'p>> {
    let liberty = load_liberty(&config.input.liberty_files)?;
    let constraints = match &plan.clocks {
        ClockSource::Sdc(path) => {
            ui::header("Input", &path.display().to_string());
            Constraints::Sdc(Box::new(sdc::parse_file(path, db, &liberty)?))
        }
        ClockSource::Ports(ports) => Constraints::Ports(ports),
    };
    let link = Link::new(db, &liberty, &plan.dont_use)?;
    ui::section("Netlist");
    let mut namer = Namer::new();
    let mut summary = NetlistSummary::default();
    if config.netlist.remove_buffers {
        let timer = PhaseTimer::start(Phase::BufferRemoval);
        let before = db.cells.len();
        summary.buffers_removed = pare_resizer::remove_buffers(db, &link, sdc.as_ref())?;
        ui::stat_row(
            "Removed:",
            &summary.buffers_removed.to_string(),
            "Cells:",
            &format!("{before} -> {}", db.cells.len()),
        );
        timer.finish(report);
    }
    if config.netlist.buffer_ports {
        let timer = PhaseTimer::start(Phase::PortBuffering);
        let stage = Stage::find(db, plan, &liberty, constraints.clocks())?;
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
        timer.finish(report);
    }
    report.netlist = Some(summary.clone());
    Ok(NetlistRepair {
        plan,
        liberty,
        constraints,
        namer,
        summary,
    })
}

/// Resizes drivers and inserts repeaters where the placed netlist breaks
/// the library's slew or load limits, as OpenROAD-flow-scripts'
/// `repair_design` does after global placement.
pub(crate) fn repair_design(
    config: &Config,
    db: &mut NetlistDB,
    repair: &mut NetlistRepair<'_>,
    report: &mut Report,
) -> anyhow::Result<()> {
    if !config.netlist.repair_design {
        return Ok(());
    }
    let timer = PhaseTimer::start(Phase::DesignRepair);
    let clocks = repair.constraints.clocks();
    let stage = Stage::find(db, repair.plan, &repair.liberty, clocks)?;
    let inputs = stage.session(db, repair.plan.wires, &mut repair.namer);
    let done = match clocks {
        Clocks::Sdc(sdc) => repair_design_on(Resizer::timed(inputs, sdc)?)?,
        Clocks::Ports(_) => repair_design_on(Resizer::new(inputs)?)?,
    };
    repair.summary.design = DesignRepairSummary::of(&done);
    report.netlist = Some(repair.summary.clone());
    timer.finish(report);
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
    repair: &mut NetlistRepair<'_>,
    report: &mut Report,
) -> anyhow::Result<bool> {
    if !config.netlist.repair_timing {
        return Ok(false);
    }
    let Some(timed) = repair.timed() else {
        return Ok(false);
    };
    let timer = PhaseTimer::start(Phase::TimingRepair);
    let stage = timed.stage(db)?;
    let mut resizer = Resizer::timed(stage.session(db, timed.plan.wires, timed.namer), timed.sdc)?;
    report.timing = Some(timing_repair_on(&mut resizer, timed.liberty)?);
    let _ = resizer.finish()?;
    timer.finish(report);
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{cts_fixture, scratch_dir, tiny};
    use pare_common::db::design::PinOwner;
    use pare_common::db::indices::PinId;

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
    /// the clock library, for the SDC `constraints` when given, else the
    /// clock `ports`.
    fn tiny_clock_network(
        ports: &[String],
        constraints: Option<&str>,
    ) -> (NetlistDB, anyhow::Result<ClockNetwork>) {
        let db = tiny();
        let liberty = load_liberty(&[clock_lib()]).unwrap();
        let sdc = constraints
            .map(|text| sdc::parse_str(text, Path::new("tiny.sdc"), &db, &liberty).unwrap());
        let clocks = sdc.as_ref().map_or(Clocks::Ports(ports), Clocks::Sdc);
        let link = Link::new(&db, &liberty, &[]).unwrap();
        let netlist = Netlist::new(&db, &link);
        let constants = Constants::find(&netlist);
        let network = clock_network(&netlist, clocks, &constants);
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
        let (db, network) = tiny_clock_network(&["in".to_string()], None);

        assert!(network.unwrap().is_clock(u1_input(&db)));
    }

    #[test]
    fn without_clock_ports_or_an_sdc_file_nothing_is_a_clock() {
        let (db, network) = tiny_clock_network(&[], None);

        assert!(!network.unwrap().is_clock(u1_input(&db)));
    }

    #[test]
    fn an_unknown_clock_port_is_an_error() {
        let (_, network) = tiny_clock_network(&["clk".to_string()], None);

        assert_eq!(
            network.unwrap_err().to_string(),
            "clock port 'clk' is not an I/O pin of the design"
        );
    }

    #[test]
    fn with_an_sdc_file_its_clocks_are_the_clocks() {
        let (db, network) = tiny_clock_network(&[], Some(CLOCK_ON_IN));

        assert!(network.unwrap().is_clock(u1_input(&db)));
    }

    #[test]
    fn no_liberty_files_is_an_error() {
        let error = load_liberty(&[]).unwrap_err();

        assert_eq!(error.to_string(), "no Liberty files");
    }
}
