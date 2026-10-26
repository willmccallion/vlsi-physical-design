//! The moves setup repair tries on a driver of a failing path.
//!
//! As `OpenROAD`'s `BaseMove` family: taking a buffer out, sizing a gate
//! up, swapping equivalent input pins, rebuffering the net, cloning the
//! gate, and splitting its loads behind a buffer.

use crate::bnet::{self, Bnet, BnetPtr, Kind};
use crate::fuzzy;
use crate::rebuffer::{Rebuffer, can_remove_buffer};
use crate::resizer::{RepairError, Resizer, buffer_delays, gate_delays};
use crate::sta::Path;
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{CellId, LayerId, NetId, PinId};
use pare_common::db::liberty::{
    Edge, EdgeValues, FuncExpr, LibCell, LibCellId, LibPort, Logic, MinMax,
};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use rustc_hash::{FxHashMap, FxHashSet};
use std::rc::Rc;

/// Fanout above which a net is cloned or split rather than rebuffered.
const SPLIT_LOAD_MIN_FANOUT: usize = 8;

/// Fanout at and above which rebuffering is not tried.
const REBUFFER_MAX_FANOUT: usize = 20;

/// Fanout a driver may take on when a buffer is removed, absent a limit.
const BUFFER_REMOVAL_MAX_FANOUT: usize = 10;

/// Inputs beyond which pin equivalence is not simulated.
const MAX_SIMULATED_INPUTS: usize = 16;

/// How much worse an already failing side input may get when a buffer is
/// removed, as a share of its violation.
const SLACK_DEGRADE_RATIO_LIMIT: f64 = 0.1;

/// A kind of move.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MoveKind {
    /// Take a buffer out.
    Unbuffer,
    /// Size a gate up.
    SizeUp,
    /// Size a buffer or inverter up to its driver's size.
    SizeUpMatch,
    /// Swap equivalent input pins.
    SwapPins,
    /// Rebuffer the net.
    Buffer,
    /// Clone the gate.
    Clone,
    /// Split the loads behind a buffer.
    SplitLoad,
}

impl MoveKind {
    /// Every kind, in a fixed order.
    pub const ALL: [Self; 7] = [
        Self::Unbuffer,
        Self::SizeUp,
        Self::SizeUpMatch,
        Self::SwapPins,
        Self::Buffer,
        Self::Clone,
        Self::SplitLoad,
    ];

    /// `OpenROAD`'s name for the move.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Unbuffer => "UnbufferMove",
            Self::SizeUp => "SizeUpMove",
            Self::SizeUpMatch => "SizeUpMatchMove",
            Self::SwapPins => "SwapPinsMove",
            Self::Buffer => "BufferMove",
            Self::Clone => "CloneMove",
            Self::SplitLoad => "SplitLoadMove",
        }
    }

    const fn index(self) -> usize {
        match self {
            Self::Unbuffer => 0,
            Self::SizeUp => 1,
            Self::SizeUpMatch => 2,
            Self::SwapPins => 3,
            Self::Buffer => 4,
            Self::Clone => 5,
            Self::SplitLoad => 6,
        }
    }
}

/// How many moves of one kind were made, and to which cells.
#[derive(Clone, Debug, Default)]
struct Tally {
    pending: usize,
    committed: usize,
    pending_cells: FxHashSet<CellId>,
    all_cells: FxHashSet<CellId>,
}

/// The moves made so far, per kind, as `OpenROAD` counts them.
#[derive(Clone, Debug, Default)]
pub struct MoveLog {
    tallies: [Tally; 7],
}

impl MoveLog {
    fn count(&mut self, kind: MoveKind, cell: CellId, count: usize) {
        let tally = &mut self.tallies[kind.index()];
        tally.pending += count;
        let _ = tally.pending_cells.insert(cell);
        let _ = tally.all_cells.insert(cell);
    }

    /// Whether a move of `kind` ever touched `cell`.
    #[must_use]
    pub fn has_moves(&self, kind: MoveKind, cell: CellId) -> bool {
        self.tallies[kind.index()].all_cells.contains(&cell)
    }

    /// Whether a move of `kind` touched `cell` since the last commit.
    #[must_use]
    pub fn has_pending(&self, kind: MoveKind, cell: CellId) -> bool {
        self.tallies[kind.index()].pending_cells.contains(&cell)
    }

    /// Moves of `kind` kept so far.
    #[must_use]
    pub const fn committed(&self, kind: MoveKind) -> usize {
        self.tallies[kind.index()].committed
    }

    /// Moves of `kind` kept or pending.
    #[must_use]
    pub const fn made(&self, kind: MoveKind) -> usize {
        self.tallies[kind.index()].committed + self.tallies[kind.index()].pending
    }

    /// Keeps the pending moves, as `commitMoves`.
    pub fn commit(&mut self) {
        for tally in &mut self.tallies {
            tally.committed += tally.pending;
            tally.pending = 0;
            tally.pending_cells.clear();
        }
    }

    /// Drops the pending moves, as `undoMoves`.
    pub fn undo(&mut self) {
        for tally in &mut self.tallies {
            tally.pending = 0;
            tally.pending_cells.clear();
        }
    }
}

/// The move set with what it remembers between moves.
#[derive(Debug)]
pub struct Moves {
    /// The moves made.
    pub log: MoveLog,
    /// The rebuffering search.
    pub rebuffer: Rebuffer,
    equivalent_pins: FxHashMap<(LibCellId, usize), Vec<usize>>,
}

/// The driver a move works on: where it sits on the path.
#[derive(Clone, Copy, Debug)]
struct Site {
    driver: PinId,
    cell: CellId,
    lib: LibCellId,
    port: usize,
    edge: Edge,
    input: Option<PinId>,
    prev_driver: Option<PinId>,
    prev_edge: Option<Edge>,
}

impl Moves {
    /// Prepares the moves over the session.
    pub fn new(resizer: &Resizer<'_>) -> Result<Self, RepairError> {
        Ok(Self {
            log: MoveLog::default(),
            rebuffer: Rebuffer::new(resizer)?,
            equivalent_pins: FxHashMap::default(),
        })
    }

    /// Tries `kind` on the driver at `index` of `path`; true when the
    /// design changed.
    pub fn try_move(
        &mut self,
        resizer: &mut Resizer<'_>,
        kind: MoveKind,
        path: &Path,
        index: usize,
        margin: f64,
    ) -> Result<bool, RepairError> {
        let Some(site) = site(resizer, path, index) else {
            return Ok(false);
        };
        if resizer.netlist().is_retired(site.driver) {
            return Ok(false);
        }
        match kind {
            MoveKind::Unbuffer => self.unbuffer(resizer, &site, margin),
            MoveKind::SizeUp => self.size_up(resizer, &site),
            MoveKind::SizeUpMatch => self.size_up_match(resizer, &site),
            MoveKind::SwapPins => Ok(self.swap_pins(resizer, &site)),
            MoveKind::Buffer => self.buffer(resizer, &site),
            MoveKind::Clone => self.clone_gate(resizer, &site),
            MoveKind::SplitLoad => self.split_load(resizer, &site),
        }
    }

    /// `SizeUpMove`: the weakest stronger size that is also faster with
    /// its extra input load on the previous driver.
    fn size_up(&mut self, resizer: &mut Resizer<'_>, site: &Site) -> Result<bool, RepairError> {
        if !resizer.is_editable(site.cell) {
            return Ok(false);
        }
        let Some(input) = site.input else {
            return Ok(false);
        };
        let Some((_, in_port)) = resizer.link.pin_port(resizer.db, input) else {
            return Ok(false);
        };
        let prev_drive = site
            .prev_driver
            .and_then(|p| resizer.link.pin_port(resizer.db, p))
            .map_or(0.0, |(lib, port)| {
                resizer.link.liberty.cell(lib).drive_resistance(port)
            });
        let load_cap = resizer.load_cap(site.driver);
        let Some(upsize) = upsize_cell(resizer, site.lib, in_port, site.port, load_cap, prev_drive)
        else {
            return Ok(false);
        };
        if !resizer.replacement_keeps_loads(site.cell, upsize) {
            return Ok(false);
        }
        if !resizer.replace_cell(site.cell, upsize)? {
            return Ok(false);
        }
        self.log.count(MoveKind::SizeUp, site.cell, 1);
        Ok(true)
    }

    /// `SizeUpMatchMove`: a buffer or inverter driven by a lone stronger
    /// buffer or inverter takes the driver's size.
    fn size_up_match(
        &mut self,
        resizer: &mut Resizer<'_>,
        site: &Site,
    ) -> Result<bool, RepairError> {
        let Some(prev_driver) = site.prev_driver else {
            return Ok(false);
        };
        if !resizer.is_editable(site.cell) {
            return Ok(false);
        }
        let PinOwner::Cell {
            cell: prev_cell, ..
        } = resizer.db.pins[prev_driver.index()].owner
        else {
            return Ok(false);
        };
        let Some((prev_lib, prev)) = resizer.link.cell(resizer.db, prev_cell) else {
            return Ok(false);
        };
        let lib = resizer.link.liberty.cell(site.lib);
        if prev_lib == site.lib || resizer.fanout(prev_driver) > 1 {
            return Ok(false);
        }
        let both_buffers = prev.is_buffer() && lib.is_buffer();
        let both_inverters = prev.is_inverter() && lib.is_inverter();
        if !both_buffers && !both_inverters {
            return Ok(false);
        }
        if resizer.link.buffer_drive_resistance(prev_lib)
            >= resizer.link.buffer_drive_resistance(site.lib)
        {
            return Ok(false);
        }
        if !resizer.replacement_keeps_loads(site.cell, prev_lib) {
            return Ok(false);
        }
        if !resizer.replace_cell(site.cell, prev_lib)? {
            return Ok(false);
        }
        self.log.count(MoveKind::SizeUpMatch, site.cell, 1);
        Ok(true)
    }

    /// `SwapPinsMove`: moves the critical input to an equivalent, faster
    /// pin of the same gate.
    fn swap_pins(&mut self, resizer: &mut Resizer<'_>, site: &Site) -> bool {
        if self.log.has_moves(MoveKind::SwapPins, site.cell) {
            return false;
        }
        let lib = resizer.link.liberty.cell(site.lib);
        if lib.dont_touch || lib.is_buffer() || lib.is_inverter() {
            return false;
        }
        let Some(input) = site.input else {
            return false;
        };
        let Some((_, in_port)) = resizer.link.pin_port(resizer.db, input) else {
            return false;
        };
        if !lib.ports[in_port].is_input() {
            return false;
        }
        let equivalent = self
            .equivalent_pins
            .entry((site.lib, in_port))
            .or_insert_with(|| equivalent_pins(lib, in_port))
            .clone();
        if equivalent.is_empty() {
            return false;
        }
        let load_cap = resizer.load_cap(site.driver);
        let input_slews = resizer.input_slews(site.cell);
        let Some(swap_port) = swap_candidate(
            resizer,
            lib,
            in_port,
            site.port,
            &equivalent,
            load_cap,
            &input_slews,
        ) else {
            return false;
        };
        let netlist = resizer.netlist();
        let (Some(pin1), Some(pin2)) = (
            netlist.pin_of_port(site.cell, in_port),
            netlist.pin_of_port(site.cell, swap_port),
        ) else {
            return false;
        };
        let net1 = resizer.db.pins[pin1.index()].net;
        let net2 = resizer.db.pins[pin2.index()].net;
        resizer.move_pin(pin1, net2);
        resizer.move_pin(pin2, net1);
        resizer.update_net(net1);
        resizer.update_net(net2);
        self.log.count(MoveKind::SwapPins, site.cell, 1);
        true
    }

    /// `BufferMove`: rebuffers a net of modest fanout.
    fn buffer(&mut self, resizer: &mut Resizer<'_>, site: &Site) -> Result<bool, RepairError> {
        let fanout = resizer.fanout(site.driver);
        if fanout <= 1 || fanout >= REBUFFER_MAX_FANOUT || !resizer.ok_to_buffer_net(site.driver) {
            return Ok(false);
        }
        let inserted = self.rebuffer.rebuffer_pin(resizer, site.driver)?;
        if inserted == 0 {
            return Ok(false);
        }
        self.log.count(MoveKind::Buffer, site.cell, inserted);
        Ok(true)
    }

    /// `CloneMove`: a second copy of a single-output gate takes the half of
    /// its loads with the most slack to spare.
    fn clone_gate(&mut self, resizer: &mut Resizer<'_>, site: &Site) -> Result<bool, RepairError> {
        if resizer.fanout(site.driver) <= SPLIT_LOAD_MIN_FANOUT
            || !resizer.ok_to_buffer_net(site.driver)
            || self.log.has_pending(MoveKind::Buffer, site.cell)
            || self.log.has_pending(MoveKind::SplitLoad, site.cell)
            || !is_single_output_combinational(resizer, site.cell, site.lib)
        {
            return Ok(false);
        }
        let Some(sta) = resizer.sta() else {
            return Ok(false);
        };
        let net = resizer.db.pins[site.driver.index()].net;
        let driver_slack = sta.slack_edge(site.driver, site.edge, MinMax::Max);
        let mut fanouts: Vec<(f64, String, PinId)> = resizer
            .netlist()
            .loads(net)
            .filter(|&l| l != site.driver)
            .map(|load| {
                let slack = sta.slack_edge(load, site.edge, MinMax::Max);
                (slack - driver_slack, resizer.db.pin_name(load), load)
            })
            .collect();
        fanouts.sort_by(|a, b| b.0.total_cmp(&a.0).then_with(|| a.1.cmp(&b.1)));
        let split = fanouts.len() / 2;
        let moving: Vec<PinId> = fanouts[..split]
            .iter()
            .map(|(_, _, pin)| *pin)
            .filter(|&p| matches!(resizer.db.pins[p.index()].owner, PinOwner::Cell { .. }))
            .collect();
        let clone_lib = half_driving_power_cell(resizer, site.lib).unwrap_or(site.lib);
        let location = {
            let netlist = resizer.netlist();
            let mut sum = netlist.pin_location(site.driver);
            let mut count = 1i64;
            for (_, _, load) in &fanouts[..split] {
                let at = netlist.pin_location(*load);
                sum = Point::new(Dbu(sum.x.get() + at.x.get()), Dbu(sum.y.get() + at.y.get()));
                count += 1;
            }
            Point::new(Dbu(sum.x.get() / count), Dbu(sum.y.get() / count))
        };
        let clone = resizer.make_instance(clone_lib, location, "clone")?;
        let original = resizer.link.liberty.cell(site.lib);
        let clone_cell = resizer.link.liberty.cell(clone_lib);
        let mut touched: Vec<NetId> = Vec::new();
        for &pin in &resizer.db.cells[site.cell.index()].pins.clone() {
            let Some((_, port)) = resizer.link.pin_port(resizer.db, pin) else {
                continue;
            };
            if !original.ports[port].is_input() {
                continue;
            }
            let Some(clone_port) = clone_cell.port(&original.ports[port].name) else {
                continue;
            };
            let input_net = resizer.db.pins[pin.index()].net;
            let _ = resizer.connect(clone, clone_lib, clone_port, input_net);
            touched.push(input_net);
        }
        let out_net = resizer.make_net("net", net)?;
        let Some(out_port) = clone_cell.ports.iter().position(LibPort::is_output) else {
            return Ok(false);
        };
        let _ = resizer.connect(clone, clone_lib, out_port, out_net);
        for load in moving {
            resizer.move_pin(load, out_net);
        }
        resizer.update_net(net);
        resizer.update_net(out_net);
        for input_net in touched {
            resizer.update_net(input_net);
        }
        self.log.count(MoveKind::Clone, clone, 1);
        self.log.count(MoveKind::Clone, site.cell, 0);
        Ok(true)
    }

    /// `SplitLoadMove`: the half of the loads with the most slack to spare
    /// move behind a buffer at the driver.
    fn split_load(&mut self, resizer: &mut Resizer<'_>, site: &Site) -> Result<bool, RepairError> {
        if resizer.fanout(site.driver) <= SPLIT_LOAD_MIN_FANOUT
            || !resizer.ok_to_buffer_net(site.driver)
        {
            return Ok(false);
        }
        let Some(sta) = resizer.sta() else {
            return Ok(false);
        };
        let net = resizer.db.pins[site.driver.index()].net;
        let driver_slack = sta.slack(site.driver, MinMax::Max);
        let mut fanouts: Vec<(f64, String, PinId)> = resizer
            .netlist()
            .loads(net)
            .filter(|&l| l != site.driver)
            .map(|load| {
                let slack = sta.slack_edge(load, site.edge, MinMax::Max);
                (slack - driver_slack, resizer.db.pin_name(load), load)
            })
            .collect();
        fanouts.sort_by(|a, b| b.0.total_cmp(&a.0).then_with(|| a.1.cmp(&b.1)));
        let split = fanouts.len() / 2;
        let moving: Vec<PinId> = fanouts[..split]
            .iter()
            .map(|(_, _, pin)| *pin)
            .filter(|&p| matches!(resizer.db.pins[p.index()].owner, PinOwner::Cell { .. }))
            .collect();
        if moving.is_empty() {
            return Ok(false);
        }
        let location = resizer.netlist().pin_location(site.driver);
        let buffer = resizer.buffers.lowest_drive;
        let inserted =
            resizer.insert_buffer_before_loads(net, &moving, buffer, location, "split")?;
        let _ = resizer.resize_to_target_slew(inserted.output)?;
        self.log.count(MoveKind::SplitLoad, inserted.cell, 1);
        Ok(true)
    }

    /// `UnbufferMove`: takes a buffer on the path out when its driver can
    /// take the loads without hurting the paths through them.
    fn unbuffer(
        &mut self,
        resizer: &mut Resizer<'_>,
        site: &Site,
        margin: f64,
    ) -> Result<bool, RepairError> {
        let lib = resizer.link.liberty.cell(site.lib);
        if !lib.is_buffer() {
            return Ok(false);
        }
        for kind in [
            MoveKind::SwapPins,
            MoveKind::Clone,
            MoveKind::SplitLoad,
            MoveKind::Buffer,
            MoveKind::SizeUp,
        ] {
            if self.log.has_moves(kind, site.cell) {
                return Ok(false);
            }
        }
        let (Some(input), Some(prev_driver)) = (site.input, site.prev_driver) else {
            return Ok(false);
        };
        let prev_edge = site.prev_edge.unwrap_or(site.edge);
        let new_fanout = resizer.fanout(prev_driver) + resizer.fanout(site.driver) - 1;
        let fanout_limit = resizer
            .fanout_limit()
            .map_or(BUFFER_REMOVAL_MAX_FANOUT, |l| l as usize);
        if new_fanout > fanout_limit {
            return Ok(false);
        }
        let Some((in_port, _)) = lib.buffer_ports() else {
            return Ok(false);
        };
        let buffer_in_cap = lib.ports[in_port].capacitance();
        let buffer_load = resizer.load_cap(site.driver);
        if let Some(limit) = resizer.cap_limit(prev_driver)
            && limit > 0.0
        {
            let new_cap = resizer.load_cap(prev_driver) + buffer_load - buffer_in_cap;
            if new_cap > limit {
                return Ok(false);
            }
        }
        if !estimated_slack_ok(resizer, site, input, prev_driver, prev_edge, margin) {
            return Ok(false);
        }
        if !can_remove_buffer(resizer, site.cell) {
            return Ok(false);
        }
        let _ = resizer.remove_buffer(site.cell)?;
        self.log.count(MoveKind::Unbuffer, site.cell, 1);
        Ok(true)
    }
}

/// Where the driver at `index` of `path` sits.
fn site(resizer: &Resizer<'_>, path: &Path, index: usize) -> Option<Site> {
    let point = path.points.get(index)?;
    let PinOwner::Cell { cell, .. } = resizer.db.pins[point.pin.index()].owner else {
        return None;
    };
    let (lib, port) = resizer.link.pin_port(resizer.db, point.pin)?;
    let input = (index >= 1).then(|| path.points[index - 1]);
    let prev = (index >= 2).then(|| path.points[index - 2]);
    Some(Site {
        driver: point.pin,
        cell,
        lib,
        port,
        edge: point.edge,
        input: input.map(|p| p.pin),
        prev_driver: prev.map(|p| p.pin),
        prev_edge: prev.map(|p| p.edge),
    })
}

/// The weakest size of `lib` that drives harder and, with the extra input
/// load on the previous driver, settles sooner than the present one, as
/// `BaseMove::upsizeCell`.
fn upsize_cell(
    resizer: &Resizer<'_>,
    lib: LibCellId,
    in_port: usize,
    drvr_port: usize,
    load_cap: f64,
    prev_drive: f64,
) -> Option<LibCellId> {
    let liberty = resizer.link.liberty;
    let cell = liberty.cell(lib);
    let in_name = &cell.ports[in_port].name;
    let out_name = &cell.ports[drvr_port].name;
    let mut candidates: Vec<(LibCellId, usize, usize)> = resizer
        .link
        .swappable_cells(lib)
        .into_iter()
        .filter_map(|id| {
            let c = liberty.cell(id);
            Some((id, c.port(in_name)?, c.port(out_name)?))
        })
        .collect();
    candidates.sort_by(|&(a, a_in, a_out), &(b, b_in, b_out)| {
        let (ca, cb) = (liberty.cell(a), liberty.cell(b));
        cb.drive_resistance(b_out)
            .total_cmp(&ca.drive_resistance(a_out))
            .then_with(|| intrinsic_delay(ca, a_out).total_cmp(&intrinsic_delay(cb, b_out)))
            .then_with(|| {
                ca.ports[a_in]
                    .capacitance()
                    .total_cmp(&cb.ports[b_in].capacitance())
            })
    });
    let drive = cell.drive_resistance(drvr_port);
    let (delays, _) = gate_delays(cell, drvr_port, load_cap, &resizer.targets.slews, None);
    let delay = prev_drive.mul_add(cell.ports[in_port].capacitance(), delays.max().max(0.0));
    for (id, c_in, c_out) in candidates {
        let c = liberty.cell(id);
        let c_drive = c.drive_resistance(c_out);
        let (c_delays, _) = gate_delays(c, c_out, load_cap, &resizer.targets.slews, None);
        let c_delay = prev_drive.mul_add(c.ports[c_in].capacitance(), c_delays.max().max(0.0));
        if c_drive < drive && c_delay < delay {
            return Some(id);
        }
    }
    None
}

/// The largest intrinsic delay of the arcs into `port`, as
/// `LibertyPort::intrinsicDelay`.
fn intrinsic_delay(cell: &LibCell, port: usize) -> f64 {
    cell.delay_arc_sets_to(port)
        .flat_map(|s| s.arcs.iter())
        .map(|a| a.gate_delay(0.0, 0.0).0)
        .filter(|&d| d > 0.0)
        .fold(0.0, f64::max)
}

/// Whether an instance is a single-output gate without state, as
/// `Resizer::isSingleOutputCombinational`.
fn is_single_output_combinational(resizer: &Resizer<'_>, cell: CellId, lib: LibCellId) -> bool {
    let c = resizer.link.liberty.cell(lib);
    resizer.link.is_logic_std_cell(resizer.db, cell)
        && !c.is_clock_gate
        && c.sequentials.is_empty()
        && c.ports.iter().filter(|p| p.is_output()).count() == 1
}

/// The usable size of `lib` whose load limit is nearest half of its own,
/// as `Resizer::halfDrivingPowerCell`.
fn half_driving_power_cell(resizer: &Resizer<'_>, lib: LibCellId) -> Option<LibCellId> {
    let liberty = resizer.link.liberty;
    let max_load = |id: LibCellId| -> f64 {
        let c = liberty.cell(id);
        c.ports
            .iter()
            .filter(|p| p.is_output())
            .find_map(|p| liberty.capacitance_limit(c, p))
            .unwrap_or(0.0)
    };
    let target = 0.5 * max_load(lib);
    let mut closest: Option<(f64, LibCellId)> = None;
    for candidate in resizer.link.swappable_cells(lib) {
        let limit = max_load(candidate);
        let diff = (limit - target).abs();
        if fuzzy::zero(diff) {
            return Some(candidate);
        }
        if closest.is_none_or(|(d, _)| diff < d) {
            closest = Some((diff, candidate));
        }
    }
    closest.map(|(_, c)| c)
}

/// The input ports of `cell` that every output's function treats the same
/// as `input`, as `SwapPinsMove::equivCellPins`.
fn equivalent_pins(cell: &LibCell, input: usize) -> Vec<usize> {
    if !cell.sequentials.is_empty() {
        return Vec::new();
    }
    let inputs: Vec<usize> = (0..cell.ports.len())
        .filter(|&p| cell.ports[p].is_input())
        .collect();
    let outputs: Vec<usize> = (0..cell.ports.len())
        .filter(|&p| cell.ports[p].is_output())
        .collect();
    if outputs.is_empty() || inputs.len() < 2 || inputs.len() > MAX_SIMULATED_INPUTS {
        return Vec::new();
    }
    if cell.ports.iter().any(|p| !p.is_input() && !p.is_output()) {
        return Vec::new();
    }
    let mut equivalent = Vec::new();
    for &candidate in &inputs {
        if candidate == input {
            continue;
        }
        let mut verdict: Option<bool> = None;
        for &output in &outputs {
            let Some(function) = &cell.ports[output].function else {
                continue;
            };
            let same = ports_equivalent(function, &inputs, cell, input, candidate);
            verdict = Some(verdict.is_none_or(|v| v) && same);
        }
        if verdict == Some(true) {
            equivalent.push(candidate);
        }
    }
    equivalent
}

/// Whether `function` gives the same value under every assignment when
/// `a` and `b` trade values, as `BaseMove::isPortEqiv`.
fn ports_equivalent(
    function: &FuncExpr,
    inputs: &[usize],
    cell: &LibCell,
    a: usize,
    b: usize,
) -> bool {
    let n = inputs.len();
    for assignment in 0..(1u32 << n) {
        let level_of = |swap: bool| {
            move |name: &str| -> Logic {
                let Some(mut port) = cell.port(name) else {
                    return Logic::Unknown;
                };
                if swap {
                    if port == a {
                        port = b;
                    } else if port == b {
                        port = a;
                    }
                }
                inputs
                    .iter()
                    .position(|&p| p == port)
                    .map_or(Logic::Unknown, |i| {
                        if (assignment >> i) & 1 == 1 {
                            Logic::One
                        } else {
                            Logic::Zero
                        }
                    })
            }
        };
        if function.eval(&level_of(false)) != function.eval(&level_of(true)) {
            return false;
        }
    }
    true
}

/// The equivalent input whose arcs into `drvr_port` are fastest at the
/// present slews, if faster than the critical input's, as
/// `SwapPinsMove::findSwapPinCandidate`.
fn swap_candidate(
    resizer: &Resizer<'_>,
    cell: &LibCell,
    input: usize,
    drvr_port: usize,
    equivalent: &[usize],
    load_cap: f64,
    input_slews: &[(String, EdgeValues)],
) -> Option<usize> {
    let mut base_delay = f64::NEG_INFINITY;
    let mut port_delays: FxHashMap<usize, f64> = FxHashMap::default();
    for set in cell.delay_arc_sets_to(drvr_port) {
        let from_name = &cell.ports[set.from].name;
        for arc in &set.arcs {
            let in_slew = input_slews
                .iter()
                .find(|(n, _)| n == from_name)
                .map_or_else(
                    || resizer.targets.slews.get(arc.from_edge),
                    |(_, s)| s.get(arc.from_edge),
                );
            let (delay, _) = arc.gate_delay(in_slew, load_cap);
            if set.from == input {
                base_delay = base_delay.max(delay);
            } else {
                let slot = port_delays.entry(set.from).or_insert(f64::NEG_INFINITY);
                *slot = slot.max(delay);
            }
        }
    }
    let mut chosen = None;
    for &port in equivalent {
        if port == input || port == drvr_port || !cell.ports[port].is_input() {
            continue;
        }
        let Some(&delay) = port_delays.get(&port) else {
            continue;
        };
        if delay < base_delay {
            chosen = Some(port);
            base_delay = delay;
        }
    }
    chosen
}

/// `BaseMove::estimatedSlackOK`: whether taking the buffer at `site` out
/// leaves every path through its loads and the previous driver's other
/// loads no worse, by estimated slews and delays.
fn estimated_slack_ok(
    resizer: &Resizer<'_>,
    site: &Site,
    buffer_input: PinId,
    prev_driver: PinId,
    prev_edge: Edge,
    margin: f64,
) -> bool {
    let Some(sta) = resizer.sta() else {
        return false;
    };
    let PinOwner::Cell {
        cell: prev_cell, ..
    } = resizer.db.pins[prev_driver.index()].owner
    else {
        return false;
    };
    let Some((prev_lib, prev_port)) = resizer.link.pin_port(resizer.db, prev_driver) else {
        return false;
    };
    let prev = resizer.link.liberty.cell(prev_lib);
    let buffer = resizer.link.liberty.cell(site.lib);
    let Some((in_port, _)) = buffer.buffer_ports() else {
        return false;
    };
    let prev_input_slews = resizer.input_slews(prev_cell);
    let old_load = resizer.load_cap(prev_driver);
    let (old_delay, old_slew) = gate_delays(
        prev,
        prev_port,
        old_load,
        &resizer.targets.slews,
        Some(&prev_input_slews),
    );
    let new_load = old_load + resizer.load_cap(site.driver) - buffer.ports[in_port].capacitance();
    let (new_delay, new_slew) = gate_delays(
        prev,
        prev_port,
        new_load,
        &resizer.targets.slews,
        Some(&prev_input_slews),
    );
    if !resizer.cap_ok(prev_driver, new_load - old_load) {
        return false;
    }
    let delay_degrade = new_delay.get(prev_edge) - old_delay.get(prev_edge);
    let buffer_out_delays = buffer_delays(
        buffer,
        &resizer.targets.slews,
        resizer.load_cap(site.driver),
    );
    let delay_improve = buffer_out_delays.get(site.edge);
    let Some(load_slews) = slews_after_removal(
        resizer,
        prev_driver,
        site.driver,
        buffer_input,
        new_slew.get(prev_edge),
    ) else {
        return false;
    };
    for (load, slew) in load_slews {
        let PinOwner::Cell { cell, .. } = resizer.db.pins[load.index()].owner else {
            continue;
        };
        let old = resizer.graph.slew(load);
        let new = EdgeValues {
            rise: slew,
            fall: slew,
        };
        if !input_slew_impact_ok(
            resizer,
            cell,
            old,
            new,
            delay_degrade - delay_improve,
            margin,
            true,
        ) {
            return false;
        }
    }
    let input_net = resizer.db.pins[prev_driver.index()].net;
    for &side in &resizer.db.nets[input_net.index()].pins {
        if side == prev_driver || side == buffer_input {
            continue;
        }
        let old_slack = sta.slack(side, MinMax::Max);
        let new_slack = old_slack - delay_degrade - margin;
        if new_slack < 0.0 {
            let degrade = old_slack - new_slack;
            if old_slack >= 0.0 || degrade > SLACK_DEGRADE_RATIO_LIMIT * old_slack.abs() {
                return false;
            }
        }
        if let PinOwner::Cell { cell, .. } = resizer.db.pins[side.index()].owner
            && !input_slew_impact_ok(
                resizer,
                cell,
                old_slew,
                new_slew,
                delay_degrade,
                margin,
                false,
            )
        {
            return false;
        }
    }
    true
}

/// `BaseMove::estimateInputSlewImpact`: whether every output of `cell`
/// keeps its slack when its input slews change from `old` to `new` and
/// its arrival shifts by `adjust`.
fn input_slew_impact_ok(
    resizer: &Resizer<'_>,
    cell: CellId,
    old: EdgeValues,
    new: EdgeValues,
    adjust: f64,
    margin: f64,
    accept_if_improves: bool,
) -> bool {
    let Some(sta) = resizer.sta() else {
        return false;
    };
    let Some((lib, c)) = resizer.link.cell(resizer.db, cell) else {
        return false;
    };
    let _ = lib;
    let netlist = resizer.netlist();
    for &pin in &resizer.db.cells[cell.index()].pins {
        let Some(port) = netlist.lib_port(pin) else {
            continue;
        };
        if !port.is_output() {
            continue;
        }
        let Some((_, port_index)) = resizer.link.pin_port(resizer.db, pin) else {
            return false;
        };
        let load_cap = resizer.load_cap(pin);
        let slews_of = |slew: EdgeValues| -> Vec<(String, EdgeValues)> {
            c.ports.iter().map(|p| (p.name.clone(), slew)).collect()
        };
        let (old_delay, _) = gate_delays(
            c,
            port_index,
            load_cap,
            &resizer.targets.slews,
            Some(&slews_of(old)),
        );
        let (new_delay, _) = gate_delays(
            c,
            port_index,
            load_cap,
            &resizer.targets.slews,
            Some(&slews_of(new)),
        );
        let delay_diff = (new_delay.rise - old_delay.rise).max(new_delay.fall - old_delay.fall);
        let old_slack = sta.slack(pin, MinMax::Max) - margin;
        let new_slack = old_slack - delay_diff - adjust - margin;
        if (accept_if_improves && fuzzy::greater(old_slack, new_slack))
            || (!accept_if_improves && new_slack < 0.0)
        {
            return false;
        }
    }
    true
}

/// The slews the previous driver's loads would see with the buffer gone,
/// by the Elmore estimate over the two nets stitched together and scaled by
/// how far the estimate is from the slews timing found, as
/// `Resizer::estimateSlewsAfterBufferRemoval`.
fn slews_after_removal(
    resizer: &Resizer<'_>,
    prev_driver: PinId,
    buffer_output: PinId,
    buffer_input: PinId,
    new_driver_slew: f64,
) -> Option<Vec<(PinId, f64)>> {
    let tree1 = tree_of(resizer, prev_driver)?;
    let tree2 = tree_of(resizer, buffer_output)?;
    let stitched = stitch(resizer, &tree1, buffer_input, &tree2)?;
    let driver_slew_with_buffer = resizer.graph.slew(prev_driver).max();
    let to_buffer = slews_in_tree(resizer, driver_slew_with_buffer, &tree1);
    let estimated_at_buffer = to_buffer.iter().find(|(p, _)| *p == buffer_input)?.1;
    let actual_at_buffer = resizer.graph.slew(buffer_input).max();
    let in_calibration = if fuzzy::zero(estimated_at_buffer) {
        1.0
    } else {
        actual_at_buffer / estimated_at_buffer
    };
    let buffer_slew = resizer.graph.slew(buffer_output).max();
    let from_buffer = slews_in_tree(resizer, buffer_slew, &tree2);
    let calibration: FxHashMap<PinId, f64> = from_buffer
        .into_iter()
        .map(|(load, estimated)| {
            let actual = resizer.graph.slew(load).max();
            let out = if fuzzy::zero(estimated) {
                1.0
            } else {
                actual / estimated
            };
            (load, in_calibration * out)
        })
        .collect();
    let mut slews = slews_in_tree(resizer, new_driver_slew, &stitched);
    for (load, slew) in &mut slews {
        if let Some(factor) = calibration.get(load) {
            *slew *= factor;
        }
    }
    Some(slews)
}

/// The tree of the net `driver` drives.
fn tree_of(resizer: &Resizer<'_>, driver: PinId) -> Option<BnetPtr> {
    let net = resizer.db.pins[driver.index()].net;
    let netlist = resizer.netlist();
    let parasitics = resizer.graph.parasitics(net)?;
    bnet::from_parasitics(&netlist, parasitics, driver, &resizer.tree_rc(), resizer)
}

/// `outer` with the load `at_load` replaced by `inner`, joined by a wire
/// where their points differ, as `Resizer::stitchTrees`; none when the
/// load is not in the tree.
fn stitch(
    resizer: &Resizer<'_>,
    outer: &BnetPtr,
    at_load: PinId,
    inner: &BnetPtr,
) -> Option<BnetPtr> {
    let rc = resizer.tree_rc();
    match &outer.kind {
        Kind::Load { pin } if *pin == at_load => Some(join_at_load(outer, inner, &rc)),
        Kind::Load { .. } => None,
        Kind::Wire { to, layer } => {
            let to = stitch(resizer, to, at_load, inner)?;
            Some(Bnet::wire(outer.at, to, *layer, &rc))
        }
        Kind::Via {
            to,
            layer,
            to_layer,
        } => {
            let to = stitch(resizer, to, at_load, inner)?;
            Some(Bnet::via(outer.at, *layer, *to_layer, to, &rc))
        }
        Kind::Buffer { .. } => None,
        Kind::Junction { left, right } => {
            match (
                stitch(resizer, left, at_load, inner),
                stitch(resizer, right, at_load, inner),
            ) {
                (Some(l), None) => Some(Bnet::junction(outer.at, l, Rc::clone(right))),
                (None, Some(r)) => Some(Bnet::junction(outer.at, Rc::clone(left), r)),
                _ => None,
            }
        }
    }
}

/// `inner` hung where the load `outer` was: as it is at the same point and
/// layer, by a wire on the load's layer, or through vias and a wire from
/// the layer `inner` starts on; as it is when either layer is unknown.
fn join_at_load(outer: &BnetPtr, inner: &BnetPtr, rc: &bnet::TreeRc<'_>) -> BnetPtr {
    let (in_layer, out_layer) = (outer.layer(), inner.layer());
    if in_layer == out_layer {
        return if outer.at == inner.at {
            Rc::clone(inner)
        } else {
            Bnet::wire(outer.at, Rc::clone(inner), in_layer, rc)
        };
    }
    let (Some(in_layer), Some(out_layer)) = (in_layer, out_layer) else {
        return Rc::clone(inner);
    };
    let mut joined = Rc::clone(inner);
    let mut layer = out_layer.index();
    while layer != in_layer.index() {
        let next = if in_layer.index() > layer {
            layer + 1
        } else {
            layer - 1
        };
        joined = Bnet::via(
            inner.at,
            LayerId::new(layer),
            LayerId::new(next),
            joined,
            rc,
        );
        layer = next;
    }
    if outer.at != inner.at {
        joined = Bnet::wire(outer.at, joined, Some(in_layer), rc);
    }
    joined
}

/// The slew at each load of a tree from the driver's slew, degraded along
/// the wires by their Elmore delay times the slew shape factor, as
/// `Resizer::estimateSlewsInTree`.
fn slews_in_tree(resizer: &Resizer<'_>, driver_slew: f64, tree: &Bnet) -> Vec<(PinId, f64)> {
    fn walk(resizer: &Resizer<'_>, node: &Bnet, upstream: f64, out: &mut Vec<(PinId, f64)>) {
        match &node.kind {
            Kind::Load { pin } => out.push((*pin, upstream)),
            Kind::Wire { to, layer } => {
                let (res, cap) = resizer.tree_rc().wire(*layer);
                let meters = node.length() as f64 / resizer.db.dbu().get() as f64 * 1e-6;
                let degradation =
                    meters * res * (to.cap + meters * cap / 2.0) * resizer.slew_rc_factor;
                walk(resizer, to, upstream + degradation, out);
            }
            Kind::Via {
                to,
                layer,
                to_layer,
            } => {
                let r_via = resizer.tree_rc().via(*layer, *to_layer);
                let degradation = r_via * to.cap * resizer.slew_rc_factor;
                walk(resizer, to, upstream + degradation, out);
            }
            Kind::Junction { left, right } => {
                walk(resizer, left, upstream, out);
                walk(resizer, right, upstream, out);
            }
            Kind::Buffer { to, .. } => walk(resizer, to, upstream, out),
        }
    }
    let mut out = Vec::new();
    walk(resizer, tree, driver_slew, &mut out);
    out
}
