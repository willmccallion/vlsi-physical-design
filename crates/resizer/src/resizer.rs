//! The state a repair session works on, as `OpenROAD`'s `Resizer` holds it.
//!
//! The design, its timing graph and analysis, the buffers and target slews
//! repair sizes against, and a journal of edits so that a repair which did
//! not pay off can be taken back.

use crate::bnet::LoadModel;
use crate::fuzzy;
use crate::graph::TimingGraph;
use crate::library::{BufferSet, Link};
use crate::netlist::{ClockNetwork, Constants, Namer, Netlist};
use crate::parasitics::WireRc;
use crate::sta::Sta;
use pare_common::db::design::{EditError, NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, MacroId, MacroPinId, NetId, PinId};
use pare_common::db::liberty::{ArcRole, Edge, EdgeValues, LibCell, LibCellId, LibPort, MinMax};
use pare_common::db::library::DuplicateName;
use pare_common::db::sdc::Sdc;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use rustc_hash::FxHashSet;
use std::collections::{HashMap, HashSet};

/// The load a buffer's target slew is measured at, in multiples of its
/// input capacitance, as `OpenROAD`'s `tgt_slew_load_cap_factor`.
const TARGET_SLEW_LOAD_FACTOR: f64 = 10.0;

/// Pessimism added to the slew shape factor, as `OpenROAD` adds it.
const SLEW_RC_PESSIMISM: f64 = 0.10;

/// Why repair stopped.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum RepairError {
    /// The netlist refused an edit.
    #[error(transparent)]
    Edit(#[from] EditError),
    /// A name was already taken.
    #[error(transparent)]
    Name(#[from] DuplicateName),
    /// A cell to insert has no master.
    #[error("cell '{0}' has no master")]
    NoMaster(String),
    /// A cell to insert lacks the pins it needs.
    #[error("cell '{0}' has no usable pins")]
    MissingPins(String),
    /// The library's slew thresholds give an unusable shape factor.
    #[error("the slew thresholds give an RC shape factor of {0:.3e}, outside 0.1 to 10")]
    ShapeFactor(f64),
}

/// Margins and limits for repair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RepairOptions {
    /// Percentage of each slew limit kept in hand.
    pub slew_margin: f64,
    /// Percentage of each capacitance limit kept in hand.
    pub cap_margin: f64,
    /// Longest wire allowed, in database units; zero for no limit.
    pub max_wire_length: i64,
}

impl Default for RepairOptions {
    fn default() -> Self {
        Self {
            slew_margin: 0.0,
            cap_margin: 0.0,
            max_wire_length: 0,
        }
    }
}

/// What repair reads about the design besides the netlist itself.
#[derive(Clone, Copy, Debug)]
pub struct Context<'a> {
    /// The library link.
    pub link: &'a Link<'a>,
    /// The clock network, which repair leaves alone.
    pub clocks: &'a ClockNetwork,
    /// The constant pins, which repair leaves alone.
    pub constants: &'a Constants,
}

/// Target slews and the load each cell meets them at, as `OpenROAD`'s
/// `tgt_slews_` and `target_load_map_`.
#[derive(Clone, Debug)]
pub struct Targets {
    /// The slews repair sizes toward, per edge.
    pub slews: EdgeValues,
    /// The load at which each Liberty cell reaches them.
    pub load: Vec<f64>,
}

/// One edit, recorded so it can be undone.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Edit {
    Master { cell: CellId, old: MacroId },
    MovedPin { pin: PinId, from: NetId },
    NewCell(CellId),
    NewNet(NetId),
    Retired(CellId),
    Renamed { net: NetId, old: String },
}

/// The edits since the last mark, as `OpenROAD`'s journal.
#[derive(Clone, Debug, Default)]
struct Journal {
    recording: bool,
    edits: Vec<Edit>,
}

/// A buffer newly inserted before some loads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InsertedBuffer {
    /// The instance.
    pub cell: CellId,
    /// Its input pin, on the loads' old net.
    pub input: PinId,
    /// Its output pin, on the new net.
    pub output: PinId,
    /// The new net the loads moved to.
    pub net: NetId,
}

/// The state of a repair session.
#[derive(Debug)]
pub struct Resizer<'a> {
    pub(crate) db: &'a mut NetlistDB,
    pub(crate) link: &'a Link<'a>,
    pub(crate) clocks: &'a ClockNetwork,
    pub(crate) constants: &'a Constants,
    pub(crate) graph: TimingGraph,
    pub(crate) sta: Option<Sta>,
    pub(crate) buffers: BufferSet,
    pub(crate) targets: Targets,
    pub(crate) namer: &'a mut Namer,
    pub(crate) options: RepairOptions,
    pub(crate) slew_rc_factor: f64,
    pub(crate) r_strongest_buffer: f64,
    journal: Journal,
    dead_nets: Vec<NetId>,
    resized_multi_output: HashSet<CellId>,
    inserted_buffers: usize,
}

impl<'a> Resizer<'a> {
    /// Prepares a session over `db`: estimates every net's parasitics and
    /// slews, and with `sdc` its timing too.
    pub fn new(
        db: &'a mut NetlistDB,
        context: Context<'a>,
        rc: WireRc,
        buffers: BufferSet,
        namer: &'a mut Namer,
        options: RepairOptions,
        sdc: Option<&Sdc>,
    ) -> Result<Self, RepairError> {
        let Context {
            link,
            clocks,
            constants,
        } = context;
        let slew_rc_factor = slew_rc_factor(link)?;
        let r_strongest_buffer = buffers
            .cells
            .iter()
            .map(|&b| link.buffer_drive_resistance(b))
            .fold(f64::MAX, f64::min);
        let targets = find_targets(link, &buffers);
        let (graph, sta) = {
            let netlist = Netlist::new(db, link);
            let graph = TimingGraph::new(&netlist, clocks, constants, rc);
            let sta = sdc.map(|sdc| Sta::new(&netlist, &graph, clocks, constants, sdc));
            (graph, sta)
        };
        Ok(Self {
            db,
            link,
            clocks,
            constants,
            graph,
            sta,
            buffers,
            targets,
            namer,
            options,
            slew_rc_factor,
            r_strongest_buffer,
            journal: Journal::default(),
            dead_nets: Vec::new(),
            resized_multi_output: HashSet::new(),
            inserted_buffers: 0,
        })
    }

    /// A view of the design as it stands.
    #[must_use]
    pub const fn netlist(&self) -> Netlist<'_> {
        Netlist::new(self.db, self.link)
    }

    /// The timing analysis, if constraints were given.
    #[must_use]
    pub const fn sta(&self) -> Option<&Sta> {
        self.sta.as_ref()
    }

    /// The timing graph.
    #[must_use]
    pub const fn graph(&self) -> &TimingGraph {
        &self.graph
    }

    /// Buffers inserted so far in the session.
    #[must_use]
    pub const fn inserted_buffers(&self) -> usize {
        self.inserted_buffers
    }

    /// Sets the longest wire design repair allows, in database units; zero
    /// for no limit.
    pub const fn set_max_wire_length(&mut self, length: i64) {
        self.options.max_wire_length = length;
    }

    /// The setup slack of every net at its driver, as `OpenROAD`'s
    /// `findResizeSlacks1` collects them for timing-driven placement:
    /// constant, clock and special nets left out. Needs timing.
    #[must_use]
    pub fn driver_net_slacks(&self) -> Vec<(NetId, f64)> {
        let Some(sta) = self.sta.as_ref() else {
            return Vec::new();
        };
        let netlist = self.netlist();
        let mut slack_of: HashMap<NetId, f64> = HashMap::new();
        for driver in self.graph.drivers_by_level(&netlist).into_iter().rev() {
            let net = self.db.pins[driver.index()].net;
            if netlist.is_special(net)
                || self.constants.is_constant(driver)
                || self.clocks.is_clock(driver)
            {
                continue;
            }
            let _ = slack_of.insert(net, sta.slack(driver, MinMax::Max));
        }
        let mut slacks: Vec<(NetId, f64)> = slack_of.into_iter().collect();
        slacks.sort_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)));
        slacks
    }

    /// Drops every retired cell and emptied net from the design and ends
    /// the session: every id held before is stale afterwards.
    pub fn finish(self) -> Result<pare_common::db::design::Renumbering, RepairError> {
        let dead: Vec<NetId> = self
            .dead_nets
            .iter()
            .copied()
            .filter(|&n| self.db.nets[n.index()].pins.is_empty())
            .collect();
        Ok(self.db.remove(&[], &dead)?)
    }

    /// The transition limit at an input pin: its own, else, as `OpenROAD`
    /// fixes up for libraries like Nangate45, the library default meant
    /// for outputs; infinite without either.
    #[must_use]
    pub fn max_input_slew(&self, port: &LibPort) -> f64 {
        max_input_slew(self.link, port)
    }

    /// Holds every input slew over its limit at that limit, each before
    /// anything downstream of it is timed, as `OpenROAD`'s `repairDesign`
    /// forward pass, and brings the timing up to date; returns the pins
    /// held.
    pub fn annotate_violating_input_slews(&mut self) -> Vec<PinId> {
        let (annotated, mut dirty) = {
            let netlist = Netlist::new(self.db, self.link);
            let link = self.link;
            let limit_of =
                |pin: PinId| netlist.lib_port(pin).map(|port| max_input_slew(link, port));
            self.graph
                .sweep_annotating(&netlist, self.clocks, &limit_of)
        };
        let netlist = Netlist::new(self.db, self.link);
        for &(load, _) in &annotated {
            dirty.extend(self.graph.drivers_fed_by(&netlist, load));
        }
        dirty.sort_unstable();
        dirty.dedup();
        if let Some(sta) = &mut self.sta {
            sta.update(&netlist, &self.graph, self.clocks, &dirty);
        }
        annotated.into_iter().map(|(load, _)| load).collect()
    }

    /// A slew limit less the margin kept in hand.
    #[must_use]
    pub fn margined_slew(&self, limit: f64) -> f64 {
        limit * (1.0 - self.options.slew_margin / 100.0)
    }

    /// A load limit less the margin kept in hand.
    #[must_use]
    pub fn margined_cap(&self, limit: f64) -> f64 {
        limit * (1.0 - self.options.cap_margin / 100.0)
    }

    /// The slew limit at a driver, if any applies, as `OpenSTA` finds it.
    #[must_use]
    pub fn driver_slew_limit(&self, driver: PinId) -> Option<f64> {
        let netlist = self.netlist();
        let own = netlist
            .lib_port(driver)
            .and_then(|p| self.link.liberty.slew_limit(p));
        let design = self
            .sta
            .as_ref()
            .and_then(|s| s.sdc().limits.max_transition);
        match (own, design) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// The load limit at a driver, if any applies.
    #[must_use]
    pub fn cap_limit(&self, driver: PinId) -> Option<f64> {
        let netlist = self.netlist();
        let own = netlist
            .lib_port(driver)
            .and_then(|p| self.link.liberty.capacitance_limit(p));
        let design = self
            .sta
            .as_ref()
            .and_then(|s| s.sdc().limits.max_capacitance);
        match (own, design) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// The fanout limit on every driver, if the constraints set one.
    #[must_use]
    pub fn fanout_limit(&self) -> Option<f64> {
        self.sta.as_ref().and_then(|s| s.sdc().limits.max_fanout)
    }

    /// The load a driver sees.
    #[must_use]
    pub fn load_cap(&self, driver: PinId) -> f64 {
        self.graph.load_cap(&self.netlist(), driver)
    }

    /// How many loads a driver's net has besides the driver.
    #[must_use]
    pub fn fanout(&self, driver: PinId) -> usize {
        let netlist = self.netlist();
        let net = self.db.pins[driver.index()].net;
        netlist.loads(net).filter(|&l| l != driver).count()
    }

    /// Whether a pin drives through a tristate output.
    #[must_use]
    pub fn is_tristate_driver(&self, driver: PinId) -> bool {
        self.netlist()
            .lib_port(driver)
            .is_some_and(|p| p.three_state.is_some())
    }

    /// Whether an instance may be resized or removed: a plain standard
    /// cell the library does not protect.
    #[must_use]
    pub fn is_editable(&self, cell: CellId) -> bool {
        self.link
            .cell(self.db, cell)
            .is_some_and(|(_, lib)| !lib.dont_touch)
            && self.link.is_logic_std_cell(self.db, cell)
    }

    /// Whether a net may be buffered, as `Resizer::okToBufferNet`: not
    /// driven by a tristate, not a supply.
    #[must_use]
    pub fn ok_to_buffer_net(&self, driver: PinId) -> bool {
        let net = self.db.pins[driver.index()].net;
        !self.is_tristate_driver(driver) && !self.netlist().is_special(net)
    }

    /// The slews at each input pin of `cell`, by Liberty port name.
    #[must_use]
    pub fn input_slews(&self, cell: CellId) -> Vec<(String, EdgeValues)> {
        let netlist = self.netlist();
        self.db.cells[cell.index()]
            .pins
            .iter()
            .filter_map(|&pin| {
                let port = netlist.lib_port(pin)?;
                Some((port.name.clone(), self.graph.slew(pin)))
            })
            .collect()
    }

    /// The worst delay and slew per output edge of `cell`'s port at
    /// `load_cap`, from the given input slews by port name or the target
    /// slews without, as `Resizer::gateDelays`.
    #[must_use]
    pub fn gate_delays(
        &self,
        cell: &LibCell,
        port: usize,
        load_cap: f64,
        input_slews: Option<&[(String, EdgeValues)]>,
    ) -> (EdgeValues, EdgeValues) {
        gate_delays(cell, port, load_cap, &self.targets.slews, input_slews)
    }

    /// The worst delay of a buffer or inverter at `load_cap` from the
    /// target input slews, as `Resizer::bufferDelay`.
    #[must_use]
    pub fn buffer_delay(&self, cell: LibCellId, load_cap: f64) -> f64 {
        buffer_delay(self.link.liberty.cell(cell), &self.targets.slews, load_cap)
    }

    /// The design's area in the library's units, over cells of the core.
    #[must_use]
    pub fn design_area(&self) -> f64 {
        self.db
            .cells
            .iter()
            .enumerate()
            .filter(|(_, c)| !c.retired)
            .filter_map(|(i, _)| self.link.cell(self.db, CellId::new(i)))
            .map(|(_, lib)| lib.area)
            .sum()
    }

    /// Starts recording edits, as `journalBegin`.
    pub fn journal_begin(&mut self) {
        self.journal.recording = true;
        self.journal.edits.clear();
    }

    /// Keeps the edits since the mark, as `journalEnd`.
    pub fn journal_end(&mut self) {
        self.journal.recording = false;
        self.journal.edits.clear();
    }

    /// Takes back every edit since the mark and brings the timing up to
    /// date, as `journalRestore`.
    pub fn journal_restore(&mut self) -> Result<(), RepairError> {
        self.journal.recording = false;
        let edits = std::mem::take(&mut self.journal.edits);
        let mut nets: FxHashSet<NetId> = FxHashSet::default();
        let mut cells: FxHashSet<CellId> = FxHashSet::default();
        for edit in edits.into_iter().rev() {
            match edit {
                Edit::Master { cell, old } => {
                    self.db.replace_master(cell, old)?;
                    let _ = cells.insert(cell);
                }
                Edit::MovedPin { pin, from } => {
                    let _ = nets.insert(self.db.pins[pin.index()].net);
                    let _ = nets.insert(from);
                    self.db.move_pin(pin, from);
                }
                Edit::NewCell(cell) => {
                    for &pin in &self.db.cells[cell.index()].pins {
                        let _ = nets.insert(self.db.pins[pin.index()].net);
                    }
                    self.db.retire_cell(cell);
                }
                Edit::NewNet(net) => self.dead_nets.push(net),
                Edit::Retired(cell) => {
                    self.db.unretire_cell(cell);
                    for &pin in &self.db.cells[cell.index()].pins {
                        let _ = nets.insert(self.db.pins[pin.index()].net);
                    }
                }
                Edit::Renamed { net, old } => self.db.rename_net(net, old)?,
            }
        }
        let mut dirty = Vec::new();
        {
            let netlist = Netlist::new(self.db, self.link);
            self.graph.grow(&netlist);
            for net in nets {
                self.graph.parasitics_invalid(&netlist, self.clocks, net);
                dirty.extend(netlist.drivers(net));
            }
            for cell in cells {
                for &pin in &self.db.cells[cell.index()].pins {
                    if netlist.is_load(pin) {
                        dirty.extend(netlist.drivers(self.db.pins[pin.index()].net));
                    }
                    if netlist.is_driver(pin) {
                        dirty.push(pin);
                    }
                }
            }
        }
        self.propagate(&dirty);
        Ok(())
    }

    fn record(&mut self, edit: Edit) {
        if self.journal.recording {
            self.journal.edits.push(edit);
        }
    }

    /// Drops the slew annotations and finds the slews they held fixed
    /// again, with everything downstream, as `OpenROAD` invalidates its
    /// delays once design repair's annotations come off.
    pub fn clear_slew_annotations(&mut self) {
        let annotated = self.graph.clear_annotations();
        let mut drivers: Vec<PinId> = Vec::new();
        {
            let netlist = Netlist::new(self.db, self.link);
            for pin in annotated {
                if netlist.is_driver(pin) {
                    drivers.push(pin);
                }
                if netlist.is_load(pin) {
                    drivers.extend(netlist.drivers(self.db.pins[pin.index()].net));
                }
            }
        }
        drivers.sort_unstable();
        drivers.dedup();
        self.propagate(&drivers);
    }

    /// Finds slews, delays and timing again from `drivers` on.
    fn propagate(&mut self, drivers: &[PinId]) {
        let netlist = Netlist::new(self.db, self.link);
        let dirty = self.graph.propagate_from(&netlist, self.clocks, drivers);
        if let Some(sta) = &mut self.sta {
            sta.update(&netlist, &self.graph, self.clocks, &dirty);
        }
    }

    /// Re-estimates a net and everything timed through it.
    pub fn update_net(&mut self, net: NetId) {
        let netlist = Netlist::new(self.db, self.link);
        let dirty = self.graph.update_net(&netlist, self.clocks, net);
        if let Some(sta) = &mut self.sta {
            sta.update(&netlist, &self.graph, self.clocks, &dirty);
        }
    }

    /// Re-estimates the nets around a cell whose master changed.
    pub fn update_cell(&mut self, cell: CellId) {
        let netlist = Netlist::new(self.db, self.link);
        let dirty = self.graph.update_cell(&netlist, self.clocks, cell);
        if let Some(sta) = &mut self.sta {
            sta.update(&netlist, &self.graph, self.clocks, &dirty);
        }
    }

    /// Gives `cell` the master of `size` and refreshes the timing around
    /// it; returns whether it changed.
    pub fn replace_cell(&mut self, cell: CellId, size: LibCellId) -> Result<bool, RepairError> {
        let master = self
            .link
            .macro_of(size)
            .ok_or_else(|| RepairError::NoMaster(self.link.liberty.cell(size).name.clone()))?;
        let old = self.db.cells[cell.index()].macro_id;
        if old == master {
            return Ok(false);
        }
        self.db.replace_master(cell, master)?;
        self.record(Edit::Master { cell, old });
        self.update_cell(cell);
        Ok(true)
    }

    /// Moves a pin to another net, recording it.
    pub fn move_pin(&mut self, pin: PinId, to: NetId) {
        let from = self.db.pins[pin.index()].net;
        if from == to {
            return;
        }
        self.db.move_pin(pin, to);
        self.record(Edit::MovedPin { pin, from });
    }

    /// Adds an instance of `cell` at `at`, named from `base`, recording
    /// it; its pins are connected by the caller.
    pub fn make_instance(
        &mut self,
        cell: LibCellId,
        at: Point<Dbu>,
        base: &str,
    ) -> Result<CellId, RepairError> {
        let lib = self.link.liberty.cell(cell);
        let master = self
            .link
            .macro_of(cell)
            .ok_or_else(|| RepairError::NoMaster(lib.name.clone()))?;
        let name = self.namer.instance(self.db, base);
        let id = self
            .db
            .add_cell(name, master, at, Orientation::N, PlacementStatus::Placed)?;
        self.record(Edit::NewCell(id));
        Ok(id)
    }

    /// Adds a net named from `base` with the signal use of `like`,
    /// recording it.
    pub fn make_net(&mut self, base: &str, like: NetId) -> Result<NetId, RepairError> {
        let name = self.namer.net(self.db, base);
        let net = self.db.add_net(&name)?;
        self.db.nets[net.index()].signal_use = self.db.nets[like.index()].signal_use;
        self.record(Edit::NewNet(net));
        Ok(net)
    }

    /// Connects Liberty port `port` of `cell` to `net`.
    pub fn connect(
        &mut self,
        cell: CellId,
        lib: LibCellId,
        port: usize,
        net: NetId,
    ) -> Option<PinId> {
        let macro_pin = self.link.macro_pin_of_port(lib, port)?;
        Some(
            self.db
                .connect_cell_pin(cell, MacroPinId::new(macro_pin), net),
        )
    }

    /// Puts a buffer of `buffer` at `at` between `net` and `loads`, which
    /// move to a new net the buffer drives, as `Resizer::insertBufferBeforeLoads`;
    /// the instance is named from `base`. The timing around both nets is
    /// refreshed.
    pub fn insert_buffer_before_loads(
        &mut self,
        net: NetId,
        loads: &[PinId],
        buffer: LibCellId,
        at: Point<Dbu>,
        base: &str,
    ) -> Result<InsertedBuffer, RepairError> {
        let lib = self.link.liberty.cell(buffer);
        let (in_port, out_port) = lib
            .buffer_ports()
            .ok_or_else(|| RepairError::MissingPins(lib.name.clone()))?;
        let cell = self.make_instance(buffer, at, base)?;
        let new_net = self.make_net("net", net)?;
        for &load in loads {
            self.move_pin(load, new_net);
        }
        let input = self
            .connect(cell, buffer, in_port, net)
            .ok_or_else(|| RepairError::MissingPins(lib.name.clone()))?;
        let output = self
            .connect(cell, buffer, out_port, new_net)
            .ok_or_else(|| RepairError::MissingPins(lib.name.clone()))?;
        self.inserted_buffers += 1;
        {
            let netlist = Netlist::new(self.db, self.link);
            self.graph.grow(&netlist);
            self.graph.parasitics_invalid(&netlist, self.clocks, net);
            self.graph
                .parasitics_invalid(&netlist, self.clocks, new_net);
        }
        let mut drivers: Vec<PinId> = self.netlist().drivers(net).collect();
        drivers.push(output);
        self.propagate(&drivers);
        Ok(InsertedBuffer {
            cell,
            input,
            output,
            net: new_net,
        })
    }

    /// Takes a buffer out, joining its nets: the input net survives unless
    /// only the output net reaches a port, and the survivor takes the
    /// removed net's name when that name is deeper in the hierarchy, as
    /// `UnbufferMove::removeBuffer`. Returns the surviving net.
    pub fn remove_buffer(&mut self, buffer: CellId) -> Result<NetId, RepairError> {
        let (in_pin, out_pin) = self
            .buffer_pins(buffer)
            .ok_or_else(|| RepairError::MissingPins(self.db.cells[buffer.index()].name.clone()))?;
        let in_net = self.db.pins[in_pin.index()].net;
        let out_net = self.db.pins[out_pin.index()].net;
        let netlist = self.netlist();
        let (survivor, removed) = if !netlist.has_port(in_net) && netlist.has_port(out_net) {
            (out_net, in_net)
        } else {
            (in_net, out_net)
        };
        let moving: Vec<PinId> = self.db.nets[removed.index()]
            .pins
            .iter()
            .copied()
            .filter(|&p| p != in_pin && p != out_pin)
            .collect();
        let survivor_name = self.db.nets[survivor.index()].name.clone();
        let removed_name = self.db.nets[removed.index()].name.clone();
        self.db.retire_cell(buffer);
        self.record(Edit::Retired(buffer));
        for pin in moving {
            self.move_pin(pin, survivor);
        }
        if hierarchy_depth(&survivor_name) > hierarchy_depth(&removed_name) {
            let placeholder = self.namer.net(self.db, "net");
            self.db.rename_net(removed, placeholder)?;
            self.record(Edit::Renamed {
                net: removed,
                old: removed_name.clone(),
            });
            self.db.rename_net(survivor, removed_name)?;
            self.record(Edit::Renamed {
                net: survivor,
                old: survivor_name,
            });
        }
        self.dead_nets.push(removed);
        {
            let netlist = Netlist::new(self.db, self.link);
            self.graph
                .parasitics_invalid(&netlist, self.clocks, survivor);
            self.graph
                .parasitics_invalid(&netlist, self.clocks, removed);
        }
        let drivers: Vec<PinId> = self.netlist().drivers(survivor).collect();
        self.propagate(&drivers);
        Ok(survivor)
    }

    /// The input and output pins of a buffer instance.
    #[must_use]
    pub fn buffer_pins(&self, buffer: CellId) -> Option<(PinId, PinId)> {
        let (lib_id, lib) = self.link.cell(self.db, buffer)?;
        let (in_port, out_port) = lib.buffer_ports()?;
        let netlist = Netlist::new(self.db, self.link);
        let _ = lib_id;
        Some((
            netlist.pin_of_port(buffer, in_port)?,
            netlist.pin_of_port(buffer, out_port)?,
        ))
    }

    /// Sizes the cell driving `driver` to the size whose target load is
    /// nearest its load, as `Resizer::resizeToTargetSlew`; returns whether
    /// the cell changed.
    pub fn resize_to_target_slew(&mut self, driver: PinId) -> Result<bool, RepairError> {
        let (cell, lib_id, load_cap, revisiting) = {
            let PinOwner::Cell { cell, .. } = self.db.pins[driver.index()].owner else {
                return Ok(false);
            };
            let Some((lib_id, lib)) = self.link.cell(self.db, cell) else {
                return Ok(false);
            };
            if !self.is_editable(cell) {
                return Ok(false);
            }
            let multiple_outputs = lib.ports.iter().filter(|p| p.is_output()).count() > 1;
            let revisiting = multiple_outputs && !self.resized_multi_output.insert(cell);
            (cell, lib_id, self.load_cap(driver), revisiting)
        };
        if load_cap <= 0.0 {
            return Ok(false);
        }
        let target = self.find_target_cell(lib_id, load_cap, revisiting);
        if target == lib_id {
            return Ok(false);
        }
        let changed = self.replace_cell(cell, target)?;
        Ok(changed && !revisiting)
    }

    /// The swappable size of `cell` whose target load is nearest
    /// `load_cap`, buffers and inverters also preferring the faster, as
    /// `Resizer::findTargetCell`.
    #[must_use]
    pub fn find_target_cell(&self, cell: LibCellId, load_cap: f64, revisiting: bool) -> LibCellId {
        let candidates = self.link.swappable_cells(cell);
        if candidates.is_empty() {
            return cell;
        }
        let lib = self.link.liberty.cell(cell);
        let is_buf_inv = lib.is_buffer() || lib.is_inverter();
        let target_of = |c: LibCellId| self.targets.load[c.index()];
        let delay_of = |c: LibCellId| {
            if is_buf_inv {
                self.buffer_delay(c, load_cap)
            } else {
                0.0
            }
        };
        let mut best = cell;
        let mut best_load = target_of(cell);
        let mut best_dist = (load_cap - best_load).abs();
        let mut best_delay = delay_of(cell);
        for candidate in candidates {
            let target = target_of(candidate);
            let dist = (load_cap - target).abs();
            let delay = delay_of(candidate);
            let better = if is_buf_inv {
                (delay < best_delay && dist < best_dist * 1.1)
                    || (dist < best_dist && delay < best_delay * 1.1)
            } else {
                dist < best_dist && (!revisiting || target > best_load)
            };
            if better {
                best = candidate;
                best_dist = dist;
                best_load = target;
                best_delay = delay;
            }
        }
        best
    }

    /// Whether giving `cell` the master `replacement` keeps every driver
    /// of its inputs within its load limit, as `BaseMove::checkMaxCapViolation`.
    #[must_use]
    pub fn replacement_keeps_loads(&self, cell: CellId, replacement: LibCellId) -> bool {
        let netlist = self.netlist();
        let Some((_, current)) = self.link.cell(self.db, cell) else {
            return true;
        };
        let new_lib = self.link.liberty.cell(replacement);
        for &pin in &self.db.cells[cell.index()].pins {
            let Some(port) = netlist.lib_port(pin) else {
                continue;
            };
            if !port.is_input() {
                continue;
            }
            let old_cap = current
                .port(&port.name)
                .map_or(0.0, |p| current.ports[p].capacitance());
            let new_cap = new_lib
                .port(&port.name)
                .map_or(0.0, |p| new_lib.ports[p].capacitance());
            let delta = new_cap - old_cap;
            if delta <= 0.0 {
                continue;
            }
            let net = self.db.pins[pin.index()].net;
            for driver in netlist.drivers(net) {
                if !self.cap_ok(driver, delta) {
                    return false;
                }
            }
        }
        true
    }

    /// Whether `driver` can take `delta` more load: within its limit, or
    /// no worse when already over it, as `BaseMove::checkMaxCapOK`.
    #[must_use]
    pub fn cap_ok(&self, driver: PinId, delta: f64) -> bool {
        let Some(limit) = self.cap_limit(driver) else {
            return true;
        };
        if limit <= 0.0 {
            return true;
        }
        let cap = self.load_cap(driver);
        let new_cap = cap + delta;
        if cap > limit {
            return new_cap <= cap;
        }
        new_cap <= limit
    }
}

impl LoadModel for Resizer<'_> {
    fn load(&self, pin: PinId) -> (f64, f64, f64) {
        let netlist = self.netlist();
        netlist
            .lib_port(pin)
            .map_or((0.0, 0.0, f64::INFINITY), |port| {
                (
                    port.capacitance(),
                    self.link.liberty.fanout_load(port),
                    self.max_input_slew(port),
                )
            })
    }
}

/// How many levels of hierarchy a name has: one more than its dividers.
fn hierarchy_depth(name: &str) -> usize {
    name.matches('/').count()
}

/// Share of the nets, from the worst slack up, that timing-driven
/// placement weights, as `OpenROAD`'s `worst_slack_nets_percent`.
pub const WORST_SLACK_NETS_SHARE: f64 = 0.1;

/// The wirelength weights timing-driven placement gives nets for their
/// `slacks`, sorted worst first, as `OpenROAD`'s `TimingBase` does.
///
/// Over the worst [`WORST_SLACK_NETS_SHARE`] of them, the weight runs from
/// `weight_max` at the worst slack down to one at the best; nets at or
/// above that best keep a weight of one and are left out. Empty when no
/// net has a finite slack.
#[must_use]
pub fn timing_net_weights(slacks: &[(NetId, f64)], weight_max: f64) -> Vec<(NetId, f64)> {
    let Some(&(_, slack_min)) = slacks.first() else {
        return Vec::new();
    };
    if slack_min.is_infinite() {
        return Vec::new();
    }
    let worst = ((slacks.len() as f64) * WORST_SLACK_NETS_SHARE).ceil() as usize;
    let slack_max = slacks[worst.clamp(1, slacks.len()) - 1].1;
    slacks
        .iter()
        .filter(|(_, slack)| *slack < slack_max)
        .map(|&(net, slack)| {
            let weight = if fuzzy::equal(slack_max, slack_min) {
                1.0
            } else {
                ((weight_max - 1.0) * (slack_max - slack) / (slack_max - slack_min)) + 1.0
            };
            (net, weight)
        })
        .collect()
}

/// The transition limit at an input pin: its own, else, as `OpenROAD`
/// fixes up for libraries like Nangate45, the library default meant for
/// outputs; infinite without either.
fn max_input_slew(link: &Link<'_>, port: &LibPort) -> f64 {
    port.max_transition
        .filter(|&l| l != 0.0)
        .or(link.liberty.defaults.max_transition)
        .unwrap_or(f64::INFINITY)
}

/// The factor between a driver's resistance times its load and the slew
/// that RC gives at the library's thresholds, as `computeSlewRCFactor`.
fn slew_rc_factor(link: &Link<'_>) -> Result<f64, RepairError> {
    let t = &link.liberty.thresholds;
    let mut factor: f64 = 0.0;
    for edge in Edge::BOTH {
        let (low, high) = match edge {
            Edge::Rise => (1.0 - t.slew_upper.rise, 1.0 - t.slew_lower.rise),
            Edge::Fall => (t.slew_lower.fall, t.slew_upper.fall),
        };
        let t_high = -high.ln();
        let t_low = -low.ln();
        let edge_factor = (t_low - t_high) / t.slew_derate;
        if edge_factor <= 0.1 || edge_factor >= 10.0 {
            return Err(RepairError::ShapeFactor(edge_factor));
        }
        factor = factor.max(edge_factor);
    }
    Ok(factor * (1.0 + SLEW_RC_PESSIMISM))
}

/// Target slews from the buffers and the load each usable cell meets them
/// at, as `Resizer::findBufferTargetSlews` and `findTargetLoads`.
fn find_targets(link: &Link<'_>, buffers: &BufferSet) -> Targets {
    let liberty = link.liberty;
    let mut sums = EdgeValues::default();
    let mut counts = [0usize; 2];
    for &buffer in &buffers.cells {
        let cell = liberty.cell(buffer);
        let Some((input, output)) = cell.buffer_ports() else {
            continue;
        };
        for set in cell
            .arc_sets
            .iter()
            .filter(|s| s.from == input && s.to == output)
        {
            for arc in &set.arcs {
                let in_cap = cell.ports[input].capacitance.get(arc.from_edge);
                let load = in_cap * TARGET_SLEW_LOAD_FACTOR;
                let (_, slew1) = arc.gate_delay(0.0, load);
                let (_, slew2) = arc.gate_delay(slew1, load);
                match arc.to_edge {
                    Edge::Rise => sums.rise += slew2,
                    Edge::Fall => sums.fall += slew2,
                }
                counts[arc.to_edge.index()] += 1;
            }
        }
    }
    let slews = EdgeValues {
        rise: if counts[0] > 0 {
            sums.rise / counts[0] as f64
        } else {
            0.0
        },
        fall: if counts[1] > 0 {
            sums.fall / counts[1] as f64
        } else {
            0.0
        },
    };
    let load = (0..liberty.cells.len())
        .map(|i| {
            let id = LibCellId(i as u32);
            if link.is_link_cell(id) && !link.dont_use(id) {
                find_target_load(liberty.cell(id), &slews)
            } else {
                0.0
            }
        })
        .collect();
    Targets { slews, load }
}

/// The mean over a cell's delay arcs of the load at which each reaches the
/// target output slew from the target input slew.
fn find_target_load(cell: &LibCell, slews: &EdgeValues) -> f64 {
    let mut sum = 0.0;
    let mut count = 0;
    for set in &cell.arc_sets {
        if !matches!(
            set.role,
            ArcRole::Combinational | ArcRole::RegClkToQ | ArcRole::RegSetClr | ArcRole::LatchEnToQ
        ) {
            continue;
        }
        for arc in &set.arcs {
            let in_slew = slews.get(arc.from_edge);
            let out_slew = slews.get(arc.to_edge);
            sum += arc_target_load(arc, in_slew, out_slew);
            count += 1;
        }
    }
    if count == 0 { 0.0 } else { sum / count as f64 }
}

/// The load at which `arc` gives `out_slew` from `in_slew`, by binary
/// search; zero when even no load exceeds it.
fn arc_target_load(arc: &pare_common::db::liberty::TimingArc, in_slew: f64, out_slew: f64) -> f64 {
    let diff = |load: f64| arc.gate_delay(in_slew, load).1 - out_slew;
    let mut cap1 = 0.0;
    let mut cap2 = 1.0e-12;
    let tol = 0.01;
    if diff(cap1) > 0.0 {
        return 0.0;
    }
    let mut diff2 = diff(cap2);
    let mut iterations = 0;
    while (cap1 - cap2).abs() > cap1.max(cap2) * tol && iterations < 200 {
        iterations += 1;
        if diff2 < 0.0 {
            cap1 = cap2;
            cap2 *= 2.0;
            diff2 = diff(cap2);
        } else {
            let cap3 = f64::midpoint(cap1, cap2);
            let diff3 = diff(cap3);
            if diff3 < 0.0 {
                cap1 = cap3;
            } else {
                cap2 = cap3;
                diff2 = diff3;
            }
        }
    }
    cap1
}

/// The worst delay and slew per output edge over the delay arcs into
/// `port` at `load_cap`, each arc fed the slew of its input port from
/// `input_slews` or, failing that, the target slews, as
/// `Resizer::gateDelays`.
#[must_use]
pub fn gate_delays(
    cell: &LibCell,
    port: usize,
    load_cap: f64,
    target_slews: &EdgeValues,
    input_slews: Option<&[(String, EdgeValues)]>,
) -> (EdgeValues, EdgeValues) {
    let mut delays = EdgeValues {
        rise: f64::NEG_INFINITY,
        fall: f64::NEG_INFINITY,
    };
    let mut slews = delays;
    for set in cell.delay_arc_sets_to(port) {
        let from_name = &cell.ports[set.from].name;
        for arc in &set.arcs {
            let in_slew = input_slews
                .and_then(|slews| slews.iter().find(|(n, _)| n == from_name))
                .map_or_else(
                    || target_slews.get(arc.from_edge),
                    |(_, s)| s.get(arc.from_edge),
                );
            let (delay, slew) = arc.gate_delay(in_slew, load_cap);
            match arc.to_edge {
                Edge::Rise => {
                    delays.rise = delays.rise.max(delay);
                    slews.rise = slews.rise.max(slew);
                }
                Edge::Fall => {
                    delays.fall = delays.fall.max(delay);
                    slews.fall = slews.fall.max(slew);
                }
            }
        }
    }
    (delays, slews)
}

/// The worst slew over the delay arcs into `port` at `load_cap` from the
/// target input slews.
#[must_use]
pub fn gate_slew(cell: &LibCell, port: usize, slews: &EdgeValues, load_cap: f64) -> f64 {
    let (_, out) = gate_delays(cell, port, load_cap, slews, None);
    out.rise.max(out.fall).max(0.0)
}

/// The worst delay of a buffer or inverter at `load_cap` from the target
/// input slews, as `Resizer::bufferDelay`.
#[must_use]
pub fn buffer_delay(cell: &LibCell, slews: &EdgeValues, load_cap: f64) -> f64 {
    let Some((_, output)) = cell.buffer_ports() else {
        return 0.0;
    };
    let (delays, _) = gate_delays(cell, output, load_cap, slews, None);
    delays.rise.max(delays.fall).max(0.0)
}

/// The worst delay per edge of a buffer at `load_cap`, as
/// `Resizer::bufferDelays`.
#[must_use]
pub fn buffer_delays(cell: &LibCell, slews: &EdgeValues, load_cap: f64) -> EdgeValues {
    let Some((_, output)) = cell.buffer_ports() else {
        return EdgeValues::default();
    };
    let (delays, _) = gate_delays(cell, output, load_cap, slews, None);
    EdgeValues {
        rise: delays.rise.max(0.0),
        fall: delays.fall.max(0.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn slacks(values: &[f64]) -> Vec<(NetId, f64)> {
        values
            .iter()
            .enumerate()
            .map(|(i, &v)| (NetId::new(i), v))
            .collect()
    }

    #[test]
    fn the_worst_tenth_of_the_nets_is_weighted_from_the_maximum_down_to_one() {
        let mut values = vec![1.0; 17];
        values[0] = -2.0;
        values[1] = -1.0;
        let weights = timing_net_weights(&slacks(&values), 5.0);

        assert_eq!(weights.len(), 1);
        assert_eq!(weights[0].0, NetId::new(0));
        assert!((weights[0].1 - 5.0).abs() < 1e-12);
    }

    #[test]
    fn weights_fall_linearly_with_slack() {
        let mut values = vec![1.0; 30];
        values[0] = -3.0;
        values[1] = -2.0;
        values[2] = -1.0;
        let weights = timing_net_weights(&slacks(&values), 5.0);

        assert_eq!(weights.len(), 2);
        assert!((weights[0].1 - 5.0).abs() < 1e-12);
        assert!((weights[1].1 - 3.0).abs() < 1e-12);
    }

    #[test]
    fn equal_slacks_and_infinite_ones_give_no_weights() {
        assert_eq!(timing_net_weights(&slacks(&[0.5, 0.5, 0.5]), 5.0).len(), 0);
        assert_eq!(
            timing_net_weights(&slacks(&[f64::INFINITY, f64::INFINITY]), 5.0).len(),
            0
        );
        assert_eq!(timing_net_weights(&[], 5.0).len(), 0);
    }
}
