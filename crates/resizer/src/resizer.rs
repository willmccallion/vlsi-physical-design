//! The state a repair session works on, as `OpenROAD`'s `Resizer` holds it.
//!
//! The design, its timing graph and analysis, the buffers and target slews
//! repair sizes against, and a journal of edits so that a repair which did
//! not pay off can be taken back.

use crate::bnet::{self, BnetPtr, LoadModel, PinLoad, TreeRc};
use crate::buffers::{MergedNets, hierarchy_depth, merged_nets};
use crate::fuzzy;
use crate::graph::{Estimator, PortConditions, TimingGraph};
use crate::library::{BufferSet, Link, SizingLimits, is_logic_std_cell};
use crate::netlist::{ClockNetwork, Constants, Namer, Netlist};
use crate::parasitics::WireRc;
use crate::route_parasitics::RouteRc;
use crate::sta::{Slack, Sta};
use pare_common::convert::SaturatingFrom;
use pare_common::db::design::{EditError, NetlistDB, PinOwner, PinPlace, PlacementStatus};
use pare_common::db::global_route::RouteSource;
use pare_common::db::indices::{CellId, IdVec, MacroId, NetId, PinId};
use pare_common::db::liberty::{
    BufferCell, Edge, EdgeValues, LibCell, LibCellId, LibPort, LibPortId, Liberty, MinMax,
};
use pare_common::db::library::DuplicateName;
use pare_common::db::sdc::Sdc;
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::units::{CellArea, Farads, MICRONS_PER_METER, Meters, Ohms, Seconds};
use rustc_hash::{FxHashMap, FxHashSet};
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
    /// A net's global route reaches fewer of its pins than it has, as
    /// `OpenROAD`'s RSZ 74.
    #[error(
        "failed to build a tree from the global route of pin '{pin}': \
         found a route to {found} pins, expected {expected}"
    )]
    RouteTree {
        /// The net's driver.
        pin: String,
        /// The loads the route reaches.
        found: usize,
        /// The loads the net has.
        expected: usize,
    },
    /// Area recovery found no buffering of a net that meets its slack
    /// threshold, as `OpenROAD`'s RSZ 501, 2004 and 2022.
    #[error("buffering pin '{0}' failed: area recovery found no solution")]
    AreaRecovery(String),
    /// The library offers no buffer to insert.
    #[error("there are no buffers to repair with")]
    NoBuffers,
    /// No wire length lets a buffer drive its own input within its slew
    /// limit, as `OpenROAD`'s RSZ 2010 and 2011.
    #[error("cannot find the wire length buffer '{0}' drives within its load's slew limit")]
    WireLengthLimit(String),
    /// The library's slew thresholds give an unusable shape factor.
    #[error("the slew thresholds give an RC shape factor of {0:.3e}, outside 0.1 to 10")]
    ShapeFactor(f64),
}

/// Margins and limits for repair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RepairOptions {
    /// Percentage of each slew limit kept in hand.
    pub(crate) slew_margin: f64,
    /// Percentage of each capacitance limit kept in hand.
    pub(crate) cap_margin: f64,
    /// Longest wire allowed, if any.
    pub(crate) max_wire_length: Option<Dbu>,
    /// How much one sizing step may grow a cell.
    pub(crate) sizing_limits: SizingLimits,
}

impl Default for RepairOptions {
    fn default() -> Self {
        Self {
            slew_margin: 0.0,
            cap_margin: 0.0,
            max_wire_length: None,
            sizing_limits: SizingLimits::OPENROAD,
        }
    }
}

/// Global routes a session's parasitics follow, as after `estimate_parasitics
/// -global_routing`.
pub struct GlobalRouting<'a> {
    /// The routes, rerouted as edits change the nets.
    pub source: &'a mut dyn RouteSource,
    /// Each layer's resistance and capacitance.
    pub rc: RouteRc,
    /// Moves a cell to a legal site, as `Opendp::legalCellPos`, which every
    /// resized or inserted cell gets.
    pub legalize: &'a dyn Fn(&mut NetlistDB, CellId),
}

impl std::fmt::Debug for GlobalRouting<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GlobalRouting")
            .field("rc", &self.rc)
            .finish_non_exhaustive()
    }
}

/// What repair reads about the design besides the netlist itself.
#[derive(Clone, Copy, Debug)]
pub struct Context<'a> {
    /// The Liberty cells matched to the design's masters.
    pub link: &'a Link<'a>,
    /// The clock network, which repair leaves alone.
    pub clocks: &'a ClockNetwork,
    /// The constant pins, which repair leaves alone.
    pub constants: &'a Constants,
    /// The wire RC clock nets are estimated with, as `set_wire_rc -clock`
    /// sets it; without one, clock nets carry their pins' load alone.
    pub clock_rc: Option<WireRc>,
}

/// The estimator for a session's parasitics: along the global routes when
/// it has them, from the placement otherwise.
fn estimator<'r>(routes: Option<&'r GlobalRouting<'_>>) -> Estimator<'r> {
    routes.map_or(Estimator::Placement, |r| {
        Estimator::Routes(&*r.source, &r.rc)
    })
}

/// Target slews and the load each cell meets them at, as `OpenROAD`'s
/// `tgt_slews_` and `target_load_map_`.
#[derive(Clone, Debug)]
pub(crate) struct Targets {
    /// The slews repair sizes toward, per edge.
    pub slews: EdgeValues<Seconds>,
    /// The load at which each Liberty cell reaches them.
    pub load: IdVec<LibCellId, Farads>,
}

/// One edit, recorded so it can be undone.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Edit {
    Master {
        cell: CellId,
        old: MacroId,
    },
    Moved {
        cell: CellId,
        from: Point<Dbu>,
    },
    MovedPin {
        pin: PinId,
        from: PinPlace,
    },
    NewCell(CellId),
    NewNet(NetId),
    /// A retired cell, with where each of its pins stood before.
    Retired {
        cell: CellId,
        places: Vec<PinPlace>,
    },
    Renamed {
        net: NetId,
        old: String,
    },
}

// The three mode tokens below are drawn as state machines in
// docs/resizer-edit-modes.md.

/// A trial of edits, open from [`Resizer::journal_begin`] until
/// [`Resizer::journal_end`] keeps it or [`Resizer::journal_restore`] takes
/// it back. Only opening makes one and closing takes it, so a trial is
/// closed once, and only after it was opened.
#[must_use = "a trial is closed by journal_end or journal_restore"]
#[derive(Debug)]
pub(crate) struct Trial(Sealed);

/// Edits batched from [`Resizer::batch_edits`] until [`Resizer::flush`]
/// brings their parasitics and timing up to date.
#[must_use = "a batch is ended by flush"]
#[derive(Debug)]
pub(crate) struct EditBatch(Sealed);

/// Timing left behind the graph from [`Resizer::defer_timing`] until
/// [`Resizer::resume_timing`].
#[must_use = "deferred timing is brought up to date by resume_timing"]
#[derive(Debug)]
pub(crate) struct DeferredTiming(Sealed);

/// The private, uncopyable content of a mode token: only this module makes
/// one, and closing the mode moves it out.
#[derive(Debug)]
struct Sealed;

/// The nets and resized cells an undo touched, whose parasitics and timing
/// it leaves to be found again.
#[derive(Debug, Default)]
struct Touched {
    nets: FxHashSet<NetId>,
    cells: FxHashSet<CellId>,
}

impl Touched {
    /// Notes every net on a pin of `cell`.
    fn add_nets_of(&mut self, db: &NetlistDB, cell: CellId) {
        for &pin in &db.cells[cell].pins {
            let _ = self.nets.insert(db.pins[pin].net);
        }
    }
}

/// What an open [`EditBatch`] has yet to bring up to date.
#[derive(Clone, Debug)]
struct PendingEdits {
    /// Drivers whose timing is to be found again.
    drivers: Vec<PinId>,
    /// Nets whose parasitics are to be estimated again.
    nets: Vec<NetId>,
    /// How many nets the design had when the batch began: the batch's own
    /// edits read the nets it adds, so those are estimated as they change.
    nets_before: usize,
}

/// A buffer newly inserted before some loads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct InsertedBuffer {
    /// The buffer instance put in.
    pub cell: CellId,
    /// Its input pin, on the loads' old net.
    pub input: PinId,
    /// Its output pin, on the new net.
    pub output: PinId,
    /// The new net the loads moved to.
    pub net: NetId,
}

/// Where a session keeps its timing analysis: [`Untimed`] for none, or the
/// [`Sta`] itself, which every edit keeps up to date.
pub trait Timing: std::fmt::Debug {
    /// The analysis, if the session has one.
    fn analysis(&self) -> Option<&Sta>;
    /// The analysis to bring up to date, if the session has one.
    fn analysis_mut(&mut self) -> Option<&mut Sta>;
}

/// No timing analysis: a session given no constraints.
#[derive(Clone, Copy, Debug, Default)]
pub struct Untimed;

impl Timing for Untimed {
    fn analysis(&self) -> Option<&Sta> {
        None
    }

    fn analysis_mut(&mut self) -> Option<&mut Sta> {
        None
    }
}

impl Timing for Sta {
    fn analysis(&self) -> Option<&Sta> {
        Some(self)
    }

    fn analysis_mut(&mut self) -> Option<&mut Sta> {
        Some(self)
    }
}

/// The slews at a cell's inputs, by Liberty port name, which equivalent
/// cells share.
pub(crate) type PortSlews<'l> = FxHashMap<&'l str, EdgeValues<Seconds>>;

/// What a repair session works on: the design and what it reads about it,
/// the signal wire it estimates with, the buffers it may insert and the
/// namer of the cells and nets it adds.
#[derive(Debug)]
pub struct SessionInputs<'a> {
    /// The design repair edits.
    pub db: &'a mut NetlistDB,
    /// What repair reads about the design besides the netlist.
    pub context: Context<'a>,
    /// The wire RC signal nets are estimated with, as `set_wire_rc -signal`.
    pub rc: WireRc,
    /// The buffers repair may insert.
    pub buffers: BufferSet,
    /// Names the cells and nets repair adds.
    pub namer: &'a mut Namer,
}

/// The state of a repair session, timed when `T` is an [`Sta`].
#[derive(Debug)]
pub struct Resizer<'a, T: Timing = Untimed> {
    pub(crate) db: &'a mut NetlistDB,
    pub(crate) link: &'a Link<'a>,
    pub(crate) clocks: &'a ClockNetwork,
    pub(crate) constants: &'a Constants,
    pub(crate) graph: TimingGraph,
    timing: T,
    pub(crate) buffers: BufferSet,
    pub(crate) targets: Targets,
    pub(crate) namer: &'a mut Namer,
    pub(crate) options: RepairOptions,
    pub(crate) slew_rc_factor: f64,
    pub(crate) r_strongest_buffer: Ohms,
    /// The edits of the open [`Trial`]; none outside a trial.
    journal: Option<Vec<Edit>>,
    dead_nets: Vec<NetId>,
    /// The multi-output cells design repair resized, as
    /// `resized_multi_output_insts_`.
    // NOTE: RepairDesign::repairDesign clears this at each run; a PARE
    // session runs design repair at most once, before anything else
    // resizes, so a set the session's length holds the same cells.
    resized_multi_output: HashSet<CellId>,
    /// What the open [`EditBatch`] has yet to bring up to date; none
    /// outside a batch.
    pending: Option<PendingEdits>,
    /// The drivers the timing analysis has yet to take in while
    /// [`DeferredTiming`] holds it back; none while it is current.
    stale_timing: Option<Vec<PinId>>,
    routes: Option<GlobalRouting<'a>>,
}

impl<'a> Resizer<'a> {
    /// Prepares a session over `inputs` without timing: estimates every
    /// net's parasitics and slews.
    pub fn new(inputs: SessionInputs<'a>) -> Result<Self, RepairError> {
        Self::build(inputs, None, PortConditions::default(), |_, _| Untimed)
    }

    /// Prepares a session as [`Resizer::new`] does, with every net's
    /// parasitics along its global route, rerouted as edits change it.
    pub fn untimed_on_global_routes(
        inputs: SessionInputs<'a>,
        routes: GlobalRouting<'a>,
    ) -> Result<Self, RepairError> {
        Self::build(inputs, Some(routes), PortConditions::default(), |_, _| {
            Untimed
        })
    }
}

impl<'a> Resizer<'a, Sta> {
    /// Prepares a session over `inputs` as [`Resizer::new`] does, timed
    /// under `sdc`.
    pub fn timed(inputs: SessionInputs<'a>, sdc: &Sdc) -> Result<Self, RepairError> {
        Self::build_timed(inputs, None, sdc)
    }

    /// Prepares a timed session as [`Resizer::timed`] does, with every net's
    /// parasitics along its global route, rerouted as edits change it.
    pub fn on_global_routes(
        inputs: SessionInputs<'a>,
        sdc: &Sdc,
        routes: GlobalRouting<'a>,
    ) -> Result<Self, RepairError> {
        Self::build_timed(inputs, Some(routes), sdc)
    }

    fn build_timed(
        inputs: SessionInputs<'a>,
        routes: Option<GlobalRouting<'a>>,
        sdc: &Sdc,
    ) -> Result<Self, RepairError> {
        let (clocks, constants) = (inputs.context.clocks, inputs.context.constants);
        let ports = PortConditions::from_sdc(sdc);
        Self::build(inputs, routes, ports, |netlist, graph| {
            Sta::new(netlist, graph, clocks, constants, sdc)
        })
    }

    /// The timing analysis; not to be read while edits are batched or the
    /// timing is deferred, when it is behind the design.
    #[must_use]
    pub fn sta(&self) -> &Sta {
        debug_assert!(
            self.pending.is_none() && self.stale_timing.is_none(),
            "timing read while edits are batched or timing is deferred"
        );
        &self.timing
    }

    /// The setup slack of every net at its driver, as `OpenROAD`'s
    /// `findResizeSlacks1` collects them for timing-driven placement:
    /// constant, clock and special nets left out.
    #[must_use]
    pub fn driver_net_slacks(&self) -> Vec<(NetId, Slack)> {
        let sta = self.sta();
        let netlist = self.netlist();
        let mut slack_of: HashMap<NetId, Slack> = HashMap::new();
        for driver in self.graph.drivers_by_level(&netlist).into_iter().rev() {
            let net = self.db.pins[driver].net;
            if netlist.is_special(net)
                || self.constants.is_constant(driver)
                || self.clocks.is_clock(driver)
            {
                continue;
            }
            let _ = slack_of.insert(net, sta.slack(driver, MinMax::Max));
        }
        let mut slacks: Vec<(NetId, Slack)> = slack_of.into_iter().collect();
        slacks.sort_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)));
        slacks
    }
}

impl<'a, T: Timing> Resizer<'a, T> {
    /// Estimates every net's parasitics and slews under the port
    /// conditions `ports`, and times the design as `timing` does once the
    /// graph is built.
    fn build(
        inputs: SessionInputs<'a>,
        routes: Option<GlobalRouting<'a>>,
        ports: PortConditions,
        timing: impl FnOnce(&Netlist<'_>, &TimingGraph) -> T,
    ) -> Result<Self, RepairError> {
        let SessionInputs {
            db,
            context,
            rc,
            buffers,
            namer,
        } = inputs;
        let Context {
            link,
            clocks,
            constants,
            clock_rc,
        } = context;
        let slew_rc_factor = slew_rc_factor(link)?;
        let r_strongest_buffer = buffers
            .cells
            .iter()
            .map(|&b| link.buffer_drive_resistance(b))
            .reduce(Ohms::min)
            .ok_or(RepairError::NoBuffers)?;
        let targets = find_targets(link, &buffers);
        let (graph, timing) = {
            let netlist = Netlist::new(db, link);
            let graph = TimingGraph::new(
                &netlist,
                clocks,
                constants,
                rc,
                clock_rc,
                estimator(routes.as_ref()),
                ports,
            );
            let timing = timing(&netlist, &graph);
            (graph, timing)
        };
        Ok(Self {
            db,
            link,
            clocks,
            constants,
            graph,
            timing,
            buffers,
            targets,
            namer,
            options: RepairOptions::default(),
            slew_rc_factor,
            r_strongest_buffer,
            journal: None,
            dead_nets: Vec::new(),
            resized_multi_output: HashSet::new(),
            pending: None,
            stale_timing: None,
            routes,
        })
    }

    /// The buffering tree of the net `driver` drives, from its parasitics;
    /// none for a net without them.
    pub(crate) fn buffered_tree(&self, driver: PinId) -> Result<Option<BnetPtr>, RepairError> {
        let net = self.db.pins[driver].net;
        let Some(parasitics) = self.graph.parasitics(net) else {
            return Ok(None);
        };
        bnet::from_parasitics(&self.netlist(), parasitics, driver, &self.tree_rc(), self)
    }

    /// The wire and via RC buffering trees take.
    #[must_use]
    pub(crate) fn tree_rc(&self) -> TreeRc<'_> {
        TreeRc {
            signal: self.graph.wire_rc(),
            layers: self.routes.as_ref().map(|r| &r.rc),
            dbu: self.db.dbu(),
        }
    }

    /// Estimates `nets` again, rerouting them first when the parasitics
    /// follow global routes, as `EstimateParasitics::updateParasitics`.
    fn estimate_nets(&mut self, nets: &[NetId]) {
        if let Some(routes) = &mut self.routes {
            routes.source.update(self.db, nets);
        }
        let netlist = Netlist::new(self.db, self.link);
        for &net in nets {
            self.graph.parasitics_invalid(
                &netlist,
                self.clocks,
                net,
                estimator(self.routes.as_ref()),
            );
        }
    }

    /// Moves `cell` to a legal site when the parasitics follow global
    /// routes, as `OpenROAD`'s resizer does after resizing or inserting.
    fn legalize_for_routes(&mut self, cell: CellId) {
        let Some(routes) = &self.routes else {
            return;
        };
        let from = self.db.cells[cell].location;
        (routes.legalize)(self.db, cell);
        if self.db.cells[cell].location != from {
            self.record(Edit::Moved { cell, from });
        }
    }

    /// Holds back the parasitics and timing updates of the edits that
    /// follow until [`Resizer::flush`], so that a run of edits on a net is
    /// estimated and propagated once; nets the batch adds are still
    /// estimated as they change, since the batch's own edits read them.
    pub(crate) fn batch_edits(&mut self) -> EditBatch {
        debug_assert!(self.pending.is_none(), "edit batches do not nest");
        self.pending = Some(PendingEdits {
            drivers: Vec::new(),
            nets: Vec::new(),
            nets_before: self.db.nets.len(),
        });
        EditBatch(Sealed)
    }

    /// Estimates the nets and finds the timing again after the edits of
    /// `batch`, and ends it.
    pub(crate) fn flush(&mut self, batch: EditBatch) {
        let EditBatch(_sealed) = batch;
        let Some(PendingEdits {
            mut drivers,
            mut nets,
            ..
        }) = self.pending.take()
        else {
            return;
        };
        nets.sort_unstable();
        nets.dedup();
        self.estimate_nets(&nets);
        drivers.sort_unstable();
        drivers.dedup();
        self.propagate(&drivers);
    }

    /// Leaves the timing analysis behind the graph until
    /// [`Resizer::resume_timing`], for a run of edits that reads slews but
    /// no timing, as `OpenSTA` leaves arrivals until they are asked for.
    pub(crate) fn defer_timing(&mut self) -> DeferredTiming {
        debug_assert!(self.stale_timing.is_none(), "timing is deferred once");
        self.stale_timing = Some(Vec::new());
        DeferredTiming(Sealed)
    }

    /// Brings the timing analysis up to date with every edit made while
    /// `deferred` held it back.
    pub(crate) fn resume_timing(&mut self, deferred: DeferredTiming) {
        let DeferredTiming(_sealed) = deferred;
        let Some(mut stale) = self.stale_timing.take() else {
            return;
        };
        stale.sort_unstable();
        stale.dedup();
        if let Some(sta) = self.timing.analysis_mut() {
            let netlist = Netlist::new(self.db, self.link);
            sta.update(&netlist, &self.graph, self.clocks, &stale);
        }
    }

    /// Estimates a net's parasitics again, or holds that for the flush
    /// while edits are batched and the net is older than the batch.
    fn invalidate_parasitics(&mut self, net: NetId) {
        if let Some(pending) = self.pending.as_mut()
            && net.index() < pending.nets_before
        {
            pending.nets.push(net);
            return;
        }
        self.estimate_nets(&[net]);
    }

    /// A view of the design as it stands.
    #[must_use]
    pub const fn netlist(&self) -> Netlist<'_> {
        Netlist::new(self.db, self.link)
    }

    /// Whether the session's parasitics follow global routes.
    #[must_use]
    pub(crate) const fn has_global_routes(&self) -> bool {
        self.routes.is_some()
    }

    /// The timing graph.
    #[must_use]
    pub(crate) const fn graph(&self) -> &TimingGraph {
        &self.graph
    }

    /// Sets the longest wire design repair allows, in database units, or
    /// none for no limit.
    pub const fn set_max_wire_length(&mut self, length: Option<Dbu>) {
        self.options.max_wire_length = length;
    }

    /// Drops every retired cell and emptied net from the design and ends
    /// the session: every id held before is stale afterwards.
    pub fn finish(self) -> Result<pare_common::db::design::Renumbering, RepairError> {
        let dead: Vec<NetId> = self
            .dead_nets
            .iter()
            .copied()
            .filter(|&n| self.db.nets[n].pins.is_empty())
            .collect();
        Ok(self.db.remove(&[], &dead)?)
    }

    /// The transition limit at an input pin, as [`max_input_slew`] finds
    /// it under the session's constraints.
    #[must_use]
    pub(crate) fn max_input_slew(&self, cell: &LibCell, port: &LibPort) -> Seconds {
        max_input_slew(self.link, self.design_slew_limit(), cell, port)
    }

    /// The transition limit the constraints set on the whole design.
    fn design_slew_limit(&self) -> Option<Seconds> {
        self.timing
            .analysis()
            .and_then(|s| s.sdc().limits.max_transition)
    }

    /// The slew limit at a load pin, where one applies: an instance pin's
    /// [`max_input_slew`], a design pin's from the constraints, as
    /// `CheckSlews::findLimit`.
    fn load_slew_limit(&self, load: PinId) -> Option<Seconds> {
        match self.netlist().lib_cell_port(load) {
            Some((cell, port)) => Some(self.max_input_slew(cell, port)).filter(|l| l.is_finite()),
            None if matches!(self.db.pins[load].owner, PinOwner::Io(_)) => self.design_slew_limit(),
            None => None,
        }
    }

    /// The late slew and the slew limit of each load of `driver`'s net
    /// that has a limit.
    #[must_use]
    pub(crate) fn load_slews(&self, driver: PinId) -> Vec<(Seconds, Seconds)> {
        let netlist = self.netlist();
        let net = self.db.pins[driver].net;
        netlist
            .loads(net)
            .filter(|&load| load != driver)
            .filter_map(|load| {
                let limit = self.load_slew_limit(load)?;
                Some((self.graph.slew(load).max(), limit))
            })
            .collect()
    }

    /// Holds every input slew over its limit at that limit, each before
    /// anything downstream of it is timed, as `OpenROAD`'s `repairDesign`
    /// forward pass, and brings the timing up to date; returns the pins
    /// held.
    pub(crate) fn annotate_violating_input_slews(&mut self) -> Vec<PinId> {
        let (annotated, mut dirty) = {
            let netlist = Netlist::new(self.db, self.link);
            let link = self.link;
            let design_limit = self.design_slew_limit();
            let limit_of = |pin: PinId| {
                netlist
                    .lib_cell_port(pin)
                    .map(|(cell, port)| max_input_slew(link, design_limit, cell, port))
            };
            self.graph
                .sweep_annotating(&netlist, self.clocks, &limit_of)
        };
        let netlist = Netlist::new(self.db, self.link);
        for &(load, _) in &annotated {
            dirty.extend(self.graph.drivers_fed_by(&netlist, load));
        }
        dirty.sort_unstable();
        dirty.dedup();
        if let Some(sta) = self.timing.analysis_mut() {
            sta.update(&netlist, &self.graph, self.clocks, &dirty);
        }
        annotated.into_iter().map(|(load, _)| load).collect()
    }

    /// A slew limit less the margin kept in hand.
    #[must_use]
    pub(crate) fn margined_slew(&self, limit: Seconds) -> Seconds {
        limit * (1.0 - self.options.slew_margin / 100.0)
    }

    /// A load limit less the margin kept in hand.
    #[must_use]
    pub(crate) fn margined_cap(&self, limit: Farads) -> Farads {
        limit * (1.0 - self.options.cap_margin / 100.0)
    }

    /// The slew limit at a driver, if any applies, as `OpenSTA` finds it.
    #[must_use]
    pub(crate) fn driver_slew_limit(&self, driver: PinId) -> Option<Seconds> {
        let netlist = self.netlist();
        let own = netlist
            .lib_cell_port(driver)
            .and_then(|(c, p)| self.link.liberty.slew_limit(c, p));
        match (own, self.design_slew_limit()) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// The load limit at a driver, if any applies.
    #[must_use]
    pub(crate) fn cap_limit(&self, driver: PinId) -> Option<Farads> {
        match self.netlist().lib_cell_port(driver) {
            Some((cell, port)) => self.port_cap_limit(cell, port),
            None => self.design_cap_limit(),
        }
    }

    /// The load limit on an output port of `cell`: the tighter of its own,
    /// or its library's default, and the constraints' design limit, as
    /// `Resizer::maxLoad` reads it.
    #[must_use]
    pub(crate) fn port_cap_limit(&self, cell: &LibCell, port: &LibPort) -> Option<Farads> {
        let own = self.link.liberty.capacitance_limit(cell, port);
        match (own, self.design_cap_limit()) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    /// The load limit the constraints set on the whole design.
    fn design_cap_limit(&self) -> Option<Farads> {
        self.timing
            .analysis()
            .and_then(|s| s.sdc().limits.max_capacitance)
    }

    /// The fanout limit at a driver, if any applies, as
    /// `CheckFanouts::findLimit`: the tighter of the design's
    /// `set_max_fanout` and, at an instance output, its port's `max_fanout`
    /// or its library's default.
    #[must_use]
    pub(crate) fn fanout_limit(&self, driver: PinId) -> Option<f64> {
        let design = self
            .timing
            .analysis()
            .and_then(|s| s.sdc().limits.max_fanout);
        let own = self
            .netlist()
            .lib_cell_port(driver)
            .and_then(|(cell, port)| {
                port.max_fanout
                    .or_else(|| self.link.liberty.cell_defaults(cell).max_fanout)
            });
        let limit = match (own, design) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        };
        // A limit of zero limits nothing, as checkFanout's callers read it.
        limit.filter(|&l| l > 0.0)
    }

    /// The fanout a driver's net presents, as `CheckFanouts::fanoutLoad`:
    /// each instance load's Liberty fanout load, one for a load without a
    /// Liberty port, nothing for a port of the design.
    #[must_use]
    pub(crate) fn fanout_load(&self, driver: PinId) -> f64 {
        let netlist = self.netlist();
        let net = self.db.pins[driver].net;
        netlist
            .loads(net)
            .filter(|&load| load != driver)
            .filter(|&load| matches!(self.db.pins[load].owner, PinOwner::Cell { .. }))
            .map(|load| {
                netlist.lib_cell_port(load).map_or(1.0, |(cell, port)| {
                    self.link.liberty.fanout_load(cell, port)
                })
            })
            .sum()
    }

    /// The load a driver sees.
    #[must_use]
    pub fn load_cap(&self, driver: PinId) -> Farads {
        self.graph.load_cap(&self.netlist(), driver)
    }

    /// How many loads a driver's net has besides the driver.
    #[must_use]
    pub fn fanout(&self, driver: PinId) -> usize {
        let netlist = self.netlist();
        let net = self.db.pins[driver].net;
        netlist.loads(net).filter(|&l| l != driver).count()
    }

    /// Whether a pin drives through a tristate output.
    #[must_use]
    pub(crate) fn is_tristate_driver(&self, driver: PinId) -> bool {
        self.netlist()
            .lib_port(driver)
            .is_some_and(|p| p.tristate_enable.is_some())
    }

    /// Whether an instance may be resized or removed: a plain standard
    /// cell the library does not protect.
    // NOTE: OpenROAD protects only set_dont_touch instances, which PARE
    // lacks; PARE honours the library's dont_touch in their stead.
    #[must_use]
    pub(crate) fn is_editable(&self, cell: CellId) -> bool {
        self.link
            .cell(self.db, cell)
            .is_some_and(|(_, lib)| !lib.dont_touch)
            && is_logic_std_cell(self.db, cell)
    }

    /// Whether a net may be buffered, as `Resizer::okToBufferNet`: not
    /// driven by a tristate, not a supply.
    #[must_use]
    pub(crate) fn ok_to_buffer_net(&self, driver: PinId) -> bool {
        let net = self.db.pins[driver].net;
        !self.is_tristate_driver(driver) && !self.netlist().is_special(net)
    }

    /// The slews at each input pin of `cell`, by Liberty port name.
    #[must_use]
    pub(crate) fn input_slews(&self, cell: CellId) -> PortSlews<'_> {
        let netlist = self.netlist();
        self.db.cells[cell]
            .pins
            .iter()
            .filter_map(|&pin| {
                let port = netlist.lib_port(pin)?;
                Some((port.name.as_str(), self.graph.slew(pin)))
            })
            .collect()
    }

    /// The worst delay of a buffer or inverter at `load_cap` from the
    /// target input slews, as `Resizer::bufferDelay`.
    #[must_use]
    pub fn buffer_delay(&self, buffer: BufferCell, load_cap: Farads) -> Seconds {
        buffer_delay(self.link.liberty, buffer, &self.targets.slews, load_cap)
    }

    /// `cell` with its buffer ports, `cell` being a size of a buffer: the
    /// sizes of a cell share its function and ports, so it is a buffer too.
    pub(crate) fn as_buffer(&self, cell: LibCellId) -> Result<BufferCell, RepairError> {
        let liberty = self.link.liberty;
        liberty
            .buffer_cell(cell)
            .ok_or_else(|| RepairError::MissingPins(liberty.cell(cell).name.clone()))
    }

    /// The size of `buffer` that drives `load_cap` at the target slews, as
    /// `findTargetCell` sizes a repeater.
    pub(crate) fn find_target_buffer(
        &self,
        buffer: BufferCell,
        load_cap: Farads,
    ) -> Result<BufferCell, RepairError> {
        self.as_buffer(self.find_target_cell(buffer.id, load_cap, false))
    }

    /// The design's area, over cells of the core.
    #[must_use]
    pub fn design_area(&self) -> CellArea {
        self.db
            .cells
            .iter()
            .enumerate()
            .filter(|(_, c)| !c.retired)
            .filter_map(|(i, _)| self.link.cell(self.db, CellId::new(i)))
            .map(|(_, lib)| lib.area)
            .sum()
    }

    /// Opens a trial, recording the edits that follow, as `journalBegin`.
    pub(crate) fn journal_begin(&mut self) -> Trial {
        debug_assert!(self.journal.is_none(), "trials do not nest");
        self.journal = Some(Vec::new());
        if let Some(routes) = self.routes.as_mut() {
            routes.source.begin_trial();
        }
        Trial(Sealed)
    }

    /// Keeps the edits of `trial`, as `journalEnd`.
    pub(crate) fn journal_end(&mut self, trial: Trial) {
        let Trial(_sealed) = trial;
        self.journal = None;
        if let Some(routes) = self.routes.as_mut() {
            routes.source.keep_trial();
        }
    }

    /// Takes back every edit of `trial`, puts back the global routes it
    /// changed, and brings the timing up to date, as `journalRestore`.
    pub(crate) fn journal_restore(&mut self, trial: Trial) -> Result<(), RepairError> {
        let Trial(_sealed) = trial;
        let edits = self.journal.take().unwrap_or_default();
        let mut touched = Touched::default();
        for edit in edits.into_iter().rev() {
            self.undo(edit, &mut touched)?;
        }
        if let Some(routes) = self.routes.as_mut() {
            routes.source.restore_trial();
        }
        self.retime_touched(touched);
        Ok(())
    }

    /// Takes back one journaled edit, noting the nets and cells it touched.
    fn undo(&mut self, edit: Edit, touched: &mut Touched) -> Result<(), RepairError> {
        match edit {
            Edit::Master { cell, old } => {
                self.set_master(cell, old)?;
                let _ = touched.cells.insert(cell);
                touched.add_nets_of(self.db, cell);
            }
            Edit::Moved { cell, from } => {
                self.db.cells[cell].location = from;
                touched.add_nets_of(self.db, cell);
            }
            Edit::MovedPin { pin, from } => {
                let current = self.db.pins[pin].net;
                let _ = touched.nets.insert(current);
                let _ = touched.nets.insert(from.net);
                self.db.put_pin_back(pin, from);
                self.graph
                    .pin_moved(&Netlist::new(self.db, self.link), pin, current);
            }
            Edit::NewCell(cell) => {
                touched.add_nets_of(self.db, cell);
                let _ = self.retire(cell);
            }
            Edit::NewNet(net) => self.dead_nets.push(net),
            Edit::Retired { cell, places } => {
                self.unretire(cell, places);
                touched.add_nets_of(self.db, cell);
            }
            Edit::Renamed { net, old } => self.db.rename_net(net, old)?,
        }
        Ok(())
    }

    /// Estimates the nets an undo touched again and finds the timing again
    /// from their drivers and the drivers around the cells it resized.
    fn retime_touched(&mut self, touched: Touched) {
        {
            let netlist = Netlist::new(self.db, self.link);
            self.graph.grow(&netlist);
        }
        let mut nets: Vec<NetId> = touched.nets.into_iter().collect();
        nets.sort_unstable();
        self.estimate_nets(&nets);
        let mut dirty = Vec::new();
        {
            let netlist = Netlist::new(self.db, self.link);
            for net in nets {
                dirty.extend_from_slice(self.graph.drivers(net));
            }
            for cell in touched.cells {
                for &pin in &self.db.cells[cell].pins {
                    if netlist.is_load(pin) {
                        dirty.extend_from_slice(self.graph.drivers(self.db.pins[pin].net));
                    }
                    if netlist.is_driver(pin) {
                        dirty.push(pin);
                    }
                }
            }
        }
        self.propagate(&dirty);
    }

    fn record(&mut self, edit: Edit) {
        if let Some(edits) = self.journal.as_mut() {
            edits.push(edit);
        }
    }

    /// Drops the slew annotations and finds the slews they held fixed
    /// again, with everything downstream, as `OpenROAD` invalidates its
    /// delays once design repair's annotations come off.
    pub(crate) fn clear_slew_annotations(&mut self) {
        let annotated = self.graph.clear_annotations();
        let mut drivers: Vec<PinId> = Vec::new();
        {
            let netlist = Netlist::new(self.db, self.link);
            for pin in annotated {
                if netlist.is_driver(pin) {
                    drivers.push(pin);
                }
                if netlist.is_load(pin) {
                    drivers.extend_from_slice(self.graph.drivers(self.db.pins[pin].net));
                }
            }
        }
        drivers.sort_unstable();
        drivers.dedup();
        self.propagate(&drivers);
    }

    /// Finds slews, delays and timing again from `drivers` on, or holds
    /// them for the flush while edits are batched.
    fn propagate(&mut self, drivers: &[PinId]) {
        if let Some(pending) = self.pending.as_mut() {
            pending.drivers.extend_from_slice(drivers);
            return;
        }
        let netlist = Netlist::new(self.db, self.link);
        let dirty = self.graph.propagate_from(&netlist, self.clocks, drivers);
        if let Some(stale) = self.stale_timing.as_mut() {
            stale.extend(dirty);
        } else if let Some(sta) = self.timing.analysis_mut() {
            sta.update(&netlist, &self.graph, self.clocks, &dirty);
        }
    }

    /// Re-estimates a net and everything timed through it.
    pub(crate) fn update_net(&mut self, net: NetId) {
        self.invalidate_parasitics(net);
        let drivers = self.graph.drivers(net).to_vec();
        self.propagate(&drivers);
    }

    /// Re-estimates the nets around a cell whose master changed.
    pub(crate) fn update_cell(&mut self, cell: CellId) {
        let (nets, drivers) = {
            let netlist = Netlist::new(self.db, self.link);
            let mut nets = Vec::new();
            let mut drivers = Vec::new();
            for &pin in &self.db.cells[cell].pins {
                let net = self.db.pins[pin].net;
                if netlist.is_load(pin) || netlist.is_driver(pin) {
                    nets.push(net);
                }
                if netlist.is_load(pin) {
                    drivers.extend_from_slice(self.graph.drivers(net));
                }
                if netlist.is_driver(pin) {
                    drivers.push(pin);
                }
            }
            (nets, drivers)
        };
        for net in nets {
            self.invalidate_parasitics(net);
        }
        self.propagate(&drivers);
    }

    /// Gives `cell` the master of `size` and refreshes the timing around
    /// it; returns whether it changed.
    pub(crate) fn replace_cell(
        &mut self,
        cell: CellId,
        size: LibCellId,
    ) -> Result<bool, RepairError> {
        let master = self
            .link
            .macro_of(size)
            .ok_or_else(|| RepairError::NoMaster(self.link.liberty.cell(size).name.clone()))?;
        let old = self.db.cells[cell].macro_id;
        if old == master {
            return Ok(false);
        }
        self.set_master(cell, master)?;
        self.record(Edit::Master { cell, old });
        self.legalize_for_routes(cell);
        self.update_cell(cell);
        Ok(true)
    }

    /// Moves a pin to another net, recording it.
    pub fn move_pin(&mut self, pin: PinId, to: NetId) {
        let from = self.db.pin_place(pin);
        if from.net == to {
            return;
        }
        self.db.move_pin(pin, to);
        self.graph
            .pin_moved(&Netlist::new(self.db, self.link), pin, from.net);
        self.record(Edit::MovedPin { pin, from });
    }

    /// Gives `cell` the master `master`, the timing graph following which
    /// of its pins drive.
    fn set_master(&mut self, cell: CellId, master: MacroId) -> Result<(), RepairError> {
        self.db.replace_master(cell, master)?;
        self.graph
            .master_replaced(&Netlist::new(self.db, self.link), cell);
        Ok(())
    }

    /// Takes `cell` off its nets, the timing graph following; returns where
    /// its pins stood.
    fn retire(&mut self, cell: CellId) -> Vec<PinPlace> {
        let places = self.db.retire_cell(cell);
        for &pin in &self.db.cells[cell].pins {
            self.graph.pin_disconnected(self.db.pins[pin].net, pin);
        }
        places
    }

    /// Puts a retired `cell` back where `places` say its pins stood, the
    /// timing graph following.
    fn unretire(&mut self, cell: CellId, places: Vec<PinPlace>) {
        self.db.unretire_cell(cell, places);
        let netlist = Netlist::new(self.db, self.link);
        for &pin in &self.db.cells[cell].pins {
            self.graph.pin_connected(&netlist, pin);
        }
    }

    /// Adds an instance of `cell` at `at`, named from `base`, recording
    /// it; its pins are connected by the caller.
    pub(crate) fn make_instance(
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
        self.legalize_for_routes(id);
        Ok(id)
    }

    /// Adds a net named from `base` with the signal use of `like`,
    /// recording it.
    pub(crate) fn make_net(&mut self, base: &str, like: NetId) -> Result<NetId, RepairError> {
        let name = self.namer.net(self.db, base);
        let net = self.db.add_net(&name)?;
        self.db.nets[net].signal_use = self.db.nets[like].signal_use;
        self.record(Edit::NewNet(net));
        Ok(net)
    }

    /// Connects Liberty port `port` of `cell` to `net`.
    pub fn connect(
        &mut self,
        cell: CellId,
        lib: LibCellId,
        port: LibPortId,
        net: NetId,
    ) -> Option<PinId> {
        let macro_pin = self.link.macro_pin_of_port(lib, port)?;
        let pin = self.db.connect_cell_pin(cell, macro_pin, net);
        self.graph
            .pin_connected(&Netlist::new(self.db, self.link), pin);
        Some(pin)
    }

    /// Puts a buffer of `buffer` at `at` between `net` and `loads`, which
    /// move to a new net the buffer drives, as `Resizer::insertBufferBeforeLoads`;
    /// the instance is named from `base`. The timing around both nets is
    /// refreshed.
    pub(crate) fn insert_buffer_before_loads(
        &mut self,
        net: NetId,
        loads: &[PinId],
        buffer: BufferCell,
        at: Point<Dbu>,
        base: &str,
    ) -> Result<InsertedBuffer, RepairError> {
        let lib = self.link.liberty.cell(buffer.id);
        let cell = self.make_instance(buffer.id, at, base)?;
        let new_net = self.make_net("net", net)?;
        for &load in loads {
            self.move_pin(load, new_net);
        }
        let input = self
            .connect(cell, buffer.id, buffer.input, net)
            .ok_or_else(|| RepairError::MissingPins(lib.name.clone()))?;
        let output = self
            .connect(cell, buffer.id, buffer.output, new_net)
            .ok_or_else(|| RepairError::MissingPins(lib.name.clone()))?;
        {
            let netlist = Netlist::new(self.db, self.link);
            self.graph.grow(&netlist);
        }
        self.invalidate_parasitics(net);
        self.invalidate_parasitics(new_net);
        let mut drivers = self.graph.drivers(net).to_vec();
        drivers.push(output);
        self.propagate(&drivers);
        Ok(InsertedBuffer {
            cell,
            input,
            output,
            net: new_net,
        })
    }

    /// Takes a buffer out, joining its nets as [`merged_nets`] picks the
    /// survivor, which takes the removed net's name when that name is
    /// shallower in the hierarchy, as `UnbufferMove::removeBuffer`. Returns
    /// the surviving net.
    pub(crate) fn remove_buffer(&mut self, buffer: CellId) -> Result<NetId, RepairError> {
        let (in_pin, out_pin) = self
            .buffer_pins(buffer)
            .ok_or_else(|| RepairError::MissingPins(self.db.cells[buffer].name.clone()))?;
        let in_net = self.db.pins[in_pin].net;
        let out_net = self.db.pins[out_pin].net;
        let MergedNets { survivor, removed } = merged_nets(&self.netlist(), in_net, out_net);
        let moving: Vec<PinId> = self.db.nets[removed]
            .pins
            .iter()
            .copied()
            .filter(|&p| p != in_pin && p != out_pin)
            .collect();
        let survivor_name = self.db.nets[survivor].name.clone();
        let removed_name = self.db.nets[removed].name.clone();
        let places = self.retire(buffer);
        self.record(Edit::Retired {
            cell: buffer,
            places,
        });
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
        self.invalidate_parasitics(survivor);
        self.invalidate_parasitics(removed);
        let drivers = self.graph.drivers(survivor).to_vec();
        self.propagate(&drivers);
        Ok(survivor)
    }

    /// The input and output pins of a buffer instance.
    #[must_use]
    pub(crate) fn buffer_pins(&self, buffer: CellId) -> Option<(PinId, PinId)> {
        let (lib_id, _) = self.link.cell(self.db, buffer)?;
        let ports = self.link.liberty.buffer_cell(lib_id)?;
        let netlist = Netlist::new(self.db, self.link);
        Some((
            netlist.pin_of_port(buffer, ports.input)?,
            netlist.pin_of_port(buffer, ports.output)?,
        ))
    }

    /// The cells an instance of `cell` may be sized to, within the sizing
    /// limits in force.
    #[must_use]
    pub(crate) fn swappable_cells(&self, cell: LibCellId) -> Vec<LibCellId> {
        self.link.swappable_cells(cell, self.options.sizing_limits)
    }

    /// Sizes the cell driving `driver` to the size whose target load is
    /// nearest its load, as `Resizer::resizeToTargetSlew`; returns whether
    /// the cell changed.
    pub(crate) fn resize_to_target_slew(&mut self, driver: PinId) -> Result<bool, RepairError> {
        let (cell, lib_id, load_cap, revisiting) = {
            let PinOwner::Cell { cell, .. } = self.db.pins[driver].owner else {
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
        if load_cap <= Farads::ZERO {
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
    pub(crate) fn find_target_cell(
        &self,
        cell: LibCellId,
        load_cap: Farads,
        revisiting: bool,
    ) -> LibCellId {
        let candidates = self.swappable_cells(cell);
        if candidates.is_empty() {
            return cell;
        }
        let lib = self.link.liberty.cell(cell);
        let is_buf_inv = lib.is_buffer() || lib.is_inverter();
        let target_of = |c: LibCellId| self.targets.load[c];
        let delay_of = |c: LibCellId| match self.link.liberty.buffer_cell(c) {
            Some(buffer) if is_buf_inv => self.buffer_delay(buffer, load_cap),
            _ => Seconds::ZERO,
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
    pub(crate) fn replacement_keeps_loads(&self, cell: CellId, replacement: LibCellId) -> bool {
        let netlist = self.netlist();
        let Some((_, current)) = self.link.cell(self.db, cell) else {
            return true;
        };
        let new_lib = self.link.liberty.cell(replacement);
        for &pin in &self.db.cells[cell].pins {
            let Some(port) = netlist.lib_port(pin) else {
                continue;
            };
            if !port.is_input() {
                continue;
            }
            let old_cap = current
                .port(&port.name)
                .map_or(Farads::ZERO, |p| current.ports[p].capacitance());
            let new_cap = new_lib
                .port(&port.name)
                .map_or(Farads::ZERO, |p| new_lib.ports[p].capacitance());
            let delta = new_cap - old_cap;
            if delta <= Farads::ZERO {
                continue;
            }
            let net = self.db.pins[pin].net;
            for &driver in self.graph.drivers(net) {
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
    pub(crate) fn cap_ok(&self, driver: PinId, delta: Farads) -> bool {
        let Some(limit) = self.cap_limit(driver) else {
            return true;
        };
        if limit <= Farads::ZERO {
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

impl<T: Timing> LoadModel for Resizer<'_, T> {
    /// What `pin` presents, as `BufferedNet`'s load constructor reads it: an
    /// instance pin's Liberty port, a design pin's load from `set_load`.
    fn load(&self, pin: PinId) -> PinLoad {
        let netlist = self.netlist();
        if let PinOwner::Io(_) = self.db.pins[pin].owner {
            return PinLoad {
                cap: self.graph.pin_capacitance(&netlist, pin, Edge::Rise),
                ..PinLoad::NONE
            };
        }
        netlist
            .lib_cell_port(pin)
            .map_or(PinLoad::NONE, |(cell, port)| PinLoad {
                cap: port.capacitance(),
                fanout: self.link.liberty.fanout_load(cell, port),
                max_slew: self.max_input_slew(cell, port),
            })
    }
}

/// Share of the nets, from the worst slack up, that timing-driven
/// placement weights, as `OpenROAD`'s `worst_slack_nets_percent`.
pub(crate) const WORST_SLACK_NETS_SHARE: f64 = 0.1;

/// The wirelength weights timing-driven placement gives nets for their
/// `slacks`, sorted worst first, as `OpenROAD`'s `TimingBase` does.
///
/// Over the worst tenth of them, the weight runs from
/// `weight_max` at the worst slack down to one at the best; nets at or
/// above that best keep a weight of one and are left out. Empty when no
/// net is constrained.
#[must_use]
pub fn timing_net_weights(slacks: &[(NetId, Slack)], weight_max: f64) -> Vec<(NetId, f64)> {
    let Some(slack_min) = slacks.first().and_then(|&(_, slack)| slack.time()) else {
        return Vec::new();
    };
    let worst = usize::saturating_from(((slacks.len() as f64) * WORST_SLACK_NETS_SHARE).ceil());
    let slack_max = slacks[worst.clamp(1, slacks.len()) - 1].1;
    let scale = WeightScale {
        slack_min,
        slack_max,
        weight_max,
    };
    slacks
        .iter()
        .filter(|(_, slack)| *slack < slack_max)
        .filter_map(|&(net, slack)| Some((net, scale.weight(slack.time()?))))
        .collect()
}

/// Net weights falling linearly from `weight_max` at `slack_min` to one at
/// `slack_max`.
struct WeightScale {
    slack_min: Seconds,
    slack_max: Slack,
    weight_max: f64,
}

impl WeightScale {
    /// The weight at `slack`. An unconstrained `slack_max` gives every
    /// constrained slack `weight_max`, as `OpenSTA`'s finite `INF` does in
    /// `gpl`.
    fn weight(&self, slack: Seconds) -> f64 {
        let Some(max) = self.slack_max.time() else {
            return self.weight_max;
        };
        if fuzzy::equal(max.get(), self.slack_min.get()) {
            return 1.0;
        }
        ((self.weight_max - 1.0) * (max - slack).get() / (max - self.slack_min).get()) + 1.0
    }
}

/// The transition limit at an input pin, as `Resizer::maxInputSlew`: the
/// tighter of its own and the constraints' `design_limit`, else, as
/// `OpenROAD` fixes up for libraries like Nangate45, the library default
/// meant for outputs; infinite without any.
fn max_input_slew(
    link: &Link<'_>,
    design_limit: Option<Seconds>,
    cell: &LibCell,
    port: &LibPort,
) -> Seconds {
    let found = match (port.max_transition, design_limit) {
        (Some(own), Some(design)) => Some(own.min(design)),
        (own, design) => own.or(design),
    };
    found
        .filter(|&l| l != Seconds::ZERO)
        .or_else(|| link.liberty.cell_defaults(cell).max_transition)
        .unwrap_or(Seconds::INFINITY)
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

/// How much `after` grew over `before`, as a percentage; zero when there
/// was no area before.
#[must_use]
pub fn area_change_percent(before: CellArea, after: CellArea) -> f64 {
    if before > CellArea::ZERO {
        (after - before).get() / before.get() * 100.0
    } else {
        0.0
    }
}

/// `meters` in database units, rounded and kept within an `int`, as
/// `Resizer::metersToDbu`.
pub(crate) fn meters_to_dbu(meters: Meters, dbu: DbuPerMicron) -> Dbu {
    // Masking the low 31 bits is what `int` truncation and `& INT_MAX` keep.
    let rounded =
        i64::saturating_from((meters.get() * f64::from(dbu.get()) * MICRONS_PER_METER).round());
    Dbu::new(rounded & i64::from(i32::MAX))
}

/// Target slews from the buffers and the load each usable cell meets them
/// at, as `Resizer::findBufferTargetSlews` and `findTargetLoads`.
pub(crate) fn find_targets(link: &Link<'_>, buffers: &BufferSet) -> Targets {
    let liberty = link.liberty;
    let mut sums = EdgeValues::<Seconds>::default();
    let mut counts = EdgeValues::<usize>::default();
    for &buffer in &buffers.cells {
        let cell = liberty.cell(buffer.id);
        let (input, output) = (buffer.input, buffer.output);
        for set in cell
            .arc_sets
            .iter()
            .filter(|s| s.from == input && s.to == output)
        {
            for arc in &set.arcs {
                let in_cap = cell.ports[input].capacitance.get(arc.from_edge);
                let load = in_cap * TARGET_SLEW_LOAD_FACTOR;
                let (_, slew1) = arc.gate_delay(Seconds::ZERO, load);
                let (_, slew2) = arc.gate_delay(slew1, load);
                sums[arc.to_edge] += slew2;
                counts[arc.to_edge] += 1;
            }
        }
    }
    let average = |edge: Edge| {
        if counts[edge] > 0 {
            sums[edge] / counts[edge] as f64
        } else {
            Seconds::ZERO
        }
    };
    let slews = EdgeValues {
        rise: average(Edge::Rise),
        fall: average(Edge::Fall),
    };
    let load = liberty
        .cells
        .ids()
        .map(|id| {
            if link.is_link_cell(id) && !link.dont_use(id) {
                find_target_load(liberty.cell(id), &slews)
            } else {
                Farads::ZERO
            }
        })
        .collect();
    Targets { slews, load }
}

/// The mean over a cell's delay arcs of the load at which each reaches the
/// target output slew from the target input slew.
fn find_target_load(cell: &LibCell, slews: &EdgeValues<Seconds>) -> Farads {
    let mut sum = Farads::ZERO;
    let mut count = 0;
    for set in cell.arc_sets.iter().filter(|s| s.role.is_logic_delay()) {
        for arc in &set.arcs {
            let in_slew = slews.get(arc.from_edge);
            let out_slew = slews.get(arc.to_edge);
            sum += arc_target_load(arc, in_slew, out_slew);
            count += 1;
        }
    }
    if count == 0 {
        Farads::ZERO
    } else {
        sum / f64::from(count)
    }
}

/// The load at which `arc` gives `out_slew` from `in_slew`, by binary
/// search; zero when even no load exceeds it.
fn arc_target_load(
    arc: &pare_common::db::liberty::TimingArc,
    in_slew: Seconds,
    out_slew: Seconds,
) -> Farads {
    let diff = |load: f64| (arc.gate_delay(in_slew, Farads::new(load)).1 - out_slew).get();
    let mut cap1 = 0.0;
    let mut cap2 = 1.0e-12;
    let tol = 0.01;
    if diff(cap1) > 0.0 {
        return Farads::ZERO;
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
    Farads::new(cap1)
}

/// The worst delay and slew per output edge over the delay arcs into
/// `port` at `load_cap`, as `Resizer::gateDelays`.
///
/// Each arc is fed the slew of its input port from `input_slews` or,
/// failing that, the target slews.
#[must_use]
pub(crate) fn gate_delays(
    cell: &LibCell,
    port: LibPortId,
    load_cap: Farads,
    target_slews: &EdgeValues<Seconds>,
    input_slews: Option<&PortSlews<'_>>,
) -> (EdgeValues<Seconds>, EdgeValues<Seconds>) {
    let mut delays = EdgeValues::both(-Seconds::INFINITY);
    let mut slews = delays;
    for set in cell.delay_arc_sets_to(port) {
        let from_name = &cell.ports[set.from].name;
        for arc in &set.arcs {
            let in_slew = input_slews
                .and_then(|slews| slews.get(from_name.as_str()))
                .map_or_else(|| target_slews.get(arc.from_edge), |s| s.get(arc.from_edge));
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
pub(crate) fn gate_slew(
    cell: &LibCell,
    port: LibPortId,
    slews: &EdgeValues<Seconds>,
    load_cap: Farads,
) -> Seconds {
    let (_, out) = gate_delays(cell, port, load_cap, slews, None);
    out.rise.max(out.fall).max(Seconds::ZERO)
}

/// The worst delay of `buffer` of `liberty` at `load_cap` from the target
/// input slews, as `Resizer::bufferDelay`.
#[must_use]
pub fn buffer_delay(
    liberty: &Liberty,
    buffer: BufferCell,
    slews: &EdgeValues<Seconds>,
    load_cap: Farads,
) -> Seconds {
    let delays = buffer_delays(liberty, buffer, slews, load_cap);
    delays.rise.max(delays.fall)
}

/// The worst delay per edge of `buffer` of `liberty` at `load_cap`, as
/// `Resizer::bufferDelays`.
#[must_use]
pub(crate) fn buffer_delays(
    liberty: &Liberty,
    buffer: BufferCell,
    slews: &EdgeValues<Seconds>,
    load_cap: Farads,
) -> EdgeValues<Seconds> {
    let cell = liberty.cell(buffer.id);
    let (delays, _) = gate_delays(cell, buffer.output, load_cap, slews, None);
    EdgeValues {
        rise: delays.rise.max(Seconds::ZERO),
        fall: delays.fall.max(Seconds::ZERO),
    }
}

#[cfg(test)]
mod tests;
