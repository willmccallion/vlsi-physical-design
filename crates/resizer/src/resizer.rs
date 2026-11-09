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

/// The edits since the last mark, as `OpenROAD`'s journal.
#[derive(Clone, Debug, Default)]
struct Journal {
    recording: bool,
    edits: Vec<Edit>,
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
    journal: Journal,
    dead_nets: Vec<NetId>,
    resized_multi_output: HashSet<CellId>,
    inserted_buffers: usize,
    /// Drivers whose timing is to be found again at the next flush, while
    /// edits are batched.
    pending: Option<Vec<PinId>>,
    /// Nets whose parasitics are to be estimated again at the next flush,
    /// while edits are batched.
    pending_nets: Vec<NetId>,
    /// How many nets the design had when the batch began: the batch's own
    /// edits read the nets it adds, so those are estimated as they change.
    nets_before_batch: usize,
    /// Whether the timing analysis is left behind the graph until
    /// [`Resizer::resume_timing`], with the drivers it has yet to take in.
    timing_deferred: bool,
    stale_timing: Vec<PinId>,
    routes: Option<GlobalRouting<'a>>,
}

impl<'a> Resizer<'a> {
    /// Prepares a session over `inputs` without timing: estimates every
    /// net's parasitics and slews.
    pub fn new(inputs: SessionInputs<'a>) -> Result<Self, RepairError> {
        Self::build(inputs, None, PortConditions::default(), |_, _| Untimed)
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
            self.pending.is_none() && !self.timing_deferred,
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
        slacks.sort_by(|a, b| {
            a.1.time()
                .total_cmp(&b.1.time())
                .then_with(|| a.0.cmp(&b.0))
        });
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
            .fold(Ohms::new(f64::MAX), Ohms::min);
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
            journal: Journal::default(),
            dead_nets: Vec::new(),
            resized_multi_output: HashSet::new(),
            inserted_buffers: 0,
            pending: None,
            pending_nets: Vec::new(),
            nets_before_batch: 0,
            timing_deferred: false,
            stale_timing: Vec::new(),
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
    pub(crate) fn batch_edits(&mut self) {
        if self.pending.is_none() {
            self.pending = Some(Vec::new());
            self.nets_before_batch = self.db.nets.len();
        }
    }

    /// Estimates the nets and finds the timing again after the edits
    /// batched so far, and ends the batch.
    pub fn flush(&mut self) {
        let Some(mut drivers) = self.pending.take() else {
            return;
        };
        let mut nets = std::mem::take(&mut self.pending_nets);
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
    pub(crate) const fn defer_timing(&mut self) {
        self.timing_deferred = true;
    }

    /// Brings the timing analysis up to date with every edit made while it
    /// was deferred.
    pub(crate) fn resume_timing(&mut self) {
        self.timing_deferred = false;
        let mut stale = std::mem::take(&mut self.stale_timing);
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
        if self.pending.is_some() && net.index() < self.nets_before_batch {
            self.pending_nets.push(net);
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

    /// Sets the longest wire design repair allows, in database units; zero
    /// for no limit.
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
            .is_some_and(|p| p.three_state.is_some())
    }

    /// Whether an instance may be resized or removed: a plain standard
    /// cell the library does not protect.
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

    /// Starts recording edits, as `journalBegin`.
    pub(crate) fn journal_begin(&mut self) {
        self.journal.recording = true;
        self.journal.edits.clear();
    }

    /// Keeps the edits since the mark, as `journalEnd`.
    pub(crate) fn journal_end(&mut self) {
        self.journal.recording = false;
        self.journal.edits.clear();
    }

    /// Takes back every edit since the mark and brings the timing up to
    /// date, as `journalRestore`.
    pub(crate) fn journal_restore(&mut self) -> Result<(), RepairError> {
        self.journal.recording = false;
        let edits = std::mem::take(&mut self.journal.edits);
        let mut nets: FxHashSet<NetId> = FxHashSet::default();
        let mut cells: FxHashSet<CellId> = FxHashSet::default();
        for edit in edits.into_iter().rev() {
            match edit {
                Edit::Master { cell, old } => {
                    self.db.replace_master(cell, old)?;
                    let _ = cells.insert(cell);
                    for &pin in &self.db.cells[cell].pins {
                        let _ = nets.insert(self.db.pins[pin].net);
                    }
                }
                Edit::Moved { cell, from } => {
                    self.db.cells[cell].location = from;
                    for &pin in &self.db.cells[cell].pins {
                        let _ = nets.insert(self.db.pins[pin].net);
                    }
                }
                Edit::MovedPin { pin, from } => {
                    let _ = nets.insert(self.db.pins[pin].net);
                    let _ = nets.insert(from.net);
                    self.db.put_pin_back(pin, from);
                }
                Edit::NewCell(cell) => {
                    for &pin in &self.db.cells[cell].pins {
                        let _ = nets.insert(self.db.pins[pin].net);
                    }
                    let _ = self.db.retire_cell(cell);
                }
                Edit::NewNet(net) => self.dead_nets.push(net),
                Edit::Retired { cell, places } => {
                    self.db.unretire_cell(cell, places);
                    for &pin in &self.db.cells[cell].pins {
                        let _ = nets.insert(self.db.pins[pin].net);
                    }
                }
                Edit::Renamed { net, old } => self.db.rename_net(net, old)?,
            }
        }
        {
            let netlist = Netlist::new(self.db, self.link);
            self.graph.grow(&netlist);
        }
        let mut nets: Vec<NetId> = nets.into_iter().collect();
        nets.sort_unstable();
        self.estimate_nets(&nets);
        let mut dirty = Vec::new();
        {
            let netlist = Netlist::new(self.db, self.link);
            for net in nets {
                dirty.extend(netlist.drivers(net));
            }
            for cell in cells {
                for &pin in &self.db.cells[cell].pins {
                    if netlist.is_load(pin) {
                        dirty.extend(netlist.drivers(self.db.pins[pin].net));
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
                    drivers.extend(netlist.drivers(self.db.pins[pin].net));
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
            pending.extend_from_slice(drivers);
            return;
        }
        let netlist = Netlist::new(self.db, self.link);
        let dirty = self.graph.propagate_from(&netlist, self.clocks, drivers);
        if self.timing_deferred {
            self.stale_timing.extend(dirty);
        } else if let Some(sta) = self.timing.analysis_mut() {
            sta.update(&netlist, &self.graph, self.clocks, &dirty);
        }
    }

    /// Re-estimates a net and everything timed through it.
    pub(crate) fn update_net(&mut self, net: NetId) {
        self.invalidate_parasitics(net);
        let drivers: Vec<PinId> = self.netlist().drivers(net).collect();
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
                    drivers.extend(netlist.drivers(net));
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
        self.db.replace_master(cell, master)?;
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
        self.record(Edit::MovedPin { pin, from });
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
        Some(self.db.connect_cell_pin(cell, macro_pin, net))
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
        self.inserted_buffers += 1;
        {
            let netlist = Netlist::new(self.db, self.link);
            self.graph.grow(&netlist);
        }
        self.invalidate_parasitics(net);
        self.invalidate_parasitics(new_net);
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
        let places = self.db.retire_cell(buffer);
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
        let drivers: Vec<PinId> = self.netlist().drivers(survivor).collect();
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
    let Some(&(_, slack_min)) = slacks.first() else {
        return Vec::new();
    };
    if !slack_min.is_constrained() {
        return Vec::new();
    }
    let worst = ((slacks.len() as f64) * WORST_SLACK_NETS_SHARE).ceil() as usize;
    let slack_max = slacks[worst.clamp(1, slacks.len()) - 1].1;
    let scale = WeightScale {
        slack_min: slack_min.time(),
        slack_max,
        weight_max,
    };
    slacks
        .iter()
        .filter(|(_, slack)| *slack < slack_max)
        .map(|&(net, slack)| (net, scale.weight(slack)))
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
    fn weight(&self, slack: Slack) -> f64 {
        let max = self.slack_max.time();
        if fuzzy::equal(max.get(), self.slack_min.get()) {
            return 1.0;
        }
        if !self.slack_max.is_constrained() {
            return self.weight_max;
        }
        ((self.weight_max - 1.0) * (max - slack.time()).get() / (max - self.slack_min).get()) + 1.0
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
    Dbu(i64::from(
        ((meters.get() * f64::from(dbu.get()) * MICRONS_PER_METER).round() as i64 as i32)
            & i32::MAX,
    ))
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
        sum / count as f64
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
mod tests {
    use super::*;
    use crate::testing::{
        DefSections, cells, cells_with, clock_cells_source, def_text, design, metal2_wires, pin,
        with_session_inputs, with_timed_session,
    };
    use std::fmt::Write;

    /// Buffers in the chain between the two flops of [`buffered_pair`].
    const CHAIN: usize = 6;

    /// Two flops on one clock, the first launching through buffers `b1` to
    /// `b6` into the second.
    fn buffered_pair() -> NetlistDB {
        let mut components = String::from("- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n");
        let mut nets =
            String::from("- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- n0 ( f1 Q ) ( b1 A ) ;\n");
        for i in 1..=CHAIN {
            writeln!(
                components,
                "- b{i} BUF_X1 + PLACED ( {} 2800 ) N ;",
                2000 + 8000 * i
            )
            .unwrap();
            let load = if i == CHAIN {
                "f2 D".to_string()
            } else {
                format!("b{} A", i + 1)
            };
            writeln!(nets, "- n{i} ( b{i} Z ) ( {load} ) ;").unwrap();
        }
        writeln!(
            components,
            "- f2 DFF_X1 + PLACED ( {} 2800 ) N ;",
            2000 + 8000 * (CHAIN + 1)
        )
        .unwrap();
        design(&format!(
            "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN chain ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS {} ;\n{components}END COMPONENTS\n\
             PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS {} ;\n{nets}END NETS\nEND DESIGN\n",
            CHAIN + 2,
            CHAIN + 2
        ))
    }

    #[test]
    fn a_buffer_put_before_a_chain_leaves_every_pin_on_the_path_its_slack() {
        let mut db = buffered_pair();
        let (b1_in, end) = (pin(&db, "b1/A"), pin(&db, "f2/D"));
        let q = db.pins[b1_in].net;
        let clock = "create_clock -name clk -period 1.0 [get_ports clk]\n";

        with_timed_session(&mut db, &cells(&clock_cells_source()), clock, |resizer| {
            let buffer = resizer.buffers.lowest_drive;
            let at = Point::new(Dbu(10_000), Dbu(2800));
            let mut first = b1_in;
            for _ in 0..CHAIN {
                first = resizer
                    .insert_buffer_before_loads(q, &[first], buffer, at, "rebuffer")
                    .unwrap()
                    .input;
            }

            let sta = resizer.sta();
            let path_slack = sta.slack(end, MinMax::Max).time();
            assert!(path_slack.is_finite());
            let netlist = resizer.netlist();
            let mut at_pin = first;
            while at_pin != end {
                let PinOwner::Cell { cell, .. } = resizer.db.pins[at_pin].owner else {
                    unreachable!()
                };
                let out = resizer.db.cells[cell]
                    .pins
                    .iter()
                    .copied()
                    .find(|&p| netlist.is_driver(p))
                    .unwrap();
                for p in [at_pin, out] {
                    let slack = sta.slack(p, MinMax::Max).time();
                    assert!(
                        fuzzy::equal(slack, path_slack),
                        "{}: {slack:?} vs {path_slack:?}",
                        resizer.db.pin_name(p)
                    );
                }
                at_pin = netlist.loads(resizer.db.pins[out].net).next().unwrap();
            }
        });
    }

    fn slacks(values: &[f64]) -> Vec<(NetId, Slack)> {
        values
            .iter()
            .enumerate()
            .map(|(i, &v)| (NetId::new(i), Slack::new(Seconds::new(v))))
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

    #[test]
    fn finite_slacks_take_the_maximum_weight_when_the_worst_tenth_reaches_infinity() {
        let mut values = vec![f64::INFINITY; 20];
        values[0] = -0.3;

        let weights = timing_net_weights(&slacks(&values), 5.0);

        assert_eq!(weights, vec![(NetId::new(0), 5.0)]);
    }

    const CLOCK: &str = "create_clock -name clk -period 1.0 [get_ports clk]\n";

    /// Two flops with buffers `b1` and `b2` between them, the first
    /// fed from a net inside a hierarchy.
    fn two_buffers() -> NetlistDB {
        design(&def_text(
            "two_buffers",
            &DefSections {
                components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - b1 BUF_X1 + PLACED ( 20000 2800 ) N ;\n\
                             - b2 BUF_X1 + PLACED ( 40000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 60000 2800 ) N ;\n",
                nets: "- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n\
                       - u1/q ( f1 Q ) ( b1 A ) ;\n- m ( b1 Z ) ( b2 A ) ;\n\
                       - d ( b2 Z ) ( f2 D ) ;\n",
                inputs: &["clk"],
                outputs: &[],
            },
        ))
    }

    fn def_of(db: &NetlistDB) -> String {
        let mut text = Vec::new();
        pare_common::db::writer::def::write(db, &mut text).unwrap();
        String::from_utf8(text).unwrap()
    }

    /// How a test session closes its journal.
    #[derive(Clone, Copy)]
    enum Close {
        Restore,
        End,
    }

    /// The DEF of [`two_buffers`] after a session that removes `b1`, which
    /// renames nets, resizes `b2` and buffers `f2`, then closes the journal
    /// as `close` says.
    fn after_edits(close: Close) -> String {
        let mut db = two_buffers();
        let lib = cells(&clock_cells_source());
        let rc = metal2_wires(&db);
        with_session_inputs(&mut db, &lib, CLOCK, rc, |inputs, sdc| {
            let mut resizer = Resizer::timed(inputs, sdc).unwrap();
            resizer.journal_begin();
            let b1 = resizer.db.cell_id("b1").unwrap();
            let _ = resizer.remove_buffer(b1).unwrap();
            let b2 = resizer.db.cell_id("b2").unwrap();
            let clock_buffer = resizer.link.liberty.cell_id("CLKBUF_X3").unwrap();
            assert!(resizer.replace_cell(b2, clock_buffer).unwrap());
            let d = resizer.db.net_id("d").unwrap();
            let load = pin(resizer.db, "f2/D");
            let buffer = resizer.buffers.lowest_drive;
            let at = Point::new(Dbu(50_000), Dbu(2800));
            let _ = resizer
                .insert_buffer_before_loads(d, &[load], buffer, at, "rebuffer")
                .unwrap();
            match close {
                Close::Restore => resizer.journal_restore().unwrap(),
                Close::End => resizer.journal_end(),
            }
            let _ = resizer.finish().unwrap();
        });
        def_of(&db)
    }

    #[test]
    fn restoring_the_journal_leaves_the_design_as_it_was() {
        let before = def_of(&two_buffers());

        let after = after_edits(Close::Restore);

        assert_eq!(after, before);
    }

    #[test]
    fn ending_the_journal_keeps_the_edits() {
        let before = def_of(&two_buffers());

        let after = after_edits(Close::End);

        assert_ne!(after, before);
        assert!(after.contains("- rebuffer1 BUF_X1") && after.contains("- b2 CLKBUF_X3"));
        assert!(!after.contains("- b1 BUF_X1"));
    }

    /// The error a timed session over [`two_buffers`] with the clock cells
    /// of `source` fails to start with.
    fn start_error(source: &str) -> Option<RepairError> {
        let mut db = two_buffers();
        let lib = cells(source);
        let rc = metal2_wires(&db);
        with_session_inputs(&mut db, &lib, CLOCK, rc, |inputs, sdc| {
            Resizer::timed(inputs, sdc).err()
        })
    }

    #[test]
    fn slew_thresholds_derated_out_of_shape_refuse_a_session() {
        let derated = clock_cells_source().replace(
            "slew_derate_from_library : 1.0",
            "slew_derate_from_library : 100.0",
        );

        let error = start_error(&derated);

        assert!(
            matches!(error, Some(RepairError::ShapeFactor(_))),
            "{error:?}"
        );
        assert!(start_error(&clock_cells_source()).is_none());
    }

    /// A buffer cell the design's library has no master for.
    const MASTERLESS_BUFFER: &str = "cell (BUF_X2) { area : 1.0 ; \
        pin (A) { direction : input ; capacitance : 1.0 ; } \
        pin (Z) { direction : output ; function : \"A\" ; } }";

    #[test]
    fn a_size_without_a_master_is_refused() {
        let mut db = two_buffers();
        let lib = cells_with(MASTERLESS_BUFFER);

        with_timed_session(&mut db, &lib, CLOCK, |resizer| {
            let b1 = resizer.db.cell_id("b1").unwrap();
            let size = resizer.link.liberty.cell_id("BUF_X2").unwrap();

            let error = resizer.replace_cell(b1, size);

            assert_eq!(error, Err(RepairError::NoMaster("BUF_X2".to_string())));
        });
    }

    #[test]
    fn a_master_lacking_a_connected_pin_is_refused() {
        let mut db = two_buffers();
        let lib = cells(&clock_cells_source());

        with_timed_session(&mut db, &lib, CLOCK, |resizer| {
            let f1 = resizer.db.cell_id("f1").unwrap();
            let inverter = resizer.link.liberty.cell_id("INV_X1").unwrap();

            let error = resizer.replace_cell(f1, inverter);

            assert!(
                matches!(error, Err(RepairError::Edit(EditError::MissingPin { .. }))),
                "{error:?}"
            );
        });
    }

    /// The clock test cells with a library fanout limit of 6, `BUF_X1`'s
    /// own of 4, and a fanout load of 2 at the flops' data inputs, 1 by
    /// default elsewhere.
    fn fanout_cells() -> Liberty {
        let source = clock_cells_source()
            .replacen(
                "default_max_transition : 0.2 ;",
                "default_max_transition : 0.2 ; default_max_fanout : 6 ; \
                 default_fanout_load : 1.0 ;",
                1,
            )
            .replacen(
                "function : \"A\" ;\n      max_capacitance : 60.0 ;",
                "function : \"A\" ;\n      max_fanout : 4 ;\n      max_capacitance : 60.0 ;",
                1,
            )
            .replace(
                "capacitance : 1.1 ;",
                "capacitance : 1.1 ; fanout_load : 2.0 ;",
            );
        cells(&source)
    }

    #[test]
    fn a_drivers_fanout_limit_is_the_tightest_of_its_own_its_librarys_and_the_designs() {
        let lib = fanout_cells();
        let limits = |constraints: &str| {
            let mut db = two_buffers();
            let mut found = (None, None);
            with_timed_session(&mut db, &lib, constraints, |resizer| {
                let (flop, buffer) = (pin(resizer.db, "f1/Q"), pin(resizer.db, "b2/Z"));
                found = (resizer.fanout_limit(flop), resizer.fanout_limit(buffer));
            });
            found
        };

        assert_eq!(limits(CLOCK), (Some(6.0), Some(4.0)));
        assert_eq!(
            limits(&format!("{CLOCK}set_max_fanout 5 [current_design]\n")),
            (Some(5.0), Some(4.0))
        );
    }

    #[test]
    fn a_nets_fanout_sums_its_loads_fanout_loads() {
        let mut db = two_buffers();
        let lib = fanout_cells();

        with_timed_session(&mut db, &lib, CLOCK, |resizer| {
            let into_flop = resizer.fanout_load(pin(resizer.db, "b2/Z"));
            let into_buffer = resizer.fanout_load(pin(resizer.db, "f1/Q"));

            assert!((into_flop - 2.0).abs() < 1e-12, "{into_flop}");
            assert!((into_buffer - 1.0).abs() < 1e-12, "{into_buffer}");
        });
    }

    #[test]
    fn a_cell_that_is_not_a_buffer_has_no_buffer_pins() {
        let mut db = two_buffers();
        let lib = cells(&clock_cells_source());

        with_timed_session(&mut db, &lib, CLOCK, |resizer| {
            let and_gate = resizer.link.liberty.cell_id("AND2_X1").unwrap();
            let f1 = resizer.db.cell_id("f1").unwrap();

            assert_eq!(
                resizer.as_buffer(and_gate),
                Err(RepairError::MissingPins("AND2_X1".to_string()))
            );
            assert_eq!(
                resizer.remove_buffer(f1),
                Err(RepairError::MissingPins("f1".to_string()))
            );
        });
    }
}
