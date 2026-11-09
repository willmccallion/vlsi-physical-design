//! Static timing analysis over the timing graph, as `OpenSTA` searches it.
//!
//! Arrivals go forward from the clocks and the constrained input ports,
//! tagged with the clock edge that launched them; required times come back
//! from the setup, hold, recovery and removal checks at register pins and
//! the delays constraining output ports, each measured against the
//! capturing clock through `OpenSTA`'s cycle accounting. Latches borrow
//! time as `OpenSTA` lets them. Clocks are ideal: every pin of a clock
//! network sees its clock's edge at the edge's time. Both passes are
//! incremental, so a repair that changes a few nets pays only for the
//! paths through them.

use crate::fuzzy;
use crate::gated::{GatedEnable, gated_enables};
use crate::graph::TimingGraph;
use crate::netlist::{ClockNetwork, Constants, Netlist};
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{CellId, IdVec, IoPinId, PinId};
use pare_common::db::liberty::{
    ArcRole, Edge, EdgeValues, FuncExpr, Logic, MinMax, SequentialKind, SideValues,
};
use pare_common::db::sdc::{ClockEdge, ClockId, MultiCyclePath, PathEnd, Sdc};
use pare_common::units::Seconds;
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// A slack as `OpenSTA` keeps it: [`Slack::UNCONSTRAINED`]
/// where nothing constrains the pin, which `OpenSTA` holds as its finite
/// `INF` and so orders after every real slack.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Slack(Seconds);

impl Slack {
    /// The slack of a pin nothing constrains.
    pub const UNCONSTRAINED: Self = Self(Seconds::INFINITY);

    /// A slack of `time`.
    #[cfg(test)]
    #[must_use]
    pub(crate) const fn new(time: Seconds) -> Self {
        Self(time)
    }

    /// The slack's time, infinite where nothing constrains the pin.
    #[must_use]
    pub const fn time(self) -> Seconds {
        self.0
    }

    /// Whether something constrains the pin.
    #[must_use]
    pub(crate) const fn is_constrained(self) -> bool {
        self.0.is_finite()
    }
}

/// What distinguishes the paths that reach a pin, as `OpenSTA`'s tags.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Tag {
    /// The clock edge that launched the path; none for an unconstrained
    /// path from an input without a delay.
    pub launch: Option<ClockEdge>,
    /// The port the path starts at, kept when an exception names it.
    pub from_port: Option<IoPinId>,
    /// Whether the path is the clock itself, within its network.
    pub is_clock: bool,
}

pare_common::define_index!(
    /// A tag of the analysis.
    TagId(u32)
);

/// The step before a point on a path.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Prev {
    pin: PinId,
    edge: Edge,
    tag: TagId,
    /// The timing group the step came through, or none for a wire.
    set: Option<u32>,
}

/// One tag's times at one pin and edge.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Entry {
    tag: TagId,
    /// Arrival per side; absent on a side nothing constrains.
    arrival: SideValues<Option<Seconds>>,
    /// Required time per side: the latest allowed on the late side, the
    /// earliest on the early side; unconstrained starts at the identity.
    required: SideValues<Seconds>,
    prev: SideValues<Option<Prev>>,
}

/// What finding a pin's arrivals again did to its entries, by how much
/// timing must follow: fanout for any change, the required search for an
/// entry without a required time.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Stored {
    /// The arrivals stand as they were.
    Same,
    /// The arrivals changed and every entry kept its required time.
    Moved,
    /// An entry is new and has no required time yet.
    Unrequired,
}

impl Entry {
    const fn new(tag: TagId) -> Self {
        Self {
            tag,
            arrival: SideValues::both(None),
            required: REQUIRED_INIT,
            prev: SideValues::both(None),
        }
    }

    fn slack(&self, side: MinMax) -> Seconds {
        slack_of(self.arrival[side], self.required[side], side)
    }
}

/// The required time nothing constrains: no bound on the early side, none
/// on the late side.
const REQUIRED_INIT: SideValues<Seconds> = SideValues {
    min: Seconds::new(f64::NEG_INFINITY),
    max: Seconds::INFINITY,
};

/// The required time of a pin `OpenSTA`'s required search never reaches,
/// one with neither fanin nor fanout such as an unconnected input port:
/// the zero its paths start with, so a late arrival leaves negative slack.
const REQUIRED_UNREACHED: SideValues<Seconds> = SideValues::both(Seconds::ZERO);

/// Whether a required time is the unconstrained one.
const fn unconstrained(required: Seconds) -> bool {
    required.get().is_infinite()
}

/// Whether two pairs of times differ at all, bit for bit, which is what a
/// change worth propagating means.
fn differ(a: SideValues<Seconds>, b: SideValues<Seconds>) -> bool {
    MinMax::BOTH
        .iter()
        .any(|&side| !same_time(a[side], b[side]))
}

/// Whether two times, either possibly infinite, are equal within
/// `OpenSTA`'s tolerance, so that a change too small to matter does not
/// propagate through the graph.
fn same_time(a: Seconds, b: Seconds) -> bool {
    if a.get().is_infinite() || b.get().is_infinite() {
        return a.get().to_bits() == b.get().to_bits();
    }
    fuzzy::equal(a, b)
}

/// Whether `candidate` is a tighter required time than `held` on `side`
/// by more than the tolerance, as `RequiredCmp::requiredSet` compares them.
fn tighter(side: MinMax, candidate: Seconds, held: Seconds) -> bool {
    delay_greater(side.opposite(), candidate, held)
}

/// Whether `a` is further toward `side` than `b` by more than the
/// tolerance, as `OpenSTA`'s `delayGreater` with a min-max.
fn delay_greater(side: MinMax, a: Seconds, b: Seconds) -> bool {
    match side {
        MinMax::Max => fuzzy::greater(a, b),
        MinMax::Min => fuzzy::less(a, b),
    }
}

/// Slack from an arrival and a required time, infinite where either is
/// absent.
fn slack_of(arrival: Option<Seconds>, required: Seconds, side: MinMax) -> Seconds {
    let Some(arrival) = arrival else {
        return Seconds::INFINITY;
    };
    if unconstrained(required) {
        return Seconds::INFINITY;
    }
    match side {
        MinMax::Max => required - arrival,
        MinMax::Min => arrival - required,
    }
}

/// How a path reached a point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Via {
    /// The point starts the path.
    Start,
    /// Along the wire from the net's driver.
    Wire,
    /// Through a timing group of the point's cell.
    Arc {
        /// Index of the group in the cell's arc sets.
        set: usize,
    },
}

/// One pin on a path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathPoint {
    /// The pin the path passes.
    pub pin: PinId,
    /// The transition the path takes at the pin.
    pub edge: Edge,
    /// What launched the path.
    pub tag: Tag,
    /// When the edge arrives.
    pub arrival: Seconds,
    /// How the path got here.
    pub via: Via,
}

/// A path from a start to an endpoint, as `OpenSTA`'s `PathExpanded`.
#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    /// The points, start first.
    pub points: Vec<PathPoint>,
    /// Index of the point the data leaves from: a register or latch output,
    /// or the start itself.
    pub start_index: usize,
    /// Which side the path was searched on.
    pub side: MinMax,
    /// The slack at its endpoint.
    pub slack: Seconds,
}

impl Path {
    /// The path's last point: the endpoint it ends at.
    #[must_use]
    pub fn end(&self) -> &PathPoint {
        &self.points[self.points.len() - 1]
    }
}

/// `OpenSTA`'s cycle accounting between a launching and a capturing clock
/// edge: when, within the launching edge's cycle, the capturing edge falls
/// for each kind of check.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Accounting {
    setup_required: Seconds,
    hold_required: Seconds,
    /// For a gated clock's hold check, made against the trailing edge in
    /// the cycle of the setup check rather than the one before.
    gated_hold_required: Seconds,
    /// For a latch whose enable is the capturing edge: when the latch opens
    /// before the closing edge the data must beat.
    latch_setup_required: Seconds,
    /// The cycles of the launching and capturing clocks the latch
    /// accounting landed on.
    latch_cycles: (i64, i64),
    /// Whether the search ran out of cycles before the clocks realigned,
    /// which `OpenSTA` warns of as having no common period.
    max_cycles_exceeded: bool,
}

/// Which check a capture is for, since a gated clock's hold check lands on
/// a different cycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Check {
    /// A register's, a latch's or an output's.
    Ordinary,
    /// A gated clock enable's.
    GatedClock,
}

impl Accounting {
    const fn required(&self, side: MinMax, check: Check) -> Seconds {
        match (side, check) {
            (MinMax::Max, _) => self.setup_required,
            (MinMax::Min, Check::Ordinary) => self.hold_required,
            (MinMax::Min, Check::GatedClock) => self.gated_hold_required,
        }
    }
}

/// Whether a latch is clocked, or held open or closed by a constant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LatchState {
    Enabled,
    Open,
    Closed,
}

/// A level-sensitive cell: which pin opens it and on which edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Latch {
    enable: PinId,
    open_edge: Edge,
    data: PinId,
    state: LatchState,
}

/// Data reaching a clocked latch's input: its tag, its edge and its arrival
/// on one side.
#[derive(Clone, Copy, Debug)]
struct LatchData {
    tag: TagId,
    edge: Edge,
    side: MinMax,
    arrival: Seconds,
}

/// What a latch does with data arriving at `arrival`: `OpenSTA`'s
/// `latchRequired` for an ideal clock.
#[derive(Clone, Copy, Debug, PartialEq)]
struct LatchTiming {
    required: Seconds,
    borrow: Seconds,
    /// The data's arrival in the enable clock's frame, when it borrows.
    adjusted_arrival: Seconds,
}

/// The timing of a design, kept current as repair edits it.
#[derive(Clone, Debug)]
pub struct Sta {
    sdc: Sdc,
    tags: IdVec<TagId, Tag>,
    tag_ids: FxHashMap<Tag, TagId>,
    /// Each pin's entries for each edge.
    entries: IdVec<PinId, EdgeValues<Vec<Entry>>>,
    endpoints: Vec<PinId>,
    is_endpoint: IdVec<PinId, bool>,
    /// Each endpoint pin's slack from its own checks, per side.
    end_slack: IdVec<PinId, SideValues<Seconds>>,
    tns: SideValues<Seconds>,
    worst: SideValues<Option<(Slack, Option<PinId>)>>,
    accountings: FxHashMap<(ClockEdge, ClockEdge), Accounting>,
    /// The pairs of clocks already warned of as having no common period.
    unaligned_clocks: FxHashSet<(ClockId, ClockId)>,
    exception_ports: FxHashSet<IoPinId>,
    latches: FxHashMap<CellId, Latch>,
    /// The enables of gated clocks, checked against their gate's clock.
    gated: FxHashMap<PinId, GatedEnable>,
    /// Whether each pin is in a propagated clock's network, below where the
    /// clock enters, so its clock arrives through the network's delays.
    propagated: IdVec<PinId, bool>,
}

/// A clock edge a check captures with, and how late it arrives at the
/// check's clock pin beyond the edge's own time on each side: zero for an
/// ideal clock, its network's delay for a propagated one.
#[derive(Clone, Copy, Debug)]
struct Capture {
    edge: ClockEdge,
    latency: SideValues<Seconds>,
}

impl Capture {
    /// The latency a check on `side` captures with: the early arrival for
    /// setup and the late one for hold, as `OpenSTA` takes the capturing
    /// clock path on the side opposite the check.
    fn latency(&self, side: MinMax) -> Seconds {
        self.latency[side.opposite()]
    }
}

impl Sta {
    /// Times `netlist` from the delays in `graph` under `sdc`.
    #[must_use]
    pub(crate) fn new(
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        clocks: &ClockNetwork,
        constants: &Constants,
        sdc: &Sdc,
    ) -> Self {
        let n = netlist.db.pins.len();
        let exception_ports = sdc
            .false_paths
            .iter()
            .flat_map(|f| f.paths.from_ports.iter().copied())
            .chain(
                sdc.multicycle_paths
                    .iter()
                    .flat_map(|m| m.paths.from_ports.iter().copied()),
            )
            .collect();
        let mut sta = Self {
            sdc: sdc.clone(),
            tags: IdVec::new(),
            tag_ids: FxHashMap::default(),
            entries: IdVec::filled(EdgeValues::default(), n),
            endpoints: Vec::new(),
            is_endpoint: IdVec::filled(false, n),
            end_slack: IdVec::filled(SideValues::both(Seconds::INFINITY), n),
            tns: SideValues::both(Seconds::ZERO),
            worst: SideValues::both(None),
            accountings: FxHashMap::default(),
            unaligned_clocks: FxHashSet::default(),
            exception_ports,
            latches: find_latches(netlist, constants),
            gated: gated_enables(netlist, clocks),
            propagated: netlist
                .db
                .pins
                .ids()
                .map(|p| clocks.is_propagated(p))
                .collect(),
        };
        sta.find_endpoints(netlist, clocks);
        let all: Vec<PinId> = netlist.db.pins.ids().collect();
        let _ = sta.find_arrivals(netlist, graph, clocks, &all);
        sta.find_requireds(netlist, graph, &all);
        sta
    }

    /// Brings the timing up to date after the delays of `dirty_drivers`
    /// changed, as `graph` reports them.
    pub(crate) fn update(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        clocks: &ClockNetwork,
        dirty_drivers: &[PinId],
    ) {
        self.grow(netlist);
        let mut seeds: Vec<PinId> = Vec::new();
        for &driver in dirty_drivers {
            seeds.push(driver);
            let net = netlist.db.pins[driver].net;
            seeds.extend(netlist.loads(net));
            seeds.extend(graph.in_arcs(driver).iter().map(|a| a.from));
        }
        let requiring = self.find_arrivals(netlist, graph, clocks, &seeds);
        seeds.extend(requiring);
        self.find_requireds(netlist, graph, &seeds);
    }

    /// Makes room for pins added since the analysis started.
    fn grow(&mut self, netlist: &Netlist<'_>) {
        let n = netlist.db.pins.len();
        self.entries.grow_to(n, EdgeValues::default());
        self.is_endpoint.grow_to(n, false);
        self.end_slack
            .grow_to(n, SideValues::both(Seconds::INFINITY));
        self.propagated.grow_to(n, false);
    }

    /// The constraints an analysis runs under.
    #[must_use]
    pub const fn sdc(&self) -> &Sdc {
        &self.sdc
    }

    fn intern(&mut self, tag: Tag) -> TagId {
        if let Some(&id) = self.tag_ids.get(&tag) {
            return id;
        }
        let id = self.tags.push(tag);
        let _ = self.tag_ids.insert(tag, id);
        id
    }

    fn entries(&self, pin: PinId, edge: Edge) -> &[Entry] {
        self.entries
            .get(pin)
            .map_or(&[], |entries| entries[edge].as_slice())
    }

    /// The pins timing ends at: register inputs with checks from a clocked
    /// pin, asynchronous controls with recovery or removal checks, and
    /// output ports with delays.
    fn find_endpoints(&mut self, netlist: &Netlist<'_>, clocks: &ClockNetwork) {
        let db = netlist.db;
        self.endpoints.clear();
        self.is_endpoint.fill(false);
        for pin in db.pins.ids() {
            if !netlist.is_load(pin) {
                continue;
            }
            let is_end = match db.pins[pin].owner {
                PinOwner::Io(io) => self.sdc.output_delays.contains_key(&io),
                PinOwner::Cell { cell, .. } => {
                    let Some((lib, port)) = netlist.link.pin_port(db, pin) else {
                        continue;
                    };
                    netlist.link.liberty.cell(lib).arc_sets.iter().any(|set| {
                        set.to == port
                            && set.role.check_side().is_some()
                            && !set.arcs.is_empty()
                            && netlist
                                .pin_of_port(cell, set.from)
                                .is_some_and(|ck| clocks.is_clock(ck))
                    })
                }
            };
            if is_end || self.gated.contains_key(&pin) {
                self.is_endpoint[pin] = true;
                self.endpoints.push(pin);
            }
        }
    }

    /// The endpoints, by pin id.
    #[must_use]
    pub fn endpoints(&self) -> &[PinId] {
        &self.endpoints
    }

    /// Whether timing ends at a pin.
    #[must_use]
    pub(crate) fn is_endpoint(&self, pin: PinId) -> bool {
        self.is_endpoint.get(pin).copied().unwrap_or(false)
    }

    /// A pin's slack on `side`: the least over its edges and tags, as
    /// `OpenSTA`'s vertex slack.
    #[must_use]
    pub fn slack(&self, pin: PinId, side: MinMax) -> Slack {
        Slack(
            Edge::BOTH
                .iter()
                .map(|&edge| self.slack_edge(pin, edge, side).0)
                .fold(Seconds::INFINITY, Seconds::min),
        )
    }

    /// A pin's slack for one edge on `side`.
    #[must_use]
    pub(crate) fn slack_edge(&self, pin: PinId, edge: Edge, side: MinMax) -> Slack {
        Slack(
            self.entries(pin, edge)
                .iter()
                .map(|e| e.slack(side))
                .fold(Seconds::INFINITY, Seconds::min),
        )
    }

    /// The arrival at a pin for `edge` on `side`: the latest on the late
    /// side, the earliest on the early side, over the tags; none where
    /// nothing arrives.
    #[must_use]
    pub fn arrival(&self, pin: PinId, edge: Edge, side: MinMax) -> Option<Seconds> {
        self.entries(pin, edge)
            .iter()
            .filter_map(|e| e.arrival[side])
            .reduce(|a, b| if side.beats(b, a) { b } else { a })
    }

    /// The late arrival of a clock's rising edge at a pin of its network,
    /// as `TritonCTS` reads it from the first such clock path: the pin's
    /// rising transitions first.
    #[must_use]
    pub fn clock_rise_arrival(&self, pin: PinId) -> Option<Seconds> {
        Edge::BOTH.iter().find_map(|&edge| {
            self.entries(pin, edge).iter().find_map(|e| {
                let tag = self.tags[e.tag];
                let rising = tag.launch.is_some_and(|l| l.edge == Edge::Rise);
                (tag.is_clock && rising)
                    .then_some(e.arrival[MinMax::Max])
                    .flatten()
            })
        })
    }

    /// The sum of the negative endpoint slacks on `side`.
    #[must_use]
    pub(crate) fn total_negative_slack(&self, side: MinMax) -> Seconds {
        self.tns[side]
    }

    /// The least endpoint slack on `side` and its endpoint.
    #[must_use]
    pub fn worst_slack(&self, side: MinMax) -> (Slack, Option<PinId>) {
        if let Some(worst) = self.worst[side] {
            return worst;
        }
        let mut worst = (Slack::UNCONSTRAINED, None);
        for &end in &self.endpoints {
            let slack = Slack(self.end_slack[end][side]);
            if fuzzy::less(slack.0, worst.0.0) {
                worst = (slack, Some(end));
            }
        }
        worst
    }

    /// Remembers the worst slacks; they stay valid until an endpoint
    /// changes.
    fn cache_worst(&mut self) {
        for side in MinMax::BOTH {
            if self.worst[side].is_none() {
                let worst = self.worst_slack(side);
                self.worst[side] = Some(worst);
            }
        }
    }

    /// The path of least slack ending at `end` on `side`, as `OpenSTA`'s
    /// `vertexWorstSlackPath`; none when nothing constrains the pin.
    #[must_use]
    pub(crate) fn worst_slack_path(
        &self,
        netlist: &Netlist<'_>,
        end: PinId,
        side: MinMax,
    ) -> Option<Path> {
        let mut best: Option<(Seconds, Edge, Entry)> = None;
        for edge in Edge::BOTH {
            for entry in self.entries(end, edge) {
                let slack = entry.slack(side);
                if slack.is_finite() && best.is_none_or(|(s, _, _)| fuzzy::less(slack, s)) {
                    best = Some((slack, edge, *entry));
                }
            }
        }
        let (slack, edge, entry) = best?;
        let mut points = Vec::new();
        let mut at = (end, edge, entry);
        loop {
            let (pin, edge, entry) = at;
            let via = match entry.prev[side] {
                None => Via::Start,
                Some(Prev { set: None, .. }) => Via::Wire,
                Some(Prev { set: Some(set), .. }) => Via::Arc { set: set as usize },
            };
            points.push(PathPoint {
                pin,
                edge,
                tag: self.tags[entry.tag],
                arrival: entry.arrival[side].unwrap_or(Seconds::ZERO),
                via,
            });
            let Some(prev) = entry.prev[side] else {
                break;
            };
            let prev_entry = self
                .entries(prev.pin, prev.edge)
                .iter()
                .find(|e| e.tag == prev.tag)
                .copied()?;
            at = (prev.pin, prev.edge, prev_entry);
            if points.len() > self.entries.len() {
                return None;
            }
        }
        points.reverse();
        let start_index = points
            .iter()
            .position(|p| match p.via {
                Via::Arc { set } => self.starts_data(netlist, p.pin, set),
                _ => false,
            })
            .unwrap_or(0);
        Some(Path {
            points,
            start_index,
            side,
            slack,
        })
    }

    /// Whether the arc of `set` into `pin` leaves a register or an open
    /// latch, where `OpenSTA`'s expanded path starts.
    fn starts_data(&self, netlist: &Netlist<'_>, pin: PinId, set: usize) -> bool {
        let PinOwner::Cell { cell, .. } = netlist.db.pins[pin].owner else {
            return false;
        };
        let Some((_, lib)) = netlist.link.cell(netlist.db, cell) else {
            return false;
        };
        let arc_set = &lib.arc_sets[set];
        if matches!(arc_set.role, ArcRole::RegClkToQ | ArcRole::LatchEnToQ) {
            return true;
        }
        self.latches.get(&cell).is_some_and(|latch| {
            latch.state == LatchState::Enabled
                && netlist.pin_of_port(cell, arc_set.from) == Some(latch.data)
        })
    }

    /// Finds arrivals again from `seeds` onward, following changes through
    /// the fanout; returns the pins the required search must visit: those
    /// left with an entry that has no required time yet, and latch inputs
    /// whose arrival moved, since a borrowing latch requires its data when
    /// it arrives.
    ///
    /// Stale levels after edits can time a load before its new driver, so
    /// a pin may lose its entries and get them back within one search,
    /// anywhere downstream of an edit.
    fn find_arrivals(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        clocks: &ClockNetwork,
        seeds: &[PinId],
    ) -> Vec<PinId> {
        let n = netlist.db.pins.len();
        let mut queued = IdVec::filled(false, n);
        let mut heap: BinaryHeap<Reverse<(u32, PinId)>> = BinaryHeap::new();
        for &pin in seeds {
            if queued.get(pin) == Some(&false) {
                queued[pin] = true;
                heap.push(Reverse((graph.level(pin), pin)));
            }
        }
        let mut requiring = Vec::new();
        while let Some(Reverse((_, pin))) = heap.pop() {
            queued[pin] = false;
            match self.visit_arrivals(netlist, graph, clocks, pin) {
                Stored::Same => continue,
                Stored::Moved if self.is_latch_data(netlist, pin) => requiring.push(pin),
                Stored::Moved => {}
                Stored::Unrequired => requiring.push(pin),
            }
            for next in successors(netlist, graph, pin) {
                if !queued[next] {
                    queued[next] = true;
                    heap.push(Reverse((graph.level(next), next)));
                }
            }
        }
        self.cache_worst();
        requiring
    }

    /// Whether a pin is the data input of a clocked latch.
    fn is_latch_data(&self, netlist: &Netlist<'_>, pin: PinId) -> bool {
        let PinOwner::Cell { cell, .. } = netlist.db.pins[pin].owner else {
            return false;
        };
        self.latches
            .get(&cell)
            .is_some_and(|l| l.data == pin && l.state == LatchState::Enabled)
    }

    /// Computes a pin's arrivals from its predecessors or its seeds and
    /// stores them.
    fn visit_arrivals(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        clocks: &ClockNetwork,
        pin: PinId,
    ) -> Stored {
        let is_driver = netlist.is_driver(pin);
        let is_load = netlist.is_load(pin);
        if !is_driver && !is_load {
            return self.clear_arrivals(pin);
        }
        let mut stored = Stored::Same;
        for edge in Edge::BOTH {
            let fresh = if clocks.is_ideal(pin) {
                self.clock_seeds(clocks, pin, edge)
            } else if clocks.is_propagated(pin) {
                self.network_arrivals(netlist, graph, pin, edge)
            } else if let PinOwner::Io(io) = netlist.db.pins[pin].owner
                && is_driver
            {
                self.port_seeds(io, edge)
            } else if is_driver {
                self.driver_arrivals(netlist, graph, pin, edge)
            } else {
                self.load_arrivals(netlist, graph, pin, edge)
            };
            stored = stored.max(self.store_arrivals(pin, edge, fresh));
        }
        if stored != Stored::Same && self.is_endpoint(pin) {
            self.refresh_endpoint(netlist, graph, pin);
        }
        stored
    }

    fn clear_arrivals(&mut self, pin: PinId) -> Stored {
        let mut stored = Stored::Same;
        for edge in Edge::BOTH {
            let slot = &mut self.entries[pin][edge];
            if !slot.is_empty() {
                slot.clear();
                stored = Stored::Moved;
            }
        }
        stored
    }

    /// Replaces a pin's entries, keeping the required times of tags that
    /// stay.
    fn store_arrivals(&mut self, pin: PinId, edge: Edge, mut fresh: Vec<Entry>) -> Stored {
        let slot = &mut self.entries[pin][edge];
        let mut unrequired = false;
        for entry in &mut fresh {
            match slot.iter().find(|o| o.tag == entry.tag) {
                Some(old) => entry.required = old.required,
                None => unrequired = true,
            }
        }
        let same_arrival = |o: &Entry, f: &Entry| {
            o.arrival
                .iter()
                .zip(f.arrival.iter())
                .all(|(x, y)| match (x, y) {
                    (Some(x), Some(y)) => same_time(*x, *y),
                    (None, None) => true,
                    _ => false,
                })
        };
        let changed = slot.len() != fresh.len()
            || slot
                .iter()
                .zip(&fresh)
                .any(|(o, f)| o.tag != f.tag || !same_arrival(o, f));
        if changed || slot.iter().zip(&fresh).any(|(o, f)| o.prev != f.prev) {
            *slot = fresh;
        }
        if unrequired {
            Stored::Unrequired
        } else if changed {
            Stored::Moved
        } else {
            Stored::Same
        }
    }

    /// The clock edges arriving at a pin of a clock network: each clock's
    /// edge at its time, as an ideal clock arrives everywhere at once.
    fn clock_seeds(&mut self, clocks: &ClockNetwork, pin: PinId, edge: Edge) -> Vec<Entry> {
        let mut entries = Vec::new();
        for &clock in clocks.clocks_of(pin) {
            let launch = ClockEdge { clock, edge };
            let time = self.sdc.edge_time(launch);
            let tag = self.intern(Tag {
                launch: Some(launch),
                from_port: None,
                is_clock: true,
            });
            entries.push(Entry {
                tag,
                arrival: SideValues::both(Some(time)),
                required: REQUIRED_INIT,
                prev: SideValues::both(None),
            });
        }
        entries
    }

    /// The arrivals at an input port: its delays after their clock edges,
    /// or an unconstrained start at time zero without any.
    fn port_seeds(&mut self, io: IoPinId, edge: Edge) -> Vec<Entry> {
        let delays = self.sdc.input_delays.get(&io).cloned().unwrap_or_default();
        if delays.is_empty() {
            let tag = self.intern(Tag {
                launch: None,
                from_port: None,
                is_clock: false,
            });
            return vec![Entry {
                tag,
                arrival: SideValues::both(Some(Seconds::ZERO)),
                required: REQUIRED_INIT,
                prev: SideValues::both(None),
            }];
        }
        let from_port = self.exception_ports.contains(&io).then_some(io);
        let mut entries = Vec::new();
        for delay in delays {
            let tag = self.intern(Tag {
                launch: Some(delay.reference),
                from_port,
                is_clock: false,
            });
            let edge_time = self.sdc.edge_time(delay.reference);
            let arrival =
                SideValues::from_fn(|side| delay.delays.get(edge, side).map(|d| edge_time + d));
            if arrival.iter().all(Option::is_none) {
                continue;
            }
            entries.push(Entry {
                tag,
                arrival,
                required: REQUIRED_INIT,
                prev: SideValues::both(None),
            });
        }
        entries
    }

    /// The clock arrivals at a pin of a propagated clock's network, through
    /// the delays of the cells and wires on the way from where the clock
    /// enters; data arriving there is not carried on, as at an ideal clock.
    fn network_arrivals(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        pin: PinId,
        edge: Edge,
    ) -> Vec<Entry> {
        let all = if netlist.is_driver(pin) {
            self.driver_arrivals(netlist, graph, pin, edge)
        } else {
            self.load_arrivals(netlist, graph, pin, edge)
        };
        all.into_iter()
            .filter(|e| self.tags[e.tag].is_clock)
            .collect()
    }

    /// The clock edges arriving at clock pin `pin` on `edge`, each with its
    /// latency there.
    fn captures(&self, pin: PinId, edge: Edge) -> Vec<Capture> {
        self.entries(pin, edge)
            .iter()
            .filter_map(|e| {
                let t = self.tags[e.tag];
                let launch = t.launch.filter(|_| t.is_clock)?;
                let time = self.sdc.edge_time(launch);
                let latency = e.arrival.map(|a| a.map_or(Seconds::ZERO, |a| a - time));
                Some(Capture {
                    edge: launch,
                    latency,
                })
            })
            .collect()
    }

    /// The transition at a check's clock pin that its margin is read at:
    /// zero for an ideal clock, as `OpenSTA` takes an ideal clock's slew,
    /// and the clock's own slew on the check's side for a propagated one.
    fn clock_slew(&self, graph: &TimingGraph, pin: PinId, edge: Edge, side: MinMax) -> Seconds {
        if self.propagated.get(pin).copied().unwrap_or(false) {
            graph.slew_side(pin, side).get(edge)
        } else {
            Seconds::ZERO
        }
    }

    /// The arrivals at a cell output from the arcs into it.
    fn driver_arrivals(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        driver: PinId,
        edge: Edge,
    ) -> Vec<Entry> {
        let mut entries: Vec<Entry> = Vec::new();
        let derate = self.cell_derates(netlist, driver);
        let latch = match netlist.db.pins[driver].owner {
            PinOwner::Cell { cell, .. } => self.latches.get(&cell).copied(),
            PinOwner::Io(_) => None,
        };
        for arc in graph.arc_delays(driver) {
            if arc.to_edge != edge || graph.is_loop_arc(arc.arc.from, driver) {
                continue;
            }
            if !netlist.is_load(arc.arc.from) {
                continue;
            }
            let through_latch = latch.filter(|l| l.data == arc.arc.from);
            if through_latch.is_some_and(|l| l.state == LatchState::Closed) {
                continue;
            }
            let from_entries: Vec<Entry> = self.entries(arc.arc.from, arc.from_edge).to_vec();
            let in_network = self.propagated.get(driver).copied().unwrap_or(false);
            for from in from_entries {
                let tag = if in_network {
                    from.tag
                } else {
                    self.data_tag(from.tag)
                };
                for side in MinMax::BOTH {
                    let Some(arrival) = from.arrival[side] else {
                        continue;
                    };
                    let delay = arc.delay[side] * derate[side];
                    let prev = Prev {
                        pin: arc.arc.from,
                        edge: arc.from_edge,
                        tag: from.tag,
                        set: Some(arc.arc.set as u32),
                    };
                    match through_latch {
                        Some(latch) if latch.state == LatchState::Enabled => {
                            let data = LatchData {
                                tag: from.tag,
                                edge: arc.from_edge,
                                side,
                                arrival,
                            };
                            for (q_tag, adjusted) in
                                self.latch_departures(netlist, graph, latch, data)
                            {
                                merge_arrival(&mut entries, q_tag, side, adjusted + delay, prev);
                            }
                        }
                        _ => merge_arrival(&mut entries, tag, side, arrival + delay, prev),
                    }
                }
            }
        }
        entries
    }

    /// The arrivals at a load from its net's drivers over the wire.
    fn load_arrivals(
        &self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        load: PinId,
        edge: Edge,
    ) -> Vec<Entry> {
        let mut entries: Vec<Entry> = Vec::new();
        let net = netlist.db.pins[load].net;
        let derate = self.sdc.derating.net;
        for driver in netlist.drivers(net) {
            if driver == load {
                continue;
            }
            for from in self.entries(driver, edge) {
                for side in MinMax::BOTH {
                    let Some(arrival) = from.arrival[side] else {
                        continue;
                    };
                    let wire = graph.wire_delay(load, edge, side);
                    merge_arrival(
                        &mut entries,
                        from.tag,
                        side,
                        wire.mul_add(derate[side], arrival),
                        Prev {
                            pin: driver,
                            edge,
                            tag: from.tag,
                            set: None,
                        },
                    );
                }
            }
        }
        entries
    }

    /// The tag a path carries once it leaves a clock network and becomes
    /// data.
    fn data_tag(&mut self, tag: TagId) -> TagId {
        let value = self.tags[tag];
        if !value.is_clock {
            return tag;
        }
        self.intern(Tag {
            is_clock: false,
            ..value
        })
    }

    /// Where data arriving at a clocked latch's input leaves its output, as
    /// `OpenSTA`'s `latchOutArrival`: only data that arrives after the
    /// latch opens passes, relaunched by the enabling edge at its own
    /// arrival; earlier data waits for the enable, which the enable's own
    /// arc carries.
    fn latch_departures(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        latch: Latch,
        data: LatchData,
    ) -> Vec<(TagId, Seconds)> {
        let tag = self.tags[data.tag];
        let Some(launch) = tag.launch else {
            return Vec::new();
        };
        let enables = self.captures(latch.enable, latch.open_edge);
        let margin = self.latch_margin(netlist, graph, latch, data.edge, data.side);
        let arrival = data.arrival;
        let mut departures = Vec::new();
        for capture in enables {
            let enable = capture.edge;
            let to = PathEnd::Clock(enable.clock);
            if self.is_false(tag, to, Some(enable.clock), MinMax::Max) {
                continue;
            }
            let timing = self.latch_timing(tag, launch, capture, to, arrival, margin);
            if fuzzy::greater(timing.borrow, Seconds::ZERO) {
                let q_tag = self.intern(Tag {
                    launch: Some(enable),
                    from_port: tag.from_port,
                    is_clock: false,
                });
                departures.push((q_tag, timing.adjusted_arrival));
            }
        }
        departures
    }

    /// The setup a latch's data pin demands before the closing edge for
    /// data on `data_edge`, as `Latches::latchSetupMargin`: the first setup
    /// arc from the enable matching the data edge and the closing edge, at
    /// `side`'s slews, whatever its sign; zero without one.
    fn latch_margin(
        &self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        latch: Latch,
        data_edge: Edge,
        side: MinMax,
    ) -> Seconds {
        let db = netlist.db;
        let (Some((lib, port)), Some((_, enable_port))) = (
            netlist.link.pin_port(db, latch.data),
            netlist.link.pin_port(db, latch.enable),
        ) else {
            return Seconds::ZERO;
        };
        let close_edge = opposite(latch.open_edge);
        let clock_slew = self.clock_slew(graph, latch.enable, close_edge, side);
        let data_slew = graph.slew_side(latch.data, side).get(data_edge);
        netlist
            .link
            .liberty
            .cell(lib)
            .arc_sets
            .iter()
            .filter(|set| set.to == port && set.from == enable_port && set.role == ArcRole::Setup)
            .find_map(|set| {
                set.arcs
                    .iter()
                    .find(|arc| arc.to_edge == data_edge && arc.from_edge == close_edge)
            })
            .map_or(Seconds::ZERO, |arc| arc.check_value(clock_slew, data_slew))
    }

    /// `OpenSTA`'s `latchRequired` for an ideal clock: data arriving before
    /// the latch opens is required by the opening edge; later data borrows
    /// from the open window, up to its width less the setup.
    fn latch_timing(
        &mut self,
        tag: Tag,
        launch: ClockEdge,
        capture: Capture,
        to: PathEnd,
        arrival: Seconds,
        margin: Seconds,
    ) -> LatchTiming {
        let enable = capture.edge;
        let accounting = self.accounting(launch, enable);
        let open_uncertainty = -self.sdc.clock(enable.clock).uncertainty(MinMax::Max);
        let adjustment = self.multicycle_adjustment(tag, launch, enable, to, MinMax::Max);
        let enable_arrival = accounting.latch_setup_required
            + open_uncertainty
            + adjustment
            + capture.latency(MinMax::Max);
        if fuzzy::less_equal(arrival, enable_arrival) {
            return LatchTiming {
                required: enable_arrival,
                borrow: Seconds::ZERO,
                adjusted_arrival: arrival,
            };
        }
        let clock = self.sdc.clock(enable.clock);
        let mut width = clock.edge_time(opposite(enable.edge)) - clock.edge_time(enable.edge);
        if width < Seconds::ZERO {
            width += clock.period;
        }
        let max_borrow = width - margin;
        let mut borrow = arrival - enable_arrival;
        let required = if fuzzy::less_equal(borrow, max_borrow) {
            arrival
        } else {
            borrow = max_borrow;
            enable_arrival + max_borrow
        };
        let (src_cycle, tgt_cycle) = accounting.latch_cycles;
        let shift = self
            .sdc
            .clock(launch.clock)
            .period
            .mul_add(src_cycle as f64, -(tgt_cycle as f64) * clock.period);
        LatchTiming {
            required,
            borrow,
            adjusted_arrival: required + shift,
        }
    }

    /// Finds required times again from `seeds` back, following changes
    /// through the fanin.
    fn find_requireds(&mut self, netlist: &Netlist<'_>, graph: &TimingGraph, seeds: &[PinId]) {
        let n = netlist.db.pins.len();
        let mut queued = IdVec::filled(false, n);
        let mut heap: BinaryHeap<(u32, PinId)> = BinaryHeap::new();
        for &pin in seeds {
            if queued.get(pin) == Some(&false) {
                queued[pin] = true;
                heap.push((graph.level(pin), pin));
            }
        }
        while let Some((_, pin)) = heap.pop() {
            queued[pin] = false;
            if !self.visit_requireds(netlist, graph, pin) {
                continue;
            }
            for prev in predecessors(netlist, graph, pin) {
                if !queued[prev] {
                    queued[prev] = true;
                    heap.push((graph.level(prev), prev));
                }
            }
        }
        self.cache_worst();
    }

    /// Computes a pin's required times from its fanout and its checks and
    /// stores them; true when they changed.
    fn visit_requireds(&mut self, netlist: &Netlist<'_>, graph: &TimingGraph, pin: PinId) -> bool {
        let mut changed = false;
        let unreached = is_isolated(netlist, graph, pin);
        for edge in Edge::BOTH {
            let entries = self.entries(pin, edge).to_vec();
            for (k, entry) in entries.iter().enumerate() {
                let required = match (unreached, self.tags[entry.tag].is_clock) {
                    (true, true) => REQUIRED_INIT,
                    (true, false) => REQUIRED_UNREACHED,
                    (false, _) => self.entry_requireds(netlist, graph, pin, edge, entry),
                };
                let slot = &mut self.entries[pin][edge][k];
                if differ(slot.required, required) {
                    slot.required = required;
                    changed = true;
                }
            }
        }
        if self.is_endpoint(pin) {
            self.refresh_endpoint(netlist, graph, pin);
        }
        changed
    }

    /// The required times of one entry, clock paths included, as
    /// `OpenSTA`'s `RequiredVisitor::visit`: the tightest its fanout imposes
    /// and, at an endpoint, its checks.
    fn entry_requireds(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        pin: PinId,
        edge: Edge,
        entry: &Entry,
    ) -> SideValues<Seconds> {
        let mut required = self.fanout_requireds(netlist, graph, pin, edge, entry.tag);
        if !self.is_endpoint(pin) {
            return required;
        }
        let checks = self.check_requireds(netlist, graph, pin, edge, entry);
        for side in MinMax::BOTH {
            if let Some(check) = checks[side]
                && tighter(side, check, required[side])
            {
                required[side] = check;
            }
        }
        required
    }

    /// The factors the delays of the arcs into `driver` are scaled by, for
    /// the early and late sides: its instance's own derate where set, else
    /// the design's.
    fn cell_derates(&self, netlist: &Netlist<'_>, driver: PinId) -> SideValues<f64> {
        let cell = match netlist.db.pins[driver].owner {
            PinOwner::Cell { cell, .. } => Some(cell),
            PinOwner::Io(_) => None,
        };
        SideValues::from_fn(|side| self.sdc.cell_derate(cell, side))
    }

    /// The required times a pin's fanout imposes for one tag.
    fn fanout_requireds(
        &self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        pin: PinId,
        edge: Edge,
        tag: TagId,
    ) -> SideValues<Seconds> {
        let mut required = REQUIRED_INIT;
        if netlist.is_driver(pin) {
            let net = netlist.db.pins[pin].net;
            let derate = self.sdc.derating.net;
            for load in netlist.loads(net) {
                if load == pin {
                    continue;
                }
                for entry in self.entries(load, edge).iter().filter(|e| e.tag == tag) {
                    for side in MinMax::BOTH {
                        let bound = entry.required[side];
                        if unconstrained(bound) {
                            continue;
                        }
                        let wire = graph.wire_delay(load, edge, side);
                        let candidate = wire.mul_add(-derate[side], bound);
                        if tighter(side, candidate, required[side]) {
                            required[side] = candidate;
                        }
                    }
                }
            }
        }
        if netlist.is_load(pin) {
            for driver in graph.drivers_fed_by(netlist, pin) {
                if graph.is_loop_arc(pin, driver) {
                    continue;
                }
                let derate = self.cell_derates(netlist, driver);
                for arc in graph.arc_delays(driver) {
                    if arc.arc.from != pin || arc.from_edge != edge || arc.arc.through_latch {
                        continue;
                    }
                    for entry in self
                        .entries(driver, arc.to_edge)
                        .iter()
                        .filter(|e| e.tag == tag)
                    {
                        for side in MinMax::BOTH {
                            let bound = entry.required[side];
                            if unconstrained(bound) {
                                continue;
                            }
                            let candidate = arc.delay[side].mul_add(-derate[side], bound);
                            if tighter(side, candidate, required[side]) {
                                required[side] = candidate;
                            }
                        }
                    }
                }
            }
        }
        required
    }

    /// The required times the checks at an endpoint impose for one entry:
    /// the tightest over its check arcs or output delays, per side.
    fn check_requireds(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        pin: PinId,
        edge: Edge,
        entry: &Entry,
    ) -> SideValues<Option<Seconds>> {
        let tag = self.tags[entry.tag];
        let Some(launch) = tag.launch else {
            return SideValues::both(None);
        };
        let mut required: SideValues<Option<Seconds>> = SideValues::both(None);
        let mut tighten = |side: MinMax, candidate: Seconds| {
            let slot = &mut required[side];
            if slot.is_none_or(|held| tighter(side, candidate, held)) {
                *slot = Some(candidate);
            }
        };
        let db = netlist.db;
        match db.pins[pin].owner {
            PinOwner::Io(io) => {
                let delays = self.sdc.output_delays.get(&io).cloned().unwrap_or_default();
                for delay in delays {
                    let target = delay.reference;
                    for side in MinMax::BOTH {
                        let Some(margin) = delay.delays.get(edge, side) else {
                            continue;
                        };
                        let to = PathEnd::Port(io);
                        if self.is_false(tag, to, Some(target.clock), side) {
                            continue;
                        }
                        let base = self.target_arrival(tag, launch, target, to, side);
                        tighten(side, base - margin);
                    }
                }
            }
            PinOwner::Cell { cell, .. } => {
                if let Some(&gate) = self.gated.get(&pin) {
                    for (side, value) in self.gated_requireds(tag, launch, entry, gate) {
                        tighten(side, value);
                    }
                }
                let Some((lib, port)) = netlist.link.pin_port(db, pin) else {
                    return SideValues::both(None);
                };
                let latch = self
                    .latches
                    .get(&cell)
                    .copied()
                    .filter(|l| l.data == pin && l.state == LatchState::Enabled);
                let sets: Vec<(usize, PinId)> = netlist
                    .link
                    .liberty
                    .cell(lib)
                    .arc_sets
                    .iter()
                    .enumerate()
                    .filter(|(_, set)| set.to == port && set.role.check_side().is_some())
                    .filter_map(|(k, set)| netlist.pin_of_port(cell, set.from).map(|ck| (k, ck)))
                    .collect();
                for (k, clock_pin) in sets {
                    let set = &netlist.link.liberty.cell(lib).arc_sets[k];
                    let Some(side) = set.role.check_side() else {
                        continue;
                    };
                    let Some(arrival) = entry.arrival[side] else {
                        continue;
                    };
                    let data_slew = graph.slew_side(pin, side).get(edge);
                    for arc in &set.arcs {
                        if arc.to_edge != edge {
                            continue;
                        }
                        let clock_slew = self.clock_slew(graph, clock_pin, arc.from_edge, side);
                        let margin = arc.check_value(clock_slew, data_slew);
                        let is_latch_setup = latch.is_some() && set.role == ArcRole::Setup;
                        let clock_edge = if is_latch_setup {
                            opposite(arc.from_edge)
                        } else {
                            arc.from_edge
                        };
                        for capture in self.captures(clock_pin, clock_edge) {
                            let target = capture.edge;
                            let to = PathEnd::Clock(target.clock);
                            if self.is_false(tag, to, Some(target.clock), side) {
                                continue;
                            }
                            let value = if is_latch_setup {
                                self.latch_timing(tag, launch, capture, to, arrival, margin)
                                    .required
                            } else {
                                let base = self.target_arrival(tag, launch, target, to, side)
                                    + capture.latency(side);
                                match side {
                                    MinMax::Max => base - margin,
                                    MinMax::Min => base + margin,
                                }
                            };
                            tighten(side, value);
                        }
                    }
                }
            }
        }
        required
    }

    /// The required times of a gated clock's enable, as `OpenSTA`'s
    /// `PathEndGatedClock` with no clock gating margin set: on each side,
    /// the clock's edge at the gate's clock input that the check is made
    /// against, in the launching edge's frame.
    fn gated_requireds(
        &mut self,
        tag: Tag,
        launch: ClockEdge,
        entry: &Entry,
        gate: GatedEnable,
    ) -> Vec<(MinMax, Seconds)> {
        let mut out = Vec::new();
        for side in MinMax::BOTH {
            if entry.arrival[side].is_none() {
                continue;
            }
            let clock_edge = gate.active.check_edge(side);
            for capture in self.captures(gate.clock, clock_edge) {
                let target = capture.edge;
                let to = PathEnd::Clock(target.clock);
                if self.is_false(tag, to, Some(target.clock), side) {
                    continue;
                }
                let required =
                    self.capture_arrival(tag, launch, target, to, side, Check::GatedClock)
                        + capture.latency(side);
                out.push((side, required));
            }
        }
        out
    }

    /// Whether an exception drops the check on `side` of a path with `tag`
    /// into `to`.
    fn is_false(&self, tag: Tag, to: PathEnd, to_clock: Option<ClockId>, side: MinMax) -> bool {
        let Some(launch) = tag.launch else {
            return true;
        };
        if self
            .sdc
            .is_false_path(PathEnd::Clock(launch.clock), to, to_clock, side)
        {
            return true;
        }
        tag.from_port.is_some_and(|port| {
            self.sdc
                .is_false_path(PathEnd::Port(port), to, to_clock, side)
        })
    }

    /// The multicycle exception bearing on `side` of a path with `tag`
    /// into `to`, a port's own before its clock's.
    fn multicycle(
        &self,
        tag: Tag,
        to: PathEnd,
        to_clock: Option<ClockId>,
        side: MinMax,
    ) -> Option<&MultiCyclePath> {
        let launch = tag.launch?;
        tag.from_port
            .and_then(|port| {
                self.sdc
                    .multicycle_path(PathEnd::Port(port), to, to_clock, side)
            })
            .or_else(|| {
                self.sdc
                    .multicycle_path(PathEnd::Clock(launch.clock), to, to_clock, side)
            })
    }

    /// When the capturing edge arrives for a check on `side`, in the
    /// launching edge's frame: `OpenSTA`'s `targetClkArrivalNoCrpr` for an
    /// ideal clock, with its uncertainty and multicycle adjustment.
    fn target_arrival(
        &mut self,
        tag: Tag,
        launch: ClockEdge,
        target: ClockEdge,
        to: PathEnd,
        side: MinMax,
    ) -> Seconds {
        self.capture_arrival(tag, launch, target, to, side, Check::Ordinary)
    }

    /// [`Self::target_arrival`] for a given kind of check.
    fn capture_arrival(
        &mut self,
        tag: Tag,
        launch: ClockEdge,
        target: ClockEdge,
        to: PathEnd,
        side: MinMax,
        check: Check,
    ) -> Seconds {
        let accounting = self.accounting(launch, target);
        let clock_time = accounting.required(side, check);
        let uncertainty = self.sdc.clock(target.clock).uncertainty(side);
        let uncertainty = match side {
            MinMax::Max => -uncertainty,
            MinMax::Min => uncertainty,
        };
        clock_time + uncertainty + self.multicycle_adjustment(tag, launch, target, to, side)
    }

    /// `OpenSTA`'s `checkMcpAdjustment`: how far a multicycle exception
    /// moves the capturing edge.
    fn multicycle_adjustment(
        &self,
        tag: Tag,
        launch: ClockEdge,
        target: ClockEdge,
        to: PathEnd,
        side: MinMax,
    ) -> Seconds {
        let period = |mcp: &MultiCyclePath| {
            let clock = if mcp.use_end_clock {
                target.clock
            } else {
                launch.clock
            };
            self.sdc.clock(clock).period
        };
        let setup_cycles =
            |mcp: &MultiCyclePath| f64::from(mcp.multiplier(MinMax::Max) - 1) * period(mcp);
        let setup = self.multicycle(tag, to, Some(target.clock), MinMax::Max);
        match side {
            MinMax::Max => setup.map_or(Seconds::ZERO, setup_cycles),
            MinMax::Min => {
                let hold = self.multicycle(tag, to, Some(target.clock), MinMax::Min);
                let hold_cycles = hold.map_or(Seconds::ZERO, |mcp| {
                    f64::from(mcp.multiplier(MinMax::Min)) * period(mcp)
                });
                setup.map_or(Seconds::ZERO, setup_cycles) - hold_cycles
            }
        }
    }

    /// The cycle accounting between two clock edges, found once; warns
    /// once for each pair of clocks without a common period, as `OpenSTA`'s
    /// warning 1010.
    fn accounting(&mut self, launch: ClockEdge, target: ClockEdge) -> Accounting {
        if let Some(&found) = self.accountings.get(&(launch, target)) {
            return found;
        }
        let found = cycle_accounting(&self.sdc, launch, target);
        let pair = (
            launch.clock.min(target.clock),
            launch.clock.max(target.clock),
        );
        if found.max_cycles_exceeded && self.unaligned_clocks.insert(pair) {
            log::warn!(
                "no common period was found between clocks {} and {}",
                self.sdc.clock(launch.clock).name,
                self.sdc.clock(target.clock).name
            );
        }
        let _ = self.accountings.insert((launch, target), found);
        found
    }

    /// Recomputes an endpoint's own-check slack and the totals it enters.
    fn refresh_endpoint(&mut self, netlist: &Netlist<'_>, graph: &TimingGraph, pin: PinId) {
        let mut slack = SideValues::both(Seconds::INFINITY);
        for edge in Edge::BOTH {
            let entries = self.entries(pin, edge).to_vec();
            for entry in entries {
                let checks = self.check_requireds(netlist, graph, pin, edge, &entry);
                for side in MinMax::BOTH {
                    if let Some(required) = checks[side] {
                        let s = slack_of(entry.arrival[side], required, side);
                        slack[side] = slack[side].min(s);
                    }
                }
            }
        }
        let old = self.end_slack[pin];
        if !differ(old, slack) {
            return;
        }
        for side in MinMax::BOTH {
            if counts_in_tns(old[side]) {
                self.tns[side] -= old[side];
            }
            if counts_in_tns(slack[side]) {
                self.tns[side] += slack[side];
            }
            self.worst[side] = None;
        }
        self.end_slack[pin] = slack;
    }
}

/// Whether an endpoint's slack enters the total negative slack, as
/// `Search::tnsIncr` takes it: below zero by more than the tolerance.
fn counts_in_tns(slack: Seconds) -> bool {
    slack.is_finite() && fuzzy::less(slack, Seconds::ZERO)
}

const fn opposite(edge: Edge) -> Edge {
    match edge {
        Edge::Rise => Edge::Fall,
        Edge::Fall => Edge::Rise,
    }
}

/// The latches of the design: cells with a level-sensitive group whose
/// enable is one pin, open on its high level or, negated, its low level;
/// a constant enable holds the latch open or closed.
fn find_latches(netlist: &Netlist<'_>, constants: &Constants) -> FxHashMap<CellId, Latch> {
    let db = netlist.db;
    let mut latches = FxHashMap::default();
    for cell in db.cells.ids() {
        let Some((_, lib)) = netlist.link.cell(db, cell) else {
            continue;
        };
        for sequential in &lib.sequentials {
            if sequential.kind != SequentialKind::Latch {
                continue;
            }
            let (enable_name, open_edge) = match &sequential.clock {
                Some(FuncExpr::Port(p)) => (p.as_str(), Edge::Rise),
                Some(FuncExpr::Not(inner)) => match inner.as_ref() {
                    FuncExpr::Port(p) => (p.as_str(), Edge::Fall),
                    _ => continue,
                },
                _ => continue,
            };
            let Some(data_name) = sequential.data.as_ref().and_then(FuncExpr::as_port) else {
                continue;
            };
            let (Some(enable_port), Some(data_port)) = (lib.port(enable_name), lib.port(data_name))
            else {
                continue;
            };
            let (Some(enable), Some(data)) = (
                netlist.pin_of_port(cell, enable_port),
                netlist.pin_of_port(cell, data_port),
            ) else {
                continue;
            };
            let state = match (constants.level(enable), open_edge) {
                (Logic::One, Edge::Rise) | (Logic::Zero, Edge::Fall) => LatchState::Open,
                (Logic::One, Edge::Fall) | (Logic::Zero, Edge::Rise) => LatchState::Closed,
                _ => LatchState::Enabled,
            };
            let _ = latches.insert(
                cell,
                Latch {
                    enable,
                    open_edge,
                    data,
                    state,
                },
            );
        }
    }
    latches
}

/// Keeps the arrival for `tag` on `side` in `entries`, adding the tag when
/// new, as `OpenSTA` keeps the latest arrival for the late side and the
/// earliest for the early side.
fn merge_arrival(entries: &mut Vec<Entry>, tag: TagId, side: MinMax, time: Seconds, prev: Prev) {
    let index = entries
        .iter()
        .position(|e| e.tag == tag)
        .unwrap_or_else(|| {
            entries.push(Entry::new(tag));
            entries.len() - 1
        });
    let entry = &mut entries[index];
    let slot = &mut entry.arrival[side];
    if slot.is_none_or(|held| delay_greater(side, time, held)) {
        *slot = Some(time);
        entry.prev[side] = Some(prev);
    }
}

/// The pins timing continues to from `pin`: the loads of a driver's net,
/// the drivers an input's arcs reach.
fn successors(netlist: &Netlist<'_>, graph: &TimingGraph, pin: PinId) -> Vec<PinId> {
    let mut next = Vec::new();
    if netlist.is_driver(pin) {
        let net = netlist.db.pins[pin].net;
        next.extend(netlist.loads(net).filter(|&l| l != pin));
    }
    if netlist.is_load(pin) {
        next.extend(
            graph
                .drivers_fed_by(netlist, pin)
                .into_iter()
                .filter(|&d| !graph.is_loop_arc(pin, d)),
        );
    }
    next
}

/// Whether timing neither reaches `pin` from another pin nor leaves it.
fn is_isolated(netlist: &Netlist<'_>, graph: &TimingGraph, pin: PinId) -> bool {
    predecessors(netlist, graph, pin).is_empty() && successors(netlist, graph, pin).is_empty()
}

/// The pins timing comes from into `pin`.
fn predecessors(netlist: &Netlist<'_>, graph: &TimingGraph, pin: PinId) -> Vec<PinId> {
    let mut prev = Vec::new();
    if netlist.is_load(pin) {
        let net = netlist.db.pins[pin].net;
        prev.extend(netlist.drivers(net).filter(|&d| d != pin));
    }
    if netlist.is_driver(pin) {
        prev.extend(
            graph
                .in_arcs(pin)
                .iter()
                .filter(|a| !a.through_latch && !graph.is_loop_arc(a.from, pin))
                .map(|a| a.from),
        );
    }
    prev
}

/// `OpenSTA`'s `CycleAccting::findDelays` for setup, hold and latch setup:
/// the first capturing edge after the launching one, the one at or before
/// it, and for a latch the opening edge before the closing edge the data
/// must beat, searched over the cycles until the two clocks realign.
fn cycle_accounting(sdc: &Sdc, launch: ClockEdge, target: ClockEdge) -> Accounting {
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
    let first_cycle = |time: Seconds, period: Seconds| -> i64 {
        if time < Seconds::ZERO {
            1
        } else if time < period {
            0
        } else {
            -1
        }
    };
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
        let mut src_cycle = first_cycle(src_edge_time, src_period);
        loop {
            let src_start = src_cycle as f64 * src_period;
            if search.tgt_past_src && search.src_past_tgt && fuzzy::equal(src_start, tgt.start) {
                return search.accounting;
            }
            if fuzzy::greater(src_start, tgt.start + tgt.period) && search.src_past_tgt {
                break;
            }
            search.record(
                &tgt,
                SourceCycle {
                    cycle: src_cycle,
                    start: src_start,
                    time: src_start + src_edge_time,
                },
            );
            src_cycle += 1;
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

impl CycleSearch {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{Estimator, PortConditions};
    use crate::library::Link;
    use crate::parasitics::WireRc;
    use crate::testing::{
        DefSections, cells_with, def_text, design_with_copies, with_timed_session,
    };
    use pare_common::db::design::NetlistDB;
    use pare_common::db::indices::LayerId;
    use pare_common::db::liberty::Liberty;
    use pare_common::db::parser::{def, lef, liberty, sdc};
    use pare_common::db::sdc::Clock;
    use pare_common::geom::dbu::DbuPerMicron;
    use pare_common::units::{FaradsPerMeter, OhmsPerMeter};
    use std::path::{Path, PathBuf};

    fn clock(name: &str, period: f64, rise: f64, fall: f64) -> Clock {
        Clock {
            name: name.into(),
            period: Seconds::new(period),
            rise: Seconds::new(rise),
            fall: Seconds::new(fall),
            sources: Vec::new(),
            uncertainty: SideValues::both(None),
            propagated: false,
        }
    }

    fn edge(clock: u32, edge: Edge) -> ClockEdge {
        ClockEdge {
            clock: ClockId(clock),
            edge,
        }
    }

    #[test]
    fn same_clock_captures_a_period_later_for_setup_and_at_once_for_hold() {
        let sdc = Sdc {
            clocks: vec![clock("clk", 2e-9, 0.0, 1e-9)].into(),
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Rise));

        assert!((acct.setup_required.get() - 2e-9).abs() < 1e-20);
        assert!(acct.hold_required.get().abs() < 1e-20);
    }

    #[test]
    fn a_gated_clocks_hold_check_is_the_trailing_edge_of_the_same_cycle() {
        let sdc = Sdc {
            clocks: vec![clock("clk", 2e-9, 0.0, 1e-9)].into(),
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Fall));

        assert!((acct.gated_hold_required.get() - 1e-9).abs() < 1e-20);
        assert!((acct.required(MinMax::Min, Check::GatedClock).get() - 1e-9).abs() < 1e-20);
        assert!((acct.required(MinMax::Min, Check::Ordinary).get() + 1e-9).abs() < 1e-20);
    }

    #[test]
    fn a_falling_capture_of_a_rising_launch_is_half_a_period_away() {
        let sdc = Sdc {
            clocks: vec![clock("clk", 2e-9, 0.0, 1e-9)].into(),
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Fall));

        assert!((acct.setup_required.get() - 1e-9).abs() < 1e-20);
        assert!((acct.hold_required.get() + 1e-9).abs() < 1e-20);
    }

    #[test]
    fn a_latch_opening_on_the_fall_is_required_at_the_fall_before_the_close() {
        let sdc = Sdc {
            clocks: vec![clock("clk", 2.2e-9, 0.0, 1.1e-9)].into(),
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Fall));

        assert!((acct.latch_setup_required.get() - 1.1e-9).abs() < 1e-20);
        assert_eq!(acct.latch_cycles, (0, 0));
    }

    #[test]
    fn a_fast_clock_captures_a_slow_launch_at_its_next_edge() {
        let sdc = Sdc {
            clocks: vec![
                clock("slow", 4e-9, 0.0, 2e-9),
                clock("fast", 1e-9, 0.0, 0.5e-9),
            ]
            .into(),
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(1, Edge::Rise));

        assert!((acct.setup_required.get() - 1e-9).abs() < 1e-20);
        assert!(acct.hold_required.get().abs() < 1e-20);
    }

    #[test]
    fn a_launch_without_a_period_is_captured_in_the_first_cycle() {
        let sdc = Sdc {
            clocks: vec![
                clock("virtual", 0.0, 0.0, 0.0),
                clock("clk", 2e-9, 0.0, 1e-9),
            ]
            .into(),
            ..Sdc::default()
        };

        let falling = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(1, Edge::Fall));
        let rising = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(1, Edge::Rise));

        assert!((falling.setup_required.get() - 1e-9).abs() < 1e-20);
        assert!((falling.hold_required.get() - 1e-9).abs() < 1e-20);
        assert!((rising.setup_required.get() - 2e-9).abs() < 1e-20);
        assert!(rising.hold_required.get().abs() < 1e-20);
        assert_eq!(rising.latch_cycles, (0, 1));
    }

    #[test]
    fn clocks_without_a_common_period_are_flagged() {
        let sdc = Sdc {
            clocks: vec![
                clock("a", 1e-9, 0.0, 0.5e-9),
                clock("b", 2f64.sqrt() * 1e-9, 0.0, 0.5e-9),
            ]
            .into(),
            ..Sdc::default()
        };

        let unrelated = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(1, Edge::Rise));
        let same = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Rise));

        assert!(unrelated.max_cycles_exceeded);
        assert!(!same.max_cycles_exceeded);
    }

    fn fixture(crate_name: &str, name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join(crate_name)
            .join("tests/fixtures")
            .join(name)
    }

    /// A flop clocked and fed from ports, beside an input port on nothing.
    fn dangling_port_design() -> NetlistDB {
        let text = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN dangle ;\n\
            UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\n\
            COMPONENTS 1 ;\n- f DFF_X1 + PLACED ( 2000 2800 ) N ;\nEND COMPONENTS\n\
            PINS 3 ;\n- clk + NET clk + DIRECTION INPUT ;\n- in + NET in + DIRECTION INPUT ;\n\
            - boot + NET boot + DIRECTION INPUT ;\nEND PINS\n\
            NETS 3 ;\n- clk ( PIN clk ) ( f CK ) ;\n- in ( PIN in ) ( f D ) ;\n\
            - boot ( PIN boot ) ;\nEND NETS\nEND DESIGN\n";
        let units = DbuPerMicron::new(2000).unwrap();
        let library = lef::parse_files(&[fixture("cts", "clock.lef")], units).unwrap();
        def::parse_str(text, Path::new("dangle.def"), library).unwrap()
    }

    /// Two flops on one clock, the first launching into the second.
    fn flop_to_flop_design() -> NetlistDB {
        let text = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN pair ;\n\
            UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\n\
            COMPONENTS 2 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
            - f2 DFF_X1 + PLACED ( 12000 2800 ) N ;\nEND COMPONENTS\n\
            PINS 2 ;\n- clk + NET clk + DIRECTION INPUT ;\n- in + NET in + DIRECTION INPUT ;\n\
            END PINS\nNETS 3 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- in ( PIN in ) ( f1 D ) ;\n\
            - q ( f1 Q ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n";
        let units = DbuPerMicron::new(2000).unwrap();
        let library = lef::parse_files(&[fixture("cts", "clock.lef")], units).unwrap();
        def::parse_str(text, Path::new("pair.def"), library).unwrap()
    }

    fn cell_pin(db: &NetlistDB, name: &str) -> PinId {
        let (cell, _) = name.split_once('/').unwrap();
        db.cells[db.cell_id(cell).unwrap()]
            .pins
            .iter()
            .copied()
            .find(|&pin| db.pin_name(pin) == name)
            .unwrap()
    }

    /// The slack at `pin` on `side` with the design timed under `constraints`
    /// and wires without resistance or capacitance.
    fn slack_under(db: &NetlistDB, constraints: &str, pin: PinId, side: MinMax) -> f64 {
        timed(db, constraints, |_, sta| sta.slack(pin, side).time().get())
    }

    /// What `inspect` reads from the design timed under `constraints` with
    /// wires without resistance or capacitance.
    fn timed<R>(
        db: &NetlistDB,
        constraints: &str,
        inspect: impl FnOnce(&Netlist<'_>, &Sta) -> R,
    ) -> R {
        let lib = liberty::parse_file(&fixture("cts", "clock.lib")).unwrap();
        timed_with(db, &lib, constraints, inspect)
    }

    /// What `inspect` reads from the design timed with the cells of `lib`
    /// under `constraints` with wires without resistance or capacitance.
    fn timed_with<R>(
        db: &NetlistDB,
        lib: &Liberty,
        constraints: &str,
        inspect: impl FnOnce(&Netlist<'_>, &Sta) -> R,
    ) -> R {
        let sdc = sdc::parse_str(constraints, Path::new("test.sdc"), db, lib).unwrap();
        let link = Link::new(db, lib, &[]).unwrap();
        let netlist = Netlist::new(db, &link);
        let constants = Constants::find(&netlist);
        let clocks = ClockNetwork::from_sdc(&netlist, &sdc, &constants);
        let rc = WireRc {
            resistance: OhmsPerMeter::ZERO,
            capacitance: FaradsPerMeter::ZERO,
            layer: LayerId(1),
        };
        let graph = TimingGraph::new(
            &netlist,
            &clocks,
            &constants,
            rc,
            None,
            Estimator::Placement,
            PortConditions::from_sdc(&sdc),
        );
        inspect(
            &netlist,
            &Sta::new(&netlist, &graph, &clocks, &constants, &sdc),
        )
    }

    #[test]
    fn an_input_port_on_nothing_keeps_the_negative_of_its_input_delay_as_slack() {
        let db = dangling_port_design();
        let constraints = "create_clock -name clk -period 2.2 [get_ports clk]\n\
            set_input_delay 0.44 -clock clk [get_ports in]\n\
            set_input_delay 0.44 -clock clk [get_ports boot]\n";
        let boot = db.io_pins[db.io_pin_id("boot").unwrap()].pin;

        let slack = slack_under(&db, constraints, boot, MinMax::Max);

        assert!((slack + 0.44e-9).abs() < 1e-15, "{slack}");
    }

    #[test]
    fn a_setup_multicycle_moves_the_hold_check_by_the_extra_cycles() {
        let db = flop_to_flop_design();
        let clock = "create_clock -name clk -period 2.2 [get_ports clk]\n";
        let multicycle = format!(
            "{clock}set_multicycle_path 3 -setup -from [get_clocks clk] -to [get_clocks clk]\n"
        );
        let d = cell_pin(&db, "f2/D");

        let single = slack_under(&db, clock, d, MinMax::Min);
        let tripled = slack_under(&db, &multicycle, d, MinMax::Min);

        assert!(
            (single - tripled - 4.4e-9).abs() < 1e-15,
            "{single} vs {tripled}"
        );
    }

    #[test]
    fn a_clock_used_as_data_has_its_check_slack_at_the_pin_and_a_path() {
        let text = "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN cad ;\n\
            UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 40000 40000 ) ;\n\
            COMPONENTS 1 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\nEND COMPONENTS\n\
            PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
            NETS 1 ;\n- clk ( PIN clk ) ( f1 CK ) ( f1 D ) ;\nEND NETS\nEND DESIGN\n";
        let units = DbuPerMicron::new(2000).unwrap();
        let library = lef::parse_files(&[fixture("cts", "clock.lef")], units).unwrap();
        let db = def::parse_str(text, Path::new("cad.def"), library).unwrap();
        let d = cell_pin(&db, "f1/D");
        let constraints = "create_clock -name clk -period 2.2 [get_ports clk]\n";

        let (pin_slack, worst, path) = timed(&db, constraints, |netlist, sta| {
            (
                sta.slack(d, MinMax::Max),
                sta.worst_slack(MinMax::Max),
                sta.worst_slack_path(netlist, d, MinMax::Max),
            )
        });

        assert_eq!(worst, (pin_slack, Some(d)));
        assert!(pin_slack.is_constrained(), "{pin_slack:?}");
        assert_eq!(path.map(|p| p.slack), Some(pin_slack.time()));
    }

    /// A latch transparent while `CK` is high, whose setup before the
    /// closing fall is negative for rising data and positive for falling
    /// data, with a tenth of a nanosecond from `D` and from `CK` to `Q`.
    const LATCH: &str = "cell (LAT) { area : 4.5 ;
        latch (\"IQ\", \"IQN\") { data_in : \"D\" ; enable : \"CK\" ; }
        pin (D) { direction : input ; capacitance : 1.0 ;
          timing () { related_pin : \"CK\" ; timing_type : setup_falling ;
            rise_constraint (scalar) { values (\"-0.05\") ; }
            fall_constraint (scalar) { values (\"0.08\") ; } } }
        pin (CK) { direction : input ; clock : true ; capacitance : 1.0 ; }
        pin (Q) { direction : output ; function : \"IQ\" ;
          timing () { related_pin : \"D\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } }
          timing () { related_pin : \"CK\" ; timing_type : rising_edge ;
            cell_rise (scalar) { values (\"0.1\") ; }
            cell_fall (scalar) { values (\"0.1\") ; } } } }";

    /// The latch `l` of [`LATCH`] clocked from `clk`, its data from the
    /// port `in`, its output to the port `out`.
    fn latch_design() -> NetlistDB {
        design_with_copies(
            &def_text(
                "latched",
                &DefSections {
                    components: "- l LAT + PLACED ( 2000 2800 ) N ;\n",
                    nets: "- clk ( PIN clk ) ( l CK ) ;\n- in ( PIN in ) ( l D ) ;\n\
                           - out ( l Q ) ( PIN out ) ;\n",
                    inputs: &["clk", "in"],
                    outputs: &["out"],
                },
            ),
            &[("DFF_X1", "LAT")],
        )
    }

    #[test]
    fn data_borrows_up_to_the_setup_of_its_own_edge_before_the_latch_closes() {
        let db = latch_design();
        let q = cell_pin(&db, "l/Q");
        let constraints = "create_clock -name clk -period 2.0 [get_ports clk]\n\
                           set_input_delay 0.98 -clock clk [get_ports in]\n";

        let (rise, fall) = timed_with(&db, &cells_with(LATCH), constraints, |_, sta| {
            (
                sta.arrival(q, Edge::Rise, MinMax::Max).unwrap(),
                sta.arrival(q, Edge::Fall, MinMax::Max).unwrap(),
            )
        });

        assert!((rise.get() - 1.08e-9).abs() < 1e-15, "{rise:?}");
        assert!((fall.get() - 1.02e-9).abs() < 1e-15, "{fall:?}");
    }

    /// A buffer as `BUF_X1` but a tenth of a nanosecond slower.
    const SLOW_BUFFER: &str = "cell (BUF_SLOW) { area : 0.8 ;
        pin (A) { direction : input ; capacitance : 0.97 ; }
        pin (Z) { direction : output ; function : \"A\" ; max_capacitance : 60.0 ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (delay_t) { values (\"0.13, 0.40\", \"0.14, 0.41\") ; }
            cell_fall (delay_t) { values (\"0.13, 0.40\", \"0.14, 0.41\") ; }
            rise_transition (delay_t) { values (\"0.01, 0.40\", \"0.02, 0.41\") ; }
            fall_transition (delay_t) { values (\"0.01, 0.40\", \"0.02, 0.41\") ; } } } }";

    #[test]
    fn a_borrowing_latch_requires_its_data_where_an_upstream_resize_moved_it() {
        let mut db = design_with_copies(
            &def_text(
                "borrow",
                &DefSections {
                    components: "- b1 BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                                 - b2 BUF_X1 + PLACED ( 4000 2800 ) N ;\n\
                                 - l LAT + PLACED ( 6000 2800 ) N ;\n",
                    nets: "- clk ( PIN clk ) ( l CK ) ;\n- in ( PIN in ) ( b1 A ) ;\n\
                           - n1 ( b1 Z ) ( b2 A ) ;\n- n2 ( b2 Z ) ( l D ) ;\n\
                           - out ( l Q ) ( PIN out ) ;\n",
                    inputs: &["clk", "in"],
                    outputs: &["out"],
                },
            ),
            &[("DFF_X1", "LAT"), ("BUF_X1", "BUF_SLOW")],
        );
        let d = cell_pin(&db, "l/D");
        let lib = cells_with(&format!("{LATCH}\n{SLOW_BUFFER}"));
        let constraints = "create_clock -name clk -period 2.0 [get_ports clk]\n\
                           set_input_delay 0.4 -clock clk [get_ports in]\n";

        with_timed_session(&mut db, &lib, constraints, |resizer| {
            let b1 = resizer.db.cell_id("b1").unwrap();
            let slow = resizer.link.liberty.cell_id("BUF_SLOW").unwrap();
            assert!(resizer.replace_cell(b1, slow).unwrap());

            let netlist = resizer.netlist();
            let sdc = resizer.sta().sdc();
            let fresh = Sta::new(
                &netlist,
                &resizer.graph,
                resizer.clocks,
                resizer.constants,
                sdc,
            );
            assert_eq!(
                resizer.sta().slack(d, MinMax::Max),
                fresh.slack(d, MinMax::Max)
            );
        });
    }

    #[test]
    fn slack_is_infinite_without_an_arrival_or_a_bound() {
        assert!(!slack_of(None, Seconds::new(1.0), MinMax::Max).is_finite());
        assert!(
            !slack_of(
                Some(Seconds::new(1.0)),
                REQUIRED_INIT[MinMax::Max],
                MinMax::Max
            )
            .is_finite()
        );
        assert!(
            (slack_of(Some(Seconds::new(1.0)), Seconds::new(3.0), MinMax::Max).get() - 2.0).abs()
                < 1e-12
        );
        assert!(
            (slack_of(Some(Seconds::new(1.0)), Seconds::new(0.5), MinMax::Min).get() - 0.5).abs()
                < 1e-12
        );
    }
}
