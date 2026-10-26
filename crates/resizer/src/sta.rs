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
use pare_common::db::indices::{CellId, IoPinId, PinId};
use pare_common::db::liberty::{ArcRole, Edge, FuncExpr, Logic, MinMax, SequentialKind};
use pare_common::db::sdc::{ClockEdge, ClockId, MultiCyclePath, PathEnd, Sdc};
use rustc_hash::{FxHashMap, FxHashSet};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

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

/// Index of a tag in the analysis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TagId(u32);

impl TagId {
    const fn index(self) -> usize {
        self.0 as usize
    }
}

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
    arrival: [Option<f64>; 2],
    /// Required time per side: the latest allowed on the late side, the
    /// earliest on the early side; unconstrained starts at the identity.
    required: [f64; 2],
    prev: [Option<Prev>; 2],
}

impl Entry {
    const fn new(tag: TagId) -> Self {
        Self {
            tag,
            arrival: [None, None],
            required: REQUIRED_INIT,
            prev: [None, None],
        }
    }

    fn slack(&self, side: MinMax) -> f64 {
        slack_of(
            self.arrival[side.index()],
            self.required[side.index()],
            side,
        )
    }
}

/// The required time nothing constrains: no bound on the early side, none
/// on the late side.
const REQUIRED_INIT: [f64; 2] = [f64::NEG_INFINITY, f64::INFINITY];

/// Whether a required time is the unconstrained one.
const fn unconstrained(required: f64) -> bool {
    required.is_infinite()
}

/// Whether two pairs of times differ at all, bit for bit, which is what a
/// change worth propagating means.
fn differ(a: [f64; 2], b: [f64; 2]) -> bool {
    a.iter().zip(&b).any(|(&x, &y)| !same_time(x, y))
}

/// Whether two times, either possibly infinite, are equal within
/// `OpenSTA`'s tolerance, so that a change too small to matter does not
/// propagate through the graph.
fn same_time(a: f64, b: f64) -> bool {
    if a.is_infinite() || b.is_infinite() {
        return a.to_bits() == b.to_bits();
    }
    fuzzy::equal(a, b)
}

/// Whether `candidate` is a tighter required time than `held` on `side`.
fn tighter(side: MinMax, candidate: f64, held: f64) -> bool {
    match side {
        MinMax::Max => candidate < held,
        MinMax::Min => candidate > held,
    }
}

/// Slack from an arrival and a required time, infinite where either is
/// absent.
fn slack_of(arrival: Option<f64>, required: f64, side: MinMax) -> f64 {
    let Some(arrival) = arrival else {
        return f64::INFINITY;
    };
    if unconstrained(required) {
        return f64::INFINITY;
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
    /// The pin.
    pub pin: PinId,
    /// Its edge.
    pub edge: Edge,
    /// What launched the path.
    pub tag: Tag,
    /// When the edge arrives.
    pub arrival: f64,
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
    pub slack: f64,
}

impl Path {
    /// The endpoint.
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
    setup_required: f64,
    hold_required: f64,
    /// For a gated clock's hold check, made against the trailing edge in
    /// the cycle of the setup check rather than the one before.
    gated_hold_required: f64,
    /// For a latch whose enable is the capturing edge: when the latch opens
    /// before the closing edge the data must beat.
    latch_setup_required: f64,
    /// The cycles of the launching and capturing clocks the latch
    /// accounting landed on.
    latch_cycles: (i64, i64),
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
    const fn required(&self, side: MinMax, check: Check) -> f64 {
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

/// What a latch does with data arriving at `arrival`: `OpenSTA`'s
/// `latchRequired` for an ideal clock.
#[derive(Clone, Copy, Debug, PartialEq)]
struct LatchTiming {
    required: f64,
    borrow: f64,
    /// The data's arrival in the enable clock's frame, when it borrows.
    adjusted_arrival: f64,
}

/// The timing of a design, kept current as repair edits it.
#[derive(Clone, Debug)]
pub struct Sta {
    sdc: Sdc,
    tags: Vec<Tag>,
    tag_ids: FxHashMap<Tag, TagId>,
    /// Entries by pin and edge: `pin * 2 + edge`.
    entries: Vec<Vec<Entry>>,
    endpoints: Vec<PinId>,
    is_endpoint: Vec<bool>,
    /// Each endpoint pin's slack from its own checks, per side.
    end_slack: Vec<[f64; 2]>,
    tns: [f64; 2],
    worst: [Option<(f64, Option<PinId>)>; 2],
    accountings: FxHashMap<(ClockEdge, ClockEdge), Accounting>,
    exception_ports: FxHashSet<IoPinId>,
    latches: FxHashMap<CellId, Latch>,
    /// The enables of gated clocks, checked against their gate's clock.
    gated: FxHashMap<PinId, GatedEnable>,
    /// Whether each pin is in a propagated clock's network, below where the
    /// clock enters, so its clock arrives through the network's delays.
    propagated: Vec<bool>,
}

/// A clock edge a check captures with, and how late it arrives at the
/// check's clock pin beyond the edge's own time on each side: zero for an
/// ideal clock, its network's delay for a propagated one.
#[derive(Clone, Copy, Debug)]
struct Capture {
    edge: ClockEdge,
    latency: [f64; 2],
}

impl Capture {
    /// The latency a check on `side` captures with: the early arrival for
    /// setup and the late one for hold, as `OpenSTA` takes the capturing
    /// clock path on the side opposite the check.
    const fn latency(&self, side: MinMax) -> f64 {
        match side {
            MinMax::Max => self.latency[0],
            MinMax::Min => self.latency[1],
        }
    }
}

impl Sta {
    /// Times `netlist` from the delays in `graph` under `sdc`.
    #[must_use]
    pub fn new(
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
            tags: Vec::new(),
            tag_ids: FxHashMap::default(),
            entries: vec![Vec::new(); 2 * n],
            endpoints: Vec::new(),
            is_endpoint: vec![false; n],
            end_slack: vec![[f64::INFINITY; 2]; n],
            tns: [0.0; 2],
            worst: [None, None],
            accountings: FxHashMap::default(),
            exception_ports,
            latches: find_latches(netlist, constants),
            gated: gated_enables(netlist, clocks),
            propagated: (0..n)
                .map(|p| clocks.is_propagated(PinId::new(p)))
                .collect(),
        };
        sta.find_endpoints(netlist, clocks);
        let all: Vec<PinId> = (0..n).map(PinId::new).collect();
        sta.find_arrivals(netlist, graph, clocks, &all);
        sta.find_requireds(netlist, graph, &all);
        sta
    }

    /// Brings the timing up to date after the delays of `dirty_drivers`
    /// changed, as `graph` reports them.
    pub fn update(
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
            let net = netlist.db.pins[driver.index()].net;
            seeds.extend(netlist.loads(net));
            seeds.extend(graph.in_arcs(driver).iter().map(|a| a.from));
        }
        self.find_arrivals(netlist, graph, clocks, &seeds);
        self.find_requireds(netlist, graph, &seeds);
    }

    /// Makes room for pins added since the analysis started.
    fn grow(&mut self, netlist: &Netlist<'_>) {
        let n = netlist.db.pins.len();
        if self.is_endpoint.len() >= n {
            return;
        }
        self.entries.resize(2 * n, Vec::new());
        self.is_endpoint.resize(n, false);
        self.end_slack.resize(n, [f64::INFINITY; 2]);
        self.propagated.resize(n, false);
    }

    /// The constraints an analysis runs under.
    #[must_use]
    pub const fn sdc(&self) -> &Sdc {
        &self.sdc
    }

    /// The tag with this id.
    #[must_use]
    pub fn tag(&self, id: TagId) -> Tag {
        self.tags[id.index()]
    }

    fn intern(&mut self, tag: Tag) -> TagId {
        if let Some(&id) = self.tag_ids.get(&tag) {
            return id;
        }
        let id = TagId(self.tags.len() as u32);
        self.tags.push(tag);
        let _ = self.tag_ids.insert(tag, id);
        id
    }

    fn entries(&self, pin: PinId, edge: Edge) -> &[Entry] {
        self.entries
            .get(pin.index() * 2 + edge.index())
            .map_or(&[], Vec::as_slice)
    }

    /// The pins timing ends at: register inputs with checks from a clocked
    /// pin, asynchronous controls with recovery or removal checks, and
    /// output ports with delays.
    fn find_endpoints(&mut self, netlist: &Netlist<'_>, clocks: &ClockNetwork) {
        let db = netlist.db;
        self.endpoints.clear();
        self.is_endpoint.fill(false);
        for i in 0..db.pins.len() {
            let pin = PinId::new(i);
            if !netlist.is_load(pin) {
                continue;
            }
            let is_end = match db.pins[i].owner {
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
                self.is_endpoint[i] = true;
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
    pub fn is_endpoint(&self, pin: PinId) -> bool {
        self.is_endpoint.get(pin.index()).copied().unwrap_or(false)
    }

    /// An endpoint's slack from its own checks on `side`, infinite for a
    /// pin that is not one.
    #[must_use]
    pub fn endpoint_slack(&self, pin: PinId, side: MinMax) -> f64 {
        self.end_slack
            .get(pin.index())
            .map_or(f64::INFINITY, |s| s[side.index()])
    }

    /// A pin's slack on `side`: the least over its edges and tags, as
    /// `OpenSTA`'s vertex slack; infinite where nothing constrains it.
    #[must_use]
    pub fn slack(&self, pin: PinId, side: MinMax) -> f64 {
        Edge::BOTH
            .iter()
            .map(|&edge| self.slack_edge(pin, edge, side))
            .fold(f64::INFINITY, f64::min)
    }

    /// A pin's slack for one edge on `side`.
    #[must_use]
    pub fn slack_edge(&self, pin: PinId, edge: Edge, side: MinMax) -> f64 {
        self.entries(pin, edge)
            .iter()
            .map(|e| e.slack(side))
            .fold(f64::INFINITY, f64::min)
    }

    /// The arrival at a pin for `edge` on `side`: the latest on the late
    /// side, the earliest on the early side, over the tags; none where
    /// nothing arrives.
    #[must_use]
    pub fn arrival(&self, pin: PinId, edge: Edge, side: MinMax) -> Option<f64> {
        self.entries(pin, edge)
            .iter()
            .filter_map(|e| e.arrival[side.index()])
            .reduce(|a, b| if side.beats(b, a) { b } else { a })
    }

    /// The required time at a pin for `edge` on `side`, over the tags;
    /// none where nothing constrains it.
    #[must_use]
    pub fn required(&self, pin: PinId, edge: Edge, side: MinMax) -> Option<f64> {
        self.entries(pin, edge)
            .iter()
            .map(|e| e.required[side.index()])
            .filter(|&r| !unconstrained(r))
            .reduce(|a, b| if tighter(side, b, a) { b } else { a })
    }

    /// The sum of the negative endpoint slacks on `side`.
    #[must_use]
    pub const fn total_negative_slack(&self, side: MinMax) -> f64 {
        self.tns[side.index()]
    }

    /// The least endpoint slack on `side` and its endpoint.
    #[must_use]
    pub fn worst_slack(&self, side: MinMax) -> (f64, Option<PinId>) {
        if let Some(worst) = self.worst[side.index()] {
            return worst;
        }
        let mut worst = (f64::INFINITY, None);
        for &end in &self.endpoints {
            let slack = self.end_slack[end.index()][side.index()];
            if slack < worst.0 {
                worst = (slack, Some(end));
            }
        }
        worst
    }

    /// Remembers the worst slacks; they stay valid until an endpoint
    /// changes.
    fn cache_worst(&mut self) {
        for side in MinMax::BOTH {
            if self.worst[side.index()].is_none() {
                let worst = self.worst_slack(side);
                self.worst[side.index()] = Some(worst);
            }
        }
    }

    /// The path of least slack ending at `end` on `side`, as `OpenSTA`'s
    /// `vertexWorstSlackPath`; none when nothing constrains the pin.
    #[must_use]
    pub fn worst_slack_path(
        &self,
        netlist: &Netlist<'_>,
        end: PinId,
        side: MinMax,
    ) -> Option<Path> {
        let mut best: Option<(f64, Edge, Entry)> = None;
        for edge in Edge::BOTH {
            for entry in self.entries(end, edge) {
                let slack = entry.slack(side);
                if slack.is_finite() && best.is_none_or(|(s, _, _)| slack < s) {
                    best = Some((slack, edge, *entry));
                }
            }
        }
        let (slack, edge, entry) = best?;
        let mut points = Vec::new();
        let mut at = (end, edge, entry);
        loop {
            let (pin, edge, entry) = at;
            let via = match entry.prev[side.index()] {
                None => Via::Start,
                Some(Prev { set: None, .. }) => Via::Wire,
                Some(Prev { set: Some(set), .. }) => Via::Arc { set: set as usize },
            };
            points.push(PathPoint {
                pin,
                edge,
                tag: self.tags[entry.tag.index()],
                arrival: entry.arrival[side.index()].unwrap_or(0.0),
                via,
            });
            let Some(prev) = entry.prev[side.index()] else {
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
        let PinOwner::Cell { cell, .. } = netlist.db.pins[pin.index()].owner else {
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
    /// the fanout.
    fn find_arrivals(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        clocks: &ClockNetwork,
        seeds: &[PinId],
    ) {
        let n = netlist.db.pins.len();
        let mut queued = vec![false; n];
        let mut heap: BinaryHeap<Reverse<(u32, usize)>> = BinaryHeap::new();
        for &pin in seeds {
            if pin.index() < n && !queued[pin.index()] {
                queued[pin.index()] = true;
                heap.push(Reverse((graph.level(pin), pin.index())));
            }
        }
        while let Some(Reverse((_, index))) = heap.pop() {
            queued[index] = false;
            let pin = PinId::new(index);
            if !self.visit_arrivals(netlist, graph, clocks, pin) {
                continue;
            }
            for next in successors(netlist, graph, pin) {
                if !queued[next.index()] {
                    queued[next.index()] = true;
                    heap.push(Reverse((graph.level(next), next.index())));
                }
            }
        }
        self.cache_worst();
    }

    /// Computes a pin's arrivals from its predecessors or its seeds and
    /// stores them; true when they changed.
    fn visit_arrivals(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        clocks: &ClockNetwork,
        pin: PinId,
    ) -> bool {
        let is_driver = netlist.is_driver(pin);
        let is_load = netlist.is_load(pin);
        if !is_driver && !is_load {
            return self.clear_arrivals(pin);
        }
        let mut changed = false;
        for edge in Edge::BOTH {
            let fresh = if clocks.is_ideal(pin) {
                self.clock_seeds(clocks, pin, edge)
            } else if clocks.is_propagated(pin) {
                self.network_arrivals(netlist, graph, pin, edge)
            } else if let PinOwner::Io(io) = netlist.db.pins[pin.index()].owner
                && is_driver
            {
                self.port_seeds(io, edge)
            } else if is_driver {
                self.driver_arrivals(netlist, graph, pin, edge)
            } else {
                self.load_arrivals(netlist, graph, pin, edge)
            };
            changed |= self.store_arrivals(pin, edge, fresh);
        }
        if changed && self.is_endpoint(pin) {
            self.refresh_endpoint(netlist, graph, pin);
        }
        changed
    }

    fn clear_arrivals(&mut self, pin: PinId) -> bool {
        let mut changed = false;
        for edge in Edge::BOTH {
            let slot = &mut self.entries[pin.index() * 2 + edge.index()];
            if !slot.is_empty() {
                slot.clear();
                changed = true;
            }
        }
        changed
    }

    /// Replaces a pin's entries, keeping the required times of tags that
    /// stay; true when the tags or arrivals changed.
    fn store_arrivals(&mut self, pin: PinId, edge: Edge, mut fresh: Vec<Entry>) -> bool {
        let slot = &mut self.entries[pin.index() * 2 + edge.index()];
        for entry in &mut fresh {
            if let Some(old) = slot.iter().find(|o| o.tag == entry.tag) {
                entry.required = old.required;
            }
        }
        let same_arrival = |o: &Entry, f: &Entry| {
            o.arrival.iter().zip(&f.arrival).all(|(x, y)| match (x, y) {
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
        changed
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
                arrival: [Some(time), Some(time)],
                required: REQUIRED_INIT,
                prev: [None, None],
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
                arrival: [Some(0.0), Some(0.0)],
                required: REQUIRED_INIT,
                prev: [None, None],
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
                MinMax::BOTH.map(|side| delay.delays.get(edge, side).map(|d| edge_time + d));
            if arrival.iter().all(Option::is_none) {
                continue;
            }
            entries.push(Entry {
                tag,
                arrival,
                required: REQUIRED_INIT,
                prev: [None, None],
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
            .filter(|e| self.tags[e.tag.index()].is_clock)
            .collect()
    }

    /// The clock edges arriving at clock pin `pin` on `edge`, each with its
    /// latency there.
    fn captures(&self, pin: PinId, edge: Edge) -> Vec<Capture> {
        self.entries(pin, edge)
            .iter()
            .filter_map(|e| {
                let t = self.tags[e.tag.index()];
                let launch = t.launch.filter(|_| t.is_clock)?;
                let time = self.sdc.edge_time(launch);
                let latency = e.arrival.map(|a| a.map_or(0.0, |a| a - time));
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
    fn clock_slew(&self, graph: &TimingGraph, pin: PinId, edge: Edge, side: MinMax) -> f64 {
        if self.propagated.get(pin.index()).copied().unwrap_or(false) {
            graph.slew_side(pin, side).get(edge)
        } else {
            0.0
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
        let latch = match netlist.db.pins[driver.index()].owner {
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
            let in_network = self
                .propagated
                .get(driver.index())
                .copied()
                .unwrap_or(false);
            for from in from_entries {
                let tag = if in_network {
                    from.tag
                } else {
                    self.data_tag(from.tag)
                };
                for side in MinMax::BOTH {
                    let Some(arrival) = from.arrival[side.index()] else {
                        continue;
                    };
                    let delay = arc.delay[side.index()] * derate[side.index()];
                    let prev = Prev {
                        pin: arc.arc.from,
                        edge: arc.from_edge,
                        tag: from.tag,
                        set: Some(arc.arc.set as u32),
                    };
                    match through_latch {
                        Some(latch) if latch.state == LatchState::Enabled => {
                            for (q_tag, adjusted) in self
                                .latch_departures(netlist, graph, latch, from.tag, arrival, side)
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
        let net = netlist.db.pins[load.index()].net;
        let derate = self.sdc.derating.net;
        for driver in netlist.drivers(net) {
            if driver == load {
                continue;
            }
            for from in self.entries(driver, edge) {
                for side in MinMax::BOTH {
                    let Some(arrival) = from.arrival[side.index()] else {
                        continue;
                    };
                    let wire = graph.wire_delay(load, edge, side);
                    merge_arrival(
                        &mut entries,
                        from.tag,
                        side,
                        wire.mul_add(derate[side.index()], arrival),
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
        let value = self.tags[tag.index()];
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
        data_tag: TagId,
        arrival: f64,
        side: MinMax,
    ) -> Vec<(TagId, f64)> {
        let tag = self.tags[data_tag.index()];
        let Some(launch) = tag.launch else {
            return Vec::new();
        };
        let enables = self.captures(latch.enable, latch.open_edge);
        let related = self.clock_slew(graph, latch.enable, opposite(latch.open_edge), side);
        let margin = latch_margin(netlist, graph, latch, side, related);
        let mut departures = Vec::new();
        for capture in enables {
            let enable = capture.edge;
            let to = PathEnd::Clock(enable.clock);
            if self.is_false(tag, to, Some(enable.clock), MinMax::Max) {
                continue;
            }
            let timing = self.latch_timing(tag, launch, capture, to, arrival, margin);
            if fuzzy::greater(timing.borrow, 0.0) {
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

    /// `OpenSTA`'s `latchRequired` for an ideal clock: data arriving before
    /// the latch opens is required by the opening edge; later data borrows
    /// from the open window, up to its width less the setup.
    fn latch_timing(
        &mut self,
        tag: Tag,
        launch: ClockEdge,
        capture: Capture,
        to: PathEnd,
        arrival: f64,
        margin: f64,
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
                borrow: 0.0,
                adjusted_arrival: arrival,
            };
        }
        let clock = self.sdc.clock(enable.clock);
        let mut width = clock.edge_time(opposite(enable.edge)) - clock.edge_time(enable.edge);
        if width < 0.0 {
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
        let shift = (src_cycle as f64).mul_add(
            self.sdc.clock(launch.clock).period,
            -(tgt_cycle as f64) * clock.period,
        );
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
        let mut queued = vec![false; n];
        let mut heap: BinaryHeap<(u32, usize)> = BinaryHeap::new();
        for &pin in seeds {
            if pin.index() < n && !queued[pin.index()] {
                queued[pin.index()] = true;
                heap.push((graph.level(pin), pin.index()));
            }
        }
        while let Some((_, index)) = heap.pop() {
            queued[index] = false;
            let pin = PinId::new(index);
            if !self.visit_requireds(netlist, graph, pin) {
                continue;
            }
            for prev in predecessors(netlist, graph, pin) {
                if !queued[prev.index()] {
                    queued[prev.index()] = true;
                    heap.push((graph.level(prev), prev.index()));
                }
            }
        }
        self.cache_worst();
    }

    /// Computes a pin's required times from its fanout and its checks and
    /// stores them; true when they changed.
    fn visit_requireds(&mut self, netlist: &Netlist<'_>, graph: &TimingGraph, pin: PinId) -> bool {
        let mut changed = false;
        for edge in Edge::BOTH {
            let entries = self.entries(pin, edge).to_vec();
            for (k, entry) in entries.iter().enumerate() {
                let required = if self.tags[entry.tag.index()].is_clock {
                    REQUIRED_INIT
                } else {
                    let mut required = self.fanout_requireds(netlist, graph, pin, edge, entry.tag);
                    if self.is_endpoint(pin) {
                        let checks = self.check_requireds(netlist, graph, pin, edge, entry);
                        for side in MinMax::BOTH {
                            if let Some(check) = checks[side.index()]
                                && tighter(side, check, required[side.index()])
                            {
                                required[side.index()] = check;
                            }
                        }
                    }
                    required
                };
                let slot = &mut self.entries[pin.index() * 2 + edge.index()][k];
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

    /// The factors the delays of the arcs into `driver` are scaled by, for
    /// the early and late sides: its instance's own derate where set, else
    /// the design's.
    fn cell_derates(&self, netlist: &Netlist<'_>, driver: PinId) -> [f64; 2] {
        let cell = match netlist.db.pins[driver.index()].owner {
            PinOwner::Cell { cell, .. } => Some(cell),
            PinOwner::Io(_) => None,
        };
        MinMax::BOTH.map(|side| self.sdc.cell_derate(cell, side))
    }

    /// The required times a pin's fanout imposes for one tag.
    fn fanout_requireds(
        &self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        pin: PinId,
        edge: Edge,
        tag: TagId,
    ) -> [f64; 2] {
        let mut required = REQUIRED_INIT;
        if netlist.is_driver(pin) {
            let net = netlist.db.pins[pin.index()].net;
            let derate = self.sdc.derating.net;
            for load in netlist.loads(net) {
                if load == pin {
                    continue;
                }
                for entry in self.entries(load, edge).iter().filter(|e| e.tag == tag) {
                    for side in MinMax::BOTH {
                        let bound = entry.required[side.index()];
                        if unconstrained(bound) {
                            continue;
                        }
                        let wire = graph.wire_delay(load, edge, side);
                        let candidate = wire.mul_add(-derate[side.index()], bound);
                        if tighter(side, candidate, required[side.index()]) {
                            required[side.index()] = candidate;
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
                            let bound = entry.required[side.index()];
                            if unconstrained(bound) {
                                continue;
                            }
                            let candidate =
                                arc.delay[side.index()].mul_add(-derate[side.index()], bound);
                            if tighter(side, candidate, required[side.index()]) {
                                required[side.index()] = candidate;
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
    ) -> [Option<f64>; 2] {
        let tag = self.tags[entry.tag.index()];
        let Some(launch) = tag.launch else {
            return [None, None];
        };
        let mut required: [Option<f64>; 2] = [None, None];
        let mut tighten = |side: MinMax, candidate: f64| {
            let slot = &mut required[side.index()];
            if slot.is_none_or(|held| tighter(side, candidate, held)) {
                *slot = Some(candidate);
            }
        };
        let db = netlist.db;
        match db.pins[pin.index()].owner {
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
                    return [None, None];
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
                    let Some(arrival) = entry.arrival[side.index()] else {
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
    ) -> Vec<(MinMax, f64)> {
        let mut out = Vec::new();
        for side in MinMax::BOTH {
            if entry.arrival[side.index()].is_none() {
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
    ) -> f64 {
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
    ) -> f64 {
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
    ) -> f64 {
        let Some(mcp) = self.multicycle(tag, to, Some(target.clock), side) else {
            return 0.0;
        };
        let period = |use_end: bool| {
            let clock = if use_end { target.clock } else { launch.clock };
            self.sdc.clock(clock).period
        };
        match side {
            MinMax::Max => f64::from(mcp.multiplier(MinMax::Max) - 1) * period(mcp.use_end_clock),
            MinMax::Min => {
                let setup = self.multicycle(tag, to, Some(target.clock), MinMax::Max);
                let hold_term = f64::from(mcp.multiplier(MinMax::Min)) * period(mcp.use_end_clock);
                match setup {
                    Some(setup) => f64::from(setup.multiplier(MinMax::Max) - 1)
                        .mul_add(period(setup.use_end_clock), -hold_term),
                    None => -hold_term,
                }
            }
        }
    }

    /// The cycle accounting between two clock edges, found once.
    fn accounting(&mut self, launch: ClockEdge, target: ClockEdge) -> Accounting {
        if let Some(&found) = self.accountings.get(&(launch, target)) {
            return found;
        }
        let found = cycle_accounting(&self.sdc, launch, target);
        let _ = self.accountings.insert((launch, target), found);
        found
    }

    /// Recomputes an endpoint's own-check slack and the totals it enters.
    fn refresh_endpoint(&mut self, netlist: &Netlist<'_>, graph: &TimingGraph, pin: PinId) {
        let mut slack = [f64::INFINITY; 2];
        for edge in Edge::BOTH {
            let entries = self.entries(pin, edge).to_vec();
            for entry in entries {
                let checks = self.check_requireds(netlist, graph, pin, edge, &entry);
                for side in MinMax::BOTH {
                    if let Some(required) = checks[side.index()] {
                        let s = slack_of(entry.arrival[side.index()], required, side);
                        slack[side.index()] = slack[side.index()].min(s);
                    }
                }
            }
        }
        let old = self.end_slack[pin.index()];
        if !differ(old, slack) {
            return;
        }
        for side in MinMax::BOTH {
            let i = side.index();
            if old[i].is_finite() && old[i] < 0.0 {
                self.tns[i] -= old[i];
            }
            if slack[i].is_finite() && slack[i] < 0.0 {
                self.tns[i] += slack[i];
            }
            self.worst[i] = None;
        }
        self.end_slack[pin.index()] = slack;
    }
}

/// The setup a latch's data pin demands before the closing edge, on
/// `side`'s slew, over its setup arcs.
fn latch_margin(
    netlist: &Netlist<'_>,
    graph: &TimingGraph,
    latch: Latch,
    side: MinMax,
    clock_slew: f64,
) -> f64 {
    let db = netlist.db;
    let Some((lib, port)) = netlist.link.pin_port(db, latch.data) else {
        return 0.0;
    };
    let slews = graph.slew_side(latch.data, side);
    netlist
        .link
        .liberty
        .cell(lib)
        .arc_sets
        .iter()
        .filter(|set| set.to == port && set.role == ArcRole::Setup)
        .flat_map(|set| set.arcs.iter())
        .map(|arc| arc.check_value(clock_slew, slews.get(arc.to_edge)))
        .fold(0.0, f64::max)
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
    for i in 0..db.cells.len() {
        let cell = CellId::new(i);
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
fn merge_arrival(entries: &mut Vec<Entry>, tag: TagId, side: MinMax, time: f64, prev: Prev) {
    let index = entries
        .iter()
        .position(|e| e.tag == tag)
        .unwrap_or_else(|| {
            entries.push(Entry::new(tag));
            entries.len() - 1
        });
    let entry = &mut entries[index];
    let slot = &mut entry.arrival[side.index()];
    if slot.is_none_or(|held| side.beats(time, held)) {
        *slot = Some(time);
        entry.prev[side.index()] = Some(prev);
    }
}

/// The pins timing continues to from `pin`: the loads of a driver's net,
/// the drivers an input's arcs reach.
fn successors(netlist: &Netlist<'_>, graph: &TimingGraph, pin: PinId) -> Vec<PinId> {
    let mut next = Vec::new();
    if netlist.is_driver(pin) {
        let net = netlist.db.pins[pin.index()].net;
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

/// The pins timing comes from into `pin`.
fn predecessors(netlist: &Netlist<'_>, graph: &TimingGraph, pin: PinId) -> Vec<PinId> {
    let mut prev = Vec::new();
    if netlist.is_load(pin) {
        let net = netlist.db.pins[pin.index()].net;
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
    let mut accounting = Accounting {
        setup_required: 0.0,
        hold_required: 0.0,
        gated_hold_required: 0.0,
        latch_setup_required: 0.0,
        latch_cycles: (0, 0),
    };
    if !(tgt_period > 0.0 && src_period > 0.0) {
        return accounting;
    }
    let first_cycle = |time: f64, period: f64| -> i64 {
        if time < 0.0 {
            1
        } else if time < period {
            0
        } else {
            -1
        }
    };
    let tgt_max_cycle: i64 = if tgt_period > src_period {
        100
    } else {
        ((src_period / tgt_period).ceil() as i64).max(1000)
    };
    let mut setup_delay = f64::INFINITY;
    let mut hold_delay = f64::INFINITY;
    let mut gated_hold_delay = f64::INFINITY;
    let mut latch_delay = f64::INFINITY;
    let mut tgt_past_src = false;
    let mut src_past_tgt = false;
    let mut tgt_cycle = first_cycle(tgt_edge_time, tgt_period);
    while tgt_cycle <= tgt_max_cycle {
        let tgt_cycle_start = tgt_cycle as f64 * tgt_period;
        let tgt_time = tgt_cycle_start + tgt_edge_time;
        let tgt_opposite = tgt_cycle_start + tgt_opposite_time;
        let mut src_cycle = first_cycle(src_edge_time, src_period);
        loop {
            let src_cycle_start = src_cycle as f64 * src_period;
            let src_time = src_cycle_start + src_edge_time;
            if tgt_past_src && src_past_tgt && fuzzy::equal(src_cycle_start, tgt_cycle_start) {
                return accounting;
            }
            if fuzzy::greater(src_cycle_start, tgt_cycle_start + tgt_period) && src_past_tgt {
                break;
            }
            if fuzzy::greater(tgt_time, src_time) {
                tgt_past_src = true;
                let delay = tgt_time - src_time;
                if fuzzy::less(delay, setup_delay) {
                    setup_delay = delay;
                    accounting.setup_required = tgt_time - src_cycle_start;
                }
            }
            if fuzzy::greater(tgt_opposite, src_time) {
                let delay = tgt_opposite - src_time;
                if fuzzy::less(delay, latch_delay) {
                    latch_delay = delay;
                    let (mut latch_time, mut latch_cycle) = (tgt_time, tgt_cycle);
                    if tgt_time > tgt_opposite {
                        latch_time -= tgt_period;
                        latch_cycle -= 1;
                    }
                    accounting.latch_setup_required = latch_time - src_cycle_start;
                    accounting.latch_cycles = (src_cycle, latch_cycle);
                }
            }
            if fuzzy::less_equal(tgt_time, src_time) {
                let delay = src_time - tgt_time;
                src_past_tgt = true;
                if fuzzy::less(delay, hold_delay) {
                    hold_delay = delay;
                    accounting.hold_required = tgt_time - src_cycle_start;
                }
            }
            if fuzzy::less_equal(tgt_opposite, src_time) {
                let delay = src_time - tgt_time;
                if fuzzy::less(delay, gated_hold_delay) {
                    gated_hold_delay = delay;
                    accounting.gated_hold_required = tgt_time - src_cycle_start;
                }
            }
            src_cycle += 1;
        }
        tgt_cycle += 1;
    }
    accounting
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::sdc::Clock;

    fn clock(name: &str, period: f64, rise: f64, fall: f64) -> Clock {
        Clock {
            name: name.into(),
            period,
            rise,
            fall,
            sources: Vec::new(),
            uncertainty: [None, None],
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
            clocks: vec![clock("clk", 2e-9, 0.0, 1e-9)],
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Rise));

        assert!((acct.setup_required - 2e-9).abs() < 1e-20);
        assert!(acct.hold_required.abs() < 1e-20);
    }

    #[test]
    fn a_gated_clocks_hold_check_is_the_trailing_edge_of_the_same_cycle() {
        let sdc = Sdc {
            clocks: vec![clock("clk", 2e-9, 0.0, 1e-9)],
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Fall));

        assert!((acct.gated_hold_required - 1e-9).abs() < 1e-20);
        assert!((acct.required(MinMax::Min, Check::GatedClock) - 1e-9).abs() < 1e-20);
        assert!((acct.required(MinMax::Min, Check::Ordinary) + 1e-9).abs() < 1e-20);
    }

    #[test]
    fn a_falling_capture_of_a_rising_launch_is_half_a_period_away() {
        let sdc = Sdc {
            clocks: vec![clock("clk", 2e-9, 0.0, 1e-9)],
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Fall));

        assert!((acct.setup_required - 1e-9).abs() < 1e-20);
        assert!((acct.hold_required + 1e-9).abs() < 1e-20);
    }

    #[test]
    fn a_latch_opening_on_the_fall_is_required_at_the_fall_before_the_close() {
        let sdc = Sdc {
            clocks: vec![clock("clk", 2.2e-9, 0.0, 1.1e-9)],
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(0, Edge::Fall));

        assert!((acct.latch_setup_required - 1.1e-9).abs() < 1e-20);
        assert_eq!(acct.latch_cycles, (0, 0));
    }

    #[test]
    fn a_fast_clock_captures_a_slow_launch_at_its_next_edge() {
        let sdc = Sdc {
            clocks: vec![
                clock("slow", 4e-9, 0.0, 2e-9),
                clock("fast", 1e-9, 0.0, 0.5e-9),
            ],
            ..Sdc::default()
        };

        let acct = cycle_accounting(&sdc, edge(0, Edge::Rise), edge(1, Edge::Rise));

        assert!((acct.setup_required - 1e-9).abs() < 1e-20);
        assert!(acct.hold_required.abs() < 1e-20);
    }

    #[test]
    fn slack_is_infinite_without_an_arrival_or_a_bound() {
        assert!(slack_of(None, 1.0, MinMax::Max).is_infinite());
        assert!(slack_of(Some(1.0), REQUIRED_INIT[1], MinMax::Max).is_infinite());
        assert!((slack_of(Some(1.0), 3.0, MinMax::Max) - 2.0).abs() < 1e-12);
        assert!((slack_of(Some(1.0), 0.5, MinMax::Min) - 0.5).abs() < 1e-12);
    }
}
