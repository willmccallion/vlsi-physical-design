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
use pare_common::db::liberty::{ArcSetId, Edge, EdgeValues, MinMax, SideValues};
use pare_common::db::sdc::{ClockEdge, ClockId, Sdc};
use pare_common::units::Seconds;
use rustc_hash::{FxHashMap, FxHashSet};

mod arrivals;
mod cycles;
mod requireds;

use arrivals::find_latches;
use cycles::cycle_accounting;

/// A pin's slack as `OpenSTA` keeps it, where nothing constraining the pin
/// is a value of its own that orders after every real slack.
// NOTE: OpenSTA holds an unconstrained slack as its finite INF (1e30); PARE
// keeps it apart from every time, ordered the same.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Slack {
    /// The slack of a constrained pin.
    Constrained(Seconds),
    /// Nothing constrains the pin.
    Unconstrained,
}

impl Slack {
    /// The slack of a time the analysis holds, where an infinite time
    /// means nothing constrains the pin.
    fn of(time: Seconds) -> Self {
        if time == Seconds::INFINITY {
            Self::Unconstrained
        } else {
            Self::Constrained(time)
        }
    }

    /// The slack's time; none where nothing constrains the pin.
    #[must_use]
    pub const fn time(self) -> Option<Seconds> {
        match self {
            Self::Constrained(time) => Some(time),
            Self::Unconstrained => None,
        }
    }

    /// Whether the pin is constrained and its slack below `bound`.
    #[must_use]
    pub(crate) fn is_below(self, bound: Seconds) -> bool {
        self.time().is_some_and(|time| time < bound)
    }

    /// The total order of slacks, an unconstrained one last.
    #[must_use]
    pub(crate) fn total_cmp(&self, other: &Self) -> std::cmp::Ordering {
        use std::cmp::Ordering;
        match (self, other) {
            (Self::Constrained(a), Self::Constrained(b)) => a.total_cmp(b),
            (Self::Constrained(_), Self::Unconstrained) => Ordering::Less,
            (Self::Unconstrained, Self::Constrained(_)) => Ordering::Greater,
            (Self::Unconstrained, Self::Unconstrained) => Ordering::Equal,
        }
    }

    /// The slack's time with an unconstrained pin's as infinite, for the
    /// arithmetic that mirrors `OpenSTA`'s `INF`.
    #[must_use]
    pub(crate) const fn time_or_infinity(self) -> Seconds {
        match self {
            Self::Constrained(time) => time,
            Self::Unconstrained => Seconds::INFINITY,
        }
    }
}

impl PartialOrd for Slack {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        use std::cmp::Ordering;
        match (self, other) {
            (Self::Constrained(a), Self::Constrained(b)) => a.partial_cmp(b),
            (Self::Constrained(_), Self::Unconstrained) => Some(Ordering::Less),
            (Self::Unconstrained, Self::Constrained(_)) => Some(Ordering::Greater),
            (Self::Unconstrained, Self::Unconstrained) => Some(Ordering::Equal),
        }
    }
}

/// The worst endpoint slack a side last found, until an endpoint changes.
#[derive(Clone, Copy, Debug, PartialEq)]
enum WorstSlack {
    /// Not found since an endpoint last changed.
    Stale,
    /// Found: the worst endpoint, none where no endpoint is constrained.
    Found(Option<EndpointSlack>),
}

/// An endpoint and its slack.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EndpointSlack {
    /// The endpoint.
    pub pin: PinId,
    /// Its slack, which something constrains.
    pub slack: Seconds,
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
    set: Option<ArcSetId>,
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

/// Whether two pairs of times differ by more than `OpenSTA`'s tolerance,
/// which is what a change worth propagating means.
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

/// The tightest required time found so far on each side.
#[derive(Clone, Copy, Debug)]
struct TightestRequired(SideValues<Option<Seconds>>);

impl Default for TightestRequired {
    fn default() -> Self {
        Self(SideValues::both(None))
    }
}

impl TightestRequired {
    /// Keeps `candidate` on `side` where it is tighter than the time held.
    fn tighten(&mut self, side: MinMax, candidate: Seconds) {
        let slot = &mut self.0[side];
        if slot.is_none_or(|held| tighter(side, candidate, held)) {
            *slot = Some(candidate);
        }
    }
}

/// One tag's entry at an endpoint and edge whose checks are being found,
/// with the clock edge that launched it.
#[derive(Clone, Copy, Debug)]
struct CheckedEnd<'e> {
    pin: PinId,
    edge: Edge,
    entry: &'e Entry,
    tag: Tag,
    launch: ClockEdge,
}

/// What a check arc's data pin feeds, which decides the clock edge its
/// setup check captures on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CheckedInput {
    /// A flop or a latch that is not transparent here.
    Register,
    /// An enabled latch's data input, whose setup is checked against the
    /// closing edge, as `OpenSTA`'s latch `PathEndLatchCheck`.
    EnabledLatch,
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
        /// The group in the cell's arc sets.
        set: ArcSetId,
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
    /// The worst endpoint on each side, once found.
    worst: SideValues<WorstSlack>,
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
            worst: SideValues::both(WorstSlack::Stale),
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

    /// The pins timing ends at, as `Search::isEndpoint`: register inputs
    /// with checks from a clocked pin, asynchronous controls with recovery
    /// or removal checks, and output ports with delays.
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
        Slack::of(
            Edge::BOTH
                .iter()
                .map(|&edge| self.edge_slack_time(pin, edge, side))
                .fold(Seconds::INFINITY, Seconds::min),
        )
    }

    /// A pin's slack for one edge on `side`.
    #[must_use]
    pub(crate) fn slack_edge(&self, pin: PinId, edge: Edge, side: MinMax) -> Slack {
        Slack::of(self.edge_slack_time(pin, edge, side))
    }

    /// The least slack time over a pin's tags for one edge, infinite where
    /// nothing constrains it.
    fn edge_slack_time(&self, pin: PinId, edge: Edge, side: MinMax) -> Seconds {
        self.entries(pin, edge)
            .iter()
            .map(|e| e.slack(side))
            .fold(Seconds::INFINITY, Seconds::min)
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

    /// The endpoint of least slack on `side`; none where no endpoint is
    /// constrained.
    #[must_use]
    pub fn worst_slack(&self, side: MinMax) -> Option<EndpointSlack> {
        if let WorstSlack::Found(worst) = self.worst[side] {
            return worst;
        }
        let mut worst: Option<EndpointSlack> = None;
        for &end in &self.endpoints {
            let slack = self.end_slack[end][side];
            if slack.is_finite() && worst.is_none_or(|w| fuzzy::less(slack, w.slack)) {
                worst = Some(EndpointSlack { pin: end, slack });
            }
        }
        worst
    }

    /// Remembers the worst slacks; they stay valid until an endpoint
    /// changes.
    fn cache_worst(&mut self) {
        for side in MinMax::BOTH {
            if self.worst[side] == WorstSlack::Stale {
                self.worst[side] = WorstSlack::Found(self.worst_slack(side));
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
                Some(Prev { set: Some(set), .. }) => Via::Arc { set },
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
            self.worst[side] = WorstSlack::Stale;
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

/// Keeps the arrival for `tag` on `side` in `entries`, adding the tag when
/// new, as `ArrivalVisitor::visitFromToPath` keeps the latest arrival for
/// the late side and the earliest for the early side.
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
        timed(db, constraints, |_, sta| {
            sta.slack(pin, side).time().unwrap().get()
        })
    }

    /// Whether `constraints` constrain `pin` on `side`.
    fn constrained_under(db: &NetlistDB, constraints: &str, pin: PinId, side: MinMax) -> bool {
        timed(db, constraints, |_, sta| {
            sta.slack(pin, side).time().is_some()
        })
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

        let pin_slack = pin_slack.time().unwrap();
        assert_eq!(
            worst,
            Some(EndpointSlack {
                pin: d,
                slack: pin_slack
            })
        );
        assert_eq!(path.map(|p| p.slack), Some(pin_slack));
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
    fn a_false_path_leaves_its_endpoint_unconstrained() {
        let db = flop_to_flop_design();
        let d = cell_pin(&db, "f2/D");
        let clock = "create_clock -name clk -period 2.2 [get_ports clk]\n";
        let false_path =
            format!("{clock}set_false_path -from [get_clocks clk] -to [get_clocks clk]\n");

        assert!(constrained_under(&db, clock, d, MinMax::Max));
        assert!(!constrained_under(&db, &false_path, d, MinMax::Max));
    }

    /// A flop clocked from `clk` driving the output port `out`.
    fn flop_to_port_design() -> NetlistDB {
        design_with_copies(
            &def_text(
                "out",
                &DefSections {
                    components: "- f DFF_X1 + PLACED ( 2000 2800 ) N ;\n",
                    nets: "- clk ( PIN clk ) ( f CK ) ;\n- out ( f Q ) ( PIN out ) ;\n",
                    inputs: &["clk"],
                    outputs: &["out"],
                },
            ),
            &[],
        )
    }

    #[test]
    fn an_output_delay_takes_its_time_off_the_capturing_edge() {
        let db = flop_to_port_design();
        let out = db.io_pins[db.io_pin_id("out").unwrap()].pin;
        let constraints = "create_clock -name clk -period 2.2 [get_ports clk]\n\
                           set_output_delay 0.5 -clock clk [get_ports out]\n";

        let (slack, arrival) = timed(&db, constraints, |_, sta| {
            let latest = Edge::BOTH
                .iter()
                .filter_map(|&edge| sta.arrival(out, edge, MinMax::Max))
                .fold(Seconds::new(f64::NEG_INFINITY), Seconds::max);
            (sta.slack(out, MinMax::Max).time().unwrap(), latest)
        });

        assert!(arrival > Seconds::ZERO);
        assert!(
            ((slack + arrival).get() - 1.7e-9).abs() < 1e-15,
            "{slack:?} + {arrival:?}"
        );
    }

    /// A flop clocked from `clk` through buffer `cb`.
    fn buffered_clock_design() -> NetlistDB {
        design_with_copies(
            &def_text(
                "cb",
                &DefSections {
                    components: "- cb BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                                 - f DFF_X1 + PLACED ( 4000 2800 ) N ;\n",
                    nets: "- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( f CK ) ;\n",
                    inputs: &["clk"],
                    outputs: &[],
                },
            ),
            &[],
        )
    }

    #[test]
    fn a_propagated_clock_arrives_through_its_buffers_and_an_ideal_one_at_once() {
        let db = buffered_clock_design();
        let ck = cell_pin(&db, "f/CK");
        let clock = "create_clock -name clk -period 2.2 [get_ports clk]\n";
        let propagated = format!("{clock}set_propagated_clock [all_clocks]\n");

        let ideal = timed(&db, clock, |_, sta| sta.clock_rise_arrival(ck));
        let late = timed(&db, &propagated, |_, sta| sta.clock_rise_arrival(ck));

        assert_eq!(ideal, Some(Seconds::ZERO));
        assert!(late.is_some_and(|t| t > Seconds::ZERO), "{late:?}");
    }

    /// The clock `clk` gated by the port `en` in and gate `g` on its way to
    /// flop `f`.
    fn gated_clock_design() -> NetlistDB {
        design_with_copies(
            &def_text(
                "gated",
                &DefSections {
                    components: "- g GATE + PLACED ( 2000 2800 ) N ;\n\
                                 - f DFF_X1 + PLACED ( 4000 2800 ) N ;\n",
                    nets: "- clk ( PIN clk ) ( g A ) ;\n- en ( PIN en ) ( g B ) ;\n\
                           - gck ( g Z ) ( f CK ) ;\n",
                    inputs: &["clk", "en"],
                    outputs: &[],
                },
            ),
            &[("AND2_X1", "GATE")],
        )
    }

    /// An and gate timed from both its inputs.
    const GATE: &str = "cell (GATE) { area : 1.0 ;
        pin (A) { direction : input ; capacitance : 1.0 ; }
        pin (B) { direction : input ; capacitance : 1.0 ; }
        pin (Z) { direction : output ; function : \"(A & B)\" ;
          timing () { related_pin : \"A\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.05\") ; }
            cell_fall (scalar) { values (\"0.05\") ; } }
          timing () { related_pin : \"B\" ; timing_sense : positive_unate ;
            cell_rise (scalar) { values (\"0.05\") ; }
            cell_fall (scalar) { values (\"0.05\") ; } } } }";

    #[test]
    fn a_gated_clock_enable_is_checked_against_its_clock() {
        let db = gated_clock_design();
        let enable = cell_pin(&db, "g/B");
        let constraints = "create_clock -name clk -period 2.2 [get_ports clk]\n\
                           set_input_delay 0.3 -clock clk [get_ports en]\n";

        let (setup, hold, ends) = timed_with(&db, &cells_with(GATE), constraints, |_, sta| {
            (
                sta.slack(enable, MinMax::Max),
                sta.slack(enable, MinMax::Min),
                sta.endpoints().contains(&enable),
            )
        });

        assert!(ends);
        assert!(
            setup.time().is_some() && hold.time().is_some(),
            "{setup:?} {hold:?}"
        );
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
