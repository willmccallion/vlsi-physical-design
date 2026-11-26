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
        prev.extend(graph.drivers(net).iter().copied().filter(|&d| d != pin));
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
mod tests;
