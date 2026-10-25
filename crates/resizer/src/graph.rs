//! The timing graph repair reads slews and delays from.
//!
//! Every pin is a vertex; wires and the cells' delay arcs are its edges,
//! levelized from the inputs so drivers can be visited in order. Each pin's
//! slew, each arc's delay and each wire's delay are found as `OpenSTA`'s
//! delay calculation finds them from the estimated parasitics.

use crate::dmp::{self, PiModel};
use crate::netlist::{ClockNetwork, Constants, Netlist};
use crate::parasitics::{NetParasitics, PiElmore, WireRc};
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{CellId, NetId, PinId};
use pare_common::db::liberty::{
    ArcRole, Edge, EdgeValues, FuncExpr, LibCell, MinMax, SequentialKind, TimingArcSet,
};
use std::collections::{BinaryHeap, VecDeque};

/// The slew `OpenSTA` seeds a root with: an input port without a declared
/// transition, or an undriven pin.
const ROOT_SLEW: f64 = 0.0;

/// An arc into a driver: from which pin of its cell, through which set.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InArc {
    /// The input pin the arc starts at.
    pub from: PinId,
    /// Index of the timing group in the cell's arc sets.
    pub set: usize,
    /// Whether the arc is a latch's data input to its output, which
    /// levelization does not follow, as `OpenSTA`'s does not: latches are
    /// where its loops break.
    pub through_latch: bool,
}

/// The delay of one edge pair of an arc into a driver, at the driver's
/// load, as `OpenSTA`'s graph holds arc delays.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArcDelay {
    /// The arc.
    pub arc: InArc,
    /// The edge at the arc's input.
    pub from_edge: Edge,
    /// The edge at the driver.
    pub to_edge: Edge,
    /// The delay for each side, in seconds: from the early input slew on
    /// the early side, the late one on the late side.
    pub delay: [f64; 2],
}

/// Slews and delays over the design, kept current as repair edits it.
///
/// Each is held for both sides of the delay range, as `OpenSTA` keeps one
/// delay calculation per side: the early side takes the smallest slew and
/// delay over a driver's arcs, the late side the largest.
#[derive(Clone, Debug)]
pub struct TimingGraph {
    level: Vec<u32>,
    in_arcs: Vec<Vec<InArc>>,
    loop_arcs: rustc_hash::FxHashSet<(PinId, PinId)>,
    slew: Vec<[EdgeValues; 2]>,
    annotated: Vec<Option<f64>>,
    arc_delays: Vec<Vec<ArcDelay>>,
    wire_delay: Vec<[EdgeValues; 2]>,
    parasitics: Vec<Option<NetParasitics>>,
    rc: WireRc,
}

impl TimingGraph {
    /// Levelizes `netlist` and finds every pin's slew.
    #[must_use]
    pub fn new(
        netlist: &Netlist<'_>,
        clocks: &ClockNetwork,
        constants: &Constants,
        rc: WireRc,
    ) -> Self {
        let db = netlist.db;
        let in_arcs = in_arcs(netlist, constants);
        let (level, loop_arcs) = levelize(netlist, &in_arcs);
        let parasitics = (0..db.nets.len())
            .map(|n| estimate_net(netlist, clocks, NetId::new(n), &rc))
            .collect();
        let mut graph = Self {
            level,
            in_arcs,
            loop_arcs,
            slew: vec![[EdgeValues::default(); 2]; db.pins.len()],
            annotated: vec![None; db.pins.len()],
            arc_delays: vec![Vec::new(); db.pins.len()],
            wire_delay: vec![[EdgeValues::default(); 2]; db.pins.len()],
            parasitics,
            rc,
        };
        graph.find_all_slews(netlist, clocks);
        graph
    }

    /// The arcs timing searches into a driver.
    #[must_use]
    pub fn in_arcs(&self, driver: PinId) -> &[InArc] {
        self.in_arcs.get(driver.index()).map_or(&[], Vec::as_slice)
    }

    /// Whether the arc from `from` to `to` closes a combinational loop,
    /// which timing does not search through, as `OpenSTA` disables loop
    /// edges.
    #[must_use]
    pub fn is_loop_arc(&self, from: PinId, to: PinId) -> bool {
        self.loop_arcs.contains(&(from, to))
    }

    /// The delays of the arcs into a driver at its present load.
    #[must_use]
    pub fn arc_delays(&self, driver: PinId) -> &[ArcDelay] {
        self.arc_delays
            .get(driver.index())
            .map_or(&[], Vec::as_slice)
    }

    /// The delay from a load's driver to the load for `edge` on `side`:
    /// the largest over the driver's arcs on the late side, the smallest
    /// on the early side, as `OpenSTA` annotates a wire.
    #[must_use]
    pub fn wire_delay(&self, load: PinId, edge: Edge, side: MinMax) -> f64 {
        self.wire_delay
            .get(load.index())
            .map_or(0.0, |d| d[side.index()].get(edge))
    }

    /// The pins driving nets, by rising level and then name, as `OpenROAD`
    /// orders the drivers it repairs.
    #[must_use]
    pub fn drivers_by_level(&self, netlist: &Netlist<'_>) -> Vec<PinId> {
        let mut drivers: Vec<PinId> = (0..netlist.db.pins.len())
            .map(PinId::new)
            .filter(|&p| netlist.is_driver(p))
            .collect();
        drivers.sort_by(|&a, &b| {
            self.level[a.index()]
                .cmp(&self.level[b.index()])
                .then_with(|| netlist.db.pin_name(a).cmp(&netlist.db.pin_name(b)))
        });
        drivers
    }

    /// The instance input pins by rising level and then name.
    #[must_use]
    pub fn loads_by_level(&self, netlist: &Netlist<'_>) -> Vec<PinId> {
        let mut loads: Vec<PinId> = (0..netlist.db.pins.len())
            .map(PinId::new)
            .filter(|&p| {
                matches!(netlist.db.pins[p.index()].owner, PinOwner::Cell { .. })
                    && netlist.is_load(p)
            })
            .collect();
        loads.sort_by(|&a, &b| {
            self.level[a.index()]
                .cmp(&self.level[b.index()])
                .then_with(|| netlist.db.pin_name(a).cmp(&netlist.db.pin_name(b)))
        });
        loads
    }

    /// A pin's level.
    #[must_use]
    pub fn level(&self, pin: PinId) -> u32 {
        self.level[pin.index()]
    }

    /// A pin's late-side slews, which the slew limits are checked against.
    #[must_use]
    pub fn slew(&self, pin: PinId) -> EdgeValues {
        self.slew_side(pin, MinMax::Max)
    }

    /// A pin's slews on `side`.
    #[must_use]
    pub fn slew_side(&self, pin: PinId, side: MinMax) -> EdgeValues {
        self.slew
            .get(pin.index())
            .map_or_else(EdgeValues::default, |s| s[side.index()])
    }

    /// Holds a pin's slew at `slew` for both edges and sides until cleared,
    /// as `OpenSTA`'s annotated slews.
    pub fn annotate_slew(&mut self, pin: PinId, slew: f64) {
        self.annotated[pin.index()] = Some(slew);
        let both = EdgeValues {
            rise: slew,
            fall: slew,
        };
        self.slew[pin.index()] = [both, both];
    }

    /// Drops every annotation.
    pub fn clear_annotations(&mut self) {
        self.annotated.fill(None);
    }

    /// The estimated parasitics of a net, if it has any.
    #[must_use]
    pub fn parasitics(&self, net: NetId) -> Option<&NetParasitics> {
        self.parasitics[net.index()].as_ref()
    }

    /// The wire resistance and capacitance per length the estimates use.
    #[must_use]
    pub const fn wire_rc(&self) -> WireRc {
        self.rc
    }

    /// The pi model and Elmore delays `driver` sees for `edge`.
    #[must_use]
    pub fn pi_elmore(&self, netlist: &Netlist<'_>, driver: PinId, edge: Edge) -> Option<PiElmore> {
        let net = netlist.db.pins[driver.index()].net;
        self.parasitics[net.index()].as_ref().and_then(|p| {
            p.pi_elmore(netlist, driver, edge, &|pin, e| {
                pin_capacitance(netlist, pin, e)
            })
        })
    }

    /// The load a driver sees: its loads' pins and the wire, the larger of
    /// the two edges, as `OpenSTA`'s `loadCap`.
    #[must_use]
    pub fn load_cap(&self, netlist: &Netlist<'_>, driver: PinId) -> f64 {
        Edge::BOTH
            .iter()
            .map(|&edge| self.load_cap_for(netlist, driver, edge))
            .fold(0.0, f64::max)
    }

    fn load_cap_for(&self, netlist: &Netlist<'_>, driver: PinId, edge: Edge) -> f64 {
        self.pi_elmore(netlist, driver, edge).map_or_else(
            || {
                let net = netlist.db.pins[driver.index()].net;
                netlist
                    .loads(net)
                    .map(|p| pin_capacitance(netlist, p, edge))
                    .sum()
            },
            |pi| pi.load_cap(),
        )
    }

    /// Re-estimates a net's parasitics after its pins or their positions
    /// changed, leaving the slews that depend on them to a later
    /// propagation from the net's drivers.
    pub fn parasitics_invalid(&mut self, netlist: &Netlist<'_>, clocks: &ClockNetwork, net: NetId) {
        if net.index() >= self.parasitics.len() || netlist.db.pins.len() > self.slew.len() {
            self.grow(netlist);
        }
        self.parasitics[net.index()] = estimate_net(netlist, clocks, net, &self.rc);
    }

    /// Re-estimates a net's parasitics after its pins or their positions
    /// changed, and finds the slews and delays that depend on them again;
    /// returns every driver whose delays were found again.
    pub fn update_net(
        &mut self,
        netlist: &Netlist<'_>,
        clocks: &ClockNetwork,
        net: NetId,
    ) -> Vec<PinId> {
        if net.index() >= self.parasitics.len() || netlist.db.pins.len() > self.slew.len() {
            self.grow(netlist);
        }
        self.parasitics[net.index()] = estimate_net(netlist, clocks, net, &self.rc);
        let drivers: Vec<PinId> = netlist.drivers(net).collect();
        self.propagate_from(netlist, clocks, &drivers)
    }

    /// Finds the slews and delays again from `drivers` onward, following
    /// changes through the fanout; returns every driver visited, in rising
    /// level order.
    pub fn propagate_from(
        &mut self,
        netlist: &Netlist<'_>,
        clocks: &ClockNetwork,
        drivers: &[PinId],
    ) -> Vec<PinId> {
        let mut queue: BinaryHeap<std::cmp::Reverse<(u32, usize)>> = drivers
            .iter()
            .map(|&d| std::cmp::Reverse((self.level[d.index()], d.index())))
            .collect();
        let mut seen = vec![false; netlist.db.pins.len()];
        let mut visited = Vec::new();
        while let Some(std::cmp::Reverse((_, index))) = queue.pop() {
            if seen[index] {
                continue;
            }
            seen[index] = true;
            let driver = PinId::new(index);
            visited.push(driver);
            let changed = self.find_driver_slews(netlist, clocks, driver);
            for load in changed {
                for next in self.drivers_fed_by(netlist, load) {
                    if !seen[next.index()] {
                        queue.push(std::cmp::Reverse((self.level[next.index()], next.index())));
                    }
                }
            }
        }
        visited
    }

    /// Makes room for pins and nets added since the graph was built; new
    /// pins take the level of their cell's inputs' drivers plus one, which
    /// is where a repeater sits.
    pub fn grow(&mut self, netlist: &Netlist<'_>) {
        let db = netlist.db;
        let constants_free = |_: CellId, _: usize, _: usize| true;
        while self.in_arcs.len() < db.pins.len() {
            let pin = PinId::new(self.in_arcs.len());
            self.in_arcs.push(in_arcs_of(netlist, pin, &constants_free));
            self.slew.push([EdgeValues::default(); 2]);
            self.annotated.push(None);
            self.arc_delays.push(Vec::new());
            self.wire_delay.push([EdgeValues::default(); 2]);
            let level = self.in_arcs[pin.index()]
                .iter()
                .map(|a| self.level[a.from.index()] + 1)
                .max()
                .or_else(|| {
                    if netlist.is_load(pin) {
                        let net = db.pins[pin.index()].net;
                        netlist
                            .drivers(net)
                            .filter(|d| d.index() < self.level.len())
                            .map(|d| self.level[d.index()] + 1)
                            .max()
                    } else {
                        None
                    }
                })
                .unwrap_or(0);
            self.level.push(level);
        }
        while self.parasitics.len() < db.nets.len() {
            self.parasitics.push(None);
        }
    }

    /// Notes that `cell` changed master: its pins keep their ids, so only
    /// the slews and delays around it need finding again; returns every
    /// driver whose delays were found again.
    pub fn update_cell(
        &mut self,
        netlist: &Netlist<'_>,
        clocks: &ClockNetwork,
        cell: CellId,
    ) -> Vec<PinId> {
        let db = netlist.db;
        let mut drivers = Vec::new();
        for &pin in &db.cells[cell.index()].pins {
            if netlist.is_load(pin) {
                drivers.extend(netlist.drivers(db.pins[pin.index()].net));
            }
            if netlist.is_driver(pin) {
                drivers.push(pin);
            }
        }
        self.propagate_from(netlist, clocks, &drivers)
    }

    /// Finds every pin's level again after the netlist changed shape, as
    /// `OpenSTA` relevelizes once a search finds the ordering stale.
    pub fn relevelize(&mut self, netlist: &Netlist<'_>) {
        self.grow(netlist);
        let (level, loop_arcs) = levelize(netlist, &self.in_arcs);
        self.level = level;
        self.loop_arcs = loop_arcs;
    }

    /// The drivers of the cell `load` belongs to that an arc from `load`
    /// reaches.
    pub(crate) fn drivers_fed_by(&self, netlist: &Netlist<'_>, load: PinId) -> Vec<PinId> {
        let db = netlist.db;
        let PinOwner::Cell { cell, .. } = db.pins[load.index()].owner else {
            return Vec::new();
        };
        db.cells[cell.index()]
            .pins
            .iter()
            .copied()
            .filter(|&out| {
                netlist.is_driver(out) && self.in_arcs[out.index()].iter().any(|a| a.from == load)
            })
            .collect()
    }

    fn find_all_slews(&mut self, netlist: &Netlist<'_>, clocks: &ClockNetwork) {
        let mut order: Vec<PinId> = (0..netlist.db.pins.len())
            .map(PinId::new)
            .filter(|&p| netlist.is_driver(p))
            .collect();
        order.sort_by_key(|&p| self.level[p.index()]);
        for driver in order {
            let _ = self.find_driver_slews(netlist, clocks, driver);
        }
    }

    /// Finds a driver's slews and delays and its loads', for both sides, as
    /// `GraphDelayCalc`'s `findDriverDelays`; returns the loads whose slew
    /// changed on either side.
    fn find_driver_slews(
        &mut self,
        netlist: &Netlist<'_>,
        clocks: &ClockNetwork,
        driver: PinId,
    ) -> Vec<PinId> {
        let db = netlist.db;
        let net = db.pins[driver.index()].net;
        let loads: Vec<PinId> = netlist.loads(net).filter(|&p| p != driver).collect();
        let mut drvr_slew = [Merged::new(MinMax::Min), Merged::new(MinMax::Max)];
        let mut load_slews: Vec<[Merged; 2]> = vec![drvr_slew; loads.len()];
        let mut load_delays: Vec<[Merged; 2]> = vec![drvr_slew; loads.len()];
        let mut arc_delays = Vec::new();
        let arcs = self.in_arcs[driver.index()].clone();
        let cell_arcs: Option<(&LibCell, CellId)> = match db.pins[driver.index()].owner {
            PinOwner::Cell { cell, .. } => netlist.link.cell(db, cell).map(|(_, lib)| (lib, cell)),
            PinOwner::Io(_) => None,
        };
        let mut any_arc = false;
        if let Some((lib, _)) = cell_arcs {
            for arc in &arcs {
                let set: &TimingArcSet = &lib.arc_sets[arc.set];
                for timing_arc in &set.arcs {
                    any_arc = true;
                    let edge = timing_arc.to_edge;
                    let pi = self.pi_elmore(netlist, driver, edge);
                    let model = pi.as_ref().map_or_else(
                        || PiModel {
                            c2: self.load_cap_for(netlist, driver, edge),
                            rpi: 0.0,
                            c1: 0.0,
                        },
                        |p| PiModel {
                            c2: p.c2,
                            rpi: p.rpi,
                            c1: p.c1,
                        },
                    );
                    let mut delay = [0.0; 2];
                    for side in MinMax::BOTH {
                        let in_slew = if set.role == ArcRole::RegClkToQ && clocks.is_clock(arc.from)
                        {
                            0.0
                        } else {
                            self.slew[arc.from.index()][side.index()].get(timing_arc.from_edge)
                        };
                        let result = dmp::gate_delay(
                            timing_arc,
                            &netlist.link.liberty.thresholds,
                            in_slew,
                            model,
                        );
                        delay[side.index()] = result.delay;
                        drvr_slew[side.index()].take(edge, result.slew);
                        for (i, &load) in loads.iter().enumerate() {
                            let elmore = pi
                                .as_ref()
                                .and_then(|p| {
                                    p.elmore.iter().find(|(l, _)| *l == load).map(|(_, e)| *e)
                                })
                                .unwrap_or(0.0);
                            let (load_delay, load_slew) = result.load_delay_slew(elmore);
                            load_slews[i][side.index()].take(edge, load_slew);
                            load_delays[i][side.index()].take(edge, load_delay);
                        }
                    }
                    arc_delays.push(ArcDelay {
                        arc: *arc,
                        from_edge: timing_arc.from_edge,
                        to_edge: edge,
                        delay,
                    });
                }
            }
        }
        let root = EdgeValues {
            rise: ROOT_SLEW,
            fall: ROOT_SLEW,
        };
        let drvr_slew = if any_arc {
            drvr_slew.map(Merged::values)
        } else {
            [root, root]
        };
        if self.annotated[driver.index()].is_none() {
            self.slew[driver.index()] = drvr_slew;
        }
        self.arc_delays[driver.index()] = arc_delays;
        let mut changed = Vec::new();
        for (i, &load) in loads.iter().enumerate() {
            let slews = if any_arc {
                load_slews[i].map(Merged::values)
            } else {
                [root, root]
            };
            self.wire_delay[load.index()] = load_delays[i].map(Merged::values);
            if self.annotated[load.index()].is_some() {
                continue;
            }
            if self.slew[load.index()] != slews {
                self.slew[load.index()] = slews;
                changed.push(load);
            }
        }
        changed
    }
}

/// Values for both edges merged over a driver's arcs on one side: the
/// largest on the late side, the smallest on the early side; zero where
/// no arc gave one.
#[derive(Clone, Copy, Debug)]
struct Merged {
    side: MinMax,
    rise: f64,
    fall: f64,
}

impl Merged {
    const fn new(side: MinMax) -> Self {
        let init = side.init_value();
        Self {
            side,
            rise: init,
            fall: init,
        }
    }

    fn take(&mut self, edge: Edge, value: f64) {
        let slot = match edge {
            Edge::Rise => &mut self.rise,
            Edge::Fall => &mut self.fall,
        };
        if self.side.beats(value, *slot) {
            *slot = value;
        }
    }

    fn values(self) -> EdgeValues {
        let settle = |v: f64| if v.is_finite() { v } else { 0.0 };
        EdgeValues {
            rise: settle(self.rise),
            fall: settle(self.fall),
        }
    }
}

/// A load pin's capacitance for `edge`: its Liberty value, or none for a
/// design pin without a declared load.
#[must_use]
pub fn pin_capacitance(netlist: &Netlist<'_>, pin: PinId, edge: Edge) -> f64 {
    netlist
        .lib_port(pin)
        .map_or(0.0, |port| port.capacitance.get(edge))
}

/// The parasitics of a net, or none for a net of fewer than two pins or one
/// an ideal clock drives.
fn estimate_net(
    netlist: &Netlist<'_>,
    clocks: &ClockNetwork,
    net: NetId,
    rc: &WireRc,
) -> Option<NetParasitics> {
    if netlist.is_special(net) || netlist.drivers(net).any(|d| clocks.is_clock(d)) {
        return None;
    }
    NetParasitics::estimate(netlist, net, rc)
}

/// The arcs `OpenSTA` searches into each driver: the delay arcs it does not
/// disable, from the connected input pins of the driver's cell.
fn in_arcs(netlist: &Netlist<'_>, constants: &Constants) -> Vec<Vec<InArc>> {
    let enabled =
        |cell: CellId, from: usize, to: usize| constants.arc_enabled(netlist, cell, from, to);
    (0..netlist.db.pins.len())
        .map(|p| in_arcs_of(netlist, PinId::new(p), &enabled))
        .collect()
}

fn in_arcs_of(
    netlist: &Netlist<'_>,
    pin: PinId,
    enabled: &dyn Fn(CellId, usize, usize) -> bool,
) -> Vec<InArc> {
    let db = netlist.db;
    let PinOwner::Cell { cell, .. } = db.pins[pin.index()].owner else {
        return Vec::new();
    };
    if !netlist.is_driver(pin) {
        return Vec::new();
    }
    let Some((lib_id, to_port)) = netlist.link.pin_port(db, pin) else {
        return Vec::new();
    };
    let lib = netlist.link.liberty.cell(lib_id);
    let latch_data = lib
        .sequentials
        .iter()
        .find(|s| s.kind == SequentialKind::Latch)
        .and_then(|s| s.data.as_ref())
        .and_then(FuncExpr::as_port)
        .and_then(|name| lib.port(name));
    let mut arcs = Vec::new();
    for (set_index, set) in lib.arc_sets.iter().enumerate() {
        if set.to != to_port || !set.role.is_searched() || set.arcs.is_empty() {
            continue;
        }
        if !enabled(cell, set.from, set.to) {
            continue;
        }
        if let Some(from) = netlist.pin_of_port(cell, set.from) {
            arcs.push(InArc {
                from,
                set: set_index,
                through_latch: set.role == ArcRole::Combinational && latch_data == Some(set.from),
            });
        }
    }
    arcs
}

/// Each pin's level: the longest path of wires and searched arcs from a
/// root, loops broken where a search returns to a pin on its own path, as
/// `OpenSTA`'s `Levelize`; with the arcs that closed loops.
fn levelize(
    netlist: &Netlist<'_>,
    in_arcs: &[Vec<InArc>],
) -> (Vec<u32>, rustc_hash::FxHashSet<(PinId, PinId)>) {
    let db = netlist.db;
    let n = db.pins.len();
    let mut successors: Vec<Vec<usize>> = vec![Vec::new(); n];
    for pin in 0..n {
        let id = PinId::new(pin);
        if netlist.is_driver(id) {
            let net = db.pins[pin].net;
            for load in netlist.loads(net) {
                if load != id {
                    successors[pin].push(load.index());
                }
            }
        }
        for arc in &in_arcs[pin] {
            if !arc.through_latch {
                successors[arc.from.index()].push(pin);
            }
        }
    }
    for s in &mut successors {
        s.sort_unstable();
        s.dedup();
    }
    let mut predecessors = vec![0usize; n];
    for s in &successors {
        for &to in s {
            predecessors[to] += 1;
        }
    }
    let back_edges = find_back_edges(&successors, &predecessors);
    let mut in_degree = vec![0usize; n];
    for (from, s) in successors.iter().enumerate() {
        for &to in s {
            if !back_edges.contains(&(from, to)) {
                in_degree[to] += 1;
            }
        }
    }
    let mut level = vec![0u32; n];
    let mut queue: VecDeque<usize> = (0..n).filter(|&v| in_degree[v] == 0).collect();
    while let Some(v) = queue.pop_front() {
        for &to in &successors[v] {
            if back_edges.contains(&(v, to)) {
                continue;
            }
            level[to] = level[to].max(level[v] + 1);
            in_degree[to] -= 1;
            if in_degree[to] == 0 {
                queue.push_back(to);
            }
        }
    }
    let loop_arcs = back_edges
        .into_iter()
        .map(|(from, to)| (PinId::new(from), PinId::new(to)))
        .collect();
    (level, loop_arcs)
}

/// The edges a depth-first search from the roots follows back onto its own
/// path.
fn find_back_edges(
    successors: &[Vec<usize>],
    predecessors: &[usize],
) -> rustc_hash::FxHashSet<(usize, usize)> {
    let n = successors.len();
    let mut color = vec![0u8; n];
    let mut back = rustc_hash::FxHashSet::default();
    let roots = (0..n).filter(|&v| predecessors[v] == 0);
    let unrooted = 0..n;
    for start in roots.chain(unrooted) {
        if color[start] != 0 {
            continue;
        }
        let mut stack: Vec<(usize, usize)> = vec![(start, 0)];
        color[start] = 1;
        while let Some(&mut (v, ref mut next)) = stack.last_mut() {
            if *next < successors[v].len() {
                let to = successors[v][*next];
                *next += 1;
                match color[to] {
                    0 => {
                        color[to] = 1;
                        stack.push((to, 0));
                    }
                    1 => {
                        let _ = back.insert((v, to));
                    }
                    _ => {}
                }
            } else {
                color[v] = 2;
                let _ = stack.pop();
            }
        }
    }
    back
}
