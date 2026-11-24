//! Required times, found backward from the checks, false paths and multicycle paths applied.

use super::{
    Accounting, Check, CheckedEnd, CheckedInput, Entry, LatchState, REQUIRED_INIT,
    REQUIRED_UNREACHED, Sta, Tag, TagId, TightestRequired, cycle_accounting, differ, is_isolated,
    opposite, predecessors, tighter, unconstrained,
};
use crate::gated::GatedEnable;
use crate::graph::TimingGraph;
use crate::netlist::Netlist;
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{CellId, IdVec, IoPinId, PinId};
use pare_common::db::liberty::{ArcRole, ArcSetId, Edge, MinMax, SideValues, TimingArcSet};
use pare_common::db::sdc::{ClockEdge, ClockId, MultiCyclePath, PathEnd};
use pare_common::units::Seconds;
use std::collections::BinaryHeap;

impl Sta {
    /// Finds required times again from `seeds` back, following changes
    /// through the fanin, as `Search::findRequireds` with its
    /// `RequiredVisitor`.
    pub(super) fn find_requireds(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        seeds: &[PinId],
    ) {
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
    pub(super) fn cell_derates(&self, netlist: &Netlist<'_>, driver: PinId) -> SideValues<f64> {
        let cell = match netlist.db.pins[driver].owner {
            PinOwner::Cell { cell, .. } => Some(cell),
            PinOwner::Io(_) => None,
        };
        SideValues::from_fn(|side| self.sdc.cell_derate(cell, side))
    }

    /// The required times a pin's fanout imposes for one tag, as
    /// `RequiredVisitor::visitFromToPath` brings them back.
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
    /// the tightest over its check arcs or output delays, per side, as
    /// `VisitPathEnds::visitCheckEnd` and `PathEndCheck::requiredTime`.
    pub(super) fn check_requireds(
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
        let end = CheckedEnd {
            pin,
            edge,
            entry,
            tag,
            launch,
        };
        let mut required = TightestRequired::default();
        match netlist.db.pins[pin].owner {
            PinOwner::Io(io) => self.output_requireds(end, io, &mut required),
            PinOwner::Cell { cell, .. } => {
                self.cell_check_requireds(netlist, graph, end, cell, &mut required);
            }
        }
        required.0
    }

    /// Tightens `required` by the output delays set on port `io`.
    fn output_requireds(
        &mut self,
        end: CheckedEnd<'_>,
        io: IoPinId,
        required: &mut TightestRequired,
    ) {
        let delays = self.sdc.output_delays.get(&io).cloned().unwrap_or_default();
        for delay in delays {
            let target = delay.reference;
            for side in MinMax::BOTH {
                let Some(margin) = delay.delays.get(end.edge, side) else {
                    continue;
                };
                let to = PathEnd::Port(io);
                if self.is_false(end.tag, to, Some(target.clock), side) {
                    continue;
                }
                let base = self.target_arrival(end.tag, end.launch, target, to, side);
                required.tighten(side, base - margin);
            }
        }
    }

    /// Tightens `required` by the gated-clock check and the check arcs at
    /// a pin of `cell`.
    fn cell_check_requireds(
        &mut self,
        netlist: &Netlist<'_>,
        graph: &TimingGraph,
        end: CheckedEnd<'_>,
        cell: CellId,
        required: &mut TightestRequired,
    ) {
        let Some((lib, port)) = netlist.link.pin_port(netlist.db, end.pin) else {
            return;
        };
        if let Some(&gate) = self.gated.get(&end.pin) {
            for (side, value) in self.gated_requireds(end.tag, end.launch, end.entry, gate) {
                required.tighten(side, value);
            }
        }
        let is_enabled_latch_data = self
            .latches
            .get(&cell)
            .is_some_and(|l| l.data == end.pin && l.state == LatchState::Enabled);
        let input = if is_enabled_latch_data {
            CheckedInput::EnabledLatch
        } else {
            CheckedInput::Register
        };
        let sets: Vec<(ArcSetId, PinId)> = netlist
            .link
            .liberty
            .cell(lib)
            .arc_sets
            .iter_enumerated()
            .filter(|(_, set)| set.to == port && set.role.check_side().is_some())
            .filter_map(|(k, set)| netlist.pin_of_port(cell, set.from).map(|ck| (k, ck)))
            .collect();
        for (k, clock_pin) in sets {
            let set = &netlist.link.liberty.cell(lib).arc_sets[k];
            self.arc_set_requireds(graph, end, set, clock_pin, input, required);
        }
    }

    /// Tightens `required` by the arcs of check group `set`, clocked at
    /// `clock_pin`, that end on the end's edge.
    fn arc_set_requireds(
        &mut self,
        graph: &TimingGraph,
        end: CheckedEnd<'_>,
        set: &TimingArcSet,
        clock_pin: PinId,
        input: CheckedInput,
        required: &mut TightestRequired,
    ) {
        let Some(side) = set.role.check_side() else {
            return;
        };
        let Some(arrival) = end.entry.arrival[side] else {
            return;
        };
        let data_slew = graph.slew_side(end.pin, side).get(end.edge);
        let is_latch_setup = input == CheckedInput::EnabledLatch && set.role == ArcRole::Setup;
        for arc in set.arcs.iter().filter(|arc| arc.to_edge == end.edge) {
            let clock_slew = self.clock_slew(graph, clock_pin, arc.from_edge, side);
            let margin = arc.check_value(clock_slew, data_slew);
            let clock_edge = if is_latch_setup {
                opposite(arc.from_edge)
            } else {
                arc.from_edge
            };
            for capture in self.captures(clock_pin, clock_edge) {
                let target = capture.edge;
                let to = PathEnd::Clock(target.clock);
                if self.is_false(end.tag, to, Some(target.clock), side) {
                    continue;
                }
                let value = if is_latch_setup {
                    self.latch_timing(end.tag, end.launch, capture, to, arrival, margin)
                        .required
                } else {
                    let base = self.target_arrival(end.tag, end.launch, target, to, side)
                        + capture.latency(side);
                    match side {
                        MinMax::Max => base - margin,
                        MinMax::Min => base + margin,
                    }
                };
                required.tighten(side, value);
            }
        }
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
    pub(super) fn is_false(
        &self,
        tag: Tag,
        to: PathEnd,
        to_clock: Option<ClockId>,
        side: MinMax,
    ) -> bool {
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
    pub(super) fn multicycle(
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
    pub(super) fn multicycle_adjustment(
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
    pub(super) fn accounting(&mut self, launch: ClockEdge, target: ClockEdge) -> Accounting {
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
}
