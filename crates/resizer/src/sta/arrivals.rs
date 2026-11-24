//! Arrival times, found forward from the clocks and constrained inputs, latches included.

use super::{
    Capture, Entry, Latch, LatchData, LatchState, LatchTiming, Prev, REQUIRED_INIT, Sta, Stored,
    Tag, TagId, merge_arrival, opposite, same_time, successors,
};
use crate::fuzzy;
use crate::graph::TimingGraph;
use crate::netlist::{ClockNetwork, Constants, Netlist};
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{CellId, IdVec, IoPinId, PinId};
use pare_common::db::liberty::{
    ArcRole, ArcSetId, Edge, FuncExpr, Logic, MinMax, SequentialKind, SideValues,
};
use pare_common::db::sdc::{ClockEdge, PathEnd};
use pare_common::units::Seconds;
use rustc_hash::FxHashMap;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

impl Sta {
    /// Whether the arc of `set` into `pin` leaves a register or an open
    /// latch, where `OpenSTA`'s expanded path starts.
    pub(super) fn starts_data(&self, netlist: &Netlist<'_>, pin: PinId, set: ArcSetId) -> bool {
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
    /// the fanout, as `Search::findArrivals` with its `ArrivalVisitor`; returns the pins the required search must visit: those
    /// left with an entry that has no required time yet, and latch inputs
    /// whose arrival moved, since a borrowing latch requires its data when
    /// it arrives.
    ///
    /// Stale levels after edits can time a load before its new driver, so
    /// a pin may lose its entries and get them back within one search,
    /// anywhere downstream of an edit.
    pub(super) fn find_arrivals(
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
    pub(super) fn captures(&self, pin: PinId, edge: Edge) -> Vec<Capture> {
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
    pub(super) fn clock_slew(
        &self,
        graph: &TimingGraph,
        pin: PinId,
        edge: Edge,
        side: MinMax,
    ) -> Seconds {
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
                        set: Some(arc.arc.set),
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
    pub(super) fn latch_timing(
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
}

/// The latches of the design, as `Latches::latchDtoQEnable` reads them:
/// cells with a level-sensitive group whose enable is one pin, open on its
/// high level or, negated, its low level; a constant enable holds the
/// latch open or closed.
pub(super) fn find_latches(
    netlist: &Netlist<'_>,
    constants: &Constants,
) -> FxHashMap<CellId, Latch> {
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
