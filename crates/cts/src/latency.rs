//! Balancing the latencies of the trees under one clock, as `TritonCTS`'s
//! `LatencyBalancer`: where a clock reaches the trees of its macros, its
//! gated nets and its registers along paths of different delay, chains of
//! root buffers are put in front of the faster ones.

// The arithmetic keeps TritonCTS's operations, whose rounding fusing would change.
#![allow(clippy::suboptimal_flops)]

use crate::CtsError;
use crate::htree::{Point, TreeContext};
use crate::triton::{ClockTrees, avg_xy, first_output, is_sink, net_cell_pins, pin_direction};
use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{MacroPinId, NetId, PinId};
use pare_common::db::liberty::{ClockGatePin, Edge, SequentialKind};
use pare_common::db::library::PinDirection;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point as DbPoint;
use pare_resizer::Link;
use std::collections::BTreeMap;

/// A pin of the clock network the balancer reasons about.
#[derive(Clone, Debug)]
struct Node {
    children: Vec<usize>,
    arrival: f64,
    buffers: i32,
    input: Option<PinId>,
}

/// Puts delay buffers in front of the faster trees of every clock that
/// drives more than one tree; `arrival` gives the late arrival of a clock's
/// rising edge at a pin. Returns how many buffers were inserted.
pub fn balance_latencies(
    db: &mut NetlistDB,
    link: &Link<'_>,
    trees: &ClockTrees,
    arrival: &dyn Fn(PinId) -> Option<f64>,
) -> Result<usize, CtsError> {
    if !trees.settings.insertion_delay {
        return Ok(0);
    }
    let mut inserted = 0;
    for root in (0..trees.builders.len()).rev() {
        let builder = &trees.builders[root];
        if builder.parent.is_some() || builder.children.is_empty() {
            continue;
        }
        let mut balancer = Balancer {
            link,
            trees,
            root,
            arrival,
            nodes: Vec::new(),
            leaf_builders: BTreeMap::new(),
            worse_delay: f32::MIN_POSITIVE,
            buffer_delay: 0.0,
            index: 0,
        };
        inserted += balancer.run(db)?;
    }
    Ok(inserted)
}

struct Balancer<'a> {
    link: &'a Link<'a>,
    trees: &'a ClockTrees,
    root: usize,
    arrival: &'a dyn Fn(PinId) -> Option<f64>,
    nodes: Vec<Node>,
    leaf_builders: BTreeMap<String, usize>,
    worse_delay: f32,
    buffer_delay: f32,
    index: usize,
}

impl Balancer<'_> {
    fn run(&mut self, db: &mut NetlistDB) -> Result<usize, CtsError> {
        self.find_leaf_builders(self.root);
        let Some(input_net) = self.trees.builders[self.root].top_input_net else {
            return Ok(0);
        };
        self.build_graph(db, input_net);
        self.buffer_delay = self.buffer_delay(0.0);
        self.balance(db, 0)?;
        Ok(self.index)
    }

    fn find_leaf_builders(&mut self, builder: usize) {
        let b = &self.trees.builders[builder];
        if b.is_leaf_tree() {
            let _ = self
                .leaf_builders
                .insert(b.top_buffer_name.clone(), builder);
        }
        for &child in &b.children {
            self.find_leaf_builders(child);
        }
    }

    /// Whether the clock passes through an input, as
    /// `LatencyBalancer::propagateClock`: through buffers, inverters and
    /// combinational cells, and through a sequential cell only at a clock
    /// gate's clock or a latch's data.
    fn propagates(&self, db: &NetlistDB, pin: PinId) -> bool {
        let Some((cell, _, _)) = pin_direction(db, pin) else {
            return false;
        };
        let Some((_, lib)) = self.link.cell(db, cell) else {
            return false;
        };
        if lib.is_inverter() || lib.is_buffer() || !lib.is_sequential() {
            return true;
        }
        let Some((_, port)) = self.link.pin_port(db, pin) else {
            return false;
        };
        let p = &lib.ports[port];
        p.clock_gate == ClockGatePin::Clock
            || lib.sequentials.iter().any(|s| {
                s.kind == SequentialKind::Latch
                    && s.data
                        .as_ref()
                        .is_some_and(|d| d.ports().contains(&p.name.as_str()))
            })
    }

    fn sink_arrival(&self, pin: PinId) -> f32 {
        (self.arrival)(pin).unwrap_or(0.0) as f32
    }

    /// The mean arrival at a tree's sinks, as
    /// `LatencyBalancer::computeAveSinkArrivals`.
    fn average_sink_arrival(&self, db: &NetlistDB, builder: usize) -> f32 {
        let mut sum = 0.0_f32;
        let mut count = 0_u32;
        for sink in &self.trees.builders[builder].clock.sinks {
            if let Some(pin) = sink.input_pin {
                self.sink_arrivals(db, pin, &mut sum, &mut count);
            }
        }
        if count == 0 { 0.0 } else { sum / count as f32 }
    }

    fn sink_arrivals(&self, db: &NetlistDB, pin: PinId, sum: &mut f32, count: &mut u32) {
        if is_sink(self.link, db, pin) {
            *sum += self.sink_arrival(pin);
            *count += 1;
            return;
        }
        let Some((cell, _, _)) = pin_direction(db, pin) else {
            return;
        };
        let Some(out) = first_output(db, cell) else {
            return;
        };
        if !self.propagates(db, pin) {
            return;
        }
        let net = db.pins[out.index()].net;
        for load in net_cell_pins(db, net) {
            if pin_direction(db, load).is_some_and(|(_, d, _)| d == PinDirection::Input) {
                self.sink_arrivals(db, load, sum, count);
            }
        }
    }

    /// The clock network from the clock's input down to the sinks and the
    /// leaf trees, as `LatencyBalancer::buildGraph`.
    fn build_graph(&mut self, db: &NetlistDB, input_net: NetId) {
        let root_output = net_cell_pins(db, input_net)
            .into_iter()
            .find(|&p| pin_direction(db, p).is_some_and(|(_, d, _)| d == PinDirection::Output));
        self.nodes.push(Node {
            children: Vec::new(),
            arrival: 0.0,
            buffers: -1,
            input: root_output,
        });
        let mut stack = vec![0];
        while let Some(driver) = stack.pop() {
            let net = match self.nodes[driver].input {
                Some(input) => {
                    let Some((cell, _, _)) = pin_direction(db, input) else {
                        continue;
                    };
                    let Some(out) = first_output(db, cell) else {
                        continue;
                    };
                    db.pins[out.index()].net
                }
                None => input_net,
            };
            for pin in net_cell_pins(db, net) {
                let Some((cell, direction, _)) = pin_direction(db, pin) else {
                    continue;
                };
                if direction != PinDirection::Input {
                    continue;
                }
                let sink = is_sink(self.link, db, pin);
                if !sink && !self.propagates(db, pin) {
                    continue;
                }
                let id = self.nodes.len();
                self.nodes.push(Node {
                    children: Vec::new(),
                    arrival: 0.0,
                    buffers: -1,
                    input: Some(pin),
                });
                self.nodes[driver].children.push(id);
                let name = &db.cells[cell.index()].name;
                if let Some(&builder) = self.leaf_builders.get(name) {
                    let average = self.average_sink_arrival(db, builder);
                    self.worse_delay = self.worse_delay.max(average);
                    self.nodes[id].arrival = f64::from(average);
                    continue;
                }
                if sink {
                    let at = self.sink_arrival(pin);
                    self.worse_delay = self.worse_delay.max(at);
                    self.nodes[id].arrival = f64::from(at);
                    continue;
                }
                stack.push(id);
            }
        }
    }

    /// The rising delay of the root buffer driving another of its kind and
    /// `extra` farads, its slew settled over three lookups, as
    /// `LatencyBalancer::computeBufferDelay`.
    fn buffer_delay(&self, extra: f64) -> f32 {
        let lib = self.link.liberty.cell(self.trees.buffers.root);
        let Some((input, output)) = lib.buffer_ports() else {
            return 0.0;
        };
        let load = lib.ports[input].capacitance.rise + extra;
        let mut worst = 0.0_f32;
        for set in lib.delay_arc_sets_to(output).filter(|s| s.from == input) {
            for arc in set
                .arcs
                .iter()
                .filter(|a| a.from_edge == Edge::Rise && a.to_edge == Edge::Rise)
            {
                let (Some(delay), Some(slew)) = (&arc.delay, &arc.slew) else {
                    continue;
                };
                let mut in_slew = 0.0_f32;
                let mut d = 0.0_f32;
                for _ in 0..3 {
                    d = delay.lookup(f64::from(in_slew), load) as f32;
                    in_slew = slew.lookup(f64::from(in_slew), load) as f32;
                }
                worst = worst.max(d);
            }
        }
        worst
    }

    fn source_position(&self, db: &NetlistDB, node: usize) -> (i64, i64) {
        if let Some(pin) = self.nodes[node].input {
            return avg_xy(db, pin);
        }
        let net = self.trees.builders[self.root].top_input_net;
        net.and_then(|n| {
            db.nets[n.index()]
                .pins
                .iter()
                .copied()
                .find(|&p| matches!(db.pins[p.index()].owner, PinOwner::Io(_)))
        })
        .map_or((0, 0), |p| {
            let c = db.pin_position(p);
            (c.x.get(), c.y.get())
        })
    }

    /// How many delay buffers a node needs to reach the slowest arrival,
    /// as `LatencyBalancer::computeNumberOfDelayBuffers`.
    fn delay_buffers_needed(&mut self, db: &NetlistDB, node: usize, src: (i64, i64)) {
        if self.nodes[node].arrival == 0.0 {
            return;
        }
        let gap = f64::from(self.worse_delay) - self.nodes[node].arrival;
        let first = (gap / f64::from(self.buffer_delay)) as i32;
        let Some(input) = self.nodes[node].input else {
            return;
        };
        let sink = avg_xy(db, input);
        let dx = (sink.0 - src.0) as f32 / (first + 1) as f32;
        let dy = (sink.1 - src.1) as f32 / (first + 1) as f32;
        let delay = self.buffer_delay(f64::from(dx.abs() + dy.abs()) * self.trees.cap_per_dbu);
        self.nodes[node].buffers = (gap / f64::from(delay)) as i32;
    }

    /// Balances a node's children bottom up, as
    /// `LatencyBalancer::balanceLatencies`.
    fn balance(&mut self, db: &mut NetlistDB, node: usize) -> Result<(), CtsError> {
        if self.nodes[node].children.is_empty() {
            return Ok(());
        }
        let src = self.source_position(db, node);
        let mut max_arrival = f64::MIN_POSITIVE;
        let mut by_need: BTreeMap<i32, Vec<PinId>> = BTreeMap::new();
        for child in self.nodes[node].children.clone() {
            self.balance(db, child)?;
            self.delay_buffers_needed(db, child, src);
            max_arrival = max_arrival.max(self.nodes[child].arrival);
            if let Some(input) = self.nodes[child].input {
                by_need
                    .entry(self.nodes[child].buffers)
                    .or_default()
                    .push(input);
            }
        }
        let mut previous = 0;
        let mut loads: Vec<PinId> = Vec::new();
        for (need, children) in by_need.into_iter().rev() {
            if need == -1 {
                continue;
            }
            if previous == 0 {
                previous = need;
                loads = children;
                continue;
            }
            let first = self.insert_delay_buffers(db, previous - need, src, &loads)?;
            loads = children;
            loads.extend(first);
            previous = need;
        }
        self.nodes[node].buffers = previous;
        self.nodes[node].arrival = max_arrival;
        Ok(())
    }

    /// A chain of `count` root buffers from the source toward the middle of
    /// `loads`, driving them, as `LatencyBalancer::insertDelayBuffers`;
    /// returns the first buffer's input.
    fn insert_delay_buffers(
        &mut self,
        db: &mut NetlistDB,
        count: i32,
        src: (i64, i64),
        loads: &[PinId],
    ) -> Result<Option<PinId>, CtsError> {
        let Some(&first_load) = loads.first() else {
            return Ok(None);
        };
        let mut driving = db.pins[first_load.index()].net;
        let (mut lo, mut hi) = ((i64::MAX, i64::MAX), (i64::MIN, i64::MIN));
        for &load in loads {
            let p = avg_xy(db, load);
            lo = (lo.0.min(p.0), lo.1.min(p.1));
            hi = (hi.0.max(p.0), hi.1.max(p.1));
        }
        let center = (i64::midpoint(lo.0, hi.0), i64::midpoint(lo.1, hi.1));
        let dx = (center.0 - src.0) as f32 / (count + 1) as f32;
        let dy = (center.1 - src.1) as f32 / (count + 1) as f32;
        let root = &self.trees.builders[self.root];
        let ctx = TreeContext {
            settings: &self.trees.settings,
            core: db.core_area(),
            dbu: db.dbu().get() as i32,
            root_buffer: self.trees.buffers.root,
            sink_buffer: self.trees.buffers.sink,
            buffer_sizes: &self.trees.buffer_sizes,
            blockages: &self.trees.blockages,
        };
        let unit = root.unit();
        let buffer = self.trees.buffers.root;
        let lib = self.link.liberty.cell(buffer);
        let (in_port, out_port) = lib
            .buffer_ports()
            .ok_or_else(|| CtsError::NotABuffer(lib.name.clone()))?;
        let pin_of = |port: usize| {
            self.link
                .macro_pin_of_port(buffer, port)
                .map(MacroPinId::new)
                .ok_or_else(|| CtsError::NotABuffer(lib.name.clone()))
        };
        let (in_pin, out_pin) = (pin_of(in_port)?, pin_of(out_port)?);
        let master = self
            .link
            .macro_of(buffer)
            .ok_or_else(|| CtsError::NoMaster(lib.name.clone()))?;
        let clock_name = root.clock.sdc_name.clone();
        let mut first_input_pin = None;
        for i in 0..count {
            let step = (i + 1) as f32;
            let at = Point::new(
                f64::from(src.0 as f32 + dx * step) / unit,
                f64::from(src.1 as f32 + dy * step) / unit,
            );
            let legal = root.legalize_one_buffer(&ctx, at, buffer);
            let location = DbPoint::new(
                Dbu(i64::from((legal.x * unit) as i32)),
                Dbu(i64::from((legal.y * unit) as i32)),
            );
            let name = format!("delaybuf_{}_{clock_name}", self.index);
            let net_name = format!("delaynet_{}_{clock_name}", self.index);
            self.index += 1;
            let cell = db
                .add_cell(
                    name,
                    master,
                    location,
                    Orientation::N,
                    PlacementStatus::Placed,
                )
                .map_err(|e| CtsError::Edit(e.to_string()))?;
            let net = db
                .add_net(&net_name)
                .map_err(|e| CtsError::Edit(e.to_string()))?;
            db.nets[net.index()].signal_use = db.nets[driving.index()].signal_use;
            db.nets[net.index()].non_default_rule = db.nets[driving.index()].non_default_rule;
            let input = db.connect_cell_pin(cell, in_pin, driving);
            let _ = db.connect_cell_pin(cell, out_pin, net);
            for &load in loads {
                db.move_pin(load, net);
            }
            driving = net;
            let _ = first_input_pin.get_or_insert(input);
        }
        Ok(first_input_pin)
    }
}
