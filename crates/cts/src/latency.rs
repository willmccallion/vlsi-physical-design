//! Balancing the latencies of the trees under one clock, as `TritonCTS`'s
//! `LatencyBalancer`: where a clock reaches the trees of its macros, its
//! gated nets and its registers along paths of different delay, chains of
//! root buffers are put in front of the faster ones.

// The arithmetic keeps TritonCTS's operations, whose rounding fusing would change.
#![allow(clippy::suboptimal_flops)]

use crate::CtsError;
use crate::htree::TreePoint;
use crate::triton::{
    ClockTrees, first_output, is_sink, liberty_insertion_delay, net_cell_pins, pin_direction,
    sink_bbox_center,
};
use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{NetId, PinId};
use pare_common::db::liberty::{
    BufferCell, ClockGatePin, Edge, LibPortId, Liberty, SequentialKind,
};
use pare_common::db::library::PinDirection;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::units::{Farads, Seconds};
use pare_resizer::Link;
use std::collections::BTreeMap;

/// A pin of the clock network the balancer reasons about.
#[derive(Clone, Debug)]
struct Node {
    children: Vec<usize>,
    /// When the clock reaches the node; none where timing gives no arrival.
    arrival: Option<f64>,
    /// The delay buffers the node needs, once balancing has counted them.
    buffers: Option<i32>,
    input: Option<PinId>,
}

/// Puts delay buffers in front of the faster trees of every clock that
/// drives more than one tree, returning how many it inserted.
///
/// `arrival` gives the late arrival of a clock's rising edge at a pin.
pub fn balance_latencies(
    db: &mut NetlistDB,
    link: &Link<'_>,
    trees: &ClockTrees,
    arrival: &dyn Fn(PinId) -> Option<Seconds>,
) -> Result<usize, CtsError> {
    if !trees.setup.settings.insertion_delay {
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
    arrival: &'a dyn Fn(PinId) -> Option<Seconds>,
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
        self.buffer_delay = self.root_buffer_delay(Farads::ZERO)?;
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

    /// When the clock reaches the registers behind a sink pin: its arrival
    /// at the pin plus the clock tree inside the sink's cell, as
    /// `LatencyBalancer::buildGraph` and `computeSinkArrivalRecur` add them;
    /// none where timing gives the pin no arrival.
    fn sink_arrival(&self, db: &NetlistDB, pin: PinId) -> Option<f32> {
        // NOTE: getVertexClkArrival warns and takes 0 for a pin with no
        // clock path; PARE leaves such a sink out of the balance.
        let arrival = (self.arrival)(pin)?.get() as f32;
        Some(arrival + liberty_insertion_delay(self.link, db, pin).get() as f32)
    }

    /// The mean arrival at a tree's sinks that have one, as
    /// `LatencyBalancer::computeAveSinkArrivals`; none when none has.
    fn average_sink_arrival(&self, db: &NetlistDB, builder: usize) -> Option<f32> {
        let mut sum = 0.0_f32;
        let mut count = 0_u32;
        for sink in &self.trees.builders[builder].clock.sinks {
            if let Some(pin) = sink.input_pin {
                self.sink_arrivals(db, pin, &mut sum, &mut count);
            }
        }
        (count > 0).then(|| sum / count as f32)
    }

    fn sink_arrivals(&self, db: &NetlistDB, pin: PinId, sum: &mut f32, count: &mut u32) {
        if is_sink(self.link, db, pin) {
            if let Some(arrival) = self.sink_arrival(db, pin) {
                *sum += arrival;
                *count += 1;
            }
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
        let net = db.pins[out].net;
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
            arrival: None,
            buffers: None,
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
                    db.pins[out].net
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
                    arrival: None,
                    buffers: None,
                    input: Some(pin),
                });
                self.nodes[driver].children.push(id);
                let name = &db.cells[cell].name;
                let arrival = if let Some(&builder) = self.leaf_builders.get(name) {
                    self.average_sink_arrival(db, builder)
                } else if sink {
                    self.sink_arrival(db, pin)
                } else {
                    stack.push(id);
                    continue;
                };
                if let Some(at) = arrival {
                    self.worse_delay = self.worse_delay.max(at);
                }
                self.nodes[id].arrival = arrival.map(f64::from);
            }
        }
    }

    /// The rising delay of the root buffer driving another of its kind and
    /// `extra` more, as `LatencyBalancer::computeBufferDelay`.
    fn root_buffer_delay(&self, extra: Farads) -> Result<f32, CtsError> {
        let root = self.trees.setup.buffers.root;
        // NOTE: computeBufferDelay gives 0 without a rising arc, which the
        // buffer count divides by; PARE refuses to balance.
        rising_delay(self.link.liberty, root, extra)
            .ok_or_else(|| CtsError::NoRisingDelay(self.link.liberty.cell(root.id).name.clone()))
    }

    fn source_position(&self, db: &NetlistDB, node: usize) -> Point<Dbu> {
        if let Some(pin) = self.nodes[node].input {
            return db.average_pin_position(pin);
        }
        let net = self.trees.builders[self.root].top_input_net;
        net.and_then(|n| {
            db.nets[n]
                .pins
                .iter()
                .copied()
                .find(|&p| matches!(db.pins[p].owner, PinOwner::Io(_)))
        })
        .map_or_else(Point::default, |p| db.pin_position(p))
    }

    /// How many delay buffers a node needs to reach the slowest arrival,
    /// as `LatencyBalancer::computeNumberOfDelayBuffers`.
    fn delay_buffers_needed(
        &mut self,
        db: &NetlistDB,
        node: usize,
        src: Point<Dbu>,
    ) -> Result<(), CtsError> {
        // NOTE: computeNumberOfDelayBuffers skips an arrival of 0, its mark
        // for none; PARE skips only a node without an arrival.
        let (Some(arrival), Some(input)) = (self.nodes[node].arrival, self.nodes[node].input)
        else {
            return Ok(());
        };
        let gap = f64::from(self.worse_delay) - arrival;
        let first = (gap / f64::from(self.buffer_delay)) as i32;
        let sink = db.average_pin_position(input);
        let dx = (sink.x - src.x).get() as f32 / (first + 1) as f32;
        let dy = (sink.y - src.y).get() as f32 / (first + 1) as f32;
        let wire = Farads::new(f64::from(dx.abs() + dy.abs()) * self.trees.cap_per_dbu.get());
        let delay = self.root_buffer_delay(wire)?;
        self.nodes[node].buffers = Some((gap / f64::from(delay)) as i32);
        Ok(())
    }

    /// Balances a node's children bottom up, as
    /// `LatencyBalancer::balanceLatencies`.
    fn balance(&mut self, db: &mut NetlistDB, node: usize) -> Result<(), CtsError> {
        if self.nodes[node].children.is_empty() {
            return Ok(());
        }
        let src = self.source_position(db, node);
        let mut max_arrival: Option<f64> = None;
        let mut by_need: BTreeMap<i32, Vec<PinId>> = BTreeMap::new();
        for child in self.nodes[node].children.clone() {
            self.balance(db, child)?;
            self.delay_buffers_needed(db, child, src)?;
            // NOTE: balanceLatencies seeds the latest arrival with DBL_MIN,
            // which a node whose children have none keeps; PARE gives none.
            if let Some(at) = self.nodes[child].arrival {
                max_arrival = Some(max_arrival.map_or(at, |latest| latest.max(at)));
            }
            if let (Some(input), Some(need)) = (self.nodes[child].input, self.nodes[child].buffers)
            {
                by_need.entry(need).or_default().push(input);
            }
        }
        let mut groups = by_need.into_iter().rev();
        let mut previous = 0;
        if let Some((need, children)) = groups.next() {
            previous = need;
            let mut loads = children;
            for (need, children) in groups {
                let first = self.insert_delay_buffers(db, previous - need, src, &loads)?;
                loads = children;
                loads.extend(first);
                previous = need;
            }
        }
        self.nodes[node].buffers = Some(previous);
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
        src: Point<Dbu>,
        loads: &[PinId],
    ) -> Result<Option<PinId>, CtsError> {
        let (Some(&first_load), Some(center)) = (loads.first(), sink_bbox_center(db, loads)) else {
            return Ok(None);
        };
        let mut driving = db.pins[first_load].net;
        let dx = (center.x - src.x).get() as f32 / (count + 1) as f32;
        let dy = (center.y - src.y).get() as f32 / (count + 1) as f32;
        let root = &self.trees.builders[self.root];
        let ctx = self.trees.setup.context(db);
        let unit = root.unit();
        let buffer = self.trees.setup.buffers.root;
        let lib = self.link.liberty.cell(buffer.id);
        let pin_of = |port: LibPortId| {
            self.link
                .macro_pin_of_port(buffer.id, port)
                .ok_or_else(|| CtsError::MissingPin(lib.name.clone()))
        };
        let (in_pin, out_pin) = (pin_of(buffer.input)?, pin_of(buffer.output)?);
        let master = self
            .link
            .macro_of(buffer.id)
            .ok_or_else(|| CtsError::NoMaster(lib.name.clone()))?;
        let clock_name = root.clock.sdc_name.clone();
        let mut first_input_pin = None;
        for i in 0..count {
            let step = (i + 1) as f32;
            let at = TreePoint::new(
                f64::from(src.x.get() as f32 + dx * step) / unit,
                f64::from(src.y.get() as f32 + dy * step) / unit,
            );
            let location = root.legalize_one_buffer(&ctx, at, buffer).to_dbu(unit);
            let name = format!("delaybuf_{}_{clock_name}", self.index);
            let net_name = format!("delaynet_{}_{clock_name}", self.index);
            self.index += 1;
            let cell = db.add_cell(
                name,
                master,
                location,
                Orientation::N,
                PlacementStatus::Placed,
            )?;
            let net = db.add_net(&net_name)?;
            db.nets[net].signal_use = db.nets[driving].signal_use;
            db.nets[net].non_default_rule = db.nets[driving].non_default_rule;
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

/// The latest rising delay of `buffer` driving its own input and `extra`
/// more, its input slew settled over three lookups, as
/// `LatencyBalancer::computeBufferDelay`; none without a rising arc of
/// positive delay.
fn rising_delay(liberty: &Liberty, buffer: BufferCell, extra: Farads) -> Option<f32> {
    let cell = liberty.cell(buffer.id);
    let load = cell.ports[buffer.input].capacitance.rise + extra;
    let mut latest: Option<f32> = None;
    for set in cell
        .delay_arc_sets_to(buffer.output)
        .filter(|s| s.from == buffer.input)
    {
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
                d = delay.lookup(Seconds::new(f64::from(in_slew)), load).get() as f32;
                in_slew = slew.lookup(Seconds::new(f64::from(in_slew)), load).get() as f32;
            }
            latest = Some(latest.map_or(d, |l| l.max(d)));
        }
    }
    latest.filter(|&d| d > 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::parser::liberty;
    use std::path::Path;

    /// The buffer `B` whose timing arc has the tables `tables`.
    fn buffer_with(tables: &str) -> (Liberty, BufferCell) {
        let src = format!(
            "library (x) {{
               capacitive_load_unit (1,ff) ;
               lu_table_template (delay_t) {{
                 variable_1 : input_net_transition ;
                 variable_2 : total_output_net_capacitance ;
                 index_1 (\"0.005, 0.2\") ;
                 index_2 (\"1.0, 100.0\") ;
               }}
               cell (B) {{
                 pin (A) {{ direction : input ; capacitance : 1.0 ; }}
                 pin (Z) {{
                   direction : output ; function : \"A\" ;
                   timing () {{ related_pin : \"A\" ; timing_sense : positive_unate ; {tables} }}
                 }}
               }}
             }}"
        );
        let lib = liberty::parse_str(&src, Path::new("x.lib")).unwrap();
        let buffer = lib.buffer_cell(lib.cell_id("B").unwrap()).unwrap();
        (lib, buffer)
    }

    const TABLE: &str = "(delay_t) { values (\"0.02, 0.08\", \"0.03, 0.09\") ; }";

    #[test]
    fn a_buffers_rising_delay_grows_with_the_load_beyond_its_own_input() {
        let (lib, buffer) = buffer_with(&format!("cell_rise {TABLE} rise_transition {TABLE}"));

        let alone = rising_delay(&lib, buffer, Farads::ZERO).unwrap();
        let loaded = rising_delay(&lib, buffer, Farads::new(50e-15)).unwrap();

        assert!(0.0 < alone && alone < loaded, "{alone} {loaded}");
    }

    #[test]
    fn a_buffer_without_a_rising_arc_has_no_rising_delay() {
        let (lib, buffer) = buffer_with(&format!("cell_fall {TABLE} fall_transition {TABLE}"));

        assert_eq!(rising_delay(&lib, buffer, Farads::ZERO), None);
    }
}
