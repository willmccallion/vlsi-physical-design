//! Rebuffering a net for timing, as `OpenROAD`'s `Rebuffer`.
//!
//! A dynamic program over the net's tree tries every usable buffer at every
//! branch and along every long wire, keeps the options that trade more
//! capacitance for more slack, picks the one giving the driver the most
//! slack, then trades buffers back for area while the slack stays within a
//! small relaxation of that best.

// The Elmore slew estimates keep OpenROAD's grouping so they can be checked.
#![allow(clippy::suboptimal_flops)]

use crate::bnet::{self, Bnet, BnetPtr, BufferSize, Fixed, Kind, SlackEdges};
use crate::dmp::{self, PiModel};
use crate::fuzzy;
use crate::resizer::{RepairError, Resizer, buffer_delays};
use crate::sta::Via;
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{CellId, LayerId, NetId, PinId};
use pare_common::db::liberty::{ArcRole, Edge, LibCellId, MinMax};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use std::rc::Rc;

/// Margin kept in hand on slew limits while rebuffering, in percent.
const SLEW_MARGIN: f64 = 20.0;

/// Margin kept in hand on load limits while rebuffering, in percent.
const CAP_MARGIN: f64 = 20.0;

/// Elmore factor for 20 to 80 percent slew thresholds.
const ELMORE_SKEW_FACTOR: f64 = 1.39;

/// How much of the critical delay the area recovery may give back.
const RELAXATION_FACTOR: f64 = 0.01;

/// Rounds of buffering for timing.
const TIMING_ROUNDS: usize = 3;

/// Rounds of area recovery, each allowed a larger share of the slack.
const AREA_ROUNDS: usize = 5;

/// The timing of a path point at the driver being rebuffered: when it
/// arrives and through which arc from which input, as `OpenROAD` keeps the
/// arrival paths for each edge.
#[derive(Clone, Copy, Debug, PartialEq)]
struct DriverPoint {
    arrival: f64,
    /// The input the data came through and the arc set, or none for a
    /// start such as an input port.
    from: Option<(PinId, Edge, usize)>,
}

/// What the search knows about the driver of the net at hand.
#[derive(Clone, Debug)]
struct Driver {
    pin: PinId,
    cell: LibCellId,
    port: usize,
    fanout_limit: f64,
    max_slew: f64,
    load_high_water_mark: f64,
    arrival_paths: [Option<DriverPoint>; 2],
}

/// The delay, slack correction and slew at the driver for a tree.
#[derive(Clone, Copy, Debug, PartialEq)]
struct DriverTiming {
    delay: f64,
    correction: f64,
    slew: f64,
}

/// The rebuffering search with the buffers it may use.
#[derive(Debug)]
pub struct Rebuffer {
    sizes: Vec<BufferSize>,
    max_wire_length: i64,
    wire_length_step: i64,
    driver: Option<Driver>,
    /// Buffers taken out while rebuffering whole designs.
    pub removed: usize,
    /// Buffers put in.
    pub inserted: usize,
}

impl Rebuffer {
    /// Characterizes the buffers the search may insert, as `Rebuffer::init`
    /// and `initOnCorner`.
    pub fn new(resizer: &Resizer<'_>) -> Result<Self, RepairError> {
        let max_wire_length = max_wire_length(resizer);
        let mut sizes: Vec<BufferSize> = find_fast_buffers(resizer)
            .into_iter()
            .map(|cell| buffer_size(resizer, cell))
            .collect::<Result<_, _>>()?;
        sizes.sort_by(|a, b| a.input_cap.total_cmp(&b.input_cap));
        let smallest = sizes
            .first()
            .ok_or_else(|| RepairError::MissingPins("no buffers to rebuffer with".into()))?;
        let wire_length_step = max_wire_length
            .min(wire_length_limit_by_load_slew(resizer, smallest))
            .min(wire_length_limit_by_max_cap(resizer, smallest));
        Ok(Self {
            sizes,
            max_wire_length,
            wire_length_step: wire_length_step.max(1),
            driver: None,
            removed: 0,
            inserted: 0,
        })
    }

    /// The wire step on `layer`, as `wireLengthStepForLayer`: the step of
    /// the signal wire RC without a layer or without the layer's RC, else
    /// the least of the longest wire repair allows and the wire the smallest
    /// buffer drives within its load's slew and its own load limit on the
    /// layer's RC.
    fn wire_length_step_on(&self, resizer: &Resizer<'_>, layer: Option<LayerId>) -> i64 {
        let (res, cap) = resizer.tree_rc().wire(layer);
        if layer.is_none() || cap <= 0.0 || res <= 0.0 {
            return self.wire_length_step;
        }
        let Some(smallest) = self.sizes.first() else {
            return self.wire_length_step;
        };
        let dbu = resizer.db.dbu().get() as f64;
        let lib = resizer.link.liberty.cell(smallest.cell);
        let cap_limit = lib.buffer_ports().and_then(|(_, output)| {
            resizer
                .link
                .liberty
                .capacitance_limit(lib, &lib.ports[output])
        });
        let cap_length = cap_limit.map_or(i64::MAX, |limit| {
            let slack = margined(limit, CAP_MARGIN) - smallest.input_cap;
            meters_to_dbu(slack / cap, dbu).max(0)
        });
        let max_slew = margined(smallest.input_max_slew, SLEW_MARGIN);
        let a = res * cap;
        let b = res * smallest.input_cap + smallest.drive_resistance * cap;
        let c = smallest.drive_resistance * smallest.input_cap - max_slew / ELMORE_SKEW_FACTOR;
        let discriminant = b * b - 4.0 * a * c;
        let mut slew_length = i64::MAX;
        if discriminant >= 0.0 {
            let meters = (-b + discriminant.sqrt()) / (2.0 * a);
            if meters > 0.0 && meters <= 1.0 {
                slew_length = meters_to_dbu(meters, dbu);
            }
        }
        self.max_wire_length.min(slew_length).min(cap_length).max(1)
    }

    /// Rebuffers the net `driver` drives for timing, as
    /// `Rebuffer::rebufferPin`; returns how many buffers went in.
    pub fn rebuffer_pin(
        &mut self,
        resizer: &mut Resizer<'_>,
        driver: PinId,
    ) -> Result<usize, RepairError> {
        let PinOwner::Cell { .. } = resizer.db.pins[driver.index()].owner else {
            return Ok(0);
        };
        let net = resizer.db.pins[driver.index()].net;
        if has_top_level_output(resizer, net) {
            return Ok(0);
        }
        self.set_driver(resizer, driver);
        let Some(tree) = make_tree(resizer, driver) else {
            return Ok(0);
        };
        let mut tree = self.annotate_load_slacks(resizer, &tree, driver);
        for _ in 0..TIMING_ROUNDS {
            match self.buffer_for_timing(resizer, &tree, true) {
                Some(better) => tree = better,
                None => return Ok(0),
            }
        }
        let timing = self.driver_timing(resizer, &tree);
        let relaxation =
            (timing.delay.max(0.0) + critical_path_delay(&tree).to_seconds()) * RELAXATION_FACTOR;
        let target = self.slack_at_driver(resizer, &tree) - Fixed::from_seconds(relaxation);
        for round in 0..AREA_ROUNDS {
            let alpha = (round + 1) as f64 / AREA_ROUNDS as f64;
            match self.recover_area(resizer, &tree, target, alpha) {
                Some(leaner) => tree = leaner,
                None => return Ok(0),
            }
        }
        let inserted = export_tree(resizer, &tree, "rebuffer")?;
        self.inserted += inserted;
        Ok(inserted)
    }

    /// Strips the buffers from every net and buffers it again for timing,
    /// as `Rebuffer::fullyRebuffer` does for timing-driven placement.
    pub fn fully_rebuffer(&mut self, resizer: &mut Resizer<'_>) -> Result<(), RepairError> {
        let drivers: Vec<PinId> = {
            let netlist = resizer.netlist();
            let mut drivers = resizer.graph.drivers_by_level(&netlist);
            drivers.retain(|&driver| {
                let net = resizer.db.pins[driver.index()].net;
                let PinOwner::Cell { cell, .. } = resizer.db.pins[driver.index()].owner else {
                    return false;
                };
                let Some((_, lib)) = resizer.link.cell(resizer.db, cell) else {
                    return false;
                };
                !netlist.is_special(net)
                    && !resizer.clocks.is_clock(driver)
                    && !resizer.constants.is_constant(driver)
                    && !resizer.is_tristate_driver(driver)
                    && (!lib.is_buffer()
                        || !resizer.link.is_logic_std_cell(resizer.db, cell)
                        || is_port_buffer(resizer, cell))
            });
            drivers.reverse();
            drivers
        };
        for driver in drivers {
            if resizer.netlist().is_retired(driver) {
                continue;
            }
            self.set_driver(resizer, driver);
            let Some(original) = self.import_buffer_tree(resizer, driver) else {
                continue;
            };
            let original = self.annotate_load_slacks(resizer, &original, driver);
            let slack = self.slack_at_driver(resizer, &original).to_seconds();
            let sta_slack = resizer
                .sta()
                .map_or(f64::INFINITY, |sta| sta.slack(driver, MinMax::Max));
            let slack_error = slack - sta_slack;
            let unbuffered = resteiner(resizer, &strip_buffers(&original));
            let Some(mut timing_tree) = unbuffered.clone() else {
                continue;
            };
            let mut found = true;
            for _ in 0..TIMING_ROUNDS {
                if let Some(better) = self.buffer_for_timing(resizer, &timing_tree, true) {
                    timing_tree = better;
                } else {
                    found = false;
                    break;
                }
            }
            if !found {
                continue;
            }
            let timing = self.driver_timing(resizer, &timing_tree);
            let target = if timing_tree.slack_edges.is_some() {
                let relaxation = (self.slack_at_driver(resizer, &timing_tree).to_seconds()
                    - slack_error.min(0.0))
                .max(0.0)
                    / 4.0
                    + (timing.delay.max(0.0) + critical_path_delay(&timing_tree).to_seconds())
                        * RELAXATION_FACTOR;
                self.slack_at_driver(resizer, &timing_tree) - Fixed::from_seconds(relaxation)
            } else {
                -Fixed::INF
            };
            let mut area_tree = timing_tree;
            let mut recovered = true;
            for round in 0..AREA_ROUNDS {
                let alpha = (round + 1) as f64 / AREA_ROUNDS as f64;
                if let Some(leaner) = self.recover_area(resizer, &area_tree, target, alpha) {
                    area_tree = leaner;
                } else {
                    recovered = false;
                    break;
                }
            }
            if !recovered {
                continue;
            }
            let old_buffers = collect_imported_buffers(resizer, driver, &original);
            resizer.batch_edits();
            self.inserted += export_tree(resizer, &area_tree, "place")?;
            for buffer in old_buffers {
                if !resizer.db.cells[buffer.index()].retired {
                    let _ = resizer.remove_buffer(buffer)?;
                    self.removed += 1;
                }
            }
            resizer.flush();
        }
        Ok(())
    }

    /// Notes the driver at hand: its fanout and slew limits, as `setPin`.
    fn set_driver(&mut self, resizer: &Resizer<'_>, pin: PinId) {
        let (cell, port) = resizer
            .link
            .pin_port(resizer.db, pin)
            .unwrap_or((LibCellId(0), 0));
        let max_slew = resizer
            .driver_slew_limit(pin)
            .map_or(f64::INFINITY, |l| margined(l, SLEW_MARGIN));
        self.driver = Some(Driver {
            pin,
            cell,
            port,
            fanout_limit: resizer.fanout_limit().unwrap_or(f64::INFINITY),
            max_slew,
            load_high_water_mark: 0.0,
            arrival_paths: [None, None],
        });
    }

    /// Gives each load the slack of its worst path measured at the driver,
    /// and remembers the driver's arrival paths, as `annotateLoadSlacks`;
    /// then times every node up from the loads, as `annotateTiming`.
    fn annotate_load_slacks(
        &mut self,
        resizer: &Resizer<'_>,
        tree: &BnetPtr,
        driver: PinId,
    ) -> BnetPtr {
        let mut arrival_paths: [Option<DriverPoint>; 2] = [None, None];
        let annotated = self.annotate_loads(resizer, tree, driver, &mut arrival_paths);
        if let Some(d) = &mut self.driver {
            d.arrival_paths = arrival_paths;
        }
        self.retime(resizer, &annotated)
    }

    fn annotate_loads(
        &self,
        resizer: &Resizer<'_>,
        node: &BnetPtr,
        driver: PinId,
        arrival_paths: &mut [Option<DriverPoint>; 2],
    ) -> BnetPtr {
        match &node.kind {
            Kind::Load { pin } => {
                let Some(sta) = resizer.sta() else {
                    return node.with_slack(Fixed::INF, None);
                };
                let netlist = resizer.netlist();
                let Some(path) = sta.worst_slack_path(&netlist, *pin, MinMax::Max) else {
                    return node.with_slack(Fixed::INF, None);
                };
                let Some(at_driver) = path.points.iter().position(|p| p.pin == driver) else {
                    return node.with_slack(Fixed::INF, None);
                };
                let driver_point = path.points[at_driver];
                let end = path.end();
                let required = end.arrival + path.slack;
                let slack = Fixed::from_seconds(required - driver_point.arrival);
                let edge = end.edge;
                if arrival_paths[edge.index()].is_none() {
                    let from = (at_driver > 0)
                        .then(|| path.points[at_driver - 1])
                        .and_then(|prev| match driver_point.via {
                            Via::Arc { set } => Some((prev.pin, prev.edge, set)),
                            _ => None,
                        });
                    arrival_paths[edge.index()] = Some(DriverPoint {
                        arrival: driver_point.arrival,
                        from,
                    });
                }
                node.with_slack(slack, Some(SlackEdges::One(edge)))
            }
            Kind::Junction { left, right } => {
                let left = self.annotate_loads(resizer, left, driver, arrival_paths);
                let right = self.annotate_loads(resizer, right, driver, arrival_paths);
                Bnet::junction(node.at, left, right)
            }
            Kind::Wire { to, layer } => {
                let to = self.annotate_loads(resizer, to, driver, arrival_paths);
                wire(resizer, node.at, *layer, to)
            }
            Kind::Via {
                to,
                layer,
                to_layer,
            } => {
                let to = self.annotate_loads(resizer, to, driver, arrival_paths);
                Bnet::via(node.at, *layer, *to_layer, to, &resizer.tree_rc())
            }
            Kind::Buffer { cell, to } => {
                let to = self.annotate_loads(resizer, to, driver, arrival_paths);
                self.buffer_node(resizer, node.at, *cell, to)
            }
        }
    }

    /// Rebuilds a tree so every node's slack, capacitance and delay follow
    /// from its children, as `annotateTiming`.
    fn retime(&self, resizer: &Resizer<'_>, node: &BnetPtr) -> BnetPtr {
        match &node.kind {
            Kind::Load { .. } => Rc::clone(node),
            Kind::Junction { left, right } => Bnet::junction(
                node.at,
                self.retime(resizer, left),
                self.retime(resizer, right),
            ),
            Kind::Wire { to, layer } => wire(resizer, node.at, *layer, self.retime(resizer, to)),
            Kind::Via {
                to,
                layer,
                to_layer,
            } => Bnet::via(
                node.at,
                *layer,
                *to_layer,
                self.retime(resizer, to),
                &resizer.tree_rc(),
            ),
            Kind::Buffer { cell, to } => {
                self.buffer_node(resizer, node.at, *cell, self.retime(resizer, to))
            }
        }
    }

    /// A buffer node of `cell` at `at` driving `to`, timed at the load.
    fn buffer_node(
        &self,
        resizer: &Resizer<'_>,
        at: Point<Dbu>,
        cell: LibCellId,
        to: BnetPtr,
    ) -> BnetPtr {
        let size = self
            .sizes
            .iter()
            .find(|s| s.cell == cell)
            .copied()
            .unwrap_or_else(|| {
                buffer_size(resizer, cell).unwrap_or(BufferSize {
                    cell,
                    input_cap: 0.0,
                    input_fanout: 0.0,
                    input_max_slew: f64::INFINITY,
                    output_cap: 0.0,
                    area: 0.0,
                    intrinsic_delay: Fixed::ZERO,
                    margined_max_cap: f64::INFINITY,
                    drive_resistance: 0.0,
                })
            });
        let delay = buffer_delay(resizer, &size, to.slack_edges, to.cap + size.output_cap);
        Bnet::buffer(at, &size, to, delay)
    }

    /// The driver's gate delay, slack correction and slew for a tree, as
    /// `drvrPinTiming`: the gate timed again at the tree's load, and the
    /// correction the arrival at the driver would take from that.
    fn driver_timing(&self, resizer: &Resizer<'_>, tree: &Bnet) -> DriverTiming {
        let Some(driver) = &self.driver else {
            return DriverTiming {
                delay: 0.0,
                correction: 0.0,
                slew: 0.0,
            };
        };
        let Some(edges) = tree.slack_edges else {
            return DriverTiming {
                delay: 0.0,
                correction: 0.0,
                slew: 0.0,
            };
        };
        let cell = resizer.link.liberty.cell(driver.cell);
        let out_cap = cell.ports[driver.port].capacitance();
        let mut delay = 0.0;
        let mut correction = f64::INFINITY;
        let mut slew = 0.0;
        for &edge in edges.edges() {
            let Some(point) = driver.arrival_paths[edge.index()] else {
                continue;
            };
            let Some((from_pin, from_edge, set)) = point.from else {
                correction = correction.min(0.0);
                continue;
            };
            let arc_set = &cell.arc_sets[set];
            let in_slew = if arc_set.role == ArcRole::RegClkToQ && resizer.clocks.is_clock(from_pin)
            {
                0.0
            } else {
                resizer.graph.slew(from_pin).get(from_edge)
            };
            let load = tree.cap + out_cap;
            let mut rf_delay = 0.0;
            let mut rf_slew = 0.0;
            for arc in arc_set
                .arcs
                .iter()
                .filter(|a| a.from_edge == from_edge && a.to_edge == edge)
            {
                let result = dmp::gate_delay(
                    arc,
                    &resizer.link.liberty.thresholds,
                    in_slew,
                    PiModel {
                        c2: load,
                        rpi: 0.0,
                        c1: 0.0,
                    },
                );
                rf_delay = f64::max(rf_delay, result.delay);
                rf_slew = f64::max(rf_slew, result.slew);
            }
            let prev_arrival = resizer
                .sta()
                .and_then(|sta| sta.arrival(from_pin, from_edge, MinMax::Max))
                .unwrap_or(0.0);
            let rf_correction = point.arrival - (prev_arrival + rf_delay);
            delay = f64::max(delay, rf_delay);
            correction = correction.min(rf_correction);
            slew = f64::max(slew, rf_slew);
        }
        if correction == f64::INFINITY {
            correction = 0.0;
        }
        DriverTiming {
            delay,
            correction,
            slew,
        }
    }

    /// The tree's slack as the driver sees it.
    fn slack_at_driver(&self, resizer: &Resizer<'_>, tree: &Bnet) -> Fixed {
        tree.slack + Fixed::from_seconds(self.driver_timing(resizer, tree).correction)
    }

    /// Whether the driver's slew keeps every load of the tree within its
    /// limit by the Elmore estimate, as `loadSlewSatisfactory`.
    fn load_slew_satisfactory(&self, resizer: &Resizer<'_>, tree: &Bnet) -> bool {
        let Some(driver) = &self.driver else {
            return true;
        };
        let rc = resizer.graph.wire_rc();
        let r_drvr = resizer
            .link
            .liberty
            .cell(driver.cell)
            .drive_resistance(driver.port);
        let meters = tree.max_load_wire_length() as f64 / resizer.db.dbu().get() as f64 * 1e-6;
        let load_slew = (r_drvr + meters * rc.resistance) * tree.cap * ELMORE_SKEW_FACTOR;
        load_slew <= margined(tree.max_load_slew, SLEW_MARGIN)
    }

    /// The slack of a candidate tree at the driver, or none when the driver
    /// could not drive it, as `evaluateOption`.
    fn evaluate_option(&mut self, resizer: &Resizer<'_>, option: &Bnet) -> Option<Fixed> {
        let timing = self.driver_timing(resizer, option);
        let slack = option.slack + Fixed::from_seconds(timing.correction);
        if !self.load_slew_satisfactory(resizer, option) {
            return None;
        }
        let driver = self.driver.as_mut()?;
        if timing.slew > driver.max_slew && option.cap > driver.load_high_water_mark {
            return None;
        }
        driver.load_high_water_mark = driver.load_high_water_mark.max(option.cap);
        Some(slack)
    }

    /// The buffering options for a tree, best slack last, as
    /// `bufferForTiming`'s recursion; the chosen one is the option with
    /// the most slack at the driver.
    fn buffer_for_timing(
        &mut self,
        resizer: &Resizer<'_>,
        tree: &BnetPtr,
        allow_rewrite: bool,
    ) -> Option<BnetPtr> {
        let options = self.timing_options(resizer, tree, allow_rewrite);
        let mut best: Option<(Fixed, BnetPtr)> = None;
        for option in options {
            let Some(slack) = self.evaluate_option(resizer, &option) else {
                continue;
            };
            if best.is_none()
                || option.slack_edges.is_none()
                || best.as_ref().is_some_and(|(s, _)| slack > *s)
            {
                best = Some((slack, option));
            }
        }
        best.map(|(_, option)| option)
    }

    fn timing_options(
        &self,
        resizer: &Resizer<'_>,
        node: &BnetPtr,
        allow_rewrite: bool,
    ) -> Vec<BnetPtr> {
        match &node.kind {
            Kind::Load { .. } => vec![Rc::clone(node)],
            Kind::Junction { left, right } => {
                let left_options = self.timing_options(resizer, left, allow_rewrite);
                let right_options = self.timing_options(resizer, right, allow_rewrite);
                self.junction_options(resizer, node, &left_options, &right_options, allow_rewrite)
            }
            Kind::Wire { to, .. } | Kind::Via { to, .. } | Kind::Buffer { to, .. } => {
                let under = to.under_wires_and_buffers();
                let options = self.timing_options(resizer, &under, allow_rewrite);
                self.wire_options(resizer, node, &under, node.wire_layer_below(), options)
            }
        }
    }

    /// Options for a wire on `layer` from `node` down to `under`: long
    /// wires are walked in steps of the layer's wire length limit with
    /// buffers offered at each step, short ones get buffers at either end.
    fn wire_options(
        &self,
        resizer: &Resizer<'_>,
        node: &Bnet,
        under: &Bnet,
        layer: Option<LayerId>,
        mut options: Vec<BnetPtr>,
    ) -> Vec<BnetPtr> {
        let mut location = under.at;
        let full_length = bnet::manhattan(node.at, location);
        let step = self.wire_length_step_on(resizer, layer);
        if full_length <= step / 2 {
            let mut near: Vec<BnetPtr> = options
                .iter()
                .map(|o| wire(resizer, node.at, layer, Rc::clone(o)))
                .collect();
            self.insert_buffer_options(resizer, &mut near, 0, None);
            if near.is_empty() {
                let mut far = options;
                self.insert_buffer_options(resizer, &mut far, full_length, None);
                near = far
                    .iter()
                    .map(|o| wire(resizer, node.at, layer, Rc::clone(o)))
                    .collect();
                self.insert_buffer_options(resizer, &mut near, 0, None);
            }
            return near;
        }
        self.insert_buffer_options(resizer, &mut options, full_length.min(step), None);
        while location != node.at {
            let last_valid = options.clone();
            let mut dx = node.at.x.get() - location.x.get();
            let mut dy = node.at.y.get() - location.y.get();
            if dx.abs() + dy.abs() >= step {
                let ratio = dx.abs() as f64 / (dx.abs() + dy.abs()) as f64;
                let dx_abs = ((ratio * step as f64) as i64).min(step);
                let dy_abs = step - dx_abs;
                dx = if dx > 0 { dx_abs } else { -dx_abs };
                dy = if dy > 0 { dy_abs } else { -dy_abs };
            }
            location = Point::new(Dbu(location.x.get() + dx), Dbu(location.y.get() + dy));
            let remaining = bnet::manhattan(node.at, location);
            options = options
                .iter()
                .map(|o| wire(resizer, location, layer, Rc::clone(o)))
                .collect();
            self.insert_buffer_options(resizer, &mut options, remaining.min(step), None);
            if options.is_empty() {
                log::warn!(
                    "rebuffering {}: no option can drive the wire step; keeping the last that could",
                    resizer
                        .db
                        .pin_name(self.driver.as_ref().map_or(PinId::new(0), |d| d.pin))
                );
                options = last_valid;
                break;
            }
        }
        options
    }

    /// The options for a junction: each side's options paired off from the
    /// most slack down, keeping the pairs that lower the capacitance.
    fn junction_options(
        &self,
        resizer: &Resizer<'_>,
        node: &Bnet,
        left: &[BnetPtr],
        right: &[BnetPtr],
        allow_rewrite: bool,
    ) -> Vec<BnetPtr> {
        let fanout_limit = self
            .driver
            .as_ref()
            .map_or(f64::INFINITY, |d| d.fanout_limit);
        let mut options = Vec::with_capacity(left.len().max(right.len()));
        let mut best_cap = f64::INFINITY;
        let (mut li, mut ri) = (left.len(), right.len());
        if li == 0 || ri == 0 {
            return options;
        }
        li -= 1;
        ri -= 1;
        loop {
            while li > 0 && left[li - 1].slack >= right[ri].slack {
                li -= 1;
            }
            while ri > 0 && right[ri - 1].slack >= left[li].slack {
                ri -= 1;
            }
            let rewritten = if allow_rewrite {
                self.rewrite_topology(resizer, node, &left[li], &right[ri], best_cap)
            } else {
                None
            };
            let junction = rewritten.unwrap_or_else(|| {
                Bnet::junction(node.at, Rc::clone(&left[li]), Rc::clone(&right[ri]))
            });
            if junction.fanout <= fanout_limit {
                best_cap = junction.cap;
                options.push(junction);
            }
            let mut exhausted = false;
            loop {
                let next_left = if li > 0 {
                    left[li - 1].slack
                } else {
                    -Fixed::INF
                };
                let next_right = if ri > 0 {
                    right[ri - 1].slack
                } else {
                    -Fixed::INF
                };
                if next_left > next_right {
                    if li == 0 {
                        exhausted = true;
                        break;
                    }
                    li -= 1;
                } else {
                    if ri == 0 {
                        exhausted = true;
                        break;
                    }
                    ri -= 1;
                }
                if left[li].cap + right[ri].cap < best_cap {
                    break;
                }
            }
            if exhausted {
                break;
            }
        }
        options.reverse();
        options
    }

    /// Tries pulling the less critical side under a buffer with the less
    /// critical half of the critical side, as `attemptTopologyRewrite`.
    fn rewrite_topology(
        &self,
        resizer: &Resizer<'_>,
        node: &Bnet,
        left: &BnetPtr,
        right: &BnetPtr,
        best_cap: f64,
    ) -> Option<BnetPtr> {
        let junction_slack = left.slack.min(right.slack);
        let (critical, aux1) = if left.slack < right.slack {
            (left.under_wires(), right.under_wires())
        } else {
            (right.under_wires(), left.under_wires())
        };
        let Kind::Junction {
            left: c_left,
            right: c_right,
        } = &critical.kind
        else {
            return None;
        };
        let (mut crit2, mut aux2) = (Rc::clone(c_left), Rc::clone(c_right));
        if crit2.slack > aux2.slack {
            std::mem::swap(&mut crit2, &mut aux2);
        }
        let aux2 = aux2.under_wires();
        if !matches!(aux1.kind, Kind::Buffer { .. }) && !matches!(aux2.kind, Kind::Buffer { .. }) {
            return None;
        }
        let aux1 = aux1.under_wires_and_buffers();
        let aux2 = aux2.under_wires_and_buffers();
        let crit2 = crit2.under_wires_and_buffers();
        let in1 = wire(resizer, node.at, None, aux1);
        let in2 = wire(resizer, node.at, None, aux2);
        let junction1 = wire(resizer, node.at, None, Bnet::junction(node.at, in1, in2));
        let in3 = wire(resizer, node.at, None, crit2);
        for size in &self.sizes {
            if fuzzy::greater_equal(size.input_cap + in3.cap, best_cap)
                || fuzzy::greater_equal(size.input_cap + in3.cap, left.cap + right.cap)
                || junction1.slack - size.intrinsic_delay < junction_slack
            {
                break;
            }
            let delay = buffer_delay(
                resizer,
                size,
                junction1.slack_edges,
                junction1.cap + size.output_cap,
            );
            let buffer_slack = junction1.slack - delay;
            if buffer_slack >= junction_slack && can_drive(resizer, size, &junction1, 0) {
                let buffer = Bnet::buffer(node.at, size, Rc::clone(&junction1), delay);
                return Some(Bnet::junction(node.at, buffer, in3));
            }
        }
        None
    }

    /// Offers every buffer size on top of the options, keeping the options
    /// that trade capacitance for slack (or, recovering area, for area at
    /// no less than a slack threshold), as `insertBufferOptions`.
    fn insert_buffer_options(
        &self,
        resizer: &Resizer<'_>,
        options: &mut Vec<BnetPtr>,
        next_segment: i64,
        area: Option<(Fixed, &Bnet)>,
    ) {
        if options.is_empty() {
            return;
        }
        let Some(strong) = self.sizes.last().copied() else {
            return;
        };
        let (area_oriented, threshold, envelope) = match area {
            Some((threshold, exemplar)) => (
                true,
                threshold,
                Some(exemplar.metrics().with_slack(threshold)),
            ),
            None => (false, Fixed::ZERO, None),
        };
        let mut assured = !area_oriented;
        let mut best_area = f64::INFINITY;
        let mut best_slack = -Fixed::INF;
        let mut new_options: Vec<BnetPtr> = Vec::with_capacity(options.len() * 2);
        let mut at = 0usize;
        let sizes = self.sizes.clone();
        for size in &sizes {
            while at < options.len() && options[at].cap <= size.input_cap {
                let option = &options[at];
                let keep = if area_oriented {
                    fuzzy::less(option.area, best_area) && option.slack >= threshold
                } else {
                    option.slack > best_slack
                } && can_drive(resizer, &strong, option, next_segment);
                if keep {
                    if let Some(envelope) = &envelope
                        && !assured
                        && option.fits(envelope)
                    {
                        assured = true;
                    }
                    best_slack = option.slack;
                    best_area = option.area;
                    new_options.push(Rc::clone(option));
                }
                at += 1;
            }
            let start = if new_options.is_empty() && at == options.len() && at > 0 {
                at - 1
            } else {
                at
            };
            let mut load_option: Option<(BnetPtr, Fixed)> = None;
            for option in &options[start..] {
                let promising = if area_oriented {
                    option.slack - size.intrinsic_delay >= threshold
                        && fuzzy::less(option.area + size.area, best_area)
                } else {
                    option.slack - size.intrinsic_delay > best_slack
                };
                if !promising || !can_drive(resizer, size, option, 0) {
                    continue;
                }
                let delay = buffer_delay(
                    resizer,
                    size,
                    option.slack_edges,
                    option.cap + size.output_cap,
                );
                let slack = option.slack - delay;
                let better = if area_oriented {
                    slack >= threshold
                } else {
                    slack > best_slack
                };
                if better {
                    load_option = Some((Rc::clone(option), delay));
                    best_slack = slack;
                    best_area = option.area + size.area;
                }
            }
            if let Some((option, delay)) = load_option {
                let buffered = Bnet::buffer(option.at, size, option, delay);
                if let Some(envelope) = &envelope
                    && !assured
                    && buffered.fits(envelope)
                {
                    assured = true;
                }
                new_options.push(buffered);
            }
        }
        while at < options.len() {
            let option = &options[at];
            let keep = if area_oriented {
                fuzzy::less(option.area, best_area) && option.slack >= threshold
            } else {
                option.slack > best_slack
            } && can_drive(resizer, &strong, option, next_segment);
            if keep {
                if let Some(envelope) = &envelope
                    && !assured
                    && option.fits(envelope)
                {
                    assured = true;
                }
                best_slack = option.slack;
                best_area = option.area;
                new_options.push(Rc::clone(option));
            }
            at += 1;
        }
        if !assured
            && let Some((threshold, exemplar)) = area
            && let Some(envelope) = &envelope
        {
            let fallback = match &exemplar.kind {
                Kind::Buffer { cell, .. } => {
                    let size = sizes.iter().find(|s| s.cell == *cell).copied();
                    let mut best: Option<(f64, BnetPtr)> = None;
                    if let Some(size) = size {
                        for option in options.iter() {
                            if best.as_ref().is_some_and(|(a, _)| option.area >= *a) {
                                continue;
                            }
                            let delay = buffer_delay(
                                resizer,
                                &size,
                                option.slack_edges,
                                option.cap + size.output_cap,
                            );
                            if can_drive(resizer, &size, option, 0)
                                && option.slack - delay >= threshold
                            {
                                let buffered =
                                    Bnet::buffer(option.at, &size, Rc::clone(option), delay);
                                if buffered.fits(envelope) {
                                    best = Some((option.area, buffered));
                                }
                            }
                        }
                    }
                    best.map(|(_, b)| b)
                }
                _ => options.iter().find(|o| o.fits(envelope)).map(Rc::clone),
            };
            match fallback {
                Some(option) => insert_assured(&mut new_options, option),
                None => log::warn!(
                    "rebuffering {}: area recovery cannot reproduce the timing solution",
                    resizer
                        .db
                        .pin_name(self.driver.as_ref().map_or(PinId::new(0), |d| d.pin))
                ),
            }
        }
        *options = new_options;
    }

    /// Trades buffers for area while every node keeps at least a share
    /// `alpha` of the way from its present slack to the slack target, as
    /// `recoverArea`.
    fn recover_area(
        &mut self,
        resizer: &Resizer<'_>,
        root: &BnetPtr,
        slack_target: Fixed,
        alpha: f64,
    ) -> Option<BnetPtr> {
        let (correction, target) = if root.slack_edges.is_some() {
            (
                Fixed::from_seconds(self.driver_timing(resizer, root).correction),
                slack_target,
            )
        } else {
            (Fixed::ZERO, -Fixed::INF)
        };
        let root = with_arrivals(root, -correction);
        let options = self.area_options(resizer, &root, target, alpha, 0);
        let mut best_slack: Option<(Fixed, BnetPtr)> = None;
        let mut best_area: Option<(f64, BnetPtr)> = None;
        for option in options {
            let Some(slack) = self.evaluate_option(resizer, &option) else {
                continue;
            };
            let unconstrained = option.slack_edges.is_none();
            if best_slack.is_none()
                || unconstrained
                || best_slack.as_ref().is_some_and(|(s, _)| slack > *s)
            {
                best_slack = Some((slack, Rc::clone(&option)));
            }
            if (slack >= target || unconstrained)
                && best_area
                    .as_ref()
                    .is_none_or(|(a, _)| fuzzy::less(option.area, *a))
            {
                best_area = Some((option.area, Rc::clone(&option)));
            }
        }
        best_area
            .map(|(_, o)| o)
            .or_else(|| best_slack.map(|(_, o)| o))
    }

    fn area_options(
        &self,
        resizer: &Resizer<'_>,
        node: &BnetPtr,
        slack_target: Fixed,
        alpha: f64,
        upstream_wire: i64,
    ) -> Vec<BnetPtr> {
        match &node.kind {
            Kind::Load { .. } => vec![Rc::clone(node)],
            Kind::Via {
                to,
                layer,
                to_layer,
            } => self
                .area_options(resizer, to, slack_target, alpha, upstream_wire)
                .into_iter()
                .map(|o| Bnet::via(node.at, *layer, *to_layer, o, &resizer.tree_rc()))
                .collect(),
            Kind::Wire { .. } | Kind::Buffer { .. } => {
                let inner = match &node.kind {
                    Kind::Buffer { to, .. } => Rc::clone(to),
                    _ => Rc::clone(node),
                };
                let mut options = if let Kind::Wire { to, layer } = &inner.kind {
                    let options =
                        self.area_options(resizer, to, slack_target, alpha, inner.length());
                    options
                        .into_iter()
                        .map(|o| wire(resizer, inner.at, *layer, o))
                        .collect()
                } else {
                    self.area_options(resizer, &inner, slack_target, alpha, 0)
                };
                let threshold = Fixed::lerp(node.slack, slack_target + node.arrival_delay, alpha);
                self.insert_buffer_options(
                    resizer,
                    &mut options,
                    upstream_wire,
                    Some((threshold, node)),
                );
                options
            }
            Kind::Junction { left, right } => {
                let left_options =
                    self.area_options(resizer, left, slack_target, alpha, upstream_wire);
                let right_options =
                    self.area_options(resizer, right, slack_target, alpha, upstream_wire);
                let threshold = Fixed::lerp(node.slack, slack_target + node.arrival_delay, alpha);
                let envelope = node.metrics().with_slack(threshold);
                let fanout_limit = self
                    .driver
                    .as_ref()
                    .map_or(f64::INFINITY, |d| d.fanout_limit);
                let mut fallback: Option<BnetPtr> = None;
                let mut options = Vec::with_capacity(left_options.len() * right_options.len());
                for l in &left_options {
                    for r in &right_options {
                        let junction = Bnet::junction(node.at, Rc::clone(l), Rc::clone(r));
                        if fallback.is_none() && junction.fits(&envelope) {
                            fallback = Some(Rc::clone(&junction));
                        }
                        if junction.fanout <= fanout_limit {
                            options.push(junction);
                        }
                    }
                }
                prune_cap_against_area(&mut options);
                if !options.iter().any(|o| o.fits(&envelope))
                    && let Some(fallback) = fallback
                {
                    insert_assured(&mut options, fallback);
                }
                options
            }
        }
    }

    /// The tree of `driver`'s net with the removable buffers under it
    /// folded in as buffer nodes, as `importBufferTree`.
    fn import_buffer_tree(&self, resizer: &Resizer<'_>, driver: PinId) -> Option<BnetPtr> {
        let tree = make_tree(resizer, driver)?;
        self.import_node(resizer, &tree)
    }

    fn import_node(&self, resizer: &Resizer<'_>, node: &BnetPtr) -> Option<BnetPtr> {
        match &node.kind {
            Kind::Wire { to, layer } => {
                let inner = self.import_node(resizer, to)?;
                Some(wire(resizer, node.at, *layer, inner))
            }
            Kind::Via {
                to,
                layer,
                to_layer,
            } => {
                let inner = self.import_node(resizer, to)?;
                Some(Bnet::via(
                    node.at,
                    *layer,
                    *to_layer,
                    inner,
                    &resizer.tree_rc(),
                ))
            }
            Kind::Junction { left, right } => {
                let left = self.import_node(resizer, left)?;
                let right = self.import_node(resizer, right)?;
                Some(Bnet::junction(node.at, left, right))
            }
            Kind::Buffer { cell, to } => {
                let inner = self.import_node(resizer, to)?;
                Some(self.buffer_node(resizer, node.at, *cell, inner))
            }
            Kind::Load { pin } => {
                let PinOwner::Cell { cell, .. } = resizer.db.pins[pin.index()].owner else {
                    return Some(Rc::clone(node));
                };
                let Some((lib_id, lib)) = resizer.link.cell(resizer.db, cell) else {
                    return Some(Rc::clone(node));
                };
                let netlist = resizer.netlist();
                let is_input = netlist
                    .lib_port(*pin)
                    .is_some_and(pare_common::db::liberty::LibPort::is_input);
                if !lib.is_buffer()
                    || !is_input
                    || !resizer.link.is_logic_std_cell(resizer.db, cell)
                    || is_port_buffer(resizer, cell)
                    || !can_remove_buffer(resizer, cell)
                {
                    return Some(Rc::clone(node));
                }
                let (_, out_pin) = resizer.buffer_pins(cell)?;
                let Some(inner) = self.import_buffer_tree(resizer, out_pin) else {
                    return Some(Rc::clone(node));
                };
                Some(self.buffer_node(resizer, node.at, lib_id, inner))
            }
        }
    }
}

/// Puts the buffers of a chosen tree into the design, bottom up, as
/// `exportBufferTree`; returns how many went in.
fn export_tree(
    resizer: &mut Resizer<'_>,
    tree: &BnetPtr,
    base: &str,
) -> Result<usize, RepairError> {
    let mut count = 0;
    let _ = export_node(resizer, tree, base, &mut count)?;
    Ok(count)
}

/// Inserts the buffers under `node` and returns the pins its parent
/// drives.
fn export_node(
    resizer: &mut Resizer<'_>,
    node: &BnetPtr,
    base: &str,
    count: &mut usize,
) -> Result<Vec<PinId>, RepairError> {
    match &node.kind {
        Kind::Load { pin } => Ok(vec![*pin]),
        Kind::Wire { to, .. } | Kind::Via { to, .. } => export_node(resizer, to, base, count),
        Kind::Junction { left, right } => {
            let mut loads = export_node(resizer, left, base, count)?;
            loads.extend(export_node(resizer, right, base, count)?);
            Ok(loads)
        }
        Kind::Buffer { cell, to } => {
            let loads = export_node(resizer, to, base, count)?;
            if loads.is_empty() {
                return Ok(loads);
            }
            let current = resizer.db.pins[loads[0].index()].net;
            let inserted =
                resizer.insert_buffer_before_loads(current, &loads, *cell, node.at, base)?;
            *count += 1;
            Ok(vec![inserted.input])
        }
    }
}

/// A fresh Steiner tree over the loads of `tree` from its top, as
/// `resteiner`; none when it has no loads.
fn resteiner(resizer: &Resizer<'_>, tree: &BnetPtr) -> Option<BnetPtr> {
    let mut sinks = Vec::new();
    collect_sinks(tree, &mut sinks);
    bnet::over_subtrees(tree.at, &sinks, &resizer.tree_rc())
}

/// The tree of `driver`'s net over its estimated parasitics.
fn make_tree(resizer: &Resizer<'_>, driver: PinId) -> Option<BnetPtr> {
    let net = resizer.db.pins[driver.index()].net;
    let netlist = resizer.netlist();
    let parasitics = resizer.graph.parasitics(net)?;
    bnet::from_parasitics(&netlist, parasitics, driver, &resizer.tree_rc(), resizer)
}

/// A wire node from `at` down to `to` on `layer`, or at the signal wire
/// RC without one.
fn wire(resizer: &Resizer<'_>, at: Point<Dbu>, layer: Option<LayerId>, to: BnetPtr) -> BnetPtr {
    Bnet::wire(at, to, layer, &resizer.tree_rc())
}

/// The worst delay of `size` over `edges` at `load_cap`, as
/// `Rebuffer::bufferDelay`; zero without edges.
fn buffer_delay(
    resizer: &Resizer<'_>,
    size: &BufferSize,
    edges: Option<SlackEdges>,
    load_cap: f64,
) -> Fixed {
    let Some(edges) = edges else {
        return Fixed::ZERO;
    };
    let cell = resizer.link.liberty.cell(size.cell);
    let delays = buffer_delays(cell, &resizer.targets.slews, load_cap);
    let mut worst = Fixed::ZERO;
    for &edge in edges.edges() {
        worst = worst.max(Fixed::from_seconds(delays.get(edge)));
    }
    worst
}

/// Whether `size` can drive `tree` plus `extra_wire` of wire within its
/// slew and load limits, as `bufferSizeCanDriveLoad`.
fn can_drive(resizer: &Resizer<'_>, size: &BufferSize, tree: &Bnet, extra_wire: i64) -> bool {
    let rc = resizer.graph.wire_rc();
    let dbu = resizer.db.dbu().get() as f64;
    let extra_cap = extra_wire as f64 / dbu * 1e-6 * rc.capacitance + size.output_cap;
    let wire_meters = (tree.max_load_wire_length() + extra_wire) as f64 / dbu * 1e-6;
    let load_slew = (size.drive_resistance + wire_meters * rc.resistance)
        * (tree.cap + extra_cap)
        * ELMORE_SKEW_FACTOR;
    load_slew <= margined(tree.max_load_slew, SLEW_MARGIN)
        && tree.cap + extra_cap <= size.margined_max_cap
}

/// Keeps an option in a list sorted by capacitance, as `insertAssuredOption`.
fn insert_assured(options: &mut Vec<BnetPtr>, assured: BnetPtr) {
    let at = options
        .iter()
        .position(|o| o.cap >= assured.cap)
        .unwrap_or(options.len());
    options.insert(at, assured);
}

/// Keeps, of options sorted by area then capacitance, those whose
/// capacitance beats every lower-area one, leaving them sorted by
/// capacitance, as `pruneCapVsAreaOptions`.
fn prune_cap_against_area(options: &mut Vec<BnetPtr>) {
    options.sort_by(|a, b| a.area.total_cmp(&b.area).then(a.cap.total_cmp(&b.cap)));
    if options.is_empty() {
        return;
    }
    let mut lowest_cap = options[0].cap;
    let mut kept = vec![Rc::clone(&options[0])];
    for option in options.iter().skip(1) {
        if fuzzy::less(option.cap, lowest_cap) {
            kept.push(Rc::clone(option));
            lowest_cap = option.cap;
        }
    }
    kept.reverse();
    *options = kept;
}

/// The tree with each node's arrival from the driver filled in, as the
/// first pass of `recoverArea`.
fn with_arrivals(node: &BnetPtr, arrival: Fixed) -> BnetPtr {
    let rebuilt = match &node.kind {
        Kind::Load { .. } => Rc::clone(node),
        Kind::Wire { to, layer } => {
            let to = with_arrivals(to, arrival + node.delay);
            Rc::new(Bnet {
                kind: Kind::Wire { to, layer: *layer },
                ..**node
            })
        }
        Kind::Via {
            to,
            layer,
            to_layer,
        } => {
            let to = with_arrivals(to, arrival + node.delay);
            Rc::new(Bnet {
                kind: Kind::Via {
                    to,
                    layer: *layer,
                    to_layer: *to_layer,
                },
                ..**node
            })
        }
        Kind::Buffer { cell, to } => {
            let to = with_arrivals(to, arrival + node.delay);
            Rc::new(Bnet {
                kind: Kind::Buffer { cell: *cell, to },
                ..**node
            })
        }
        Kind::Junction { left, right } => {
            let left = with_arrivals(left, arrival);
            let right = with_arrivals(right, arrival);
            Rc::new(Bnet {
                kind: Kind::Junction { left, right },
                ..**node
            })
        }
    };
    rebuilt.with_arrival(arrival)
}

/// The delay from the tree's top to its most critical load, as
/// `criticalPathDelay`.
fn critical_path_delay(root: &Bnet) -> Fixed {
    fn worst_load_slack(node: &Bnet) -> Fixed {
        match &node.kind {
            Kind::Load { .. } => node.slack,
            Kind::Wire { to, .. } | Kind::Via { to, .. } | Kind::Buffer { to, .. } => {
                worst_load_slack(to)
            }
            Kind::Junction { left, right } => worst_load_slack(left).min(worst_load_slack(right)),
        }
    }
    worst_load_slack(root) - root.slack
}

/// The tree without its buffers, as `stripTreeBuffers`.
fn strip_buffers(node: &BnetPtr) -> BnetPtr {
    match &node.kind {
        Kind::Load { .. } => Rc::clone(node),
        Kind::Buffer { to, .. } => strip_buffers(to),
        Kind::Wire { to, layer } => Rc::new(Bnet {
            kind: Kind::Wire {
                to: strip_buffers(to),
                layer: *layer,
            },
            ..**node
        }),
        Kind::Via {
            to,
            layer,
            to_layer,
        } => Rc::new(Bnet {
            kind: Kind::Via {
                to: strip_buffers(to),
                layer: *layer,
                to_layer: *to_layer,
            },
            ..**node
        }),
        Kind::Junction { left, right } => Rc::new(Bnet {
            kind: Kind::Junction {
                left: strip_buffers(left),
                right: strip_buffers(right),
            },
            ..**node
        }),
    }
}

/// The loads and buffers a new Steiner tree must reach, as `resteiner`
/// gathers them.
fn collect_sinks(node: &BnetPtr, sinks: &mut Vec<BnetPtr>) {
    match &node.kind {
        Kind::Load { .. } | Kind::Buffer { .. } => sinks.push(Rc::clone(node)),
        Kind::Wire { to, .. } | Kind::Via { to, .. } => collect_sinks(to, sinks),
        Kind::Junction { left, right } => {
            collect_sinks(left, sinks);
            collect_sinks(right, sinks);
        }
    }
}

/// The buffers downstream of `driver` that `imported` did not keep as
/// loads: those to remove once the net is rebuffered, as
/// `collectImportedTreeBufferInstances`.
fn collect_imported_buffers(resizer: &Resizer<'_>, driver: PinId, imported: &Bnet) -> Vec<CellId> {
    let kept: rustc_hash::FxHashSet<PinId> = imported.loads().into_iter().collect();
    let netlist = resizer.netlist();
    let mut buffers = Vec::new();
    let mut queue = vec![resizer.db.pins[driver.index()].net];
    while let Some(net) = queue.pop() {
        for load in netlist.loads(net) {
            let PinOwner::Cell { cell, .. } = resizer.db.pins[load.index()].owner else {
                continue;
            };
            let Some((_, lib)) = resizer.link.cell(resizer.db, cell) else {
                continue;
            };
            if !lib.is_buffer() || kept.contains(&load) {
                continue;
            }
            let Some((in_pin, out_pin)) = resizer.buffer_pins(cell) else {
                continue;
            };
            if in_pin != load {
                continue;
            }
            buffers.push(cell);
            queue.push(resizer.db.pins[out_pin.index()].net);
        }
    }
    buffers
}

/// Whether a buffer sits directly on a port's net with nothing else, as
/// `isPortBuffer`.
fn is_port_buffer(resizer: &Resizer<'_>, cell: CellId) -> bool {
    let netlist = resizer.netlist();
    resizer.db.cells[cell.index()].pins.iter().any(|&pin| {
        let net = resizer.db.pins[pin.index()].net;
        let instance_pins = resizer.db.nets[net.index()]
            .pins
            .iter()
            .filter(|&&p| matches!(resizer.db.pins[p.index()].owner, PinOwner::Cell { .. }))
            .count();
        instance_pins == 1 && netlist.has_port(net)
    })
}

/// Whether a buffer may be taken out: a plain standard cell with both its
/// nets free of ports that constraints name, as `canRemoveBuffer`.
pub(crate) fn can_remove_buffer(resizer: &Resizer<'_>, cell: CellId) -> bool {
    let Some((_, lib)) = resizer.link.cell(resizer.db, cell) else {
        return false;
    };
    if !lib.is_buffer() || !resizer.is_editable(cell) {
        return false;
    }
    let Some((in_pin, out_pin)) = resizer.buffer_pins(cell) else {
        return false;
    };
    let netlist = resizer.netlist();
    let in_net = resizer.db.pins[in_pin.index()].net;
    let out_net = resizer.db.pins[out_pin.index()].net;
    let constrained = |net: NetId| {
        let Some(sta) = resizer.sta() else {
            return false;
        };
        resizer.db.nets[net.index()].pins.iter().any(|&p| {
            matches!(resizer.db.pins[p.index()].owner, PinOwner::Io(io)
                if sta.sdc().input_delays.contains_key(&io)
                    || sta.sdc().output_delays.contains_key(&io)
                    || sta.sdc().clock_at(io).is_some())
        })
    };
    let removed = if netlist.has_port(out_net) {
        in_net
    } else {
        out_net
    };
    let between_ports = netlist.has_port(in_net) && netlist.has_port(out_net);
    !constrained(removed) && !between_ports
}

/// Whether a net reaches an output port of the design.
fn has_top_level_output(resizer: &Resizer<'_>, net: NetId) -> bool {
    resizer.db.nets[net.index()].pins.iter().any(|&p| {
        matches!(resizer.db.pins[p.index()].owner, PinOwner::Io(io)
            if matches!(resizer.db.io_pins[io.index()].direction,
                pare_common::db::library::PinDirection::Output | pare_common::db::library::PinDirection::Inout))
    })
}

fn margined(limit: f64, percent: f64) -> f64 {
    limit * (1.0 - percent / 100.0)
}

/// What a buffer presents and can drive, as `BufferSize` with
/// `characterizeBufferLimits`.
fn buffer_size(resizer: &Resizer<'_>, cell: LibCellId) -> Result<BufferSize, RepairError> {
    let lib = resizer.link.liberty.cell(cell);
    let (input, output) = lib
        .buffer_ports()
        .ok_or_else(|| RepairError::MissingPins(lib.name.clone()))?;
    let intrinsic = lib
        .delay_arc_sets_to(output)
        .flat_map(|s| s.arcs.iter())
        .map(|a| a.gate_delay(0.0, 0.0).0)
        .filter(|&d| d > 0.0)
        .fold(0.0, f64::max);
    let cap_limit = resizer
        .link
        .liberty
        .capacitance_limit(lib, &lib.ports[output])
        .map_or(f64::INFINITY, |l| margined(l, CAP_MARGIN));
    let margined_max_cap = cap_limit.min(load_limit_by_driver_slew(resizer, cell));
    Ok(BufferSize {
        cell,
        input_cap: lib.ports[input].capacitance(),
        input_fanout: resizer.link.liberty.fanout_load(lib, &lib.ports[input]),
        input_max_slew: resizer.max_input_slew(lib, &lib.ports[input]),
        output_cap: lib.ports[output].capacitance(),
        area: lib.area,
        intrinsic_delay: Fixed::from_seconds(intrinsic),
        margined_max_cap,
        drive_resistance: lib.drive_resistance(output),
    })
}

/// The load at which a buffer's own slew reaches its limit, by bisection,
/// as `findBufferLoadLimitImpliedByDriverSlew`.
fn load_limit_by_driver_slew(resizer: &Resizer<'_>, cell: LibCellId) -> f64 {
    let lib = resizer.link.liberty.cell(cell);
    let Some((input, output)) = lib.buffer_ports() else {
        return f64::INFINITY;
    };
    let Some(max_slew) = resizer.link.liberty.slew_limit(lib, &lib.ports[output]) else {
        return f64::INFINITY;
    };
    let max_slew = margined(max_slew, SLEW_MARGIN);
    let in_slew = margined(resizer.max_input_slew(lib, &lib.ports[input]), SLEW_MARGIN);
    let objective = |load: f64| -> f64 {
        lib.delay_arc_sets_to(output)
            .flat_map(|s| s.arcs.iter())
            .map(|a| a.gate_delay(in_slew, load).1)
            .fold(f64::NEG_INFINITY, f64::max)
            - max_slew
    };
    let drive = lib.drive_resistance(output);
    if drive == 0.0 {
        return f64::INFINITY;
    }
    let mut cap1 = 0.0;
    let mut cap2 = max_slew / drive * 2.0;
    let tol = 0.01;
    let mut diff1 = objective(cap2);
    let mut iterations = 0;
    while (cap1 - cap2).abs() > cap1.max(cap2) * tol && iterations < 200 {
        iterations += 1;
        if diff1 < 0.0 {
            cap1 = cap2;
            cap2 *= 2.0;
            diff1 = objective(cap2);
        } else {
            let cap3 = f64::midpoint(cap1, cap2);
            let diff2 = objective(cap3);
            if diff2 < 0.0 {
                cap1 = cap3;
            } else {
                cap2 = cap3;
                diff1 = diff2;
            }
        }
    }
    cap1
}

/// The wire a buffer can drive before its load's slew limit is reached, in
/// database units, as `wireLengthLimitImpliedByLoadSlew`.
fn wire_length_limit_by_load_slew(resizer: &Resizer<'_>, size: &BufferSize) -> i64 {
    let rc = resizer.graph.wire_rc();
    let dbu = resizer.db.dbu().get() as f64;
    let max_slew = margined(size.input_max_slew, SLEW_MARGIN);
    let a = rc.resistance * rc.capacitance;
    let b = size
        .input_cap
        .mul_add(rc.resistance, size.drive_resistance * rc.capacitance);
    let c = size.drive_resistance * size.input_cap - max_slew / ELMORE_SKEW_FACTOR;
    let discriminant = (4.0 * a).mul_add(-c, b * b);
    if discriminant < 0.0 || a <= 0.0 {
        return i64::MAX;
    }
    let meters = (-b + discriminant.sqrt()) / (2.0 * a);
    if meters > 1.0 {
        return i64::MAX;
    }
    if meters <= 0.0 {
        return 1;
    }
    (meters * 1e6 * dbu) as i64
}

/// The wire a buffer can drive before its load limit is reached, as
/// `wireLengthLimitImpliedByMaxCap`.
fn wire_length_limit_by_max_cap(resizer: &Resizer<'_>, size: &BufferSize) -> i64 {
    let lib = resizer.link.liberty.cell(size.cell);
    let Some((_, output)) = lib.buffer_ports() else {
        return i64::MAX;
    };
    let Some(limit) = resizer
        .link
        .liberty
        .capacitance_limit(lib, &lib.ports[output])
    else {
        return i64::MAX;
    };
    let rc = resizer.graph.wire_rc();
    if rc.capacitance <= 0.0 {
        return i64::MAX;
    }
    let dbu = resizer.db.dbu().get() as f64;
    let slack = margined(limit, CAP_MARGIN) - size.input_cap;
    ((slack / rc.capacitance) * 1e6 * dbu).max(0.0) as i64
}

/// `meters` in database units, rounded and kept within an `int`, as
/// `Resizer::metersToDbu`.
fn meters_to_dbu(meters: f64, dbu: f64) -> i64 {
    i64::from(((meters * dbu * 1e6).round() as i64 as i32) & i32::MAX)
}

/// The buffers worth inserting for speed: of the repair buffers by rising
/// input capacitance, those no other buffer beats at every load, as
/// `findFastBuffers`.
fn find_fast_buffers(resizer: &Resizer<'_>) -> Vec<LibCellId> {
    let input_cap = |cell: LibCellId| resizer.link.buffer_input_capacitance(cell);
    let mut by_input: Vec<LibCellId> = resizer.buffers.cells.clone();
    by_input.sort_by(|&a, &b| input_cap(a).total_cmp(&input_cap(b)));
    let r_max = resizer
        .link
        .buffer_drive_resistance(resizer.buffers.lowest_drive);
    let mut fast: Vec<LibCellId> = Vec::new();
    for size in by_input {
        if fast.is_empty() {
            fast.push(size);
            continue;
        }
        let back = fast[fast.len() - 1];
        if !outmatched(resizer, size, back, r_max) {
            while let Some(&last) = fast.last() {
                if outmatched(resizer, last, size, r_max) {
                    let _ = fast.pop();
                } else {
                    break;
                }
            }
            fast.push(size);
        }
    }
    fast
}

/// Whether `better` is at least as fast as `worse` at every load either
/// table covers, once its extra input capacitance is charged against it,
/// as `bufferSizeOutmatched`.
fn outmatched(resizer: &Resizer<'_>, worse: LibCellId, better: LibCellId, r_max: f64) -> bool {
    let liberty = resizer.link.liberty;
    let (w, b) = (liberty.cell(worse), liberty.cell(better));
    let (Some((w_in, w_out)), Some((b_in, _))) = (w.buffer_ports(), b.buffer_ports()) else {
        return false;
    };
    let extra_input = (b.ports[b_in].capacitance() - w.ports[w_in].capacitance()).max(0.0);
    let penalty = r_max * extra_input;
    let limit = liberty.capacitance_limit(w, &w.ports[w_out]).unwrap_or(0.0);
    let mut points: Vec<f64> = Vec::new();
    for cell in [w, b] {
        let Some((input, output)) = cell.buffer_ports() else {
            continue;
        };
        for set in cell
            .arc_sets
            .iter()
            .filter(|s| s.role == ArcRole::Combinational && s.from == input && s.to == output)
        {
            for arc in &set.arcs {
                if let Some(table) = &arc.delay {
                    points.extend(table.capacitance_points());
                }
            }
        }
    }
    if points.is_empty() {
        return false;
    }
    points.sort_by(f64::total_cmp);
    points.dedup();
    let cutoff = w.ports[w_in].capacitance().min(b.ports[b_in].capacitance()) * 20.0;
    for p in points {
        if (limit == 0.0 || p <= limit) && p < cutoff {
            let bd = resizer.buffer_delay(better, p);
            let wd = resizer.buffer_delay(worse, p);
            if bd + penalty > wd {
                return false;
            }
        }
    }
    true
}

/// The wire length beyond which a buffer is worth inserting, in database
/// units.
///
/// The shortest, over the repair buffers, at which one buffer driving the
/// wire whole is slower than two each driving half, as `findMaxWireLength`.
pub fn max_wire_length(resizer: &Resizer<'_>) -> i64 {
    let rc = resizer.graph.wire_rc();
    if rc.resistance <= 0.0 {
        return i64::MAX;
    }
    let dbu = resizer.db.dbu().get() as f64;
    let mut shortest = f64::INFINITY;
    for &buffer in &resizer.buffers.cells {
        let meters = buffer_max_wire_length(resizer, buffer);
        shortest = shortest.min(meters);
    }
    if !shortest.is_finite() {
        return i64::MAX;
    }
    (shortest * 1e6 * dbu).round() as i64
}

/// `findMaxWireLength` for one buffer, in meters.
fn buffer_max_wire_length(resizer: &Resizer<'_>, buffer: LibCellId) -> f64 {
    let lib = resizer.link.liberty.cell(buffer);
    let Some((input, output)) = lib.buffer_ports() else {
        return f64::INFINITY;
    };
    let rc = resizer.graph.wire_rc();
    let drive = lib.drive_resistance(output);
    let delay = |length: f64| -> f64 {
        let resistance = length * rc.resistance;
        let capacitance = length * rc.capacitance;
        let mut worst = f64::NEG_INFINITY;
        for set in lib.delay_arc_sets_to(output) {
            for arc in &set.arcs {
                let load_cap = lib.ports[input].capacitance.get(arc.to_edge);
                let model = PiModel {
                    c2: capacitance / 2.0,
                    rpi: resistance,
                    c1: capacitance / 2.0 + load_cap,
                };
                let elmore = resistance * (capacitance / 2.0 + load_cap);
                let result = dmp::gate_delay(
                    arc,
                    &resizer.link.liberty.thresholds,
                    resizer.targets.slews.get(arc.from_edge),
                    model,
                );
                let (wire_delay, _) = result.load_delay_slew(elmore);
                worst = worst.max(result.delay + wire_delay);
            }
        }
        worst
    };
    let diff = |length: f64| delay(length) - 2.0 * delay(length / 2.0);
    let mut low = 0.0;
    let mut high = drive / rc.resistance;
    if !high.is_finite() || high <= 0.0 {
        return f64::INFINITY;
    }
    let tol = 0.01;
    let mut diff_high = diff(high);
    let mut iterations = 0;
    while (low - high).abs() > low.max(high) * tol && iterations < 200 {
        iterations += 1;
        if diff_high < 0.0 {
            low = high;
            high *= 2.0;
            diff_high = diff(high);
        } else {
            let mid = f64::midpoint(low, high);
            if diff(mid) < 0.0 {
                low = mid;
            } else {
                high = mid;
                diff_high = diff(mid);
            }
        }
    }
    low
}
