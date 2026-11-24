//! The timing pass: buffering options built bottom up over the tree, the best at the driver kept.

use super::{
    AreaRecoveryFailed, DriverPoint, DriverTiming, ELMORE_SKEW_FACTOR, JunctionSides, Rebuffer,
    SLEW_MARGIN, Topology, buffer_size, margined, timing_through, wire,
};
use crate::bnet::{Bnet, BnetPtr, BufferSize, Fixed, Kind, Metrics, SlackEdges};
use crate::fuzzy;
use crate::resizer::{Resizer, buffer_delays};
use crate::sta::{Sta, Via};
use pare_common::convert::SaturatingFrom;
use pare_common::db::indices::{LayerId, PinId};
use pare_common::db::liberty::{BufferCell, EdgeValues, MinMax};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::units::{CellArea, Farads, Meters, Seconds};
use std::rc::Rc;

impl Rebuffer {
    /// Gives each load the slack of its worst path measured at the driver,
    /// and remembers the driver's arrival paths, as `annotateLoadSlacks`;
    /// then times every node up from the loads, as `annotateTiming`.
    pub(super) fn annotate_load_slacks(
        &mut self,
        resizer: &Resizer<'_, Sta>,
        tree: &BnetPtr,
        driver: PinId,
    ) -> BnetPtr {
        let mut arrival_paths: EdgeValues<Option<DriverPoint>> = EdgeValues::default();
        let annotated = self.annotate_loads(resizer, tree, driver, &mut arrival_paths);
        if let Some(d) = &mut self.driver {
            d.arrival_paths = arrival_paths;
        }
        self.retime(resizer, &annotated)
    }

    fn annotate_loads(
        &self,
        resizer: &Resizer<'_, Sta>,
        node: &BnetPtr,
        driver: PinId,
        arrival_paths: &mut EdgeValues<Option<DriverPoint>>,
    ) -> BnetPtr {
        match &node.kind {
            Kind::Load { pin } => {
                let sta = resizer.sta();
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
                if arrival_paths[edge].is_none() {
                    let from = (at_driver > 0)
                        .then(|| path.points[at_driver - 1])
                        .and_then(|prev| match driver_point.via {
                            Via::Arc { set } => Some((prev.pin, prev.edge, set)),
                            _ => None,
                        });
                    arrival_paths[edge] = Some(DriverPoint {
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
            Kind::Buffer { buffer, to } => {
                let to = self.annotate_loads(resizer, to, driver, arrival_paths);
                self.buffer_node(resizer, node.at, *buffer, to)
            }
        }
    }

    /// Rebuilds a tree so every node's slack, capacitance and delay follow
    /// from its children, as `annotateTiming`.
    fn retime(&self, resizer: &Resizer<'_, Sta>, node: &BnetPtr) -> BnetPtr {
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
            Kind::Buffer { buffer, to } => {
                self.buffer_node(resizer, node.at, *buffer, self.retime(resizer, to))
            }
        }
    }

    /// A buffer node of `buffer` at `at` driving `to`, timed at the load.
    pub(super) fn buffer_node(
        &self,
        resizer: &Resizer<'_, Sta>,
        at: Point<Dbu>,
        buffer: BufferCell,
        to: BnetPtr,
    ) -> BnetPtr {
        let size = self
            .sizes
            .iter()
            .find(|s| s.buffer == buffer)
            .copied()
            .unwrap_or_else(|| buffer_size(resizer, buffer));
        let delay = buffer_delay(resizer, &size, to.slack_edges, to.cap + size.output_cap);
        Bnet::buffer(at, &size, to, delay)
    }

    /// The driver's gate delay, slack correction and slew for a tree, as
    /// `drvrPinTiming`: the gate timed again at the tree's load, and the
    /// correction the arrival at the driver would take from that.
    pub(super) fn driver_timing(&self, resizer: &Resizer<'_, Sta>, tree: &Bnet) -> DriverTiming {
        let (Some(driver), Some(edges)) = (&self.driver, tree.slack_edges) else {
            return DriverTiming::NONE;
        };
        let cell = resizer.link.liberty.cell(driver.cell);
        let load = tree.cap + cell.ports[driver.port].capacitance();
        let mut delay = Seconds::ZERO;
        let mut correction = Seconds::INFINITY;
        let mut slew = Seconds::ZERO;
        for &edge in edges.edges() {
            let Some(point) = driver.arrival_paths[edge] else {
                continue;
            };
            let Some(through) = timing_through(resizer, cell, &point, edge, load) else {
                correction = correction.min(Seconds::ZERO);
                continue;
            };
            delay = delay.max(through.delay);
            correction = correction.min(through.correction);
            slew = slew.max(through.slew);
        }
        if correction == Seconds::INFINITY {
            correction = Seconds::ZERO;
        }
        DriverTiming {
            delay,
            correction,
            slew,
        }
    }

    /// The tree's slack as the driver sees it.
    pub(super) fn slack_at_driver(&self, resizer: &Resizer<'_, Sta>, tree: &Bnet) -> Fixed {
        tree.slack + Fixed::from_seconds(self.driver_timing(resizer, tree).correction)
    }

    /// Whether the driver's slew keeps every load of the tree within its
    /// limit by the Elmore estimate, as `loadSlewSatisfactory`.
    fn load_slew_satisfactory(&self, resizer: &Resizer<'_, Sta>, tree: &Bnet) -> bool {
        let Some(driver) = &self.driver else {
            return true;
        };
        let rc = resizer.graph.wire_rc();
        let r_drvr = resizer
            .link
            .liberty
            .cell(driver.cell)
            .drive_resistance(driver.port);
        let meters = Meters::from_dbu(tree.max_load_wire_length(), resizer.db.dbu());
        let load_slew = (r_drvr + meters * rc.resistance) * tree.cap * ELMORE_SKEW_FACTOR;
        load_slew <= margined(tree.max_load_slew, SLEW_MARGIN)
    }

    /// The slack of a candidate tree at the driver, or none when the driver
    /// could not drive it, as `evaluateOption`.
    pub(super) fn evaluate_option(
        &mut self,
        resizer: &Resizer<'_, Sta>,
        option: &Bnet,
    ) -> Option<Fixed> {
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
    pub(super) fn buffer_for_timing(
        &mut self,
        resizer: &Resizer<'_, Sta>,
        tree: &BnetPtr,
        topology: Topology,
    ) -> Option<BnetPtr> {
        let options = self.timing_options(resizer, tree, topology);
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
        resizer: &Resizer<'_, Sta>,
        node: &BnetPtr,
        topology: Topology,
    ) -> Vec<BnetPtr> {
        match &node.kind {
            Kind::Load { .. } => vec![Rc::clone(node)],
            Kind::Junction { left, right } => {
                let left_options = self.timing_options(resizer, left, topology);
                let right_options = self.timing_options(resizer, right, topology);
                let sides = JunctionSides {
                    left: &left_options,
                    right: &right_options,
                };
                self.junction_options(resizer, node, sides, topology)
            }
            Kind::Wire { to, .. } | Kind::Via { to, .. } | Kind::Buffer { to, .. } => {
                let under = to.under_wires_and_buffers();
                let options = self.timing_options(resizer, &under, topology);
                self.wire_options(resizer, node, &under, node.wire_layer_below(), options)
            }
        }
    }

    /// Options for a wire on `layer` from `node` down to `under`: long
    /// wires are walked in steps of the layer's wire length limit with
    /// buffers offered at each step, short ones get buffers at either end.
    fn wire_options(
        &self,
        resizer: &Resizer<'_, Sta>,
        node: &Bnet,
        under: &Bnet,
        layer: Option<LayerId>,
        mut options: Vec<BnetPtr>,
    ) -> Vec<BnetPtr> {
        let mut location = under.at;
        let full_length = node.at.manhattan(location);
        let step = self.wire_length_step_on(resizer, layer);
        let Some(step) = step.filter(|&step| full_length > step / 2) else {
            return self.short_wire_options(resizer, node, layer, options, full_length);
        };
        self.insert_buffer_options(resizer, &mut options, full_length.min(step));
        while location != node.at {
            let last_valid = options.clone();
            let mut dx = node.at.x - location.x;
            let mut dy = node.at.y - location.y;
            if dx.abs() + dy.abs() >= step {
                let ratio = dx.abs().to_f64() / (dx.abs() + dy.abs()).to_f64();
                let dx_abs = Dbu::new(i64::saturating_from(ratio * step.to_f64())).min(step);
                let dy_abs = step - dx_abs;
                dx = if dx > Dbu::ZERO { dx_abs } else { -dx_abs };
                dy = if dy > Dbu::ZERO { dy_abs } else { -dy_abs };
            }
            location = Point::new(location.x + dx, location.y + dy);
            let remaining = node.at.manhattan(location);
            options = options
                .iter()
                .map(|o| wire(resizer, location, layer, Rc::clone(o)))
                .collect();
            self.insert_buffer_options(resizer, &mut options, remaining.min(step));
            if options.is_empty() {
                let net = self
                    .driver
                    .as_ref()
                    .map_or_else(|| "a net".to_owned(), |d| resizer.db.pin_name(d.pin));
                log::warn!(
                    "rebuffering {net}: no option can drive the wire step; keeping the last that could"
                );
                options = last_valid;
                break;
            }
        }
        options
    }

    /// Options for a wire no longer than half a step: buffers at its near
    /// end, or, when none can go there, at its far end.
    fn short_wire_options(
        &self,
        resizer: &Resizer<'_, Sta>,
        node: &Bnet,
        layer: Option<LayerId>,
        options: Vec<BnetPtr>,
        full_length: Dbu,
    ) -> Vec<BnetPtr> {
        let mut near: Vec<BnetPtr> = options
            .iter()
            .map(|o| wire(resizer, node.at, layer, Rc::clone(o)))
            .collect();
        self.insert_buffer_options(resizer, &mut near, Dbu::ZERO);
        if near.is_empty() {
            let mut far = options;
            self.insert_buffer_options(resizer, &mut far, full_length);
            near = far
                .iter()
                .map(|o| wire(resizer, node.at, layer, Rc::clone(o)))
                .collect();
            self.insert_buffer_options(resizer, &mut near, Dbu::ZERO);
        }
        near
    }

    /// The options for a junction: each side's options paired off from the
    /// most slack down, keeping the pairs that lower the capacitance, each
    /// pair rewritten where `topology` allows and that pays.
    pub(super) fn junction_options(
        &self,
        resizer: &Resizer<'_, Sta>,
        node: &Bnet,
        sides: JunctionSides<'_>,
        topology: Topology,
    ) -> Vec<BnetPtr> {
        let JunctionSides { left, right } = sides;
        let fanout_limit = self
            .driver
            .as_ref()
            .map_or(f64::INFINITY, |d| d.fanout_limit);
        let mut options = Vec::with_capacity(left.len().max(right.len()));
        let mut best_cap = Farads::INFINITY;
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
            let rewritten = match topology {
                Topology::Rewritable => {
                    self.rewrite_topology(resizer, node, &left[li], &right[ri], best_cap)
                }
                Topology::Kept => None,
            };
            let junction = rewritten.unwrap_or_else(|| {
                Bnet::junction(node.at, Rc::clone(&left[li]), Rc::clone(&right[ri]))
            });
            if junction.fanout <= fanout_limit {
                best_cap = junction.cap;
                options.push(junction);
            }
            let Some(next) = sides.next_cheaper_pair((li, ri), best_cap) else {
                break;
            };
            (li, ri) = next;
        }
        options.reverse();
        options
    }

    /// Tries pulling the less critical side under a buffer with the less
    /// critical half of the critical side, as `attemptTopologyRewrite`.
    fn rewrite_topology(
        &self,
        resizer: &Resizer<'_, Sta>,
        node: &Bnet,
        left: &BnetPtr,
        right: &BnetPtr,
        best_cap: Farads,
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
            if buffer_slack >= junction_slack && can_drive(resizer, size, &junction1, Dbu::ZERO) {
                let buffer = Bnet::buffer(node.at, size, Rc::clone(&junction1), delay);
                return Some(Bnet::junction(node.at, buffer, in3));
            }
        }
        None
    }

    /// Offers every buffer size on top of the options, keeping the options
    /// that trade capacitance for slack, as `insertBufferOptions` buffering
    /// for timing.
    fn insert_buffer_options(
        &self,
        resizer: &Resizer<'_, Sta>,
        options: &mut Vec<BnetPtr>,
        next_segment: Dbu,
    ) {
        *options = self
            .buffered_frontier(resizer, options, next_segment, None)
            .kept;
    }

    /// Offers every buffer size on top of the options, keeping the options
    /// that trade area for slack at no less than `threshold`, as
    /// `insertBufferOptions` recovering area; fails, as it does with RSZ
    /// 501, when no option reproduces what `exemplar` timed.
    pub(super) fn insert_area_options(
        &self,
        resizer: &Resizer<'_, Sta>,
        options: &mut Vec<BnetPtr>,
        next_segment: Dbu,
        (threshold, exemplar): (Fixed, &Bnet),
    ) -> Result<(), AreaRecoveryFailed> {
        if options.is_empty() {
            return Ok(());
        }
        let area = Some((threshold, exemplar));
        let mut frontier = self.buffered_frontier(resizer, options, next_segment, area);
        if !frontier.assured
            && let Some(envelope) = &frontier.envelope
        {
            let option =
                assured_fallback(resizer, &self.sizes, options, threshold, exemplar, envelope)
                    .ok_or(AreaRecoveryFailed)?;
            insert_assured(&mut frontier.kept, option);
        }
        *options = frontier.kept;
        Ok(())
    }
}

/// What a pass of `insert_buffer_options` keeps: the options so far, the
/// best slack and area among them, and whether one meets the envelope of
/// the solution area recovery must reproduce.
pub(super) struct Frontier {
    /// The slack every kept option must reach when recovering area.
    area_threshold: Option<Fixed>,
    pub(super) envelope: Option<Metrics>,
    assured: bool,
    pub(super) best_area: CellArea,
    pub(super) best_slack: Fixed,
    pub(super) kept: Vec<BnetPtr>,
}

impl Frontier {
    pub(super) fn new(area: Option<(Fixed, &Bnet)>, capacity: usize) -> Self {
        Self {
            area_threshold: area.map(|(threshold, _)| threshold),
            envelope: area.map(|(threshold, exemplar)| exemplar.metrics().with_slack(threshold)),
            assured: area.is_none(),
            best_area: CellArea::INFINITY,
            best_slack: -Fixed::INF,
            kept: Vec::with_capacity(capacity),
        }
    }

    /// Whether an option of `slack` and `area` is worth more than those kept.
    fn improves(&self, slack: Fixed, area: CellArea) -> bool {
        match self.area_threshold {
            Some(threshold) => fuzzy::less(area, self.best_area) && slack >= threshold,
            None => slack > self.best_slack,
        }
    }

    /// Keeps `option` as it is if it improves on those kept and the strongest
    /// buffer could drive it.
    pub(super) fn offer(
        &mut self,
        resizer: &Resizer<'_, Sta>,
        strong: &BufferSize,
        option: &BnetPtr,
        next_segment: Dbu,
    ) {
        if self.improves(option.slack, option.area)
            && can_drive(resizer, strong, option, next_segment)
        {
            self.best_slack = option.slack;
            self.best_area = option.area;
            self.push(Rc::clone(option));
        }
    }

    /// The option `size` buffers best, with that buffer's delay, raising
    /// the best slack and area to the buffered option's.
    pub(super) fn best_buffered(
        &mut self,
        resizer: &Resizer<'_, Sta>,
        size: &BufferSize,
        options: &[BnetPtr],
    ) -> Option<(BnetPtr, Fixed)> {
        let mut best = None;
        for option in options {
            if !self.improves(option.slack - size.intrinsic_delay, option.area + size.area)
                || !can_drive(resizer, size, option, Dbu::ZERO)
            {
                continue;
            }
            let delay = buffer_delay(
                resizer,
                size,
                option.slack_edges,
                option.cap + size.output_cap,
            );
            let slack = option.slack - delay;
            let better = match self.area_threshold {
                Some(threshold) => slack >= threshold,
                None => slack > self.best_slack,
            };
            if better {
                best = Some((Rc::clone(option), delay));
                self.best_slack = slack;
                self.best_area = option.area + size.area;
            }
        }
        best
    }

    pub(super) fn push(&mut self, option: BnetPtr) {
        if let Some(envelope) = &self.envelope
            && !self.assured
            && option.fits(envelope)
        {
            self.assured = true;
        }
        self.kept.push(option);
    }
}

/// An option that meets `envelope` when none kept does: the cheapest
/// buffered by the exemplar's buffer at no less than `threshold`, or for an
/// unbuffered exemplar the first option within it.
fn assured_fallback(
    resizer: &Resizer<'_, Sta>,
    sizes: &[BufferSize],
    options: &[BnetPtr],
    threshold: Fixed,
    exemplar: &Bnet,
    envelope: &Metrics,
) -> Option<BnetPtr> {
    let Kind::Buffer { buffer, .. } = &exemplar.kind else {
        return options.iter().find(|o| o.fits(envelope)).map(Rc::clone);
    };
    let size = sizes.iter().find(|s| s.buffer == *buffer).copied()?;
    let mut best: Option<(CellArea, BnetPtr)> = None;
    for option in options {
        if best.as_ref().is_some_and(|(a, _)| option.area >= *a) {
            continue;
        }
        let delay = buffer_delay(
            resizer,
            &size,
            option.slack_edges,
            option.cap + size.output_cap,
        );
        if can_drive(resizer, &size, option, Dbu::ZERO) && option.slack - delay >= threshold {
            let buffered = Bnet::buffer(option.at, &size, Rc::clone(option), delay);
            if buffered.fits(envelope) {
                best = Some((option.area, buffered));
            }
        }
    }
    best.map(|(_, b)| b)
}

/// The worst delay of `size` over `edges` at `load_cap`, as
/// `Rebuffer::bufferDelay`; zero without edges.
pub(super) fn buffer_delay(
    resizer: &Resizer<'_, Sta>,
    size: &BufferSize,
    edges: Option<SlackEdges>,
    load_cap: Farads,
) -> Fixed {
    let Some(edges) = edges else {
        return Fixed::ZERO;
    };
    let delays = buffer_delays(
        resizer.link.liberty,
        size.buffer,
        &resizer.targets.slews,
        load_cap,
    );
    let mut worst = Fixed::ZERO;
    for &edge in edges.edges() {
        worst = worst.max(Fixed::from_seconds(delays.get(edge)));
    }
    worst
}

/// Whether `size` can drive `tree` plus `extra_wire` of wire within its
/// slew and load limits, as `bufferSizeCanDriveLoad`.
fn can_drive(resizer: &Resizer<'_, Sta>, size: &BufferSize, tree: &Bnet, extra_wire: Dbu) -> bool {
    let rc = resizer.graph.wire_rc();
    let dbu = resizer.db.dbu();
    let extra_cap = Meters::from_dbu(extra_wire, dbu) * rc.capacitance + size.output_cap;
    let wire_meters = Meters::from_dbu(tree.max_load_wire_length() + extra_wire, dbu);
    let load_slew = (size.drive_resistance + wire_meters * rc.resistance)
        * (tree.cap + extra_cap)
        * ELMORE_SKEW_FACTOR;
    load_slew <= margined(tree.max_load_slew, SLEW_MARGIN)
        && tree.cap + extra_cap <= size.margined_max_cap
}

/// Keeps an option in a list sorted by capacitance, as `insertAssuredOption`.
pub(super) fn insert_assured(options: &mut Vec<BnetPtr>, assured: BnetPtr) {
    let at = options
        .iter()
        .position(|o| o.cap >= assured.cap)
        .unwrap_or(options.len());
    options.insert(at, assured);
}

/// Keeps, of options sorted by area then capacitance, those whose
/// capacitance beats every lower-area one, leaving them sorted by
/// capacitance, as `pruneCapVsAreaOptions`.
pub(super) fn prune_cap_against_area(options: &mut Vec<BnetPtr>) {
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
pub(super) fn with_arrivals(node: &BnetPtr, arrival: Fixed) -> BnetPtr {
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
        Kind::Buffer { buffer, to } => {
            let to = with_arrivals(to, arrival + node.delay);
            Rc::new(Bnet {
                kind: Kind::Buffer {
                    buffer: *buffer,
                    to,
                },
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
pub(super) fn critical_path_delay(root: &Bnet) -> Fixed {
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
