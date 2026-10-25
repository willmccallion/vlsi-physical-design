//! `repair_design` as `OpenROAD`'s resizer does it.
//!
//! From the outputs back to the inputs, every driver whose slew, load or
//! wire breaks a limit is resized to the smallest size that meets it, and
//! where sizing is not enough, repeaters are inserted along the net's
//! Steiner tree, splitting wires and isolating branches until the Elmore
//! estimate of every load's slew is within its limit.

use crate::bnet::{self, BnetPtr, Kind};
use crate::resizer::{RepairError, Resizer, gate_slew};
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{NetId, PinId};
use pare_common::db::liberty::{ArcRole, LibCellId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use std::collections::HashSet;

/// How far short of the computed split a repeater is placed, as
/// `OpenROAD`'s `length_margin`.
const LENGTH_MARGIN: f64 = 0.05;

/// What repair found and did.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RepairSummary {
    /// Drivers whose own or whose loads' slew broke a limit.
    pub slew_violations: usize,
    /// Drivers whose load broke their capacitance limit.
    pub cap_violations: usize,
    /// Nets with a wire over the length limit.
    pub length_violations: usize,
    /// Nets changed.
    pub repaired_nets: usize,
    /// Instances given another size.
    pub resized: usize,
    /// Repeaters inserted.
    pub inserted: usize,
}

/// What decides where a wire is split for a repeater.
#[derive(Clone, Copy, Debug)]
struct SplitInput {
    length: i64,
    wire_length: i64,
    wire_length_ref: i64,
    load_cap: f64,
    ref_cap: f64,
    max_cap: f64,
    load_slew: f64,
    max_load_slew_margined: f64,
    r_drvr: f64,
}

/// What a repaired branch presents to its parent.
#[derive(Clone, Debug)]
struct Branch {
    cap: f64,
    fanout: f64,
    max_load_slew: f64,
    wire_length: i64,
    loads: Vec<PinId>,
}

/// The design repair in progress.
#[derive(Debug)]
pub struct Repairer<'r, 'a> {
    resizer: &'r mut Resizer<'a>,
    summary: RepairSummary,
    load_slew_violation_drivers: HashSet<PinId>,
}

impl<'r, 'a> Repairer<'r, 'a> {
    /// Prepares a design repair over the session.
    pub fn new(resizer: &'r mut Resizer<'a>) -> Self {
        Self {
            resizer,
            summary: RepairSummary::default(),
            load_slew_violation_drivers: HashSet::new(),
        }
    }

    /// Repairs every driver from the outputs back to the inputs, as
    /// `RepairDesign::repairDesign`, and returns what it did.
    pub fn repair_design(mut self) -> Result<RepairSummary, RepairError> {
        self.annotate_violating_loads();
        let drivers = {
            let netlist = self.resizer.netlist();
            self.resizer.graph.drivers_by_level(&netlist)
        };
        for driver in drivers.into_iter().rev() {
            self.repair_driver(driver)?;
        }
        self.resizer.clear_slew_annotations();
        Ok(self.summary)
    }

    /// Holds every violating input slew at its limit so the repair of one
    /// driver does not upsize the next, as `OpenROAD`'s forward pass.
    fn annotate_violating_loads(&mut self) {
        let netlist = self.resizer.netlist();
        let loads = self.resizer.graph.loads_by_level(&netlist);
        let mut annotations = Vec::new();
        for load in loads {
            let Some(port) = netlist.lib_port(load) else {
                continue;
            };
            let limit = self.resizer.max_input_slew(port);
            if self.resizer.graph.slew(load).max() > limit {
                annotations.push((load, limit));
                let net = self.resizer.db.pins[load.index()].net;
                self.load_slew_violation_drivers
                    .extend(netlist.drivers(net));
            }
        }
        for (load, limit) in annotations {
            self.resizer.graph.annotate_slew(load, limit);
        }
    }

    fn repair_driver(&mut self, driver: PinId) -> Result<(), RepairError> {
        let (net, skip) = {
            let netlist = self.resizer.netlist();
            let net = self.resizer.db.pins[driver.index()].net;
            let skip = netlist.is_special(net)
                || self.resizer.clocks.is_clock(driver)
                || self.resizer.constants.is_constant(driver)
                || (!netlist.has_instance_pins(net) && !netlist.has_port(net));
            (net, skip)
        };
        if skip {
            return Ok(());
        }
        self.repair_net(net, driver)
    }

    /// `RepairDesign::repairNet` for one driver.
    fn repair_net(&mut self, net: NetId, driver: PinId) -> Result<(), RepairError> {
        let mut repaired = false;
        let mut max_cap = f64::INFINITY;
        let mut repair_cap = false;
        let mut repair_load_slew = false;

        let tristate = self.resizer.is_tristate_driver(driver);
        let (slew, limit) = self.driver_slew_check(driver);
        let mut slew_violation = false;
        if let Some(limit) = limit
            && slew > limit
        {
            slew_violation = true;
            if self.repair_driver_slew(driver)? {
                self.summary.resized += 1;
            }
            let (slew_after, _) = self.driver_slew_check(driver);
            if slew_after > limit
                && let Some(cap) = self.slew_load_cap(driver, limit)
            {
                max_cap = cap;
                repair_cap = true;
            }
        }
        if !tristate
            && (self.loads_violate_slew(driver)
                || self.load_slew_violation_drivers.contains(&driver))
        {
            slew_violation = true;
            repair_load_slew = true;
        }
        if slew_violation {
            self.summary.slew_violations += 1;
        }

        if !tristate {
            let cap = self.resizer.load_cap(driver);
            if let Some(limit) = self.resizer.cap_limit(driver) {
                let margined = self.resizer.margined_cap(limit);
                if cap > margined {
                    max_cap = margined;
                    repair_cap = true;
                    self.summary.cap_violations += 1;
                }
            }
        }

        if !tristate {
            let tree = {
                let netlist = self.resizer.netlist();
                self.resizer.graph.parasitics(net).and_then(|p| {
                    bnet::from_parasitics(
                        &netlist,
                        p,
                        driver,
                        &self.resizer.graph.wire_rc(),
                        self.resizer,
                    )
                })
            };
            if let Some(tree) = tree {
                let wire_length = tree.max_load_wire_length();
                let max_length = self.resizer.options.max_wire_length;
                let repair_wire = max_length > 0 && wire_length > max_length;
                if repair_wire {
                    self.summary.length_violations += 1;
                }
                if repair_cap || repair_load_slew || repair_wire {
                    repaired = true;
                    let _ = self.repair_tree(&tree, driver, max_cap)?;
                }
            }
        }
        if repaired {
            self.summary.repaired_nets += 1;
        }
        Ok(())
    }

    /// The driver's slew and its margined limit, if one applies.
    fn driver_slew_check(&self, driver: PinId) -> (f64, Option<f64>) {
        let slew = self.resizer.graph.slew(driver).max();
        let limit = self
            .resizer
            .driver_slew_limit(driver)
            .map(|l| self.resizer.margined_slew(l));
        (slew, limit)
    }

    /// Whether any load of the driver's net has a slew over its limit, as
    /// `Resizer::checkLoadSlews`.
    fn loads_violate_slew(&self, driver: PinId) -> bool {
        let netlist = self.resizer.netlist();
        let net = self.resizer.db.pins[driver.index()].net;
        netlist.loads(net).filter(|&p| p != driver).any(|load| {
            let Some(port) = netlist.lib_port(load) else {
                return false;
            };
            let slew = self.resizer.graph.slew(load).max();
            match port.max_transition.filter(|&l| l != 0.0) {
                Some(limit) => slew > self.resizer.margined_slew(limit),
                None => self
                    .resizer
                    .link
                    .liberty
                    .defaults
                    .max_transition
                    .is_some_and(|limit| slew > limit),
            }
        })
    }

    /// Picks the smallest size of the driver's cell that meets its slew
    /// limit at its load, or the least violating one, as
    /// `RepairDesign::repairDriverSlew`; returns whether the cell changed.
    fn repair_driver_slew(&mut self, driver: PinId) -> Result<bool, RepairError> {
        let (cell, lib_id, port_name, load_cap, candidates) = {
            let PinOwner::Cell { cell, .. } = self.resizer.db.pins[driver.index()].owner else {
                return Ok(false);
            };
            let Some((lib_id, lib)) = self.resizer.link.cell(self.resizer.db, cell) else {
                return Ok(false);
            };
            if !self.resizer.is_editable(cell) {
                return Ok(false);
            }
            let Some((_, port)) = self.resizer.link.pin_port(self.resizer.db, driver) else {
                return Ok(false);
            };
            (
                cell,
                lib_id,
                lib.ports[port].name.clone(),
                self.resizer.load_cap(driver),
                self.resizer.link.swappable_cells(lib_id),
            )
        };
        if candidates.is_empty() {
            return Ok(false);
        }
        let input_slews = self.resizer.input_slews(cell);
        let mut sizes: Vec<(f64, f64, LibCellId)> = Vec::new();
        for size in candidates {
            let size_cell = self.resizer.link.liberty.cell(size);
            let Some(port) = size_cell.port(&port_name) else {
                continue;
            };
            let mut violation = 0.0;
            if let Some(limit) = self.resizer.link.liberty.slew_limit(&size_cell.ports[port]) {
                let limit = self.resizer.margined_slew(limit);
                for set in size_cell.arc_sets.iter().filter(|s| s.to == port) {
                    if !matches!(
                        set.role,
                        ArcRole::Combinational
                            | ArcRole::RegClkToQ
                            | ArcRole::RegSetClr
                            | ArcRole::LatchEnToQ
                    ) {
                        continue;
                    }
                    let from_name = &size_cell.ports[set.from].name;
                    for arc in &set.arcs {
                        let in_slew = input_slews.iter().find(|(n, _)| n == from_name).map_or(
                            0.0,
                            |(_, s)| {
                                if set.role == ArcRole::RegClkToQ {
                                    0.0
                                } else {
                                    s.get(arc.from_edge)
                                }
                            },
                        );
                        let (_, arc_slew) = arc.gate_delay(in_slew, load_cap);
                        if arc_slew > limit {
                            violation = f64::max(violation, arc_slew - limit);
                        }
                    }
                }
            }
            sizes.push((violation, size_cell.area, size));
        }
        sizes.sort_by(|a, b| {
            if a.0 == 0.0 && b.0 == 0.0 {
                a.1.total_cmp(&b.1)
            } else {
                a.0.total_cmp(&b.0)
            }
        });
        let Some(&(_, _, selected)) = sizes.first() else {
            return Ok(false);
        };
        if selected == lib_id {
            return Ok(false);
        }
        self.resizer.replace_cell(cell, selected)
    }

    /// The load at which the driver's slew would equal `slew`, by binary
    /// search over the gate tables at the target input slews, as
    /// `RepairDesign::findSlewLoadCap`.
    fn slew_load_cap(&self, driver: PinId, slew: f64) -> Option<f64> {
        let (lib_id, port) = self.resizer.link.pin_port(self.resizer.db, driver)?;
        let cell = self.resizer.link.liberty.cell(lib_id);
        let drive_res = cell.drive_resistance(port);
        if drive_res == 0.0 {
            return Some(f64::INFINITY);
        }
        let slews = &self.resizer.targets.slews;
        let diff = |cap: f64| gate_slew(cell, port, slews, cap) - slew;
        let mut cap1 = 0.0;
        let mut cap2 = slew / drive_res * 2.0;
        let tol = 0.01;
        let mut diff1 = diff(cap2);
        let mut iterations = 0;
        while (cap1 - cap2).abs() > cap1.max(cap2) * tol && iterations < 200 {
            iterations += 1;
            if diff1 < 0.0 {
                cap1 = cap2;
                cap2 *= 2.0;
                diff1 = diff(cap2);
            } else {
                let cap3 = f64::midpoint(cap1, cap2);
                let diff2 = diff(cap3);
                if diff2 < 0.0 {
                    cap1 = cap3;
                } else {
                    cap2 = cap3;
                    diff1 = diff2;
                }
            }
        }
        Some(cap1)
    }

    /// Repairs a buffered tree bottom up, as `RepairDesign::repairNet` over
    /// a `BufferedNet`.
    fn repair_tree(
        &mut self,
        tree: &BnetPtr,
        driver: PinId,
        max_cap: f64,
    ) -> Result<Branch, RepairError> {
        let own = self
            .resizer
            .link
            .pin_port(self.resizer.db, driver)
            .map_or(0.0, |(lib, port)| {
                self.resizer.link.liberty.cell(lib).drive_resistance(port)
            });
        let r_drvr = own.max(self.resizer.r_strongest_buffer);
        self.repair_branch(tree, driver, max_cap, r_drvr)
    }

    fn repair_branch(
        &mut self,
        tree: &BnetPtr,
        driver: PinId,
        max_cap: f64,
        r_drvr: f64,
    ) -> Result<Branch, RepairError> {
        match &tree.kind {
            Kind::Load { pin } => Ok(self.load_branch(*pin)),
            Kind::Junction { left, right } => {
                self.repair_junction(tree.at, left, right, driver, max_cap, r_drvr)
            }
            Kind::Wire { to } | Kind::Buffer { to, .. } => {
                self.repair_wire(tree.at, to, driver, max_cap, r_drvr)
            }
        }
    }

    /// A load: its pin's capacitance, fanout load and slew limit, as
    /// `OpenROAD`'s `BufferedNet` of type load.
    fn load_branch(&self, pin: PinId) -> Branch {
        let (cap, fanout, max_load_slew) = bnet::LoadModel::load(self.resizer, pin);
        Branch {
            cap,
            fanout,
            max_load_slew,
            wire_length: 0,
            loads: vec![pin],
        }
    }

    /// `RepairDesign::repairNetJunc`.
    // The slew estimates keep OpenROAD's grouping so they can be checked.
    #[allow(clippy::suboptimal_flops)]
    fn repair_junction(
        &mut self,
        at: Point<Dbu>,
        left: &BnetPtr,
        right: &BnetPtr,
        driver: PinId,
        max_cap: f64,
        r_drvr: f64,
    ) -> Result<Branch, RepairError> {
        let mut left = self.repair_branch(left, driver, max_cap, r_drvr)?;
        let mut right = self.repair_branch(right, driver, max_cap, r_drvr)?;
        let load_cap = left.cap + right.cap;
        let max_load_slew = left.max_load_slew.min(right.max_load_slew);
        let buffer_cell =
            self.resizer
                .find_target_cell(self.resizer.buffers.lowest_drive, load_cap, false);
        let factor = self.resizer.slew_rc_factor;

        let mut repeater_left = false;
        let mut repeater_right = false;
        let slew_violation = r_drvr * load_cap * factor > self.resizer.margined_slew(max_load_slew);
        if slew_violation {
            let slack_left =
                self.resizer.margined_slew(left.max_load_slew) - r_drvr * left.cap * factor;
            let slack_right =
                self.resizer.margined_slew(right.max_load_slew) - r_drvr * right.cap * factor;
            if slack_left < slack_right {
                repeater_left = true;
            } else {
                repeater_right = true;
            }
        }
        let cap_violation = load_cap > max_cap;
        if cap_violation {
            if left.cap > right.cap {
                repeater_left = true;
            } else {
                repeater_right = true;
            }
        }
        let max_length = self.resizer.options.max_wire_length;
        let length_violation = max_length > 0 && left.wire_length + right.wire_length > max_length;
        if length_violation {
            if left.wire_length > right.wire_length {
                repeater_left = true;
            } else {
                repeater_right = true;
            }
        }
        let reason = if length_violation {
            "max_length"
        } else if cap_violation {
            "max_cap"
        } else {
            "load_slew"
        };
        if repeater_left {
            let _ = self.make_repeater(reason, at, buffer_cell, &mut left)?;
            left.wire_length = 0;
        }
        if repeater_right {
            let _ = self.make_repeater(reason, at, buffer_cell, &mut right)?;
            right.wire_length = 0;
        }
        let mut loads = left.loads;
        loads.extend(right.loads);
        Ok(Branch {
            cap: left.cap + right.cap,
            fanout: left.fanout + right.fanout,
            max_load_slew: left.max_load_slew.min(right.max_load_slew),
            wire_length: left.wire_length.max(right.wire_length),
            loads,
        })
    }

    /// `RepairDesign::repairNetWire`: backs up from the far end of a wire
    /// toward `from`, inserting repeaters where length, load or slew would
    /// break a limit.
    // The slew estimates keep OpenROAD's grouping so they can be checked.
    #[allow(clippy::suboptimal_flops)]
    fn repair_wire(
        &mut self,
        from: Point<Dbu>,
        to: &BnetPtr,
        driver: PinId,
        max_cap: f64,
        r_drvr: f64,
    ) -> Result<Branch, RepairError> {
        let mut branch = self.repair_branch(to, driver, max_cap, r_drvr)?;
        let to_loc = to.at;
        let dbu = self.resizer.db.dbu().get() as f64;
        let rc = self.resizer.graph.wire_rc();
        let (wire_res, wire_cap) = (rc.resistance, rc.capacitance);
        let factor = self.resizer.slew_rc_factor;
        let max_length = self.resizer.options.max_wire_length;
        let meters = |l: i64| l as f64 / dbu * 1e-6;
        let elmore_slew = |length: i64, ref_cap: f64| {
            let r_wire = meters(length) * wire_res;
            let c_wire = meters(length) * wire_cap;
            (r_drvr * (c_wire + ref_cap) + r_wire * ref_cap + r_wire * c_wire / 2.0) * factor
        };
        let wire_drop = |length: i64, ref_cap: f64| {
            let r_wire = meters(length) * wire_res;
            let c_wire = meters(length) * wire_cap;
            r_wire * (c_wire / 2.0 + ref_cap) * factor
        };

        let (mut to_x, mut to_y) = (to_loc.x.get(), to_loc.y.get());
        let (from_x, from_y) = (from.x.get(), from.y.get());
        let mut length = (from_x - to_x).abs() + (from_y - to_y).abs();
        let mut wire_length = branch.wire_length + length;
        let mut wire_length_ref = branch.wire_length;
        let mut max_load_slew = branch.max_load_slew;
        let mut max_load_slew_margined = self.resizer.margined_slew(max_load_slew);
        let mut ref_cap = branch.cap;
        let mut load_cap = meters(length) * wire_cap + ref_cap;
        let mut load_slew = elmore_slew(length, ref_cap);
        let mut buffer_cell =
            self.resizer
                .find_target_cell(self.resizer.buffers.lowest_drive, load_cap, false);
        branch.cap = load_cap;
        branch.max_load_slew = max_load_slew - wire_drop(length, ref_cap);

        while (max_length > 0 && wire_length > max_length)
            || (wire_cap > 0.0 && max_cap > 0.0 && load_cap > max_cap)
            || load_slew > max_load_slew_margined
        {
            let split_length = self.split_length(SplitInput {
                length,
                wire_length,
                wire_length_ref,
                load_cap,
                ref_cap,
                max_cap,
                load_slew,
                max_load_slew_margined,
                r_drvr,
            });
            let buf_dist = if split_length >= length {
                length as f64
            } else {
                split_length as f64 * (1.0 - LENGTH_MARGIN)
            };
            let d = if length == 0 {
                0.0
            } else {
                buf_dist / length as f64
            };
            let buf_x = to_x + (d * (from_x - to_x) as f64) as i64;
            let buf_y = to_y + (d * (from_y - to_y) as f64) as i64;
            let at = Point::new(Dbu(buf_x), Dbu(buf_y));
            let mut leaf = Branch {
                cap: ref_cap,
                fanout: branch.fanout,
                max_load_slew,
                wire_length: 0,
                loads: std::mem::take(&mut branch.loads),
            };
            let inserted = self.make_repeater("wire", at, buffer_cell, &mut leaf)?;
            branch.loads = leaf.loads;
            if !inserted {
                break;
            }
            branch.fanout = leaf.fanout;
            length -= buf_dist as i64;
            wire_length = length;
            to_x = buf_x;
            to_y = buf_y;
            wire_length_ref = 0;
            ref_cap = leaf.cap;
            max_load_slew = leaf.max_load_slew;
            load_cap = meters(length) * wire_cap + ref_cap;
            max_load_slew_margined = self.resizer.margined_slew(max_load_slew);
            load_slew = elmore_slew(length, ref_cap);
            buffer_cell =
                self.resizer
                    .find_target_cell(self.resizer.buffers.lowest_drive, load_cap, false);
            branch.cap = load_cap;
            branch.max_load_slew = max_load_slew - wire_drop(length, ref_cap);
        }
        let _ = wire_length_ref;
        branch.wire_length = wire_length;
        Ok(branch)
    }

    /// How far back from the far end of a wire a repeater must go so the
    /// rest meets the length, load and slew limits, in database units, as
    /// the split length of `RepairDesign::repairNetWire`.
    // The slew estimates keep OpenROAD's grouping so they can be checked.
    #[allow(clippy::suboptimal_flops, clippy::suspicious_operation_groupings)]
    fn split_length(&self, input: SplitInput) -> i64 {
        let dbu = self.resizer.db.dbu().get() as f64;
        let to_dbu = |m: f64| (m * dbu * 1e6) as i64;
        let rc = self.resizer.graph.wire_rc();
        let (wire_res, wire_cap) = (rc.resistance, rc.capacitance);
        let max_length = self.resizer.options.max_wire_length;
        let mut split = if max_length > 0 && input.wire_length > max_length {
            (max_length - input.wire_length_ref)
                .max(0)
                .min(input.length / 2)
        } else {
            i64::MAX
        };
        if wire_cap > 0.0 && input.load_cap > input.max_cap {
            split = split.min(to_dbu((input.max_cap - input.ref_cap) / wire_cap).max(0));
        }
        if input.load_slew > input.max_load_slew_margined {
            let a = wire_res * wire_cap / 2.0;
            let b = input.r_drvr * wire_cap + wire_res * input.ref_cap;
            let c = input.r_drvr * input.ref_cap
                - input.max_load_slew_margined / self.resizer.slew_rc_factor;
            let l = if a > 1e-12 {
                let discriminant = b * b - 4.0 * a * c;
                if discriminant >= 0.0 {
                    (-b + discriminant.sqrt()) / (2.0 * a)
                } else {
                    0.0
                }
            } else if b > 1e-12 {
                -c / b
            } else {
                0.0
            };
            split = if l >= 0.0 { split.min(to_dbu(l)) } else { 0 };
        }
        split
    }

    /// Inserts a buffer at `at` driving the loads of `branch`, sized to its
    /// load, and makes the branch the buffer's input, as
    /// `RepairDesign::makeRepeater`; false when nothing could be inserted.
    fn make_repeater(
        &mut self,
        reason: &str,
        at: Point<Dbu>,
        buffer: LibCellId,
        branch: &mut Branch,
    ) -> Result<bool, RepairError> {
        if branch.loads.is_empty() {
            return Ok(false);
        }
        let old_net = self.resizer.db.pins[branch.loads[0].index()].net;
        let inserted =
            self.resizer
                .insert_buffer_before_loads(old_net, &branch.loads, buffer, at, reason)?;
        self.summary.inserted += 1;
        let _ = self.resizer.resize_to_target_slew(inserted.output)?;
        let sized = self
            .resizer
            .link
            .cell(self.resizer.db, inserted.cell)
            .map_or(buffer, |(id, _)| id);
        let sized_cell = self.resizer.link.liberty.cell(sized);
        let (sized_in, _) = sized_cell
            .buffer_ports()
            .ok_or_else(|| RepairError::MissingPins(sized_cell.name.clone()))?;
        let in_lib_port = &sized_cell.ports[sized_in];
        branch.loads = vec![inserted.input];
        branch.cap = in_lib_port.capacitance();
        branch.fanout = self.resizer.link.liberty.fanout_load(in_lib_port);
        branch.max_load_slew = self.resizer.max_input_slew(in_lib_port);
        Ok(true)
    }
}
