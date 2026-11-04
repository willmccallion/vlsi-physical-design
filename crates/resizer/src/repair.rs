//! `repair_design` as `OpenROAD`'s resizer does it.
//!
//! From the outputs back to the inputs, every driver whose slew, load or
//! wire breaks a limit is resized to the smallest size that meets it, and
//! where sizing is not enough, repeaters are inserted along the net's
//! Steiner tree, splitting wires and isolating branches until the Elmore
//! estimate of every load's slew is within its limit.

use crate::bnet::{self, BnetPtr, Kind};
use crate::library::SizingLimits;
use crate::resizer::{PortSlews, RepairError, Resizer, Timing, gate_slew, meters_to_dbu};
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{NetId, PinId};
use pare_common::db::liberty::{ArcRole, LibCellId};
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::geom::point::Point;
use pare_common::units::{Farads, FaradsPerMeter, Meters, Ohms, OhmsPerMeter, Seconds};
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

/// What a driver's net needs buffered for, once the driver is sized.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct NetNeeds {
    /// The load to buffer the net down to, when its load or its driver's
    /// slew breaks a limit.
    max_cap: Option<Farads>,
    /// Whether a load's slew breaks its limit.
    load_slew: bool,
}

/// Which limits a net's repair checks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Checks {
    /// Slews, loads and wire length.
    All,
    /// Wire length alone, as clock nets are repaired.
    LengthOnly,
}

/// What decides where a wire is split for a repeater.
#[derive(Clone, Copy, Debug)]
struct SplitInput {
    wire_res: OhmsPerMeter,
    wire_cap: FaradsPerMeter,
    length: i64,
    wire_length: i64,
    wire_length_ref: i64,
    load_cap: Farads,
    ref_cap: Farads,
    max_cap: Farads,
    load_slew: Seconds,
    max_load_slew_margined: Seconds,
    r_drvr: Ohms,
}

/// A driver's wire as `RepairDesign::repairNetWire` estimates it: its
/// resistance and capacitance per length, driven through `r_drvr`.
#[derive(Clone, Copy, Debug)]
struct DriverWire {
    res: OhmsPerMeter,
    cap: FaradsPerMeter,
    r_drvr: Ohms,
    slew_rc_factor: f64,
    dbu_per_micron: DbuPerMicron,
}

impl DriverWire {
    fn meters(&self, length: i64) -> Meters {
        Meters::from_dbu(Dbu(length), self.dbu_per_micron)
    }

    /// The load at the driver of `length` of wire with `ref_cap` beyond it.
    fn load_cap(&self, length: i64, ref_cap: Farads) -> Farads {
        self.meters(length) * self.cap + ref_cap
    }

    /// The Elmore slew at the far end of `length` of wire with `ref_cap`
    /// beyond it.
    // The slew estimates keep OpenROAD's grouping so they can be checked.
    #[allow(clippy::suboptimal_flops)]
    fn elmore_slew(&self, length: i64, ref_cap: Farads) -> Seconds {
        let r_wire = self.meters(length) * self.res;
        let c_wire = self.meters(length) * self.cap;
        (self.r_drvr * (c_wire + ref_cap) + r_wire * ref_cap + r_wire * c_wire / 2.0)
            * self.slew_rc_factor
    }

    /// The slew `length` of wire with `ref_cap` beyond it adds on its own.
    fn slew_drop(&self, length: i64, ref_cap: Farads) -> Seconds {
        let r_wire = self.meters(length) * self.res;
        let c_wire = self.meters(length) * self.cap;
        r_wire * (c_wire / 2.0 + ref_cap) * self.slew_rc_factor
    }
}

/// The part of a wire left to repair: `length` from its far end `to` back
/// to where it starts, carrying `ref_cap` and the load slew limit
/// `max_load_slew` beyond `to`.
#[derive(Clone, Copy, Debug)]
struct WireRest {
    to: Point<Dbu>,
    length: i64,
    /// Its length with the wire beyond `to` up to the next repeater.
    wire_length: i64,
    /// The length of the wire beyond `to` up to the next repeater.
    wire_length_ref: i64,
    ref_cap: Farads,
    max_load_slew: Seconds,
}

/// Which branches of a junction get a repeater, and why.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct JunctionRepeaters {
    left: bool,
    right: bool,
    reason: &'static str,
}

/// What a repaired branch presents to its parent.
#[derive(Clone, Debug)]
struct Branch {
    cap: Farads,
    fanout: f64,
    max_load_slew: Seconds,
    wire_length: i64,
    loads: Vec<PinId>,
}

/// The design repair in progress.
#[derive(Debug)]
pub struct Repairer<'r, 'a, T: Timing> {
    resizer: &'r mut Resizer<'a, T>,
    summary: RepairSummary,
    load_slew_violation_drivers: HashSet<PinId>,
}

impl<'r, 'a, T: Timing> Repairer<'r, 'a, T> {
    /// Prepares a design repair over the session.
    pub fn new(resizer: &'r mut Resizer<'a, T>) -> Self {
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
        self.resizer.defer_timing();
        for driver in drivers.into_iter().rev() {
            self.repair_driver(driver)?;
        }
        self.resizer.resume_timing();
        self.resizer.clear_slew_annotations();
        Ok(self.summary)
    }

    /// Holds every violating input slew at its limit so the repair of one
    /// driver does not upsize the next, as `OpenROAD`'s forward pass.
    fn annotate_violating_loads(&mut self) {
        let annotated = self.resizer.annotate_violating_input_slews();
        let netlist = self.resizer.netlist();
        for load in annotated {
            let net = self.resizer.db.pins[load.index()].net;
            self.load_slew_violation_drivers
                .extend(netlist.drivers(net));
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
        self.repair_net(net, driver, Checks::All)
    }

    /// Buffers every clock net with a wire longer than `max_length`
    /// database units, its slews and loads left unchecked and its driver
    /// unsized, with margins of zero and the strongest of `clock_buffers`
    /// standing for the strongest buffer, as `RepairDesign::repairClkNets`
    /// runs `repairNet` over the drivers of the clock network.
    pub fn repair_clock_nets(
        mut self,
        clock_buffers: &[LibCellId],
        max_length: Option<Dbu>,
    ) -> Result<RepairSummary, RepairError> {
        let saved_options = self.resizer.options;
        let saved_strongest = self.resizer.r_strongest_buffer;
        self.resizer.options.slew_margin = 0.0;
        self.resizer.options.cap_margin = 0.0;
        self.resizer.options.max_wire_length = max_length;
        self.resizer.options.sizing_limits = SizingLimits::NONE;
        self.resizer.r_strongest_buffer = clock_buffers
            .iter()
            .map(|&b| self.resizer.link.buffer_drive_resistance(b))
            .fold(Ohms::new(f64::MAX), Ohms::min);
        let drivers: Vec<PinId> = {
            let netlist = self.resizer.netlist();
            (0..self.resizer.db.pins.len())
                .map(PinId::new)
                .filter(|&p| {
                    self.resizer.clocks.is_clock(p)
                        && netlist.is_driver(p)
                        && !netlist.is_special(self.resizer.db.pins[p.index()].net)
                })
                .collect()
        };
        self.resizer.defer_timing();
        let result = drivers.into_iter().try_for_each(|driver| {
            let net = self.resizer.db.pins[driver.index()].net;
            self.repair_net(net, driver, Checks::LengthOnly)
        });
        self.resizer.resume_timing();
        self.resizer.options = saved_options;
        self.resizer.r_strongest_buffer = saved_strongest;
        result?;
        Ok(self.summary)
    }

    /// `RepairDesign::repairNet` for one driver: its own slew repaired by
    /// sizing, then its net buffered where loads, load slews or wire
    /// lengths still break their limits.
    fn repair_net(&mut self, net: NetId, driver: PinId, checks: Checks) -> Result<(), RepairError> {
        let tristate = self.resizer.is_tristate_driver(driver);
        let needs = match checks {
            Checks::All => self.check_limits(driver)?,
            Checks::LengthOnly => NetNeeds::default(),
        };
        if tristate {
            return Ok(());
        }
        let Some(tree) = self.driver_tree(net, driver) else {
            return Ok(());
        };
        let too_long = self
            .resizer
            .options
            .max_wire_length
            .is_some_and(|max| tree.max_load_wire_length() > max.get());
        if too_long {
            self.summary.length_violations += 1;
        }
        if needs.max_cap.is_none() && !needs.load_slew && !too_long {
            return Ok(());
        }
        self.resizer.batch_edits();
        let _ = self.repair_tree(&tree, driver, needs.max_cap.unwrap_or(Farads::INFINITY))?;
        self.resizer.flush();
        self.summary.repaired_nets += 1;
        Ok(())
    }

    /// Checks the driver's slew, resizing it when the slew is over its
    /// limit, then, unless it is a tristate driver, its loads' slews and its
    /// load, counting each violation; returns what buffering its net needs.
    fn check_limits(&mut self, driver: PinId) -> Result<NetNeeds, RepairError> {
        let tristate = self.resizer.is_tristate_driver(driver);
        let mut needs = NetNeeds::default();
        let mut slew_violation = false;
        let (slew, limit) = self.driver_slew_check(driver);
        if let Some(limit) = limit
            && slew > limit
        {
            slew_violation = true;
            if self.repair_driver_slew(driver)? {
                self.summary.resized += 1;
            }
            let (slew_after, _) = self.driver_slew_check(driver);
            if slew_after > limit {
                needs.max_cap = self.slew_load_cap(driver, limit);
            }
        }
        if !tristate
            && (self.loads_violate_slew(driver)
                || self.load_slew_violation_drivers.contains(&driver))
        {
            slew_violation = true;
            needs.load_slew = true;
        }
        if slew_violation {
            self.summary.slew_violations += 1;
        }
        if !tristate && let Some(limit) = self.resizer.cap_limit(driver) {
            let margined = self.resizer.margined_cap(limit);
            if self.resizer.load_cap(driver) > margined {
                needs.max_cap = Some(margined);
                self.summary.cap_violations += 1;
            }
        }
        Ok(needs)
    }

    /// The buffering tree of the driver's net, from its parasitics.
    fn driver_tree(&self, net: NetId, driver: PinId) -> Option<BnetPtr> {
        let netlist = self.resizer.netlist();
        let parasitics = self.resizer.graph.parasitics(net)?;
        bnet::from_parasitics(
            &netlist,
            parasitics,
            driver,
            &self.resizer.tree_rc(),
            self.resizer,
        )
    }

    /// The driver's slew and its margined limit, if one applies.
    fn driver_slew_check(&self, driver: PinId) -> (Seconds, Option<Seconds>) {
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
            let Some((cell, port)) = netlist.lib_cell_port(load) else {
                return false;
            };
            let slew = self.resizer.graph.slew(load).max();
            match port.max_transition.filter(|&l| l != Seconds::ZERO) {
                Some(limit) => slew > self.resizer.margined_slew(limit),
                None => self
                    .resizer
                    .link
                    .liberty
                    .cell_defaults(cell)
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
                self.resizer.swappable_cells(lib_id),
            )
        };
        if candidates.is_empty() {
            return Ok(false);
        }
        let input_slews = self.resizer.input_slews(cell);
        let mut sizes: Vec<(Seconds, f64, LibCellId)> = candidates
            .into_iter()
            .filter_map(|size| {
                let violation = self.slew_violation(size, &port_name, &input_slews, load_cap)?;
                Some((violation, self.resizer.link.liberty.cell(size).area, size))
            })
            .collect();
        sizes.sort_by(|a, b| {
            if a.0 == Seconds::ZERO && b.0 == Seconds::ZERO {
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

    /// How far the slew out of port `port_name` of `size` would pass its
    /// limit, driving `load_cap` from `input_slews`, over the arcs through
    /// the cell's logic into it; none when `size` has no such port.
    fn slew_violation(
        &self,
        size: LibCellId,
        port_name: &str,
        input_slews: &PortSlews<'_>,
        load_cap: Farads,
    ) -> Option<Seconds> {
        let liberty = &self.resizer.link.liberty;
        let size_cell = liberty.cell(size);
        let port = size_cell.port(port_name)?;
        let Some(limit) = liberty.slew_limit(size_cell, &size_cell.ports[port]) else {
            return Some(Seconds::ZERO);
        };
        let limit = self.resizer.margined_slew(limit);
        let mut violation = Seconds::ZERO;
        let sets = size_cell
            .arc_sets
            .iter()
            .filter(|s| s.to == port && s.role.is_logic_delay());
        for set in sets {
            let from_name = &size_cell.ports[set.from].name;
            let from_slews = input_slews.get(from_name.as_str());
            for arc in &set.arcs {
                let in_slew = match from_slews {
                    Some(slews) if set.role != ArcRole::RegClkToQ => slews.get(arc.from_edge),
                    _ => Seconds::ZERO,
                };
                let arc_slew = arc.gate_delay(in_slew, load_cap).1;
                if arc_slew > limit {
                    violation = violation.max(arc_slew - limit);
                }
            }
        }
        Some(violation)
    }

    /// The load at which the driver's slew would equal `slew`, by binary
    /// search over the gate tables at the target input slews, as
    /// `RepairDesign::findSlewLoadCap`.
    fn slew_load_cap(&self, driver: PinId, slew: Seconds) -> Option<Farads> {
        let (lib_id, port) = self.resizer.link.pin_port(self.resizer.db, driver)?;
        let cell = self.resizer.link.liberty.cell(lib_id);
        let drive_res = cell.drive_resistance(port);
        if drive_res == Ohms::ZERO {
            return Some(Farads::INFINITY);
        }
        let slews = &self.resizer.targets.slews;
        let diff = |cap: f64| (gate_slew(cell, port, slews, Farads::new(cap)) - slew).get();
        let mut cap1 = 0.0;
        let mut cap2 = (slew / drive_res * 2.0).get();
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
        Some(Farads::new(cap1))
    }

    /// Repairs a buffered tree bottom up, as `RepairDesign::repairNet` over
    /// a `BufferedNet`.
    fn repair_tree(
        &mut self,
        tree: &BnetPtr,
        driver: PinId,
        max_cap: Farads,
    ) -> Result<Branch, RepairError> {
        let own = self
            .resizer
            .link
            .pin_port(self.resizer.db, driver)
            .map_or(Ohms::ZERO, |(lib, port)| {
                self.resizer.link.liberty.cell(lib).drive_resistance(port)
            });
        let r_drvr = own.max(self.resizer.r_strongest_buffer);
        self.repair_branch(tree, driver, max_cap, r_drvr)
    }

    fn repair_branch(
        &mut self,
        tree: &BnetPtr,
        driver: PinId,
        max_cap: Farads,
        r_drvr: Ohms,
    ) -> Result<Branch, RepairError> {
        match &tree.kind {
            Kind::Load { pin } => Ok(self.load_branch(*pin)),
            Kind::Junction { left, right } => {
                self.repair_junction(tree.at, left, right, driver, max_cap, r_drvr)
            }
            Kind::Wire { to, layer } => {
                let (r, c) = self.resizer.tree_rc().wire(*layer);
                self.repair_wire(tree.at, to, (r, c), driver, max_cap, r_drvr)
            }
            Kind::Buffer { to, .. } => {
                let (r, c) = self.resizer.tree_rc().wire(None);
                self.repair_wire(tree.at, to, (r, c), driver, max_cap, r_drvr)
            }
            Kind::Via {
                to,
                layer,
                to_layer,
            } => {
                let r_via = self.resizer.tree_rc().via(*layer, *to_layer);
                self.repair_via(to, r_via, driver, max_cap, r_drvr)
            }
        }
    }

    /// `RepairDesign::repairNetVia`: the subtree's load, with its slew
    /// limit lowered by the slew the via's resistance adds.
    // The slew estimate keeps OpenROAD's grouping so it can be checked.
    #[allow(clippy::suboptimal_flops)]
    fn repair_via(
        &mut self,
        to: &BnetPtr,
        r_via: Ohms,
        driver: PinId,
        max_cap: Farads,
        r_drvr: Ohms,
    ) -> Result<Branch, RepairError> {
        let mut branch = self.repair_branch(to, driver, max_cap, r_drvr)?;
        branch.max_load_slew -= r_via * branch.cap * self.resizer.slew_rc_factor;
        Ok(branch)
    }

    /// A load: its pin's capacitance, fanout load and slew limit, as
    /// `OpenROAD`'s `BufferedNet` of type load.
    fn load_branch(&self, pin: PinId) -> Branch {
        let load = bnet::LoadModel::load(self.resizer, pin);
        Branch {
            cap: load.cap,
            fanout: load.fanout,
            max_load_slew: load.max_slew,
            wire_length: 0,
            loads: vec![pin],
        }
    }

    /// `RepairDesign::repairNetJunc`.
    fn repair_junction(
        &mut self,
        at: Point<Dbu>,
        left: &BnetPtr,
        right: &BnetPtr,
        driver: PinId,
        max_cap: Farads,
        r_drvr: Ohms,
    ) -> Result<Branch, RepairError> {
        let mut left = self.repair_branch(left, driver, max_cap, r_drvr)?;
        let mut right = self.repair_branch(right, driver, max_cap, r_drvr)?;
        let load_cap = left.cap + right.cap;
        let buffer_cell =
            self.resizer
                .find_target_cell(self.resizer.buffers.lowest_drive, load_cap, false);
        let repeaters = self.junction_repeaters(&left, &right, max_cap, r_drvr);
        if repeaters.left {
            let _ = self.make_repeater(repeaters.reason, at, buffer_cell, &mut left)?;
            left.wire_length = 0;
        }
        if repeaters.right {
            let _ = self.make_repeater(repeaters.reason, at, buffer_cell, &mut right)?;
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

    /// Which of a junction's branches need a repeater for the slew, load
    /// and length limits of the two together: for the slew, the one with
    /// less slack; for the load and length, the heavier and longer.
    // The slew estimates keep OpenROAD's grouping so they can be checked.
    #[allow(clippy::suboptimal_flops)]
    fn junction_repeaters(
        &self,
        left: &Branch,
        right: &Branch,
        max_cap: Farads,
        r_drvr: Ohms,
    ) -> JunctionRepeaters {
        let load_cap = left.cap + right.cap;
        let max_load_slew = left.max_load_slew.min(right.max_load_slew);
        let factor = self.resizer.slew_rc_factor;
        let mut repeaters = JunctionRepeaters {
            left: false,
            right: false,
            reason: "load_slew",
        };
        if r_drvr * load_cap * factor > self.resizer.margined_slew(max_load_slew) {
            let slack_left =
                self.resizer.margined_slew(left.max_load_slew) - r_drvr * left.cap * factor;
            let slack_right =
                self.resizer.margined_slew(right.max_load_slew) - r_drvr * right.cap * factor;
            if slack_left < slack_right {
                repeaters.left = true;
            } else {
                repeaters.right = true;
            }
        }
        if load_cap > max_cap {
            repeaters.reason = "max_cap";
            if left.cap > right.cap {
                repeaters.left = true;
            } else {
                repeaters.right = true;
            }
        }
        let max_length = self.resizer.options.max_wire_length;
        if max_length.is_some_and(|max| left.wire_length + right.wire_length > max.get()) {
            repeaters.reason = "max_length";
            if left.wire_length > right.wire_length {
                repeaters.left = true;
            } else {
                repeaters.right = true;
            }
        }
        repeaters
    }

    /// `RepairDesign::repairNetWire`: backs up from the far end of a wire
    /// toward `from`, inserting repeaters where length, load or slew would
    /// break a limit.
    fn repair_wire(
        &mut self,
        from: Point<Dbu>,
        to: &BnetPtr,
        (res, cap): (OhmsPerMeter, FaradsPerMeter),
        driver: PinId,
        max_cap: Farads,
        r_drvr: Ohms,
    ) -> Result<Branch, RepairError> {
        let mut branch = self.repair_branch(to, driver, max_cap, r_drvr)?;
        let wire = DriverWire {
            res,
            cap,
            r_drvr,
            slew_rc_factor: self.resizer.slew_rc_factor,
            dbu_per_micron: self.resizer.db.dbu(),
        };
        let length = from.manhattan(to.at).get();
        let mut rest = WireRest {
            to: to.at,
            length,
            wire_length: branch.wire_length + length,
            wire_length_ref: branch.wire_length,
            ref_cap: branch.cap,
            max_load_slew: branch.max_load_slew,
        };
        Self::carry_wire(&wire, &rest, &mut branch);
        while self.wire_breaks_limits(&wire, &rest, max_cap) {
            let split_length = self.split_length(SplitInput {
                wire_res: wire.res,
                wire_cap: wire.cap,
                length: rest.length,
                wire_length: rest.wire_length,
                wire_length_ref: rest.wire_length_ref,
                load_cap: wire.load_cap(rest.length, rest.ref_cap),
                ref_cap: rest.ref_cap,
                max_cap,
                load_slew: wire.elmore_slew(rest.length, rest.ref_cap),
                max_load_slew_margined: self.resizer.margined_slew(rest.max_load_slew),
                r_drvr,
            });
            let buf_dist = if split_length >= rest.length {
                rest.length as f64
            } else {
                split_length as f64 * (1.0 - LENGTH_MARGIN)
            };
            let at = point_back_along(rest.to, from, buf_dist, rest.length);
            let buffer_cell = self.resizer.find_target_cell(
                self.resizer.buffers.lowest_drive,
                wire.load_cap(rest.length, rest.ref_cap),
                false,
            );
            let mut leaf = Branch {
                cap: rest.ref_cap,
                fanout: branch.fanout,
                max_load_slew: rest.max_load_slew,
                wire_length: 0,
                loads: std::mem::take(&mut branch.loads),
            };
            let inserted = self.make_repeater("wire", at, buffer_cell, &mut leaf)?;
            branch.loads = leaf.loads;
            if !inserted {
                break;
            }
            branch.fanout = leaf.fanout;
            let length = rest.length - buf_dist as i64;
            rest = WireRest {
                to: at,
                length,
                wire_length: length,
                wire_length_ref: 0,
                ref_cap: leaf.cap,
                max_load_slew: leaf.max_load_slew,
            };
            Self::carry_wire(&wire, &rest, &mut branch);
        }
        branch.wire_length = rest.wire_length;
        Ok(branch)
    }

    /// Sets what `branch` presents through the rest of the wire: its load
    /// and the load slew limit the wire leaves it.
    fn carry_wire(wire: &DriverWire, rest: &WireRest, branch: &mut Branch) {
        branch.cap = wire.load_cap(rest.length, rest.ref_cap);
        branch.max_load_slew = rest.max_load_slew - wire.slew_drop(rest.length, rest.ref_cap);
    }

    /// Whether the rest of the wire breaks the length limit, the load
    /// limit `max_cap`, or the load slew limit beyond it.
    fn wire_breaks_limits(&self, wire: &DriverWire, rest: &WireRest, max_cap: Farads) -> bool {
        let max_length = self.resizer.options.max_wire_length;
        max_length.is_some_and(|max| rest.wire_length > max.get())
            || (wire.cap > FaradsPerMeter::ZERO
                && max_cap > Farads::ZERO
                && wire.load_cap(rest.length, rest.ref_cap) > max_cap)
            || wire.elmore_slew(rest.length, rest.ref_cap)
                > self.resizer.margined_slew(rest.max_load_slew)
    }

    /// How far back from the far end of a wire a repeater must go so the
    /// rest meets the length, load and slew limits, in database units, as
    /// the split length of `RepairDesign::repairNetWire`.
    // The slew estimates keep OpenROAD's grouping so they can be checked.
    #[allow(clippy::suboptimal_flops, clippy::suspicious_operation_groupings)]
    fn split_length(&self, input: SplitInput) -> i64 {
        let dbu = self.resizer.db.dbu();
        let to_dbu = |m: f64| meters_to_dbu(Meters::new(m), dbu).get();
        let (wire_res, wire_cap) = (input.wire_res.get(), input.wire_cap.get());
        let (load_cap, ref_cap, max_cap) = (
            input.load_cap.get(),
            input.ref_cap.get(),
            input.max_cap.get(),
        );
        let (r_drvr, load_slew, max_load_slew) = (
            input.r_drvr.get(),
            input.load_slew.get(),
            input.max_load_slew_margined.get(),
        );
        let mut split = match self.resizer.options.max_wire_length {
            Some(max) if input.wire_length > max.get() => (max.get() - input.wire_length_ref)
                .max(0)
                .min(input.length / 2),
            _ => i64::MAX,
        };
        if wire_cap > 0.0 && load_cap > max_cap {
            split = split.min(to_dbu((max_cap - ref_cap) / wire_cap).max(0));
        }
        if load_slew > max_load_slew {
            // `OpenROAD` solves the quadratic in single precision.
            let a = (wire_res * wire_cap / 2.0) as f32;
            let b = (r_drvr * wire_cap + wire_res * ref_cap) as f32;
            let c = (r_drvr * ref_cap - max_load_slew / self.resizer.slew_rc_factor) as f32;
            let l = if f64::from(a) > 1e-12 {
                let discriminant = b * b - 4.0 * a * c;
                if discriminant >= 0.0 {
                    (-b + discriminant.sqrt()) / (2.0 * a)
                } else {
                    0.0
                }
            } else if f64::from(b) > 1e-12 {
                -c / b
            } else {
                0.0
            };
            split = if l >= 0.0 {
                split.min(to_dbu(f64::from(l)))
            } else {
                0
            };
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
        branch.fanout = self
            .resizer
            .link
            .liberty
            .fanout_load(sized_cell, in_lib_port);
        branch.max_load_slew = self.resizer.max_input_slew(sized_cell, in_lib_port);
        Ok(true)
    }
}
/// The point `distance` back from `to` toward `from` along the straight
/// line between them, `length` long, truncated to the database grid as
/// `repairNetWire` places its repeaters.
// OpenROAD rounds the product before the sum; a fused one could move it.
#[allow(clippy::suboptimal_flops)]
fn point_back_along(to: Point<Dbu>, from: Point<Dbu>, distance: f64, length: i64) -> Point<Dbu> {
    let share = if length == 0 {
        0.0
    } else {
        distance / length as f64
    };
    let along = |to: Dbu, from: Dbu| {
        let (to, from) = (to.get(), from.get());
        Dbu((to as f64 + share * (from - to) as f64) as i64)
    };
    Point::new(along(to.x, from.x), along(to.y, from.y))
}
