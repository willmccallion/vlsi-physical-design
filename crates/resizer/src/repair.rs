//! `repair_design` as `OpenROAD`'s resizer does it.
//!
//! From the outputs back to the inputs, every driver whose slew, load or
//! wire breaks a limit is resized to the smallest size that meets it, and
//! where sizing is not enough, repeaters are inserted along the net's
//! Steiner tree, splitting wires and isolating branches until the Elmore
//! estimate of every load's slew is within its limit.

use crate::graph::TimingGraph;
use crate::library::{BufferSet, Link};
use crate::netlist::{ClockNetwork, Constants, Namer, Netlist};
use crate::parasitics::NetParasitics;
use pare_common::db::design::{EditError, NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, MacroPinId, NetId, PinId};
use pare_common::db::liberty::{ArcRole, Edge, EdgeValues, LibCell, LibCellId, LibPort};
use pare_common::db::library::DuplicateName;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use std::collections::HashSet;

/// The load a buffer's target slew is measured at, in multiples of its
/// input capacitance, as `OpenROAD`'s `tgt_slew_load_cap_factor`.
const TARGET_SLEW_LOAD_FACTOR: f64 = 10.0;

/// How far short of the computed split a repeater is placed, as
/// `OpenROAD`'s `length_margin`.
const LENGTH_MARGIN: f64 = 0.05;

/// Pessimism added to the slew shape factor, as `OpenROAD` adds it.
const SLEW_RC_PESSIMISM: f64 = 0.10;

/// Why repair stopped.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum RepairError {
    /// The netlist refused an edit.
    #[error(transparent)]
    Edit(#[from] EditError),
    /// A name was already taken.
    #[error(transparent)]
    Name(#[from] DuplicateName),
    /// A cell to insert has no master.
    #[error("cell '{0}' has no master")]
    NoMaster(String),
    /// The library's slew thresholds give an unusable shape factor.
    #[error("the slew thresholds give an RC shape factor of {0:.3e}, outside 0.1 to 10")]
    ShapeFactor(f64),
}

/// Margins and limits for repair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RepairOptions {
    /// Percentage of each slew limit kept in hand.
    pub slew_margin: f64,
    /// Percentage of each capacitance limit kept in hand.
    pub cap_margin: f64,
    /// Longest wire allowed, in database units; zero for no limit.
    pub max_wire_length: i64,
}

impl Default for RepairOptions {
    fn default() -> Self {
        Self {
            slew_margin: 0.0,
            cap_margin: 0.0,
            max_wire_length: 0,
        }
    }
}

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

/// Target slews and the load each cell meets them at.
#[derive(Clone, Debug)]
struct Targets {
    slews: EdgeValues,
    load: Vec<f64>,
}

/// One net of a buffered tree, as `OpenROAD`'s `BufferedNet`.
#[derive(Clone, Debug)]
enum Tree {
    Load {
        at: Point<Dbu>,
        pin: PinId,
    },
    Junction {
        at: Point<Dbu>,
        left: Box<Self>,
        right: Box<Self>,
    },
    Wire {
        from: Point<Dbu>,
        to: Box<Self>,
    },
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

/// What repair reads about the design besides the netlist itself.
#[derive(Clone, Copy, Debug)]
pub struct Context<'a> {
    /// The library link.
    pub link: &'a Link<'a>,
    /// The clock network, which repair leaves alone.
    pub clocks: &'a ClockNetwork,
    /// The constant pins, which repair leaves alone.
    pub constants: &'a Constants,
}

/// The repair in progress.
#[derive(Debug)]
pub struct Repairer<'a> {
    db: &'a mut NetlistDB,
    link: &'a Link<'a>,
    clocks: &'a ClockNetwork,
    constants: &'a Constants,
    graph: TimingGraph,
    buffers: BufferSet,
    targets: Targets,
    namer: &'a mut Namer,
    options: RepairOptions,
    summary: RepairSummary,
    slew_rc_factor: f64,
    r_strongest_buffer: f64,
    resized_multi_output: HashSet<CellId>,
    load_slew_violation_drivers: HashSet<PinId>,
}

impl<'a> Repairer<'a> {
    /// Prepares repair over `db` with slews from `graph`.
    pub fn new(
        db: &'a mut NetlistDB,
        context: Context<'a>,
        graph: TimingGraph,
        buffers: BufferSet,
        namer: &'a mut Namer,
        options: RepairOptions,
    ) -> Result<Self, RepairError> {
        let Context {
            link,
            clocks,
            constants,
        } = context;
        let slew_rc_factor = slew_rc_factor(link)?;
        let r_strongest_buffer = buffers
            .cells
            .iter()
            .map(|&b| link.buffer_drive_resistance(b))
            .fold(f64::MAX, f64::min);
        let targets = find_targets(link, &buffers);
        Ok(Self {
            db,
            link,
            clocks,
            constants,
            graph,
            buffers,
            targets,
            namer,
            options,
            summary: RepairSummary::default(),
            slew_rc_factor,
            r_strongest_buffer,
            resized_multi_output: HashSet::new(),
            load_slew_violation_drivers: HashSet::new(),
        })
    }

    /// Repairs every driver from the outputs back to the inputs, as
    /// `RepairDesign::repairDesign`, and returns what it did.
    pub fn repair_design(mut self) -> Result<RepairSummary, RepairError> {
        self.annotate_violating_loads();
        let drivers = {
            let netlist = Netlist::new(self.db, self.link);
            self.graph.drivers_by_level(&netlist)
        };
        for driver in drivers.into_iter().rev() {
            self.repair_driver(driver)?;
        }
        self.graph.clear_annotations();
        Ok(self.summary)
    }

    /// Holds every violating input slew at its limit so the repair of one
    /// driver does not upsize the next, as `OpenROAD`'s forward pass.
    fn annotate_violating_loads(&mut self) {
        let netlist = Netlist::new(self.db, self.link);
        let loads = self.graph.loads_by_level(&netlist);
        let mut annotations = Vec::new();
        for load in loads {
            let Some(port) = netlist.lib_port(load) else {
                continue;
            };
            let limit = self.max_input_slew(port);
            if self.graph.slew(load).max() > limit {
                annotations.push((load, limit));
                let net = self.db.pins[load.index()].net;
                self.load_slew_violation_drivers
                    .extend(netlist.drivers(net));
            }
        }
        for (load, limit) in annotations {
            self.graph.annotate_slew(load, limit);
        }
    }

    /// The transition limit at an input pin: its own, else, as `OpenROAD`
    /// fixes up for libraries like Nangate45, the library default meant for
    /// outputs; infinite without either.
    fn max_input_slew(&self, port: &LibPort) -> f64 {
        port.max_transition
            .filter(|&l| l != 0.0)
            .or(self.link.liberty.defaults.max_transition)
            .unwrap_or(f64::INFINITY)
    }

    fn margined_slew(&self, limit: f64) -> f64 {
        limit * (1.0 - self.options.slew_margin / 100.0)
    }

    fn repair_driver(&mut self, driver: PinId) -> Result<(), RepairError> {
        let (net, skip) = {
            let netlist = Netlist::new(self.db, self.link);
            let net = self.db.pins[driver.index()].net;
            let skip = netlist.is_special(net)
                || self.clocks.is_clock(driver)
                || self.constants.is_constant(driver)
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

        let tristate = self.is_tristate_driver(driver);
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
            let cap = {
                let netlist = Netlist::new(self.db, self.link);
                self.graph.load_cap(&netlist, driver)
            };
            if let Some(limit) = self.cap_limit(driver) {
                let margined = limit * (1.0 - self.options.cap_margin / 100.0);
                if cap > margined {
                    max_cap = margined;
                    repair_cap = true;
                    self.summary.cap_violations += 1;
                }
            }
        }

        if !tristate {
            let tree = {
                let netlist = Netlist::new(self.db, self.link);
                self.graph
                    .parasitics(net)
                    .and_then(|p| buffered_tree(&netlist, p, driver))
            };
            if let Some(tree) = tree {
                let wire_length = tree_wire_length(&tree);
                let repair_wire =
                    self.options.max_wire_length > 0 && wire_length > self.options.max_wire_length;
                if repair_wire {
                    self.summary.length_violations += 1;
                }
                if repair_cap || repair_load_slew || repair_wire {
                    repaired = true;
                    let before = self.summary.inserted;
                    let _ = self.repair_tree(&tree, driver, max_cap)?;
                    if self.summary.inserted > before {
                        let netlist = Netlist::new(self.db, self.link);
                        self.graph.grow(&netlist);
                        self.graph.update_net(&netlist, self.clocks, net);
                    }
                }
            }
        }
        if repaired {
            self.summary.repaired_nets += 1;
        }
        Ok(())
    }

    fn is_tristate_driver(&self, driver: PinId) -> bool {
        let netlist = Netlist::new(self.db, self.link);
        netlist
            .lib_port(driver)
            .is_some_and(|p| p.three_state.is_some())
    }

    /// The driver's slew and its margined limit, if one applies.
    fn driver_slew_check(&self, driver: PinId) -> (f64, Option<f64>) {
        let netlist = Netlist::new(self.db, self.link);
        let slew = self.graph.slew(driver).max();
        let limit = netlist
            .lib_port(driver)
            .and_then(|p| self.link.liberty.slew_limit(p))
            .map(|l| self.margined_slew(l));
        (slew, limit)
    }

    fn cap_limit(&self, driver: PinId) -> Option<f64> {
        let netlist = Netlist::new(self.db, self.link);
        netlist
            .lib_port(driver)
            .and_then(|p| self.link.liberty.capacitance_limit(p))
    }

    /// Whether any load of the driver's net has a slew over its limit, as
    /// `Resizer::checkLoadSlews`.
    fn loads_violate_slew(&self, driver: PinId) -> bool {
        let netlist = Netlist::new(self.db, self.link);
        let net = self.db.pins[driver.index()].net;
        netlist.loads(net).filter(|&p| p != driver).any(|load| {
            let Some(port) = netlist.lib_port(load) else {
                return false;
            };
            let slew = self.graph.slew(load).max();
            match port.max_transition.filter(|&l| l != 0.0) {
                Some(limit) => slew > self.margined_slew(limit),
                None => self
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
            let netlist = Netlist::new(self.db, self.link);
            let PinOwner::Cell { cell, .. } = self.db.pins[driver.index()].owner else {
                return Ok(false);
            };
            let Some((lib_id, lib)) = self.link.cell(self.db, cell) else {
                return Ok(false);
            };
            if lib.dont_touch || !self.link.is_logic_std_cell(self.db, cell) {
                return Ok(false);
            }
            let Some((_, port)) = self.link.pin_port(self.db, driver) else {
                return Ok(false);
            };
            let load_cap = self.graph.load_cap(&netlist, driver);
            (
                cell,
                lib_id,
                lib.ports[port].name.clone(),
                load_cap,
                self.link.swappable_cells(lib_id),
            )
        };
        if candidates.is_empty() {
            return Ok(false);
        }
        let input_slews = self.input_slews(cell);
        let mut sizes: Vec<(f64, f64, LibCellId)> = Vec::new();
        for size in candidates {
            let size_cell = self.link.liberty.cell(size);
            let Some(port) = size_cell.port(&port_name) else {
                continue;
            };
            let mut violation = 0.0;
            if let Some(limit) = self.link.liberty.slew_limit(&size_cell.ports[port]) {
                let limit = self.margined_slew(limit);
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
        self.replace_cell(cell, selected)
    }

    /// The slews at each input pin of `cell`, by Liberty port name.
    fn input_slews(&self, cell: CellId) -> Vec<(String, EdgeValues)> {
        let netlist = Netlist::new(self.db, self.link);
        self.db.cells[cell.index()]
            .pins
            .iter()
            .filter_map(|&pin| {
                let port = netlist.lib_port(pin)?;
                Some((port.name.clone(), self.graph.slew(pin)))
            })
            .collect()
    }

    /// Gives `cell` the master of `size` and refreshes the slews around it;
    /// returns whether it changed.
    fn replace_cell(&mut self, cell: CellId, size: LibCellId) -> Result<bool, RepairError> {
        let master = self
            .link
            .macro_of(size)
            .ok_or_else(|| RepairError::NoMaster(self.link.liberty.cell(size).name.clone()))?;
        if self.db.cells[cell.index()].macro_id == master {
            return Ok(false);
        }
        self.db.replace_master(cell, master)?;
        let netlist = Netlist::new(self.db, self.link);
        self.graph.update_cell(&netlist, self.clocks, cell);
        Ok(true)
    }

    /// The load at which the driver's slew would equal `slew`, by binary
    /// search over the gate tables at the target input slews, as
    /// `RepairDesign::findSlewLoadCap`.
    fn slew_load_cap(&self, driver: PinId, slew: f64) -> Option<f64> {
        let (lib_id, port) = self.link.pin_port(self.db, driver)?;
        let cell = self.link.liberty.cell(lib_id);
        let drive_res = cell.drive_resistance(port);
        if drive_res == 0.0 {
            return Some(f64::INFINITY);
        }
        let diff = |cap: f64| gate_slew(cell, port, &self.targets.slews, cap) - slew;
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
        tree: &Tree,
        driver: PinId,
        max_cap: f64,
    ) -> Result<Branch, RepairError> {
        let own = self
            .link
            .pin_port(self.db, driver)
            .map_or(0.0, |(lib, port)| {
                self.link.liberty.cell(lib).drive_resistance(port)
            });
        let r_drvr = own.max(self.r_strongest_buffer);
        self.repair_branch(tree, driver, max_cap, r_drvr)
    }

    fn repair_branch(
        &mut self,
        tree: &Tree,
        driver: PinId,
        max_cap: f64,
        r_drvr: f64,
    ) -> Result<Branch, RepairError> {
        match tree {
            Tree::Load { pin, .. } => Ok(self.load_branch(*pin)),
            Tree::Junction { at, left, right } => {
                self.repair_junction(*at, left, right, driver, max_cap, r_drvr)
            }
            Tree::Wire { from, to } => self.repair_wire(*from, to, driver, max_cap, r_drvr),
        }
    }

    /// A load: its pin's capacitance, fanout load and slew limit, as
    /// `OpenROAD`'s `BufferedNet` of type load.
    fn load_branch(&self, pin: PinId) -> Branch {
        let netlist = Netlist::new(self.db, self.link);
        let (cap, fanout, max_load_slew) =
            netlist
                .lib_port(pin)
                .map_or((0.0, 0.0, f64::INFINITY), |port| {
                    (
                        port.capacitance(),
                        self.link.liberty.fanout_load(port),
                        self.max_input_slew(port),
                    )
                });
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
        left: &Tree,
        right: &Tree,
        driver: PinId,
        max_cap: f64,
        r_drvr: f64,
    ) -> Result<Branch, RepairError> {
        let mut left = self.repair_branch(left, driver, max_cap, r_drvr)?;
        let mut right = self.repair_branch(right, driver, max_cap, r_drvr)?;
        let load_cap = left.cap + right.cap;
        let max_load_slew = left.max_load_slew.min(right.max_load_slew);
        let buffer_cell = self.find_target_cell(self.buffers.lowest_drive, load_cap, false);
        let factor = self.slew_rc_factor;

        let mut repeater_left = false;
        let mut repeater_right = false;
        let slew_violation = r_drvr * load_cap * factor > self.margined_slew(max_load_slew);
        if slew_violation {
            let slack_left = self.margined_slew(left.max_load_slew) - r_drvr * left.cap * factor;
            let slack_right = self.margined_slew(right.max_load_slew) - r_drvr * right.cap * factor;
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
        let max_length = self.options.max_wire_length;
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
        to: &Tree,
        driver: PinId,
        max_cap: f64,
        r_drvr: f64,
    ) -> Result<Branch, RepairError> {
        let mut branch = self.repair_branch(to, driver, max_cap, r_drvr)?;
        let to_loc = tree_location(to);
        let dbu = self.db.dbu().get() as f64;
        let rc = self.graph.wire_rc();
        let (wire_res, wire_cap) = (rc.resistance, rc.capacitance);
        let factor = self.slew_rc_factor;
        let max_length = self.options.max_wire_length;
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
        let mut max_load_slew_margined = self.margined_slew(max_load_slew);
        let mut ref_cap = branch.cap;
        let mut load_cap = meters(length) * wire_cap + ref_cap;
        let mut load_slew = elmore_slew(length, ref_cap);
        let mut buffer_cell = self.find_target_cell(self.buffers.lowest_drive, load_cap, false);
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
            max_load_slew_margined = self.margined_slew(max_load_slew);
            load_slew = elmore_slew(length, ref_cap);
            buffer_cell = self.find_target_cell(self.buffers.lowest_drive, load_cap, false);
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
        let dbu = self.db.dbu().get() as f64;
        let to_dbu = |m: f64| (m * dbu * 1e6) as i64;
        let rc = self.graph.wire_rc();
        let (wire_res, wire_cap) = (rc.resistance, rc.capacitance);
        let max_length = self.options.max_wire_length;
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
            let c =
                input.r_drvr * input.ref_cap - input.max_load_slew_margined / self.slew_rc_factor;
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
        let lib = self.link.liberty.cell(buffer);
        let master = self
            .link
            .macro_of(buffer)
            .ok_or_else(|| RepairError::NoMaster(lib.name.clone()))?;
        let Some((in_port, out_port)) = lib.buffer_ports() else {
            return Ok(false);
        };
        let (Some(in_pin), Some(out_pin)) = (
            self.link.macro_pin_of_port(buffer, in_port),
            self.link.macro_pin_of_port(buffer, out_port),
        ) else {
            return Ok(false);
        };
        let old_net = self.db.pins[branch.loads[0].index()].net;
        let name = self.namer.instance(self.db, reason);
        let cell = self
            .db
            .add_cell(name, master, at, Orientation::N, PlacementStatus::Placed)?;
        let net_name = self.namer.net(self.db, "net");
        let new_net = self.db.add_net(&net_name)?;
        self.db.nets[new_net.index()].signal_use = self.db.nets[old_net.index()].signal_use;
        for &load in &branch.loads {
            self.db.move_pin(load, new_net);
        }
        let input = self
            .db
            .connect_cell_pin(cell, MacroPinId::new(in_pin), old_net);
        let output = self
            .db
            .connect_cell_pin(cell, MacroPinId::new(out_pin), new_net);
        self.summary.inserted += 1;

        {
            let netlist = Netlist::new(self.db, self.link);
            self.graph.grow(&netlist);
            self.graph.update_net(&netlist, self.clocks, new_net);
        }
        let _ = self.resize_to_target_slew(output)?;
        let sized = self.link.cell(self.db, cell).map_or(buffer, |(id, _)| id);
        let sized_cell = self.link.liberty.cell(sized);
        let (sized_in, _) = sized_cell.buffer_ports().unwrap_or((in_port, out_port));
        let in_lib_port = &sized_cell.ports[sized_in];
        branch.loads = vec![input];
        branch.cap = in_lib_port.capacitance();
        branch.fanout = self.link.liberty.fanout_load(in_lib_port);
        branch.max_load_slew = self.max_input_slew(in_lib_port);
        Ok(true)
    }

    /// Sizes the cell driving `driver` to the size whose target load is
    /// nearest its load, as `Resizer::resizeToTargetSlew`; returns whether
    /// the cell changed.
    fn resize_to_target_slew(&mut self, driver: PinId) -> Result<bool, RepairError> {
        let (cell, lib_id, load_cap, revisiting) = {
            let netlist = Netlist::new(self.db, self.link);
            let PinOwner::Cell { cell, .. } = self.db.pins[driver.index()].owner else {
                return Ok(false);
            };
            let Some((lib_id, lib)) = self.link.cell(self.db, cell) else {
                return Ok(false);
            };
            if lib.dont_touch || !self.link.is_logic_std_cell(self.db, cell) {
                return Ok(false);
            }
            let multiple_outputs = lib.ports.iter().filter(|p| p.is_output()).count() > 1;
            let revisiting = multiple_outputs && !self.resized_multi_output.insert(cell);
            (
                cell,
                lib_id,
                self.graph.load_cap(&netlist, driver),
                revisiting,
            )
        };
        if load_cap <= 0.0 {
            return Ok(false);
        }
        let target = self.find_target_cell(lib_id, load_cap, revisiting);
        if target == lib_id {
            return Ok(false);
        }
        let changed = self.replace_cell(cell, target)?;
        Ok(changed && !revisiting)
    }

    /// The swappable size of `cell` whose target load is nearest
    /// `load_cap`, buffers and inverters also preferring the faster, as
    /// `Resizer::findTargetCell`.
    fn find_target_cell(&self, cell: LibCellId, load_cap: f64, revisiting: bool) -> LibCellId {
        let candidates = self.link.swappable_cells(cell);
        if candidates.is_empty() {
            return cell;
        }
        let lib = self.link.liberty.cell(cell);
        let is_buf_inv = lib.is_buffer() || lib.is_inverter();
        let target_of = |c: LibCellId| self.targets.load[c.index()];
        let delay_of = |c: LibCellId| {
            if is_buf_inv {
                buffer_delay(self.link.liberty.cell(c), &self.targets.slews, load_cap)
            } else {
                0.0
            }
        };
        let mut best = cell;
        let mut best_load = target_of(cell);
        let mut best_dist = (load_cap - best_load).abs();
        let mut best_delay = delay_of(cell);
        for candidate in candidates {
            let target = target_of(candidate);
            let dist = (load_cap - target).abs();
            let delay = delay_of(candidate);
            let better = if is_buf_inv {
                (delay < best_delay && dist < best_dist * 1.1)
                    || (dist < best_dist && delay < best_delay * 1.1)
            } else {
                dist < best_dist && (!revisiting || target > best_load)
            };
            if better {
                best = candidate;
                best_dist = dist;
                best_load = target;
                best_delay = delay;
            }
        }
        best
    }
}

/// The factor between a driver's resistance times its load and the slew
/// that RC gives at the library's thresholds, as `computeSlewRCFactor`.
fn slew_rc_factor(link: &Link<'_>) -> Result<f64, RepairError> {
    let t = &link.liberty.thresholds;
    let mut factor: f64 = 0.0;
    for edge in Edge::BOTH {
        let (low, high) = match edge {
            Edge::Rise => (1.0 - t.slew_upper.rise, 1.0 - t.slew_lower.rise),
            Edge::Fall => (t.slew_lower.fall, t.slew_upper.fall),
        };
        let t_high = -high.ln();
        let t_low = -low.ln();
        let edge_factor = (t_low - t_high) / t.slew_derate;
        if edge_factor <= 0.1 || edge_factor >= 10.0 {
            return Err(RepairError::ShapeFactor(edge_factor));
        }
        factor = factor.max(edge_factor);
    }
    Ok(factor * (1.0 + SLEW_RC_PESSIMISM))
}

/// Target slews from the buffers and the load each usable cell meets them
/// at, as `Resizer::findBufferTargetSlews` and `findTargetLoads`.
fn find_targets(link: &Link<'_>, buffers: &BufferSet) -> Targets {
    let liberty = link.liberty;
    let mut sums = EdgeValues::default();
    let mut counts = [0usize; 2];
    for &buffer in &buffers.cells {
        let cell = liberty.cell(buffer);
        let Some((input, output)) = cell.buffer_ports() else {
            continue;
        };
        for set in cell
            .arc_sets
            .iter()
            .filter(|s| s.from == input && s.to == output)
        {
            for arc in &set.arcs {
                let in_cap = cell.ports[input].capacitance.get(arc.from_edge);
                let load = in_cap * TARGET_SLEW_LOAD_FACTOR;
                let (_, slew1) = arc.gate_delay(0.0, load);
                let (_, slew2) = arc.gate_delay(slew1, load);
                match arc.to_edge {
                    Edge::Rise => sums.rise += slew2,
                    Edge::Fall => sums.fall += slew2,
                }
                counts[arc.to_edge.index()] += 1;
            }
        }
    }
    let slews = EdgeValues {
        rise: if counts[0] > 0 {
            sums.rise / counts[0] as f64
        } else {
            0.0
        },
        fall: if counts[1] > 0 {
            sums.fall / counts[1] as f64
        } else {
            0.0
        },
    };
    let load = (0..liberty.cells.len())
        .map(|i| {
            let id = LibCellId(i as u32);
            if link.is_link_cell(id) && !link.dont_use(id) {
                find_target_load(liberty.cell(id), &slews)
            } else {
                0.0
            }
        })
        .collect();
    Targets { slews, load }
}

/// The mean over a cell's delay arcs of the load at which each reaches the
/// target output slew from the target input slew.
fn find_target_load(cell: &LibCell, slews: &EdgeValues) -> f64 {
    let mut sum = 0.0;
    let mut count = 0;
    for set in &cell.arc_sets {
        if !matches!(
            set.role,
            ArcRole::Combinational | ArcRole::RegClkToQ | ArcRole::RegSetClr | ArcRole::LatchEnToQ
        ) {
            continue;
        }
        for arc in &set.arcs {
            let in_slew = slews.get(arc.from_edge);
            let out_slew = slews.get(arc.to_edge);
            sum += arc_target_load(arc, in_slew, out_slew);
            count += 1;
        }
    }
    if count == 0 { 0.0 } else { sum / count as f64 }
}

/// The load at which `arc` gives `out_slew` from `in_slew`, by binary
/// search; zero when even no load exceeds it.
fn arc_target_load(arc: &pare_common::db::liberty::TimingArc, in_slew: f64, out_slew: f64) -> f64 {
    let diff = |load: f64| arc.gate_delay(in_slew, load).1 - out_slew;
    let mut cap1 = 0.0;
    let mut cap2 = 1.0e-12;
    let tol = 0.01;
    if diff(cap1) > 0.0 {
        return 0.0;
    }
    let mut diff2 = diff(cap2);
    let mut iterations = 0;
    while (cap1 - cap2).abs() > cap1.max(cap2) * tol && iterations < 200 {
        iterations += 1;
        if diff2 < 0.0 {
            cap1 = cap2;
            cap2 *= 2.0;
            diff2 = diff(cap2);
        } else {
            let cap3 = f64::midpoint(cap1, cap2);
            let diff3 = diff(cap3);
            if diff3 < 0.0 {
                cap1 = cap3;
            } else {
                cap2 = cap3;
                diff2 = diff3;
            }
        }
    }
    cap1
}

/// The worst slew over the delay arcs into `port` at `load_cap` from the
/// target input slews, as `Resizer::gateDelays`.
fn gate_slew(cell: &LibCell, port: usize, slews: &EdgeValues, load_cap: f64) -> f64 {
    cell.delay_arc_sets_to(port)
        .flat_map(|s| s.arcs.iter())
        .map(|arc| arc.gate_delay(slews.get(arc.from_edge), load_cap).1)
        .fold(0.0, f64::max)
}

/// The worst delay of a buffer or inverter at `load_cap` from the target
/// input slews, as `Resizer::bufferDelay`.
fn buffer_delay(cell: &LibCell, slews: &EdgeValues, load_cap: f64) -> f64 {
    let Some((_, output)) = cell.buffer_ports() else {
        return 0.0;
    };
    cell.delay_arc_sets_to(output)
        .flat_map(|s| s.arcs.iter())
        .map(|arc| arc.gate_delay(slews.get(arc.from_edge), load_cap).0)
        .fold(0.0, f64::max)
}

/// The buffered tree of a net's parasitics seen from `driver`, as
/// `Resizer::makeBufferedNetSteiner`.
fn buffered_tree(netlist: &Netlist<'_>, parasitics: &NetParasitics, driver: PinId) -> Option<Tree> {
    let tree = &parasitics.tree;
    let children = tree.children();
    let mut visited_locations: HashSet<(i64, i64)> = HashSet::new();
    build_tree(
        netlist,
        parasitics,
        &children,
        None,
        tree.root,
        driver,
        &mut visited_locations,
    )
}

fn build_tree(
    netlist: &Netlist<'_>,
    parasitics: &NetParasitics,
    children: &[Vec<usize>],
    from: Option<usize>,
    to: usize,
    driver: PinId,
    visited: &mut HashSet<(i64, i64)>,
) -> Option<Tree> {
    let tree = &parasitics.tree;
    let at = tree.nodes[to];
    let mut built: Option<Tree> = None;
    let key = (at.x.get(), at.y.get());
    if to < tree.terminals && visited.insert(key) {
        for &pin in &parasitics.terminal_pins[to] {
            if pin == driver || !netlist.is_load(pin) {
                continue;
            }
            let leaf = Tree::Load { at, pin };
            built = Some(join(built, leaf, at));
        }
    }
    for &child in &children[to] {
        if let Some(sub) = build_tree(
            netlist,
            parasitics,
            children,
            Some(to),
            child,
            driver,
            visited,
        ) {
            built = Some(join(built, sub, at));
        }
    }
    let built = built?;
    if let Some(from) = from
        && tree.nodes[from] != at
    {
        return Some(Tree::Wire {
            from: tree.nodes[from],
            to: Box::new(built),
        });
    }
    Some(built)
}

fn join(existing: Option<Tree>, next: Tree, at: Point<Dbu>) -> Tree {
    match existing {
        None => next,
        Some(left) => Tree::Junction {
            at,
            left: Box::new(left),
            right: Box::new(next),
        },
    }
}

/// Where a tree's top sits: a load at its pin, a junction or wire at its
/// own point.
const fn tree_location(tree: &Tree) -> Point<Dbu> {
    match tree {
        Tree::Load { at, .. } | Tree::Junction { at, .. } | Tree::Wire { from: at, .. } => *at,
    }
}

/// The longest wire from the tree's top to any load, as
/// `BufferedNet::maxLoadWireLength`.
fn tree_wire_length(tree: &Tree) -> i64 {
    match tree {
        Tree::Load { .. } => 0,
        Tree::Junction { left, right, .. } => tree_wire_length(left).max(tree_wire_length(right)),
        Tree::Wire { from, to } => {
            let end = tree_location(to);
            (from.x.get() - end.x.get()).abs()
                + (from.y.get() - end.y.get()).abs()
                + tree_wire_length(to)
        }
    }
}
