//! Rebuffering a net for timing, as `OpenROAD`'s `Rebuffer`.
//!
//! A dynamic program over the net's tree tries every usable buffer at every
//! branch and along every long wire, keeps the options that trade more
//! capacitance for more slack, picks the one giving the driver the most
//! slack, then trades buffers back for area while the slack stays within a
//! small relaxation of that best.

// The Elmore slew estimates keep OpenROAD's grouping so they can be checked.
#![allow(clippy::suboptimal_flops)]

use crate::bnet::{self, Bnet, BnetPtr, BufferSize, Fixed};
use crate::dmp::{self, PiModel};
use crate::library::is_logic_std_cell;
use crate::resizer::{RepairError, Resizer, meters_to_dbu};
use crate::sta::Sta;
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{CellId, LayerId, PinId};
use pare_common::db::liberty::{
    ArcRole, ArcSetId, Edge, EdgeValues, LibCell, LibCellId, LibPortId, MinMax,
};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::units::{Farads, Meters, Ohms, Seconds};

mod area;
mod limits;
mod timing;
mod tree;

use limits::{
    buffer_size, find_fast_buffers, least_length, margined, wire_length_limit_by_load_slew,
    wire_length_limit_by_max_cap,
};
pub use limits::{max_wire_length, max_wire_length_of};
use timing::{
    Frontier, critical_path_delay, insert_assured, prune_cap_against_area, with_arrivals,
};
pub(crate) use tree::can_remove_buffer;
use tree::{
    collect_imported_buffers, collect_sinks, export_tree, has_top_level_output, is_port_buffer,
    strip_buffers,
};

/// Margin kept in hand on slew limits while rebuffering, in percent, as
/// `Rebuffer::slew_margin_`.
const SLEW_MARGIN: f64 = 20.0;

/// Margin kept in hand on load limits while rebuffering, in percent, as
/// `Rebuffer::cap_margin_`.
const CAP_MARGIN: f64 = 20.0;

/// Elmore factor for 20 to 80 percent slew thresholds, as
/// `Rebuffer::elmore_skew_factor_`.
const ELMORE_SKEW_FACTOR: f64 = 1.39;

/// How much of the critical delay the area recovery may give back, as
/// `Rebuffer::relaxation_factor_`.
const RELAXATION_FACTOR: f64 = 0.01;

/// Rounds of buffering for timing, as `Rebuffer::rebufferPin` runs them.
const TIMING_ROUNDS: usize = 3;

/// Rounds of area recovery, each allowed a larger share of the slack, as
/// `Rebuffer::rebufferPin` runs them.
const AREA_ROUNDS: usize = 5;

/// The timing of a path point at the driver being rebuffered: when it
/// arrives and through which arc from which input, as `OpenROAD` keeps the
/// arrival paths for each edge.
#[derive(Clone, Copy, Debug, PartialEq)]
struct DriverPoint {
    arrival: Seconds,
    /// The input the data came through and the arc set, or none for a
    /// start such as an input port.
    from: Option<(PinId, Edge, ArcSetId)>,
}

/// What the search knows about the driver of the net at hand.
#[derive(Clone, Debug)]
struct Driver {
    pin: PinId,
    cell: LibCellId,
    port: LibPortId,
    fanout_limit: f64,
    max_slew: Seconds,
    load_high_water_mark: Farads,
    arrival_paths: EdgeValues<Option<DriverPoint>>,
}

/// Area recovery found no option meeting its slack threshold, which
/// `OpenROAD` reports as RSZ 501, 2004 or 2022.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct AreaRecoveryFailed;

/// The error area recovery's failure on `driver`'s net is.
fn area_recovery_error(resizer: &Resizer<'_, Sta>, driver: PinId) -> RepairError {
    RepairError::AreaRecovery(resizer.db.pin_name(driver))
}

/// Whether buffering may restructure a tree at its junctions, as
/// `bufferForTiming`'s `allow_topology_rewrite`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Topology {
    /// A junction may pull its less critical loads under a new buffer.
    Rewritable,
    /// The tree branches as it does.
    Kept,
}

impl Topology {
    /// What `Rebuffer::rebufferPin` allows in `resizer`'s session:
    /// rewrites on placement parasitics only, never along global routes.
    const fn of_session(resizer: &Resizer<'_, Sta>) -> Self {
        if resizer.has_global_routes() {
            Self::Kept
        } else {
            Self::Rewritable
        }
    }
}

/// The options for the two sides of a junction, each best slack last.
#[derive(Clone, Copy, Debug)]
struct JunctionSides<'o> {
    left: &'o [BnetPtr],
    right: &'o [BnetPtr],
}

impl JunctionSides<'_> {
    /// The next pair of options down from `(left, right)`, stepping the
    /// side whose next option has more slack, that sums to less
    /// capacitance than `best_cap`; none once the stepped side runs out.
    fn next_cheaper_pair(
        &self,
        (left, right): (usize, usize),
        best_cap: Farads,
    ) -> Option<(usize, usize)> {
        let (mut li, mut ri) = (left, right);
        loop {
            let next_left = if li > 0 {
                self.left[li - 1].slack
            } else {
                -Fixed::INF
            };
            let next_right = if ri > 0 {
                self.right[ri - 1].slack
            } else {
                -Fixed::INF
            };
            if next_left > next_right {
                li = li.checked_sub(1)?;
            } else {
                ri = ri.checked_sub(1)?;
            }
            if self.left[li].cap + self.right[ri].cap < best_cap {
                return Some((li, ri));
            }
        }
    }
}

/// The delay, slack correction and slew at the driver for a tree.
#[derive(Clone, Copy, Debug, PartialEq)]
struct DriverTiming {
    delay: Seconds,
    correction: Seconds,
    slew: Seconds,
}

impl DriverTiming {
    /// No delay, correction or slew, for a tree with no driver or no
    /// timing.
    const NONE: Self = Self {
        delay: Seconds::ZERO,
        correction: Seconds::ZERO,
        slew: Seconds::ZERO,
    };
}

/// The driver's timing on `edge` through the arc `point` arrived by, the
/// gate timed again at `load`; none where `point` starts a path.
fn timing_through(
    resizer: &Resizer<'_, Sta>,
    cell: &LibCell,
    point: &DriverPoint,
    edge: Edge,
    load: Farads,
) -> Option<DriverTiming> {
    let (from_pin, from_edge, set) = point.from?;
    let arc_set = &cell.arc_sets[set];
    let in_slew = if arc_set.role == ArcRole::RegClkToQ && resizer.clocks.is_ideal(from_pin) {
        Seconds::ZERO
    } else {
        resizer.graph.slew(from_pin).get(from_edge)
    };
    let mut delay = Seconds::ZERO;
    let mut slew = Seconds::ZERO;
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
                rpi: Ohms::ZERO,
                c1: Farads::ZERO,
            },
        );
        delay = delay.max(result.delay);
        slew = slew.max(result.slew);
    }
    let prev_arrival = resizer
        .sta()
        .arrival(from_pin, from_edge, MinMax::Max)
        .unwrap_or(Seconds::ZERO);
    Some(DriverTiming {
        delay,
        correction: point.arrival - (prev_arrival + delay),
        slew,
    })
}

/// The rebuffering search with the buffers it may use.
#[derive(Debug)]
pub struct Rebuffer {
    sizes: Vec<BufferSize>,
    max_wire_length: Option<Dbu>,
    wire_length_step: Option<Dbu>,
    driver: Option<Driver>,
    /// Buffers taken out while rebuffering whole designs.
    pub removed: usize,
    /// Buffers put in while rebuffering whole designs.
    pub inserted: usize,
}

impl Rebuffer {
    /// Characterizes the buffers the search may insert, as `Rebuffer::init`
    /// and `initOnCorner`.
    pub fn new(resizer: &Resizer<'_, Sta>) -> Result<Self, RepairError> {
        let max_wire_length = max_wire_length(resizer);
        let mut sizes: Vec<BufferSize> = find_fast_buffers(resizer)
            .into_iter()
            .map(|buffer| buffer_size(resizer, buffer))
            .collect();
        sizes.sort_by(|a, b| a.input_cap.total_cmp(&b.input_cap));
        let smallest = sizes.first().ok_or(RepairError::NoBuffers)?;
        let wire_length_step = least_length([
            max_wire_length,
            wire_length_limit_by_load_slew(resizer, smallest)?,
            wire_length_limit_by_max_cap(resizer, smallest),
        ]);
        Ok(Self {
            sizes,
            max_wire_length,
            wire_length_step,
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
    fn wire_length_step_on(
        &self,
        resizer: &Resizer<'_, Sta>,
        layer: Option<LayerId>,
    ) -> Option<Dbu> {
        let (res, cap) = resizer.tree_rc().wire(layer);
        let (res, cap) = (res.get(), cap.get());
        if layer.is_none() || cap <= 0.0 || res <= 0.0 {
            return self.wire_length_step;
        }
        let Some(smallest) = self.sizes.first() else {
            return self.wire_length_step;
        };
        let dbu = resizer.db.dbu();
        let lib = resizer.link.liberty.cell(smallest.buffer.id);
        let input_cap = smallest.input_cap.get();
        let drive_resistance = smallest.drive_resistance.get();
        let cap_limit = resizer
            .link
            .liberty
            .capacitance_limit(lib, &lib.ports[smallest.buffer.output]);
        let cap_length = cap_limit.map(|limit| {
            let slack = margined(limit, CAP_MARGIN).get() - input_cap;
            meters_to_dbu(Meters::new(slack / cap), dbu).max(Dbu::ZERO)
        });
        let max_slew = margined(smallest.input_max_slew, SLEW_MARGIN).get();
        let a = res * cap;
        let b = res * input_cap + drive_resistance * cap;
        let c = drive_resistance * input_cap - max_slew / ELMORE_SKEW_FACTOR;
        let discriminant = b * b - 4.0 * a * c;
        let mut slew_length = None;
        if discriminant >= 0.0 {
            let meters = (-b + discriminant.sqrt()) / (2.0 * a);
            if meters > 0.0 && meters <= 1.0 {
                slew_length = Some(meters_to_dbu(Meters::new(meters), dbu));
            }
        }
        least_length([self.max_wire_length, slew_length, cap_length])
    }

    /// Rebuffers the net `driver` drives for timing, as
    /// `Rebuffer::rebufferPin`; returns how many buffers went in.
    pub(crate) fn rebuffer_pin(
        &mut self,
        resizer: &mut Resizer<'_, Sta>,
        driver: PinId,
    ) -> Result<usize, RepairError> {
        let PinOwner::Cell { .. } = resizer.db.pins[driver].owner else {
            return Ok(0);
        };
        let net = resizer.db.pins[driver].net;
        if has_top_level_output(resizer, net) {
            return Ok(0);
        }
        if !self.set_driver(resizer, driver) {
            return Ok(0);
        }
        let Some(tree) = resizer.buffered_tree(driver)? else {
            return Ok(0);
        };
        let mut tree = self.annotate_load_slacks(resizer, &tree, driver);
        let topology = Topology::of_session(resizer);
        for _ in 0..TIMING_ROUNDS {
            match self.buffer_for_timing(resizer, &tree, topology) {
                Some(better) => tree = better,
                None => return Ok(0),
            }
        }
        let timing = self.driver_timing(resizer, &tree);
        let relaxation = (timing.delay.max(Seconds::ZERO)
            + critical_path_delay(&tree).to_seconds())
            * RELAXATION_FACTOR;
        let target = self.slack_at_driver(resizer, &tree) - Fixed::from_seconds(relaxation);
        for round in 0..AREA_ROUNDS {
            let alpha = (round + 1) as f64 / AREA_ROUNDS as f64;
            tree = self
                .recover_area(resizer, &tree, target, alpha)
                .map_err(|AreaRecoveryFailed| area_recovery_error(resizer, driver))?;
        }
        export_tree(resizer, &tree, "rebuffer")
    }

    /// Strips the buffers from every net and buffers it again for timing,
    /// as `Rebuffer::fullyRebuffer` does for timing-driven placement.
    pub fn fully_rebuffer(&mut self, resizer: &mut Resizer<'_, Sta>) -> Result<(), RepairError> {
        for driver in rebuffer_candidates(resizer) {
            self.rebuffer_driver(resizer, driver)?;
        }
        Ok(())
    }

    /// Buffers `driver`'s net again for timing, then recovers area within
    /// a relaxed slack target, and puts the result in the design.
    fn rebuffer_driver(
        &mut self,
        resizer: &mut Resizer<'_, Sta>,
        driver: PinId,
    ) -> Result<(), RepairError> {
        if resizer.netlist().is_retired(driver) || !self.set_driver(resizer, driver) {
            return Ok(());
        }
        let Some(original) = self.import_buffer_tree(resizer, driver)? else {
            return Ok(());
        };
        let original = self.annotate_load_slacks(resizer, &original, driver);
        let slack = self.slack_at_driver(resizer, &original).to_seconds();
        let sta_slack = resizer.sta().slack(driver, MinMax::Max).time_or_infinity();
        let slack_error = slack - sta_slack;
        let Some(unbuffered) = resteiner(resizer, &strip_buffers(&original)) else {
            return Ok(());
        };
        let Some(timing_tree) = self.buffer_rounds_for_timing(resizer, unbuffered) else {
            return Ok(());
        };
        let target = self.area_target(resizer, &timing_tree, slack_error);
        let mut area_tree = timing_tree;
        for round in 0..AREA_ROUNDS {
            let alpha = (round + 1) as f64 / AREA_ROUNDS as f64;
            area_tree = self
                .recover_area(resizer, &area_tree, target, alpha)
                .map_err(|AreaRecoveryFailed| area_recovery_error(resizer, driver))?;
        }
        let old_buffers = collect_imported_buffers(resizer, driver, &original);
        let batch = resizer.batch_edits();
        let replaced = self.replace_buffers(resizer, &area_tree, &old_buffers);
        resizer.flush(batch);
        replaced?;
        Ok(())
    }

    /// `tree` buffered for timing [`TIMING_ROUNDS`] times over; none where
    /// a round finds no buffering.
    fn buffer_rounds_for_timing(
        &mut self,
        resizer: &Resizer<'_, Sta>,
        tree: BnetPtr,
    ) -> Option<BnetPtr> {
        let mut tree = tree;
        for _ in 0..TIMING_ROUNDS {
            tree = self.buffer_for_timing(resizer, &tree, Topology::Rewritable)?;
        }
        Some(tree)
    }

    /// The slack area recovery must keep at the driver of `timing_tree`:
    /// its slack less a relaxation that grows with the slack to spare and
    /// the path's delay, `slack_error` being how far the tree's slack sits
    /// above the analysis's.
    fn area_target(
        &self,
        resizer: &Resizer<'_, Sta>,
        timing_tree: &Bnet,
        slack_error: Seconds,
    ) -> Fixed {
        if timing_tree.slack_edges.is_none() {
            return -Fixed::INF;
        }
        let timing = self.driver_timing(resizer, timing_tree);
        let relaxation = (self.slack_at_driver(resizer, timing_tree).to_seconds()
            - slack_error.min(Seconds::ZERO))
        .max(Seconds::ZERO)
            / 4.0
            + (timing.delay.max(Seconds::ZERO) + critical_path_delay(timing_tree).to_seconds())
                * RELAXATION_FACTOR;
        self.slack_at_driver(resizer, timing_tree) - Fixed::from_seconds(relaxation)
    }

    /// Puts the buffers of `tree` into the design and takes `old_buffers`
    /// out.
    fn replace_buffers(
        &mut self,
        resizer: &mut Resizer<'_, Sta>,
        tree: &BnetPtr,
        old_buffers: &[CellId],
    ) -> Result<(), RepairError> {
        self.inserted += export_tree(resizer, tree, "place")?;
        for &buffer in old_buffers {
            if !resizer.db.cells[buffer].retired {
                let _ = resizer.remove_buffer(buffer)?;
                self.removed += 1;
            }
        }
        Ok(())
    }

    /// Notes the driver at hand: its fanout and slew limits, as `setPin`;
    /// whether it has a Liberty port to model it by.
    #[must_use]
    fn set_driver(&mut self, resizer: &Resizer<'_, Sta>, pin: PinId) -> bool {
        let Some((cell, port)) = resizer.link.pin_port(resizer.db, pin) else {
            self.driver = None;
            return false;
        };
        let max_slew = resizer
            .driver_slew_limit(pin)
            .map_or(Seconds::INFINITY, |l| margined(l, SLEW_MARGIN));
        self.driver = Some(Driver {
            pin,
            cell,
            port,
            fanout_limit: resizer.fanout_limit(pin).unwrap_or(f64::INFINITY),
            max_slew,
            load_high_water_mark: Farads::ZERO,
            arrival_paths: EdgeValues::default(),
        });
        true
    }
}

/// The drivers [`Rebuffer::fully_rebuffer`] buffers again, from the
/// outputs back: every signal driver but tristates, constants, clocks and
/// the logic buffers it strips, as `Rebuffer::fullyRebuffer` picks them.
fn rebuffer_candidates(resizer: &Resizer<'_, Sta>) -> Vec<PinId> {
    let netlist = resizer.netlist();
    let mut drivers = resizer.graph.drivers_by_level(&netlist);
    drivers.retain(|&driver| {
        let net = resizer.db.pins[driver].net;
        let PinOwner::Cell { cell, .. } = resizer.db.pins[driver].owner else {
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
                || !is_logic_std_cell(resizer.db, cell)
                || is_port_buffer(resizer, cell))
    });
    drivers.reverse();
    drivers
}

/// A fresh Steiner tree over the loads of `tree` from its top, as
/// `resteiner`; none when it has no loads.
fn resteiner(resizer: &Resizer<'_, Sta>, tree: &BnetPtr) -> Option<BnetPtr> {
    let mut sinks = Vec::new();
    collect_sinks(tree, &mut sinks);
    bnet::over_subtrees(tree.at, &sinks, &resizer.tree_rc())
}

/// A wire node from `at` down to `to` on `layer`, or at the signal wire
/// RC without one.
fn wire(
    resizer: &Resizer<'_, Sta>,
    at: Point<Dbu>,
    layer: Option<LayerId>,
    to: BnetPtr,
) -> BnetPtr {
    Bnet::wire(at, to, layer, &resizer.tree_rc())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bnet::{Kind, PinLoad};
    use crate::resizer::GlobalRouting;
    use crate::route_parasitics::RouteRc;
    use crate::testing::{
        cells, clock_cells_source, design, metal2_wires, pin, with_session_inputs,
        with_timed_session, with_wired_session,
    };
    use pare_common::db::design::NetlistDB;
    use pare_common::db::global_route::{NetRoute, RouteSource};
    use pare_common::db::indices::NetId;
    use std::rc::Rc;

    /// A flop launching into another, both clocked through a clock buffer
    /// whose output slew reaches their clock pins.
    fn buffered_clock_pair() -> NetlistDB {
        design(
            "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN pair ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 3 ;\n- cb CLKBUF_X3 + PLACED ( 2000 2800 ) N ;\n\
             - f1 DFF_X1 + PLACED ( 12000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 40000 2800 ) N ;\n\
             END COMPONENTS\nPINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 3 ;\n- clk ( PIN clk ) ( cb A ) ;\n- ck ( cb Z ) ( f1 CK ) ( f2 CK ) ;\n\
             - q ( f1 Q ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n",
        )
    }

    #[test]
    fn rebuffering_sees_every_load_of_a_net_when_two_share_a_place() {
        let mut db = design(
            "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN fork ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 4 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - b1 BUF_X1 + PLACED ( 30000 2800 ) N ;\n- b2 BUF_X1 + PLACED ( 30000 2800 ) N ;\n\
             - b3 BUF_X1 + PLACED ( 50000 2800 ) N ;\nEND COMPONENTS\n\
             PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 2 ;\n- clk ( PIN clk ) ( f1 CK ) ;\n\
             - q ( f1 Q ) ( b1 A ) ( b2 A ) ( b3 A ) ;\nEND NETS\nEND DESIGN\n",
        );
        let q = pin(&db, "f1/Q");
        let clock = "create_clock -name clk -period 0.5 [get_ports clk]\n";

        with_timed_session(&mut db, &cells(&clock_cells_source()), clock, |resizer| {
            let tree = resizer.buffered_tree(q).unwrap().unwrap();
            let mut sinks = Vec::new();
            collect_sinks(&tree, &mut sinks);
            let mut names: Vec<String> = sinks
                .iter()
                .filter_map(|s| match s.kind {
                    Kind::Load { pin } => Some(resizer.db.pin_name(pin)),
                    _ => None,
                })
                .collect();
            names.sort();

            assert_eq!(names, ["b1/A", "b2/A", "b3/A"]);
        });
    }

    /// A flop launching through buffer `bf`, placed with `status`, into
    /// another.
    fn buffer_between_flops(status: &str) -> NetlistDB {
        design(&format!(
            "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN between ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 3 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - bf BUF_X1 + {status} ( 30000 2800 ) N ;\n- f2 DFF_X1 + PLACED ( 50000 2800 ) N ;\n\
             END COMPONENTS\nPINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 3 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n\
             - q ( f1 Q ) ( bf A ) ;\n- d ( bf Z ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n"
        ))
    }

    /// Whether setup repair may take buffer `bf` out of `db` under
    /// `constraints`.
    fn removable_under(mut db: NetlistDB, constraints: &str) -> bool {
        let mut removable = false;
        with_timed_session(&mut db, &cells(&clock_cells_source()), constraints, |r| {
            removable = can_remove_buffer(r, r.db.cell_id("bf").unwrap());
        });
        removable
    }

    const CLOCK: &str = "create_clock -name clk -period 0.5 [get_ports clk]\n";

    #[test]
    fn a_buffer_that_cannot_drive_its_own_input_within_its_slew_limit_is_refused() {
        let mut db = buffer_between_flops("PLACED");
        let tight = clock_cells_source().replacen(
            "capacitance : 0.97 ;",
            "capacitance : 0.97 ; max_transition : 0.000001 ;",
            1,
        );

        with_wired_session(&mut db, &cells(&tight), CLOCK, |resizer| {
            let error = Rebuffer::new(resizer).err();

            assert_eq!(
                error,
                Some(RepairError::WireLengthLimit("BUF_X1".to_string()))
            );
        });
    }

    #[test]
    fn rebuffering_a_whole_design_replaces_a_buffer_chain_and_keeps_it_timed() {
        let mut db = design(
            "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN chain ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 800000 40000 ) ;\n\
             COMPONENTS 4 ;\n- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
             - b1 BUF_X1 + PLACED ( 4000 2800 ) N ;\n- b2 BUF_X1 + PLACED ( 6000 2800 ) N ;\n\
             - f2 DFF_X1 + PLACED ( 602000 2800 ) N ;\nEND COMPONENTS\n\
             PINS 1 ;\n- clk + NET clk + DIRECTION INPUT ;\nEND PINS\n\
             NETS 4 ;\n- clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ;\n- q ( f1 Q ) ( b1 A ) ;\n\
             - m ( b1 Z ) ( b2 A ) ;\n- d ( b2 Z ) ( f2 D ) ;\nEND NETS\nEND DESIGN\n",
        );
        let lib = cells(&clock_cells_source());
        let d = pin(&db, "f2/D");

        with_wired_session(&mut db, &lib, CLOCK, |resizer| {
            let mut rebuffer = Rebuffer::new(resizer).unwrap();

            rebuffer.fully_rebuffer(resizer).unwrap();

            assert!(rebuffer.removed > 0, "{rebuffer:?}");
            assert!(resizer.sta().slack(d, MinMax::Max).time().is_some());
            let netlist = resizer.netlist();
            let q = pin(resizer.db, "f1/Q");
            assert!(netlist.loads(resizer.db.pins[q].net).count() > 0);
        });
    }

    #[test]
    fn a_placed_buffer_may_be_taken_out_and_a_fixed_one_may_not() {
        assert!(removable_under(buffer_between_flops("PLACED"), CLOCK));
        assert!(!removable_under(buffer_between_flops("FIXED"), CLOCK));
    }

    #[test]
    fn a_buffer_between_constrained_ports_may_be_taken_out() {
        let db = design(
            "VERSION 5.8 ;\nDIVIDERCHAR \"/\" ;\nBUSBITCHARS \"[]\" ;\nDESIGN ports ;\n\
             UNITS DISTANCE MICRONS 2000 ;\nDIEAREA ( 0 0 ) ( 80000 40000 ) ;\n\
             COMPONENTS 1 ;\n- bf BUF_X1 + PLACED ( 30000 2800 ) N ;\nEND COMPONENTS\n\
             PINS 3 ;\n- clk + NET clk + DIRECTION INPUT ;\n- in + NET in + DIRECTION INPUT ;\n\
             - out + NET out + DIRECTION OUTPUT ;\nEND PINS\n\
             NETS 3 ;\n- clk ( PIN clk ) ;\n- in ( PIN in ) ( bf A ) ;\n\
             - out ( bf Z ) ( PIN out ) ;\nEND NETS\nEND DESIGN\n",
        );
        let delays = format!(
            "{CLOCK}set_input_delay 0.1 -clock clk [get_ports in]\n\
             set_output_delay 0.1 -clock clk [get_ports out]\n"
        );

        assert!(removable_under(db, &delays));
    }

    #[test]
    fn a_buffer_with_its_own_derate_may_not_be_taken_out() {
        let derated = format!("{CLOCK}set_timing_derate -late 1.1 [get_cells bf]\n");

        assert!(!removable_under(buffer_between_flops("PLACED"), &derated));
    }

    /// A load of `cap_ff` femtofarads at the origin with `slack_ns`
    /// nanoseconds of slack.
    fn load_with(pin: PinId, cap_ff: f64, slack_ns: f64) -> BnetPtr {
        let load = PinLoad {
            cap: Farads::new(cap_ff * 1e-15),
            fanout: 1.0,
            max_slew: Seconds::INFINITY,
        };
        let slack = Fixed::from_seconds(Seconds::new(slack_ns * 1e-9));
        Bnet::load(Point::new(Dbu::ZERO, Dbu::ZERO), pin, load).with_slack(slack, None)
    }

    /// Whether a junction pulls a side under a buffer of its own making.
    fn is_rewritten(junction: &Bnet) -> bool {
        matches!(&junction.kind, Kind::Junction { left, .. }
            if matches!(&left.kind, Kind::Buffer { to, .. }
                if matches!(to.under_wires().kind, Kind::Junction { .. })))
    }

    #[test]
    fn a_junction_rewrites_its_topology_only_where_allowed() {
        let mut db = buffer_between_flops("PLACED");
        let pin = pin(&db, "f2/D");

        with_timed_session(&mut db, &cells(&clock_cells_source()), CLOCK, |resizer| {
            let rebuffer = Rebuffer::new(resizer).unwrap();
            let critical = Bnet::junction(
                Point::new(Dbu::ZERO, Dbu::ZERO),
                load_with(pin, 1.0, 1.0),
                load_with(pin, 5.0, 5.0),
            );
            let buffered = Bnet::buffer(
                Point::new(Dbu::ZERO, Dbu::ZERO),
                &rebuffer.sizes[0],
                load_with(pin, 5.0, 10.0),
                Fixed::ZERO,
            );
            let node = Bnet::junction(
                Point::new(Dbu::ZERO, Dbu::ZERO),
                Rc::clone(&critical),
                Rc::clone(&buffered),
            );
            let (left, right) = ([critical], [buffered]);
            let sides = JunctionSides {
                left: &left,
                right: &right,
            };

            let rewritable = rebuffer.junction_options(resizer, &node, sides, Topology::Rewritable);
            let kept = rebuffer.junction_options(resizer, &node, sides, Topology::Kept);

            assert!(rewritable.iter().any(|o| is_rewritten(o)));
            assert!(!kept.iter().any(|o| is_rewritten(o)));
        });
    }

    /// Global routes that have none to give.
    struct NoRoutes;

    impl RouteSource for NoRoutes {
        fn update(&mut self, _: &NetlistDB, _: &[NetId]) {}

        fn begin_trial(&mut self) {}

        fn keep_trial(&mut self) {}

        fn restore_trial(&mut self) {}

        fn route(&self, _: &NetlistDB, _: NetId) -> Option<NetRoute> {
            None
        }
    }

    #[test]
    fn rebuffering_rewrites_topology_on_placement_parasitics_only() {
        let mut db = buffer_between_flops("PLACED");
        let lib = cells(&clock_cells_source());
        let rc = RouteRc::new(&db.library, &[], &[], LayerId(0));
        let wires = metal2_wires(&db);

        with_timed_session(&mut db, &lib, CLOCK, |resizer| {
            assert_eq!(Topology::of_session(resizer), Topology::Rewritable);
        });
        with_session_inputs(&mut db, &lib, CLOCK, wires, |inputs, sdc| {
            let mut source = NoRoutes;
            let routes = GlobalRouting {
                source: &mut source,
                rc,
                legalize: &|_, _| {},
            };
            let resizer = Resizer::on_global_routes(inputs, sdc, routes).unwrap();

            assert_eq!(Topology::of_session(&resizer), Topology::Kept);
        });
    }

    #[test]
    fn a_flop_on_a_propagated_clock_is_rebuffered_from_its_clock_pin_slew() {
        let mut db = buffered_clock_pair();
        let q = pin(&db, "f1/Q");
        let constraints = "create_clock -name clk -period 0.5 [get_ports clk]\n\
                           set_propagated_clock [all_clocks]\n";

        with_timed_session(
            &mut db,
            &cells(&clock_cells_source()),
            constraints,
            |resizer| {
                let mut rebuffer = Rebuffer::new(resizer).unwrap();
                assert!(rebuffer.set_driver(resizer, q));
                let tree = resizer.buffered_tree(q).unwrap().unwrap();
                let tree = rebuffer.annotate_load_slacks(resizer, &tree, q);

                let seen = rebuffer.slack_at_driver(resizer, &tree).to_seconds();
                let timed = resizer.sta().slack(q, MinMax::Max).time().unwrap();

                assert!(
                    (seen - timed).abs() < Seconds::new(1e-14),
                    "{seen:?} vs {timed:?}"
                );
            },
        );
    }
}
