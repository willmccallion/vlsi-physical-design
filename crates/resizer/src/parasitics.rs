//! Wire parasitics estimated from a placement, as `OpenROAD`'s
//! `estimate_parasitics -placement`.
//!
//! Each net gets a Steiner tree over its pins with the signal layer's
//! resistance and capacitance per length along the branches, reduced for
//! each driver to a pi model and an Elmore delay to every load, as `OpenSTA`
//! reduces it.

use crate::dmp::PiModel;
use crate::netlist::Netlist;
use crate::steiner::{self, SteinerTree};
use pare_common::db::global_route::NetRoute;
use pare_common::db::indices::{IdVec, LayerId, NetId, PinId};
use pare_common::db::liberty::Edge;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::units::{
    FARADS_PER_FEMTOFARAD, Farads, FaradsPerMeter, MICRONS_PER_METER, Meters, Ohms, OhmsPerMeter,
    Seconds,
};

/// The resistance `OpenROAD` gives a zero-length connection so the network
/// stays joined.
const JOIN_RESISTANCE: Ohms = Ohms::new(1.0e-3);

pare_common::define_index!(
    /// A node of a net's parasitic network.
    RcNode(u32)
);

/// A resistor of a parasitic network.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Resistor {
    /// The node at one end.
    pub a: RcNode,
    /// The node at the other end.
    pub b: RcNode,
    /// Its resistance.
    pub resistance: Ohms,
}

/// Resistance and capacitance per length of signal wire.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WireRc {
    /// Resistance per length.
    pub resistance: OhmsPerMeter,
    /// Capacitance per length.
    pub capacitance: FaradsPerMeter,
    /// The routing layer signal wires stand on.
    pub layer: LayerId,
}

impl WireRc {
    /// Wire RC from values per micron: ohms and femtofarads.
    #[must_use]
    pub fn per_micron(ohm_per_um: f64, ff_per_um: f64, layer: LayerId) -> Self {
        Self {
            resistance: OhmsPerMeter::new(ohm_per_um * MICRONS_PER_METER),
            capacitance: FaradsPerMeter::new(ff_per_um * FARADS_PER_FEMTOFARAD * MICRONS_PER_METER),
            layer,
        }
    }
}

/// The pi model `OpenSTA` reduces a driver's network to, with the Elmore
/// delay to each load.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct PiElmore {
    /// Capacitance nearest the driver.
    pub c2: Farads,
    /// Resistance between the two capacitances.
    pub rpi: Ohms,
    /// Capacitance beyond it.
    pub c1: Farads,
    /// Every load pin with its Elmore delay from the driver.
    pub elmore: Vec<(PinId, Seconds)>,
    /// Capacitance of the wire alone.
    pub wire_cap: Farads,
    /// Capacitance of the load pins alone.
    pub pin_cap: Farads,
}

impl PiElmore {
    /// The load the driver sees: pins and wire together.
    #[must_use]
    pub fn load_cap(&self) -> Farads {
        self.wire_cap + self.pin_cap
    }

    /// The pi model the driver's delay is fitted to.
    #[must_use]
    pub const fn model(&self) -> PiModel {
        PiModel {
            c2: self.c2,
            rpi: self.rpi,
            c1: self.c1,
        }
    }

    /// The Elmore delay to `load`, zero for a pin it does not reach.
    #[must_use]
    pub fn elmore_to(&self, load: PinId) -> Seconds {
        self.elmore
            .iter()
            .find(|(pin, _)| *pin == load)
            .map_or(Seconds::ZERO, |&(_, delay)| delay)
    }
}

/// What a net's wires follow.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum WireShape {
    /// A Steiner tree over the pins' positions.
    Steiner {
        /// The net's Steiner tree over its pins.
        tree: SteinerTree,
        /// The pins at each terminal of the tree.
        terminal_pins: Vec<Vec<PinId>>,
    },
    /// The net's global route.
    Route(NetRoute),
}

/// The RC network of one net.
#[derive(Clone, Debug, PartialEq)]
pub struct NetParasitics {
    /// What the wires follow.
    pub(crate) shape: WireShape,
    resistors: Vec<Resistor>,
    /// The wire capacitance at each node.
    wire_cap: IdVec<RcNode, Farads>,
    pin_node: Vec<(PinId, RcNode)>,
}

impl NetParasitics {
    /// Estimates the parasitics of `net` from where its pins are, or `None`
    /// for a net with fewer than two pins.
    #[must_use]
    pub fn estimate(netlist: &Netlist<'_>, net: NetId, rc: &WireRc) -> Option<Self> {
        let db = netlist.db;
        let mut pins: Vec<(PinId, Point<Dbu>)> = db.nets[net]
            .pins
            .iter()
            .map(|&p| (p, netlist.pin_location(p)))
            .collect();
        if pins.len() < 2 {
            return None;
        }
        pins.sort_by_key(|&(_, at)| at);
        let driver = pins
            .iter()
            .position(|&(p, _)| netlist.is_driver(p))
            .unwrap_or(0);
        let points: Vec<Point<Dbu>> = pins.iter().map(|&(_, at)| at).collect();
        let tree = steiner::prim_dijkstra(&points, driver, steiner::DEFAULT_ALPHA);
        if !steiner::is_valid(&tree) {
            log::debug!(
                "Steiner tree of {} crosses itself; OpenROAD would fall back to FLUTE here",
                db.nets[net].name
            );
        }
        let terminal_pins: Vec<Vec<PinId>> = pins.iter().map(|&(p, _)| vec![p]).collect();

        let mut resistors = Vec::new();
        let mut wire_cap: IdVec<RcNode, Farads> = IdVec::filled(Farads::ZERO, tree.nodes.len());
        for (index, parent_index) in tree.branches() {
            let (node, parent) = (RcNode::new(index), RcNode::new(parent_index));
            let length_dbu = tree.nodes[index].manhattan(tree.nodes[parent_index]);
            if length_dbu == Dbu(0) {
                resistors.push(Resistor {
                    a: node,
                    b: parent,
                    resistance: JOIN_RESISTANCE,
                });
                continue;
            }
            let length = Meters::from_dbu(length_dbu, db.dbu());
            let cap = length * rc.capacitance;
            wire_cap[node] += cap / 2.0;
            wire_cap[parent] += cap / 2.0;
            resistors.push(Resistor {
                a: node,
                b: parent,
                resistance: length * rc.resistance,
            });
        }

        let mut pin_node = Vec::with_capacity(pins.len());
        for (terminal, &(pin, _)) in pins.iter().enumerate() {
            let pin_layer = db.pin_reference_layer(pin).unwrap_or(rc.layer);
            let cuts = pin_layer.index().abs_diff(rc.layer.index());
            let pin_rc_node = wire_cap.push(Farads::ZERO);
            let mut from = pin_rc_node;
            for cut in 0..cuts.max(1) {
                let to = if cut + 1 == cuts.max(1) {
                    RcNode::new(terminal)
                } else {
                    wire_cap.push(Farads::ZERO)
                };
                resistors.push(Resistor {
                    a: from,
                    b: to,
                    resistance: JOIN_RESISTANCE,
                });
                from = to;
            }
            pin_node.push((pin, pin_rc_node));
        }
        Some(Self {
            shape: WireShape::Steiner {
                tree,
                terminal_pins,
            },
            resistors,
            wire_cap,
            pin_node,
        })
    }

    /// A network from its resistors, each node's wire capacitance and the
    /// node of each pin.
    pub(crate) const fn from_network(
        shape: WireShape,
        resistors: Vec<Resistor>,
        wire_cap: IdVec<RcNode, Farads>,
        pin_node: Vec<(PinId, RcNode)>,
    ) -> Self {
        Self {
            shape,
            resistors,
            wire_cap,
            pin_node,
        }
    }

    /// The resistors.
    #[must_use]
    pub fn resistors(&self) -> &[Resistor] {
        &self.resistors
    }

    /// The wire capacitance at each node.
    #[must_use]
    pub const fn node_caps(&self) -> &IdVec<RcNode, Farads> {
        &self.wire_cap
    }

    /// The node of each pin.
    #[must_use]
    pub fn pin_nodes(&self) -> &[(PinId, RcNode)] {
        &self.pin_node
    }

    /// The pin at each node, where a node is a pin's.
    fn pins_by_node(&self) -> IdVec<RcNode, Option<PinId>> {
        let mut by_node = IdVec::filled(None, self.wire_cap.len());
        for &(pin, node) in &self.pin_node {
            by_node[node] = Some(pin);
        }
        by_node
    }

    /// The resistors at each node, with the node at the other end.
    fn adjacency(&self) -> IdVec<RcNode, Vec<(RcNode, Ohms)>> {
        let mut adjacent = IdVec::filled(Vec::new(), self.wire_cap.len());
        for r in &self.resistors {
            adjacent[r.a].push((r.b, r.resistance));
            adjacent[r.b].push((r.a, r.resistance));
        }
        adjacent
    }

    /// Reduces the network seen from `driver` for `edge`, with each load
    /// pin's capacitance from `pin_cap`, as `OpenSTA`'s `reduceToPiElmore`.
    #[must_use]
    pub(crate) fn pi_elmore(
        &self,
        netlist: &Netlist<'_>,
        driver: PinId,
        edge: Edge,
        pin_cap: &dyn Fn(PinId, Edge) -> Farads,
    ) -> Option<PiElmore> {
        let &(_, root) = self.pin_node.iter().find(|&&(p, _)| p == driver)?;
        let load_pins: IdVec<RcNode, Option<PinId>> = self
            .pins_by_node()
            .into_iter()
            .map(|pin| pin.filter(|&p| p != driver && netlist.is_load(p)))
            .collect();
        let load_caps: IdVec<RcNode, f64> = load_pins
            .iter()
            .map(|pin| pin.map_or(0.0, |p| pin_cap(p, edge).get()))
            .collect();
        let mut reduction = Reduction {
            adjacent: self.adjacency(),
            wire_cap: &self.wire_cap,
            load_caps: &load_caps,
            downstream: IdVec::filled(0.0, self.wire_cap.len()),
            visited: IdVec::filled(false, self.wire_cap.len()),
        };
        let (y1, y2, y3) = reduction.moments(root, None);
        let (c2, rpi, c1) = if y2 == 0.0 && y3 == 0.0 {
            (0.0, 0.0, y1)
        } else {
            (y1 - y2 * y2 / y3, -y3 * y3 / (y2 * y2 * y2), y2 * y2 / y3)
        };
        let mut elmore = Vec::new();
        reduction.visited.fill(false);
        reduction.elmore(root, None, 0.0, &load_pins, &mut elmore);
        let wire_cap: f64 = self.wire_cap.iter().map(|c| c.get()).sum();
        let pin_cap_total: f64 = load_caps.iter().sum();
        Some(PiElmore {
            c2: Farads::new(c2),
            rpi: Ohms::new(rpi),
            c1: Farads::new(c1),
            elmore: elmore
                .into_iter()
                .map(|(pin, delay)| (pin, Seconds::new(delay)))
                .collect(),
            wire_cap: Farads::new(wire_cap),
            pin_cap: Farads::new(pin_cap_total),
        })
    }
}

/// A network's reduction seen from its driver: the resistors at each node,
/// the wire and load capacitance at each, and each node's downstream
/// capacitance once the moments are found.
struct Reduction<'n> {
    adjacent: IdVec<RcNode, Vec<(RcNode, Ohms)>>,
    wire_cap: &'n IdVec<RcNode, Farads>,
    load_caps: &'n IdVec<RcNode, f64>,
    downstream: IdVec<RcNode, f64>,
    visited: IdVec<RcNode, bool>,
}

impl Reduction<'_> {
    /// The first three admittance moments below `node`, reached from
    /// `from`, as `OpenSTA`'s `reducePiDfs`, recording each node's
    /// downstream capacitance.
    // The sums keep OpenSTA's grouping so they can be checked against it.
    #[allow(clippy::suboptimal_flops)]
    fn moments(&mut self, node: RcNode, from: Option<RcNode>) -> (f64, f64, f64) {
        self.visited[node] = true;
        let mut y1 = self.wire_cap[node].get() + self.load_caps[node];
        let mut y2 = 0.0;
        let mut y3 = 0.0;
        let mut cap = y1;
        for k in 0..self.adjacent[node].len() {
            let (next, r) = self.adjacent[node][k];
            let r = r.get();
            if Some(next) == from || self.visited[next] {
                continue;
            }
            let (d1, d2, d3) = self.moments(next, Some(node));
            y1 += d1;
            y2 += d2 - r * d1 * d1;
            y3 += d3 - 2.0 * r * d1 * d2 + r * r * d1 * d1 * d1;
            cap += self.downstream[next];
        }
        self.downstream[node] = cap;
        (y1, y2, y3)
    }

    /// Records the Elmore delay from the driver of each load pin below
    /// `node`, reached from `from` after `delay`, as `OpenSTA`'s
    /// `reduceElmoreDfs`.
    fn elmore(
        &mut self,
        node: RcNode,
        from: Option<RcNode>,
        delay: f64,
        load_pins: &IdVec<RcNode, Option<PinId>>,
        out: &mut Vec<(PinId, f64)>,
    ) {
        self.visited[node] = true;
        if from.is_some()
            && let Some(pin) = load_pins[node]
        {
            out.push((pin, delay));
        }
        for k in 0..self.adjacent[node].len() {
            let (next, r) = self.adjacent[node][k];
            if Some(next) == from || self.visited[next] {
                continue;
            }
            let next_delay = r.get().mul_add(self.downstream[next], delay);
            self.elmore(next, Some(node), next_delay, load_pins, out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::library::Link;
    use crate::testing::{
        DefSections, cells, clock_cells_source, def_text, design, metal2_wires, pin,
    };
    use pare_common::db::design::NetlistDB;

    /// A flop driving two buffers, one level with it 20 microns away and
    /// one 100 microns above that, and a buffer whose output drives nothing.
    fn fanout() -> NetlistDB {
        design(&def_text(
            "fanout",
            &DefSections {
                components: "- f1 DFF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - near BUF_X1 + PLACED ( 42000 2800 ) N ;\n\
                             - far BUF_X1 + PLACED ( 42000 202800 ) N ;\n",
                nets: "- q ( f1 Q ) ( near A ) ( far A ) ;\n- lone ( near Z ) ;\n",
                inputs: &[],
                outputs: &[],
            },
        ))
    }

    /// The half perimeter of the box around `pins`' locations.
    fn half_perimeter(netlist: &Netlist<'_>, pins: &[PinId]) -> Meters {
        let points: Vec<Point<Dbu>> = pins.iter().map(|&p| netlist.pin_location(p)).collect();
        let span = |axis: fn(&Point<Dbu>) -> Dbu| {
            let values = points.iter().map(axis);
            values.clone().max().unwrap() - values.min().unwrap()
        };
        Meters::from_dbu(span(|p| p.x) + span(|p| p.y), netlist.db.dbu())
    }

    fn relative_gap(a: f64, b: f64) -> f64 {
        (a - b).abs() / b.abs()
    }

    #[test]
    fn a_net_of_one_pin_has_no_parasitics() {
        let db = fanout();
        let lib = cells(&clock_cells_source());
        let link = Link::new(&db, &lib, &[]).unwrap();
        let netlist = Netlist::new(&db, &link);

        let estimate =
            NetParasitics::estimate(&netlist, db.net_id("lone").unwrap(), &metal2_wires(&db));

        assert_eq!(estimate, None);
    }

    #[test]
    fn three_pins_get_the_wire_of_their_steiner_tree() {
        let db = fanout();
        let lib = cells(&clock_cells_source());
        let link = Link::new(&db, &lib, &[]).unwrap();
        let netlist = Netlist::new(&db, &link);
        let rc = metal2_wires(&db);
        let pins = [pin(&db, "f1/Q"), pin(&db, "near/A"), pin(&db, "far/A")];

        let estimate = NetParasitics::estimate(&netlist, db.net_id("q").unwrap(), &rc).unwrap();

        let length = half_perimeter(&netlist, &pins);
        let wire_cap: f64 = estimate.node_caps().iter().map(|c| c.get()).sum();
        let wire_res: f64 = estimate
            .resistors()
            .iter()
            .filter(|r| r.resistance != JOIN_RESISTANCE)
            .map(|r| r.resistance.get())
            .sum();
        assert!(relative_gap(wire_cap, (length * rc.capacitance).get()) < 1e-9);
        assert!(relative_gap(wire_res, (length * rc.resistance).get()) < 1e-9);
        assert_eq!(estimate.pin_nodes().len(), pins.len());
    }

    #[test]
    fn the_farther_load_sees_the_longer_elmore_delay() {
        let db = fanout();
        let lib = cells(&clock_cells_source());
        let link = Link::new(&db, &lib, &[]).unwrap();
        let netlist = Netlist::new(&db, &link);
        let estimate =
            NetParasitics::estimate(&netlist, db.net_id("q").unwrap(), &metal2_wires(&db)).unwrap();

        let pi = estimate
            .pi_elmore(&netlist, pin(&db, "f1/Q"), Edge::Rise, &|_, _| {
                Farads::new(1.0e-15)
            })
            .unwrap();

        let near = pi.elmore_to(pin(&db, "near/A"));
        let far = pi.elmore_to(pin(&db, "far/A"));
        assert!(Seconds::ZERO < near && near < far, "{near:?} {far:?}");
    }
}
