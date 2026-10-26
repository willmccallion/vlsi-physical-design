//! Wire parasitics estimated from a placement, as `OpenROAD`'s
//! `estimate_parasitics -placement`.
//!
//! Each net gets a Steiner tree over its pins with the signal layer's
//! resistance and capacitance per length along the branches, reduced for
//! each driver to a pi model and an Elmore delay to every load, as `OpenSTA`
//! reduces it.

use crate::netlist::Netlist;
use crate::steiner::{self, SteinerTree};
use pare_common::db::global_route::NetRoute;
use pare_common::db::indices::{LayerId, NetId, PinId};
use pare_common::db::liberty::Edge;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;

/// The resistance `OpenROAD` gives a zero-length connection so the network
/// stays joined, in ohms.
const JOIN_RESISTANCE: f64 = 1.0e-3;

/// Resistance and capacitance per meter of signal wire.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WireRc {
    /// Ohms per meter.
    pub resistance: f64,
    /// Farads per meter.
    pub capacitance: f64,
    /// The routing layer signal wires stand on.
    pub layer: LayerId,
}

impl WireRc {
    /// Wire RC from values per micron: ohms and femtofarads.
    #[must_use]
    pub fn per_micron(ohm_per_um: f64, ff_per_um: f64, layer: LayerId) -> Self {
        Self {
            resistance: ohm_per_um * 1e6,
            capacitance: ff_per_um * 1e-15 * 1e6,
            layer,
        }
    }
}

/// The pi model `OpenSTA` reduces a driver's network to, with the Elmore
/// delay to each load.
#[derive(Clone, Debug, PartialEq)]
pub struct PiElmore {
    /// Capacitance nearest the driver, in farads.
    pub c2: f64,
    /// Resistance between the two capacitances, in ohms.
    pub rpi: f64,
    /// Capacitance beyond it, in farads.
    pub c1: f64,
    /// Every load pin with its Elmore delay from the driver, in seconds.
    pub elmore: Vec<(PinId, f64)>,
    /// Capacitance of the wire alone, in farads.
    pub wire_cap: f64,
    /// Capacitance of the load pins alone, in farads.
    pub pin_cap: f64,
}

impl PiElmore {
    /// The load the driver sees: pins and wire together.
    #[must_use]
    pub fn load_cap(&self) -> f64 {
        self.wire_cap + self.pin_cap
    }
}

/// What a net's wires follow.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum WireShape {
    /// A Steiner tree over the pins' positions.
    Steiner {
        /// The tree.
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
    pub shape: WireShape,
    node_count: usize,
    resistors: Vec<(usize, usize, f64)>,
    wire_cap: Vec<f64>,
    pin_node: Vec<(PinId, usize)>,
}

impl NetParasitics {
    /// Estimates the parasitics of `net` from where its pins are, or `None`
    /// for a net with fewer than two pins.
    #[must_use]
    pub fn estimate(netlist: &Netlist<'_>, net: NetId, rc: &WireRc) -> Option<Self> {
        let db = netlist.db;
        let dbu = db.dbu().get() as f64;
        let mut pins: Vec<(PinId, Point<Dbu>)> = db.nets[net.index()]
            .pins
            .iter()
            .map(|&p| (p, netlist.pin_location(p)))
            .collect();
        if pins.len() < 2 {
            return None;
        }
        pins.sort_by_key(|(_, at)| (at.x.get(), at.y.get()));
        let driver = pins
            .iter()
            .position(|&(p, _)| netlist.is_driver(p))
            .unwrap_or(0);
        let points: Vec<Point<Dbu>> = pins.iter().map(|&(_, at)| at).collect();
        let tree = steiner::prim_dijkstra(&points, driver, steiner::DEFAULT_ALPHA);
        if !steiner::is_valid(&tree) {
            log::debug!(
                "Steiner tree of {} crosses itself; OpenROAD would fall back to FLUTE here",
                db.nets[net.index()].name
            );
        }
        let terminal_pins: Vec<Vec<PinId>> = pins.iter().map(|&(p, _)| vec![p]).collect();

        let mut node_count = tree.nodes.len();
        let mut resistors = Vec::new();
        let mut wire_cap = vec![0.0; node_count];
        for (node, &parent) in tree.parent.iter().enumerate() {
            if parent == node {
                continue;
            }
            let (a, b) = (tree.nodes[node], tree.nodes[parent]);
            let length_dbu = (a.x.get() - b.x.get()).abs() + (a.y.get() - b.y.get()).abs();
            if length_dbu == 0 {
                resistors.push((node, parent, JOIN_RESISTANCE));
                continue;
            }
            let length = length_dbu as f64 / dbu * 1e-6;
            let cap = length * rc.capacitance;
            let res = length * rc.resistance;
            wire_cap[node] += cap / 2.0;
            wire_cap[parent] += cap / 2.0;
            resistors.push((node, parent, res));
        }

        let mut pin_node = Vec::with_capacity(pins.len());
        for (terminal, &(pin, _)) in pins.iter().enumerate() {
            let pin_layer = db.pin_reference_layer(pin).unwrap_or(rc.layer);
            let cuts = pin_layer.index().abs_diff(rc.layer.index());
            let pin_node_index = node_count;
            node_count += 1;
            wire_cap.push(0.0);
            let mut from = pin_node_index;
            for cut in 0..cuts.max(1) {
                let to = if cut + 1 == cuts.max(1) {
                    terminal
                } else {
                    node_count += 1;
                    wire_cap.push(0.0);
                    node_count - 1
                };
                resistors.push((from, to, JOIN_RESISTANCE));
                from = to;
            }
            pin_node.push((pin, pin_node_index));
        }
        Some(Self {
            shape: WireShape::Steiner {
                tree,
                terminal_pins,
            },
            node_count,
            resistors,
            wire_cap,
            pin_node,
        })
    }

    /// A network of `node_count` nodes from its resistors, each node's
    /// wire capacitance and the node of each pin.
    pub(crate) const fn from_network(
        shape: WireShape,
        node_count: usize,
        resistors: Vec<(usize, usize, f64)>,
        wire_cap: Vec<f64>,
        pin_node: Vec<(PinId, usize)>,
    ) -> Self {
        Self {
            shape,
            node_count,
            resistors,
            wire_cap,
            pin_node,
        }
    }

    /// The resistors: the nodes at each end and ohms.
    #[must_use]
    pub fn resistors(&self) -> &[(usize, usize, f64)] {
        &self.resistors
    }

    /// The wire capacitance at each node, in farads.
    #[must_use]
    pub fn node_caps(&self) -> &[f64] {
        &self.wire_cap
    }

    /// The node of each pin.
    #[must_use]
    pub fn pin_nodes(&self) -> &[(PinId, usize)] {
        &self.pin_node
    }

    /// The pin at each node, where a node is a pin's.
    fn pins_by_node(&self) -> Vec<Option<PinId>> {
        let mut by_node = vec![None; self.node_count];
        for &(pin, node) in &self.pin_node {
            by_node[node] = Some(pin);
        }
        by_node
    }

    /// The resistors at each node.
    fn adjacency(&self) -> Vec<Vec<(usize, f64)>> {
        let mut adjacent = vec![Vec::new(); self.node_count];
        for &(a, b, r) in &self.resistors {
            adjacent[a].push((b, r));
            adjacent[b].push((a, r));
        }
        adjacent
    }

    /// Reduces the network seen from `driver` for `edge`, with each load
    /// pin's capacitance from `pin_cap`, as `OpenSTA`'s `reduceToPiElmore`.
    #[must_use]
    pub fn pi_elmore(
        &self,
        netlist: &Netlist<'_>,
        driver: PinId,
        edge: Edge,
        pin_cap: &dyn Fn(PinId, Edge) -> f64,
    ) -> Option<PiElmore> {
        let &(_, root) = self.pin_node.iter().find(|&&(p, _)| p == driver)?;
        let pins = self.pins_by_node();
        let adjacent = self.adjacency();
        let load_cap_at = |node: usize| -> f64 {
            pins[node]
                .filter(|&p| p != driver && netlist.is_load(p))
                .map_or(0.0, |p| pin_cap(p, edge))
        };
        let mut downstream = vec![0.0; self.node_count];
        let mut visited = vec![false; self.node_count];
        let (y1, y2, y3) = self.moments(
            root,
            usize::MAX,
            &adjacent,
            &load_cap_at,
            &mut downstream,
            &mut visited,
        );
        let (c2, rpi, c1) = if y2 == 0.0 && y3 == 0.0 {
            (0.0, 0.0, y1)
        } else {
            (y1 - y2 * y2 / y3, -y3 * y3 / (y2 * y2 * y2), y2 * y2 / y3)
        };
        let mut elmore = Vec::new();
        let mut visited = vec![false; self.node_count];
        elmore_dfs(
            root,
            usize::MAX,
            0.0,
            &adjacent,
            &downstream,
            &pins,
            driver,
            netlist,
            &mut visited,
            &mut elmore,
        );
        let wire_cap: f64 = self.wire_cap.iter().sum();
        let pin_cap_total: f64 = (0..self.node_count).map(load_cap_at).sum();
        Some(PiElmore {
            c2,
            rpi,
            c1,
            elmore,
            wire_cap,
            pin_cap: pin_cap_total,
        })
    }

    /// The first three admittance moments below `node`, as `OpenSTA`'s
    /// `reducePiDfs`, recording each node's downstream capacitance.
    // The sums keep OpenSTA's grouping so they can be checked against it.
    #[allow(clippy::too_many_arguments, clippy::suboptimal_flops)]
    fn moments(
        &self,
        node: usize,
        from: usize,
        adjacent: &[Vec<(usize, f64)>],
        load_cap_at: &dyn Fn(usize) -> f64,
        downstream: &mut [f64],
        visited: &mut [bool],
    ) -> (f64, f64, f64) {
        visited[node] = true;
        let mut y1 = self.wire_cap[node] + load_cap_at(node);
        let mut y2 = 0.0;
        let mut y3 = 0.0;
        let mut cap = y1;
        for &(next, r) in &adjacent[node] {
            if next == from || visited[next] {
                continue;
            }
            let (d1, d2, d3) = self.moments(next, node, adjacent, load_cap_at, downstream, visited);
            y1 += d1;
            y2 += d2 - r * d1 * d1;
            y3 += d3 - 2.0 * r * d1 * d2 + r * r * d1 * d1 * d1;
            cap += downstream[next];
        }
        downstream[node] = cap;
        (y1, y2, y3)
    }
}

/// Records each load's Elmore delay from the driver, as `OpenSTA`'s
/// `reduceElmoreDfs`.
#[allow(clippy::too_many_arguments)]
fn elmore_dfs(
    node: usize,
    from: usize,
    delay: f64,
    adjacent: &[Vec<(usize, f64)>],
    downstream: &[f64],
    pins: &[Option<PinId>],
    driver: PinId,
    netlist: &Netlist<'_>,
    visited: &mut [bool],
    out: &mut Vec<(PinId, f64)>,
) {
    visited[node] = true;
    if from != usize::MAX
        && let Some(pin) = pins[node]
        && pin != driver
        && netlist.is_load(pin)
    {
        out.push((pin, delay));
    }
    for &(next, r) in &adjacent[node] {
        if next == from || visited[next] {
            continue;
        }
        let next_delay = r.mul_add(downstream[next], delay);
        elmore_dfs(
            next, node, next_delay, adjacent, downstream, pins, driver, netlist, visited, out,
        );
    }
}
