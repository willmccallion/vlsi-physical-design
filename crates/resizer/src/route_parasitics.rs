//! Wire parasitics from global routes, as `OpenROAD`'s `estimate_parasitics
//! -global_routing` makes them.
//!
//! Each segment of a net's route becomes a resistor between its ends with
//! half its capacitance at each. Each pin is joined to the gcell center its
//! route reaches it in by a wire on the layer above its own, through the via
//! between, or on its own layer where the route does not climb from it.
//! Values pass through single precision where `OpenROAD` holds them so.

// The formulas keep OpenROAD's grouping so they can be checked against it.
#![allow(clippy::suboptimal_flops)]

use crate::parasitics::{NetParasitics, RcNode, Resistor, WireShape};
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::{NetRoute, RoutePoint};
use pare_common::db::indices::{CutLayerId, IdVec, LayerId, NetId};
use pare_common::db::library::Library;
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::units::{
    FARADS_PER_PICOFARAD, Farads, FaradsPerMeter, MICRONS_PER_METER, Meters, Ohms, OhmsPerMeter,
};
use rustc_hash::FxHashMap;

/// The resistance and capacitance of each layer that global route
/// parasitics use.
#[derive(Clone, Debug, PartialEq)]
pub struct RouteRc {
    /// Per routing layer, `set_layer_rc`'s resistance and capacitance per
    /// length, zero where not given.
    layer: IdVec<LayerId, (OhmsPerMeter, FaradsPerMeter)>,
    /// Per routing layer, the same from the LEF, as `OpenROAD` falls back.
    lef_layer: IdVec<LayerId, (OhmsPerMeter, FaradsPerMeter)>,
    /// Per routing layer, its default wire width.
    width: IdVec<LayerId, Dbu>,
    /// Per cut layer, `set_layer_rc -via`'s resistance, zero where not given.
    via: IdVec<CutLayerId, Ohms>,
    /// Per cut layer, the LEF's resistance per cut.
    lef_via: IdVec<CutLayerId, Ohms>,
    min_layer: LayerId,
    dbu_per_micron: DbuPerMicron,
}

impl RouteRc {
    /// The layer values of `library`, overridden per routing layer by
    /// `layers` and per cut layer by `vias`; wires below `min_layer` are
    /// left out, as the global router does not route there.
    #[must_use]
    pub fn new(
        library: &Library,
        layers: &[(LayerId, OhmsPerMeter, FaradsPerMeter)],
        vias: &[(CutLayerId, Ohms)],
        min_layer: LayerId,
    ) -> Self {
        let dbu_per_micron = library.dbu_per_micron;
        // NOTE: OpenROAD quirk kept: a zero resistance or capacitance means
        // not given and falls back to the LEF's, as EstimateParasitics stores
        // set_layer_rc's values.
        let mut layer = IdVec::filled(
            (OhmsPerMeter::ZERO, FaradsPerMeter::ZERO),
            library.routing_layers.len(),
        );
        for &(id, r, c) in layers {
            layer[id] = (r, c);
        }
        let mut via = IdVec::filled(Ohms::ZERO, library.cut_layers.len());
        for &(cut, r) in vias {
            via[cut] = r;
        }
        let lef_layer = library
            .routing_layers
            .iter()
            .map(|l| lef_layer_rc(l.width, &l.electrical, dbu_per_micron))
            .collect();
        Self {
            layer,
            lef_layer,
            width: library.routing_layers.iter().map(|l| l.width).collect(),
            via,
            lef_via: library
                .cut_layers
                .iter()
                .map(|c| Ohms::new(c.resistance.unwrap_or(0.0)))
                .collect(),
            min_layer,
            dbu_per_micron,
        }
    }

    /// `set_layer_rc`'s resistance and capacitance per length for a routing
    /// layer, zero where not given, as `EstimateParasitics::layerRC`.
    #[must_use]
    pub fn layer_rc(&self, layer: LayerId) -> (OhmsPerMeter, FaradsPerMeter) {
        self.layer[layer]
    }

    /// The resistance of one cut on the cut layer above routing layer
    /// `below`: `set_layer_rc -via`'s or else the LEF's, as
    /// `BufferedNet::viaResistance` adds them up.
    #[must_use]
    pub(crate) fn via_resistance(&self, below: LayerId) -> Ohms {
        let cut = below.cut_above();
        match self.via.get(cut) {
            Some(&r) if r != Ohms::ZERO => r,
            _ => self.lef_via.get(cut).copied().unwrap_or(Ohms::ZERO),
        }
    }

    /// The resistance and capacitance of `length` of wire on `layer`, its
    /// resistance divided by how much wider `ndr_width` is than the layer's,
    /// as `MakeWireParasitics::layerRC`.
    // OpenROAD returns them in single precision.
    fn wire(&self, length: Dbu, layer: LayerId, ndr_width: Option<Dbu>) -> (f32, f32) {
        let (r, c) = self.layer[layer];
        let (mut r_per_meter, mut cap_per_meter) = (r.get(), c.get());
        let (lef_r, lef_c) = self.lef_layer[layer];
        let (lef_r, lef_c) = (lef_r.get(), lef_c.get());
        if r_per_meter == 0.0 {
            r_per_meter = lef_r;
        }
        if cap_per_meter == 0.0 {
            cap_per_meter = lef_c;
        }
        if let Some(width) = ndr_width {
            let ratio = width.get() as f32 / self.width[layer].get() as f32;
            r_per_meter /= f64::from(ratio);
        }
        let meters = Meters::from_dbu(length, self.dbu_per_micron).get() as f32;
        (
            (r_per_meter * f64::from(meters)) as f32,
            (cap_per_meter * f64::from(meters)) as f32,
        )
    }

    /// The resistance of the via from routing layer `below` to the one
    /// above, as `MakeWireParasitics::getCutLayerRes` for one cut.
    // OpenROAD returns it in single precision.
    fn cut(&self, below: LayerId) -> f32 {
        self.via_resistance(below).get() as f32
    }
}

/// A routing layer's resistance and capacitance per length from its LEF
/// values, as `MakeWireParasitics::layerRC` falls back to them.
// OpenROAD holds them in single precision.
fn lef_layer_rc(
    width: Dbu,
    electrical: &pare_common::db::library::LayerElectrical,
    dbu_per_micron: DbuPerMicron,
) -> (OhmsPerMeter, FaradsPerMeter) {
    let width_microns = dbu_per_micron.to_microns(width) as f32;
    let res_ohm_per_micron =
        (electrical.resistance_per_square.unwrap_or(0.0) / f64::from(width_microns)) as f32;
    let cap_pf_per_micron = (f64::from(width_microns)
        * electrical.capacitance_per_area.unwrap_or(0.0)
        + 2.0 * electrical.edge_capacitance.unwrap_or(0.0)) as f32;
    (
        OhmsPerMeter::new(MICRONS_PER_METER * f64::from(res_ohm_per_micron)),
        FaradsPerMeter::new(
            MICRONS_PER_METER * FARADS_PER_PICOFARAD * f64::from(cap_pf_per_micron),
        ),
    )
}

impl NetParasitics {
    /// The parasitics of `net` along its global route, as
    /// `MakeWireParasitics::estimateParasitics`.
    #[must_use]
    pub fn from_route(db: &NetlistDB, net: NetId, route: NetRoute, rc: &RouteRc) -> Self {
        let ndr_width = |layer: LayerId| {
            let rule = db.nets[net].non_default_rule?;
            db.non_default_rules[rule]
                .layers
                .iter()
                .find(|l| l.layer == layer)
                .map(|l| l.width)
        };
        let mut network = Network::default();
        for segment in &route.segments {
            let valid = |layer: LayerId| layer >= rc.min_layer || segment.is_via();
            let from = valid(segment.from.layer).then(|| network.node(segment.from));
            let to = valid(segment.to.layer).then(|| network.node(segment.to));
            let (Some(from), Some(to)) = (from, to) else {
                continue;
            };
            let (res, cap) = if segment.length() == Dbu(0) {
                (rc.cut(segment.from.layer.min(segment.to.layer)), 0.0)
            } else if segment.from.layer == segment.to.layer {
                rc.wire(
                    segment.length(),
                    segment.from.layer,
                    ndr_width(segment.from.layer),
                )
            } else {
                log::warn!("Non wire or via route found on net {}.", db.nets[net].name);
                (0.0, 0.0)
            };
            network.wire(from, to, res, cap);
        }
        let mut pin_node = Vec::with_capacity(route.pins.len());
        for location in &route.pins {
            let pin = network.add_node();
            pin_node.push((location.pin, pin));
            let above = location.layer.above();
            let (grid, layer, via_res) = if let Some(&node) = network.nodes.get(&RoutePoint {
                at: location.grid,
                layer: above,
            }) {
                (Some(node), above, rc.cut(location.layer))
            } else {
                let at = RoutePoint {
                    at: location.grid,
                    layer: location.layer,
                };
                (network.nodes.get(&at).copied(), location.layer, 0.0)
            };
            let Some(grid) = grid else {
                log::warn!(
                    "Missing route to pin {} in net {}.",
                    db.pin_name(location.pin),
                    db.nets[net].name
                );
                continue;
            };
            let length = Dbu((location.at.x.get() - location.grid.x.get()).abs()
                + (location.at.y.get() - location.grid.y.get()).abs());
            let (res, cap) = rc.wire(length, layer, ndr_width(layer));
            network.wire(pin, grid, res + via_res, cap);
        }
        Self::from_network(
            WireShape::Route(route),
            network.resistors,
            network
                .cap
                .iter()
                .map(|&c| Farads::new(f64::from(c)))
                .collect(),
            pin_node,
        )
    }
}

/// A network under construction: a node per route point, the resistors
/// between them and each node's capacitance.
#[derive(Default)]
struct Network {
    nodes: FxHashMap<RoutePoint, RcNode>,
    resistors: Vec<Resistor>,
    /// Each node's capacitance in farads, in single precision as `OpenROAD`
    /// adds it up.
    cap: IdVec<RcNode, f32>,
}

impl Network {
    fn add_node(&mut self) -> RcNode {
        self.cap.push(0.0)
    }

    fn node(&mut self, at: RoutePoint) -> RcNode {
        if let Some(&node) = self.nodes.get(&at) {
            return node;
        }
        let node = self.add_node();
        let _ = self.nodes.insert(at, node);
        node
    }

    /// A resistor from `a` to `b` with half of `cap` at each end.
    fn wire(&mut self, a: RcNode, b: RcNode, res: f32, cap: f32) {
        self.cap[a] += cap / 2.0;
        self.resistors.push(Resistor {
            a,
            b,
            resistance: Ohms::new(f64::from(res)),
        });
        self.cap[b] += cap / 2.0;
    }
}
