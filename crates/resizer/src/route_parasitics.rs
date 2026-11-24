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
use pare_common::convert::NearestFrom;
use pare_common::db::design::NetlistDB;
use pare_common::db::global_route::{NetRoute, PinGridLocation, RoutePoint};
use pare_common::db::indices::{CutLayerId, IdVec, LayerId, NetId};
use pare_common::db::library::Library;
use pare_common::geom::dbu::{Dbu, DbuPerMicron};
use pare_common::units::{Farads, FaradsPerMeter, Meters, Ohms, OhmsPerMeter};
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
            .map(|l| l.electrical.wire_rc(l.width, dbu_per_micron))
            .collect();
        Self {
            layer,
            lef_layer,
            width: library.routing_layers.iter().map(|l| l.width).collect(),
            via,
            lef_via: library
                .cut_layers
                .iter()
                .map(|c| c.resistance.unwrap_or(Ohms::ZERO))
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
        let meters = f32::nearest_from(Meters::from_dbu(length, self.dbu_per_micron).get());
        (
            f32::nearest_from(r_per_meter * f64::from(meters)),
            f32::nearest_from(cap_per_meter * f64::from(meters)),
        )
    }

    /// The resistance of the via from routing layer `below` to the one
    /// above, as `MakeWireParasitics::getCutLayerRes` for one cut.
    // OpenROAD returns it in single precision.
    fn cut(&self, below: LayerId) -> f32 {
        f32::nearest_from(self.via_resistance(below).get())
    }
}

impl NetParasitics {
    /// The parasitics of `net` along its global route, as
    /// `MakeWireParasitics::estimateParasitics`.
    #[must_use]
    pub fn from_route(db: &NetlistDB, net: NetId, route: NetRoute, rc: &RouteRc) -> Self {
        let routed = RoutedNet { db, net, rc };
        let mut network = Network::default();
        routed.add_segments(&mut network, &route);
        let mut pin_node = Vec::with_capacity(route.pins.len());
        for location in &route.pins {
            let pin = network.add_node();
            pin_node.push((location.pin, pin));
            routed.join_pin(&mut network, pin, location);
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

/// A net whose global route is being turned into parasitics.
struct RoutedNet<'a> {
    db: &'a NetlistDB,
    net: NetId,
    rc: &'a RouteRc,
}

impl RoutedNet<'_> {
    /// The width the net's non-default rule gives wires on `layer`.
    fn ndr_width(&self, layer: LayerId) -> Option<Dbu> {
        // NOTE: MakeWireParasitics::layerRC reads the rule for the layer
        // unchecked and crashes where the NDR has none; PARE leaves the
        // resistance unscaled there.
        let rule = self.db.nets[self.net].non_default_rule?;
        self.db.non_default_rules[rule]
            .layers
            .iter()
            .find(|l| l.layer == layer)
            .map(|l| l.width)
    }

    /// Adds a wire or cut for each segment of `route` with both ends on
    /// layers the parasitics count.
    fn add_segments(&self, network: &mut Network, route: &NetRoute) {
        let (db, rc) = (self.db, self.rc);
        for segment in &route.segments {
            let valid = |layer: LayerId| layer >= rc.min_layer || segment.is_via();
            let from = valid(segment.from.layer).then(|| network.node(segment.from));
            let to = valid(segment.to.layer).then(|| network.node(segment.to));
            let (Some(from), Some(to)) = (from, to) else {
                continue;
            };
            let (res, cap) = if segment.length() == Dbu::ZERO {
                (rc.cut(segment.from.layer.min(segment.to.layer)), 0.0)
            } else if segment.from.layer == segment.to.layer {
                rc.wire(
                    segment.length(),
                    segment.from.layer,
                    self.ndr_width(segment.from.layer),
                )
            } else {
                log::warn!(
                    "Non wire or via route found on net {}.",
                    db.nets[self.net].name
                );
                (0.0, 0.0)
            };
            network.wire(from, to, res, cap);
        }
    }

    /// Wires node `pin` to the route at its grid point, through the via up
    /// where the route arrives on the layer above.
    fn join_pin(&self, network: &mut Network, pin: RcNode, location: &PinGridLocation) {
        let node_above = location.layer.above().and_then(|above| {
            let at = RoutePoint {
                at: location.grid,
                layer: above,
            };
            network.nodes.get(&at).map(|&node| (node, above))
        });
        let (grid, layer, via_res) = if let Some((node, above)) = node_above {
            (Some(node), above, self.rc.cut(location.layer))
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
                self.db.pin_name(location.pin),
                self.db.nets[self.net].name
            );
            return;
        };
        let length = location.at.manhattan(location.grid);
        let (res, cap) = self.rc.wire(length, layer, self.ndr_width(layer));
        network.wire(pin, grid, res + via_res, cap);
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
