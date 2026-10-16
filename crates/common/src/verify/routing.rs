//! Connectivity of routed nets and wiring totals. Design rules are the
//! business of [`super::drc`].

use super::shapes::{is_signal_net, route_elem_shapes};
use crate::db::design::{NetlistDB, RouteElem, TrackAxis};
use crate::db::indices::NetId;
use crate::db::library::{Direction, LayerRef, Shape};
use crate::geom::dbu::Dbu;
use rayon::prelude::*;
use std::collections::HashSet;

/// A signal net whose pins are not all connected.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Open {
    /// The net.
    pub net: NetId,
    /// How many separate groups its pins fall into.
    pub components: usize,
}

/// Wiring totals.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct WiringStats {
    /// Centerline wirelength per routing layer.
    pub wirelength: Vec<Dbu>,
    /// Number of via instances.
    pub vias: usize,
    /// Wires that are neither horizontal nor vertical.
    pub non_manhattan: usize,
    /// Wires running against their layer's preferred direction.
    pub wrong_way: usize,
    /// Preferred-direction wires whose centerline is not on a track.
    pub off_track: usize,
}

/// What the connectivity check found.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RoutingReport {
    /// Nets with disconnected pins.
    pub opens: Vec<Open>,
    /// Wiring totals.
    pub stats: WiringStats,
}

impl RoutingReport {
    /// Whether every signal net is connected.
    pub const fn is_connected(&self) -> bool {
        self.opens.is_empty()
    }
}

/// Checks that every signal net of `db` is connected and totals its wiring.
pub fn check_routing(db: &NetlistDB) -> RoutingReport {
    RoutingReport {
        opens: opens(db),
        stats: wiring_stats(db),
    }
}

/// Whether metal or cut shapes on these layers can conduct into each other.
fn layers_connect(a: LayerRef, b: LayerRef) -> bool {
    match (a, b) {
        (LayerRef::Routing(x), LayerRef::Routing(y)) => x == y,
        (LayerRef::Cut(x), LayerRef::Cut(y)) => x == y,
        (LayerRef::Cut(c), LayerRef::Routing(m)) | (LayerRef::Routing(m), LayerRef::Cut(c)) => {
            m == c.below() || m == c.above()
        }
    }
}

const fn find(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

const fn union(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    if ra != rb {
        parent[ra] = rb;
    }
}

/// How many separately connected groups the pins of `net` form, counting
/// its wiring and the shapes of its pins.
pub fn net_components(db: &NetlistDB, net: NetId) -> usize {
    let n = &db.nets[net.index()];
    let mut shapes: Vec<(Shape, Option<usize>)> = n
        .routing
        .iter()
        .flat_map(|e| route_elem_shapes(db, e))
        .map(|s| (s, None))
        .collect();
    let mut shapeless_pins = 0;
    for (k, &pin) in n.pins.iter().enumerate() {
        let pin_shapes = db.pin_shapes(pin);
        if pin_shapes.is_empty() {
            shapeless_pins += 1;
        }
        shapes.extend(pin_shapes.into_iter().map(|s| (s, Some(k))));
    }

    let mut parent: Vec<usize> = (0..shapes.len()).collect();
    let mut first_shape_of_pin: Vec<Option<usize>> = vec![None; n.pins.len()];
    for (i, (_, pin)) in shapes.iter().enumerate() {
        if let Some(k) = *pin {
            match first_shape_of_pin[k] {
                Some(first) => union(&mut parent, first, i),
                None => first_shape_of_pin[k] = Some(i),
            }
        }
    }

    let mut order: Vec<usize> = (0..shapes.len()).collect();
    order.sort_by_key(|&i| shapes[i].0.rect.min.x);
    for (pos, &a) in order.iter().enumerate() {
        let ra = shapes[a].0;
        for &b in &order[pos + 1..] {
            let rb = shapes[b].0;
            if rb.rect.min.x > ra.rect.max.x {
                break;
            }
            if layers_connect(ra.layer, rb.layer) && ra.rect.touches(&rb.rect) {
                union(&mut parent, a, b);
            }
        }
    }

    let roots: HashSet<usize> = first_shape_of_pin
        .iter()
        .flatten()
        .map(|&i| find(&mut parent, i))
        .collect();
    roots.len() + shapeless_pins
}

fn opens(db: &NetlistDB) -> Vec<Open> {
    (0..db.nets.len())
        .into_par_iter()
        .map(NetId::new)
        .filter(|&net| is_signal_net(db, net))
        .filter_map(|net| {
            let components = net_components(db, net);
            (components > 1).then_some(Open { net, components })
        })
        .collect()
}

fn wiring_stats(db: &NetlistDB) -> WiringStats {
    let layers = &db.library.routing_layers;
    let mut tracks: HashSet<(usize, Dbu)> = HashSet::new();
    let mut has_tracks = vec![false; layers.len()];
    for t in &db.tracks {
        for &l in &t.layers {
            let preferred = match layers[l.index()].direction {
                Direction::Horizontal => TrackAxis::Y,
                Direction::Vertical => TrackAxis::X,
            };
            if t.axis == preferred {
                has_tracks[l.index()] = true;
                tracks.extend(t.coords().map(|c| (l.index(), c)));
            }
        }
    }

    let mut stats = WiringStats {
        wirelength: vec![Dbu::ZERO; layers.len()],
        ..WiringStats::default()
    };
    for net in &db.nets {
        for elem in &net.routing {
            match *elem {
                RouteElem::Via { .. } => stats.vias += 1,
                RouteElem::Patch { .. } => {}
                RouteElem::Wire { layer, from, to } => {
                    let l = layer.index();
                    let (dx, dy) = ((to.x - from.x).abs(), (to.y - from.y).abs());
                    stats.wirelength[l] += dx + dy;
                    if dx != Dbu::ZERO && dy != Dbu::ZERO {
                        stats.non_manhattan += 1;
                        continue;
                    }
                    let (along, across) = match layers[l].direction {
                        Direction::Horizontal => (dx, from.y),
                        Direction::Vertical => (dy, from.x),
                    };
                    let length = dx + dy;
                    if length > Dbu::ZERO && along == Dbu::ZERO {
                        stats.wrong_way += 1;
                    } else if has_tracks[l] && !tracks.contains(&(l, across)) {
                        stats.off_track += 1;
                    }
                }
            }
        }
    }
    stats
}
