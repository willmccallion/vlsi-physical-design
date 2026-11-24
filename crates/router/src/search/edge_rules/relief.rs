//! Where a net's own block pins relieve its wires of the fixed metal they
//! come near, as `TritonRoute` clears the fixed shape cost around a block
//! pin while the pin's net is routed (`initMazeCost_terms` with
//! `resetFixedShape`).

use crate::access::{AccessPoint, Node, PinKind};
use crate::tech::RouteTech;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{LayerId, NetId, PinId};
use pare_common::db::library::{Direction, LayerRef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// Which ends of a wire or jog a net's own block pins relieve.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Relief {
    /// Neither end: all the fixed metal the edge meets counts.
    None,
    /// Both ends: no fixed metal counts.
    Whole,
    /// One end: only the fixed metal the other end's wire comes near
    /// counts, as `TritonRoute` charges the fixed shapes of the node an edge
    /// enters.
    Partly {
        /// The end still charged.
        charged: Node,
    },
}

/// The surroundings of one block pin shape on a routable layer, inside
/// which a node's fixed metal stops counting for the pin's net.
#[derive(Clone, Copy, Debug)]
pub(super) struct Band {
    layer: LayerId,
    rect: Rect<Dbu>,
    /// The spacing the net's wire needs from the shape: a node is inside
    /// when its wire comes closer than this.
    reach: Dbu,
    /// Half the net's wire width on the layer.
    half_width: Dbu,
    /// Whether wires running along the x axis are relieved: the pin has an
    /// access point leaving it that way.
    horizontal: bool,
    /// Whether wires running along the y axis are relieved.
    vertical: bool,
}

impl Band {
    /// Whether a wire through the node at `p` comes nearer the shape than
    /// its reach, measured from the corners of the wire's square around the
    /// node as `modMinSpacingCostPlanarHelper` measures.
    fn holds(&self, p: Point<Dbu>) -> bool {
        let h = self.half_width;
        let corners = [
            Point::new(p.x - h, p.y - h),
            Point::new(p.x - h, p.y + h),
            Point::new(p.x + h, p.y - h),
            Point::new(p.x + h, p.y + h),
        ];
        let reach = i128::from(self.reach.get());
        corners
            .iter()
            .any(|&c| distance_squared(c, &self.rect) < reach * reach)
    }

    /// Whether it relieves wires running horizontally or else vertically.
    const fn relieves(&self, horizontal: bool) -> bool {
        if horizontal {
            self.horizontal
        } else {
            self.vertical
        }
    }
}

/// The squared distance from `p` to `rect`, zero inside it.
fn distance_squared(p: Point<Dbu>, rect: &Rect<Dbu>) -> i128 {
    let gap = |lo: Dbu, hi: Dbu, at: Dbu| i128::from((lo - at).max(at - hi).max(Dbu::ZERO).get());
    let dx = gap(rect.min.x, rect.max.x, p.x);
    let dy = gap(rect.min.y, rect.max.y, p.y);
    dx * dx + dy * dy
}

/// The bands around the shapes of `net`'s pins on hard blocks, each
/// relieving the orientations the pin's access points `candidates` leave
/// it by.
// NOTE: initMazeCost_terms takes the orientations from whichever access
// point of the pin it reads last; PARE takes those of any of them.
pub(super) fn bands_of(
    db: &NetlistDB,
    tech: &RouteTech,
    net: NetId,
    candidates: &[Vec<AccessPoint>],
) -> Vec<Band> {
    let rule = tech.net_rule(net);
    let mut bands = Vec::new();
    for (&pin, points) in db.nets[net].pins.iter().zip(candidates) {
        if PinKind::of(db, pin) != PinKind::Block {
            continue;
        }
        let (horizontal, vertical) = orientations(tech, points);
        for (layer, rect) in routable_shapes(db, tech, pin) {
            let width = tech.wire_width(rule, layer);
            let lt = tech.layer(layer);
            let narrow = rect.width().min(rect.height());
            let long = rect.width().max(rect.height());
            let reach = lt
                .spacing_table
                .as_ref()
                .map_or(lt.spacing, |t| {
                    t.spacing(narrow, width, long).max(lt.spacing)
                })
                .max(tech.wire_spacing(rule, layer));
            bands.push(Band {
                layer,
                rect,
                reach,
                half_width: width / 2,
                horizontal,
                vertical,
            });
        }
    }
    bands
}

/// Whether some access point in `points` leaves its pin horizontally, and
/// whether one leaves it vertically.
fn orientations(tech: &RouteTech, points: &[AccessPoint]) -> (bool, bool) {
    points.iter().fold((false, false), |(h, v), a| {
        let (along, across) = (a.ways.has_along(), a.ways.has_across());
        match tech.layer(a.node.layer).direction {
            Direction::Horizontal => (h || along, v || across),
            Direction::Vertical => (h || across, v || along),
        }
    })
}

/// The shapes of `pin` on routable layers.
fn routable_shapes(db: &NetlistDB, tech: &RouteTech, pin: PinId) -> Vec<(LayerId, Rect<Dbu>)> {
    db.pin_shapes(pin)
        .into_iter()
        .filter_map(|s| match s.layer {
            LayerRef::Routing(l) if tech.routable(l) => Some((l, s.rect)),
            _ => None,
        })
        .collect()
}

/// Which ends of a wire or jog from `a` to `b` some band of `bands`
/// relieves; a horizontal edge only by bands relieving horizontal wires.
pub(super) fn relief_of(
    tech: &RouteTech,
    bands: &[Band],
    (a, b): (Node, Node),
    horizontal: bool,
) -> Relief {
    let held = |n: Node| {
        let p = tech.layer(n.layer).point(n.track, n.cross);
        bands
            .iter()
            .any(|band| band.layer == n.layer && band.relieves(horizontal) && band.holds(p))
    };
    match (held(a), held(b)) {
        (true, true) => Relief::Whole,
        (true, false) => Relief::Partly { charged: b },
        (false, true) => Relief::Partly { charged: a },
        (false, false) => Relief::None,
    }
}
