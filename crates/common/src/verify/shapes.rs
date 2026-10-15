//! Metal and cut rectangles of a design, tagged with what they belong to.

use crate::db::design::{IoPin, NetlistDB, PinOwner, RouteElem};
use crate::db::indices::{CellId, NetId};
use crate::db::library::{LayerRef, Shape};
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;
use crate::geom::rect::Rect;

/// What a shape is electrically part of.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Owner {
    /// A signal net.
    Net(NetId),
    /// Something no signal may touch: an obstruction, a supply or unused
    /// pin, a special net, or a routing blockage.
    Obstacle,
}

/// Where a shape comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    /// A cell, pin, obstruction, special net or blockage.
    Fixed,
    /// A routed wire.
    Wire,
    /// A routed via.
    Via,
}

/// A rectangle on a layer and what it belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OwnedShape {
    /// The geometry.
    pub shape: Shape,
    /// Its net, if any.
    pub owner: Owner,
    /// Where it comes from.
    pub source: Source,
}

impl OwnedShape {
    /// Whether it is routed wiring rather than part of a cell, pin or obstacle.
    pub const fn is_wiring(&self) -> bool {
        !matches!(self.source, Source::Fixed)
    }
}

/// The rectangle a default-width wire covers, extended by half its width
/// past each end, or `None` if the wire is not horizontal or vertical.
pub fn wire_rect(from: Point<Dbu>, to: Point<Dbu>, width: Dbu) -> Option<Rect<Dbu>> {
    if from.x != to.x && from.y != to.y {
        return None;
    }
    let lo = width / 2;
    let hi = width - lo;
    let r = Rect::from_corners(from, to);
    Some(Rect::new(
        Point::new(r.min.x - lo, r.min.y - lo),
        Point::new(r.max.x + hi, r.max.y + hi),
    ))
}

/// The shapes of one routing element; empty for a diagonal wire.
pub fn route_elem_shapes(db: &NetlistDB, elem: &RouteElem) -> Vec<Shape> {
    match *elem {
        RouteElem::Wire { layer, from, to } => wire_rect(from, to, db.library.layer(layer).width)
            .map(|rect| Shape {
                layer: LayerRef::Routing(layer),
                rect,
            })
            .into_iter()
            .collect(),
        RouteElem::Via { via, at } => db
            .library
            .via(via)
            .shapes
            .iter()
            .map(|s| Shape {
                layer: s.layer,
                rect: s.rect.translate(at),
            })
            .collect(),
    }
}

/// Whether the router is responsible for connecting this net.
pub fn is_signal_net(db: &NetlistDB, net: NetId) -> bool {
    let n = &db.nets[net.index()];
    !n.signal_use.is_supply() && n.pins.len() >= 2
}

/// Routed wiring of every signal net.
pub fn wiring_shapes(db: &NetlistDB) -> Vec<OwnedShape> {
    let mut out = Vec::new();
    for (i, net) in db.nets.iter().enumerate() {
        let owner = Owner::Net(NetId::new(i));
        for elem in &net.routing {
            let source = match elem {
                RouteElem::Wire { .. } => Source::Wire,
                RouteElem::Via { .. } => Source::Via,
            };
            out.extend(
                route_elem_shapes(db, elem)
                    .into_iter()
                    .map(|shape| OwnedShape {
                        shape,
                        owner,
                        source,
                    }),
            );
        }
    }
    out
}

/// Everything the wiring must connect to or avoid: pins, obstructions,
/// special nets and routing blockages.
pub fn fixed_shapes(db: &NetlistDB) -> Vec<OwnedShape> {
    fixed_shapes_keeping_io(db, |_| true)
}

/// [`fixed_shapes`] with only the I/O pins `keep` accepts.
pub fn fixed_shapes_keeping_io(db: &NetlistDB, keep: impl Fn(&IoPin) -> bool) -> Vec<OwnedShape> {
    let mut out = Vec::new();
    let fixed = |shape: Shape, owner: Owner| OwnedShape {
        shape,
        owner,
        source: Source::Fixed,
    };

    for i in 0..db.num_cells() {
        let id = CellId::new(i);
        let c = &db.cells[i];
        let m = db.cell_macro(id);
        let place = |s: &Shape| Shape {
            layer: s.layer,
            rect: c.orient.place_rect(&s.rect, m.size, c.location),
        };
        let mut pin_net = vec![None; m.pins.len()];
        for &pin in &c.pins {
            if let PinOwner::Cell { pin: mp, .. } = db.pins[pin.index()].owner {
                let net = db.pins[pin.index()].net;
                if is_signal_net(db, net) {
                    pin_net[mp.index()] = Some(net);
                }
            }
        }
        for (k, mp) in m.pins.iter().enumerate() {
            let owner = pin_net[k].map_or(Owner::Obstacle, Owner::Net);
            out.extend(mp.shapes.iter().map(|s| fixed(place(s), owner)));
        }
        out.extend(
            m.obstructions
                .iter()
                .map(|s| fixed(place(s), Owner::Obstacle)),
        );
    }

    for p in db.io_pins.iter().filter(|p| keep(p)) {
        let owner = if is_signal_net(db, p.net) {
            Owner::Net(p.net)
        } else {
            Owner::Obstacle
        };
        out.extend(p.placed_shapes().map(|s| fixed(s, owner)));
    }

    for n in &db.special_nets {
        for w in &n.wires {
            let shape = Shape {
                layer: LayerRef::Routing(w.layer),
                rect: w.rect(),
            };
            out.push(fixed(shape, Owner::Obstacle));
        }
        out.extend(n.rects.iter().map(|&s| fixed(s, Owner::Obstacle)));
        for &(via, at) in &n.vias {
            out.extend(db.library.via(via).shapes.iter().map(|s| {
                fixed(
                    Shape {
                        layer: s.layer,
                        rect: s.rect.translate(at),
                    },
                    Owner::Obstacle,
                )
            }));
        }
    }

    out.extend(
        db.routing_blockages
            .iter()
            .map(|&s| fixed(s, Owner::Obstacle)),
    );
    out
}

/// A dense index for every layer a shape can be on: routing layers first,
/// then cut layers.
pub const fn layer_slot(db: &NetlistDB, layer: LayerRef) -> usize {
    match layer {
        LayerRef::Routing(l) => l.index(),
        LayerRef::Cut(c) => db.library.routing_layers.len() + c.index(),
    }
}
