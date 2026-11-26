//! Metal and cut rectangles of a design, tagged with what they belong to.

use crate::db::design::{
    Cell, IoPin, NetlistDB, NonDefaultRule, PinOwner, RouteElem, SpecialNet, SpecialWire,
};
use crate::db::indices::{IdVec, LayerId, NetId};
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

/// A fixed rectangle on a layer and what it belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct OwnedShape {
    /// Its layer and rectangle.
    pub shape: Shape,
    /// Its net, if any.
    pub owner: Owner,
    /// Whether it is a cell obstruction or routing blockage rather than a pin
    /// or wire, which spacing rules may take as default-width metal.
    pub obstruction: bool,
}

/// The rectangle a default-width wire covers, extended by half its width
/// past each end, or `None` if the wire is not horizontal or vertical.
pub fn wire_rect(from: Point<Dbu>, to: Point<Dbu>, width: Dbu) -> Option<Rect<Dbu>> {
    let half = width / 2;
    wire_rect_with_ends(from, to, width, half, half)
}

/// The rectangle a wire of `width` covers, extended past `from` by
/// `from_extension` and past `to` by `to_extension`; `None` for a diagonal
/// wire.
pub fn wire_rect_with_ends(
    from: Point<Dbu>,
    to: Point<Dbu>,
    width: Dbu,
    from_extension: Dbu,
    to_extension: Dbu,
) -> Option<Rect<Dbu>> {
    if from.x != to.x && from.y != to.y {
        return None;
    }
    let lo = width / 2;
    let hi = width - lo;
    let from_is_min = from.x <= to.x && from.y <= to.y;
    let (at_min, at_max) = if from_is_min {
        (from_extension, to_extension)
    } else {
        (to_extension, from_extension)
    };
    let r = Rect::from_corners(from, to);
    let (dx, dy) = if from.y == to.y {
        (Point::new(at_min, lo), Point::new(at_max, hi))
    } else {
        (Point::new(lo, at_min), Point::new(hi, at_max))
    };
    Some(Rect::new(r.min - dx, r.max + dy))
}

/// The nondefault rule the wires of `net` follow, if any.
fn rule_of(db: &NetlistDB, net: NetId) -> Option<&NonDefaultRule> {
    db.nets[net]
        .non_default_rule
        .map(|r| &db.non_default_rules[r])
}

/// The width wires of `net` have on `layer`: the layer's unless the net's
/// nondefault rule sets one.
pub fn wire_width(db: &NetlistDB, net: NetId, layer: LayerId) -> Dbu {
    rule_of(db, net)
        .and_then(|r| r.layers.iter().find(|l| l.layer == layer))
        .map_or_else(|| db.library.layer(layer).width, |l| l.width)
}

/// The rectangle a wire of `net` covers: its width on the layer, extended
/// past each end as the path says, else by half that width or by the
/// nondefault rule's extension.
fn wire_shape(
    db: &NetlistDB,
    net: NetId,
    layer: LayerId,
    from: Point<Dbu>,
    to: Point<Dbu>,
    extensions: (Option<Dbu>, Option<Dbu>),
) -> Option<Rect<Dbu>> {
    let rule = rule_of(db, net).and_then(|r| r.layers.iter().find(|l| l.layer == layer));
    let width = rule.map_or_else(|| db.library.layer(layer).width, |l| l.width);
    let default = rule.and_then(|l| l.wire_extension).unwrap_or(width / 2);
    wire_rect_with_ends(
        from,
        to,
        width,
        extensions.0.unwrap_or(default),
        extensions.1.unwrap_or(default),
    )
}

/// The shapes of one routing element of `net`; empty for a diagonal wire.
pub fn route_elem_shapes(db: &NetlistDB, net: NetId, elem: &RouteElem) -> Vec<Shape> {
    match *elem {
        RouteElem::Wire {
            layer,
            from,
            to,
            extensions,
        } => wire_shape(db, net, layer, from, to, extensions)
            .map(|rect| Shape {
                layer: LayerRef::Routing(layer),
                rect,
            })
            .into_iter()
            .collect(),
        _ => default_width_shapes(db, elem),
    }
}

/// The shapes of one routing element with every wire at its layer's default
/// width, which is how `OpenROAD` reads routed wiring whatever the net's
/// nondefault rule says.
pub fn default_width_shapes(db: &NetlistDB, elem: &RouteElem) -> Vec<Shape> {
    match *elem {
        RouteElem::Wire {
            layer,
            from,
            to,
            extensions,
        } => {
            let width = db.library.layer(layer).width;
            let half = width / 2;
            wire_rect_with_ends(
                from,
                to,
                width,
                extensions.0.unwrap_or(half),
                extensions.1.unwrap_or(half),
            )
        }
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
        RouteElem::Patch { layer, rect } => vec![Shape {
            layer: LayerRef::Routing(layer),
            rect,
        }],
    }
}

/// The metal of a special net as `TritonRoute` reads it.
///
/// Every wire and `RECT` is a path along its centerline, and one without a
/// `SHAPE` grows by half its width at both ends. A `RECT` odd in both
/// dimensions, which `OpenDB` drops, stays as written.
pub fn special_net_metal(net: &SpecialNet) -> impl Iterator<Item = Shape> + '_ {
    let wires = net.wires.iter().map(|w| Shape {
        layer: LayerRef::Routing(w.layer),
        rect: special_wire_box(w),
    });
    let rects = net.rects.iter().map(|s| Shape {
        layer: s.layer,
        rect: if s.shape.is_some() {
            s.rect
        } else {
            special_rect_box(&s.rect)
        },
    });
    wires.chain(rects)
}

fn special_wire_box(wire: &SpecialWire) -> Rect<Dbu> {
    let rect = wire.rect();
    if wire.shape.is_some() {
        return rect;
    }
    let along_y = wire.from.y != wire.to.y;
    extend_along(&rect, along_y, wire.width / 2)
}

/// A `RECT` read as a path: an even side is the width (the shorter one when
/// both are even) and the box grows by half of it along the other axis.
fn special_rect_box(rect: &Rect<Dbu>) -> Rect<Dbu> {
    let (dx, dy) = (rect.width(), rect.height());
    let even = |d: Dbu| d.is_multiple_of(Dbu::new(2));
    let along_y = match (even(dx), even(dy)) {
        (true, true) => dy >= dx,
        (true, false) => true,
        (false, true) => false,
        // NOTE: dbSBox::create drops such a box; PARE keeps it as given.
        (false, false) => return *rect,
    };
    let width = if along_y { dx } else { dy };
    extend_along(rect, along_y, width / 2)
}

fn extend_along(rect: &Rect<Dbu>, along_y: bool, by: Dbu) -> Rect<Dbu> {
    let step = if along_y {
        Point::new(Dbu::ZERO, by)
    } else {
        Point::new(by, Dbu::ZERO)
    };
    Rect::new(rect.min - step, rect.max + step)
}

/// Whether the router is responsible for connecting this net.
pub fn is_signal_net(db: &NetlistDB, net: NetId) -> bool {
    let n = &db.nets[net];
    !n.signal_use.is_supply() && n.pins.len() >= 2
}

/// Everything the wiring must connect to or avoid: pins, obstructions,
/// special nets and routing blockages.
pub fn fixed_shapes(db: &NetlistDB) -> Vec<OwnedShape> {
    fixed_shapes_keeping_io(db, |_| true)
}

/// [`fixed_shapes`] with only the I/O pins `keep` accepts.
pub fn fixed_shapes_keeping_io(db: &NetlistDB, keep: impl Fn(&IoPin) -> bool) -> Vec<OwnedShape> {
    fixed_shapes_of(db, |_| true, keep)
}

/// The shapes placement cannot move: those of fixed cells and fixed I/O
/// pins, special nets and routing blockages.
pub fn obstruction_shapes(db: &NetlistDB) -> Vec<OwnedShape> {
    fixed_shapes_of(
        db,
        |c| !c.status.is_movable(),
        |p: &IoPin| !p.status.is_movable(),
    )
}

/// [`fixed_shapes`] of only the cells and I/O pins the filters accept.
fn fixed_shapes_of(
    db: &NetlistDB,
    keep_cell: impl Fn(&Cell) -> bool,
    keep_io: impl Fn(&IoPin) -> bool,
) -> Vec<OwnedShape> {
    let mut out = Vec::new();
    let fixed = |shape: Shape, owner: Owner| OwnedShape {
        shape,
        owner,
        obstruction: false,
    };
    let blocker = |shape: Shape| OwnedShape {
        shape,
        owner: Owner::Obstacle,
        obstruction: true,
    };

    for (id, c) in db.cells.iter_enumerated() {
        if !keep_cell(c) {
            continue;
        }
        let m = db.cell_macro(id);
        let place = |s: &Shape| Shape {
            layer: s.layer,
            rect: c.orient.place_rect(&s.rect, m.size, c.location),
        };
        let mut pin_net = IdVec::filled(None, m.pins.len());
        for &pin in &c.pins {
            if let PinOwner::Cell { pin: mp, .. } = db.pins[pin].owner {
                let net = db.pins[pin].net;
                if is_signal_net(db, net) {
                    pin_net[mp] = Some(net);
                }
            }
        }
        for (k, mp) in m.pins.iter_enumerated() {
            let owner = pin_net[k].map_or(Owner::Obstacle, Owner::Net);
            out.extend(mp.shapes.iter().map(|s| fixed(place(s), owner)));
        }
        out.extend(m.obstructions.iter().map(|s| blocker(place(s))));
    }

    for p in db.io_pins.iter().filter(|p| keep_io(p)) {
        let owner = if is_signal_net(db, p.net) {
            Owner::Net(p.net)
        } else {
            Owner::Obstacle
        };
        out.extend(p.placed_shapes().map(|s| fixed(s, owner)));
    }

    for n in &db.special_nets {
        out.extend(special_net_metal(n).map(|s| fixed(s, Owner::Obstacle)));
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

    out.extend(db.routing_blockages.iter().map(|&s| blocker(s)));
    out
}

#[cfg(test)]
mod tests;
