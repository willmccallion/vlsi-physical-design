//! Metal and cut rectangles of a design, tagged with what they belong to.

use crate::db::design::{Cell, IoPin, NetlistDB, PinOwner, RouteElem, SpecialNet, SpecialWire};
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
    /// A patch of metal on routed wiring.
    Patch,
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
        rect: special_rect_box(&s.rect),
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
    let even = |d: Dbu| d.get() % 2 == 0;
    let along_y = match (even(dx), even(dy)) {
        (true, true) => dy >= dx,
        (true, false) => true,
        (false, true) => false,
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
                RouteElem::Patch { .. } => Source::Patch,
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
        source: Source::Fixed,
    };

    for i in 0..db.num_cells() {
        let id = CellId::new(i);
        let c = &db.cells[i];
        if !keep_cell(c) {
            continue;
        }
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::db::indices::LayerId;

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::from_corners(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    fn wire(from: (i64, i64), to: (i64, i64), shape: Option<&str>) -> SpecialWire {
        SpecialWire {
            layer: LayerId(0),
            width: Dbu(100),
            from: Point::new(Dbu(from.0), Dbu(from.1)),
            to: Point::new(Dbu(to.0), Dbu(to.1)),
            extensions: (Dbu::ZERO, Dbu::ZERO),
            shape: shape.map(str::to_string),
        }
    }

    #[test]
    fn a_shaped_special_wire_keeps_its_box() {
        let w = wire((0, 500), (1000, 500), Some("STRIPE"));
        assert_eq!(special_wire_box(&w), rect(0, 450, 1000, 550));
    }

    #[test]
    fn an_unshaped_special_wire_grows_by_half_its_width_at_both_ends() {
        assert_eq!(
            special_wire_box(&wire((0, 500), (1000, 500), None)),
            rect(-50, 450, 1050, 550)
        );
        assert_eq!(
            special_wire_box(&wire((500, 0), (500, 1000), None)),
            rect(450, -50, 550, 1050)
        );
    }

    #[test]
    fn a_special_rect_with_one_even_side_grows_along_the_other() {
        assert_eq!(
            special_rect_box(&rect(0, 0, 900, 45)),
            rect(0, -450, 900, 495)
        );
        assert_eq!(
            special_rect_box(&rect(0, 0, 45, 900)),
            rect(-450, 0, 495, 900)
        );
    }

    #[test]
    fn a_special_rect_even_both_ways_takes_the_shorter_side_as_its_width() {
        assert_eq!(
            special_rect_box(&rect(0, 0, 400, 100)),
            rect(-50, 0, 450, 100)
        );
        assert_eq!(
            special_rect_box(&rect(0, 0, 100, 400)),
            rect(0, -50, 100, 450)
        );
        assert_eq!(
            special_rect_box(&rect(0, 0, 200, 200)),
            rect(0, -100, 200, 300)
        );
    }

    #[test]
    fn a_special_rect_odd_both_ways_stays_as_written() {
        assert_eq!(special_rect_box(&rect(0, 0, 45, 45)), rect(0, 0, 45, 45));
    }
}
