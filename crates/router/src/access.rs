//! Pin access: the routing nodes a net may use to reach each of its pins.
//!
//! A pin on a routable layer is reached by any node of that layer inside
//! one of its shapes. A pin below the lowest routable layer is reached
//! through a stack of vias at a lowest-layer node. Any via defined on each
//! cut may be used: the lowest one's metal must sit inside the pin, and all
//! of the stack's metal must keep clear of the cell's other shapes. Among
//! the vias that fit, the one sticking out least across the routable
//! layer's tracks is chosen. Candidates are ordered by distance from the
//! pin's reference point.
//!
//! When no crossing falls on a usable spot, the pin is touched at an
//! off-crossing point on one of its tracks, and a short stub along that
//! track joins it to the nearest crossing. Candidates without a stub come
//! first.

use crate::tech::{RouteTech, ViaTech, to_u32};
use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{LayerId, PinId, ViaId};
use pare_common::db::library::{LayerRef, Shape};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// A routing node: a crossing on one track of one layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Node {
    /// Routing layer.
    pub layer: LayerId,
    /// Track index on that layer.
    pub track: u32,
    /// Crossing index along the track.
    pub cross: u32,
}

/// One way into a pin.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AccessPoint {
    /// The routing node a wire attaches to.
    pub node: Node,
    /// Where the pin is touched, on the node's track; the node's own point
    /// unless a stub along the track is needed.
    pub at: Point<Dbu>,
    /// Layer of the pin shape reached, at or below `node.layer`.
    pub pin_layer: LayerId,
    /// Vias from the pin layer up to the node's layer, bottom first.
    pub vias: Vec<ViaId>,
}

/// Most candidates kept per pin.
const MAX_CANDIDATES: usize = 8;

/// Access candidates for a pin, best first; empty if none is legal.
pub fn access_points(db: &NetlistDB, tech: &RouteTech, pin: PinId) -> Vec<AccessPoint> {
    let shapes = db.pin_shapes(pin);
    let target = db.pin_position(pin);
    let blockers = neighbour_shapes(db, pin);
    let mut found: Vec<((bool, i64), AccessPoint)> = Vec::new();

    for shape in &shapes {
        let LayerRef::Routing(layer) = shape.layer else {
            continue;
        };
        if tech.routable(layer) {
            let on_grid = nodes_inside(tech, layer, &shape.rect, |node, at| {
                let access = AccessPoint {
                    node,
                    at,
                    pin_layer: layer,
                    vias: Vec::new(),
                };
                found.push(((false, manhattan(at, target)), access));
                true
            });
            if !on_grid {
                stub_accesses(
                    tech,
                    layer,
                    &shape.rect,
                    |_| Some(Vec::new()),
                    |a, stub| {
                        found.push((
                            (true, stub + manhattan(a.at, target)),
                            AccessPoint {
                                pin_layer: layer,
                                ..a
                            },
                        ));
                    },
                );
            }
        } else if layer < tech.min_layer {
            let stack = |at: Point<Dbu>| via_stack(tech, layer, at, &shapes, &blockers);
            let on_grid = nodes_inside(tech, tech.min_layer, &shape.rect, |node, at| {
                let Some(vias) = stack(at) else { return false };
                let access = AccessPoint {
                    node,
                    at,
                    pin_layer: layer,
                    vias,
                };
                found.push(((false, manhattan(at, target)), access));
                true
            });
            if !on_grid {
                stub_accesses(tech, tech.min_layer, &shape.rect, stack, |a, stub| {
                    found.push((
                        (true, stub + manhattan(a.at, target)),
                        AccessPoint {
                            pin_layer: layer,
                            ..a
                        },
                    ));
                });
            }
        }
    }
    found.sort_by_key(|(key, a)| (*key, a.node));
    found.dedup_by_key(|(_, a)| a.node);
    found
        .into_iter()
        .take(MAX_CANDIDATES)
        .map(|(_, a)| a)
        .collect()
}

/// Off-crossing accesses on each track of `layer` through `rect`: the point
/// inside the rectangle nearest a crossing where `stack` finds vias, joined
/// to that crossing. Calls `f` with the access and its stub length.
fn stub_accesses(
    tech: &RouteTech,
    layer: LayerId,
    rect: &Rect<Dbu>,
    stack: impl Fn(Point<Dbu>) -> Option<Vec<ViaId>>,
    mut f: impl FnMut(AccessPoint, i64),
) {
    let lt = tech.layer(layer);
    let ((a0, a1), (b0, b1)) = lt.across_along_rect(rect);
    let center = b0.midpoint(b1);
    let k = lt.cross.partition_point(|c| c.at < center);
    let nearest = [k.checked_sub(1), (k < lt.cross.len()).then_some(k)];
    for track in lt.tracks_within(a0, a1) {
        let across = lt.tracks[track as usize];
        for cross in nearest.into_iter().flatten() {
            let node_along = lt.cross[cross].at;
            let candidates = [node_along.clamp(b0, b1), center];
            for along in candidates {
                let at = match lt.direction {
                    pare_common::db::library::Direction::Horizontal => Point::new(along, across),
                    pare_common::db::library::Direction::Vertical => Point::new(across, along),
                };
                if let Some(vias) = stack(at) {
                    let node = Node {
                        layer,
                        track,
                        cross: to_u32(cross),
                    };
                    f(
                        AccessPoint {
                            node,
                            at,
                            pin_layer: layer,
                            vias,
                        },
                        (along - node_along).abs().get(),
                    );
                    break;
                }
            }
        }
    }
}

fn manhattan(a: Point<Dbu>, b: Point<Dbu>) -> i64 {
    ((a.x - b.x).abs() + (a.y - b.y).abs()).get()
}

/// Calls `f` for every node of `layer` whose point lies inside `rect`;
/// returns whether `f` accepted any.
fn nodes_inside(
    tech: &RouteTech,
    layer: LayerId,
    rect: &Rect<Dbu>,
    mut f: impl FnMut(Node, Point<Dbu>) -> bool,
) -> bool {
    let lt = tech.layer(layer);
    let ((a0, a1), (b0, b1)) = lt.across_along_rect(rect);
    let mut any = false;
    for track in lt.tracks_within(a0, a1) {
        for cross in lt.cross_within(b0, b1) {
            let node = Node {
                layer,
                track,
                cross,
            };
            any |= f(node, lt.point(track, cross));
        }
    }
    any
}

/// Other shapes of the pin's cell, which the access vias must keep clear of.
fn neighbour_shapes(db: &NetlistDB, pin: PinId) -> Vec<Shape> {
    let PinOwner::Cell { cell, pin: own } = db.pins[pin.index()].owner else {
        return Vec::new();
    };
    let c = &db.cells[cell.index()];
    let m = db.cell_macro(cell);
    let place = |s: &Shape| Shape {
        layer: s.layer,
        rect: c.orient.place_rect(&s.rect, m.size, c.location),
    };
    let others = m
        .pins
        .iter()
        .enumerate()
        .filter(|&(k, _)| to_u32(k) != own.0)
        .flat_map(|(_, p)| p.shapes.iter().map(place));
    others.chain(m.obstructions.iter().map(place)).collect()
}

/// Whether `metal` on `layer` stays at least the layer spacing from every blocker.
fn clear_of(tech: &RouteTech, layer: LayerId, metal: &Rect<Dbu>, blockers: &[Shape]) -> bool {
    let reach = metal.expand(tech.layer(layer).spacing);
    blockers
        .iter()
        .all(|s| s.layer != LayerRef::Routing(layer) || !s.rect.overlaps(&reach))
}

/// The vias, bottom first, that carry a pin on `from` up to the lowest
/// routable layer at `at`, or `None` if no combination fits.
fn via_stack(
    tech: &RouteTech,
    from: LayerId,
    at: Point<Dbu>,
    pin: &[Shape],
    blockers: &[Shape],
) -> Option<Vec<ViaId>> {
    let mut stack = Vec::new();
    for l in from.index()..tech.min_layer.index() {
        let below = LayerId::new(l);
        let above = LayerId::new(l + 1);
        let fits = |v: &ViaTech| {
            let bottom = v.bottom.translate(at);
            let top = v.top.translate(at);
            let on_pin = below != from
                || pin
                    .iter()
                    .any(|s| s.layer == LayerRef::Routing(from) && s.rect.encloses(&bottom));
            on_pin
                && clear_of(tech, below, &bottom, blockers)
                && clear_of(tech, above, &top, blockers)
        };
        let via = tech.access_vias[l].iter().find(|v| fits(v))?;
        stack.push(via.via);
    }
    Some(stack)
}
