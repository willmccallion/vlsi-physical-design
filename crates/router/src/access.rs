//! Pin access: the routing nodes a net may use to reach each of its pins.
//!
//! A pin on a routable layer is reached by any node of that layer inside
//! one of its shapes. A pin below the lowest routable layer is reached
//! through a stack of vias at a lowest-layer node. Any via defined on each
//! cut may be used: the lowest one's cuts must sit on the pin, and all of
//! the stack's metal must keep clear of the cell's other shapes. Every via
//! that fits onto the pin gives a candidate of its own, since which one
//! suits a pin depends on the vias its neighbours' pins take; the vias above
//! it are those sticking out least across the routable layer's tracks.
//! Candidates are ordered by distance from the pin's reference point, then
//! by how far the lowest via sticks out across those tracks.
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

/// Most access nodes kept per pin, each with every via stack that fits.
const MAX_CANDIDATE_NODES: usize = 8;

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
                    |_| vec![Vec::new()],
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
            let stacks = |at: Point<Dbu>| via_stacks(tech, layer, at, &shapes, &blockers);
            let on_grid = nodes_inside(tech, tech.min_layer, &shape.rect, |node, at| {
                let fitting = stacks(at);
                let any = !fitting.is_empty();
                for vias in fitting {
                    let access = AccessPoint {
                        node,
                        at,
                        pin_layer: layer,
                        vias,
                    };
                    found.push(((false, manhattan(at, target)), access));
                }
                any
            });
            if !on_grid {
                stub_accesses(tech, tech.min_layer, &shape.rect, stacks, |a, stub| {
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
    best_per_node(found)
}

/// The candidates of the first [`MAX_CANDIDATE_NODES`] distinct nodes of
/// `found`, which is in order of preference, each kept once.
fn best_per_node(found: Vec<((bool, i64), AccessPoint)>) -> Vec<AccessPoint> {
    let mut nodes: Vec<Node> = Vec::new();
    let mut kept: Vec<AccessPoint> = Vec::new();
    for (_, a) in found {
        if kept.iter().any(|k| k.node == a.node && k.vias == a.vias) {
            continue;
        }
        if !nodes.contains(&a.node) {
            if nodes.len() == MAX_CANDIDATE_NODES {
                continue;
            }
            nodes.push(a.node);
        }
        kept.push(a);
    }
    kept
}

/// Off-crossing accesses on each track of `layer` through `rect`: the point
/// inside the rectangle nearest a crossing where `stack` finds vias, joined
/// to that crossing. Calls `f` with the access and its stub length.
fn stub_accesses(
    tech: &RouteTech,
    layer: LayerId,
    rect: &Rect<Dbu>,
    stacks: impl Fn(Point<Dbu>) -> Vec<Vec<ViaId>>,
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
                let fitting = stacks(at);
                if !fitting.is_empty() {
                    let node = Node {
                        layer,
                        track,
                        cross: to_u32(cross),
                    };
                    for vias in fitting {
                        f(
                            AccessPoint {
                                node,
                                at,
                                pin_layer: layer,
                                vias,
                            },
                            (along - node_along).abs().get(),
                        );
                    }
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

/// Whether `pad` on `layer`, metal of the pin with shapes `pin`, keeps at
/// least the layer spacing from every shape of the pin it does not touch: a
/// pad sticking out of one part of a pin must not end just short of another
/// part, which the checker counts as a spacing violation within the net.
fn clear_of_own_pin(tech: &RouteTech, layer: LayerId, pad: &Rect<Dbu>, pin: &[Shape]) -> bool {
    let reach = pad.expand(tech.layer(layer).spacing);
    pin.iter()
        .filter(|s| s.layer == LayerRef::Routing(layer))
        .all(|s| s.rect.touches(pad) || !s.rect.overlaps(&reach))
}

/// Every stack of vias, bottom first, that carries a pin on `from` up to
/// the lowest routable layer at `at`: one for each via that fits onto the
/// pin, topped by the first vias that fit above it. The lowest via's cuts
/// must lie on the pin; its metal may stick out of it, keeping clear of the
/// cell's other shapes like every via's metal, and of the parts of the pin
/// it does not touch.
fn via_stacks(
    tech: &RouteTech,
    from: LayerId,
    at: Point<Dbu>,
    pin: &[Shape],
    blockers: &[Shape],
) -> Vec<Vec<ViaId>> {
    let fits = |l: usize, v: &ViaTech| {
        let (below, above) = (LayerId::new(l), LayerId::new(l + 1));
        let bottom = v.bottom.translate(at);
        let top = v.top.translate(at);
        let on_pin = below != from
            || (v.cuts.iter().all(|cut| {
                let cut = cut.translate(at);
                pin.iter()
                    .any(|s| s.layer == LayerRef::Routing(from) && s.rect.encloses(&cut))
            }) && clear_of_own_pin(tech, below, &bottom, pin));
        on_pin && clear_of(tech, below, &bottom, blockers) && clear_of(tech, above, &top, blockers)
    };
    let first = from.index();
    let upper: Option<Vec<ViaId>> = (first + 1..tech.min_layer.index())
        .map(|l| {
            tech.access_vias[l]
                .iter()
                .find(|v| fits(l, v))
                .map(|v| v.via)
        })
        .collect();
    let Some(upper) = upper else {
        return Vec::new();
    };
    tech.access_vias[first]
        .iter()
        .filter(|v| fits(first, v))
        .map(|v| {
            std::iter::once(v.via)
                .chain(upper.iter().copied())
                .collect()
        })
        .collect()
}
