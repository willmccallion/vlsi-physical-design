//! Whether fixed metal leaves a cell's pins a via to reach them by.
//!
//! A pin below the routing layers is reached by a via whose cut lies inside
//! the pin at a crossing of the pin layer's and the first routing layer's
//! tracks, and whose metal on the routing layer keeps that layer's spacing
//! from the fixed metal of every other owner there: power stripes,
//! blockages, macro obstructions and other fixed pins. A pin with no such
//! via is blocked, as is a pin whose own metal on the routing layer comes
//! within that spacing of such fixed metal, and placement must not leave a
//! cell where one of its pins is.

use crate::db::design::{NetlistDB, PinOwner};
use crate::db::indices::{CellId, CutLayerId, LayerId, MacroPinId, NetId};
use crate::db::library::{Direction, LayerRef, Macro};
use crate::geom::dbu::Dbu;
use crate::geom::orient::Orientation;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use crate::verify::shapes::{Owner, obstruction_shapes};
use rstar::{AABB, RTree, RTreeObject};

/// A fixed shape on the routing layer and the net it belongs to, if any.
#[derive(Clone, Debug)]
struct Fixed {
    rect: Rect<Dbu>,
    net: Option<NetId>,
}

impl RTreeObject for Fixed {
    type Envelope = AABB<[i64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        envelope(&self.rect)
    }
}

fn envelope(r: &Rect<Dbu>) -> AABB<[i64; 2]> {
    AABB::from_corners(
        [r.min.x.get(), r.min.y.get()],
        [r.max.x.get(), r.max.y.get()],
    )
}

/// A via from the pin layer up: its cut and its metal on the routing
/// layer, centered on the origin.
#[derive(Clone, Debug)]
struct ViaUp {
    cut: Rect<Dbu>,
    pad: Rect<Dbu>,
}

/// The check of pins on the layer below the lowest routing layer against
/// the fixed metal on that routing layer.
#[derive(Debug)]
pub struct PinAccess {
    pin_layer: LayerId,
    above: LayerId,
    /// Coordinates across the routing layer of its tracks.
    tracks_above: Vec<Dbu>,
    /// Coordinates across the pin layer of its tracks.
    tracks_pin: Vec<Dbu>,
    above_vertical: bool,
    vias: Vec<ViaUp>,
    spacing: Dbu,
    fixed: RTree<Fixed>,
}

impl PinAccess {
    /// The check for pins on the layer below `above`, the lowest layer
    /// signal wires use, or `None` when no library via joins the two.
    pub fn new(db: &NetlistDB, above: LayerId) -> Option<Self> {
        let pin_layer = LayerId::new(above.index().checked_sub(1)?);
        let cut = CutLayerId::new(pin_layer.index());
        let lib = &db.library;
        let vias: Vec<ViaUp> = lib
            .vias
            .iter()
            .filter(|v| v.cut == cut && !v.from_def)
            .filter_map(|v| {
                let cut_rect = v
                    .shapes
                    .iter()
                    .find(|s| s.layer == LayerRef::Cut(cut))?
                    .rect;
                let pad = v
                    .shapes
                    .iter()
                    .filter(|s| s.layer == LayerRef::Routing(above))
                    .map(|s| s.rect)
                    .reduce(|a, b| a.union(&b))?;
                Some(ViaUp { cut: cut_rect, pad })
            })
            .collect();
        if vias.is_empty() {
            return None;
        }
        let fixed: Vec<Fixed> = obstruction_shapes(db)
            .into_iter()
            .filter(|s| s.shape.layer == LayerRef::Routing(above))
            .map(|s| Fixed {
                rect: s.shape.rect,
                net: match s.owner {
                    Owner::Net(n) => Some(n),
                    Owner::Obstacle => None,
                },
            })
            .collect();
        Some(Self {
            pin_layer,
            above,
            tracks_above: db.layer_tracks(above),
            tracks_pin: db.layer_tracks(pin_layer),
            above_vertical: lib.layer(above).direction == Direction::Vertical,
            vias,
            spacing: lib.layer(above).min_spacing,
            fixed: RTree::bulk_load(fixed),
        })
    }

    /// How many signal pins of `cell`, placed at `at` with `orient`, no via
    /// can reach.
    pub fn blocked_pins(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        orient: Orientation,
    ) -> usize {
        let c = &db.cells[cell.index()];
        let m = db.library.cell_macro(c.macro_id);
        c.pins
            .iter()
            .filter(|&&p| {
                let pin = &db.pins[p.index()];
                let PinOwner::Cell { pin: mp, .. } = pin.owner else {
                    return false;
                };
                !db.nets[pin.net.index()].signal_use.is_supply()
                    && self.pin_blocked(m, mp, pin.net, at, orient)
            })
            .count()
    }

    /// Whether the pin's metal on the routing layer conflicts with fixed
    /// metal, or none of its shapes on the pin layer can be reached while
    /// it has no clear metal on the routing layer. A pin with shapes on
    /// neither layer is never blocked here.
    fn pin_blocked(
        &self,
        m: &Macro,
        mp: MacroPinId,
        net: NetId,
        at: Point<Dbu>,
        orient: Orientation,
    ) -> bool {
        let mut conflict_above = false;
        let mut clear_above = false;
        let mut on_pin_layer = false;
        let mut reachable_below = false;
        for s in &m.pins[mp.index()].shapes {
            let rect = orient.place_rect(&s.rect, m.size, at);
            if s.layer == LayerRef::Routing(self.above) {
                if self.clear(&rect, net) {
                    clear_above = true;
                } else {
                    conflict_above = true;
                }
            } else if s.layer == LayerRef::Routing(self.pin_layer) {
                on_pin_layer = true;
                reachable_below |= self.reachable(&rect, net);
            }
        }
        conflict_above || (on_pin_layer && !reachable_below && !clear_above)
    }

    /// Whether some via with its cut inside `rect`, at a crossing of the two
    /// layers' tracks, keeps the spacing from fixed metal of other nets.
    fn reachable(&self, rect: &Rect<Dbu>, net: NetId) -> bool {
        let (xs, ys) = if self.above_vertical {
            (&self.tracks_above, &self.tracks_pin)
        } else {
            (&self.tracks_pin, &self.tracks_above)
        };
        self.vias.iter().any(|via| {
            within(xs, rect.min.x - via.cut.min.x, rect.max.x - via.cut.max.x).any(|x| {
                within(ys, rect.min.y - via.cut.min.y, rect.max.y - via.cut.max.y)
                    .any(|y| self.clear(&via.pad.translate(Point::new(x, y)), net))
            })
        })
    }

    /// Whether `pad` keeps the spacing from every fixed shape of another net.
    fn clear(&self, pad: &Rect<Dbu>, net: NetId) -> bool {
        let zone = pad.expand(self.spacing);
        self.fixed
            .locate_in_envelope_intersecting(envelope(&zone))
            .all(|f| f.net == Some(net) || !f.rect.overlaps(&zone))
    }
}

/// The track coordinates in `[lo, hi]`.
fn within(tracks: &[Dbu], lo: Dbu, hi: Dbu) -> impl Iterator<Item = Dbu> + '_ {
    let a = tracks.partition_point(|&t| t < lo);
    let b = tracks.partition_point(|&t| t <= hi).max(a);
    tracks[a..b].iter().copied()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::db::design::{PlacementStatus, SpecialNet, SpecialWire, TrackAxis, TrackDef};
    use crate::db::indices::MacroPinId;
    use crate::db::library::{
        CoreKind, CutLayer, Library, MacroClass, MacroPin, PinDirection, RoutingLayer, Shape,
        SignalUse, Site, SiteClass, Symmetry, ViaDef,
    };
    use crate::db::rules::{CutLayerRules, RoutingLayerRules};
    use crate::geom::dbu::DbuPerMicron;

    const fn pt(x: i64, y: i64) -> Point<Dbu> {
        Point::new(Dbu(x), Dbu(y))
    }

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(pt(x0, y0), pt(x1, y1))
    }

    fn layer(name: &str, direction: Direction) -> RoutingLayer {
        RoutingLayer {
            name: name.into(),
            direction,
            pitch_x: Dbu(200),
            pitch_y: Dbu(200),
            offset_x: None,
            offset_y: None,
            width: Dbu(100),
            min_spacing: Dbu(100),
            spacing_table: None,
            rules: RoutingLayerRules::default(),
        }
    }

    /// Horizontal M1 under vertical M2 with a 0.1 cut via whose M2 pad is
    /// 0.1 by 0.18; one 0.4 by 2.0 cell master with a 0.1-wide M1 pin
    /// centred 0.1 in from its left edge; a 0.58-wide M2 ground stripe at
    /// x = 3.9 over a 10 by 4 die.
    fn design() -> (NetlistDB, crate::db::indices::MacroId) {
        let mut lib = Library::new(DbuPerMicron::new(1000).unwrap());
        let m1 = lib
            .add_routing_layer(layer("M1", Direction::Horizontal))
            .unwrap();
        let v1 = lib
            .add_cut_layer(CutLayer {
                name: "V1".into(),
                width: Some(Dbu(100)),
                spacing: Some(Dbu(120)),
                rules: CutLayerRules::default(),
            })
            .unwrap();
        let m2 = lib
            .add_routing_layer(layer("M2", Direction::Vertical))
            .unwrap();
        let _ = lib
            .add_via(ViaDef {
                name: "V12".into(),
                is_default: true,
                cut: v1,
                shapes: vec![
                    Shape {
                        layer: LayerRef::Routing(m1),
                        rect: rect(-90, -50, 90, 50),
                    },
                    Shape {
                        layer: LayerRef::Cut(v1),
                        rect: rect(-50, -50, 50, 50),
                    },
                    Shape {
                        layer: LayerRef::Routing(m2),
                        rect: rect(-50, -90, 50, 90),
                    },
                ],
                from_def: false,
            })
            .unwrap();
        let symmetry = Symmetry {
            x: true,
            y: true,
            r90: false,
        };
        let site = lib
            .add_site(Site {
                name: "s".into(),
                class: SiteClass::Core,
                size: pt(200, 2000),
                symmetry,
            })
            .unwrap();
        let mut m = Macro::new(
            "C".into(),
            MacroClass::Core(CoreKind::Regular),
            pt(400, 2000),
            symmetry,
            Some(site),
        );
        let _ = m.add_pin(MacroPin {
            name: "a".into(),
            direction: PinDirection::Input,
            signal_use: SignalUse::Signal,
            shapes: vec![Shape {
                layer: LayerRef::Routing(m1),
                rect: rect(50, 500, 150, 1500),
            }],
        });
        let m = lib.add_macro(m).unwrap();
        let mut upper = Macro::new(
            "U".into(),
            MacroClass::Core(CoreKind::Regular),
            pt(400, 2000),
            symmetry,
            Some(site),
        );
        let _ = upper.add_pin(MacroPin {
            name: "a".into(),
            direction: PinDirection::Input,
            signal_use: SignalUse::Signal,
            shapes: vec![Shape {
                layer: LayerRef::Routing(m2),
                rect: rect(50, 500, 150, 1500),
            }],
        });
        let _ = lib.add_macro(upper).unwrap();
        let mut db = NetlistDB::new("t".into(), lib);
        db.die_area = rect(0, 0, 10_000, 4000);
        db.tracks.push(TrackDef {
            axis: TrackAxis::X,
            start: Dbu(100),
            count: 50,
            step: Dbu(200),
            layers: vec![m2],
        });
        db.tracks.push(TrackDef {
            axis: TrackAxis::Y,
            start: Dbu(0),
            count: 21,
            step: Dbu(200),
            layers: vec![m1],
        });
        db.special_nets.push(SpecialNet {
            name: "vss".into(),
            signal_use: SignalUse::Ground,
            connections: Vec::new(),
            wires: vec![SpecialWire {
                layer: m2,
                width: Dbu(580),
                from: pt(3900, 0),
                to: pt(3900, 4000),
                extensions: (Dbu::ZERO, Dbu::ZERO),
                shape: Some("STRIPE".into()),
            }],
            rects: Vec::new(),
            vias: Vec::new(),
        });
        (db, m)
    }

    fn cell_at(db: &mut NetlistDB, m: crate::db::indices::MacroId, x: i64) -> CellId {
        let cell = db
            .add_cell(
                format!("c{x}"),
                m,
                pt(x, 0),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        let net = db.net_or_insert(&format!("n{x}"));
        let _ = db.connect_cell_pin(cell, MacroPinId(0), net);
        cell
    }

    fn blocked(x: i64) -> usize {
        let (mut db, m) = design();
        let cell = cell_at(&mut db, m, x);
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        access.blocked_pins(&db, cell, pt(x, 0), Orientation::N)
    }

    #[test]
    fn a_pin_under_the_stripe_is_blocked() {
        assert_eq!(blocked(3800), 1);
    }

    #[test]
    fn a_pin_whose_via_pad_would_come_within_spacing_of_the_stripe_is_blocked() {
        assert_eq!(blocked(3400), 1);
    }

    #[test]
    fn a_pin_whose_via_pad_keeps_the_spacing_is_free() {
        assert_eq!(blocked(3200), 0);
    }

    #[test]
    fn a_pin_far_from_the_stripe_is_free() {
        assert_eq!(blocked(1000), 0);
    }

    fn blocked_upper(x: i64) -> usize {
        let (mut db, _) = design();
        let cell = cell_at(&mut db, crate::db::indices::MacroId(1), x);
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        access.blocked_pins(&db, cell, pt(x, 0), Orientation::N)
    }

    #[test]
    fn a_routing_layer_pin_within_spacing_of_the_stripe_is_blocked() {
        assert_eq!(blocked_upper(3400), 1);
    }

    #[test]
    fn a_routing_layer_pin_clear_of_the_stripe_is_free() {
        assert_eq!(blocked_upper(3200), 0);
    }

    #[test]
    fn a_design_without_a_via_up_from_the_pins_has_no_check() {
        let (mut db, _) = design();
        db.library.vias.clear();

        assert!(PinAccess::new(&db, LayerId(1)).is_none());
    }
}
