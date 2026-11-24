//! The ways a wire may leave an access point's node, each judged by the
//! design rule check as `FlexPA` judges an access point's directions.

use super::AccessPoint;
use crate::tech::RouteTech;
use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{LayerId, PinId};
use pare_common::db::library::{Direction, MacroClass};
use pare_common::db::shapes::{wire_rect, wire_rect_with_ends};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// How far past a pin that is not a block's a planar way is judged, in
/// wire widths: `FlexPA::genEndPoint`'s step of three widths.
const STEP_WIDTHS: i64 = 3;

/// One way a wire leaves a node: along its track or across to a
/// neighbouring track, toward lower or higher coordinates, or through the
/// via down or up.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Way {
    /// Along the track, toward lower coordinates.
    AlongLower,
    /// Along the track, toward higher coordinates.
    AlongHigher,
    /// Across to the track below.
    AcrossLower,
    /// Across to the track above.
    AcrossHigher,
    /// Through the via to the layer below.
    Down,
    /// Through the via to the layer above.
    Up,
}

impl Way {
    /// The ways that stay on the node's layer.
    pub const PLANAR: [Self; 4] = [
        Self::AlongLower,
        Self::AlongHigher,
        Self::AcrossLower,
        Self::AcrossHigher,
    ];

    const fn bit(self) -> u8 {
        match self {
            Self::AlongLower => 1,
            Self::AlongHigher => 2,
            Self::AcrossLower => 4,
            Self::AcrossHigher => 8,
            Self::Down => 16,
            Self::Up => 32,
        }
    }

    /// Whether it stays on the node's layer.
    pub(crate) const fn is_planar(self) -> bool {
        !matches!(self, Self::Down | Self::Up)
    }

    /// Whether it runs against its layer's preferred direction.
    const fn is_across(self) -> bool {
        matches!(self, Self::AcrossLower | Self::AcrossHigher)
    }

    /// Whether it heads toward lower coordinates.
    const fn is_lower(self) -> bool {
        matches!(self, Self::AlongLower | Self::AcrossLower)
    }
}

/// A set of ways, as `frAccessPoint` keeps the accesses an access point
/// allows.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Ways(u8);

impl Ways {
    /// No way at all.
    pub const NONE: Self = Self(0);
    /// Every way.
    pub const ALL: Self = Self(63);

    /// Whether `way` is among them.
    #[must_use]
    pub const fn contains(self, way: Way) -> bool {
        self.0 & way.bit() != 0
    }

    /// These and `way`.
    #[must_use]
    pub const fn with(self, way: Way) -> Self {
        Self(self.0 | way.bit())
    }

    /// These and `other`.
    #[must_use]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    /// Whether there is none.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Whether a way along the track is among them: `hasHorzAccess` on a
    /// horizontal layer, `hasVertAccess` on a vertical one.
    #[must_use]
    pub const fn has_along(self) -> bool {
        self.contains(Way::AlongLower) || self.contains(Way::AlongHigher)
    }

    /// Whether a way across the tracks is among them.
    #[must_use]
    pub const fn has_across(self) -> bool {
        self.contains(Way::AcrossLower) || self.contains(Way::AcrossHigher)
    }

    /// The bits a state file records them by.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// The ways `bits` record; none when a bit stands for no way.
    #[must_use]
    pub const fn from_bits(bits: u8) -> Option<Self> {
        if bits & !Self::ALL.0 == 0 {
            Some(Self(bits))
        } else {
            None
        }
    }
}

/// What a pin belongs to, which decides how pin access judges its access
/// points and how routing treats them, as `TritonRoute` tells pins apart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinKind {
    /// A pin of a standard cell, a CORE master (`FlexPA::isStdCell`).
    Core,
    /// A pin of a hard block, a BLOCK master (`dbMasterType::isBlock`).
    Block,
    /// A pin of a pad or ring cell, a macro to `FlexPA::isMacroCell` but no
    /// block.
    PadOrRing,
    /// A pin of an end cap or cover cell, to pin access neither a standard
    /// cell nor a macro.
    EndcapOrCover,
    /// An I/O pin of the design.
    Io,
}

impl PinKind {
    /// Whether detailed routing treats it as a standard cell's, as
    /// `FlexDRWorker::initMazeCost_ap_helper` tells the pins of blocks,
    /// pads, rings and the design's I/O apart from the rest.
    #[must_use]
    pub const fn is_standard_in_routing(self) -> bool {
        matches!(self, Self::Core | Self::EndcapOrCover)
    }

    /// The kind of `pin` of `db`.
    pub fn of(db: &NetlistDB, pin: PinId) -> Self {
        let PinOwner::Cell { cell, .. } = db.pins[pin].owner else {
            return Self::Io;
        };
        match db.cell_macro(cell).class {
            MacroClass::Core(_) => Self::Core,
            MacroClass::Block => Self::Block,
            MacroClass::Pad | MacroClass::Ring => Self::PadOrRing,
            MacroClass::Endcap | MacroClass::Cover => Self::EndcapOrCover,
        }
    }
}

/// How wide a search for via ways is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum ViaSearch {
    /// Only vias whose metal stays inside the pin's cell.
    WithinCell,
    /// Any via, once no access point of the pin found a way within its
    /// cell: `FlexPA::filterViaAccess` with `deep_search`.
    Deep,
}

/// What the ways out of one pin's access points on one routable layer are
/// judged against.
pub(super) struct WayCheck<'a, J> {
    pub(super) db: &'a NetlistDB,
    pub(super) tech: &'a RouteTech,
    pub(super) layer: LayerId,
    pub(super) kind: PinKind,
    /// The pin's metal on `layer`.
    pub(super) pin_metal: &'a [Rect<Dbu>],
    /// The box of the pin's cell; none for an I/O pin.
    pub(super) cell_box: Option<Rect<Dbu>>,
    /// Whether metal on a layer breaks no rule against the fixed shapes.
    pub(super) clean: &'a J,
}

impl<J: Fn(LayerId, &[Rect<Dbu>]) -> bool> WayCheck<'_, J> {
    /// Sets the ways out of each of `candidates`, keyed for ordering, as
    /// `FlexPA` judges access points: their planar ways, then, unless the
    /// pin is an I/O pin and one of them has a planar way, their via ways
    /// within the pin's cell and, when none can be used, through any via.
    /// Returns whether some candidate can be used; when none can, every one
    /// is left with no way out.
    pub(super) fn judge<K>(&self, candidates: &mut [(K, AccessPoint)]) -> bool {
        for (_, a) in candidates.iter_mut() {
            a.ways = self.planar_ways(a);
        }
        if self.kind == PinKind::Io && candidates.iter().any(|(_, a)| !a.ways.is_empty()) {
            return true;
        }
        for search in [ViaSearch::WithinCell, ViaSearch::Deep] {
            for (_, a) in candidates.iter_mut() {
                a.ways = a.ways.union(self.via_ways(a, search));
            }
            if candidates.iter().any(|(_, a)| self.usable(a)) {
                return true;
            }
        }
        false
    }

    /// Whether `a` can be used: a standard cell's pin on the lowest metal
    /// layer needs the via up, any other pin some way.
    // NOTE: FlexPA tests the layer against VIA_ACCESS_LAYERNUM (2), its
    // number for the first metal layer; PARE takes the lowest routing layer.
    pub(super) fn usable(&self, a: &AccessPoint) -> bool {
        if self.via_only() {
            a.ways.contains(Way::Up)
        } else {
            !a.ways.is_empty()
        }
    }

    /// Whether only vias may leave the pin: a standard cell's pin on the
    /// lowest metal layer, which `FlexPA::createMultipleAccessPoints` gives
    /// no planar access.
    fn via_only(&self) -> bool {
        self.kind == PinKind::Core && self.layer == LayerId(0)
    }

    /// The planar ways out of `a`'s node whose wire is clean, as
    /// `FlexPA::filterPlanarAccess` judges each: a wire from the node, cut
    /// off there, to past the pin, together with `a`'s stub.
    // NOTE: createSingleAccessPoint offers no wrong-way access on a
    // unidirectional layer (several masks, RECTONLY); PARE's search jogs on
    // every layer, so every planar way is judged.
    fn planar_ways(&self, a: &AccessPoint) -> Ways {
        if self.via_only() {
            return Ways::NONE;
        }
        let begin = self.node_point(a);
        let stub = self.stub(a);
        Way::PLANAR
            .into_iter()
            .filter(|&way| {
                let end = self.end_point(begin, self.layer, way);
                !self.pin_metal.iter().any(|r| r.contains(end))
                    && self.segment(begin, end, self.layer).is_some_and(|wire| {
                        let mut metal = vec![wire];
                        metal.extend(stub);
                        (self.clean)(self.layer, &metal)
                    })
            })
            .fold(Ways::NONE, Ways::with)
    }

    /// The via ways out of `a`'s node, as `FlexPA::filterViaAccess` judges
    /// them: up for every pin, down for an I/O pin only. A via counts when
    /// its metal stays inside the pin's cell, unless the search is deep,
    /// and its metal with `a`'s stub is clean, together with a wire on the
    /// via's far layer in some planar way.
    // NOTE: FlexPA tries two vias of the cut and judges the cut layer too;
    // PARE's search takes one via a cut, and the judge sees routing layers.
    fn via_ways(&self, a: &AccessPoint, search: ViaSearch) -> Ways {
        let up = (self.layer < self.tech.max_layer)
            .then(|| (Way::Up, self.layer, LayerId::new(self.layer.index() + 1)));
        let down = (self.kind == PinKind::Io && self.layer > self.tech.min_layer)
            .then(|| (Way::Down, LayerId::new(self.layer.index() - 1), self.layer));
        [up, down]
            .into_iter()
            .flatten()
            .filter(|&(way, lower, upper)| self.via_clean(a, way, lower, upper, search))
            .fold(Ways::NONE, |ways, (way, _, _)| ways.with(way))
    }

    /// Whether the via between `lower` and `upper` at `a`'s node, leaving
    /// it by `way`, is usable.
    fn via_clean(
        &self,
        a: &AccessPoint,
        way: Way,
        lower: LayerId,
        upper: LayerId,
        search: ViaSearch,
    ) -> bool {
        let at = self.node_point(a);
        let via = self.tech.via_above(lower);
        let (bottom, top) = (via.bottom.translate(at), via.top.translate(at));
        if search == ViaSearch::WithinCell
            && let Some(cell) = self.cell_box
            && !(cell.encloses(&bottom) && cell.encloses(&top))
        {
            return false;
        }
        let (own_pad, far_pad, far) = if way == Way::Up {
            (bottom, top, upper)
        } else {
            (top, bottom, lower)
        };
        let mut own = vec![own_pad];
        own.extend(self.stub(a));
        (self.clean)(self.layer, &own)
            && Way::PLANAR.into_iter().any(|planar| {
                let end = self.end_point(at, far, planar);
                self.segment(at, end, far)
                    .is_some_and(|wire| (self.clean)(far, &[far_pad, wire]))
            })
    }

    /// The point a planar wire from `begin` on `layer` by `way` is judged
    /// to: a pitch past a block's pin, as `FlexPA::genEndPoint` takes for a
    /// macro, else three wire widths on.
    // NOTE: FlexPA takes the first polygon of a block's pin on the layer and
    // warns when there are more; PARE takes the box of all of them.
    fn end_point(&self, begin: Point<Dbu>, layer: LayerId, way: Way) -> Point<Dbu> {
        let lt = self.tech.layer(layer);
        let horizontal = (lt.direction == Direction::Horizontal) != way.is_across();
        let along_x = |x: Dbu| Point::new(x, begin.y);
        let along_y = |y: Dbu| Point::new(begin.x, y);
        let pin_box = self.pin_metal.iter().copied().reduce(|a, b| a.union(&b));
        if self.kind == PinKind::Block
            && let Some(pin) = pin_box
        {
            let pitch = self.pitch(layer);
            return match (horizontal, way.is_lower()) {
                (true, true) => along_x(pin.min.x - pitch),
                (true, false) => along_x(pin.max.x + pitch),
                (false, true) => along_y(pin.min.y - pitch),
                (false, false) => along_y(pin.max.y + pitch),
            };
        }
        let sign = if way.is_lower() { -1 } else { 1 };
        let step = lt.width * STEP_WIDTHS * sign;
        if horizontal {
            along_x(begin.x + step)
        } else {
            along_y(begin.y + step)
        }
    }

    /// The wire from `begin` to `end` on `layer`, cut off at `begin` and
    /// extended past `end` by half its width, as `FlexPA` lays the segment
    /// it judges a way by.
    fn segment(&self, begin: Point<Dbu>, end: Point<Dbu>, layer: LayerId) -> Option<Rect<Dbu>> {
        let width = self.tech.layer(layer).width;
        wire_rect_with_ends(begin, end, width, Dbu::ZERO, width / 2)
    }

    /// The metal of `a`'s stub, joining its node to where it touches the
    /// pin, as routing lays it; none when it touches the pin at the node.
    fn stub(&self, a: &AccessPoint) -> Option<Rect<Dbu>> {
        let node = self.node_point(a);
        (node != a.at)
            .then(|| wire_rect(node, a.at, self.tech.layer(self.layer).width))
            .flatten()
    }

    fn node_point(&self, a: &AccessPoint) -> Point<Dbu> {
        self.tech
            .layer(a.node.layer)
            .point(a.node.track, a.node.cross)
    }

    /// The pitch `FlexPA` steps past a block's pin by.
    // NOTE: frLayer::getPitch gives dbTechLayer::getPitch, the LEF's x pitch
    // whatever the layer's direction; PARE keeps that.
    fn pitch(&self, layer: LayerId) -> Dbu {
        self.db.library.routing_layers[layer].pitch_x
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ways_round_trip_through_their_bits() {
        let ways = Ways::NONE.with(Way::AlongLower).with(Way::Up);

        assert_eq!(Ways::from_bits(ways.bits()), Some(ways));
        assert!(ways.contains(Way::Up) && !ways.contains(Way::Down));
    }

    #[test]
    fn bits_beyond_the_six_ways_are_refused() {
        assert_eq!(Ways::from_bits(64), None);
    }
}
