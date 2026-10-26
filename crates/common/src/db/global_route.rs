//! Global routes as `OpenROAD`'s global router hands them on: straight
//! segments between gcell centers, and where each pin meets them.

use crate::db::design::NetlistDB;
use crate::db::indices::{LayerId, NetId, PinId};
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;

/// A gcell center on a routing layer, where the segments of a route meet,
/// as `OpenROAD`'s `RoutePt`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RoutePoint {
    /// The gcell center.
    pub at: Point<Dbu>,
    /// The routing layer.
    pub layer: LayerId,
}

/// A straight piece of a global route, as `OpenROAD`'s `GSegment`: a wire
/// along one layer, or a via between layers at one point.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RouteSegment {
    /// One end.
    pub from: RoutePoint,
    /// The other end.
    pub to: RoutePoint,
}

impl RouteSegment {
    /// Whether both ends stand at one point, as `GSegment::isVia`.
    #[must_use]
    pub fn is_via(&self) -> bool {
        self.from.at == self.to.at
    }

    /// The Manhattan distance between the ends.
    #[must_use]
    pub const fn length(&self) -> Dbu {
        Dbu((self.from.at.x.get() - self.to.at.x.get()).abs()
            + (self.from.at.y.get() - self.to.at.y.get()).abs())
    }
}

/// Where a pin meets its net's global route, as `OpenROAD`'s
/// `PinGridLocation`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PinGridLocation {
    /// The pin.
    pub pin: PinId,
    /// The pin's own position, which the wire to the route starts from.
    pub at: Point<Dbu>,
    /// The center of the gcell the route reaches the pin in.
    pub grid: Point<Dbu>,
    /// The layer the route reaches the pin on.
    pub layer: LayerId,
}

/// A net's global route and where its pins meet it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct NetRoute {
    /// The segments, in no particular order.
    pub segments: Vec<RouteSegment>,
    /// Where each pin of the net meets them.
    pub pins: Vec<PinGridLocation>,
}

/// Global routes that follow a design as repair edits it, as `OpenROAD`'s
/// incremental global routing does.
pub trait RouteSource {
    /// Brings the routes of `nets` in step with `db`: a net whose pins now
    /// meet other gcells, or that is new, is routed again.
    fn update(&mut self, db: &NetlistDB, nets: &[NetId]);

    /// The route of `net` with where its pins meet it, if it has one.
    fn route(&self, db: &NetlistDB, net: NetId) -> Option<NetRoute>;
}
