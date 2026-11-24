//! Reserving each pin's access point, judged against the fixed shapes, before any net is routed.

use super::judge;
use super::judge::ShapeTiles;
use super::resident;
use crate::access::{AccessPoint, access_points, fellow_boxes, order_toward};
use crate::bucket::{Bucket, SpatialHash, buckets_between};
use crate::occupancy::{Occupancy, Origin, Owner, Proximity};
use crate::search::OwnVias;
use crate::shared_access::SharedAccess;
use crate::tech::RouteTech;
use crate::wiring::{access_elems, elem_cuts, elem_metal};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{CutLayerId, IdVec, LayerId, NetId, ViaId};
use pare_common::db::shapes::{fixed_shapes, is_signal_net};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_verify::drc::{Layout, Rules};
use rayon::prelude::*;

/// The fixed shapes of the design as access reservation sees them, for the
/// metal below the routable layers that the occupancy does not hold.
pub(super) struct FixedView<'f> {
    pub(super) layout: &'f Layout,
    pub(super) tiles: &'f ShapeTiles,
    /// Side of the buckets reserved pads below the routable layers are
    /// kept in.
    pub(super) bucket: Dbu,
}

/// A pad below the routable layers: its layer, metal and net.
type LowPad = (LayerId, Rect<Dbu>, NetId);

/// The pads below the routable layers that reserved access points put
/// down, by bucket.
pub(super) struct LowPads {
    pads: SpatialHash<LowPad>,
}

impl LowPads {
    /// No pads, to be kept in buckets of the positive side `bucket`.
    pub(super) fn new(bucket: Dbu) -> Self {
        Self {
            pads: SpatialHash::new(Point::new(Dbu::ZERO, Dbu::ZERO), bucket),
        }
    }

    fn keys(&self, r: &Rect<Dbu>) -> impl Iterator<Item = Point<Bucket>> + use<> {
        buckets_between(self.pads.bucket(r.min), self.pads.bucket(r.max))
    }

    pub(super) fn add(&mut self, layer: LayerId, rect: Rect<Dbu>, net: NetId) {
        for key in self.keys(&rect) {
            self.pads.insert(key, (layer, rect, net));
        }
    }

    /// Whether `rect` on `layer`, of `net`, keeps `spacing` from every pad
    /// of another net.
    pub(super) fn clear(&self, layer: LayerId, rect: &Rect<Dbu>, net: NetId, spacing: Dbu) -> bool {
        let reach = rect.expand(spacing);
        self.keys(&reach).all(|key| {
            self.pads
                .in_bucket(key)
                .iter()
                .all(|(l, r, n)| *l != layer || *n == net || !r.overlaps(&reach))
        })
    }
}

/// The occupancy of `db`'s fixed shapes, its added metal split into pieces
/// no longer than `gcell`.
pub(super) fn fixed_occupancy(db: &NetlistDB, tech: &RouteTech, gcell: Dbu) -> Occupancy {
    let start = std::time::Instant::now();
    let mut occupancy = Occupancy::new(tech, gcell);
    occupancy.add_fixed(tech, fixed_shapes(db));
    log::debug!(
        "fixed shapes recorded in {:.1}s, {} resident",
        start.elapsed().as_secs_f64(),
        resident()
    );
    occupancy
}

/// Each signal net's access points of each pin, found on its own; none
/// for a pin whose cell's kind shares its access points.
type OwnAccess = IdVec<NetId, Vec<Option<Vec<AccessPoint>>>>;

/// What fixed metal access points are judged clean against.
pub(super) struct FixedJudge<'a> {
    pub(super) layout: &'a Layout,
    pub(super) tiles: &'a ShapeTiles,
    pub(super) rules: &'a Rules,
}

/// The access points of every pin of every signal net of `db` whose
/// cell's kind does not share them, each judged against fixed metal.
pub(super) fn own_access_points(
    db: &NetlistDB,
    tech: &RouteTech,
    shared: &SharedAccess,
    judged: &FixedJudge<'_>,
) -> OwnAccess {
    (0..db.nets.len())
        .into_par_iter()
        .map(|n| {
            let net = NetId::new(n);
            if !is_signal_net(db, net) {
                return Vec::new();
            }
            let clean = |layer: LayerId, metal: &[Rect<Dbu>]| {
                judge::wiring_is_clean(judged.layout, judged.tiles, judged.rules, net, layer, metal)
            };
            db.nets[net]
                .pins
                .iter()
                .map(|&p| {
                    shared
                        .count(db, p)
                        .is_none()
                        .then(|| access_points(db, tech, p, &clean))
                })
                .collect()
        })
        .collect::<Vec<_>>()
        .into()
}

/// Chooses one access point per pin and records its metal as the net's
/// own, so other nets keep clear of it from the start; the chosen point
/// becomes the pin's only one.
///
/// Pins with the fewest candidates choose first, each taking the candidate
/// nearest the net's other pins that clashes with nothing recorded before
/// it and whose vias,
/// counting the via up from its node, leave room for those of the net's
/// pins that chose earlier. Below the routable layers, which the occupancy
/// does not hold, a candidate's pads must keep the layer's spacing from
/// the fixed shapes of other owners, those of neighbouring cells among
/// them, and from the pads other nets reserved there, as `TritonRoute`'s
/// pin access weighs a cell's access vias against its neighbours. A pin
/// with no such candidate keeps all of them, except that a pin with a
/// single one still records it.
pub(super) fn reserve_accesses(
    db: &NetlistDB,
    tech: &RouteTech,
    shared: &SharedAccess,
    own: &OwnAccess,
    occupancy: &mut Occupancy,
    fixed: &FixedView<'_>,
) -> IdVec<NetId, Vec<Vec<AccessPoint>>> {
    let fellows = fellow_boxes_of_nets(db, own);
    let candidates_of = |n: NetId, k: usize| {
        let mut candidates = match &own[n][k] {
            Some(c) => c.clone(),
            None => shared
                .candidates(db, db.nets[n].pins[k])
                .unwrap_or_default(),
        };
        if let Some(fellows) = &fellows[n][k] {
            order_toward(&mut candidates, fellows);
        }
        candidates
    };
    let mut access: IdVec<NetId, Vec<Vec<AccessPoint>>> = own
        .iter()
        .map(|pins| vec![Vec::new(); pins.len()])
        .collect();
    let mut reservations = Reservations {
        db,
        tech,
        occupancy,
        fixed,
        low_pads: LowPads::new(fixed.bucket),
        vias: own
            .ids()
            .map(|n| OwnVias::new(db, tech, tech.net_rule(n)))
            .collect(),
    };
    for (count, n, k) in reservation_order(db, shared, own) {
        access[n][k] = candidates_of(n, k);
        let clear = access[n][k]
            .iter()
            .position(|a| reservations.is_clear(n, a));
        let chosen = match clear {
            Some(c) => c,
            None if count == 1 => 0,
            None => {
                log::debug!(
                    "net {} pin {k}: none of {count} access points is clear, leaving the choice to routing",
                    db.nets[n].name
                );
                continue;
            }
        };
        log::trace!(
            "net {} pin {k}: reserved access point {chosen} of {count} at {:?}",
            db.nets[n].name,
            access[n][k][chosen].at
        );
        let picked = access[n][k].swap_remove(chosen);
        reservations.reserve(n, &picked);
        access[n][k] = vec![picked];
    }
    access
}

/// For each pin of each net with candidates, the box of the net's other
/// pins that its candidates are ordered toward.
fn fellow_boxes_of_nets(
    db: &NetlistDB,
    own: &IdVec<NetId, Vec<Option<Vec<AccessPoint>>>>,
) -> IdVec<NetId, Vec<Option<Rect<Dbu>>>> {
    own.iter_enumerated()
        .map(|(n, pins)| {
            if pins.is_empty() {
                return Vec::new();
            }
            let points: Vec<Point<Dbu>> = db.nets[n]
                .pins
                .iter()
                .map(|&p| db.pin_position(p))
                .collect();
            fellow_boxes(&points)
        })
        .collect()
}

/// The pins with candidates as `(count, net, pin)`, those with the fewest
/// candidates first.
fn reservation_order(
    db: &NetlistDB,
    shared: &SharedAccess,
    own: &IdVec<NetId, Vec<Option<Vec<AccessPoint>>>>,
) -> Vec<(usize, NetId, usize)> {
    let count_of = |n: NetId, k: usize| match &own[n][k] {
        Some(c) => c.len(),
        None => shared.count(db, db.nets[n].pins[k]).unwrap_or(0),
    };
    let mut order: Vec<(usize, NetId, usize)> = own
        .iter_enumerated()
        .flat_map(|(n, pins)| (0..pins.len()).map(move |k| (n, k)))
        .map(|(n, k)| (count_of(n, k), n, k))
        .filter(|&(count, _, _)| count > 0)
        .collect();
    order.sort_unstable();
    order
}

/// The access points reserved so far: their metal and cuts in the
/// occupancy or, below the routable layers, among the low pads, and each
/// net's vias.
struct Reservations<'r, 'f> {
    db: &'r NetlistDB,
    tech: &'r RouteTech,
    occupancy: &'r mut Occupancy,
    fixed: &'r FixedView<'f>,
    low_pads: LowPads,
    vias: IdVec<NetId, OwnVias>,
}

impl Reservations<'_, '_> {
    fn metal_of(&self, n: NetId, a: &AccessPoint) -> Vec<(LayerId, Rect<Dbu>)> {
        access_elems(self.tech, a)
            .iter()
            .flat_map(|e| elem_metal(self.db, n, e))
            .collect()
    }

    fn cuts_of(&self, a: &AccessPoint) -> Vec<(CutLayerId, Rect<Dbu>)> {
        access_elems(self.tech, a)
            .iter()
            .flat_map(|e| elem_cuts(self.db, e))
            .collect()
    }

    /// Whether net `n` can take `a`: its vias leave room for those its
    /// pins chose earlier, and its metal and cuts are clear of every
    /// reservation and, below the routable layers, of fixed shapes.
    fn is_clear(&self, n: NetId, a: &AccessPoint) -> bool {
        let (db, tech) = (self.db, self.tech);
        let owner = Owner::Net(n);
        let leaves_room = footprint(tech, a)
            .iter()
            .all(|&(via, at)| self.vias[n].crowded(db, tech, via, at) == 0);
        let metal = self.metal_of(n, a);
        let metal_clear = metal.iter().all(|(layer, rect)| {
            !tech.routable(*layer)
                || self
                    .occupancy
                    .rect_conflicts(tech, *layer, rect, owner)
                    .all(|c| c.proximity != Proximity::Within)
        });
        let low_clear = metal.iter().all(|(layer, rect)| {
            if tech.routable(*layer) {
                return true;
            }
            let spacing = tech.layer(*layer).spacing;
            self.low_pads.clear(*layer, rect, n, spacing)
                && judge::clear_of_fixed(
                    self.fixed.layout,
                    self.fixed.tiles,
                    n,
                    *layer,
                    rect,
                    spacing,
                )
        });
        leaves_room
            && metal_clear
            && low_clear
            && self.cuts_of(a).iter().all(|(cut, rect)| {
                self.occupancy
                    .cut_conflicts(*cut, rect, owner)
                    .next()
                    .is_none()
            })
    }

    /// Reserves `a` for net `n`.
    fn reserve(&mut self, n: NetId, a: &AccessPoint) {
        let owner = Owner::Net(n);
        for (layer, rect) in self.metal_of(n, a) {
            if self.tech.routable(layer) {
                self.occupancy
                    .add_rect(self.tech, layer, &rect, owner, Origin::Fixed);
            } else {
                self.low_pads.add(layer, rect, n);
            }
        }
        for (cut, rect) in self.cuts_of(a) {
            self.occupancy.add_cut(cut, &rect, owner, Origin::Fixed);
        }
        for (via, at) in footprint(self.tech, a) {
            self.vias[n].add(via, at);
        }
    }
}

/// The vias an access point commits its net to: its via stack, and the via
/// up from its node that a route leaving the pin upward would take.
fn footprint(tech: &RouteTech, a: &AccessPoint) -> Vec<(ViaId, Point<Dbu>)> {
    let node = tech.layer(a.node.layer).point(a.node.track, a.node.cross);
    let up = (a.node.layer < tech.max_layer).then(|| (tech.via_above(a.node.layer).via, node));
    a.vias.iter().map(|&v| (v, a.at)).chain(up).collect()
}
