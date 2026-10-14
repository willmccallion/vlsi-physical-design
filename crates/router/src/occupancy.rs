//! Which owner uses which stretch of which track.
//!
//! Every shape on a routable layer is recorded as an interval along each
//! track whose wire it could short or come too close to: tracks whose
//! centerline lies within the shape's extent widened by the spacing plus
//! half a wire width. Two occupants of a track conflict when their
//! intervals come within the layer spacing, unless both are intrusions
//! from neighbouring tracks, which are checked on their own tracks. Long
//! intervals are split into chunks so a query scans a short window.

use crate::tech::{LayerTech, RouteTech};
use pare_common::db::indices::LayerId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::rect::Rect;
use rustc_hash::FxHashMap;

/// What occupies a stretch of track.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Owner {
    /// A signal net, by index.
    Net(u32),
    /// Something no net may approach: an obstruction, a supply or unused
    /// pin, a special net or a blockage.
    Obstacle,
}

/// Where an occupant's metal comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Origin {
    /// Pins, obstructions and reservations, which stay for the whole run.
    Fixed,
    /// Routed metal, which rip-up removes.
    Wiring,
}

/// One interval on a track.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Occupant {
    /// Start of the metal along the track.
    pub lo: Dbu,
    /// End of the metal along the track.
    pub hi: Dbu,
    /// Who it belongs to.
    pub owner: Owner,
    /// Whether the metal covers this track's centerline, rather than
    /// reaching toward it from a neighbouring track.
    pub centered: bool,
    /// Whether rip-up removes it.
    pub origin: Origin,
}

/// A track of a layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct TrackRef {
    /// Routing layer.
    pub layer: LayerId,
    /// Track index on it.
    pub track: u32,
}

/// Interval occupancy of every track of every routable layer.
#[derive(Clone, Debug)]
pub struct Occupancy {
    /// `[layer][track]`, each sorted by `lo`.
    tracks: Vec<Vec<Vec<Occupant>>>,
    chunk: Dbu,
}

/// Tracks whose wires `rect` could touch or crowd, and whether each runs
/// through it.
pub(crate) fn affected_tracks<'t>(
    lt: &'t LayerTech,
    rect: &Rect<Dbu>,
) -> impl Iterator<Item = (u32, bool)> + use<'t> {
    let ((a0, a1), _) = lt.across_along_rect(rect);
    let reach = lt.spacing + lt.width / 2;
    let range = lt.tracks_within(a0 - reach + Dbu(1), a1 + reach - Dbu(1));
    range.map(move |t| {
        let c = lt.tracks[t as usize];
        (t, a0 <= c && c <= a1)
    })
}

impl Occupancy {
    /// Empty occupancy for every layer; intervals longer than `chunk` are split.
    pub fn new(tech: &RouteTech, chunk: Dbu) -> Self {
        Self {
            tracks: tech
                .layers
                .iter()
                .map(|l| vec![Vec::new(); l.tracks.len()])
                .collect(),
            chunk: chunk.max(Dbu(1)),
        }
    }

    /// Records `rect` on `layer` for `owner`; returns the tracks touched.
    pub fn add_rect(
        &mut self,
        tech: &RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
        origin: Origin,
    ) -> Vec<TrackRef> {
        let lt = tech.layer(layer);
        let (_, (lo, hi)) = lt.across_along_rect(rect);
        let mut touched = Vec::new();
        for (track, centered) in affected_tracks(lt, rect) {
            let list = &mut self.tracks[layer.index()][track as usize];
            let mut start = lo;
            loop {
                let end = (start + self.chunk).min(hi);
                let occupant = Occupant {
                    lo: start,
                    hi: end,
                    owner,
                    centered,
                    origin,
                };
                let at = list.partition_point(|o| o.lo < start);
                list.insert(at, occupant);
                if end >= hi {
                    break;
                }
                start = end;
            }
            touched.push(TrackRef { layer, track });
        }
        touched
    }

    /// Removes the wiring of `owner` from the given tracks, keeping its
    /// fixed metal.
    pub fn remove_wiring(&mut self, tracks: &[TrackRef], owner: Owner) {
        for t in tracks {
            self.tracks[t.layer.index()][t.track as usize]
                .retain(|o| o.owner != owner || o.origin == Origin::Fixed);
        }
    }

    /// Occupants of other owners within `spacing` of `[lo, hi]` on a track.
    /// Intrusions are skipped when the query is itself an intrusion.
    pub fn conflicts(
        &self,
        t: TrackRef,
        lo: Dbu,
        hi: Dbu,
        spacing: Dbu,
        owner: Owner,
        centered: bool,
    ) -> impl Iterator<Item = &Occupant> + use<'_> {
        let list = &self.tracks[t.layer.index()][t.track as usize];
        let from = list.partition_point(|o| o.lo < lo - spacing - self.chunk);
        let to = list.partition_point(|o| o.lo < hi + spacing);
        list[from..to].iter().filter(move |o| {
            o.owner != owner
                && (centered || o.centered)
                && o.lo < hi + spacing
                && lo < o.hi + spacing
        })
    }

    /// The conflicts `rect` on `layer` would have, as (track, occupant) pairs.
    pub fn rect_conflicts<'a>(
        &'a self,
        tech: &'a RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
    ) -> impl Iterator<Item = (TrackRef, &'a Occupant)> + use<'a> {
        let lt = tech.layer(layer);
        let (_, (lo, hi)) = lt.across_along_rect(rect);
        let spacing = lt.spacing;
        affected_tracks(lt, rect).flat_map(move |(track, centered)| {
            let t = TrackRef { layer, track };
            self.conflicts(t, lo, hi, spacing, owner, centered)
                .map(move |o| (t, o))
        })
    }

    /// Every conflicting pair of occupants, by track.
    pub fn all_conflicts(&self, tech: &RouteTech) -> Vec<(TrackRef, Occupant, Occupant)> {
        let mut out = Vec::new();
        for (l, tracks) in self.tracks.iter().enumerate() {
            let layer = LayerId::new(l);
            if !tech.routable(layer) {
                continue;
            }
            let spacing = tech.layer(layer).spacing;
            for (t, list) in tracks.iter().enumerate() {
                let track = TrackRef {
                    layer,
                    track: crate::tech::to_u32(t),
                };
                for (i, a) in list.iter().enumerate() {
                    for b in &list[i + 1..] {
                        if b.lo >= a.hi + spacing {
                            break;
                        }
                        let clash = a.owner != b.owner
                            && !(a.owner == Owner::Obstacle && b.owner == Owner::Obstacle)
                            && (a.centered || b.centered)
                            && b.lo < a.hi + spacing
                            && a.lo < b.hi + spacing;
                        if clash {
                            out.push((track, *a, *b));
                        }
                    }
                }
            }
        }
        out
    }
}

/// A small, separately owned occupancy layered over the shared one, for a
/// worker that routes several nets in sequence while others run in parallel.
#[derive(Clone, Debug, Default)]
pub struct LocalOccupancy {
    tracks: FxHashMap<TrackRef, Vec<Occupant>>,
}

impl LocalOccupancy {
    /// Records `rect` on `layer` for `owner`.
    pub fn add_rect(&mut self, tech: &RouteTech, layer: LayerId, rect: &Rect<Dbu>, owner: Owner) {
        let lt = tech.layer(layer);
        let (_, (lo, hi)) = lt.across_along_rect(rect);
        for (track, centered) in affected_tracks(lt, rect) {
            self.tracks
                .entry(TrackRef { layer, track })
                .or_default()
                .push(Occupant {
                    lo,
                    hi,
                    owner,
                    centered,
                    origin: Origin::Wiring,
                });
        }
    }

    /// Forgets every rectangle of `owner`.
    pub fn remove_owner(&mut self, owner: Owner) {
        for list in self.tracks.values_mut() {
            list.retain(|o| o.owner != owner);
        }
    }

    /// The conflicts `rect` on `layer` would have here.
    pub fn rect_conflicts<'a>(
        &'a self,
        tech: &'a RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
    ) -> impl Iterator<Item = (TrackRef, &'a Occupant)> + use<'a> {
        let lt = tech.layer(layer);
        let (_, (lo, hi)) = lt.across_along_rect(rect);
        let spacing = lt.spacing;
        affected_tracks(lt, rect).flat_map(move |(track, centered)| {
            let t = TrackRef { layer, track };
            self.tracks
                .get(&t)
                .into_iter()
                .flatten()
                .filter_map(move |o| {
                    let clash = o.owner != owner
                        && (centered || o.centered)
                        && o.lo < hi + spacing
                        && lo < o.hi + spacing;
                    clash.then_some((t, o))
                })
        })
    }
}
