//! Which owner uses which stretch of which track.
//!
//! Every shape on a routable layer is recorded as an interval along each
//! track whose wire it could short or come too close to: tracks whose
//! centerline lies within the shape's extent widened by the spacing plus
//! half a wire width. Two occupants of a track conflict when their
//! intervals come within the layer spacing, unless both are intrusions
//! from neighbouring tracks, which are checked on their own tracks. Long
//! intervals are split into chunks so a query scans a short window.
//!
//! Via cuts are kept per cut layer as exact rectangles in square buckets.
//! Two cuts of different owners conflict when they touch or their edges
//! come closer than the cut layer's spacing.

use crate::tech::{CutTech, LayerTech, RouteTech};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{CutLayerId, LayerId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
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

/// One via cut.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CutOccupant {
    /// The cut's extent.
    pub rect: Rect<Dbu>,
    /// Who it belongs to.
    pub owner: Owner,
    /// Whether rip-up removes it.
    pub origin: Origin,
}

/// One bucket of one cut layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CutRef {
    /// Cut layer.
    pub cut: CutLayerId,
    /// Bucket column and row.
    pub bucket: (i64, i64),
}

/// Where an owner's metal or cuts were recorded.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Slot {
    /// A track of a routing layer.
    Track(TrackRef),
    /// A bucket of a cut layer.
    Cut(CutRef),
}

/// The cuts of one cut layer.
#[derive(Clone, Debug)]
struct CutLayer {
    rules: CutTech,
    bucket: Dbu,
    buckets: FxHashMap<(i64, i64), Vec<CutOccupant>>,
}

impl CutLayer {
    fn new(rules: CutTech) -> Self {
        Self {
            rules,
            bucket: (rules.reach() * 4).max(Dbu(1)),
            buckets: FxHashMap::default(),
        }
    }

    const fn key(&self, p: Point<Dbu>) -> (i64, i64) {
        (
            p.x.get().div_euclid(self.bucket.get()),
            p.y.get().div_euclid(self.bucket.get()),
        )
    }

    fn add(&mut self, cut: CutOccupant) -> (i64, i64) {
        let half = cut.rect.width().max(cut.rect.height());
        self.rules.max_half = self.rules.max_half.max(half / 2 + half % 2);
        let key = self.key(cut.rect.center());
        self.buckets.entry(key).or_default().push(cut);
        key
    }

    /// Cuts of owners other than `owner` too close to `rect`.
    fn conflicts(&self, rect: Rect<Dbu>, owner: Owner) -> impl Iterator<Item = &CutOccupant> + '_ {
        let spacing = self.rules.spacing;
        let area = rect.expand(self.rules.reach());
        let (x0, y0) = self.key(area.min);
        let (x1, y1) = self.key(area.max);
        (x0..=x1)
            .flat_map(move |x| (y0..=y1).map(move |y| (x, y)))
            .filter_map(|k| self.buckets.get(&k))
            .flatten()
            .filter(move |c| c.owner != owner && cuts_clash(&rect, &c.rect, spacing))
    }
}

/// Whether two cuts touch or their edges are closer than `spacing`.
pub fn cuts_clash(a: &Rect<Dbu>, b: &Rect<Dbu>, spacing: Dbu) -> bool {
    let gap = |lo_a: Dbu, hi_a: Dbu, lo_b: Dbu, hi_b: Dbu| {
        i128::from((lo_a - hi_b).max(lo_b - hi_a).max(Dbu::ZERO).get())
    };
    let dx = gap(a.min.x, a.max.x, b.min.x, b.max.x);
    let dy = gap(a.min.y, a.max.y, b.min.y, b.max.y);
    let s = i128::from(spacing.get());
    a.touches(b) || dx * dx + dy * dy < s * s
}

/// Whether two shapes of one net on `layer` neither touch nor keep the
/// layer's spacing.
pub(crate) fn too_close(db: &NetlistDB, layer: LayerRef, a: &Rect<Dbu>, b: &Rect<Dbu>) -> bool {
    let spacing = match layer {
        LayerRef::Routing(l) => db.library.layer(l).min_spacing,
        LayerRef::Cut(c) => db.library.cut_layers[c.index()].spacing.unwrap_or_default(),
    };
    let d = gap_sq(a, b);
    let s = i128::from(spacing.get());
    d > 0 && d < s * s
}

/// Squared distance between two rectangles; zero when they touch.
fn gap_sq(a: &Rect<Dbu>, b: &Rect<Dbu>) -> i128 {
    let gap = |lo_a: Dbu, hi_a: Dbu, lo_b: Dbu, hi_b: Dbu| {
        i128::from((lo_a - hi_b).max(lo_b - hi_a).max(Dbu::ZERO).get())
    };
    let dx = gap(a.min.x, a.max.x, b.min.x, b.max.x);
    let dy = gap(a.min.y, a.max.y, b.min.y, b.max.y);
    dx * dx + dy * dy
}

/// Interval occupancy of every track of every routable layer, and the cuts
/// of every cut layer.
#[derive(Clone, Debug)]
pub struct Occupancy {
    /// `[layer][track]`, each sorted by `lo`.
    tracks: Vec<Vec<Vec<Occupant>>>,
    chunk: Dbu,
    cuts: Vec<CutLayer>,
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
            cuts: tech.cuts.iter().map(|&c| CutLayer::new(c)).collect(),
        }
    }

    /// Records a cut on `cut` for `owner`; returns where it was put.
    pub fn add_cut(
        &mut self,
        cut: CutLayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
        origin: Origin,
    ) -> CutRef {
        let bucket = self.cuts[cut.index()].add(CutOccupant {
            rect: *rect,
            owner,
            origin,
        });
        CutRef { cut, bucket }
    }

    /// Cuts of owners other than `owner` that a cut at `rect` on `cut` would
    /// touch or crowd.
    pub fn cut_conflicts(
        &self,
        cut: CutLayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
    ) -> impl Iterator<Item = &CutOccupant> + '_ {
        self.cuts[cut.index()].conflicts(*rect, owner)
    }

    /// Every conflicting pair of cuts, by cut layer.
    pub fn all_cut_conflicts(&self) -> Vec<(CutLayerId, CutOccupant, CutOccupant)> {
        let mut out = Vec::new();
        for (c, layer) in self.cuts.iter().enumerate() {
            let cut = CutLayerId::new(c);
            for list in layer.buckets.values() {
                for a in list {
                    for b in layer.conflicts(a.rect, a.owner) {
                        let both_fixed = a.owner == Owner::Obstacle && b.owner == Owner::Obstacle;
                        let key = |c: &CutOccupant| (c.rect.min.x, c.rect.min.y, c.owner);
                        if !both_fixed && key(a) < key(b) {
                            out.push((cut, *a, *b));
                        }
                    }
                }
            }
        }
        out
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

    /// Removes the wiring of `owner` from the given slots, keeping its
    /// fixed metal and cuts.
    pub fn remove_wiring(&mut self, slots: &[Slot], owner: Owner) {
        for slot in slots {
            match *slot {
                Slot::Track(t) => self.tracks[t.layer.index()][t.track as usize]
                    .retain(|o| o.owner != owner || o.origin == Origin::Fixed),
                Slot::Cut(c) => {
                    if let Some(list) = self.cuts[c.cut.index()].buckets.get_mut(&c.bucket) {
                        list.retain(|o| o.owner != owner || o.origin == Origin::Fixed);
                    }
                }
            }
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
    cuts: FxHashMap<CutLayerId, Vec<CutOccupant>>,
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

    /// Records a cut on `cut` for `owner`.
    pub fn add_cut(&mut self, cut: CutLayerId, rect: &Rect<Dbu>, owner: Owner) {
        self.cuts.entry(cut).or_default().push(CutOccupant {
            rect: *rect,
            owner,
            origin: Origin::Wiring,
        });
    }

    /// Forgets every rectangle and cut of `owner`.
    pub fn remove_owner(&mut self, owner: Owner) {
        for list in self.tracks.values_mut() {
            list.retain(|o| o.owner != owner);
        }
        for list in self.cuts.values_mut() {
            list.retain(|o| o.owner != owner);
        }
    }

    /// Cuts here that a cut at `rect` on `cut` of `owner` would touch or crowd.
    pub fn cut_conflicts<'a>(
        &'a self,
        tech: &'a RouteTech,
        cut: CutLayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
    ) -> impl Iterator<Item = &'a CutOccupant> + use<'a> {
        let spacing = tech.cuts[cut.index()].spacing;
        let rect = *rect;
        self.cuts
            .get(&cut)
            .into_iter()
            .flatten()
            .filter(move |c| c.owner != owner && cuts_clash(&rect, &c.rect, spacing))
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
