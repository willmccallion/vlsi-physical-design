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
//! come closer than the cut layer's spacing. Shapes larger than any via cut,
//! such as cut-layer obstructions over macros, are filed separately in
//! coarser buckets so they do not widen every query.

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

/// Where a cut is filed on its layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum CutBucket {
    /// A cut no larger than a via cut, in the bucket holding its center.
    Small((i64, i64)),
    /// A larger shape, in every coarse bucket from `min` to `max`.
    Large {
        /// Lowest coarse bucket column and row.
        min: (i64, i64),
        /// Highest coarse bucket column and row.
        max: (i64, i64),
    },
}

/// Where one cut was recorded.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct CutRef {
    /// Cut layer.
    pub cut: CutLayerId,
    /// Its bucket or buckets.
    pub bucket: CutBucket,
}

/// Where an owner's metal or cuts were recorded.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Slot {
    /// A track of a routing layer.
    Track(TrackRef),
    /// A bucket of a cut layer.
    Cut(CutRef),
}

/// How many fine buckets wide a coarse bucket of large shapes is.
const LARGE_BUCKET_SCALE: i64 = 64;

/// The cuts of one cut layer: via-sized cuts by the fine bucket holding
/// their center, larger shapes in every coarse bucket they touch.
#[derive(Clone, Debug)]
struct CutLayer {
    rules: CutTech,
    bucket: Dbu,
    buckets: FxHashMap<(i64, i64), Vec<CutOccupant>>,
    large: FxHashMap<(i64, i64), Vec<CutOccupant>>,
}

impl CutLayer {
    fn new(rules: CutTech) -> Self {
        Self {
            rules,
            bucket: (rules.reach() * 4).max(Dbu(1)),
            buckets: FxHashMap::default(),
            large: FxHashMap::default(),
        }
    }

    const fn key(&self, p: Point<Dbu>) -> (i64, i64) {
        (
            p.x.get().div_euclid(self.bucket.get()),
            p.y.get().div_euclid(self.bucket.get()),
        )
    }

    const fn large_key(&self, p: Point<Dbu>) -> (i64, i64) {
        let size = self.bucket.get() * LARGE_BUCKET_SCALE;
        (p.x.get().div_euclid(size), p.y.get().div_euclid(size))
    }

    /// Whether a shape fits within the largest via cut, so a query widened
    /// by the layer's reach finds it by its center.
    fn is_small(&self, rect: &Rect<Dbu>) -> bool {
        let side = rect.width().max(rect.height());
        side.get() <= self.rules.max_half.get() * 2
    }

    fn add(&mut self, cut: CutOccupant) -> CutBucket {
        if self.is_small(&cut.rect) {
            let key = self.key(cut.rect.center());
            self.buckets.entry(key).or_default().push(cut);
            return CutBucket::Small(key);
        }
        let (min, max) = (self.large_key(cut.rect.min), self.large_key(cut.rect.max));
        for key in keys_between(min, max) {
            self.large.entry(key).or_default().push(cut);
        }
        CutBucket::Large { min, max }
    }

    /// Keeps only the cuts in `bucket` that `keep` accepts.
    fn retain_in(&mut self, bucket: CutBucket, keep: impl Fn(&CutOccupant) -> bool) {
        match bucket {
            CutBucket::Small(key) => {
                if let Some(list) = self.buckets.get_mut(&key) {
                    list.retain(&keep);
                }
            }
            CutBucket::Large { min, max } => {
                for key in keys_between(min, max) {
                    if let Some(list) = self.large.get_mut(&key) {
                        list.retain(&keep);
                    }
                }
            }
        }
    }

    /// Cuts of owners other than `owner` too close to `rect`.
    fn conflicts(&self, rect: Rect<Dbu>, owner: Owner) -> impl Iterator<Item = &CutOccupant> + '_ {
        let spacing = self.rules.spacing;
        let near = rect.expand(self.rules.reach());
        let small = keys_between(self.key(near.min), self.key(near.max))
            .filter_map(|k| self.buckets.get(&k))
            .flatten();
        let around = rect.expand(spacing);
        let (q0, q1) = (self.large_key(around.min), self.large_key(around.max));
        let large = keys_between(q0, q1)
            .filter_map(move |k| self.large.get(&k).map(|list| (k, list)))
            .flat_map(move |(k, list)| {
                list.iter().filter(move |c| {
                    let first = self.large_key(c.rect.min);
                    (first.0.max(q0.0), first.1.max(q0.1)) == k
                })
            });
        small
            .chain(large)
            .filter(move |c| c.owner != owner && cuts_clash(&rect, &c.rect, spacing))
    }

    /// Every cut once: each large shape is taken from its first bucket.
    fn all(&self) -> impl Iterator<Item = &CutOccupant> + '_ {
        let large = self.large.iter().flat_map(move |(&k, list)| {
            list.iter().filter(move |c| self.large_key(c.rect.min) == k)
        });
        self.buckets.values().flatten().chain(large)
    }

    fn retain(&mut self, keep: impl Fn(&CutOccupant) -> bool) {
        for list in self.buckets.values_mut().chain(self.large.values_mut()) {
            list.retain(&keep);
        }
    }
}

/// Bucket keys from `min` to `max` inclusive, row by row.
fn keys_between(min: (i64, i64), max: (i64, i64)) -> impl Iterator<Item = (i64, i64)> {
    (min.0..=max.0).flat_map(move |x| (min.1..=max.1).map(move |y| (x, y)))
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
    /// `[layer][track]`.
    tracks: Vec<Vec<Track>>,
    /// The along-track coordinate where each layer's first bucket starts.
    origins: Vec<Dbu>,
    chunk: Dbu,
    cuts: Vec<CutLayer>,
}

/// One track's occupants, bucketed by which stretch of `chunk` along the
/// track each starts in; no occupant is longer than a chunk, so a query
/// only looks at the buckets around it.
#[derive(Clone, Debug, Default)]
struct Track {
    buckets: Vec<Vec<Occupant>>,
}

/// Tracks whose wires `rect` could touch or crowd, and whether each runs
/// through it.
pub(crate) fn affected_tracks<'t>(
    lt: &'t LayerTech,
    rect: &Rect<Dbu>,
) -> impl Iterator<Item = (u32, bool)> + use<'t> {
    let ((a0, a1), _) = lt.across_along_rect(rect);
    let reach = lt.spacing_around(rect).0 + lt.width / 2;
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
                .map(|l| vec![Track::default(); l.tracks.len()])
                .collect(),
            origins: tech
                .layers
                .iter()
                .map(|l| l.cross.first().map_or(Dbu::ZERO, |c| c.at))
                .collect(),
            chunk: chunk.max(Dbu(1)),
            cuts: tech.cuts.iter().map(|&c| CutLayer::new(c)).collect(),
        }
    }

    /// The bucket of a track of `layer` holding occupants that start at
    /// `along`; coordinates before the layer's origin share the first one.
    fn bucket(&self, layer: usize, along: Dbu) -> usize {
        usize::try_from(
            (along - self.origins[layer])
                .get()
                .div_euclid(self.chunk.get()),
        )
        .unwrap_or(0)
    }

    /// Occupants of a track that start in the buckets covering `[lo, hi]`.
    fn starting_within(&self, t: TrackRef, lo: Dbu, hi: Dbu) -> impl Iterator<Item = &Occupant> {
        let l = t.layer.index();
        let track = &self.tracks[l][t.track as usize];
        let first = self.bucket(l, lo).min(track.buckets.len());
        let last = (self.bucket(l, hi) + 1).min(track.buckets.len());
        track.buckets[first..last.max(first)].iter().flatten()
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
            for a in layer.all() {
                for b in layer.conflicts(a.rect, a.owner) {
                    let both_fixed = a.owner == Owner::Obstacle && b.owner == Owner::Obstacle;
                    let key = |c: &CutOccupant| (c.rect.min.x, c.rect.min.y, c.owner);
                    if !both_fixed && key(a) < key(b) {
                        out.push((cut, *a, *b));
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
        let wider = lt.spacing_around(rect).1 - lt.spacing;
        let (lo, hi) = (lo - wider, hi + wider);
        let mut touched = Vec::new();
        for (track, centered) in affected_tracks(lt, rect) {
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
                let b = self.bucket(layer.index(), start);
                let buckets = &mut self.tracks[layer.index()][track as usize].buckets;
                if buckets.len() <= b {
                    buckets.resize_with(b + 1, Vec::new);
                }
                buckets[b].push(occupant);
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
                Slot::Track(t) => {
                    for bucket in &mut self.tracks[t.layer.index()][t.track as usize].buckets {
                        bucket.retain(|o| o.owner != owner || o.origin == Origin::Fixed);
                    }
                }
                Slot::Cut(c) => self.cuts[c.cut.index()]
                    .retain_in(c.bucket, |o| o.owner != owner || o.origin == Origin::Fixed),
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
        self.starting_within(t, lo - spacing - self.chunk, hi + spacing)
            .filter(move |o| {
                o.owner != owner
                    && (centered || o.centered)
                    && o.lo < hi + spacing
                    && lo < o.hi + spacing
            })
    }

    /// Fixed metal and obstacles on a track that overlap `(lo, hi)`.
    pub fn fixed_within(&self, t: TrackRef, lo: Dbu, hi: Dbu) -> impl Iterator<Item = &Occupant> {
        self.starting_within(t, lo - self.chunk, hi)
            .filter(move |o| o.origin == Origin::Fixed && o.lo < hi && lo < o.hi)
    }

    /// Owners of wiring on `layer` whose metal runs along a track inside
    /// `rect`.
    pub fn wiring_owners_in<'a>(
        &'a self,
        tech: &'a RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
    ) -> impl Iterator<Item = Owner> + use<'a> {
        let lt = tech.layer(layer);
        let ((a0, a1), (lo, hi)) = lt.across_along_rect(rect);
        let chunk = self.chunk;
        lt.tracks_within(a0, a1).flat_map(move |track| {
            self.starting_within(TrackRef { layer, track }, lo - chunk, hi)
                .filter(move |o| {
                    o.centered && o.origin == Origin::Wiring && lo <= o.hi && o.lo <= hi
                })
                .map(|o| o.owner)
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
            let reach =
                usize::try_from((self.chunk + spacing).get() / self.chunk.get()).unwrap_or(0) + 1;
            for (t, track) in tracks.iter().enumerate() {
                let track_ref = TrackRef {
                    layer,
                    track: crate::tech::to_u32(t),
                };
                let clash = |a: &Occupant, b: &Occupant| {
                    a.owner != b.owner
                        && !(a.owner == Owner::Obstacle && b.owner == Owner::Obstacle)
                        && (a.centered || b.centered)
                        && b.lo < a.hi + spacing
                        && a.lo < b.hi + spacing
                };
                for (i, bucket) in track.buckets.iter().enumerate() {
                    for (k, a) in bucket.iter().enumerate() {
                        let later = bucket[k + 1..].iter().chain(
                            track.buckets[i + 1..(i + 1 + reach).min(track.buckets.len())]
                                .iter()
                                .flatten(),
                        );
                        for b in later {
                            if clash(a, b) {
                                out.push((track_ref, *a, *b));
                            }
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
    cuts: FxHashMap<CutLayerId, CutLayer>,
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
    pub fn add_cut(&mut self, tech: &RouteTech, cut: CutLayerId, rect: &Rect<Dbu>, owner: Owner) {
        let _ = self
            .cuts
            .entry(cut)
            .or_insert_with(|| CutLayer::new(tech.cuts[cut.index()]))
            .add(CutOccupant {
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
        for layer in self.cuts.values_mut() {
            layer.retain(|o| o.owner != owner);
        }
    }

    /// Cuts here that a cut at `rect` on `cut` of `owner` would touch or crowd.
    pub fn cut_conflicts<'a>(
        &'a self,
        cut: CutLayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
    ) -> impl Iterator<Item = &'a CutOccupant> + use<'a> {
        let rect = *rect;
        self.cuts
            .get(&cut)
            .into_iter()
            .flat_map(move |layer| layer.conflicts(rect, owner))
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

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::from_corners(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    /// Via cuts are 100 wide with spacing 100, so fine buckets are 800 wide
    /// and coarse ones 51200.
    fn layer() -> CutLayer {
        CutLayer::new(CutTech {
            spacing: Dbu(100),
            max_half: Dbu(50),
        })
    }

    fn cut(rect: Rect<Dbu>, owner: Owner, origin: Origin) -> CutOccupant {
        CutOccupant {
            rect,
            owner,
            origin,
        }
    }

    #[test]
    fn a_cut_beside_the_far_edge_of_a_large_obstruction_conflicts_with_it() {
        let mut cuts = layer();
        let _ = cuts.add(cut(
            rect(0, 0, 300_000, 40_000),
            Owner::Obstacle,
            Origin::Fixed,
        ));

        let near_edge = cuts
            .conflicts(rect(300_050, 100, 300_150, 200), Owner::Net(1))
            .count();
        let clear = cuts
            .conflicts(rect(300_100, 100, 300_200, 200), Owner::Net(1))
            .count();

        assert_eq!(near_edge, 1);
        assert_eq!(clear, 0);
    }

    #[test]
    fn a_large_shape_is_reported_once_however_many_buckets_a_query_spans() {
        let mut cuts = layer();
        let _ = cuts.add(cut(
            rect(0, 0, 300_000, 40_000),
            Owner::Obstacle,
            Origin::Fixed,
        ));

        let spanning = cuts
            .conflicts(rect(51_100, 39_950, 51_300, 40_050), Owner::Net(1))
            .count();

        assert_eq!(spanning, 1);
    }

    #[test]
    fn removing_wiring_keeps_fixed_cuts_in_both_tiers() {
        let mut cuts = layer();
        let wire = cuts.add(cut(rect(0, 0, 100, 100), Owner::Net(1), Origin::Wiring));
        let pad = cuts.add(cut(rect(0, 0, 100, 100), Owner::Net(1), Origin::Fixed));
        let big = cuts.add(cut(
            rect(1000, 0, 60_000, 500),
            Owner::Net(1),
            Origin::Wiring,
        ));

        for bucket in [wire, pad, big] {
            cuts.retain_in(bucket, |o| {
                o.owner != Owner::Net(1) || o.origin == Origin::Fixed
            });
        }

        assert_eq!(cuts.all().count(), 1);
        assert_eq!(cuts.all().next().map(|c| c.origin), Some(Origin::Fixed));
    }

    #[test]
    fn every_cut_is_listed_once() {
        let mut cuts = layer();
        let _ = cuts.add(cut(rect(0, 0, 100, 100), Owner::Net(1), Origin::Wiring));
        let _ = cuts.add(cut(
            rect(-60_000, -60_000, 60_000, 60_000),
            Owner::Obstacle,
            Origin::Fixed,
        ));

        assert_eq!(cuts.all().count(), 2);
    }
}
