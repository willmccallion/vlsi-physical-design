//! Which owner uses which stretch of which track.
//!
//! Every shape on a routable layer is recorded as an interval along each
//! track whose wire it could short or come too close to: tracks whose
//! centerline lies within the shape's extent widened by the spacing plus
//! half a wire width. Each interval carries its shape's width and whole
//! extent and the spacing its owner's rule demands, so that a query is
//! kept from it by what the layer's spacing table gives the two widths at
//! their actual parallel run, as the checker measures it. Two occupants
//! of a track conflict when their intervals come within that spacing,
//! unless both are intrusions from neighbouring tracks, which are checked
//! on their own tracks. Long intervals are split into chunks so a query
//! scans a short window.
//!
//! Via cuts are kept per cut layer as exact rectangles in square buckets.
//! Two cuts of different owners conflict when they touch or their edges
//! come closer than the cut layer's spacing. Shapes larger than any via cut,
//! such as cut-layer obstructions over macros, are filed separately in
//! coarser buckets so they do not widen every query.

use crate::tech::{CutTech, LayerTech, RouteTech};
use pare_common::db::indices::{CutLayerId, LayerId, NonDefaultRuleId};
use pare_common::db::library::LayerRef;
use pare_common::db::rules::SpacingTable;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::shapes::{self, OwnedShape};
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
    /// The smaller side of the shape the occupant comes from.
    pub width: Dbu,
    /// The whole shape's extent along the track, beyond this chunk.
    pub extent: (Dbu, Dbu),
    /// The shape's extent across the tracks.
    pub across: (Dbu, Dbu),
    /// The spacing the owner's rule demands, at least the layer's.
    pub hard: Dbu,
}

/// A layer's rules for how far apart two shapes side by side must stay.
#[derive(Clone, Copy, Debug)]
pub struct LayerSpacing<'a> {
    width: Dbu,
    spacing: Dbu,
    table: Option<&'a SpacingTable>,
}

impl<'a> LayerSpacing<'a> {
    /// The rules of `lt`.
    pub const fn of(lt: &'a LayerTech) -> Self {
        Self {
            width: lt.width,
            spacing: lt.spacing,
            table: lt.spacing_table.as_ref(),
        }
    }

    /// The spacing `query` must keep from occupant `o`: the layer's table
    /// at the two widths and their parallel run, or the layer's spacing,
    /// and never less than either owner's demand.
    pub fn required(&self, o: &Occupant, query: Query) -> Dbu {
        let run = query.hi.min(o.extent.1) - query.lo.max(o.extent.0);
        self.required_at(o, query, run)
    }

    /// [`Self::required`] for a parallel run of `run`.
    fn required_at(&self, o: &Occupant, query: Query, run: Dbu) -> Dbu {
        let table = self.table.map_or(self.spacing, |t| {
            t.spacing(o.width.max(self.width), query.width.max(self.width), run)
        });
        table.max(self.spacing).max(query.spacing).max(o.hard)
    }

    /// How near `query` comes to `o`, measured along the track and across
    /// it separately, or `None` when it keeps clear even of a run along the
    /// occupant's whole length.
    pub fn proximity(&self, o: &Occupant, query: Query) -> Option<Proximity> {
        let across = (query.across.0 - o.across.1).max(o.across.0 - query.across.1);
        let near = |s: Dbu| across < s && o.lo < query.hi + s && query.lo < o.hi + s;
        if near(self.required(o, query)) {
            return Some(Proximity::Within);
        }
        near(self.required_at(o, query, o.extent.1 - o.extent.0)).then_some(Proximity::Alongside)
    }
}

/// How near a query comes to an occupant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Proximity {
    /// Within the spacing their widths and actual parallel run require: a
    /// violation if both are made.
    Within,
    /// Clear by that measure, but within the spacing a wire running the
    /// occupant's whole length would need: a wire step that may become a
    /// violation as it goes on.
    Alongside,
}

/// An occupant a query comes near, and how near.
#[derive(Clone, Copy, Debug)]
pub struct Conflict<'a> {
    /// The track the occupant was met on.
    pub track: TrackRef,
    /// The occupant.
    pub occupant: &'a Occupant,
    /// How near the query comes to it.
    pub proximity: Proximity,
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
    #[cfg(test)]
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
pub(crate) fn too_close(spacing: Dbu, a: &Rect<Dbu>, b: &Rect<Dbu>) -> bool {
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
    /// Each layer's wire width.
    widths: Vec<Dbu>,
    /// Each layer's minimum spacing.
    min_spacing: Vec<Dbu>,
    /// Each layer's spacing table.
    tables: Vec<Option<SpacingTable>>,
    /// The largest spacing any occupant of each layer can carry.
    reach: Vec<Dbu>,
    /// The longest an occupant added after the fixed metal may be.
    chunk: Dbu,
    cuts: Vec<CutLayer>,
}

/// One track's occupants: the fixed metal recorded before routing, whole and
/// sorted by start with the furthest end reached so far, and what is added
/// later, wiring and reservations, in pieces no longer than a chunk sorted by
/// start, so that a query only looks back one chunk among them.
#[derive(Clone, Debug, Default)]
struct Track {
    fixed: Vec<Occupant>,
    /// For each entry of `fixed`, the largest end among it and those before.
    fixed_reach: Vec<Dbu>,
    added: Vec<Occupant>,
}

impl Track {
    /// Sorts the fixed metal and records how far it reaches.
    fn settle_fixed(&mut self) {
        self.fixed.sort_by_key(|o| (o.lo, o.hi));
        let mut furthest = Dbu(i64::MIN);
        self.fixed_reach = self
            .fixed
            .iter()
            .map(|o| {
                furthest = furthest.max(o.hi);
                furthest
            })
            .collect();
    }

    /// The occupants whose stretch meets `[lo, hi]`, fixed ones first; added
    /// ones are at most `chunk` long.
    fn meeting(&self, lo: Dbu, hi: Dbu, chunk: Dbu) -> impl Iterator<Item = &Occupant> {
        let first = self.fixed_reach.partition_point(|&end| end < lo);
        let fixed = self.fixed[first..]
            .iter()
            .take_while(move |o| o.lo <= hi)
            .filter(move |o| o.hi >= lo);
        let start = self.added.partition_point(|o| o.lo < lo - chunk);
        let added = self.added[start..]
            .iter()
            .take_while(move |o| o.lo <= hi)
            .filter(move |o| o.hi >= lo);
        fixed.chain(added)
    }

    /// Adds `occupant` among the added ones, keeping them sorted.
    fn add(&mut self, occupant: Occupant) {
        let at = self
            .added
            .partition_point(|o| (o.lo, o.hi) <= (occupant.lo, occupant.hi));
        self.added.insert(at, occupant);
    }
}

/// Tracks whose wires `rect` could touch or crowd, and whether each runs
/// through it.
/// The stretch of one track that a shape blocks.
#[derive(Clone, Copy, Debug)]
struct Blocked {
    track: u32,
    /// Whether the track runs through the shape itself.
    centered: bool,
    lo: Dbu,
    hi: Dbu,
    /// The smaller side of the shape.
    width: Dbu,
    /// The shape's extent across the tracks.
    across: (Dbu, Dbu),
    /// The spacing the owner's rule demands.
    hard: Dbu,
}

impl Blocked {
    /// The occupant for the chunk `[lo, hi]` of this stretch.
    const fn occupant(&self, lo: Dbu, hi: Dbu, owner: Owner, origin: Origin) -> Occupant {
        Occupant {
            lo,
            hi,
            owner,
            centered: self.centered,
            origin,
            width: self.width,
            extent: (self.lo, self.hi),
            across: self.across,
            hard: self.hard,
        }
    }
}

/// The stretches of `lt`'s tracks that metal covering `rect` blocks: every
/// track a wire alongside its whole length would come too close to, given
/// `spacing`, the least spacing the shape's owner demands.
fn blocked_stretches<'t>(
    lt: &'t LayerTech,
    rect: &Rect<Dbu>,
    spacing: Dbu,
) -> impl Iterator<Item = Blocked> + use<'t> {
    let (across, (lo, hi)) = lt.across_along_rect(rect);
    let reach = lt.spacing_around(rect).0.max(spacing);
    let width = narrow_side(rect);
    affected_tracks(lt, rect, reach).map(move |(track, centered)| Blocked {
        track,
        centered,
        lo,
        hi,
        width,
        across,
        hard: spacing,
    })
}

/// The smaller side of `rect`.
fn narrow_side(rect: &Rect<Dbu>) -> Dbu {
    rect.width().min(rect.height())
}

/// The non-default rule `owner` follows.
fn rule_of(tech: &RouteTech, owner: Owner) -> Option<NonDefaultRuleId> {
    match owner {
        Owner::Net(n) => tech.net_rule(n as usize),
        Owner::Obstacle => None,
    }
}

/// A shape asking which occupants of a track it would come too close to.
#[derive(Clone, Copy, Debug)]
pub struct Query {
    /// Start of its extent along the track.
    pub lo: Dbu,
    /// End of its extent along the track.
    pub hi: Dbu,
    /// Its extent across the tracks.
    pub across: (Dbu, Dbu),
    /// Its smaller side.
    pub width: Dbu,
    /// The spacing its owner's rule demands.
    pub spacing: Dbu,
}

impl Query {
    /// The query for `rect` on a layer, whose owner demands `spacing`.
    pub fn of(lt: &LayerTech, rect: &Rect<Dbu>, spacing: Dbu) -> Self {
        let (across, (lo, hi)) = lt.across_along_rect(rect);
        Self {
            lo,
            hi,
            across,
            width: narrow_side(rect),
            spacing,
        }
    }
}

/// Where fixed metal lies along each track.
pub trait FixedMetal {
    /// The stretches of fixed metal on `track` that overlap `(lo, hi)`.
    fn fixed_stretches(
        &self,
        track: TrackRef,
        lo: Dbu,
        hi: Dbu,
    ) -> impl Iterator<Item = (Dbu, Dbu)>;
}

impl FixedMetal for Occupancy {
    fn fixed_stretches(
        &self,
        track: TrackRef,
        lo: Dbu,
        hi: Dbu,
    ) -> impl Iterator<Item = (Dbu, Dbu)> {
        let l = track.layer.index();
        let min = self.min_spacing[l];
        let extra = self.reach[l] - min;
        let rules = self.rules(track.layer);
        self.meeting(track, lo - extra, hi + extra)
            .filter(|o| o.origin == Origin::Fixed)
            .map(move |o| {
                let wire = Query {
                    lo,
                    hi,
                    across: o.across,
                    width: rules.width,
                    spacing: min,
                };
                let beside = rules.required(o, wire) - min;
                (o.lo - beside, o.hi + beside)
            })
            .filter(move |&(start, end)| start < hi && lo < end)
    }
}

/// The fixed metal of every track as sorted, disjoint stretches: what
/// global routing capacity needs of it, without the pieces one gcell long
/// that [`Occupancy`] keeps for wiring.
#[derive(Clone, Debug, Default)]
pub struct FixedTracks {
    stretches: Vec<Vec<Vec<(Dbu, Dbu)>>>,
}

impl FixedTracks {
    /// The stretches `shapes` block on the layers `tech` routes, as
    /// [`Occupancy::add_fixed`] would record them.
    pub fn new(tech: &RouteTech, shapes: impl IntoIterator<Item = OwnedShape>) -> Self {
        let mut stretches: Vec<Vec<Vec<(Dbu, Dbu)>>> = tech
            .layers
            .iter()
            .map(|l| vec![Vec::new(); l.tracks.len()])
            .collect();
        for s in shapes {
            let LayerRef::Routing(layer) = s.shape.layer else {
                continue;
            };
            if !tech.routable(layer) {
                continue;
            }
            let lt = tech.layer(layer);
            let rule = match s.owner {
                shapes::Owner::Net(n) => tech.net_rule(n.index()),
                shapes::Owner::Obstacle => None,
            };
            let spacing = tech.wire_spacing(rule, layer);
            let extra = lt.spacing_around(&s.shape.rect).1.max(spacing) - lt.spacing;
            for b in blocked_stretches(lt, &s.shape.rect, spacing) {
                stretches[layer.index()][b.track as usize].push((b.lo - extra, b.hi + extra));
            }
        }
        for track in stretches.iter_mut().flatten() {
            merge_stretches(track);
        }
        Self { stretches }
    }
}

impl FixedMetal for FixedTracks {
    fn fixed_stretches(
        &self,
        track: TrackRef,
        lo: Dbu,
        hi: Dbu,
    ) -> impl Iterator<Item = (Dbu, Dbu)> {
        let list = self
            .stretches
            .get(track.layer.index())
            .and_then(|l| l.get(track.track as usize))
            .map_or(&[][..], Vec::as_slice);
        let first = list.partition_point(|&(_, end)| end <= lo);
        list[first..]
            .iter()
            .copied()
            .take_while(move |&(start, _)| start < hi)
    }
}

/// Sorts `stretches` and joins those that overlap or touch.
fn merge_stretches(stretches: &mut Vec<(Dbu, Dbu)>) {
    stretches.sort_unstable();
    let mut merged: Vec<(Dbu, Dbu)> = Vec::with_capacity(stretches.len());
    for &(lo, hi) in stretches.iter() {
        match merged.last_mut() {
            Some(last) if lo <= last.1 => last.1 = last.1.max(hi),
            _ => merged.push((lo, hi)),
        }
    }
    *stretches = merged;
}

pub(crate) fn affected_tracks<'t>(
    lt: &'t LayerTech,
    rect: &Rect<Dbu>,
    spacing: Dbu,
) -> impl Iterator<Item = (u32, bool)> + use<'t> {
    let ((a0, a1), _) = lt.across_along_rect(rect);
    let reach = spacing + lt.width / 2;
    let range = lt.tracks_within(a0 - reach + Dbu(1), a1 + reach - Dbu(1));
    range.map(move |t| {
        let c = lt.tracks[t as usize];
        (t, a0 <= c && c <= a1)
    })
}

impl Occupancy {
    /// Empty occupancy for every layer; metal added after the fixed metal is
    /// split into pieces no longer than `chunk`.
    pub fn new(tech: &RouteTech, chunk: Dbu) -> Self {
        Self {
            tracks: tech
                .layers
                .iter()
                .map(|l| vec![Track::default(); l.tracks.len()])
                .collect(),
            widths: tech.layers.iter().map(|l| l.width).collect(),
            min_spacing: tech.layers.iter().map(|l| l.spacing).collect(),
            tables: tech
                .layers
                .iter()
                .map(|l| l.spacing_table.clone())
                .collect(),
            reach: (0..tech.layers.len())
                .map(|l| {
                    let lt = &tech.layers[l];
                    let table = lt
                        .spacing_table
                        .as_ref()
                        .map_or(Dbu::ZERO, SpacingTable::max_spacing);
                    tech.max_wire_spacing(LayerId::new(l))
                        .max(table)
                        .max(lt.spacing)
                })
                .collect(),
            chunk: chunk.max(Dbu(1)),
            cuts: tech.cuts.iter().map(|&c| CutLayer::new(c)).collect(),
        }
    }

    /// Occupants of a track whose stretch meets `[lo, hi]`.
    fn meeting(&self, t: TrackRef, lo: Dbu, hi: Dbu) -> impl Iterator<Item = &Occupant> {
        self.tracks[t.layer.index()][t.track as usize].meeting(lo, hi, self.chunk)
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

    /// Records `shapes` as fixed metal and cuts, on the layers `tech` routes:
    /// each shape once, whole, on every track it blocks.
    pub fn add_fixed(&mut self, tech: &RouteTech, shapes: impl IntoIterator<Item = OwnedShape>) {
        let mut settle: Vec<TrackRef> = Vec::new();
        for s in shapes {
            let owner = match s.owner {
                shapes::Owner::Net(n) => Owner::Net(n.0),
                shapes::Owner::Obstacle => Owner::Obstacle,
            };
            match s.shape.layer {
                LayerRef::Routing(layer) if tech.routable(layer) => {
                    let spacing = tech.wire_spacing(rule_of(tech, owner), layer);
                    for b in blocked_stretches(tech.layer(layer), &s.shape.rect, spacing) {
                        let track = &mut self.tracks[layer.index()][b.track as usize];
                        if track.fixed.len() == track.fixed_reach.len() {
                            settle.push(TrackRef {
                                layer,
                                track: b.track,
                            });
                        }
                        track.fixed.push(b.occupant(b.lo, b.hi, owner, Origin::Fixed));
                    }
                }
                LayerRef::Routing(_) => {}
                LayerRef::Cut(cut) => {
                    let _ = self.add_cut(cut, &s.shape.rect, owner, Origin::Fixed);
                }
            }
        }
        for t in settle {
            self.tracks[t.layer.index()][t.track as usize].settle_fixed();
        }
    }

    /// Records `rect` on `layer` for `owner`, in pieces no longer than a
    /// chunk; returns the tracks touched.
    pub fn add_rect(
        &mut self,
        tech: &RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
        origin: Origin,
    ) -> Vec<TrackRef> {
        let mut touched = Vec::new();
        let spacing = tech.wire_spacing(rule_of(tech, owner), layer);
        for blocked in blocked_stretches(tech.layer(layer), rect, spacing) {
            let track = &mut self.tracks[layer.index()][blocked.track as usize];
            let mut start = blocked.lo;
            loop {
                let end = (start + self.chunk).min(blocked.hi);
                track.add(blocked.occupant(start, end, owner, origin));
                if end >= blocked.hi {
                    break;
                }
                start = end;
            }
            touched.push(TrackRef {
                layer,
                track: blocked.track,
            });
        }
        touched
    }

    /// Removes the wiring of `owner` from the given slots, keeping its
    /// fixed metal and cuts.
    pub fn remove_wiring(&mut self, slots: &[Slot], owner: Owner) {
        for slot in slots {
            match *slot {
                Slot::Track(t) => self.tracks[t.layer.index()][t.track as usize]
                    .added
                    .retain(|o| o.owner != owner || o.origin == Origin::Fixed),
                Slot::Cut(c) => self.cuts[c.cut.index()]
                    .retain_in(c.bucket, |o| o.owner != owner || o.origin == Origin::Fixed),
            }
        }
    }

    /// The spacing rules of `layer`.
    fn rules(&self, layer: LayerId) -> LayerSpacing<'_> {
        let l = layer.index();
        LayerSpacing {
            width: self.widths[l],
            spacing: self.min_spacing[l],
            table: self.tables[l].as_ref(),
        }
    }

    /// Occupants of other owners on a track that `query` comes near, and
    /// how near. Intrusions are skipped when the query is itself an
    /// intrusion.
    pub fn conflicts(
        &self,
        t: TrackRef,
        query: Query,
        owner: Owner,
        centered: bool,
    ) -> impl Iterator<Item = Conflict<'_>> + use<'_> {
        let rules = self.rules(t.layer);
        let reach = query.spacing.max(self.reach[t.layer.index()]);
        self.meeting(t, query.lo - reach, query.hi + reach)
            .filter(move |o| o.owner != owner && (centered || o.centered))
            .filter_map(move |o| {
                rules.proximity(o, query).map(|proximity| Conflict {
                    track: t,
                    occupant: o,
                    proximity,
                })
            })
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
        lt.tracks_within(a0, a1).flat_map(move |track| {
            self.meeting(TrackRef { layer, track }, lo, hi)
                .filter(|o| o.centered && o.origin == Origin::Wiring)
                .map(|o| o.owner)
        })
    }

    /// Owners with metal of any origin running along a track inside `rect`
    /// on `layer`: nets whose wiring or reserved pin access lies there.
    pub fn owners_in<'a>(
        &'a self,
        tech: &'a RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
    ) -> impl Iterator<Item = Owner> + use<'a> {
        let lt = tech.layer(layer);
        let ((a0, a1), (lo, hi)) = lt.across_along_rect(rect);
        lt.tracks_within(a0, a1).flat_map(move |track| {
            self.meeting(TrackRef { layer, track }, lo, hi)
                .filter(|o| o.centered)
                .map(|o| o.owner)
        })
    }

    /// The occupants `rect` of `owner` on `layer` comes near.
    pub fn rect_conflicts<'a>(
        &'a self,
        tech: &'a RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
    ) -> impl Iterator<Item = Conflict<'a>> + use<'a> {
        let lt = tech.layer(layer);
        let query = Query::of(lt, rect, tech.wire_spacing(rule_of(tech, owner), layer));
        affected_tracks(lt, rect, query.spacing).flat_map(move |(track, centered)| {
            self.conflicts(TrackRef { layer, track }, query, owner, centered)
        })
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
    /// Records `rect` on `layer` as wiring of `owner` routed here.
    pub fn add_rect(&mut self, tech: &RouteTech, layer: LayerId, rect: &Rect<Dbu>, owner: Owner) {
        self.record_rect(tech, layer, rect, owner, Origin::Wiring);
    }

    /// Records `rect` on `layer` as wiring of `owner` that stays when the
    /// owner is routed again here: wiring of a net ripped up in this worker
    /// that lies beyond the worker's reach.
    pub fn keep_rect(&mut self, tech: &RouteTech, layer: LayerId, rect: &Rect<Dbu>, owner: Owner) {
        self.record_rect(tech, layer, rect, owner, Origin::Fixed);
    }

    fn record_rect(
        &mut self,
        tech: &RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
        origin: Origin,
    ) {
        let spacing = tech.wire_spacing(rule_of(tech, owner), layer);
        for b in blocked_stretches(tech.layer(layer), rect, spacing) {
            self.tracks
                .entry(TrackRef {
                    layer,
                    track: b.track,
                })
                .or_default()
                .push(b.occupant(b.lo, b.hi, owner, origin));
        }
    }

    /// Records a cut on `cut` as wiring of `owner` routed here.
    pub fn add_cut(&mut self, tech: &RouteTech, cut: CutLayerId, rect: &Rect<Dbu>, owner: Owner) {
        self.record_cut(tech, cut, rect, owner, Origin::Wiring);
    }

    /// Records a cut on `cut` of `owner` that stays when the owner is routed
    /// again here.
    pub fn keep_cut(&mut self, tech: &RouteTech, cut: CutLayerId, rect: &Rect<Dbu>, owner: Owner) {
        self.record_cut(tech, cut, rect, owner, Origin::Fixed);
    }

    fn record_cut(
        &mut self,
        tech: &RouteTech,
        cut: CutLayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
        origin: Origin,
    ) {
        let _ = self
            .cuts
            .entry(cut)
            .or_insert_with(|| CutLayer::new(tech.cuts[cut.index()]))
            .add(CutOccupant {
                rect: *rect,
                owner,
                origin,
            });
    }

    /// Forgets the wiring routed here for `owner`, keeping what stays.
    pub fn remove_owner(&mut self, owner: Owner) {
        for list in self.tracks.values_mut() {
            list.retain(|o| o.owner != owner || o.origin == Origin::Fixed);
        }
        for layer in self.cuts.values_mut() {
            layer.retain(|o| o.owner != owner || o.origin == Origin::Fixed);
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

    /// The occupants here that `rect` of `owner` on `layer` comes near.
    pub fn rect_conflicts<'a>(
        &'a self,
        tech: &'a RouteTech,
        layer: LayerId,
        rect: &Rect<Dbu>,
        owner: Owner,
    ) -> impl Iterator<Item = Conflict<'a>> + use<'a> {
        let lt = tech.layer(layer);
        let rules = LayerSpacing::of(lt);
        let query = Query::of(lt, rect, tech.wire_spacing(rule_of(tech, owner), layer));
        affected_tracks(lt, rect, query.spacing).flat_map(move |(track, centered)| {
            let t = TrackRef { layer, track };
            self.tracks
                .get(&t)
                .into_iter()
                .flatten()
                .filter(move |o| o.owner != owner && (centered || o.centered))
                .filter_map(move |o| {
                    rules.proximity(o, query).map(|proximity| Conflict {
                        track: t,
                        occupant: o,
                        proximity,
                    })
                })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn overlapping_and_touching_stretches_merge() {
        let mut stretches = vec![
            (Dbu(50), Dbu(60)),
            (Dbu(0), Dbu(10)),
            (Dbu(10), Dbu(20)),
            (Dbu(15), Dbu(30)),
        ];

        merge_stretches(&mut stretches);

        assert_eq!(stretches, vec![(Dbu(0), Dbu(30)), (Dbu(50), Dbu(60))]);
    }

    #[test]
    fn fixed_tracks_return_the_stretches_overlapping_a_range() {
        let fixed = FixedTracks {
            stretches: vec![vec![vec![
                (Dbu(0), Dbu(30)),
                (Dbu(50), Dbu(60)),
                (Dbu(80), Dbu(90)),
            ]]],
        };
        let track = TrackRef {
            layer: LayerId(0),
            track: 0,
        };

        let got: Vec<(Dbu, Dbu)> = fixed.fixed_stretches(track, Dbu(30), Dbu(80)).collect();

        assert_eq!(got, vec![(Dbu(50), Dbu(60))]);
    }

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
