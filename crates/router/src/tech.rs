//! The routing view of the technology: tracks per layer, where vias can go,
//! and which via to use through each cut.
//!
//! Wires run only in each layer's preferred direction, centered on its
//! tracks. A routing node on layer `L` is a point on one of `L`'s tracks
//! where a track of the layer below or above crosses it, so every via
//! lands on a track of both layers it joins.

use crate::RouteError;
use crate::global::Gcells;
use pare_common::config::RoutingConfig;
use pare_common::db::design::NetlistDB;
use pare_common::db::design::NonDefaultRule;
use pare_common::db::indices::{CutLayerId, IdVec, LayerId, NetId, NonDefaultRuleId, ViaId};
use pare_common::db::library::{Direction, LayerRef, Library, Shape};
use pare_common::db::rules::SpacingTable;
use pare_common::geom::dbu::{Dbu, DbuArea};
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// A position along a layer's tracks where a via may go.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cross {
    /// Coordinate along the track.
    pub at: Dbu,
    /// Index of this coordinate among the layer-below's tracks, if it is one.
    pub down: Option<u32>,
    /// Index of this coordinate among the layer-above's tracks, if it is one.
    pub up: Option<u32>,
}

/// Tracks and via positions of one routing layer.
#[derive(Clone, Debug)]
pub struct LayerTech {
    /// Preferred direction.
    pub direction: Direction,
    /// Track centerlines, as the coordinate across the wire, increasing.
    pub tracks: Vec<Dbu>,
    /// Via positions along every track, increasing.
    pub cross: Vec<Cross>,
    /// For each track, its index among the layer-below's crossings.
    pub track_down: Vec<Option<u32>>,
    /// For each track, its index among the layer-above's crossings.
    pub track_up: Vec<Option<u32>>,
    /// Wire width.
    pub width: Dbu,
    /// Minimum spacing to other nets.
    pub spacing: Dbu,
    /// Spacing by width and parallel run, if the layer has a table.
    pub spacing_table: Option<SpacingTable>,
}

impl LayerTech {
    /// The point at a track and crossing.
    pub fn point(&self, track: u32, cross: u32) -> Point<Dbu> {
        let (t, c) = (self.tracks[track as usize], self.cross[cross as usize].at);
        match self.direction {
            Direction::Horizontal => Point::new(c, t),
            Direction::Vertical => Point::new(t, c),
        }
    }

    /// Splits a point into (across, along) coordinates for this layer.
    pub(crate) const fn across_along(&self, p: Point<Dbu>) -> (Dbu, Dbu) {
        match self.direction {
            Direction::Horizontal => (p.y, p.x),
            Direction::Vertical => (p.x, p.y),
        }
    }

    /// Splits a rectangle into its (across, along) coordinate ranges.
    pub const fn across_along_rect(&self, r: &Rect<Dbu>) -> ((Dbu, Dbu), (Dbu, Dbu)) {
        match self.direction {
            Direction::Horizontal => ((r.min.y, r.max.y), (r.min.x, r.max.x)),
            Direction::Vertical => ((r.min.x, r.max.x), (r.min.y, r.max.y)),
        }
    }

    /// The spacing other nets' wires on this layer must keep from `rect`:
    /// across its tracks, the spacing for its width and a run alongside its
    /// whole length, as if a wire could follow it all the way; along them,
    /// the spacing for its width at no run.
    pub fn spacing_around(&self, rect: &Rect<Dbu>) -> (Dbu, Dbu) {
        let Some(table) = &self.spacing_table else {
            return (self.spacing, self.spacing);
        };
        let ((a0, a1), (b0, b1)) = self.across_along_rect(rect);
        let width = (a1 - a0).min(b1 - b0).max(self.width);
        let across = table.spacing(width, self.width, b1 - b0).max(self.spacing);
        let along = table
            .spacing(width, self.width, Dbu::ZERO)
            .max(self.spacing);
        (across, along)
    }

    /// The smallest distance between neighbouring tracks.
    pub fn pitch(&self) -> Dbu {
        self.tracks
            .windows(2)
            .map(|w| w[1] - w[0])
            .min()
            .unwrap_or_default()
    }

    /// Indices of tracks whose centerline lies in `[lo, hi]`.
    pub fn tracks_within(&self, lo: Dbu, hi: Dbu) -> std::ops::Range<u32> {
        let a = self.tracks.partition_point(|&t| t < lo);
        let b = self.tracks.partition_point(|&t| t <= hi);
        index_u32(a)..index_u32(b)
    }

    /// Indices of crossings in `[lo, hi]`.
    pub fn cross_within(&self, lo: Dbu, hi: Dbu) -> std::ops::Range<u32> {
        let a = self.cross.partition_point(|c| c.at < lo);
        let b = self.cross.partition_point(|c| c.at <= hi);
        index_u32(a)..index_u32(b)
    }
}

/// A via chosen for one cut, with its metal relative to its center.
#[derive(Clone, Debug)]
pub struct ViaTech {
    /// The library's definition of the via.
    pub via: ViaId,
    /// Metal on the layer below.
    pub bottom: Rect<Dbu>,
    /// Metal on the layer above.
    pub top: Rect<Dbu>,
    /// Its cut rectangles, relative to its center.
    pub cuts: Vec<Rect<Dbu>>,
    /// Center distance below which two of these cuts break the cut spacing.
    pub cut_reach: Dbu,
}

/// Spacing rules of one cut layer.
#[derive(Clone, Copy, Debug)]
pub struct CutTech {
    /// Minimum distance between cuts of different owners, edge to edge.
    pub spacing: Dbu,
    /// Half the largest side of any cut on the layer.
    pub max_half: Dbu,
}

impl CutTech {
    /// How far beyond a cut another cut's center can be and still be too
    /// close to it.
    pub const fn reach(&self) -> Dbu {
        Dbu(self.spacing.0 + self.max_half.0)
    }
}

/// The wire width and spacing a non-default rule gives each routing layer.
#[derive(Clone, Debug)]
pub(crate) struct RuleTech {
    /// Wire width per layer: the rule's, or the layer's where it is silent.
    width: IdVec<LayerId, Dbu>,
    /// Spacing other metal must keep per layer: the rule's where it sets one
    /// and that is larger, else the layer's.
    spacing: IdVec<LayerId, Dbu>,
}

impl RuleTech {
    fn new(db: &NetlistDB, rule: &NonDefaultRule) -> Self {
        let layers = &db.library.routing_layers;
        let per_layer = |l: LayerId| rule.layers.iter().find(|x| x.layer == l);
        Self {
            width: layers
                .iter_enumerated()
                .map(|(l, layer)| per_layer(l).map_or(layer.width, |x| x.width))
                .collect(),
            spacing: layers
                .iter_enumerated()
                .map(|(l, layer)| {
                    let own = per_layer(l).and_then(|x| x.spacing).unwrap_or(Dbu::ZERO);
                    own.max(layer.min_spacing)
                })
                .collect(),
        }
    }
}

/// Everything the router needs to know about the technology.
#[derive(Clone, Debug)]
pub struct RouteTech {
    /// Lowest layer signal wires may use.
    pub min_layer: LayerId,
    /// Highest layer signal wires may use.
    pub max_layer: LayerId,
    /// Every routing layer.
    pub layers: IdVec<LayerId, LayerTech>,
    /// The via on each cut.
    pub vias: IdVec<CutLayerId, ViaTech>,
    /// Every via on each cut, least overhang on the layer above first, for
    /// reaching pins below the routable layers.
    pub access_vias: IdVec<CutLayerId, Vec<ViaTech>>,
    /// Spacing rules of every cut layer.
    pub cuts: IdVec<CutLayerId, CutTech>,
    /// Every non-default rule's widths and spacings.
    rules: IdVec<NonDefaultRuleId, RuleTech>,
    /// The non-default rule each net follows.
    net_rules: IdVec<NetId, Option<NonDefaultRuleId>>,
    /// The track spacing a gcell's edge is a multiple of.
    pub gcell_pitch: Dbu,
    /// The step every wire end must fall on, one without a stated grid.
    pub manufacturing_grid: Dbu,
}

/// The non-default rule each net of `db` follows, indexed by net.
fn net_rules(db: &NetlistDB) -> IdVec<NetId, Option<NonDefaultRuleId>> {
    db.nets.iter().map(|n| n.non_default_rule).collect()
}

/// Routing layers whose track spacings choose the gcell pitch: the second
/// to the fourth, counted from one.
const GCELL_PITCH_LAYERS: usize = 4;

/// The track spacing gcells are sized by, as `OpenROAD` sizes them
/// (`getGCellTileSize` of `odb::dbBlock`): the middle of the second, third
/// and fourth routing layers' average track spacings, or the highest
/// routing layer's when that is below the fourth, each rounded down to an
/// even number. `spacing` gives a layer's spacing by index from zero.
fn gcell_pitch(spacing: impl Fn(LayerId) -> Dbu, max_layer: LayerId) -> Dbu {
    let even = |s: Dbu| if s.get() % 2 == 0 { s } else { s - Dbu(1) };
    if max_layer.index() + 1 < GCELL_PITCH_LAYERS {
        return even(spacing(max_layer));
    }
    let mut middle = [LayerId(1), LayerId(2), LayerId(3)].map(|l| even(spacing(l)));
    middle.sort_unstable();
    middle[1]
}

/// How far from the origin, in database units, the die may reach: half the
/// 32-bit range the occupancy stores, leaving room for shapes and spacing
/// beyond the die's edges.
pub(crate) const MAX_DIE_REACH: Dbu = Dbu(1 << 30);

/// How far the farthest corner of `die` lies from the origin along either
/// axis.
fn reach_from_origin(die: &Rect<Dbu>) -> Dbu {
    [die.min.x, die.min.y, die.max.x, die.max.y]
        .iter()
        .map(|c| c.abs())
        .max()
        .unwrap_or_default()
}

/// `i` as the 32-bit index the router stores. Every such index counts
/// things 32 bits hold: tracks and crossings, whose coordinates lie within
/// [`MAX_DIE_REACH`] of the origin, the design's nets and shapes, and search
/// nodes of grids [`check_grid_span`] keeps within 16 bits a side.
pub(crate) fn index_u32(i: usize) -> u32 {
    debug_assert!(u32::try_from(i).is_ok(), "router index {i} past 32 bits");
    i as u32
}

/// The most gcells the global grid spans across, as its 16-bit coordinates
/// hold.
pub(crate) const MAX_GRID_SPAN: Gcells = Gcells(u16::MAX as i64);

/// Fails when `die` spans more than [`MAX_GRID_SPAN`] gcells of `gcell`.
fn check_grid_span(die: &Rect<Dbu>, gcell: Dbu) -> Result<(), RouteError> {
    let cells = Gcells::covering(die.width().max(die.height()), gcell);
    if cells > MAX_GRID_SPAN {
        return Err(RouteError::GridTooLarge(cells));
    }
    Ok(())
}

impl RouteTech {
    /// Builds the routing technology of `db`.
    pub fn new(db: &NetlistDB, config: &RoutingConfig) -> Result<Self, RouteError> {
        let reach = reach_from_origin(&db.die_area);
        if reach > MAX_DIE_REACH {
            return Err(RouteError::DieTooLarge(reach));
        }
        let lib = &db.library;
        let range = config.layer_range(lib)?;
        let (min_layer, max_layer) = (*range.start(), *range.end());

        let tracks: IdVec<LayerId, Vec<Dbu>> = lib
            .routing_layers
            .ids()
            .map(|l| db.layer_tracks(l))
            .collect();
        if let Some(l) = min_layer.through(max_layer).find(|&l| tracks[l].is_empty()) {
            return Err(RouteError::NoTracks(lib.routing_layers[l].name.clone()));
        }

        let gcell_pitch = gcell_pitch(|l| db.average_track_spacing(l), max_layer);
        check_grid_span(
            &db.die_area,
            gcell_pitch * i64::from(config.gcell_tracks.get()),
        )?;
        let layers: IdVec<LayerId, LayerTech> =
            tracks.ids().map(|l| layer_tech(lib, &tracks, l)).collect();

        let cuts = lib.cut_layers.ids();
        let vias = cuts
            .clone()
            .map(|cut| {
                ranked_vias(db, cut, &layers, Overhang::BothLayers)
                    .into_iter()
                    .next()
                    .ok_or_else(|| RouteError::NoVia(lib.cut_layers[cut].name.clone()))
            })
            .collect::<Result<Vec<_>, _>>()?
            .into();
        let access_vias = cuts
            .clone()
            .map(|cut| ranked_vias(db, cut, &layers, Overhang::UpperOnly))
            .collect();
        let cuts = cuts.map(|cut| cut_tech(db, cut)).collect();
        Ok(Self {
            min_layer,
            max_layer,
            layers,
            vias,
            access_vias,
            cuts,
            rules: db
                .non_default_rules
                .iter()
                .map(|r| RuleTech::new(db, r))
                .collect(),
            net_rules: net_rules(db),
            gcell_pitch,
            manufacturing_grid: lib
                .rules
                .manufacturing_grid
                .filter(|g| g.get() > 0)
                .unwrap_or(Dbu(1)),
        })
    }

    /// Takes each net's rule from `db` again, after edits renumbered its
    /// nets or added some.
    pub(crate) fn follow_net_rules(&mut self, db: &NetlistDB) {
        self.net_rules = net_rules(db);
    }

    /// The non-default rule `net` follows, if any; a net added since the
    /// rules were last read follows none.
    pub fn net_rule(&self, net: NetId) -> Option<NonDefaultRuleId> {
        self.net_rules.get(net).copied().flatten()
    }

    /// The width of wires on `layer` under `rule`.
    pub fn wire_width(&self, rule: Option<NonDefaultRuleId>, layer: LayerId) -> Dbu {
        rule.and_then(|r| self.rules.get(r))
            .map_or_else(|| self.layer(layer).width, |r| r.width[layer])
    }

    /// The spacing other metal must keep from wires on `layer` under `rule`.
    pub fn wire_spacing(&self, rule: Option<NonDefaultRuleId>, layer: LayerId) -> Dbu {
        rule.and_then(|r| self.rules.get(r))
            .map_or_else(|| self.layer(layer).spacing, |r| r.spacing[layer])
    }

    /// The largest spacing any rule asks of wires on `layer`.
    pub(crate) fn max_wire_spacing(&self, layer: LayerId) -> Dbu {
        self.rules
            .iter()
            .map(|r| r.spacing[layer])
            .fold(self.layer(layer).spacing, Dbu::max)
    }

    /// The layer data of `layer`.
    pub fn layer(&self, layer: LayerId) -> &LayerTech {
        &self.layers[layer]
    }

    /// The via between `layer` and the layer above it.
    pub(crate) fn via_above(&self, layer: LayerId) -> &ViaTech {
        &self.vias[layer.cut_above()]
    }

    /// Every layer signal wires may use, from the lowest up.
    pub fn routable_layers(&self) -> impl Iterator<Item = LayerId> + use<> {
        self.min_layer.through(self.max_layer)
    }

    /// Whether `layer` may carry signal wires.
    pub fn routable(&self, layer: LayerId) -> bool {
        self.min_layer <= layer && layer <= self.max_layer
    }
}

/// Sorted, deduplicated union of two coordinate lists.
fn merge_cross(a: Option<&[Dbu]>, b: Option<&[Dbu]>) -> Vec<Dbu> {
    let mut all: Vec<Dbu> = a.into_iter().chain(b).flatten().copied().collect();
    all.sort_unstable();
    all.dedup();
    all
}

/// How far a metal rectangle reaches across a layer's tracks beyond a
/// centered wire.
fn across_overhang(rect: &Rect<Dbu>, layer: &LayerTech) -> Dbu {
    let ((lo, hi), _) = layer.across_along_rect(rect);
    let half = layer.width / 2;
    (hi - half).max(-lo - half).max(Dbu::ZERO)
}

/// The tracks and crossings of routing layer `l`, whose tracks and its
/// neighbours' are in `tracks`.
fn layer_tech(lib: &Library, tracks: &IdVec<LayerId, Vec<Dbu>>, l: LayerId) -> LayerTech {
    let tracks_of = |layer: Option<LayerId>| layer.and_then(|t| tracks.get(t)).map(Vec::as_slice);
    let below = tracks_of(l.below());
    let above = tracks_of(Some(l.above()));
    let below_cross = l
        .below()
        .map(|b| merge_cross(tracks_of(b.below()), Some(&tracks[l])));
    let above_cross =
        above.map(|_| merge_cross(Some(&tracks[l]), tracks_of(Some(l.above().above()))));
    let layer = &lib.routing_layers[l];
    LayerTech {
        direction: layer.direction,
        track_down: tracks[l]
            .iter()
            .map(|&t| track_index(below_cross.as_deref(), t))
            .collect(),
        track_up: tracks[l]
            .iter()
            .map(|&t| track_index(above_cross.as_deref(), t))
            .collect(),
        cross: merge_cross(below, above)
            .into_iter()
            .map(|at| Cross {
                at,
                down: track_index(below, at),
                up: track_index(above, at),
            })
            .collect(),
        tracks: tracks[l].clone(),
        width: layer.width,
        spacing: layer.min_spacing,
        spacing_table: layer.spacing_table.clone(),
    }
}

/// Where `at` lies in the sorted coordinates `coords`, when it is one.
fn track_index(coords: Option<&[Dbu]>, at: Dbu) -> Option<u32> {
    coords
        .and_then(|c| c.binary_search(&at).ok())
        .map(index_u32)
}

/// The largest extent, either way, of the cuts on `cut` among `shapes`;
/// zero without one.
fn largest_cut<'s>(shapes: impl Iterator<Item = &'s Shape>, cut: CutLayerId) -> Dbu {
    shapes
        .filter(|s| s.layer == LayerRef::Cut(cut))
        .map(|s| s.rect.width().max(s.rect.height()))
        .max()
        .unwrap_or_default()
}

/// The spacing of cut layer `cut` and the largest cut of any via on it.
fn cut_tech(db: &NetlistDB, cut: CutLayerId) -> CutTech {
    let lib = &db.library;
    let max_half = largest_cut(lib.vias.iter().flat_map(|v| v.shapes.iter()), cut);
    CutTech {
        spacing: lib.cut_layers[cut].spacing.unwrap_or_default(),
        max_half: Dbu(max_half.get().div_euclid(2) + max_half.get().rem_euclid(2)),
    }
}

/// Which layers' metal a via is ranked by, by how far it sticks out across
/// their tracks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Overhang {
    /// Both layers, as vias between routed wires.
    BothLayers,
    /// The upper layer alone, as vias up from a pin.
    UpperOnly,
}

/// Every LEF via through `cut`, ordered by how far its metal sticks out
/// across the tracks of the layers `overhang` names, then LEF defaults
/// first, then smaller metal.
fn ranked_vias(
    db: &NetlistDB,
    cut: CutLayerId,
    layers: &IdVec<LayerId, LayerTech>,
    overhang: Overhang,
) -> Vec<ViaTech> {
    let lib = &db.library;
    let mut ranked: Vec<((Dbu, bool, DbuArea), ViaTech)> = Vec::new();
    for (i, v) in lib.vias.iter().enumerate() {
        if v.cut != cut || v.is_from_def() {
            continue;
        }
        let metal = |layer: LayerId| {
            v.shapes
                .iter()
                .filter(|s| s.layer == LayerRef::Routing(layer))
                .map(|s| s.rect)
                .reduce(|a, b| a.union(&b))
        };
        let (Some(bottom), Some(top)) = (metal(v.bottom()), metal(v.top())) else {
            continue;
        };
        let top_overhang = across_overhang(&top, &layers[v.top()]);
        let reach = match overhang {
            Overhang::BothLayers => top_overhang.max(across_overhang(&bottom, &layers[v.bottom()])),
            Overhang::UpperOnly => top_overhang,
        };
        let key = (reach, !v.is_default(), bottom.area() + top.area());
        let cut_size = largest_cut(v.shapes.iter(), cut);
        let spacing = lib.cut_layers[cut].spacing.unwrap_or_default();
        let cuts = v
            .shapes
            .iter()
            .filter(|s| s.layer == LayerRef::Cut(cut))
            .map(|s| s.rect)
            .collect();
        ranked.push((
            key,
            ViaTech {
                via: ViaId::new(i),
                bottom,
                top,
                cuts,
                cut_reach: cut_size + spacing,
            },
        ));
    }
    ranked.sort_by_key(|(key, v)| (*key, v.via));
    ranked.into_iter().map(|(_, v)| v).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn die(width: i64) -> Rect<Dbu> {
        Rect::new(Point::new(Dbu(0), Dbu(0)), Point::new(Dbu(width), Dbu(10)))
    }

    #[test]
    fn a_die_of_as_many_gcells_as_the_grid_holds_is_routable() {
        assert_eq!(check_grid_span(&die(65_535 * 100), Dbu(100)), Ok(()));
    }

    #[test]
    fn a_die_past_the_grid_s_span_is_refused_with_its_gcells() {
        assert_eq!(
            check_grid_span(&die(65_535 * 100 + 1), Dbu(100)),
            Err(RouteError::GridTooLarge(Gcells(65_536)))
        );
    }

    fn spacings(of: &[i64]) -> impl Fn(LayerId) -> Dbu + '_ {
        let of: IdVec<LayerId, i64> = of.to_vec().into();
        move |l| Dbu(of[l])
    }

    #[test]
    fn gcells_are_sized_by_the_middle_spacing_of_the_second_to_fourth_layers() {
        let pitch = gcell_pitch(spacings(&[280, 380, 280, 560, 560]), LayerId(4));

        assert_eq!(pitch, Dbu(380));
    }

    #[test]
    fn an_odd_track_spacing_is_rounded_down_to_an_even_one() {
        let pitch = gcell_pitch(spacings(&[200, 401, 401, 401]), LayerId(3));

        assert_eq!(pitch, Dbu(400));
    }

    #[test]
    fn below_four_routing_layers_the_highest_layer_sizes_the_gcells() {
        let pitch = gcell_pitch(spacings(&[200, 300, 500]), LayerId(2));

        assert_eq!(pitch, Dbu(500));
    }
}
