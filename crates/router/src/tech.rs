//! The routing view of the technology: tracks per layer, where vias can go,
//! and which via to use through each cut.
//!
//! Wires run only in each layer's preferred direction, centered on its
//! tracks. A routing node on layer `L` is a point on one of `L`'s tracks
//! where a track of the layer below or above crosses it, so every via
//! lands on a track of both layers it joins.

use crate::RouteError;
use pare_common::db::design::{NetlistDB, TrackAxis};
use pare_common::db::indices::{CutLayerId, LayerId, ViaId};
use pare_common::db::library::{Direction, LayerRef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::util::config::RoutingConfig;

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
    pub const fn across_along(&self, p: Point<Dbu>) -> (Dbu, Dbu) {
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

    /// Indices of tracks whose centerline lies in `[lo, hi]`.
    pub fn tracks_within(&self, lo: Dbu, hi: Dbu) -> std::ops::Range<u32> {
        let a = self.tracks.partition_point(|&t| t < lo);
        let b = self.tracks.partition_point(|&t| t <= hi);
        to_u32(a)..to_u32(b)
    }

    /// Indices of crossings in `[lo, hi]`.
    pub fn cross_within(&self, lo: Dbu, hi: Dbu) -> std::ops::Range<u32> {
        let a = self.cross.partition_point(|c| c.at < lo);
        let b = self.cross.partition_point(|c| c.at <= hi);
        to_u32(a)..to_u32(b)
    }
}

/// A via chosen for one cut, with its metal relative to its center.
#[derive(Clone, Debug)]
pub struct ViaTech {
    /// The via definition.
    pub via: ViaId,
    /// Metal on the layer below.
    pub bottom: Rect<Dbu>,
    /// Metal on the layer above.
    pub top: Rect<Dbu>,
    /// Center distance below which two of these cuts break the cut spacing.
    pub cut_reach: Dbu,
}

/// Everything the router needs to know about the technology.
#[derive(Clone, Debug)]
pub struct RouteTech {
    /// Lowest layer signal wires may use.
    pub min_layer: LayerId,
    /// Highest layer signal wires may use.
    pub max_layer: LayerId,
    /// Every routing layer, indexed by layer id.
    pub layers: Vec<LayerTech>,
    /// The via for each cut, indexed by cut id.
    pub vias: Vec<ViaTech>,
    /// Every via on each cut, least overhang on the layer above first, for
    /// reaching pins below the routable layers.
    pub access_vias: Vec<Vec<ViaTech>>,
}

pub(crate) fn to_u32(i: usize) -> u32 {
    u32::try_from(i).unwrap_or(u32::MAX)
}

impl RouteTech {
    /// Builds the routing technology of `db`.
    pub fn new(db: &NetlistDB, config: &RoutingConfig) -> Result<Self, RouteError> {
        let lib = &db.library;
        let count = lib.routing_layers.len();
        let range = config.layer_range(lib)?;
        let (min_layer, max_layer) = (*range.start(), *range.end());

        let tracks: Vec<Vec<Dbu>> = (0..count)
            .map(|l| layer_tracks(db, LayerId::new(l)))
            .collect();
        let routable = min_layer.index()..=max_layer.index();
        if let Some(l) = routable.into_iter().find(|&l| tracks[l].is_empty()) {
            return Err(RouteError::NoTracks(lib.routing_layers[l].name.clone()));
        }

        let mut layers = Vec::with_capacity(count);
        for l in 0..count {
            let below = l.checked_sub(1).map(|b| &tracks[b]);
            let above = tracks.get(l + 1);
            let cross = merge_cross(below, above);
            let index_in = |coords: Option<&Vec<Dbu>>, t: Dbu| {
                coords.and_then(|c| c.binary_search(&t).ok()).map(to_u32)
            };
            let below_cross = l
                .checked_sub(1)
                .map(|b| merge_cross(b.checked_sub(1).map(|bb| &tracks[bb]), Some(&tracks[l])));
            let above_cross =
                (l + 1 < count).then(|| merge_cross(Some(&tracks[l]), tracks.get(l + 2)));
            let position = |list: &Option<Vec<Dbu>>, t: Dbu| {
                list.as_ref()
                    .and_then(|c| c.binary_search(&t).ok())
                    .map(to_u32)
            };
            let layer = &lib.routing_layers[l];
            layers.push(LayerTech {
                direction: layer.direction,
                track_down: tracks[l]
                    .iter()
                    .map(|&t| position(&below_cross, t))
                    .collect(),
                track_up: tracks[l]
                    .iter()
                    .map(|&t| position(&above_cross, t))
                    .collect(),
                cross: cross
                    .into_iter()
                    .map(|at| Cross {
                        at,
                        down: index_in(below, at),
                        up: index_in(above, at),
                    })
                    .collect(),
                tracks: tracks[l].clone(),
                width: layer.width,
                spacing: layer.min_spacing,
            });
        }

        let cuts = 0..lib.cut_layers.len();
        let vias = cuts
            .clone()
            .map(|c| {
                ranked_vias(db, CutLayerId::new(c), &layers, true)
                    .into_iter()
                    .next()
                    .ok_or_else(|| RouteError::NoVia(lib.cut_layers[c].name.clone()))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let access_vias = cuts
            .map(|c| ranked_vias(db, CutLayerId::new(c), &layers, false))
            .collect();
        Ok(Self {
            min_layer,
            max_layer,
            layers,
            vias,
            access_vias,
        })
    }

    /// The layer data of `layer`.
    pub fn layer(&self, layer: LayerId) -> &LayerTech {
        &self.layers[layer.index()]
    }

    /// The via between `layer` and the layer above it.
    pub fn via_above(&self, layer: LayerId) -> &ViaTech {
        &self.vias[layer.index()]
    }

    /// Whether `layer` may carry signal wires.
    pub fn routable(&self, layer: LayerId) -> bool {
        self.min_layer <= layer && layer <= self.max_layer
    }
}

/// Sorted, deduplicated union of two coordinate lists.
fn merge_cross(a: Option<&Vec<Dbu>>, b: Option<&Vec<Dbu>>) -> Vec<Dbu> {
    let mut all: Vec<Dbu> = a.into_iter().chain(b).flatten().copied().collect();
    all.sort_unstable();
    all.dedup();
    all
}

/// Track coordinates of a layer in its preferred direction, from DEF
/// `TRACKS`, or from its LEF pitch and offset across the die if none are given.
fn layer_tracks(db: &NetlistDB, layer: LayerId) -> Vec<Dbu> {
    let l = db.library.layer(layer);
    let axis = match l.direction {
        Direction::Horizontal => TrackAxis::Y,
        Direction::Vertical => TrackAxis::X,
    };
    let mut coords: Vec<Dbu> = db
        .tracks
        .iter()
        .filter(|t| t.axis == axis && t.layers.contains(&layer))
        .flat_map(pare_common::db::design::TrackDef::coords)
        .collect();
    if coords.is_empty() {
        let (lo, hi, offset) = match l.direction {
            Direction::Horizontal => (db.die_area.min.y, db.die_area.max.y, l.offset_y),
            Direction::Vertical => (db.die_area.min.x, db.die_area.max.x, l.offset_x),
        };
        let pitch = l.track_pitch();
        if pitch > Dbu::ZERO {
            let mut t = lo + offset.unwrap_or(pitch / 2);
            while t < hi {
                coords.push(t);
                t += pitch;
            }
        }
    }
    coords.sort_unstable();
    coords.dedup();
    coords
}

/// How far a metal rectangle reaches across a layer's tracks beyond a
/// centered wire.
fn across_overhang(rect: &Rect<Dbu>, layer: &LayerTech) -> Dbu {
    let ((lo, hi), _) = layer.across_along_rect(rect);
    let half = layer.width / 2;
    (hi - half).max(-lo - half).max(Dbu::ZERO)
}

/// Every LEF via through `cut`, ordered by how far its metal sticks out
/// across the tracks of both layers (or of the upper layer only, when
/// `both` is false), then LEF defaults first, then smaller metal.
fn ranked_vias(db: &NetlistDB, cut: CutLayerId, layers: &[LayerTech], both: bool) -> Vec<ViaTech> {
    let lib = &db.library;
    let mut ranked: Vec<((Dbu, bool, i64), ViaTech)> = Vec::new();
    for (i, v) in lib.vias.iter().enumerate() {
        if v.cut != cut || v.from_def {
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
        let top_overhang = across_overhang(&top, &layers[v.top().index()]);
        let overhang = if both {
            top_overhang.max(across_overhang(&bottom, &layers[v.bottom().index()]))
        } else {
            top_overhang
        };
        let key = (overhang, !v.is_default, (bottom.area() + top.area()).get());
        let cut_size = v
            .shapes
            .iter()
            .filter(|s| s.layer == LayerRef::Cut(cut))
            .map(|s| s.rect.width().max(s.rect.height()))
            .max()
            .unwrap_or_default();
        let spacing = lib.cut_layers[cut.index()].spacing.unwrap_or_default();
        ranked.push((
            key,
            ViaTech {
                via: ViaId::new(i),
                bottom,
                top,
                cut_reach: cut_size + spacing,
            },
        ));
    }
    ranked.sort_by_key(|(key, v)| (*key, v.via));
    ranked.into_iter().map(|(_, v)| v).collect()
}
