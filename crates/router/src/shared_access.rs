//! Pin access shared by the cells of one kind.
//!
//! A standard cell's pins below the routable layers are reached through via
//! stacks judged against the cell's own shapes alone, at crossings of the
//! lowest routable layer near them. Two cells of one master and orientation
//! around which that layer's tracks and crossings fall alike, relative to
//! each cell, therefore have the same candidates, shifted by whole tracks
//! and crossings. They are found once per kind of cell, as `TritonRoute`'s
//! pin access finds them once per unique instance, and every cell's are
//! derived from its kind's.

use crate::access::{AccessPoint, Node, access_points};
use crate::tech::{RouteTech, to_u32};
use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{CellId, LayerId, MacroId, PinId, ViaId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use std::collections::HashMap;
use std::ops::Range;

/// How many gcell pitches beyond a cell's edge its kind takes the grid in.
const MARGIN_PITCHES: i64 = 4;

/// What a cell's pin access depends on.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct Kind {
    master: MacroId,
    orient: Orientation,
    /// The lowest routable layer's tracks around the cell, relative to it.
    tracks: Vec<Dbu>,
    /// That layer's crossings around the cell, relative to it.
    crossings: Vec<Dbu>,
}

/// The part of the lowest routable layer's grid around one cell.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Window {
    tracks: Range<u32>,
    crossings: Range<u32>,
    location: Point<Dbu>,
}

/// One access candidate of a kind, relative to a cell of it.
#[derive(Clone, Debug, PartialEq, Eq)]
struct RelativeAccess {
    layer: LayerId,
    track: u32,
    cross: u32,
    at: Point<Dbu>,
    pin_layer: LayerId,
    vias: Vec<ViaId>,
}

/// The access candidates of every pin of the cells that share them, by
/// kind of cell.
#[derive(Debug, Default)]
pub struct SharedAccess {
    /// For each cell, its kind and its window of the grid.
    cells: Vec<(u32, Window)>,
    /// For each kind, the candidates of each pin of its master, none for a
    /// pin whose candidates cannot be shared.
    kinds: Vec<Vec<Option<Vec<RelativeAccess>>>>,
}

impl SharedAccess {
    /// Finds the kinds of the cells of `db` and the candidates of each.
    pub fn new(db: &NetlistDB, tech: &RouteTech) -> Self {
        let cells: Vec<(Kind, Window)> = (0..db.num_cells())
            .into_par_iter()
            .map(|c| kind_of(db, tech, CellId::new(c)))
            .collect();
        let mut index: HashMap<Kind, u32> = HashMap::new();
        let mut representatives: Vec<CellId> = Vec::new();
        let cells: Vec<(u32, Window)> = cells
            .into_iter()
            .enumerate()
            .map(|(c, (kind, window))| {
                let next = to_u32(representatives.len());
                let k = *index.entry(kind).or_insert_with(|| {
                    representatives.push(CellId::new(c));
                    next
                });
                (k, window)
            })
            .collect();
        let kinds = representatives
            .par_iter()
            .map(|&cell| kind_candidates(db, tech, cell, &cells[cell.index()].1))
            .collect();
        Self { cells, kinds }
    }

    /// How many kinds of cell there are.
    pub const fn kinds(&self) -> usize {
        self.kinds.len()
    }

    /// The candidates of `pin` derived from its cell's kind, or none when
    /// they are not shared and must be found for the pin itself.
    pub fn candidates(&self, db: &NetlistDB, pin: PinId) -> Option<Vec<AccessPoint>> {
        let PinOwner::Cell { cell, pin: own } = db.pins[pin.index()].owner else {
            return None;
        };
        let (kind, window) = self.cells.get(cell.index())?;
        let table = self.kinds[*kind as usize].get(own.index())?.as_ref()?;
        Some(
            table
                .iter()
                .map(|r| AccessPoint {
                    node: Node {
                        layer: r.layer,
                        track: window.tracks.start + r.track,
                        cross: window.crossings.start + r.cross,
                    },
                    at: window.location + r.at,
                    pin_layer: r.pin_layer,
                    vias: r.vias.clone(),
                })
                .collect(),
        )
    }

    /// How many candidates `pin` has, without deriving them, when they are
    /// shared.
    pub fn count(&self, db: &NetlistDB, pin: PinId) -> Option<usize> {
        let PinOwner::Cell { cell, pin: own } = db.pins[pin.index()].owner else {
            return None;
        };
        let (kind, _) = self.cells.get(cell.index())?;
        self.kinds[*kind as usize]
            .get(own.index())?
            .as_ref()
            .map(Vec::len)
    }
}

/// The kind of `cell` and its window of the lowest routable layer's grid.
fn kind_of(db: &NetlistDB, tech: &RouteTech, cell: CellId) -> (Kind, Window) {
    let c = &db.cells[cell.index()];
    let lt = tech.layer(tech.min_layer);
    let window = db.cell_bbox(cell).expand(tech.gcell_pitch * MARGIN_PITCHES);
    let ((a0, a1), (b0, b1)) = lt.across_along_rect(&window);
    let ((origin_across, _), (origin_along, _)) =
        lt.across_along_rect(&Rect::new(c.location, c.location));
    let tracks = lt.tracks_within(a0, a1);
    let crossings = lt.cross_within(b0, b1);
    let kind = Kind {
        master: c.macro_id,
        orient: c.orient,
        tracks: tracks
            .clone()
            .map(|t| lt.tracks[t as usize] - origin_across)
            .collect(),
        crossings: crossings
            .clone()
            .map(|k| lt.cross[k as usize].at - origin_along)
            .collect(),
    };
    let window = Window {
        tracks,
        crossings,
        location: c.location,
    };
    (kind, window)
}

/// The candidates of each pin of the master of `cell`, a cell of its kind,
/// relative to its `window`: none for a pin it does not connect, one with
/// a shape on a routable layer, whose candidates depend on what lies around
/// it, or one with a candidate outside the window.
fn kind_candidates(
    db: &NetlistDB,
    tech: &RouteTech,
    cell: CellId,
    window: &Window,
) -> Vec<Option<Vec<RelativeAccess>>> {
    let master = db.cell_macro(cell);
    let mut table = vec![None; master.pins.len()];
    for &pin in &db.cells[cell.index()].pins {
        let PinOwner::Cell { pin: own, .. } = db.pins[pin.index()].owner else {
            continue;
        };
        if !below_routable_layers(db, tech, pin) {
            continue;
        }
        let relative: Option<Vec<RelativeAccess>> = access_points(db, tech, pin, &|_, _| true)
            .iter()
            .map(|a| relative_to(a, window))
            .collect();
        if let Some(slot) = table.get_mut(own.index()) {
            *slot = relative;
        }
    }
    table
}

/// Whether every shape of `pin` lies below the lowest routable layer.
fn below_routable_layers(db: &NetlistDB, tech: &RouteTech, pin: PinId) -> bool {
    db.pin_shapes(pin)
        .iter()
        .all(|s| matches!(s.layer, LayerRef::Routing(l) if l < tech.min_layer))
}

/// `a` relative to `window`, or none when its node lies outside it.
fn relative_to(a: &AccessPoint, window: &Window) -> Option<RelativeAccess> {
    if !window.tracks.contains(&a.node.track) || !window.crossings.contains(&a.node.cross) {
        return None;
    }
    Some(RelativeAccess {
        layer: a.node.layer,
        track: a.node.track - window.tracks.start,
        cross: a.node.cross - window.crossings.start,
        at: a.at - window.location,
        pin_layer: a.pin_layer,
        vias: a.vias.clone(),
    })
}
