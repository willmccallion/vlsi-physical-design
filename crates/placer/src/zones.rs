//! Fence regions as zones of the rows.
//!
//! Every movable cell belongs to one zone: the fence region it is assigned
//! to, or the space outside every fence. A cell of a fence must lie inside
//! the union of the fence's rectangles, and a cell outside every fence may
//! not overlap one. Global placement, legalization and detailed placement
//! all take the stretches of row each zone may use from here, so they agree
//! on the same geometry.

use crate::rows::free_intervals;
use pare_common::db::design::{NetlistDB, RegionKind};
use pare_common::db::indices::{CellId, IdVec, RegionId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::Interval;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// The part of the rows a movable cell is confined to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Zone {
    /// Inside the rectangles of a fence region.
    Fence(RegionId),
    /// Outside every fence region.
    Outside,
}

pare_common::define_index!(
    /// A zone's position in [`Zones::all`]: each fence, then the space
    /// outside them.
    ZoneId(u32)
);

/// Why the design's regions cannot be turned into zones.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum ZoneError {
    /// Members of an inclusive region would share its rows with cells
    /// outside it, which zones of their own cannot express.
    #[error("inclusive region '{0}' is not supported")]
    InclusiveRegion(String),
}

/// A fence region's rectangles, under its DEF name.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Fence {
    region: RegionId,
    name: String,
    rects: Vec<Rect<Dbu>>,
}

/// The zone of every cell and the rectangles of every fence.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Zones {
    of_cell: IdVec<CellId, ZoneId>,
    fences: Vec<Fence>,
}

impl Zones {
    /// The zones of `db`'s cells. Members of a guide region are placed as
    /// if they had none, since a guide only suggests where they should go.
    pub fn new(db: &NetlistDB) -> Result<Self, ZoneError> {
        let mut fences = Vec::new();
        let mut zone_of_region = IdVec::filled(None, db.regions.len());
        for (id, region) in db.regions.iter_enumerated() {
            match region.kind {
                RegionKind::Fence => {
                    zone_of_region[id] = Some(ZoneId::new(fences.len()));
                    fences.push(Fence {
                        region: id,
                        name: region.name.clone(),
                        rects: region.rects.clone(),
                    });
                }
                RegionKind::Inclusive => {
                    return Err(ZoneError::InclusiveRegion(region.name.clone()));
                }
                RegionKind::Guide => {}
            }
        }
        let outside = ZoneId::new(fences.len());
        let of_cell = db
            .cells
            .iter()
            .map(|c| c.region.and_then(|r| zone_of_region[r]).unwrap_or(outside))
            .collect();
        Ok(Self { of_cell, fences })
    }

    /// The zone of `cell`.
    pub fn of(&self, cell: CellId) -> Zone {
        self.zone(self.of_cell[cell])
    }

    /// The position of `cell`'s zone in [`Self::all`].
    pub fn id_of(&self, cell: CellId) -> ZoneId {
        self.of_cell[cell]
    }

    /// The zone at `id` in [`Self::all`].
    fn zone(&self, id: ZoneId) -> Zone {
        self.fences
            .get(id.index())
            .map_or(Zone::Outside, |f| Zone::Fence(f.region))
    }

    /// The fence of `zone`, if it is one.
    fn fence(&self, zone: Zone) -> Option<&Fence> {
        match zone {
            Zone::Fence(r) => self.fences.iter().find(|f| f.region == r),
            Zone::Outside => None,
        }
    }

    /// How a message names `zone`: a fence by its DEF name.
    pub fn describe(&self, zone: Zone) -> String {
        self.fence(zone).map_or_else(
            || "the space outside every fence".to_string(),
            |f| format!("fence region '{}'", f.name),
        )
    }

    /// Every zone: each fence, then the space outside them.
    pub fn all(&self) -> impl Iterator<Item = Zone> + '_ {
        self.fences
            .iter()
            .map(|f| Zone::Fence(f.region))
            .chain(std::iter::once(Zone::Outside))
    }

    /// Whether the design has any fence.
    pub(crate) const fn has_fences(&self) -> bool {
        !self.fences.is_empty()
    }

    /// The rectangles of a fence, or nothing for the space outside them.
    pub(crate) fn fence_rects(&self, zone: Zone) -> &[Rect<Dbu>] {
        self.fence(zone).map_or(&[], |f| f.rects.as_slice())
    }

    /// The stretches of `span` along X where a cell of `zone` spanning the
    /// band `band` along Y may sit, ignoring obstacles.
    pub fn intervals(&self, zone: Zone, span: Interval, band: Interval) -> Vec<Interval> {
        match zone {
            Zone::Fence(_) => self
                .fence(zone)
                .map_or_else(Vec::new, |f| inside(&f.rects, span, band)),
            Zone::Outside => free_intervals(self.fences.iter().flat_map(|f| &f.rects), span, band),
        }
    }
}

/// The stretches of `span` along X where the band `band` along Y lies
/// inside the union of `rects`.
fn inside(rects: &[Rect<Dbu>], span: Interval, band: Interval) -> Vec<Interval> {
    if span.is_empty() || band.is_empty() {
        return Vec::new();
    }
    let whole = Rect::new(Point::new(span.lo, band.lo), Point::new(span.hi, band.hi));
    let near: Vec<Rect<Dbu>> = rects
        .iter()
        .copied()
        .filter(|r| r.overlaps(&whole))
        .collect();
    let mut xs: Vec<Dbu> = near
        .iter()
        .flat_map(|r| [r.min.x, r.max.x])
        .filter(|&x| span.lo < x && x < span.hi)
        .chain([span.lo, span.hi])
        .collect();
    xs.sort_unstable();
    xs.dedup();
    let mut out: Vec<Interval> = Vec::new();
    for slab in xs.windows(2) {
        let piece = Rect::new(Point::new(slab[0], band.lo), Point::new(slab[1], band.hi));
        if !piece.covered_by(&near) {
            continue;
        }
        match out.last_mut() {
            Some(last) if last.hi == slab[0] => last.hi = slab[1],
            _ => out.push(Interval::new(slab[0], slab[1])),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(
            Point::new(Dbu::new(x0), Dbu::new(y0)),
            Point::new(Dbu::new(x1), Dbu::new(y1)),
        )
    }

    fn spans(list: &[(i64, i64)]) -> Vec<Interval> {
        list.iter()
            .map(|&(a, b)| Interval::new(Dbu::new(a), Dbu::new(b)))
            .collect()
    }

    fn span(lo: i64, hi: i64) -> Interval {
        Interval::new(Dbu::new(lo), Dbu::new(hi))
    }

    #[test]
    fn a_fence_covering_the_band_gives_its_width() {
        let fence = [rect(100, 0, 400, 2000)];

        let got = inside(&fence, span(0, 1000), span(0, 1000));

        assert_eq!(got, spans(&[(100, 400)]));
    }

    #[test]
    fn abutting_fence_rectangles_merge_into_one_interval() {
        let fence = [rect(100, 0, 400, 2000), rect(400, 0, 700, 2000)];

        let got = inside(&fence, span(0, 1000), span(0, 1000));

        assert_eq!(got, spans(&[(100, 700)]));
    }

    #[test]
    fn a_fence_covering_part_of_the_band_height_gives_nothing() {
        let fence = [rect(100, 0, 400, 500)];

        let got = inside(&fence, span(0, 1000), span(0, 1000));

        assert_eq!(got, []);
    }

    #[test]
    fn fence_rectangles_stacked_over_the_band_cover_it() {
        let fence = [rect(100, 0, 400, 500), rect(100, 500, 400, 1000)];

        let got = inside(&fence, span(0, 1000), span(0, 1000));

        assert_eq!(got, spans(&[(100, 400)]));
    }

    #[test]
    fn outside_leaves_the_gaps_between_fences() {
        let fences = [rect(100, 0, 400, 2000), rect(600, 0, 700, 2000)];

        let got = free_intervals(&fences, span(0, 1000), span(0, 1000));

        assert_eq!(got, spans(&[(0, 100), (400, 600), (700, 1000)]));
    }

    #[test]
    fn a_fence_touching_part_of_the_band_height_still_blocks_outside_cells() {
        let fences = [rect(100, 900, 400, 2000)];

        let got = free_intervals(&fences, span(0, 1000), span(0, 1000));

        assert_eq!(got, spans(&[(0, 100), (400, 1000)]));
    }

    #[test]
    fn a_fence_only_abutting_the_band_does_not_block_it() {
        let fences = [rect(100, 1000, 400, 2000)];

        let got = free_intervals(&fences, span(0, 1000), span(0, 1000));

        assert_eq!(got, spans(&[(0, 1000)]));
    }
}
