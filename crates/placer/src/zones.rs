//! Fence regions as zones of the rows.
//!
//! Every movable cell belongs to one zone: the fence region it is assigned
//! to, or the space outside every fence. A cell of a fence must lie inside
//! the union of the fence's rectangles, and a cell outside every fence may
//! not overlap one. Global placement, legalization and detailed placement
//! all take the stretches of row each zone may use from here, so they agree
//! on the same geometry.

use pare_common::db::design::{NetlistDB, RegionKind};
use pare_common::db::indices::{CellId, IdVec, RegionId};
use pare_common::geom::dbu::Dbu;
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

    /// The X intervals of `[lo, hi)` where a cell of `zone` spanning the band
    /// `[y0, y1)` may sit, ignoring obstacles.
    pub fn intervals(&self, zone: Zone, lo: Dbu, hi: Dbu, y0: Dbu, y1: Dbu) -> Vec<(Dbu, Dbu)> {
        match zone {
            Zone::Fence(_) => self
                .fence(zone)
                .map_or_else(Vec::new, |f| inside(&f.rects, lo, hi, y0, y1)),
            Zone::Outside => outside(self.fences.iter().flat_map(|f| &f.rects), lo, hi, y0, y1),
        }
    }
}

/// The X intervals of `[lo, hi)` where the band `[y0, y1)` lies inside the
/// union of `rects`.
fn inside(rects: &[Rect<Dbu>], lo: Dbu, hi: Dbu, y0: Dbu, y1: Dbu) -> Vec<(Dbu, Dbu)> {
    if hi <= lo || y1 <= y0 {
        return Vec::new();
    }
    let band = Rect::new(Point::new(lo, y0), Point::new(hi, y1));
    let near: Vec<Rect<Dbu>> = rects
        .iter()
        .copied()
        .filter(|r| r.overlaps(&band))
        .collect();
    let mut xs: Vec<Dbu> = near
        .iter()
        .flat_map(|r| [r.min.x, r.max.x])
        .filter(|&x| lo < x && x < hi)
        .chain([lo, hi])
        .collect();
    xs.sort_unstable();
    xs.dedup();
    let mut out: Vec<(Dbu, Dbu)> = Vec::new();
    for slab in xs.windows(2) {
        let piece = Rect::new(Point::new(slab[0], y0), Point::new(slab[1], y1));
        if !piece.covered_by(&near) {
            continue;
        }
        match out.last_mut() {
            Some(last) if last.1 == slab[0] => last.1 = slab[1],
            _ => out.push((slab[0], slab[1])),
        }
    }
    out
}

/// The X intervals of `[lo, hi)` where the band `[y0, y1)` overlaps none of
/// `fences`.
fn outside<'a>(
    fences: impl Iterator<Item = &'a Rect<Dbu>>,
    lo: Dbu,
    hi: Dbu,
    y0: Dbu,
    y1: Dbu,
) -> Vec<(Dbu, Dbu)> {
    if hi <= lo {
        return Vec::new();
    }
    let mut blocked: Vec<(Dbu, Dbu)> = fences
        .filter(|r| r.min.y < y1 && r.max.y > y0 && r.min.x < hi && r.max.x > lo)
        .map(|r| (r.min.x, r.max.x))
        .collect();
    blocked.sort_unstable();
    let mut free = Vec::new();
    let mut cursor = lo;
    for (b0, b1) in blocked {
        if b0 > cursor {
            free.push((cursor, b0.min(hi)));
        }
        cursor = cursor.max(b1);
        if cursor >= hi {
            break;
        }
    }
    if cursor < hi {
        free.push((cursor, hi));
    }
    free
}

/// The overlaps of two sorted lists of disjoint intervals.
pub(crate) fn intersect(a: &[(Dbu, Dbu)], b: &[(Dbu, Dbu)]) -> Vec<(Dbu, Dbu)> {
    let mut out = Vec::new();
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        let lo = a[i].0.max(b[j].0);
        let hi = a[i].1.min(b[j].1);
        if lo < hi {
            out.push((lo, hi));
        }
        if a[i].1 < b[j].1 {
            i += 1;
        } else {
            j += 1;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    fn spans(list: &[(i64, i64)]) -> Vec<(Dbu, Dbu)> {
        list.iter().map(|&(a, b)| (Dbu(a), Dbu(b))).collect()
    }

    #[test]
    fn a_fence_covering_the_band_gives_its_width() {
        let fence = [rect(100, 0, 400, 2000)];

        let got = inside(&fence, Dbu(0), Dbu(1000), Dbu(0), Dbu(1000));

        assert_eq!(got, spans(&[(100, 400)]));
    }

    #[test]
    fn abutting_fence_rectangles_merge_into_one_interval() {
        let fence = [rect(100, 0, 400, 2000), rect(400, 0, 700, 2000)];

        let got = inside(&fence, Dbu(0), Dbu(1000), Dbu(0), Dbu(1000));

        assert_eq!(got, spans(&[(100, 700)]));
    }

    #[test]
    fn a_fence_covering_part_of_the_band_height_gives_nothing() {
        let fence = [rect(100, 0, 400, 500)];

        let got = inside(&fence, Dbu(0), Dbu(1000), Dbu(0), Dbu(1000));

        assert_eq!(got, []);
    }

    #[test]
    fn fence_rectangles_stacked_over_the_band_cover_it() {
        let fence = [rect(100, 0, 400, 500), rect(100, 500, 400, 1000)];

        let got = inside(&fence, Dbu(0), Dbu(1000), Dbu(0), Dbu(1000));

        assert_eq!(got, spans(&[(100, 400)]));
    }

    #[test]
    fn outside_leaves_the_gaps_between_fences() {
        let fences = [rect(100, 0, 400, 2000), rect(600, 0, 700, 2000)];

        let got = outside(fences.iter(), Dbu(0), Dbu(1000), Dbu(0), Dbu(1000));

        assert_eq!(got, spans(&[(0, 100), (400, 600), (700, 1000)]));
    }

    #[test]
    fn a_fence_touching_part_of_the_band_height_still_blocks_outside_cells() {
        let fences = [rect(100, 900, 400, 2000)];

        let got = outside(fences.iter(), Dbu(0), Dbu(1000), Dbu(0), Dbu(1000));

        assert_eq!(got, spans(&[(0, 100), (400, 1000)]));
    }

    #[test]
    fn a_fence_only_abutting_the_band_does_not_block_it() {
        let fences = [rect(100, 1000, 400, 2000)];

        let got = outside(fences.iter(), Dbu(0), Dbu(1000), Dbu(0), Dbu(1000));

        assert_eq!(got, spans(&[(0, 1000)]));
    }

    #[test]
    fn intervals_intersect_pairwise() {
        let a = spans(&[(0, 100), (200, 400)]);
        let b = spans(&[(50, 250), (300, 350)]);

        assert_eq!(
            intersect(&a, &b),
            spans(&[(50, 100), (200, 250), (300, 350)])
        );
    }
}
