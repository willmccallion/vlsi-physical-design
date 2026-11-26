//! Abacus legalization (Spindler, Schlichtmann and Johannes, ISPD 2008).
//!
//! Movable macros are placed first at the nearest free row- and site-aligned
//! spot. Standard cells are then taken in order of their global X position;
//! each is trial-placed in candidate rows by appending it to that row's
//! clusters and collapsing them, and is committed to the row with the least
//! displacement. Cluster positions snap to the site grid and cells take
//! their row's orientation. Every cell, macros included, is only tried in
//! the stretches of row its zone may use, so fence regions hold.

use pare_common::convert::SaturatingFrom;
use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::indices::CellId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

use crate::rows::{
    RowSpan, classify, expand_rows, fixed_obstacles, free_intervals, rows_outward, snap_down,
    snap_up, squared,
};
use crate::zones::{Zone, Zones};
use pare_common::geom::interval::{Interval, intersect_sorted};

/// Why legalization failed.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum LegalizeError {
    /// The design has no placement rows.
    #[error("the design has no placement rows")]
    NoRows,
    /// No row has room for a standard cell.
    #[error("no row has room for cell '{0}'")]
    CellDoesNotFit(String),
    /// No free spot inside the die fits a macro.
    #[error("no free spot fits macro '{0}'")]
    MacroDoesNotFit(String),
    /// Two rows overlap at one Y without sharing a site grid and
    /// orientation, so their sites cannot be told apart.
    #[error("rows overlap at y = {0} on different site grids or orientations")]
    OverlappingRows(Dbu),
}

/// Moves every movable cell to a legal, row- and site-aligned position
/// inside its zone.
pub fn legalize(db: &mut NetlistDB, zones: &Zones) -> Result<(), LegalizeError> {
    if db.rows.is_empty() {
        return Err(LegalizeError::NoRows);
    }
    let rows = expand_rows(db);
    if let Some(y) = overlapping_row(&rows) {
        return Err(LegalizeError::OverlappingRows(y));
    }
    let (macros, std_cells) = classify(db, &rows);

    let mut obstacles = fixed_obstacles(db);
    legalize_macros(db, zones, &rows, &macros, &mut obstacles)?;

    let mut subrows = build_subrows(db, zones, &rows, &obstacles);
    let ranges = row_ranges(&subrows, rows.len());
    let mut order = std_cells;
    order.sort_by_key(|&c| (db.cells[c].location.x, c));
    for cell in order {
        place_cell(db, &rows, &ranges, &mut subrows, cell, zones.of(cell))?;
    }
    for sub in &subrows {
        sub.commit(db, &rows[sub.row]);
    }
    Ok(())
}

/// The Y of the first two rows, sorted by Y then X, that still overlap.
fn overlapping_row(rows: &[RowSpan]) -> Option<Dbu> {
    rows.windows(2)
        .find(|w| w[0].y == w[1].y && w[1].x_min < w[0].x_max)
        .map(|w| w[0].y)
}

fn legalize_macros(
    db: &mut NetlistDB,
    zones: &Zones,
    rows: &[RowSpan],
    macros: &[CellId],
    obstacles: &mut Vec<Rect<Dbu>>,
) -> Result<(), LegalizeError> {
    let mut order = macros.to_vec();
    order.sort_by_key(|&c| {
        let size = db.cell_size(c);
        std::cmp::Reverse((size.x * size.y, c))
    });
    let die = db.die_area;
    for cell in order {
        let size = db.cell_size(cell);
        let target = db.cells[cell].location;
        let mut best: Option<(i128, Point<Dbu>)> = None;
        for r in rows_outward(rows, target.y) {
            let row = &rows[r];
            let dy = squared(row.y - target.y);
            if best.is_some_and(|(cost, _)| dy >= cost) {
                break;
            }
            if row.y + size.y > die.max.y {
                continue;
            }
            let band = Interval::new(row.y, row.y + size.y);
            let free = free_intervals(obstacles.iter(), die.x_span(), band);
            let allowed = zones.intervals(zones.of(cell), die.x_span(), band);
            for Interval { lo, hi } in intersect_sorted(&free, &allowed) {
                let lo = snap_up(lo, row.x_min, row.site);
                let hi = snap_down(hi - size.x, row.x_min, row.site);
                if lo > hi {
                    continue;
                }
                let x = snap_down(target.x.clamp(lo, hi), row.x_min, row.site).max(lo);
                let cost = dy + squared(x - target.x);
                if best.is_none_or(|(c, _)| cost < c) {
                    best = Some((cost, Point::new(x, row.y)));
                }
            }
        }
        let (_, at) =
            best.ok_or_else(|| LegalizeError::MacroDoesNotFit(db.cells[cell].name.clone()))?;
        let c = &mut db.cells[cell];
        c.location = at;
        c.status = PlacementStatus::Placed;
        obstacles.push(Rect::new(at, at + size));
    }
    Ok(())
}

/// A group of abutting cells placed where their total width-weighted squared
/// displacement is least.
#[derive(Clone, Copy)]
struct Cluster {
    /// Index of the cluster's first cell in the sub-row's cell list.
    first: usize,
    weight: f64,
    q: f64,
    width: Dbu,
    x: Dbu,
}

/// A contiguous stretch of free sites in a row, all in one zone.
struct SubRow {
    row: usize,
    zone: Zone,
    lo: Dbu,
    hi: Dbu,
    used: Dbu,
    cells: Vec<(CellId, Dbu)>,
    clusters: Vec<Cluster>,
}

impl SubRow {
    fn free(&self) -> Dbu {
        self.hi - self.lo - self.used
    }

    /// Where a cluster lands: its optimum, snapped to a site and kept inside.
    fn position(&self, q: f64, weight: f64, width: Dbu, site: Dbu) -> Dbu {
        let ideal = q / weight;
        let sites = i64::saturating_from(((ideal - self.lo.to_f64()) / site.to_f64()).round());
        (self.lo + site * sites).clamp(self.lo, self.hi - width)
    }

    /// The x a new cell of `width` aiming for `target` would get, without
    /// changing the sub-row.
    fn trial(&self, target: Dbu, width: Dbu, site: Dbu) -> Dbu {
        let mut weight = width.to_f64();
        let mut q = weight * target.to_f64();
        let mut total = width;
        let mut x = self.position(q, weight, total, site);
        for c in self.clusters.iter().rev() {
            if c.x + c.width <= x {
                break;
            }
            q = weight.mul_add(-c.width.to_f64(), c.q + q);
            weight += c.weight;
            total += c.width;
            x = self.position(q, weight, total, site);
        }
        x + total - width
    }

    fn add(&mut self, cell: CellId, target: Dbu, width: Dbu, site: Dbu) {
        let w = width.to_f64();
        self.cells.push((cell, width));
        self.used += width;
        let mut cluster = Cluster {
            first: self.cells.len() - 1,
            weight: w,
            q: w * target.to_f64(),
            width,
            x: Dbu::ZERO,
        };
        cluster.x = self.position(cluster.q, cluster.weight, cluster.width, site);
        while let Some(prev) = self.clusters.last().copied() {
            if prev.x + prev.width <= cluster.x {
                break;
            }
            let _ = self.clusters.pop();
            cluster = Cluster {
                first: prev.first,
                weight: prev.weight + cluster.weight,
                q: cluster
                    .weight
                    .mul_add(-prev.width.to_f64(), prev.q + cluster.q),
                width: prev.width + cluster.width,
                x: Dbu::ZERO,
            };
            cluster.x = self.position(cluster.q, cluster.weight, cluster.width, site);
        }
        self.clusters.push(cluster);
    }

    fn commit(&self, db: &mut NetlistDB, row: &RowSpan) {
        for (k, cluster) in self.clusters.iter().enumerate() {
            let end = self
                .clusters
                .get(k + 1)
                .map_or(self.cells.len(), |c| c.first);
            let mut x = cluster.x;
            for &(cell, width) in &self.cells[cluster.first..end] {
                let c = &mut db.cells[cell];
                c.location = Point::new(x, row.y);
                c.orient = row.orient;
                c.status = PlacementStatus::Placed;
                x += width;
            }
        }
    }
}

/// Sub-rows of every row between obstacles, split by zone, grouped by row.
fn build_subrows(
    db: &NetlistDB,
    zones: &Zones,
    rows: &[RowSpan],
    obstacles: &[Rect<Dbu>],
) -> Vec<SubRow> {
    let die = db.die_area;
    let mut out = Vec::new();
    for (r, row) in rows.iter().enumerate() {
        let span = Interval::new(row.x_min.max(die.min.x), row.x_max.min(die.max.x));
        let band = row.band();
        let free = free_intervals(obstacles, span, band);
        for zone in zones.all() {
            for Interval { lo: a, hi: b } in
                intersect_sorted(&free, &zones.intervals(zone, span, band))
            {
                let a = snap_up(a, row.x_min, row.site);
                let b = snap_down(b, row.x_min, row.site);
                if b > a {
                    out.push(SubRow {
                        row: r,
                        zone,
                        lo: a,
                        hi: b,
                        used: Dbu::ZERO,
                        cells: Vec::new(),
                        clusters: Vec::new(),
                    });
                }
            }
        }
    }
    out
}

/// For each row, the half-open range of its sub-rows.
fn row_ranges(subrows: &[SubRow], num_rows: usize) -> Vec<(usize, usize)> {
    let mut ranges = vec![(0, 0); num_rows];
    let mut start = 0;
    while start < subrows.len() {
        let row = subrows[start].row;
        let end = start + subrows[start..].partition_point(|s| s.row == row);
        ranges[row] = (start, end);
        start = end;
    }
    ranges
}

fn place_cell(
    db: &NetlistDB,
    rows: &[RowSpan],
    ranges: &[(usize, usize)],
    subrows: &mut [SubRow],
    cell: CellId,
    zone: Zone,
) -> Result<(), LegalizeError> {
    let target = db.cells[cell].location;
    let size = db.cell_size(cell);

    let mut best: Option<(i128, usize)> = None;
    for r in rows_outward(rows, target.y) {
        let row = &rows[r];
        let dy = squared(row.y - target.y);
        if best.is_some_and(|(cost, _)| dy >= cost) {
            break;
        }
        if row.height != size.y || !db.cell_macro(cell).symmetry.allows(row.orient) {
            continue;
        }
        let width = snap_up(size.x, Dbu::ZERO, row.site);
        let (start, end) = ranges[r];
        for (s, sub) in subrows.iter().enumerate().take(end).skip(start) {
            if sub.zone != zone || sub.free() < width {
                continue;
            }
            let x = sub.trial(target.x.clamp(sub.lo, sub.hi - width), width, row.site);
            let cost = dy + squared(x - target.x);
            if best.is_none_or(|(c, _)| cost < c) {
                best = Some((cost, s));
            }
        }
    }

    let (_, s) = best.ok_or_else(|| LegalizeError::CellDoesNotFit(db.cells[cell].name.clone()))?;
    let site = rows[subrows[s].row].site;
    let width = snap_up(size.x, Dbu::ZERO, site);
    let sub = &mut subrows[s];
    let target_x = target.x.clamp(sub.lo, sub.hi - width);
    sub.add(cell, target_x, width, site);
    Ok(())
}

#[cfg(test)]
mod tests;
