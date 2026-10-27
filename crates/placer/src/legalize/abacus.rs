//! Abacus legalization (Spindler, Schlichtmann and Johannes, ISPD 2008).
//!
//! Movable macros are placed first at the nearest free row- and site-aligned
//! spot. Standard cells are then taken in order of their global X position;
//! each is trial-placed in candidate rows by appending it to that row's
//! clusters and collapsing them, and is committed to the row with the least
//! displacement. Cluster positions snap to the site grid and cells take
//! their row's orientation. Every cell, macros included, is only tried in
//! the stretches of row its zone may use, so fence regions hold.

use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::indices::CellId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

use crate::rows::{
    RowSpan, classify, expand_rows, fixed_obstacles, free_intervals, orientation_allowed,
    rows_outward, snap_down, snap_up, squared,
};
use crate::zones::{Zone, Zones, intersect};

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
    OverlappingRows(i64),
}

/// Moves every movable cell to a legal, row- and site-aligned position
/// inside its zone.
pub fn legalize(db: &mut NetlistDB, zones: &Zones) -> Result<(), LegalizeError> {
    if db.rows.is_empty() {
        return Err(LegalizeError::NoRows);
    }
    let rows = expand_rows(db);
    if let Some(y) = overlapping_row(&rows) {
        return Err(LegalizeError::OverlappingRows(y.get()));
    }
    let (macros, std_cells) = classify(db, &rows);

    let mut obstacles = fixed_obstacles(db);
    legalize_macros(db, zones, &rows, &macros, &mut obstacles)?;

    let mut subrows = build_subrows(db, zones, &rows, &obstacles);
    let ranges = row_ranges(&subrows, rows.len());
    let mut order = std_cells;
    order.sort_by_key(|&c| (db.cells[c.index()].location.x, c));
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
        let target = db.cells[cell.index()].location;
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
            let (y0, y1) = (row.y, row.y + size.y);
            let free = free_intervals(obstacles, die.min.x, die.max.x, y0, y1);
            let allowed = zones.intervals(zones.of(cell), die.min.x, die.max.x, y0, y1);
            for (lo, hi) in intersect(&free, &allowed) {
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
        let (_, at) = best
            .ok_or_else(|| LegalizeError::MacroDoesNotFit(db.cells[cell.index()].name.clone()))?;
        let c = &mut db.cells[cell.index()];
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
        let sites = ((ideal - self.lo.to_f64()) / site.to_f64()).round() as i64;
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
                let c = &mut db.cells[cell.index()];
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
        let lo = row.x_min.max(die.min.x);
        let hi = row.x_max.min(die.max.x);
        let (y0, y1) = (row.y, row.y + row.height);
        let free = free_intervals(obstacles, lo, hi, y0, y1);
        for zone in zones.all() {
            for (a, b) in intersect(&free, &zones.intervals(zone, lo, hi, y0, y1)) {
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
    let target = db.cells[cell.index()].location;
    let size = db.cell_size(cell);

    let mut best: Option<(i128, usize)> = None;
    for r in rows_outward(rows, target.y) {
        let row = &rows[r];
        let dy = squared(row.y - target.y);
        if best.is_some_and(|(cost, _)| dy >= cost) {
            break;
        }
        if row.height != size.y || !orientation_allowed(db, cell, row.orient) {
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

    let (_, s) =
        best.ok_or_else(|| LegalizeError::CellDoesNotFit(db.cells[cell.index()].name.clone()))?;
    let site = rows[subrows[s].row].site;
    let width = snap_up(size.x, Dbu::ZERO, site);
    let sub = &mut subrows[s];
    let target_x = target.x.clamp(sub.lo, sub.hi - width);
    sub.add(cell, target_x, width, site);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::design::{Region, RegionKind, Row};
    use pare_common::db::indices::{RegionId, SiteId};
    use pare_common::db::library::{
        CoreKind, Library, Macro, MacroClass, Site, SiteClass, Symmetry,
    };
    use pare_common::geom::dbu::DbuPerMicron;
    use pare_common::geom::orient::Orientation;
    use pare_common::verify::placement::placement_violations;

    fn pt(x: i64, y: i64) -> Point<Dbu> {
        Point::new(Dbu(x), Dbu(y))
    }

    fn legalize_zoned(db: &mut NetlistDB) -> Result<(), LegalizeError> {
        let zones = Zones::new(db).unwrap();
        legalize(db, &zones)
    }

    /// Two 10-site rows (N then FS) of 10 x 100 sites over a 100 x 200 die.
    fn design(cells: &[(Point<Dbu>, PlacementStatus, i64)]) -> NetlistDB {
        let mut lib = Library::new(DbuPerMicron::new(1000).unwrap());
        let symmetry = Symmetry {
            x: true,
            y: true,
            r90: false,
        };
        let site = lib
            .add_site(Site {
                name: "s".into(),
                class: SiteClass::Core,
                size: pt(10, 100),
                symmetry,
            })
            .unwrap();
        let mut db = NetlistDB::new("t".into(), lib);
        db.die_area = Rect::new(pt(0, 0), pt(100, 200));
        for (k, orient) in [Orientation::N, Orientation::FS].into_iter().enumerate() {
            db.rows.push(Row {
                name: format!("r{k}"),
                site: SiteId(0),
                origin: pt(0, 100 * k as i64),
                orient,
                num_x: 10,
                num_y: 1,
                step: pt(10, 0),
            });
        }
        for (i, &(at, status, width)) in cells.iter().enumerate() {
            let name = format!("m{i}");
            let m = db
                .library
                .add_macro(Macro::new(
                    name.clone(),
                    MacroClass::Core(CoreKind::Regular),
                    pt(width, 100),
                    symmetry,
                    Some(site),
                ))
                .unwrap();
            let _ = db.add_cell(name, m, at, Orientation::N, status).unwrap();
        }
        db
    }

    /// Fences the left half of both rows and assigns `members` to it.
    fn fence_left_half(db: &mut NetlistDB, members: &[usize]) {
        db.regions.push(Region {
            name: "f".into(),
            rects: vec![Rect::new(pt(0, 0), pt(50, 200))],
            kind: RegionKind::Fence,
            group_order: None,
        });
        for &m in members {
            db.cells[m].region = Some(RegionId(0));
        }
    }

    #[test]
    fn a_fence_member_far_outside_its_fence_is_legalized_inside_it() {
        let mut db = design(&[(pt(80, 0), PlacementStatus::Placed, 20)]);
        fence_left_half(&mut db, &[0]);

        legalize_zoned(&mut db).unwrap();

        assert!(
            db.cells[0].location.x <= Dbu(30),
            "{:?}",
            db.cells[0].location
        );
        assert_eq!(placement_violations(&db), []);
    }

    #[test]
    fn a_cell_outside_every_fence_is_legalized_out_of_one() {
        let mut db = design(&[(pt(10, 0), PlacementStatus::Placed, 20)]);
        fence_left_half(&mut db, &[]);

        legalize_zoned(&mut db).unwrap();

        assert!(
            db.cells[0].location.x >= Dbu(50),
            "{:?}",
            db.cells[0].location
        );
        assert_eq!(placement_violations(&db), []);
    }

    #[test]
    fn a_full_fence_is_an_error_even_with_room_outside() {
        let mut db = design(&[(pt(0, 0), PlacementStatus::Placed, 50); 5]);
        fence_left_half(&mut db, &[0, 1, 2, 3, 4]);

        assert!(matches!(
            legalize_zoned(&mut db),
            Err(LegalizeError::CellDoesNotFit(_))
        ));
    }

    #[test]
    fn overlapping_cells_end_up_legal() {
        let mut db = design(&[(pt(33, 7), PlacementStatus::Placed, 20); 6]);

        legalize_zoned(&mut db).unwrap();

        assert!(
            placement_violations(&db).is_empty(),
            "{:?}",
            placement_violations(&db)
        );
    }

    #[test]
    fn cells_take_their_row_orientation() {
        let mut db = design(&[(pt(40, 95), PlacementStatus::Placed, 20)]);

        legalize_zoned(&mut db).unwrap();

        assert_eq!(db.cells[0].location, pt(40, 100));
        assert_eq!(db.cells[0].orient, Orientation::FS);
    }

    #[test]
    fn a_cell_already_on_a_site_does_not_move() {
        let mut db = design(&[(pt(30, 0), PlacementStatus::Placed, 20)]);

        legalize_zoned(&mut db).unwrap();

        assert_eq!(db.cells[0].location, pt(30, 0));
    }

    #[test]
    fn cells_flow_around_a_fixed_cell() {
        let mut db = design(&[
            (pt(40, 0), PlacementStatus::Fixed, 20),
            (pt(42, 0), PlacementStatus::Placed, 20),
        ]);

        legalize_zoned(&mut db).unwrap();

        assert_eq!(db.cells[0].location, pt(40, 0));
        assert_eq!(placement_violations(&db), []);
    }

    #[test]
    fn more_cells_than_sites_is_an_error() {
        let mut db = design(&[(pt(0, 0), PlacementStatus::Placed, 50); 5]);

        assert!(matches!(
            legalize_zoned(&mut db),
            Err(LegalizeError::CellDoesNotFit(_))
        ));
    }

    #[test]
    fn cells_fill_the_row_from_the_right_edge_inward() {
        let mut db = design(&[(pt(95, 0), PlacementStatus::Placed, 30); 3]);

        legalize_zoned(&mut db).unwrap();

        let mut xs: Vec<Dbu> = db
            .cells
            .iter()
            .filter(|c| c.location.y == Dbu(0))
            .map(|c| c.location.x)
            .collect();
        xs.sort();
        assert_eq!(xs, [Dbu(10), Dbu(40), Dbu(70)]);
    }
}
