//! Placement rows split into single-height spans, and helpers for fitting
//! cells into them.

use pare_common::db::design::{BlockageKind, NetlistDB};
use pare_common::db::indices::CellId;
use pare_common::db::library::MacroClass;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// One single-height row of sites.
pub(crate) struct RowSpan {
    pub(crate) y: Dbu,
    pub(crate) height: Dbu,
    pub(crate) x_min: Dbu,
    pub(crate) x_max: Dbu,
    pub(crate) site: Dbu,
    pub(crate) orient: Orientation,
}

/// Rows from the design, with multi-row `DO ... BY n` statements split up,
/// sorted by Y. Rows at the same Y that overlap or abut on one site grid
/// with one orientation are merged, so no site is counted twice.
pub(crate) fn expand_rows(db: &NetlistDB) -> Vec<RowSpan> {
    let mut rows = Vec::new();
    for r in &db.rows {
        let site = r.orient.oriented_size(db.library.site(r.site).size);
        let step = if r.step.x > Dbu::ZERO {
            r.step.x
        } else {
            site.x
        };
        for k in 0..i64::from(r.num_y) {
            rows.push(RowSpan {
                y: r.origin.y + r.step.y * k,
                height: site.y,
                x_min: r.origin.x,
                x_max: r.origin.x + step * i64::from(r.num_x.saturating_sub(1)) + site.x,
                site: step,
                orient: r.orient,
            });
        }
    }
    rows.sort_by_key(|r| (r.y, r.x_min));
    merge_overlapping(rows)
}

/// Merges each row into the one before it when they share Y, height,
/// orientation and site grid and overlap or abut; `rows` are sorted by Y
/// then X.
fn merge_overlapping(rows: Vec<RowSpan>) -> Vec<RowSpan> {
    let mut merged: Vec<RowSpan> = Vec::with_capacity(rows.len());
    for row in rows {
        if let Some(last) = merged.last_mut()
            && last.y == row.y
            && last.height == row.height
            && last.orient == row.orient
            && last.site == row.site
            && (row.x_min - last.x_min) % row.site.get() == Dbu::ZERO
            && row.x_min <= last.x_max
        {
            last.x_max = last.x_max.max(row.x_max);
            continue;
        }
        merged.push(row);
    }
    merged
}

/// Splits movable cells into macros, placed freely, and single-row cells.
pub(crate) fn classify(db: &NetlistDB, rows: &[RowSpan]) -> (Vec<CellId>, Vec<CellId>) {
    let mut macros = Vec::new();
    let mut cells = Vec::new();
    for id in db.cells.ids() {
        if !db.cells[id].status.is_movable() {
            continue;
        }
        let m = db.cell_macro(id);
        let fits_a_row =
            matches!(m.class, MacroClass::Core(_)) && rows.iter().any(|r| r.height == m.size.y);
        if fits_a_row {
            cells.push(id);
        } else {
            macros.push(id);
        }
    }
    (macros, cells)
}

/// Fixed cells and hard placement blockages.
pub(crate) fn fixed_obstacles(db: &NetlistDB) -> Vec<Rect<Dbu>> {
    let mut out: Vec<Rect<Dbu>> = db
        .cells
        .ids()
        .filter(|&i| !db.cells[i].status.is_movable())
        .map(|i| db.cell_bbox(i))
        .collect();
    out.extend(db.hard_placement_blockages());
    out
}

/// Which sites of a blockage global placement blocks, as
/// `PlacerBase::initInstsForUnusableSites` walks them: the first, then each
/// one while the share blocked so far is at most `blocked_share`.
struct SitePattern {
    blocked_share: f32,
    visited: u32,
    blocked: u32,
}

impl SitePattern {
    // In single precision, as gpl's floats.
    fn new(kind: BlockageKind) -> Self {
        let max_density = match kind {
            BlockageKind::Hard | BlockageKind::Soft => 0.0,
            BlockageKind::Partial(density) => density as f32,
        };
        Self {
            blocked_share: (100.0 - max_density) / 100.0,
            visited: 0,
            blocked: 0,
        }
    }

    const fn blocks_every_site(&self) -> bool {
        self.blocked_share >= 1.0
    }

    // In single precision, as gpl's floats.
    fn next_site_is_blocked(&mut self) -> bool {
        let blocked =
            self.visited == 0 || self.blocked as f32 / self.visited as f32 <= self.blocked_share;
        self.visited += 1;
        self.blocked += u32::from(blocked);
        blocked
    }

    /// The stretches of the sites `[x0, x1)` of `row` this pattern blocks.
    fn stretches(&mut self, row: &RowSpan, x0: Dbu, x1: Dbu) -> Vec<Rect<Dbu>> {
        let stretch =
            |a: Dbu, b: Dbu| Rect::new(Point::new(a, row.y), Point::new(b, row.y + row.height));
        if self.blocks_every_site() {
            return vec![stretch(x0, x1)];
        }
        let mut out = Vec::new();
        let mut run = None;
        let mut x = x0;
        while x < x1 {
            let blocked = self.next_site_is_blocked();
            match (blocked, run) {
                (true, None) => run = Some(x),
                (false, Some(start)) => {
                    out.push(stretch(start, x));
                    run = None;
                }
                _ => {}
            }
            x += row.site;
        }
        if let Some(start) = run {
            out.push(stretch(start, x1));
        }
        out
    }
}

/// The stretches of each of `rows` global placement keeps out of: the sites
/// any placement blockage touches, soft ones included, as `gpl` fills them
/// with dummy instances.
pub(crate) fn blocked_stretches(db: &NetlistDB, rows: &[RowSpan]) -> Vec<Vec<Rect<Dbu>>> {
    let mut blocked = vec![Vec::new(); rows.len()];
    for blockage in &db.placement_blockages {
        let b = blockage.rect;
        let mut pattern = SitePattern::new(blockage.kind);
        let first = rows.partition_point(|r| r.y + r.height <= b.min.y);
        for (k, row) in rows.iter().enumerate().skip(first) {
            if row.y >= b.max.y {
                break;
            }
            let x0 = snap_down(b.min.x.max(row.x_min), row.x_min, row.site);
            let x1 = snap_up(b.max.x.min(row.x_max), row.x_min, row.site);
            if x0 < x1 {
                blocked[k].extend(pattern.stretches(row, x0, x1));
            }
        }
    }
    blocked
}

/// Free X intervals of `[lo, hi)` not covered by any obstacle overlapping the
/// open band `(y0, y1)`.
pub(crate) fn free_intervals(
    obstacles: &[Rect<Dbu>],
    lo: Dbu,
    hi: Dbu,
    y0: Dbu,
    y1: Dbu,
) -> Vec<(Dbu, Dbu)> {
    let mut blocked: Vec<(Dbu, Dbu)> = obstacles
        .iter()
        .filter(|r| r.min.y < y1 && r.max.y > y0 && r.min.x < hi && r.max.x > lo)
        .map(|r| (r.min.x, r.max.x))
        .collect();
    blocked.sort();
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

/// Rounds `x` down to the site grid starting at `origin`.
pub(crate) fn snap_down(x: Dbu, origin: Dbu, site: Dbu) -> Dbu {
    origin + site * (x - origin).get().div_euclid(site.get())
}

/// Rounds `x` up to the site grid starting at `origin`.
pub(crate) fn snap_up(x: Dbu, origin: Dbu, site: Dbu) -> Dbu {
    let down = snap_down(x, origin, site);
    if down == x { down } else { down + site }
}

/// Row indices in order of vertical distance from `y`, nearest first.
pub(crate) fn rows_outward(rows: &[RowSpan], y: Dbu) -> impl Iterator<Item = usize> + '_ {
    let start = rows.partition_point(|r| r.y < y);
    let mut below = start;
    let mut above = start;
    std::iter::from_fn(move || {
        let down = below.checked_sub(1).map(|i| (y - rows[i].y, i));
        let up = (above < rows.len()).then(|| (rows[above].y - y, above));
        match (down, up) {
            (Some(d), Some(u)) if d.0 < u.0 => {
                below -= 1;
                Some(d.1)
            }
            (_, Some(u)) => {
                above += 1;
                Some(u.1)
            }
            (Some(d), None) => {
                below -= 1;
                Some(d.1)
            }
            (None, None) => None,
        }
    })
}

pub(crate) fn squared(d: Dbu) -> i128 {
    i128::from(d.get()) * i128::from(d.get())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn blocked_widths(kind: BlockageKind, sites: i64) -> Vec<(i64, i64)> {
        let row = span(0, 0, 200 * sites);
        SitePattern::new(kind)
            .stretches(&row, Dbu::ZERO, row.x_max)
            .iter()
            .map(|r| (r.min.x.get(), r.max.x.get()))
            .collect()
    }

    #[test]
    fn a_soft_blockage_blocks_its_whole_stretch_at_once() {
        assert_eq!(blocked_widths(BlockageKind::Soft, 4), [(0, 800)]);
    }

    #[test]
    fn a_partial_blockage_blocks_every_other_site_at_half_density() {
        assert_eq!(
            blocked_widths(BlockageKind::Partial(50.0), 4),
            [(0, 200), (400, 600)]
        );
    }

    #[test]
    fn a_partial_blockage_of_full_density_still_blocks_its_first_site() {
        assert_eq!(blocked_widths(BlockageKind::Partial(100.0), 3), [(0, 200)]);
    }

    fn span(y: i64, x_min: i64, x_max: i64) -> RowSpan {
        RowSpan {
            y: Dbu(y),
            height: Dbu(2000),
            x_min: Dbu(x_min),
            x_max: Dbu(x_max),
            site: Dbu(200),
            orient: Orientation::N,
        }
    }

    fn extents(rows: &[RowSpan]) -> Vec<(i64, i64, i64)> {
        rows.iter()
            .map(|r| (r.y.get(), r.x_min.get(), r.x_max.get()))
            .collect()
    }

    #[test]
    fn a_row_inside_another_at_the_same_height_merges_into_it() {
        let rows = vec![span(0, 0, 10_000), span(0, 200, 6_000)];

        assert_eq!(extents(&merge_overlapping(rows)), [(0, 0, 10_000)]);
    }

    #[test]
    fn abutting_rows_on_one_grid_merge_but_separate_ones_do_not() {
        let rows = vec![
            span(0, 0, 4_000),
            span(0, 4_000, 6_000),
            span(0, 8_000, 9_000),
        ];

        assert_eq!(
            extents(&merge_overlapping(rows)),
            [(0, 0, 6_000), (0, 8_000, 9_000)]
        );
    }

    #[test]
    fn rows_off_each_others_site_grid_stay_apart() {
        let rows = vec![span(0, 0, 4_000), span(0, 100, 6_000)];

        assert_eq!(merge_overlapping(rows).len(), 2);
    }
}
