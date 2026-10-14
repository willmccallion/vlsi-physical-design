//! Placement rows split into single-height spans, and helpers for fitting
//! cells into them.

use pare_common::db::design::NetlistDB;
use pare_common::db::indices::CellId;
use pare_common::db::library::MacroClass;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
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
/// sorted by Y.
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
    rows
}

/// Splits movable cells into macros, placed freely, and single-row cells.
pub(crate) fn classify(db: &NetlistDB, rows: &[RowSpan]) -> (Vec<CellId>, Vec<CellId>) {
    let mut macros = Vec::new();
    let mut cells = Vec::new();
    for i in 0..db.num_cells() {
        let id = CellId::new(i);
        if !db.cells[i].status.is_movable() {
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
    let mut out: Vec<Rect<Dbu>> = (0..db.num_cells())
        .filter(|&i| !db.cells[i].status.is_movable())
        .map(|i| db.cell_bbox(CellId::new(i)))
        .collect();
    out.extend(db.placement_blockages.iter().copied());
    out
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

/// Whether a cell may take this row orientation given its symmetry.
pub(crate) fn orientation_allowed(db: &NetlistDB, cell: CellId, row: Orientation) -> bool {
    let symmetry = db.cell_macro(cell).symmetry;
    match row {
        Orientation::N => true,
        Orientation::FS => symmetry.x,
        Orientation::FN => symmetry.y,
        Orientation::S => symmetry.x && symmetry.y,
        _ => false,
    }
}
