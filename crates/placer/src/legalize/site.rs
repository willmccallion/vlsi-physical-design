//! Single cells moved to a legal site, as `OpenDP`'s `legalCellPos` moves
//! each cell the resizer edits once nets follow global routes.
//!
//! A cell goes off any macro under it, into the core, onto the site grid
//! and onto a row. It may still overlap others; detailed placement settles
//! that.

use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::indices::CellId;
use pare_common::db::library::{MacroClass, SiteClass};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// The core's site grid as `OpenDP` builds it, with the macros on it.
#[derive(Clone, Debug)]
pub struct SiteGrid {
    core: Rect<Dbu>,
    site: i64,
    /// The bottom and top of every row, relative to the core, ascending.
    row_edges: Vec<i64>,
    sites_per_row: i64,
    /// Each macro's grid cells: columns and row edges, ends excluded.
    blocks: Vec<BlockCells>,
}

#[derive(Clone, Copy, Debug)]
struct BlockCells {
    columns: (i64, i64),
    rows: (usize, usize),
    /// Its box relative to the core.
    bbox: Rect<Dbu>,
}

impl SiteGrid {
    /// The grid of `db`'s core rows; none without a core row.
    #[must_use]
    pub fn new(db: &NetlistDB) -> Option<Self> {
        let core = db.core_area();
        let rows: Vec<_> = db
            .rows
            .iter()
            .filter(|r| db.library.site(r.site).class != SiteClass::Pad)
            .collect();
        let site = db.library.site(rows.first()?.site).size.x.get();
        let mut row_edges: Vec<i64> = rows
            .iter()
            .flat_map(|r| {
                let bottom = r.origin.y.get() - core.min.y.get();
                [bottom, bottom + db.library.site(r.site).size.y.get()]
            })
            .collect();
        row_edges.sort_unstable();
        row_edges.dedup();
        let mut grid = Self {
            core,
            site,
            row_edges,
            sites_per_row: core.width().get().div_euclid(site),
            blocks: Vec::new(),
        };
        grid.blocks = db
            .cells
            .iter()
            .enumerate()
            .filter(|(_, c)| {
                c.status == PlacementStatus::Fixed
                    && db.library.cell_macro(c.macro_id).class == MacroClass::Block
            })
            .map(|(i, _)| {
                let to_core = Point::new(Dbu(-core.min.x.get()), Dbu(-core.min.y.get()));
                let bbox = db.cell_bbox(CellId::new(i)).translate(to_core);
                let x = bbox.min.x.get().div_euclid(site);
                let width = (bbox.width().get() + site - 1).div_euclid(site);
                let bottom = grid.snap_down(bbox.min.y.get());
                let top_edge = grid.row_edge(bottom) + bbox.height().get();
                BlockCells {
                    columns: (x, x + width),
                    rows: (bottom, grid.end_row(top_edge)),
                    bbox,
                }
            })
            .collect();
        Some(grid)
    }

    /// Where `cell` goes, as `legalCellPos` moves it, if it moves; its
    /// orientation stays as it is.
    #[must_use]
    pub fn legal_position(&self, db: &NetlistDB, cell: CellId) -> Option<Point<Dbu>> {
        let c = &db.cells[cell.index()];
        let size = db.library.cell_macro(c.macro_id).size;
        let (width, height) = (size.x.get(), size.y.get());
        let init = (
            c.location.x.get() - self.core.min.x.get(),
            c.location.y.get() - self.core.min.y.get(),
        );
        let off_macro = self.point_off_macro(init, width, height);
        let legal = self.legal_point(off_macro, width, height);
        if legal == init {
            return None;
        }
        let x = legal.0.div_euclid(self.site) * self.site;
        let y = self.row_edge(self.snap_down(legal.1));
        Some(Point::new(
            Dbu(self.core.min.x.get() + x),
            Dbu(self.core.min.y.get() + y),
        ))
    }

    /// Moves `cell` where [`Self::legal_position`] puts it.
    pub fn legalize(&self, db: &mut NetlistDB, cell: CellId) {
        if let Some(at) = self.legal_position(db, cell) {
            db.cells[cell.index()].location = at;
        }
    }

    /// The point beside the macro at a corner of the cell's grid cells, as
    /// `pointOffMacro`; the point as it is when there is none.
    fn point_off_macro(&self, init: (i64, i64), width: i64, height: i64) -> (i64, i64) {
        let x_lo = init.0.div_euclid(self.site);
        let x_hi = (init.0 + width + self.site - 1).div_euclid(self.site);
        let y_lo = self.snap_down(init.1);
        let y_hi = self.end_row(init.1 + height);
        let corners = [(x_lo, y_lo), (x_hi, y_lo), (x_lo, y_hi), (x_hi, y_hi)];
        let block = corners.iter().find_map(|&(x, y)| self.block_at(x, y));
        block.map_or(init, |b| {
            self.nearest_block_edge(init, width, height, &b.bbox)
        })
    }

    /// The macro covering grid cell `(x, y)`, if any; none off the grid.
    fn block_at(&self, x: i64, y: usize) -> Option<&BlockCells> {
        let on_grid = (0..self.sites_per_row).contains(&x) && y + 1 < self.row_edges.len();
        if !on_grid {
            return None;
        }
        self.blocks
            .iter()
            .rev()
            .find(|b| (b.columns.0..b.columns.1).contains(&x) && (b.rows.0..b.rows.1).contains(&y))
    }

    /// The cell moved off `block` across its nearest edge, as
    /// `nearestBlockEdge`.
    fn nearest_block_edge(
        &self,
        at: (i64, i64),
        width: i64,
        height: i64,
        block: &Rect<Dbu>,
    ) -> (i64, i64) {
        let x_min = (at.0 - block.min.x.get()).abs();
        let x_max = (block.max.x.get() - (at.0 + width)).abs();
        let y_min = (at.1 - block.min.y.get()).abs();
        let y_max = (block.max.y.get() - (at.1 + height)).abs();
        let nearest = x_min.min(x_max).min(y_min).min(y_max);
        let to = if nearest == x_min {
            (block.min.x.get() - width, at.1)
        } else if nearest == x_max {
            (block.max.x.get(), at.1)
        } else if nearest == y_min {
            (at.0, block.min.y.get() - height)
        } else {
            (at.0, block.max.y.get())
        };
        self.legal_point(to, width, height)
    }

    /// The point inside the core, on the site grid and on a row edge, as
    /// `legalPt` with a point.
    fn legal_point(&self, at: (i64, i64), width: i64, height: i64) -> (i64, i64) {
        let core_x =
            at.0.clamp(0, (self.sites_per_row * self.site - width).max(0));
        let column = (core_x as f64 / self.site as f64).round() as i64;
        let core_y = at.1.clamp(0, (self.core.max.y.get() - height).max(0));
        (column * self.site, self.row_edge(self.round_row(core_y)))
    }

    /// The last row edge at or below `y`, as `gridSnapDownY`.
    fn snap_down(&self, y: i64) -> usize {
        self.row_edges
            .partition_point(|&e| e <= y)
            .saturating_sub(1)
    }

    /// The first row edge at or above `y`, as `gridEndY`.
    fn end_row(&self, y: i64) -> usize {
        self.row_edges.partition_point(|&e| e < y)
    }

    /// The nearer of the row edges around `y`, the upper on a tie, as
    /// `gridRoundY`.
    fn round_row(&self, y: i64) -> usize {
        let below = self.snap_down(y);
        if below + 1 < self.row_edges.len()
            && (self.row_edges[below] - y).abs() >= (self.row_edges[below + 1] - y).abs()
        {
            return below + 1;
        }
        below
    }

    /// The offset of row edge `index` from the core's bottom, the core's
    /// top past the last, as `gridYToDbu`.
    fn row_edge(&self, index: usize) -> i64 {
        self.row_edges
            .get(index)
            .copied()
            .unwrap_or_else(|| self.core.max.y.get())
    }
}
