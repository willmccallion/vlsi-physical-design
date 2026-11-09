//! Single cells moved to a legal site, as `OpenDP`'s `legalCellPos` moves
//! each cell the resizer edits once nets follow global routes.
//!
//! A cell goes off any macro under it, into the core, onto the site grid
//! and onto a row. It may still overlap others; detailed placement settles
//! that.

use super::grid::{GridPt, GridRect, GridX, GridY};
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
    site: Dbu,
    /// The bottom and top of every row, relative to the core, ascending.
    row_edges: Vec<Dbu>,
    sites_per_row: GridX,
    /// Each macro's grid cells.
    blocks: Vec<BlockCells>,
}

#[derive(Clone, Copy, Debug)]
struct BlockCells {
    cells: GridRect,
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
        let site = db.library.site(rows.first()?.site).size.x;
        let mut row_edges: Vec<Dbu> = rows
            .iter()
            .flat_map(|r| {
                let bottom = r.origin.y - core.min.y;
                [bottom, bottom + db.library.site(r.site).size.y]
            })
            .collect();
        row_edges.sort_unstable();
        row_edges.dedup();
        let mut grid = Self {
            core,
            site,
            row_edges,
            sites_per_row: GridX::from_dbu_floor(core.width(), site),
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
                let to_core = Point::new(-core.min.x, -core.min.y);
                let bbox = db.cell_bbox(CellId::new(i)).translate(to_core);
                let x = GridX::from_dbu_floor(bbox.min.x, site);
                let width = GridX::from_dbu_floor(bbox.width() + site - Dbu(1), site);
                let bottom = grid.snap_down(bbox.min.y);
                let top_edge = grid.row_edge(bottom) + bbox.height();
                BlockCells {
                    cells: GridRect {
                        min: GridPt::new(x, bottom),
                        max: GridPt::new(x + width, grid.end_row(top_edge)),
                    },
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
        let c = &db.cells[cell];
        let size = db.library.cell_macro(c.macro_id).size;
        let init = c.location - self.core.min;
        let off_macro = self.point_off_macro(init, size);
        let legal = self.legal_point(off_macro, size);
        if legal == init {
            return None;
        }
        let x = GridX::from_dbu_floor(legal.x, self.site).to_dbu(self.site);
        let y = self.row_edge(self.snap_down(legal.y));
        Some(self.core.min + Point::new(x, y))
    }

    /// Moves `cell` where [`Self::legal_position`] puts it.
    pub fn legalize(&self, db: &mut NetlistDB, cell: CellId) {
        if let Some(at) = self.legal_position(db, cell) {
            db.cells[cell].location = at;
        }
    }

    /// The point beside the macro at a corner of the cell's grid cells, as
    /// `pointOffMacro`; the point as it is when there is none.
    fn point_off_macro(&self, init: Point<Dbu>, size: Point<Dbu>) -> Point<Dbu> {
        let x_lo = GridX::from_dbu_floor(init.x, self.site);
        let x_hi = GridX::from_dbu_floor(init.x + size.x + self.site - Dbu(1), self.site);
        let y_lo = self.snap_down(init.y);
        let y_hi = self.end_row(init.y + size.y);
        let corners = [
            GridPt::new(x_lo, y_lo),
            GridPt::new(x_hi, y_lo),
            GridPt::new(x_lo, y_hi),
            GridPt::new(x_hi, y_hi),
        ];
        let block = corners.iter().find_map(|&at| self.block_at(at));
        block.map_or(init, |b| self.nearest_block_edge(init, size, &b.bbox))
    }

    /// The macro covering grid cell `at`, if any; none off the grid.
    fn block_at(&self, at: GridPt) -> Option<&BlockCells> {
        let on_grid =
            (GridX::ZERO..self.sites_per_row).contains(&at.x) && at.y + 1 < self.edge_count();
        if !on_grid {
            return None;
        }
        self.blocks.iter().rev().find(|b| b.cells.contains(at))
    }

    /// The cell moved off `block` across its nearest edge, as
    /// `nearestBlockEdge`.
    fn nearest_block_edge(
        &self,
        at: Point<Dbu>,
        size: Point<Dbu>,
        block: &Rect<Dbu>,
    ) -> Point<Dbu> {
        let x_min = (at.x - block.min.x).abs();
        let x_max = (block.max.x - (at.x + size.x)).abs();
        let y_min = (at.y - block.min.y).abs();
        let y_max = (block.max.y - (at.y + size.y)).abs();
        let nearest = x_min.min(x_max).min(y_min).min(y_max);
        let to = if nearest == x_min {
            Point::new(block.min.x - size.x, at.y)
        } else if nearest == x_max {
            Point::new(block.max.x, at.y)
        } else if nearest == y_min {
            Point::new(at.x, block.min.y - size.y)
        } else {
            Point::new(at.x, block.max.y)
        };
        self.legal_point(to, size)
    }

    /// The point inside the core, on the site grid and on a row edge, as
    /// `legalPt` with a point.
    fn legal_point(&self, at: Point<Dbu>, size: Point<Dbu>) -> Point<Dbu> {
        let core_x = at.x.clamp(
            Dbu::ZERO,
            (self.sites_per_row.to_dbu(self.site) - size.x).max(Dbu::ZERO),
        );
        let column = GridX::from_dbu_round(core_x, self.site);
        // NOTE: diverges from OpenDP, which bounds this by the core's absolute top.
        let core_y =
            at.y.clamp(Dbu::ZERO, (self.core.height() - size.y).max(Dbu::ZERO));
        Point::new(
            column.to_dbu(self.site),
            self.row_edge(self.round_row(core_y)),
        )
    }

    /// How many row edges there are: one past the last row.
    const fn edge_count(&self) -> GridY {
        GridY::new(self.row_edges.len() as i64)
    }

    /// The last row edge at or below `y`, as `gridSnapDownY`.
    fn snap_down(&self, y: Dbu) -> GridY {
        GridY::new(
            self.row_edges
                .partition_point(|&e| e <= y)
                .saturating_sub(1) as i64,
        )
    }

    /// The first row edge at or above `y`, as `gridEndY`.
    fn end_row(&self, y: Dbu) -> GridY {
        GridY::new(self.row_edges.partition_point(|&e| e < y) as i64)
    }

    /// The nearer of the row edges around `y`, the upper on a tie, as
    /// `gridRoundY`.
    fn round_row(&self, y: Dbu) -> GridY {
        let below = self.snap_down(y);
        if below + 1 < self.edge_count()
            && (self.row_edge(below) - y).abs() >= (self.row_edge(below + 1) - y).abs()
        {
            return below + 1;
        }
        below
    }

    /// The offset of row edge `index` from the core's bottom, the core's
    /// top past the last, as `gridYToDbu`.
    fn row_edge(&self, index: GridY) -> Dbu {
        usize::try_from(index.get())
            .ok()
            .and_then(|i| self.row_edges.get(i))
            .copied()
            .unwrap_or(self.core.max.y)
    }
}
