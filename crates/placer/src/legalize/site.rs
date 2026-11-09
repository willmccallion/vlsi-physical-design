//! Single cells moved to a legal site, as `OpenDP`'s `legalCellPos` moves
//! each cell the resizer edits once nets follow global routes.
//!
//! A cell goes off any macro under it, into the core, onto the site grid
//! and onto a row. It may still overlap others; detailed placement settles
//! that.

use super::grid::{GridPt, GridRect, RowGrid, core_rows};
use super::opendp::DetailedPlacementError;
use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::indices::CellId;
use pare_common::db::library::MacroClass;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// The core's site grid as `OpenDP` builds it, with the macros on it.
#[derive(Clone, Debug)]
pub struct SiteGrid {
    rows: RowGrid,
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
    /// The grid of `db`'s core rows with its fixed macros painted on it as
    /// `Opendp::setFixedGridCells` paints them.
    ///
    /// # Errors
    ///
    /// When there is no core row, or the rows' sites differ in width.
    pub fn new(db: &NetlistDB) -> Result<Self, DetailedPlacementError> {
        let rows = RowGrid::new(db, &core_rows(db))?;
        let blocks = db
            .cells
            .ids()
            .filter(|&id| {
                db.cells[id].status == PlacementStatus::Fixed
                    && db.cell_macro(id).class == MacroClass::Block
            })
            .map(|id| {
                let bbox = rows.relative(db.cell_bbox(id));
                BlockCells {
                    cells: rows.covering(&bbox),
                    bbox,
                }
            })
            .collect();
        Ok(Self { rows, blocks })
    }

    /// Where `cell` goes, as `legalCellPos` moves it, if it moves; its
    /// orientation stays as it is.
    #[must_use]
    pub fn legal_position(&self, db: &NetlistDB, cell: CellId) -> Option<Point<Dbu>> {
        let c = &db.cells[cell];
        let size = c.orient.oriented_size(db.cell_macro(cell).size);
        let core = self.rows.core();
        let init = c.location - core.min;
        let off_macro = self.point_off_macro(init, size);
        let legal = self.rows.legal_pt(off_macro, size);
        (legal != init).then(|| core.min + legal)
    }

    /// Moves `cell` where [`Self::legal_position`] puts it.
    pub fn legalize(&self, db: &mut NetlistDB, cell: CellId) {
        if let Some(at) = self.legal_position(db, cell) {
            db.cells[cell].location = at;
        }
    }

    /// The point beside the macro on a corner of the grid cells the cell
    /// covers, as `pointOffMacro`; the point as it is when there is none.
    fn point_off_macro(&self, init: Point<Dbu>, size: Point<Dbu>) -> Point<Dbu> {
        let cells = self.rows.covering(&Rect::new(init, init + size));
        let corners = [
            cells.min,
            GridPt::new(cells.max.x, cells.min.y),
            GridPt::new(cells.min.x, cells.max.y),
            cells.max,
        ];
        let block = corners.iter().find_map(|&at| self.block_at(at));
        block.map_or(init, |b| self.rows.nearest_block_edge(init, size, &b.bbox))
    }

    /// The macro covering grid cell `at`, if any; none off the grid.
    fn block_at(&self, at: GridPt) -> Option<&BlockCells> {
        if !self.rows.contains(at) {
            return None;
        }
        self.blocks.iter().rev().find(|b| b.cells.contains(at))
    }
}
