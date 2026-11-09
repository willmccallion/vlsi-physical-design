//! Placement legality: die containment, overlaps, blockages, rows, sites,
//! row orientation and placement regions.

use crate::db::design::{NetlistDB, PlacementStatus, RegionKind};
use crate::db::indices::{CellId, IdVec};
use crate::db::library::MacroClass;
use crate::geom::dbu::Dbu;
use crate::geom::rect::Rect;
use std::collections::HashMap;

/// A placement rule a cell breaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlacementViolation {
    /// A cell has no location, so no other rule is checked for it.
    Unplaced(CellId),
    /// A movable cell extends past the die boundary.
    OutsideDie(CellId),
    /// A movable cell overlaps another placed cell, the first one found.
    Overlap(CellId, CellId),
    /// A movable cell overlaps a hard placement blockage.
    InBlockage(CellId),
    /// A movable standard cell does not sit inside a row.
    NotInRow(CellId),
    /// A movable standard cell is in a row but not on one of its sites.
    OffSite(CellId),
    /// A movable standard cell's orientation does not match its row.
    WrongOrientation(CellId),
    /// A movable cell assigned to a fence or inclusive region is not
    /// entirely inside it.
    OutsideRegion(CellId),
    /// A movable cell not assigned to a fence region overlaps one.
    InFence(CellId),
}

/// Checks every movable cell; fixed cells only matter as obstacles, and
/// unplaced cells are reported once without further checks.
pub fn placement_violations(db: &NetlistDB) -> Vec<PlacementViolation> {
    let mut violations = Vec::new();
    let rows = RowIndex::new(db);
    for (id, cell) in db.cells.iter_enumerated() {
        if cell.status == PlacementStatus::Unplaced {
            violations.push(PlacementViolation::Unplaced(id));
            continue;
        }
        if !cell.status.is_movable() {
            continue;
        }
        let bbox = db.cell_bbox(id);
        if !db.die_area.encloses(&bbox) {
            violations.push(PlacementViolation::OutsideDie(id));
        }
        if db.hard_placement_blockages().any(|b| b.overlaps(&bbox)) {
            violations.push(PlacementViolation::InBlockage(id));
        }
        if matches!(db.cell_macro(id).class, MacroClass::Core(_)) {
            violations.extend(rows.check(db, id, &bbox));
        }
        violations.extend(region_violation(db, id, &bbox));
    }
    violations.extend(overlaps(db));
    violations
}

/// The region rule a movable cell at `bbox` breaks: a member of a fence or
/// inclusive region must lie inside it, and no other cell may overlap a
/// fence. Guide regions constrain nothing.
fn region_violation(db: &NetlistDB, id: CellId, bbox: &Rect<Dbu>) -> Option<PlacementViolation> {
    match db.cells[id].region {
        Some(r) => {
            let region = &db.regions[r];
            let confined = matches!(region.kind, RegionKind::Fence | RegionKind::Inclusive);
            (confined && !bbox.covered_by(&region.rects))
                .then_some(PlacementViolation::OutsideRegion(id))
        }
        None => db
            .regions
            .iter()
            .filter(|region| region.kind == RegionKind::Fence)
            .any(|region| region.rects.iter().any(|r| r.overlaps(bbox)))
            .then_some(PlacementViolation::InFence(id)),
    }
}

/// Rows grouped by the Y coordinate of their origin.
struct RowIndex {
    by_y: HashMap<Dbu, Vec<usize>>,
}

impl RowIndex {
    fn new(db: &NetlistDB) -> Self {
        let mut by_y: HashMap<Dbu, Vec<usize>> = HashMap::new();
        for (i, r) in db.rows.iter().enumerate() {
            by_y.entry(r.origin.y).or_default().push(i);
        }
        Self { by_y }
    }

    fn check(&self, db: &NetlistDB, id: CellId, bbox: &Rect<Dbu>) -> Option<PlacementViolation> {
        let row = self.by_y.get(&bbox.min.y).and_then(|candidates| {
            candidates.iter().map(|&r| &db.rows[r]).find(|r| {
                let span = r.bbox(&db.library);
                span.min.x <= bbox.min.x && bbox.max.x <= span.max.x
            })
        });
        let Some(row) = row else {
            return Some(PlacementViolation::NotInRow(id));
        };
        let step = row.step.x.get();
        if step > 0 && (bbox.min.x - row.origin.x).get() % step != 0 {
            return Some(PlacementViolation::OffSite(id));
        }
        let orient = db.cells[id].orient;
        let mirrored_ok = db.cell_macro(id).symmetry.y && orient == row.orient.mirrored_about_y();
        if orient != row.orient && !mirrored_ok {
            return Some(PlacementViolation::WrongOrientation(id));
        }
        None
    }
}

/// One violation per movable cell that overlaps another placed cell, naming
/// the first one found; like `OpenROAD`, this counts overlapping cells rather
/// than pairs, so a stack of cells costs linear time and memory.
fn overlaps(db: &NetlistDB) -> Vec<PlacementViolation> {
    let boxes: IdVec<CellId, Rect<Dbu>> = db.cells.ids().map(|id| db.cell_bbox(id)).collect();
    let checked = |i: CellId| {
        let r = boxes[i];
        db.cells[i].status != PlacementStatus::Unplaced
            && r.width() > Dbu::ZERO
            && r.height() > Dbu::ZERO
    };
    let bin = boxes
        .ids()
        .filter(|&i| db.cells[i].status.is_movable())
        .map(|i| boxes[i].width().max(boxes[i].height()))
        .max()
        .unwrap_or(Dbu(1))
        .max(Dbu(1));

    let mut bins: HashMap<Bin, Vec<CellId>> = HashMap::new();
    for i in boxes.ids().filter(|&i| checked(i)) {
        for key in bins_covering(&boxes[i], bin) {
            bins.entry(key).or_default().push(i);
        }
    }

    let mut found = Vec::new();
    for a in boxes
        .ids()
        .filter(|&a| checked(a) && db.cells[a].status.is_movable())
    {
        let partner = bins_covering(&boxes[a], bin)
            .filter_map(|key| bins.get(&key))
            .flatten()
            .copied()
            .find(|&b| b != a && boxes[b].overlaps(&boxes[a]));
        if let Some(b) = partner {
            found.push(PlacementViolation::Overlap(a, b));
        }
    }
    found
}

/// One square of the overlap check's grid, counted in squares from the
/// origin.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct Bin {
    column: i64,
    row: i64,
}

/// The `side`-sized squares a non-empty rectangle touches.
fn bins_covering(r: &Rect<Dbu>, side: Dbu) -> impl Iterator<Item = Bin> {
    let squares =
        |lo: Dbu, hi: Dbu| lo.get().div_euclid(side.get())..=(hi.get() - 1).div_euclid(side.get());
    let rows = squares(r.min.y, r.max.y);
    squares(r.min.x, r.max.x)
        .flat_map(move |column| rows.clone().map(move |row| Bin { column, row }))
}
