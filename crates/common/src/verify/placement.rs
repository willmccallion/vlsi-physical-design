//! Placement legality: die containment, overlaps, blockages, rows, sites and
//! row orientation.

use crate::db::design::NetlistDB;
use crate::db::indices::CellId;
use crate::db::library::MacroClass;
use crate::geom::dbu::Dbu;
use crate::geom::orient::Orientation;
use crate::geom::rect::Rect;
use std::collections::HashMap;

/// A placement rule a cell breaks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlacementViolation {
    /// A movable cell extends past the die boundary.
    OutsideDie(CellId),
    /// Two cells overlap, at least one of them movable.
    Overlap(CellId, CellId),
    /// A movable cell overlaps a hard placement blockage.
    InBlockage(CellId),
    /// A movable standard cell does not sit inside a row.
    NotInRow(CellId),
    /// A movable standard cell is in a row but not on one of its sites.
    OffSite(CellId),
    /// A movable standard cell's orientation does not match its row.
    WrongOrientation(CellId),
}

/// Checks every movable cell; fixed cells only matter as obstacles.
pub fn placement_violations(db: &NetlistDB) -> Vec<PlacementViolation> {
    let mut violations = Vec::new();
    let rows = RowIndex::new(db);
    for i in 0..db.num_cells() {
        let id = CellId::new(i);
        let cell = &db.cells[i];
        if !cell.status.is_movable() {
            continue;
        }
        let bbox = db.cell_bbox(id);
        if !db.die_area.encloses(&bbox) {
            violations.push(PlacementViolation::OutsideDie(id));
        }
        if db.placement_blockages.iter().any(|b| b.overlaps(&bbox)) {
            violations.push(PlacementViolation::InBlockage(id));
        }
        if matches!(db.cell_macro(id).class, MacroClass::Core(_)) {
            violations.extend(rows.check(db, id, &bbox));
        }
    }
    violations.extend(overlaps(db));
    violations
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
        let orient = db.cells[id.index()].orient;
        let mirrored_ok = db.cell_macro(id).symmetry.y && orient == flip_y(row.orient);
        if orient != row.orient && !mirrored_ok {
            return Some(PlacementViolation::WrongOrientation(id));
        }
        None
    }
}

/// The orientation mirrored about the Y axis.
const fn flip_y(o: Orientation) -> Orientation {
    match o {
        Orientation::N => Orientation::FN,
        Orientation::FN => Orientation::N,
        Orientation::S => Orientation::FS,
        Orientation::FS => Orientation::S,
        Orientation::E => Orientation::FE,
        Orientation::FE => Orientation::E,
        Orientation::W => Orientation::FW,
        Orientation::FW => Orientation::W,
    }
}

/// Pairs of overlapping cells where at least one may move.
fn overlaps(db: &NetlistDB) -> Vec<PlacementViolation> {
    let boxes: Vec<Rect<Dbu>> = (0..db.num_cells())
        .map(|i| db.cell_bbox(CellId::new(i)))
        .collect();
    let movable: Vec<bool> = db.cells.iter().map(|c| c.status.is_movable()).collect();
    let bin = boxes
        .iter()
        .zip(&movable)
        .filter(|(_, m)| **m)
        .map(|(r, _)| r.width().max(r.height()))
        .max()
        .unwrap_or(Dbu(1))
        .max(Dbu(1))
        .get();

    let mut bins: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (i, r) in boxes.iter().enumerate() {
        if r.width() <= Dbu::ZERO || r.height() <= Dbu::ZERO {
            continue;
        }
        for bx in r.min.x.get().div_euclid(bin)..=(r.max.x.get() - 1).div_euclid(bin) {
            for by in r.min.y.get().div_euclid(bin)..=(r.max.y.get() - 1).div_euclid(bin) {
                bins.entry((bx, by)).or_default().push(i);
            }
        }
    }

    let mut found = Vec::new();
    for (&(bx, by), members) in &bins {
        for (k, &a) in members.iter().enumerate() {
            for &b in &members[k + 1..] {
                let (ra, rb) = (boxes[a], boxes[b]);
                if !(movable[a] || movable[b]) || !ra.overlaps(&rb) {
                    continue;
                }
                let owner = (
                    ra.min.x.max(rb.min.x).get().div_euclid(bin),
                    ra.min.y.max(rb.min.y).get().div_euclid(bin),
                );
                if owner == (bx, by) {
                    found.push(PlacementViolation::Overlap(
                        CellId::new(a.min(b)),
                        CellId::new(a.max(b)),
                    ));
                }
            }
        }
    }
    found.sort_by_key(|v| match *v {
        PlacementViolation::Overlap(a, b) => (a, b),
        _ => (CellId(0), CellId(0)),
    });
    found
}
