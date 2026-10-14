//! Placement legality checks.

use crate::db::design::NetlistDB;
use crate::db::indices::CellId;
use crate::geom::dbu::Dbu;
use crate::geom::rect::Rect;
use crate::util::ui;
use std::collections::HashMap;

/// A placement rule a cell breaks.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlacementViolation {
    /// The cell extends past the die boundary.
    OutsideDie(CellId),
    /// Two movable cells overlap.
    Overlap(CellId, CellId),
}

/// Finds movable cells outside the die and overlapping pairs of movable cells.
pub fn placement_violations(db: &NetlistDB) -> Vec<PlacementViolation> {
    let movable: Vec<(CellId, Rect<Dbu>)> = (0..db.num_cells())
        .map(CellId::new)
        .filter(|&c| db.cells[c.index()].status.is_movable())
        .map(|c| (c, db.cell_bbox(c)))
        .collect();

    let mut violations: Vec<PlacementViolation> = movable
        .iter()
        .filter(|(_, r)| !db.die_area.encloses(r))
        .map(|&(c, _)| PlacementViolation::OutsideDie(c))
        .collect();

    let bin = movable
        .iter()
        .map(|(_, r)| r.width().max(r.height()))
        .max()
        .unwrap_or(Dbu(1))
        .max(Dbu(1))
        .get();
    let mut bins: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
    for (k, (_, r)) in movable.iter().enumerate() {
        for bx in r.min.x.get().div_euclid(bin)..=r.max.x.get().div_euclid(bin) {
            for by in r.min.y.get().div_euclid(bin)..=r.max.y.get().div_euclid(bin) {
                bins.entry((bx, by)).or_default().push(k);
            }
        }
    }
    for (&(bx, by), members) in &bins {
        for (i, &a) in members.iter().enumerate() {
            for &b in &members[i + 1..] {
                let (ra, rb) = (movable[a].1, movable[b].1);
                if !ra.overlaps(&rb) {
                    continue;
                }
                let owner = (
                    ra.min.x.max(rb.min.x).get().div_euclid(bin),
                    ra.min.y.max(rb.min.y).get().div_euclid(bin),
                );
                if owner == (bx, by) {
                    violations.push(PlacementViolation::Overlap(movable[a].0, movable[b].0));
                }
            }
        }
    }
    violations
}

/// Reports placement violations and fails if there are any.
pub fn run_placement_check(db: &NetlistDB) -> Result<(), String> {
    let violations = placement_violations(db);
    if violations.is_empty() {
        ui::check("Placement is valid.");
        return Ok(());
    }
    for v in violations.iter().take(20) {
        match *v {
            PlacementViolation::OutsideDie(c) => {
                log::error!("Cell '{}' is outside the die", db.cells[c.index()].name);
            }
            PlacementViolation::Overlap(a, b) => log::error!(
                "Cells '{}' and '{}' overlap",
                db.cells[a.index()].name,
                db.cells[b.index()].name
            ),
        }
    }
    let msg = format!(
        "Placement verification failed: {} violations",
        violations.len()
    );
    ui::fail(&msg);
    Err(msg)
}
