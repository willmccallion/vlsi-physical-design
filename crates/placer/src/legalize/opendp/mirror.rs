//! `OpenDP`'s `optimize_mirroring`: cells mirrored about their vertical axis.
//!
//! Cells with a pin on the edge of one of their nets' bounding boxes are
//! mirrored where that does not lengthen their nets and their master allows it.

use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, IdVec, NetId, PinId};
use pare_common::db::library::MacroClass;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// Nets with more cell pins than this are left as they are.
const MAX_CELL_PINS: usize = 100;

/// The bounds of points, empty until one is added, as `odb::Rect` after
/// `mergeInit`.
type Bounds = Option<Rect<Dbu>>;

fn merge(bounds: Bounds, p: Point<Dbu>) -> Rect<Dbu> {
    let at = Rect::new(p, p);
    bounds.map_or(at, |b| b.union(&at))
}

fn bounds_hpwl(bounds: Bounds) -> Dbu {
    bounds.map_or(Dbu::ZERO, |b| b.width() + b.height())
}

/// Where a pin is, as `dbITerm::getAvgXY` for cell pins and the center of
/// an I/O pin's box. `None` for an unplaced I/O pin.
fn pin_location(db: &NetlistDB, pin: PinId) -> Option<Point<Dbu>> {
    match db.pins[pin].owner {
        PinOwner::Cell { .. } => Some(db.average_pin_position(pin)),
        PinOwner::Io(io) => {
            let io = &db.io_pins[io];
            let placed = matches!(
                io.status,
                PlacementStatus::Placed | PlacementStatus::Fixed | PlacementStatus::Cover
            );
            if !placed {
                return None;
            }
            let b = io
                .placed_shapes()
                .map(|s| s.rect)
                .reduce(|a, b| a.union(&b))?;
            Some(Point::new(
                Dbu(b.min.x.get().midpoint(b.max.x.get())),
                Dbu(b.min.y.get().midpoint(b.max.y.get())),
            ))
        }
    }
}

/// The bounding box of a net's pins, as `dbNet::getTermBBox`.
fn term_bounds(db: &NetlistDB, net: NetId) -> Bounds {
    db.nets[net]
        .pins
        .iter()
        .filter_map(|&p| pin_location(db, p))
        .fold(None, |b, p| Some(merge(b, p)))
}

/// The half-perimeter wirelength of the signal nets' pins, as
/// `odb::WireLengthEvaluator::hpwl`.
pub(super) fn hpwl(db: &NetlistDB) -> Dbu {
    db.nets
        .ids()
        .filter(|&n| !db.nets[n].signal_use.is_supply())
        .map(|n| bounds_hpwl(term_bounds(db, n)))
        .sum()
}

/// Mirrors cells about their vertical axis where that does not lengthen
/// their nets, as `optimize_mirroring`, returning how many it mirrored.
///
/// The candidates are the movable core cells with a pin on the edge of a
/// net's box, nets taken from the longest down. `OpenDP` takes nets of equal
/// length in the order of their addresses in memory; PARE takes them in
/// the design's order.
pub fn optimize_mirroring(db: &mut NetlistDB) -> usize {
    let ignored: IdVec<NetId, bool> = db
        .nets
        .iter()
        .map(|net| {
            net.signal_use.is_supply()
                || net
                    .pins
                    .iter()
                    .filter(|&&p| matches!(db.pins[p].owner, PinOwner::Cell { .. }))
                    .count()
                    > MAX_CELL_PINS
        })
        .collect();
    let mut boxes: IdVec<NetId, Bounds> = db.nets.ids().map(|n| term_bounds(db, n)).collect();
    let mut by_length: Vec<NetId> = db.nets.ids().collect();
    by_length.sort_by_key(|&n| std::cmp::Reverse(bounds_hpwl(boxes[n])));
    let candidates = mirror_candidates(db, &by_length, &ignored, &boxes);
    let mut mirrored = 0;
    for cell in candidates {
        let nets = cell_nets(db, cell);
        let before = nets_hpwl(&nets, &ignored, &boxes);
        let saved: Vec<Bounds> = nets.iter().map(|&n| boxes[n]).collect();
        let orient = db.cells[cell].orient;
        db.cells[cell].orient = orient.mirrored_about_y();
        for &net in &nets {
            if !ignored[net] {
                boxes[net] = term_bounds(db, net);
            }
        }
        if nets_hpwl(&nets, &ignored, &boxes) > before {
            db.cells[cell].orient = orient;
            for (&net, saved) in nets.iter().zip(saved) {
                boxes[net] = saved;
            }
        } else {
            mirrored += 1;
        }
    }
    mirrored
}

/// `OptimizeMirroring::findMirrorCandidates`.
fn mirror_candidates(
    db: &NetlistDB,
    nets: &[NetId],
    ignored: &IdVec<NetId, bool>,
    boxes: &IdVec<NetId, Bounds>,
) -> Vec<CellId> {
    let mut seen = IdVec::filled(false, db.cells.len());
    let mut candidates = Vec::new();
    for &n in nets {
        if ignored[n] {
            continue;
        }
        let Some(bounds) = boxes[n] else {
            continue;
        };
        for &pin in &db.nets[n].pins {
            let PinOwner::Cell {
                cell,
                pin: macro_pin,
            } = db.pins[pin].owner
            else {
                continue;
            };
            if !may_mirror(db, cell)
                || seen[cell]
                || db.cell_macro(cell).pin(macro_pin).shapes.is_empty()
            {
                continue;
            }
            let Some(at) = pin_location(db, pin) else {
                continue;
            };
            if at.x == bounds.min.x
                || at.x == bounds.max.x
                || at.y == bounds.min.y
                || at.y == bounds.max.y
            {
                seen[cell] = true;
                candidates.push(cell);
            }
        }
    }
    candidates
}

/// Whether `cell` is a movable core cell whose master may be mirrored
/// about the vertical axis.
fn may_mirror(db: &NetlistDB, cell: CellId) -> bool {
    let c = &db.cells[cell];
    let master = db.cell_macro(cell);
    matches!(master.class, MacroClass::Core(_))
        && !matches!(c.status, PlacementStatus::Fixed | PlacementStatus::Cover)
        // NOTE: diverges from OpenDP, which mirrors without asking the master.
        && master.symmetry.allows(c.orient.mirrored_about_y())
}

/// The nets of a cell's pins, once per pin, as `dbInst::getITerms` lists
/// them.
fn cell_nets(db: &NetlistDB, cell: CellId) -> Vec<NetId> {
    db.cells[cell]
        .pins
        .iter()
        .map(|&p| db.pins[p].net)
        .collect()
}

/// `OptimizeMirroring::hpwl`: the boxes of a cell's nets, a net counted for
/// each of its pins on it.
fn nets_hpwl(nets: &[NetId], ignored: &IdVec<NetId, bool>, boxes: &IdVec<NetId, Bounds>) -> Dbu {
    nets.iter()
        .filter(|&&n| !ignored[n])
        .map(|&n| bounds_hpwl(boxes[n]))
        .sum()
}
