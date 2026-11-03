//! `OpenDP`'s `optimize_mirroring`: cells with a pin on the edge of one of
//! their nets' bounding boxes are mirrored about their vertical axis where
//! that does not lengthen their nets.

use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, NetId, PinId};
use pare_common::db::library::MacroClass;

/// Nets with more cell pins than this are left as they are.
const MAX_CELL_PINS: usize = 100;

/// The bounds of points, empty until one is added, as `odb::Rect` after
/// `mergeInit`.
type Bounds = Option<(i64, i64, i64, i64)>;

fn merge(bounds: Bounds, (x, y): (i64, i64)) -> (i64, i64, i64, i64) {
    bounds.map_or((x, y, x, y), |(x0, y0, x1, y1)| {
        (x0.min(x), y0.min(y), x1.max(x), y1.max(y))
    })
}

fn bounds_hpwl(bounds: Bounds) -> i64 {
    bounds.map_or(0, |(x0, y0, x1, y1)| (x1 - x0) + (y1 - y0))
}

/// Where a pin is, as `dbITerm::getAvgXY` for cell pins and the center of
/// an I/O pin's box. `None` for an unplaced I/O pin.
fn pin_location(db: &NetlistDB, pin: PinId) -> Option<(i64, i64)> {
    match db.pins[pin.index()].owner {
        PinOwner::Cell { .. } => {
            let at = db.average_pin_position(pin);
            Some((at.x.get(), at.y.get()))
        }
        PinOwner::Io(io) => {
            let io = &db.io_pins[io.index()];
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
            Some((
                b.min.x.get().midpoint(b.max.x.get()),
                b.min.y.get().midpoint(b.max.y.get()),
            ))
        }
    }
}

/// The bounding box of a net's pins, as `dbNet::getTermBBox`.
fn term_bounds(db: &NetlistDB, net: NetId) -> Bounds {
    db.nets[net.index()]
        .pins
        .iter()
        .filter_map(|&p| pin_location(db, p))
        .fold(None, |b, p| Some(merge(b, p)))
}

/// The half-perimeter wirelength of the signal nets' pins, as
/// `odb::WireLengthEvaluator::hpwl`.
pub(super) fn hpwl(db: &NetlistDB) -> i64 {
    (0..db.nets.len())
        .filter(|&n| !db.nets[n].signal_use.is_supply())
        .map(|n| bounds_hpwl(term_bounds(db, NetId::new(n))))
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
    let ignored: Vec<bool> = db
        .nets
        .iter()
        .map(|net| {
            net.signal_use.is_supply()
                || net
                    .pins
                    .iter()
                    .filter(|&&p| matches!(db.pins[p.index()].owner, PinOwner::Cell { .. }))
                    .count()
                    > MAX_CELL_PINS
        })
        .collect();
    let mut boxes: Vec<Bounds> = (0..db.nets.len())
        .map(|n| term_bounds(db, NetId::new(n)))
        .collect();
    let mut by_length: Vec<usize> = (0..db.nets.len()).collect();
    by_length.sort_by_key(|&n| std::cmp::Reverse(bounds_hpwl(boxes[n])));
    let candidates = mirror_candidates(db, &by_length, &ignored, &boxes);
    let mut mirrored = 0;
    for cell in candidates {
        let nets = cell_nets(db, cell);
        let before = nets_hpwl(&nets, &ignored, &boxes);
        let saved: Vec<Bounds> = nets.iter().map(|n| boxes[n.index()]).collect();
        let orient = db.cells[cell.index()].orient;
        db.cells[cell.index()].orient = orient.mirrored_about_y();
        for net in &nets {
            if !ignored[net.index()] {
                boxes[net.index()] = term_bounds(db, *net);
            }
        }
        if nets_hpwl(&nets, &ignored, &boxes) > before {
            db.cells[cell.index()].orient = orient;
            for (net, saved) in nets.iter().zip(saved) {
                boxes[net.index()] = saved;
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
    nets: &[usize],
    ignored: &[bool],
    boxes: &[Bounds],
) -> Vec<CellId> {
    let mut seen = vec![false; db.cells.len()];
    let mut candidates = Vec::new();
    for &n in nets {
        if ignored[n] {
            continue;
        }
        let Some((x0, y0, x1, y1)) = boxes[n] else {
            continue;
        };
        for &pin in &db.nets[n].pins {
            let PinOwner::Cell {
                cell,
                pin: macro_pin,
            } = db.pins[pin.index()].owner
            else {
                continue;
            };
            let c = &db.cells[cell.index()];
            let movable_core = matches!(db.cell_macro(cell).class, MacroClass::Core(_))
                && !matches!(c.status, PlacementStatus::Fixed | PlacementStatus::Cover);
            if !movable_core
                || seen[cell.index()]
                || db.cell_macro(cell).pin(macro_pin).shapes.is_empty()
            {
                continue;
            }
            let Some((x, y)) = pin_location(db, pin) else {
                continue;
            };
            if x == x0 || x == x1 || y == y0 || y == y1 {
                seen[cell.index()] = true;
                candidates.push(cell);
            }
        }
    }
    candidates
}

/// The nets of a cell's pins, once per pin, as `dbInst::getITerms` lists
/// them.
fn cell_nets(db: &NetlistDB, cell: CellId) -> Vec<NetId> {
    db.cells[cell.index()]
        .pins
        .iter()
        .map(|&p| db.pins[p.index()].net)
        .collect()
}

/// `OptimizeMirroring::hpwl`: the boxes of a cell's nets, a net counted for
/// each of its pins on it.
fn nets_hpwl(nets: &[NetId], ignored: &[bool], boxes: &[Bounds]) -> i64 {
    nets.iter()
        .filter(|n| !ignored[n.index()])
        .map(|n| bounds_hpwl(boxes[n.index()]))
        .sum()
}
