//! Orienting and flipping, as `DetailedOrient` running `orient -f`: each
//! cell is turned to suit its row, then mirrored about its vertical axis
//! where that shortens its nets and breaks no placement rule.

use super::{Improver, SKIP_NETS_LARGER_THAN};
use pare_common::db::library::Symmetry;
use pare_common::geom::orient::Orientation;

pub(super) fn run(imp: &mut Improver) {
    if imp.total_hpwl() == 0 {
        return;
    }
    let problems = orient_cells(imp);
    if problems > 0 {
        log::warn!("{problems} cells could not be oriented for their rows");
    }
    flip_cells(imp);
}

/// Turns each movable cell to suit the bottom row it lies in, counting the
/// ones it cannot, as `DetailedOrient::orientCells`.
fn orient_cells(imp: &mut Improver) -> usize {
    let mut problems = 0;
    for n in 0..imp.cell_count() {
        if imp.is_fixed(n) {
            continue;
        }
        let Some(row) = imp.cell_segs[n].iter().map(|&s| imp.segments[s].row).min() else {
            problems += 1;
            continue;
        };
        let oriented = if imp.rows_spanned(n) == 1 {
            orient_single_row_cell(imp, n, row)
        } else {
            row + imp.rows_spanned(n) <= imp.arch.rows.len()
                && legal_symmetry(imp.grid.nodes[n].symmetry, imp.grid.nodes[n].orient)
        };
        if !oriented {
            problems += 1;
        }
    }
    problems
}

/// `DetailedOrient::orientSingleHeightCellForRow`: a cell upside down for
/// its row is turned over, mirrored ones only where the master allows.
fn orient_single_row_cell(imp: &mut Improver, n: usize, row: usize) -> bool {
    use Orientation::{FN, FS, N, S};
    let symmetry = imp.grid.nodes[n].symmetry;
    let cell = imp.grid.nodes[n].orient;
    let to = match (imp.arch.rows[row].orient, cell) {
        (N | FN, N | FN) | (FS | S, FS | S) => return legal_symmetry(symmetry, cell),
        (N | FN, FS) => N,
        (N | FN, S) => FN,
        (FS | S, N) => FS,
        (FS | S, FN) => S,
        _ => return false,
    };
    if !legal_symmetry(symmetry, to) {
        return false;
    }
    imp.orient_node(n, to);
    true
}

/// Whether a master's symmetry allows an orientation, as
/// `DetailedOrient::isLegalSym`.
const fn legal_symmetry(symmetry: Symmetry, orient: Orientation) -> bool {
    match orient {
        Orientation::N => true,
        Orientation::FS => symmetry.x,
        Orientation::FN => symmetry.y,
        Orientation::S => symmetry.x && symmetry.y,
        Orientation::W | Orientation::E => symmetry.r90,
        Orientation::FW | Orientation::FE => symmetry.r90 && symmetry.x && symmetry.y,
    }
}

/// Mirrors single-row cells about their vertical axis where the rows allow
/// it and it shortens their nets' horizontal spans, as
/// `DetailedOrient::flipCells`.
#[allow(clippy::cast_precision_loss)] // as OpenDP's doubles
fn flip_cells(imp: &mut Improver) {
    let mut marks = vec![0_usize; imp.net.edges.len()];
    let mut traversal = 0;
    for s in 0..imp.segments.len() {
        let seg = imp.segments[s];
        if !imp.arch.rows[seg.row].symmetry.y {
            continue;
        }
        let count = imp.cells_in_seg[s].len();
        for i in 0..count {
            let n = imp.cells_in_seg[s][i];
            if imp.rows_spanned(n) != 1 {
                continue;
            }
            traversal += 1;
            let (mut old, mut new) = (0.0, 0.0);
            for &pin in &imp.net.node_pins[n] {
                let e = imp.net.pins[pin].edge;
                let pins = &imp.net.edges[e];
                if pins.len() <= 1 || pins.len() >= SKIP_NETS_LARGER_THAN || marks[e] == traversal {
                    continue;
                }
                marks[e] = traversal;
                let (mut old_lo, mut old_hi) = (f64::MAX, f64::MIN);
                let (mut new_lo, mut new_hi) = (f64::MAX, f64::MIN);
                for &p in pins {
                    let other = &imp.net.pins[p];
                    let center = imp.left(other.node) as f64 + 0.5 * imp.width(other.node) as f64;
                    let x = center + other.offset.0 as f64;
                    old_lo = old_lo.min(x);
                    old_hi = old_hi.max(x);
                    let x = if other.node == n {
                        center - other.offset.0 as f64
                    } else {
                        x
                    };
                    new_lo = new_lo.min(x);
                    new_hi = new_hi.max(x);
                }
                old += old_hi - old_lo;
                new += new_hi - new_lo;
            }
            if new >= old {
                continue;
            }
            let cells = &imp.cells_in_seg[s];
            let lx = i.checked_sub(1).map_or(seg.min_x, |p| imp.right(cells[p]));
            let rx = cells.get(i + 1).map_or(seg.max_x, |&r| imp.left(r));
            if imp.left(n) < lx || imp.right(n) > rx {
                continue;
            }
            let from = imp.grid.nodes[n].orient;
            let to = match from {
                Orientation::N => Orientation::FN,
                Orientation::S => Orientation::FS,
                Orientation::FN => Orientation::N,
                Orientation::FS => Orientation::S,
                _ => continue,
            };
            imp.orient_node(n, to);
            if imp.has_placement_violation(n) {
                imp.orient_node(n, from);
            }
        }
    }
}
