//! Global and vertical swaps, as `legacy::DetailedGlobalSwap` and
//! `DetailedVerticalSwap`: each cell outside its optimal region tries a move
//! or swap toward it, kept when the wirelength does not grow.

use super::mis::converged;
use super::{EdgeId, NetworkPinId, NodeId, SegmentId};
use super::{Improver, Result, SKIP_NETS_LARGER_THAN};
use pare_common::db::indices::IdVec;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// The wirelength objective the swaps and random moves weigh, as
/// `DetailedHPWL`: each net of two to 99 pins, cached by edge.
pub(super) struct Hpwl {
    by_edge: IdVec<EdgeId, Dbu>,
    pending: Vec<EdgeId>,
}

impl Hpwl {
    pub(super) fn new(imp: &Improver) -> Self {
        Self {
            by_edge: IdVec::filled(Dbu::ZERO, imp.net.edges.len()),
            pending: Vec::new(),
        }
    }

    const fn counts(pins: usize) -> bool {
        pins > 1 && pins < SKIP_NETS_LARGER_THAN
    }

    /// The whole objective, refreshing the cache, as `DetailedHPWL::curr`.
    pub(super) fn current(&mut self, imp: &Improver) -> Dbu {
        let mut total = Dbu::ZERO;
        for (e, pins) in imp.net.edges.iter_enumerated() {
            if Self::counts(pins.len()) {
                self.by_edge[e] = imp.edge_hpwl(e);
                total += self.by_edge[e];
            }
        }
        total
    }

    /// How much the journaled trial shortens the objective, as
    /// `DetailedHPWL::delta`.
    pub(super) fn delta(&mut self, imp: &Improver) -> Dbu {
        let (mut old, mut new) = (Dbu::ZERO, Dbu::ZERO);
        for &e in &imp.journal.edges {
            if Self::counts(imp.net.edges[e].len()) {
                self.pending.push(e);
                old += self.by_edge[e];
                new += imp.edge_hpwl(e);
            }
        }
        old - new
    }

    /// Keeps the trial's wirelengths, as `DetailedHPWL::accept`.
    pub(super) fn accept(&mut self, imp: &Improver) {
        for e in self.pending.drain(..) {
            self.by_edge[e] = imp.edge_hpwl(e);
        }
    }
}

/// Global swaps, `gs -p passes -t tolerance`.
pub(super) fn run_global(imp: &mut Improver, passes: usize, tolerance: f64) -> Result<()> {
    run(imp, passes, tolerance, global_trial)
}

/// Vertical swaps, `vs -p passes -t tolerance`.
pub(super) fn run_vertical(imp: &mut Improver, passes: usize, tolerance: f64) -> Result<()> {
    run(imp, passes, tolerance, vertical_trial)
}

type Trial = fn(&mut Improver, NodeId) -> Result<bool>;

fn run(imp: &mut Improver, passes: usize, tolerance: f64, trial: Trial) -> Result<()> {
    let tolerance = tolerance.max(0.01);
    let mut hpwl = imp.total_hpwl();
    if hpwl == Dbu::ZERO {
        return Ok(());
    }
    for _ in 0..passes.max(1) {
        let last = hpwl;
        sweep(imp, trial)?;
        hpwl = imp.total_hpwl();
        if converged(last, hpwl, tolerance) {
            break;
        }
    }
    Ok(())
}

/// One trial per single-row cell in random order, each kept unless it
/// lengthens the nets, as `DetailedGlobalSwap::globalSwap` and
/// `DetailedVerticalSwap::verticalSwap`.
fn sweep(imp: &mut Improver, trial: Trial) -> Result<()> {
    imp.resort_segments();
    let mut candidates = imp.single_height.clone();
    imp.shuffle(&mut candidates);
    let mut objective = Hpwl::new(imp);
    let _ = objective.current(imp);
    for n in candidates {
        if !trial(imp, n)? {
            continue;
        }
        if objective.delta(imp) >= Dbu::ZERO {
            objective.accept(imp);
            imp.accept_move();
        } else {
            imp.reject_move()?;
        }
    }
    Ok(())
}

/// The box between the medians of the cell's nets' bounding boxes without
/// it, its pin offsets taken off, as `getRange`.
pub(super) fn optimal_region(imp: &Improver, n: NodeId) -> Option<Rect<Dbu>> {
    let (mut xs, mut ys) = (Vec::new(), Vec::new());
    let arch = &imp.arch;
    for &pin in &imp.net.node_pins[n] {
        let p = &imp.net.pins[pin];
        let edge = &imp.net.edges[p.edge];
        if edge.len() <= 1 || edge.len() > SKIP_NETS_LARGER_THAN {
            continue;
        }
        let Some(others) = others_box(imp, edge, n) else {
            continue;
        };
        xs.push((others.min.x - p.offset.x).max(arch.min_x).min(arch.max_x));
        xs.push((others.max.x - p.offset.x).min(arch.max_x).max(arch.min_x));
        ys.push((others.min.y - p.offset.y).max(arch.min_y).min(arch.max_y));
        ys.push((others.max.y - p.offset.y).min(arch.max_y).max(arch.min_y));
    }
    if xs.len() <= 1 {
        return None;
    }
    let mid = xs.len() / 2;
    xs.sort_unstable();
    ys.sort_unstable();
    Some(Rect::new(
        Point::new(xs[mid - 1], ys[mid - 1]),
        Point::new(xs[mid], ys[mid]),
    ))
}

/// The box of an edge's pins on other cells, as `calculateEdgeBB`.
fn others_box(imp: &Improver, edge: &[NetworkPinId], n: NodeId) -> Option<Rect<Dbu>> {
    let mut bounds: Option<Rect<Dbu>> = None;
    for &pin in edge {
        if imp.net.pins[pin].node == n {
            continue;
        }
        let p = imp.pin_position(pin);
        let at = Rect::new(p, p);
        bounds = Some(bounds.map_or(at, |b| b.union(&at)));
    }
    bounds
}

/// The cell's center and its optimal region, unless it is already there.
fn region_elsewhere(imp: &Improver, n: NodeId) -> Option<Rect<Dbu>> {
    let yi = imp.bottom(n).to_f64() + 0.5 * imp.height(n).to_f64();
    let xi = imp.left(n).to_f64() + 0.5 * imp.width(n).to_f64();
    let region = optimal_region(imp, n)?;
    let inside = xi >= region.min.x.to_f64()
        && xi <= region.max.x.to_f64()
        && yi >= region.min.y.to_f64()
        && yi <= region.max.y.to_f64();
    (!inside).then_some(region)
}

/// The left edge and bottom that center the cell in a box, as
/// `std::floor(0.5 * (min + max) - 0.5 * size)`.
fn centered(min: Dbu, max: Dbu, size: Dbu) -> Dbu {
    Dbu((0.5 * (min + max).to_f64() - 0.5 * size.to_f64()).floor() as i64)
}

/// The first segment of row `r` whose span holds `x`.
pub(super) fn segment_at(imp: &Improver, r: usize, x: Dbu) -> Option<SegmentId> {
    imp.segs_in_row[r].iter().copied().find(|&s| {
        let seg = &imp.segments[s];
        x >= seg.min_x && x <= seg.max_x
    })
}

/// A move or else a swap of the cell to the middle of its optimal region
/// within reach, as `legacy::DetailedGlobalSwap::generate`.
fn global_trial(imp: &mut Improver, n: NodeId) -> Result<bool> {
    let Some(region) = region_elsewhere(imp, n) else {
        return Ok(false);
    };
    let reach = imp.max_displacement;
    let (x0, x1) = clip_to_reach(region.min.x, region.max.x, imp.left(n), reach.x);
    let (y0, y1) = clip_to_reach(region.min.y, region.max.y, imp.bottom(n), reach.y);
    let [si] = imp.cell_segs[n][..] else {
        return Ok(false);
    };
    let xj = centered(x0, x1, imp.width(n));
    let yj = centered(y0, y1, imp.height(n));
    let rj = imp.arch.closest_row(yj);
    let Some(sj) = segment_at(imp, rj, xj) else {
        return Ok(false);
    };
    if imp.group(n) != imp.segments[sj].region {
        return Ok(false);
    }
    if imp.try_move(n, si, xj, sj)? {
        return Ok(true);
    }
    imp.try_swap(n, si, xj, sj)
}

/// The part of `[lo, hi]` within `reach` of `at`, or the reachable span
/// toward it when none is.
fn clip_to_reach(lo: Dbu, hi: Dbu, at: Dbu, reach: Dbu) -> (Dbu, Dbu) {
    if at + reach <= lo {
        (at, at + reach)
    } else if at - reach >= hi {
        (at - reach, at)
    } else {
        (
            if lo > at - reach { lo } else { at - reach },
            if hi < at + reach { hi } else { at + reach },
        )
    }
}

/// A move or else a swap of the cell one or two rows toward its optimal
/// region, picked at random, as `DetailedVerticalSwap::generate`.
fn vertical_trial(imp: &mut Improver, n: NodeId) -> Result<bool> {
    let yi = imp.bottom(n).to_f64() + 0.5 * imp.height(n).to_f64();
    let Some(region) = region_elsewhere(imp, n) else {
        return Ok(false);
    };
    let [si] = imp.cell_segs[n][..] else {
        return Ok(false);
    };
    let ri = imp.segments[si].row;
    let xj = centered(region.min.x, region.max.x, imp.width(n));
    let yj = centered(region.min.y, region.max.y, imp.height(n));
    let last = imp.arch.rows.len() - 1;
    let (rmin, rmax) = if yj > Dbu(i64::from(yi as i32)) {
        ((ri + 1).min(last), (ri + 2).min(last))
    } else {
        (ri.saturating_sub(2), ri.saturating_sub(1))
    };
    let rj = rmin + imp.random(rmax - rmin + 1);
    let Some(sj) = segment_at(imp, rj, xj) else {
        return Ok(false);
    };
    if imp.group(n) != imp.segments[sj].region {
        return Ok(false);
    }
    if imp.try_move(n, si, xj, sj)? {
        return Ok(true);
    }
    imp.try_swap(n, si, xj, sj)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cell_is_centered_in_a_box_rounding_down() {
        assert_eq!(centered(Dbu(0), Dbu(11), Dbu(4)), Dbu(3));
    }

    #[test]
    fn a_cell_centered_left_of_the_origin_rounds_toward_minus_infinity() {
        assert_eq!(centered(Dbu(-10), Dbu(-1), Dbu(4)), Dbu(-8));
    }
}
