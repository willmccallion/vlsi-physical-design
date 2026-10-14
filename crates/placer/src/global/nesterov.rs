//! Nesterov's method on wirelength plus weighted density energy, with the
//! step length predicted from the local Lipschitz constant and backtracking
//! when the prediction overshoots.

use super::density::{DensityGrid, Footprint};
use super::wirelength::{Netlist, Wirelength};
use super::{GlobalParams, GlobalResult, Outcome, Progress};
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// Density penalty at the start, relative to the ratio of the wirelength and
/// density gradient norms.
const INITIAL_PENALTY: f64 = 8e-5;

/// Per-iteration bounds on how much the density penalty may change.
const PENALTY_GROWTH: (f64, f64) = (0.95, 1.05);

/// Bins every net would have to stretch by in one iteration for the
/// density penalty to stop growing; large, so that it only throttles
/// runaway spreading.
const REFERENCE_STRETCH_BINS: f64 = 2.0;

/// Times one iteration may shorten its step before accepting it.
const MAX_BACKTRACKS: usize = 10;

/// What is being placed: cells first, then fillers.
#[derive(Debug)]
pub(super) struct Objects {
    /// Center of each object.
    pub(super) centers: Vec<Point<f64>>,
    /// Width and height of each object.
    pub(super) sizes: Vec<Point<f64>>,
    /// How many of the objects are cells.
    pub(super) cells: usize,
}

/// The objective and the buffers to evaluate it.
struct Objective<'a> {
    grid: &'a mut DensityGrid,
    netlist: &'a Netlist,
    footprints: Vec<Footprint>,
    areas: Vec<f64>,
    pins: Vec<f64>,
    bounds: Vec<Rect<f64>>,
    cells: usize,
    wirelength_grad: Vec<Point<f64>>,
    density_grad: Vec<Point<f64>>,
    scratch: Vec<Point<f64>>,
}

/// The state of a placement besides its gradient.
#[derive(Clone, Copy, Debug)]
struct Evaluation {
    wirelength: Wirelength,
    overflow: f64,
}

impl<'a> Objective<'a> {
    fn new(
        objects: &Objects,
        die: &Rect<f64>,
        grid: &'a mut DensityGrid,
        netlist: &'a Netlist,
    ) -> Self {
        let n = objects.centers.len();
        Self {
            footprints: objects.sizes.iter().map(|&s| grid.footprint(s)).collect(),
            areas: objects.sizes.iter().map(|s| s.x * s.y).collect(),
            pins: netlist.pin_counts(n),
            bounds: objects
                .sizes
                .iter()
                .map(|&s| center_bounds(die, s))
                .collect(),
            cells: objects.cells,
            grid,
            netlist,
            wirelength_grad: vec![Point::new(0.0, 0.0); n],
            density_grad: vec![Point::new(0.0, 0.0); n],
            scratch: Vec::new(),
        }
    }

    /// Evaluates both gradients, unweighted, at `centers`.
    fn evaluate(&mut self, centers: &[Point<f64>], gamma: f64) -> Evaluation {
        let wirelength =
            self.netlist
                .gradient(centers, gamma, &mut self.scratch, &mut self.wirelength_grad);
        let overflow = self.grid.update(centers, &self.footprints, self.cells);
        self.grid
            .gradient(centers, &self.footprints, &mut self.density_grad);
        Evaluation {
            wirelength,
            overflow,
        }
    }

    /// The preconditioned gradient of wirelength plus `penalty` times
    /// density energy, written to `out`.
    fn gradient(
        &mut self,
        centers: &[Point<f64>],
        gamma: f64,
        penalty: f64,
        out: &mut [Point<f64>],
    ) -> Evaluation {
        let eval = self.evaluate(centers, gamma);
        for (i, g) in out.iter_mut().enumerate() {
            let combined = self.wirelength_grad[i] + self.density_grad[i] * penalty;
            let scale = 1.0 / penalty.mul_add(self.areas[i], self.pins[i]).max(1.0);
            *g = combined * scale;
        }
        eval
    }

    /// The initial density penalty, from the sizes of the two gradients.
    fn initial_penalty(&self) -> f64 {
        let l1 = |v: &[Point<f64>]| v.iter().map(|g| g.x.abs() + g.y.abs()).sum::<f64>();
        let wirelength = l1(&self.wirelength_grad);
        let density = l1(&self.density_grad);
        if wirelength > 0.0 && density > 0.0 {
            INITIAL_PENALTY * wirelength / density
        } else {
            INITIAL_PENALTY
        }
    }

    fn clamp(&self, centers: &mut [Point<f64>]) {
        for (c, b) in centers.iter_mut().zip(&self.bounds) {
            *c = Point::new(c.x.clamp(b.min.x, b.max.x), c.y.clamp(b.min.y, b.max.y));
        }
    }
}

/// The range of centers that keep an object of `size` inside `die`.
fn center_bounds(die: &Rect<f64>, size: Point<f64>) -> Rect<f64> {
    let half = size * 0.5;
    let lo = die.min + half;
    let hi = die.max - half;
    let mid = Point::new(
        f64::midpoint(die.min.x, die.max.x),
        f64::midpoint(die.min.y, die.max.y),
    );
    Rect::new(
        Point::new(lo.x.min(mid.x), lo.y.min(mid.y)),
        Point::new(hi.x.max(mid.x), hi.y.max(mid.y)),
    )
}

/// Smoothing length of the wirelength model for an overflow: wide while
/// cells overlap heavily, close to exact once they have spread.
fn gamma_for(overflow: f64, bin: Point<f64>) -> f64 {
    let tau = overflow.clamp(0.1, 1.0);
    let exponent = (20.0_f64 / 9.0).mul_add(tau, -11.0 / 9.0);
    8.0 * f64::midpoint(bin.x, bin.y) * 10f64.powf(exponent)
}

/// How much to scale the density penalty after HPWL changed by `delta`,
/// against a `reference` change at which it holds steady.
fn penalty_growth(delta: f64, reference: f64) -> f64 {
    let (lo, hi) = PENALTY_GROWTH;
    if delta < 0.0 {
        hi
    } else {
        hi.powf(1.0 - delta / reference).max(lo)
    }
}

fn distance(a: &[Point<f64>], b: &[Point<f64>]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(p, q)| {
            let d = *p - *q;
            d.x.mul_add(d.x, d.y * d.y)
        })
        .sum::<f64>()
        .sqrt()
}

fn all_finite(v: &[Point<f64>]) -> bool {
    v.iter().all(|p| p.x.is_finite() && p.y.is_finite())
}

/// Minimizes wirelength plus density energy over the objects' centers,
/// leaving the best placement found in `objects.centers`.
pub(super) fn minimize(
    objects: &mut Objects,
    die: &Rect<f64>,
    grid: &mut DensityGrid,
    netlist: &Netlist,
    params: &GlobalParams,
    mut report: impl FnMut(&Progress),
) -> GlobalResult {
    let bin = grid.bin();
    let reference_growth =
        REFERENCE_STRETCH_BINS * f64::midpoint(bin.x, bin.y) * netlist.len().max(1) as f64;
    let mut objective = Objective::new(objects, die, grid, netlist);
    let n = objects.centers.len();

    let mut major = objects.centers.clone();
    objective.clamp(&mut major);
    let mut reference = major.clone();
    let mut gamma = gamma_for(1.0, bin);

    let first = objective.evaluate(&reference, gamma);
    let mut penalty = objective.initial_penalty();
    let mut grad = vec![Point::new(0.0, 0.0); n];
    let mut eval = objective.gradient(&reference, gamma, penalty, &mut grad);
    let mut step = initial_step(&mut objective, &reference, &grad, gamma, penalty, bin);

    let mut a = 1.0f64;
    let mut prev_hpwl = first.wirelength.hpwl;
    let mut next_major = vec![Point::new(0.0, 0.0); n];
    let mut next_reference = vec![Point::new(0.0, 0.0); n];
    let mut next_grad = vec![Point::new(0.0, 0.0); n];
    let mut outcome = Outcome::IterationLimit;
    let mut iterations = 0;

    for k in 0..params.max_iterations {
        iterations = k + 1;
        let a_next = f64::midpoint(1.0, (4.0 * a).mul_add(a, 1.0).sqrt());
        let momentum = (a - 1.0) / a_next;
        let mut next_eval = eval;
        let mut next_step = step;
        for attempt in 0..=MAX_BACKTRACKS {
            for i in 0..n {
                next_major[i] = reference[i] - grad[i] * step;
            }
            objective.clamp(&mut next_major);
            for i in 0..n {
                next_reference[i] = next_major[i] + (next_major[i] - major[i]) * momentum;
            }
            objective.clamp(&mut next_reference);
            next_eval = objective.gradient(&next_reference, gamma, penalty, &mut next_grad);
            let moved = distance(&next_reference, &reference);
            let changed = distance(&next_grad, &grad);
            next_step = if changed > 0.0 { moved / changed } else { step };
            if next_step >= 0.95 * step || attempt == MAX_BACKTRACKS {
                break;
            }
            step = next_step;
        }

        if !(next_step.is_finite() && all_finite(&next_major) && all_finite(&next_grad)) {
            outcome = Outcome::Diverged;
            break;
        }
        std::mem::swap(&mut major, &mut next_major);
        std::mem::swap(&mut reference, &mut next_reference);
        std::mem::swap(&mut grad, &mut next_grad);
        eval = next_eval;
        step = next_step;
        a = a_next;

        report(&Progress {
            iteration: k,
            hpwl: eval.wirelength.hpwl,
            overflow: eval.overflow,
            penalty,
            step,
        });
        if eval.overflow <= params.target_overflow {
            outcome = Outcome::Converged;
            break;
        }
        penalty *= penalty_growth(eval.wirelength.hpwl - prev_hpwl, reference_growth);
        prev_hpwl = eval.wirelength.hpwl;
        gamma = gamma_for(eval.overflow, bin);
    }

    objects.centers.copy_from_slice(&major);
    let last = objective.evaluate(&major, gamma);
    GlobalResult {
        outcome,
        iterations,
        overflow: last.overflow,
        hpwl: last.wirelength.hpwl,
    }
}

/// A first step length from the gradient change over a small probe move.
fn initial_step(
    objective: &mut Objective<'_>,
    at: &[Point<f64>],
    grad: &[Point<f64>],
    gamma: f64,
    penalty: f64,
    bin: Point<f64>,
) -> f64 {
    let largest = grad
        .iter()
        .map(|g| g.x.abs().max(g.y.abs()))
        .fold(0.0, f64::max);
    let fallback = 0.01 * f64::midpoint(bin.x, bin.y);
    if largest <= 0.0 {
        return fallback;
    }
    let scale = fallback / largest;
    let mut probe: Vec<Point<f64>> = at.iter().zip(grad).map(|(&p, &g)| p - g * scale).collect();
    objective.clamp(&mut probe);
    let mut probe_grad = vec![Point::new(0.0, 0.0); at.len()];
    let _ = objective.gradient(&probe, gamma, penalty, &mut probe_grad);
    let changed = distance(&probe_grad, grad);
    if changed > 0.0 {
        distance(&probe, at) / changed
    } else {
        fallback
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gamma_shrinks_from_eighty_bins_to_under_one_as_overflow_falls() {
        let bin = Point::new(2.0, 2.0);

        assert!((gamma_for(1.0, bin) - 160.0).abs() < 1e-9);
        assert!((gamma_for(0.1, bin) - 1.6).abs() < 1e-9);
        assert!((gamma_for(0.01, bin) - 1.6).abs() < 1e-9);
    }

    #[test]
    fn the_penalty_grows_fastest_while_hpwl_falls() {
        assert!((penalty_growth(-1.0, 100.0) - 1.05).abs() < 1e-12);
        assert!((penalty_growth(100.0, 100.0) - 1.0).abs() < 1e-12);
        assert!((penalty_growth(1e9, 100.0) - 0.95).abs() < 1e-12);
    }

    #[test]
    fn a_large_object_is_kept_centered_on_a_small_die() {
        let die = Rect::new(Point::new(0.0, 0.0), Point::new(10.0, 10.0));

        let b = center_bounds(&die, Point::new(12.0, 4.0));

        assert_eq!((b.min.x, b.max.x), (5.0, 5.0));
        assert_eq!((b.min.y, b.max.y), (2.0, 8.0));
    }
}
