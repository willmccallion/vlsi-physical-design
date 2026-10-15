//! Nesterov's method on wirelength plus weighted density energy, with the
//! step length predicted from the local Lipschitz constant and backtracking
//! when the prediction overshoots. Every field weighs its own density
//! energy, and its objects stop moving once its overflow reaches the target.
//!
//! A single field's weight follows ePlace: it grows while wirelength
//! shrinks. With several fields that one signal cannot tell which field
//! still needs spreading, so their weights follow `DREAMPlace` 3.0 instead: a
//! quadratic density penalty whose weights take normalized subgradient steps
//! on each field's own energy.

use super::density::{Excess, Footprint};
use super::routability::{CHECK_OVERFLOW, CongestionMap, Routability, SNAPSHOT_OVERFLOW, Verdict};
use super::wirelength::{Netlist, Wirelength};
use super::{Field, GlobalParams, GlobalResult, Outcome, Progress, ZoneOutcome};
use crate::zones::Zone;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;

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

/// Weight of the quadratic density term against the linear one, relative to
/// each field's initial energy, as in `DREAMPlace`.
const QUADRATIC_COEFFICIENT: f64 = 2000.0;

/// Bounds of the factor the subgradient step length grows by each
/// iteration, as in `DREAMPlace`.
const SCHEDULE_STEP_GROWTH: (f64, f64) = (1.03, 1.04);

/// Largest density weight of a scheduled field, in `DREAMPlace`'s units of
/// one site width, where the weight scales as length to the minus three.
const SCHEDULED_WEIGHT_CAP: f64 = 10.0;

/// Overflow every field must be under before the preconditioner's damping
/// of large objects starts doubling, every [`DAMPING_PERIOD`] iterations up
/// to [`MAX_DAMPING`], as in `DREAMPlace`.
const DAMPING_OVERFLOW: f64 = 0.3;
const DAMPING_PERIOD: usize = 20;
const MAX_DAMPING: f64 = 1024.0;

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
    fields: &'a mut [Field],
    netlist: &'a Netlist,
    /// The field of each object.
    field_of: Vec<usize>,
    footprints: Vec<Footprint>,
    areas: Vec<f64>,
    pins: Vec<f64>,
    bounds: Vec<Rect<f64>>,
    wirelength_grad: Vec<Point<f64>>,
    density_grad: Vec<Point<f64>>,
    scratch: Vec<Point<f64>>,
}

/// The state of a placement besides its gradient.
#[derive(Clone, Debug)]
struct Evaluation {
    wirelength: Wirelength,
    /// Cell area above the target density over all cell area, every field
    /// together.
    overflow: f64,
    /// The overflow of each field.
    field_overflow: Vec<f64>,
    /// The electrostatic energy of each field.
    field_energy: Vec<f64>,
}

/// How strongly each field's density energy pulls its objects apart.
#[derive(Clone, Debug)]
struct Weights {
    /// Each field's density weight.
    penalties: Vec<f64>,
    /// Whether each field has stopped moving.
    frozen: Vec<bool>,
    /// With several fields, the schedule of the quadratic penalty.
    schedule: Option<Schedule>,
}

/// How an object's gradient is preconditioned.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Precondition {
    /// ePlace's: pins plus the field's weight times the object's area.
    Weighted,
    /// `DREAMPlace` 3.0's for a fence field: pins plus its area, unweighted,
    /// so its density force keeps growing with the weight.
    Unweighted,
}

/// `DREAMPlace` 3.0's density weights for several fields. Field `k`'s energy
/// `D` enters the objective as `λ D (1 + c D / D0)`, with `c` the
/// [`QUADRATIC_COEFFICIENT`] over two and `D0` its energy at the start;
/// `λ = u (1 + 2c D / D0)`, where `u` takes subgradient steps along the
/// normalized energies.
#[derive(Clone, Debug)]
struct Schedule {
    /// One over each field's initial energy, or zero for a field without any.
    precondition: Vec<f64>,
    /// The subgradient iterate of each field.
    u: Vec<f64>,
    /// The subgradient step length.
    step: f64,
    /// The largest weight, [`SCHEDULED_WEIGHT_CAP`] in microns.
    cap: f64,
    /// The site width, in microns: `DREAMPlace`'s unit of length, which its
    /// preconditioner is tuned for.
    site: f64,
    /// How each field's objects are preconditioned.
    kinds: Vec<Precondition>,
    /// The factor damping large objects, doubled late in placement.
    damping: f64,
}

impl Weights {
    /// ePlace's weights: one per field, all free to move.
    fn eplace(penalties: Vec<f64>) -> Self {
        let frozen = vec![false; penalties.len()];
        Self {
            penalties,
            frozen,
            schedule: None,
        }
    }

    /// What field `k`'s density gradient is multiplied by when its energy
    /// is `energy`: its weight, times the quadratic term's derivative.
    fn multiplier(&self, k: usize, energy: f64) -> f64 {
        let quadratic = self
            .schedule
            .as_ref()
            .map_or(0.0, |s| QUADRATIC_COEFFICIENT * s.precondition[k] * energy);
        self.penalties[k] * (1.0 + quadratic)
    }

    fn highest(&self) -> f64 {
        self.penalties.iter().copied().fold(0.0, f64::max)
    }

    /// The denominator of the preconditioner for an object of field `k`
    /// with `pins` pins and `area`, at least one. ePlace's weighs the area
    /// by the field's weight; with a schedule it follows `DREAMPlace`,
    /// computed in its units of one site width.
    fn precondition(&self, k: usize, pins: f64, area: f64) -> f64 {
        let Some(s) = self.schedule.as_ref() else {
            return self.penalties[k].mul_add(area, pins).max(1.0);
        };
        let scaled = match s.kinds[k] {
            Precondition::Weighted => self.penalties[k] * area * s.site,
            Precondition::Unweighted => area / (s.site * s.site),
        };
        s.damping.mul_add(scaled, pins).max(1.0)
    }

    /// Doubles the damping of large objects every [`DAMPING_PERIOD`]
    /// iterations once every field's overflow is under
    /// [`DAMPING_OVERFLOW`], up to [`MAX_DAMPING`].
    fn damp(&mut self, iteration: usize, eval: &Evaluation) {
        let Some(s) = self.schedule.as_mut() else {
            return;
        };
        let spread = eval.field_overflow.iter().all(|&o| o < DAMPING_OVERFLOW);
        if spread && s.damping < MAX_DAMPING && iteration.is_multiple_of(DAMPING_PERIOD) {
            s.damping *= 2.0;
        }
    }

    /// Logs every field's overflow, energy against its initial energy, and
    /// weight.
    fn log(&self, iteration: usize, eval: &Evaluation) {
        for (k, (&overflow, &energy)) in eval
            .field_overflow
            .iter()
            .zip(&eval.field_energy)
            .enumerate()
        {
            let relative = self
                .schedule
                .as_ref()
                .map_or(f64::NAN, |s| energy * s.precondition[k]);
            log::debug!(
                "iteration {iteration} field {k}: overflow {overflow:.3}, energy/initial {relative:.3e}, \
                 weight {:.3e}{}",
                self.penalties[k],
                if self.frozen[k] { " (frozen)" } else { "" }
            );
        }
    }

    /// Updates the weights of the fields still moving after an iteration
    /// that evaluated to `eval`, with `growth` the ePlace factor.
    fn update(&mut self, eval: &Evaluation, growth: f64) {
        let Some(schedule) = self.schedule.as_mut() else {
            for (penalty, _) in self
                .penalties
                .iter_mut()
                .zip(&self.frozen)
                .filter(|&(_, &frozen)| !frozen)
            {
                *penalty *= growth;
            }
            return;
        };
        let normalized: Vec<f64> = eval
            .field_energy
            .iter()
            .zip(&schedule.precondition)
            .map(|(d, p)| d * p)
            .collect();
        let half = QUADRATIC_COEFFICIENT / 2.0;
        let direction: Vec<f64> = normalized
            .iter()
            .map(|&n| (half * n).mul_add(n, n))
            .collect();
        let length = norm(&direction);
        if length > 0.0 {
            for (u, d) in schedule.u.iter_mut().zip(&direction) {
                *u = (schedule.step * d).mul_add(1.0 / length, *u);
            }
        }
        for (k, penalty) in self.penalties.iter_mut().enumerate() {
            if !self.frozen[k] {
                let weight = schedule.u[k] * QUADRATIC_COEFFICIENT.mul_add(normalized[k], 1.0);
                *penalty = weight.min(schedule.cap);
            }
        }
        let rate = (QUADRATIC_COEFFICIENT * norm(&normalized)).ln().max(0.0);
        let (lo, hi) = SCHEDULE_STEP_GROWTH;
        schedule.step *= (rate / (1.0 + rate)).mul_add(hi - lo, lo);
    }
}

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

impl<'a> Objective<'a> {
    /// `fields` must not be empty; they all share one bin grid.
    fn new(
        objects: &Objects,
        die: &Rect<f64>,
        fields: &'a mut [Field],
        netlist: &'a Netlist,
    ) -> Self {
        let n = objects.centers.len();
        let mut field_of = vec![0; n];
        for (k, f) in fields.iter().enumerate() {
            for i in f.cells.clone().chain(f.fillers.clone()) {
                field_of[i] = k;
            }
        }
        let footprints = objects
            .sizes
            .iter()
            .map(|&s| fields[0].grid.footprint(s))
            .collect();
        Self {
            field_of,
            footprints,
            areas: objects.sizes.iter().map(|s| s.x * s.y).collect(),
            pins: netlist.pin_counts(n),
            bounds: objects
                .sizes
                .iter()
                .map(|&s| center_bounds(die, s))
                .collect(),
            fields,
            netlist,
            wirelength_grad: vec![Point::new(0.0, 0.0); n],
            density_grad: vec![Point::new(0.0, 0.0); n],
            scratch: Vec::new(),
        }
    }

    /// Evaluates both gradients, unweighted, at `centers`: wirelength over
    /// every object, and each object's density energy in its own field.
    fn evaluate(&mut self, centers: &[Point<f64>], gamma: f64) -> Evaluation {
        let wirelength =
            self.netlist
                .gradient(centers, gamma, &mut self.scratch, &mut self.wirelength_grad);
        let footprints = &self.footprints;
        let excess: Vec<Excess> = self
            .fields
            .par_iter_mut()
            .map(|f| {
                f.grid
                    .update(centers, footprints, f.cells.clone(), f.fillers.clone())
            })
            .collect();
        let fields: &[Field] = self.fields;
        let field_of = &self.field_of;
        self.density_grad
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, g)| {
                *g = fields[field_of[i]]
                    .grid
                    .gradient_at(centers[i], &footprints[i]);
            });
        let total = excess.iter().fold(Excess::default(), |a, e| Excess {
            over: a.over + e.over,
            cells: a.cells + e.cells,
            energy: a.energy + e.energy,
        });
        Evaluation {
            wirelength,
            overflow: total.fraction(),
            field_overflow: excess.iter().map(|e| e.fraction()).collect(),
            field_energy: excess.iter().map(|e| e.energy).collect(),
        }
    }

    /// The preconditioned gradient of wirelength plus each field's weighted
    /// density energy, written to `out`; objects of frozen fields get none.
    fn gradient(
        &mut self,
        centers: &[Point<f64>],
        gamma: f64,
        weights: &Weights,
        out: &mut [Point<f64>],
    ) -> Evaluation {
        let eval = self.evaluate(centers, gamma);
        let multipliers: Vec<f64> = eval
            .field_energy
            .iter()
            .enumerate()
            .map(|(k, &energy)| weights.multiplier(k, energy))
            .collect();
        for (i, g) in out.iter_mut().enumerate() {
            let k = self.field_of[i];
            if weights.frozen[k] {
                *g = Point::new(0.0, 0.0);
                continue;
            }
            let combined = self.wirelength_grad[i] + self.density_grad[i] * multipliers[k];
            *g = combined * (1.0 / weights.precondition(k, self.pins[i], self.areas[i]));
        }
        eval
    }

    /// The L1 norms of the wirelength and density gradients over each
    /// field's objects, at the last evaluation.
    fn gradient_norms(&self) -> Vec<(f64, f64)> {
        let l1 = |g: Point<f64>| g.x.abs() + g.y.abs();
        self.fields
            .iter()
            .map(|f| {
                let objects = || f.cells.clone().chain(f.fillers.clone());
                let wirelength: f64 = objects().map(|i| l1(self.wirelength_grad[i])).sum();
                let density: f64 = objects().map(|i| l1(self.density_grad[i])).sum();
                (wirelength, density)
            })
            .collect()
    }

    /// The weights to start from after evaluating `first`: ePlace's for a
    /// single field, scaled so the density gradient starts at
    /// [`INITIAL_PENALTY`] times the wirelength gradient; `DREAMPlace`'s
    /// schedule, scaled the same way over all fields together, for several,
    /// with `site` its unit of length in microns.
    fn initial_weights(&self, first: &Evaluation, site: f64) -> Weights {
        let norms = self.gradient_norms();
        if norms.len() == 1 {
            let penalties = norms
                .iter()
                .map(|&(wirelength, density)| {
                    if wirelength > 0.0 && density > 0.0 {
                        INITIAL_PENALTY * wirelength / density
                    } else {
                        INITIAL_PENALTY
                    }
                })
                .collect();
            return Weights::eplace(penalties);
        }
        let precondition: Vec<f64> = first
            .field_energy
            .iter()
            .map(|&d| if d > 0.0 { 1.0 / d } else { 0.0 })
            .collect();
        let half = QUADRATIC_COEFFICIENT / 2.0;
        let normalized: Vec<f64> = first
            .field_energy
            .iter()
            .zip(&precondition)
            .map(|(d, p)| d * p)
            .collect();
        let mut u: Vec<f64> = normalized
            .iter()
            .map(|&n| (half * n).mul_add(n, n))
            .collect();
        let s: Vec<f64> = normalized
            .iter()
            .map(|&n| QUADRATIC_COEFFICIENT.mul_add(n, 1.0))
            .collect();
        let wirelength: f64 = norms.iter().map(|&(w, _)| w).sum();
        let density: f64 = norms
            .iter()
            .zip(u.iter().zip(&s))
            .map(|(&(_, d), (u, s))| u * s * d)
            .sum();
        if wirelength > 0.0 && density > 0.0 {
            let scale = INITIAL_PENALTY * wirelength / density;
            for x in &mut u {
                *x *= scale;
            }
        }
        let cap = SCHEDULED_WEIGHT_CAP / site.powi(3);
        let penalties = u.iter().zip(&s).map(|(u, s)| (u * s).min(cap)).collect();
        let step = (SCHEDULE_STEP_GROWTH.0 - 1.0) * norm(&u);
        let kinds = self
            .fields
            .iter()
            .map(|f| match f.zone {
                Zone::Outside => Precondition::Weighted,
                Zone::Fence(_) => Precondition::Unweighted,
            })
            .collect();
        Weights {
            frozen: vec![false; u.len()],
            penalties,
            schedule: Some(Schedule {
                precondition,
                u,
                step,
                cap,
                site,
                kinds,
                damping: 1.0,
            }),
        }
    }

    /// Whether each object may move: those of fields not yet frozen.
    fn moving(&self, frozen: &[bool]) -> Vec<bool> {
        self.field_of.iter().map(|&k| !frozen[k]).collect()
    }

    /// How each field ended, at the overflows of `eval`.
    fn outcomes(&self, eval: &Evaluation) -> Vec<ZoneOutcome> {
        self.fields
            .iter()
            .zip(&eval.field_overflow)
            .map(|(f, &overflow)| ZoneOutcome {
                zone: f.zone,
                cells: f.cells.len(),
                utilization: f.utilization,
                target: f.grid.target(),
                overflow,
            })
            .collect()
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
/// Where Nesterov's method stands: enough to resume from.
#[derive(Clone, Debug)]
struct State {
    major: Vec<Point<f64>>,
    reference: Vec<Point<f64>>,
    step: f64,
    weights: Weights,
    gamma: f64,
    a: f64,
    prev_hpwl: f64,
}

/// How one stretch of iterations ended.
enum Phase {
    /// Placement is finished.
    Finished(GlobalResult),
    /// Spreading reached the point where routability is checked.
    Paused { iteration: usize, state: State },
}

/// Minimizes wirelength plus density energy over the objects' centers,
/// leaving the best placement found in `objects.centers`. With
/// routability enabled, congestion is checked once cells have nearly
/// spread, and placement resumes from a saved state after each inflation.
pub(super) fn minimize(
    objects: &mut Objects,
    die: &Rect<f64>,
    fields: &mut [Field],
    netlist: &Netlist,
    site: f64,
    params: &GlobalParams,
    mut report: impl FnMut(&Progress),
) -> GlobalResult {
    let Some(dim) = fields.first().map(|f| f.grid.dim()) else {
        let mut grad = vec![Point::new(0.0, 0.0); objects.centers.len()];
        let wirelength = netlist.gradient(&objects.centers, 1.0, &mut Vec::new(), &mut grad);
        return GlobalResult {
            outcome: Outcome::Converged,
            iterations: 0,
            overflow: 0.0,
            hpwl: wirelength.hpwl,
            congestion: Vec::new(),
            zones: Vec::new(),
        };
    };
    let mut routability = params.routability.map(|r| Routability::new(r, objects));
    let mut saved: Option<State> = None;
    let mut resume: Option<State> = None;
    let mut first = 0;
    loop {
        let checking = routability.as_ref().is_some_and(Routability::active);
        let phase = run(
            objects,
            die,
            fields,
            netlist,
            site,
            params,
            &mut report,
            Start {
                first,
                resume: resume.take(),
                checking,
            },
            &mut saved,
        );
        match (phase, routability.as_mut(), params.routability) {
            (Phase::Finished(mut result), r, _) => {
                result.congestion = r.map(|r| std::mem::take(&mut r.rounds)).unwrap_or_default();
                return result;
            }
            (Phase::Paused { iteration, state }, Some(r), Some(p)) => {
                let map = CongestionMap::rudy(netlist, &objects.centers, die, dim, p.supply);
                let verdict = r.check(iteration, &map, objects, fields);
                resume = match verdict {
                    Verdict::Resume => saved.clone().or(Some(state)),
                    Verdict::Done => Some(state),
                };
                first = iteration + 1;
            }
            (Phase::Paused { iteration, state }, _, _) => {
                resume = Some(state);
                first = iteration + 1;
            }
        }
    }
}

/// Where a stretch of iterations starts.
struct Start {
    first: usize,
    resume: Option<State>,
    checking: bool,
}

/// Runs iterations from `start` until placement converges, the iteration
/// limit is reached, or, when `start.checking`, density overflow falls to
/// [`CHECK_OVERFLOW`]; the state at [`SNAPSHOT_OVERFLOW`] is kept in `saved`.
#[allow(clippy::too_many_arguments)]
fn run(
    objects: &mut Objects,
    die: &Rect<f64>,
    fields: &mut [Field],
    netlist: &Netlist,
    site: f64,
    params: &GlobalParams,
    report: &mut impl FnMut(&Progress),
    start: Start,
    saved: &mut Option<State>,
) -> Phase {
    let bin = fields[0].grid.bin();
    let reference_growth =
        REFERENCE_STRETCH_BINS * f64::midpoint(bin.x, bin.y) * netlist.len().max(1) as f64;
    let mut objective = Objective::new(objects, die, fields, netlist);
    let n = objects.centers.len();
    let mut grad = vec![Point::new(0.0, 0.0); n];

    let mut s = if let Some(state) = start.resume {
        restart(&mut objective, state, &mut grad, bin)
    } else {
        let mut major = objects.centers.clone();
        objective.clamp(&mut major);
        let gamma = gamma_for(1.0, bin);
        let first = objective.evaluate(&major, gamma);
        let weights = objective.initial_weights(&first, site);
        let _ = objective.gradient(&major, gamma, &weights, &mut grad);
        let step = initial_step(&mut objective, &major, &grad, gamma, &weights, bin);
        State {
            reference: major.clone(),
            major,
            step,
            weights,
            gamma,
            a: 1.0,
            prev_hpwl: first.wirelength.hpwl,
        }
    };
    let mut eval = objective.gradient(&s.reference, s.gamma, &s.weights, &mut grad);
    let mut moving = objective.moving(&s.weights.frozen);

    let mut next_major = vec![Point::new(0.0, 0.0); n];
    let mut next_reference = vec![Point::new(0.0, 0.0); n];
    let mut next_grad = vec![Point::new(0.0, 0.0); n];
    let mut outcome = Outcome::IterationLimit;
    let mut iterations = start.first;

    for k in start.first..params.max_iterations {
        iterations = k + 1;
        let a_next = f64::midpoint(1.0, (4.0 * s.a).mul_add(s.a, 1.0).sqrt());
        let momentum = (s.a - 1.0) / a_next;
        let mut next_eval = eval.clone();
        let mut next_step = s.step;
        for attempt in 0..=MAX_BACKTRACKS {
            for i in 0..n {
                next_major[i] = if moving[i] {
                    s.reference[i] - grad[i] * s.step
                } else {
                    s.major[i]
                };
            }
            objective.clamp(&mut next_major);
            for i in 0..n {
                next_reference[i] = if moving[i] {
                    next_major[i] + (next_major[i] - s.major[i]) * momentum
                } else {
                    s.major[i]
                };
            }
            objective.clamp(&mut next_reference);
            next_eval = objective.gradient(&next_reference, s.gamma, &s.weights, &mut next_grad);
            let moved = distance(&next_reference, &s.reference);
            let changed = distance(&next_grad, &grad);
            next_step = if changed > 0.0 {
                moved / changed
            } else {
                s.step
            };
            if next_step >= 0.95 * s.step || attempt == MAX_BACKTRACKS {
                break;
            }
            s.step = next_step;
        }

        if !(next_step.is_finite() && all_finite(&next_major) && all_finite(&next_grad)) {
            outcome = Outcome::Diverged;
            break;
        }
        std::mem::swap(&mut s.major, &mut next_major);
        std::mem::swap(&mut s.reference, &mut next_reference);
        std::mem::swap(&mut grad, &mut next_grad);
        eval = next_eval;
        s.step = next_step;
        s.a = a_next;

        report(&Progress {
            iteration: k,
            hpwl: eval.wirelength.hpwl,
            overflow: eval.overflow,
            penalty: s.weights.highest(),
            step: s.step,
        });
        if k.is_multiple_of(50) && log::log_enabled!(log::Level::Debug) {
            s.weights.log(k, &eval);
        }
        let mut froze = false;
        for (frozen, &overflow) in s.weights.frozen.iter_mut().zip(&eval.field_overflow) {
            if !*frozen && overflow <= params.target_overflow {
                *frozen = true;
                froze = true;
            }
        }
        if froze {
            moving = objective.moving(&s.weights.frozen);
        }
        s.weights.damp(k, &eval);
        let converged = s.weights.frozen.iter().all(|&f| f);
        if !converged {
            let growth = penalty_growth(eval.wirelength.hpwl - s.prev_hpwl, reference_growth);
            s.weights.update(&eval, growth);
            s.prev_hpwl = eval.wirelength.hpwl;
            s.gamma = gamma_for(eval.overflow, bin);
        }
        if start.checking {
            if saved.is_none() && eval.overflow <= SNAPSHOT_OVERFLOW {
                *saved = Some(s.clone());
            }
            if eval.overflow <= CHECK_OVERFLOW {
                objects.centers.copy_from_slice(&s.major);
                return Phase::Paused {
                    iteration: k,
                    state: s,
                };
            }
        }
        if converged {
            outcome = Outcome::Converged;
            break;
        }
    }

    objects.centers.copy_from_slice(&s.major);
    let last = objective.evaluate(&s.major, s.gamma);
    Phase::Finished(GlobalResult {
        outcome,
        iterations,
        overflow: last.overflow,
        hpwl: last.wirelength.hpwl,
        congestion: Vec::new(),
        zones: objective.outcomes(&last),
    })
}

/// `state` with momentum cleared and the step length estimated afresh, for
/// resuming after object sizes changed.
fn restart(
    objective: &mut Objective<'_>,
    state: State,
    grad: &mut [Point<f64>],
    bin: Point<f64>,
) -> State {
    let _ = objective.gradient(&state.major, state.gamma, &state.weights, grad);
    let step = initial_step(
        objective,
        &state.major,
        grad,
        state.gamma,
        &state.weights,
        bin,
    );
    State {
        reference: state.major.clone(),
        step,
        a: 1.0,
        ..state
    }
}

/// A first step length from the gradient change over a small probe move.
fn initial_step(
    objective: &mut Objective<'_>,
    at: &[Point<f64>],
    grad: &[Point<f64>],
    gamma: f64,
    weights: &Weights,
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
    let _ = objective.gradient(&probe, gamma, weights, &mut probe_grad);
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

    fn evaluation(field_energy: Vec<f64>) -> Evaluation {
        Evaluation {
            wirelength: Wirelength::default(),
            overflow: 0.5,
            field_overflow: vec![0.5; field_energy.len()],
            field_energy,
        }
    }

    #[test]
    fn eplace_weights_grow_by_the_given_factor_except_when_frozen() {
        let mut weights = Weights::eplace(vec![2.0, 4.0]);
        weights.frozen[1] = true;

        weights.update(&evaluation(vec![1.0, 1.0]), 1.05);

        assert!((weights.penalties[0] - 2.1).abs() < 1e-12);
        assert!((weights.penalties[1] - 4.0).abs() < 1e-12);
    }

    #[test]
    fn a_scheduled_weight_steps_along_its_normalized_energy() {
        let mut weights = Weights {
            penalties: vec![1.0, 1.0],
            frozen: vec![false, false],
            schedule: Some(Schedule {
                precondition: vec![0.1, 0.05],
                u: vec![1.0, 1.0],
                step: 0.1,
                cap: f64::MAX,
                site: 1.0,
                kinds: vec![Precondition::Weighted; 2],
                damping: 1.0,
            }),
        };

        weights.update(&evaluation(vec![10.0, 20.0]), 1.05);

        let u = 0.1f64.mul_add(std::f64::consts::FRAC_1_SQRT_2, 1.0);
        let penalty = u * 2001.0;
        assert!((weights.penalties[0] - penalty).abs() < 1e-9);
        assert!((weights.penalties[1] - penalty).abs() < 1e-9);
        let rate = (2000.0 * std::f64::consts::SQRT_2).ln();
        let growth = (rate / (1.0 + rate)).mul_add(0.01, 1.03);
        let expected_step = 0.1 * growth;
        let step = weights.schedule.map_or(0.0, |s| s.step);
        assert!((step - expected_step).abs() < 1e-12);
    }

    #[test]
    fn the_quadratic_term_scales_a_scheduled_weight_by_the_energy_left() {
        let weights = Weights {
            penalties: vec![3.0],
            frozen: vec![false],
            schedule: Some(Schedule {
                precondition: vec![0.5],
                u: vec![1.0],
                step: 0.1,
                cap: f64::MAX,
                site: 1.0,
                kinds: vec![Precondition::Weighted],
                damping: 1.0,
            }),
        };

        let expected = 3.0 * 1001.0;
        assert!((weights.multiplier(0, 1.0) - expected).abs() < 1e-9);
        assert!((weights.multiplier(0, 0.0) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn a_large_object_is_kept_centered_on_a_small_die() {
        let die = Rect::new(Point::new(0.0, 0.0), Point::new(10.0, 10.0));

        let b = center_bounds(&die, Point::new(12.0, 4.0));

        assert_eq!((b.min.x, b.max.x), (5.0, 5.0));
        assert_eq!((b.min.y, b.max.y), (2.0, 8.0));
    }
}
