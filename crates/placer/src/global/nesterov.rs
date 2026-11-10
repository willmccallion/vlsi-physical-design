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

use super::density::{Charge, Excess, Footprint};
use super::routability::{
    CHECK_OVERFLOW, CongestionMap, Routability, RoutabilityParams, SETTLE_FRACTION,
    SNAPSHOT_OVERFLOW, Verdict,
};
use super::wirelength::{Netlist, Wirelength};
use super::{
    Field, FieldId, GlobalParams, GlobalPlacementError, GlobalResult, ObjectId, ObjectPoints,
    ObjectRange, Outcome, Probe, Progress, Remap, Setup, TimingDriver, ZoneOutcome,
};
use crate::model::{CellKind, PlacementModel};
use crate::zones::Zone;
use pare_common::config::Share;
use pare_common::db::indices::{CellId, IdVec};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rand::rngs::StdRng;
use rayon::prelude::*;

/// Density penalty at the start, relative to the ratio of the wirelength and
/// density gradient norms, `gpl`'s `initDensityPenaltyFactor` of 0.00008.
const INITIAL_PENALTY: f64 = 8e-5;

/// Per-iteration bounds on how much the density penalty may change, `gpl`'s
/// `minPhiCoef` and `maxPhiCoef` of 0.95 and 1.05.
const PENALTY_GROWTH: (f64, f64) = (0.95, 1.05);

/// The growth in wirelength over one iteration at which the density penalty
/// holds steady, `gpl`'s default `-reference_hpwl`.
const REFERENCE_HPWL: Dbu = Dbu(446_000_000);

/// Overflow below which the penalty's highest growth is lowered, once, as
/// `gpl`'s `nesterovAdjustPhi` lowers it below 0.35.
const HOLD_GROWTH_OVERFLOW: f64 = 0.35;

/// What that lowers the highest growth by, `nesterovAdjustPhi`'s 0.99.
const HOLD_GROWTH_FACTOR: f64 = 0.99;

/// Trial steps one iteration may evaluate, `gpl`'s `maxBackTrack` of 10.
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
    pub(super) centers: ObjectPoints,
    /// Width and height of each object.
    pub(super) sizes: ObjectPoints,
    /// The kind of each cell, the first objects.
    pub(super) kinds: IdVec<ObjectId, CellKind>,
}

impl Objects {
    /// The objects that are cells, the first ones.
    pub(super) const fn cells(&self) -> ObjectRange {
        ObjectRange::new(ObjectId(0), ObjectId::new(self.kinds.len()))
    }
}

/// The objective and the buffers to evaluate it.
struct Objective<'a> {
    fields: &'a mut IdVec<FieldId, Field>,
    netlist: &'a Netlist,
    die: Rect<f64>,
    /// The field of each object.
    field_of: IdVec<ObjectId, FieldId>,
    /// Whether each object is in play: a cell, or a filler not removed.
    active: IdVec<ObjectId, bool>,
    footprints: IdVec<ObjectId, Footprint>,
    areas: IdVec<ObjectId, f64>,
    pins: IdVec<ObjectId, f64>,
    bounds: IdVec<ObjectId, Rect<f64>>,
    wirelength_grad: ObjectPoints,
    density_grad: ObjectPoints,
    /// The wirelength gradient of each terminal.
    scratch: Vec<Point<f64>>,
}

/// A value for each field.
type FieldValues = IdVec<FieldId, f64>;

/// The state of a placement besides its gradient.
#[derive(Clone, Debug)]
struct Evaluation {
    wirelength: Wirelength,
    /// Cell area above the target density over all cell area, every field
    /// together.
    overflow: f64,
    /// The overflow of each field.
    field_overflow: FieldValues,
    /// The electrostatic energy of each field.
    field_energy: FieldValues,
}

/// How strongly each field's density energy pulls its objects apart.
#[derive(Clone, Debug)]
struct Weights {
    /// Each field's density weight.
    penalties: FieldValues,
    /// Whether each field has stopped moving.
    frozen: IdVec<FieldId, bool>,
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
    precondition: FieldValues,
    /// The subgradient iterate of each field.
    u: FieldValues,
    /// The subgradient step length.
    step: f64,
    /// The largest weight, [`SCHEDULED_WEIGHT_CAP`] in microns.
    cap: f64,
    /// The site width, in microns: `DREAMPlace`'s unit of length, which its
    /// preconditioner is tuned for.
    site: f64,
    /// How each field's objects are preconditioned.
    kinds: IdVec<FieldId, Precondition>,
    /// The factor damping large objects, doubled late in placement.
    damping: f64,
}

impl Weights {
    /// ePlace's weights: one per field, all free to move.
    fn eplace(penalties: FieldValues) -> Self {
        let frozen = IdVec::filled(false, penalties.len());
        Self {
            penalties,
            frozen,
            schedule: None,
        }
    }

    /// What field `k`'s density gradient is multiplied by when its energy
    /// is `energy`: its weight, times the quadratic term's derivative.
    fn multiplier(&self, k: FieldId, energy: f64) -> f64 {
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
    fn precondition(&self, k: FieldId, pins: f64, area: f64) -> f64 {
        let Some(s) = self.schedule.as_ref() else {
            return self.penalties[k].mul_add(area, pins).max(1.0);
        };
        let scaled = match s.kinds[k] {
            Precondition::Weighted => self.penalties[k] * area * s.site,
            Precondition::Unweighted => area / (s.site * s.site),
        };
        s.damping.mul_add(scaled, pins).max(1.0)
    }

    /// Rescales the schedule's energy reference of every field still moving
    /// by `before` over `after`, the fields' density energies at one
    /// placement before and after their cells changed size, so that the
    /// schedule reads the same normalized energy there as it did before the
    /// change and goes on as it would have. Nothing changes when the
    /// energies are equal.
    fn rescale_energy(&mut self, before: &FieldValues, after: &FieldValues) {
        let Some(s) = self.schedule.as_mut() else {
            return;
        };
        for ((k, &b), &a) in before.iter_enumerated().zip(after) {
            if self.frozen[k] || b <= 0.0 || a <= 0.0 {
                continue;
            }
            s.precondition[k] *= b / a;
        }
    }

    /// Carries the weights to the fields of a rebuilt setup: a field with
    /// no old counterpart starts at the mean weight, unfrozen.
    fn remap(&mut self, old_field_of_new: &IdVec<FieldId, Option<FieldId>>) {
        let mean_penalty = mean(&self.penalties);
        self.penalties = remap_fields(&self.penalties, old_field_of_new, mean_penalty);
        self.frozen = old_field_of_new
            .iter()
            .map(|o| o.is_some_and(|k| self.frozen[k]))
            .collect();
        if let Some(s) = self.schedule.as_mut() {
            let mean_u = mean(&s.u);
            s.precondition = remap_fields(&s.precondition, old_field_of_new, 0.0);
            s.u = remap_fields(&s.u, old_field_of_new, mean_u);
            s.kinds = remap_fields(&s.kinds, old_field_of_new, Precondition::Weighted);
        }
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
        for ((k, &overflow), &energy) in eval
            .field_overflow
            .iter_enumerated()
            .zip(&eval.field_energy)
        {
            let relative = self
                .schedule
                .as_ref()
                .map_or(f64::NAN, |s| energy * s.precondition[k]);
            log::debug!(
                "iteration {iteration} field {}: overflow {overflow:.3}, energy/initial {relative:.3e}, \
                 weight {:.3e}{}",
                k.index(),
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
        let normalized: FieldValues = eval
            .field_energy
            .iter()
            .zip(&schedule.precondition)
            .map(|(d, p)| d * p)
            .collect();
        let half = QUADRATIC_COEFFICIENT / 2.0;
        let direction: FieldValues = normalized
            .iter()
            .map(|&n| (half * n).mul_add(n, n))
            .collect();
        let length = norm(&direction);
        if length > 0.0 {
            for (u, d) in schedule.u.iter_mut().zip(&direction) {
                *u = (schedule.step * d).mul_add(1.0 / length, *u);
            }
        }
        for (k, penalty) in self.penalties.iter_enumerated_mut() {
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

fn norm(v: &FieldValues) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

impl<'a> Objective<'a> {
    /// `fields` must not be empty; they all share one bin grid.
    fn new(
        objects: &Objects,
        die: &Rect<f64>,
        fields: &'a mut IdVec<FieldId, Field>,
        netlist: &'a Netlist,
    ) -> Self {
        let n = objects.centers.len();
        let mut field_of = IdVec::filled(FieldId(0), n);
        for (k, f) in fields.iter_enumerated() {
            for i in f.objects() {
                field_of[i] = k;
            }
        }
        let mut objective = Self {
            die: *die,
            field_of,
            active: IdVec::new(),
            footprints: IdVec::new(),
            areas: IdVec::new(),
            pins: netlist.pin_counts(n),
            bounds: IdVec::new(),
            fields,
            netlist,
            wirelength_grad: IdVec::filled(Point::new(0.0, 0.0), n),
            density_grad: IdVec::filled(Point::new(0.0, 0.0), n),
            scratch: Vec::new(),
        };
        objective.refresh(objects);
        objective
    }

    /// Takes in the objects' sizes and the fillers each field keeps.
    fn refresh(&mut self, objects: &Objects) {
        let grid = &self.fields[FieldId(0)].grid;
        self.footprints = objects
            .sizes
            .iter_enumerated()
            .map(|(i, &size)| grid.footprint(size, charge_of(objects.kinds.get(i))))
            .collect();
        self.areas = objects.sizes.iter().map(|s| s.x * s.y).collect();
        self.bounds = objects
            .sizes
            .iter()
            .map(|&s| center_bounds(&self.die, s))
            .collect();
        self.active = IdVec::filled(false, objects.centers.len());
        for f in self.fields.iter() {
            for i in f.live_objects() {
                self.active[i] = true;
            }
        }
    }

    /// Evaluates both gradients, unweighted, at `centers`: wirelength over
    /// every object, and each object's density energy in its own field.
    fn evaluate(&mut self, centers: &ObjectPoints, gamma: f64) -> Evaluation {
        let wirelength =
            self.netlist
                .gradient(centers, gamma, &mut self.scratch, &mut self.wirelength_grad);
        let footprints = &self.footprints;
        let excess: IdVec<FieldId, Excess> = self
            .fields
            .par_iter_enumerated_mut()
            .map(|(_, f)| {
                f.grid
                    .update(centers, footprints, f.cells, f.live_fillers())
            })
            .collect::<Vec<Excess>>()
            .into();
        let fields: &IdVec<FieldId, Field> = self.fields;
        let field_of = &self.field_of;
        self.density_grad
            .par_iter_enumerated_mut()
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
    /// density energy, written to `out`, as `gpl`'s `updateGradients`;
    /// objects of frozen fields and removed fillers get none.
    fn gradient(
        &mut self,
        centers: &ObjectPoints,
        gamma: f64,
        weights: &Weights,
        out: &mut ObjectPoints,
    ) -> Evaluation {
        let eval = self.evaluate(centers, gamma);
        let multipliers: FieldValues = eval
            .field_energy
            .iter_enumerated()
            .map(|(k, &energy)| weights.multiplier(k, energy))
            .collect();
        for (i, g) in out.iter_enumerated_mut() {
            let k = self.field_of[i];
            if weights.frozen[k] || !self.active[i] {
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
    fn gradient_norms(&self) -> IdVec<FieldId, (f64, f64)> {
        let l1 = |g: Point<f64>| g.x.abs() + g.y.abs();
        self.fields
            .iter()
            .map(|f| {
                let wirelength: f64 = f.live_objects().map(|i| l1(self.wirelength_grad[i])).sum();
                let density: f64 = f.live_objects().map(|i| l1(self.density_grad[i])).sum();
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
        let precondition: FieldValues = first
            .field_energy
            .iter()
            .map(|&d| if d > 0.0 { 1.0 / d } else { 0.0 })
            .collect();
        let half = QUADRATIC_COEFFICIENT / 2.0;
        let normalized: FieldValues = first
            .field_energy
            .iter()
            .zip(&precondition)
            .map(|(d, p)| d * p)
            .collect();
        let mut u: FieldValues = normalized
            .iter()
            .map(|&n| (half * n).mul_add(n, n))
            .collect();
        let s: FieldValues = normalized
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
            frozen: IdVec::filled(false, u.len()),
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

    /// Whether each object may move: those in play of fields not yet frozen.
    fn moving(&self, frozen: &IdVec<FieldId, bool>) -> IdVec<ObjectId, bool> {
        self.field_of
            .iter()
            .zip(&self.active)
            .map(|(&k, &active)| active && !frozen[k])
            .collect()
    }

    /// How far each field's objects in play moved from `before` to `after`:
    /// the root mean square change of their coordinates.
    fn displacements(&self, before: &ObjectPoints, after: &ObjectPoints) -> FieldValues {
        self.fields
            .iter()
            .map(|f| {
                let mut sum = 0.0;
                let mut count = 0;
                for i in f.live_objects() {
                    let d = after[i] - before[i];
                    sum = d.x.mul_add(d.x, d.y.mul_add(d.y, sum));
                    count += 1;
                }
                if count == 0 {
                    0.0
                } else {
                    (sum / (2 * count) as f64).sqrt()
                }
            })
            .collect()
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

    fn clamp(&self, centers: &mut ObjectPoints) {
        for (c, b) in centers.iter_mut().zip(&self.bounds) {
            *c = Point::new(c.x.clamp(b.min.x, b.max.x), c.y.clamp(b.min.y, b.max.y));
        }
    }
}

/// How much of an object's area its bins count: a movable macro's only to
/// the target density; a filler, which has no kind, all of it.
const fn charge_of(kind: Option<&CellKind>) -> Charge {
    match kind {
        Some(CellKind::Macro) => Charge::AtTarget,
        Some(CellKind::Standard) | None => Charge::Whole,
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
/// cells overlap heavily, close to exact once they have spread. The four
/// bins are `OpenROAD`'s `initWireLengthCoef` of 0.25, ePlace's being eight.
fn gamma_for(overflow: f64, bin: Point<f64>) -> f64 {
    let tau = overflow.clamp(0.1, 1.0);
    let exponent = (20.0_f64 / 9.0).mul_add(tau, -11.0 / 9.0);
    4.0 * f64::midpoint(bin.x, bin.y) * 10f64.powf(exponent)
}

/// How much to scale the density penalty after HPWL changed by `delta`,
/// against a `reference` change at which it holds steady, as `getPhiCoef`.
fn penalty_growth(delta: f64, reference: f64, highest: f64) -> f64 {
    if delta < 0.0 {
        highest
    } else {
        highest.powf(1.0 - delta / reference).max(PENALTY_GROWTH.0)
    }
}

/// The highest growth of the density penalty over a run.
#[derive(Clone, Copy, Debug, PartialEq)]
struct GrowthLimit {
    highest: f64,
    held: bool,
}

impl GrowthLimit {
    const fn new() -> Self {
        Self {
            highest: PENALTY_GROWTH.1,
            held: false,
        }
    }

    /// Lowers the limit the first time `overflow` falls far enough, as
    /// `nesterovAdjustPhi`. A revert to a snapshot keeps it lowered.
    fn observe(&mut self, overflow: f64) {
        if !self.held && overflow < HOLD_GROWTH_OVERFLOW {
            self.held = true;
            self.highest *= HOLD_GROWTH_FACTOR;
        }
    }
}

fn distance(a: &ObjectPoints, b: &ObjectPoints) -> f64 {
    a.iter()
        .zip(b)
        .map(|(p, q)| {
            let d = *p - *q;
            d.x.mul_add(d.x, d.y * d.y)
        })
        .sum::<f64>()
        .sqrt()
}

fn all_finite(v: &ObjectPoints) -> bool {
    v.iter().all(|p| p.x.is_finite() && p.y.is_finite())
}
/// Where Nesterov's method stands: enough to resume from.
#[derive(Clone, Debug)]
struct State {
    major: ObjectPoints,
    reference: ObjectPoints,
    step: f64,
    weights: Weights,
    gamma: f64,
    a: f64,
    prev_hpwl: f64,
}

/// The state routability resumes from, with each field's density energy
/// there as the cells were sized when it was saved.
#[derive(Clone, Debug)]
struct Snapshot {
    state: State,
    energy: FieldValues,
}

/// Each field's displacement in its last iteration and the largest it has
/// had, which tell when placement has settled.
// NOTE: gpl's routabilitySnapshot measures congestion as soon as overflow
// reaches routability_end_overflow; waiting to settle is PARE's own gate.
#[derive(Clone, Debug)]
struct Settling {
    last: FieldValues,
    peak: FieldValues,
}

impl Settling {
    fn new(fields: usize) -> Self {
        Self {
            last: IdVec::filled(0.0, fields),
            peak: IdVec::filled(0.0, fields),
        }
    }

    /// Records the `displacements` of the fields not `frozen`.
    fn observe(&mut self, displacements: &FieldValues, frozen: &IdVec<FieldId, bool>) {
        for (k, &d) in displacements.iter_enumerated() {
            if !frozen[k] {
                self.last[k] = d;
                self.peak[k] = self.peak[k].max(d);
            }
        }
    }

    /// Carries the record to the fields of a rebuilt setup.
    fn remap(&mut self, old_field_of_new: &IdVec<FieldId, Option<FieldId>>) {
        self.last = remap_fields(&self.last, old_field_of_new, 0.0);
        self.peak = remap_fields(&self.peak, old_field_of_new, 0.0);
    }

    /// Whether every field last moved at most [`SETTLE_FRACTION`] of its
    /// peak.
    fn settled(&self) -> bool {
        self.last
            .iter()
            .zip(&self.peak)
            .all(|(&d, &peak)| peak > 0.0 && d <= SETTLE_FRACTION * peak)
    }
}

/// The state to resume from after routability changed object sizes: the
/// `snapshot` for every field still moving, as `OpenROAD` does not revert a
/// region that has converged, and the HPWL the penalty last saw.
fn revert(snapshot: &State, current: &State, fields: &IdVec<FieldId, Field>) -> State {
    let mut s = snapshot.clone();
    s.prev_hpwl = current.prev_hpwl;
    for (k, f) in fields.iter_enumerated() {
        if !current.weights.frozen[k] {
            continue;
        }
        for i in f.objects() {
            s.major[i] = current.major[i];
            s.reference[i] = current.reference[i];
        }
        s.weights.frozen[k] = true;
        s.weights.penalties[k] = current.weights.penalties[k];
        if let (Some(to), Some(from)) = (
            s.weights.schedule.as_mut(),
            current.weights.schedule.as_ref(),
        ) {
            to.u[k] = from.u[k];
        }
    }
    s
}

fn mean(values: &FieldValues) -> f64 {
    values.iter().sum::<f64>() / values.len().max(1) as f64
}

/// The iterations routability gives back to the run while it waits for
/// spreading to finish, up to the run's own budget each round.
#[derive(Clone, Copy, Debug)]
struct Refund {
    given: usize,
    cap: usize,
}

impl Refund {
    const fn new(cap: usize) -> Self {
        Self { given: 0, cap }
    }

    /// Gives back one iteration, unless this round has given back its cap.
    // NOTE: gpl's doNesterovPlace raises maxNesterovIter without bound while
    // overflow exceeds routability_end_overflow; capped so a stalled spread ends.
    const fn take(&mut self) -> bool {
        if self.given >= self.cap {
            return false;
        }
        self.given += 1;
        true
    }

    /// Starts a routability round with its whole cap to give.
    const fn start_round(&mut self) {
        self.given = 0;
    }
}

/// Tries `step`, then each shorter step the last trial suggests, until a
/// trial suggests more than 95% of the step it tried or `MAX_BACKTRACKS`
/// trials have run, as `gpl`'s `doBackTracking`; returns the step the last
/// trial suggests.
fn backtrack_steps(step: f64, mut trial: impl FnMut(f64) -> f64) -> f64 {
    let mut step = step;
    for _ in 0..MAX_BACKTRACKS {
        let suggested = trial(step);
        if suggested > 0.95 * step {
            return suggested;
        }
        step = suggested;
    }
    step
}

/// Why a stretch of iterations ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Flow {
    /// The run is over.
    Done,
    /// A timing-driven repair is due at the objects' present positions.
    Timing,
}

/// Where a run stands between stretches of iterations, each carried out
/// with the objective of the setup at hand.
struct Run {
    s: Option<State>,
    grad: ObjectPoints,
    next_major: ObjectPoints,
    next_reference: ObjectPoints,
    next_grad: ObjectPoints,
    settling: Settling,
    snapshot: Option<Snapshot>,
    routability: Option<Routability>,
    limit: usize,
    refund: Refund,
    /// The wirelength growth at which the density penalty holds steady.
    reference_hpwl: f64,
    growth_limit: GrowthLimit,
    iterations: usize,
    k: usize,
    outcome: Outcome,
    /// Which of the timing overflows have fired.
    timing_fired: Vec<bool>,
    timing_rounds: usize,
    timing_on: bool,
    /// Whether the setup changed since the last stretch, so the penalty's
    /// reference wirelength is stale and frozen fields may need to thaw.
    rebuilt: bool,
    /// The final placement's evaluation, congestion and zone outcomes.
    last: Option<(Evaluation, Option<f64>, Vec<ZoneOutcome>)>,
}

/// What a run calls out to: the congestion probe for routability, the
/// driver of timing-driven repairs, and the report of each iteration.
pub(super) struct Hooks<'a, D, R> {
    pub probe: Option<Probe<'a>>,
    pub timing: Option<&'a mut D>,
    pub report: R,
}

/// Minimizes wirelength plus density energy over the objects' centers,
/// leaving the placement in `model`. With routability enabled, congested
/// standard cells grow as in `OpenROAD` once spreading has nearly finished
/// and settled, and placement resumes from a snapshot. With a timing
/// driver, the design is repaired at the overflows `params.timing` names
/// and placement goes on with the repaired model, which replaces `model`.
pub(super) fn minimize<D: TimingDriver, R: FnMut(&Progress)>(
    model: &mut PlacementModel,
    mut setup: Setup,
    params: &GlobalParams,
    mut hooks: Hooks<'_, D, R>,
    rng: &mut StdRng,
) -> Result<GlobalResult, GlobalPlacementError<D::Error>> {
    let Some(bin) = setup.fields.iter().next().map(|f| f.grid.bin()) else {
        return Ok(unspread_result(&setup));
    };
    let mut run = Run::new(&setup, params, model.units.to_microns(REFERENCE_HPWL));
    loop {
        let flow = run.resume_setup(&mut setup, model, params, bin, &mut hooks);
        if flow == Flow::Done {
            break;
        }
        write_positions(model, &setup);
        let repaired = match hooks.timing.as_mut() {
            Some(driver) => driver.repair(model).map_err(GlobalPlacementError::Repair)?,
            None => None,
        };
        let Some(reweighted) = repaired else {
            run.timing_on = false;
            continue;
        };
        let (rebuilt, remap) =
            setup.rebuild(&reweighted.model, &reweighted.cell_map, params, rng)?;
        run.remap(&remap, &rebuilt);
        if let Some(p) = hooks.probe.as_mut() {
            p.refresh(&reweighted.model);
        }
        *model = reweighted.model;
        setup = rebuilt;
        run.timing_rounds += 1;
    }
    write_positions(model, &setup);
    Ok(run.result())
}

/// The result of a run with no field to spread: the wirelength of the
/// objects where they are.
fn unspread_result(setup: &Setup) -> GlobalResult {
    let mut grad = IdVec::filled(Point::new(0.0, 0.0), setup.objects.centers.len());
    let wirelength =
        setup
            .netlist
            .gradient(&setup.objects.centers, 1.0, &mut Vec::new(), &mut grad);
    GlobalResult {
        outcome: Outcome::Converged,
        iterations: 0,
        overflow: 0.0,
        hpwl: wirelength.hpwl,
        congestion: Vec::new(),
        final_congestion: None,
        zones: Vec::new(),
        timing_rounds: 0,
    }
}

/// Writes the cell objects' centers into `model` as cell positions.
fn write_positions(model: &mut PlacementModel, setup: &Setup) {
    for (k, &i) in setup.movable.iter_enumerated() {
        model.positions[i] = setup.objects.centers[k] - model.sizes[i] * 0.5;
    }
}

impl Run {
    fn new(setup: &Setup, params: &GlobalParams, reference_hpwl: f64) -> Self {
        let n = setup.objects.centers.len();
        let zero = IdVec::filled(Point::new(0.0, 0.0), n);
        Self {
            s: None,
            grad: zero.clone(),
            next_major: zero.clone(),
            next_reference: zero.clone(),
            next_grad: zero,
            settling: Settling::new(setup.fields.len()),
            snapshot: None,
            routability: params
                .routability
                .clone()
                .map(|r| Routability::new(r, &setup.objects, &setup.fields)),
            limit: params.max_iterations,
            refund: Refund::new(params.max_iterations),
            reference_hpwl,
            growth_limit: GrowthLimit::new(),
            iterations: 0,
            k: 0,
            outcome: Outcome::IterationLimit,
            timing_fired: params
                .timing
                .as_ref()
                .map_or_else(Vec::new, |t| vec![false; t.overflows.len()]),
            timing_rounds: 0,
            timing_on: params.timing.is_some(),
            rebuilt: false,
            last: None,
        }
    }

    /// Resumes the run on `setup` until it is over or a timing-driven
    /// repair is due.
    fn resume_setup<D, R: FnMut(&Progress)>(
        &mut self,
        setup: &mut Setup,
        model: &PlacementModel,
        params: &GlobalParams,
        bin: Point<f64>,
        hooks: &mut Hooks<'_, D, R>,
    ) -> Flow {
        let Setup {
            objects,
            fields,
            netlist,
            movable,
            ..
        } = setup;
        let mut objective = Objective::new(objects, &model.die, fields, netlist);
        let mut context = StretchContext {
            netlist,
            movable,
            probe: &mut hooks.probe,
            params,
            bin,
            site: model.site_width,
        };
        self.resume(&mut objective, objects, &mut context, &mut hooks.report)
    }

    /// What the run came to.
    fn result(&mut self) -> GlobalResult {
        let (last, final_congestion, zones) = self.last.take().unwrap_or_else(|| {
            let empty = Evaluation {
                wirelength: Wirelength::default(),
                overflow: 0.0,
                field_overflow: IdVec::new(),
                field_energy: IdVec::new(),
            };
            (empty, None, Vec::new())
        });
        GlobalResult {
            outcome: self.outcome,
            iterations: self.iterations,
            overflow: last.overflow,
            hpwl: last.wirelength.hpwl,
            congestion: self
                .routability
                .take()
                .map(|r| r.rounds)
                .unwrap_or_default(),
            final_congestion,
            zones,
            timing_rounds: self.timing_rounds,
        }
    }

    /// Continues the run with the objects of a rebuilt setup, each new
    /// object taking the state of the old one it continues and the rest
    /// starting where the setup has them.
    fn remap(&mut self, remap: &Remap, rebuilt: &Setup) {
        let carry = |old: &ObjectPoints| -> ObjectPoints {
            remap
                .old_of_new
                .iter_enumerated()
                .map(|(i, k)| k.map_or(rebuilt.objects.centers[i], |k| old[k]))
                .collect()
        };
        if let Some(s) = self.s.as_mut() {
            s.major = carry(&s.major);
            s.reference = carry(&s.reference);
            s.weights.remap(&remap.old_field_of_new);
        }
        if let Some(snapshot) = self.snapshot.as_mut() {
            snapshot.state.major = carry(&snapshot.state.major);
            snapshot.state.reference = carry(&snapshot.state.reference);
            snapshot.state.weights.remap(&remap.old_field_of_new);
            snapshot.energy = remap_fields(&snapshot.energy, &remap.old_field_of_new, 0.0);
        }
        self.settling.remap(&remap.old_field_of_new);
        if let Some(r) = self.routability.as_mut() {
            r.remap(remap, &rebuilt.objects, &rebuilt.fields);
        }
        let zero = IdVec::filled(Point::new(0.0, 0.0), remap.old_of_new.len());
        self.grad.clone_from(&zero);
        self.next_major.clone_from(&zero);
        self.next_reference.clone_from(&zero);
        self.next_grad = zero;
        self.rebuilt = true;
    }

    /// Iterates until the run is over or a timing-driven repair is due,
    /// leaving the placement in `objects.centers`.
    fn resume(
        &mut self,
        objective: &mut Objective<'_>,
        objects: &mut Objects,
        context: &mut StretchContext<'_, '_>,
        report: &mut impl FnMut(&Progress),
    ) -> Flow {
        let (mut s, mut eval) = self.start(objective, objects, context);
        let mut moving = objective.moving(&s.weights.frozen);
        let flow = loop {
            if self.k >= self.limit {
                break Flow::Done;
            }
            self.iterations = self.k + 1;
            let Some(next_eval) = self.step(objective, &mut s, &eval, &moving) else {
                self.outcome = Outcome::Diverged;
                break Flow::Done;
            };
            eval = next_eval;
            report(&Progress {
                iteration: self.k,
                hpwl: eval.wirelength.hpwl,
                overflow: eval.overflow,
                penalty: s.weights.highest(),
                step: s.step,
            });
            if let Some(flow) =
                self.settle(objective, objects, context, &mut s, &mut eval, &mut moving)
            {
                break flow;
            }
            self.k += 1;
        };
        objects.centers.clone_from(&s.major);
        if flow == Flow::Done {
            self.record_last(objective, objects, context, &s);
        }
        self.s = Some(s);
        flow
    }

    /// The state to go on from, made afresh on the first call, and its
    /// evaluation; after a rebuild the fields still overflowing move again.
    fn start(
        &mut self,
        objective: &mut Objective<'_>,
        objects: &Objects,
        context: &StretchContext<'_, '_>,
    ) -> (State, Evaluation) {
        let mut s = match self.s.take() {
            Some(s) => s,
            None => initial_state(
                objective,
                objects,
                context.bin,
                context.site,
                &mut self.grad,
            ),
        };
        let eval = objective.gradient(&s.reference, s.gamma, &s.weights, &mut self.grad);
        if self.rebuilt {
            s.prev_hpwl = eval.wirelength.hpwl;
            for (frozen, &overflow) in s.weights.frozen.iter_mut().zip(&eval.field_overflow) {
                if overflow > context.params.target_overflow.get() {
                    *frozen = false;
                }
            }
            self.rebuilt = false;
        }
        (s, eval)
    }

    /// Takes one Nesterov step from `s`, as `gpl`'s `nesterovUpdateCoordinates`
    /// and `updateNextIter`; none when it diverged.
    fn step(
        &mut self,
        objective: &mut Objective<'_>,
        s: &mut State,
        eval: &Evaluation,
        moving: &IdVec<ObjectId, bool>,
    ) -> Option<Evaluation> {
        let a_next = f64::midpoint(1.0, (4.0 * s.a).mul_add(s.a, 1.0).sqrt());
        let momentum = (s.a - 1.0) / a_next;
        let (next_eval, next_step) = self.backtrack(objective, s, eval, moving, momentum);
        if !(next_step.is_finite() && all_finite(&self.next_major) && all_finite(&self.next_grad)) {
            return None;
        }
        if self.routability.is_some() {
            self.settling.observe(
                &objective.displacements(&s.reference, &self.next_reference),
                &s.weights.frozen,
            );
        }
        std::mem::swap(&mut s.major, &mut self.next_major);
        std::mem::swap(&mut s.reference, &mut self.next_reference);
        std::mem::swap(&mut self.grad, &mut self.next_grad);
        s.step = next_step;
        s.a = a_next;
        Some(next_eval)
    }

    /// Updates the penalties, frozen fields and inflation after a step, and
    /// says how the run goes on: stopping once every field has spread, or
    /// for a timing-driven repair when one is due.
    fn settle(
        &mut self,
        objective: &mut Objective<'_>,
        objects: &mut Objects,
        context: &mut StretchContext<'_, '_>,
        s: &mut State,
        eval: &mut Evaluation,
        moving: &mut IdVec<ObjectId, bool>,
    ) -> Option<Flow> {
        let params = context.params;
        if self.k.is_multiple_of(50) && log::log_enabled!(log::Level::Debug) {
            s.weights.log(self.k, eval);
        }
        if freeze_spread_fields(s, eval, params.target_overflow.get()) {
            *moving = objective.moving(&s.weights.frozen);
        }
        s.weights.damp(self.k, eval);
        let converged = s.weights.frozen.iter().all(|&f| f);
        if !converged {
            self.grow_penalty(s, eval, context.bin);
        }
        let overflow = mean(&eval.field_overflow);
        if self.check_routability(objective, objects, context, s, eval) {
            *eval = objective.gradient(&s.reference, s.gamma, &s.weights, &mut self.grad);
            *moving = objective.moving(&s.weights.frozen);
        }
        if self.timing_round_due(params, overflow) {
            self.k += 1;
            return Some(Flow::Timing);
        }
        if converged {
            self.outcome = Outcome::Converged;
            return Some(Flow::Done);
        }
        None
    }

    /// Takes the next step from `s`, shortening it while the step the new
    /// gradients suggest falls short of it, as `gpl`'s `doBackTracking`;
    /// returns the new evaluation and that suggested step, leaving the new
    /// positions and gradients in the run's `next_` buffers.
    fn backtrack(
        &mut self,
        objective: &mut Objective<'_>,
        s: &State,
        eval: &Evaluation,
        moving: &IdVec<ObjectId, bool>,
        momentum: f64,
    ) -> (Evaluation, f64) {
        let mut next_eval = eval.clone();
        let next_step = backtrack_steps(s.step, |step| {
            for (i, &moves) in moving.iter_enumerated() {
                self.next_major[i] = if moves {
                    s.reference[i] - self.grad[i] * step
                } else {
                    s.major[i]
                };
            }
            objective.clamp(&mut self.next_major);
            for (i, &moves) in moving.iter_enumerated() {
                self.next_reference[i] = if moves {
                    self.next_major[i] + (self.next_major[i] - s.major[i]) * momentum
                } else {
                    s.major[i]
                };
            }
            objective.clamp(&mut self.next_reference);
            next_eval = objective.gradient(
                &self.next_reference,
                s.gamma,
                &s.weights,
                &mut self.next_grad,
            );
            let moved = distance(&self.next_reference, &s.reference);
            let changed = distance(&self.next_grad, &self.grad);
            if changed > 0.0 { moved / changed } else { step }
        });
        (next_eval, next_step)
    }

    /// Raises the density penalty by the wirelength's growth since the
    /// last iteration, and the wirelength smoothing with the overflow.
    fn grow_penalty(&mut self, s: &mut State, eval: &Evaluation, bin: Point<f64>) {
        let growth = penalty_growth(
            eval.wirelength.hpwl - s.prev_hpwl,
            self.reference_hpwl,
            self.growth_limit.highest,
        );
        self.growth_limit.observe(eval.overflow);
        s.weights.update(eval, growth);
        s.prev_hpwl = eval.wirelength.hpwl;
        s.gamma = gamma_for(eval.overflow, bin);
    }

    /// Takes the snapshot, extends the iteration limit while routability
    /// waits, and once placement has spread and settled measures the
    /// congestion; returns whether it grew cells and went back to the
    /// snapshot, so `s` needs its gradients again.
    fn check_routability(
        &mut self,
        objective: &mut Objective<'_>,
        objects: &mut Objects,
        context: &mut StretchContext<'_, '_>,
        s: &mut State,
        eval: &Evaluation,
    ) -> bool {
        let Some(r) = self.routability.as_mut() else {
            return false;
        };
        let overflow = mean(&eval.field_overflow);
        if self.snapshot.is_some() && overflow > CHECK_OVERFLOW && self.refund.take() {
            self.limit += 1;
        }
        if self.snapshot.is_none() && overflow <= SNAPSHOT_OVERFLOW {
            self.snapshot = Some(Snapshot {
                state: s.clone(),
                energy: eval.field_energy.clone(),
            });
        }
        let Some(saved) = &self.snapshot else {
            return false;
        };
        if !(r.active() && overflow <= CHECK_OVERFLOW && self.settling.settled()) {
            return false;
        }
        objects.centers.clone_from(&s.major);
        let map = congestion_map(
            context.probe,
            context.netlist,
            objects,
            context.movable,
            &objective.die,
            r.params(),
        );
        if r.check(self.k, &map, objects, objective.fields) == Verdict::Done {
            return false;
        }
        self.refund.start_round();
        *s = revert(&saved.state, s, objective.fields);
        objective.refresh(objects);
        let resized = objective.evaluate(&s.reference, s.gamma);
        s.weights
            .rescale_energy(&saved.energy, &resized.field_energy);
        true
    }

    /// Whether a timing-driven repair is due at `overflow`, routability
    /// first finishing any check it waits on.
    fn timing_round_due(&mut self, params: &GlobalParams, overflow: f64) -> bool {
        let Some(timing) = params.timing.as_ref().filter(|_| self.timing_on) else {
            return false;
        };
        let routability_pending = self.snapshot.is_some()
            && self.routability.as_ref().is_some_and(Routability::active)
            && overflow > CHECK_OVERFLOW;
        !routability_pending && timing_due(&timing.overflows, &mut self.timing_fired, overflow)
    }

    /// Keeps the final placement's evaluation, congestion and zone outcomes.
    fn record_last(
        &mut self,
        objective: &mut Objective<'_>,
        objects: &Objects,
        context: &mut StretchContext<'_, '_>,
        s: &State,
    ) {
        let last = objective.evaluate(&s.major, s.gamma);
        let final_congestion = self.routability.as_ref().map(|r| {
            congestion_map(
                context.probe,
                context.netlist,
                objects,
                context.movable,
                &objective.die,
                r.params(),
            )
            .metric()
        });
        let zones = objective.outcomes(&last);
        self.last = Some((last, final_congestion, zones));
    }
}

/// What a stretch of iterations reads besides the objective: the netlist
/// and movable cells routability measures, the probe it measures with, the
/// run's parameters, the bin size and the site width.
struct StretchContext<'a, 'p> {
    netlist: &'a Netlist,
    movable: &'a IdVec<ObjectId, CellId>,
    probe: &'a mut Option<Probe<'p>>,
    params: &'a GlobalParams,
    bin: Point<f64>,
    site: f64,
}

/// Freezes each field whose overflow has fallen to `target`; returns
/// whether any froze.
fn freeze_spread_fields(s: &mut State, eval: &Evaluation, target: f64) -> bool {
    let mut froze = false;
    for (frozen, &overflow) in s.weights.frozen.iter_mut().zip(&eval.field_overflow) {
        if !*frozen && overflow <= target {
            *frozen = true;
            froze = true;
        }
    }
    froze
}

/// The state to start from: the weights scaled to the first gradients and
/// a step probed from them.
fn initial_state(
    objective: &mut Objective<'_>,
    objects: &Objects,
    bin: Point<f64>,
    site: f64,
    grad: &mut ObjectPoints,
) -> State {
    let mut major = objects.centers.clone();
    objective.clamp(&mut major);
    let gamma = gamma_for(1.0, bin);
    let first = objective.evaluate(&major, gamma);
    let weights = objective.initial_weights(&first, site);
    let _ = objective.gradient(&major, gamma, &weights, grad);
    let step = initial_step(objective, &major, grad, gamma, &weights, bin);
    State {
        reference: major.clone(),
        major,
        step,
        weights,
        gamma,
        a: 1.0,
        prev_hpwl: first.wirelength.hpwl,
    }
}

/// Whether a timing-driven repair is due at `overflow`, marking the
/// thresholds it passes, as `OpenROAD`'s `isTimingNetWeightOverflow`: each
/// threshold, from the largest down, fires once when the overflow in whole
/// percent is under it.
fn timing_due(thresholds: &[Share], fired: &mut [bool], overflow: f64) -> bool {
    let percent = (overflow * 100.0).round();
    let as_percent = |t: Share| (t.get() * 100.0).round();
    let mut order: Vec<usize> = (0..thresholds.len()).collect();
    order.sort_by(|&a, &b| thresholds[b].get().total_cmp(&thresholds[a].get()));
    let Some(&largest) = order.first() else {
        return false;
    };
    if percent > as_percent(thresholds[largest]) {
        return false;
    }
    let mut due = false;
    for i in order {
        if as_percent(thresholds[i]) <= percent {
            break;
        }
        if !fired[i] {
            fired[i] = true;
            due = true;
        }
    }
    due
}

/// Values per field carried to the fields of a rebuilt setup, `default`
/// where a field is new.
pub(super) fn remap_fields<T: Copy>(
    old: &IdVec<FieldId, T>,
    old_field_of_new: &IdVec<FieldId, Option<FieldId>>,
    default: T,
) -> IdVec<FieldId, T> {
    old_field_of_new
        .iter()
        .map(|o| o.map_or(default, |k| old[k]))
        .collect()
}

/// The congestion of `objects` where they are: what `probe` measures, or
/// RUDY without one.
fn congestion_map(
    probe: &mut Option<Probe<'_>>,
    netlist: &Netlist,
    objects: &Objects,
    movable: &IdVec<ObjectId, CellId>,
    die: &Rect<f64>,
    params: &RoutabilityParams,
) -> CongestionMap {
    match probe {
        Some(probe) => CongestionMap::measured(probe.measure(objects, movable), params),
        None => CongestionMap::rudy(netlist, &objects.centers, die, params),
    }
}

/// A first step length from the gradient change over a small probe move,
/// in place of `gpl`'s `initDensity2` and `updateInitialPrevSLPCoordi`.
fn initial_step(
    objective: &mut Objective<'_>,
    at: &ObjectPoints,
    grad: &ObjectPoints,
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
    let mut probe: ObjectPoints = at.iter().zip(grad).map(|(&p, &g)| p - g * scale).collect();
    objective.clamp(&mut probe);
    let mut probe_grad = IdVec::filled(Point::new(0.0, 0.0), at.len());
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
    fn gamma_shrinks_from_forty_bins_to_under_one_as_overflow_falls() {
        let bin = Point::new(2.0, 2.0);

        assert!((gamma_for(1.0, bin) - 80.0).abs() < 1e-9);
        assert!((gamma_for(0.1, bin) - 0.8).abs() < 1e-9);
        assert!((gamma_for(0.01, bin) - 0.8).abs() < 1e-9);
    }

    #[test]
    fn the_penalty_grows_fastest_while_hpwl_falls() {
        assert!((penalty_growth(-1.0, 100.0, 1.05) - 1.05).abs() < 1e-12);
        assert!((penalty_growth(100.0, 100.0, 1.05) - 1.0).abs() < 1e-12);
        assert!((penalty_growth(1e9, 100.0, 1.05) - 0.95).abs() < 1e-12);
    }

    #[test]
    fn a_step_kept_whole_takes_one_trial() {
        let mut trials = 0;

        let step = backtrack_steps(1.0, |step| {
            trials += 1;
            step
        });

        assert_eq!((trials, step), (1, 1.0));
    }

    #[test]
    fn a_step_that_keeps_shrinking_stops_after_gpl_s_ten_trials() {
        let mut trials = 0;

        let step = backtrack_steps(1.0, |step| {
            trials += 1;
            step / 2.0
        });

        assert_eq!((trials, step), (MAX_BACKTRACKS, 0.5_f64.powi(10)));
    }

    #[test]
    fn a_round_gives_back_iterations_up_to_its_cap() {
        let mut refund = Refund::new(2);

        let taken = [refund.take(), refund.take(), refund.take()];

        assert_eq!(taken, [true, true, false]);
    }

    #[test]
    fn each_routability_round_gives_back_its_own_cap() {
        let mut refund = Refund::new(2);
        let _ = (refund.take(), refund.take());

        refund.start_round();

        assert!(refund.take());
    }

    #[test]
    fn the_growth_limit_is_lowered_once_as_overflow_falls() {
        let mut limit = GrowthLimit::new();

        limit.observe(0.5);
        let before = limit.highest;
        limit.observe(0.3);
        let after = limit.highest;
        limit.observe(0.2);

        assert!((before - 1.05).abs() < 1e-12);
        assert!((after - 1.0395).abs() < 1e-12);
        assert!((limit.highest - after).abs() < 1e-12);
    }

    fn evaluation(field_energy: Vec<f64>) -> Evaluation {
        Evaluation {
            wirelength: Wirelength::default(),
            overflow: 0.5,
            field_overflow: vec![0.5; field_energy.len()].into(),
            field_energy: field_energy.into(),
        }
    }

    #[test]
    fn eplace_weights_grow_by_the_given_factor_except_when_frozen() {
        let mut weights = Weights::eplace(vec![2.0, 4.0].into());
        weights.frozen[FieldId(1)] = true;

        weights.update(&evaluation(vec![1.0, 1.0]), 1.05);

        assert!((weights.penalties[FieldId(0)] - 2.1).abs() < 1e-12);
        assert!((weights.penalties[FieldId(1)] - 4.0).abs() < 1e-12);
    }

    #[test]
    fn a_scheduled_weight_steps_along_its_normalized_energy() {
        let mut weights = Weights {
            penalties: vec![1.0, 1.0].into(),
            frozen: vec![false, false].into(),
            schedule: Some(Schedule {
                precondition: vec![0.1, 0.05].into(),
                u: vec![1.0, 1.0].into(),
                step: 0.1,
                cap: f64::MAX,
                site: 1.0,
                kinds: vec![Precondition::Weighted; 2].into(),
                damping: 1.0,
            }),
        };

        weights.update(&evaluation(vec![10.0, 20.0]), 1.05);

        let u = 0.1f64.mul_add(std::f64::consts::FRAC_1_SQRT_2, 1.0);
        let penalty = u * 2001.0;
        assert!((weights.penalties[FieldId(0)] - penalty).abs() < 1e-9);
        assert!((weights.penalties[FieldId(1)] - penalty).abs() < 1e-9);
        let rate = (2000.0 * std::f64::consts::SQRT_2).ln();
        let growth = (rate / (1.0 + rate)).mul_add(0.01, 1.03);
        let expected_step = 0.1 * growth;
        let step = weights.schedule.map_or(0.0, |s| s.step);
        assert!((step - expected_step).abs() < 1e-12);
    }

    #[test]
    fn the_quadratic_term_scales_a_scheduled_weight_by_the_energy_left() {
        let weights = Weights {
            penalties: vec![3.0].into(),
            frozen: vec![false].into(),
            schedule: Some(Schedule {
                precondition: vec![0.5].into(),
                u: vec![1.0].into(),
                step: 0.1,
                cap: f64::MAX,
                site: 1.0,
                kinds: vec![Precondition::Weighted].into(),
                damping: 1.0,
            }),
        };

        let expected = 3.0 * 1001.0;
        assert!((weights.multiplier(FieldId(0), 1.0) - expected).abs() < 1e-9);
        assert!((weights.multiplier(FieldId(0), 0.0) - 3.0).abs() < 1e-12);
    }

    fn scheduled(frozen: Vec<bool>) -> Weights {
        Weights {
            penalties: vec![5.0, 7.0].into(),
            frozen: frozen.into(),
            schedule: Some(Schedule {
                precondition: vec![0.5, 0.25].into(),
                u: vec![0.01, 0.02].into(),
                step: 0.1,
                cap: f64::MAX,
                site: 1.0,
                kinds: vec![Precondition::Weighted; 2].into(),
                damping: 1.0,
            }),
        }
    }

    #[test]
    fn a_rescaled_schedule_reads_grown_energies_as_the_ungrown_ones() {
        let mut plain = scheduled(vec![false, false]);
        let mut grown = scheduled(vec![false, false]);

        plain.update(&evaluation(vec![2.0, 4.0]), 1.05);
        grown.rescale_energy(&vec![2.0, 4.0].into(), &vec![6.0, 10.0].into());
        grown.update(&evaluation(vec![6.0, 10.0]), 1.05);

        for (a, b) in plain.penalties.iter().zip(&grown.penalties) {
            assert!((a - b).abs() < 1e-9 * a.abs(), "{plain:?} vs {grown:?}");
        }
    }

    #[test]
    fn rescaling_to_unchanged_energies_changes_nothing() {
        let mut weights = scheduled(vec![false, false]);
        let before = weights.schedule.as_ref().unwrap().precondition.clone();

        weights.rescale_energy(&vec![2.0, 4.0].into(), &vec![2.0, 4.0].into());

        assert_eq!(weights.schedule.as_ref().unwrap().precondition, before);
    }

    #[test]
    fn a_frozen_field_keeps_its_energy_reference() {
        let mut weights = scheduled(vec![false, true]);

        weights.rescale_energy(&vec![2.0, 4.0].into(), &vec![6.0, 10.0].into());

        let s = weights.schedule.as_ref().unwrap();
        assert!((s.precondition[FieldId(0)] - 0.5 / 3.0).abs() < 1e-12);
        assert!((s.precondition[FieldId(1)] - 0.25).abs() < 1e-12);
    }

    fn shares(values: [f64; 2]) -> [Share; 2] {
        values.map(|v| Share::new(v).unwrap())
    }

    #[test]
    fn timing_thresholds_fire_once_each_from_the_largest_down() {
        let thresholds = shares([0.64, 0.20]);
        let mut fired = [false, false];

        assert!(!timing_due(&thresholds, &mut fired, 0.70));
        assert!(timing_due(&thresholds, &mut fired, 0.63));
        assert_eq!(fired, [true, false]);
        assert!(!timing_due(&thresholds, &mut fired, 0.50));
        assert!(timing_due(&thresholds, &mut fired, 0.19));
        assert_eq!(fired, [true, true]);
        assert!(!timing_due(&thresholds, &mut fired, 0.05));
    }

    #[test]
    fn an_overflow_under_every_threshold_fires_them_all_at_once() {
        let thresholds = shares([0.20, 0.64]);
        let mut fired = [false, false];

        assert!(timing_due(&thresholds, &mut fired, 0.10));
        assert_eq!(fired, [true, true]);
    }

    #[test]
    fn remapped_weights_carry_fields_by_their_old_index() {
        let mut weights = Weights::eplace(vec![2.0, 4.0].into());
        weights.frozen[FieldId(1)] = true;

        weights.remap(&vec![Some(FieldId(1)), None, Some(FieldId(0))].into());

        assert_eq!(weights.penalties, IdVec::from(vec![4.0, 3.0, 2.0]));
        assert_eq!(weights.frozen, IdVec::from(vec![true, false, false]));
    }

    #[test]
    fn a_large_object_is_kept_centered_on_a_small_die() {
        let die = Rect::new(Point::new(0.0, 0.0), Point::new(10.0, 10.0));

        let b = center_bounds(&die, Point::new(12.0, 4.0));

        assert_eq!((b.min.x, b.max.x), (5.0, 5.0));
        assert_eq!((b.min.y, b.max.y), (2.0, 8.0));
    }
}
