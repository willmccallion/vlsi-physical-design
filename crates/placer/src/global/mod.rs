//! Global placement in the style of ePlace and `RePlAce`.
//!
//! Cells are electric charges: the field of their density spreads them
//! apart while a smooth wirelength model pulls connected cells together.
//! Filler cells take up the whitespace, so a design that uses a small part
//! of its die still packs to the target density instead of scattering.
//! Nesterov's method minimizes the sum, raising the weight of density until
//! the cell area above the target density falls below a threshold. With
//! routability enabled, standard cells in congested gcells are inflated
//! along the way, as in `OpenROAD`.
//!
//! Fence regions are handled as in `DREAMPlace` 3.0: each zone's cells and
//! fillers form an electrostatic system of their own, whose fixed charge
//! covers everything outside the zone, so the field pushes them into it.
//! Every system has its own target density, density weight and stopping
//! point; wirelength couples them all.

mod density;
mod nesterov;
mod poisson;
mod routability;
mod wirelength;

pub use routability::{CongestionEstimator, CongestionRound, RoutabilityParams, RoutingTiles};

use crate::model::{PinAnchor, PlacementModel};
use crate::zones::Zone;
use density::DensityGrid;
use nesterov::Objects;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::convert::Infallible;
use std::ops::Range;
use wirelength::{Netlist, Terminal};

/// Settings for global placement.
#[derive(Clone, Debug, PartialEq)]
pub struct GlobalParams {
    /// Highest fraction of a bin cells should fill, in `(0, 1]`.
    pub target_density: f64,
    /// Fraction of cell area above the target density at which to stop.
    pub target_overflow: f64,
    /// Iteration limit.
    pub max_iterations: usize,
    /// Bins per side of the density grid.
    pub bins: usize,
    /// Seed for the initial placement.
    pub seed: u64,
    /// Inflation of cells in routing-congested bins, if enabled.
    pub routability: Option<RoutabilityParams>,
    /// Repair of the design for timing along the way, if enabled.
    pub timing: Option<TimingParams>,
}

/// When timing-driven placement repairs the design, as `OpenROAD`'s
/// `timing_driven_net_reweight_overflow`.
#[derive(Clone, Debug, PartialEq)]
pub struct TimingParams {
    /// Overflows at which the design is repaired and its nets reweighted,
    /// each once, as the overflow falls below it.
    pub overflows: Vec<f64>,
}

/// The design after a timing-driven repair: the model to go on with, and
/// where each cell of the model before went.
#[derive(Debug)]
pub struct Reweighted {
    /// The repaired design, its nets weighted for timing.
    pub model: PlacementModel,
    /// For each cell of the model before the repair, its index in the new
    /// model, or none if the repair removed it.
    pub cell_map: Vec<Option<usize>>,
}

/// What repairs the design between iterations of timing-driven placement.
pub trait TimingDriver {
    /// What a repair can fail with.
    type Error;

    /// Repairs the design at the positions of `model` and weights its nets
    /// for timing; none turns timing-driven placement off for the rest of
    /// the run.
    fn repair(&mut self, model: &PlacementModel) -> Result<Option<Reweighted>, Self::Error>;
}

/// The driver of a run without timing-driven placement: it never repairs.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoTiming;

impl TimingDriver for NoTiming {
    type Error = Infallible;

    fn repair(&mut self, _: &PlacementModel) -> Result<Option<Reweighted>, Infallible> {
        Ok(None)
    }
}

/// One iteration's state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Progress {
    /// Iteration number, from zero.
    pub iteration: usize,
    /// Half-perimeter wirelength, in microns.
    pub hpwl: f64,
    /// Cell area above the target density, as a fraction of all cell area.
    pub overflow: f64,
    /// Weight of density energy against wirelength.
    pub penalty: f64,
    /// Step length.
    pub step: f64,
}

/// How global placement ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Overflow fell to the target.
    Converged,
    /// The iteration limit was reached first.
    IterationLimit,
    /// The optimization produced non-finite values and stopped early.
    Diverged,
}

/// The result of global placement.
#[derive(Clone, Debug, PartialEq)]
pub struct GlobalResult {
    /// How it ended.
    pub outcome: Outcome,
    /// Iterations run.
    pub iterations: usize,
    /// Final overflow.
    pub overflow: f64,
    /// Final half-perimeter wirelength, in microns.
    pub hpwl: f64,
    /// Each routability round, in order.
    pub congestion: Vec<CongestionRound>,
    /// The congestion metric of the final placement, with routability
    /// enabled.
    pub final_congestion: Option<f64>,
    /// How each zone with cells ended.
    pub zones: Vec<ZoneOutcome>,
    /// Timing-driven repairs made along the way.
    pub timing_rounds: usize,
}

/// How the cells of one zone were spread.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ZoneOutcome {
    /// The zone.
    pub zone: Zone,
    /// Its movable cells.
    pub cells: usize,
    /// Their area over the free area of the zone's rows.
    pub utilization: f64,
    /// The density they were spread to.
    pub target: f64,
    /// Their final overflow.
    pub overflow: f64,
}

/// One electrostatic system: the cells of one zone and their fillers,
/// spread over that zone's rows only.
#[derive(Debug)]
pub(super) struct Field {
    zone: Zone,
    grid: DensityGrid,
    /// Where its objects may go.
    area: Vec<Rect<f64>>,
    /// Object indices of its cells.
    cells: Range<usize>,
    /// Object indices of its fillers.
    fillers: Range<usize>,
    /// How many of its fillers, from the first, are in play: routability
    /// removes them from the last.
    kept_fillers: usize,
    /// Width and height of each filler.
    filler: Point<f64>,
    utilization: f64,
}

impl Field {
    /// Object indices of the fillers in play.
    const fn live_fillers(&self) -> Range<usize> {
        self.fillers.start..self.fillers.start + self.kept_fillers
    }
}

/// How far above its cells' utilization a zone's target density is set
/// when the requested target is lower, so that it can still be met.
const ZONE_DENSITY_MARGIN: f64 = 0.01;

/// Fewest and most bins per side of the density grid.
const MIN_BINS: usize = 16;
const MAX_BINS: usize = 512;

/// Share of the room left above a design's utilization that its default
/// target density adds, as `OpenROAD-flow-scripts`' `PLACE_DENSITY_LB_ADDON`.
const DENSITY_HEADROOM_SHARE: f64 = 0.2;

/// The target density for a design whose movable cells fill `utilization`
/// of the free area, as `OpenROAD-flow-scripts` sets it.
///
/// As its `place_density_with_lb_addon`, this is the utilization rounded up
/// to a hundredth, as `OpenROAD` reports it, plus a fifth of the room left
/// above it and a hundredth, at most one.
pub fn default_target_density(utilization: f64) -> f64 {
    let lower_bound = (utilization * 100.0).ceil() / 100.0;
    let raised = (1.0 - lower_bound).mul_add(DENSITY_HEADROOM_SHARE, lower_bound);
    (raised + 0.01).min(1.0)
}

/// Bins per side for `model`: sized for the average cell at the target
/// density, as `OpenROAD` sizes them, but no shorter than the row pitch when
/// channels separate the rows.
///
/// Each bin then averages rows and channels, so cells may cross a channel
/// to a less crowded row instead of facing a wall of blocked bins;
/// legalization moves them into the rows.
pub fn default_bins(model: &PlacementModel, target_density: f64) -> usize {
    let movable = (0..model.num_cells()).filter(|&i| model.movable[i]);
    let (area, count) = movable.fold((0.0, 0usize), |(area, count), i| {
        (model.sizes[i].x.mul_add(model.sizes[i].y, area), count + 1)
    });
    let average = if count == 0 { 0.0 } else { area / count as f64 };
    let by_area = bins_for_cell_area(model.die.area(), average, target_density);
    match channel_pitch(&model.rows) {
        Some(pitch) => {
            let rows = (model.die.height() / pitch).floor().max(1.0) as usize;
            let by_pitch = 1usize << rows.ilog2();
            by_area.min(by_pitch).max(MIN_BINS)
        }
        None => by_area,
    }
}

/// The largest power of two whose square of bins is at most the number of
/// bins of the ideal area: an average cell's area over the target density,
/// so that a bin holds about one cell's worth of charge and the smoothing of
/// a cell over its bins reaches its neighbours. Bins of a quarter of that
/// area leave a placement at a loose utilization lumpy at the bin scale,
/// with an overflow floor the placer cannot get under.
fn bins_for_cell_area(die_area: f64, average_cell_area: f64, target_density: f64) -> usize {
    if average_cell_area <= 0.0 || target_density <= 0.0 {
        return MIN_BINS;
    }
    let ideal_bins = die_area / (average_cell_area / target_density);
    let mut bins = MIN_BINS;
    while bins < MAX_BINS && ((2 * bins) as f64).powi(2) <= ideal_bins {
        bins *= 2;
    }
    bins
}

/// The distance between consecutive rows, if the rows are separated by
/// gaps rather than abutting.
fn channel_pitch(rows: &[Rect<f64>]) -> Option<f64> {
    let mut bands: Vec<(f64, f64)> = rows.iter().map(|r| (r.min.y, r.max.y)).collect();
    bands.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    bands.dedup();
    if !bands.windows(2).any(|w| w[1].0 > w[0].1) {
        return None;
    }
    let mut pitches: Vec<f64> = bands.windows(2).map(|w| w[1].0 - w[0].0).collect();
    pitches.sort_by(f64::total_cmp);
    pitches.get(pitches.len() / 2).copied().filter(|&p| p > 0.0)
}

/// Places the movable cells of `model`, calling `report` after every
/// iteration.
///
/// With `timing`, the design is repaired and its nets reweighted at the
/// overflows `params.timing` names; the model then changes to the repaired
/// design.
pub fn place<D: TimingDriver>(
    model: &mut PlacementModel,
    params: &GlobalParams,
    estimator: Option<&mut dyn CongestionEstimator>,
    timing: Option<&mut D>,
    report: impl FnMut(&Progress),
) -> Result<GlobalResult, D::Error> {
    let mut rng = StdRng::seed_from_u64(params.seed);
    let setup = Setup::new(model, params, &mut rng);
    let probe = estimator.map(|estimator| Probe::new(estimator, model));
    nesterov::minimize(model, setup, params, probe, timing, &mut rng, report)
}

/// What a run works on, built from one model: the objects, their fields
/// and the nets between them.
struct Setup {
    objects: Objects,
    fields: Vec<Field>,
    netlist: Netlist,
    /// The model cell of each object that is a cell.
    movable: Vec<usize>,
    /// Each cell object's size in the model, before any inflation.
    model_sizes: Vec<Point<f64>>,
}

/// How the objects of a rebuilt setup continue those of the one before.
struct Remap {
    /// For each new object, the old object it continues.
    old_of_new: Vec<Option<usize>>,
    /// For each new field, the old field of the same zone.
    old_field_of_new: Vec<Option<usize>>,
    /// Cell area the new objects add over the old ones.
    added_area: f64,
}

impl Setup {
    fn new(model: &PlacementModel, params: &GlobalParams, rng: &mut StdRng) -> Self {
        let movable = movable_cells(model);
        let fixed = fixed_rects(model);
        let mut objects = Objects {
            centers: initial_centers(model, &movable, rng),
            sizes: movable.iter().map(|&i| model.sizes[i]).collect(),
            kinds: movable.iter().map(|&i| model.kinds[i]).collect(),
        };
        let mut fields = build_fields(model, &movable, &fixed, params);
        for field in &mut fields {
            add_fillers(field, &mut objects, rng);
        }
        Self {
            netlist: build_netlist(model, &movable),
            model_sizes: objects.sizes.clone(),
            objects,
            fields,
            movable,
        }
    }

    /// The setup for `model`, the design after a repair, continuing this
    /// one: cells the repair kept keep their objects, with their sizes
    /// unless the repair changed them, each field keeps its fillers and
    /// its target density rises by the area its cells gained over the
    /// free area, as `OpenROAD` raises it; new cells start where the
    /// repair put them.
    fn rebuild(
        &self,
        model: &PlacementModel,
        cell_map: &[Option<usize>],
        params: &GlobalParams,
        rng: &mut StdRng,
    ) -> (Self, Remap) {
        let movable = movable_cells(model);
        let fixed = fixed_rects(model);
        let mut old_object_of_old_cell = vec![None; cell_map.len()];
        for (k, &i) in self.movable.iter().enumerate() {
            old_object_of_old_cell[i] = Some(k);
        }
        let mut old_object_of_new_cell = vec![None; model.num_cells()];
        for (i, &new) in cell_map.iter().enumerate() {
            if let (Some(j), Some(k)) = (new, old_object_of_old_cell[i]) {
                old_object_of_new_cell[j] = Some(k);
            }
        }
        let mut objects = Objects {
            centers: Vec::with_capacity(movable.len()),
            sizes: Vec::with_capacity(movable.len()),
            kinds: movable.iter().map(|&i| model.kinds[i]).collect(),
        };
        let mut old_of_new = Vec::with_capacity(movable.len());
        let area = |size: Point<f64>| size.x * size.y;
        let mut added_area = 0.0;
        for &i in &movable {
            let size = model.sizes[i];
            match old_object_of_new_cell[i] {
                Some(k) if self.model_sizes[k] == size => {
                    objects.centers.push(self.objects.centers[k]);
                    objects.sizes.push(self.objects.sizes[k]);
                    old_of_new.push(Some(k));
                }
                Some(k) => {
                    objects.centers.push(self.objects.centers[k]);
                    objects.sizes.push(size);
                    old_of_new.push(Some(k));
                    added_area += area(size) - area(self.model_sizes[k]);
                }
                None => {
                    objects.centers.push(model.positions[i] + size * 0.5);
                    objects.sizes.push(size);
                    old_of_new.push(None);
                    added_area += area(size);
                }
            }
        }
        let model_sizes = objects.sizes.clone();
        for (k, &i) in self.movable.iter().enumerate() {
            if cell_map[i].is_none() {
                added_area -= area(self.model_sizes[k]);
            }
        }
        let mut fields = build_fields(model, &movable, &fixed, params);
        let mut old_field_of_new = Vec::with_capacity(fields.len());
        for field in &mut fields {
            let old = self.fields.iter().position(|f| f.zone == field.zone);
            old_field_of_new.push(old);
            let Some(old) = old.map(|o| &self.fields[o]) else {
                add_fillers(field, &mut objects, rng);
                old_of_new.extend(field.fillers.clone().map(|_| None));
                continue;
            };
            let first = objects.centers.len();
            for k in old.fillers.clone() {
                objects.centers.push(self.objects.centers[k]);
                objects.sizes.push(self.objects.sizes[k]);
                old_of_new.push(Some(k));
            }
            field.fillers = first..objects.centers.len();
            field.kept_fillers = old.kept_fillers;
            field.filler = old.filler;
            let cell_area = |objects: &Objects, cells: &Range<usize>| -> f64 {
                cells
                    .clone()
                    .map(|i| objects.sizes[i].x * objects.sizes[i].y)
                    .sum()
            };
            let gained = cell_area(&objects, &field.cells) - cell_area(&self.objects, &old.cells);
            let free = field.grid.free_area();
            if free > 0.0 {
                let target = old.grid.target() + gained / free;
                if target > 1.0 {
                    log::warn!(
                        "timing-driven repair raised a field's target density to {target:.3}"
                    );
                }
                field.grid.set_target(target);
                field.utilization = cell_area(&objects, &field.cells) / free;
            }
        }
        let setup = Self {
            netlist: build_netlist(model, &movable),
            model_sizes,
            objects,
            fields,
            movable,
        };
        let remap = Remap {
            old_of_new,
            old_field_of_new,
            added_area,
        };
        (setup, remap)
    }
}

/// The model's movable cells, grouped by zone.
fn movable_cells(model: &PlacementModel) -> Vec<usize> {
    let mut movable: Vec<usize> = (0..model.num_cells())
        .filter(|&i| model.movable[i])
        .collect();
    movable.sort_by_key(|&i| (model.zone_of[i], i));
    movable
}

/// The boxes of the model's fixed cells.
fn fixed_rects(model: &PlacementModel) -> Vec<Rect<f64>> {
    (0..model.num_cells())
        .filter(|&i| !model.movable[i])
        .map(|i| Rect::new(model.positions[i], model.positions[i] + model.sizes[i]))
        .collect()
}

/// Gives `field` the fillers that bring its free area to its target
/// density, as new objects scattered over its area.
fn add_fillers(field: &mut Field, objects: &mut Objects, rng: &mut StdRng) {
    let cells = &objects.sizes[field.cells.clone()];
    let (size, count) = filler_layout(cells, field.grid.free_area(), field.grid.target());
    let first = objects.centers.len();
    for _ in 0..count {
        objects.centers.push(random_point(&field.area, rng));
        objects.sizes.push(size);
    }
    field.fillers = first..objects.centers.len();
    field.kept_fillers = count;
    field.filler = size;
}

/// Measures congestion where the objects are, through an estimator that
/// sees every cell of the model.
struct Probe<'a> {
    estimator: &'a mut dyn CongestionEstimator,
    /// The center of every cell of the model.
    centers: Vec<Point<f64>>,
}

impl<'a> Probe<'a> {
    fn new(estimator: &'a mut dyn CongestionEstimator, model: &PlacementModel) -> Self {
        let mut probe = Self {
            estimator,
            centers: Vec::new(),
        };
        probe.refresh(model);
        probe
    }

    /// Takes in the cells of `model`.
    fn refresh(&mut self, model: &PlacementModel) {
        self.centers = (0..model.num_cells())
            .map(|i| model.positions[i] + model.sizes[i] * 0.5)
            .collect();
    }

    /// The congestion of every gcell with the cell objects where
    /// `objects` has them.
    fn measure(&mut self, objects: &Objects, movable: &[usize]) -> Vec<f64> {
        for (k, &i) in movable.iter().enumerate() {
            self.centers[i] = objects.centers[k];
        }
        self.estimator.congestion(&self.centers)
    }
}

/// One field for each zone with movable cells, over the cells of
/// `movable`, which are grouped by zone; their fillers are added later.
fn build_fields(
    model: &PlacementModel,
    movable: &[usize],
    fixed: &[Rect<f64>],
    params: &GlobalParams,
) -> Vec<Field> {
    let mut fields = Vec::new();
    let mut start = 0;
    for (k, zone) in model.zones.iter().enumerate() {
        let end = movable.partition_point(|&i| model.zone_of[i] <= k);
        if end == start {
            continue;
        }
        let area = zone_area(model, k);
        let mut grid =
            DensityGrid::new(&model.die, params.bins, params.target_density, &area, fixed);
        let cell_area: f64 = movable[start..end]
            .iter()
            .map(|&i| model.sizes[i].x * model.sizes[i].y)
            .sum();
        let free = grid.free_area();
        let utilization = if free > 0.0 {
            cell_area / free
        } else {
            f64::INFINITY
        };
        grid.set_target(zone_target(params.target_density, utilization));
        fields.push(Field {
            zone: zone.zone,
            grid,
            area,
            cells: start..end,
            fillers: 0..0,
            kept_fillers: 0,
            filler: Point::new(0.0, 0.0),
            utilization,
        });
        start = end;
    }
    fields
}

/// The density a zone's cells are spread to: the requested target, or just
/// above their utilization when that is higher, since the target could
/// never be met otherwise.
fn zone_target(requested: f64, utilization: f64) -> f64 {
    if utilization > requested {
        (utilization + ZONE_DENSITY_MARGIN).min(1.0)
    } else {
        requested
    }
}

/// The size and number of fillers that bring the free area to the target
/// density: as tall as the median cell, with the mean area of the cells
/// left after dropping the smallest and largest twentieth.
fn filler_layout(cells: &[Point<f64>], free_area: f64, target: f64) -> (Point<f64>, usize) {
    let cell_area: f64 = cells.iter().map(|s| s.x * s.y).sum();
    let filler_area = target.mul_add(free_area, -cell_area).max(0.0);
    if cells.is_empty() {
        return (Point::new(0.0, 0.0), 0);
    }
    let mut areas: Vec<f64> = cells.iter().map(|s| s.x * s.y).collect();
    areas.sort_by(f64::total_cmp);
    let trim = areas.len() / 20;
    let kept = &areas[trim..areas.len() - trim];
    let mean_area = kept.iter().sum::<f64>() / kept.len() as f64;
    let mut heights: Vec<f64> = cells.iter().map(|s| s.y).collect();
    heights.sort_by(f64::total_cmp);
    let height = heights[heights.len() / 2];
    if mean_area <= 0.0 || height <= 0.0 {
        return (Point::new(0.0, 0.0), 0);
    }
    let size = Point::new(mean_area / height, height);
    (size, (filler_area / mean_area) as usize)
}

/// The rows, or the whole die for a design without rows.
fn placeable_area(model: &PlacementModel) -> Vec<Rect<f64>> {
    if model.rows.is_empty() {
        vec![model.die]
    } else {
        model.rows.clone()
    }
}

/// The area the cells of zone `k` may use.
fn zone_area(model: &PlacementModel, k: usize) -> Vec<Rect<f64>> {
    model
        .zones
        .get(k)
        .map_or_else(|| placeable_area(model), |z| z.rects.clone())
}

/// Cell centers in a small random cluster at the middle of the rows.
fn initial_centers(model: &PlacementModel, movable: &[usize], rng: &mut StdRng) -> Vec<Point<f64>> {
    let placeable = placeable_area(model);
    let lo = placeable.iter().fold(model.die.max, |p, r| {
        Point::new(p.x.min(r.min.x), p.y.min(r.min.y))
    });
    let hi = placeable.iter().fold(model.die.min, |p, r| {
        Point::new(p.x.max(r.max.x), p.y.max(r.max.y))
    });
    let area = Rect::new(lo, hi);
    let mid = Point::new(f64::midpoint(lo.x, hi.x), f64::midpoint(lo.y, hi.y));
    let spread = Point::new(area.width() * 0.005, area.height() * 0.005);
    movable
        .iter()
        .map(|_| {
            Point::new(
                rng.gen_range(-1.0..=1.0_f64).mul_add(spread.x, mid.x),
                rng.gen_range(-1.0..=1.0_f64).mul_add(spread.y, mid.y),
            )
        })
        .collect()
}

/// A uniformly random point in the union of disjoint `rects`.
fn random_point(rects: &[Rect<f64>], rng: &mut StdRng) -> Point<f64> {
    let total: f64 = rects.iter().map(Rect::area).sum();
    let mut pick = rng.gen_range(0.0..=total);
    let chosen = rects
        .iter()
        .find(|r| {
            let inside = pick <= r.area();
            pick -= r.area();
            inside
        })
        .or_else(|| rects.last());
    chosen.map_or(Point::new(0.0, 0.0), |r| {
        Point::new(
            rng.gen_range(r.min.x..=r.max.x),
            rng.gen_range(r.min.y..=r.max.y),
        )
    })
}

/// The model's nets with movable cells as objects and everything else as
/// fixed points.
fn build_netlist(model: &PlacementModel, movable: &[usize]) -> Netlist {
    let mut object_of_cell = vec![None; model.num_cells()];
    for (k, &i) in movable.iter().enumerate() {
        object_of_cell[i] = Some(k);
    }
    let mut netlist = Netlist::default();
    for net in &model.nets {
        let pins = net.pins.iter().map(|pin| {
            let terminal = match pin.anchor {
                PinAnchor::Cell { cell, offset } => match object_of_cell[cell] {
                    Some(index) => Terminal::Object {
                        index,
                        offset: offset - model.sizes[cell] * 0.5,
                    },
                    None => Terminal::Fixed(model.positions[cell] + offset),
                },
                PinAnchor::Fixed(p) => Terminal::Fixed(p),
            };
            (terminal, pin.extent)
        });
        netlist.push(pins, net.weight);
    }
    netlist
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bins_hold_about_one_average_cell_at_the_target_density() {
        // ibex in Nangate45: a 239 by 239 micron die, cells of 1.98 square
        // microns on average, placed at 0.65. OpenROAD picks 128 bins a side.
        let bins = bins_for_cell_area(57_358.0, 1.983, 0.65);

        assert_eq!(bins, 128);
    }

    #[test]
    fn bins_double_once_a_quarter_of_the_ideal_count_is_reached() {
        let four_squares = 4.0 * 128.0 * 128.0;

        let one_short = bins_for_cell_area(four_squares - 1.0, 1.0, 1.0);
        let exact = bins_for_cell_area(four_squares, 1.0, 1.0);

        assert_eq!(one_short, 128);
        assert_eq!(exact, 256);
    }

    #[test]
    fn bins_stay_within_the_grid_limits() {
        assert_eq!(bins_for_cell_area(1.0, 100.0, 0.5), MIN_BINS);
        assert_eq!(bins_for_cell_area(1e12, 1.0, 1.0), MAX_BINS);
        assert_eq!(bins_for_cell_area(1e6, 0.0, 0.5), MIN_BINS);
    }

    #[test]
    fn the_default_target_density_adds_a_fifth_of_the_headroom() {
        assert!((default_target_density(0.65) - 0.73).abs() < 1e-9);
    }

    #[test]
    fn the_default_target_density_starts_from_the_utilization_rounded_up() {
        assert!((default_target_density(0.0836) - 0.282).abs() < 1e-9);
    }

    #[test]
    fn the_default_target_density_is_at_most_one() {
        assert!((default_target_density(0.995) - 1.0).abs() < f64::EPSILON);
    }

    fn row(y: f64, height: f64) -> Rect<f64> {
        Rect::new(Point::new(0.0, y), Point::new(100.0, y + height))
    }

    #[test]
    fn rows_separated_by_channels_report_their_pitch() {
        let rows = [row(32.0, 64.0), row(160.0, 64.0), row(288.0, 64.0)];

        assert_eq!(channel_pitch(&rows), Some(128.0));
    }

    #[test]
    fn abutting_rows_have_no_channels() {
        let rows = [row(0.0, 1.4), row(1.4, 1.4), row(2.8, 1.4)];

        assert_eq!(channel_pitch(&rows), None);
    }

    #[test]
    fn fillers_bring_the_free_area_to_the_target_density() {
        let cells = vec![Point::new(2.0, 1.0); 10];

        let (size, count) = filler_layout(&cells, 100.0, 0.5);

        assert_eq!(size, Point::new(2.0, 1.0));
        assert_eq!(count, 15);
    }

    #[test]
    fn a_full_design_gets_no_fillers() {
        let cells = vec![Point::new(2.0, 1.0); 10];

        let (size, count) = filler_layout(&cells, 20.0, 0.9);

        assert_eq!(size, Point::new(2.0, 1.0));
        assert_eq!(count, 0);
    }
}
