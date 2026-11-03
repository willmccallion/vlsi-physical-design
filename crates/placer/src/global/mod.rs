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
mod initial;
mod nesterov;
mod poisson;
mod routability;
mod wirelength;

pub use initial::INITIAL_PLACE_ROUNDS;
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
    /// Most rounds of `gpl`'s bound-to-bound initial placement; with none,
    /// cells stay where they start.
    pub initial_rounds: usize,
    /// Where cells start before initial placement.
    pub start: Start,
    /// Bins of every density grid; with none, each zone's grid is sized as
    /// `gpl` sizes it.
    pub bins: Option<BinCount>,
    /// Seed for the initial placement.
    pub seed: u64,
    /// Inflation of cells in routing-congested bins, if enabled.
    pub routability: Option<RoutabilityParams>,
    /// Repair of the design for timing along the way, if enabled.
    pub timing: Option<TimingParams>,
}

/// Bins along each side of a density grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BinCount {
    /// Columns.
    pub x: usize,
    /// Rows.
    pub y: usize,
}

impl BinCount {
    /// As many columns as rows.
    #[must_use]
    pub const fn square(n: usize) -> Self {
        Self { x: n, y: n }
    }
}

/// Where movable cells start before initial placement, as `gpl` chooses
/// with `forceCenterInitialPlace`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Start {
    /// At the center of their fence or of the core.
    Center,
    /// A cell that came placed at its own center, unless it is in a fence;
    /// the rest at the center of their fence or of the core.
    Placed,
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

/// Fewest bins per side of a density grid over rows separated by channels.
const MIN_CHANNEL_BINS: usize = 16;

/// The largest power of two whose square `gpl` tries as a bin count; past
/// it, it settles on twice that.
const MAX_GPL_BINS: i64 = 1024;

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

/// Bins for the grid of `model`'s cells outside every fence, at
/// `target_density`: as `gpl` counts them, but no shorter than the row
/// pitch when channels separate the rows.
#[must_use]
pub fn default_bins(model: &PlacementModel, target_density: f64) -> BinCount {
    let outside: Vec<usize> = (0..model.num_cells())
        .filter(|&i| model.movable[i] && model_zone(model, i) == Some(Zone::Outside))
        .collect();
    zone_bins(model, &core(model), &outside, target_density)
}

fn model_zone(model: &PlacementModel, cell: usize) -> Option<Zone> {
    model.zones.get(model.zone_of[cell]).map(|z| z.zone)
}

/// Bins for a zone's grid over `region`, holding the movable `cells` at
/// `target_density`: as `gpl` counts them, but no shorter than the row
/// pitch when channels separate the rows.
///
/// Each bin then averages rows and channels, so cells may cross a channel
/// to a less crowded row instead of facing a wall of blocked bins;
/// legalization moves them into the rows.
fn zone_bins(
    model: &PlacementModel,
    region: &Rect<f64>,
    cells: &[usize],
    target_density: f64,
) -> BinCount {
    let dbu = |microns: f64| model.units.to_dbu_rounded(microns).get();
    let areas: Vec<i64> = cells
        .iter()
        .map(|&i| dbu(model.density_sizes[i].x) * dbu(model.density_sizes[i].y))
        .collect();
    let by_area = gpl_bins(
        dbu(region.width()),
        dbu(region.height()),
        &areas,
        target_density,
    );
    match channel_pitch(&model.rows) {
        Some(pitch) => {
            let rows = (region.height() / pitch).floor().max(1.0) as usize;
            let by_pitch = 1usize << rows.ilog2();
            BinCount {
                x: by_area.x.min(by_pitch).max(MIN_CHANNEL_BINS),
                y: by_area.y.min(by_pitch).max(MIN_CHANNEL_BINS),
            }
        }
        None => by_area,
    }
}

/// `gpl`'s bin count for a region `width` by `height` holding cells of
/// `areas`, all in database units: the smallest power of two `n` from 2
/// whose `n` by `n` times the aspect ratio bins number at least a quarter
/// of the ideal count, the region's area over the average cell's at the
/// target density. The longer side takes the aspect ratio, rounded down to
/// a power of two.
fn gpl_bins(width: i64, height: i64, areas: &[i64], target_density: f64) -> BinCount {
    let average = if areas.is_empty() {
        0
    } else {
        areas.iter().sum::<i64>() / areas.len() as i64
    };
    let ideal_bin_area = if target_density == 0.0 {
        0
    } else {
        (average as f32 / target_density as f32).round() as i64
    };
    let ideal = if ideal_bin_area == 0 {
        0
    } else {
        width * height / ideal_bin_area
    }
    .max(4);
    let ratio = 1i64
        << (width.max(height) / width.min(height).max(1))
            .max(1)
            .ilog2();
    let mut n = 2;
    while n <= MAX_GPL_BINS {
        let bins = n * n * ratio;
        if (n == 2 || bins <= ideal) && 4 * bins > ideal {
            break;
        }
        n *= 2;
    }
    let (short, long) = (n as usize, (n * ratio) as usize);
    if width > height {
        BinCount { x: long, y: short }
    } else {
        BinCount { x: short, y: long }
    }
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
        let netlist = build_netlist(model, &movable);
        let mut objects = Objects {
            centers: initial_centers(model, &movable, &netlist, params, rng),
            sizes: movable.iter().map(|&i| model.density_sizes[i]).collect(),
            kinds: movable.iter().map(|&i| model.kinds[i]).collect(),
        };
        let mut fields = build_fields(model, &movable, &fixed, params);
        for field in &mut fields {
            add_fillers(field, &mut objects, rng);
        }
        Self {
            netlist,
            model_sizes: movable.iter().map(|&i| model.sizes[i]).collect(),
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
        let model_sizes = movable.iter().map(|&i| model.sizes[i]).collect();
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
        let region = match zone.zone {
            Zone::Fence(_) => bounding_box(&area),
            Zone::Outside => core(model),
        };
        let bins = params.bins.unwrap_or_else(|| {
            zone_bins(model, &region, &movable[start..end], params.target_density)
        });
        let mut grid = DensityGrid::new(&region, bins, params.target_density, &area, fixed);
        let cell_area: f64 = movable[start..end]
            .iter()
            .map(|&i| model.density_sizes[i].x * model.density_sizes[i].y)
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

/// The bounding box of the rows, as `OpenROAD`'s core area, or the die for
/// a design without rows.
fn core(model: &PlacementModel) -> Rect<f64> {
    bounding_box(&placeable_area(model))
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

/// Where the movable cells start, as `gpl` starts them: from `params.start`,
/// moved by up to `params.initial_rounds` rounds of initial placement over
/// `netlist`, each then shifted at random by up to a micron on each axis.
fn initial_centers(
    model: &PlacementModel,
    movable: &[usize],
    netlist: &Netlist,
    params: &GlobalParams,
    rng: &mut StdRng,
) -> Vec<Point<f64>> {
    let core = core(model);
    let fences: Vec<Option<Rect<f64>>> = movable
        .iter()
        .map(|&i| match model.zones.get(model.zone_of[i]) {
            Some(zone) if matches!(zone.zone, Zone::Fence(_)) => Some(bounding_box(&zone.rects)),
            _ => None,
        })
        .collect();
    let mut centers: Vec<Point<f64>> = movable
        .iter()
        .zip(&fences)
        .map(|(&i, fence)| match fence {
            Some(fence) => center_of(fence),
            None if params.start == Start::Placed && model.placed[i] => {
                model.positions[i] + model.sizes[i] * 0.5
            }
            None => center_of(&core),
        })
        .collect();
    let bounds: Vec<Rect<f64>> = fences.iter().map(|f| f.unwrap_or(core)).collect();
    let min_diff = model.units.to_microns(initial::MIN_DIFF_LENGTH);
    initial::place(
        netlist,
        &mut centers,
        core,
        &bounds,
        params.initial_rounds,
        min_diff,
    );
    for center in &mut centers {
        *center += Point::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0));
    }
    centers
}

const fn center_of(rect: &Rect<f64>) -> Point<f64> {
    Point::new(
        f64::midpoint(rect.min.x, rect.max.x),
        f64::midpoint(rect.min.y, rect.max.y),
    )
}

fn bounding_box(rects: &[Rect<f64>]) -> Rect<f64> {
    rects
        .iter()
        .copied()
        .reduce(|a, b| {
            Rect::new(
                Point::new(a.min.x.min(b.min.x), a.min.y.min(b.min.y)),
                Point::new(a.max.x.max(b.max.x), a.max.y.max(b.max.y)),
            )
        })
        .unwrap_or_default()
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
            (terminal, pin.average)
        });
        netlist.push(pins, net.weight);
    }
    netlist
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::zones::Zones;
    use pare_common::db::parser::{def, lef};
    use std::path::Path;

    fn tiny_model() -> PlacementModel {
        let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("../common/tests/fixtures");
        let units = def::read_units(&fixtures.join("tiny.def")).unwrap();
        let library = lef::parse_files(&[fixtures.join("tiny.lef")], units).unwrap();
        let db = def::parse(&fixtures.join("tiny.def"), library).unwrap();
        PlacementModel::from_db(&db, &Zones::new(&db).unwrap())
    }

    /// Where the fixture's first cell, placed in it, starts from `start`.
    fn first_cell_start(model: &PlacementModel, start: Start) -> Point<f64> {
        let params = GlobalParams {
            target_density: 0.7,
            target_overflow: 0.1,
            max_iterations: 0,
            initial_rounds: 0,
            start,
            bins: None,
            seed: 0,
            routability: None,
            timing: None,
        };
        let movable = movable_cells(model);
        let netlist = build_netlist(model, &movable);
        let mut rng = StdRng::seed_from_u64(0);
        let centers = initial_centers(model, &movable, &netlist, &params, &mut rng);
        centers[movable.iter().position(|&i| i == 0).unwrap()]
    }

    fn within_a_micron(a: Point<f64>, b: Point<f64>) -> bool {
        (a.x - b.x).abs() <= 1.0 && (a.y - b.y).abs() <= 1.0
    }

    #[test]
    fn a_placed_cell_starts_at_its_own_center_when_placed_cells_keep_their_place() {
        let model = tiny_model();
        let own_center = model.positions[0] + model.sizes[0] * 0.5;

        let start = first_cell_start(&model, Start::Placed);

        assert!(model.placed[0]);
        assert!(within_a_micron(start, own_center), "{start:?}");
    }

    #[test]
    fn a_placed_cell_starts_at_the_core_center_when_cells_start_centered() {
        let model = tiny_model();
        let core_center = center_of(&core(&model));

        let start = first_cell_start(&model, Start::Center);

        assert!(within_a_micron(start, core_center), "{start:?}");
    }

    #[test]
    fn bins_hold_about_one_average_cell_at_the_target_density() {
        // jpeg in Nangate45 without its I/O pins: a core of 668,800 by 666,400
        // units, cells of 1.560 square microns on average, placed at 0.906.
        // OpenROAD counts 64,711 ideal bins and picks 128 a side.
        let bins = gpl_bins(668_800, 666_400, &[6_239_859], 0.906);

        assert_eq!(bins, BinCount::square(128));
    }

    #[test]
    fn bins_double_once_the_ideal_count_reaches_four_times_their_square() {
        let one_short = gpl_bins(255, 256, &[1], 1.0);
        let exact = gpl_bins(256, 256, &[1], 1.0);

        assert_eq!(one_short, BinCount::square(128));
        assert_eq!(exact, BinCount::square(256));
    }

    #[test]
    fn a_region_smaller_than_a_cell_gets_two_bins_a_side() {
        assert_eq!(gpl_bins(10, 10, &[100], 0.5), BinCount::square(2));
        assert_eq!(gpl_bins(10, 10, &[], 0.5), BinCount::square(2));
    }

    #[test]
    fn bins_stop_doubling_past_1024_a_side() {
        assert_eq!(
            gpl_bins(1 << 20, 1 << 20, &[1], 1.0),
            BinCount::square(2048)
        );
    }

    #[test]
    fn a_wide_region_gets_more_columns_by_its_aspect_ratio() {
        assert_eq!(gpl_bins(400, 100, &[1], 1.0), BinCount { x: 256, y: 64 });
        assert_eq!(gpl_bins(100, 400, &[1], 1.0), BinCount { x: 64, y: 256 });
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
