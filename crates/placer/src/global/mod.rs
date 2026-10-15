//! Global placement in the style of ePlace and `RePlAce`.
//!
//! Cells are electric charges: the field of their density spreads them
//! apart while a smooth wirelength model pulls connected cells together.
//! Filler cells take up the whitespace, so a design that uses a small part
//! of its die still packs to the target density instead of scattering.
//! Nesterov's method minimizes the sum, raising the weight of density until
//! the cell area above the target density falls below a threshold. With
//! routability enabled, cells in bins whose estimated routing demand exceeds
//! the tracks there are inflated along the way.
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

pub use routability::{CongestionRound, RoutabilityParams, TrackSupply};

use crate::model::{PinAnchor, PlacementModel};
use crate::zones::Zone;
use density::DensityGrid;
use nesterov::Objects;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
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
    /// How each zone with cells ended.
    pub zones: Vec<ZoneOutcome>,
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
    utilization: f64,
}

/// How far above its cells' utilization a zone's target density is set
/// when the requested target is lower, so that it can still be met.
const ZONE_DENSITY_MARGIN: f64 = 0.01;

/// Fewest and most bins per side of the density grid.
const MIN_BINS: usize = 16;
const MAX_BINS: usize = 512;

/// Bins per side for `model`: about one per cell, but no shorter than the
/// row pitch when channels separate the rows.
///
/// Each bin then averages rows and channels, so cells may cross a channel
/// to a less crowded row instead of facing a wall of blocked bins;
/// legalization moves them into the rows.
pub fn default_bins(model: &PlacementModel) -> usize {
    let cells = model.movable.iter().filter(|&&m| m).count();
    let by_cells = ((cells as f64).sqrt().ceil() as usize)
        .next_power_of_two()
        .clamp(64, MAX_BINS);
    match channel_pitch(&model.rows) {
        Some(pitch) => {
            let rows = (model.die.height() / pitch).floor().max(1.0) as usize;
            let by_pitch = 1usize << rows.ilog2();
            by_cells.min(by_pitch).max(MIN_BINS)
        }
        None => by_cells,
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
pub fn place(
    model: &mut PlacementModel,
    params: &GlobalParams,
    report: impl FnMut(&Progress),
) -> GlobalResult {
    let mut movable: Vec<usize> = (0..model.num_cells())
        .filter(|&i| model.movable[i])
        .collect();
    movable.sort_by_key(|&i| (model.zone_of[i], i));
    let fixed: Vec<Rect<f64>> = (0..model.num_cells())
        .filter(|&i| !model.movable[i])
        .map(|i| Rect::new(model.positions[i], model.positions[i] + model.sizes[i]))
        .collect();

    let mut rng = StdRng::seed_from_u64(params.seed);
    let mut objects = Objects {
        centers: initial_centers(model, &movable, &mut rng),
        sizes: movable.iter().map(|&i| model.sizes[i]).collect(),
        cells: movable.len(),
    };
    let mut fields = build_fields(model, &movable, &fixed, params);
    for field in &mut fields {
        let cells = &objects.sizes[field.cells.clone()];
        let (size, count) = filler_layout(cells, field.grid.free_area(), field.grid.target());
        let first = objects.centers.len();
        for _ in 0..count {
            objects.centers.push(random_point(&field.area, &mut rng));
            objects.sizes.push(size);
        }
        field.fillers = first..objects.centers.len();
    }
    let netlist = build_netlist(model, &movable);

    let result = nesterov::minimize(
        &mut objects,
        &model.die,
        &mut fields,
        &netlist,
        model.site_width,
        params,
        report,
    );

    for (k, &i) in movable.iter().enumerate() {
        model.positions[i] = objects.centers[k] - model.sizes[i] * 0.5;
    }
    result
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
    let filler_area = target.mul_add(free_area, -cell_area);
    if cells.is_empty() || filler_area <= 0.0 {
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
        let terminals = net.pins.iter().map(|pin| match *pin {
            PinAnchor::Cell { cell, offset } => match object_of_cell[cell] {
                Some(index) => Terminal::Object {
                    index,
                    offset: offset - model.sizes[cell] * 0.5,
                },
                None => Terminal::Fixed(model.positions[cell] + offset),
            },
            PinAnchor::Fixed(p) => Terminal::Fixed(p),
        });
        netlist.push(terminals, net.weight);
    }
    netlist
}

#[cfg(test)]
mod tests {
    use super::*;

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

        let (_, count) = filler_layout(&cells, 20.0, 0.9);

        assert_eq!(count, 0);
    }
}
