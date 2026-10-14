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

mod density;
mod nesterov;
mod poisson;
mod routability;
mod wirelength;

pub use routability::{CongestionRound, RoutabilityParams, TrackSupply};

use crate::model::{PinAnchor, PlacementModel};
use density::DensityGrid;
use nesterov::Objects;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
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
}

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
    match channel_pitch(&model.regions) {
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
    let movable: Vec<usize> = (0..model.num_cells())
        .filter(|&i| model.movable[i])
        .collect();
    let fixed: Vec<Rect<f64>> = (0..model.num_cells())
        .filter(|&i| !model.movable[i])
        .map(|i| Rect::new(model.positions[i], model.positions[i] + model.sizes[i]))
        .collect();
    let mut grid = DensityGrid::new(
        &model.die,
        params.bins,
        params.target_density,
        &model.regions,
        &fixed,
    );

    let mut rng = StdRng::seed_from_u64(params.seed);
    let cell_sizes: Vec<Point<f64>> = movable.iter().map(|&i| model.sizes[i]).collect();
    let (filler_size, fillers) =
        filler_layout(&cell_sizes, grid.free_area(), params.target_density);
    let mut objects = Objects {
        centers: initial_centers(model, &movable, &mut rng),
        sizes: cell_sizes,
        cells: movable.len(),
    };
    let regions = placeable_regions(model);
    for _ in 0..fillers {
        objects.centers.push(random_point(&regions, &mut rng));
        objects.sizes.push(filler_size);
    }
    let netlist = build_netlist(model, &movable);

    let result = nesterov::minimize(
        &mut objects,
        &model.die,
        &mut grid,
        &netlist,
        params,
        report,
    );

    for (k, &i) in movable.iter().enumerate() {
        model.positions[i] = objects.centers[k] - model.sizes[i] * 0.5;
    }
    result
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
fn placeable_regions(model: &PlacementModel) -> Vec<Rect<f64>> {
    if model.regions.is_empty() {
        vec![model.die]
    } else {
        model.regions.clone()
    }
}

/// Cell centers in a small random cluster at the middle of the rows.
fn initial_centers(model: &PlacementModel, movable: &[usize], rng: &mut StdRng) -> Vec<Point<f64>> {
    let regions = placeable_regions(model);
    let lo = regions.iter().fold(model.die.max, |p, r| {
        Point::new(p.x.min(r.min.x), p.y.min(r.min.y))
    });
    let hi = regions.iter().fold(model.die.min, |p, r| {
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

/// A uniformly random point in the union of disjoint `regions`.
fn random_point(regions: &[Rect<f64>], rng: &mut StdRng) -> Point<f64> {
    let total: f64 = regions.iter().map(Rect::area).sum();
    let mut pick = rng.gen_range(0.0..=total);
    let region = regions
        .iter()
        .find(|r| {
            let inside = pick <= r.area();
            pick -= r.area();
            inside
        })
        .or_else(|| regions.last());
    region.map_or(Point::new(0.0, 0.0), |r| {
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
