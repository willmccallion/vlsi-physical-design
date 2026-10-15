//! Routability-driven placement in the style of `RePlAce`.
//!
//! Routing demand is estimated with RUDY: each net's wire spreads evenly
//! over its bounding box, its horizontal part as demand on horizontal
//! tracks and its vertical part on vertical ones, and a bin's congestion is
//! the larger ratio of demand to the tracks crossing it. When spreading is
//! nearly done, cells in congested bins grow by a power of their bin's
//! congestion; fillers of the same field shrink by the area its cells
//! gained, and the field's target density rises only once no filler area is
//! left. Placement then resumes
//! from an earlier, less spread state so the inflated cells push their
//! neighbours apart. This repeats until the most congested bins fall below
//! a target, or a few rounds pass, after which the inflation that measured
//! least congestion is kept.

use super::Field;
use super::nesterov::Objects;
use super::wirelength::Netlist;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::LayerId;
use pare_common::db::library::Direction;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::ops::RangeInclusive;

/// Density overflow at which the state to resume from is saved.
pub(super) const SNAPSHOT_OVERFLOW: f64 = 0.6;

/// Density overflow at which congestion is measured and cells inflated.
pub(super) const CHECK_OVERFLOW: f64 = 0.3;

/// Exponent of congestion giving a cell's area growth.
const INFLATION_EXPONENT: f64 = 2.5;

/// Largest area growth of one cell in one round.
const MAX_INFLATION: f64 = 2.5;

/// Highest target density inflation may raise the target to.
const MAX_DENSITY: f64 = 0.95;

/// Rounds of inflation.
const MAX_ROUNDS: usize = 4;

/// Fractions of the most congested bins averaged into the congestion metric.
const TOP_FRACTIONS: [f64; 2] = [0.005, 0.01];

/// Mean wirelength of a Steiner tree over `n` random points in area `A` is
/// about this times `sqrt(n A)` (Chung and Hwang).
const STEINER_COEFFICIENT: f64 = 0.7655;

/// Routing tracks per micron of die, after the share kept back from global
/// routing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackSupply {
    /// Horizontal tracks per micron of height, over all layers.
    pub horizontal: f64,
    /// Vertical tracks per micron of width, over all layers.
    pub vertical: f64,
}

impl TrackSupply {
    /// The tracks of the routing `layers` of `db`, keeping back the
    /// `reserve` fraction as global routing does.
    pub fn from_db(db: &NetlistDB, layers: RangeInclusive<LayerId>, reserve: f64) -> Self {
        let units = db.dbu();
        let mut supply = Self {
            horizontal: 0.0,
            vertical: 0.0,
        };
        for l in layers.start().index()..=layers.end().index() {
            let layer = db.library.layer(LayerId::new(l));
            let pitch = units.to_microns(layer.track_pitch());
            if pitch <= 0.0 {
                continue;
            }
            let tracks = (1.0 - reserve) / pitch;
            match layer.direction {
                Direction::Horizontal => supply.horizontal += tracks,
                Direction::Vertical => supply.vertical += tracks,
            }
        }
        supply
    }
}

/// Settings for routability-driven placement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoutabilityParams {
    /// Routing supply the congestion is measured against.
    pub supply: TrackSupply,
    /// Congestion metric at or below which no more inflation is done.
    pub target_congestion: f64,
}

/// One round of congestion measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CongestionRound {
    /// Iteration at which congestion was measured.
    pub iteration: usize,
    /// Mean demand over supply of the most congested bins.
    pub congestion: f64,
    /// Cells grown in this round.
    pub inflated: usize,
    /// Cell area after this round's growth, over the original cell area.
    pub area_ratio: f64,
    /// Target density after this round.
    pub target_density: f64,
}

/// Demand over supply of each bin of a square grid over the die.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct CongestionMap {
    dim: usize,
    origin: Point<f64>,
    bin: Point<f64>,
    ratio: Vec<f64>,
}

impl CongestionMap {
    /// RUDY congestion of `netlist` at `centers` on a `dim`×`dim` grid.
    pub(super) fn rudy(
        netlist: &Netlist,
        centers: &[Point<f64>],
        die: &Rect<f64>,
        dim: usize,
        supply: TrackSupply,
    ) -> Self {
        let bin = Point::new(die.width() / dim as f64, die.height() / dim as f64);
        let mut horizontal = vec![0.0; dim * dim];
        let mut vertical = vec![0.0; dim * dim];
        for (bbox, pins) in netlist.bounding_boxes(centers) {
            let (w, h) = (bbox.width(), bbox.height());
            if w + h <= 0.0 {
                continue;
            }
            let spread_w = w.max(bin.x);
            let spread_h = h.max(bin.y);
            let length = steiner_length(w, h, spread_w * spread_h, pins);
            let scale = length / (w + h) / (spread_w * spread_h);
            let centre = Point::new(
                f64::midpoint(bbox.min.x, bbox.max.x),
                f64::midpoint(bbox.min.y, bbox.max.y),
            );
            let half = Point::new(spread_w / 2.0, spread_h / 2.0);
            let spread = Rect::new(centre - half, centre + half);
            for_each_overlap(die.min, bin, dim, &spread, |idx, area| {
                horizontal[idx] = (area * scale).mul_add(w, horizontal[idx]);
                vertical[idx] = (area * scale).mul_add(h, vertical[idx]);
            });
        }
        let h_capacity = bin.x * bin.y * supply.horizontal;
        let v_capacity = bin.x * bin.y * supply.vertical;
        let ratio = horizontal
            .iter()
            .zip(&vertical)
            .map(|(&hd, &vd)| demand_ratio(hd, h_capacity).max(demand_ratio(vd, v_capacity)))
            .collect();
        Self {
            dim,
            origin: die.min,
            bin,
            ratio,
        }
    }

    /// The congestion of the bin holding `p`.
    fn at(&self, p: Point<f64>) -> f64 {
        let cell =
            |v: f64, o: f64, s: f64| (((v - o) / s).floor().max(0.0) as usize).min(self.dim - 1);
        let x = cell(p.x, self.origin.x, self.bin.x);
        let y = cell(p.y, self.origin.y, self.bin.y);
        self.ratio[y * self.dim + x]
    }

    /// The mean congestion of the most congested bins, averaged over the
    /// fractions in [`TOP_FRACTIONS`].
    pub(super) fn metric(&self) -> f64 {
        let mut sorted = self.ratio.clone();
        sorted.sort_by(|a, b| b.total_cmp(a));
        let top_mean = |fraction: f64| {
            let count = ((sorted.len() as f64 * fraction).ceil() as usize).clamp(1, sorted.len());
            sorted[..count].iter().sum::<f64>() / count as f64
        };
        TOP_FRACTIONS.iter().map(|&f| top_mean(f)).sum::<f64>() / TOP_FRACTIONS.len() as f64
    }
}

/// Estimated routed length of a net with a `w`×`h` bounding box spread
/// over `area`: its half-perimeter, or the Steiner tree length of `pins`
/// random points when that is longer.
fn steiner_length(w: f64, h: f64, area: f64, pins: usize) -> f64 {
    (w + h).max(STEINER_COEFFICIENT * (pins as f64 * area).sqrt())
}

fn demand_ratio(demand: f64, capacity: f64) -> f64 {
    if capacity > 0.0 {
        demand / capacity
    } else if demand > 0.0 {
        f64::INFINITY
    } else {
        0.0
    }
}

/// Calls `f` with the index and overlap area of each bin `rect` covers.
fn for_each_overlap(
    origin: Point<f64>,
    bin: Point<f64>,
    dim: usize,
    rect: &Rect<f64>,
    mut f: impl FnMut(usize, f64),
) {
    let span = |lo: f64, hi: f64, o: f64, s: f64| {
        let first = ((lo - o) / s).floor().max(0.0) as usize;
        let last = (((hi - o) / s).ceil().max(0.0) as usize).min(dim);
        (first..last).filter_map(move |i| {
            let start = (i as f64).mul_add(s, o);
            let len = hi.min(start + s) - lo.max(start);
            (len > 0.0).then_some((i, len))
        })
    };
    for (y, ly) in span(rect.min.y, rect.max.y, origin.y, bin.y) {
        for (x, lx) in span(rect.min.x, rect.max.x, origin.x, bin.x) {
            f(y * dim + x, lx * ly);
        }
    }
}

/// The sizes and each field's target density in effect when a congestion
/// was measured.
#[derive(Clone, Debug)]
struct Inflation {
    congestion: f64,
    sizes: Vec<Point<f64>>,
    targets: Vec<f64>,
}

/// What to do after measuring congestion.
#[derive(Debug, PartialEq)]
pub(super) enum Verdict {
    /// Congestion is low enough, or rounds are spent; carry on.
    Done,
    /// Cells were inflated; resume from the saved state.
    Resume,
}

/// Routability state across the rounds of one placement.
#[derive(Debug)]
pub(super) struct Routability {
    params: RoutabilityParams,
    original_area: f64,
    tried: Vec<Inflation>,
    /// Every measurement so far.
    pub(super) rounds: Vec<CongestionRound>,
    done: bool,
}

impl Routability {
    pub(super) fn new(params: RoutabilityParams, objects: &Objects) -> Self {
        Self {
            params,
            original_area: cell_area(objects),
            tried: Vec::new(),
            rounds: Vec::new(),
            done: false,
        }
    }

    pub(super) const fn active(&self) -> bool {
        !self.done
    }

    /// Measures congestion at `centers` and inflates `objects` in place,
    /// updating each field's target density, unless congestion is low
    /// enough or the rounds are spent. When the rounds are spent without
    /// reaching the target, the sizes that measured least congestion are
    /// restored.
    pub(super) fn check(
        &mut self,
        iteration: usize,
        map: &CongestionMap,
        objects: &mut Objects,
        fields: &mut [Field],
    ) -> Verdict {
        let congestion = map.metric();
        self.tried.push(Inflation {
            congestion,
            sizes: objects.sizes.clone(),
            targets: fields.iter().map(|f| f.grid.target()).collect(),
        });
        let mut round = CongestionRound {
            iteration,
            congestion,
            inflated: 0,
            area_ratio: cell_area(objects) / self.original_area,
            target_density: highest_target(fields),
        };
        if congestion <= self.params.target_congestion {
            self.done = true;
            self.rounds.push(round);
            return Verdict::Done;
        }
        if self.tried.len() > MAX_ROUNDS {
            self.done = true;
            if let Some(best) = self
                .tried
                .iter()
                .min_by(|a, b| a.congestion.total_cmp(&b.congestion))
            {
                objects.sizes.clone_from(&best.sizes);
                for (f, &t) in fields.iter_mut().zip(&best.targets) {
                    f.grid.set_target(t);
                }
            }
            round.area_ratio = cell_area(objects) / self.original_area;
            round.target_density = highest_target(fields);
            self.rounds.push(round);
            return Verdict::Done;
        }
        round.inflated = fields.iter_mut().map(|f| inflate(map, objects, f)).sum();
        round.area_ratio = cell_area(objects) / self.original_area;
        round.target_density = highest_target(fields);
        self.rounds.push(round);
        Verdict::Resume
    }
}

fn highest_target(fields: &[Field]) -> f64 {
    fields.iter().map(|f| f.grid.target()).fold(0.0, f64::max)
}

fn cell_area(objects: &Objects) -> f64 {
    objects.sizes[..objects.cells]
        .iter()
        .map(|s| s.x * s.y)
        .sum()
}

/// Grows every cell of `field` in a bin with congestion above one by that
/// congestion to the power [`INFLATION_EXPONENT`], at most
/// [`MAX_INFLATION`], and shrinks the field's fillers to make room, raising
/// its target density once they are gone. Growth is scaled down if it would
/// need a target above [`MAX_DENSITY`]. Returns how many cells grew.
fn inflate(map: &CongestionMap, objects: &mut Objects, field: &mut Field) -> usize {
    let cells = field.cells.clone();
    let growth: Vec<f64> = objects.centers[cells.clone()]
        .iter()
        .map(|&center| {
            let c = map.at(center);
            if c > 1.0 {
                c.powf(INFLATION_EXPONENT).min(MAX_INFLATION)
            } else {
                1.0
            }
        })
        .collect();
    let area = |s: Point<f64>| s.x * s.y;
    let current: f64 = objects.sizes[cells.clone()].iter().map(|&s| area(s)).sum();
    let wanted: f64 = objects.sizes[cells.clone()]
        .iter()
        .zip(&growth)
        .map(|(&s, g)| area(s) * (g - 1.0))
        .sum();
    let fillers: f64 = objects.sizes[field.fillers.clone()]
        .iter()
        .map(|&s| area(s))
        .sum();
    let free_area = field.grid.free_area();
    let room = MAX_DENSITY.mul_add(free_area, -current).max(0.0);
    let scale = if wanted > room && wanted > 0.0 {
        room / wanted
    } else {
        1.0
    };
    let mut inflated = 0;
    for (size, g) in objects.sizes[cells].iter_mut().zip(&growth) {
        if *g > 1.0 {
            *size = *size * scale.mul_add(g - 1.0, 1.0).sqrt();
            inflated += 1;
        }
    }
    let added = wanted * scale;
    let left = (fillers - added).max(0.0);
    let shrink = if fillers > 0.0 {
        (left / fillers).sqrt()
    } else {
        0.0
    };
    for size in &mut objects.sizes[field.fillers.clone()] {
        *size = *size * shrink;
    }
    if free_area > 0.0 {
        let needed = (current + added) / free_area;
        field
            .grid
            .set_target(field.grid.target().max(needed).min(MAX_DENSITY));
    }
    inflated
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::global::density::DensityGrid;
    use crate::global::wirelength::Terminal;
    use crate::zones::Zone;

    fn die(side: f64) -> Rect<f64> {
        Rect::new(Point::new(0.0, 0.0), Point::new(side, side))
    }

    fn two_pin(netlist: &mut Netlist, a: Point<f64>, b: Point<f64>) {
        netlist.push([Terminal::Fixed(a), Terminal::Fixed(b)], 1.0);
    }

    #[test]
    fn a_horizontal_net_puts_its_length_on_the_horizontal_tracks_of_its_row() {
        let mut netlist = Netlist::default();
        two_pin(&mut netlist, Point::new(0.5, 1.5), Point::new(3.5, 1.5));
        let supply = TrackSupply {
            horizontal: 2.0,
            vertical: 2.0,
        };

        let map = CongestionMap::rudy(&netlist, &[], &die(4.0), 4, supply);

        let row: Vec<f64> = map.ratio[4..8].to_vec();
        assert!((row[1] - 0.5).abs() < 1e-12, "{row:?}");
        assert!((row[0] - 0.25).abs() < 1e-12, "{row:?}");
        assert!(map.ratio[..4].iter().all(|&r| r == 0.0));
    }

    #[test]
    fn many_pins_are_charged_a_steiner_tree_longer_than_the_half_perimeter() {
        assert!((steiner_length(10.0, 10.0, 100.0, 2) - 20.0).abs() < 1e-12);
        assert!(steiner_length(10.0, 10.0, 100.0, 1000) > 200.0);
    }

    #[test]
    fn the_metric_averages_the_most_congested_bins() {
        let mut ratio = vec![0.0; 400];
        ratio[0] = 3.0;
        ratio[1] = 1.0;
        ratio[2] = 1.0;
        ratio[3] = 1.0;
        let map = CongestionMap {
            dim: 20,
            origin: Point::new(0.0, 0.0),
            bin: Point::new(1.0, 1.0),
            ratio,
        };

        assert!((map.metric() - f64::midpoint(2.0, 1.5)).abs() < 1e-12);
    }

    #[test]
    fn inflation_takes_area_from_the_fillers_before_raising_the_target() {
        let mut objects = Objects {
            centers: vec![
                Point::new(0.5, 0.5),
                Point::new(3.5, 3.5),
                Point::new(2.0, 2.0),
            ],
            sizes: vec![
                Point::new(1.0, 1.0),
                Point::new(1.0, 1.0),
                Point::new(2.0, 2.0),
            ],
            cells: 2,
        };
        let mut ratio = vec![0.0; 16];
        ratio[0] = (1.0 / INFLATION_EXPONENT).exp2();
        let map = CongestionMap {
            dim: 4,
            origin: Point::new(0.0, 0.0),
            bin: Point::new(1.0, 1.0),
            ratio,
        };
        let mut field = Field {
            zone: Zone::Outside,
            grid: DensityGrid::new(&die(4.0), 4, 0.5, &[], &[]),
            area: vec![die(4.0)],
            cells: 0..2,
            fillers: 2..3,
            utilization: 2.0 / 16.0,
        };

        let inflated = inflate(&map, &mut objects, &mut field);

        assert_eq!(inflated, 1);
        assert!(objects.sizes[0].x.mul_add(objects.sizes[0].y, -2.0).abs() < 1e-9);
        assert!(objects.sizes[2].x.mul_add(objects.sizes[2].y, -3.0).abs() < 1e-9);
        assert!((field.grid.target() - 0.5).abs() < 1e-12);
    }
}
