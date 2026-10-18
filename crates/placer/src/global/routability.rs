//! Routability-driven placement as `OpenROAD`'s global placer does it.
//!
//! Routing demand is estimated with RUDY over the global router's gcells,
//! per direction as `DREAMPlace` does: each net lays horizontal wire as long
//! as its bounding box is wide and vertical wire as long as it is tall,
//! each spread evenly over the box and weighted by [`risa_weight`], and a
//! gcell's congestion is the larger of the two directions' share of track
//! length that this wire and the obstructions take, leaving out a direction
//! obstructions nearly fill there. Once spreading
//! has nearly finished and settled, standard cells in congested gcells grow;
//! their field removes fillers to make room and raises its target density by
//! whatever the fillers cannot cover. Placement then resumes from an earlier,
//! less spread snapshot so the grown cells push their neighbours apart. The
//! total growth is held to a budget spent over several rounds. Rounds end,
//! keeping the growth, once congestion falls below a target; when congestion
//! stops improving, a field would need a density above [`MAX_DENSITY`], the
//! budget is spent or [`MAX_ROUNDS`] pass, the sizes revert to those of the
//! round that measured least congestion.

use super::Field;
use super::nesterov::Objects;
use super::wirelength::Netlist;
use crate::model::CellKind;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// Mean density overflow of the fields at which the state to resume from is
/// saved.
pub(super) const SNAPSHOT_OVERFLOW: f64 = 0.6;

/// Mean density overflow of the fields at or below which congestion is
/// measured, once placement has settled.
pub(super) const CHECK_OVERFLOW: f64 = 0.3;

/// A field has settled once an iteration moves its objects at most this
/// fraction of the most any iteration has.
pub(super) const SETTLE_FRACTION: f64 = 0.6;

/// Gcells with at least this share of their capacity blocked are neither
/// measured nor acted on, and a direction with this share of its tracks
/// blocked in a gcell is not measured there.
const IGNORE_BLOCKAGE: f64 = 0.8;

/// Congestion from which a gcell's cells grow.
const MIN_CONGESTION_FOR_INFLATION: f64 = 0.95;

/// Power of congestion over [`MIN_CONGESTION_FOR_INFLATION`] that a cell's
/// area grows by.
const INFLATION_EXPONENT: i32 = 2;

/// Largest area growth of one cell in one round.
const MAX_INFLATION: f64 = 3.0;

/// Growth allowed over all rounds, as a share of the original cell area.
const MAX_INFLATION_TOTAL: f64 = 0.30;

/// One round may spend at most the growth budget divided by this.
const BUDGET_PASSES: f64 = 6.0;

/// Highest target density routability may raise a field to.
const MAX_DENSITY: f64 = 0.99;

/// Relative drop in congestion that counts as a new minimum.
const MIN_IMPROVEMENT: f64 = 0.005;

/// Rounds in a row without a new minimum after which routability stops.
const MAX_STALLED_ROUNDS: usize = 3;

/// Rounds after which routability stops.
const MAX_ROUNDS: usize = 50;

/// Fractions of the most congested gcells averaged into the congestion
/// metric.
const TOP_FRACTIONS: [f64; 2] = [0.005, 0.01];

/// The global router's gcells, with each one's routing capacity in each
/// direction and the part of it obstructions take.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingTiles {
    /// Lower-left corner of the grid.
    pub origin: Point<f64>,
    /// Edge of one gcell.
    pub size: f64,
    /// Columns.
    pub nx: usize,
    /// Rows.
    pub ny: usize,
    /// Tracks of the horizontal layers.
    pub horizontal: TileTracks,
    /// Tracks of the vertical layers.
    pub vertical: TileTracks,
}

/// Tracks of one direction through each gcell, row by row.
#[derive(Clone, Debug, PartialEq)]
pub struct TileTracks {
    /// Tracks before any reduction.
    pub capacity: Vec<f64>,
    /// Tracks taken before placement.
    pub reduction: Vec<f64>,
}

impl TileTracks {
    /// The share of gcell `i`'s track length, `length` per track, that
    /// `wire` and the reduction take together, as `DREAMPlace` counts
    /// blockages as demand already there. None when obstructions take
    /// [`IGNORE_BLOCKAGE`] of the tracks or more, as wire routes around
    /// such a gcell rather than through it.
    fn utilization(&self, i: usize, wire: f64, length: f64) -> Option<f64> {
        let capacity = self.capacity.get(i).copied().unwrap_or(0.0);
        let taken = self.reduction.get(i).copied().unwrap_or(0.0);
        (capacity > 0.0 && taken < IGNORE_BLOCKAGE * capacity)
            .then(|| (wire + taken * length) / (capacity * length))
    }
}

impl RoutingTiles {
    /// The share of gcell `i`'s capacity over both directions that is
    /// taken, all of it when it has none.
    fn blockage(&self, i: usize) -> f64 {
        let at = |v: &[f64]| v.get(i).copied().unwrap_or(0.0);
        let capacity = at(&self.horizontal.capacity) + at(&self.vertical.capacity);
        if capacity <= 0.0 {
            return 1.0;
        }
        ((at(&self.horizontal.reduction) + at(&self.vertical.reduction)) / capacity).min(1.0)
    }

    /// The length of column or row `i` of an axis starting at `origin`,
    /// clipped to `end`.
    fn length(&self, i: usize, origin: f64, end: f64) -> f64 {
        let start = (i as f64).mul_add(self.size, origin);
        ((start + self.size).min(end) - start).max(0.0)
    }

    /// The column or row of each gcell `span` of an axis overlaps, with the
    /// length of the overlap and the share of the span it holds. A span of
    /// no length lies in the one gcell holding it, all of it there.
    fn axis_shares(&self, (lo, hi): (f64, f64), origin: f64, end: f64, n: usize) -> Vec<AxisShare> {
        let size = self.size;
        let length = |i: usize| self.length(i, origin, end);
        let cell = |v: f64| (((v - origin) / size).floor().max(0.0) as usize).min(n - 1);
        if hi <= lo {
            let i = cell(lo);
            return vec![AxisShare {
                index: i,
                overlap: 0.0,
                share: 1.0,
            }];
        }
        (cell(lo)..=cell(hi))
            .filter_map(|i| {
                let start = (i as f64).mul_add(size, origin);
                let overlap = hi.min(start + length(i)) - lo.max(start);
                (overlap > 0.0).then(|| AxisShare {
                    index: i,
                    overlap,
                    share: overlap / (hi - lo),
                })
            })
            .collect()
    }
}

/// Where a net's box meets one column or row of gcells.
#[derive(Clone, Copy, Debug, PartialEq)]
struct AxisShare {
    index: usize,
    /// Length of the box inside the column or row.
    overlap: f64,
    /// That length over the box's length, or 1 for a box of no length.
    share: f64,
}

/// The wire a net with `pins` pins takes over the half perimeter of its
/// box, from the wiring distribution map of RISA (Cheng, ICCAD 1994) as
/// `DREAMPlace` uses it.
const fn risa_weight(pins: usize) -> f64 {
    match pins {
        0..=3 => 1.0,
        4 => 1.0828,
        5 => 1.1536,
        6 => 1.2206,
        7 => 1.2823,
        8 => 1.3385,
        9 => 1.3991,
        10 => 1.4493,
        11..=15 => 1.6899,
        16..=20 => 1.8924,
        21..=25 => 2.0743,
        26..=30 => 2.2334,
        31..=35 => 2.3892,
        36..=40 => 2.5356,
        41..=45 => 2.6625,
        _ => 2.7933,
    }
}

/// Routing congestion over the gcells of [`RoutingTiles`] measured outside
/// the placer, such as by a global router.
pub trait CongestionEstimator {
    /// The congestion of every gcell, row by row, with each of the model's
    /// cells centered at its entry of `centers`, in microns: the share of
    /// the routing capacity left to the gcell that its routes take.
    fn congestion(&mut self, centers: &[Point<f64>]) -> Vec<f64>;
}

/// Settings for routability-driven placement.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutabilityParams {
    /// Where congestion is measured, and what is left to route through.
    pub tiles: RoutingTiles,
    /// Congestion metric below which no more inflation is done.
    pub target_congestion: f64,
}

/// One round of congestion measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CongestionRound {
    /// Iteration at which congestion was measured.
    pub iteration: usize,
    /// Mean congestion of the most congested gcells.
    pub congestion: f64,
    /// Cells grown in this round.
    pub inflated: usize,
    /// Cell area after this round, over the original cell area.
    pub area_ratio: f64,
    /// Highest target density of any field after this round.
    pub target_density: f64,
}

/// RUDY congestion of every gcell.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct CongestionMap {
    origin: Point<f64>,
    size: f64,
    nx: usize,
    ny: usize,
    /// Demand over the capacity left, infinite where none is.
    congestion: Vec<f64>,
    /// Whether the placer measures and acts on each gcell: whether a
    /// standard cell's center lies in it and obstructions leave enough.
    actionable: Vec<bool>,
}

impl CongestionMap {
    /// RUDY congestion of `netlist` with `objects` where they are, over the
    /// gcells of `params` clipped to `die`: in each gcell, the larger of the
    /// horizontal and the vertical [`TileTracks::utilization`], or none
    /// when neither is measured.
    pub(super) fn rudy(
        netlist: &Netlist,
        objects: &Objects,
        die: &Rect<f64>,
        params: &RoutabilityParams,
    ) -> Self {
        let t = &params.tiles;
        let mut horizontal = vec![0.0; t.nx * t.ny];
        let mut vertical = vec![0.0; t.nx * t.ny];
        for (bbox, pins) in netlist.bounding_boxes(&objects.centers) {
            let weight = risa_weight(pins);
            let columns = t.axis_shares((bbox.min.x, bbox.max.x), t.origin.x, die.max.x, t.nx);
            let rows = t.axis_shares((bbox.min.y, bbox.max.y), t.origin.y, die.max.y, t.ny);
            for row in &rows {
                for column in &columns {
                    let idx = row.index * t.nx + column.index;
                    horizontal[idx] = (weight * column.overlap).mul_add(row.share, horizontal[idx]);
                    vertical[idx] = (weight * row.overlap).mul_add(column.share, vertical[idx]);
                }
            }
        }
        let congestion = (0..t.nx * t.ny)
            .map(|i| {
                let width = t.length(i % t.nx, t.origin.x, die.max.x);
                let height = t.length(i / t.nx, t.origin.y, die.max.y);
                let across = t.horizontal.utilization(i, horizontal[i], width);
                let up = t.vertical.utilization(i, vertical[i], height);
                across.into_iter().chain(up).fold(0.0, f64::max)
            })
            .collect();
        Self::over(congestion, objects, params)
    }

    /// The congestion a [`CongestionEstimator`] measured, over the gcells
    /// of `params`.
    pub(super) fn measured(
        congestion: Vec<f64>,
        objects: &Objects,
        params: &RoutabilityParams,
    ) -> Self {
        Self::over(congestion, objects, params)
    }

    /// `congestion` over the gcells of `params`, acted on where a standard
    /// cell of `objects` is centered and obstructions leave enough.
    fn over(congestion: Vec<f64>, objects: &Objects, params: &RoutabilityParams) -> Self {
        let t = &params.tiles;
        let mut map = Self {
            origin: t.origin,
            size: t.size,
            nx: t.nx,
            ny: t.ny,
            congestion,
            actionable: vec![false; t.nx * t.ny],
        };
        for i in 0..objects.cells() {
            if objects.kinds[i] == CellKind::Standard
                && let Some(idx) = map.index(objects.centers[i])
            {
                map.actionable[idx] = true;
            }
        }
        for (i, a) in map.actionable.iter_mut().enumerate() {
            *a &= t.blockage(i) < IGNORE_BLOCKAGE;
        }
        map
    }

    fn index(&self, p: Point<f64>) -> Option<usize> {
        if self.nx == 0 || self.ny == 0 || self.size <= 0.0 {
            return None;
        }
        let cell =
            |v: f64, o: f64, n: usize| (((v - o) / self.size).floor().max(0.0) as usize).min(n - 1);
        Some(cell(p.y, self.origin.y, self.ny) * self.nx + cell(p.x, self.origin.x, self.nx))
    }

    /// The area growth of a standard cell centered at `p`: the congestion of
    /// its gcell over [`MIN_CONGESTION_FOR_INFLATION`] to the power
    /// [`INFLATION_EXPONENT`], at most [`MAX_INFLATION`], and none below that
    /// congestion or in a gcell the placer does not act on.
    fn inflation_at(&self, p: Point<f64>) -> f64 {
        let Some(i) = self.index(p) else {
            return 1.0;
        };
        let c = self.congestion[i];
        if !self.actionable[i] || c < MIN_CONGESTION_FOR_INFLATION {
            return 1.0;
        }
        (c / MIN_CONGESTION_FOR_INFLATION)
            .powi(INFLATION_EXPONENT)
            .min(MAX_INFLATION)
    }

    /// The mean congestion of the most congested gcells the placer acts on,
    /// averaged over the fractions in [`TOP_FRACTIONS`].
    pub(super) fn metric(&self) -> f64 {
        let mut sorted: Vec<f64> = self
            .congestion
            .iter()
            .zip(&self.actionable)
            .filter(|&(c, &a)| a && c.is_finite() && *c >= 0.0)
            .map(|(&c, _)| c)
            .collect();
        if sorted.is_empty() {
            return 0.0;
        }
        sorted.sort_by(|a, b| b.total_cmp(a));
        let top_mean = |fraction: f64| {
            let count = ((sorted.len() as f64 * fraction).ceil() as usize).clamp(1, sorted.len());
            sorted[..count].iter().sum::<f64>() / count as f64
        };
        TOP_FRACTIONS.iter().map(|&f| top_mean(f)).sum::<f64>() / TOP_FRACTIONS.len() as f64
    }
}

/// The growth in effect at the round that measured least congestion.
#[derive(Clone, Debug)]
struct Minimum {
    congestion: f64,
    sizes: Vec<Point<f64>>,
    targets: Vec<f64>,
    kept_fillers: Vec<usize>,
    grown: Vec<f64>,
}

/// What to do after measuring congestion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Verdict {
    /// Congestion is below the target: carry on from the current state,
    /// without further rounds.
    Done,
    /// Cells grew: resume from the snapshot and measure again.
    Resume,
    /// Routability is over and sizes reverted to the round with least
    /// congestion: resume from the snapshot without further rounds.
    Revert,
}

/// Routability state across the rounds of one placement.
#[derive(Debug)]
pub(super) struct Routability {
    params: RoutabilityParams,
    /// The budget's base: standard cell area, and macro area at its field's
    /// target density.
    original_area: f64,
    /// All cell area before any growth.
    initial_cell_area: f64,
    /// Area each field's cells have grown by.
    grown: Vec<f64>,
    minimum: Option<Minimum>,
    stalled: usize,
    calls: usize,
    /// Every measurement so far.
    pub(super) rounds: Vec<CongestionRound>,
    done: bool,
}

impl Routability {
    pub(super) fn new(params: RoutabilityParams, objects: &Objects, fields: &[Field]) -> Self {
        let original_area = fields
            .iter()
            .flat_map(|f| {
                let target = f.grid.target();
                f.cells.clone().map(move |i| (i, target))
            })
            .map(|(i, target)| {
                let area = objects.sizes[i].x * objects.sizes[i].y;
                match objects.kinds[i] {
                    CellKind::Standard => area,
                    CellKind::Macro => area * target,
                }
            })
            .sum();
        Self {
            params,
            original_area,
            initial_cell_area: cell_area(objects),
            grown: vec![0.0; fields.len()],
            minimum: None,
            stalled: 0,
            calls: 0,
            rounds: Vec::new(),
            done: false,
        }
    }

    pub(super) const fn active(&self) -> bool {
        !self.done
    }

    pub(super) const fn params(&self) -> &RoutabilityParams {
        &self.params
    }

    /// Measures the congestion of `map` and grows the congested standard
    /// cells of `fields`, deciding what placement does next.
    pub(super) fn check(
        &mut self,
        iteration: usize,
        map: &CongestionMap,
        objects: &mut Objects,
        fields: &mut [Field],
    ) -> Verdict {
        self.calls += 1;
        let congestion = map.metric();
        let mut round = CongestionRound {
            iteration,
            congestion,
            inflated: 0,
            area_ratio: cell_area(objects) / self.initial_cell_area,
            target_density: highest_target(fields),
        };
        if congestion < self.params.target_congestion {
            self.done = true;
            self.rounds.push(round);
            return Verdict::Done;
        }
        self.track_minimum(congestion, objects, fields);

        let budget = self.budget_scale(map, objects);
        let mut over_density = false;
        for (k, field) in fields.iter_mut().enumerate() {
            let (inflated, added) = grow(map, objects, field, budget.scale);
            round.inflated += inflated;
            self.grown[k] += added;
            remove_fillers(objects, field, added);
            if field.grid.target() > MAX_DENSITY {
                over_density = true;
                break;
            }
        }
        let ended = over_density
            || self.calls >= MAX_ROUNDS
            || self.stalled >= MAX_STALLED_ROUNDS
            || budget.spent;
        let verdict = if ended {
            self.revert(objects, fields);
            self.done = true;
            Verdict::Revert
        } else {
            Verdict::Resume
        };
        round.area_ratio = cell_area(objects) / self.initial_cell_area;
        round.target_density = highest_target(fields);
        self.rounds.push(round);
        verdict
    }

    /// Keeps the growth in effect as the new minimum when `congestion`
    /// improves on the old one by [`MIN_IMPROVEMENT`], or counts a round
    /// without improvement.
    fn track_minimum(&mut self, congestion: f64, objects: &Objects, fields: &[Field]) {
        let improved = self
            .minimum
            .as_ref()
            .is_none_or(|m| m.congestion - congestion > m.congestion * MIN_IMPROVEMENT);
        if !improved {
            self.stalled += 1;
            return;
        }
        self.minimum = Some(Minimum {
            congestion,
            sizes: objects.sizes[..objects.cells()].to_vec(),
            targets: fields.iter().map(|f| f.grid.target()).collect(),
            kept_fillers: fields.iter().map(|f| f.kept_fillers).collect(),
            grown: self.grown.clone(),
        });
        self.stalled = 0;
    }

    /// How much of the growth `map` asks for this round may take: at most the
    /// whole budget over [`BUDGET_PASSES`], and no more than is left.
    fn budget_scale(&self, map: &CongestionMap, objects: &Objects) -> BudgetScale {
        let budget = MAX_INFLATION_TOTAL * self.original_area;
        let spent: f64 = self.grown.iter().sum();
        let allowed = (budget / BUDGET_PASSES).min(budget - spent);
        let wanted: f64 = (0..objects.cells())
            .filter(|&i| objects.kinds[i] == CellKind::Standard)
            .map(|i| {
                let size = objects.sizes[i];
                size.x * size.y * (map.inflation_at(objects.centers[i]) - 1.0)
            })
            .sum();
        if wanted <= 0.0 || wanted <= allowed {
            return BudgetScale {
                scale: 1.0,
                spent: spent + wanted >= budget,
            };
        }
        BudgetScale {
            scale: (allowed / wanted).max(0.0),
            spent: spent + allowed >= budget,
        }
    }

    /// Restores the sizes, target densities and fillers of the round that
    /// measured least congestion.
    fn revert(&mut self, objects: &mut Objects, fields: &mut [Field]) {
        let Some(m) = &self.minimum else {
            return;
        };
        objects.sizes[..m.sizes.len()].copy_from_slice(&m.sizes);
        for ((f, &target), &kept) in fields.iter_mut().zip(&m.targets).zip(&m.kept_fillers) {
            f.grid.set_target(target);
            f.kept_fillers = kept;
        }
        self.grown.clone_from(&m.grown);
    }
}

/// The share of its asked-for growth a round takes, and whether that spends
/// the budget.
#[derive(Clone, Copy, Debug, PartialEq)]
struct BudgetScale {
    scale: f64,
    spent: bool,
}

fn highest_target(fields: &[Field]) -> f64 {
    fields.iter().map(|f| f.grid.target()).fold(0.0, f64::max)
}

fn cell_area(objects: &Objects) -> f64 {
    objects.sizes[..objects.cells()]
        .iter()
        .map(|s| s.x * s.y)
        .sum()
}

/// Grows every standard cell of `field` in a congested gcell by `scale` of
/// the growth `map` asks for. Returns how many cells grew and the area they
/// gained.
fn grow(map: &CongestionMap, objects: &mut Objects, field: &Field, scale: f64) -> (usize, f64) {
    let mut inflated = 0;
    let mut added = 0.0;
    for i in field.cells.clone() {
        if objects.kinds[i] != CellKind::Standard {
            continue;
        }
        let ratio = scale.mul_add(map.inflation_at(objects.centers[i]) - 1.0, 1.0);
        if ratio <= 1.0 {
            continue;
        }
        let before = objects.sizes[i].x * objects.sizes[i].y;
        objects.sizes[i] = objects.sizes[i] * ratio.sqrt();
        added += objects.sizes[i].x.mul_add(objects.sizes[i].y, -before);
        inflated += 1;
    }
    (inflated, added)
}

/// Makes room in `field` for `added` area of cell growth: removes whole
/// fillers, the last kept first, and when more than one filler's area is
/// still missing, raises the target density to cover the cells and the
/// fillers left.
fn remove_fillers(objects: &Objects, field: &mut Field, added: f64) {
    let single = field.filler.x * field.filler.y;
    let removed = if single > 0.0 {
        ((added / single).floor() as usize).min(field.kept_fillers)
    } else {
        0
    };
    field.kept_fillers -= removed;
    let missing = (removed as f64).mul_add(-single, added);
    let free = field.grid.free_area();
    if missing <= single || free <= 0.0 {
        return;
    }
    let cells: f64 = objects.sizes[field.cells.clone()]
        .iter()
        .map(|s| s.x * s.y)
        .sum();
    let total = (field.kept_fillers as f64).mul_add(single, cells);
    field.grid.set_target(total / free);
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

    /// A 4x4 grid of unit gcells of 5 tracks each way, `taken` of the 10
    /// reduced, half in each direction.
    fn params(taken: &[f64]) -> RoutabilityParams {
        let half: Vec<f64> = taken.iter().map(|t| t / 2.0).collect();
        RoutabilityParams {
            tiles: RoutingTiles {
                origin: Point::new(0.0, 0.0),
                size: 1.0,
                nx: 4,
                ny: 4,
                horizontal: TileTracks {
                    capacity: vec![5.0; 16],
                    reduction: half.clone(),
                },
                vertical: TileTracks {
                    capacity: vec![5.0; 16],
                    reduction: half,
                },
            },
            target_congestion: 1.0,
        }
    }

    fn map_of(congestion: Vec<f64>) -> CongestionMap {
        let n = congestion.len();
        CongestionMap {
            origin: Point::new(0.0, 0.0),
            size: 1.0,
            nx: 4,
            ny: n / 4,
            congestion,
            actionable: vec![true; n],
        }
    }

    /// Standard cells of unit size at `centers`.
    fn cells_at(centers: &[Point<f64>]) -> Objects {
        Objects {
            centers: centers.to_vec(),
            sizes: vec![Point::new(1.0, 1.0); centers.len()],
            kinds: vec![CellKind::Standard; centers.len()],
        }
    }

    fn two_pin(a: Point<f64>, b: Point<f64>) -> Netlist {
        let mut netlist = Netlist::default();
        netlist.push([Terminal::Fixed(a), Terminal::Fixed(b)], 1.0);
        netlist
    }

    fn field(cells: std::ops::Range<usize>, fillers: std::ops::Range<usize>) -> Field {
        Field {
            zone: Zone::Outside,
            grid: DensityGrid::new(&die(4.0), 4, 0.5, &[], &[]),
            area: vec![die(4.0)],
            cells,
            kept_fillers: fillers.len(),
            fillers,
            filler: Point::new(1.0, 1.0),
            utilization: 0.125,
        }
    }

    #[test]
    fn a_net_spreads_its_wire_of_each_direction_evenly_over_its_box() {
        let netlist = two_pin(Point::new(0.5, 1.25), Point::new(3.5, 1.75));

        let map = CongestionMap::rudy(&netlist, &cells_at(&[]), &die(4.0), &params(&[0.0; 16]));

        assert!(
            (map.congestion[5] - 1.0 / 5.0).abs() < 1e-12,
            "{:?}",
            map.congestion
        );
        assert!(
            (map.congestion[4] - 0.5 / 5.0).abs() < 1e-12,
            "{:?}",
            map.congestion
        );
        assert!(map.congestion[..4].iter().all(|&c| c == 0.0));
    }

    #[test]
    fn a_flat_net_lays_its_wire_along_its_row() {
        let netlist = two_pin(Point::new(0.5, 1.5), Point::new(2.5, 1.5));

        let map = CongestionMap::rudy(&netlist, &cells_at(&[]), &die(4.0), &params(&[0.0; 16]));

        let expected = [0.1, 0.2, 0.1, 0.0];
        for (got, want) in map.congestion[4..8].iter().zip(expected) {
            assert!((got - want).abs() < 1e-12, "{:?}", map.congestion);
        }
        assert!(map.congestion[8..].iter().all(|&c| c == 0.0));
    }

    #[test]
    fn the_direction_with_fewer_tracks_sets_the_congestion() {
        let netlist = two_pin(Point::new(0.0, 0.0), Point::new(4.0, 4.0));
        let mut params = params(&[0.0; 16]);
        params.tiles.horizontal.capacity = vec![8.0; 16];
        params.tiles.vertical.capacity = vec![2.0; 16];

        let map = CongestionMap::rudy(&netlist, &cells_at(&[]), &die(4.0), &params);

        assert!(
            (map.congestion[0] - 0.25 / 2.0).abs() < 1e-12,
            "{:?}",
            map.congestion
        );
    }

    #[test]
    fn a_net_of_many_pins_lays_more_wire_than_its_half_perimeter() {
        let mut netlist = Netlist::default();
        let corners = [(0.0, 0.0), (4.0, 0.0), (0.0, 4.0), (4.0, 4.0), (2.0, 2.0)];
        netlist.push(corners.map(|(x, y)| Terminal::Fixed(Point::new(x, y))), 1.0);

        let map = CongestionMap::rudy(&netlist, &cells_at(&[]), &die(4.0), &params(&[0.0; 16]));

        assert!(
            (map.congestion[0] - 1.1536 * 0.25 / 5.0).abs() < 1e-12,
            "{:?}",
            map.congestion
        );
    }

    #[test]
    fn obstructions_count_as_demand_already_there() {
        let netlist = two_pin(Point::new(0.0, 0.0), Point::new(4.0, 4.0));
        let mut taken = vec![0.0; 16];
        taken[0] = 5.0;

        let map = CongestionMap::rudy(&netlist, &cells_at(&[]), &die(4.0), &params(&taken));

        assert!(
            (map.congestion[0] - (0.25 + 2.5) / 5.0).abs() < 1e-12,
            "{:?}",
            map.congestion
        );
        assert!(
            (map.congestion[2] - 0.25 / 5.0).abs() < 1e-12,
            "{:?}",
            map.congestion
        );
    }

    #[test]
    fn a_direction_obstructions_nearly_fill_is_not_measured() {
        let netlist = two_pin(Point::new(0.0, 0.0), Point::new(4.0, 4.0));
        let mut params = params(&[0.0; 16]);
        params.tiles.horizontal.reduction[0] = 4.5;

        let map = CongestionMap::rudy(&netlist, &cells_at(&[]), &die(4.0), &params);

        assert!(
            (map.congestion[0] - 0.25 / 5.0).abs() < 1e-12,
            "{:?}",
            map.congestion
        );
    }

    #[test]
    fn only_gcells_holding_a_standard_cell_and_enough_capacity_are_acted_on() {
        let mut objects = cells_at(&[
            Point::new(0.5, 0.5),
            Point::new(1.5, 0.5),
            Point::new(2.5, 0.5),
        ]);
        objects.kinds[2] = CellKind::Macro;
        let mut taken = vec![0.0; 16];
        taken[1] = 8.0;

        let map = CongestionMap::rudy(&Netlist::default(), &objects, &die(4.0), &params(&taken));

        assert_eq!(map.actionable[..4], [true, false, false, false]);
    }

    #[test]
    fn a_gcell_without_capacity_is_fully_blocked() {
        let mut tiles = params(&[0.0; 16]).tiles;
        tiles.horizontal.capacity[3] = 0.0;
        tiles.vertical.capacity[3] = 0.0;

        assert!((tiles.blockage(3) - 1.0).abs() < 1e-12);
        assert!(tiles.blockage(2).abs() < 1e-12);
    }

    #[test]
    fn the_metric_averages_the_most_congested_gcells() {
        let mut congestion = vec![0.0; 400];
        congestion[0] = 3.0;
        congestion[1] = 1.0;
        congestion[2] = 1.0;
        congestion[3] = 1.0;

        assert!((map_of(congestion).metric() - f64::midpoint(2.0, 1.5)).abs() < 1e-12);
    }

    #[test]
    fn measured_congestion_is_taken_as_given_where_the_placer_acts() {
        let params = params(&[0.0; 16]);
        let objects = cells_at(&[Point::new(0.5, 0.5)]);
        let mut congestion = vec![0.0; 16];
        congestion[0] = 2.0;
        congestion[5] = 2.0;

        let map = CongestionMap::measured(congestion, &objects, &params);

        assert!((map.inflation_at(Point::new(0.5, 0.5)) - MAX_INFLATION).abs() < 1e-12);
        assert!((map.inflation_at(Point::new(1.5, 1.5)) - 1.0).abs() < 1e-12);
        assert!((map.metric() - 2.0).abs() < 1e-12);
    }

    #[test]
    fn cells_grow_by_the_squared_congestion_over_the_threshold_up_to_a_cap() {
        let map = map_of(vec![0.95 * std::f64::consts::SQRT_2, 9.0, 0.9, 1.0]);

        assert!((map.inflation_at(Point::new(0.5, 0.5)) - 2.0).abs() < 1e-12);
        assert!((map.inflation_at(Point::new(1.5, 0.5)) - 3.0).abs() < 1e-12);
        assert!((map.inflation_at(Point::new(2.5, 0.5)) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn growth_removes_whole_fillers_from_the_last_kept() {
        let mut objects = cells_at(&[Point::new(0.5, 0.5), Point::new(0.4, 0.6)]);
        objects.centers.extend([Point::new(2.0, 2.0); 3]);
        objects.sizes.extend([Point::new(1.0, 1.0); 3]);
        let mut congestion = vec![0.0; 16];
        congestion[0] = 0.95 * 1.5;
        let mut f = field(0..2, 2..5);

        let (inflated, added) = grow(&map_of(congestion), &mut objects, &f, 1.0);
        remove_fillers(&objects, &mut f, added);

        assert_eq!(inflated, 2);
        assert!((added - 2.5).abs() < 1e-9);
        assert_eq!(f.kept_fillers, 1);
        assert!((f.grid.target() - 0.5).abs() < 1e-12);
    }

    #[test]
    fn growth_past_the_fillers_raises_the_target_to_cover_the_cells() {
        let objects = Objects {
            centers: vec![Point::new(1.0, 1.0)],
            sizes: vec![Point::new(3.0, 3.0)],
            kinds: vec![CellKind::Standard],
        };
        let mut f = field(0..1, 1..1);

        remove_fillers(&objects, &mut f, 5.0);

        assert!((f.grid.target() - 9.0 / 16.0).abs() < 1e-12);
    }

    #[test]
    fn a_round_takes_at_most_a_sixth_of_the_growth_budget() {
        let objects = cells_at(&[Point::new(0.5, 0.5); 10]);
        let routability = Routability::new(params(&[0.0; 16]), &objects, &[field(0..10, 10..10)]);
        let mut congestion = vec![0.0; 16];
        congestion[0] = 9.0;

        let scale = routability.budget_scale(&map_of(congestion), &objects);

        let allowed = 0.30 * 10.0 / 6.0;
        assert!((scale.scale - allowed / 20.0).abs() < 1e-12);
        assert!(!scale.spent);
    }

    #[test]
    fn congestion_below_the_target_keeps_the_sizes_and_ends_routability() {
        let mut objects = cells_at(&[Point::new(0.5, 0.5)]);
        let mut fields = [field(0..1, 1..1)];
        let mut routability = Routability::new(params(&[0.0; 16]), &objects, &fields);

        let verdict = routability.check(0, &map_of(vec![0.5; 16]), &mut objects, &mut fields);

        assert_eq!(verdict, Verdict::Done);
        assert!(!routability.active());
    }

    #[test]
    fn three_rounds_without_improvement_revert_to_the_least_congested_sizes() {
        let mut objects = cells_at(&[Point::new(0.5, 0.5)]);
        let mut fields = [field(0..1, 1..1)];
        let mut routability = Routability::new(params(&[0.0; 16]), &objects, &fields);
        let mut congestion = vec![0.0; 16];
        congestion[0] = 1.2;

        let verdicts: Vec<Verdict> = (0..4)
            .map(|k| routability.check(k, &map_of(congestion.clone()), &mut objects, &mut fields))
            .collect();

        assert_eq!(
            verdicts,
            [
                Verdict::Resume,
                Verdict::Resume,
                Verdict::Resume,
                Verdict::Revert
            ]
        );
        assert_eq!(objects.sizes[0], Point::new(1.0, 1.0));
        assert!((fields[0].grid.target() - 0.5).abs() < 1e-12);
    }
}
