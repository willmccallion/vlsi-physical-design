//! Grouping of clock sinks under leaf buffers, as `TritonCTS`'s
//! `SinkClustering`: the sinks are ordered along a space-filling curve and
//! cut into runs, each closed when it would exceed the leaf buffer's load or
//! the cluster size and diameter; of the cuttings starting at each offset,
//! the one whose clusters are tightest wins.

// The arithmetic keeps TritonCTS's operations, whose rounding fusing would change.
#![allow(clippy::suboptimal_flops)]

use crate::capacitance::Farads32;
use crate::htree::TreePoint;
use crate::techchar::SegmentLength;
use pare_common::units::FaradsPerDbu;

/// How much load a leaf buffer may drive, as a multiple of its own input
/// capacitance, as `kMaxCapFactor`.
const MAX_CAP_FACTOR: f64 = 10.0;

/// The clustering of one tree's sinks, kept across runs so the best
/// solution over every size and diameter tried survives.
#[derive(Clone, Debug)]
pub(crate) struct SinkClustering {
    points: Vec<TreePoint>,
    caps: Vec<Farads32>,
    insertion_lengths: Vec<SegmentLength>,
    order: Vec<(f64, usize)>,
    best_solution: Vec<Vec<usize>>,
    best_cost: f64,
    max_internal_diameter: f32,
    cap_per_unit: f32,
    use_max_cap: bool,
    cap_per_dbu: FaradsPerDbu,
    sink_buffer_input_cap: Farads32,
    first_run: bool,
    x_span: f64,
    y_span: f64,
    /// The length that maps to one unit of the normalized square.
    span: f64,
}

impl SinkClustering {
    pub const fn new(
        use_max_cap: bool,
        cap_per_dbu: FaradsPerDbu,
        sink_buffer_input_cap: Farads32,
    ) -> Self {
        Self {
            points: Vec::new(),
            caps: Vec::new(),
            insertion_lengths: Vec::new(),
            order: Vec::new(),
            best_solution: Vec::new(),
            best_cost: f64::MAX,
            max_internal_diameter: 10.0,
            cap_per_unit: 0.0,
            use_max_cap,
            cap_per_dbu,
            sink_buffer_input_cap,
            first_run: true,
            x_span: 0.0,
            y_span: 0.0,
            span: 1.0,
        }
    }

    /// Adds a sink at (`x`, `y`) in wire segment units with input
    /// capacitance `cap`, standing for `insertion` more clock wire beyond
    /// its pin.
    pub(crate) fn add_point(&mut self, x: f64, y: f64, cap: Farads32, insertion: SegmentLength) {
        self.points.push(TreePoint::new(x, y));
        self.caps.push(cap);
        self.insertion_lengths.push(insertion);
    }

    /// The best clusters found so far, as indices of the points added.
    pub fn solution(&self) -> &[Vec<usize>] {
        &self.best_solution
    }

    /// Tries clusters of up to `group_size` sinks no wider than
    /// `max_diameter` length units, the tree's length unit being
    /// `scale_factor` database units.
    pub fn run(&mut self, group_size: usize, max_diameter: f32, scale_factor: i32) {
        self.normalize_points(max_diameter, scale_factor);
        if self.first_run {
            self.order = self
                .points
                .iter()
                .enumerate()
                .map(|(i, p)| (theta(p.x, p.y), i))
                .collect();
            self.order
                .sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        }
        self.find_best_matching(group_size);
        self.first_run = false;
    }

    fn normalize_points(&mut self, max_diameter: f32, scale_factor: i32) {
        if self.first_run {
            let (mut x_min, mut x_max) = (f64::INFINITY, f64::NEG_INFINITY);
            let (mut y_min, mut y_max) = (f64::INFINITY, f64::NEG_INFINITY);
            for p in &self.points {
                x_min = x_min.min(p.x);
                x_max = x_max.max(p.x);
                y_min = y_min.min(p.y);
                y_max = y_max.max(p.y);
            }
            self.x_span = x_max - x_min;
            self.y_span = y_max - y_min;
            let (x_span, y_span) = (self.x_span, self.y_span);
            for p in &mut self.points {
                let x = if x_span == 0.0 {
                    0.0
                } else {
                    (p.x - x_min) / x_span
                };
                let y = if y_span == 0.0 {
                    0.0
                } else {
                    (p.y - y_min) / y_span
                };
                *p = TreePoint::new(x, y);
            }
        }
        self.span = match (self.x_span == 0.0, self.y_span == 0.0) {
            (true, true) => 1.0,
            (true, false) => self.y_span,
            (false, true) => self.x_span,
            (false, false) => self.x_span.min(self.y_span),
        };
        self.max_internal_diameter = (f64::from(max_diameter) / self.span) as f32;
        self.cap_per_unit = (self.cap_per_dbu.get() * f64::from(scale_factor) * self.span) as f32;
    }

    fn limit_exceeded(&self, size: usize, cost: f64, cap_cost: f64, size_limit: usize) -> bool {
        if self.use_max_cap {
            return cap_cost > f64::from(self.sink_buffer_input_cap.get()) * MAX_CAP_FACTOR;
        }
        size >= size_limit || cost > f64::from(self.max_internal_diameter)
    }

    /// The distance between points `a` and `b` in the normalized square,
    /// with the clock wire their sinks stand for, as
    /// `TreeBuilder::computeDist` measures it.
    fn distance(&self, a: usize, b: usize) -> f64 {
        // NOTE: diverges from TritonCTS, whose SinkClustering hands the
        // normalized points to HTreeBuilder::computeDist, which looks the
        // insertion delays up by unnormalized location and so never finds
        // them; PARE adds each sink's length, normalized like the diameter.
        let insertion = self.insertion_lengths[a].get() + self.insertion_lengths[b].get();
        self.points[a].dist(self.points[b]) + insertion / self.span
    }

    /// Cuts the curve order into clusters starting at each offset below
    /// `group_size`, as `SinkClustering::findBestMatching`.
    fn find_best_matching(&mut self, group_size: usize) {
        let mut runs: Vec<Run> = (0..group_size).map(|_| Run::default()).collect();
        let n = self.order.len();
        for i in 0..n {
            for (j, run) in runs.iter_mut().enumerate() {
                if i + j < n {
                    let idx = self.order[i + j].1;
                    self.extend(run, idx, group_size);
                }
            }
        }
        for i in 0..group_size.min(n) {
            let idx = self.order[i].1;
            for run in runs.iter_mut().skip(i + 1) {
                self.extend(run, idx, group_size);
            }
        }
        let mut best = None;
        for (j, run) in runs.iter().enumerate().skip(1) {
            if run.cost < self.best_cost {
                best = Some(j);
                self.best_cost = run.cost;
            }
        }
        if let Some(j) = best {
            self.best_solution = std::mem::take(&mut runs[j].clusters);
        }
    }

    /// Adds point `idx` to the open cluster of `run`, closing it first if
    /// the point would break its limits.
    fn extend(&self, run: &mut Run, idx: usize, group_size: usize) {
        if run.clusters.is_empty() {
            run.clusters.push(Vec::new());
        }
        let open = run.clusters.last().map_or(&[][..], Vec::as_slice);
        let mut distance_cost = 0.0_f64;
        let mut cap_cost = f64::from(self.caps[idx].get());
        for &other in open {
            let cost = self.distance(idx, other);
            if self.use_max_cap {
                cap_cost += cost * f64::from(self.cap_per_unit) + f64::from(self.caps[other].get());
            }
            distance_cost = distance_cost.max(cost);
        }
        if self.limit_exceeded(open.len(), distance_cost, cap_cost, group_size) {
            if run.previous_cost == 0.0 {
                run.previous_cost = f64::from(self.max_internal_diameter);
            }
            run.cost += run.previous_cost;
            run.clusters.push(Vec::new());
            run.previous_cost = 0.0;
        } else {
            run.previous_cost = run.previous_cost.max(distance_cost);
        }
        if let Some(cluster) = run.clusters.last_mut() {
            cluster.push(idx);
        }
    }
}

/// The clusters cut from one offset, and their cost so far.
#[derive(Clone, Debug, Default)]
struct Run {
    clusters: Vec<Vec<usize>>,
    cost: f64,
    previous_cost: f64,
}

/// A point's position along the space-filling curve over the unit square,
/// as `SinkClustering::computeTheta`.
fn theta(x: f64, y: f64) -> f64 {
    let is_one = |v: f64| (1.0 - v).abs() < f64::EPSILON;
    if is_one(x) && is_one(y) {
        return 0.5;
    }
    let qx = ((2.0 * x) as u32).min(1);
    let qy = ((2.0 * y) as u32).min(1);
    let quad = match (qx, qy) {
        (0, 0) => 0,
        (0, 1) => 1,
        (1, 1) => 2,
        _ => 3,
    };
    let mut t = theta(2.0 * (x - 0.5).abs(), 2.0 * (y - 0.5).abs());
    if quad % 2 == 1 {
        t = 1.0 - t;
    }
    ((f64::from(quad) + t) / 4.0 + 7.0 / 8.0).fract()
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::units::Farads;

    const FEMTOFARAD: f64 = 1e-15;

    /// Sinks of `cap_ff` femtofarads at the corners of a square 100 length
    /// units a side, added counterclockwise from the origin.
    fn square_corners(use_max_cap: bool, cap_ff: f64) -> SinkClustering {
        let sink_buffer_input_cap = Farads32::of(Farads::new(FEMTOFARAD));
        let mut clustering =
            SinkClustering::new(use_max_cap, FaradsPerDbu::new(0.0), sink_buffer_input_cap);
        for (x, y) in [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)] {
            clustering.add_point(
                x,
                y,
                Farads32::of(Farads::new(cap_ff * FEMTOFARAD)),
                SegmentLength::default(),
            );
        }
        clustering
    }

    #[test]
    fn the_curve_visits_the_unit_squares_corners_up_across_and_down() {
        let corners = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)];

        let thetas = corners.map(|(x, y)| theta(x, y));

        let expected = [0.0, 0.25, 0.5, 0.75];
        assert!(
            thetas
                .iter()
                .zip(expected)
                .all(|(t, e)| (t - e).abs() < f64::EPSILON),
            "{thetas:?}"
        );
    }

    #[test]
    fn sinks_are_cut_along_the_curve_into_groups_of_the_size() {
        let mut clustering = square_corners(false, 1.0);

        clustering.run(2, 1000.0, 1);

        assert_eq!(clustering.solution(), [vec![3, 2], vec![1, 0]]);
    }

    #[test]
    fn sinks_farther_apart_than_the_diameter_stand_alone() {
        let mut clustering = square_corners(false, 1.0);

        clustering.run(4, 50.0, 1);

        assert_eq!(clustering.solution(), [vec![3], vec![2], vec![1], vec![0]]);
    }

    #[test]
    fn under_a_load_limit_the_load_cuts_clusters_rather_than_the_size() {
        // Ten times the sink buffer's input capacitance holds two sinks of
        // four times it, not three.
        let mut clustering = square_corners(true, 4.0);

        clustering.run(4, 1000.0, 1);

        assert_eq!(clustering.solution(), [vec![3, 2], vec![1, 0]]);
    }

    #[test]
    fn a_group_size_of_one_finds_no_clustering() {
        let mut clustering = square_corners(false, 1.0);

        clustering.run(1, 1000.0, 1);

        assert_eq!(clustering.solution(), &[] as &[Vec<usize>]);
    }

    #[test]
    fn a_later_run_keeps_the_earlier_clustering_unless_it_costs_less() {
        let mut clustering = square_corners(false, 1.0);
        clustering.run(2, 1000.0, 1);

        clustering.run(4, 50.0, 1);

        assert_eq!(clustering.solution(), [vec![3, 2], vec![1, 0]]);
    }
}
