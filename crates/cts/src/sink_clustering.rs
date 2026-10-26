//! Grouping of clock sinks under leaf buffers, as `TritonCTS`'s
//! `SinkClustering`: the sinks are ordered along a space-filling curve and
//! cut into runs, each closed when it would exceed the leaf buffer's load or
//! the cluster size and diameter; of the cuttings starting at each offset,
//! the one whose clusters are tightest wins.

// The arithmetic keeps TritonCTS's operations, whose rounding fusing would change.
#![allow(clippy::suboptimal_flops)]

use crate::htree::Point;

/// How much load a leaf buffer may drive, as a multiple of its own input
/// capacitance, as `kMaxCapFactor`.
const MAX_CAP_FACTOR: f64 = 10.0;

/// The clustering of one tree's sinks, kept across runs so the best
/// solution over every size and diameter tried survives.
#[derive(Clone, Debug)]
pub(crate) struct SinkClustering {
    points: Vec<Point>,
    caps: Vec<f32>,
    order: Vec<(f64, usize)>,
    best_solution: Vec<Vec<usize>>,
    best_cost: f64,
    max_internal_diameter: f32,
    cap_per_unit: f32,
    use_max_cap: bool,
    cap_per_dbu: f64,
    sink_buffer_input_cap: f64,
    first_run: bool,
    x_span: f64,
    y_span: f64,
}

impl SinkClustering {
    pub const fn new(use_max_cap: bool, cap_per_dbu: f64, sink_buffer_input_cap: f64) -> Self {
        Self {
            points: Vec::new(),
            caps: Vec::new(),
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
        }
    }

    pub fn add_point(&mut self, x: f64, y: f64, cap: f32) {
        self.points.push(Point::new(x, y));
        self.caps.push(cap);
    }

    /// The best clusters found so far, as indices of the points added.
    pub fn solution(&self) -> &[Vec<usize>] {
        &self.best_solution
    }

    /// Tries clusters of up to `group_size` sinks no wider than
    /// `max_diameter` length units, the tree's length unit being
    /// `scale_factor` database units, with `dist` the tree's distance.
    pub fn run(
        &mut self,
        group_size: usize,
        max_diameter: f32,
        scale_factor: i32,
        dist: &dyn Fn(Point, Point) -> f64,
    ) {
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
        self.find_best_matching(group_size, dist);
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
                *p = Point::new(x, y);
            }
        }
        let span = match (self.x_span == 0.0, self.y_span == 0.0) {
            (true, true) => 1.0,
            (true, false) => self.y_span,
            (false, true) => self.x_span,
            (false, false) => self.x_span.min(self.y_span),
        };
        self.max_internal_diameter = (f64::from(max_diameter) / span) as f32;
        self.cap_per_unit = (self.cap_per_dbu * f64::from(scale_factor) * span) as f32;
    }

    fn limit_exceeded(&self, size: usize, cost: f64, cap_cost: f64, size_limit: usize) -> bool {
        if self.use_max_cap {
            return cap_cost > self.sink_buffer_input_cap * MAX_CAP_FACTOR;
        }
        size >= size_limit || cost > f64::from(self.max_internal_diameter)
    }

    /// Cuts the curve order into clusters starting at each offset below
    /// `group_size`, as `SinkClustering::findBestMatching`.
    fn find_best_matching(&mut self, group_size: usize, dist: &dyn Fn(Point, Point) -> f64) {
        let mut runs: Vec<Run> = (0..group_size).map(|_| Run::default()).collect();
        let n = self.order.len();
        for i in 0..n {
            for (j, run) in runs.iter_mut().enumerate() {
                if i + j < n {
                    let idx = self.order[i + j].1;
                    self.extend(run, idx, group_size, dist);
                }
            }
        }
        for i in 0..group_size.min(n) {
            let idx = self.order[i].1;
            for run in runs.iter_mut().skip(i + 1) {
                self.extend(run, idx, group_size, dist);
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
    fn extend(
        &self,
        run: &mut Run,
        idx: usize,
        group_size: usize,
        dist: &dyn Fn(Point, Point) -> f64,
    ) {
        if run.clusters.is_empty() {
            run.clusters.push(Vec::new());
        }
        let p = self.points[idx];
        let open = run.clusters.last().map_or(&[][..], Vec::as_slice);
        let mut distance_cost = 0.0_f64;
        let mut cap_cost = f64::from(self.caps[idx]);
        for &other in open {
            let cost = dist(p, self.points[other]);
            if self.use_max_cap {
                cap_cost += cost * f64::from(self.cap_per_unit) + f64::from(self.caps[other]);
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
