//! Capacitated k-means splitting a branch's sinks between its two new
//! branching points, as `TritonCTS`'s `CKMeans::Clustering`.
//!
//! Each round assigns the sinks by a minimum-cost flow under the clusters'
//! capacities, moves each mean to its sinks' centroid and keeps the means
//! one segment apart about the branching point. `TritonCTS` asks LEMON's
//! network simplex for more flow than there are sinks; the flow it leaves
//! is the cheapest assignment of every sink that fits the capacities, and
//! among equally cheap ones the simplex's own choice, so the simplex is
//! run as LEMON runs it.

use pare_common::convert::SaturatingFrom;
use pare_common::network_simplex::{Arc, min_cost_flow};

/// The farthest a sink may be from a mean and still be assigned to it, as
/// `Clustering::Kmeans` passes `minCostFlow`.
const MAX_DIST: f32 = 5200.0;

/// A sink or branching point in the tree's length units, in the single
/// precision `TritonCTS` keeps sinks in.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct SinkPoint {
    pub x: f32,
    pub y: f32,
}

impl SinkPoint {
    pub(crate) const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

/// A sink to cluster, in the tree's length units.
#[derive(Clone, Copy, Debug)]
struct Sink {
    at: SinkPoint,
    cluster: Option<usize>,
}

/// The clustering of one branch's sinks.
#[derive(Clone, Debug)]
pub(crate) struct Clustering {
    sinks: Vec<Sink>,
    clusters: Vec<Vec<usize>>,
    segment_length: f32,
    branching_point: SinkPoint,
}

const fn dist(a: SinkPoint, b: SinkPoint) -> f32 {
    (a.x - b.x).abs() + (a.y - b.y).abs()
}

impl Clustering {
    pub fn new(sinks: &[SinkPoint], branching_point: SinkPoint) -> Self {
        Self {
            sinks: sinks.iter().map(|&at| Sink { at, cluster: None }).collect(),
            clusters: Vec::new(),
            segment_length: 0.0,
            branching_point,
        }
    }

    /// Runs k-means once from the two `means`, as `iterKmeans(1, 2, cap,
    /// max, power, means)`, leaving the means where it puts them.
    pub fn run(&mut self, cap: usize, max_rounds: u32, power: u32, means: &mut [SinkPoint; 2]) {
        self.segment_length = dist(means[0], means[1]);
        let mut tmp = *means;
        let silhouette = self.kmeans(cap, max_rounds, power, &mut tmp);
        let solution: Vec<Option<usize>> = if silhouette > -1.0 {
            *means = tmp;
            self.sinks.iter().map(|s| s.cluster).collect()
        } else {
            vec![Some(0); self.sinks.len()]
        };
        for (sink, cluster) in self.sinks.iter_mut().zip(solution) {
            sink.cluster = cluster;
        }
        self.fix_segment_lengths(means);
    }

    /// The sinks of each cluster, by index, from the last round.
    pub fn clusters(&self) -> &[Vec<usize>] {
        &self.clusters
    }

    fn fix_segment_lengths(&self, means: &mut [SinkPoint; 2]) {
        let half = self.segment_length / 2.0;
        fix_segment(self.branching_point, half, &mut means[0]);
        fix_segment(self.branching_point, half, &mut means[1]);
    }

    fn kmeans(
        &mut self,
        cap: usize,
        max_rounds: u32,
        power: u32,
        means: &mut [SinkPoint; 2],
    ) -> f32 {
        for sink in &mut self.sinks {
            sink.cluster = None;
        }
        let mut round = 1;
        loop {
            self.fix_segment_lengths(means);
            self.min_cost_flow(means, cap, power);
            let mut clusters: Vec<Vec<usize>> = vec![Vec::new(); 2];
            for (i, sink) in self.sinks.iter().enumerate() {
                let position = match sink.cluster {
                    Some(c) if c < 2 => c,
                    _ => nearest_open_cluster(means, &clusters, cap, sink.at),
                };
                clusters[position].push(i);
            }
            let mut delta = 0.0_f32;
            for (mean, members) in means.iter_mut().zip(&clusters) {
                if members.is_empty() {
                    continue;
                }
                let (mut sum_x, mut sum_y) = (0.0_f32, 0.0_f32);
                for &m in members {
                    sum_x += self.sinks[m].at.x;
                    sum_y += self.sinks[m].at.y;
                }
                let previous = *mean;
                *mean = SinkPoint::new(sum_x / members.len() as f32, sum_y / members.len() as f32);
                delta += (previous.x - mean.x).abs() + (previous.y - mean.y).abs();
            }
            self.clusters = clusters;
            if round > max_rounds || delta < 0.5 {
                break;
            }
            round += 1;
        }
        self.silhouette(means)
    }

    fn silhouette(&self, means: &[SinkPoint; 2]) -> f32 {
        let mut sum = 0.0_f32;
        for sink in &self.sinks {
            // NOTE: calcSilh quirk kept: an unassigned sink scores in_d = 0, a
            // perfect fit, as the clusters TritonCTS picks depend on it.
            let (mut in_d, mut out_d) = (0.0_f32, f32::MAX);
            for (j, &mean) in means.iter().enumerate() {
                let d = dist(mean, sink.at);
                if sink.cluster == Some(j) {
                    in_d = d;
                } else {
                    out_d = out_d.min(d);
                }
            }
            let larger = out_d.max(in_d);
            if larger == 0.0 {
                if out_d == 0.0 {
                    sum -= 1.0;
                }
                if in_d == 0.0 {
                    sum += 1.0;
                }
            } else {
                sum += (out_d - in_d) / larger;
            }
        }
        sum / self.sinks.len() as f32
    }

    /// Assigns the sinks to clusters by the flow LEMON's network simplex
    /// leaves when asked to send `2 * cap` plus the odd sink from the
    /// sinks to the clusters, each sink an arc to every mean within reach
    /// costing its distance to the `power`, as `Clustering::minCostFlow`; a
    /// sink without flow keeps its assignment.
    fn min_cost_flow(&mut self, means: &[SinkPoint; 2], cap: usize, power: u32) {
        let n = self.sinks.len();
        let (source, target) = (0, 1);
        let sink_node = |i: usize| 2 + i;
        let cluster_node = |j: usize| 2 + n + j;
        let mut arcs: Vec<Arc> = (0..n)
            .map(|i| Arc {
                source,
                target: sink_node(i),
                capacity: 1,
                cost: 0,
            })
            .collect();
        let mut sink_arcs = Vec::new();
        for (i, sink) in self.sinks.iter().enumerate() {
            for (j, &mean) in means.iter().enumerate() {
                let d = f64::from(dist(mean, sink.at));
                if d > f64::from(MAX_DIST) {
                    continue;
                }
                let d = d.powf(f64::from(power));
                if d < f64::from(i32::MAX) {
                    sink_arcs.push((arcs.len(), i, j));
                    arcs.push(Arc {
                        source: sink_node(i),
                        target: cluster_node(j),
                        capacity: 1,
                        cost: i32::saturating_from(d),
                    });
                }
            }
        }
        let remaining = n % means.len();
        for j in 0..means.len() {
            arcs.push(Arc {
                source: cluster_node(j),
                target,
                capacity: i32::saturating_from(cap + usize::from(j < remaining)),
                cost: 0,
            });
        }
        let supply = i32::saturating_from(means.len() * cap + remaining);
        // NOTE: Clustering::minCostFlow ignores run()'s outcome
        // (Clustering.cpp:370) and reads the flows as the simplex left them.
        let flows = min_cost_flow(2 + n + means.len(), &arcs, source, target, supply)
            .unwrap_or_else(|failure| failure.flows);
        for (a, i, j) in sink_arcs {
            if flows[a] != 0 {
                self.sinks[i].cluster = Some(j);
            }
        }
    }
}

/// The nearest cluster with room, or the first when none has room, as the
/// fallback of `Clustering::Kmeans`.
fn nearest_open_cluster(
    means: &[SinkPoint; 2],
    clusters: &[Vec<usize>],
    cap: usize,
    point: SinkPoint,
) -> usize {
    let mut best: Option<(usize, f32)> = None;
    for (j, &mean) in means.iter().enumerate() {
        if clusters[j].len() >= cap {
            continue;
        }
        let d = dist(mean, point);
        if best.is_none_or(|(_, b)| d < b) {
            best = Some((j, d));
        }
    }
    best.map_or(0, |(j, _)| j)
}

/// Moves `movable` along the line from `fixed` to lie `target` from it,
/// as `Clustering::fixSegment`.
fn fix_segment(fixed: SinkPoint, target: f32, movable: &mut SinkPoint) {
    let actual = dist(fixed, *movable);
    if actual == 0.0 {
        return;
    }
    let ratio = target / actual;
    let dx = (movable.x - fixed.x) * ratio;
    let dy = (movable.y - fixed.y) * ratio;
    *movable = SinkPoint::new(fixed.x + dx, fixed.y + dy);
}

#[cfg(test)]
mod tests;
