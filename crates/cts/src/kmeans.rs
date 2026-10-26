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

use crate::simplex::{Arc, min_cost_flow};

const MAX_DIST: f32 = 5200.0;

/// A sink to cluster, in the tree's length units.
#[derive(Clone, Copy, Debug)]
struct Sink {
    x: f32,
    y: f32,
    cluster: Option<usize>,
}

/// The clustering of one branch's sinks.
#[derive(Clone, Debug)]
pub(crate) struct Clustering {
    sinks: Vec<Sink>,
    clusters: Vec<Vec<usize>>,
    segment_length: f32,
    branching_point: (f32, f32),
}

const fn dist(a: (f32, f32), b: (f32, f32)) -> f32 {
    (a.0 - b.0).abs() + (a.1 - b.1).abs()
}

impl Clustering {
    pub fn new(sinks: &[(f32, f32)], branching_point: (f32, f32)) -> Self {
        Self {
            sinks: sinks
                .iter()
                .map(|&(x, y)| Sink {
                    x,
                    y,
                    cluster: None,
                })
                .collect(),
            clusters: Vec::new(),
            segment_length: 0.0,
            branching_point,
        }
    }

    /// Runs k-means once from the two `means`, as `iterKmeans(1, 2, cap,
    /// max, power, means)`, leaving the means where it puts them.
    pub fn run(&mut self, cap: usize, max_rounds: u32, power: u32, means: &mut [(f32, f32); 2]) {
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

    fn fix_segment_lengths(&self, means: &mut [(f32, f32); 2]) {
        let half = self.segment_length / 2.0;
        fix_segment(self.branching_point, half, &mut means[0]);
        fix_segment(self.branching_point, half, &mut means[1]);
    }

    fn kmeans(
        &mut self,
        cap: usize,
        max_rounds: u32,
        power: u32,
        means: &mut [(f32, f32); 2],
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
                    _ => nearest_open_cluster(means, &clusters, cap, (sink.x, sink.y)),
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
                    sum_x += self.sinks[m].x;
                    sum_y += self.sinks[m].y;
                }
                let previous = *mean;
                *mean = (sum_x / members.len() as f32, sum_y / members.len() as f32);
                delta += (previous.0 - mean.0).abs() + (previous.1 - mean.1).abs();
            }
            self.clusters = clusters;
            if round > max_rounds || delta < 0.5 {
                break;
            }
            round += 1;
        }
        self.silhouette(means)
    }

    fn silhouette(&self, means: &[(f32, f32); 2]) -> f32 {
        let mut sum = 0.0_f32;
        for sink in &self.sinks {
            let (mut in_d, mut out_d) = (0.0_f32, f32::MAX);
            for (j, &mean) in means.iter().enumerate() {
                let d = dist(mean, (sink.x, sink.y));
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
    fn min_cost_flow(&mut self, means: &[(f32, f32); 2], cap: usize, power: u32) {
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
                let d = f64::from(dist(mean, (sink.x, sink.y)));
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
                        cost: d as i32,
                    });
                }
            }
        }
        let remaining = n % means.len();
        for j in 0..means.len() {
            arcs.push(Arc {
                source: cluster_node(j),
                target,
                capacity: (cap + usize::from(j < remaining)) as i32,
                cost: 0,
            });
        }
        let supply = (means.len() * cap + remaining) as i32;
        let flows = min_cost_flow(2 + n + means.len(), &arcs, source, target, supply);
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
    means: &[(f32, f32); 2],
    clusters: &[Vec<usize>],
    cap: usize,
    point: (f32, f32),
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
fn fix_segment(fixed: (f32, f32), target: f32, movable: &mut (f32, f32)) {
    let actual = dist(fixed, *movable);
    if actual == 0.0 {
        return;
    }
    let ratio = target / actual;
    let dx = (movable.0 - fixed.0) * ratio;
    let dy = (movable.1 - fixed.1) * ratio;
    *movable = (fixed.0 + dx, fixed.1 + dy);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sinks_split_toward_the_nearer_mean_within_capacity() {
        let sinks = [(0.0, 0.0), (1.0, 0.0), (9.0, 0.0), (10.0, 0.0)];
        let mut clustering = Clustering::new(&sinks, (5.0, 0.0));
        let mut means = [(0.0, 0.0), (10.0, 0.0)];

        clustering.run(2, 5, 4, &mut means);

        assert_eq!(clustering.clusters(), &[vec![0, 1], vec![2, 3]]);
    }

    #[test]
    fn capacity_forces_the_cheapest_sinks_to_move() {
        let sinks = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (10.0, 0.0)];
        let mut clustering = Clustering::new(&sinks, (5.0, 0.0));
        let mut means = [(0.0, 0.0), (10.0, 0.0)];

        clustering.run(2, 0, 4, &mut means);

        assert_eq!(clustering.clusters()[0].len(), 2);
        assert_eq!(clustering.clusters()[1].len(), 2);
    }
}
