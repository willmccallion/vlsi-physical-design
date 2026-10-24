//! Sets of track-graph edges held as runs of wire.
//!
//! A route's wire edges come in long stretches of consecutive crossings
//! along one track, so a set of edges keeps each stretch as one [`Run`]
//! rather than one edge per crossing, and its vias apart. It reads back as
//! the same edges in the order a sorted list of them takes, so code that
//! walks a route's edges sees what it always saw.

use crate::access::Node;
use crate::wiring::Edge;
use pare_common::db::indices::LayerId;

/// A straight stretch of wire along one track: the wire edges from
/// crossing `from` up to crossing `to`, which joins the stretch's last
/// edge to the crossing after it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Run {
    /// Routing layer.
    pub layer: LayerId,
    /// Track index on that layer.
    pub track: u32,
    /// First crossing.
    pub from: u32,
    /// Last crossing, after `from`.
    pub to: u32,
}

impl Run {
    /// The crossing `cross` of its track.
    pub const fn node(self, cross: u32) -> Node {
        Node {
            layer: self.layer,
            track: self.track,
            cross,
        }
    }

    /// Its wire edges, first crossing first.
    pub fn edges(self) -> impl Iterator<Item = Edge> {
        (self.from..self.to).map(move |cross| Edge::Wire(self.node(cross)))
    }

    /// The single wire edge from crossing `cross` to the next.
    const fn step(n: Node) -> Self {
        Self {
            layer: n.layer,
            track: n.track,
            from: n.cross,
            to: n.cross + 1,
        }
    }

    const fn same_track(self, other: Self) -> bool {
        self.layer.0 == other.layer.0 && self.track == other.track
    }
}

/// A set of edges of the track graph: its wire edges merged into runs along
/// each track, sorted, apart and not touching, and its vias sorted without
/// repeats.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EdgeSet {
    runs: Vec<Run>,
    vias: Vec<(Node, Node)>,
}

impl EdgeSet {
    /// The set of the wire edges of `runs`.
    pub fn from_runs(runs: impl IntoIterator<Item = Run>) -> Self {
        let mut set = Self {
            runs: runs.into_iter().filter(|r| r.to > r.from).collect(),
            vias: Vec::new(),
        };
        set.runs = merged(std::mem::take(&mut set.runs));
        set
    }

    /// Adds `edges` to the set.
    pub fn extend(&mut self, edges: impl IntoIterator<Item = Edge>) {
        let mut added_runs = false;
        let mut added_vias = false;
        for e in edges {
            match e {
                Edge::Wire(n) => {
                    self.runs.push(Run::step(n));
                    added_runs = true;
                }
                Edge::Via { lower, upper } => {
                    self.vias.push((lower, upper));
                    added_vias = true;
                }
            }
        }
        if added_runs {
            self.runs = merged(std::mem::take(&mut self.runs));
        }
        if added_vias {
            self.vias.sort_unstable();
            self.vias.dedup();
        }
    }

    /// Adds every edge of `other` to the set.
    pub fn union(&mut self, other: &Self) {
        if !other.runs.is_empty() {
            self.runs.extend_from_slice(&other.runs);
            self.runs = merged(std::mem::take(&mut self.runs));
        }
        if !other.vias.is_empty() {
            self.vias.extend_from_slice(&other.vias);
            self.vias.sort_unstable();
            self.vias.dedup();
        }
    }

    /// Removes every edge of `other` from the set.
    pub fn subtract(&mut self, other: &Self) {
        if !other.runs.is_empty() {
            self.runs = runs_without(&self.runs, &other.runs);
        }
        if !other.vias.is_empty() {
            self.vias.retain(|v| other.vias.binary_search(v).is_err());
        }
    }

    /// Keeps only the edges `keep` accepts.
    pub fn retain(&mut self, mut keep: impl FnMut(Edge) -> bool) {
        let mut runs = Vec::with_capacity(self.runs.len());
        for &run in &self.runs {
            let mut start = None;
            for cross in run.from..run.to {
                match (keep(Edge::Wire(run.node(cross))), start) {
                    (true, None) => start = Some(cross),
                    (false, Some(from)) => {
                        runs.push(Run {
                            from,
                            to: cross,
                            ..run
                        });
                        start = None;
                    }
                    _ => {}
                }
            }
            if let Some(from) = start {
                runs.push(Run { from, ..run });
            }
        }
        self.runs = runs;
        self.vias
            .retain(|&(lower, upper)| keep(Edge::Via { lower, upper }));
    }

    /// The set of the part of each run `clip` keeps, which must lie within
    /// the run, and the vias `keep_via` accepts.
    #[must_use]
    pub fn clipped(
        &self,
        clip: impl FnMut(Run) -> Option<Run>,
        keep_via: impl FnMut(&(Node, Node)) -> bool,
    ) -> Self {
        Self {
            runs: self.runs.iter().copied().filter_map(clip).collect(),
            vias: self.vias.iter().copied().filter(keep_via).collect(),
        }
    }

    /// Its runs of wire, sorted by layer, track and first crossing.
    pub fn runs(&self) -> &[Run] {
        &self.runs
    }

    /// Its vias, each as its lower and upper crossing, sorted.
    pub fn vias(&self) -> &[(Node, Node)] {
        &self.vias
    }

    /// Its edges, in the order a sorted list of them takes: the wire edges
    /// by layer, track and crossing, then the vias.
    pub fn iter(&self) -> impl Iterator<Item = Edge> + '_ {
        self.runs.iter().flat_map(|r| r.edges()).chain(
            self.vias
                .iter()
                .map(|&(lower, upper)| Edge::Via { lower, upper }),
        )
    }

    /// How many edges it holds.
    pub fn len(&self) -> usize {
        self.runs
            .iter()
            .map(|r| (r.to - r.from) as usize)
            .sum::<usize>()
            + self.vias.len()
    }

    /// Whether it holds no edge.
    pub const fn is_empty(&self) -> bool {
        self.runs.is_empty() && self.vias.is_empty()
    }
}

impl FromIterator<Edge> for EdgeSet {
    fn from_iter<I: IntoIterator<Item = Edge>>(edges: I) -> Self {
        let mut set = Self::default();
        set.extend(edges);
        set
    }
}

/// `runs` sorted, with every two on one track that overlap or touch made
/// one.
fn merged(mut runs: Vec<Run>) -> Vec<Run> {
    runs.sort_unstable();
    let mut out: Vec<Run> = Vec::with_capacity(runs.len());
    for run in runs {
        match out.last_mut() {
            Some(last) if last.same_track(run) && run.from <= last.to => {
                last.to = last.to.max(run.to);
            }
            _ => out.push(run),
        }
    }
    out
}

/// The parts of the sorted, merged `runs` that the sorted, merged `cut`
/// leaves.
fn runs_without(runs: &[Run], cut: &[Run]) -> Vec<Run> {
    let mut out = Vec::with_capacity(runs.len());
    let mut k = 0;
    for &run in runs {
        while k < cut.len()
            && (cut[k].layer, cut[k].track, cut[k].to) <= (run.layer, run.track, run.from)
        {
            k += 1;
        }
        let mut from = run.from;
        let mut j = k;
        while j < cut.len() && cut[j].same_track(run) && cut[j].from < run.to {
            if cut[j].from > from {
                out.push(Run {
                    from,
                    to: cut[j].from,
                    ..run
                });
            }
            from = from.max(cut[j].to);
            j += 1;
        }
        if from < run.to {
            out.push(Run { from, ..run });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(layer: u8, track: u32, cross: u32) -> Node {
        Node {
            layer: LayerId(layer),
            track,
            cross,
        }
    }

    fn wire(layer: u8, track: u32, cross: u32) -> Edge {
        Edge::Wire(node(layer, track, cross))
    }

    fn via(track: u32, cross: u32) -> Edge {
        Edge::Via {
            lower: node(1, track, cross),
            upper: node(2, cross, track),
        }
    }

    /// The edges in a sorted list without repeats, as routes held them.
    fn sorted(edges: &[Edge]) -> Vec<Edge> {
        let mut list = edges.to_vec();
        list.sort_unstable();
        list.dedup();
        list
    }

    fn mixed() -> Vec<Edge> {
        vec![
            wire(1, 4, 7),
            via(4, 9),
            wire(1, 4, 5),
            wire(1, 4, 6),
            wire(2, 3, 1),
            wire(1, 4, 9),
            via(2, 2),
            wire(1, 4, 6),
            wire(1, 5, 0),
        ]
    }

    #[test]
    fn a_set_reads_back_its_edges_as_a_sorted_list_without_repeats() {
        let set: EdgeSet = mixed().into_iter().collect();

        assert_eq!(set.iter().collect::<Vec<_>>(), sorted(&mixed()));
        assert_eq!(set.len(), sorted(&mixed()).len());
    }

    #[test]
    fn consecutive_wire_edges_on_a_track_become_one_run() {
        let set: EdgeSet = mixed().into_iter().collect();

        assert_eq!(
            set.runs(),
            [
                Run {
                    layer: LayerId(1),
                    track: 4,
                    from: 5,
                    to: 8
                },
                Run {
                    layer: LayerId(1),
                    track: 4,
                    from: 9,
                    to: 10
                },
                Run {
                    layer: LayerId(1),
                    track: 5,
                    from: 0,
                    to: 1
                },
                Run {
                    layer: LayerId(2),
                    track: 3,
                    from: 1,
                    to: 2
                },
            ]
        );
    }

    #[test]
    fn subtracting_leaves_what_removing_each_edge_leaves() {
        let mut set: EdgeSet = mixed().into_iter().collect();
        let gone = [wire(1, 4, 6), via(4, 9), wire(1, 5, 0), wire(3, 1, 1)];

        set.subtract(&gone.into_iter().collect());

        let expected: Vec<Edge> = sorted(&mixed())
            .into_iter()
            .filter(|e| !gone.contains(e))
            .collect();
        assert_eq!(set.iter().collect::<Vec<_>>(), expected);
    }

    #[test]
    fn subtracting_a_stretch_longer_than_a_run_removes_all_of_it() {
        let mut set = EdgeSet::from_runs([
            Run {
                layer: LayerId(1),
                track: 4,
                from: 5,
                to: 8,
            },
            Run {
                layer: LayerId(1),
                track: 4,
                from: 12,
                to: 20,
            },
        ]);

        set.subtract(&EdgeSet::from_runs([Run {
            layer: LayerId(1),
            track: 4,
            from: 0,
            to: 14,
        }]));

        assert_eq!(
            set.runs(),
            [Run {
                layer: LayerId(1),
                track: 4,
                from: 14,
                to: 20
            }]
        );
    }

    #[test]
    fn retaining_splits_a_run_where_an_edge_is_dropped() {
        let mut set = EdgeSet::from_runs([Run {
            layer: LayerId(1),
            track: 4,
            from: 0,
            to: 6,
        }]);

        set.retain(|e| e != wire(1, 4, 2) && e != wire(1, 4, 3));

        assert_eq!(
            set.runs(),
            [
                Run {
                    layer: LayerId(1),
                    track: 4,
                    from: 0,
                    to: 2
                },
                Run {
                    layer: LayerId(1),
                    track: 4,
                    from: 4,
                    to: 6
                },
            ]
        );
    }

    #[test]
    fn a_union_merges_runs_that_meet() {
        let mut set = EdgeSet::from_runs([Run {
            layer: LayerId(1),
            track: 4,
            from: 0,
            to: 3,
        }]);

        set.union(&EdgeSet::from_runs([Run {
            layer: LayerId(1),
            track: 4,
            from: 3,
            to: 5,
        }]));

        assert_eq!(
            set.runs(),
            [Run {
                layer: LayerId(1),
                track: 4,
                from: 0,
                to: 5
            }]
        );
    }

    #[test]
    fn sets_of_the_same_edges_are_equal_however_they_were_built() {
        let built: EdgeSet = mixed().into_iter().collect();
        let mut grown = EdgeSet::default();
        for e in mixed().into_iter().rev() {
            grown.extend([e]);
        }

        assert_eq!(built, grown);
    }
}
