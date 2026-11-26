//! Sets of track-graph edges held as runs of wire.
//!
//! A route's wire edges come in long stretches of consecutive crossings
//! along one track, so a set of edges keeps each stretch as one [`Run`]
//! rather than one edge per crossing, and its vias and jogs apart. It reads back as
//! the same edges in the order a sorted list of them takes, so code that
//! walks a route's edges sees what it always saw.

use crate::access::Node;
use crate::tech::{CrossIdx, IndexRange, TrackIdx};
use crate::wiring::Edge;
use pare_common::db::indices::LayerId;

/// A straight stretch of wire along one track: the wire edges from
/// crossing `from` up to crossing `to`, which joins the stretch's last
/// edge to the crossing after it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Run {
    /// Routing layer.
    pub layer: LayerId,
    /// The track on that layer.
    pub track: TrackIdx,
    /// First crossing.
    pub from: CrossIdx,
    /// Last crossing, after `from`.
    pub to: CrossIdx,
}

impl Run {
    /// The crossing `cross` of its track.
    pub const fn node(self, cross: CrossIdx) -> Node {
        Node {
            layer: self.layer,
            track: self.track,
            cross,
        }
    }

    /// Its wire edges, first crossing first.
    pub fn edges(self) -> impl Iterator<Item = Edge> {
        self.crosses()
            .map(move |cross| Edge::Wire(self.node(cross)))
    }

    /// Its crossings but the last: the starts of its wire edges.
    const fn crosses(self) -> IndexRange<CrossIdx> {
        IndexRange {
            start: self.from,
            end: self.to,
        }
    }

    /// The single wire edge from crossing `cross` to the next.
    const fn step(n: Node) -> Self {
        Self {
            layer: n.layer,
            track: n.track,
            from: n.cross,
            to: n.cross.next(),
        }
    }

    const fn same_track(self, other: Self) -> bool {
        self.layer.0 == other.layer.0 && self.track.0 == other.track.0
    }
}

/// A set of edges of the track graph: its wire edges merged into runs along
/// each track, sorted, apart and not touching, and its vias and jogs each
/// sorted without repeats.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct EdgeSet {
    runs: Vec<Run>,
    vias: Vec<(Node, Node)>,
    jogs: Vec<Node>,
}

impl EdgeSet {
    /// Bytes its vectors hold on the heap.
    pub(crate) const fn heap_bytes(&self) -> usize {
        self.runs.capacity() * size_of::<Run>()
            + self.vias.capacity() * size_of::<(Node, Node)>()
            + self.jogs.capacity() * size_of::<Node>()
    }

    /// The set of the wire edges of `runs`.
    pub fn from_runs(runs: impl IntoIterator<Item = Run>) -> Self {
        let mut set = Self {
            runs: runs.into_iter().filter(|r| r.to > r.from).collect(),
            vias: Vec::new(),
            jogs: Vec::new(),
        };
        set.runs = merged(std::mem::take(&mut set.runs));
        set
    }

    /// Adds `edges` to the set.
    pub fn extend(&mut self, edges: impl IntoIterator<Item = Edge>) {
        let (runs, vias, jogs) = (self.runs.len(), self.vias.len(), self.jogs.len());
        for e in edges {
            match e {
                Edge::Wire(n) => self.runs.push(Run::step(n)),
                Edge::Via { lower, upper } => self.vias.push((lower, upper)),
                Edge::Jog(n) => self.jogs.push(n),
            }
        }
        if self.runs.len() > runs {
            self.runs = merged(std::mem::take(&mut self.runs));
        }
        if self.vias.len() > vias {
            sort_and_dedup(&mut self.vias);
        }
        if self.jogs.len() > jogs {
            sort_and_dedup(&mut self.jogs);
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
            sort_and_dedup(&mut self.vias);
        }
        if !other.jogs.is_empty() {
            self.jogs.extend_from_slice(&other.jogs);
            sort_and_dedup(&mut self.jogs);
        }
    }

    /// Removes every edge of `other` from the set.
    pub(crate) fn subtract(&mut self, other: &Self) {
        if !other.runs.is_empty() {
            self.runs = runs_without(&self.runs, &other.runs);
        }
        if !other.vias.is_empty() {
            self.vias.retain(|v| other.vias.binary_search(v).is_err());
        }
        if !other.jogs.is_empty() {
            self.jogs.retain(|j| other.jogs.binary_search(j).is_err());
        }
    }

    /// Keeps only the edges `keep` accepts.
    pub fn retain(&mut self, mut keep: impl FnMut(Edge) -> bool) {
        let mut runs = Vec::with_capacity(self.runs.len());
        for &run in &self.runs {
            let mut start = None;
            for cross in run.crosses() {
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
        self.jogs.retain(|&n| keep(Edge::Jog(n)));
    }

    /// The set of the part of each run `clip` keeps, which must lie within
    /// the run, and the vias and jogs `keep_ends` accepts by the crossings
    /// they join.
    #[must_use]
    pub fn clipped(
        &self,
        clip: impl FnMut(Run) -> Option<Run>,
        mut keep_ends: impl FnMut((Node, Node)) -> bool,
    ) -> Self {
        Self {
            runs: self.runs.iter().copied().filter_map(clip).collect(),
            vias: self
                .vias
                .iter()
                .copied()
                .filter(|&v| keep_ends(v))
                .collect(),
            jogs: self
                .jogs
                .iter()
                .filter(|&&n| keep_ends(Edge::Jog(n).ends()))
                .copied()
                .collect(),
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

    /// Its jogs, each as the crossing on the lower of its two tracks,
    /// sorted.
    pub fn jogs(&self) -> &[Node] {
        &self.jogs
    }

    /// Its edges, in the order a sorted list of them takes: the wire edges
    /// by layer, track and crossing, then the vias, then the jogs.
    pub fn iter(&self) -> impl Iterator<Item = Edge> + '_ {
        self.runs
            .iter()
            .flat_map(|r| r.edges())
            .chain(
                self.vias
                    .iter()
                    .map(|&(lower, upper)| Edge::Via { lower, upper }),
            )
            .chain(self.jogs.iter().map(|&n| Edge::Jog(n)))
    }
}

/// Sorts `items` and drops repeats.
fn sort_and_dedup<T: Ord>(items: &mut Vec<T>) {
    items.sort_unstable();
    items.dedup();
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
mod tests;
