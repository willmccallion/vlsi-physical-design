//! Set matching, as `DetailedMis`: cells of one size and colour, no two
//! sharing a net, are gathered from nearby bins and reassigned to each
//! other's spots by a minimum-cost matching on wirelength.

use super::{ImprovementError, Improver, Move, Result};
use super::{NodeId, SegmentId};
use pare_common::db::indices::IdVec;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::network_simplex::{Arc, min_cost_flow};
use std::collections::VecDeque;

/// Cells a matching gathers before it stops widening its search.
const MAX_PROBLEM_SIZE: usize = 25;
/// Times a cell may take part in matchings in one pass.
const MAX_TIMES_USED: usize = 2;
/// Nets with more pins than this neither colour cells nor count in costs.
const SKIP_EDGES_LARGER_THAN: usize = 100;

/// The bins cells are gathered from, as `DetailedMis`'s grid of buckets.
struct Bins {
    step: (f64, f64),
    dim: (usize, usize),
    /// The cells in each bin.
    nodes: Vec<Vec<NodeId>>,
    traversal: Vec<usize>,
    bin_of: IdVec<NodeId, Option<usize>>,
}

/// `DetailedMis::run` minimizing wirelength.
pub(super) fn run(imp: &mut Improver, passes: usize, tolerance: f64) -> Result<()> {
    let tolerance = tolerance.max(0.01);
    imp.resort_segments();
    let mut hpwl = imp.total_hpwl();
    if hpwl == Dbu::ZERO {
        return Ok(());
    }
    let mut candidates = imp.movable_cells();
    if candidates.is_empty() {
        return Ok(());
    }
    let colors = color_cells(imp, &candidates);
    let mut bins = Bins::new(imp, &candidates);
    let mut traversal = 0;
    for _ in 0..passes.max(1) {
        place(imp, &mut candidates, &colors, &mut bins, &mut traversal)?;
        let last = hpwl;
        hpwl = imp.total_hpwl();
        if converged(last, hpwl, tolerance) {
            break;
        }
    }
    imp.resort_segments();
    Ok(())
}

/// Whether a pass changed the wirelength by no more than `tolerance` of
/// it, as each pass's stopping test.
pub(super) fn converged(last: Dbu, current: Dbu, tolerance: f64) -> bool {
    last == Dbu::ZERO || (current - last).abs().to_f64() / last.to_f64() <= tolerance
}

/// Colours the movable cells greedily so that no two on a net share a
/// colour, as `DetailedMis::colorCells` with `ColorGraph::greedyColoring`.
fn color_cells(imp: &Improver, candidates: &[NodeId]) -> IdVec<NodeId, Option<usize>> {
    let nodes = imp.net.node_count();
    let mut movable = IdVec::filled(false, nodes);
    for &n in candidates {
        movable[n] = true;
    }
    let mut adjacent: IdVec<NodeId, Vec<NodeId>> = IdVec::filled(Vec::new(), nodes);
    for edge in &imp.net.edges {
        if edge.len() <= 1 || edge.len() > SKIP_EDGES_LARGER_THAN {
            continue;
        }
        for (k, &pi) in edge.iter().enumerate() {
            let ni = imp.net.pins[pi].node;
            if !movable[ni] {
                continue;
            }
            for &pj in &edge[k + 1..] {
                let nj = imp.net.pins[pj].node;
                if movable[nj] && nj != ni {
                    adjacent[ni].push(nj);
                    adjacent[nj].push(ni);
                }
            }
        }
    }
    if nodes == 0 {
        return IdVec::new();
    }
    let mut color: IdVec<NodeId, Option<usize>> = IdVec::filled(None, nodes);
    color[NodeId(0)] = Some(0);
    let mut taken_by: Vec<Option<NodeId>> = vec![None; nodes];
    for v in imp.net.nodes().skip(1) {
        adjacent[v].sort_unstable();
        adjacent[v].dedup();
        for &u in &adjacent[v] {
            if let Some(c) = color[u] {
                taken_by[c] = Some(v);
            }
        }
        color[v] = Some((0..nodes).find(|&c| taken_by[c] != Some(v)).unwrap_or(0));
    }
    imp.net
        .nodes()
        .map(|n| color[n].filter(|_| movable[n]))
        .collect()
}

impl Bins {
    /// Bins about five cells wide and high, as `DetailedMis::buildGrid`.
    fn new(imp: &Improver, candidates: &[NodeId]) -> Self {
        let count = candidates.len() as f64;
        let avg_h = candidates
            .iter()
            .map(|&n| imp.height(n).to_f64())
            .sum::<f64>()
            / count;
        let avg_w = candidates
            .iter()
            .map(|&n| imp.width(n).to_f64())
            .sum::<f64>()
            / count;
        let side = (MAX_PROBLEM_SIZE as f64).sqrt();
        let step = (avg_w * side, avg_h * side);
        let dim = (
            ((imp.arch.max_x - imp.arch.min_x).to_f64() / step.0).ceil() as usize,
            ((imp.arch.max_y - imp.arch.min_y).to_f64() / step.1).ceil() as usize,
        );
        Self {
            step,
            dim,
            nodes: vec![Vec::new(); dim.0 * dim.1],
            traversal: vec![0; dim.0 * dim.1],
            bin_of: IdVec::filled(None, imp.net.node_count()),
        }
    }

    const fn index(&self, i: usize, j: usize) -> usize {
        i * self.dim.1 + j
    }

    /// Bins the cells by center, as `DetailedMis::populateGrid`.
    fn populate(&mut self, imp: &Improver, candidates: &[NodeId]) {
        for bin in &mut self.nodes {
            bin.clear();
        }
        self.bin_of.fill(None);
        let last = |dim: usize| dim as i32 - 1;
        for &n in candidates {
            let y = imp.bottom(n).to_f64() + 0.5 * imp.height(n).to_f64();
            let x = imp.left(n).to_f64() + 0.5 * imp.width(n).to_f64();
            let j = (((y - imp.arch.min_y.to_f64()) / self.step.1) as i32)
                .min(last(self.dim.1))
                .max(0);
            let i = (((x - imp.arch.min_x.to_f64()) / self.step.0) as i32)
                .min(last(self.dim.0))
                .max(0);
            let bin = self.index(i as usize, j as usize);
            self.nodes[bin].push(n);
            self.bin_of[n] = Some(bin);
        }
    }
}

/// One pass of matchings seeded from every candidate in random order, as
/// `DetailedMis::place`.
fn place(
    imp: &mut Improver,
    candidates: &mut [NodeId],
    colors: &IdVec<NodeId, Option<usize>>,
    bins: &mut Bins,
    traversal: &mut usize,
) -> Result<()> {
    bins.populate(imp, candidates);
    let mut times_used = IdVec::filled(0, imp.net.node_count());
    imp.shuffle(candidates);
    for &n in candidates.iter() {
        if times_used[n] >= MAX_TIMES_USED {
            continue;
        }
        let Some(neighbours) = gather_neighbours(imp, n, colors, bins, traversal) else {
            continue;
        };
        solve_match(imp, &neighbours)?;
        for &m in &neighbours {
            times_used[m] += 1;
        }
    }
    Ok(())
}

/// The cell and others of its size, colour and region from its bin and
/// then ever farther bins, as `DetailedMis::gatherNeighbours`.
fn gather_neighbours(
    imp: &Improver,
    n: NodeId,
    colors: &IdVec<NodeId, Option<usize>>,
    bins: &mut Bins,
    traversal: &mut usize,
) -> Option<Vec<NodeId>> {
    let start = bins.bin_of[n]?;
    let mut neighbours = vec![n];
    let spanned = imp.rows_spanned(n);
    let mut queue = VecDeque::from([start]);
    *traversal += 1;
    while let Some(bin) = queue.pop_front() {
        if bins.traversal[bin] == *traversal {
            continue;
        }
        bins.traversal[bin] = *traversal;
        for &m in &bins.nodes[bin] {
            let alike = m != n
                && colors[n] == colors[m]
                && imp.width(n) == imp.width(m)
                && imp.height(n) == imp.height(m)
                && imp.group(n) == imp.group(m)
                && imp.rows_spanned(m) == spanned;
            if alike {
                neighbours.push(m);
            }
        }
        if neighbours.len() >= MAX_PROBLEM_SIZE {
            break;
        }
        let (i, j) = (bin / bins.dim.1, bin % bins.dim.1);
        if i > 0 {
            queue.push_back(bins.index(i - 1, j));
        }
        if i + 1 < bins.dim.0 {
            queue.push_back(bins.index(i + 1, j));
        }
        if j > 0 {
            queue.push_back(bins.index(i, j - 1));
        }
        if j + 1 < bins.dim.1 {
            queue.push_back(bins.index(i, j + 1));
        }
    }
    Some(neighbours)
}

/// Reassigns the cells to each other's spots by a minimum-cost matching,
/// undoing it if it breaks a placement rule, as `DetailedMis::solveMatch`.
fn solve_match(imp: &mut Improver, nodes: &[NodeId]) -> Result<()> {
    if nodes.len() <= 1 {
        return Ok(());
    }
    let count = nodes.len();
    let pos: Vec<Point<Dbu>> = nodes.iter().map(|&n| imp.position(n)).collect();
    let segs: Vec<Vec<SegmentId>> = nodes.iter().map(|&n| imp.cell_segs[n].clone()).collect();
    let cell_node = |i: usize| 2 * i;
    let spot_node = |i: usize| 2 * i + 1;
    let (supply, demand) = (2 * count, 2 * count + 1);
    let mut arcs = Vec::new();
    let mut assignment = Vec::new();
    for (i, &n) in nodes.iter().enumerate() {
        arcs.push(Arc {
            source: supply,
            target: cell_node(i),
            capacity: 1,
            cost: 0,
        });
        arcs.push(Arc {
            source: spot_node(i),
            target: demand,
            capacity: 1,
            cost: 0,
        });
        let orig = imp.orig(n);
        for (j, &at) in pos.iter().enumerate() {
            if i != j
                && ((at.x - orig.x).abs() > imp.max_displacement.x
                    || (at.y - orig.y).abs() > imp.max_displacement.y)
            {
                continue;
            }
            let center = Point::new(at.x + imp.width(n) / 2, at.y + imp.height(n) / 2);
            let cost = hpwl_at(imp, n, center).to_f64();
            assignment.push((arcs.len(), i, j));
            arcs.push(Arc {
                source: cell_node(i),
                target: spot_node(j),
                capacity: 1,
                cost: cost.min(f64::from(i32::MAX)) as i32,
            });
        }
    }
    let count_i32 = count as i32;
    let flow = min_cost_flow(2 * count + 2, &arcs, supply, demand, count_i32);
    let mut journal = Vec::new();
    for &(arc, i, j) in assignment.iter().rev() {
        if flow[arc] == 0 || i == j {
            continue;
        }
        let (n, m) = (nodes[i], nodes[j]);
        if imp.rows_spanned(n) != imp.rows_spanned(m)
            || imp.width(n) != imp.width(m)
            || imp.height(n) != imp.height(m)
            || segs[i].len() != imp.rows_spanned(n)
            || segs[j].len() != imp.rows_spanned(n)
        {
            return Err(ImprovementError::Internal(
                "Unable to interpret flow during matching",
            ));
        }
        for &s in &segs[i] {
            imp.remove_cell_from_segment(n, s)?;
        }
        imp.erase(n);
        imp.set_position(n, pos[j]);
        imp.paint(n);
        for &s in &segs[j] {
            imp.add_cell_to_segment(n, s)?;
        }
        journal.push(Move {
            node: n,
            from: pos[i],
            from_segs: segs[i].clone(),
            to_segs: segs[j].clone(),
        });
    }
    if nodes.iter().any(|&n| imp.has_placement_violation(n)) {
        imp.undo(&journal)?;
    }
    Ok(())
}

/// The wirelength of a cell's nets with it centered at `center`, as
/// `DetailedMis::getHpwl`; a net the cell has several pins on counts as
/// often.
fn hpwl_at(imp: &Improver, n: NodeId, center: Point<Dbu>) -> Dbu {
    let mut hpwl = Dbu::ZERO;
    for &pin in &imp.net.node_pins[n] {
        let edge = &imp.net.edges[imp.net.pins[pin].edge];
        if edge.len() <= 1 || edge.len() > SKIP_EDGES_LARGER_THAN {
            continue;
        }
        let (mut x0, mut y0) = (Dbu(i64::MAX), Dbu(i64::MAX));
        let (mut x1, mut y1) = (Dbu(i64::MIN), Dbu(i64::MIN));
        for &p in edge {
            let other = &imp.net.pins[p];
            let at = if other.node == n {
                center + other.offset
            } else {
                imp.pin_position(p)
            };
            x0 = x0.min(at.x);
            y0 = y0.min(at.y);
            x1 = x1.max(at.x);
            y1 = y1.max(at.y);
        }
        hpwl += (x1 - x0) + (y1 - y0);
    }
    hpwl
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 0.005;

    #[test]
    fn a_pass_converges_when_it_changes_the_wirelength_within_the_tolerance() {
        assert!(converged(Dbu(1000), Dbu(995), TOLERANCE));
    }

    #[test]
    fn a_pass_saving_more_than_the_tolerance_has_not_converged() {
        assert!(!converged(Dbu(1000), Dbu(990), TOLERANCE));
    }

    #[test]
    fn a_pass_lengthening_the_nets_past_the_tolerance_has_not_converged() {
        assert!(!converged(Dbu(1000), Dbu(1010), TOLERANCE));
    }

    #[test]
    fn no_wirelength_to_improve_has_converged() {
        assert!(converged(Dbu(0), Dbu(50), TOLERANCE));
    }
}
