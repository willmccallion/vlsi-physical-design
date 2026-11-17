//! Set matching, as `DetailedMis`: cells of one size and colour, no two
//! sharing a net, are gathered from nearby bins and reassigned to each
//! other's spots by a minimum-cost matching on wirelength.

use super::{ImprovementError, Improver, Move, Result, SKIP_NETS_LARGER_THAN};
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
    let colors = color_cells(imp, &candidates)?;
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
fn color_cells(imp: &Improver, candidates: &[NodeId]) -> Result<IdVec<NodeId, Option<usize>>> {
    let nodes = imp.net.node_count();
    let mut movable = IdVec::filled(false, nodes);
    for &n in candidates {
        movable[n] = true;
    }
    let mut adjacent: IdVec<NodeId, Vec<NodeId>> = IdVec::filled(Vec::new(), nodes);
    for edge in &imp.net.edges {
        if edge.len() <= 1 || edge.len() > SKIP_NETS_LARGER_THAN {
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
        return Ok(IdVec::new());
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
        let free = (0..nodes)
            .find(|&c| taken_by[c] != Some(v))
            .ok_or(ImprovementError::Internal("no colour left for a cell"))?;
        color[v] = Some(free);
    }
    Ok(imp
        .net
        .nodes()
        .map(|n| color[n].filter(|_| movable[n]))
        .collect())
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

/// The cell and others of its size, colour, region and rails from its bin
/// and then ever farther bins, as `DetailedMis::gatherNeighbours`.
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
                && imp.net.rails[n] == imp.net.rails[m]
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
    let spots = Spots {
        at: nodes.iter().map(|&n| imp.position(n)).collect(),
        segs: nodes.iter().map(|&n| imp.cell_segs[n].clone()).collect(),
    };
    let matching = Matching::new(imp, nodes, &spots.at);
    let count_i32 = count as i32;
    // NOTE: solveMatch first checks with Preflow that every cell can be
    // matched; each cell's arc to its own spot makes that always so here.
    let flow = min_cost_flow(
        2 * count + 2,
        &matching.arcs,
        supply_node(count),
        demand_node(count),
        count_i32,
    );
    let Ok(flow) = flow else {
        return Ok(());
    };
    let journal = move_to_matched_spots(imp, nodes, &spots, &matching.choices, &flow)?;
    if nodes.iter().any(|&n| imp.has_placement_violation(n)) {
        imp.undo(&journal)?;
    }
    Ok(())
}

/// Where each cell of a matching stands, and its segments.
struct Spots {
    at: Vec<Point<Dbu>>,
    segs: Vec<Vec<SegmentId>>,
}

/// An arc of a matching that sends cell `cell` to spot `spot`.
#[derive(Clone, Copy, Debug)]
struct Choice {
    arc: usize,
    cell: usize,
    spot: usize,
}

/// The flow network of a matching of cells to spots: the supply feeds
/// each cell, each spot feeds the demand, and each cell may take each
/// spot within its reach at the cost of its wirelength there.
struct Matching {
    arcs: Vec<Arc>,
    choices: Vec<Choice>,
}

const fn cell_node(i: usize) -> usize {
    2 * i
}

const fn spot_node(i: usize) -> usize {
    2 * i + 1
}

const fn supply_node(count: usize) -> usize {
    2 * count
}

const fn demand_node(count: usize) -> usize {
    2 * count + 1
}

impl Matching {
    fn new(imp: &Improver, nodes: &[NodeId], spots: &[Point<Dbu>]) -> Self {
        let count = nodes.len();
        let mut matching = Self {
            arcs: Vec::new(),
            choices: Vec::new(),
        };
        for (i, &n) in nodes.iter().enumerate() {
            matching
                .arcs
                .push(unit_arc(supply_node(count), cell_node(i), 0));
            matching
                .arcs
                .push(unit_arc(spot_node(i), demand_node(count), 0));
            let orig = imp.orig(n);
            for (j, &at) in spots.iter().enumerate() {
                if i != j
                    && ((at.x - orig.x).abs() > imp.max_displacement.x
                        || (at.y - orig.y).abs() > imp.max_displacement.y)
                {
                    continue;
                }
                let center = Point::new(at.x + imp.width(n) / 2, at.y + imp.height(n) / 2);
                let cost = hpwl_at(imp, n, center).to_f64();
                matching.choices.push(Choice {
                    arc: matching.arcs.len(),
                    cell: i,
                    spot: j,
                });
                let cost = cost.min(f64::from(i32::MAX)) as i32;
                matching
                    .arcs
                    .push(unit_arc(cell_node(i), spot_node(j), cost));
            }
        }
        matching
    }
}

const fn unit_arc(source: usize, target: usize, cost: i32) -> Arc {
    Arc {
        source,
        target,
        capacity: 1,
        cost,
    }
}

/// Moves each cell the flow sends to another spot there, last choice
/// first, and returns the moves.
fn move_to_matched_spots(
    imp: &mut Improver,
    nodes: &[NodeId],
    spots: &Spots,
    choices: &[Choice],
    flow: &[i32],
) -> Result<Vec<Move>> {
    let mut journal = Vec::new();
    for &Choice {
        arc,
        cell: i,
        spot: j,
    } in choices.iter().rev()
    {
        if flow[arc] == 0 || i == j {
            continue;
        }
        let (n, m) = (nodes[i], nodes[j]);
        if imp.rows_spanned(n) != imp.rows_spanned(m)
            || imp.width(n) != imp.width(m)
            || imp.height(n) != imp.height(m)
            || spots.segs[i].len() != imp.rows_spanned(n)
            || spots.segs[j].len() != imp.rows_spanned(n)
        {
            return Err(ImprovementError::Internal(
                "Unable to interpret flow during matching",
            ));
        }
        for &s in &spots.segs[i] {
            imp.remove_cell_from_segment(n, s)?;
        }
        imp.erase(n);
        imp.set_position(n, spots.at[j]);
        imp.paint(n)?;
        for &s in &spots.segs[j] {
            imp.add_cell_to_segment(n, s)?;
        }
        journal.push(Move {
            node: n,
            from: spots.at[i],
            from_segs: spots.segs[i].clone(),
            to_segs: spots.segs[j].clone(),
        });
    }
    Ok(journal)
}

/// The wirelength of a cell's nets with it centered at `center`, as
/// `DetailedMis::getHpwl`; a net the cell has several pins on counts as
/// often.
fn hpwl_at(imp: &Improver, n: NodeId, center: Point<Dbu>) -> Dbu {
    let mut hpwl = Dbu::ZERO;
    for &pin in &imp.net.node_pins[n] {
        let edge = &imp.net.edges[imp.net.pins[pin].edge];
        if edge.len() <= 1 || edge.len() > SKIP_NETS_LARGER_THAN {
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
    use super::super::tests::{improver_of, tiny_from_one_spot};
    use super::*;
    use pare_common::db::library::SignalUse;

    const TOLERANCE: f64 = 0.005;

    /// The node of the cell named `name`.
    fn node(imp: &Improver, name: &str) -> NodeId {
        imp.grid
            .nodes
            .ids()
            .find(|&n| imp.grid.nodes[n].name == name)
            .unwrap()
    }

    /// The cells set matching gathers around `name`'s.
    fn neighbours_of(imp: &Improver, name: &str) -> Vec<NodeId> {
        let candidates = imp.movable_cells();
        let colors = color_cells(imp, &candidates).unwrap();
        let mut bins = Bins::new(imp, &candidates);
        bins.populate(imp, &candidates);
        gather_neighbours(imp, node(imp, name), &colors, &mut bins, &mut 0).unwrap()
    }

    #[test]
    fn inverters_alike_are_gathered_together() {
        let imp = improver_of(tiny_from_one_spot());

        let neighbours = neighbours_of(&imp, "u2");

        assert!(neighbours.contains(&node(&imp, "u5")));
    }

    #[test]
    fn an_inverter_with_its_rails_swapped_is_not_gathered() {
        let mut db = tiny_from_one_spot();
        let u5 = db.cell_id("u5").unwrap();
        let mut swapped = db.cell_macro(u5).clone();
        swapped.name = "INV_SWAPPED".into();
        let height = swapped.size.y;
        for pin in &mut swapped.pins {
            if matches!(pin.signal_use, SignalUse::Power | SignalUse::Ground) {
                for shape in &mut pin.shapes {
                    let (lo, hi) = (shape.rect.min.y, shape.rect.max.y);
                    (shape.rect.min.y, shape.rect.max.y) = (height - hi, height - lo);
                }
            }
        }
        db.cells[u5].macro_id = db.library.add_macro(swapped).unwrap();
        let imp = improver_of(db);

        let neighbours = neighbours_of(&imp, "u2");

        assert!(!neighbours.contains(&node(&imp, "u5")));
    }

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
