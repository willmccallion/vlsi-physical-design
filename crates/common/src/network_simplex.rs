//! LEMON's network simplex as `TritonCTS` and `OpenDP` run it.
//!
//! Their flows, ties included, come out as they do there: a `ListDigraph`,
//! whose nodes and arcs iterate last-created first, its arcs mixed into the
//! simplex's order, `int` flows and costs with their wrapping arithmetic,
//! equality supplies, and the block search pivot rule.

const STATE_UPPER: i32 = -1;
const STATE_TREE: i32 = 0;
const STATE_LOWER: i32 = 1;
const DIR_DOWN: i32 = -1;
const DIR_UP: i32 = 1;
const MAX: i32 = i32::MAX;
const INF: i32 = MAX;
const ART_COST: i32 = i32::MAX / 2 + 1;

/// An arc as it was added.
#[derive(Clone, Copy, Debug)]
pub struct Arc {
    /// The node the arc leaves, in the order the nodes were added.
    pub source: usize,
    /// The node the arc enters.
    pub target: usize,
    /// The most flow the arc carries; none is the least.
    pub capacity: i32,
    /// The cost of a unit of flow along it.
    pub cost: i32,
}

/// Why the simplex found no optimal flow, as LEMON's `ProblemType` says.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum FlowProblem {
    /// No flow meets the supplies.
    #[error("no flow meets the supplies")]
    Infeasible,
    /// The cost falls without bound.
    #[error("the cost falls without bound")]
    Unbounded,
}

/// A run that ended without an optimal flow: why, and the flow on each arc
/// as the simplex left it, which `TritonCTS` reads all the same.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("{problem}")]
pub struct NoOptimalFlow {
    /// Why there is no optimal flow.
    pub problem: FlowProblem,
    /// The flow on each arc, in the order the arcs were added.
    pub flows: Vec<i32>,
}

/// The flow on each arc, in the order the arcs were added, after the
/// simplex sends `supply` from `source` to `target` over `node_count` nodes,
/// or why it found no optimal flow.
pub fn min_cost_flow(
    node_count: usize,
    arcs: &[Arc],
    source: usize,
    target: usize,
    supply: i32,
) -> Result<Vec<i32>, NoOptimalFlow> {
    let mut ns = Simplex::new(node_count, arcs);
    ns.supply[ns.node_id[source]] = supply;
    ns.supply[ns.node_id[target]] = supply.wrapping_neg();
    let outcome = ns.run();
    let flows = (0..arcs.len()).map(|a| ns.flow[ns.arc_id[a]]).collect();
    match outcome {
        Ok(()) => Ok(flows),
        Err(problem) => Err(NoOptimalFlow { problem, flows }),
    }
}

struct Simplex {
    node_num: usize,
    arc_num: usize,
    node_id: Vec<usize>,
    arc_id: Vec<usize>,
    source: Vec<usize>,
    target: Vec<usize>,
    cap: Vec<i32>,
    cost: Vec<i32>,
    supply: Vec<i32>,
    flow: Vec<i32>,
    pi: Vec<i32>,
    /// The parent of each node but the root, which has none.
    parent: Vec<usize>,
    pred: Vec<usize>,
    pred_dir: Vec<i32>,
    thread: Vec<usize>,
    rev_thread: Vec<usize>,
    succ_num: Vec<i32>,
    last_succ: Vec<usize>,
    state: Vec<i32>,
    dirty_revs: Vec<usize>,
    root: usize,
    search_arc_num: usize,
    all_arc_num: usize,
    in_arc: usize,
    join: usize,
    u_in: usize,
    v_in: usize,
    u_out: usize,
    delta: i32,
}

/// The leaving arc's subtree root's thread links and size as they were
/// before the pivot.
#[derive(Clone, Copy)]
struct OldSubtree {
    rev_thread: usize,
    succ_num: i32,
    last_succ: usize,
}

impl Simplex {
    fn new(node_count: usize, arcs: &[Arc]) -> Self {
        let node_num = node_count;
        let arc_num = arcs.len();
        let node_id: Vec<usize> = (0..node_count).map(|c| node_count - 1 - c).collect();
        let mut out: Vec<Vec<usize>> = vec![Vec::new(); node_count];
        for (a, arc) in arcs.iter().enumerate() {
            out[arc.source].push(a);
        }
        let iteration: Vec<usize> = (0..node_count)
            .rev()
            .flat_map(|n| out[n].iter().rev().copied().collect::<Vec<_>>())
            .collect();
        let max_arc_num = arc_num + 2 * node_num;
        let all_node_num = node_num + 1;
        let mut s = Self {
            node_num,
            arc_num,
            node_id,
            arc_id: vec![0; arc_num],
            source: vec![0; max_arc_num],
            target: vec![0; max_arc_num],
            cap: vec![0; max_arc_num],
            cost: vec![0; max_arc_num],
            supply: vec![0; all_node_num],
            flow: vec![0; max_arc_num],
            pi: vec![0; all_node_num],
            parent: vec![0; node_num],
            pred: vec![0; all_node_num],
            pred_dir: vec![0; all_node_num],
            thread: vec![0; all_node_num],
            rev_thread: vec![0; all_node_num],
            succ_num: vec![0; all_node_num],
            last_succ: vec![0; all_node_num],
            state: vec![0; max_arc_num],
            dirty_revs: Vec::new(),
            root: node_num,
            search_arc_num: 0,
            all_arc_num: 0,
            in_arc: 0,
            join: 0,
            u_in: 0,
            v_in: 0,
            u_out: 0,
            delta: 0,
        };
        let mixing = node_num > 1;
        let skip = (arc_num / node_num.max(1)).max(3);
        let (mut i, mut j) = (0, 0);
        for (k, &a) in iteration.iter().enumerate() {
            let id = if mixing { i } else { k };
            s.arc_id[a] = id;
            s.source[id] = s.node_id[arcs[a].source];
            s.target[id] = s.node_id[arcs[a].target];
            s.cap[id] = arcs[a].capacity;
            s.cost[id] = arcs[a].cost;
            if mixing {
                i += skip;
                if i >= arc_num {
                    j += 1;
                    i = j;
                }
            }
        }
        s
    }

    /// `NetworkSimplex::init` for equal supplies, then `start` with the
    /// block search rule; the flows are left in `flow`.
    fn run(&mut self) -> Result<(), FlowProblem> {
        if self.node_num == 0 {
            return Err(FlowProblem::Infeasible);
        }
        let sum_supply = self.supply[..self.node_num]
            .iter()
            .fold(0_i32, |a, &b| a.wrapping_add(b));
        if sum_supply > 0 {
            return Err(FlowProblem::Infeasible);
        }
        for e in 0..self.arc_num {
            self.flow[e] = 0;
            self.state[e] = STATE_LOWER;
        }
        let root = self.root;
        self.thread[root] = 0;
        self.rev_thread[0] = root;
        self.succ_num[root] = (self.node_num + 1) as i32;
        self.last_succ[root] = root - 1;
        self.supply[root] = sum_supply.wrapping_neg();
        self.pi[root] = 0;
        if sum_supply != 0 {
            // NOTE: NetworkSimplex::init solves supplies short of the demand
            // too; this port, run with equal supplies alone, refuses them.
            return Err(FlowProblem::Infeasible);
        }
        self.search_arc_num = self.arc_num;
        self.all_arc_num = self.arc_num + self.node_num;
        for u in 0..self.node_num {
            let e = self.arc_num + u;
            self.parent[u] = root;
            self.pred[u] = e;
            self.thread[u] = u + 1;
            self.rev_thread[u + 1] = u;
            self.succ_num[u] = 1;
            self.last_succ[u] = u;
            self.cap[e] = INF;
            self.state[e] = STATE_TREE;
            if self.supply[u] >= 0 {
                self.pred_dir[u] = DIR_UP;
                self.pi[u] = 0;
                self.source[e] = u;
                self.target[e] = root;
                self.flow[e] = self.supply[u];
                self.cost[e] = 0;
            } else {
                self.pred_dir[u] = DIR_DOWN;
                self.pi[u] = ART_COST;
                self.source[e] = root;
                self.target[e] = u;
                self.flow[e] = self.supply[u].wrapping_neg();
                self.cost[e] = ART_COST;
            }
        }
        let block = ((self.search_arc_num as f64).sqrt() as usize).max(10);
        let mut next_arc = 0;
        while let Some(next) = self.find_entering_arc(block, next_arc) {
            next_arc = next;
            self.find_join_node();
            let change = self.find_leaving_arc();
            if self.delta == MAX {
                return Err(FlowProblem::Unbounded);
            }
            self.change_flow(change);
            if change {
                self.update_tree_structure();
                self.update_potential();
            }
        }
        let artificial = self.search_arc_num..self.all_arc_num;
        if self.flow[artificial].iter().any(|&f| f != 0) {
            return Err(FlowProblem::Infeasible);
        }
        Ok(())
    }

    fn reduced(&self, e: usize) -> i32 {
        self.state[e].wrapping_mul(
            self.cost[e]
                .wrapping_add(self.pi[self.source[e]])
                .wrapping_sub(self.pi[self.target[e]]),
        )
    }

    /// `BlockSearchPivotRule::findEnteringArc`: the most negative reduced
    /// cost within the first block holding one, searching on from where
    /// the last search stopped; returns where the next search starts.
    fn find_entering_arc(&mut self, block: usize, next_arc: usize) -> Option<usize> {
        let mut min = 0_i32;
        let mut cnt = block;
        let order = (next_arc..self.search_arc_num).chain(0..next_arc);
        for e in order {
            let c = self.reduced(e);
            if c < min {
                min = c;
                self.in_arc = e;
            }
            cnt -= 1;
            if cnt == 0 {
                if min < 0 {
                    return Some(e);
                }
                cnt = block;
            }
        }
        (min < 0).then_some(next_arc)
    }

    /// The parent of `u`, which is not the root.
    fn up(&self, u: usize) -> usize {
        self.parent[u]
    }

    /// The parent of `u`, or none for the root.
    fn parent_of(&self, u: usize) -> Option<usize> {
        (u != self.root).then(|| self.parent[u])
    }

    fn find_join_node(&mut self) {
        let mut u = self.source[self.in_arc];
        let mut v = self.target[self.in_arc];
        while u != v {
            if self.succ_num[u] < self.succ_num[v] {
                u = self.up(u);
            } else {
                v = self.up(v);
            }
        }
        self.join = u;
    }

    fn find_leaving_arc(&mut self) -> bool {
        let (first, second) = if self.state[self.in_arc] == STATE_LOWER {
            (self.source[self.in_arc], self.target[self.in_arc])
        } else {
            (self.target[self.in_arc], self.source[self.in_arc])
        };
        self.delta = self.cap[self.in_arc];
        let mut result = 0;
        let mut u = first;
        while u != self.join {
            let e = self.pred[u];
            let mut d = self.flow[e];
            if self.pred_dir[u] == DIR_DOWN {
                let c = self.cap[e];
                d = if c == MAX { INF } else { c.wrapping_sub(d) };
            }
            if d < self.delta {
                self.delta = d;
                self.u_out = u;
                result = 1;
            }
            u = self.up(u);
        }
        let mut u = second;
        while u != self.join {
            let e = self.pred[u];
            let mut d = self.flow[e];
            if self.pred_dir[u] == DIR_UP {
                let c = self.cap[e];
                d = if c == MAX { INF } else { c.wrapping_sub(d) };
            }
            if d <= self.delta {
                self.delta = d;
                self.u_out = u;
                result = 2;
            }
            u = self.up(u);
        }
        if result == 1 {
            self.u_in = first;
            self.v_in = second;
        } else {
            self.u_in = second;
            self.v_in = first;
        }
        result != 0
    }

    fn change_flow(&mut self, change: bool) {
        if self.delta > 0 {
            let val = self.state[self.in_arc].wrapping_mul(self.delta);
            self.flow[self.in_arc] = self.flow[self.in_arc].wrapping_add(val);
            let mut u = self.source[self.in_arc];
            while u != self.join {
                let e = self.pred[u];
                self.flow[e] = self.flow[e].wrapping_sub(self.pred_dir[u].wrapping_mul(val));
                u = self.up(u);
            }
            let mut u = self.target[self.in_arc];
            while u != self.join {
                let e = self.pred[u];
                self.flow[e] = self.flow[e].wrapping_add(self.pred_dir[u].wrapping_mul(val));
                u = self.up(u);
            }
        }
        if change {
            self.state[self.in_arc] = STATE_TREE;
            let out = self.pred[self.u_out];
            self.state[out] = if self.flow[out] == 0 {
                STATE_LOWER
            } else {
                STATE_UPPER
            };
        } else {
            self.state[self.in_arc] = -self.state[self.in_arc];
        }
    }

    /// Rehangs the spanning tree on the entering arc, as LEMON's
    /// `updateTreeStructure`.
    fn update_tree_structure(&mut self) {
        let u_out = self.u_out;
        let old = OldSubtree {
            rev_thread: self.rev_thread[u_out],
            succ_num: self.succ_num[u_out],
            last_succ: self.last_succ[u_out],
        };
        let v_out = self.up(u_out);
        if self.u_in == u_out {
            self.rehang_leaving_root(old);
        } else {
            self.reverse_stem(old);
        }
        self.update_last_succ(v_out, old);
        self.update_succ_num(v_out, old.succ_num);
    }

    /// Hangs the leaving arc's subtree root, which is the entering arc's
    /// end, from the entering arc's other end.
    fn rehang_leaving_root(&mut self, old: OldSubtree) {
        let (u_in, v_in, u_out) = (self.u_in, self.v_in, self.u_out);
        self.parent[u_in] = v_in;
        self.pred[u_in] = self.in_arc;
        self.pred_dir[u_in] = self.entering_direction();
        if self.thread[v_in] != u_out {
            let mut after = self.thread[old.last_succ];
            self.thread[old.rev_thread] = after;
            self.rev_thread[after] = old.rev_thread;
            after = self.thread[v_in];
            self.thread[v_in] = u_out;
            self.rev_thread[u_out] = v_in;
            self.thread[old.last_succ] = after;
            self.rev_thread[after] = old.last_succ;
        }
    }

    /// Reverses the stem from the entering arc's end up to the leaving
    /// arc's subtree root, rethreading it and moving each node's
    /// predecessor arc and subtree size one step down the stem.
    fn reverse_stem(&mut self, old: OldSubtree) {
        let (u_in, v_in, u_out) = (self.u_in, self.v_in, self.u_out);
        let thread_continue = if old.rev_thread == v_in {
            self.thread[old.last_succ]
        } else {
            self.thread[v_in]
        };
        let mut stem = u_in;
        let mut par_stem = v_in;
        let mut last = self.last_succ[u_in];
        let mut after = self.thread[last];
        self.thread[v_in] = u_in;
        self.dirty_revs.clear();
        self.dirty_revs.push(v_in);
        while stem != u_out {
            let next_stem = self.up(stem);
            self.thread[last] = next_stem;
            self.dirty_revs.push(last);
            let before = self.rev_thread[stem];
            self.thread[before] = after;
            self.rev_thread[after] = before;
            self.parent[stem] = par_stem;
            par_stem = stem;
            stem = next_stem;
            last = if self.last_succ[stem] == self.last_succ[par_stem] {
                self.rev_thread[par_stem]
            } else {
                self.last_succ[stem]
            };
            after = self.thread[last];
        }
        self.parent[u_out] = par_stem;
        self.thread[last] = thread_continue;
        self.rev_thread[thread_continue] = last;
        self.last_succ[u_out] = last;
        if old.rev_thread != v_in {
            self.thread[old.rev_thread] = after;
            self.rev_thread[after] = old.rev_thread;
        }
        for i in 0..self.dirty_revs.len() {
            let u = self.dirty_revs[i];
            let t = self.thread[u];
            self.rev_thread[t] = u;
        }
        let mut tmp_sc = 0_i32;
        let tmp_ls = self.last_succ[u_out];
        let mut u = u_out;
        while u != u_in {
            let p = self.up(u);
            self.pred[u] = self.pred[p];
            self.pred_dir[u] = -self.pred_dir[p];
            tmp_sc += self.succ_num[u] - self.succ_num[p];
            self.succ_num[u] = tmp_sc;
            self.last_succ[p] = tmp_ls;
            u = p;
        }
        self.pred[u_in] = self.in_arc;
        self.pred_dir[u_in] = self.entering_direction();
        self.succ_num[u_in] = old.succ_num;
    }

    /// The direction of the entering arc seen from its end `u_in`.
    fn entering_direction(&self) -> i32 {
        if self.u_in == self.source[self.in_arc] {
            DIR_UP
        } else {
            DIR_DOWN
        }
    }

    /// Updates the last successors above the entering arc's other end and
    /// above the leaving arc, up to their join.
    fn update_last_succ(&mut self, v_out: usize, old: OldSubtree) {
        let (v_in, u_out, join) = (self.v_in, self.u_out, self.join);
        let up_limit_out = (self.last_succ[join] == v_in).then_some(join);
        let last_succ_out = self.last_succ[u_out];
        let mut u = Some(v_in);
        while let Some(at) = u
            && self.last_succ[at] == v_in
        {
            self.last_succ[at] = last_succ_out;
            u = self.parent_of(at);
        }
        let replacement = if join != old.rev_thread && v_in != old.rev_thread {
            old.rev_thread
        } else if last_succ_out != old.last_succ {
            last_succ_out
        } else {
            return;
        };
        let mut u = Some(v_out);
        while let Some(at) = u
            && u != up_limit_out
            && self.last_succ[at] == old.last_succ
        {
            self.last_succ[at] = replacement;
            u = self.parent_of(at);
        }
    }

    /// Moves the subtree's size from the leaving side to the entering side
    /// up to the join.
    fn update_succ_num(&mut self, v_out: usize, moved: i32) {
        let join = self.join;
        let mut u = self.v_in;
        while u != join {
            self.succ_num[u] += moved;
            u = self.up(u);
        }
        let mut u = v_out;
        while u != join {
            self.succ_num[u] -= moved;
            u = self.up(u);
        }
    }

    fn update_potential(&mut self) {
        let sigma = self.pi[self.v_in]
            .wrapping_sub(self.pi[self.u_in])
            .wrapping_sub(self.pred_dir[self.u_in].wrapping_mul(self.cost[self.in_arc]));
        let end = self.thread[self.last_succ[self.u_in]];
        let mut u = self.u_in;
        while u != end {
            self.pi[u] = self.pi[u].wrapping_add(sigma);
            u = self.thread[u];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn arc(source: usize, target: usize, capacity: i32, cost: i32) -> Arc {
        Arc {
            source,
            target,
            capacity,
            cost,
        }
    }

    #[test]
    fn the_cheaper_of_two_paths_carries_the_flow() {
        let arcs = [
            arc(0, 1, 1, 5),
            arc(0, 2, 1, 1),
            arc(1, 3, 1, 0),
            arc(2, 3, 1, 0),
        ];

        let flows = min_cost_flow(4, &arcs, 0, 3, 1);

        assert_eq!(flows, Ok(vec![0, 1, 0, 1]));
    }

    #[test]
    fn a_supply_no_path_can_carry_is_infeasible() {
        let arcs = [arc(0, 1, 1, 0), arc(1, 2, 1, 0)];

        let outcome = min_cost_flow(3, &arcs, 0, 2, 2);

        assert_eq!(outcome.map_err(|e| e.problem), Err(FlowProblem::Infeasible));
    }
}
