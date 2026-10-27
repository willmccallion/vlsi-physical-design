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

/// The flow on each arc, in the order the arcs were added, after the
/// simplex sends `supply` from `source` to `target` over `node_count` nodes.
pub fn min_cost_flow(
    node_count: usize,
    arcs: &[Arc],
    source: usize,
    target: usize,
    supply: i32,
) -> Vec<i32> {
    let mut ns = Simplex::new(node_count, arcs);
    ns.supply[ns.node_id[source]] = supply;
    ns.supply[ns.node_id[target]] = supply.wrapping_neg();
    ns.run();
    (0..arcs.len()).map(|a| ns.flow[ns.arc_id[a]]).collect()
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
    parent: Vec<i64>,
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
            parent: vec![0; all_node_num],
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
    /// block search rule; the outcome is left in the flows.
    fn run(&mut self) {
        if self.node_num == 0 {
            return;
        }
        let sum_supply = self.supply[..self.node_num]
            .iter()
            .fold(0_i32, |a, &b| a.wrapping_add(b));
        if sum_supply > 0 {
            return;
        }
        for e in 0..self.arc_num {
            self.flow[e] = 0;
            self.state[e] = STATE_LOWER;
        }
        let root = self.root;
        self.parent[root] = -1;
        self.thread[root] = 0;
        self.rev_thread[0] = root;
        self.succ_num[root] = (self.node_num + 1) as i32;
        self.last_succ[root] = root - 1;
        self.supply[root] = sum_supply.wrapping_neg();
        self.pi[root] = 0;
        if sum_supply != 0 {
            return;
        }
        self.search_arc_num = self.arc_num;
        self.all_arc_num = self.arc_num + self.node_num;
        for u in 0..self.node_num {
            let e = self.arc_num + u;
            self.parent[u] = root as i64;
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
                return;
            }
            self.change_flow(change);
            if change {
                self.update_tree_structure();
                self.update_potential();
            }
        }
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

    fn up(&self, u: usize) -> usize {
        self.parent[u] as usize
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

    #[allow(clippy::too_many_lines)]
    fn update_tree_structure(&mut self) {
        let (u_in, v_in, u_out) = (self.u_in, self.v_in, self.u_out);
        let old_rev_thread = self.rev_thread[u_out];
        let old_succ_num = self.succ_num[u_out];
        let old_last_succ = self.last_succ[u_out];
        let v_out = self.up(u_out);
        if u_in == u_out {
            self.parent[u_in] = v_in as i64;
            self.pred[u_in] = self.in_arc;
            self.pred_dir[u_in] = if u_in == self.source[self.in_arc] {
                DIR_UP
            } else {
                DIR_DOWN
            };
            if self.thread[v_in] != u_out {
                let mut after = self.thread[old_last_succ];
                self.thread[old_rev_thread] = after;
                self.rev_thread[after] = old_rev_thread;
                after = self.thread[v_in];
                self.thread[v_in] = u_out;
                self.rev_thread[u_out] = v_in;
                self.thread[old_last_succ] = after;
                self.rev_thread[after] = old_last_succ;
            }
        } else {
            let thread_continue = if old_rev_thread == v_in {
                self.thread[old_last_succ]
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
                self.parent[stem] = par_stem as i64;
                par_stem = stem;
                stem = next_stem;
                last = if self.last_succ[stem] == self.last_succ[par_stem] {
                    self.rev_thread[par_stem]
                } else {
                    self.last_succ[stem]
                };
                after = self.thread[last];
            }
            self.parent[u_out] = par_stem as i64;
            self.thread[last] = thread_continue;
            self.rev_thread[thread_continue] = last;
            self.last_succ[u_out] = last;
            if old_rev_thread != v_in {
                self.thread[old_rev_thread] = after;
                self.rev_thread[after] = old_rev_thread;
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
            self.pred_dir[u_in] = if u_in == self.source[self.in_arc] {
                DIR_UP
            } else {
                DIR_DOWN
            };
            self.succ_num[u_in] = old_succ_num;
        }
        let join = self.join;
        let up_limit_out = if self.last_succ[join] == v_in {
            join as i64
        } else {
            -1
        };
        let last_succ_out = self.last_succ[u_out];
        let mut u = v_in as i64;
        while u != -1 && self.last_succ[u as usize] == v_in {
            self.last_succ[u as usize] = last_succ_out;
            u = self.parent[u as usize];
        }
        if join != old_rev_thread && v_in != old_rev_thread {
            let mut u = v_out as i64;
            while u != up_limit_out && self.last_succ[u as usize] == old_last_succ {
                self.last_succ[u as usize] = old_rev_thread;
                u = self.parent[u as usize];
            }
        } else if last_succ_out != old_last_succ {
            let mut u = v_out as i64;
            while u != up_limit_out && self.last_succ[u as usize] == old_last_succ {
                self.last_succ[u as usize] = last_succ_out;
                u = self.parent[u as usize];
            }
        }
        let mut u = v_in;
        while u != join {
            self.succ_num[u] += old_succ_num;
            u = self.up(u);
        }
        let mut u = v_out;
        while u != join {
            self.succ_num[u] -= old_succ_num;
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
