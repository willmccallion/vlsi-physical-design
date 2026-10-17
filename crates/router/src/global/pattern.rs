//! 3D pattern routing of a net's Steiner tree, as in CUGR.
//!
//! Every tree edge runs straight or as one of its two L shapes, each leg on
//! a layer of matching direction. Layers are chosen for the whole tree at
//! once by dynamic programming from the leaves up: a node's via stack spans
//! its pins' layers and the layers its edges attach on, and costs the via
//! cost per layer it climbs; a bend costs the vias between its two legs;
//! every leg costs its gcell edges at the current congestion. Edges and
//! via pads that fixed metal blocks outright cost so much that a plan takes
//! them only when every plan must. The tree comes out as paths in the
//! grid, each starting on the paths before it.

use super::steiner::{Spot, steiner_tree};
use super::{BLOCKED_COST, GCell, GlobalCosts, GlobalGrid, GlobalNet};
use pare_common::db::indices::LayerId;
use pare_common::db::library::Direction;

/// How a tree edge leaves its child for its parent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Shape {
    /// Child and parent share a gcell.
    Stacked,
    /// One leg along a row or column.
    Straight,
    /// Along the child's row first, then the parent's column.
    RowFirst,
    /// Along the child's column first, then the parent's row.
    ColumnFirst,
}

/// The cheapest way found to join a child to its parent with the parent
/// end on some layer.
#[derive(Clone, Copy, Debug)]
struct EdgeChoice {
    cost: f64,
    shape: Shape,
    /// Layer at the child's end.
    child_layer: usize,
}

/// The cheapest subtree under a node when its parent edge attaches on
/// some layer: its via stack's span.
#[derive(Clone, Copy, Debug)]
struct NodeChoice {
    cost: f64,
    low: usize,
    high: usize,
}

/// Paths of `net`'s tree through `grid`, priced at `costs`.
pub(super) fn route_pattern(
    grid: &GlobalGrid,
    net: &GlobalNet,
    costs: GlobalCosts,
) -> Vec<Vec<GCell>> {
    let terminals = &net.terminals;
    let mut spots: Vec<Spot> = terminals.iter().map(|c| (c.x, c.y)).collect();
    spots.sort_unstable();
    spots.dedup();
    if spots.is_empty() {
        return Vec::new();
    }
    let tree = steiner_tree(&spots);
    let layers = grid.directions.len();
    let base = grid.min_layer.index();
    let mut pins: Vec<Option<(usize, usize)>> = vec![None; tree.nodes.len()];
    for t in terminals {
        let Ok(k) = spots.binary_search(&(t.x, t.y)) else {
            continue;
        };
        let l = t.layer.index() - base;
        pins[k] = Some(pins[k].map_or((l, l), |(lo, hi)| (lo.min(l), hi.max(l))));
    }
    let mut children: Vec<Vec<usize>> = vec![Vec::new(); tree.nodes.len()];
    for &v in &tree.order[1..] {
        children[tree.parent[v]].push(v);
    }

    let planner = Planner {
        grid,
        net,
        costs,
        layers,
        base,
    };
    let mut node_best: Vec<Vec<Option<NodeChoice>>> = vec![Vec::new(); tree.nodes.len()];
    let mut edge_best: Vec<Vec<Option<EdgeChoice>>> = vec![Vec::new(); tree.nodes.len()];
    for &v in tree.order.iter().rev() {
        for &c in &children[v] {
            edge_best[c] = planner.edges(tree.nodes[c], tree.nodes[v], &node_best[c]);
        }
        node_best[v] = planner.node(tree.nodes[v], pins[v], &children[v], &edge_best);
    }

    let mut span = vec![(0, 0); tree.nodes.len()];
    let mut attach = vec![0; tree.nodes.len()];
    let Some(root) = (0..layers)
        .filter_map(|l| node_best[0][l].map(|n| (n.cost, l, n)))
        .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)))
    else {
        return Vec::new();
    };
    span[0] = (root.2.low, root.2.high);
    let cell = |spot: Spot, l: usize| GCell {
        layer: LayerId::new(base + l),
        x: spot.0,
        y: spot.1,
    };
    let mut paths = vec![
        (span[0].0..=span[0].1)
            .map(|l| cell(tree.nodes[0], l))
            .collect(),
    ];
    for &v in &tree.order {
        for &c in &children[v] {
            let (low, high) = span[v];
            let Some((parent_layer, edge)) = (low..=high)
                .filter_map(|l| edge_best[c][l].map(|e| (l, e)))
                .min_by(|a, b| a.1.cost.total_cmp(&b.1.cost).then(a.0.cmp(&b.0)))
            else {
                continue;
            };
            attach[c] = edge.child_layer;
            let Some(n) = node_best[c][edge.child_layer] else {
                continue;
            };
            span[c] = (n.low, n.high);
            paths.push(planner.edge_path(tree.nodes[v], parent_layer, tree.nodes[c], edge));
            let a = attach[c];
            if n.low < a {
                paths.push((n.low..=a).rev().map(|l| cell(tree.nodes[c], l)).collect());
            }
            if a < n.high {
                paths.push((a..=n.high).map(|l| cell(tree.nodes[c], l)).collect());
            }
        }
    }
    paths
}

/// What the dynamic program needs to price layers.
struct Planner<'a> {
    grid: &'a GlobalGrid,
    net: &'a GlobalNet,
    costs: GlobalCosts,
    layers: usize,
    base: usize,
}

impl Planner<'_> {
    fn direction(&self, l: usize) -> Direction {
        self.grid.directions[l]
    }

    const fn layer(&self, l: usize) -> LayerId {
        LayerId::new(self.base + l)
    }

    fn via(&self, a: usize, b: usize) -> f64 {
        self.costs.via * a.abs_diff(b) as f64
    }

    /// Cost of a via stack at `spot` between layers `a` and `b`: the via
    /// cost per layer climbed, and the blocked cost for every layer whose
    /// cell there has no room for a pad.
    fn stack(&self, spot: Spot, a: usize, b: usize) -> f64 {
        let (lo, hi) = (a.min(b), a.max(b));
        if lo == hi {
            return 0.0;
        }
        let blocked = (lo..=hi)
            .filter(|&l| {
                self.grid.via_blocked(GCell {
                    layer: self.layer(l),
                    x: spot.0,
                    y: spot.1,
                })
            })
            .count();
        BLOCKED_COST.mul_add(blocked as f64, self.via(a, b))
    }

    /// Cost of a leg on layer `l` from `from` to `to` along one row or
    /// column, or `None` if `l` runs the other way.
    fn leg(&self, l: usize, from: Spot, to: Spot) -> Option<f64> {
        let horizontal = from.1 == to.1;
        let wanted = if horizontal {
            Direction::Horizontal
        } else {
            Direction::Vertical
        };
        if self.direction(l) != wanted {
            return None;
        }
        let (lo, hi) = if horizontal {
            (from.0.min(to.0), from.0.max(to.0))
        } else {
            (from.1.min(to.1), from.1.max(to.1))
        };
        let demand = self.net.demand(l);
        let cost = (lo..hi)
            .map(|k| {
                let start = if horizontal {
                    GCell {
                        layer: self.layer(l),
                        x: k,
                        y: from.1,
                    }
                } else {
                    GCell {
                        layer: self.layer(l),
                        x: from.0,
                        y: k,
                    }
                };
                let e = self.grid.edge(start);
                if self.grid.blocked[e] {
                    BLOCKED_COST
                } else {
                    self.grid.edge_cost(e, demand, self.costs)
                }
            })
            .sum();
        Some(cost)
    }

    /// For each layer at the parent end, the cheapest way to join `child`,
    /// whose subtree costs `below` by attach layer, to `parent`.
    fn edges(
        &self,
        child: Spot,
        parent: Spot,
        below: &[Option<NodeChoice>],
    ) -> Vec<Option<EdgeChoice>> {
        let mut best: Vec<Option<EdgeChoice>> = vec![None; self.layers];
        let mut offer = |parent_layer: usize, child_layer: usize, cost: f64, shape: Shape| {
            let Some(sub) = below[child_layer] else {
                return;
            };
            let total = cost + sub.cost;
            if best[parent_layer].is_none_or(|b| total < b.cost) {
                best[parent_layer] = Some(EdgeChoice {
                    cost: total,
                    shape,
                    child_layer,
                });
            }
        };
        if child == parent {
            for a in 0..self.layers {
                for b in 0..self.layers {
                    offer(b, a, self.stack(child, a, b), Shape::Stacked);
                }
            }
        } else if child.0 == parent.0 || child.1 == parent.1 {
            for l in 0..self.layers {
                if let Some(cost) = self.leg(l, child, parent) {
                    offer(l, l, cost, Shape::Straight);
                }
            }
        } else {
            for (shape, bend) in [
                (Shape::RowFirst, (parent.0, child.1)),
                (Shape::ColumnFirst, (child.0, parent.1)),
            ] {
                let first: Vec<Option<f64>> =
                    (0..self.layers).map(|a| self.leg(a, child, bend)).collect();
                let second: Vec<Option<f64>> = (0..self.layers)
                    .map(|b| self.leg(b, bend, parent))
                    .collect();
                for (a, ca) in first.iter().enumerate() {
                    for (b, cb) in second.iter().enumerate() {
                        if let (Some(ca), Some(cb)) = (ca, cb) {
                            offer(b, a, ca + cb + self.stack(bend, a, b), shape);
                        }
                    }
                }
            }
        }
        best
    }

    /// For each layer its parent edge may attach on, the cheapest via stack
    /// at `spot` joining a node's pins and its children's edges.
    fn node(
        &self,
        spot: Spot,
        pins: Option<(usize, usize)>,
        children: &[usize],
        edge_best: &[Vec<Option<EdgeChoice>>],
    ) -> Vec<Option<NodeChoice>> {
        let mut best: Vec<Option<NodeChoice>> = vec![None; self.layers];
        for low in 0..self.layers {
            for high in low..self.layers {
                if pins.is_some_and(|(lo, hi)| lo < low || hi > high) {
                    continue;
                }
                let mut cost = self.stack(spot, low, high);
                let joined = children.iter().all(|&c| {
                    let cheapest = (low..=high)
                        .filter_map(|l| edge_best[c][l].map(|e| e.cost))
                        .min_by(f64::total_cmp);
                    cheapest.is_some_and(|e| {
                        cost += e;
                        true
                    })
                });
                if !joined {
                    continue;
                }
                for slot in &mut best[low..=high] {
                    if slot.is_none_or(|b| cost < b.cost) {
                        *slot = Some(NodeChoice { cost, low, high });
                    }
                }
            }
        }
        best
    }

    /// The cells of an edge from `parent` on `parent_layer` to `child`.
    fn edge_path(
        &self,
        parent: Spot,
        parent_layer: usize,
        child: Spot,
        edge: EdgeChoice,
    ) -> Vec<GCell> {
        let mut path = Vec::new();
        let walk = |from: Spot, to: Spot, l: usize, path: &mut Vec<GCell>| {
            let steps = from.0.abs_diff(to.0).max(from.1.abs_diff(to.1));
            for k in 0..=steps {
                let toward = |a: u16, b: u16| {
                    if b >= a {
                        a + k.min(b - a)
                    } else {
                        a - k.min(a - b)
                    }
                };
                let cell = GCell {
                    layer: self.layer(l),
                    x: toward(from.0, to.0),
                    y: toward(from.1, to.1),
                };
                if path.last() != Some(&cell) {
                    path.push(cell);
                }
            }
        };
        let climb = |at: Spot, from: usize, to: usize, path: &mut Vec<GCell>| {
            let layers: Vec<usize> = if from <= to {
                (from..=to).collect()
            } else {
                (to..=from).rev().collect()
            };
            for l in layers {
                let cell = GCell {
                    layer: self.layer(l),
                    x: at.0,
                    y: at.1,
                };
                if path.last() != Some(&cell) {
                    path.push(cell);
                }
            }
        };
        match edge.shape {
            Shape::Stacked => climb(parent, parent_layer, edge.child_layer, &mut path),
            Shape::Straight => walk(parent, child, parent_layer, &mut path),
            Shape::RowFirst | Shape::ColumnFirst => {
                let bend = if edge.shape == Shape::RowFirst {
                    (parent.0, child.1)
                } else {
                    (child.0, parent.1)
                };
                walk(parent, bend, parent_layer, &mut path);
                climb(bend, parent_layer, edge.child_layer, &mut path);
                walk(bend, child, edge.child_layer, &mut path);
            }
        }
        path
    }
}
