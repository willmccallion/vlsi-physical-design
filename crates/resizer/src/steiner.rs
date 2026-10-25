//! Rectilinear Steiner trees over a net's pins, as `OpenROAD`'s `stt` builds
//! them for parasitic estimation.
//!
//! A Prim-Dijkstra spanning tree weighs each edge against the path back to
//! the driver; Steiner points then join edges that can share a run, and
//! nodes of degree above three are split.

use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// The Prim-Dijkstra trade-off `OpenROAD` defaults to: the share of the
/// path length from the driver that an edge's weight carries.
pub const DEFAULT_ALPHA: f64 = 0.3;

/// A tree over points in the plane, the given terminals first and the
/// Steiner points after them, each node naming its parent and the root
/// naming itself.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SteinerTree {
    /// Every node's position.
    pub nodes: Vec<Point<Dbu>>,
    /// Every node's parent, the root's being itself.
    pub parent: Vec<usize>,
    /// How many of the nodes are terminals.
    pub terminals: usize,
    /// The root, which is the driver's terminal.
    pub root: usize,
}

impl SteinerTree {
    /// The children of each node.
    #[must_use]
    pub fn children(&self) -> Vec<Vec<usize>> {
        let mut children = vec![Vec::new(); self.nodes.len()];
        for (node, &parent) in self.parent.iter().enumerate() {
            if parent != node {
                children[parent].push(node);
            }
        }
        children
    }

    /// Total wire length.
    #[must_use]
    pub fn length(&self) -> i64 {
        self.parent
            .iter()
            .enumerate()
            .map(|(node, &parent)| manhattan(self.nodes[node], self.nodes[parent]))
            .sum()
    }
}

const fn manhattan(a: Point<Dbu>, b: Point<Dbu>) -> i64 {
    (a.x.get() - b.x.get()).abs() + (a.y.get() - b.y.get()).abs()
}

/// Builds a Steiner tree over `points`, rooted at `driver`, as `OpenROAD`'s
/// `pdr::primDijkstra` with trade-off `alpha`. The points must not be
/// empty.
#[must_use]
pub fn prim_dijkstra(points: &[Point<Dbu>], driver: usize, alpha: f64) -> SteinerTree {
    let mut graph = Graph {
        points: points.to_vec(),
        adjacent: vec![Vec::new(); points.len()],
    };
    let neighbors = nearest_neighbors(points);
    build_spanning_tree(&mut graph, driver, alpha, &neighbors);
    steinerize(&mut graph);
    split_high_degree_nodes(&mut graph);
    graph.into_tree(points.len(), driver)
}

/// An undirected graph over points.
struct Graph {
    points: Vec<Point<Dbu>>,
    adjacent: Vec<Vec<usize>>,
}

impl Graph {
    fn add_edge(&mut self, a: usize, b: usize) {
        self.adjacent[a].push(b);
        self.adjacent[b].push(a);
    }

    fn remove_edge(&mut self, a: usize, b: usize) {
        if let Some(i) = self.adjacent[a].iter().position(|&n| n == b) {
            let _ = self.adjacent[a].remove(i);
        }
        if let Some(i) = self.adjacent[b].iter().position(|&n| n == a) {
            let _ = self.adjacent[b].remove(i);
        }
    }

    fn add_node(&mut self, at: Point<Dbu>) -> usize {
        self.points.push(at);
        self.adjacent.push(Vec::new());
        self.points.len() - 1
    }

    /// Reads the graph out as a tree rooted at `driver`.
    fn into_tree(self, terminals: usize, driver: usize) -> SteinerTree {
        let n = self.points.len();
        let mut parent = vec![usize::MAX; n];
        let mut order = vec![driver];
        parent[driver] = driver;
        let mut i = 0;
        while i < order.len() {
            let node = order[i];
            for &next in &self.adjacent[node] {
                if parent[next] == usize::MAX {
                    parent[next] = node;
                    order.push(next);
                }
            }
            i += 1;
        }
        for (node, p) in parent.iter_mut().enumerate() {
            if *p == usize::MAX {
                *p = node;
            }
        }
        SteinerTree {
            nodes: self.points,
            parent,
            terminals,
            root: driver,
        }
    }
}

/// For each point, the nearest point in each quadrant, found by sweeping
/// up in y, as `OpenROAD`'s `get_nearest_neighbors`.
fn nearest_neighbors(points: &[Point<Dbu>]) -> Vec<Vec<usize>> {
    let n = points.len();
    let mut neighbors = vec![Vec::new(); n];
    let mut upper_right = vec![i64::MAX; n];
    let mut lower_right = vec![i64::MAX; n];
    let mut upper_left = vec![i64::MIN; n];
    let mut lower_left = vec![i64::MIN; n];
    let mut sorted: Vec<usize> = (0..n).collect();
    sorted.sort_by_key(|&i| (points[i].y.get(), points[i].x.get()));
    for idx in 0..n {
        let pt = sorted[idx];
        let pt_x = points[pt].x.get();
        for &below in &sorted[..idx] {
            let below_x = points[below].x.get();
            if below_x <= pt_x && pt_x < upper_right[below] {
                neighbors[below].push(pt);
                upper_right[below] = pt_x;
            } else if upper_left[below] < pt_x && pt_x < below_x {
                neighbors[below].push(pt);
                upper_left[below] = pt_x;
            }
        }
        for &below in sorted[..idx].iter().rev() {
            let below_x = points[below].x.get();
            if pt_x <= below_x && below_x < lower_right[pt] {
                neighbors[pt].push(below);
                lower_right[pt] = below_x;
            } else if lower_left[pt] < below_x && below_x < pt_x {
                neighbors[pt].push(below);
                lower_left[pt] = below_x;
            }
        }
    }
    neighbors
}

/// A candidate edge in the Prim-Dijkstra search, ordered so the heap pops
/// the lowest weight first, then the lowest parent and node.
#[derive(Clone, Copy, Debug, PartialEq)]
struct SearchEdge {
    weight: f64,
    path_length: i64,
    parent: Option<usize>,
    node: usize,
}

impl Eq for SearchEdge {}

impl Ord for SearchEdge {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .weight
            .total_cmp(&self.weight)
            .then_with(|| other.parent.cmp(&self.parent))
            .then_with(|| other.node.cmp(&self.node))
    }
}

impl PartialOrd for SearchEdge {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Prim's algorithm with Dijkstra's path lengths mixed into the edge
/// weights by `alpha`, over the nearest-neighbor candidates.
fn build_spanning_tree(graph: &mut Graph, driver: usize, alpha: f64, neighbors: &[Vec<usize>]) {
    let n = graph.points.len();
    let mut visited = vec![false; n];
    let mut best: Vec<Option<SearchEdge>> = vec![None; n];
    let mut heap = BinaryHeap::new();
    let start = SearchEdge {
        weight: 0.0,
        path_length: 0,
        parent: None,
        node: driver,
    };
    best[driver] = Some(start);
    heap.push(start);
    let mut num_visited = 0;
    while let Some(edge) = heap.pop() {
        if visited[edge.node] || best[edge.node] != Some(edge) {
            continue;
        }
        visited[edge.node] = true;
        num_visited += 1;
        if let Some(parent) = edge.parent {
            graph.add_edge(parent, edge.node);
        }
        if num_visited == n {
            break;
        }
        for &neighbor in &neighbors[edge.node] {
            if visited[neighbor] {
                continue;
            }
            let edge_length = manhattan(graph.points[neighbor], graph.points[edge.node]);
            let candidate = SearchEdge {
                weight: alpha.mul_add(edge.path_length as f64, edge_length as f64),
                path_length: edge_length + edge.path_length,
                parent: Some(edge.node),
                node: neighbor,
            };
            let better = best[neighbor].is_none_or(|b| candidate.weight <= b.weight);
            if better {
                best[neighbor] = Some(candidate);
                heap.push(candidate);
            }
        }
    }
}

/// A Steiner point that would shorten two edges at a node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Candidate {
    gain: i64,
    node: usize,
    other1: usize,
    other2: usize,
    at: Point<Dbu>,
}

impl Ord for Candidate {
    fn cmp(&self, other: &Self) -> Ordering {
        (self.gain, self.at.x.get(), self.at.y.get()).cmp(&(
            other.gain,
            other.at.x.get(),
            other.at.y.get(),
        ))
    }
}

impl PartialOrd for Candidate {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// The best pair of edges at `node` to join through a Steiner point, with
/// the wire that saves; a gain of zero when none helps.
fn best_steiner_for_node(graph: &Graph, node: usize) -> Candidate {
    let pt = graph.points[node];
    let mut best = Candidate {
        gain: 0,
        node,
        other1: node,
        other2: node,
        at: pt,
    };
    let adjacent = &graph.adjacent[node];
    for i in 0..adjacent.len() {
        for j in (i + 1)..adjacent.len() {
            let p1 = graph.points[adjacent[i]];
            let p2 = graph.points[adjacent[j]];
            let mut steiner = pt;
            if p1.x.min(p2.x) > pt.x {
                steiner.x = p1.x.min(p2.x);
            } else if p1.x.max(p2.x) < pt.x {
                steiner.x = p1.x.max(p2.x);
            }
            if p1.y.min(p2.y) > pt.y {
                steiner.y = p1.y.min(p2.y);
            } else if p1.y.max(p2.y) < pt.y {
                steiner.y = p1.y.max(p2.y);
            }
            let gain = manhattan(steiner, pt);
            if gain > best.gain {
                best = Candidate {
                    gain,
                    node,
                    other1: adjacent[i],
                    other2: adjacent[j],
                    at: steiner,
                };
            }
        }
    }
    best
}

/// Adds Steiner points greedily by largest saving until none remains, as
/// `OpenROAD`'s `steinerize`.
fn steinerize(graph: &mut Graph) {
    let mut heap: BinaryHeap<Candidate> = BinaryHeap::new();
    let mut version = vec![0usize; graph.points.len()];
    let mut stamps: Vec<(Candidate, usize)> = Vec::new();
    let push = |graph: &Graph,
                node: usize,
                heap: &mut BinaryHeap<Candidate>,
                version: &mut Vec<usize>,
                stamps: &mut Vec<(Candidate, usize)>| {
        version[node] += 1;
        let candidate = best_steiner_for_node(graph, node);
        heap.push(candidate);
        stamps.push((candidate, version[node]));
    };
    for node in 0..graph.points.len() {
        push(graph, node, &mut heap, &mut version, &mut stamps);
    }
    while let Some(best) = heap.pop() {
        if best.gain == 0 {
            break;
        }
        let current = version[best.node];
        let fresh = stamps.iter().any(|(c, v)| *c == best && *v == current);
        if !fresh {
            continue;
        }
        let (node, opp1, opp2) = (best.node, best.other1, best.other2);
        if !graph.adjacent[node].contains(&opp1) || !graph.adjacent[node].contains(&opp2) {
            continue;
        }
        let (steiner, new_node) = if best.at == graph.points[opp1] {
            (opp1, false)
        } else if best.at == graph.points[opp2] {
            (opp2, false)
        } else {
            (graph.add_node(best.at), true)
        };
        if new_node {
            version.push(0);
        }
        if steiner != opp1 {
            graph.remove_edge(node, opp1);
            graph.add_edge(steiner, opp1);
        }
        if steiner != opp2 {
            graph.remove_edge(node, opp2);
            graph.add_edge(steiner, opp2);
        }
        if new_node {
            graph.add_edge(steiner, node);
            push(graph, steiner, &mut heap, &mut version, &mut stamps);
        }
        for touched in [node, opp1, opp2] {
            push(graph, touched, &mut heap, &mut version, &mut stamps);
        }
    }
}

/// Splits every node of degree above three into nodes joined by zero-length
/// edges, as `OpenROAD` does so routers may assume degree three.
fn split_high_degree_nodes(graph: &mut Graph) {
    let mut to_process: std::collections::VecDeque<usize> = (0..graph.points.len()).collect();
    while let Some(node) = to_process.pop_front() {
        if graph.adjacent[node].len() <= 3 {
            continue;
        }
        let at = graph.points[node];
        let new_node = graph.add_node(at);
        let moving: Vec<usize> = graph.adjacent[node].iter().skip(2).copied().collect();
        for other in moving {
            graph.remove_edge(node, other);
            graph.add_edge(new_node, other);
        }
        graph.add_edge(node, new_node);
        to_process.push_back(new_node);
    }
}

/// Whether no two branches of positive area cross without sharing a corner,
/// as `OpenROAD`'s `checkTree`; trees of over a hundred terminals pass
/// unchecked, as they will be buffered anyway.
#[must_use]
pub fn is_valid(tree: &SteinerTree) -> bool {
    if tree.terminals > 100 {
        return true;
    }
    let rects: Vec<(Point<Dbu>, Point<Dbu>)> = tree
        .parent
        .iter()
        .enumerate()
        .map(|(node, &parent)| {
            let (a, b) = (tree.nodes[node], tree.nodes[parent]);
            (
                Point::new(a.x.min(b.x), a.y.min(b.y)),
                Point::new(a.x.max(b.x), a.y.max(b.y)),
            )
        })
        .collect();
    let area_zero = |r: &(Point<Dbu>, Point<Dbu>)| r.0.x == r.1.x || r.0.y == r.1.y;
    let intersects = |r: &(Point<Dbu>, Point<Dbu>), s: &(Point<Dbu>, Point<Dbu>)| {
        r.0.x <= s.1.x && s.0.x <= r.1.x && r.0.y <= s.1.y && s.0.y <= r.1.y
    };
    let corners = |r: &(Point<Dbu>, Point<Dbu>)| {
        [r.0, r.1, Point::new(r.0.x, r.1.y), Point::new(r.1.x, r.0.y)]
    };
    for i in 0..rects.len() {
        for j in (i + 1)..rects.len() {
            let (r, s) = (&rects[i], &rects[j]);
            if area_zero(r) || area_zero(s) || !intersects(r, s) {
                continue;
            }
            let share_corner = corners(r).iter().any(|c| corners(s).contains(c));
            if !share_corner {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: i64, y: i64) -> Point<Dbu> {
        Point::new(Dbu(x), Dbu(y))
    }

    fn connected_to_root(tree: &SteinerTree) -> bool {
        (0..tree.nodes.len()).all(|mut n| {
            for _ in 0..tree.nodes.len() {
                if n == tree.root {
                    return true;
                }
                n = tree.parent[n];
            }
            false
        })
    }

    #[test]
    fn two_points_make_one_edge() {
        let tree = prim_dijkstra(&[pt(0, 0), pt(10, 5)], 0, DEFAULT_ALPHA);
        assert_eq!(tree.nodes.len(), 2);
        assert_eq!(tree.parent, vec![0, 0]);
        assert_eq!(tree.length(), 15);
    }

    #[test]
    fn an_l_of_three_points_gets_a_steiner_point() {
        let tree = prim_dijkstra(&[pt(0, 0), pt(10, 10), pt(10, -10)], 0, DEFAULT_ALPHA);
        assert_eq!(tree.terminals, 3);
        assert_eq!(tree.nodes.len(), 4);
        assert_eq!(tree.nodes[3], pt(10, 0));
        assert_eq!(tree.length(), 30);
        assert!(connected_to_root(&tree));
        assert!(is_valid(&tree));
    }

    #[test]
    fn a_star_splits_into_degree_three_nodes() {
        let points = [pt(0, 0), pt(5, 0), pt(-5, 0), pt(0, 5), pt(0, -5)];
        let tree = prim_dijkstra(&points, 0, DEFAULT_ALPHA);
        assert!(tree.children().iter().all(|c| c.len() <= 3));
        assert!(connected_to_root(&tree));
        assert_eq!(tree.length(), 20);
    }

    #[test]
    fn duplicate_points_stay_connected() {
        let points = [pt(3, 3), pt(3, 3), pt(9, 1), pt(9, 1), pt(0, 7)];
        let tree = prim_dijkstra(&points, 0, DEFAULT_ALPHA);
        assert!(connected_to_root(&tree));
        assert_eq!(tree.terminals, 5);
    }

    #[test]
    fn a_wider_spread_is_valid_and_shorter_than_a_star() {
        let points: Vec<Point<Dbu>> = (0..12).map(|i| pt((i * 37) % 101, (i * 53) % 89)).collect();
        let tree = prim_dijkstra(&points, 0, DEFAULT_ALPHA);
        assert!(connected_to_root(&tree));
        assert!(is_valid(&tree));
        let star: i64 = points[1..].iter().map(|&p| manhattan(p, points[0])).sum();
        assert!(tree.length() < star);
    }
}
