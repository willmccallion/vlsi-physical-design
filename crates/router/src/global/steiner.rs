//! Rectilinear Steiner trees over a net's pin gcells.
//!
//! A minimum spanning tree over the pins (exact for small nets, joining each
//! pin to its nearest earlier one along a space-filling curve for large ones)
//! is shortened by three-point Steiner points: wherever a node and two of
//! its neighbours can be joined through the median of their positions with
//! less wire than the two edges they replace, the median becomes a node.

use crate::tree::connection_order;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// A position in the plane of the gcell grid, as (column, row).
pub(super) type Spot = (u16, u16);

/// Passes over all nodes looking for Steiner points.
const MAX_PASSES: usize = 16;

/// A tree over gcell positions, rooted at node 0.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct SteinerTree {
    /// Each node's position: the given spots first, then Steiner points.
    pub(super) nodes: Vec<Spot>,
    /// Each node's parent; the root's parent is itself.
    pub(super) parent: Vec<usize>,
    /// Every node, each after its parent.
    pub(super) order: Vec<usize>,
}

const fn distance(a: Spot, b: Spot) -> u32 {
    (a.0.abs_diff(b.0) as u32) + (a.1.abs_diff(b.1) as u32)
}

fn median(a: u16, b: u16, c: u16) -> u16 {
    a.max(b).min(a.min(b).max(c))
}

/// Wire needed to join three spots through their median.
fn star_length(a: Spot, b: Spot, c: Spot) -> u32 {
    let span = |p: u16, q: u16, r: u16| u32::from(p.max(q).max(r) - p.min(q).min(r));
    span(a.0, b.0, c.0) + span(a.1, b.1, c.1)
}

/// A Steiner tree through `spots`, which must be distinct.
pub(super) fn steiner_tree(spots: &[Spot]) -> SteinerTree {
    let mut nodes: Vec<Spot> = spots.to_vec();
    let mut adjacent: Vec<Vec<usize>> = vec![Vec::new(); nodes.len()];
    if nodes.len() > 1 {
        let boxes: Vec<Rect<Dbu>> = nodes
            .iter()
            .map(|&(x, y)| {
                let p = Point::new(Dbu(i64::from(x)), Dbu(i64::from(y)));
                Rect::new(p, p)
            })
            .collect();
        let (_, joins) = connection_order(&boxes);
        for join in joins {
            adjacent[join.pin].push(join.parent);
            adjacent[join.parent].push(join.pin);
        }
    }
    add_steiner_points(&mut nodes, &mut adjacent);
    root_at_first(nodes, &adjacent)
}

/// Inserts median points while they shorten the tree.
fn add_steiner_points(nodes: &mut Vec<Spot>, adjacent: &mut Vec<Vec<usize>>) {
    for _ in 0..MAX_PASSES {
        let mut improved = false;
        for v in 0..nodes.len() {
            let Some((a, b)) = best_pair(nodes, &adjacent[v], v) else {
                continue;
            };
            improved = true;
            let s = (
                median(nodes[v].0, nodes[a].0, nodes[b].0),
                median(nodes[v].1, nodes[a].1, nodes[b].1),
            );
            unlink(adjacent, v, a);
            unlink(adjacent, v, b);
            if s == nodes[a] {
                link(adjacent, v, a);
                link(adjacent, a, b);
            } else if s == nodes[b] {
                link(adjacent, v, b);
                link(adjacent, b, a);
            } else {
                let id = nodes.len();
                nodes.push(s);
                adjacent.push(Vec::new());
                link(adjacent, id, v);
                link(adjacent, id, a);
                link(adjacent, id, b);
            }
        }
        if !improved {
            break;
        }
    }
}

/// The two neighbours of `v` whose median with it saves the most wire, if
/// any saves some.
fn best_pair(nodes: &[Spot], neighbours: &[usize], v: usize) -> Option<(usize, usize)> {
    let mut best: Option<(u32, usize, usize)> = None;
    for (i, &a) in neighbours.iter().enumerate() {
        for &b in &neighbours[i + 1..] {
            let before = distance(nodes[v], nodes[a]) + distance(nodes[v], nodes[b]);
            let after = star_length(nodes[v], nodes[a], nodes[b]);
            let gain = before - after;
            if gain > 0 && best.is_none_or(|(g, _, _)| gain > g) {
                best = Some((gain, a, b));
            }
        }
    }
    best.map(|(_, a, b)| (a, b))
}

fn link(adjacent: &mut [Vec<usize>], a: usize, b: usize) {
    adjacent[a].push(b);
    adjacent[b].push(a);
}

fn unlink(adjacent: &mut [Vec<usize>], a: usize, b: usize) {
    adjacent[a].retain(|&x| x != b);
    adjacent[b].retain(|&x| x != a);
}

fn root_at_first(nodes: Vec<Spot>, adjacent: &[Vec<usize>]) -> SteinerTree {
    let mut parent: Vec<usize> = (0..nodes.len()).collect();
    let mut seen = vec![false; nodes.len()];
    let mut order = Vec::with_capacity(nodes.len());
    if !nodes.is_empty() {
        seen[0] = true;
        order.push(0);
        let mut next = 0;
        while next < order.len() {
            let v = order[next];
            next += 1;
            for &w in &adjacent[v] {
                if !seen[w] {
                    seen[w] = true;
                    parent[w] = v;
                    order.push(w);
                }
            }
        }
    }
    SteinerTree {
        nodes,
        parent,
        order,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn length(tree: &SteinerTree) -> u32 {
        tree.order
            .iter()
            .map(|&v| distance(tree.nodes[v], tree.nodes[tree.parent[v]]))
            .sum()
    }

    #[test]
    fn three_pins_meet_at_their_median() {
        let tree = steiner_tree(&[(0, 0), (10, 5), (4, 10)]);

        assert_eq!(length(&tree), 20);
        assert!(tree.nodes.contains(&(4, 5)));
    }

    #[test]
    fn a_cross_of_four_pins_shares_its_centre() {
        let tree = steiner_tree(&[(5, 0), (5, 10), (0, 5), (10, 5)]);

        assert_eq!(length(&tree), 20);
        assert_eq!(tree.order.len(), tree.nodes.len());
    }

    #[test]
    fn every_node_is_reached_after_its_parent() {
        let spots: Vec<Spot> = (0..60u16).map(|i| (i * 7 % 23, i * 11 % 31)).collect();
        let mut unique = spots;
        unique.sort_unstable();
        unique.dedup();

        let tree = steiner_tree(&unique);

        let mut placed = vec![false; tree.nodes.len()];
        for &v in &tree.order {
            assert!(v == 0 || placed[tree.parent[v]], "{v} before its parent");
            placed[v] = true;
        }
        assert!(placed.iter().all(|&p| p));
    }
}
