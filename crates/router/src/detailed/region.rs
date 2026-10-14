//! Routing again region by region.
//!
//! The die is cut into square regions a few gcells wide. In every region a
//! pass covers, the pieces of the chosen nets that lie wholly inside are
//! ripped up and routed again, negotiating with each other for a few local
//! rounds: every net in every region in the first pass, as `TritonRoute`'s
//! first detailed routing iteration does, and afterwards only the nets of
//! each conflict in the region holding it. Wiring outside the region stays
//! put, and where it crosses into the region it becomes a terminal the new
//! pieces must reach. Regions that do not touch are routed in parallel, and
//! the region grid shifts between passes so a conflict on one region's edge
//! falls inside another's.

use super::DetailedRouter;
use crate::access::Node;
use crate::occupancy::{LocalOccupancy, Owner};
use crate::search::{Costs, Search};
use crate::wiring::{Edge, NetRoute, path_edges};
use pare_common::db::indices::NetId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BTreeMap;

/// Edge of a region, in gcells.
const REGION_GCELLS: i64 = 6;

/// Local rip-up and reroute rounds within one region.
const LOCAL_ROUNDS: usize = 6;

/// One region's position in the region grid.
type RegionKey = (i64, i64);

/// The part of a net a region worker routes again.
#[derive(Debug)]
struct Piece {
    net: NetId,
    /// Groups of crossings that must be joined; each group is already
    /// connected outside the region or is one pin's access point.
    groups: Vec<Vec<Node>>,
}

/// Which nets a repair pass routes again.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scope {
    /// Every net in every region it passes through.
    Everything,
    /// The nets of each conflict, in the region holding it.
    Conflicts,
}

/// What a pass did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RepairStats {
    /// Regions repaired.
    pub regions: usize,
    /// Net pieces routed again.
    pub pieces: usize,
}

impl DetailedRouter<'_> {
    /// Routes again, region by region, the nets `scope` selects, with the
    /// region grid shifted by a third of a region per `pass`.
    pub fn repair(&mut self, costs: Costs, pass: usize, scope: Scope) -> RepairStats {
        let shift = self.gcell * (REGION_GCELLS * (pass as i64 % 3) / 3);
        let size = self.gcell * REGION_GCELLS;
        let origin = self.grid_origin - Point::new(shift, shift);
        let key = |p: Point<Dbu>| {
            (
                (p.x - origin.x).get().div_euclid(size.get()),
                (p.y - origin.y).get().div_euclid(size.get()),
            )
        };
        let mut regions: BTreeMap<RegionKey, FxHashSet<NetId>> = BTreeMap::new();
        match scope {
            Scope::Conflicts => {
                for m in self.markers() {
                    let nets = regions.entry(key(m.at)).or_default();
                    nets.extend(m.nets.iter().flatten());
                }
            }
            Scope::Everything => {
                for (n, route) in self.routes.iter().enumerate() {
                    let nodes = route
                        .edges
                        .iter()
                        .flat_map(|e| <[Node; 2]>::from(e.ends()))
                        .chain(route.access.iter().map(|a| a.node));
                    for node in nodes {
                        let _ = regions
                            .entry(key(self.point(node)))
                            .or_default()
                            .insert(NetId::new(n));
                    }
                }
            }
        }
        let mut stats = RepairStats {
            regions: regions.len(),
            pieces: 0,
        };
        for colour in 0..4 {
            let batch: Vec<(Rect<Dbu>, Vec<NetId>)> = regions
                .iter()
                .filter(|((rx, ry), _)| rx.rem_euclid(2) + 2 * ry.rem_euclid(2) == colour)
                .map(|(&(rx, ry), nets)| {
                    let min = origin + Point::new(size * rx, size * ry);
                    let area = Rect::new(min, min + Point::new(size, size));
                    let mut nets: Vec<NetId> = nets.iter().copied().collect();
                    nets.sort_unstable();
                    (area, nets)
                })
                .collect();
            stats.pieces += self.repair_batch(&batch, costs);
        }
        stats
    }

    /// Repairs regions that do not touch each other; returns the number of
    /// pieces routed again.
    fn repair_batch(&mut self, batch: &[(Rect<Dbu>, Vec<NetId>)], costs: Costs) -> usize {
        let mut work: Vec<(Rect<Dbu>, Vec<Piece>)> = Vec::with_capacity(batch.len());
        let mut removed: FxHashMap<(usize, NetId), Vec<Edge>> = FxHashMap::default();
        let mut touched: Vec<NetId> = Vec::new();
        for (r, (area, nets)) in batch.iter().enumerate() {
            let mut pieces = Vec::new();
            for &net in nets {
                if !self.routes[net.index()].complete {
                    continue;
                }
                let (inside, piece) = self.carve(net, area);
                let _ = removed.insert((r, net), inside);
                touched.push(net);
                if let Some(piece) = piece {
                    pieces.push(piece);
                }
            }
            work.push((*area, pieces));
        }
        touched.sort_unstable();
        touched.dedup();
        for &net in &touched {
            let mut route = self.routes[net.index()].clone();
            let gone: FxHashSet<Edge> = removed
                .iter()
                .filter(|((_, n), _)| *n == net)
                .flat_map(|(_, edges)| edges.iter().copied())
                .collect();
            route.edges.retain(|e| !gone.contains(e));
            self.commit(net, &route);
        }

        let routed: Vec<Vec<Option<Vec<Edge>>>> = work
            .par_iter()
            .map(|(area, pieces)| self.route_region(area, pieces, costs))
            .collect();

        let mut count = 0;
        let mut additions: FxHashMap<NetId, Vec<Edge>> = FxHashMap::default();
        for (r, ((_, pieces), results)) in work.iter().zip(routed).enumerate() {
            for (piece, result) in pieces.iter().zip(results) {
                count += 1;
                let edges = result
                    .unwrap_or_else(|| removed.get(&(r, piece.net)).cloned().unwrap_or_default());
                additions.entry(piece.net).or_default().extend(edges);
            }
        }
        for (net, edges) in additions {
            let mut route = self.routes[net.index()].clone();
            route.edges.extend(edges);
            route.edges.sort_unstable();
            route.edges.dedup();
            self.commit(net, &route);
        }
        count
    }

    /// Splits `net`'s edges inside `area` off its route. Returns them, and
    /// the piece to route again if the rest of the net reaches `area` in
    /// more than one place.
    fn carve(&self, net: NetId, area: &Rect<Dbu>) -> (Vec<Edge>, Option<Piece>) {
        let route = &self.routes[net.index()];
        let inside = |n: Node| contains_half_open(area, self.point(n));
        let (removed, kept): (Vec<Edge>, Vec<Edge>) = route.edges.iter().partition(|e| {
            let (a, b) = e.ends();
            inside(a) && inside(b)
        });

        let mut sets = DisjointSets::default();
        for e in &kept {
            let (a, b) = e.ends();
            sets.union(a, b);
        }
        let mut terminals: Vec<Node> = kept
            .iter()
            .flat_map(|e| <[Node; 2]>::from(e.ends()))
            .filter(|&n| inside(n))
            .collect();
        terminals.extend(route.access.iter().map(|a| a.node).filter(|&n| inside(n)));
        terminals.sort_unstable();
        terminals.dedup();

        let mut groups: BTreeMap<Node, Vec<Node>> = BTreeMap::new();
        for n in terminals {
            groups.entry(sets.find(n)).or_default().push(n);
        }
        let groups: Vec<Vec<Node>> = groups.into_values().collect();
        let piece = (groups.len() > 1).then_some(Piece { net, groups });
        (removed, piece)
    }

    /// Routes a region's pieces, rerouting those still in conflict with a
    /// growing penalty. `None` for a piece that could not be connected.
    fn route_region(
        &self,
        area: &Rect<Dbu>,
        pieces: &[Piece],
        costs: Costs,
    ) -> Vec<Option<Vec<Edge>>> {
        let window = Rect::new(area.min, area.max - Point::new(Dbu(1), Dbu(1)));
        let mut results: Vec<Option<Vec<Edge>>> = vec![None; pieces.len()];
        let mut overlay = LocalOccupancy::default();
        let mut todo: Vec<usize> = (0..pieces.len()).collect();
        let mut penalty = costs.penalty;
        for _ in 0..LOCAL_ROUNDS {
            for &i in &todo {
                let owner = Owner::Net(pieces[i].net.0);
                overlay.remove_owner(owner);
                let search = Search {
                    db: self.db,
                    tech: self.tech,
                    occupancy: &self.occupancy,
                    overlay: Some(&overlay),
                    history: &self.history,
                    owner,
                    costs: Costs { penalty, ..costs },
                    guide: None,
                };
                results[i] = connect_groups(&search, &pieces[i].groups, &window);
                if let Some(edges) = &results[i] {
                    for (layer, rect) in self.edge_metal(edges) {
                        overlay.add_rect(self.tech, layer, &rect, owner);
                    }
                }
            }
            todo = (0..pieces.len())
                .filter(|&i| {
                    results[i]
                        .as_ref()
                        .is_some_and(|edges| self.clashes(pieces[i].net, edges, &overlay))
                })
                .collect();
            if todo.is_empty() {
                break;
            }
            penalty *= 2.0;
        }
        results
    }

    /// Whether the metal of `edges` of `net` conflicts with anything.
    fn clashes(&self, net: NetId, edges: &[Edge], overlay: &LocalOccupancy) -> bool {
        let owner = Owner::Net(net.0);
        self.edge_metal(edges).iter().any(|(layer, rect)| {
            self.occupancy
                .rect_conflicts(self.tech, *layer, rect, owner)
                .next()
                .is_some()
                || overlay
                    .rect_conflicts(self.tech, *layer, rect, owner)
                    .next()
                    .is_some()
        })
    }

    /// The metal of some edges on routable layers.
    fn edge_metal(&self, edges: &[Edge]) -> Vec<(pare_common::db::indices::LayerId, Rect<Dbu>)> {
        let route = NetRoute {
            edges: edges.to_vec(),
            ..NetRoute::default()
        };
        route
            .metal(self.db, self.tech)
            .into_iter()
            .filter(|(layer, _)| self.tech.routable(*layer))
            .collect()
    }
}

/// Joins `groups` into one tree inside `window`: each search runs from the
/// tree so far to the nearest remaining group.
fn connect_groups(
    search: &Search<'_>,
    groups: &[Vec<Node>],
    window: &Rect<Dbu>,
) -> Option<Vec<Edge>> {
    let (first, rest) = groups.split_first()?;
    let mut tree: Vec<Node> = first.clone();
    let mut remaining: Vec<&Vec<Node>> = rest.iter().collect();
    let mut edges = Vec::new();
    while !remaining.is_empty() {
        let next = nearest_group(search, &tree, &remaining);
        let group = remaining.swap_remove(next);
        let sources: Vec<(Node, f64)> = tree.iter().map(|&n| (n, 0.0)).collect();
        let targets: FxHashMap<Node, (usize, f64)> = group.iter().map(|&n| (n, (0, 0.0))).collect();
        let (path, _) = search.find(&sources, &targets, window)?;
        edges.extend(path_edges(&path));
        tree.extend(path);
        tree.extend(group.iter().copied());
        tree.sort_unstable();
        tree.dedup();
    }
    edges.sort_unstable();
    edges.dedup();
    Some(edges)
}

/// Index of the group in `groups` closest to any node of `tree`.
fn nearest_group(search: &Search<'_>, tree: &[Node], groups: &[&Vec<Node>]) -> usize {
    let point = |n: Node| search.tech.layer(n.layer).point(n.track, n.cross);
    let distance = |g: &Vec<Node>| {
        g.iter()
            .flat_map(|&a| {
                tree.iter().map(move |&b| {
                    let (p, q) = (point(a), point(b));
                    ((p.x - q.x).abs() + (p.y - q.y).abs()).get()
                })
            })
            .min()
            .unwrap_or(i64::MAX)
    };
    (0..groups.len())
        .min_by_key(|&i| (distance(groups[i]), i))
        .unwrap_or(0)
}

/// Whether `p` is in `r`, counting its lower and left edges but not its
/// upper and right ones, so that adjacent regions do not share a point.
fn contains_half_open(r: &Rect<Dbu>, p: Point<Dbu>) -> bool {
    r.min.x <= p.x && p.x < r.max.x && r.min.y <= p.y && p.y < r.max.y
}

/// Union-find over crossings.
#[derive(Debug, Default)]
struct DisjointSets {
    parent: FxHashMap<Node, Node>,
}

impl DisjointSets {
    fn find(&mut self, n: Node) -> Node {
        let mut root = n;
        while let Some(&p) = self.parent.get(&root) {
            if p == root {
                break;
            }
            root = p;
        }
        let mut cur = n;
        while cur != root {
            let next = self.parent.get(&cur).copied().unwrap_or(root);
            let _ = self.parent.insert(cur, root);
            cur = next;
        }
        root
    }

    fn union(&mut self, a: Node, b: Node) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            let _ = self.parent.insert(ra.max(rb), ra.min(rb));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::indices::LayerId;

    fn node(cross: u32) -> Node {
        Node {
            layer: LayerId(1),
            track: 0,
            cross,
        }
    }

    #[test]
    fn joined_crossings_share_a_root() {
        let mut sets = DisjointSets::default();

        sets.union(node(1), node(2));
        sets.union(node(3), node(2));

        assert_eq!(sets.find(node(3)), sets.find(node(1)));
        assert_ne!(sets.find(node(4)), sets.find(node(1)));
    }

    #[test]
    fn a_region_holds_its_lower_left_edges_but_not_its_upper_right_ones() {
        let r = Rect::new(Point::new(Dbu(0), Dbu(0)), Point::new(Dbu(10), Dbu(10)));

        assert!(contains_half_open(&r, Point::new(Dbu(0), Dbu(9))));
        assert!(!contains_half_open(&r, Point::new(Dbu(10), Dbu(5))));
    }
}
