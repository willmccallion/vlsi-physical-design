//! Routing again region by region.
//!
//! The die is cut into square regions a few gcells wide. In every region a
//! pass covers, the pieces of the chosen nets that lie wholly inside are
//! ripped up and routed again, negotiating with each other for a few local
//! rounds: the nets of each conflict in the region holding it, and when
//! that stalls the nets near it too. Wiring outside the region stays put,
//! and where it crosses into the region it becomes a terminal the new
//! pieces must reach. Regions that do not touch are routed in parallel, and
//! the region grid shifts between passes so a conflict on one region's edge
//! falls inside another's.

use super::DetailedRouter;
use crate::access::Node;
use crate::occupancy::{LocalOccupancy, Owner};
use crate::search::{Costs, Guide, MAX_PENALTY, OwnVias, Search};
use crate::wiring::{DisjointSets, Edge, NetRoute, path_edges};
use pare_common::db::indices::{CutLayerId, LayerId, NetId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::BTreeMap;

/// Edge of a region, in gcells.
pub(super) const REGION_GCELLS: i64 = 6;

/// Local rip-up and reroute rounds within one region.
const LOCAL_ROUNDS: usize = 6;

/// One region's position in the region grid.
pub(super) type RegionKey = (i64, i64);

/// Square regions of [`REGION_GCELLS`] gcells from an origin.
#[derive(Clone, Copy, Debug)]
pub(super) struct RegionGrid {
    origin: Point<Dbu>,
    size: Dbu,
}

impl RegionGrid {
    /// The grid of one pass: aligned with the gcells, shifted by a third of
    /// a region per `pass`.
    pub(super) fn of_pass(router: &DetailedRouter<'_>, pass: usize) -> Self {
        let shift = router.gcell * (REGION_GCELLS * (pass as i64 % 3) / 3);
        Self {
            origin: router.grid_origin - Point::new(shift, shift),
            size: router.gcell * REGION_GCELLS,
        }
    }

    /// The region holding `p`, counting regions' lower and left edges as
    /// theirs.
    pub(super) const fn key(&self, p: Point<Dbu>) -> RegionKey {
        (
            (p.x.get() - self.origin.x.get()).div_euclid(self.size.get()),
            (p.y.get() - self.origin.y.get()).div_euclid(self.size.get()),
        )
    }

    /// The area of a region.
    pub(super) fn area(&self, (rx, ry): RegionKey) -> Rect<Dbu> {
        let min = self.origin + Point::new(self.size * rx, self.size * ry);
        Rect::new(min, min + Point::new(self.size, self.size))
    }
}

/// Which of four batches a region is routed in; regions of one batch do
/// not touch.
pub(super) const fn colour((rx, ry): RegionKey) -> i64 {
    rx.rem_euclid(2) + 2 * ry.rem_euclid(2)
}

/// The part of a net a region worker routes again.
#[derive(Debug)]
pub(super) struct Piece {
    pub(super) net: NetId,
    /// Groups of crossings that must be joined; each group is already
    /// connected outside the region or is one pin's access point.
    pub(super) groups: Vec<Vec<Node>>,
    /// The cells the piece should keep to, if any; it may leave them when
    /// they hold no way through.
    pub(super) guide: Option<Guide>,
}

/// Which nets a repair pass routes again.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Scope {
    /// The nets of each conflict, in the region holding it.
    Conflicts,
    /// The nets of each conflict and every net with wiring within
    /// [`NEAR_TRACKS`] tracks of it, in the region holding it, so that
    /// wiring hemming a conflict in can move aside, as `TritonRoute`'s
    /// near-violation rip-up does.
    NearConflicts,
}

/// Tracks of the lowest routing layer around a conflict within which
/// [`Scope::NearConflicts`] rips nets up.
const NEAR_TRACKS: i64 = 3;

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
        let grid = RegionGrid::of_pass(self, pass);
        let key = |p: Point<Dbu>| grid.key(p);
        let mut regions: BTreeMap<RegionKey, FxHashSet<NetId>> = BTreeMap::new();
        match scope {
            Scope::Conflicts => {
                for m in self.markers() {
                    let nets = regions.entry(key(m.at)).or_default();
                    nets.extend(m.nets.iter().flatten());
                }
            }
            Scope::NearConflicts => {
                let reach = self.tech.layer(self.tech.min_layer).pitch() * NEAR_TRACKS;
                for m in self.markers() {
                    let near = self.nets_near(m.at, reach);
                    let nets = regions.entry(key(m.at)).or_default();
                    nets.extend(m.nets.iter().flatten());
                    nets.extend(near);
                }
            }
        }
        let mut stats = RepairStats {
            regions: regions.len(),
            pieces: 0,
        };
        for batch_colour in 0..4 {
            let batch: Vec<(Rect<Dbu>, Vec<NetId>)> = regions
                .iter()
                .filter(|&(&k, _)| colour(k) == batch_colour)
                .map(|(&k, nets)| {
                    let mut nets: Vec<NetId> = nets.iter().copied().collect();
                    nets.sort_unstable();
                    (grid.area(k), nets)
                })
                .collect();
            stats.pieces += self.repair_batch(&batch, costs);
        }
        stats
    }

    /// Repairs regions that do not touch each other; returns the number of
    /// pieces routed again.
    fn repair_batch(&mut self, batch: &[(Rect<Dbu>, Vec<NetId>)], costs: Costs) -> usize {
        let mut regions_of: BTreeMap<NetId, Vec<usize>> = BTreeMap::new();
        for (r, (_, nets)) in batch.iter().enumerate() {
            for &net in nets {
                if self.routes[net.index()].complete {
                    regions_of.entry(net).or_default().push(r);
                }
            }
        }
        let mut pieces: Vec<Vec<Piece>> = (0..batch.len()).map(|_| Vec::new()).collect();
        let mut removed: FxHashMap<(usize, NetId), Vec<Edge>> = FxHashMap::default();
        for (&net, regions) in &regions_of {
            let areas: Vec<Rect<Dbu>> = regions.iter().map(|&r| batch[r].0).collect();
            let mut gone: FxHashSet<Edge> = FxHashSet::default();
            for (k, (inside, piece)) in self.carve(net, &areas).into_iter().enumerate() {
                gone.extend(inside.iter().copied());
                let _ = removed.insert((regions[k], net), inside);
                if let Some(piece) = piece {
                    pieces[regions[k]].push(piece);
                }
            }
            let mut route = self.routes[net.index()].clone();
            route.edges.retain(|e| !gone.contains(e));
            self.commit(net, &route);
        }
        let work: Vec<(Rect<Dbu>, Vec<Piece>)> =
            batch.iter().map(|(area, _)| *area).zip(pieces).collect();

        let routed: Vec<Vec<Option<Vec<Edge>>>> = work
            .par_iter()
            .enumerate()
            .map(|(r, (area, pieces))| {
                let old: Vec<&[Edge]> = pieces
                    .iter()
                    .map(|p| removed.get(&(r, p.net)).map_or(&[][..], Vec::as_slice))
                    .collect();
                let results = self.route_region(area, pieces, costs);
                let new: Vec<&[Edge]> = results
                    .iter()
                    .zip(&old)
                    .map(|(r, &o)| r.as_deref().unwrap_or(o))
                    .collect();
                if self.clashing(area, pieces, &new) > self.clashing(area, pieces, &old) {
                    return vec![None; pieces.len()];
                }
                results
            })
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

    /// Splits `net`'s edges inside each of `areas`, which do not touch, off
    /// its route. Returns, per area, the edges taken out and the piece to
    /// route again if the wiring left outside every area reaches that area
    /// in more than one place. Judging connectivity by what is left outside
    /// all areas at once means that joining each area's groups restores every
    /// connection the removed edges made.
    fn carve(&self, net: NetId, areas: &[Rect<Dbu>]) -> Vec<(Vec<Edge>, Option<Piece>)> {
        let route = &self.routes[net.index()];
        let area_of = |n: Node| {
            let p = self.point(n);
            areas.iter().position(|a| contains_half_open(a, p))
        };
        let mut removed: Vec<Vec<Edge>> = vec![Vec::new(); areas.len()];
        let mut kept: Vec<Edge> = Vec::new();
        for &e in &route.edges {
            let (a, b) = e.ends();
            match (area_of(a), area_of(b)) {
                (Some(i), Some(j)) if i == j => removed[i].push(e),
                _ => kept.push(e),
            }
        }

        let mut sets = DisjointSets::default();
        for e in &kept {
            let (a, b) = e.ends();
            sets.union(a, b);
        }
        let mut terminals: Vec<Vec<Node>> = vec![Vec::new(); areas.len()];
        let ends = kept.iter().flat_map(|e| <[Node; 2]>::from(e.ends()));
        for n in ends.chain(route.access.iter().map(|a| a.node)) {
            if let Some(i) = area_of(n) {
                terminals[i].push(n);
            }
        }

        removed
            .into_iter()
            .zip(terminals)
            .map(|(inside, mut nodes)| {
                nodes.sort_unstable();
                nodes.dedup();
                let mut groups: BTreeMap<Node, Vec<Node>> = BTreeMap::new();
                for n in nodes {
                    groups.entry(sets.find(n)).or_default().push(n);
                }
                let groups: Vec<Vec<Node>> = groups.into_values().collect();
                let piece = (groups.len() > 1).then_some(Piece {
                    net,
                    groups,
                    guide: None,
                });
                (inside, piece)
            })
            .collect()
    }

    /// Routes a region's pieces, rerouting those still in conflict with a
    /// growing penalty; `None` for a piece that could not be connected.
    pub(super) fn route_region(
        &self,
        area: &Rect<Dbu>,
        pieces: &[Piece],
        costs: Costs,
    ) -> Vec<Option<Vec<Edge>>> {
        let window = Rect::new(area.min, area.max - Point::new(Dbu(1), Dbu(1)));
        let mut results: Vec<Option<Vec<Edge>>> = vec![None; pieces.len()];
        let mut overlay = LocalOccupancy::default();
        let mut order: Vec<usize> = (0..pieces.len()).collect();
        order.sort_by_cached_key(|&i| (self.extent(&pieces[i]), pieces[i].net));
        let mut todo = order.clone();
        let mut penalty = costs.penalty;
        for _ in 0..LOCAL_ROUNDS {
            for &i in &todo {
                let owner = Owner::Net(pieces[i].net.0);
                overlay.remove_owner(owner);
                let guided = Search {
                    db: self.db,
                    tech: self.tech,
                    occupancy: &self.occupancy,
                    overlay: Some(&overlay),
                    history: &self.history,
                    owner,
                    costs: Costs { penalty, ..costs },
                    guide: pieces[i].guide.as_ref(),
                    own_vias: None,
                };
                let free = Search {
                    guide: None,
                    ..guided
                };
                let own = self.vias_near(pieces[i].net, area);
                let groups = &pieces[i].groups;
                results[i] = if guided.guide.is_some() {
                    self.connect_groups(&guided, groups, &window, own.clone())
                        .or_else(|| self.connect_groups(&free, groups, &window, own))
                } else {
                    self.connect_groups(&free, groups, &window, own)
                };
                if let Some(edges) = &results[i] {
                    for (layer, rect) in self.edge_metal(edges) {
                        overlay.add_rect(self.tech, layer, &rect, owner);
                    }
                    for (cut, rect) in self.edge_cuts(edges) {
                        overlay.add_cut(self.tech, cut, &rect, owner);
                    }
                }
            }
            todo = order
                .iter()
                .copied()
                .filter(|&i| {
                    results[i].as_ref().is_some_and(|edges| {
                        self.clashes(pieces[i].net, edges, &overlay)
                            || self.crowds_own_vias(pieces[i].net, area, edges)
                    })
                })
                .collect();
            if todo.is_empty() {
                break;
            }
            penalty = (penalty * 2.0).min(MAX_PENALTY);
        }
        results
    }

    /// Half-perimeter of the box around a piece's nodes. Pieces are routed
    /// smallest first: a short piece tied to its pins has few ways through,
    /// while one passing through the region can shift to another track.
    fn extent(&self, piece: &Piece) -> Dbu {
        let mut points = piece.groups.iter().flatten().map(|&n| self.point(n));
        let Some(first) = points.next() else {
            return Dbu::ZERO;
        };
        let (min, max) = points.fold((first, first), |(lo, hi), p| {
            (
                Point::new(lo.x.min(p.x), lo.y.min(p.y)),
                Point::new(hi.x.max(p.x), hi.y.max(p.y)),
            )
        });
        (max.x - min.x) + (max.y - min.y)
    }

    /// How many of `pieces` clash with anything when wired as `wiring`.
    fn clashing(&self, area: &Rect<Dbu>, pieces: &[Piece], wiring: &[&[Edge]]) -> usize {
        let mut overlay = LocalOccupancy::default();
        for (piece, edges) in pieces.iter().zip(wiring) {
            let owner = Owner::Net(piece.net.0);
            for (layer, rect) in self.edge_metal(edges) {
                overlay.add_rect(self.tech, layer, &rect, owner);
            }
            for (cut, rect) in self.edge_cuts(edges) {
                overlay.add_cut(self.tech, cut, &rect, owner);
            }
        }
        pieces
            .iter()
            .zip(wiring)
            .filter(|(piece, edges)| {
                self.clashes(piece.net, edges, &overlay)
                    || self.crowds_own_vias(piece.net, area, edges)
            })
            .count()
    }

    /// Joins `groups` into one tree inside `window`: each search runs from
    /// the tree so far to the nearest remaining group, keeping new vias
    /// clear of `own`, the net's vias so far.
    fn connect_groups(
        &self,
        search: &Search<'_>,
        groups: &[Vec<Node>],
        window: &Rect<Dbu>,
        mut own: OwnVias,
    ) -> Option<Vec<Edge>> {
        let (first, rest) = groups.split_first()?;
        let mut tree: Vec<Node> = first.clone();
        let mut remaining: Vec<&Vec<Node>> = rest.iter().collect();
        let mut edges = Vec::new();
        while !remaining.is_empty() {
            let next = nearest_group(search, &tree, &remaining);
            let group = remaining.swap_remove(next);
            let sources: Vec<(Node, f64)> = tree.iter().map(|&n| (n, 0.0)).collect();
            let targets: FxHashMap<Node, (usize, f64)> =
                group.iter().map(|&n| (n, (0, 0.0))).collect();
            let joining = Search {
                own_vias: Some(&own),
                ..*search
            };
            let (path, _) = joining.find(&sources, &targets, window)?;
            for (via, at) in self.path_vias(&path) {
                own.add(via, at);
            }
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

    /// The vias `net` has near `area`, which wiring routed there must not
    /// crowd.
    fn vias_near(&self, net: NetId, area: &Rect<Dbu>) -> OwnVias {
        let near = area.expand(self.gcell);
        let mut own = OwnVias::new(self.db);
        for &(via, at) in &self.vias[net.index()] {
            if near.contains(at) {
                own.add(via, at);
            }
        }
        own
    }

    /// Whether the vias of `edges` crowd each other or the vias `net` has
    /// near `area`.
    fn crowds_own_vias(&self, net: NetId, area: &Rect<Dbu>, edges: &[Edge]) -> bool {
        let mut own = self.vias_near(net, area);
        let route = NetRoute {
            edges: edges.to_vec(),
            ..NetRoute::default()
        };
        route.vias(self.tech).into_iter().any(|(via, at)| {
            let crowded = own.crowded(self.db, via, at) > 0;
            own.add(via, at);
            crowded
        })
    }

    /// Nets with wiring within `reach` of `at` on any routable layer.
    fn nets_near(&self, at: Point<Dbu>, reach: Dbu) -> Vec<NetId> {
        let probe = Rect::new(at, at).expand(reach);
        let mut nets: Vec<NetId> = (self.tech.min_layer.index()..=self.tech.max_layer.index())
            .flat_map(|l| {
                self.occupancy
                    .wiring_owners_in(self.tech, LayerId::new(l), &probe)
            })
            .filter_map(|o| match o {
                Owner::Net(n) => Some(NetId(n)),
                Owner::Obstacle => None,
            })
            .collect();
        nets.sort_unstable();
        nets.dedup();
        nets
    }

    /// Whether the metal or cuts of `edges` of `net` conflict with anything.
    fn clashes(&self, net: NetId, edges: &[Edge], overlay: &LocalOccupancy) -> bool {
        let owner = Owner::Net(net.0);
        let metal = self.edge_metal(edges).iter().any(|(layer, rect)| {
            self.occupancy
                .rect_conflicts(self.tech, *layer, rect, owner)
                .next()
                .is_some()
                || overlay
                    .rect_conflicts(self.tech, *layer, rect, owner)
                    .next()
                    .is_some()
        });
        metal
            || self.edge_cuts(edges).iter().any(|(cut, rect)| {
                self.occupancy
                    .cut_conflicts(*cut, rect, owner)
                    .next()
                    .is_some()
                    || overlay.cut_conflicts(*cut, rect, owner).next().is_some()
            })
    }

    /// The cuts of some edges.
    fn edge_cuts(&self, edges: &[Edge]) -> Vec<(CutLayerId, Rect<Dbu>)> {
        NetRoute {
            edges: edges.to_vec(),
            ..NetRoute::default()
        }
        .cuts(self.db, self.tech)
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
pub(super) fn contains_half_open(r: &Rect<Dbu>, p: Point<Dbu>) -> bool {
    r.min.x <= p.x && p.x < r.max.x && r.min.y <= p.y && p.y < r.max.y
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_region_holds_its_lower_left_edges_but_not_its_upper_right_ones() {
        let r = Rect::new(Point::new(Dbu(0), Dbu(0)), Point::new(Dbu(10), Dbu(10)));

        assert!(contains_half_open(&r, Point::new(Dbu(0), Dbu(9))));
        assert!(!contains_half_open(&r, Point::new(Dbu(10), Dbu(5))));
    }
}
