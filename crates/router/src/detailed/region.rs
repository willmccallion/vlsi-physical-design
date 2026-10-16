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
use super::judge::{Candidate, Judgment};
use crate::access::Node;
use crate::occupancy::{LocalOccupancy, Owner};
use crate::search::{Costs, Guide, MAX_PENALTY, OwnVias, Search, WindowCache};
use crate::wiring::{DisjointSets, Edge, NetRoute, path_edges};
use pare_common::db::indices::{CutLayerId, LayerId, NetId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::drc::Marker as DrcMarker;
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

/// What a region worker did to one net: the edges it took out of the
/// net's committed wiring, and the edges it routed, `None` when the net
/// could not be connected.
#[derive(Clone, Debug)]
pub(super) struct Routed {
    pub(super) net: NetId,
    pub(super) removed: Vec<Edge>,
    pub(super) edges: Option<Vec<Edge>>,
}

impl Routed {
    /// The net's wiring with `with` in place of what was taken out.
    fn candidate<'e>(&'e self, with: &'e [Edge]) -> Candidate<'e> {
        Candidate {
            net: self.net,
            without: &self.removed,
            with,
        }
    }

    /// The same work with its routing given up.
    fn undone(self) -> Self {
        Self {
            edges: None,
            ..self
        }
    }
}

/// A net ripped up inside a region while its pieces were being routed.
#[derive(Debug)]
struct Victim {
    net: NetId,
    removed: Vec<Edge>,
    /// What to route again, or `None` when the wiring taken out joined
    /// nothing and is simply dropped.
    piece: Option<Piece>,
}

/// The pieces a region worker routes: those it was given and the victims
/// it ripped up, indexed together, the given first.
struct Work<'p> {
    given: &'p [Piece],
    victims: Vec<Victim>,
    results: Vec<Option<Vec<Edge>>>,
    /// The victims as owners, whose shared wiring the search disregards.
    ripped: Vec<Owner>,
}

impl<'p> Work<'p> {
    fn new(given: &'p [Piece]) -> Self {
        Self {
            given,
            victims: Vec::new(),
            results: vec![None; given.len()],
            ripped: Vec::new(),
        }
    }

    const fn len(&self) -> usize {
        self.given.len() + self.victims.len()
    }

    fn net(&self, i: usize) -> NetId {
        self.given
            .get(i)
            .map_or_else(|| self.victims[i - self.given.len()].net, |p| p.net)
    }

    /// The piece at `i`, if there is anything to route for it.
    fn piece(&self, i: usize) -> Option<&Piece> {
        match self.given.get(i) {
            Some(p) => Some(p),
            None => self.victims[i - self.given.len()].piece.as_ref(),
        }
    }

    fn holds(&self, net: NetId) -> bool {
        (0..self.len()).any(|i| self.net(i) == net)
    }

    /// Adds a victim and returns its index; a victim with nothing to route
    /// counts as routed with no edges.
    fn add_victim(&mut self, net: NetId, removed: Vec<Edge>, piece: Option<Piece>) -> usize {
        self.results.push(piece.is_none().then(Vec::new));
        self.ripped.push(Owner::Net(net.0));
        self.victims.push(Victim {
            net,
            removed,
            piece,
        });
        self.len() - 1
    }

    /// Every piece's wiring as it stands, for the judge.
    fn candidates(&self) -> Vec<Candidate<'_>> {
        (0..self.len())
            .map(|i| Candidate {
                net: self.net(i),
                without: self
                    .victims
                    .get(i.wrapping_sub(self.given.len()))
                    .map_or(&[][..], |v| v.removed.as_slice()),
                with: self.results[i].as_deref().unwrap_or(&[]),
            })
            .collect()
    }

    /// The work as done, with `results` as the wiring of each piece; a
    /// victim added after `results` were taken is left as it was.
    fn into_routed(self, results: Vec<Option<Vec<Edge>>>) -> Vec<Routed> {
        let given = self.given.len();
        results
            .into_iter()
            .enumerate()
            .map(|(i, edges)| Routed {
                net: self.net(i),
                removed: if i < given {
                    Vec::new()
                } else {
                    self.victims[i - given].removed.clone()
                },
                edges,
            })
            .collect()
    }
}

/// Edges to take out of and add to nets' committed wiring.
#[derive(Debug, Default)]
pub(super) struct Changes {
    by_net: FxHashMap<NetId, Change>,
}

#[derive(Debug, Default)]
struct Change {
    removed: FxHashSet<Edge>,
    added: Vec<Edge>,
}

impl Changes {
    /// Records that `removed` leaves `net`'s wiring and `added` joins it.
    pub(super) fn apply(&mut self, net: NetId, removed: Vec<Edge>, added: Vec<Edge>) {
        let change = self.by_net.entry(net).or_default();
        change.removed.extend(removed);
        change.added.extend(added);
    }
}

/// The wiring `item`'s net had inside its region before the worker ran:
/// the edges carved out of a given piece, or those taken from a victim.
fn wiring_before<'e>(
    item: &'e Routed,
    given: bool,
    carved: &'e FxHashMap<(usize, NetId), Vec<Edge>>,
    region: usize,
) -> &'e [Edge] {
    if given {
        carved
            .get(&(region, item.net))
            .map_or(&[][..], Vec::as_slice)
    } else {
        &item.removed
    }
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

        let routed: Vec<Vec<Routed>> = work
            .par_iter()
            .enumerate()
            .map(|(r, (area, pieces))| {
                let routed = self.route_region(area, pieces, costs);
                let given = pieces.len();
                let old: Vec<Candidate<'_>> = routed
                    .iter()
                    .enumerate()
                    .map(|(i, item)| item.candidate(wiring_before(item, i < given, &removed, r)))
                    .collect();
                let new: Vec<Candidate<'_>> = routed
                    .iter()
                    .enumerate()
                    .map(|(i, item)| {
                        let before = wiring_before(item, i < given, &removed, r);
                        item.candidate(item.edges.as_deref().unwrap_or(before))
                    })
                    .collect();
                let violations = |c: &[Candidate<'_>]| self.judge(area, c).findings.violations.len();
                let (after, before) = (violations(&new), violations(&old));
                if after > before {
                    log::trace!(
                        "region ({}, {})-({}, {}): new wiring rejected, {after} violations against {before}",
                        area.min.x.get(),
                        area.min.y.get(),
                        area.max.x.get(),
                        area.max.y.get()
                    );
                    return routed.into_iter().map(Routed::undone).collect();
                }
                routed
            })
            .collect();

        let mut count = 0;
        let mut changes = Changes::default();
        for (r, ((_, pieces), routed)) in work.iter().zip(routed).enumerate() {
            for (i, item) in routed.into_iter().enumerate() {
                count += 1;
                match item.edges {
                    Some(edges) => changes.apply(item.net, item.removed, edges),
                    None if i < pieces.len() => {
                        let restored = removed.get(&(r, item.net)).cloned().unwrap_or_default();
                        changes.apply(item.net, Vec::new(), restored);
                    }
                    None => {}
                }
            }
        }
        self.commit_changes(changes);
        count
    }

    /// Commits every net's wiring with `changes` made to it.
    pub(super) fn commit_changes(&mut self, changes: Changes) {
        for (net, change) in changes.by_net {
            let mut route = self.routes[net.index()].clone();
            route.edges.retain(|e| !change.removed.contains(e));
            route.edges.extend(change.added);
            route.edges.sort_unstable();
            route.edges.dedup();
            self.commit(net, &route);
        }
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
    /// growing penalty, and returns the wiring of the local round the judge
    /// liked best, as `TritonRoute` keeps a worker's best iteration. A net
    /// the judge finds in violation that is not among the pieces is ripped up
    /// inside the region too and routed again in the next local round, as
    /// `TritonRoute` queues the victims of a route; it comes back after the
    /// pieces, with the edges taken from it.
    pub(super) fn route_region(
        &self,
        area: &Rect<Dbu>,
        pieces: &[Piece],
        costs: Costs,
    ) -> Vec<Routed> {
        let window = Rect::new(area.min, area.max - Point::new(Dbu(1), Dbu(1)));
        let mut work = Work::new(pieces);
        let mut best: Option<(Score, Vec<Option<Vec<Edge>>>)> = None;
        let mut overlay = LocalOccupancy::default();
        let mut todo: Vec<usize> = self.ordered(&work, (0..pieces.len()).collect());
        let mut penalty = costs.penalty;
        for round in 0..LOCAL_ROUNDS {
            for &i in &todo {
                let Some(piece) = work.piece(i) else {
                    continue;
                };
                let owner = Owner::Net(piece.net.0);
                overlay.remove_owner(owner);
                let guided = Search {
                    db: self.db,
                    tech: self.tech,
                    occupancy: &self.occupancy,
                    overlay: Some(&overlay),
                    history: &self.history,
                    owner,
                    costs: Costs { penalty, ..costs },
                    guide: piece.guide.as_ref(),
                    own_vias: None,
                    ripped: &work.ripped,
                };
                let free = Search {
                    guide: None,
                    ..guided
                };
                let own = self.vias_near(piece.net, area);
                let groups = &piece.groups;
                let result = if guided.guide.is_some() {
                    self.connect_groups(&guided, groups, &window, own.clone())
                        .or_else(|| self.connect_groups(&free, groups, &window, own))
                } else {
                    self.connect_groups(&free, groups, &window, own)
                };
                if let Some(edges) = &result {
                    for (layer, rect) in self.edge_metal(edges) {
                        overlay.add_rect(self.tech, layer, &rect, owner);
                    }
                    for (cut, rect) in self.edge_cuts(edges) {
                        overlay.add_cut(self.tech, cut, &rect, owner);
                    }
                }
                work.results[i] = result;
            }
            let judgment = self.judge(area, &work.candidates());
            let score = Score {
                unconnected: work.results.iter().filter(|r| r.is_none()).count(),
                violations: judgment.findings.violations.len(),
            };
            if best.as_ref().is_none_or(|(b, _)| score < *b) {
                best = Some((score, work.results.clone()));
            }
            if round + 1 == LOCAL_ROUNDS {
                break;
            }
            self.price_markers(&judgment.findings.violations, &mut overlay);
            let mut again: Vec<usize> = (0..work.len())
                .filter(|&i| work.results[i].is_some() && judgment.nets.contains(&work.net(i)))
                .collect();
            let mut victims: Vec<NetId> = judgment
                .nets
                .iter()
                .copied()
                .filter(|n| !work.holds(*n))
                .collect();
            victims.sort_unstable();
            for net in victims {
                again.extend(self.rip_victim(net, area, &mut work, &mut overlay));
            }
            if again.is_empty() {
                break;
            }
            todo = self.ordered(&work, again);
            self.log_judgment(area, round, &judgment);
            penalty = (penalty * 2.0).min(MAX_PENALTY);
        }
        let kept = best.map_or_else(|| work.results.clone(), |(_, kept)| kept);
        work.into_routed(kept)
    }

    /// Rips `net`'s wiring inside `area` up for `work` when it can be routed
    /// again there: its wiring beyond the area stays in the worker's view,
    /// and its piece joins the work. Returns the index of what is to route.
    fn rip_victim(
        &self,
        net: NetId,
        area: &Rect<Dbu>,
        work: &mut Work<'_>,
        overlay: &mut LocalOccupancy,
    ) -> Option<usize> {
        if !self.routes[net.index()].complete {
            return None;
        }
        let (inside, piece) = self.carve(net, &[*area]).pop()?;
        if inside.is_empty() {
            return None;
        }
        let route = &self.routes[net.index()];
        let gone: FxHashSet<Edge> = inside.iter().copied().collect();
        let kept = NetRoute {
            edges: route
                .edges
                .iter()
                .copied()
                .filter(|e| !gone.contains(e))
                .collect(),
            access: route.access.clone(),
            ..NetRoute::default()
        };
        let near = area.expand(self.halo);
        let owner = Owner::Net(net.0);
        for (layer, rect) in kept.metal(self.db, self.tech) {
            if self.tech.routable(layer) && rect.touches(&near) {
                overlay.keep_rect(self.tech, layer, &rect, owner);
            }
        }
        for (cut, rect) in kept.cuts(self.db, self.tech) {
            if rect.touches(&near) {
                overlay.keep_cut(self.tech, cut, &rect, owner);
            }
        }
        Some(work.add_victim(net, inside, piece))
    }

    /// `indices` of `work` in the order to route them: smallest extent
    /// first, since a short piece tied to its pins has few ways through,
    /// while one passing through the region can shift to another track.
    fn ordered(&self, work: &Work<'_>, mut indices: Vec<usize>) -> Vec<usize> {
        indices.sort_by_cached_key(|&i| {
            let extent = work.piece(i).map_or(Dbu::ZERO, |p| self.extent(p));
            (extent, work.net(i))
        });
        indices
    }

    /// Logs what the judge still holds against a region's pieces.
    fn log_judgment(&self, area: &Rect<Dbu>, round: usize, judgment: &Judgment) {
        if !log::log_enabled!(log::Level::Trace) {
            return;
        }
        let name = |o: pare_common::verify::drc::OwnerIndex| match self.fixed.owners()[o.index()] {
            pare_common::verify::drc::Owner::Net(n) => self.db.nets[n.index()].name.clone(),
            other => format!("{other:?}"),
        };
        log::trace!(
            "region ({}, {})-({}, {}) local round {round} still violates:",
            area.min.x.get(),
            area.min.y.get(),
            area.max.x.get(),
            area.max.y.get()
        );
        for m in &judgment.findings.violations {
            log::trace!(
                "    {} on {:?} at ({}, {})-({}, {}) between {} and {}",
                m.violation.name(),
                m.layer,
                m.bbox.min.x.get(),
                m.bbox.min.y.get(),
                m.bbox.max.x.get(),
                m.bbox.max.y.get(),
                name(m.owners.0),
                name(m.owners.1)
            );
        }
    }

    /// Makes the search pay the conflict penalty wherever the judge found a
    /// violation, so that the next route of a piece avoids the spot, as
    /// `TritonRoute` adds marker costs.
    fn price_markers(&self, markers: &[DrcMarker], overlay: &mut LocalOccupancy) {
        for m in markers {
            match m.layer {
                LayerRef::Routing(layer) if self.tech.routable(layer) => {
                    overlay.add_rect(self.tech, layer, &m.bbox, Owner::Obstacle);
                }
                LayerRef::Routing(_) => {}
                LayerRef::Cut(cut) => overlay.add_cut(self.tech, cut, &m.bbox, Owner::Obstacle),
            }
        }
    }

    /// Half-perimeter of the box around a piece's nodes.
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
        let mut cache = WindowCache::new(search, *window);
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
            let (path, _) = cache.find(&joining, &sources, &targets)?;
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

/// How well a local round did: pieces left unconnected count before
/// violations, so that a connected result with a marker beats a gap.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct Score {
    unconnected: usize,
    violations: usize,
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
