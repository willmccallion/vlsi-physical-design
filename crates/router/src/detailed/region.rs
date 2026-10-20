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

use super::judge::{Candidate, Judgment};
use super::trace;
use super::{DetailedRouter, Marker};
use crate::access::Node;
use crate::occupancy::{LocalOccupancy, Owner};
use crate::search::{Costs, Marks, OwnVias, Search, WindowCache};
use crate::wiring::{DisjointSets, Edge, NetRoute, Patch, Wiring, path_edges, tidy_patches};
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

/// Local rounds in a row a net under a non-default rule may keep its route
/// while the nets it violates with are rerouted instead, as `TritonRoute`'s
/// `NDR_NETS_RIPUP_HARDINESS`.
const NDR_RIPUP_HARDINESS: u8 = 3;

/// Routing priority of a net under a non-default rule over other nets, as
/// `TritonRoute`'s `NDR_NETS_ABS_PRIORITY`: such nets route first.
const NDR_PRIORITY: u8 = 2;

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

/// One region a repair pass routes again: its area, the nets chosen
/// there, and the indices of the last round's markers touching it.
#[derive(Debug)]
pub(super) struct RegionJob {
    area: Rect<Dbu>,
    nets: Vec<NetId>,
    markers: Vec<usize>,
}

/// The part of a net a region worker routes again.
#[derive(Debug)]
pub(super) struct Piece {
    pub(super) net: NetId,
    /// Groups of crossings that must be joined; each group is already
    /// connected outside the region or is one pin's access point.
    pub(super) groups: Vec<Vec<Node>>,
}

/// The violations in a worker's area with its nets' wiring as it was and
/// as the worker left it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Verdict {
    pub(super) before: usize,
    pub(super) after: usize,
}

impl Verdict {
    /// Whether the new wiring is worse and is not kept, as `TritonRoute`
    /// keeps a worker's result unless it has more markers than it began
    /// with.
    pub(super) const fn rejected(self) -> bool {
        self.after > self.before
    }

    /// Whether the new wiring has fewer violations.
    pub(super) const fn improved(self) -> bool {
        self.after < self.before
    }
}

/// What a region worker did to one net: the wiring it took out of the
/// net's committed wiring, and the wiring it routed, `None` when the net
/// could not be connected.
#[derive(Clone, Debug)]
pub(super) struct Routed {
    pub(super) net: NetId,
    pub(super) removed: Wiring,
    pub(super) wiring: Option<Wiring>,
}

impl Routed {
    /// The net's wiring with `with` in place of what was taken out.
    const fn candidate<'e>(&'e self, with: &'e Wiring) -> Candidate<'e> {
        Candidate {
            net: self.net,
            without: &self.removed,
            with,
        }
    }

    /// The same work with its routing given up.
    fn undone(self) -> Self {
        Self {
            wiring: None,
            ..self
        }
    }
}

/// A net ripped up inside a region while its pieces were being routed.
#[derive(Debug)]
struct Victim {
    net: NetId,
    removed: Wiring,
    /// What to route again, or `None` when the wiring taken out joined
    /// nothing and is simply dropped.
    piece: Option<Piece>,
}

/// The pieces a region worker routes: those it was given and the victims
/// it ripped up, indexed together, the given first.
struct Work<'p> {
    given: &'p [Piece],
    victims: Vec<Victim>,
    results: Vec<Option<Wiring>>,
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
    fn add_victim(&mut self, net: NetId, removed: Wiring, piece: Option<Piece>) -> usize {
        self.results.push(piece.is_none().then(Wiring::default));
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
                without: self.removed(i),
                with: self.results[i].as_ref().unwrap_or(&NOTHING),
            })
            .collect()
    }

    /// What was taken out of the net of piece `i`: nothing for a given
    /// piece, whose wiring was carved before the worker ran.
    fn removed(&self, i: usize) -> &Wiring {
        self.victims
            .get(i.wrapping_sub(self.given.len()))
            .map_or(&NOTHING, |v| &v.removed)
    }

    /// The work as done, with `results` as the wiring of each piece; a
    /// victim added after `results` were taken is left as it was.
    fn into_routed(self, results: Vec<Option<Wiring>>) -> Vec<Routed> {
        results
            .into_iter()
            .enumerate()
            .map(|(i, wiring)| Routed {
                net: self.net(i),
                removed: self.removed(i).clone(),
                wiring,
            })
            .collect()
    }
}

/// No wiring at all.
static NOTHING: Wiring = Wiring {
    edges: Vec::new(),
    patches: Vec::new(),
};

/// Wiring to take out of and add to nets' committed wiring.
#[derive(Debug, Default)]
pub(super) struct Changes {
    by_net: FxHashMap<NetId, Change>,
}

#[derive(Debug, Default)]
struct Change {
    removed_edges: FxHashSet<Edge>,
    removed_patches: Vec<Patch>,
    added: Wiring,
}

impl Changes {
    /// Records that `removed` leaves `net`'s wiring and `added` joins it.
    pub(super) fn apply(&mut self, net: NetId, removed: Wiring, added: Wiring) {
        let change = self.by_net.entry(net).or_default();
        change.removed_edges.extend(removed.edges);
        change.removed_patches.extend(removed.patches);
        change.added.edges.extend(added.edges);
        change.added.patches.extend(added.patches);
    }
}

/// The wiring `item`'s net had inside its region before the worker ran:
/// what was carved out of a given piece, or what was taken from a victim.
fn wiring_before<'e>(
    item: &'e Routed,
    given: bool,
    carved: &'e FxHashMap<(usize, NetId), Wiring>,
    region: usize,
) -> &'e Wiring {
    if given {
        carved.get(&(region, item.net)).unwrap_or(&NOTHING)
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
    /// Regions whose new wiring has fewer violations than the old.
    pub improved: usize,
}

impl DetailedRouter<'_> {
    /// Routes again, region by region, the nets `scope` selects, with the
    /// region grid shifted by a third of a region per `pass`.
    pub fn repair(&mut self, costs: Costs, pass: usize, scope: Scope) -> RepairStats {
        let grid = RegionGrid::of_pass(self, pass);
        let mut regions: BTreeMap<RegionKey, FxHashSet<NetId>> = BTreeMap::new();
        for m in self.markers() {
            let nets = self.nets_of(m, scope);
            regions.entry(grid.key(m.at)).or_default().extend(nets);
        }
        let mut stats = RepairStats {
            regions: regions.len(),
            ..RepairStats::default()
        };
        let touching = self.markers_by_region(&grid);
        for batch_colour in 0..4 {
            let batch: Vec<RegionJob> = regions
                .iter()
                .filter(|&(&k, _)| colour(k) == batch_colour)
                .map(|(&k, nets)| {
                    let mut nets: Vec<NetId> = nets.iter().copied().collect();
                    nets.sort_unstable();
                    RegionJob {
                        area: grid.area(k),
                        nets,
                        markers: touching.get(&k).cloned().unwrap_or_default(),
                    }
                })
                .collect();
            let done = self.repair_batch(&batch, costs);
            stats.pieces += done.pieces;
            stats.improved += done.improved;
        }
        stats
    }

    /// The nets `scope` rips up for marker `m`.
    pub(super) fn nets_of(&self, m: &Marker, scope: Scope) -> Vec<NetId> {
        let mut nets: Vec<NetId> = m.nets.iter().flatten().copied().collect();
        if scope == Scope::NearConflicts {
            let reach = self.tech.layer(self.tech.min_layer).pitch() * NEAR_TRACKS;
            nets.extend(self.nets_near(m.at, reach));
        }
        nets.sort_unstable();
        nets.dedup();
        nets
    }

    /// The indices of the last round's markers touching each region of
    /// `grid`.
    fn markers_by_region(&self, grid: &RegionGrid) -> FxHashMap<RegionKey, Vec<usize>> {
        let mut touching: FxHashMap<RegionKey, Vec<usize>> = FxHashMap::default();
        for (i, m) in self.markers().iter().enumerate() {
            let (x0, y0) = grid.key(m.bbox.min);
            let (x1, y1) = grid.key(m.bbox.max);
            for ry in y0..=y1 {
                for rx in x0..=x1 {
                    touching.entry((rx, ry)).or_default().push(i);
                }
            }
        }
        touching
    }

    /// A worker's marks before it routes: those of the last round's
    /// `markers` that touch `area`, on the wiring of their nets as it
    /// stands, as `TritonRoute` seeds a worker's marker cost from the
    /// markers in it.
    pub(super) fn seed_marks(&self, area: &Rect<Dbu>, markers: &[usize]) -> Marks {
        let mut marks = Marks::default();
        for &i in markers {
            let m = &self.markers()[i];
            let mut nets: Vec<NetId> = m.nets.iter().flatten().copied().collect();
            nets.dedup();
            let routes: Vec<(NetId, &NetRoute)> =
                nets.iter().map(|&n| (n, &self.routes[n.index()])).collect();
            self.mark_violation(&mut marks, area, m.layer, &m.bbox, &routes);
        }
        marks
    }

    /// Adds to `marks` the violations a local round's `judgment` found, on
    /// the wiring of their nets as `work` proposes it.
    fn mark_judgment(
        &self,
        marks: &mut Marks,
        area: &Rect<Dbu>,
        judgment: &Judgment,
        work: &Work<'_>,
    ) {
        let candidates = work.candidates();
        let mut proposed: FxHashMap<NetId, NetRoute> = FxHashMap::default();
        for (m, owners) in judgment.findings.violations.iter().zip(&judgment.owners) {
            let mut nets: Vec<NetId> = owners.iter().flatten().copied().collect();
            nets.dedup();
            for &n in &nets {
                if let Some(c) = candidates.iter().find(|c| c.net == n) {
                    let _ = proposed.entry(n).or_insert_with(|| self.as_committed(c));
                }
            }
            let routes: Vec<(NetId, &NetRoute)> = nets
                .iter()
                .map(|&n| {
                    (
                        n,
                        proposed.get(&n).unwrap_or_else(|| &self.routes[n.index()]),
                    )
                })
                .collect();
            self.mark_violation(marks, area, m.layer, &m.bbox, &routes);
        }
    }

    /// Repairs regions that do not touch each other; returns the pieces
    /// routed again and the regions improved.
    fn repair_batch(&mut self, batch: &[RegionJob], costs: Costs) -> RepairStats {
        let seeds: Vec<Marks> = batch
            .par_iter()
            .map(|job| self.seed_marks(&job.area, &job.markers))
            .collect();
        let mut regions_of: BTreeMap<NetId, Vec<usize>> = BTreeMap::new();
        for (r, job) in batch.iter().enumerate() {
            for &net in &job.nets {
                if self.routes[net.index()].complete {
                    regions_of.entry(net).or_default().push(r);
                }
            }
        }
        let mut pieces: Vec<Vec<Piece>> = (0..batch.len()).map(|_| Vec::new()).collect();
        let mut removed: FxHashMap<(usize, NetId), Wiring> = FxHashMap::default();
        let mut carved = Changes::default();
        let carvings: Vec<Vec<(Wiring, Option<Piece>)>> = regions_of
            .par_iter()
            .map(|(&net, regions)| {
                let areas: Vec<Rect<Dbu>> = regions.iter().map(|&r| batch[r].area).collect();
                self.carve(net, &areas)
            })
            .collect();
        for ((&net, regions), carving) in regions_of.iter().zip(carvings) {
            for (k, (inside, piece)) in carving.into_iter().enumerate() {
                carved.apply(net, inside.clone(), Wiring::default());
                let _ = removed.insert((regions[k], net), inside);
                if let Some(piece) = piece {
                    pieces[regions[k]].push(piece);
                }
            }
        }
        self.commit_changes(carved);
        let work: Vec<(Rect<Dbu>, Vec<Piece>)> =
            batch.iter().map(|job| job.area).zip(pieces).collect();

        let routed: Vec<(Vec<Routed>, Verdict)> = work
            .par_iter()
            .zip(seeds)
            .enumerate()
            .map(|(r, ((area, pieces), marks))| {
                let routed = self.route_region(area, pieces, costs, marks);
                let given = pieces.len();
                let verdict = self.judge_change(area, &routed, |i, item| {
                    wiring_before(item, i < given, &removed, r)
                });
                if verdict.rejected() {
                    return (routed.into_iter().map(Routed::undone).collect(), verdict);
                }
                (routed, verdict)
            })
            .collect();

        let mut stats = RepairStats::default();
        let mut changes = Changes::default();
        for (r, ((_, pieces), (routed, verdict))) in work.iter().zip(routed).enumerate() {
            stats.improved += usize::from(verdict.improved());
            for (i, item) in routed.into_iter().enumerate() {
                stats.pieces += 1;
                match item.wiring {
                    Some(wiring) => changes.apply(item.net, item.removed, wiring),
                    None if i < pieces.len() => {
                        let restored = removed.get(&(r, item.net)).cloned().unwrap_or_default();
                        changes.apply(item.net, Wiring::default(), restored);
                    }
                    None => {}
                }
            }
        }
        self.commit_changes(changes);
        stats
    }

    /// Judges a worker's `routed` work in `area` against the wiring its
    /// nets had there before, `before_of` each item; logs the verdict for
    /// traced regions.
    pub(super) fn judge_change<'w>(
        &self,
        area: &Rect<Dbu>,
        routed: &'w [Routed],
        before_of: impl Fn(usize, &'w Routed) -> &'w Wiring,
    ) -> Verdict {
        let old: Vec<Candidate<'_>> = routed
            .iter()
            .enumerate()
            .map(|(i, item)| item.candidate(before_of(i, item)))
            .collect();
        let new: Vec<Candidate<'_>> = routed
            .iter()
            .enumerate()
            .map(|(i, item)| {
                item.candidate(item.wiring.as_ref().unwrap_or_else(|| before_of(i, item)))
            })
            .collect();
        let violations = |c: &[Candidate<'_>]| self.judge(area, c).findings.violations.len();
        let verdict = Verdict {
            before: violations(&old),
            after: violations(&new),
        };
        let (after, before) = (verdict.after, verdict.before);
        let outcome = if verdict.rejected() {
            "new wiring rejected"
        } else {
            "new wiring kept"
        };
        if self
            .trace
            .wants_region(area, routed.iter().map(|item| item.net))
        {
            log::info!(
                "trace region {}: {outcome} with {after} violations against {before} before",
                trace::region_text(area),
            );
        }
        if verdict.rejected() {
            log::trace!(
                "region ({}, {})-({}, {}): {outcome}, {after} violations against {before}",
                area.min.x.get(),
                area.min.y.get(),
                area.max.x.get(),
                area.max.y.get()
            );
        }
        verdict
    }

    /// Commits every net's wiring with `changes` made to it.
    pub(super) fn commit_changes(&mut self, changes: Changes) {
        let changes: Vec<(NetId, Change)> = changes.by_net.into_iter().collect();
        let routes: Vec<(NetId, NetRoute)> = changes
            .into_par_iter()
            .map(|(net, change)| {
                let mut route = self.routes[net.index()].clone();
                route.edges.retain(|e| !change.removed_edges.contains(&e));
                route
                    .patches
                    .retain(|p| !change.removed_patches.contains(p));
                route.edges.extend(change.added.edges);
                route.patches.extend(change.added.patches);
                tidy_patches(&mut route.patches);
                (net, route)
            })
            .collect();
        self.commit_all(routes);
    }

    /// Splits `net`'s edges inside each of `areas`, which do not touch, off
    /// its route. Returns, per area, the edges taken out and the piece to
    /// route again if the wiring left outside every area reaches that area
    /// in more than one place. Judging connectivity by what is left outside
    /// all areas at once means that joining each area's groups restores every
    /// connection the removed edges made.
    fn carve(&self, net: NetId, areas: &[Rect<Dbu>]) -> Vec<(Wiring, Option<Piece>)> {
        let route = &self.routes[net.index()];
        let area_of = |n: Node| {
            let p = self.point(n);
            areas.iter().position(|a| contains_half_open(a, p))
        };
        let mut removed: Vec<Wiring> = vec![Wiring::default(); areas.len()];
        let mut kept: Vec<Edge> = Vec::new();
        for e in route.edges.iter() {
            let (a, b) = e.ends();
            match (area_of(a), area_of(b)) {
                (Some(i), Some(j)) if i == j => removed[i].edges.push(e),
                _ => kept.push(e),
            }
        }
        for &p in &route.patches {
            let inside = areas
                .iter()
                .position(|a| contains_half_open(a, p.rect.center()));
            if let Some(i) = inside {
                removed[i].patches.push(p);
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
                let piece = (groups.len() > 1).then_some(Piece { net, groups });
                (inside, piece)
            })
            .collect()
    }

    /// Routes a region's pieces, rerouting those still in conflict, and
    /// returns the wiring of the local round the judge liked best, as
    /// `TritonRoute` keeps a worker's best iteration. The costs stay the
    /// round's throughout; what changes between local rounds are the
    /// `marks`, which decay after every piece routed and gain the
    /// violations each local round leaves, as a `TritonRoute` worker's
    /// marker cost does. A net
    /// the judge finds in violation that is not among the pieces is ripped up
    /// inside the region too and routed again in the next local round, as
    /// `TritonRoute` queues the victims of a route; it comes back after the
    /// pieces, with the edges taken from it.
    pub(super) fn route_region(
        &self,
        area: &Rect<Dbu>,
        pieces: &[Piece],
        costs: Costs,
        marks: Marks,
    ) -> Vec<Routed> {
        let work = Work::new(pieces);
        let todo = self.ordered(&work, (0..pieces.len()).collect());
        self.negotiate(area, work, LocalOccupancy::default(), todo, costs, marks)
    }

    /// Routes `nets` again inside `area` as [`Self::route_region`] routes
    /// pieces, ripping each up first in the worker's own view only, as it
    /// rips up victims: the committed wiring stays as it is, so workers on
    /// overlapping areas can run side by side.
    pub(super) fn route_ripping(
        &self,
        area: &Rect<Dbu>,
        nets: &[NetId],
        costs: Costs,
        marks: Marks,
    ) -> Vec<Routed> {
        let mut work = Work::new(&[]);
        let mut overlay = LocalOccupancy::default();
        let ripped: Vec<usize> = nets
            .iter()
            .filter_map(|&net| self.rip_victim(net, area, &mut work, &mut overlay))
            .collect();
        let todo = self.ordered(&work, ripped);
        self.negotiate(area, work, overlay, todo, costs, marks)
    }

    /// The local rounds of a region worker, starting with the pieces of
    /// `work` at `todo` and `overlay` holding what was ripped up.
    fn negotiate(
        &self,
        area: &Rect<Dbu>,
        mut work: Work<'_>,
        mut overlay: LocalOccupancy,
        mut todo: Vec<usize>,
        costs: Costs,
        mut marks: Marks,
    ) -> Vec<Routed> {
        let window = Rect::new(area.min, area.max - Point::new(Dbu(1), Dbu(1)));
        let mut best: Option<(Score, Vec<Option<Wiring>>)> = None;
        let mut avoided: FxHashMap<NetId, u8> = FxHashMap::default();
        for round in 0..LOCAL_ROUNDS {
            for &i in &todo {
                let Some(piece) = work.piece(i) else {
                    continue;
                };
                let owner = Owner::Net(piece.net.0);
                overlay.remove_owner(owner);
                let search = Search {
                    db: self.db,
                    tech: self.tech,
                    occupancy: &self.occupancy,
                    overlay: Some(&overlay),
                    marks: &marks,
                    owner,
                    costs,
                    guide: None,
                    own_vias: None,
                    ripped: &work.ripped,
                };
                let own = self.vias_near(piece.net, area);
                let edges = self.connect_groups(&search, &piece.groups, &window, own);
                if self.trace.wants(piece.net, area) {
                    self.trace_piece(area, round, piece.net, &search, edges.as_deref());
                }
                let result = edges.map(|edges| {
                    let wiring = Wiring {
                        edges,
                        patches: Vec::new(),
                    };
                    let candidate = Candidate {
                        net: piece.net,
                        without: work.removed(i),
                        with: &wiring,
                    };
                    let patches = self.min_area_patches(&candidate, area, &search);
                    Wiring { patches, ..wiring }
                });
                if let Some(wiring) = &result {
                    for (layer, rect) in self.edge_metal(piece.net, &wiring.edges) {
                        overlay.add_rect(self.tech, layer, &rect, owner);
                    }
                    for (cut, rect) in self.edge_cuts(&wiring.edges) {
                        overlay.add_cut(self.tech, cut, &rect, owner);
                    }
                    for p in &wiring.patches {
                        overlay.add_rect(self.tech, p.layer, &p.rect, owner);
                    }
                }
                work.results[i] = result;
                marks.decay(costs.decay);
            }
            let judgment = self.judge(area, &work.candidates());
            let score = Score {
                unconnected: work.results.iter().filter(|r| r.is_none()).count(),
                violations: judgment.findings.violations.len(),
            };
            let traced = self
                .trace
                .wants_region(area, (0..work.len()).map(|i| work.net(i)));
            if traced {
                self.trace_judgment(area, round, &score, &judgment);
            }
            if best.as_ref().is_none_or(|(b, _)| score < *b) {
                best = Some((score, work.results.clone()));
                if traced {
                    log::info!(
                        "trace region {} local {round} is the best so far",
                        trace::region_text(area)
                    );
                }
            }
            if round + 1 == LOCAL_ROUNDS {
                break;
            }
            self.mark_judgment(&mut marks, area, &judgment, &work);
            let reroute = self.to_reroute(&judgment, &mut avoided);
            let mut again: Vec<usize> = (0..work.len())
                .filter(|&i| work.results[i].is_some() && reroute.contains(&work.net(i)))
                .collect();
            let mut victims: Vec<NetId> = reroute
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
        let mut edges = route.edges.clone();
        edges.subtract(&inside.edges.iter().copied().collect());
        let kept = NetRoute {
            edges,
            patches: route
                .patches
                .iter()
                .copied()
                .filter(|p| !inside.patches.contains(p))
                .collect(),
            access: route.access.clone(),
            ..NetRoute::default()
        };
        let near = area.expand(self.halo);
        let owner = Owner::Net(net.0);
        for (layer, rect) in kept.metal(self.db, self.tech, net) {
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

    /// `indices` of `work` in the order to route them: nets of higher
    /// [`Self::priority`] first, then smallest extent first, since a short
    /// piece tied to its pins has few ways through, while one passing
    /// through the region can shift to another track.
    fn ordered(&self, work: &Work<'_>, mut indices: Vec<usize>) -> Vec<usize> {
        indices.sort_by_cached_key(|&i| {
            let extent = work.piece(i).map_or(Dbu::ZERO, |p| self.extent(p));
            let net = work.net(i);
            (std::cmp::Reverse(self.priority(net)), extent, net)
        });
        indices
    }

    /// How early `net` routes: [`NDR_PRIORITY`] under a non-default rule,
    /// whose wider wires and spacing have the fewest ways through.
    fn priority(&self, net: NetId) -> u8 {
        if self.tech.net_rule(net.index()).is_some() {
            NDR_PRIORITY
        } else {
            0
        }
    }

    /// The nets to reroute after `judgment`: every net in a violation,
    /// except that, as `TritonRoute` does, where a marker has more than one
    /// owner a net under a non-default rule keeps its route while some other
    /// net of the marker is rerouted, for up to [`NDR_RIPUP_HARDINESS`]
    /// rounds in a row, counted in `avoided`.
    fn to_reroute(
        &self,
        judgment: &Judgment,
        avoided: &mut FxHashMap<NetId, u8>,
    ) -> FxHashSet<NetId> {
        let can_avoid = |n: NetId, avoided: &FxHashMap<NetId, u8>| {
            self.priority(n) > 0 && avoided.get(&n).copied().unwrap_or(0) < NDR_RIPUP_HARDINESS
        };
        let mut reroute: FxHashSet<NetId> = FxHashSet::default();
        let mut spared: FxHashSet<NetId> = FxHashSet::default();
        for owners in &judgment.owners {
            let several = owners[0] != owners[1];
            let nets = owners.iter().flatten().copied();
            let (keep, rip): (Vec<NetId>, Vec<NetId>) =
                nets.partition(|&n| several && can_avoid(n, avoided));
            if rip.is_empty() {
                reroute.extend(keep);
            } else {
                reroute.extend(rip);
                spared.extend(keep);
            }
        }
        for n in spared.difference(&reroute) {
            *avoided.entry(*n).or_default() += 1;
        }
        for n in &reroute {
            let _ = avoided.remove(n);
        }
        reroute
    }

    /// Logs a traced piece's route and what it cost the search.
    fn trace_piece(
        &self,
        area: &Rect<Dbu>,
        round: usize,
        net: NetId,
        search: &Search<'_>,
        edges: Option<&[Edge]>,
    ) {
        let name = &self.db.nets[net.index()].name;
        let region = trace::region_text(area);
        let Some(edges) = edges else {
            log::info!("trace region {region} local {round}: {name} found no route");
            return;
        };
        let cost = trace::cost_text(self.db, &search.explain(edges));
        log::info!(
            "trace region {region} local {round}: {name} routed {} | cost {cost}",
            trace::route_text(self.db, self.tech, edges)
        );
    }

    /// Logs a traced region's judgment after a local round.
    fn trace_judgment(&self, area: &Rect<Dbu>, round: usize, score: &Score, judgment: &Judgment) {
        let name = |o: pare_common::verify::drc::OwnerIndex| match self.fixed.owners()[o.index()] {
            pare_common::verify::drc::Owner::Net(n) => self.db.nets[n.index()].name.clone(),
            other => format!("{other:?}"),
        };
        log::info!(
            "trace region {} local {round}: {} violations, {} pieces unconnected",
            trace::region_text(area),
            score.violations,
            score.unconnected
        );
        for m in &judgment.findings.violations {
            log::info!(
                "trace     {} on {:?} at {} between {} and {}",
                m.violation.name(),
                m.layer,
                trace::region_text(&m.bbox),
                name(m.owners.0),
                name(m.owners.1)
            );
        }
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
        let mut own = OwnVias::new(self.db, self.tech, self.tech.net_rule(net.index()));
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
            edges: edges.iter().copied().collect(),
            ..NetRoute::default()
        }
        .cuts(self.db, self.tech)
    }

    /// The metal of some edges of `net` on routable layers.
    fn edge_metal(
        &self,
        net: NetId,
        edges: &[Edge],
    ) -> Vec<(pare_common::db::indices::LayerId, Rect<Dbu>)> {
        let route = NetRoute {
            edges: edges.iter().copied().collect(),
            ..NetRoute::default()
        };
        route
            .metal(self.db, self.tech, net)
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
