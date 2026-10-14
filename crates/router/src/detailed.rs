//! Detailed routing on the track graph.
//!
//! Each net grows a tree from one pin's access points, connecting the
//! nearest remaining pin each time with a multi-source A* search inside its
//! global route. Nets are routed in parallel batches that cannot interact,
//! and nets in conflict are ripped up and rerouted until none remain.

use crate::access::{AccessPoint, Node, access_points};
use crate::global::{GCell, GlobalGrid, GlobalRoutes, RouteTree};
use crate::occupancy::{LocalOccupancy, Occupancy, Origin, Owner, TrackRef};
pub use crate::search::Costs;
use crate::search::{Guide, History, Search, Slack};
use crate::tech::RouteTech;
use crate::tree::{connection_order, hilbert_order};
use pare_common::db::design::{NetlistDB, RouteElem};
use pare_common::db::indices::{LayerId, NetId, ViaId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::shapes::{self, fixed_shapes, is_signal_net, wire_rect};
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::{BTreeMap, VecDeque};

/// The result of routing one net.
#[derive(Clone, Debug, Default)]
pub struct NetRoute {
    /// Wiring to store in the design.
    pub elems: Vec<RouteElem>,
    /// Metal to record in the occupancy, by layer.
    pub metal: Vec<(LayerId, Rect<Dbu>)>,
    /// Whether every pin was connected.
    pub complete: bool,
}

/// State shared by all detailed routing rounds.
#[derive(Debug)]
pub struct DetailedRouter<'a> {
    db: &'a NetlistDB,
    tech: &'a RouteTech,
    /// Occupancy of every track.
    pub occupancy: Occupancy,
    history: History,
    access: Vec<Vec<Vec<AccessPoint>>>,
    routed: Vec<Vec<TrackRef>>,
    vias: Vec<Vec<(ViaId, Point<Dbu>)>>,
    window_margin: Dbu,
    gcell: Dbu,
    grid_origin: Point<Dbu>,
    guides: Vec<Option<NetGuide>>,
    rip_ups: Vec<u8>,
}

impl<'a> DetailedRouter<'a> {
    /// Prepares routing: computes pin access and records obstacles and
    /// pins on the routable layers.
    pub fn new(db: &'a NetlistDB, tech: &'a RouteTech, gcell: Dbu) -> Self {
        let mut occupancy = Occupancy::new(tech, gcell);
        for s in fixed_shapes(db) {
            let LayerRef::Routing(layer) = s.shape.layer else {
                continue;
            };
            if !tech.routable(layer) {
                continue;
            }
            let owner = match s.owner {
                shapes::Owner::Net(n) => Owner::Net(n.0),
                shapes::Owner::Obstacle => Owner::Obstacle,
            };
            let _ = occupancy.add_rect(tech, layer, &s.shape.rect, owner, Origin::Fixed);
        }
        let mut access: Vec<Vec<Vec<AccessPoint>>> = (0..db.nets.len())
            .into_par_iter()
            .map(|n| {
                if !is_signal_net(db, NetId::new(n)) {
                    return Vec::new();
                }
                db.nets[n]
                    .pins
                    .iter()
                    .map(|&p| access_points(db, tech, p))
                    .collect()
            })
            .collect();
        reserve_accesses(db, tech, &mut access, &mut occupancy);
        Self {
            db,
            tech,
            occupancy,
            history: History::new(gcell / 4),
            access,
            routed: vec![Vec::new(); db.nets.len()],
            rip_ups: vec![0; db.nets.len()],
            vias: vec![Vec::new(); db.nets.len()],
            window_margin: gcell * 4,
            gcell,
            grid_origin: db.die_area.min,
            guides: vec![None; db.nets.len()],
        }
    }

    /// Nets that have a pin with no access point.
    pub fn inaccessible_nets(&self) -> Vec<NetId> {
        (0..self.access.len())
            .filter(|&n| self.access[n].iter().any(Vec::is_empty))
            .map(NetId::new)
            .collect()
    }

    /// Routes `nets` in parallel, so that every net sees the wiring of each
    /// net routed before it that it could touch.
    ///
    /// Nets are scheduled by tiles while the tiles spread the work: once one
    /// block would hold more than a thread's share of a level's nets, the
    /// rest are routed in speculative batches instead.
    pub fn route_nets(&mut self, nets: &[NetId], costs: Costs) -> Vec<(NetId, NetRoute)> {
        let die = self.db.die_area;
        let die_span = (die.max.x - die.min.x).max(die.max.y - die.min.y);
        let threads = rayon::current_num_threads().max(1);
        let mut done = Vec::with_capacity(nets.len());
        let mut pending = nets.to_vec();
        let mut tile = self.gcell * TILE_GCELLS;
        while !pending.is_empty() && tile * MIN_TILES_ACROSS <= die_span {
            let (blocks, spanning) = self.partition_by_tile(&pending, tile);
            let tiled: usize = blocks.values().map(Vec::len).sum();
            let largest = blocks.values().map(Vec::len).max().unwrap_or(0);
            if largest * threads > tiled {
                break;
            }
            self.route_blocks(&blocks, costs, &mut done);
            pending = spanning;
            tile = tile * 4;
        }
        self.route_speculatively(pending, costs, &mut done);
        done
    }

    /// Routes nets in parallel batches against the same occupancy, then
    /// commits them in order. A net whose wiring touches a net committed
    /// earlier in its batch is routed once more in the next batch; after
    /// that, conflicts are left to negotiation.
    fn route_speculatively(
        &mut self,
        nets: Vec<NetId>,
        costs: Costs,
        done: &mut Vec<(NetId, NetRoute)>,
    ) {
        let batch_size = rayon::current_num_threads().max(1);
        let mut pending: VecDeque<NetId> = nets.into();
        let mut retried: FxHashSet<NetId> = FxHashSet::default();
        while !pending.is_empty() {
            let take = batch_size.min(pending.len());
            let batch: Vec<NetId> = pending.drain(..take).collect();
            let results: Vec<(NetId, NetRoute)> = batch
                .par_iter()
                .map(|&n| (n, self.route_net(n, costs, None)))
                .collect();
            let mut committed: FxHashSet<u32> = FxHashSet::default();
            let mut deferred = Vec::new();
            for (n, route) in results {
                if !retried.contains(&n) && self.touches_any(n, &route, &committed) {
                    let _ = retried.insert(n);
                    deferred.push(n);
                    continue;
                }
                self.commit(n, &route);
                let _ = committed.insert(n.0);
                done.push((n, route));
            }
            for n in deferred.into_iter().rev() {
                pending.push_front(n);
            }
        }
    }

    /// Whether `route` of `net` conflicts with the wiring of any net in `nets`.
    fn touches_any(&self, net: NetId, route: &NetRoute, nets: &FxHashSet<u32>) -> bool {
        route.metal.iter().any(|(layer, rect)| {
            self.tech.routable(*layer)
                && self
                    .occupancy
                    .rect_conflicts(self.tech, *layer, rect, Owner::Net(net.0))
                    .any(|(_, o)| matches!(o.owner, Owner::Net(other) if nets.contains(&other)))
        })
    }

    /// Routes the blocks in nine phases; blocks of one phase are at least a
    /// tile apart, so they are routed in parallel without their nets meeting.
    fn route_blocks(
        &mut self,
        blocks: &TileBlocks,
        costs: Costs,
        done: &mut Vec<(NetId, NetRoute)>,
    ) {
        for phase in 0..9 {
            let work: Vec<&Vec<NetId>> = blocks
                .iter()
                .filter(|((bx, by), _)| bx.rem_euclid(3) + 3 * by.rem_euclid(3) == phase)
                .map(|(_, nets)| nets)
                .collect();
            let results: Vec<Vec<(NetId, NetRoute)>> = work
                .par_iter()
                .map(|nets| self.route_block(nets, costs))
                .collect();
            for (n, route) in results.into_iter().flatten() {
                self.commit(n, &route);
                done.push((n, route));
            }
        }
    }

    /// Groups nets whose window fits in a 2x2 block of `tile` squares by the
    /// block's lower-left tile; the rest are returned separately.
    fn partition_by_tile(&self, nets: &[NetId], tile: Dbu) -> (TileBlocks, Vec<NetId>) {
        let tile = tile.get();
        let origin = self.grid_origin;
        let mut blocks: TileBlocks = BTreeMap::new();
        let mut spanning = Vec::new();
        for &n in nets {
            let w = self.window(n);
            let tx = |v: Dbu| (v - origin.x).get().div_euclid(tile);
            let ty = |v: Dbu| (v - origin.y).get().div_euclid(tile);
            let (x0, x1, y0, y1) = (tx(w.min.x), tx(w.max.x), ty(w.min.y), ty(w.max.y));
            if x1 - x0 <= 1 && y1 - y0 <= 1 {
                blocks.entry((x0, y0)).or_default().push(n);
            } else {
                spanning.push(n);
            }
        }
        (blocks, spanning)
    }

    /// Routes one block's nets in order, each seeing the ones before it.
    fn route_block(&self, nets: &[NetId], costs: Costs) -> Vec<(NetId, NetRoute)> {
        let mut overlay = LocalOccupancy::default();
        let mut out = Vec::with_capacity(nets.len());
        for &n in nets {
            let route = self.route_net(n, costs, Some(&overlay));
            for (layer, rect) in &route.metal {
                if self.tech.routable(*layer) {
                    overlay.add_rect(self.tech, *layer, rect, Owner::Net(n.0));
                }
            }
            out.push((n, route));
        }
        out
    }

    /// Removes a net's metal from the occupancy and loosens its guide.
    pub fn rip_up(&mut self, net: NetId) {
        let tracks = std::mem::take(&mut self.routed[net.index()]);
        self.occupancy.remove_wiring(&tracks, Owner::Net(net.0));
        self.rip_ups[net.index()] = self.rip_ups[net.index()].saturating_add(1);
    }

    fn slack(&self, net: NetId) -> Slack {
        Slack::after_rip_ups(self.rip_ups[net.index()])
    }

    fn commit(&mut self, net: NetId, route: &NetRoute) {
        let mut touched = Vec::new();
        for (layer, rect) in &route.metal {
            if self.tech.routable(*layer) {
                touched.extend(self.occupancy.add_rect(
                    self.tech,
                    *layer,
                    rect,
                    Owner::Net(net.0),
                    Origin::Wiring,
                ));
            }
        }
        touched.sort_unstable();
        touched.dedup();
        self.routed[net.index()] = touched;
        self.vias[net.index()] = route
            .elems
            .iter()
            .filter_map(|e| match *e {
                RouteElem::Via { via, at } => Some((via, at)),
                RouteElem::Wire { .. } => None,
            })
            .collect();
    }

    /// Records every current conflict in the history and returns the nets
    /// involved.
    pub fn conflicted_nets(&mut self, history_step: f64) -> Vec<NetId> {
        let mut nets = FxHashSet::default();
        for (track, a, b) in self.occupancy.all_conflicts(self.tech) {
            self.history.bump(track, a.lo.max(b.lo), history_step);
            for o in [a.owner, b.owner] {
                if let Owner::Net(n) = o {
                    let _ = nets.insert(NetId(n));
                }
            }
        }
        for n in 0..self.vias.len() {
            for (via, at) in self.crowded_cuts(n) {
                let bottom = self.db.library.via(via).bottom();
                for layer in [bottom, LayerId::new(bottom.index() + 1)] {
                    if let Some(track) = self.track_at(layer, at) {
                        self.history.bump(
                            track,
                            self.tech.layer(layer).across_along(at).1,
                            history_step,
                        );
                    }
                }
                let _ = nets.insert(NetId::new(n));
            }
        }
        let mut out: Vec<NetId> = nets.into_iter().collect();
        out.sort_unstable();
        out
    }

    /// Vias of net `n` within cut spacing of another of its vias on the same cut.
    fn crowded_cuts(&self, n: usize) -> Vec<(ViaId, Point<Dbu>)> {
        let vias = &self.vias[n];
        let mut out = Vec::new();
        for (i, &(va, a)) in vias.iter().enumerate() {
            let cut_a = self.db.library.via(va).cut;
            for &(vb, b) in &vias[i + 1..] {
                if self.db.library.via(vb).cut != cut_a || a == b {
                    continue;
                }
                let reach = self.tech.vias[cut_a.index()].cut_reach;
                if (a.x - b.x).abs() < reach && (a.y - b.y).abs() < reach {
                    out.push((va, a));
                    out.push((vb, b));
                }
            }
        }
        out
    }

    /// The track of `layer` running through `p`, if one does.
    fn track_at(&self, layer: LayerId, p: Point<Dbu>) -> Option<TrackRef> {
        if !self.tech.routable(layer) {
            return None;
        }
        let lt = self.tech.layer(layer);
        let across = lt.across_along(p).0;
        let i = lt.tracks.binary_search(&across).ok()?;
        Some(TrackRef {
            layer,
            track: crate::tech::to_u32(i),
        })
    }

    /// Nets whose metal sits where an access point of one of `nets` would
    /// go; those spots also gain history so the nets move off them.
    pub fn access_blockers(&mut self, nets: &[NetId], history_step: f64) -> Vec<NetId> {
        let mut blockers = FxHashSet::default();
        for &n in nets {
            for candidates in &self.access[n.index()] {
                for a in candidates {
                    let mut metal = NetRoute::default();
                    emit_access(self.db, self.tech, a, &mut metal);
                    for (layer, rect) in &metal.metal {
                        if !self.tech.routable(*layer) {
                            continue;
                        }
                        let hits: Vec<(TrackRef, Dbu, Owner)> = self
                            .occupancy
                            .rect_conflicts(self.tech, *layer, rect, Owner::Net(n.0))
                            .map(|(t, o)| (t, o.lo, o.owner))
                            .collect();
                        for (t, lo, owner) in hits {
                            if let Owner::Net(m) = owner {
                                let _ = blockers.insert(NetId(m));
                                self.history.bump(t, lo, history_step);
                            }
                        }
                    }
                }
            }
        }
        let mut out: Vec<NetId> = blockers.into_iter().collect();
        out.sort_unstable();
        out
    }

    /// Global routing terminals: the gcell of each pin's best access point.
    pub fn terminals(&self, net: NetId, grid: &GlobalGrid) -> Option<Vec<GCell>> {
        self.access[net.index()]
            .iter()
            .map(|c| {
                let a = c.first()?;
                let p = self
                    .tech
                    .layer(a.node.layer)
                    .point(a.node.track, a.node.cross);
                Some(grid.cell_at(a.node.layer, p))
            })
            .collect()
    }

    /// Restricts each net's detailed route to its global route.
    pub fn set_guides(&mut self, routes: GlobalRoutes, grid: &GlobalGrid) {
        for (net, paths) in routes.paths {
            let tree = RouteTree::new(&paths);
            let bbox = tree
                .cells()
                .iter()
                .map(|c| grid.cell_rect(c.x, c.y))
                .reduce(|a, b| a.union(&b));
            if let (Some(bbox), Some(terminals)) = (bbox, self.terminals(net, grid)) {
                self.guides[net.index()] = Some(NetGuide {
                    tree,
                    terminals,
                    bbox,
                });
            }
        }
    }

    /// The window a net searches in: its guide, or else its pins' bounding
    /// box plus a margin.
    fn window(&self, net: NetId) -> Rect<Dbu> {
        if let Some(guide) = &self.guides[net.index()] {
            return guide.bbox.expand(self.gcell * self.slack(net).gcells);
        }
        let pins = &self.db.nets[net.index()].pins;
        pins.iter()
            .map(|&p| {
                let at = self.db.pin_position(p);
                Rect::new(at, at)
            })
            .reduce(|a, b| a.union(&b))
            .unwrap_or_default()
            .expand(self.window_margin)
    }

    fn route_net(&self, net: NetId, costs: Costs, overlay: Option<&LocalOccupancy>) -> NetRoute {
        let pins = &self.access[net.index()];
        if pins.iter().any(Vec::is_empty) || pins.len() < 2 {
            return NetRoute::default();
        }
        let net_guide = self.guides[net.index()].as_ref();
        let slack = self.slack(net);
        let guide = net_guide.map(|g| {
            Guide::new(
                g.tree.cells(),
                self.grid_origin,
                self.gcell,
                self.tech,
                slack,
            )
        });
        let mut search = Search {
            db: self.db,
            tech: self.tech,
            occupancy: &self.occupancy,
            overlay,
            history: &self.history,
            owner: Owner::Net(net.0),
            costs,
            guide: guide.as_ref(),
        };
        let window = self.window(net);
        if let Some(route) = self.route_tree(&search, pins, &window, net_guide) {
            return route;
        }
        search.guide = None;
        let mut window = window.expand(self.window_margin);
        for _ in 0..2 {
            if let Some(route) = self.route_tree(&search, pins, &window, None) {
                return route;
            }
            window = window.expand(self.window_margin * 4);
        }
        NetRoute::default()
    }

    /// Connects every pin of a net, searching first along the branches of
    /// `guide`'s global route when given. Nets with more than
    /// [`CHUNKED_PINS`] pins are split into runs of nearby pins whose
    /// subtrees are routed in parallel and then joined.
    fn route_tree(
        &self,
        search: &Search<'_>,
        pins: &[Vec<AccessPoint>],
        window: &Rect<Dbu>,
        guide: Option<&NetGuide>,
    ) -> Option<NetRoute> {
        let all: Vec<usize> = (0..pins.len()).collect();
        let tree = if pins.len() > CHUNKED_PINS {
            self.grow_chunked(search, pins, window, guide)?
        } else {
            self.grow(search, pins, &all, window, guide)?
        };
        let mut route = NetRoute {
            complete: true,
            ..NetRoute::default()
        };
        for path in &tree.paths {
            emit_path(self.db, self.tech, path, &mut route);
        }
        for &(pin, choice) in &tree.used {
            emit_access(self.db, self.tech, &pins[pin][choice], &mut route);
        }
        Some(route)
    }

    /// Joins the `members` pins one at a time, each to the nearest member
    /// joined before it.
    fn grow(
        &self,
        search: &Search<'_>,
        pins: &[Vec<AccessPoint>],
        members: &[usize],
        window: &Rect<Dbu>,
        guide: Option<&NetGuide>,
    ) -> Option<SubTree> {
        let points: Vec<Point<Dbu>> = members
            .iter()
            .map(|&m| self.point(pins[m][0].node))
            .collect();
        let (first, joins) = connection_order(&points);
        let mut tree = TreeIndex::new(self.grid_origin, self.gcell);
        let mut out = SubTree::default();
        let first_pin = members[first];
        if joins.is_empty() {
            out.used.push((first_pin, 0));
            out.nodes.push(pins[first_pin][0].node);
            return Some(out);
        }
        let first_sources: Vec<(Node, f64)> = pins[first_pin]
            .iter()
            .map(|a| (a.node, search.access_cost(a)))
            .collect();

        for join in joins {
            let pin = members[join.pin];
            if let Some(k) = pins[pin].iter().position(|a| tree.contains(a.node)) {
                out.used.push((pin, k));
                continue;
            }
            let targets: FxHashMap<Node, (usize, f64)> = pins[pin]
                .iter()
                .enumerate()
                .map(|(k, a)| (a.node, (k, search.access_cost(a))))
                .collect();
            let starting = tree.is_empty();
            let sources_in = |local: &Rect<Dbu>| {
                if starting {
                    first_sources.clone()
                } else {
                    tree.nodes_in(local)
                }
            };
            let corridor = guide.and_then(|g| g.corridor(members[join.parent], pin));
            let (path, reached) = self.connect(
                search,
                (points[join.pin], points[join.parent]),
                corridor.as_deref(),
                window,
                sources_in,
                &targets,
            )?;
            if starting {
                let start = pins[first_pin].iter().position(|a| a.node == path[0])?;
                out.used.push((first_pin, start));
            }
            out.used.push((pin, reached));
            for &n in &path {
                tree.insert(n, self.point(n));
            }
            out.paths.push(path);
        }
        out.nodes = tree.nodes.into_iter().collect();
        Some(out)
    }

    /// The nodes of `sub` near `rep`, or all of them if none is; a small
    /// target area keeps the search's distance estimate sharp.
    fn join_targets(&self, sub: &SubTree, rep: Point<Dbu>) -> FxHashMap<Node, (usize, f64)> {
        let reach = self.gcell * JOIN_GCELLS;
        let near: FxHashMap<Node, (usize, f64)> = sub
            .nodes
            .iter()
            .filter(|&&n| {
                let p = self.point(n);
                (p.x - rep.x).abs() + (p.y - rep.y).abs() <= reach
            })
            .map(|&n| (n, (0, 0.0)))
            .collect();
        if near.is_empty() {
            sub.nodes.iter().map(|&n| (n, (0, 0.0))).collect()
        } else {
            near
        }
    }

    /// Searches along the `corridor` of gcells from the tree to the target
    /// first, then windows around the `ends` that grow until a path from the
    /// sources inside them reaches a target, or they cover `window`.
    fn connect(
        &self,
        search: &Search<'_>,
        ends: (Point<Dbu>, Point<Dbu>),
        corridor: Option<&[GCell]>,
        window: &Rect<Dbu>,
        sources_in: impl Fn(&Rect<Dbu>) -> Vec<(Node, f64)>,
        targets: &FxHashMap<Node, (usize, f64)>,
    ) -> Option<(Vec<Node>, usize)> {
        if let Some(cells) = corridor {
            if let Some(found) = self.route_along(search, cells, window, &sources_in, targets) {
                return Some(found);
            }
            let area = search.guide.and_then(|g| g.area(cells));
            if let Some(local) = area.and_then(|c| c.intersection(window)) {
                let sources = sources_in(&local);
                if !sources.is_empty()
                    && let Some(found) = search.find(&sources, targets, &local)
                {
                    return Some(found);
                }
            }
        }
        let (from, to) = ends;
        let mut margin = self.gcell * 2;
        loop {
            let around = Rect::from_corners(from, to).expand(margin);
            let local = around.intersection(window).unwrap_or(*window);
            let sources = sources_in(&local);
            if !sources.is_empty()
                && let Some(found) = search.find(&sources, targets, &local)
            {
                return Some(found);
            }
            if local.encloses(window) {
                return None;
            }
            margin = margin * 4;
        }
    }

    /// Follows a long corridor in segments, each searching only the gcells
    /// from the end of the last segment to the next waypoint gcell, so a
    /// search never floods more than a few gcells. `None` for a corridor too
    /// short to split, or when a segment finds no path.
    fn route_along(
        &self,
        search: &Search<'_>,
        cells: &[GCell],
        window: &Rect<Dbu>,
        sources_in: &impl Fn(&Rect<Dbu>) -> Vec<(Node, f64)>,
        targets: &FxHashMap<Node, (usize, f64)>,
    ) -> Option<(Vec<Node>, usize)> {
        let guide = search.guide?;
        let stops = waypoints(cells, SEGMENT_GCELLS);
        if stops.is_empty() {
            return None;
        }
        let mut path: Vec<Node> = Vec::new();
        let mut start = 0;
        for &stop in &stops {
            let local = guide.area(&cells[start..=stop])?.intersection(window)?;
            let sources = match path.last() {
                Some(&end) => vec![(end, 0.0)],
                None => sources_in(&local),
            };
            let goal = self.nodes_in_cell(search, guide.cell_rect(cells[stop]), cells[stop]);
            let (segment, _) = search.find(&sources, &goal, &local)?;
            append_segment(&mut path, segment);
            start = stop;
        }
        let local = guide.area(&cells[start..])?.intersection(window)?;
        let end = *path.last()?;
        let (segment, reached) = search.find(&[(end, 0.0)], targets, &local)?;
        append_segment(&mut path, segment);
        Some((path, reached))
    }

    /// Every node the search may use in `area`, the extent of `cell`, on
    /// the cell's layer and the layers next to it.
    fn nodes_in_cell(
        &self,
        search: &Search<'_>,
        area: Rect<Dbu>,
        cell: GCell,
    ) -> FxHashMap<Node, (usize, f64)> {
        let lowest = cell
            .layer
            .index()
            .saturating_sub(1)
            .max(self.tech.min_layer.index());
        let highest = (cell.layer.index() + 1).min(self.tech.max_layer.index());
        let mut nodes = FxHashMap::default();
        for l in lowest..=highest {
            let layer = LayerId::new(l);
            let lt = self.tech.layer(layer);
            let ((a0, a1), (b0, b1)) = lt.across_along_rect(&area);
            for track in lt.tracks_within(a0, a1) {
                for cross in lt.cross_within(b0, b1) {
                    let node = Node {
                        layer,
                        track,
                        cross,
                    };
                    if search.allowed(layer, lt.point(track, cross)) {
                        let _ = nodes.insert(node, (0, 0.0));
                    }
                }
            }
        }
        nodes
    }

    /// Routes runs of nearby pins in parallel, then joins their subtrees.
    fn grow_chunked(
        &self,
        search: &Search<'_>,
        pins: &[Vec<AccessPoint>],
        window: &Rect<Dbu>,
        guide: Option<&NetGuide>,
    ) -> Option<SubTree> {
        let points: Vec<Point<Dbu>> = pins.iter().map(|c| self.point(c[0].node)).collect();
        let order = hilbert_order(&points);
        let chunks: Vec<&[usize]> = order.chunks(CHUNK_PINS).collect();
        let subtrees: Vec<SubTree> = chunks
            .par_iter()
            .map(|members| self.grow(search, pins, members, window, guide))
            .collect::<Option<Vec<_>>>()?;

        let reps: Vec<Point<Dbu>> = chunks.iter().map(|c| points[c[0]]).collect();
        let (first, joins) = connection_order(&reps);
        let mut tree = TreeIndex::new(self.grid_origin, self.gcell);
        for &n in &subtrees[first].nodes {
            tree.insert(n, self.point(n));
        }
        let mut out = SubTree::default();
        for join in joins {
            let sub = &subtrees[join.pin];
            if !sub.nodes.iter().any(|&n| tree.contains(n)) {
                let targets = self.join_targets(sub, reps[join.pin]);
                let corridor =
                    guide.and_then(|g| g.corridor(chunks[join.parent][0], chunks[join.pin][0]));
                let (path, _) = self.connect(
                    search,
                    (reps[join.pin], reps[join.parent]),
                    corridor.as_deref(),
                    window,
                    |r| tree.nodes_in(r),
                    &targets,
                )?;
                for &n in &path {
                    tree.insert(n, self.point(n));
                }
                out.paths.push(path);
            }
            for &n in &sub.nodes {
                tree.insert(n, self.point(n));
            }
        }
        for sub in subtrees {
            out.paths.extend(sub.paths);
            out.used.extend(sub.used);
        }
        Some(out)
    }

    fn point(&self, n: Node) -> Point<Dbu> {
        self.tech.layer(n.layer).point(n.track, n.cross)
    }
}

/// Nets grouped by the lower-left tile of the 2x2 block their window fits.
type TileBlocks = BTreeMap<(i64, i64), Vec<NetId>>;

impl NetGuide {
    /// The gcells of the global route from pin `from` to pin `to`.
    fn corridor(&self, from: usize, to: usize) -> Option<Vec<GCell>> {
        self.tree.branch(self.terminals[from], self.terminals[to])
    }
}

/// Indices of the cells where a long corridor is split: one about every
/// `every` steps between gcells, none closer than half that to the end.
fn waypoints(cells: &[GCell], every: usize) -> Vec<usize> {
    let steps: Vec<usize> = cells
        .windows(2)
        .scan(0, |n, w| {
            *n += usize::from((w[0].x, w[0].y) != (w[1].x, w[1].y));
            Some(*n)
        })
        .collect();
    let total = steps.last().copied().unwrap_or(0);
    let mut stops = Vec::new();
    let mut last = 0;
    for (i, &n) in steps.iter().enumerate() {
        if n - last >= every && total - n >= every / 2 {
            stops.push(i + 1);
            last = n;
        }
    }
    stops
}

/// Appends a segment that starts where `path` ends.
fn append_segment(path: &mut Vec<Node>, segment: Vec<Node>) {
    let skip = usize::from(path.last() == segment.first());
    path.extend(segment.into_iter().skip(skip));
}

/// A net's global route and the gcell of each of its pins.
#[derive(Clone, Debug)]
struct NetGuide {
    tree: RouteTree,
    terminals: Vec<GCell>,
    bbox: Rect<Dbu>,
}

/// Gcells between the waypoints of a long connection.
const SEGMENT_GCELLS: usize = 6;

/// Tile levels stop once fewer tiles than this span the die.
const MIN_TILES_ACROSS: i64 = 8;

/// Edge, in gcells, of the smallest tiles that schedule nets for parallel
/// routing; each further level of tiles is four times larger.
const TILE_GCELLS: i64 = 8;

/// Nets with more pins than this are routed as parallel chunks.
const CHUNKED_PINS: usize = 1024;

/// Pins per chunk of a chunked net.
const CHUNK_PINS: usize = 256;

/// Distance, in gcells, from a chunk's first pin within which another chunk
/// joins its subtree.
const JOIN_GCELLS: i64 = 2;

/// The paths of a tree over some pins, the access point used for each pin,
/// and every node it covers.
#[derive(Debug, Default)]
struct SubTree {
    paths: Vec<Vec<Node>>,
    used: Vec<(usize, usize)>,
    nodes: Vec<Node>,
}

/// A tree node and its location.
type NodeAt = (Node, Point<Dbu>);

/// The nodes of a growing tree, bucketed by gcell to find those in a window.
struct TreeIndex {
    origin: Point<Dbu>,
    gcell: Dbu,
    nodes: FxHashSet<Node>,
    buckets: FxHashMap<(i64, i64), Vec<NodeAt>>,
}

impl TreeIndex {
    fn new(origin: Point<Dbu>, gcell: Dbu) -> Self {
        Self {
            origin,
            gcell,
            nodes: FxHashSet::default(),
            buckets: FxHashMap::default(),
        }
    }

    fn key(&self, p: Point<Dbu>) -> (i64, i64) {
        (
            (p.x - self.origin.x).get().div_euclid(self.gcell.get()),
            (p.y - self.origin.y).get().div_euclid(self.gcell.get()),
        )
    }

    fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    fn contains(&self, n: Node) -> bool {
        self.nodes.contains(&n)
    }

    fn insert(&mut self, n: Node, p: Point<Dbu>) {
        if self.nodes.insert(n) {
            let key = self.key(p);
            self.buckets.entry(key).or_default().push((n, p));
        }
    }

    /// Tree nodes inside `r`, as zero-cost search sources.
    fn nodes_in(&self, r: &Rect<Dbu>) -> Vec<(Node, f64)> {
        let (x0, y0) = self.key(r.min);
        let (x1, y1) = self.key(r.max);
        let mut out = Vec::new();
        for bx in x0..=x1 {
            for by in y0..=y1 {
                for &(n, p) in self.buckets.get(&(bx, by)).into_iter().flatten() {
                    if r.contains(p) {
                        out.push((n, 0.0));
                    }
                }
            }
        }
        out
    }
}

/// Chooses one access point per pin and records its metal as the net's
/// own, so other nets keep clear of it from the start; the chosen point
/// becomes the pin's only one.
///
/// Pins with the fewest candidates choose first, each taking its best
/// candidate that clashes with nothing recorded before it. A pin with no
/// such candidate keeps all of them, except that a pin with a single one
/// still records it.
fn reserve_accesses(
    db: &NetlistDB,
    tech: &RouteTech,
    access: &mut [Vec<Vec<AccessPoint>>],
    occupancy: &mut Occupancy,
) {
    let mut order: Vec<(usize, usize, usize)> = access
        .iter()
        .enumerate()
        .flat_map(|(n, pins)| {
            pins.iter()
                .enumerate()
                .filter(|(_, c)| !c.is_empty())
                .map(move |(k, c)| (c.len(), n, k))
        })
        .collect();
    order.sort_unstable();
    for (count, n, k) in order {
        let owner = Owner::Net(crate::tech::to_u32(n));
        let metal_of = |a: &AccessPoint| {
            let mut route = NetRoute::default();
            emit_access(db, tech, a, &mut route);
            route.metal
        };
        let clear = access[n][k].iter().position(|a| {
            metal_of(a).iter().all(|(layer, rect)| {
                !tech.routable(*layer)
                    || occupancy
                        .rect_conflicts(tech, *layer, rect, owner)
                        .next()
                        .is_none()
            })
        });
        let chosen = match clear {
            Some(c) => c,
            None if count == 1 => 0,
            None => continue,
        };
        for (layer, rect) in metal_of(&access[n][k][chosen]) {
            if tech.routable(layer) {
                let _ = occupancy.add_rect(tech, layer, &rect, owner, Origin::Fixed);
            }
        }
        let picked = access[n][k].swap_remove(chosen);
        access[n][k] = vec![picked];
    }
}

/// Adds the wires and vias of a node path to `route`.
fn emit_path(db: &NetlistDB, tech: &RouteTech, path: &[Node], route: &mut NetRoute) {
    let point = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
    let mut start = 0;
    for i in 1..=path.len() {
        let breaks = i == path.len()
            || path[i].layer != path[start].layer
            || path[i].track != path[start].track;
        if !breaks {
            continue;
        }
        let run = &path[start..i];
        let b = path[i - 1];
        if let Some((first, last)) = span_of_run(run)
            && first != last
        {
            add_wire(db, b.layer, point(first), point(last), route);
        }
        if i < path.len() {
            let lower = path[i].layer.min(b.layer);
            add_via(db, tech.via_above(lower).via, point(b), route);
        }
        start = i;
    }
}

/// The nodes at either end of a run along one track. A run may double back
/// where two searches meet, so its ends are not always its first and last
/// nodes.
fn span_of_run(run: &[Node]) -> Option<(Node, Node)> {
    let lowest = run.iter().min_by_key(|n| n.cross)?;
    let highest = run.iter().max_by_key(|n| n.cross)?;
    Some((*lowest, *highest))
}

/// Adds a pin's access stub and via stack to `route`.
fn emit_access(db: &NetlistDB, tech: &RouteTech, a: &AccessPoint, route: &mut NetRoute) {
    let node = tech.layer(a.node.layer).point(a.node.track, a.node.cross);
    if node != a.at {
        add_wire(db, a.node.layer, node, a.at, route);
    }
    for &via in &a.vias {
        add_via(db, via, a.at, route);
    }
}

fn add_wire(
    db: &NetlistDB,
    layer: LayerId,
    from: Point<Dbu>,
    to: Point<Dbu>,
    route: &mut NetRoute,
) {
    route.elems.push(RouteElem::Wire { layer, from, to });
    if let Some(r) = wire_rect(from, to, db.library.layer(layer).width) {
        route.metal.push((layer, r));
    }
}

fn add_via(db: &NetlistDB, via: ViaId, at: Point<Dbu>, route: &mut NetRoute) {
    route.elems.push(RouteElem::Via { via, at });
    for s in &db.library.via(via).shapes {
        if let LayerRef::Routing(l) = s.layer {
            route.metal.push((l, s.rect.translate(at)));
        }
    }
}

/// Signal nets with at least two pins, shortest bounding box first.
pub fn routable_nets(db: &NetlistDB) -> Vec<NetId> {
    let mut nets: Vec<(i64, NetId)> = (0..db.nets.len())
        .map(NetId::new)
        .filter(|&n| is_signal_net(db, n))
        .map(|n| {
            let bbox = db.nets[n.index()]
                .pins
                .iter()
                .map(|&p| {
                    let at = db.pin_position(p);
                    Rect::new(at, at)
                })
                .reduce(|a, b| a.union(&b))
                .unwrap_or_default();
            ((bbox.width() + bbox.height()).get(), n)
        })
        .collect();
    nets.sort_unstable();
    nets.into_iter().map(|(_, n)| n).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(layer: u8, x: u16, y: u16) -> GCell {
        GCell {
            layer: LayerId::new(usize::from(layer)),
            x,
            y,
        }
    }

    #[test]
    fn waypoints_split_a_corridor_every_few_gcells_and_ignore_layer_changes() {
        let mut cells = vec![cell(1, 0, 0), cell(2, 0, 0)];
        cells.extend((1..=10).map(|x| cell(2, x, 0)));

        let stops = waypoints(&cells, 4);

        assert_eq!(stops, [5, 9]);
    }

    #[test]
    fn a_run_that_doubles_back_spans_its_farthest_crossings() {
        let node = |cross| Node {
            layer: LayerId::new(2),
            track: 7,
            cross,
        };
        let run = [node(5), node(9), node(6), node(3)];

        assert_eq!(span_of_run(&run), Some((node(3), node(9))));
    }

    #[test]
    fn a_short_corridor_has_no_waypoints() {
        let cells: Vec<GCell> = (0..=5).map(|x| cell(2, x, 0)).collect();

        assert!(waypoints(&cells, 4).is_empty());
    }
}
