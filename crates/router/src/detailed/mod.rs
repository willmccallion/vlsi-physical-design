//! Detailed routing on the track graph.
//!
//! The first pass routes every net region by region along its global route
//! (`initial`); conflicts left over are repaired region by region
//! (`region`), and once they are few, over the global routes of their nets
//! and on tiles around them (`tiles`), as `flow` decides round by round. A
//! net the regions cannot route is routed whole instead: a
//! tree grown through its track wires and its pins' access points, joining
//! the nearest remaining terminal each time with a multi-source A* search
//! inside its global route.

mod flow;
mod initial;
mod judge;
mod marking;
mod patch;
mod region;
mod schedule;
mod tiles;
mod trace;

pub use flow::{Flow, FlowState};
pub use region::{RepairStats, Scope};
pub use schedule::{RipUp, round_costs, round_strategy};
pub use trace::Trace;

use crate::access::{AccessPoint, Node, access_points};
use crate::global::{GCell, GlobalGrid, GlobalRoutes, RouteTree};
use crate::occupancy::{Occupancy, Origin, Owner, Proximity, Slot};
pub use crate::search::Costs;
use crate::search::{GUIDE_SLACK_GCELLS, Guide, Marks, OwnVias, Search};
use crate::tech::RouteTech;
use crate::track::TrackWire;
use crate::tree::{connection_order, gap, hilbert_order};
pub use crate::wiring::NetRoute;
use crate::wiring::{Edge, access_elems, elem_cuts, elem_metal, gap_fills, path_edges};
use judge::ShapeTiles;
use pare_common::db::design::NetlistDB;
use pare_common::db::design::RouteElem;
use pare_common::db::indices::{LayerId, NetId, NonDefaultRuleId, ViaId};
use pare_common::db::library::LayerRef;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::drc::{Layout, Rules};
use pare_common::verify::shapes::{fixed_shapes, is_signal_net};
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};
use std::collections::VecDeque;

/// Why saved detailed routes cannot be put back.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RestoreRoutesError {
    /// A saved route of the net uses a node that is not a track crossing.
    #[error("the saved route of net {} leaves the tracks", .0.index())]
    OffTracks(NetId),
    /// A saved route of the net reaches a pin by an access point pin access
    /// does not offer, as when the placement or pin access has changed.
    #[error("the saved route of net {} uses an access point no longer offered", .0.index())]
    Access(NetId),
}

/// State shared by all detailed routing rounds.
#[derive(Debug)]
pub struct DetailedRouter<'a> {
    db: &'a NetlistDB,
    tech: &'a RouteTech,
    /// Occupancy of every track.
    pub occupancy: Occupancy,
    access: Vec<Vec<Vec<AccessPoint>>>,
    routed: Vec<Vec<Slot>>,
    routes: Vec<NetRoute>,
    vias: Vec<Vec<(ViaId, Point<Dbu>)>>,
    window_margin: Dbu,
    gcell: Dbu,
    grid_origin: Point<Dbu>,
    guides: Vec<Option<NetGuide>>,
    wires: Vec<Vec<TrackWire>>,
    /// The design rules the judge applies.
    rules: Rules,
    /// The fixed shapes of the design, with every net an owner.
    fixed: Layout,
    fixed_tiles: ShapeTiles,
    /// How far beyond a region its judge looks: twice the farthest any rule
    /// reaches.
    halo: Dbu,
    /// The wiring of every net as the design will hold it, kept in step
    /// with `routes`.
    elems: Vec<Vec<RouteElem>>,
    /// Every violation found by the last round's judge.
    markers: Vec<Marker>,
    /// What to log in detail.
    trace: Trace,
}

impl<'a> DetailedRouter<'a> {
    /// Prepares routing: computes pin access and records obstacles, pins
    /// and fixed cuts.
    pub fn new(db: &'a NetlistDB, tech: &'a RouteTech, gcell: Dbu) -> Self {
        let start = std::time::Instant::now();
        let mut occupancy = Occupancy::new(tech, gcell);
        occupancy.add_fixed(tech, fixed_shapes(db));
        log::debug!(
            "fixed shapes recorded in {:.1}s, {} MB resident",
            start.elapsed().as_secs_f64(),
            pare_common::util::memory::current_rss_mb().unwrap_or(0)
        );
        let start = std::time::Instant::now();
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
        log::debug!(
            "access points found in {:.1}s, {} MB resident",
            start.elapsed().as_secs_f64(),
            pare_common::util::memory::current_rss_mb().unwrap_or(0)
        );
        let start = std::time::Instant::now();
        reserve_accesses(db, tech, &mut access, &mut occupancy);
        log::debug!(
            "access points reserved in {:.1}s, {} MB resident",
            start.elapsed().as_secs_f64(),
            pare_common::util::memory::current_rss_mb().unwrap_or(0)
        );
        let start = std::time::Instant::now();
        let rules = Rules::from_design(db);
        let fixed = Layout::fixed_from_db(db);
        let fixed_tiles = ShapeTiles::new(&fixed, &db.die_area, gcell);
        log::debug!(
            "fixed layout built for the judge in {:.1}s, {} MB resident",
            start.elapsed().as_secs_f64(),
            pare_common::util::memory::current_rss_mb().unwrap_or(0)
        );
        Self {
            db,
            tech,
            occupancy,
            access,
            routed: vec![Vec::new(); db.nets.len()],
            routes: vec![NetRoute::default(); db.nets.len()],
            vias: vec![Vec::new(); db.nets.len()],
            window_margin: gcell * 4,
            gcell,
            grid_origin: db.die_area.min,
            guides: vec![None; db.nets.len()],
            wires: vec![Vec::new(); db.nets.len()],
            halo: rules.max_reach() * 2,
            rules,
            fixed,
            fixed_tiles,
            elems: vec![Vec::new(); db.nets.len()],
            markers: Vec::new(),
            trace: Trace::default(),
        }
    }

    /// Where each pin of each net is reached: the point of its best access
    /// point, by net.
    pub fn pin_points(&self) -> Vec<Vec<Point<Dbu>>> {
        self.access
            .iter()
            .map(|pins| {
                pins.iter()
                    .filter_map(|c| c.first().map(|a| self.point(a.node)))
                    .collect()
            })
            .collect()
    }

    /// Records the wires of track assignment as their nets' first wiring;
    /// routing then joins each net's pins through them.
    pub fn set_track_wires(&mut self, wires: Vec<TrackWire>) {
        for w in wires {
            self.wires[w.net.index()].push(w);
        }
        for n in 0..self.wires.len() {
            if self.wires[n].is_empty() {
                continue;
            }
            let mut edges: Vec<Edge> = self.wires[n].iter().flat_map(|w| w.edges()).collect();
            edges.sort_unstable();
            edges.dedup();
            let route = NetRoute {
                edges,
                ..NetRoute::default()
            };
            self.commit(NetId::new(n), &route);
        }
    }

    /// Logs every pin that has no access point, with its net.
    pub fn log_inaccessible_pins(&self) {
        for (n, pins) in self.access.iter().enumerate() {
            for (k, points) in pins.iter().enumerate() {
                if points.is_empty() {
                    let net = &self.db.nets[n];
                    log::warn!(
                        "no access to pin {} of net {}",
                        self.db.pin_name(net.pins[k]),
                        net.name
                    );
                }
            }
        }
    }

    /// Nets that have a pin with no access point.
    pub fn inaccessible_nets(&self) -> Vec<NetId> {
        (0..self.access.len())
            .filter(|&n| self.access[n].iter().any(Vec::is_empty))
            .map(NetId::new)
            .collect()
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
                .map(|&n| (n, self.route_net(n, costs)))
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
        let owner = Owner::Net(net.0);
        let listed = |o: Owner| matches!(o, Owner::Net(other) if nets.contains(&other));
        let metal = route
            .metal(self.db, self.tech, net)
            .iter()
            .any(|(layer, rect)| {
                self.tech.routable(*layer)
                    && self
                        .occupancy
                        .rect_conflicts(self.tech, *layer, rect, owner)
                        .any(|c| c.proximity == Proximity::Within && listed(c.occupant.owner))
            });
        metal
            || route.cuts(self.db, self.tech).iter().any(|(cut, rect)| {
                self.occupancy
                    .cut_conflicts(*cut, rect, owner)
                    .any(|o| listed(o.owner))
            })
    }

    /// Replaces a net's recorded routing with `route`, its narrow gaps
    /// along tracks filled.
    fn commit(&mut self, net: NetId, route: &NetRoute) {
        let mut route = route.clone();
        route
            .edges
            .extend(gap_fills(&route, self.db, self.tech, net));
        route.edges.sort_unstable();
        route.edges.dedup();
        let route = &route;
        let previous = std::mem::take(&mut self.routed[net.index()]);
        self.occupancy.remove_wiring(&previous, Owner::Net(net.0));
        let owner = Owner::Net(net.0);
        let mut touched = Vec::new();
        for (layer, rect) in route.metal(self.db, self.tech, net) {
            if self.tech.routable(layer) {
                let tracks =
                    self.occupancy
                        .add_rect(self.tech, layer, &rect, owner, Origin::Wiring);
                touched.extend(tracks.into_iter().map(Slot::Track));
            }
        }
        for (cut, rect) in route.cuts(self.db, self.tech) {
            let bucket = self.occupancy.add_cut(cut, &rect, owner, Origin::Wiring);
            touched.push(Slot::Cut(bucket));
        }
        touched.sort_unstable();
        touched.dedup();
        self.routed[net.index()] = touched;
        self.vias[net.index()] = route.vias(self.tech);
        self.elems[net.index()] = route.elems(self.tech);
        self.routes[net.index()] = route.clone();
    }

    /// The current routing of every net.
    pub fn routes(&self) -> &[NetRoute] {
        &self.routes
    }

    /// Every violation the last round's judge found, with the nets in it.
    fn markers(&self) -> &[Marker] {
        &self.markers
    }

    /// How many violations the last round's judge found.
    pub const fn marker_count(&self) -> usize {
        self.markers.len()
    }

    /// Logs every remaining violation at debug level.
    pub fn log_conflicts(&self) {
        for m in &self.markers {
            log::debug!(
                "violation at ({}, {}) on {:?} between {:?} and {:?}",
                m.at.x.get(),
                m.at.y.get(),
                m.layer,
                m.nets[0],
                m.nets[1]
            );
        }
    }

    /// Judges the whole design and returns the nets of its violations.
    pub fn rejudge(&mut self) -> Vec<NetId> {
        let (markers, counts) = self.exact_markers();
        for (kind, count) in &counts {
            log::debug!("round markers: {} {}", kind.name(), count);
        }
        for m in &markers {
            self.trace_marker(m);
        }
        let mut nets: Vec<NetId> = markers
            .iter()
            .flat_map(|m| m.nets.iter().flatten().copied())
            .collect();
        nets.sort_unstable();
        nets.dedup();
        self.markers = markers;
        nets
    }

    /// Logs what `trace` asks for from now on.
    pub fn set_trace(&mut self, trace: Trace) {
        self.trace = trace;
    }

    /// Puts saved routes back, committing every route as routing it would
    /// have.
    ///
    /// # Errors
    ///
    /// When a route has a node off the tracks of its layer or an access
    /// point pin access does not offer its net now.
    pub fn restore(&mut self, routes: &[(NetId, NetRoute)]) -> Result<(), RestoreRoutesError> {
        for (net, route) in routes {
            let nodes = route.edges.iter().flat_map(|e| match *e {
                Edge::Wire(n) => [n, n],
                Edge::Via { lower, upper } => [lower, upper],
            });
            if !nodes
                .chain(route.access.iter().map(|a| a.node))
                .all(|n| self.on_tracks(n))
            {
                return Err(RestoreRoutesError::OffTracks(*net));
            }
            let offered =
                |a: &AccessPoint| self.access[net.index()].iter().flatten().any(|o| o == a);
            if !route.access.iter().all(offered) {
                return Err(RestoreRoutesError::Access(*net));
            }
        }
        for (net, route) in routes {
            self.commit(*net, route);
        }
        Ok(())
    }

    /// Whether `n` is a crossing of a routable layer's tracks.
    fn on_tracks(&self, n: Node) -> bool {
        if !self.tech.routable(n.layer) {
            return false;
        }
        let lt = self.tech.layer(n.layer);
        (n.track as usize) < lt.tracks.len() && (n.cross as usize) < lt.cross.len()
    }

    /// Logs a violation of a traced net or in the traced area.
    fn trace_marker(&self, m: &Marker) {
        let at = Rect::new(m.at, m.at);
        let traced =
            m.nets.iter().flatten().any(|&n| self.trace.wants(n, &at)) || self.trace.meets(&at);
        if !traced {
            return;
        }
        let names: Vec<String> = m
            .nets
            .iter()
            .flatten()
            .map(|n| self.db.nets[n.index()].name.clone())
            .collect();
        log::info!(
            "trace marker {:?} at {} between {}",
            m.layer,
            trace::region_text(&m.bbox),
            names.join(" and ")
        );
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
            return guide.bbox.expand(self.gcell * GUIDE_SLACK_GCELLS);
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

    /// Routes a net through its track wires and pins, or through its pins
    /// alone when the wires cannot all be joined.
    fn route_net(&self, net: NetId, costs: Costs) -> NetRoute {
        let pins = &self.access[net.index()];
        if pins.iter().any(Vec::is_empty) || pins.len() < 2 {
            return NetRoute::default();
        }
        let wires = self.wires[net.index()].as_slice();
        if !wires.is_empty() {
            let terms = Terminals { pins, wires };
            if let Some(route) = self.route_terminals(net, costs, &terms) {
                return route;
            }
        }
        let terms = Terminals { pins, wires: &[] };
        self.route_terminals(net, costs, &terms).unwrap_or_default()
    }

    /// Joins every terminal inside the net's guide, or failing that in ever
    /// larger windows without it.
    fn route_terminals(&self, net: NetId, costs: Costs, terms: &Terminals<'_>) -> Option<NetRoute> {
        let net_guide = self.guides[net.index()].as_ref();
        let guide =
            net_guide.map(|g| Guide::new(g.tree.cells(), self.grid_origin, self.gcell, self.tech));
        let unmarked = Marks::default();
        let mut search = Search {
            db: self.db,
            tech: self.tech,
            occupancy: &self.occupancy,
            overlay: None,
            marks: &unmarked,
            owner: Owner::Net(net.0),
            costs,
            guide: guide.as_ref(),
            own_vias: None,
            ripped: &[],
        };
        let window = self.window(net);
        if let Some(route) = self.route_tree(&search, terms, &window, net_guide) {
            return Some(route);
        }
        search.guide = None;
        let mut window = window.expand(self.window_margin);
        for _ in 0..2 {
            if let Some(route) = self.route_tree(&search, terms, &window, None) {
                return Some(route);
            }
            window = window.expand(self.window_margin * 4);
        }
        None
    }

    /// Connects every terminal, searching first along the branches of
    /// `guide`'s global route when given. Nets with more than
    /// [`CHUNKED_PINS`] terminals are split into runs of nearby terminals
    /// whose subtrees are routed in parallel and then joined.
    fn route_tree(
        &self,
        search: &Search<'_>,
        terms: &Terminals<'_>,
        window: &Rect<Dbu>,
        guide: Option<&NetGuide>,
    ) -> Option<NetRoute> {
        let all: Vec<usize> = (0..terms.len()).collect();
        let tree = if terms.len() > CHUNKED_PINS {
            self.grow_chunked(search, terms, window, guide)?
        } else {
            self.grow(search, terms, &all, window, guide)?
        };
        let mut edges: Vec<Edge> = tree.paths.iter().flat_map(|p| path_edges(p)).collect();
        edges.extend(terms.wires.iter().flat_map(|w| w.edges()));
        edges.sort_unstable();
        edges.dedup();
        Some(NetRoute {
            edges,
            patches: Vec::new(),
            access: tree
                .used
                .iter()
                .map(|&(pin, choice)| terms.pins[pin][choice].clone())
                .collect(),
            complete: true,
        })
    }

    /// Joins the `members` terminals one at a time, each to the nearest
    /// member joined before it.
    fn grow(
        &self,
        search: &Search<'_>,
        terms: &Terminals<'_>,
        members: &[usize],
        window: &Rect<Dbu>,
        guide: Option<&NetGuide>,
    ) -> Option<SubTree> {
        let extents: Vec<Rect<Dbu>> = members
            .iter()
            .map(|&m| terms.extent(self.tech, m))
            .collect();
        let (first, joins) = connection_order(&extents);
        let mut tree = TreeIndex::new(self.grid_origin, self.gcell);
        let mut out = SubTree::default();
        let first_term = members[first];
        if joins.is_empty() {
            self.join_terminal(terms, first_term, None, &mut tree, &mut out);
            out.nodes = tree.nodes.into_iter().collect();
            return Some(out);
        }
        let first_sources: Vec<(Node, f64)> = terms.candidates(search, first_term);
        let guide = guide.filter(|_| terms.wires.is_empty());
        let mut own = self.reserved_access_vias(terms, search.rule());

        for join in joins {
            let term = members[join.pin];
            let candidates = terms.candidates(search, term);
            if let Some(&(n, _)) = candidates.iter().find(|(n, _)| tree.contains(*n)) {
                self.join_terminal(terms, term, Some(n), &mut tree, &mut out);
                continue;
            }
            let targets: FxHashMap<Node, (usize, f64)> = candidates
                .iter()
                .enumerate()
                .map(|(k, &(n, cost))| (n, (k, cost)))
                .collect();
            let starting = tree.is_empty();
            let sources_in = |local: &Rect<Dbu>| {
                if starting {
                    first_sources.clone()
                } else {
                    tree.nodes_in(local)
                }
            };
            let corridor = guide.and_then(|g| g.corridor(members[join.parent], term));
            let joining = Search {
                own_vias: Some(&own),
                ..*search
            };
            let (path, _) = self.connect(
                &joining,
                closest_points(&extents[join.pin], &extents[join.parent]),
                corridor.as_deref(),
                window,
                sources_in,
                &targets,
            )?;
            for (via, at) in self.path_vias(&path) {
                own.add(via, at);
            }
            for &n in &path {
                tree.insert(n, self.point(n));
            }
            if starting {
                self.join_terminal(
                    terms,
                    first_term,
                    path.first().copied(),
                    &mut tree,
                    &mut out,
                );
            }
            self.join_terminal(terms, term, path.last().copied(), &mut tree, &mut out);
            out.paths.push(path);
        }
        out.nodes = tree.nodes.into_iter().collect();
        Some(out)
    }

    /// Records terminal `term` as joined at node `at`: a pin keeps the
    /// access point there (its best one when `at` is `None`), a track wire
    /// joins the tree along its whole length.
    fn join_terminal(
        &self,
        terms: &Terminals<'_>,
        term: usize,
        at: Option<Node>,
        tree: &mut TreeIndex,
        out: &mut SubTree,
    ) {
        match terms.kind(term) {
            Terminal::Pin(pin) => {
                let choices = &terms.pins[pin];
                let k = at
                    .and_then(|n| choices.iter().position(|a| a.node == n))
                    .unwrap_or(0);
                out.used.push((pin, k));
                if let Some(a) = choices.get(k) {
                    tree.insert(a.node, self.point(a.node));
                }
            }
            Terminal::Wire(wire) => {
                for n in wire.nodes() {
                    tree.insert(n, self.point(n));
                }
            }
        }
    }

    /// The nodes of `sub` near `rep`, or all of them if none is; a small
    /// target area keeps the search's distance estimate sharp.
    fn join_targets(&self, sub: &SubTree, rep: &Rect<Dbu>) -> FxHashMap<Node, (usize, f64)> {
        let reach = (self.gcell * JOIN_GCELLS).get();
        let near: FxHashMap<Node, (usize, f64)> = sub
            .nodes
            .iter()
            .filter(|&&n| {
                let p = self.point(n);
                gap(&Rect::new(p, p), rep) <= reach
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

    /// Routes runs of nearby terminals in parallel, then joins their subtrees.
    fn grow_chunked(
        &self,
        search: &Search<'_>,
        terms: &Terminals<'_>,
        window: &Rect<Dbu>,
        guide: Option<&NetGuide>,
    ) -> Option<SubTree> {
        let extents: Vec<Rect<Dbu>> = (0..terms.len())
            .map(|t| terms.extent(self.tech, t))
            .collect();
        let centres: Vec<Point<Dbu>> = extents.iter().map(Rect::center).collect();
        let order = hilbert_order(&centres);
        let chunks: Vec<&[usize]> = order.chunks(CHUNK_PINS).collect();
        let subtrees: Vec<SubTree> = chunks
            .par_iter()
            .map(|members| self.grow(search, terms, members, window, guide))
            .collect::<Option<Vec<_>>>()?;

        let reps: Vec<Rect<Dbu>> = chunks.iter().map(|c| extents[c[0]]).collect();
        let (first, joins) = connection_order(&reps);
        let mut tree = TreeIndex::new(self.grid_origin, self.gcell);
        for &n in &subtrees[first].nodes {
            tree.insert(n, self.point(n));
        }
        let guide = guide.filter(|_| terms.wires.is_empty());
        let mut own = self.reserved_access_vias(terms, search.rule());
        for sub in &subtrees {
            for (via, at) in sub.paths.iter().flat_map(|p| self.path_vias(p)) {
                own.add(via, at);
            }
        }
        let mut out = SubTree::default();
        for join in joins {
            let sub = &subtrees[join.pin];
            if !sub.nodes.iter().any(|&n| tree.contains(n)) {
                let targets = self.join_targets(sub, &reps[join.pin]);
                let corridor =
                    guide.and_then(|g| g.corridor(chunks[join.parent][0], chunks[join.pin][0]));
                let joining = Search {
                    own_vias: Some(&own),
                    ..*search
                };
                let (path, _) = self.connect(
                    &joining,
                    closest_points(&reps[join.pin], &reps[join.parent]),
                    corridor.as_deref(),
                    window,
                    |r| tree.nodes_in(r),
                    &targets,
                )?;
                for &n in &path {
                    tree.insert(n, self.point(n));
                }
                for (via, at) in self.path_vias(&path) {
                    own.add(via, at);
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

    /// The vias a path of crossings takes, with their locations.
    fn path_vias<'p>(&'p self, path: &'p [Node]) -> impl Iterator<Item = (ViaId, Point<Dbu>)> + 'p {
        path_edges(path).filter_map(|e| match e {
            Edge::Via { lower, .. } => {
                Some((self.tech.via_above(lower.layer).via, self.point(lower)))
            }
            Edge::Wire(_) => None,
        })
    }

    /// The access vias of the pins among `terms` whose access point is
    /// reserved, as the start of a net's own vias.
    fn reserved_access_vias(
        &self,
        terms: &Terminals<'_>,
        rule: Option<NonDefaultRuleId>,
    ) -> OwnVias {
        let mut own = OwnVias::new(self.db, self.tech, rule);
        for choices in terms.pins {
            if let [a] = choices.as_slice() {
                for &via in &a.vias {
                    own.add(via, a.at);
                }
            }
        }
        own
    }
}

impl NetGuide {
    /// The gcells of the global route from pin `from` to pin `to`.
    fn corridor(&self, from: usize, to: usize) -> Option<Vec<GCell>> {
        self.tree.branch(self.terminals[from], self.terminals[to])
    }
}

/// The pins and track wires a net's tree joins, numbered pins first.
#[derive(Clone, Copy, Debug)]
struct Terminals<'p> {
    pins: &'p [Vec<AccessPoint>],
    wires: &'p [TrackWire],
}

/// One terminal of a net.
#[derive(Clone, Copy, Debug)]
enum Terminal {
    /// A pin, by index, reached through one of its access points.
    Pin(usize),
    /// A track wire, reached anywhere along it.
    Wire(TrackWire),
}

impl Terminals<'_> {
    const fn len(&self) -> usize {
        self.pins.len() + self.wires.len()
    }

    const fn kind(&self, term: usize) -> Terminal {
        match term.checked_sub(self.pins.len()) {
            None => Terminal::Pin(term),
            Some(w) => Terminal::Wire(self.wires[w]),
        }
    }

    /// Where the terminal lies: its best access point, or its wire's span.
    fn extent(&self, tech: &RouteTech, term: usize) -> Rect<Dbu> {
        let point = |n: Node| tech.layer(n.layer).point(n.track, n.cross);
        match self.kind(term) {
            Terminal::Pin(pin) => {
                let p = self.pins[pin]
                    .first()
                    .map(|a| point(a.node))
                    .unwrap_or_default();
                Rect::new(p, p)
            }
            Terminal::Wire(w) => Rect::from_corners(
                tech.layer(w.layer).point(w.track, w.from),
                tech.layer(w.layer).point(w.track, w.to),
            ),
        }
    }

    /// The nodes that join the terminal, each with the cost of joining there.
    fn candidates(&self, search: &Search<'_>, term: usize) -> Vec<(Node, f64)> {
        match self.kind(term) {
            Terminal::Pin(pin) => self.pins[pin]
                .iter()
                .map(|a| (a.node, search.access_cost(a)))
                .collect(),
            Terminal::Wire(w) => w.nodes().map(|n| (n, 0.0)).collect(),
        }
    }
}

/// The closest points of two rectangles, the first on `a`.
fn closest_points(a: &Rect<Dbu>, b: &Rect<Dbu>) -> (Point<Dbu>, Point<Dbu>) {
    let axis = |a0: Dbu, a1: Dbu, b0: Dbu, b1: Dbu| {
        if a1 < b0 {
            (a1, b0)
        } else if b1 < a0 {
            (a0, b1)
        } else {
            let m = a0.max(b0).midpoint(a1.min(b1));
            (m, m)
        }
    };
    let (ax, bx) = axis(a.min.x, a.max.x, b.min.x, b.max.x);
    let (ay, by) = axis(a.min.y, a.max.y, b.min.y, b.max.y);
    (Point::new(ax, ay), Point::new(bx, by))
}

/// A violation's location, layer and the nets in it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Marker {
    at: Point<Dbu>,
    /// The violation's extent.
    bbox: Rect<Dbu>,
    layer: LayerRef,
    nets: [Option<NetId>; 2],
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
/// candidate that clashes with nothing recorded before it and whose vias,
/// counting the via up from its node, leave room for those of the net's
/// pins that chose earlier. A pin with no such candidate keeps all of
/// them, except that a pin with a single one still records it.
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
    let mut reserved: Vec<OwnVias> = (0..access.len())
        .map(|n| OwnVias::new(db, tech, tech.net_rule(n)))
        .collect();
    for (count, n, k) in order {
        let owner = Owner::Net(crate::tech::to_u32(n));
        let metal_of = |a: &AccessPoint| {
            access_elems(tech, a)
                .iter()
                .flat_map(|e| elem_metal(db, NetId::new(n), e))
                .collect::<Vec<_>>()
        };
        let cuts_of = |a: &AccessPoint| {
            access_elems(tech, a)
                .iter()
                .flat_map(|e| elem_cuts(db, e))
                .collect::<Vec<_>>()
        };
        let clear = access[n][k].iter().position(|a| {
            let leaves_room = footprint(tech, a)
                .iter()
                .all(|&(via, at)| reserved[n].crowded(db, tech, via, at) == 0);
            let metal_clear = metal_of(a).iter().all(|(layer, rect)| {
                !tech.routable(*layer)
                    || occupancy
                        .rect_conflicts(tech, *layer, rect, owner)
                        .all(|c| c.proximity != Proximity::Within)
            });
            leaves_room
                && metal_clear
                && cuts_of(a)
                    .iter()
                    .all(|(cut, rect)| occupancy.cut_conflicts(*cut, rect, owner).next().is_none())
        });
        let chosen = match clear {
            Some(c) => c,
            None if count == 1 => 0,
            None => {
                log::debug!(
                    "net {} pin {k}: none of {count} access points is clear, leaving the choice to routing",
                    db.nets[n].name
                );
                continue;
            }
        };
        log::trace!(
            "net {} pin {k}: reserved access point {chosen} of {count} at {:?}",
            db.nets[n].name,
            access[n][k][chosen].at
        );
        for (layer, rect) in metal_of(&access[n][k][chosen]) {
            if tech.routable(layer) {
                let _ = occupancy.add_rect(tech, layer, &rect, owner, Origin::Fixed);
            }
        }
        for (cut, rect) in cuts_of(&access[n][k][chosen]) {
            let _ = occupancy.add_cut(cut, &rect, owner, Origin::Fixed);
        }
        let picked = access[n][k].swap_remove(chosen);
        for (via, at) in footprint(tech, &picked) {
            reserved[n].add(via, at);
        }
        access[n][k] = vec![picked];
    }
}

/// The vias an access point commits its net to: its via stack, and the via
/// up from its node that a route leaving the pin upward would take.
fn footprint(tech: &RouteTech, a: &AccessPoint) -> Vec<(ViaId, Point<Dbu>)> {
    let node = tech.layer(a.node.layer).point(a.node.track, a.node.cross);
    let up = (a.node.layer < tech.max_layer).then(|| (tech.via_above(a.node.layer).via, node));
    a.vias.iter().map(|&v| (v, a.at)).chain(up).collect()
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
    fn a_short_corridor_has_no_waypoints() {
        let cells: Vec<GCell> = (0..=5).map(|x| cell(2, x, 0)).collect();

        assert!(waypoints(&cells, 4).is_empty());
    }
}
