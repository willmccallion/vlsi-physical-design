//! Routing a net whole: a tree grown through its track wires and access points, joining the nearest remaining terminal each time.

use super::{CONNECT_MARGIN_GCELLS, DetailedRouter, MARGIN_GROWTH, UNGUIDED_WINDOWS};
use crate::access::{AccessPoint, Node};
use crate::global::{GCell, Gcells, GlobalGrid, GlobalRoutes, RouteTree};
use crate::occupancy::Owner;
use crate::runs::EdgeSet;
use crate::search::Costs;
use crate::search::{GUIDE_SLACK_GCELLS, Guide, Marks, OwnVias, Search};
use crate::tech::RouteTech;
use crate::track::TrackWire;
use crate::tree::{connection_order, gap, hilbert_order};
use crate::wiring::NetRoute;
use crate::wiring::path_edges;
use pare_common::db::indices::{LayerId, NetId, NonDefaultRuleId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::Interval;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use rustc_hash::{FxHashMap, FxHashSet};

impl DetailedRouter<'_> {
    /// Restricts each net's detailed route to its global route.
    pub(crate) fn set_guides(&mut self, routes: GlobalRoutes, grid: &GlobalGrid) {
        for (net, placed) in routes.trees {
            let tree = RouteTree::new(placed.paths());
            let bbox = tree
                .cells()
                .iter()
                .map(|c| grid.cell_rect(c.x, c.y))
                .reduce(|a, b| a.union(&b));
            if let (Some(bbox), Some(terminals)) = (bbox, self.terminals(net, grid)) {
                self.guides[net] = Some(NetGuide {
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
        if let Some(guide) = &self.guides[net] {
            return guide.bbox.expand(GUIDE_SLACK_GCELLS.length(self.gcell));
        }
        let pins = &self.db.nets[net].pins;
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
    pub(super) fn route_net(&self, net: NetId, costs: Costs) -> NetRoute {
        let pins = &self.access[net];
        if pins.iter().any(Vec::is_empty) || pins.len() < 2 {
            return NetRoute::default();
        }
        let wires = self.wires[net].as_slice();
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
        let net_guide = self.guides[net].as_ref();
        let guide = net_guide
            .and_then(|g| Guide::new(g.tree.cells(), self.grid_origin, self.gcell, self.tech));
        let unmarked = Marks::default();
        let mut search = Search {
            db: self.db,
            tech: self.tech,
            occupancy: &self.occupancy,
            overlay: None,
            marks: &unmarked,
            owner: Owner::Net(net),
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
        for _ in 0..UNGUIDED_WINDOWS {
            if let Some(route) = self.route_tree(&search, terms, &window, None) {
                return Some(route);
            }
            window = window.expand(self.window_margin * MARGIN_GROWTH);
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
        let mut edges: EdgeSet = tree.paths.iter().flat_map(|p| path_edges(p)).collect();
        edges.union(&EdgeSet::from_runs(terms.wires.iter().map(|w| w.run())));
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
            let ends = PathEnds {
                first: starting.then_some(first_term),
                last: term,
            };
            self.add_path(terms, path, ends, &mut tree, &mut out);
        }
        out.nodes = tree.nodes.into_iter().collect();
        Some(out)
    }

    /// Adds `path` to the tree, joining terminal `ends.last` at its last
    /// node and, when the tree was empty, `ends.first` at its first.
    fn add_path(
        &self,
        terms: &Terminals<'_>,
        path: Vec<Node>,
        ends: PathEnds,
        tree: &mut TreeIndex,
        out: &mut SubTree,
    ) {
        for &n in &path {
            tree.insert(n, self.point(n));
        }
        if let Some(first) = ends.first {
            self.join_terminal(terms, first, path.first().copied(), tree, out);
        }
        self.join_terminal(terms, ends.last, path.last().copied(), tree, out);
        out.paths.push(path);
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
        let reach = JOIN_GCELLS.length(self.gcell);
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
        let mut margin = CONNECT_MARGIN_GCELLS.length(self.gcell);
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
            margin = margin * MARGIN_GROWTH;
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
            let (Interval { lo: a0, hi: a1 }, Interval { lo: b0, hi: b1 }) =
                lt.across_along_rect(&area);
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

/// Indices of the cells where a long corridor is split: one about every
/// `every` steps between gcells, none closer than half that to the end.
pub(super) fn waypoints(cells: &[GCell], every: usize) -> Vec<usize> {
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
pub(super) struct NetGuide {
    pub(super) tree: RouteTree,
    pub(super) terminals: Vec<GCell>,
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
const JOIN_GCELLS: Gcells = Gcells(2);

/// The paths of a tree over some pins, the access point used for each pin,
/// and every node it covers.
#[derive(Debug, Default)]
struct SubTree {
    paths: Vec<Vec<Node>>,
    used: Vec<(usize, usize)>,
    nodes: Vec<Node>,
}

/// The terminals a path joins: the one at its end, and the one at its
/// start where the path is the tree's first.
#[derive(Clone, Copy, Debug)]
struct PathEnds {
    first: Option<usize>,
    last: usize,
}

/// A tree node and its location.
type NodeAt = (Node, Point<Dbu>);

/// The nodes of a growing tree, bucketed by gcell to find those in a window.
struct TreeIndex {
    origin: Point<Dbu>,
    gcell: Dbu,
    nodes: FxHashSet<Node>,
    buckets: FxHashMap<Point<Gcells>, Vec<NodeAt>>,
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

    fn key(&self, p: Point<Dbu>) -> Point<Gcells> {
        Point::new(
            Gcells::holding(p.x - self.origin.x, self.gcell),
            Gcells::holding(p.y - self.origin.y, self.gcell),
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
        let (low, high) = (self.key(r.min), self.key(r.max));
        let mut out = Vec::new();
        for x in low.x.through(high.x) {
            for y in low.y.through(high.y) {
                for &(n, p) in self.buckets.get(&Point::new(x, y)).into_iter().flatten() {
                    if r.contains(p) {
                        out.push((n, 0.0));
                    }
                }
            }
        }
        out
    }
}
