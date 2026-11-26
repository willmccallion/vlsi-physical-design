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
mod reservation;
mod schedule;
mod tiles;
mod trace;
mod whole_net;

use reservation::{FixedJudge, FixedView, fixed_occupancy, own_access_points, reserve_accesses};
use whole_net::NetGuide;

pub use flow::{Flow, FlowState};
pub use region::{RepairStats, Scope};
pub(crate) use schedule::{RipUp, round_costs, round_strategy};
pub use trace::Trace;

use crate::access::{AccessPoint, Node};
use crate::global::{GCell, Gcells, GlobalGrid};
use crate::occupancy::{Occupancy, Origin, Owner, Proximity, Slot};
use crate::runs::EdgeSet;
pub use crate::search::Costs;
use crate::search::EdgeRules;
use crate::shared_access::SharedAccess;
use crate::tech::RouteTech;
use crate::track::TrackWire;
pub use crate::wiring::NetRoute;
use crate::wiring::{Edge, Patch, elem_cuts, elem_metal, gap_fills, path_edges};
use judge::ShapeTiles;
use pare_common::db::design::NetlistDB;
use pare_common::db::design::RouteElem;
use pare_common::db::indices::{IdVec, LayerId, NetId, PinId, ViaId};
use pare_common::db::library::LayerRef;
use pare_common::db::shapes::is_signal_net;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_verify::drc::{Layout, Rules};
use rayon::prelude::*;
use rustc_hash::FxHashSet;
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
    /// A saved via of the net does not join a node to the one right above
    /// it.
    #[error("the saved route of net {} has a via between nodes not one above the other", .0.index())]
    Via(NetId),
    /// A saved jog of the net leaves a crossing no jog may leave.
    #[error("the saved route of net {} has a jog where none may run", .0.index())]
    Jog(NetId),
}

/// How many nets' commits are worked out in parallel at a time, which
/// bounds the memory their wiring takes while they wait to be recorded.
const COMMIT_CHUNK: usize = 4096;

/// How many nets are routed speculatively at a time; rayon spreads each
/// batch over the threads. Fixed rather than the thread count, so that which
/// nets reroute against which, and so the routes, do not depend on the
/// machine; 28 keeps a 28-thread machine's batches. PARE tunable, no
/// `OpenROAD` counterpart.
const SPECULATIVE_BATCH: usize = 28;

/// Gcells a net's search window reaches past its guide or its pins' box.
/// PARE tunable, no `OpenROAD` counterpart.
const WINDOW_MARGIN_GCELLS: Gcells = Gcells(4);

/// Windows, each wider than the last, a net is searched in without its
/// guide once the guide holds no route. PARE tunable, no `OpenROAD`
/// counterpart.
const UNGUIDED_WINDOWS: usize = 2;

/// Gcells one connection's search reaches past the box of its two ends at
/// first. PARE tunable, no `OpenROAD` counterpart.
const CONNECT_MARGIN_GCELLS: Gcells = Gcells(2);

/// Factor a search margin grows by each time its window holds no route.
/// PARE tunable, no `OpenROAD` counterpart.
const MARGIN_GROWTH: i64 = 4;

/// A net's commit worked out and waiting to be recorded.
#[derive(Debug)]
struct PreparedCommit {
    net: NetId,
    route: NetRoute,
    elems: Vec<RouteElem>,
    metal: Vec<(LayerId, Rect<Dbu>)>,
    cuts: Vec<(pare_common::db::indices::CutLayerId, Rect<Dbu>)>,
    /// Where the net's wiring before this commit is recorded.
    previous: Vec<Slot>,
}

/// State shared by all detailed routing rounds.
#[derive(Debug)]
pub struct DetailedRouter<'a> {
    db: &'a NetlistDB,
    tech: &'a RouteTech,
    /// Occupancy of every track.
    pub occupancy: Occupancy,
    access: IdVec<NetId, Vec<Vec<AccessPoint>>>,
    /// Which edges around pins and obstructions are blocked to which nets.
    edge_rules: EdgeRules,
    routes: IdVec<NetId, NetRoute>,
    window_margin: Dbu,
    gcell: Dbu,
    grid_origin: Point<Dbu>,
    guides: IdVec<NetId, Option<NetGuide>>,
    wires: IdVec<NetId, Vec<TrackWire>>,
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
    elems: IdVec<NetId, Vec<RouteElem>>,
    /// Every violation found by the last round's judge.
    markers: Vec<Marker>,
    /// What to log in detail.
    trace: Trace,
}

/// Where a pin meets global routing: the gcell of its access point and the
/// layer of the pin shape that point reaches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct PinTerminal {
    /// The pin the terminal reaches.
    pub pin: PinId,
    /// The gcell, on the layer the access point's wire starts on.
    pub cell: GCell,
    /// The layer of the pin shape reached.
    pub pin_layer: LayerId,
}

impl PinTerminal {
    /// The terminal access point `a` gives `pin`.
    pub(crate) fn of(pin: PinId, a: &AccessPoint, tech: &RouteTech, grid: &GlobalGrid) -> Self {
        let p = tech.layer(a.node.layer).point(a.node.track, a.node.cross);
        Self {
            pin,
            cell: grid.cell_at(a.node.layer, p),
            pin_layer: a.pin_layer,
        }
    }
}

/// The rules and fixed shapes the wiring of an access point is judged
/// against.
#[derive(Debug)]
pub(crate) struct PinJudge {
    rules: Rules,
    fixed: Layout,
    fixed_tiles: ShapeTiles,
}

impl PinJudge {
    /// Whether `metal` of `net` on `layer` breaks no rule against the fixed
    /// shapes.
    pub fn clean(&self, net: NetId, layer: LayerId, metal: &[Rect<Dbu>]) -> bool {
        judge::wiring_is_clean(
            &self.fixed,
            &self.fixed_tiles,
            &self.rules,
            net,
            layer,
            metal,
        )
    }
}

/// This process's resident memory for a log line; "unknown" where the
/// kernel does not report it.
fn resident() -> String {
    pare_common::memory::current_rss_mb()
        .map_or_else(|| "unknown".to_string(), |mb| format!("{mb} MB"))
}

/// Runs `step`, logging `what` it did with how long it took and the
/// memory resident after.
fn timed<T>(what: &str, step: impl FnOnce() -> T) -> T {
    let start = std::time::Instant::now();
    let done = step();
    log::debug!(
        "{what} in {:.1}s, {} resident",
        start.elapsed().as_secs_f64(),
        resident()
    );
    done
}

/// Bytes the vectors of `nested` hold on the heap, with their headers.
fn nested_bytes<T>(nested: &IdVec<NetId, Vec<T>>) -> usize {
    nested
        .iter()
        .map(|v| v.capacity() * size_of::<T>())
        .sum::<usize>()
        + nested.len() * size_of::<Vec<T>>()
}

impl<'a> DetailedRouter<'a> {
    /// Logs the megabytes each record of the wiring holds.
    pub(crate) fn log_footprint(&self, stage: &str) {
        if !log::log_enabled!(log::Level::Debug) {
            return;
        }
        let mb = |bytes: usize| bytes / (1 << 20);
        let routes: usize = self
            .routes
            .iter()
            .map(|r| {
                r.edges.heap_bytes()
                    + r.patches.capacity() * size_of::<Patch>()
                    + r.access.capacity() * size_of::<AccessPoint>()
            })
            .sum();
        let (fixed, added, cuts) = self.occupancy.heap_bytes();
        log::debug!(
            "{stage} footprint: routes {} MB, elems {} MB, wires {} MB, \
             access {} MB, occupancy fixed {} MB, added {} MB, cuts {} MB, {} resident",
            mb(routes + self.routes.len() * size_of::<NetRoute>()),
            mb(nested_bytes(&self.elems)),
            mb(nested_bytes(&self.wires)),
            mb(nested_bytes(&self.access)),
            mb(fixed),
            mb(added),
            mb(cuts),
            resident()
        );
    }

    /// Prepares routing: computes pin access and records obstacles, pins
    /// and fixed cuts.
    pub fn new(db: &'a NetlistDB, tech: &'a RouteTech, gcell: Dbu) -> Self {
        let mut occupancy = fixed_occupancy(db, tech, gcell);
        let (rules, fixed, fixed_tiles) = timed("fixed layout built for the judge", || {
            let fixed = judge::fixed_layout(db);
            let tiles = ShapeTiles::new(&fixed, &db.die_area, gcell);
            (Rules::from_design(db), fixed, tiles)
        });
        let (shared, own) = timed("access points found", || {
            let shared = SharedAccess::new(db, tech);
            let judged = FixedJudge {
                layout: &fixed,
                tiles: &fixed_tiles,
                rules: &rules,
            };
            let own = own_access_points(db, tech, &shared, &judged);
            (shared, own)
        });
        log::debug!("{} kinds of cell share their access points", shared.kinds());
        let access = timed("access points reserved", || {
            let fixed_view = FixedView {
                layout: &fixed,
                tiles: &fixed_tiles,
                bucket: gcell,
            };
            reserve_accesses(db, tech, &shared, &own, &mut occupancy, &fixed_view)
        });
        drop(own);
        let edge_rules = timed("edge rules built", || {
            EdgeRules::new(db, tech, &access, gcell)
        });
        Self {
            db,
            tech,
            occupancy,
            access,
            edge_rules,
            routes: IdVec::filled(NetRoute::default(), db.nets.len()),
            window_margin: WINDOW_MARGIN_GCELLS.length(gcell),
            gcell,
            grid_origin: db.die_area.min,
            guides: IdVec::filled(None, db.nets.len()),
            wires: IdVec::filled(Vec::new(), db.nets.len()),
            halo: rules.max_reach() * 2,
            rules,
            fixed,
            fixed_tiles,
            elems: IdVec::filled(Vec::new(), db.nets.len()),
            markers: Vec::new(),
            trace: Trace::default(),
        }
    }

    /// Where each pin of each net is reached: the point of its best access
    /// point, by net.
    pub(crate) fn pin_points(&self) -> IdVec<NetId, Vec<Point<Dbu>>> {
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
    pub(crate) fn set_track_wires(&mut self, wires: Vec<TrackWire>) {
        for w in wires {
            self.wires[w.net].push(w);
        }
        let routes: Vec<(NetId, NetRoute)> = self
            .wires
            .iter_enumerated()
            .filter(|(_, wires)| !wires.is_empty())
            .map(|(n, wires)| {
                let route = NetRoute {
                    edges: EdgeSet::from_runs(wires.iter().map(|w| w.run())),
                    ..NetRoute::default()
                };
                (n, route)
            })
            .collect();
        self.commit_all(routes);
    }

    /// Logs every pin that has no access point, with its net.
    pub(crate) fn log_inaccessible_pins(&self) {
        for (n, pins) in self.access.iter_enumerated() {
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
    pub(crate) fn inaccessible_nets(&self) -> Vec<NetId> {
        self.access
            .iter_enumerated()
            .filter(|(_, pins)| pins.iter().any(Vec::is_empty))
            .map(|(n, _)| n)
            .collect()
    }

    /// Routes nets in parallel batches against the same occupancy, then
    /// commits them in order. A net whose wiring touches a net committed
    /// earlier in its batch is routed once more in the next batch; after
    /// that, conflicts are left to negotiation.
    fn route_speculatively(&mut self, nets: Vec<NetId>, costs: Costs) {
        let mut pending: VecDeque<NetId> = nets.into();
        let mut retried: FxHashSet<NetId> = FxHashSet::default();
        while !pending.is_empty() {
            let take = SPECULATIVE_BATCH.min(pending.len());
            let batch: Vec<NetId> = pending.drain(..take).collect();
            let results: Vec<(NetId, NetRoute)> = batch
                .par_iter()
                .map(|&n| (n, self.route_net(n, costs)))
                .collect();
            let mut committed: FxHashSet<NetId> = FxHashSet::default();
            let mut deferred = Vec::new();
            for (n, route) in results {
                if !retried.contains(&n) && self.touches_any(n, &route, &committed) {
                    let _ = retried.insert(n);
                    deferred.push(n);
                    continue;
                }
                self.commit(n, &route);
                let _ = committed.insert(n);
            }
            for n in deferred.into_iter().rev() {
                pending.push_front(n);
            }
        }
    }

    /// Whether `route` of `net` conflicts with the wiring of any net in `nets`.
    fn touches_any(&self, net: NetId, route: &NetRoute, nets: &FxHashSet<NetId>) -> bool {
        let owner = Owner::Net(net);
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
        let prepared = self.prepare_commit(net, route.clone());
        self.apply_commit(prepared);
    }

    /// Commits each net's route of `routes`, in their order: what each
    /// changes is worked out for many nets at once in parallel, then
    /// recorded one net after another as [`Self::commit`] records it; a
    /// chunk of them ends before a net it already holds, so that each is
    /// worked out from the wiring the commits before it left.
    fn commit_all(&mut self, routes: Vec<(NetId, NetRoute)>) {
        let mut routes = routes.into_iter().peekable();
        while routes.peek().is_some() {
            let mut held: FxHashSet<NetId> = FxHashSet::default();
            let mut chunk: Vec<(NetId, NetRoute)> = Vec::new();
            while chunk.len() < COMMIT_CHUNK
                && let Some(next) = routes.next_if(|(net, _)| !held.contains(net))
            {
                let _ = held.insert(next.0);
                chunk.push(next);
            }
            let prepared: Vec<PreparedCommit> = chunk
                .into_par_iter()
                .map(|(net, route)| self.prepare_commit(net, route))
                .collect();
            for p in prepared {
                self.apply_commit(p);
            }
        }
    }

    /// What committing `route` for `net` records, worked out without
    /// changing anything: the route with its narrow gaps filled, its
    /// wiring, metal and cuts, and where its present wiring is recorded.
    fn prepare_commit(&self, net: NetId, mut route: NetRoute) -> PreparedCommit {
        route
            .edges
            .extend(gap_fills(&route, self.db, self.tech, net));
        let mut elems = route.elems(self.tech);
        elems.shrink_to_fit();
        let metal = elems
            .iter()
            .flat_map(|e| elem_metal(self.db, net, e))
            .filter(|(layer, _)| self.tech.routable(*layer))
            .collect();
        let cuts = elems.iter().flat_map(|e| elem_cuts(self.db, e)).collect();
        PreparedCommit {
            net,
            route,
            elems,
            metal,
            cuts,
            previous: self.wiring_slots(net),
        }
    }

    /// The routable metal of the wiring `net` has now.
    fn wiring_metal(&self, net: NetId) -> Vec<(LayerId, Rect<Dbu>)> {
        self.elems[net]
            .iter()
            .flat_map(|e| elem_metal(self.db, net, e))
            .filter(|(layer, _)| self.tech.routable(*layer))
            .collect()
    }

    /// Where the wiring `net` has now is recorded in the occupancy.
    fn wiring_slots(&self, net: NetId) -> Vec<Slot> {
        let cuts: Vec<_> = self.elems[net]
            .iter()
            .flat_map(|e| elem_cuts(self.db, e))
            .collect();
        self.occupancy
            .slots_of(self.tech, Owner::Net(net), &self.wiring_metal(net), &cuts)
    }

    /// The vias of the wiring `net` has now.
    fn wiring_vias(&self, net: NetId) -> impl Iterator<Item = (ViaId, Point<Dbu>)> + '_ {
        self.elems[net].iter().filter_map(|e| match *e {
            RouteElem::Via { via, at } => Some((via, at)),
            RouteElem::Wire { .. } | RouteElem::Patch { .. } => None,
        })
    }

    /// Records a prepared commit: takes the net's previous wiring out of
    /// the occupancy and puts the new one in.
    fn apply_commit(&mut self, p: PreparedCommit) {
        let net = p.net;
        let owner = Owner::Net(net);
        self.occupancy.remove_wiring(&p.previous, owner);
        for (layer, rect) in &p.metal {
            self.occupancy
                .add_rect(self.tech, *layer, rect, owner, Origin::Wiring);
        }
        for (cut, rect) in &p.cuts {
            self.occupancy.add_cut(*cut, rect, owner, Origin::Wiring);
        }
        self.elems[net] = p.elems;
        self.routes[net] = p.route;
    }

    /// The current routing of every net.
    pub const fn routes(&self) -> &IdVec<NetId, NetRoute> {
        &self.routes
    }

    /// Every violation the last round's judge found, with the nets in it.
    fn markers(&self) -> &[Marker] {
        &self.markers
    }

    /// How many violations the last round's judge found.
    pub(crate) const fn marker_count(&self) -> usize {
        self.markers.len()
    }

    /// Logs every remaining violation at debug level.
    pub(crate) fn log_conflicts(&self) {
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
    pub(crate) fn rejudge(&mut self) -> Vec<NetId> {
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
    pub(crate) fn set_trace(&mut self, trace: Trace) {
        self.trace = trace;
    }

    /// Puts saved routes back, committing every route as routing it would
    /// have.
    ///
    /// # Errors
    ///
    /// When a route has a node off the tracks of its layer or an access
    /// point pin access does not offer its net now.
    pub(crate) fn restore(
        &mut self,
        routes: &[(NetId, NetRoute)],
    ) -> Result<(), RestoreRoutesError> {
        for (net, route) in routes {
            let nodes = route.edges.iter().flat_map(|e| <[Node; 2]>::from(e.ends()));
            if !nodes
                .chain(route.access.iter().map(|a| a.node))
                .all(|n| self.on_tracks(n))
            {
                return Err(RestoreRoutesError::OffTracks(*net));
            }
            if !route.edges.vias().iter().all(|&(l, u)| self.stacked(l, u)) {
                return Err(RestoreRoutesError::Via(*net));
            }
            if !route.edges.jogs().iter().all(|&n| self.tech.jog_allowed(n)) {
                return Err(RestoreRoutesError::Jog(*net));
            }
            let offered = |a: &AccessPoint| self.access[*net].iter().flatten().any(|o| o == a);
            if !route.access.iter().all(offered) {
                return Err(RestoreRoutesError::Access(*net));
            }
        }
        self.commit_all(routes.to_vec());
        Ok(())
    }

    /// Whether `n` is a crossing of a routable layer's tracks.
    fn on_tracks(&self, n: Node) -> bool {
        if !self.tech.routable(n.layer) {
            return false;
        }
        let lt = self.tech.layer(n.layer);
        n.track.index() < lt.tracks.len() && n.cross.index() < lt.cross.len()
    }

    /// Whether `upper` is the node right above `lower`: on the next layer up
    /// at the same point.
    fn stacked(&self, lower: Node, upper: Node) -> bool {
        upper.layer.index() == lower.layer.index() + 1 && self.point(lower) == self.point(upper)
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
            .map(|&n| self.db.nets[n].name.clone())
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
        self.access[net]
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

    /// Each pin of `net` with the gcell of its best access point and the
    /// layer of the pin shape that point reaches; none when a pin has no
    /// access point.
    pub(crate) fn pin_terminals(&self, net: NetId, grid: &GlobalGrid) -> Option<Vec<PinTerminal>> {
        self.access[net]
            .iter()
            .zip(&self.db.nets[net].pins)
            .map(|(c, &pin)| {
                let a = c.first()?;
                Some(PinTerminal::of(pin, a, self.tech, grid))
            })
            .collect()
    }

    /// The design rules and fixed shapes access points are judged against,
    /// kept once routing is over.
    pub(crate) fn into_pin_judge(self) -> PinJudge {
        PinJudge {
            rules: self.rules,
            fixed: self.fixed,
            fixed_tiles: self.fixed_tiles,
        }
    }

    fn point(&self, n: Node) -> Point<Dbu> {
        self.tech.layer(n.layer).point(n.track, n.cross)
    }

    /// The vias a path of crossings takes, with their locations.
    fn path_vias<'p>(&'p self, path: &'p [Node]) -> impl Iterator<Item = (ViaId, Point<Dbu>)> + 'p {
        path_edges(path).filter_map(|e| self.via_of(e))
    }

    /// The via `edge` places and where, if it is a via.
    fn via_of(&self, edge: Edge) -> Option<(ViaId, Point<Dbu>)> {
        match edge {
            Edge::Via { lower, .. } => {
                Some((self.tech.via_above(lower.layer).via, self.point(lower)))
            }
            Edge::Wire(_) | Edge::Jog(_) => None,
        }
    }
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

/// Signal nets with at least two pins, shortest bounding box first.
pub(crate) fn routable_nets(db: &NetlistDB) -> Vec<NetId> {
    let mut nets: Vec<(Dbu, NetId)> = (0..db.nets.len())
        .map(NetId::new)
        .filter(|&n| is_signal_net(db, n))
        .map(|n| {
            let bbox = db.nets[n]
                .pins
                .iter()
                .map(|&p| {
                    let at = db.pin_position(p);
                    Rect::new(at, at)
                })
                .reduce(|a, b| a.union(&b))
                .unwrap_or_default();
            (bbox.width() + bbox.height(), n)
        })
        .collect();
    nets.sort_unstable();
    nets.into_iter().map(|(_, n)| n).collect()
}

#[cfg(test)]
mod tests;
