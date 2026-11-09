//! Global and detailed routing on the design's tracks.
//!
//! Global routing finds a corridor of gcells for every net; track assignment
//! puts the corridor's long straight runs on tracks; detailed routing then
//! joins those wires and the pins with wires on tracks and vias at track
//! crossings inside the corridors, repairing conflicts until no two nets
//! touch or come closer than the layer spacing.

pub mod access;
pub mod detailed;
pub mod global;
pub mod occupancy;
pub mod runs;
mod search;
pub mod session;
pub mod shared_access;
pub mod state;
pub mod tech;
pub mod track;
pub mod tree;
pub mod wiring;

use detailed::{
    DetailedRouter, Flow, FlowState, RepairStats, RestoreRoutesError, RipUp, Scope, round_costs,
    round_strategy, routable_nets,
};
pub use global::{CapacityReduction, rudy_wire_width};
use global::{
    GCell, GlobalGrid, GlobalNet, GlobalRoutes, RestoreError, Tracks, restore_global, route_global,
    track_demand,
};
use occupancy::FixedTracks;
use pare_common::config::{LayerRangeError, RoutingConfig, Share};
use pare_common::db::design::{
    GCellGridDef, NetlistDB, PinOwner, PlacementStatus, RouteElem, TrackAxis,
};
use pare_common::db::indices::{LayerId, NetId, PinId};
use pare_common::db::library::{Direction, Shape};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::shapes::obstruction_shapes;
use rustc_hash::{FxHashMap, FxHashSet};
use search::Costs;
pub use session::GlobalSession;
use state::{DetailedState, GridShape};
pub use state::{RouteState, StateParseError};
use std::time::{Duration, Instant};
use tech::{MAX_DIE_REACH, MAX_GRID_SPAN, RouteTech};
use track::assign_tracks;

/// What routing achieved.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct RoutingSummary {
    /// Signal nets routed.
    pub nets: usize,
    /// Nets with a pin no access point reaches.
    pub inaccessible: Vec<NetId>,
    /// Nets whose search failed to connect every pin.
    pub incomplete: Vec<NetId>,
    /// Negotiation rounds run.
    pub rounds: usize,
    /// Nets still in conflict when negotiation stopped.
    pub conflicted: Vec<NetId>,
    /// Whether the run reached its save point and handed over its
    /// state; false without a save point.
    pub state_saved: bool,
}

/// Where a routing run resumes from, when it saves its state, what it
/// traces and whom it reports its progress to.
#[derive(Default)]
pub struct RunOptions<'a> {
    /// A state saved by an earlier run on the same design, to go on from.
    pub resume: Option<&'a RouteState>,
    /// When to save the run's state, and what to hand it to.
    pub save: Option<Save<'a>>,
    /// The nets and area whose detailed routing to log in detail.
    pub trace: TraceRequest,
    /// Takes the run's progress as it goes.
    pub report: Option<&'a mut dyn FnMut(&Progress<'_>)>,
}

impl std::fmt::Debug for RunOptions<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RunOptions")
            .field("resume", &self.resume)
            .field("save", &self.save)
            .field("trace", &self.trace)
            .finish_non_exhaustive()
    }
}

/// What a routing run reports as it goes, for its caller to show.
#[derive(Clone, Debug, PartialEq)]
pub enum Progress<'a> {
    /// A stage of the run finished after `elapsed`.
    Stage {
        /// The stage.
        stage: Stage,
        /// How long it took.
        elapsed: Duration,
    },
    /// Global routing finished on a grid of `gcells` columns and rows.
    GlobalRouted {
        /// The grid's columns and rows.
        gcells: (u16, u16),
        /// Tracks the routes take beyond the edges' capacity.
        overflow: i64,
        /// Each routing layer's totals, lowest first.
        layers: &'a [LayerLoad<'a>],
    },
    /// Settling the global routes after repair kept some and routed the
    /// rest again.
    GlobalSettled {
        /// Nets whose routes were kept.
        kept: usize,
        /// Nets routed again.
        rerouted: usize,
    },
    /// A detailed round finished; the first is the first pass, or the round
    /// a resumed run was saved after.
    DetailedRound(RoundReport),
}

/// A stage of a routing run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// Finding pin access.
    PinAccess,
    /// Building the gcell grid.
    GCellGrid,
    /// Routing every net globally.
    GlobalRoute,
    /// Taking the global routes from a saved state.
    GlobalRestore,
    /// Keeping or routing again the global routes after repair.
    GlobalSettle,
    /// Assigning tracks along the global routes.
    TrackAssignment,
}

/// One routing layer's global routing totals, in whole tracks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LayerLoad<'a> {
    /// The layer's name.
    pub name: &'a str,
    /// Its preferred direction.
    pub direction: Direction,
    /// Tracks its edges offer.
    pub capacity: i64,
    /// Tracks the routes take.
    pub usage: i64,
    /// Usage beyond capacity.
    pub overflow: i64,
}

/// What one detailed round did.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoundReport {
    /// The round; 0 is the first pass.
    pub round: usize,
    /// Design rule markers left.
    pub markers: usize,
    /// Nets still in conflict.
    pub conflicted: usize,
    /// Pieces ripped up and routed again.
    pub repaired: usize,
    /// The cost of a conflict this round.
    pub conflict_cost: f64,
    /// The flow the round ran.
    pub flow: &'static str,
    /// How long it took.
    pub elapsed: Duration,
}

/// Nets, by name, and an area whose detailed routing to log in detail.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TraceRequest {
    /// Nets to trace wherever they are routed.
    pub nets: Vec<String>,
    /// An area whose regions to trace with every net in them.
    pub area: Option<Rect<Dbu>>,
}

/// When a run saves its state and what it hands the state to.
pub struct Save<'a> {
    /// The point of the run after which to save.
    pub after: SavePoint,
    /// Takes the state once that point is reached; an error stops the run.
    pub write: &'a mut dyn FnMut(&RouteState) -> Result<(), SaveError>,
}

/// Why a run's state could not be saved, as its saver reports it.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
#[error("{0}")]
pub struct SaveError(pub String);

impl std::fmt::Debug for Save<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Save")
            .field("after", &self.after)
            .finish_non_exhaustive()
    }
}

/// A point of a routing run after which its state can be saved.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SavePoint {
    /// Global routing.
    Global,
    /// A detailed round; 0 is the first pass.
    Round(usize),
}

impl RunOptions<'_> {
    /// Hands the state `build` makes to the save, if it is due after
    /// `point`, returning whether it was.
    fn save_at(
        &mut self,
        point: SavePoint,
        build: impl FnOnce() -> RouteState,
    ) -> Result<bool, SaveError> {
        let Some(save) = self.save.as_mut().filter(|s| s.after == point) else {
            return Ok(false);
        };
        (save.write)(&build())?;
        Ok(true)
    }
}

/// Routes every signal net of `db` and stores the wiring in the design.
pub fn route(db: &mut NetlistDB, config: &RoutingConfig) -> Result<RoutingSummary, RouteError> {
    route_with(db, config, RunOptions::default())
}

/// Routes every signal net of `db` as [`route`] does, resuming from and
/// saving state as `run` says.
///
/// A run resumed from a state saved after a detailed round goes on from
/// the round after it exactly as the run that saved it would have.
pub fn route_with(
    db: &mut NetlistDB,
    config: &RoutingConfig,
    mut run: RunOptions<'_>,
) -> Result<RoutingSummary, RouteError> {
    check_routable(db)?;
    let tech = RouteTech::new(db, config)?;
    let gcell = gcell_size(&tech, config);
    let mut silent = |_: &Progress<'_>| {};
    let report = run.report.take().unwrap_or(&mut silent);
    let (routes, summary, gcells) = {
        let routed = global_stage(db, &tech, config, gcell, run.resume, report)?;
        route_detailed(db, &tech, config, gcell, routed, &mut run, report)?
    };
    write_wiring(db, routes, gcells);
    Ok(summary)
}

/// Routes every signal net of `db` in detail along the global routes of
/// `global`, which follow the design as repair left it.
///
/// A net whose route misses one of its pins' gcells now is routed globally
/// again first.
///
/// # Errors
///
/// When `db` cannot be routed under `config`, or a saved state cannot be
/// written.
pub fn route_on(
    db: &mut NetlistDB,
    config: &RoutingConfig,
    global: GlobalSession,
    mut run: RunOptions<'_>,
) -> Result<RoutingSummary, RouteError> {
    check_routable(db)?;
    let (tech, grid, routes, routed) = global.into_parts(db);
    let gcell = gcell_size(&tech, config);
    let mut silent = |_: &Progress<'_>| {};
    let report = run.report.take().unwrap_or(&mut silent);
    let (routes, summary, gcells) = {
        let settled = settled_stage(
            db,
            &tech,
            config,
            gcell,
            Settling {
                grid,
                global: routes,
                routed,
            },
            report,
        );
        route_detailed(db, &tech, config, gcell, settled, &mut run, report)?
    };
    write_wiring(db, routes, gcells);
    Ok(summary)
}

/// Puts the detailed routes and the gcell grid they were routed on into
/// `db`.
fn write_wiring(
    db: &mut NetlistDB,
    routes: Vec<(NetId, Vec<RouteElem>)>,
    gcells: Vec<GCellGridDef>,
) {
    db.gcell_grid = gcells;
    for net in &mut db.nets {
        net.routing.clear();
    }
    for (n, elems) in routes {
        db.nets[n.index()].routing = elems;
    }
}

/// Each routed net's wiring, with the summary and the gcell grid routed on.
type DetailedResult = (
    Vec<(NetId, Vec<RouteElem>)>,
    RoutingSummary,
    Vec<GCellGridDef>,
);

/// Track assignment and detailed routing over the global routes of
/// `routed`, saving state as `run` says.
fn route_detailed(
    db: &NetlistDB,
    tech: &RouteTech,
    config: &RoutingConfig,
    gcell: Dbu,
    routed: GlobalStage<'_>,
    run: &mut RunOptions<'_>,
    report: &mut dyn FnMut(&Progress<'_>),
) -> Result<DetailedResult, RouteError> {
    let via = 4.0 * tech.layer(tech.min_layer).pitch().to_f64();
    let GlobalStage {
        mut dr,
        nets,
        grid,
        global,
    } = routed;
    let (nx, ny) = grid.size();
    let saver = StateSaver {
        db,
        nets: &nets,
        grid: grid_shape(tech, &grid),
        global: run.save.is_some().then(|| named_global(db, &global)),
    };
    let mut state_saved = run.save_at(SavePoint::Global, || saver.global_state())?;
    dr.set_trace(trace_of(db, &run.trace)?);
    let start = Instant::now();
    let wires = assign_tracks(tech, &dr.occupancy, &grid, &global, &dr.pin_points());
    dr.set_guides(global, &grid);
    dr.set_track_wires(wires);
    report(&Progress::Stage {
        stage: Stage::TrackAssignment,
        elapsed: start.elapsed(),
    });
    dr.log_inaccessible_pins();
    let mut summary = RoutingSummary {
        nets: nets.len(),
        inaccessible: dr.inaccessible_nets(),
        ..RoutingSummary::default()
    };
    let mut negotiation = match run.resume.and_then(|s| s.detailed.as_ref()) {
        Some(saved) => Negotiation::resumed(&mut dr, db, saved, via, report)?,
        None => Negotiation::after_first_pass(&mut dr, &nets, via, report),
    };
    summary.rounds = negotiation.first_round - 1;
    if negotiation.first_round == 1 {
        state_saved |= run.save_at(SavePoint::Round(0), || {
            saver.detailed_state(&dr, 0, &negotiation)
        })?;
    }
    for round in negotiation.first_round..=config.detailed_iterations {
        if negotiation.conflicted.is_empty() {
            break;
        }
        let start = Instant::now();
        let costs = round_costs(round, via);
        let (flow, repaired) = negotiation.round(&mut dr, round, costs);
        summary.rounds = round;
        state_saved |= run.save_at(SavePoint::Round(round), || {
            saver.detailed_state(&dr, round, &negotiation)
        })?;
        report(&Progress::DetailedRound(RoundReport {
            round,
            markers: dr.marker_count(),
            conflicted: negotiation.conflicted.len(),
            repaired: repaired.pieces,
            conflict_cost: costs.conflict,
            flow: flow.name(),
            elapsed: start.elapsed(),
        }));
    }
    dr.log_conflicts();
    summary.conflicted = negotiation.conflicted;
    summary.state_saved = state_saved;
    summary.incomplete = nets
        .iter()
        .copied()
        .filter(|n| {
            let route = &dr.routes()[n.index()];
            !(route.complete && route.is_connected())
        })
        .collect();
    let wiring: Vec<(NetId, Vec<RouteElem>)> = nets
        .iter()
        .map(|&n| (n, dr.routes()[n.index()].elems(tech)))
        .collect();
    Ok((wiring, summary, gcell_grid(db.die_area.min, gcell, nx, ny)))
}

/// Where detailed routing's negotiation stands: the next round, the nets
/// in conflict, the scope of the next regular round and the flows.
struct Negotiation {
    first_round: usize,
    conflicted: Vec<NetId>,
    scope: Scope,
    flows: FlowState,
}

impl Negotiation {
    /// Resumes from the routes saved after round `saved.round`.
    fn resumed(
        dr: &mut DetailedRouter<'_>,
        db: &NetlistDB,
        saved: &DetailedState,
        via: f64,
        report: &mut dyn FnMut(&Progress<'_>),
    ) -> Result<Self, RouteError> {
        let start = Instant::now();
        let routes = named_routes(db, &saved.routes)?;
        dr.restore(&routes)?;
        dr.occupancy.shrink_to_fit();
        let conflicted = dr.rejudge();
        report(&Progress::DetailedRound(RoundReport {
            round: saved.round,
            markers: dr.marker_count(),
            conflicted: conflicted.len(),
            repaired: 0,
            conflict_cost: round_costs(saved.round, via).conflict,
            flow: saved.flow.name(),
            elapsed: start.elapsed(),
        }));
        Ok(Self {
            first_round: saved.round + 1,
            conflicted,
            scope: saved.scope,
            flows: FlowState::after(saved.round, saved.flow, saved.effective),
        })
    }

    /// Routes `nets` for the first time.
    fn after_first_pass(
        dr: &mut DetailedRouter<'_>,
        nets: &[NetId],
        via: f64,
        report: &mut dyn FnMut(&Progress<'_>),
    ) -> Self {
        let start = Instant::now();
        dr.log_footprint("track assignment");
        let first = dr.route_initial(nets, round_costs(0, via));
        dr.occupancy.shrink_to_fit();
        dr.log_footprint("first pass");
        let conflicted = dr.rejudge();
        let flows = FlowState::after_first_pass();
        report(&Progress::DetailedRound(RoundReport {
            round: 0,
            markers: dr.marker_count(),
            conflicted: conflicted.len(),
            repaired: first.pieces,
            conflict_cost: round_costs(0, via).conflict,
            flow: flows.current().name(),
            elapsed: start.elapsed(),
        }));
        Self {
            first_round: 1,
            conflicted,
            scope: Scope::Conflicts,
            flows,
        }
    }

    /// Runs round `round` at `costs`, returning the flow it took and what
    /// its repair did.
    fn round(
        &mut self,
        dr: &mut DetailedRouter<'_>,
        round: usize,
        costs: Costs,
    ) -> (Flow, RepairStats) {
        let flow = self.flows.next(round, dr.marker_count());
        let tile_scope = match round_strategy(round).ripup {
            RipUp::NearDrc => Scope::NearConflicts,
            RipUp::Drc | RipUp::All => Scope::Conflicts,
        };
        let repaired = match flow {
            Flow::Regular => dr.repair(costs, round, self.scope),
            Flow::Guides => dr.repair_guides(tile_scope, costs),
            Flow::Stubborn => dr.repair_stubborn(tile_scope, costs),
            Flow::Skip => RepairStats::default(),
        };
        if matches!(flow, Flow::Guides | Flow::Stubborn) {
            self.flows.set_effective(repaired.improved > 0);
        }
        if flow != Flow::Skip {
            let before = self.conflicted.len();
            self.conflicted = dr.rejudge();
            if flow == Flow::Regular {
                self.scope = next_scope(self.scope, self.conflicted.len() < before);
            }
        }
        (flow, repaired)
    }
}

/// What a saved state names: the design, its routed nets, the gcell grid
/// and, when a save is due, the global routes by net name.
struct StateSaver<'a> {
    db: &'a NetlistDB,
    nets: &'a [NetId],
    grid: GridShape,
    global: Option<Vec<(String, Vec<Vec<GCell>>)>>,
}

impl StateSaver<'_> {
    /// The state of a run saved after global routing.
    fn global_state(&self) -> RouteState {
        RouteState {
            design: self.db.name.clone(),
            grid: self.grid,
            global: self.global.clone().unwrap_or_default(),
            detailed: None,
        }
    }

    /// The state of a run saved after detailed round `round`.
    fn detailed_state(
        &self,
        dr: &DetailedRouter<'_>,
        round: usize,
        negotiation: &Negotiation,
    ) -> RouteState {
        let routes = self
            .nets
            .iter()
            .map(|&n| {
                (
                    self.db.nets[n.index()].name.clone(),
                    dr.routes()[n.index()].clone(),
                )
            })
            .collect();
        RouteState {
            detailed: Some(DetailedState {
                round,
                scope: negotiation.scope,
                flow: negotiation.flows.current(),
                effective: negotiation.flows.last_effective(),
                routes,
            }),
            ..self.global_state()
        }
    }
}

/// The trace `request` asks for, with its nets found by name.
fn trace_of(db: &NetlistDB, request: &TraceRequest) -> Result<detailed::Trace, RouteError> {
    let ids = net_ids(db);
    let nets = request
        .nets
        .iter()
        .map(|name| {
            ids.get(name.as_str())
                .copied()
                .ok_or_else(|| RouteError::TraceNet(name.clone()))
        })
        .collect::<Result<Vec<NetId>, _>>()?;
    Ok(detailed::Trace::new(nets, request.area))
}

/// Every globally routed net's name with the paths of its tree, by name.
fn named_global(db: &NetlistDB, global: &GlobalRoutes) -> Vec<(String, Vec<Vec<GCell>>)> {
    let mut out: Vec<(String, Vec<Vec<GCell>>)> = global
        .paths
        .iter()
        .map(|(n, paths)| (db.nets[n.index()].name.clone(), paths.clone()))
        .collect();
    out.sort_unstable_by(|a, b| a.0.cmp(&b.0));
    out
}

/// The shape of `grid` as a saved state records it.
const fn grid_shape(tech: &RouteTech, grid: &GlobalGrid) -> GridShape {
    let (nx, ny) = grid.size();
    GridShape {
        gcell: grid.gcell(),
        nx,
        ny,
        min_layer: tech.min_layer,
        max_layer: tech.max_layer,
    }
}

/// The nets `routes` name, with their routes.
fn named_routes(
    db: &NetlistDB,
    routes: &[(String, wiring::NetRoute)],
) -> Result<Vec<(NetId, wiring::NetRoute)>, RouteError> {
    let ids = net_ids(db);
    routes
        .iter()
        .map(|(name, route)| {
            ids.get(name.as_str())
                .map(|&n| (n, route.clone()))
                .ok_or_else(|| RouteError::StateNet(name.clone()))
        })
        .collect()
}

fn net_ids(db: &NetlistDB) -> FxHashMap<&str, NetId> {
    db.nets
        .iter()
        .enumerate()
        .map(|(i, n)| (n.name.as_str(), NetId::new(i)))
        .collect()
}

/// Global routing and track assignment of every signal net of `db`, for
/// measuring them on their own; the design is left as it is.
pub fn route_global_only(
    db: &NetlistDB,
    config: &RoutingConfig,
    report: &mut dyn FnMut(&Progress<'_>),
) -> Result<GlobalOnly, RouteError> {
    check_routable(db)?;
    let tech = RouteTech::new(db, config)?;
    let gcell = gcell_size(&tech, config);
    let GlobalStage {
        dr,
        nets,
        grid,
        global,
        ..
    } = global_stage(db, &tech, config, gcell, None, report)?;
    let start = Instant::now();
    let _ = assign_tracks(&tech, &dr.occupancy, &grid, &global, &dr.pin_points());
    report(&Progress::Stage {
        stage: Stage::TrackAssignment,
        elapsed: start.elapsed(),
    });
    let mut guides: Vec<NetGuide> = global
        .paths
        .iter()
        .map(|(&net, paths)| NetGuide {
            net,
            boxes: guide_boxes(&grid, paths, db.die_area),
        })
        .collect();
    guides.sort_unstable_by_key(|g| g.net);
    dr.log_inaccessible_pins();
    Ok(GlobalOnly {
        summary: RoutingSummary {
            nets: nets.len(),
            inaccessible: dr.inaccessible_nets(),
            ..RoutingSummary::default()
        },
        guides,
    })
}

/// What global routing alone gives: its summary and every routed net's
/// global route as guides.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct GlobalOnly {
    /// The nets routed and those with a pin no access point reaches.
    pub summary: RoutingSummary,
    /// Each routed net's guides, by net.
    pub guides: Vec<NetGuide>,
}

/// A net's global route as the boxes of gcells it runs through on each
/// layer, one box per run of gcells along a layer, as route guides give it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NetGuide {
    /// The net.
    pub net: NetId,
    /// Layer and box of each run, clipped to the die.
    pub boxes: Vec<(LayerId, Rect<Dbu>)>,
}

/// The boxes of each run of consecutive gcells on one layer along `paths`
/// of `grid`, clipped to `die`, without repeats.
fn guide_boxes(
    grid: &GlobalGrid,
    paths: &[Vec<GCell>],
    die: Rect<Dbu>,
) -> Vec<(LayerId, Rect<Dbu>)> {
    let mut boxes: Vec<(LayerId, Rect<Dbu>)> = Vec::new();
    for path in paths {
        for run in path.chunk_by(|a, b| a.layer == b.layer) {
            let area = run
                .iter()
                .map(|c| grid.cell_rect(c.x, c.y))
                .reduce(|a, b| a.union(&b));
            if let Some(clipped) = area.and_then(|a| a.intersection(&die)) {
                boxes.push((run[0].layer, clipped));
            }
        }
    }
    boxes.sort_unstable_by_key(|&(l, r)| (l, r.min.x, r.min.y, r.max.x, r.max.y));
    boxes.dedup();
    boxes
}

/// Why `db` cannot be routed, if it cannot.
fn check_routable(db: &NetlistDB) -> Result<(), RouteError> {
    let mut unplaced = db
        .cells
        .iter()
        .filter(|c| c.status == PlacementStatus::Unplaced);
    if let Some(first) = unplaced.next() {
        return Err(RouteError::UnplacedCells {
            count: 1 + unplaced.count(),
            first: first.name.clone(),
        });
    }
    if let Some(error) = overlapping_pins(db) {
        return Err(error);
    }
    Ok(())
}

/// The detailed router with its pin access, the nets to route, and their
/// global routes on the gcell grid.
struct GlobalStage<'a> {
    dr: DetailedRouter<'a>,
    nets: Vec<NetId>,
    grid: GlobalGrid,
    global: global::GlobalRoutes,
}

/// Pin access, the gcell grid and global routing, or the global routes of
/// `resume`, reported stage by stage.
fn global_stage<'a>(
    db: &'a NetlistDB,
    tech: &'a RouteTech,
    config: &RoutingConfig,
    gcell: Dbu,
    resume: Option<&RouteState>,
    report: &mut dyn FnMut(&Progress<'_>),
) -> Result<GlobalStage<'a>, RouteError> {
    let start = Instant::now();
    let dr = DetailedRouter::new(db, tech, gcell);
    let nets = routable_nets(db);
    report(&Progress::Stage {
        stage: Stage::PinAccess,
        elapsed: start.elapsed(),
    });

    let start = Instant::now();
    let mut grid = GlobalGrid::new(
        tech,
        &dr.occupancy,
        db.die_area,
        gcell,
        config.global_capacity_reserve,
    );
    report(&Progress::Stage {
        stage: Stage::GCellGrid,
        elapsed: start.elapsed(),
    });
    let start = Instant::now();
    let global_nets: Vec<GlobalNet> = nets
        .iter()
        .filter_map(|&net| {
            Some(GlobalNet {
                net,
                terminals: dr.terminals(net, &grid)?,
                demand: track_demand(db, tech, net),
            })
        })
        .collect();
    let global = match resume {
        Some(state) => restored_global(db, tech, &grid, &global_nets, state)?,
        None => route_global(&mut grid, &global_nets, config.global_iterations),
    };
    report(&Progress::Stage {
        stage: if resume.is_some() {
            Stage::GlobalRestore
        } else {
            Stage::GlobalRoute
        },
        elapsed: start.elapsed(),
    });
    let layers: Vec<LayerLoad<'_>> = grid
        .layer_totals()
        .into_iter()
        .map(|t| LayerLoad {
            name: &db.library.layer(t.layer).name,
            direction: t.direction,
            capacity: t.capacity.rounded(),
            usage: t.usage.rounded(),
            overflow: t.overflow.rounded(),
        })
        .collect();
    report(&Progress::GlobalRouted {
        gcells: grid.size(),
        overflow: grid.overflow().rounded(),
        layers: &layers,
    });
    Ok(GlobalStage {
        dr,
        nets,
        grid,
        global,
    })
}

/// The global routes a session kept, to settle against pin access found
/// afresh: its grid, its routes and the nets they were routed for.
struct Settling {
    grid: GlobalGrid,
    global: GlobalRoutes,
    routed: FxHashMap<NetId, GlobalNet>,
}

/// Pin access found afresh, with the global routes a session kept: a net
/// keeps its route while the route reaches the gcell of each of its pins'
/// access points now, and is routed again otherwise; routes of nets no
/// longer routed are lifted.
fn settled_stage<'a>(
    db: &'a NetlistDB,
    tech: &'a RouteTech,
    config: &RoutingConfig,
    gcell: Dbu,
    settling: Settling,
    report: &mut dyn FnMut(&Progress<'_>),
) -> GlobalStage<'a> {
    let Settling {
        mut grid,
        mut global,
        mut routed,
    } = settling;
    let start = Instant::now();
    let dr = DetailedRouter::new(db, tech, gcell);
    let nets = routable_nets(db);
    report(&Progress::Stage {
        stage: Stage::PinAccess,
        elapsed: start.elapsed(),
    });
    let start = Instant::now();
    let mut dirty = Vec::new();
    let mut kept = FxHashSet::default();
    for &net in &nets {
        let Some(terminals) = dr.terminals(net, &grid) else {
            continue;
        };
        let fresh = GlobalNet {
            net,
            terminals,
            demand: global::track_demand(db, tech, net),
        };
        let old = routed.remove(&net);
        let tree = global.paths.remove(&net);
        match (old, tree) {
            (Some(old), Some(tree)) if reaches(&tree, &fresh.terminals) => {
                grid.lift(&old, &tree);
                grid.place(&fresh, &tree);
                let _ = global.paths.insert(net, tree);
                let _ = kept.insert(net);
            }
            (old, tree) => {
                if let (Some(old), Some(tree)) = (old, tree) {
                    grid.lift(&old, &tree);
                }
                dirty.push(fresh);
            }
        }
    }
    for (net, old) in routed {
        if let Some(tree) = global.paths.remove(&net) {
            grid.lift(&old, &tree);
        }
    }
    let rerouted = global::reroute(&mut grid, &dirty, config.global_iterations);
    global.paths.extend(rerouted.paths);
    report(&Progress::Stage {
        stage: Stage::GlobalSettle,
        elapsed: start.elapsed(),
    });
    report(&Progress::GlobalSettled {
        kept: kept.len(),
        rerouted: dirty.len(),
    });
    GlobalStage {
        dr,
        nets,
        grid,
        global,
    }
}

/// Whether a tree's cells include every terminal.
fn reaches(tree: &[Vec<GCell>], terminals: &[GCell]) -> bool {
    let cells: FxHashSet<GCell> = tree.iter().flatten().copied().collect();
    terminals.iter().all(|t| cells.contains(t))
}

/// The global routes `state` saved, put back on `grid` once the state is
/// known to be of this design and grid.
fn restored_global(
    db: &NetlistDB,
    tech: &RouteTech,
    grid: &GlobalGrid,
    nets: &[GlobalNet],
    state: &RouteState,
) -> Result<GlobalRoutes, RouteError> {
    if state.design != db.name {
        return Err(RouteError::StateDesign(state.design.clone()));
    }
    if state.grid != grid_shape(tech, grid) {
        return Err(RouteError::StateGrid);
    }
    let ids = net_ids(db);
    let saved = state
        .global
        .iter()
        .map(|(name, paths)| {
            ids.get(name.as_str())
                .map(|&net| (net, paths.clone()))
                .ok_or_else(|| RouteError::StateNet(name.clone()))
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(restore_global(grid, nets, saved)?)
}

/// The scope of the next repair pass: conflicts only, except for one pass
/// that also rips up wiring near each conflict after a conflicts-only pass
/// that left no fewer nets in conflict.
const fn next_scope(previous: Scope, improved: bool) -> Scope {
    match previous {
        Scope::Conflicts if !improved => Scope::NearConflicts,
        _ => Scope::Conflicts,
    }
}

/// Every gcell's routing capacity and what of it is taken before placement.
///
/// What is taken is the metal placement cannot move, that of fixed cells
/// and I/O pins, special nets and routing blockages, and each layer's share
/// of `adjustments`, by layer index, held back, none for a layer past its
/// end, as `OpenROAD`'s RUDY counts them.
pub fn capacity_reduction(
    db: &NetlistDB,
    config: &RoutingConfig,
    adjustments: &[Share],
) -> Result<CapacityReduction, RouteError> {
    let tech = RouteTech::new(db, config)?;
    let fixed = FixedTracks::new(&tech, obstruction_shapes(db));
    let grid = GlobalGrid::new(
        &tech,
        &fixed,
        db.die_area,
        gcell_size(&tech, config),
        config.global_capacity_reserve,
    );
    Ok(grid.capacity_reduction(&tech, adjustments, layers_wire_width(db, &tech)))
}

/// The RUDY wire width over the routing layers of `tech`, each at its LEF
/// pitch, or its width plus spacing without one, as `OpenROAD` takes it.
fn layers_wire_width(db: &NetlistDB, tech: &RouteTech) -> Dbu {
    let pitches: Vec<Dbu> = (tech.min_layer.index()..=tech.max_layer.index())
        .map(|l| {
            let layer = &db.library.routing_layers[l];
            if layer.pitch_x > Dbu::ZERO {
                layer.pitch_x
            } else {
                layer.width + layer.min_spacing
            }
        })
        .collect();
    rudy_wire_width(&pitches)
}

/// Global routing of every signal net with the cells wherever placement
/// has them at the moment, for measuring congestion while placing.
///
/// This is how `OpenROAD`'s placer runs its global router. The routes are
/// negotiated for `ESTIMATE_ROUNDS` rounds, so that the congestion cheap
/// detours remove does not count and what detailed routing would face does.
#[derive(Debug)]
pub struct GlobalRouteEstimator<'a> {
    db: &'a NetlistDB,
    tech: RouteTech,
    grid: GlobalGrid,
    nets: Vec<EstimatedNet>,
    rounds: usize,
}

/// A net to route, with the layer each pin is reached on.
#[derive(Debug)]
struct EstimatedNet {
    net: NetId,
    pins: Vec<(PinId, LayerId)>,
    demand: Vec<Tracks>,
}

impl<'a> GlobalRouteEstimator<'a> {
    /// The estimator of `db` under `config`, over the gcell grid of
    /// [`capacity_reduction`].
    pub fn new(db: &'a NetlistDB, config: &RoutingConfig) -> Result<Self, RouteError> {
        let tech = RouteTech::new(db, config)?;
        let fixed = FixedTracks::new(&tech, obstruction_shapes(db));
        let grid = GlobalGrid::new(
            &tech,
            &fixed,
            db.die_area,
            gcell_size(&tech, config),
            config.global_capacity_reserve,
        );
        let nets = routable_nets(db)
            .into_iter()
            .map(|net| EstimatedNet {
                net,
                pins: db.nets[net.index()]
                    .pins
                    .iter()
                    .map(|&pin| (pin, terminal_layer(db, &tech, pin)))
                    .collect(),
                demand: track_demand(db, &tech, net),
            })
            .collect();
        Ok(Self {
            db,
            tech,
            grid,
            nets,
            rounds: config.global_iterations.min(ESTIMATE_ROUNDS),
        })
    }

    /// The grid's capacities and what fixed metal and each layer's share
    /// of `adjustments`, by layer index, take of them.
    #[must_use]
    pub fn capacity_reduction(&self, adjustments: &[Share]) -> CapacityReduction {
        let wire_width = layers_wire_width(self.db, &self.tech);
        self.grid
            .capacity_reduction(&self.tech, adjustments, wire_width)
    }

    /// Each gcell's congestion, row by row, once every net is routed with
    /// each cell's lower-left corner at its entry of `locations`, which is
    /// indexed like the design's cells.
    pub fn congestion(&mut self, locations: &[Point<Dbu>]) -> Vec<f64> {
        let nets: Vec<GlobalNet> = self
            .nets
            .iter()
            .map(|n| GlobalNet {
                net: n.net,
                terminals: n
                    .pins
                    .iter()
                    .map(|&(pin, layer)| {
                        let at = match self.db.pins[pin.index()].owner {
                            PinOwner::Cell { cell, .. } => {
                                self.db.pin_position_at(pin, locations[cell.index()])
                            }
                            PinOwner::Io(_) => self.db.pin_position(pin),
                        };
                        self.grid.cell_at(layer, at)
                    })
                    .collect(),
                demand: n.demand.clone(),
            })
            .collect();
        self.grid.clear();
        let _ = route_global(&mut self.grid, &nets, self.rounds);
        self.grid.congestion(&self.tech)
    }
}

/// Negotiation rounds of a congestion estimate: enough for the detours that
/// cost little, after which the rounds grow expensive and move little.
const ESTIMATE_ROUNDS: usize = 10;

/// The routable layer global routing reaches `pin` on: the lowest layer
/// the pin has a shape on, brought into the routable range.
fn terminal_layer(db: &NetlistDB, tech: &RouteTech, pin: PinId) -> LayerId {
    db.pin_reference_layer(pin)
        .map_or(tech.min_layer, |l| l.clamp(tech.min_layer, tech.max_layer))
}

/// The edge of a gcell: `config.gcell_tracks` gcell pitches.
fn gcell_size(tech: &RouteTech, config: &RoutingConfig) -> Dbu {
    tech.gcell_pitch * i64::from(config.gcell_tracks.get())
}

/// The DEF gcell grid of `nx` by `ny` gcells of edge `gcell` from `origin`.
fn gcell_grid(origin: Point<Dbu>, gcell: Dbu, nx: u16, ny: u16) -> Vec<GCellGridDef> {
    let lines = |start: Dbu, cells: u16| GCellGridDef {
        axis: TrackAxis::X,
        start,
        count: u32::from(cells) + 1,
        step: gcell,
    };
    vec![
        lines(origin.x, nx),
        GCellGridDef {
            axis: TrackAxis::Y,
            ..lines(origin.y, ny)
        },
    ]
}

/// An error naming the cells whose pins of different nets overlap, which
/// no routing can keep apart.
fn overlapping_pins(db: &NetlistDB) -> Option<RouteError> {
    let mut first = None;
    let mut count = 0;
    for cell in &db.cells {
        let pins: Vec<(NetId, Vec<Shape>)> = cell
            .pins
            .iter()
            .map(|&p| (db.pins[p.index()].net, db.pin_shapes(p)))
            .collect();
        let clash = pins.iter().enumerate().find_map(|(i, (a, shapes_a))| {
            pins[i + 1..]
                .iter()
                .find(|(b, shapes_b)| b != a && shapes_touch(shapes_a, shapes_b))
                .map(|(b, _)| (*a, *b))
        });
        if let Some((a, b)) = clash {
            count += 1;
            let _ = first.get_or_insert_with(|| {
                (
                    cell.name.clone(),
                    db.nets[a.index()].name.clone(),
                    db.nets[b.index()].name.clone(),
                )
            });
        }
    }
    first.map(|(cell, net_a, net_b)| RouteError::OverlappingPins {
        count,
        cell,
        nets: (net_a, net_b),
    })
}

fn shapes_touch(a: &[Shape], b: &[Shape]) -> bool {
    a.iter().any(|x| {
        b.iter()
            .any(|y| x.layer == y.layer && x.rect.overlaps(&y.rect))
    })
}

/// Why routing could not run.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RouteError {
    /// The design has cells without a location.
    #[error("{count} cells are unplaced, starting with '{first}'")]
    UnplacedCells {
        /// How many.
        count: usize,
        /// The first one's name.
        first: String,
    },
    /// Pins of different nets overlap inside cells, as in Bookshelf designs
    /// whose pins are points at the cell center.
    #[error(
        "pins of different nets overlap in {count} cells, first in '{cell}' \
         (nets '{}' and '{}')",
        nets.0,
        nets.1
    )]
    OverlappingPins {
        /// How many cells.
        count: usize,
        /// The first one's name.
        cell: String,
        /// Two of its nets whose pins overlap.
        nets: (String, String),
    },
    /// The configured routing layers are unusable.
    #[error(transparent)]
    Layers(#[from] LayerRangeError),
    /// A routable layer has no tracks and no pitch to derive them from.
    #[error("routing layer '{0}' has no tracks")]
    NoTracks(String),
    /// No usable via is defined through a cut layer.
    #[error("no via is defined through cut layer '{0}'")]
    NoVia(String),
    /// The die reaches further from the origin than the router's 32-bit
    /// occupancy coordinates allow.
    #[error(
        "the die reaches {0} database units from the origin, beyond the router's limit of {MAX_DIE_REACH}"
    )]
    DieTooLarge(i64),
    /// The die spans more gcells than the global grid's coordinates hold.
    #[error(
        "the die spans {0} gcells, beyond the global router's {MAX_GRID_SPAN}; raise [routing] gcell_tracks"
    )]
    GridTooLarge(i64),
    /// A saved state is of another design.
    #[error("the route state was saved for design '{0}'")]
    StateDesign(String),
    /// A saved state was routed on another gcell grid or layer range.
    #[error("the route state was saved on another gcell grid or layer range")]
    StateGrid,
    /// A saved state names a net the design does not have.
    #[error("the route state names net '{0}', which the design does not have")]
    StateNet(String),
    /// A net to trace is not in the design.
    #[error("no net '{0}' to trace")]
    TraceNet(String),
    /// Saved global routes do not fit the design as it is placed now.
    #[error(transparent)]
    GlobalRestore(#[from] RestoreError),
    /// Saved detailed routes do not fit the design as it is placed now.
    #[error(transparent)]
    DetailedRestore(#[from] RestoreRoutesError),
    /// The run's state could not be saved.
    #[error("cannot save the route state: {0}")]
    Save(#[from] SaveError),
}
