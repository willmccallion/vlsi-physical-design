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
mod search;
pub mod state;
pub mod tech;
pub mod track;
pub mod tree;
pub mod wiring;

use detailed::{DetailedRouter, RestoreRoutesError, Scope, round_costs, routable_nets};
pub use global::{CapacityReduction, TileTracks};
use global::{
    GCell, GlobalGrid, GlobalNet, GlobalRoutes, RestoreError, Tracks, restore_global, route_global,
    track_demand,
};
use occupancy::FixedTracks;
use pare_common::db::design::{
    GCellGridDef, NetlistDB, PinOwner, PlacementStatus, RouteElem, TrackAxis,
};
use pare_common::db::indices::{LayerId, NetId, PinId};
use pare_common::db::library::Shape;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::util::config::{LayerRangeError, RoutingConfig};
use pare_common::util::{memory, ui};
use pare_common::verify::shapes::obstruction_shapes;
use rustc_hash::FxHashMap;
use state::{DetailedState, GridShape};
pub use state::{RouteState, StateParseError};
use std::time::Instant;
use tech::RouteTech;
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
}

/// Where a routing run resumes from and when it saves its state.
#[derive(Debug, Default)]
pub struct Session<'a> {
    /// A state saved by an earlier run on the same design, to go on from.
    pub resume: Option<&'a RouteState>,
    /// When to save the run's state, and what to hand it to.
    pub save: Option<Save<'a>>,
    /// The nets and area whose detailed routing to log in detail.
    pub trace: TraceRequest,
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
    /// Takes the state once that point is reached.
    pub write: &'a mut dyn FnMut(&RouteState),
}

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

impl Session<'_> {
    /// Hands the state `build` makes to the save, if it is due after `point`.
    fn save_at(&mut self, point: SavePoint, build: impl FnOnce() -> RouteState) {
        if let Some(save) = self.save.as_mut()
            && save.after == point
        {
            (save.write)(&build());
        }
    }
}

/// Routes every signal net of `db` and stores the wiring in the design.
pub fn route(db: &mut NetlistDB, config: &RoutingConfig) -> Result<RoutingSummary, RouteError> {
    route_with(db, config, Session::default())
}

/// Routes every signal net of `db` as [`route`] does, resuming from and
/// saving state as `session` says.
///
/// A run resumed from a state saved after a detailed round goes on from
/// the round after it exactly as the run that saved it would have.
pub fn route_with(
    db: &mut NetlistDB,
    config: &RoutingConfig,
    mut session: Session<'_>,
) -> Result<RoutingSummary, RouteError> {
    check_routable(db, config)?;
    let tech = RouteTech::new(db, config)?;
    let gcell = gcell_size(&tech, config);
    let via = 4.0 * tech.layer(tech.min_layer).pitch().to_f64();

    let (routes, summary, gcells) = {
        let GlobalStage {
            mut dr,
            nets,
            grid,
            global,
        } = global_stage(db, &tech, config, gcell, session.resume)?;
        let (nx, ny) = grid.size();
        let named_global = session.save.is_some().then(|| named_global(db, &global));
        let shape = grid_shape(&tech, &grid);
        session.save_at(SavePoint::Global, || RouteState {
            design: db.name.clone(),
            grid: shape,
            global: named_global.clone().unwrap_or_default(),
            detailed: None,
        });
        dr.set_trace(trace_of(db, &session.trace)?);
        let start = Instant::now();
        let wires = assign_tracks(&tech, &dr.occupancy, &grid, &global, &dr.pin_points());
        dr.set_guides(global, &grid);
        dr.set_track_wires(wires);
        stage("Track assign:", start);
        dr.log_inaccessible_pins();
        let mut summary = RoutingSummary {
            nets: nets.len(),
            inaccessible: dr.inaccessible_nets(),
            ..RoutingSummary::default()
        };
        let start = Instant::now();
        let resumed = session.resume.and_then(|s| s.detailed.as_ref());
        let (first_round, mut conflicted, mut scope, mut progress) = if let Some(saved) = resumed {
            let routes = named_routes(db, &saved.routes)?;
            dr.restore(&routes)?;
            ui::dr_table_header();
            let conflicted = dr.rejudge();
            ui::dr_iter(
                saved.round,
                dr.marker_count(),
                conflicted.len(),
                0,
                round_costs(saved.round, via).conflict,
                start.elapsed().as_millis(),
            );
            let progress = Progress {
                best: saved.best,
                since_best: saved.since_best,
            };
            (saved.round + 1, conflicted, saved.scope, progress)
        } else {
            let first = dr.route_initial(&nets, round_costs(0, via));
            ui::dr_table_header();
            let conflicted = dr.rejudge();
            ui::dr_iter(
                0,
                dr.marker_count(),
                conflicted.len(),
                first.pieces,
                round_costs(0, via).conflict,
                start.elapsed().as_millis(),
            );
            let progress = Progress::new(dr.marker_count());
            (1, conflicted, Scope::Conflicts, progress)
        };
        if first_round == 1 {
            session.save_at(SavePoint::Round(0), || {
                detailed_state(
                    db,
                    &dr,
                    &nets,
                    shape,
                    named_global.clone(),
                    0,
                    scope,
                    &progress,
                )
            });
        }
        for round in first_round..=config.detailed_iterations {
            if conflicted.is_empty() {
                break;
            }
            if progress.stalled() {
                log::info!(
                    "detailed routing stopped: no fewer than {} markers in the last {} rounds",
                    progress.best,
                    STALL_ROUNDS
                );
                break;
            }
            let start = Instant::now();
            let costs = round_costs(round, via);
            let repaired = dr.repair(costs, round, scope);
            let before = conflicted.len();
            conflicted = dr.rejudge();
            scope = next_scope(scope, conflicted.len() < before);
            progress.record(dr.marker_count());
            summary.rounds = round;
            session.save_at(SavePoint::Round(round), || {
                let global = named_global.clone();
                detailed_state(db, &dr, &nets, shape, global, round, scope, &progress)
            });
            ui::dr_iter(
                round,
                dr.marker_count(),
                conflicted.len(),
                repaired.pieces,
                costs.conflict,
                start.elapsed().as_millis(),
            );
        }
        dr.log_conflicts();
        summary.conflicted = conflicted;
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
            .map(|&n| (n, dr.routes()[n.index()].elems(&tech)))
            .collect();
        (wiring, summary, gcell_grid(db.die_area.min, gcell, nx, ny))
    };
    db.gcell_grid = gcells;
    for net in &mut db.nets {
        net.routing.clear();
    }
    for (n, elems) in routes {
        db.nets[n.index()].routing = elems;
    }
    Ok(summary)
}

/// The state of a run saved after detailed round `round`.
#[allow(clippy::too_many_arguments)]
fn detailed_state(
    db: &NetlistDB,
    dr: &DetailedRouter<'_>,
    nets: &[NetId],
    grid: GridShape,
    global: Option<Vec<(String, Vec<Vec<GCell>>)>>,
    round: usize,
    scope: Scope,
    progress: &Progress,
) -> RouteState {
    let routes = nets
        .iter()
        .map(|&n| {
            (
                db.nets[n.index()].name.clone(),
                dr.routes()[n.index()].clone(),
            )
        })
        .collect();
    RouteState {
        design: db.name.clone(),
        grid,
        global: global.unwrap_or_default(),
        detailed: Some(DetailedState {
            round,
            scope,
            best: progress.best,
            since_best: progress.since_best,
            routes,
        }),
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
pub fn route_global_only(db: &NetlistDB, config: &RoutingConfig) -> Result<GlobalOnly, RouteError> {
    check_routable(db, config)?;
    let tech = RouteTech::new(db, config)?;
    let gcell = gcell_size(&tech, config);
    let GlobalStage {
        dr,
        nets,
        grid,
        global,
        ..
    } = global_stage(db, &tech, config, gcell, None)?;
    let start = Instant::now();
    let _ = assign_tracks(&tech, &dr.occupancy, &grid, &global, &dr.pin_points());
    stage("Track assign:", start);
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

/// Why `db` cannot be routed under `config`, if it cannot.
fn check_routable(db: &NetlistDB, config: &RoutingConfig) -> Result<(), RouteError> {
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
    check_reserve(config)?;
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
) -> Result<GlobalStage<'a>, RouteError> {
    let start = Instant::now();
    let dr = DetailedRouter::new(db, tech, gcell);
    let nets = routable_nets(db);
    stage("Pin access:", start);

    let start = Instant::now();
    let mut grid = GlobalGrid::new(
        tech,
        &dr.occupancy,
        db.die_area,
        gcell,
        config.global_capacity_reserve,
    );
    stage("GCell grid:", start);
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
        Some(state) => restored_global(db, tech, &mut grid, &global_nets, state)?,
        None => route_global(&mut grid, &global_nets, config.global_iterations),
    };
    let (nx, ny) = grid.size();
    stage(
        if resume.is_some() {
            "Global restored:"
        } else {
            "Global route:"
        },
        start,
    );
    ui::stat_row(
        "GCells:",
        &format!("{nx}x{ny}"),
        "GR overflow:",
        &grid.overflow().rounded().to_string(),
    );
    ui::gr_layer_table_header();
    for t in grid.layer_totals() {
        ui::gr_layer_row(
            &db.library.layer(t.layer).name,
            t.direction,
            t.capacity.rounded(),
            t.usage.rounded(),
            t.overflow.rounded(),
        );
    }
    Ok(GlobalStage {
        dr,
        nets,
        grid,
        global,
    })
}

/// The global routes `state` saved, put back on `grid` once the state is
/// known to be of this design and grid.
fn restored_global(
    db: &NetlistDB,
    tech: &RouteTech,
    grid: &mut GlobalGrid,
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

/// Rounds of detailed routing without fewer markers than the best so far
/// after which the repair is given up as stalled.
const STALL_ROUNDS: usize = 8;

/// The best marker count of detailed routing and how long ago it was set.
struct Progress {
    best: usize,
    since_best: usize,
}

impl Progress {
    const fn new(markers: usize) -> Self {
        Self {
            best: markers,
            since_best: 0,
        }
    }

    const fn record(&mut self, markers: usize) {
        if markers < self.best {
            self.best = markers;
            self.since_best = 0;
        } else {
            self.since_best += 1;
        }
    }

    const fn stalled(&self) -> bool {
        self.since_best >= STALL_ROUNDS
    }
}

/// Every gcell's routing capacity and what the metal placement cannot move
/// takes of it: that of fixed cells and I/O pins, special nets and routing
/// blockages.
pub fn capacity_reduction(
    db: &NetlistDB,
    config: &RoutingConfig,
) -> Result<CapacityReduction, RouteError> {
    check_reserve(config)?;
    let tech = RouteTech::new(db, config)?;
    let fixed = FixedTracks::new(&tech, obstruction_shapes(db));
    let grid = GlobalGrid::new(
        &tech,
        &fixed,
        db.die_area,
        gcell_size(&tech, config),
        config.global_capacity_reserve,
    );
    Ok(grid.capacity_reduction(&tech))
}

/// Global routing of every signal net with the cells wherever placement
/// has them at the moment, for measuring congestion while placing.
///
/// This is how `OpenROAD`'s placer runs its global router. The routes are
/// negotiated for [`ESTIMATE_ROUNDS`] rounds, so that the congestion cheap
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
        check_reserve(config)?;
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

    /// The grid's capacities and what fixed metal takes of them.
    pub fn capacity_reduction(&self) -> CapacityReduction {
        self.grid.capacity_reduction(&self.tech)
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

fn check_reserve(config: &RoutingConfig) -> Result<(), RouteError> {
    if (0.0..1.0).contains(&config.global_capacity_reserve) {
        Ok(())
    } else {
        Err(RouteError::CapacityReserve(config.global_capacity_reserve))
    }
}

/// The edge of a gcell: `config.gcell_tracks` gcell pitches.
fn gcell_size(tech: &RouteTech, config: &RoutingConfig) -> Dbu {
    tech.gcell_pitch * i64::from(config.gcell_tracks.max(1))
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

fn stage(label: &str, start: Instant) {
    let resident = memory::current_rss_mb().map_or_else(String::new, |mb| format!("{mb} MB"));
    ui::stat_row(
        label,
        &format!("{:.1}s", start.elapsed().as_secs_f64()),
        "Resident:",
        &resident,
    );
}

/// Why routing could not run.
#[derive(Debug, thiserror::Error, PartialEq)]
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
    /// The global capacity reserve is outside `[0, 1)`.
    #[error("global_capacity_reserve must be in [0, 1), got {0}")]
    CapacityReserve(f64),
    /// The configured routing layers are unusable.
    #[error(transparent)]
    Layers(#[from] LayerRangeError),
    /// A routable layer has no tracks and no pitch to derive them from.
    #[error("routing layer '{0}' has no tracks")]
    NoTracks(String),
    /// No usable via is defined through a cut layer.
    #[error("no via is defined through cut layer '{0}'")]
    NoVia(String),
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
}
