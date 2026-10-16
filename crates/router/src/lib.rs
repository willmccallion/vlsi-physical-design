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
pub mod tech;
pub mod track;
pub mod tree;
pub mod wiring;

use detailed::{Costs, DetailedRouter, Scope, routable_nets};
pub use global::CapacityReduction;
use global::{GlobalGrid, GlobalNet, route_global};
use occupancy::FixedTracks;
use pare_common::db::design::{GCellGridDef, NetlistDB, PlacementStatus, RouteElem, TrackAxis};
use pare_common::db::indices::NetId;
use pare_common::db::library::Shape;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::util::config::{LayerRangeError, RoutingConfig};
use pare_common::util::ui;
use pare_common::verify::shapes::obstruction_shapes;
use search::MAX_PENALTY;
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

/// Routes every signal net of `db` and stores the wiring in the design.
pub fn route(db: &mut NetlistDB, config: &RoutingConfig) -> Result<RoutingSummary, RouteError> {
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
    let tech = RouteTech::new(db, config)?;
    let gcell = gcell_size(&tech, config);
    let pitch = tech.layer(tech.min_layer).pitch().to_f64();
    let mut costs = Costs {
        via: 4.0 * pitch,
        penalty: 8.0 * pitch,
        history: 4.0 * pitch,
    };

    let (routes, summary, gcells) = {
        let start = Instant::now();
        let mut dr = DetailedRouter::new(db, &tech, gcell);
        let nets = routable_nets(db);
        stage("Pin access:", start);

        let start = Instant::now();
        let mut grid = GlobalGrid::new(
            &tech,
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
                })
            })
            .collect();
        let global = route_global(&mut grid, &global_nets, config.global_iterations);
        let (nx, ny) = grid.size();
        stage("Global route:", start);
        ui::stat_row(
            "GCells:",
            &format!("{nx}x{ny}"),
            "GR overflow:",
            &grid.overflow().to_string(),
        );
        let start = Instant::now();
        let wires = assign_tracks(&tech, &dr.occupancy, &grid, &global, &dr.pin_points());
        dr.set_guides(global, &grid);
        dr.set_track_wires(wires);
        stage("Track assign:", start);
        let mut summary = RoutingSummary {
            nets: nets.len(),
            inaccessible: dr.inaccessible_nets(),
            ..RoutingSummary::default()
        };
        let start = Instant::now();
        let first = dr.route_initial(&nets, costs);
        ui::dr_table_header();
        let mut conflicted = dr.conflicted_nets(1.0);
        ui::dr_iter(
            0,
            dr.marker_count(),
            conflicted.len(),
            first.pieces,
            costs.penalty / pitch,
            start.elapsed().as_millis(),
        );
        let mut scope = Scope::Conflicts;
        let mut progress = Progress::new(dr.marker_count());
        for round in 1..=config.detailed_iterations {
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
            costs.penalty = (costs.penalty * 1.5).min(MAX_PENALTY);
            let repaired = dr.repair(costs, round, scope);
            let before = conflicted.len();
            conflicted = dr.conflicted_nets(1.0);
            scope = next_scope(scope, conflicted.len() < before);
            progress.record(dr.marker_count());
            summary.rounds = round;
            ui::dr_iter(
                round,
                dr.marker_count(),
                conflicted.len(),
                repaired.pieces,
                costs.penalty / pitch,
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

/// Every gcell's routing capacity and what global routing loses of it
/// before any cell is placed.
///
/// The losses are the capacity reserve and the metal placement cannot move:
/// that of fixed cells and I/O pins, special nets and routing blockages.
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

fn check_reserve(config: &RoutingConfig) -> Result<(), RouteError> {
    if (0.0..1.0).contains(&config.global_capacity_reserve) {
        Ok(())
    } else {
        Err(RouteError::CapacityReserve(config.global_capacity_reserve))
    }
}

/// The edge of a gcell: `config.gcell_tracks` pitches of the lowest
/// routing layer.
fn gcell_size(tech: &RouteTech, config: &RoutingConfig) -> Dbu {
    tech.layer(tech.min_layer).pitch() * i64::from(config.gcell_tracks.max(1))
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
    ui::stat_row(
        label,
        &format!("{:.1}s", start.elapsed().as_secs_f64()),
        "",
        "",
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
}
