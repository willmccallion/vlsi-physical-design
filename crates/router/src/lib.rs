//! Routing Algorithms for Netlist Connectivity.
//!
//! This crate implements two-stage routing: global routing to generate coarse
//! guides and detailed routing to produce final wire paths. Uses A* pathfinding
//! with congestion-aware cost functions and rip-up-and-reroute to resolve
//! routing conflicts.

pub mod algo;
/// Detailed routing on a fine grid using global routing guides.
///
/// Executes detailed routing on a fine grid using global routing guides as
/// constraints. Performs initial routing of all nets in batches, then iteratively
/// reroutes congested nets using rip-up-and-reroute. Uses spatial batching to
/// parallelize routing while avoiding conflicts. Tracks congestion history and
/// adaptively increases penalties for persistent congestion. Generates routing
/// segments from grid paths and updates the database. Includes oracle for guide
/// constraint checking, router for A* pathfinding, and scheduler for spatial
/// batching of routing operations.
pub mod detailed;
/// Global routing on a coarse grid to generate routing guides.
///
/// Routes all nets on a coarse grid (typically 100-200x coarser than detailed
/// routing) to determine preferred routing regions. Uses A* pathfinding with
/// congestion-aware costs and rip-up-and-reroute to resolve conflicts. The
/// resulting paths are expanded into guides that constrain detailed routing,
/// improving routability and reducing runtime by focusing detailed routing on
/// feasible regions.
pub mod global;
/// Routing grid data structures for tracking wire occupancy and congestion.
///
/// Defines the interface and implementation for routing grids that track wire
/// occupancy, obstacles, and congestion. The grid is used by routing algorithms
/// to check feasibility and compute routing costs. Provides methods for querying
/// and modifying grid state including obstacles, wire occupancy, congestion,
/// and routing costs. Implementations must be thread-safe to support parallel
/// routing operations.
pub mod grid;
/// Utility modules for routing coordinate conversion and transformations.
///
/// Provides coordinate conversion between physical coordinates (floating-point
/// microns) and discrete grid coordinates (integer indices). Handles coordinate
/// system transformations including origin offsets and scaling factors. The
/// converter maintains scaling factors and origin offsets to transform between
/// continuous physical coordinate space and discrete grid space used by routing
/// algorithms.
pub mod utils;

use pare_common::db::core::NetlistDB;
use pare_common::util::config::{DetailedRoutingConfig, GlobalRoutingConfig};

/// Result from the routing workflow including congestion data for feedback.
#[derive(Debug)]
pub struct RoutingResult {
    /// Flat congestion map (row-major): ratio of usage/capacity per `GCell`.
    /// Values > 1.0 indicate overflow.
    pub congestion_map: Vec<f32>,
    /// Width of the congestion grid in `GCells`.
    pub congestion_grid_w: u32,
    /// Height of the congestion grid in `GCells`.
    pub congestion_grid_h: u32,
}

/// Executes the complete routing workflow: global routing followed by detailed routing.
///
/// First performs global routing on a coarse grid to generate routing guides,
/// then executes detailed routing on a fine grid using these guides. The guides
/// constrain detailed routing to preferred regions, improving routability and
/// reducing runtime. Returns a `RoutingResult` containing congestion data that
/// can be fed back to the placer for congestion-driven placement iterations.
pub fn route(
    db: &mut NetlistDB,
    global_config: &GlobalRoutingConfig,
    detailed_config: &DetailedRoutingConfig,
) -> Result<RoutingResult, String> {
    let (guides, coarse_converter, congestion_map, grid_w, grid_h) =
        global::run(db, global_config)?;

    detailed::run(db, detailed_config, &guides, &coarse_converter)?;

    Ok(RoutingResult {
        congestion_map,
        congestion_grid_w: grid_w,
        congestion_grid_h: grid_h,
    })
}
