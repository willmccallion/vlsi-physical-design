//! Pathfinding Algorithms for Routing.
//!
//! Contains the A* pathfinding algorithm used by both global and detailed
//! routing to find wire paths between pins. The algorithm uses congestion-aware
//! cost functions and guide constraints to find high-quality routes.

pub mod astar;
pub mod pattern;
