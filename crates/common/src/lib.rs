//! Common Data Structures and Utilities for EDA Toolchain.
//!
//! This crate provides the foundational data structures and algorithms shared
//! across placement and routing modules. It includes the netlist database that
//! represents the design hierarchy, geometric primitives for spatial operations,
//! and utility functions for configuration, logging, visualization, and verification.

pub mod db;
/// Geometric primitives and spatial indexing structures.
///
/// Provides fundamental geometric types (points, rectangles, coordinates) and
/// spatial indexing data structures (R-trees) for efficient range queries and
/// overlap detection. These are used throughout placement and routing for
/// spatial reasoning, collision detection, and coordinate transformations.
pub mod geom;
/// Utility modules for configuration, logging, verification, and visualization.
///
/// Provides supporting functionality including configuration management from
/// TOML files, logging infrastructure setup, design rule checking (DRC) and
/// layout-versus-schematic (LVS) verification, synthetic benchmark generation,
/// performance profiling, and visualization utilities for debugging and analysis.
pub mod util;
