//! Geometric Data Structures and Spatial Indexing.
//!
//! This module provides fundamental geometric primitives (points, rectangles,
//! coordinates) and spatial indexing structures for efficient range queries
//! and overlap detection. These are used throughout placement and routing
//! for spatial reasoning and collision detection.

pub mod coord;
/// Two-dimensional point type with generic coordinate representation.
///
/// Provides a generic point type parameterized by coordinate type (f64 for
/// physical coordinates, i32 for integer coordinates) with arithmetic operations
/// for vector math. Used throughout the codebase to represent physical positions,
/// offsets, and spatial relationships. The C representation ensures compatibility
/// with external libraries that expect struct layout.
pub mod point;
/// Axis-aligned rectangle type defined by minimum and maximum corners.
///
/// Represents rectangular regions in 2D space for die areas, cell bounding boxes,
/// and spatial queries. The min point is the bottom-left corner and the max point
/// is the top-right corner. This structure is used extensively in placement and
/// routing algorithms for overlap detection, boundary checking, and spatial
/// region queries.
pub mod rect;
/// R-tree spatial index for efficient range queries and overlap detection.
///
/// Provides logarithmic-time queries for rectangles that intersect a given query
/// region using an R-tree data structure. Used during placement to quickly find
/// cells that may overlap with a given region without checking every cell in the
/// design. The index maintains rectangles with associated identifiers to map
/// query results back to cell indices or other entities.
pub mod rtree;
