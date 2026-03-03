//! Netlist Database and Parser Modules.
//!
//! This module contains the core data structures for representing the design
//! netlist (cells, nets, pins, layers) and parsers for industry-standard formats
//! including LEF (Library Exchange Format), DEF (Design Exchange Format), and
//! Bookshelf placement format.

pub mod core;
/// Type-safe index types for netlist database entities.
///
/// Defines strongly-typed index wrappers (CellId, NetId, PinId, RowId) that
/// prevent accidental mixing of indices from different collections. These
/// indices are implemented as transparent wrappers around u32 with zero-cost
/// abstraction, providing compile-time safety without runtime overhead.
pub mod indices;
/// Parsers for industry-standard netlist and library formats.
///
/// Contains parsers for LEF (Library Exchange Format), DEF (Design Exchange
/// Format), and Bookshelf placement benchmark format. These parsers extract
/// cell libraries, routing layer definitions, netlist connectivity, placements,
/// and routing information into the unified database representation.
pub mod parser;
