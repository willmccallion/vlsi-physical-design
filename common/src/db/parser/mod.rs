//! Netlist Parser Modules for Industry-Standard Formats.
//!
//! This module contains parsers for three major input formats:
//! - LEF (Library Exchange Format): Defines cell libraries and routing layers
//! - DEF (Design Exchange Format): Defines the netlist, placements, and routing
//! - Bookshelf: Academic placement benchmark format with separate files for
//!   nodes, nets, placement, and site/row definitions

pub mod bookshelf;
/// Parser for DEF (Design Exchange Format) files.
///
/// Processes industry-standard DEF format to extract netlist connectivity,
/// component placements, pin locations, routing track definitions, and die
/// area specifications. Handles unit conversion from DEF's micron-based
/// coordinates to internal floating-point representation. The parser maintains
/// state for the current section (COMPONENTS, PINS, NETS) and populates the
/// database incrementally as sections are processed.
pub mod def;
/// Parser for LEF (Library Exchange Format) files.
///
/// Processes industry-standard LEF format to extract cell library definitions
/// including macro cell dimensions, pin locations within cells, and routing
/// layer properties (pitch, width, direction). Maintains parsing state to
/// track the current layer, macro, and pin being processed. If no layers are
/// found, synthesizes a default 6-layer routing stack for compatibility.
pub mod lef;
