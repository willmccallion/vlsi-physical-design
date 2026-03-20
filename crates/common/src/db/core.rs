//! Core Netlist Database Data Structures.
//!
//! This module defines the central `NetlistDB` structure that holds all design
//! information including cells, nets, pins, routing layers, and spatial
//! relationships. It serves as the single source of truth for the design state
//! throughout the placement and routing algorithms.

use crate::db::indices::{CellId, NetId, PinId};
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use std::collections::HashMap;

/// Routing direction for a metal layer.
///
/// Determines the preferred routing orientation for wires on a given layer.
/// Vertical layers route primarily in the Y direction, horizontal layers in
/// the X direction. This alternation enables efficient routing by allowing
/// orthogonal wire crossings without shorts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LayerDirection {
    /// Wires on this layer route primarily in the vertical (Y) direction.
    Vertical,
    /// Wires on this layer route primarily in the horizontal (X) direction.
    Horizontal,
    /// Layer direction is unspecified or unknown.
    Unknown,
}

/// Metadata for a single routing layer in the metal stack.
///
/// Contains the physical properties of a metal layer including its routing
/// direction, track pitch (spacing between adjacent tracks), and wire width.
/// The index field provides a unique identifier for the layer within the
/// stack, typically corresponding to metal layer numbers (M1, M2, etc.).
#[derive(Clone, Debug)]
pub struct LayerData {
    /// Human-readable layer name (e.g., "M1", "M2").
    pub name: String,
    /// Zero-based index of this layer in the layer stack.
    pub index: u8,
    /// Preferred routing direction for wires on this layer.
    pub direction: LayerDirection,
    /// Spacing between adjacent routing tracks in physical units.
    pub pitch: f64,
    /// Width of wires on this layer in physical units.
    pub width: f64,
    /// Minimum spacing between wires from different nets on this layer.
    /// Derived from (pitch - width) or explicitly set from LEF SPACING rules.
    pub min_spacing: f64,
}

/// A single wire segment connecting two points on a routing layer.
///
/// Represents a straight-line wire segment between two physical coordinates
/// on a specific metal layer. Used to store the final routed net topology
/// after detailed routing completes. Degenerate segments (p1 == p2) represent
/// vias for layer transitions.
#[derive(Clone, Debug)]
pub struct RouteSegment {
    /// Layer index on which this segment resides.
    pub layer: u8,
    /// Starting point of the wire segment in physical coordinates.
    pub p1: Point<f64>,
    /// Ending point of the wire segment in physical coordinates.
    pub p2: Point<f64>,
}

/// Data structure representing a single cell instance in the design.
///
/// Contains the cell's name, library reference, physical dimensions, placement
/// constraints, and list of pins. The `is_fixed` flag indicates that the cell
/// position is predetermined (e.g., I/O pads, pre-placed macros) and should
/// not be moved by the placer. The `is_macro` flag distinguishes large blocks
/// from standard cells for legalization purposes.
#[derive(Clone, Debug)]
pub struct CellData {
    /// Instance name of this cell in the design hierarchy.
    pub name: String,
    /// Library cell name that defines this cell's physical properties.
    pub lib_name: String,
    /// Width of the cell in physical units.
    pub width: f64,
    /// Height of the cell in physical units.
    pub height: f64,
    /// Whether this cell's position is fixed and cannot be moved.
    pub is_fixed: bool,
    /// Whether this cell is a macro (large block) rather than a standard cell.
    pub is_macro: bool,
    /// List of pin identifiers belonging to this cell.
    pub pins: Vec<PinId>,
}

/// Data structure representing a single net (signal) in the design.
///
/// A net connects multiple pins together, forming the electrical connectivity
/// of the circuit. The weight field allows critical nets to be prioritized
/// during placement and routing. The `route_segments` field stores the final
/// wire paths after routing completes.
#[derive(Clone, Debug)]
pub struct NetData {
    /// Name of the net in the design hierarchy.
    pub name: String,
    /// Weight factor for prioritizing this net during optimization.
    pub weight: f64,
    /// List of pin identifiers that this net connects.
    pub pins: Vec<PinId>,
    /// Final routed wire segments for this net after detailed routing.
    pub route_segments: Vec<RouteSegment>,
}

/// Definition of a routing track grid for a specific layer.
///
/// Specifies the regular grid of routing tracks that wires must align to
/// on a given layer. Tracks are uniformly spaced starting from the start
/// coordinate with the specified step size. This information is parsed
/// from DEF TRACKS statements and used to constrain wire placement during
/// detailed routing.
#[derive(Clone, Debug, Default)]
pub struct TrackDef {
    /// Layer name for which these tracks are defined.
    pub layer: String,
    /// Direction of the tracks ("X" for horizontal, "Y" for vertical).
    pub direction: String,
    /// Starting coordinate of the first track.
    pub start: f64,
    /// Number of tracks in this grid.
    pub num_tracks: u32,
    /// Spacing between adjacent tracks.
    pub step: f64,
}

/// A resolved grid of routing track coordinates for a single layer.
///
/// Contains the sorted physical coordinates of all legal routing tracks
/// on a layer. Used to snap routed wire coordinates to legal track positions
/// during segment generation.
#[derive(Clone, Debug)]
pub struct TrackGrid {
    /// Sorted track coordinates in physical units.
    pub coords: Vec<f64>,
}

impl TrackGrid {
    /// Builds a track grid from a DEF `TrackDef`.
    pub fn from_track_def(td: &TrackDef) -> Self {
        let mut coords = Vec::with_capacity(td.num_tracks as usize);
        for i in 0..td.num_tracks {
            coords.push((i as f64).mul_add(td.step, td.start));
        }
        Self { coords }
    }

    /// Builds a track grid from pitch and die extent (fallback when no `TrackDef` exists).
    pub fn from_pitch(die_min: f64, die_max: f64, pitch: f64) -> Self {
        if pitch <= 0.0 {
            return Self { coords: Vec::new() };
        }
        let mut coords = Vec::new();
        let n = ((die_max - die_min) / pitch).floor() as usize + 1;
        for i in 0..n {
            let pos = (i as f64).mul_add(pitch, die_min);
            if pos > die_max {
                break;
            }
            coords.push(pos);
        }
        Self { coords }
    }

    /// Snaps a coordinate to the nearest track using binary search.
    /// Returns the original coordinate if no tracks are available.
    pub fn snap(&self, coord: f64) -> f64 {
        if self.coords.is_empty() {
            return coord;
        }
        let idx = self.coords.partition_point(|&t| t < coord);
        if idx == 0 {
            self.coords[0]
        } else if idx >= self.coords.len() {
            self.coords[self.coords.len() - 1]
        } else {
            let lo = self.coords[idx - 1];
            let hi = self.coords[idx];
            if (coord - lo) <= (hi - coord) { lo } else { hi }
        }
    }
}

/// Central database structure containing all design information.
///
/// This is the primary data structure that holds the complete netlist,
/// including cells, nets, pins, routing layers, and spatial information.
/// It serves as the interface between parsers, placement algorithms, and
/// routing algorithms. All design modifications during placement and routing
/// are reflected in this structure.
#[derive(Debug)]
pub struct NetlistDB {
    /// Metal routing layers in the technology stack, ordered by index.
    pub layers: Vec<LayerData>,
    /// All cell instances in the design (standard cells, macros, pads).
    pub cells: Vec<CellData>,
    /// All signal and power nets connecting pins across the design.
    pub nets: Vec<NetData>,
    /// Routing track definitions per layer for legal wire positions.
    pub tracks: Vec<TrackDef>,

    /// Pin offsets relative to their parent cell's origin.
    pub pin_offsets: Vec<Point<f64>>,
    /// Human-readable pin names for reporting and debugging.
    pub pin_names: Vec<String>,
    /// Maps each pin to the cell instance that owns it.
    pub pin_to_cell: Vec<CellId>,
    /// Maps each pin to the net it belongs to.
    pub pin_to_net: Vec<NetId>,

    /// Current physical positions of each cell instance.
    pub positions: Vec<Point<f64>>,
    /// Bounding rectangle of the chip die area.
    pub die_area: Rect,

    /// Lookup table from cell name to cell identifier.
    pub cell_name_map: HashMap<String, CellId>,
    /// Lookup table from net name to net identifier.
    pub net_name_map: HashMap<String, NetId>,
    /// Lookup table from layer name to layer index.
    pub layer_name_map: HashMap<String, u8>,

    /// Pin offset data for each macro, keyed by macro name then pin name.
    pub macro_pins: HashMap<String, HashMap<String, Point<f64>>>,
    /// Width and height dimensions for each macro type.
    pub macro_sizes: HashMap<String, (f64, f64)>,
}

impl Default for NetlistDB {
    fn default() -> Self {
        Self::new()
    }
}

impl NetlistDB {
    /// Creates a new empty netlist database with pre-allocated capacity.
    ///
    /// Initializes all vectors and hash maps with default values and
    /// reasonable initial capacities to reduce reallocation overhead
    /// during parsing and optimization. The die area is set to a default
    /// empty rectangle that will be populated by the parser.
    pub fn new() -> Self {
        Self {
            layers: Vec::new(),
            cells: Vec::with_capacity(1000),
            nets: Vec::with_capacity(1000),
            tracks: Vec::new(),
            pin_offsets: Vec::with_capacity(5000),
            pin_names: Vec::with_capacity(5000),
            pin_to_cell: Vec::with_capacity(5000),
            pin_to_net: Vec::with_capacity(5000),
            positions: Vec::with_capacity(1000),
            die_area: Rect::default(),
            cell_name_map: HashMap::new(),
            net_name_map: HashMap::new(),
            layer_name_map: HashMap::new(),
            macro_pins: HashMap::new(),
            macro_sizes: HashMap::new(),
        }
    }

    /// Returns the total number of cell instances in the design.
    ///
    /// This count includes all cells regardless of whether they are fixed,
    /// movable, standard cells, or macros.
    pub const fn num_cells(&self) -> usize {
        self.cells.len()
    }
    /// Returns the total number of nets in the design.
    ///
    /// This includes all nets regardless of pin count or connectivity.
    pub const fn num_nets(&self) -> usize {
        self.nets.len()
    }

    /// Computes the absolute physical position of a pin given its cell's position.
    ///
    /// Adds the pin's offset (relative to the cell origin) to the cell's
    /// position to obtain the pin's absolute coordinates in the design space.
    /// This is used extensively during wirelength calculation and routing
    /// to determine where wires must connect.
    #[inline]
    pub fn get_pin_position(&self, pin: PinId, cell_pos: &Point<f64>) -> Point<f64> {
        let offset = self.pin_offsets[pin.index()];
        *cell_pos + offset
    }

    /// Adds a new routing layer to the metal stack.
    ///
    /// Creates a `LayerData` entry with the specified properties and assigns
    /// it the next available layer index. Updates the layer name map for
    /// efficient lookup by name. This is called during LEF parsing and when
    /// synthesizing default layers for formats that don't specify layer information.
    pub fn add_layer(&mut self, name: String, direction: LayerDirection, pitch: f64, width: f64) {
        let idx = self.layers.len() as u8;
        let min_spacing = (pitch - width).max(0.0);
        let _ = self.layer_name_map.insert(name.clone(), idx);
        self.layers.push(LayerData {
            name,
            index: idx,
            direction,
            pitch,
            width,
            min_spacing,
        });
    }

    /// Sets an explicit minimum spacing for a layer, overriding the default
    /// (pitch - width) value. Called when LEF SPACING rules are parsed.
    pub fn set_layer_spacing(&mut self, layer_name: &str, spacing: f64) {
        if let Some(&idx) = self.layer_name_map.get(layer_name) {
            self.layers[idx as usize].min_spacing = spacing;
        }
    }

    /// Adds a new cell instance to the design and returns its identifier.
    ///
    /// Creates a `CellData` entry with the specified properties, initializes
    /// its position to the origin, and updates the cell name map for lookup.
    /// The cell is initially marked as not a macro; this can be updated later
    /// based on size heuristics or explicit annotations.
    pub fn add_cell(
        &mut self,
        name: String,
        lib_name: String,
        width: f64,
        height: f64,
        is_fixed: bool,
    ) -> CellId {
        let id = CellId::new(self.cells.len());
        self.cells.push(CellData {
            name: name.clone(),
            lib_name,
            width,
            height,
            is_fixed,
            is_macro: false,
            pins: Vec::new(),
        });
        self.positions.push(Point::new(0.0, 0.0));
        let _ = self.cell_name_map.insert(name, id);
        id
    }

    /// Adds a new net to the design or returns the existing net's ID if it already exists.
    ///
    /// Checks the net name map first to avoid duplicate net creation. If the net
    /// is new, creates a `NetData` entry with default weight and empty pin/segment
    /// lists. This deduplication is important for formats that may reference
    /// the same net multiple times.
    pub fn add_net(&mut self, name: String) -> NetId {
        if let Some(&id) = self.net_name_map.get(&name) {
            return id;
        }
        let id = NetId::new(self.nets.len());
        self.nets.push(NetData {
            name: name.clone(),
            weight: 1.0,
            pins: Vec::new(),
            route_segments: Vec::new(),
        });
        let _ = self.net_name_map.insert(name, id);
        id
    }

    /// Adds a new pin to the design and associates it with a cell and net.
    ///
    /// Creates pin data structures including offset, name, and bidirectional
    /// mappings between pins, cells, and nets. Updates both the cell's pin
    /// list and the net's pin list to maintain referential integrity. The
    /// offset is relative to the cell's origin and is used to compute absolute
    /// pin positions during placement and routing.
    pub fn add_pin(&mut self, cell: CellId, net: NetId, offset: Point<f64>, name: String) {
        let pid = PinId::new(self.pin_offsets.len());
        self.pin_offsets.push(offset);
        self.pin_names.push(name);
        self.pin_to_cell.push(cell);
        self.pin_to_net.push(net);

        self.cells[cell.index()].pins.push(pid);
        self.nets[net.index()].pins.push(pid);
    }

    /// Builds per-layer track grids for snapping wire coordinates.
    ///
    /// For each routing layer, uses the first matching `TrackDef` if available,
    /// otherwise falls back to generating tracks from layer pitch and die extent.
    /// Returns a Vec indexed by layer index.
    pub fn build_track_grids(&self) -> Vec<TrackGrid> {
        let mut grids = Vec::with_capacity(self.layers.len());
        for layer in &self.layers {
            // Try to find a TrackDef for this layer
            let td = self.tracks.iter().find(|t| t.layer == layer.name);
            let grid = if let Some(td) = td {
                TrackGrid::from_track_def(td)
            } else if layer.pitch > 0.0 {
                // Use layer direction to decide which die extent to cover
                match layer.direction {
                    LayerDirection::Horizontal => {
                        // Horizontal layer: tracks run horizontally, snapping Y
                        TrackGrid::from_pitch(self.die_area.min.y, self.die_area.max.y, layer.pitch)
                    }
                    LayerDirection::Vertical => {
                        // Vertical layer: tracks run vertically, snapping X
                        TrackGrid::from_pitch(self.die_area.min.x, self.die_area.max.x, layer.pitch)
                    }
                    LayerDirection::Unknown => {
                        TrackGrid::from_pitch(self.die_area.min.x, self.die_area.max.x, layer.pitch)
                    }
                }
            } else {
                TrackGrid { coords: Vec::new() }
            };
            grids.push(grid);
        }
        grids
    }

    /// Returns the identifier of the virtual I/O cell, creating it if necessary.
    ///
    /// The virtual I/O cell is a special cell that holds all I/O pins that
    /// are not associated with regular cell instances. This allows I/O pins
    /// to be treated uniformly in the database structure. The cell is created
    /// with zero dimensions and fixed placement, and its position is never
    /// modified by the placer.
    pub fn get_or_create_io_cell(&mut self) -> CellId {
        if let Some(&id) = self.cell_name_map.get("IO_VIRTUAL_CELL") {
            return id;
        }
        let id = self.add_cell(
            "IO_VIRTUAL_CELL".to_string(),
            "IO_LIB".to_string(),
            0.0,
            0.0,
            true,
        );
        self.positions[id.index()] = Point::new(0.0, 0.0);
        id
    }
}
