//! The placed and routed design: instances, nets, rows, tracks and wiring.

use crate::db::indices::{
    CellId, CutLayerId, IoPinId, LayerId, MacroId, MacroPinId, NetId, NonDefaultRuleId, PinId,
    RegionId, SiteId, ViaId, ViaRuleId,
};
use crate::db::library::{DuplicateName, LayerRef, Library, Macro, PinDirection, Shape, SignalUse};
use crate::geom::dbu::{Dbu, DbuPerMicron};
use crate::geom::orient::Orientation;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use std::collections::HashMap;

/// DEF placement status of a component or pin.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PlacementStatus {
    /// Has no location yet.
    #[default]
    Unplaced,
    /// Placed, but tools may move it.
    Placed,
    /// Placed and must not move.
    Fixed,
    /// Fixed at the chip level, such as a bump.
    Cover,
}

impl PlacementStatus {
    /// Whether placement may move the object.
    pub const fn is_movable(self) -> bool {
        matches!(self, Self::Unplaced | Self::Placed)
    }

    /// The DEF keyword, or `None` for unplaced.
    pub const fn def_keyword(self) -> Option<&'static str> {
        match self {
            Self::Unplaced => None,
            Self::Placed => Some("PLACED"),
            Self::Fixed => Some("FIXED"),
            Self::Cover => Some("COVER"),
        }
    }
}

/// A placed instance of a cell master.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Cell {
    /// Instance name.
    pub name: String,
    /// The master it instantiates.
    pub macro_id: MacroId,
    /// Lower-left corner of the oriented bounding box.
    pub location: Point<Dbu>,
    /// Placement orientation.
    pub orient: Orientation,
    /// Placement status.
    pub status: PlacementStatus,
    /// The region the instance is constrained to, if any.
    pub region: Option<RegionId>,
    /// Net connections on this instance.
    pub pins: Vec<PinId>,
}

/// How a region constrains the cells assigned to it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegionKind {
    /// Assigned cells must lie inside; other cells may too (no `TYPE`).
    Inclusive,
    /// Assigned cells must lie inside and no other cell may (`FENCE`).
    Fence,
    /// Assigned cells should lie inside (`GUIDE`).
    Guide,
}

impl RegionKind {
    /// The DEF `TYPE` keyword, or `None` for an inclusive region.
    pub const fn def_keyword(self) -> Option<&'static str> {
        match self {
            Self::Inclusive => None,
            Self::Fence => Some("FENCE"),
            Self::Guide => Some("GUIDE"),
        }
    }
}

/// A placement region from the DEF `REGIONS` section.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Region {
    /// Region name.
    pub name: String,
    /// The area, as a union of rectangles.
    pub rects: Vec<Rect<Dbu>>,
    /// How it constrains its cells.
    pub kind: RegionKind,
}

/// A metal layer's width and spacing under a non-default rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NonDefaultLayer {
    /// The layer.
    pub layer: LayerId,
    /// Wire width.
    pub width: Dbu,
    /// Spacing to other nets, if it differs from the layer's.
    pub spacing: Option<Dbu>,
    /// How far wires extend past their ends, if not half the width.
    pub wire_extension: Option<Dbu>,
}

/// A DEF `NONDEFAULTRULES` entry: wider wires, spacing or vias for some nets.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct NonDefaultRule {
    /// Rule name.
    pub name: String,
    /// Whether the spacing is a hard rule rather than a preference.
    pub hard_spacing: bool,
    /// Per-layer widths and spacings.
    pub layers: Vec<NonDefaultLayer>,
    /// Vias the rule's nets may use.
    pub vias: Vec<ViaId>,
    /// Via generation rules the rule's nets may use.
    pub via_rules: Vec<ViaRuleId>,
    /// Least number of cuts per via on a cut layer.
    pub min_cuts: Vec<(CutLayerId, u32)>,
}

/// A row of placement sites.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Row {
    /// Row name.
    pub name: String,
    /// Site the row is made of.
    pub site: SiteId,
    /// Lower-left corner of the first site.
    pub origin: Point<Dbu>,
    /// Orientation of every site in the row.
    pub orient: Orientation,
    /// Number of sites along X.
    pub num_x: u32,
    /// Number of sites along Y.
    pub num_y: u32,
    /// Distance between site origins.
    pub step: Point<Dbu>,
}

impl Row {
    /// The area the row's sites cover.
    pub fn bbox(&self, library: &Library) -> Rect<Dbu> {
        let site = self.orient.oriented_size(library.site(self.site).size);
        let last = |n: u32, step: Dbu| step * i64::from(n.saturating_sub(1));
        Rect::new(
            self.origin,
            Point::new(
                self.origin.x + last(self.num_x, self.step.x) + site.x,
                self.origin.y + last(self.num_y, self.step.y) + site.y,
            ),
        )
    }
}

/// Which coordinate a set of tracks is spaced along.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackAxis {
    /// Vertical tracks at a sequence of X positions.
    X,
    /// Horizontal tracks at a sequence of Y positions.
    Y,
}

impl TrackAxis {
    /// The DEF keyword.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::X => "X",
            Self::Y => "Y",
        }
    }
}

/// A DEF `GCELLGRID` statement: evenly spaced gcell boundaries along one
/// axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GCellGridDef {
    /// Coordinate the boundaries are spaced along.
    pub axis: TrackAxis,
    /// Position of the first boundary.
    pub start: Dbu,
    /// Number of boundaries.
    pub count: u32,
    /// Distance between boundaries.
    pub step: Dbu,
}

/// A DEF `TRACKS` statement.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TrackDef {
    /// Coordinate the tracks are spaced along.
    pub axis: TrackAxis,
    /// Position of the first track.
    pub start: Dbu,
    /// Number of tracks.
    pub count: u32,
    /// Distance between tracks.
    pub step: Dbu,
    /// Layers the tracks apply to.
    pub layers: Vec<LayerId>,
}

impl TrackDef {
    /// Track positions in increasing order.
    pub fn coords(&self) -> impl Iterator<Item = Dbu> + '_ {
        (0..i64::from(self.count)).map(|i| self.start + self.step * i)
    }
}

/// A top-level I/O pin of the design.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IoPin {
    /// Pin name.
    pub name: String,
    /// Net the pin belongs to.
    pub net: NetId,
    /// Signal direction.
    pub direction: PinDirection,
    /// What the pin carries.
    pub signal_use: SignalUse,
    /// Geometry relative to `location`, before orientation.
    pub shapes: Vec<Shape>,
    /// Placement point.
    pub location: Point<Dbu>,
    /// Orientation applied to `shapes`.
    pub orient: Orientation,
    /// Placement status.
    pub status: PlacementStatus,
    /// The net connection for this pin.
    pub pin: PinId,
}

impl IoPin {
    /// Shapes in design coordinates.
    pub fn placed_shapes(&self) -> impl Iterator<Item = Shape> + '_ {
        self.shapes.iter().map(|s| Shape {
            layer: s.layer,
            rect: self.orient.apply_rect(&s.rect).translate(self.location),
        })
    }
}

/// What a net connection is attached to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PinOwner {
    /// A pin of a placed instance.
    Cell {
        /// The instance.
        cell: CellId,
        /// Which pin of its master.
        pin: MacroPinId,
    },
    /// A top-level I/O pin.
    Io(IoPinId),
}

/// One connection of a net.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pin {
    /// What the connection is on.
    pub owner: PinOwner,
    /// The net it belongs to.
    pub net: NetId,
}

/// A piece of regular net wiring.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RouteElem {
    /// A default-width wire between two points on one layer; both ends extend
    /// by half the wire width, as DEF specifies for regular wiring.
    Wire {
        /// Layer of the wire.
        layer: LayerId,
        /// One end of the centerline.
        from: Point<Dbu>,
        /// The other end of the centerline.
        to: Point<Dbu>,
    },
    /// A via instance centered at a point.
    Via {
        /// The via definition.
        via: ViaId,
        /// Its center.
        at: Point<Dbu>,
    },
    /// A rectangle of metal patched onto the wiring (DEF `RECT`), as routers
    /// add to meet minimum area or enclosure.
    Patch {
        /// Layer of the patch.
        layer: LayerId,
        /// The rectangle.
        rect: Rect<Dbu>,
    },
}

/// A signal net.
#[derive(Clone, Debug, PartialEq)]
pub struct Net {
    /// Net name.
    pub name: String,
    /// What the net carries.
    pub signal_use: SignalUse,
    /// Its connections.
    pub pins: Vec<PinId>,
    /// Placement weight.
    pub weight: f64,
    /// The non-default routing rule the net uses, if any.
    pub non_default_rule: Option<NonDefaultRuleId>,
    /// Routed wiring.
    pub routing: Vec<RouteElem>,
}

/// A wire segment of a special (power) net.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SpecialWire {
    /// Layer of the wire.
    pub layer: LayerId,
    /// Wire width.
    pub width: Dbu,
    /// One end of the centerline.
    pub from: Point<Dbu>,
    /// The other end of the centerline.
    pub to: Point<Dbu>,
    /// How far the wire extends past `from` and past `to`; zero unless the
    /// DEF path gives an extension.
    pub extensions: (Dbu, Dbu),
    /// The DEF `SHAPE` keyword, such as `STRIPE` or `FOLLOWPIN`.
    pub shape: Option<String>,
}

impl SpecialWire {
    /// The rectangle the wire covers.
    pub fn rect(&self) -> Rect<Dbu> {
        let half = self.width / 2;
        let (from_ext, to_ext) = self.extensions;
        if self.from.y == self.to.y {
            let (lo, hi) = if self.from.x <= self.to.x {
                (self.from.x - from_ext, self.to.x + to_ext)
            } else {
                (self.to.x - to_ext, self.from.x + from_ext)
            };
            let y = self.from.y - half;
            Rect::new(Point::new(lo, y), Point::new(hi, y + self.width))
        } else {
            let (lo, hi) = if self.from.y <= self.to.y {
                (self.from.y - from_ext, self.to.y + to_ext)
            } else {
                (self.to.y - to_ext, self.from.y + from_ext)
            };
            let x = self.from.x - half;
            Rect::new(Point::new(x, lo), Point::new(x + self.width, hi))
        }
    }
}

/// A net from the DEF `SPECIALNETS` section, typically power or ground.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SpecialNet {
    /// Net name.
    pub name: String,
    /// What the net carries.
    pub signal_use: SignalUse,
    /// `( component pin )` connections as written, where `*` matches every component.
    pub connections: Vec<(String, String)>,
    /// Wire segments.
    pub wires: Vec<SpecialWire>,
    /// Rectangles given directly with `+ RECT`.
    pub rects: Vec<SpecialRect>,
    /// Via instances.
    pub vias: Vec<(ViaId, Point<Dbu>)>,
}

/// A rectangle of special wiring given directly (DEF `+ RECT`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SpecialRect {
    /// Layer of the rectangle.
    pub layer: LayerRef,
    /// The rectangle.
    pub rect: Rect<Dbu>,
    /// The DEF `SHAPE` keyword in force for it, such as `STRIPE`.
    pub shape: Option<String>,
}

/// The complete design database.
#[derive(Clone, Debug, PartialEq)]
pub struct NetlistDB {
    /// Design name.
    pub name: String,
    /// Technology and cell masters.
    pub library: Library,
    /// Chip boundary.
    pub die_area: Rect<Dbu>,
    /// Placement rows.
    pub rows: Vec<Row>,
    /// Routing tracks.
    pub tracks: Vec<TrackDef>,
    /// The gcells routing works in.
    pub gcell_grid: Vec<GCellGridDef>,
    /// Placed instances.
    pub cells: Vec<Cell>,
    /// Signal nets.
    pub nets: Vec<Net>,
    /// Net connections.
    pub pins: Vec<Pin>,
    /// Top-level I/O pins.
    pub io_pins: Vec<IoPin>,
    /// Power and other special nets.
    pub special_nets: Vec<SpecialNet>,
    /// Areas no wire may use, per layer.
    pub routing_blockages: Vec<Shape>,
    /// Areas no cell may be placed in.
    pub placement_blockages: Vec<Rect<Dbu>>,
    /// Placement regions.
    pub regions: Vec<Region>,
    /// Non-default routing rules.
    pub non_default_rules: Vec<NonDefaultRule>,
    cell_by_name: HashMap<String, CellId>,
    net_by_name: HashMap<String, NetId>,
    io_pin_by_name: HashMap<String, IoPinId>,
}

impl NetlistDB {
    /// Creates an empty design over a library.
    pub fn new(name: String, library: Library) -> Self {
        Self {
            name,
            library,
            die_area: Rect::default(),
            rows: Vec::new(),
            tracks: Vec::new(),
            gcell_grid: Vec::new(),
            cells: Vec::new(),
            nets: Vec::new(),
            pins: Vec::new(),
            io_pins: Vec::new(),
            special_nets: Vec::new(),
            routing_blockages: Vec::new(),
            placement_blockages: Vec::new(),
            regions: Vec::new(),
            non_default_rules: Vec::new(),
            cell_by_name: HashMap::new(),
            net_by_name: HashMap::new(),
            io_pin_by_name: HashMap::new(),
        }
    }

    /// Database units per micron.
    pub const fn dbu(&self) -> DbuPerMicron {
        self.library.dbu_per_micron
    }

    /// Number of instances.
    pub const fn num_cells(&self) -> usize {
        self.cells.len()
    }

    /// Number of signal nets.
    pub const fn num_nets(&self) -> usize {
        self.nets.len()
    }

    /// Adds an instance.
    pub fn add_cell(
        &mut self,
        name: String,
        macro_id: MacroId,
        location: Point<Dbu>,
        orient: Orientation,
        status: PlacementStatus,
    ) -> Result<CellId, DuplicateName> {
        let id = CellId::new(self.cells.len());
        if self.cell_by_name.contains_key(&name) {
            return Err(DuplicateName {
                kind: "component",
                name,
            });
        }
        let _ = self.cell_by_name.insert(name.clone(), id);
        self.cells.push(Cell {
            name,
            macro_id,
            location,
            orient,
            status,
            region: None,
            pins: Vec::new(),
        });
        Ok(id)
    }

    /// Returns the net with this name, creating it if needed.
    pub fn net_or_insert(&mut self, name: &str) -> NetId {
        if let Some(&id) = self.net_by_name.get(name) {
            return id;
        }
        let id = NetId::new(self.nets.len());
        let _ = self.net_by_name.insert(name.to_string(), id);
        self.nets.push(Net {
            name: name.to_string(),
            signal_use: SignalUse::Signal,
            pins: Vec::new(),
            weight: 1.0,
            non_default_rule: None,
            routing: Vec::new(),
        });
        id
    }

    /// Connects a pin of an instance to a net.
    pub fn connect_cell_pin(&mut self, cell: CellId, pin: MacroPinId, net: NetId) -> PinId {
        let id = PinId::new(self.pins.len());
        self.pins.push(Pin {
            owner: PinOwner::Cell { cell, pin },
            net,
        });
        self.cells[cell.index()].pins.push(id);
        self.nets[net.index()].pins.push(id);
        id
    }

    /// Adds an I/O pin and connects it to its net.
    #[allow(clippy::too_many_arguments)]
    pub fn add_io_pin(
        &mut self,
        name: String,
        net: NetId,
        direction: PinDirection,
        signal_use: SignalUse,
        shapes: Vec<Shape>,
        location: Point<Dbu>,
        orient: Orientation,
        status: PlacementStatus,
    ) -> Result<IoPinId, DuplicateName> {
        let id = IoPinId::new(self.io_pins.len());
        if self.io_pin_by_name.contains_key(&name) {
            return Err(DuplicateName { kind: "pin", name });
        }
        let _ = self.io_pin_by_name.insert(name.clone(), id);
        let pin = PinId::new(self.pins.len());
        self.pins.push(Pin {
            owner: PinOwner::Io(id),
            net,
        });
        self.nets[net.index()].pins.push(pin);
        self.io_pins.push(IoPin {
            name,
            net,
            direction,
            signal_use,
            shapes,
            location,
            orient,
            status,
            pin,
        });
        Ok(id)
    }

    /// Looks up an instance by name.
    pub fn cell_id(&self, name: &str) -> Option<CellId> {
        self.cell_by_name.get(name).copied()
    }

    /// Looks up a net by name.
    pub fn net_id(&self, name: &str) -> Option<NetId> {
        self.net_by_name.get(name).copied()
    }

    /// Looks up an I/O pin by name.
    pub fn io_pin_id(&self, name: &str) -> Option<IoPinId> {
        self.io_pin_by_name.get(name).copied()
    }

    /// The master of an instance.
    pub fn cell_macro(&self, cell: CellId) -> &Macro {
        self.library.cell_macro(self.cells[cell.index()].macro_id)
    }

    /// Width and height of an instance as placed.
    pub fn cell_size(&self, cell: CellId) -> Point<Dbu> {
        let c = &self.cells[cell.index()];
        c.orient
            .oriented_size(self.library.cell_macro(c.macro_id).size)
    }

    /// The area an instance covers.
    pub fn cell_bbox(&self, cell: CellId) -> Rect<Dbu> {
        let c = &self.cells[cell.index()];
        Rect::new(c.location, c.location + self.cell_size(cell))
    }

    /// The design location of a connection's reference point.
    pub fn pin_position(&self, pin: PinId) -> Point<Dbu> {
        match self.pins[pin.index()].owner {
            PinOwner::Cell { cell, pin } => {
                let c = &self.cells[cell.index()];
                let m = self.library.cell_macro(c.macro_id);
                let local = m
                    .pin(pin)
                    .reference_point()
                    .unwrap_or_else(|| Rect::new(Point::default(), m.size).center());
                c.orient.place_point(local, m.size, c.location)
            }
            PinOwner::Io(io) => {
                let p = &self.io_pins[io.index()];
                p.placed_shapes()
                    .map(|s| s.rect)
                    .reduce(|a, b| a.union(&b))
                    .map_or(p.location, |r| r.center())
            }
        }
    }

    /// A connection's geometry in design coordinates.
    pub fn pin_shapes(&self, pin: PinId) -> Vec<Shape> {
        match self.pins[pin.index()].owner {
            PinOwner::Cell { cell, pin } => {
                let c = &self.cells[cell.index()];
                let m = self.library.cell_macro(c.macro_id);
                m.pin(pin)
                    .shapes
                    .iter()
                    .map(|s| Shape {
                        layer: s.layer,
                        rect: c.orient.place_rect(&s.rect, m.size, c.location),
                    })
                    .collect()
            }
            PinOwner::Io(io) => self.io_pins[io.index()].placed_shapes().collect(),
        }
    }

    /// A readable name for a connection, such as `u1/A` or `PIN clk`.
    pub fn pin_name(&self, pin: PinId) -> String {
        match self.pins[pin.index()].owner {
            PinOwner::Cell { cell, pin } => {
                let c = &self.cells[cell.index()];
                format!(
                    "{}/{}",
                    c.name,
                    self.library.cell_macro(c.macro_id).pin(pin).name
                )
            }
            PinOwner::Io(io) => format!("PIN {}", self.io_pins[io.index()].name),
        }
    }
}
