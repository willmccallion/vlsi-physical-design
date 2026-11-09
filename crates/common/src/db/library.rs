//! Technology and cell library: layers, sites, cell masters and vias.

use crate::db::indices::{
    CutLayerId, IdVec, LayerId, MacroId, MacroPinId, SiteId, TableFull, ViaId, ViaRuleId,
};
use crate::db::rules::{
    CellEdge, CutLayerRules, CutSpacingCondition, Lef58CutSpacingCondition, LibraryRules,
    RoutingLayerRules, SpacingTable, ViaRule,
};
use crate::geom::dbu::{Dbu, DbuPerMicron};
use crate::geom::orient::Orientation;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use std::collections::HashMap;

/// Preferred routing direction of a layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Direction {
    /// Wires run along X.
    Horizontal,
    /// Wires run along Y.
    Vertical,
}

/// A metal layer that carries wires.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutingLayer {
    /// LEF layer name.
    pub name: String,
    /// Preferred wire direction.
    pub direction: Direction,
    /// Spacing between vertical tracks.
    pub pitch_x: Dbu,
    /// Spacing between horizontal tracks.
    pub pitch_y: Dbu,
    /// Offset of the first vertical track from the origin.
    pub offset_x: Option<Dbu>,
    /// Offset of the first horizontal track from the origin.
    pub offset_y: Option<Dbu>,
    /// Default wire width.
    pub width: Dbu,
    /// Minimum spacing between shapes of different nets at zero parallel run.
    pub min_spacing: Dbu,
    /// Spacing that grows with width and parallel run, if the layer has one.
    pub spacing_table: Option<SpacingTable>,
    /// Further rules on the layer's shapes.
    pub rules: RoutingLayerRules,
    /// Its resistance and capacitance, as the LEF gives them.
    pub electrical: LayerElectrical,
}

/// A routing layer's resistance and capacitance from the LEF, in its units.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LayerElectrical {
    /// Ohms per square (`RESISTANCE RPERSQ`).
    pub resistance_per_square: Option<f64>,
    /// Picofarads per square micron (`CAPACITANCE CPERSQDIST`).
    pub capacitance_per_area: Option<f64>,
    /// Picofarads per micron of edge (`EDGECAPACITANCE`).
    pub edge_capacitance: Option<f64>,
}

impl RoutingLayer {
    /// Pitch of the tracks wires run on in the preferred direction.
    pub const fn track_pitch(&self) -> Dbu {
        match self.direction {
            Direction::Horizontal => self.pitch_y,
            Direction::Vertical => self.pitch_x,
        }
    }
}

/// A via cut layer between two routing layers.
#[derive(Clone, Debug, PartialEq)]
pub struct CutLayer {
    /// LEF layer name.
    pub name: String,
    /// Cut width, if given.
    pub width: Option<Dbu>,
    /// Edge-to-edge spacing every pair of cuts of different nets needs, if the
    /// layer has an unconditional rule; a center-to-center rule counts as its
    /// value less the cut width, which is exact along an axis and stricter
    /// diagonally. A parallel-overlap property rule counts as unconditional,
    /// since only cuts of one net can share the metal that exempts a pair.
    pub spacing: Option<Dbu>,
    /// All of the layer's cut spacing and enclosure rules.
    pub rules: CutLayerRules,
    /// Resistance of one cut in ohms (`RESISTANCE`), if given.
    pub resistance: Option<f64>,
}

impl CutLayer {
    /// Derives [`CutLayer::spacing`] from the layer's unconditional
    /// different-net rules.
    pub(crate) fn unconditional_spacing(width: Option<Dbu>, rules: &CutLayerRules) -> Option<Dbu> {
        let edge_to_edge = |spacing: Dbu, center_to_center: bool| {
            if center_to_center {
                width.map(|w| spacing - w)
            } else {
                Some(spacing)
            }
        };
        let statements = rules
            .spacings
            .iter()
            .filter(|r| !r.same_net && r.condition == CutSpacingCondition::Always)
            .filter_map(|r| edge_to_edge(r.spacing, r.center_to_center));
        let properties = rules
            .lef58_spacings
            .iter()
            .filter(|r| !r.same_net && r.condition == Lef58CutSpacingCondition::ParallelOverlap)
            .filter_map(|r| edge_to_edge(r.spacing, r.center_to_center));
        statements.chain(properties).max()
    }
}

/// A layer a shape can be drawn on.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum LayerRef {
    /// A metal layer.
    Routing(LayerId),
    /// A via cut layer.
    Cut(CutLayerId),
}

/// A rectangle on a layer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Shape {
    /// Layer the rectangle is on.
    pub layer: LayerRef,
    /// Where it lies on its layer, relative to whatever owns it.
    pub rect: Rect<Dbu>,
}

/// Allowed mirrorings and rotation of a site or macro.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Symmetry {
    /// May be mirrored about the X axis.
    pub x: bool,
    /// May be mirrored about the Y axis.
    pub y: bool,
    /// May be rotated by 90 degrees.
    pub r90: bool,
}

impl Symmetry {
    /// Whether a master of this symmetry may take `orient`, as `OpenDP`'s
    /// `checkMasterSym`.
    #[must_use]
    pub const fn allows(self, orient: Orientation) -> bool {
        match orient {
            Orientation::N => true,
            Orientation::FS => self.x,
            Orientation::FN => self.y,
            Orientation::S => self.x && self.y,
            Orientation::W | Orientation::E => self.r90,
            Orientation::FW | Orientation::FE => self.r90 && self.x && self.y,
        }
    }
}

/// What a placement site is for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SiteClass {
    /// Rows of standard cells.
    #[default]
    Core,
    /// I/O pad rings.
    Pad,
}

/// A placement site.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Site {
    /// LEF site name.
    pub name: String,
    /// What the site is for.
    pub class: SiteClass,
    /// Width and height.
    pub size: Point<Dbu>,
    /// Allowed symmetries.
    pub symmetry: Symmetry,
}

/// The kind of core cell, from the LEF `CLASS CORE` subclass.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoreKind {
    /// An ordinary logic cell.
    Regular,
    /// A feedthrough cell.
    Feedthru,
    /// A tie-high cell.
    TieHigh,
    /// A tie-low cell.
    TieLow,
    /// A filler cell.
    Spacer,
    /// An antenna diode cell.
    AntennaCell,
    /// A well-tap cell.
    WellTap,
}

/// The LEF class of a cell master.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MacroClass {
    /// A standard cell.
    Core(CoreKind),
    /// A hard block.
    Block,
    /// An I/O pad.
    Pad,
    /// An end-cap cell at a row end.
    Endcap,
    /// A cover cell, fixed at the chip level.
    Cover,
    /// A ring around the core.
    Ring,
}

/// Signal direction of a pin.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PinDirection {
    /// Driven from outside.
    Input,
    /// Drives the net.
    Output,
    /// Either way.
    #[default]
    Inout,
    /// Passes through the cell.
    Feedthru,
}

impl PinDirection {
    /// The LEF/DEF keyword.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Input => "INPUT",
            Self::Output => "OUTPUT",
            Self::Inout => "INOUT",
            Self::Feedthru => "FEEDTHRU",
        }
    }
}

/// What a pin or net carries.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SignalUse {
    /// An ordinary signal.
    #[default]
    Signal,
    /// A power supply.
    Power,
    /// A ground supply.
    Ground,
    /// A clock, as LEF's `USE CLOCK`.
    Clock,
    /// An analog signal.
    Analog,
    /// A scan chain signal.
    Scan,
    /// A reset signal.
    Reset,
    /// A tie-off.
    Tieoff,
}

impl SignalUse {
    /// The LEF/DEF keyword.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Signal => "SIGNAL",
            Self::Power => "POWER",
            Self::Ground => "GROUND",
            Self::Clock => "CLOCK",
            Self::Analog => "ANALOG",
            Self::Scan => "SCAN",
            Self::Reset => "RESET",
            Self::Tieoff => "TIEOFF",
        }
    }

    /// Whether this is a supply that the signal router leaves alone.
    pub const fn is_supply(self) -> bool {
        matches!(self, Self::Power | Self::Ground)
    }
}

/// A pin of a cell master.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MacroPin {
    /// Pin name.
    pub name: String,
    /// Signal direction.
    pub direction: PinDirection,
    /// What the pin carries.
    pub signal_use: SignalUse,
    /// Port geometry in macro coordinates.
    pub shapes: Vec<Shape>,
}

impl MacroPin {
    /// The lowest routing layer the pin has a shape on.
    pub(crate) fn reference_layer(&self) -> Option<LayerId> {
        self.shapes
            .iter()
            .filter_map(|s| match s.layer {
                LayerRef::Routing(layer) => Some(layer),
                LayerRef::Cut(_) => None,
            })
            .min()
    }

    /// A point inside the pin used as its location for wirelength and routing:
    /// the center of the largest shape on the lowest routing layer it uses.
    pub fn reference_point(&self) -> Option<Point<Dbu>> {
        let lowest = self.reference_layer()?;
        self.shapes
            .iter()
            .filter(|s| s.layer == LayerRef::Routing(lowest))
            .max_by_key(|s| s.rect.area())
            .map(|s| s.rect.center())
    }
}

/// A cell master.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Macro {
    /// Master name.
    pub name: String,
    /// LEF class.
    pub class: MacroClass,
    /// Width and height.
    pub size: Point<Dbu>,
    /// Allowed symmetries.
    pub symmetry: Symmetry,
    /// Site the cell is built on, for row-based cells.
    pub site: Option<SiteId>,
    /// Its pins, indexed by [`MacroPinId`].
    pub pins: IdVec<MacroPinId, MacroPin>,
    /// Routing obstructions in macro coordinates.
    pub obstructions: Vec<Shape>,
    /// Edge types for cell edge spacing rules.
    pub edges: Vec<CellEdge>,
    pin_by_name: HashMap<String, MacroPinId>,
}

impl Macro {
    /// Creates a macro with no pins or obstructions.
    pub fn new(
        name: String,
        class: MacroClass,
        size: Point<Dbu>,
        symmetry: Symmetry,
        site: Option<SiteId>,
    ) -> Self {
        Self {
            name,
            class,
            size,
            symmetry,
            site,
            pins: IdVec::new(),
            obstructions: Vec::new(),
            edges: Vec::new(),
            pin_by_name: HashMap::new(),
        }
    }

    /// Adds a pin, failing if one with that name exists.
    pub fn add_pin(&mut self, pin: MacroPin) -> Result<MacroPinId, AddError> {
        let id = self.pins.try_next_id("pin")?;
        insert_unique(&mut self.pin_by_name, "pin", &pin.name, id)?;
        let _ = self.pins.push(pin);
        Ok(id)
    }

    /// Looks up a pin by name.
    pub fn pin_id(&self, name: &str) -> Option<MacroPinId> {
        self.pin_by_name.get(name).copied()
    }

    /// The pin with this id.
    pub fn pin(&self, id: MacroPinId) -> &MacroPin {
        &self.pins[id]
    }

    /// Whether the macro is a multi-row block rather than a standard cell.
    pub const fn is_block(&self) -> bool {
        matches!(
            self.class,
            MacroClass::Block | MacroClass::Pad | MacroClass::Ring | MacroClass::Cover
        )
    }
}

/// Where a via was defined.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ViaSource {
    /// In LEF, marked `DEFAULT` for its cut layer.
    LefDefault,
    /// In LEF, not marked default.
    Lef,
    /// In the DEF `VIAS` section.
    Def,
}

/// A fixed via: a cut and the metal enclosing it on the layers above and below.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ViaDef {
    /// Via name.
    pub name: String,
    /// The layer its cuts are on.
    pub cut: CutLayerId,
    /// Geometry on the bottom metal, cut and top metal, centered on the origin.
    pub shapes: Vec<Shape>,
    /// Where it was defined.
    pub source: ViaSource,
}

impl ViaDef {
    /// Whether LEF marks it as a default via for its cut layer.
    #[must_use]
    pub const fn is_default(&self) -> bool {
        matches!(self.source, ViaSource::LefDefault)
    }

    /// Whether the DEF `VIAS` section defined it.
    #[must_use]
    pub const fn is_from_def(&self) -> bool {
        matches!(self.source, ViaSource::Def)
    }

    /// The routing layer below the cut.
    pub const fn bottom(&self) -> LayerId {
        self.cut.below()
    }

    /// The routing layer above the cut, if its id can be counted.
    pub const fn top(&self) -> Option<LayerId> {
        self.cut.above()
    }
}

/// All technology and cell master data of a design.
#[derive(Clone, Debug, PartialEq)]
pub struct Library {
    /// Database units per micron every value here is expressed in.
    pub dbu_per_micron: DbuPerMicron,
    /// Routing layers from the bottom up.
    pub routing_layers: IdVec<LayerId, RoutingLayer>,
    /// Cut layers; cut `n` lies between routing layers `n` and `n + 1`.
    pub cut_layers: IdVec<CutLayerId, CutLayer>,
    /// Placement sites.
    pub sites: IdVec<SiteId, Site>,
    /// Cell masters.
    pub macros: IdVec<MacroId, Macro>,
    /// Via definitions.
    pub vias: IdVec<ViaId, ViaDef>,
    /// Rules for generating via arrays.
    pub via_rules: IdVec<ViaRuleId, ViaRule>,
    /// Technology-wide rules.
    pub rules: LibraryRules,
    layer_by_name: HashMap<String, LayerRef>,
    site_by_name: HashMap<String, SiteId>,
    macro_by_name: HashMap<String, MacroId>,
    via_by_name: HashMap<String, ViaId>,
}

/// Why an entry was not added to a library table.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum AddError {
    /// Its name is taken.
    #[error(transparent)]
    Duplicate(#[from] DuplicateName),
    /// Its table holds every index there is.
    #[error(transparent)]
    Full(#[from] TableFull),
}

/// A library entry whose name is already taken.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("duplicate {kind} '{name}'")]
pub struct DuplicateName {
    /// What kind of entry it was.
    pub kind: &'static str,
    /// The repeated name.
    pub name: String,
}

impl Library {
    /// Creates an empty library at the given resolution.
    pub fn new(dbu_per_micron: DbuPerMicron) -> Self {
        Self {
            dbu_per_micron,
            routing_layers: IdVec::new(),
            cut_layers: IdVec::new(),
            sites: IdVec::new(),
            macros: IdVec::new(),
            vias: IdVec::new(),
            via_rules: IdVec::new(),
            rules: LibraryRules::default(),
            layer_by_name: HashMap::new(),
            site_by_name: HashMap::new(),
            macro_by_name: HashMap::new(),
            via_by_name: HashMap::new(),
        }
    }

    /// Appends the next routing layer of the stack.
    pub fn add_routing_layer(&mut self, layer: RoutingLayer) -> Result<LayerId, AddError> {
        let id = self.routing_layers.try_next_id("routing layer")?;
        self.claim_layer_name(&layer.name, LayerRef::Routing(id))?;
        let _ = self.routing_layers.push(layer);
        Ok(id)
    }

    /// Appends the next cut layer of the stack.
    pub fn add_cut_layer(&mut self, layer: CutLayer) -> Result<CutLayerId, AddError> {
        let id = self.cut_layers.try_next_id("cut layer")?;
        self.claim_layer_name(&layer.name, LayerRef::Cut(id))?;
        let _ = self.cut_layers.push(layer);
        Ok(id)
    }

    fn claim_layer_name(&mut self, name: &str, layer: LayerRef) -> Result<(), DuplicateName> {
        if self.layer_by_name.contains_key(name) {
            return Err(DuplicateName {
                kind: "layer",
                name: name.to_string(),
            });
        }
        let _ = self.layer_by_name.insert(name.to_string(), layer);
        Ok(())
    }

    /// Adds a site.
    pub fn add_site(&mut self, site: Site) -> Result<SiteId, AddError> {
        let id = self.sites.try_next_id("site")?;
        insert_unique(&mut self.site_by_name, "site", &site.name, id)?;
        let _ = self.sites.push(site);
        Ok(id)
    }

    /// Adds a cell master.
    pub fn add_macro(&mut self, m: Macro) -> Result<MacroId, AddError> {
        let id = self.macros.try_next_id("macro")?;
        insert_unique(&mut self.macro_by_name, "macro", &m.name, id)?;
        let _ = self.macros.push(m);
        Ok(id)
    }

    /// Adds a via definition.
    pub fn add_via(&mut self, via: ViaDef) -> Result<ViaId, AddError> {
        let id = self.vias.try_next_id("via")?;
        insert_unique(&mut self.via_by_name, "via", &via.name, id)?;
        let _ = self.vias.push(via);
        Ok(id)
    }

    /// Looks up a routing or cut layer by name.
    pub fn layer_ref(&self, name: &str) -> Option<LayerRef> {
        self.layer_by_name.get(name).copied()
    }

    /// Looks up a cut layer by name.
    pub fn cut_layer_id(&self, name: &str) -> Option<CutLayerId> {
        match self.layer_ref(name)? {
            LayerRef::Cut(id) => Some(id),
            LayerRef::Routing(_) => None,
        }
    }

    /// Looks up a routing layer by name.
    pub fn routing_layer_id(&self, name: &str) -> Option<LayerId> {
        match self.layer_ref(name)? {
            LayerRef::Routing(id) => Some(id),
            LayerRef::Cut(_) => None,
        }
    }

    /// Looks up a site by name.
    pub fn site_id(&self, name: &str) -> Option<SiteId> {
        self.site_by_name.get(name).copied()
    }

    /// Looks up a cell master by name.
    pub fn macro_id(&self, name: &str) -> Option<MacroId> {
        self.macro_by_name.get(name).copied()
    }

    /// Looks up a via by name.
    pub fn via_id(&self, name: &str) -> Option<ViaId> {
        self.via_by_name.get(name).copied()
    }

    /// The routing layer with this id.
    pub fn layer(&self, id: LayerId) -> &RoutingLayer {
        &self.routing_layers[id]
    }

    /// The cell master with this id.
    pub fn cell_macro(&self, id: MacroId) -> &Macro {
        &self.macros[id]
    }

    /// The via with this id.
    pub fn via(&self, id: ViaId) -> &ViaDef {
        &self.vias[id]
    }

    /// The site with this id.
    pub fn site(&self, id: SiteId) -> &Site {
        &self.sites[id]
    }

    /// The name of a routing or cut layer.
    pub(crate) fn layer_name(&self, layer: LayerRef) -> &str {
        match layer {
            LayerRef::Routing(id) => &self.routing_layers[id].name,
            LayerRef::Cut(id) => &self.cut_layers[id].name,
        }
    }

    /// The via a router should use through `cut`: the first default via on
    /// that cut, or the first via on it if none is marked default.
    pub fn default_via(&self, cut: CutLayerId) -> Option<ViaId> {
        let on_cut = |v: &&ViaDef| v.cut == cut && !v.is_from_def();
        self.vias
            .iter()
            .position(|v| on_cut(&v) && v.is_default())
            .or_else(|| self.vias.iter().position(|v| on_cut(&v)))
            .map(ViaId::new)
    }
}

fn insert_unique<I: Copy>(
    map: &mut HashMap<String, I>,
    kind: &'static str,
    name: &str,
    id: I,
) -> Result<(), DuplicateName> {
    if map.contains_key(name) {
        return Err(DuplicateName {
            kind,
            name: name.to_string(),
        });
    }
    let _ = map.insert(name.to_string(), id);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn routing_layer(name: String) -> RoutingLayer {
        RoutingLayer {
            name,
            direction: Direction::Horizontal,
            pitch_x: Dbu(200),
            pitch_y: Dbu(200),
            offset_x: None,
            offset_y: None,
            width: Dbu(100),
            min_spacing: Dbu(100),
            spacing_table: None,
            rules: RoutingLayerRules::default(),
            electrical: LayerElectrical::default(),
        }
    }

    #[test]
    fn a_routing_layer_past_what_layer_ids_count_is_refused() {
        let mut library = Library::new(DbuPerMicron::new(1000).unwrap());
        for k in 0..256 {
            assert!(
                library
                    .add_routing_layer(routing_layer(format!("m{k}")))
                    .is_ok()
            );
        }

        let refused = library.add_routing_layer(routing_layer("m256".into()));

        assert!(refused.is_err());
        assert_eq!(library.routing_layers.len(), 256);
        assert_eq!(library.layer_ref("m256"), None);
    }
}
