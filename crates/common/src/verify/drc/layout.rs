//! The shapes a design rule check sees, and who owns them.
//!
//! Owners follow `OpenROAD`: a net owns its wiring, its pins and the special
//! wiring of the special net of its name; an instance owns its
//! obstructions; a blockage owns itself; a pin on no net is its own owner,
//! except supply pins, which share one owner per supply.

use crate::db::design::{NetlistDB, PinOwner, PlacementStatus};
use crate::db::indices::{BlockageId, CellId, MacroPinId, NetId, NonDefaultRuleId, SpecialNetId};
use crate::db::library::{LayerRef, Shape, SignalUse};
use crate::geom::dbu::Dbu;
use crate::geom::rect::Rect;
use crate::verify::shapes::{default_width_shapes, route_elem_shapes, special_net_metal};
use std::collections::HashMap;

/// What a group of shapes belongs to. Shapes of one owner never short or
/// space against each other as different nets do.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum Owner {
    /// A net, with the special wiring of the special net of its name.
    Net(NetId),
    /// A special net with no regular net of its name.
    SpecialNet(SpecialNetId),
    /// The obstructions of an instance.
    Obstruction(CellId),
    /// A routing blockage.
    Blockage(BlockageId),
    /// An instance pin on no net.
    UnconnectedPin(CellId, MacroPinId),
    /// Every power pin on no net.
    FloatingPower,
    /// Every ground pin on no net.
    FloatingGround,
}

impl Owner {
    /// Whether the owner is an obstruction or blockage rather than metal
    /// that carries a signal.
    pub const fn is_blockage(self) -> bool {
        matches!(self, Self::Obstruction(_) | Self::Blockage(_))
    }

    /// The owner's name in `db`, for reports.
    pub fn name(self, db: &NetlistDB) -> String {
        match self {
            Self::Net(n) => db.nets[n.index()].name.clone(),
            Self::SpecialNet(s) => db.special_nets[s.index()].name.clone(),
            Self::Obstruction(c) => format!("{} (obstruction)", db.cells[c.index()].name),
            Self::Blockage(b) => format!("blockage {}", b.index()),
            Self::UnconnectedPin(c, p) => {
                format!(
                    "{}/{}",
                    db.cells[c.index()].name,
                    db.cell_macro(c).pin(p).name
                )
            }
            Self::FloatingPower => "floating power".to_string(),
            Self::FloatingGround => "floating ground".to_string(),
        }
    }
}

/// An owner's position in [`Layout::owners`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct OwnerIndex(pub u32);

impl OwnerIndex {
    /// The position in the owner table.
    pub const fn index(self) -> usize {
        self.0 as usize
    }
}

/// Which step of the flow put a shape where it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Origin {
    /// Routed wiring, which routing can move.
    Routing,
    /// A pin or obstruction of a cell, or an I/O pin, that placement can move.
    Placement,
    /// Metal nothing in the flow moves: a fixed cell or I/O pin, special
    /// wiring or a blockage.
    Input,
}

impl Origin {
    /// Whether the shape is fixed as far as routing is concerned: part of a
    /// cell, an I/O pin, special wiring or a blockage rather than routed
    /// wiring.
    pub const fn is_fixed(self) -> bool {
        !matches!(self, Self::Routing)
    }

    /// The origin of a cell or I/O pin with placement status `status`.
    pub const fn of_placed(status: PlacementStatus) -> Self {
        if status.is_movable() {
            Self::Placement
        } else {
            Self::Input
        }
    }
}

/// One rectangle of the layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LayoutShape {
    /// Whose it is.
    pub owner: OwnerIndex,
    /// The rectangle.
    pub rect: Rect<Dbu>,
    /// Where it comes from.
    pub origin: Origin,
}

impl LayoutShape {
    /// Whether it is fixed rather than routed wiring.
    pub const fn is_fixed(&self) -> bool {
        self.origin.is_fixed()
    }
}

/// Every shape on every layer, with its owner.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Layout {
    owners: Vec<Owner>,
    supply: Vec<bool>,
    rule_of: Vec<Option<NonDefaultRuleId>>,
    index_of: HashMap<Owner, OwnerIndex>,
    metal: Vec<Vec<LayoutShape>>,
    cut: Vec<Vec<LayoutShape>>,
}

impl Layout {
    /// An empty layout with `metal` routing layers and `cut` cut layers.
    pub fn new(metal: usize, cut: usize) -> Self {
        Self {
            metal: vec![Vec::new(); metal],
            cut: vec![Vec::new(); cut],
            ..Self::default()
        }
    }

    /// An empty layout with the owners of `other`, so that markers found in
    /// either name the same owners.
    pub fn with_owners_of(other: &Self) -> Self {
        Self {
            owners: other.owners.clone(),
            supply: other.supply.clone(),
            rule_of: other.rule_of.clone(),
            index_of: other.index_of.clone(),
            ..Self::new(other.metal.len(), other.cut.len())
        }
    }

    /// The index of `owner`, adding it if new; `supply` says whether it
    /// carries power or ground.
    pub fn owner(&mut self, owner: Owner, supply: bool) -> OwnerIndex {
        if let Some(&index) = self.index_of.get(&owner) {
            return index;
        }
        let index = OwnerIndex(u32::try_from(self.owners.len()).unwrap_or(u32::MAX));
        self.owners.push(owner);
        self.supply.push(supply);
        self.rule_of.push(None);
        let _ = self.index_of.insert(owner, index);
        index
    }

    /// Puts `owner`'s shapes under nondefault rule `rule`.
    pub fn set_non_default_rule(&mut self, owner: OwnerIndex, rule: NonDefaultRuleId) {
        if let Some(slot) = self.rule_of.get_mut(owner.index()) {
            *slot = Some(rule);
        }
    }

    /// The nondefault rule `owner`'s shapes follow, if any.
    pub fn non_default_rule(&self, owner: OwnerIndex) -> Option<NonDefaultRuleId> {
        self.rule_of.get(owner.index()).copied().flatten()
    }

    /// Adds a rectangle; one on a layer the layout does not have, or
    /// without area, is ignored.
    pub fn add(&mut self, layer: LayerRef, owner: OwnerIndex, rect: Rect<Dbu>, origin: Origin) {
        if rect.width() < Dbu::ZERO || rect.height() < Dbu::ZERO {
            return;
        }
        self.add_shape(
            layer,
            &LayoutShape {
                owner,
                rect,
                origin,
            },
        );
    }

    /// Adds a shape as it is, on a layer the layout has.
    pub fn add_shape(&mut self, layer: LayerRef, shape: &LayoutShape) {
        let shapes = match layer {
            LayerRef::Routing(l) => self.metal.get_mut(l.index()),
            LayerRef::Cut(c) => self.cut.get_mut(c.index()),
        };
        if let Some(shapes) = shapes {
            shapes.push(*shape);
        }
    }

    /// Every owner, by index.
    pub fn owners(&self) -> &[Owner] {
        &self.owners
    }

    /// Whether `owner` carries power or ground.
    pub fn is_supply(&self, owner: OwnerIndex) -> bool {
        self.supply.get(owner.index()).copied().unwrap_or(false)
    }

    /// The shapes on a routing layer.
    pub fn metal(&self, layer: usize) -> &[LayoutShape] {
        self.metal.get(layer).map_or(&[], Vec::as_slice)
    }

    /// The shapes on a cut layer.
    pub fn cut(&self, layer: usize) -> &[LayoutShape] {
        self.cut.get(layer).map_or(&[], Vec::as_slice)
    }

    /// Number of routing layers.
    pub const fn metal_layers(&self) -> usize {
        self.metal.len()
    }

    /// Number of cut layers.
    pub const fn cut_layers(&self) -> usize {
        self.cut.len()
    }

    /// The layout of `db`: placed cells' pins and obstructions, placed I/O
    /// pins, special wiring and routing blockages as fixed shapes, and the
    /// wiring of regular nets as routed shapes.
    pub fn from_db(db: &NetlistDB) -> Self {
        let mut layout = Self::new(db.library.routing_layers.len(), db.library.cut_layers.len());
        let special = SpecialOwners::new(db);
        for (i, cell) in db.cells.iter().enumerate() {
            if cell.status == PlacementStatus::Unplaced {
                continue;
            }
            layout.add_cell(db, &special, CellId::new(i));
        }
        for io in &db.io_pins {
            if io.status == PlacementStatus::Unplaced {
                continue;
            }
            let owner = layout.owner(Owner::Net(io.net), net_is_supply(db, &special, io.net));
            let origin = Origin::of_placed(io.status);
            for s in io.placed_shapes() {
                layout.add(s.layer, owner, s.rect, origin);
            }
        }
        for (i, net) in db.special_nets.iter().enumerate() {
            let (owner, supply) = special.owner(db, SpecialNetId::new(i));
            let owner = layout.owner(owner, supply);
            for s in special_net_metal(net) {
                layout.add(s.layer, owner, s.rect, Origin::Input);
            }
            for &(via, at) in &net.vias {
                for s in &db.library.via(via).shapes {
                    layout.add(s.layer, owner, s.rect.translate(at), Origin::Input);
                }
            }
        }
        for (i, s) in db.routing_blockages.iter().enumerate() {
            let owner = layout.owner(Owner::Blockage(BlockageId::new(i)), false);
            layout.add(s.layer, owner, s.rect, Origin::Input);
        }
        for (i, net) in db.nets.iter().enumerate() {
            let id = NetId::new(i);
            if net.routing.is_empty() && net.non_default_rule.is_none() {
                continue;
            }
            let owner = layout.owner(Owner::Net(id), net_is_supply(db, &special, id));
            if let Some(rule) = net.non_default_rule {
                layout.set_non_default_rule(owner, rule);
            }
            for elem in &net.routing {
                for s in route_elem_shapes(db, id, elem) {
                    layout.add(s.layer, owner, s.rect, Origin::Routing);
                }
            }
        }
        layout
    }

    /// The nets of `db` under a nondefault rule as `OpenROAD` reads them:
    /// their wiring at the layers' default widths, with their pins, under
    /// the owners of `layout`.
    pub fn default_width_view_of_rule_nets(db: &NetlistDB, layout: &Self) -> Self {
        let mut view = Self::with_owners_of(layout);
        for (i, net) in db.nets.iter().enumerate() {
            if net.non_default_rule.is_none() || net.routing.is_empty() {
                continue;
            }
            let id = NetId::new(i);
            let Some(&owner) = layout.index_of.get(&Owner::Net(id)) else {
                continue;
            };
            for &pin in &net.pins {
                let origin = match db.pins[pin.index()].owner {
                    PinOwner::Cell { cell, .. } => Origin::of_placed(db.cells[cell.index()].status),
                    PinOwner::Io(io) => Origin::of_placed(db.io_pins[io.index()].status),
                };
                for s in db.pin_shapes(pin) {
                    view.add(s.layer, owner, s.rect, origin);
                }
            }
            for elem in &net.routing {
                for s in default_width_shapes(db, elem) {
                    view.add(s.layer, owner, s.rect, Origin::Routing);
                }
            }
        }
        view
    }

    fn add_cell(&mut self, db: &NetlistDB, special: &SpecialOwners, cell: CellId) {
        let c = &db.cells[cell.index()];
        let m = db.cell_macro(cell);
        let origin = Origin::of_placed(c.status);
        let place = |s: &Shape| c.orient.place_rect(&s.rect, m.size, c.location);
        let mut net_of_pin: Vec<Option<NetId>> = vec![None; m.pins.len()];
        for &pin in &c.pins {
            if let PinOwner::Cell { pin: mp, .. } = db.pins[pin.index()].owner
                && let Some(slot) = net_of_pin.get_mut(mp.index())
            {
                *slot = Some(db.pins[pin.index()].net);
            }
        }
        for (k, mp) in m.pins.iter().enumerate() {
            let pin = MacroPinId::new(k);
            let (owner, supply) = match net_of_pin[k] {
                Some(net) => (Owner::Net(net), net_is_supply(db, special, net)),
                None => match special.connection(&c.name, &mp.name) {
                    Some(s) => special.owner(db, s),
                    None => match mp.signal_use {
                        SignalUse::Power => (Owner::FloatingPower, true),
                        SignalUse::Ground => (Owner::FloatingGround, true),
                        _ => (Owner::UnconnectedPin(cell, pin), false),
                    },
                },
            };
            let owner = self.owner(owner, supply);
            for s in &mp.shapes {
                self.add(s.layer, owner, place(s), origin);
            }
        }
        if !m.obstructions.is_empty() {
            let owner = self.owner(Owner::Obstruction(cell), false);
            for s in &m.obstructions {
                self.add(s.layer, owner, place(s), origin);
            }
        }
    }
}

/// The owners of special nets and the pins their connections name.
struct SpecialOwners {
    regular_net: Vec<Option<NetId>>,
    every_instance: HashMap<String, SpecialNetId>,
    named: HashMap<(String, String), SpecialNetId>,
    special_of_net: HashMap<NetId, SpecialNetId>,
}

impl SpecialOwners {
    fn new(db: &NetlistDB) -> Self {
        let net_by_name: HashMap<&str, NetId> = db
            .nets
            .iter()
            .enumerate()
            .map(|(i, n)| (n.name.as_str(), NetId::new(i)))
            .collect();
        let mut owners = Self {
            regular_net: Vec::with_capacity(db.special_nets.len()),
            every_instance: HashMap::new(),
            named: HashMap::new(),
            special_of_net: HashMap::new(),
        };
        for (i, net) in db.special_nets.iter().enumerate() {
            let id = SpecialNetId::new(i);
            let regular = net_by_name.get(net.name.as_str()).copied();
            owners.regular_net.push(regular);
            if let Some(n) = regular {
                let _ = owners.special_of_net.insert(n, id);
            }
            for (component, pin) in &net.connections {
                if component == "*" {
                    let _ = owners.every_instance.insert(pin.clone(), id);
                } else {
                    let _ = owners.named.insert((component.clone(), pin.clone()), id);
                }
            }
        }
        owners
    }

    /// The special net the connections of special nets put the pin `pin`
    /// of instance `cell` on.
    fn connection(&self, cell: &str, pin: &str) -> Option<SpecialNetId> {
        self.named
            .get(&(cell.to_string(), pin.to_string()))
            .or_else(|| self.every_instance.get(pin))
            .copied()
    }

    /// The owner of special net `s` and whether it carries a supply.
    fn owner(&self, db: &NetlistDB, s: SpecialNetId) -> (Owner, bool) {
        let supply = db.special_nets[s.index()].signal_use.is_supply();
        match self.regular_net.get(s.index()).copied().flatten() {
            Some(net) => (
                Owner::Net(net),
                supply || db.nets[net.index()].signal_use.is_supply(),
            ),
            None => (Owner::SpecialNet(s), supply),
        }
    }
}

fn net_is_supply(db: &NetlistDB, special: &SpecialOwners, net: NetId) -> bool {
    db.nets[net.index()].signal_use.is_supply()
        || special
            .special_of_net
            .get(&net)
            .is_some_and(|s| db.special_nets[s.index()].signal_use.is_supply())
}
