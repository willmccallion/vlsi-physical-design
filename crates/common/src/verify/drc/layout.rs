//! The shapes a design rule check sees, and who owns them.
//!
//! Owners follow `OpenROAD`: a net owns its wiring, its pins and the special
//! wiring of the special net of its name; an instance owns its
//! obstructions; a blockage owns itself; a pin on no net is its own owner,
//! except supply pins, which share one owner per supply.

use crate::db::design::{NetlistDB, PinOwner, PlacementStatus};
use crate::db::indices::{BlockageId, CellId, MacroPinId, NetId, SpecialNetId};
use crate::db::library::{LayerRef, Shape, SignalUse};
use crate::geom::dbu::Dbu;
use crate::geom::rect::Rect;
use crate::verify::shapes::route_elem_shapes;
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

/// One rectangle of the layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LayoutShape {
    /// Whose it is.
    pub owner: OwnerIndex,
    /// The rectangle.
    pub rect: Rect<Dbu>,
    /// Whether it is fixed: part of a cell, an I/O pin, special wiring or a
    /// blockage, rather than routed wiring.
    pub fixed: bool,
}

/// Every shape on every layer, with its owner.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Layout {
    owners: Vec<Owner>,
    supply: Vec<bool>,
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

    /// The index of `owner`, adding it if new; `supply` says whether it
    /// carries power or ground.
    pub fn owner(&mut self, owner: Owner, supply: bool) -> OwnerIndex {
        if let Some(&index) = self.index_of.get(&owner) {
            return index;
        }
        let index = OwnerIndex(u32::try_from(self.owners.len()).unwrap_or(u32::MAX));
        self.owners.push(owner);
        self.supply.push(supply);
        let _ = self.index_of.insert(owner, index);
        index
    }

    /// Adds a rectangle; one on a layer the layout does not have, or
    /// without area, is ignored.
    pub fn add(&mut self, layer: LayerRef, owner: OwnerIndex, rect: Rect<Dbu>, fixed: bool) {
        if rect.width() < Dbu::ZERO || rect.height() < Dbu::ZERO {
            return;
        }
        let shapes = match layer {
            LayerRef::Routing(l) => self.metal.get_mut(l.index()),
            LayerRef::Cut(c) => self.cut.get_mut(c.index()),
        };
        if let Some(shapes) = shapes {
            shapes.push(LayoutShape { owner, rect, fixed });
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
            for s in io.placed_shapes() {
                layout.add(s.layer, owner, s.rect, true);
            }
        }
        for (i, net) in db.special_nets.iter().enumerate() {
            let (owner, supply) = special.owner(db, SpecialNetId::new(i));
            let owner = layout.owner(owner, supply);
            for w in &net.wires {
                layout.add(LayerRef::Routing(w.layer), owner, w.rect(), true);
            }
            for s in &net.rects {
                layout.add(s.layer, owner, s.rect, true);
            }
            for &(via, at) in &net.vias {
                for s in &db.library.via(via).shapes {
                    layout.add(s.layer, owner, s.rect.translate(at), true);
                }
            }
        }
        for (i, s) in db.routing_blockages.iter().enumerate() {
            let owner = layout.owner(Owner::Blockage(BlockageId::new(i)), false);
            layout.add(s.layer, owner, s.rect, true);
        }
        for (i, net) in db.nets.iter().enumerate() {
            let id = NetId::new(i);
            if net.routing.is_empty() {
                continue;
            }
            let owner = layout.owner(Owner::Net(id), net_is_supply(db, &special, id));
            for elem in &net.routing {
                for s in route_elem_shapes(db, elem) {
                    layout.add(s.layer, owner, s.rect, false);
                }
            }
        }
        layout
    }

    fn add_cell(&mut self, db: &NetlistDB, special: &SpecialOwners, cell: CellId) {
        let c = &db.cells[cell.index()];
        let m = db.cell_macro(cell);
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
                self.add(s.layer, owner, place(s), true);
            }
        }
        if !m.obstructions.is_empty() {
            let owner = self.owner(Owner::Obstruction(cell), false);
            for s in &m.obstructions {
                self.add(s.layer, owner, place(s), true);
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
