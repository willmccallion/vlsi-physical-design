//! The improver's netlist, as `Opendp::createNetwork` builds it: the
//! legalizer's cells, then the placed I/O pins of signal nets, then the
//! fillers that close gaps between rows, with each signal net's pins on them.

use super::{EdgeId, ImprovementError, NetworkPinId, NodeId, Placer, Result};
use pare_common::db::design::{Net, NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, IdVec, IoPinId, MacroPinId};
use pare_common::db::library::{Macro, SignalUse};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::Interval;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// A pin, as `Pin`: where it is from its node's center.
#[derive(Clone, Copy, Debug)]
pub(super) struct Pin {
    pub(super) node: NodeId,
    pub(super) edge: EdgeId,
    pub(super) offset: Point<Dbu>,
}

/// A node that never moves and is not one of the legalizer's cells: an I/O
/// pin, or a filler closing a gap in the rows.
#[derive(Clone, Copy, Debug)]
pub(super) struct FixedNode {
    pub(super) left: Dbu,
    pub(super) bottom: Dbu,
    pub(super) width: Dbu,
    pub(super) height: Dbu,
    pub(super) terminal: bool,
}

/// The supply along an edge of a master, as `Architecture::Row`'s power
/// types.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Rail {
    Power,
    Ground,
}

/// The supplies along a master's bottom and top edges, as `getMasterPwrs`
/// finds them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Rails {
    bottom: Rail,
    top: Rail,
}

#[derive(Debug)]
pub(super) struct Network {
    /// How many of the nodes, the first, are the legalizer's cells, which
    /// keep their [`NodeId`]s.
    pub(super) cells: usize,
    /// The nodes after the cells.
    fixed: Vec<FixedNode>,
    /// Where each cell started, relative to the core.
    pub(super) orig: IdVec<NodeId, Point<Dbu>>,
    /// Each cell's rails, none when its master lacks power or ground.
    pub(super) rails: IdVec<NodeId, Option<Rails>>,
    pub(super) pins: IdVec<NetworkPinId, Pin>,
    /// The pins of every node.
    pub(super) node_pins: IdVec<NodeId, Vec<NetworkPinId>>,
    /// The pins of each signal net.
    pub(super) edges: IdVec<EdgeId, Vec<NetworkPinId>>,
    /// Hard placement blockages, relative to the core.
    pub(super) blockages: Vec<Rect<Dbu>>,
}

impl Network {
    pub(super) fn new(db: &NetlistDB, grid: &Placer) -> Result<Self> {
        let cells = grid.nodes.len();
        let mut node_of_cell = IdVec::filled(None, db.cells.len());
        for (n, node) in grid.nodes.iter_enumerated() {
            node_of_cell[node.cell] = Some(n);
        }
        let core = grid.rows.core();
        let io = io_pin_nodes(db, cells, core)?;
        let orig = grid
            .nodes
            .iter()
            .map(|n| Point::new(n.left, n.bottom))
            .collect();
        let rails = grid
            .nodes
            .iter()
            .map(|n| master_rails(db.cell_macro(n.cell)))
            .collect();
        let mut network = Self {
            cells,
            fixed: io.fixed,
            orig,
            rails,
            pins: IdVec::new(),
            node_pins: IdVec::filled(Vec::new(), cells),
            edges: IdVec::new(),
            blockages: db
                .hard_placement_blockages()
                .map(|b| Rect::new(b.min - core.min, b.max - core.min))
                .collect(),
        };
        for _ in 0..network.fixed.len() {
            let _ = network.node_pins.push(Vec::new());
        }
        let nodes = NodesOf {
            cell: node_of_cell,
            io: io.node_of_io,
        };
        for net in db.nets.iter().filter(|net| !net.signal_use.is_supply()) {
            network.add_net(db, net, &nodes);
        }
        Ok(network)
    }

    /// Adds `net` as an edge with a pin for each of its pins on a node:
    /// those on cells first, then those on I/O pins.
    fn add_net(&mut self, db: &NetlistDB, net: &Net, nodes: &NodesOf) {
        let edge = self.edges.push(Vec::new());
        for &pin in &net.pins {
            let PinOwner::Cell {
                cell,
                pin: macro_pin,
            } = db.pins[pin].owner
            else {
                continue;
            };
            let Some(node) = nodes.cell[cell] else {
                continue;
            };
            self.add_pin(node, edge, cell_pin_offset(db, cell, macro_pin));
        }
        for &pin in &net.pins {
            if let PinOwner::Io(io) = db.pins[pin].owner
                && let Some(node) = nodes.io[io]
            {
                self.add_pin(node, edge, Point::new(Dbu::ZERO, Dbu::ZERO));
            }
        }
    }

    fn add_pin(&mut self, node: NodeId, edge: EdgeId, offset: Point<Dbu>) {
        let pin = self.pins.push(Pin { node, edge, offset });
        self.node_pins[node].push(pin);
        self.edges[edge].push(pin);
    }

    /// Adds a fixed node closing a gap in a row, as
    /// `Network::addFillerNode`.
    pub(super) fn add_filler(&mut self, left: Dbu, bottom: Dbu, width: Dbu, height: Dbu) {
        self.fixed.push(FixedNode {
            left,
            bottom,
            width,
            height,
            terminal: false,
        });
        let _ = self.node_pins.push(Vec::new());
    }

    pub(super) const fn node_count(&self) -> usize {
        self.cells + self.fixed.len()
    }

    /// Every node: the cells, then the fixed nodes.
    pub(super) fn nodes(&self) -> impl Iterator<Item = NodeId> + use<> {
        (0..self.node_count()).map(NodeId::new)
    }

    /// Whether `n` is one of the legalizer's cells rather than a fixed node.
    pub(super) const fn is_cell(&self, n: NodeId) -> bool {
        n.index() < self.cells
    }

    /// The fixed node `n`, which must come after the cells.
    pub(super) fn fixed_node(&self, n: NodeId) -> &FixedNode {
        &self.fixed[n.index() - self.cells]
    }
}

/// A master's rails: each edge carries whichever supply reaches nearer it;
/// none unless it has both power and ground shapes, as `getMasterPwrs`.
// NOTE: getMasterPwrs takes the center of each pin port's box; PARE's
// library keeps a pin's shapes but not its ports, so it takes each shape's.
fn master_rails(m: &Macro) -> Option<Rails> {
    let centers = |supply: SignalUse| {
        m.pins
            .iter()
            .filter(|p| p.signal_use == supply)
            .flat_map(|p| &p.shapes)
            .map(|s| (s.rect.min.y + s.rect.max.y) / 2)
            .fold(None, |span: Option<Interval>, y| {
                Some(span.map_or_else(
                    || Interval::new(y, y),
                    |s| Interval::new(s.lo.min(y), s.hi.max(y)),
                ))
            })
    };
    let (power, ground) = (centers(SignalUse::Power)?, centers(SignalUse::Ground)?);
    let nearer = |is_power: bool| if is_power { Rail::Power } else { Rail::Ground };
    Some(Rails {
        bottom: nearer(power.lo < ground.lo),
        top: nearer(power.hi > ground.hi),
    })
}

const fn is_placed(status: PlacementStatus) -> bool {
    matches!(
        status,
        PlacementStatus::Placed | PlacementStatus::Fixed | PlacementStatus::Cover
    )
}
/// The node each cell and each placed signal I/O pin is, where it is one.
struct NodesOf {
    cell: IdVec<CellId, Option<NodeId>>,
    io: IdVec<IoPinId, Option<NodeId>>,
}

/// The fixed terminal nodes of the placed signal I/O pins.
struct IoPinNodes {
    /// One per placed signal I/O pin, numbered after the cell nodes.
    fixed: Vec<FixedNode>,
    /// The node of each I/O pin, where it is one.
    node_of_io: IdVec<IoPinId, Option<NodeId>>,
}

/// A fixed terminal node for each placed signal I/O pin, numbered after
/// the `cells` cell nodes.
fn io_pin_nodes(db: &NetlistDB, cells: usize, core: Rect<Dbu>) -> Result<IoPinNodes> {
    let mut fixed = Vec::new();
    let mut node_of_io = IdVec::filled(None, db.io_pins.len());
    for (i, io) in db.io_pins.iter_enumerated() {
        if db.nets[io.net].signal_use.is_supply() || !is_placed(io.status) {
            continue;
        }
        let bbox = io
            .placed_shapes()
            .map(|s| s.rect)
            .reduce(|a, b| a.union(&b))
            .ok_or_else(|| ImprovementError::PinWithoutShapes(io.name.clone()))?;
        node_of_io[i] = Some(NodeId::new(cells + fixed.len()));
        fixed.push(FixedNode {
            left: bbox.min.x - core.min.x,
            bottom: bbox.min.y - core.min.y,
            width: bbox.width(),
            // NOTE: Network::addNode(dbBTerm*) takes yMax - yMax, putting
            // the pin at its shape's bottom; PARE takes the shape's height.
            height: bbox.height(),
            terminal: true,
        });
    }
    Ok(IoPinNodes { fixed, node_of_io })
}

/// Where a cell's pin sits from the middle of its master: the center of
/// the box around the pin's shapes, as `Network::addPin` measures it.
fn cell_pin_offset(db: &NetlistDB, cell: CellId, macro_pin: MacroPinId) -> Point<Dbu> {
    let m = db.cell_macro(cell);
    let bbox = m
        .pin(macro_pin)
        .shapes
        .iter()
        .map(|s| s.rect)
        .reduce(|a, b| a.union(&b))
        .unwrap_or_default();
    // NOTE: Network::addPin takes Rect::xCenter, truncating toward zero;
    // PARE rounds down as Rect::center does, which differs only for pin
    // shapes below or left of the master's origin.
    let center = bbox.center();
    Point::new(center.x - m.size.x / 2, center.y - m.size.y / 2)
}
