//! The improver's netlist, as `Opendp::createNetwork` builds it: the
//! legalizer's cells, then the placed I/O pins of signal nets, then the
//! fillers that close gaps between rows, with each signal net's pins on them.

use super::{EdgeId, ImprovementError, NetworkPinId, NodeId, Placer, Result};
use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::IdVec;
use pare_common::geom::dbu::Dbu;
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

#[derive(Debug)]
pub(super) struct Network {
    /// How many of the nodes, the first, are the legalizer's cells, which
    /// keep their [`NodeId`]s.
    pub(super) cells: usize,
    /// The nodes after the cells.
    fixed: Vec<FixedNode>,
    /// Where each cell started, relative to the core.
    pub(super) orig: IdVec<NodeId, Point<Dbu>>,
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
        let orig = grid
            .nodes
            .iter()
            .map(|n| Point::new(n.left, n.bottom))
            .collect();
        let mut network = Self {
            cells,
            fixed,
            orig,
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
        for net in &db.nets {
            if net.signal_use.is_supply() {
                continue;
            }
            let edge = network.edges.push(Vec::new());
            for &pin in &net.pins {
                let PinOwner::Cell {
                    cell,
                    pin: macro_pin,
                } = db.pins[pin].owner
                else {
                    continue;
                };
                let Some(node) = node_of_cell[cell] else {
                    continue;
                };
                let m = db.cell_macro(cell);
                let bbox = m
                    .pin(macro_pin)
                    .shapes
                    .iter()
                    .map(|s| s.rect)
                    .reduce(|a, b| a.union(&b))
                    .unwrap_or_default();
                let center_x = Dbu(bbox.min.x.get().midpoint(bbox.max.x.get()));
                let center_y = Dbu(bbox.min.y.get().midpoint(bbox.max.y.get()));
                let offset = Point::new(center_x - m.size.x / 2, center_y - m.size.y / 2);
                network.add_pin(node, edge, offset);
            }
            for &pin in &net.pins {
                if let PinOwner::Io(io) = db.pins[pin].owner
                    && let Some(node) = node_of_io[io]
                {
                    network.add_pin(node, edge, Point::new(Dbu::ZERO, Dbu::ZERO));
                }
            }
        }
        Ok(network)
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

const fn is_placed(status: PlacementStatus) -> bool {
    matches!(
        status,
        PlacementStatus::Placed | PlacementStatus::Fixed | PlacementStatus::Cover
    )
}
