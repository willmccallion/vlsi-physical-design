//! The improver's netlist, as `Opendp::createNetwork` builds it: the
//! legalizer's cells, then the placed I/O pins of signal nets, then the
//! fillers that close gaps between rows, with each signal net's pins on them.

use super::{ImprovementError, Placer, Result};
use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::rect::Rect;

/// A pin, as `Pin`: where it is from its node's center.
#[derive(Clone, Copy, Debug)]
pub(super) struct Pin {
    pub(super) node: usize,
    pub(super) edge: usize,
    pub(super) offset: (i64, i64),
}

/// A node that never moves and is not one of the legalizer's cells: an I/O
/// pin, or a filler closing a gap in the rows.
#[derive(Clone, Copy, Debug)]
pub(super) struct FixedNode {
    pub(super) left: i64,
    pub(super) bottom: i64,
    pub(super) width: i64,
    pub(super) height: i64,
    pub(super) terminal: bool,
}

#[derive(Debug)]
pub(super) struct Network {
    /// Nodes below this are the legalizer's cells, by index.
    pub(super) cells: usize,
    /// The nodes after the cells.
    pub(super) fixed: Vec<FixedNode>,
    /// Where each cell started, relative to the core.
    pub(super) orig: Vec<(i64, i64)>,
    pub(super) pins: Vec<Pin>,
    pub(super) node_pins: Vec<Vec<usize>>,
    /// The pins of each signal net.
    pub(super) edges: Vec<Vec<usize>>,
    /// Hard placement blockages, relative to the core.
    pub(super) blockages: Vec<Rect<Dbu>>,
}

impl Network {
    pub(super) fn new(db: &NetlistDB, grid: &Placer) -> Result<Self> {
        let cells = grid.nodes.len();
        let mut node_of_cell = vec![None; db.cells.len()];
        for (n, node) in grid.nodes.iter().enumerate() {
            node_of_cell[node.cell.index()] = Some(n);
        }
        let core = grid.core;
        let mut fixed = Vec::new();
        let mut node_of_io = vec![None; db.io_pins.len()];
        for (i, io) in db.io_pins.iter().enumerate() {
            if db.nets[io.net.index()].signal_use.is_supply() || !is_placed(io.status) {
                continue;
            }
            let bbox = io
                .placed_shapes()
                .map(|s| s.rect)
                .reduce(|a, b| a.union(&b))
                .ok_or_else(|| ImprovementError::PinWithoutShapes(io.name.clone()))?;
            node_of_io[i] = Some(cells + fixed.len());
            fixed.push(FixedNode {
                left: (bbox.min.x - core.min.x).get(),
                bottom: (bbox.min.y - core.min.y).get(),
                width: bbox.width().get(),
                height: 0,
                terminal: true,
            });
        }
        let orig = grid.nodes.iter().map(|n| (n.left, n.bottom)).collect();
        let mut network = Self {
            cells,
            fixed,
            orig,
            pins: Vec::new(),
            node_pins: vec![Vec::new(); cells],
            edges: Vec::new(),
            blockages: db
                .placement_blockages
                .iter()
                .map(|b| Rect::new(b.min - core.min, b.max - core.min))
                .collect(),
        };
        network
            .node_pins
            .resize(cells + network.fixed.len(), Vec::new());
        for net in &db.nets {
            if net.signal_use.is_supply() {
                continue;
            }
            let edge = network.edges.len();
            network.edges.push(Vec::new());
            for &pin in &net.pins {
                let PinOwner::Cell {
                    cell,
                    pin: macro_pin,
                } = db.pins[pin.index()].owner
                else {
                    continue;
                };
                let Some(node) = node_of_cell[cell.index()] else {
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
                let center_x = bbox.min.x.get().midpoint(bbox.max.x.get());
                let center_y = bbox.min.y.get().midpoint(bbox.max.y.get());
                let offset = (center_x - m.size.x.get() / 2, center_y - m.size.y.get() / 2);
                network.add_pin(node, edge, offset);
            }
            for &pin in &net.pins {
                if let PinOwner::Io(io) = db.pins[pin.index()].owner
                    && let Some(node) = node_of_io[io.index()]
                {
                    network.add_pin(node, edge, (0, 0));
                }
            }
        }
        Ok(network)
    }

    fn add_pin(&mut self, node: usize, edge: usize, offset: (i64, i64)) {
        let pin = self.pins.len();
        self.pins.push(Pin { node, edge, offset });
        self.node_pins[node].push(pin);
        self.edges[edge].push(pin);
    }

    /// Adds a fixed node closing a gap in a row, as
    /// `Network::addFillerNode`.
    pub(super) fn add_filler(&mut self, left: i64, bottom: i64, width: i64, height: i64) {
        self.fixed.push(FixedNode {
            left,
            bottom,
            width,
            height,
            terminal: false,
        });
        self.node_pins.push(Vec::new());
    }

    pub(super) const fn node_count(&self) -> usize {
        self.cells + self.fixed.len()
    }
}

const fn is_placed(status: PlacementStatus) -> bool {
    matches!(
        status,
        PlacementStatus::Placed | PlacementStatus::Fixed | PlacementStatus::Cover
    )
}
