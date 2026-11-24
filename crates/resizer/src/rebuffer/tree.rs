//! Buffer trees: importing a net's existing buffers and putting the chosen ones into the design.

use super::{Rebuffer, wire};
use crate::bnet::{Bnet, BnetPtr, Kind};
use crate::buffers::removable_buffer;
use crate::library::is_logic_std_cell;
use crate::resizer::{RepairError, Resizer};
use crate::sta::Sta;
use pare_common::db::design::PinOwner;
use pare_common::db::indices::{CellId, NetId, PinId};
use std::rc::Rc;

impl Rebuffer {
    /// The tree of `driver`'s net with the removable buffers under it
    /// folded in as buffer nodes, as `importBufferTree`.
    pub(super) fn import_buffer_tree(
        &self,
        resizer: &Resizer<'_, Sta>,
        driver: PinId,
    ) -> Result<Option<BnetPtr>, RepairError> {
        let Some(tree) = resizer.buffered_tree(driver)? else {
            return Ok(None);
        };
        self.import_node(resizer, &tree).map(Some)
    }

    fn import_node(
        &self,
        resizer: &Resizer<'_, Sta>,
        node: &BnetPtr,
    ) -> Result<BnetPtr, RepairError> {
        Ok(match &node.kind {
            Kind::Wire { to, layer } => {
                let inner = self.import_node(resizer, to)?;
                wire(resizer, node.at, *layer, inner)
            }
            Kind::Via {
                to,
                layer,
                to_layer,
            } => {
                let inner = self.import_node(resizer, to)?;
                Bnet::via(node.at, *layer, *to_layer, inner, &resizer.tree_rc())
            }
            Kind::Junction { left, right } => {
                let left = self.import_node(resizer, left)?;
                let right = self.import_node(resizer, right)?;
                Bnet::junction(node.at, left, right)
            }
            Kind::Buffer { buffer, to } => {
                let inner = self.import_node(resizer, to)?;
                self.buffer_node(resizer, node.at, *buffer, inner)
            }
            Kind::Load { pin } => return self.import_load(resizer, node, *pin),
        })
    }

    /// A load of an imported tree, with the tree a removable buffer there
    /// drives folded in under it.
    fn import_load(
        &self,
        resizer: &Resizer<'_, Sta>,
        node: &BnetPtr,
        pin: PinId,
    ) -> Result<BnetPtr, RepairError> {
        let PinOwner::Cell { cell, .. } = resizer.db.pins[pin].owner else {
            return Ok(Rc::clone(node));
        };
        let Some((lib_id, lib)) = resizer.link.cell(resizer.db, cell) else {
            return Ok(Rc::clone(node));
        };
        let netlist = resizer.netlist();
        let is_input = netlist
            .lib_port(pin)
            .is_some_and(pare_common::db::liberty::LibPort::is_input);
        let buffer = resizer
            .link
            .liberty
            .buffer_cell(lib_id)
            .filter(|_| lib.is_buffer());
        let Some(buffer) = buffer.filter(|_| {
            is_input
                && is_logic_std_cell(resizer.db, cell)
                && !is_port_buffer(resizer, cell)
                && can_remove_buffer(resizer, cell)
        }) else {
            return Ok(Rc::clone(node));
        };
        let Some((_, out_pin)) = resizer.buffer_pins(cell) else {
            return Ok(Rc::clone(node));
        };
        let Some(inner) = self.import_buffer_tree(resizer, out_pin)? else {
            return Ok(Rc::clone(node));
        };
        Ok(self.buffer_node(resizer, node.at, buffer, inner))
    }
}

/// Puts the buffers of a chosen tree into the design, bottom up, as
/// `exportBufferTree`; returns how many went in.
pub(super) fn export_tree(
    resizer: &mut Resizer<'_, Sta>,
    tree: &BnetPtr,
    base: &str,
) -> Result<usize, RepairError> {
    let mut count = 0;
    let _ = export_node(resizer, tree, base, &mut count)?;
    Ok(count)
}

/// Inserts the buffers under `node` and returns the pins its parent
/// drives.
fn export_node(
    resizer: &mut Resizer<'_, Sta>,
    node: &BnetPtr,
    base: &str,
    count: &mut usize,
) -> Result<Vec<PinId>, RepairError> {
    match &node.kind {
        Kind::Load { pin } => Ok(vec![*pin]),
        Kind::Wire { to, .. } | Kind::Via { to, .. } => export_node(resizer, to, base, count),
        Kind::Junction { left, right } => {
            let mut loads = export_node(resizer, left, base, count)?;
            loads.extend(export_node(resizer, right, base, count)?);
            Ok(loads)
        }
        Kind::Buffer { buffer, to } => {
            let loads = export_node(resizer, to, base, count)?;
            if loads.is_empty() {
                return Ok(loads);
            }
            let current = resizer.db.pins[loads[0]].net;
            let inserted =
                resizer.insert_buffer_before_loads(current, &loads, *buffer, node.at, base)?;
            *count += 1;
            Ok(vec![inserted.input])
        }
    }
}

/// The tree without its buffers, as `stripTreeBuffers`.
pub(super) fn strip_buffers(node: &BnetPtr) -> BnetPtr {
    match &node.kind {
        Kind::Load { .. } => Rc::clone(node),
        Kind::Buffer { to, .. } => strip_buffers(to),
        Kind::Wire { to, layer } => Rc::new(Bnet {
            kind: Kind::Wire {
                to: strip_buffers(to),
                layer: *layer,
            },
            ..**node
        }),
        Kind::Via {
            to,
            layer,
            to_layer,
        } => Rc::new(Bnet {
            kind: Kind::Via {
                to: strip_buffers(to),
                layer: *layer,
                to_layer: *to_layer,
            },
            ..**node
        }),
        Kind::Junction { left, right } => Rc::new(Bnet {
            kind: Kind::Junction {
                left: strip_buffers(left),
                right: strip_buffers(right),
            },
            ..**node
        }),
    }
}

/// The loads and buffers a new Steiner tree must reach, as `resteiner`
/// gathers them.
pub(super) fn collect_sinks(node: &BnetPtr, sinks: &mut Vec<BnetPtr>) {
    match &node.kind {
        Kind::Load { .. } | Kind::Buffer { .. } => sinks.push(Rc::clone(node)),
        Kind::Wire { to, .. } | Kind::Via { to, .. } => collect_sinks(to, sinks),
        Kind::Junction { left, right } => {
            collect_sinks(left, sinks);
            collect_sinks(right, sinks);
        }
    }
}

/// The buffers downstream of `driver` that `imported` did not keep as
/// loads: those to remove once the net is rebuffered, as
/// `collectImportedTreeBufferInstances`.
pub(super) fn collect_imported_buffers(
    resizer: &Resizer<'_, Sta>,
    driver: PinId,
    imported: &Bnet,
) -> Vec<CellId> {
    let kept: rustc_hash::FxHashSet<PinId> = imported.loads().into_iter().collect();
    let netlist = resizer.netlist();
    let mut buffers = Vec::new();
    let mut queue = vec![resizer.db.pins[driver].net];
    while let Some(net) = queue.pop() {
        for load in netlist.loads(net) {
            let PinOwner::Cell { cell, .. } = resizer.db.pins[load].owner else {
                continue;
            };
            let Some((_, lib)) = resizer.link.cell(resizer.db, cell) else {
                continue;
            };
            if !lib.is_buffer() || kept.contains(&load) {
                continue;
            }
            let Some((in_pin, out_pin)) = resizer.buffer_pins(cell) else {
                continue;
            };
            if in_pin != load {
                continue;
            }
            buffers.push(cell);
            queue.push(resizer.db.pins[out_pin].net);
        }
    }
    buffers
}

/// Whether a buffer sits directly on a port's net with nothing else, as
/// `isPortBuffer`.
pub(super) fn is_port_buffer(resizer: &Resizer<'_, Sta>, cell: CellId) -> bool {
    let netlist = resizer.netlist();
    resizer.db.cells[cell].pins.iter().any(|&pin| {
        let net = resizer.db.pins[pin].net;
        let instance_pins = resizer.db.nets[net]
            .pins
            .iter()
            .filter(|&&p| matches!(resizer.db.pins[p].owner, PinOwner::Cell { .. }))
            .count();
        instance_pins == 1 && netlist.has_port(net)
    })
}

/// Whether setup repair may take a buffer out, as
/// `UnbufferMove::canRemoveBuffer` honouring dont-touch and fixed cells,
/// with both its pins connected, as `UnbufferMove::removeBuffer` requires.
pub(crate) fn can_remove_buffer(resizer: &Resizer<'_, Sta>, cell: CellId) -> bool {
    removable_buffer(resizer.db, resizer.link, Some(resizer.sta().sdc()), cell).is_some()
        && resizer.buffer_pins(cell).is_some()
}

/// Whether a net reaches an output port of the design.
pub(super) fn has_top_level_output(resizer: &Resizer<'_, Sta>, net: NetId) -> bool {
    resizer.db.nets[net].pins.iter().any(|&p| {
        matches!(resizer.db.pins[p].owner, PinOwner::Io(io)
            if matches!(resizer.db.io_pins[io].direction,
                pare_common::db::library::PinDirection::Output | pare_common::db::library::PinDirection::Inout))
    })
}
