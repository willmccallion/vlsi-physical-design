//! Buffers removed before placement and added around the ports, as
//! `OpenROAD`'s `remove_buffers` and `buffer_ports` do.

use crate::library::{BufferSet, Link, is_logic_std_cell};
use crate::netlist::{ClockNetwork, Constants, Namer, Netlist};
use pare_common::db::design::{EditError, NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, IdVec, IoPinId, MacroPinId, NetId, PinId};
use pare_common::db::liberty::{BufferCell, LibPortId};
use pare_common::db::library::{DuplicateName, PinDirection};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;

/// Why buffers could not be changed.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum BufferError {
    /// The netlist refused an edit.
    #[error(transparent)]
    Edit(#[from] EditError),
    /// A name was already taken.
    #[error(transparent)]
    Name(#[from] DuplicateName),
    /// The buffer to insert has no master.
    #[error("buffer cell '{0}' has no master")]
    NoMaster(String),
    /// The buffer's input or output pin is missing from its master.
    #[error("buffer cell '{0}' lacks its pins in the master")]
    MissingPins(String),
}

/// Removes every buffer that may go, as `OpenROAD`'s `remove_buffers`
/// without arguments.
///
/// A buffer goes when it is a plain standard cell the library calls a
/// buffer and does not protect, it is not fixed, and its nets merge into
/// one. Returns how many went.
pub fn remove_buffers(db: &mut NetlistDB, link: &Link<'_>) -> Result<usize, BufferError> {
    let mut removed_cells = IdVec::filled(false, db.cells.len());
    let mut removed_nets = IdVec::filled(false, db.nets.len());
    let mut renames: Vec<(String, String)> = Vec::new();
    let mut count = 0;
    for instance in db.cells.ids() {
        let Some((lib_id, lib)) = link.cell(db, instance) else {
            continue;
        };
        if !is_logic_std_cell(db, instance) || !lib.is_buffer() || lib.dont_touch {
            continue;
        }
        if db.cells[instance].status == PlacementStatus::Fixed {
            continue;
        }
        let Some(ports) = link.liberty.buffer_cell(lib_id) else {
            continue;
        };
        let pin_for = |port: LibPortId| -> Option<PinId> {
            let macro_pin = link.macro_pin_of_port(ports.id, port)?;
            db.cells[instance].pins.iter().copied().find(
                |&p| matches!(db.pins[p].owner, PinOwner::Cell { pin, .. } if pin == macro_pin),
            )
        };
        let Some(in_pin) = pin_for(ports.input) else {
            log::warn!(
                "buffer {} has no input net and stays",
                db.cells[instance].name
            );
            continue;
        };
        let Some(out_pin) = pin_for(ports.output) else {
            removed_cells[instance] = true;
            count += 1;
            continue;
        };
        let in_net = db.pins[in_pin].net;
        let out_net = db.pins[out_pin].net;
        if in_net == out_net {
            removed_cells[instance] = true;
            count += 1;
            continue;
        }
        let has_port = |net: NetId| {
            db.nets[net]
                .pins
                .iter()
                .any(|&p| matches!(db.pins[p].owner, PinOwner::Io(_)))
        };
        let (survivor, removed) = if !has_port(in_net) && has_port(out_net) {
            (out_net, in_net)
        } else {
            (in_net, out_net)
        };
        let survivor_name = db.nets[survivor].name.clone();
        let removed_name = db.nets[removed].name.clone();
        if hierarchy_depth(&survivor_name) > hierarchy_depth(&removed_name) {
            renames.push((survivor_name, removed_name));
        }
        let moving: Vec<PinId> = db.nets[removed]
            .pins
            .iter()
            .copied()
            .filter(|&p| match db.pins[p].owner {
                PinOwner::Cell { cell, .. } => cell != instance && !removed_cells[cell],
                PinOwner::Io(_) => true,
            })
            .collect();
        for pin in moving {
            db.move_pin(pin, survivor);
        }
        removed_cells[instance] = true;
        removed_nets[removed] = true;
        count += 1;
    }
    let cells: Vec<CellId> = removed_cells
        .iter_enumerated()
        .filter(|&(_, &removed)| removed)
        .map(|(id, _)| id)
        .collect();
    let nets: Vec<NetId> = removed_nets
        .iter_enumerated()
        .filter(|&(_, &removed)| removed)
        .map(|(id, _)| id)
        .collect();
    let _ = db.remove(&cells, &nets)?;
    for (survivor, name) in renames {
        if let Some(net) = db.net_id(&survivor)
            && db.net_id(&name).is_none()
        {
            db.rename_net(net, name)?;
        }
    }
    Ok(count)
}

/// How deep in the design hierarchy a name sits: its path separators.
fn hierarchy_depth(name: &str) -> usize {
    name.matches('/').count()
}

/// What buffering the ports did.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PortBuffers {
    /// Buffers added after input ports.
    pub inputs: usize,
    /// Buffers added before output ports.
    pub outputs: usize,
}

/// Buffers the design's ports with the weakest buffer, as `OpenROAD`'s
/// `buffer_ports`.
///
/// An input gets a buffer after it unless it is a clock, is constant,
/// drives nothing, drives only buffers or drives a protected cell; an
/// output gets one before it unless it is constant, undriven, or driven by
/// a tristate or protected cell.
pub fn buffer_ports(
    db: &mut NetlistDB,
    link: &Link<'_>,
    buffers: &BufferSet,
    clocks: &ClockNetwork,
    constants: &Constants,
    namer: &mut Namer,
) -> Result<PortBuffers, BufferError> {
    let buffer = buffers.lowest_drive;
    let mut done = PortBuffers::default();
    for io in db.io_pins.ids() {
        let (pin, direction, net) = {
            let p = &db.io_pins[io];
            (p.pin, p.direction, p.net)
        };
        let netlist = Netlist::new(db, link);
        match direction {
            PinDirection::Input => {
                if clocks.is_clock(pin)
                    || constants.is_constant(pin)
                    || netlist.is_special(net)
                    || !netlist.has_instance_pins(net)
                    || !input_wants_buffer(&netlist, pin, net)
                {
                    continue;
                }
                insert_after_driver(db, link, buffer, io, namer)?;
                done.inputs += 1;
            }
            PinDirection::Output => {
                if netlist.is_special(net)
                    || has_tristate_or_protected_driver(&netlist, net)
                    || constants.is_constant(pin)
                    || !netlist.has_instance_pins(net)
                {
                    continue;
                }
                insert_before_load(db, link, buffer, io, namer)?;
                done.outputs += 1;
            }
            PinDirection::Inout | PinDirection::Feedthru => {}
        }
    }
    Ok(done)
}

/// Whether an input port's net has a load that is not a buffer and none
/// that is protected, as `OpenROAD`'s `bufferInput` requires.
fn input_wants_buffer(netlist: &Netlist<'_>, port_pin: PinId, net: NetId) -> bool {
    let mut has_non_buffer = false;
    for pin in &netlist.db.nets[net].pins {
        if *pin == port_pin {
            continue;
        }
        let PinOwner::Cell { cell, .. } = netlist.db.pins[*pin].owner else {
            continue;
        };
        match netlist.link.cell(netlist.db, cell) {
            Some((_, lib)) if lib.dont_touch => {
                log::warn!(
                    "input {} can't be buffered due to protected fanout {}",
                    netlist.db.pin_name(port_pin),
                    netlist.db.cells[cell].name
                );
                return false;
            }
            Some((_, lib)) if !lib.is_buffer() => has_non_buffer = true,
            Some(_) => {}
            None => has_non_buffer = true,
        }
    }
    has_non_buffer
}

/// Whether a net is driven by a tristate output or a protected cell.
fn has_tristate_or_protected_driver(netlist: &Netlist<'_>, net: NetId) -> bool {
    netlist.drivers(net).any(|pin| {
        if netlist
            .lib_port(pin)
            .is_some_and(|p| p.three_state.is_some())
        {
            return true;
        }
        match netlist.db.pins[pin].owner {
            PinOwner::Cell { cell, .. } => netlist
                .link
                .cell(netlist.db, cell)
                .is_some_and(|(_, lib)| lib.dont_touch),
            PinOwner::Io(_) => false,
        }
    })
}

/// A buffer placed with its origin at `at`, not yet connected.
struct NewBuffer {
    cell: CellId,
    input: MacroPinId,
    output: MacroPinId,
}

fn make_buffer(
    db: &mut NetlistDB,
    link: &Link<'_>,
    buffer: BufferCell,
    at: Point<Dbu>,
    base: &str,
    namer: &mut Namer,
) -> Result<NewBuffer, BufferError> {
    let lib = link.liberty.cell(buffer.id);
    let master = link
        .macro_of(buffer.id)
        .ok_or_else(|| BufferError::NoMaster(lib.name.clone()))?;
    let input = link
        .macro_pin_of_port(buffer.id, buffer.input)
        .ok_or_else(|| BufferError::MissingPins(lib.name.clone()))?;
    let output = link
        .macro_pin_of_port(buffer.id, buffer.output)
        .ok_or_else(|| BufferError::MissingPins(lib.name.clone()))?;
    let name = namer.instance(db, base);
    let cell = db.add_cell(name, master, at, Orientation::N, PlacementStatus::Placed)?;
    Ok(NewBuffer {
        cell,
        input,
        output,
    })
}

/// A new net for the port to sit on, named after the port as `OpenROAD`
/// names it, the port's old net taking a generated name if it had the
/// port's.
fn port_net(db: &mut NetlistDB, io: IoPinId, namer: &mut Namer) -> Result<NetId, BufferError> {
    let port_name = db.io_pins[io].name.clone();
    let old = db.io_pins[io].net;
    if db.nets[old].name == port_name {
        let fresh = namer.net(db, "net");
        db.rename_net(old, fresh)?;
    }
    let name = namer.net_if_needed(db, &port_name);
    Ok(db.add_net(&name)?)
}

/// Puts a buffer between an input port and everything it drove: the port
/// moves to a new net feeding the buffer, whose output drives the old net.
fn insert_after_driver(
    db: &mut NetlistDB,
    link: &Link<'_>,
    buffer: BufferCell,
    io: IoPinId,
    namer: &mut Namer,
) -> Result<(), BufferError> {
    let port_pin = db.io_pins[io].pin;
    let old = db.io_pins[io].net;
    let at = db.pin_position(port_pin);
    let new = make_buffer(db, link, buffer, at, "input", namer)?;
    let fresh = port_net(db, io, namer)?;
    db.move_pin(port_pin, fresh);
    let _ = db.connect_cell_pin(new.cell, new.input, fresh);
    let _ = db.connect_cell_pin(new.cell, new.output, old);
    Ok(())
}

/// Puts a buffer between an output port and whatever drove it: the port
/// moves to a new net the buffer drives, fed from the old net.
fn insert_before_load(
    db: &mut NetlistDB,
    link: &Link<'_>,
    buffer: BufferCell,
    io: IoPinId,
    namer: &mut Namer,
) -> Result<(), BufferError> {
    let port_pin = db.io_pins[io].pin;
    let old = db.io_pins[io].net;
    let at = db.pin_position(port_pin);
    let new = make_buffer(db, link, buffer, at, "output", namer)?;
    let fresh = port_net(db, io, namer)?;
    db.move_pin(port_pin, fresh);
    let _ = db.connect_cell_pin(new.cell, new.input, old);
    let _ = db.connect_cell_pin(new.cell, new.output, fresh);
    Ok(())
}
