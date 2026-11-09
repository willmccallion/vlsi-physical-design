//! Buffers removed before placement and added around the ports, as
//! `OpenROAD`'s `remove_buffers` and `buffer_ports` do.

use crate::library::{BufferSet, Link, is_logic_std_cell};
use crate::netlist::{ClockNetwork, Constants, Namer, Netlist};
use pare_common::db::design::{EditError, NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, IdVec, IoPinId, MacroPinId, NetId, PinId, TableIndex};
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
    let mut removal = BufferRemoval {
        cells: IdVec::filled(false, db.cells.len()),
        nets: IdVec::filled(false, db.nets.len()),
        renames: Vec::new(),
    };
    for instance in db.cells.ids() {
        let Some(buffer) = removable_buffer(db, link, instance) else {
            continue;
        };
        let pin_for = |port: LibPortId| -> Option<PinId> {
            let macro_pin = link.macro_pin_of_port(buffer.id, port)?;
            db.cell_pin(instance, macro_pin)
        };
        let Some(in_pin) = pin_for(buffer.input) else {
            log::warn!(
                "buffer {} has no input net and stays",
                db.cells[instance].name
            );
            continue;
        };
        if let Some(out_pin) = pin_for(buffer.output) {
            let (in_net, out_net) = (db.pins[in_pin].net, db.pins[out_pin].net);
            if in_net != out_net {
                removal.merge_nets(db, instance, in_net, out_net);
            }
        }
        removal.cells[instance] = true;
    }
    removal.apply(db)
}

/// The buffer `instance` is when it may go: a plain standard cell the
/// library calls a buffer and does not protect, not fixed.
fn removable_buffer(db: &NetlistDB, link: &Link<'_>, instance: CellId) -> Option<BufferCell> {
    let (lib_id, lib) = link.cell(db, instance)?;
    let removable = is_logic_std_cell(db, instance)
        && lib.is_buffer()
        && !lib.dont_touch
        && db.cells[instance].status != PlacementStatus::Fixed;
    removable
        .then(|| link.liberty.buffer_cell(lib_id))
        .flatten()
}

/// The buffers and nets [`remove_buffers`] takes out, and the nets it
/// renames.
struct BufferRemoval {
    cells: IdVec<CellId, bool>,
    nets: IdVec<NetId, bool>,
    /// The surviving net's name and the deeper name it takes afterwards.
    renames: Vec<(String, String)>,
}

impl BufferRemoval {
    /// Merges buffer `instance`'s input and output nets: the one with a port
    /// survives, else the input, and the other's connections move to it.
    fn merge_nets(&mut self, db: &mut NetlistDB, instance: CellId, in_net: NetId, out_net: NetId) {
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
            self.renames.push((survivor_name, removed_name));
        }
        let moving: Vec<PinId> = db.nets[removed]
            .pins
            .iter()
            .copied()
            .filter(|&p| match db.pins[p].owner {
                PinOwner::Cell { cell, .. } => cell != instance && !self.cells[cell],
                PinOwner::Io(_) => true,
            })
            .collect();
        for pin in moving {
            db.move_pin(pin, survivor);
        }
        self.nets[removed] = true;
    }

    /// Takes the buffers and nets out of `db` and renames the survivors;
    /// returns how many buffers went.
    fn apply(self, db: &mut NetlistDB) -> Result<usize, BufferError> {
        let cells: Vec<CellId> = flagged(&self.cells);
        let nets: Vec<NetId> = flagged(&self.nets);
        let _ = db.remove(&cells, &nets)?;
        for (survivor, name) in self.renames {
            if let Some(net) = db.net_id(&survivor)
                && db.net_id(&name).is_none()
            {
                db.rename_net(net, name)?;
            }
        }
        Ok(cells.len())
    }
}

/// The indices `flags` marks.
fn flagged<I: TableIndex>(flags: &IdVec<I, bool>) -> Vec<I> {
    flags
        .iter_enumerated()
        .filter(|&(_, &marked)| marked)
        .map(|(id, _)| id)
        .collect()
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{DefSections, cells, clock_cells_source, def_text, design, pin};

    /// Buffers to remove: `b0` and `b1` in a chain from port `in0` to flop
    /// `f1`, `b2` from port `in1` to flop `f2`, `b3` from `f1` to port
    /// `out0`, and `b4` between nets inside and outside a hierarchy; and a
    /// fixed buffer `bf`, the only load of port `in2`. Port `out1` is driven
    /// by `f2` and port `out2` by nothing.
    fn buffered() -> NetlistDB {
        design(&def_text(
            "buffered",
            &DefSections {
                components: "- b0 BUF_X1 + PLACED ( 2000 2800 ) N ;\n\
                             - b1 BUF_X1 + PLACED ( 4000 2800 ) N ;\n\
                             - b2 BUF_X1 + PLACED ( 6000 2800 ) N ;\n\
                             - b3 BUF_X1 + PLACED ( 8000 2800 ) N ;\n\
                             - b4 BUF_X1 + PLACED ( 10000 2800 ) N ;\n\
                             - bf BUF_X1 + FIXED ( 12000 2800 ) N ;\n\
                             - f1 DFF_X1 + PLACED ( 20000 2800 ) N ;\n\
                             - f2 DFF_X1 + PLACED ( 30000 2800 ) N ;\n\
                             - f3 DFF_X1 + PLACED ( 40000 2800 ) N ;\n\
                             - f4 DFF_X1 + PLACED ( 50000 2800 ) N ;\n",
                nets: "- in0 ( PIN in0 ) ( b0 A ) ;\n- n0 ( b0 Z ) ( b1 A ) ;\n\
                       - n1 ( b1 Z ) ( f1 D ) ;\n\
                       - in1 ( PIN in1 ) ( b2 A ) ;\n- n2 ( b2 Z ) ( f2 D ) ;\n\
                       - q ( f1 Q ) ( b3 A ) ;\n- out0 ( PIN out0 ) ( b3 Z ) ;\n\
                       - u1/u2/x ( f3 Q ) ( b4 A ) ;\n- y ( b4 Z ) ( f4 D ) ;\n\
                       - in2 ( PIN in2 ) ( bf A ) ;\n- nf ( bf Z ) ( f3 D ) ;\n\
                       - out1 ( PIN out1 ) ( f2 Q ) ;\n- out2 ( PIN out2 ) ;\n\
                       - clk ( PIN clk ) ( f1 CK ) ( f2 CK ) ( f3 CK ) ( f4 CK ) ;\n",
                inputs: &["in0", "in1", "in2", "clk"],
                outputs: &["out0", "out1", "out2"],
            },
        ))
    }

    #[test]
    fn every_buffer_that_is_not_fixed_goes() {
        let mut db = buffered();
        let lib = cells(&clock_cells_source());
        let link = Link::new(&db, &lib, &[]).unwrap();

        let removed = remove_buffers(&mut db, &link).unwrap();

        assert_eq!(removed, 5);
        let left: Vec<&str> = db.cells.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(left, ["bf", "f1", "f2", "f3", "f4"]);
        let in0 = db.io_pins[db.io_pin_id("in0").unwrap()].net;
        assert_eq!(db.pins[pin(&db, "f1/D")].net, in0);
    }

    #[test]
    fn the_net_with_a_port_survives_and_a_deeper_survivor_takes_the_shallower_name() {
        let mut db = buffered();
        let lib = cells(&clock_cells_source());
        let link = Link::new(&db, &lib, &[]).unwrap();

        let _ = remove_buffers(&mut db, &link).unwrap();

        let net_name = |name: &str| db.nets[db.pins[pin(&db, name)].net].name.as_str();
        assert_eq!(net_name("f1/Q"), "out0");
        assert_eq!(net_name("f4/D"), "y");
        assert_eq!(net_name("f3/Q"), "y");
        assert!(db.net_id("u1/u2/x").is_none() && db.net_id("q").is_none());
    }

    #[test]
    fn ports_driving_or_driven_by_logic_get_a_buffer_once_the_buffers_are_gone() {
        let mut db = buffered();
        let lib = cells(&clock_cells_source());
        let link = Link::new(&db, &lib, &[]).unwrap();
        let _ = remove_buffers(&mut db, &link).unwrap();
        let link = Link::new(&db, &lib, &[]).unwrap();
        let buffers = link.find_buffers(&db).unwrap();
        let (clocks, constants) = {
            let netlist = Netlist::new(&db, &link);
            let constants = Constants::find(&netlist);
            let clocks =
                ClockNetwork::from_ports(&netlist, &["clk".to_string()], &constants).unwrap();
            (clocks, constants)
        };

        let buffered = buffer_ports(
            &mut db,
            &link,
            &buffers,
            &clocks,
            &constants,
            &mut Namer::new(),
        )
        .unwrap();

        assert_eq!(
            buffered,
            PortBuffers {
                inputs: 2,
                outputs: 2
            }
        );
        let port_net = |port: &str| db.io_pins[db.io_pin_id(port).unwrap()].net;
        assert_eq!(db.nets[port_net("in0")].name, "in0");
        assert_eq!(db.nets[port_net("in0")].pins.len(), 2);
    }
}
