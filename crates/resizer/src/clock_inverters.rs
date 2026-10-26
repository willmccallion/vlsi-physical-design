//! Clock inverters cloned next to each of their loads, as `OpenROAD`'s
//! `repair_clock_inverters` does before clock tree synthesis so that the
//! tree reaches the registers rather than one inverter they share.

use crate::library::Link;
use crate::netlist::{ClockNetwork, Namer, Netlist};
use crate::resizer::RepairError;
use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus, Renumbering};
use pare_common::db::indices::{CellId, MacroPinId, NetId, PinId};
use pare_common::db::liberty::ArcRole;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use std::collections::VecDeque;

/// What cloning the clock inverters did.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ClockInverterRepair {
    /// Inverters cloned.
    pub inverters: usize,
    /// Clones made.
    pub clones: usize,
    /// Where every cell, net and pin went when the originals were removed.
    pub renumbering: Option<Renumbering>,
}

/// Gives every load of every inverter in the clock network a clone of its
/// own, placed at the load, and removes each original that no longer drives
/// a port, as `Resizer::repairClkInverters`.
pub fn repair_clock_inverters(
    db: &mut NetlistDB,
    link: &Link<'_>,
    clocks: &ClockNetwork,
    namer: &mut Namer,
) -> Result<ClockInverterRepair, RepairError> {
    let inverters = find_clock_inverters(db, link, clocks);
    let mut done = ClockInverterRepair::default();
    let mut removed = Vec::new();
    for inverter in inverters {
        if db.cells[inverter.index()].status == PlacementStatus::Fixed
            || link
                .cell(db, inverter)
                .is_some_and(|(_, lib)| lib.dont_touch)
        {
            continue;
        }
        if clone_inverter(db, link, inverter, namer, &mut done)? {
            removed.push(inverter);
        }
        done.inverters += 1;
    }
    if !removed.is_empty() {
        let dead_nets: Vec<NetId> = removed
            .iter()
            .filter_map(|&cell| output_pin(db, link, cell))
            .map(|pin| db.pins[pin.index()].net)
            .collect();
        for &cell in &removed {
            db.retire_cell(cell);
        }
        done.renumbering = Some(db.remove(&[], &dead_nets)?);
    }
    Ok(done)
}

fn output_pin(db: &NetlistDB, link: &Link<'_>, cell: CellId) -> Option<PinId> {
    let (lib_id, lib) = link.cell(db, cell)?;
    let (_, out) = lib.buffer_ports()?;
    let macro_pin = link.macro_pin_of_port(lib_id, out)?;
    db.cells[cell.index()].pins.iter().copied().find(|&p| {
        matches!(db.pins[p.index()].owner, PinOwner::Cell { pin, .. } if pin.index() == macro_pin)
    })
}

/// The inverters the clocks pass through, breadth first from where they
/// enter, through wires and combinational arcs, stopping at register
/// clocks, as `Resizer::findClkInverters`.
fn find_clock_inverters(db: &NetlistDB, link: &Link<'_>, clocks: &ClockNetwork) -> Vec<CellId> {
    let netlist = Netlist::new(db, link);
    let mut seen = vec![false; db.pins.len()];
    let mut queue: VecDeque<PinId> = clocks.sources().iter().copied().collect();
    for &p in &queue {
        seen[p.index()] = true;
    }
    let mut found = Vec::new();
    while let Some(pin) = queue.pop_front() {
        let port = link.pin_port(db, pin);
        if netlist.is_driver(pin)
            && let Some((lib, _)) = port
            && link.liberty.cell(lib).is_inverter()
            && let PinOwner::Cell { cell, .. } = db.pins[pin.index()].owner
            && !found.contains(&cell)
        {
            found.push(cell);
        }
        if port.is_some_and(|(lib, p)| link.liberty.cell(lib).is_register_clock(p)) {
            continue;
        }
        let mut next = Vec::new();
        if netlist.is_driver(pin) {
            next.extend(netlist.loads(db.pins[pin.index()].net));
        } else if let (Some((lib, from)), PinOwner::Cell { cell, .. }) =
            (port, db.pins[pin.index()].owner)
        {
            for set in &link.liberty.cell(lib).arc_sets {
                if set.from == from
                    && set.role == ArcRole::Combinational
                    && let Some(out) = netlist.pin_of_port(cell, set.to)
                {
                    next.push(out);
                }
            }
        }
        for n in next {
            if !seen[n.index()] {
                seen[n.index()] = true;
                queue.push_back(n);
            }
        }
    }
    found
}

/// Clones one inverter for each instance load; returns whether the
/// original is left driving nothing.
fn clone_inverter(
    db: &mut NetlistDB,
    link: &Link<'_>,
    inverter: CellId,
    namer: &mut Namer,
    done: &mut ClockInverterRepair,
) -> Result<bool, RepairError> {
    let Some((lib_id, lib)) = link.cell(db, inverter) else {
        return Ok(false);
    };
    let Some((in_port, out_port)) = lib.buffer_ports() else {
        return Ok(false);
    };
    let (Some(in_macro), Some(out_macro)) = (
        link.macro_pin_of_port(lib_id, in_port),
        link.macro_pin_of_port(lib_id, out_port),
    ) else {
        return Ok(false);
    };
    let pin_of = |db: &NetlistDB, macro_pin: usize| {
        db.cells[inverter.index()].pins.iter().copied().find(|&p| {
            matches!(db.pins[p.index()].owner, PinOwner::Cell { pin, .. } if pin.index() == macro_pin)
        })
    };
    let (Some(in_pin), Some(out_pin)) = (pin_of(db, in_macro), pin_of(db, out_macro)) else {
        return Ok(false);
    };
    let in_net = db.pins[in_pin.index()].net;
    let out_net = db.pins[out_pin.index()].net;
    let master = db.cells[inverter.index()].macro_id;
    let base = format!("{}_", db.cells[inverter.index()].name);
    let loads: Vec<PinId> = db.nets[out_net.index()]
        .pins
        .iter()
        .copied()
        .filter(|&p| p != out_pin && matches!(db.pins[p.index()].owner, PinOwner::Cell { .. }))
        .collect();
    for load in loads {
        let at = db.pin_position(load);
        let name = namer.instance(db, &base);
        let clone = db.add_cell(
            name,
            master,
            Point::new(at.x, at.y),
            Orientation::N,
            PlacementStatus::Placed,
        )?;
        let net_name = namer.net(db, "net");
        let clone_net = db.add_net(&net_name)?;
        db.nets[clone_net.index()].signal_use = db.nets[in_net.index()].signal_use;
        let _ = db.connect_cell_pin(clone, MacroPinId::new(in_macro), in_net);
        let _ = db.connect_cell_pin(clone, MacroPinId::new(out_macro), clone_net);
        db.move_pin(load, clone_net);
        done.clones += 1;
    }
    let drives_port = db.nets[out_net.index()]
        .pins
        .iter()
        .any(|&p| matches!(db.pins[p.index()].owner, PinOwner::Io(_)));
    Ok(!drives_port)
}
