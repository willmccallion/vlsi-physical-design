//! Each net's drivers, kept as edits connect and disconnect pins rather than
//! found from the net's pins at every look, which on a clock net costs every
//! flop the net clocks each time timing passes one of them.

use super::TimingGraph;
use crate::netlist::Netlist;
use pare_common::db::indices::{CellId, IdVec, NetId, PinId};

/// Each net's drivers in `netlist`, in the order the net lists its pins.
pub(super) fn find_drivers(netlist: &Netlist<'_>) -> IdVec<NetId, Vec<PinId>> {
    netlist
        .db
        .nets
        .ids()
        .map(|net| netlist.drivers(net).collect())
        .collect()
}

impl TimingGraph {
    /// The pins driving `net`, in the order the net lists them, as the
    /// wire edges into a load of `OpenSTA`'s graph lead back to them.
    #[must_use]
    pub(crate) fn drivers(&self, net: NetId) -> &[PinId] {
        self.net_drivers.get(net).map_or(&[], Vec::as_slice)
    }

    /// Takes in `pin`, put on its net, as `Sta::connectPinAfter` makes the
    /// wire edges from a driver. A driver that is its net's last pin or
    /// only driver goes last; one put in among others has the net's drivers
    /// found again. Taking in a pin already listed changes nothing, so the
    /// pins of a cell put back together can be taken in one by one.
    pub(crate) fn pin_connected(&mut self, netlist: &Netlist<'_>, pin: PinId) {
        if !netlist.is_driver(pin) {
            return;
        }
        let net = netlist.db.pins[pin].net;
        let is_last = netlist.db.nets[net].pins.last() == Some(&pin);
        let drivers = self.net_drivers_mut(net);
        if drivers.contains(&pin) {
            return;
        }
        if drivers.is_empty() || is_last {
            drivers.push(pin);
        } else {
            *drivers = netlist.drivers(net).collect();
        }
    }

    /// Drops `pin`, taken off `net`, as `Sta::disconnectPinBefore` deletes
    /// the wire edges from a driver.
    pub(crate) fn pin_disconnected(&mut self, net: NetId, pin: PinId) {
        if let Some(drivers) = self.net_drivers.get_mut(net) {
            drivers.retain(|&p| p != pin);
        }
    }

    /// Takes in `pin`, moved to its net from `from`.
    pub(crate) fn pin_moved(&mut self, netlist: &Netlist<'_>, pin: PinId, from: NetId) {
        self.pin_disconnected(from, pin);
        self.pin_connected(netlist, pin);
    }

    /// Takes in `cell`'s new master, as `Sta::replaceCellAfter` makes its
    /// instance's edges again: each net on which the master changed whether
    /// a pin of the cell drives has its drivers found again.
    pub(crate) fn master_replaced(&mut self, netlist: &Netlist<'_>, cell: CellId) {
        for &pin in &netlist.db.cells[cell].pins {
            let net = netlist.db.pins[pin].net;
            if self.drivers(net).contains(&pin) != netlist.is_driver(pin) {
                *self.net_drivers_mut(net) = netlist.drivers(net).collect();
            }
        }
    }

    /// The drivers of `net`, a net added since the graph was built included.
    fn net_drivers_mut(&mut self, net: NetId) -> &mut Vec<PinId> {
        self.net_drivers.grow_to(net.index() + 1, Vec::new());
        &mut self.net_drivers[net]
    }
}

#[cfg(test)]
mod tests;
