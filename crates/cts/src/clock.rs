//! A clock net as the tree builders see it, `TritonCTS`'s `Clock`: its sinks,
//! the buffers a builder adds and the subnets joining them.

use crate::capacitance::Farads32;
use pare_common::convert::SaturatingFrom;
use pare_common::db::indices::{CellId, NetId, PinId};
use pare_common::db::liberty::BufferCell;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::units::Seconds;

/// Whether a clock instance is a buffer the tree adds or a sink it reaches.
/// A buffer the tree adds.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct ClockBuffer {
    pub name: String,
    pub master: BufferCell,
    pub location: Point<Dbu>,
    /// The buffer's instance once it exists.
    pub cell: Option<CellId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub(crate) enum InstRef {
    Sink(usize),
    Buffer(usize),
}

/// A net of the tree: its driver first, then its sinks.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct SubNet {
    pub name: String,
    pub insts: Vec<InstRef>,
    pub leaf: bool,
    pub level: Option<u32>,
    pub net: Option<NetId>,
}

impl SubNet {
    pub fn driver(&self) -> InstRef {
        self.insts[0]
    }

    pub fn sinks(&self) -> &[InstRef] {
        self.insts.get(1..).unwrap_or(&[])
    }

    pub fn add(&mut self, inst: InstRef) {
        self.insts.push(inst);
    }
}

/// A clock net and the tree built for it.
#[derive(Clone, Debug)]
pub(crate) struct Clock {
    pub net_name: String,
    pub sdc_name: String,
    pub sinks: Vec<ClockSink>,
    pub buffers: Vec<ClockBuffer>,
    pub subnets: Vec<SubNet>,
    pub net: NetId,
}

impl Clock {
    pub fn new(net: NetId, net_name: &str, sdc_name: &str) -> Self {
        Self {
            net_name: escape_slashes(net_name),
            sdc_name: sdc_name.to_string(),
            sinks: Vec::new(),
            buffers: Vec::new(),
            subnets: Vec::new(),
            net,
        }
    }

    pub(crate) fn add_sink(&mut self, sink: ClockSink) {
        self.sinks.push(sink);
    }

    /// Adds a buffer named `name` plus the net's name, as
    /// `Clock::addClockBuffer`.
    pub(crate) fn add_buffer(
        &mut self,
        name: &str,
        master: BufferCell,
        location: Point<Dbu>,
    ) -> InstRef {
        self.push_buffer(name, master, location, None)
    }

    /// Adds a buffer as [`Clock::add_buffer`] does, its instance `cell`
    /// already in the design.
    pub(crate) fn add_written_buffer(
        &mut self,
        name: &str,
        master: BufferCell,
        location: Point<Dbu>,
        cell: CellId,
    ) -> InstRef {
        self.push_buffer(name, master, location, Some(cell))
    }

    fn push_buffer(
        &mut self,
        name: &str,
        master: BufferCell,
        location: Point<Dbu>,
        cell: Option<CellId>,
    ) -> InstRef {
        let inst = InstRef::Buffer(self.buffers.len());
        self.buffers.push(ClockBuffer {
            name: format!("{name}_{}", self.net_name),
            master,
            location,
            cell,
        });
        inst
    }

    /// Adds a subnet named `name` plus the net's name.
    pub(crate) fn add_subnet(&mut self, name: &str) -> usize {
        self.subnets.push(SubNet {
            name: format!("{name}_{}", self.net_name),
            insts: Vec::new(),
            leaf: false,
            level: None,
            net: None,
        });
        self.subnets.len() - 1
    }

    /// The name of a sink or buffer.
    pub(crate) fn name(&self, inst: InstRef) -> &str {
        match inst {
            InstRef::Sink(i) => &self.sinks[i].name,
            InstRef::Buffer(i) => &self.buffers[i].name,
        }
    }

    /// The box around the sinks with the outer hundredth on each side left
    /// out, as `Clock::computeSinkRegion`.
    pub(crate) fn sink_region(&self) -> Rect<Dbu> {
        let mut xs: Vec<Dbu> = self.sinks.iter().map(|s| s.location.x).collect();
        let mut ys: Vec<Dbu> = self.sinks.iter().map(|s| s.location.y).collect();
        xs.sort_unstable();
        ys.sort_unstable();
        let n = xs.len();
        let outliers = usize::saturating_from(0.01 * n as f64);
        Rect::new(
            Point::new(xs[outliers], ys[outliers]),
            Point::new(xs[n - outliers - 1], ys[n - outliers - 1]),
        )
    }
}

/// A sink as the clock gathers it.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ClockSink {
    pub name: String,
    pub location: Point<Dbu>,
    pub pin: PinId,
    pub input_cap: Farads32,
    /// The delay through the sink's own clock tree.
    pub insertion_delay: Seconds,
}

/// A name with each `/` escaped by a backslash, as `OpenSTA`'s
/// `escapeChars` for a net name used as an instance name suffix.
fn escape_slashes(name: &str) -> String {
    let mut out = String::with_capacity(name.len());
    for c in name.chars() {
        if c == '/' {
            out.push('\\');
        }
        out.push(c);
    }
    out
}

#[cfg(test)]
mod tests;
