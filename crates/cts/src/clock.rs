//! A clock net as the tree builders see it, `TritonCTS`'s `Clock`: its sinks,
//! the buffers a builder adds and the subnets joining them.

use pare_common::db::indices::{CellId, NetId, PinId};
use pare_common::db::liberty::LibCellId;
use rustc_hash::FxHashMap;

/// A point in database units.
pub(crate) type DbuPoint = (i64, i64);

/// Whether a clock instance is a buffer the tree adds or a sink it reaches.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum InstKind {
    Buffer,
    Sink,
}

/// A buffer or sink of a clock tree.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ClockInst {
    pub name: String,
    pub master: Option<LibCellId>,
    pub kind: InstKind,
    pub location: DbuPoint,
    /// The sink's clock pin, or the buffer's input once it exists.
    pub input_pin: Option<PinId>,
    /// The buffer's instance once it exists.
    pub cell: Option<CellId>,
    pub input_cap: f32,
    /// Insertion delay as a wire length in database units.
    pub insertion_delay: f64,
    pub output_cap: f32,
    pub ideal_output_cap: f32,
}

/// Which clock instance: a sink or a buffer, by index.
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
    pub sinks: Vec<ClockInst>,
    pub buffers: Vec<ClockInst>,
    pub subnets: Vec<SubNet>,
    pub net: Option<NetId>,
    pub driver_pin: Option<PinId>,
    pub max_level: usize,
    by_name: FxHashMap<String, InstRef>,
}

impl Clock {
    pub fn new(net_name: &str, sdc_name: &str) -> Self {
        Self {
            net_name: escape_slashes(net_name),
            sdc_name: sdc_name.to_string(),
            sinks: Vec::new(),
            buffers: Vec::new(),
            subnets: Vec::new(),
            net: None,
            driver_pin: None,
            max_level: 0,
            by_name: FxHashMap::default(),
        }
    }

    pub fn add_sink(&mut self, sink: ClockSink) {
        self.sinks.push(ClockInst {
            name: sink.name,
            master: None,
            kind: InstKind::Sink,
            location: sink.location,
            input_pin: Some(sink.pin),
            cell: None,
            input_cap: sink.input_cap,
            insertion_delay: sink.insertion_delay,
            output_cap: 0.0,
            ideal_output_cap: 0.0,
        });
    }

    /// Adds a buffer named `name` plus the net's name, as
    /// `Clock::addClockBuffer`.
    pub fn add_buffer(&mut self, name: &str, master: LibCellId, location: DbuPoint) -> InstRef {
        let full = format!("{name}_{}", self.net_name);
        let inst = InstRef::Buffer(self.buffers.len());
        let _ = self.by_name.insert(full.clone(), inst);
        self.buffers.push(ClockInst {
            name: full,
            master: Some(master),
            kind: InstKind::Buffer,
            location,
            input_pin: None,
            cell: None,
            input_cap: 0.0,
            insertion_delay: 0.0,
            output_cap: 0.0,
            ideal_output_cap: 0.0,
        });
        inst
    }

    /// Adds a subnet named `name` plus the net's name.
    pub fn add_subnet(&mut self, name: &str) -> usize {
        self.subnets.push(SubNet {
            name: format!("{name}_{}", self.net_name),
            insts: Vec::new(),
            leaf: false,
            level: None,
            net: None,
        });
        self.subnets.len() - 1
    }

    pub fn inst(&self, inst: InstRef) -> &ClockInst {
        match inst {
            InstRef::Sink(i) => &self.sinks[i],
            InstRef::Buffer(i) => &self.buffers[i],
        }
    }

    pub fn inst_mut(&mut self, inst: InstRef) -> &mut ClockInst {
        match inst {
            InstRef::Sink(i) => &mut self.sinks[i],
            InstRef::Buffer(i) => &mut self.buffers[i],
        }
    }

    /// The box around the sinks with the outer hundredth on each side left
    /// out, as `Clock::computeSinkRegion`.
    pub fn sink_region(&self) -> (DbuPoint, DbuPoint) {
        let mut xs: Vec<i64> = self.sinks.iter().map(|s| s.location.0).collect();
        let mut ys: Vec<i64> = self.sinks.iter().map(|s| s.location.1).collect();
        xs.sort_unstable();
        ys.sort_unstable();
        let n = xs.len();
        let outliers = (0.01 * n as f64) as usize;
        (
            (xs[outliers], ys[outliers]),
            (xs[n - outliers - 1], ys[n - outliers - 1]),
        )
    }
}

/// A sink as the clock gathers it.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ClockSink {
    pub name: String,
    pub location: DbuPoint,
    pub pin: PinId,
    pub input_cap: f32,
    pub insertion_delay: f64,
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
