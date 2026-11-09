//! Netlist repair as `OpenROAD`'s resizer does it before and after placement:
//! stripping the buffers synthesis left, buffering the ports, and once the
//! cells have positions, sizing drivers and inserting repeaters until every
//! pin's slew and every driver's load are within the library's limits.

pub mod bnet;
mod buffers;
mod clock_inverters;
pub mod dmp;
pub mod fuzzy;
mod gated;
mod graph;
mod hold;
mod library;
mod moves;
mod netlist;
mod parasitics;
mod rebuffer;
mod repair;
mod resizer;
mod route_parasitics;
mod setup;
mod sta;
pub mod steiner;
#[cfg(test)]
mod testing;

pub use buffers::{BufferError, PortBuffers, buffer_ports, remove_buffers};
pub use clock_inverters::{ClockInverterRepair, repair_clock_inverters};
pub use hold::{HoldOptions, HoldSummary, repair_hold};
pub use library::{BufferSet, Link, LinkError};
pub use netlist::{ClockNetwork, Constants, Namer, Netlist, NetlistError};
pub use parasitics::{NetParasitics, WireRc};
pub use rebuffer::{Rebuffer, max_wire_length, max_wire_length_of};
pub use repair::{RepairSummary, Repairer};
pub use resizer::{
    Context, GlobalRouting, RepairError, Resizer, SessionInputs, Timing, Untimed,
    area_change_percent, timing_net_weights,
};
pub use route_parasitics::RouteRc;
pub use setup::{SetupOptions, SetupProgress, SetupSummary, repair_setup};
pub use sta::{Path, PathPoint, Slack, Sta, Tag, Via};
