//! Netlist repair as `OpenROAD`'s resizer does it before and after placement:
//! stripping the buffers synthesis left, buffering the ports, and once the
//! cells have positions, sizing drivers and inserting repeaters until every
//! pin's slew and every driver's load are within the library's limits.

pub mod bnet;
pub mod buffers;
pub mod dmp;
pub mod fuzzy;
pub mod graph;
pub mod library;
pub mod netlist;
pub mod parasitics;
pub mod repair;
pub mod resizer;
pub mod sta;
pub mod steiner;

pub use buffers::{BufferError, PortBuffers, buffer_ports, remove_buffers};
pub use library::{BufferSet, Link, LinkError};
pub use netlist::{ClockNetwork, Constants, Namer, Netlist, NetlistError};
pub use repair::{RepairSummary, Repairer};
pub use resizer::{Context, RepairError, RepairOptions, Resizer};
pub use sta::{Path, PathPoint, Sta, Tag, Via};
