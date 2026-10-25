//! Netlist repair as `OpenROAD`'s resizer does it before and after placement:
//! stripping the buffers synthesis left, buffering the ports, and once the
//! cells have positions, sizing drivers and inserting repeaters until every
//! pin's slew and every driver's load are within the library's limits.

pub mod buffers;
pub mod library;
pub mod netlist;

pub use buffers::{BufferError, PortBuffers, buffer_ports, remove_buffers};
pub use library::{BufferSet, Link, LinkError};
pub use netlist::{ClockNetwork, Constants, Namer, Netlist, NetlistError};
