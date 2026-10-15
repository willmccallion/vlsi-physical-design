//! Placement: global placement by electrostatic spreading, I/O pin
//! placement on the die boundary, Abacus legalization onto rows and sites,
//! and detailed placement, all keeping cells to their fence regions.

pub mod detailed;
pub mod global;
pub mod io;
pub mod legalize;
pub mod model;
pub mod rows;
pub mod zones;
