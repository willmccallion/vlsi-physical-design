//! Placement: global placement by electrostatic spreading, I/O pin
//! placement on the die boundary, Abacus legalization onto rows and sites,
//! and detailed placement, all keeping cells to their fence regions.

#![deny(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::cast_lossless
)]

pub mod detailed;
pub mod global;
pub mod io;
pub mod legalize;
pub mod model;
mod munkres;
pub mod rows;
pub mod zones;
