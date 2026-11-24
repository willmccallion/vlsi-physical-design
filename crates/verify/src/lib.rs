//! Independent checks of placement and routing results.

#![deny(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::cast_lossless
)]

pub mod drc;
pub mod pin_access;
pub mod placement;
pub mod routing;
