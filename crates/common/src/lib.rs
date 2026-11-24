//! Shared design database, geometry, units and configuration for the PARE placer and router.

pub mod config;
pub mod convert;
pub mod db;
pub mod geom;
mod keyword_enum;
pub mod memory;
pub mod network_simplex;
mod newtype_ops;
#[cfg(feature = "testing")]
pub mod testing;
pub mod units;
