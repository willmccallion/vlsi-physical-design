//! Shared design database, geometry, units and configuration for the PARE placer and router.

pub mod config;
pub mod db;
pub mod geom;
pub mod memory;
pub mod network_simplex;
#[cfg(feature = "testing")]
pub mod testing;
pub mod units;
pub mod verify;
