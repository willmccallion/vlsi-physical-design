//! Shared design database, geometry and utilities for the PARE placer and router.

pub mod db;
pub mod geom;
#[cfg(feature = "testing")]
pub mod testing;
pub mod units;
pub mod util;
pub mod verify;
