pub mod algo;
pub mod detailed_router;
pub mod global_router;
pub mod grid;
pub mod utils;

use eda_common::db::core::NetlistDB;
use eda_common::util::config::Config;

pub fn route(db: &mut NetlistDB, config: &Config) -> Result<(), String> {
    let (guides, coarse_converter) = global_router::run(db, &config.global_routing)?;

    detailed_router::run(db, &config.detailed_routing, &guides, &coarse_converter)?;

    Ok(())
}
