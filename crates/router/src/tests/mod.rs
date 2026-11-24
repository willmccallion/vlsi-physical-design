//! Routing and its parts on the shared fixture designs.

mod access;
mod golden;
mod macro_edge;
mod route;
mod track;

use pare_common::config::RoutingConfig;
use pare_common::db::design::{NetlistDB, PlacementStatus};
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use std::num::NonZeroU32;
use std::path::{Path, PathBuf};

/// A file of the common crate's test fixtures.
pub(crate) fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

/// The design of the fixture files `name`.lef and `name`.def.
fn fixture_design(name: &str) -> NetlistDB {
    let def_file = fixture(&format!("{name}.def"));
    let units = def::read_units(&def_file).unwrap();
    let library = lef::parse_files(&[fixture(&format!("{name}.lef"))], units).unwrap();
    def::parse(&def_file, library).unwrap()
}

/// The shared tiny fixture design, two of its cells unplaced.
pub(crate) fn tiny() -> NetlistDB {
    fixture_design("tiny")
}

/// A hard block whose left edge holds eight small metal3 pins between
/// obstruction strips, as memory macros have, placed so that a metal4
/// track crosses every pin, and one net joining the pins to an I/O pin.
pub(crate) fn macro_edge() -> NetlistDB {
    fixture_design("macro_edge")
}

/// Places the cell `name` of `db` at `at`.
pub(crate) fn place(db: &mut NetlistDB, name: &str, at: Point<Dbu>) {
    let cell = db.cells.iter_mut().find(|c| c.name == name).unwrap();
    cell.location = at;
    cell.status = PlacementStatus::Placed;
}

/// Detailed rounds the placed fixture is routed for: enough to pass
/// through the regular, guides, stubborn and skipped rounds, since some of
/// its violations never clear.
pub(crate) const TINY_ROUNDS: usize = 12;

/// The fixture with its two unplaced cells placed, and a config that cuts
/// it into many gcells.
pub(crate) fn placed_tiny() -> (NetlistDB, RoutingConfig) {
    let mut db = tiny();
    place(&mut db, "u5", Point::new(Dbu::new(2000), Dbu::new(4000)));
    place(&mut db, "u6", Point::new(Dbu::new(3000), Dbu::new(4000)));
    let config = RoutingConfig {
        gcell_tracks: NonZeroU32::new(3).unwrap(),
        detailed_iterations: TINY_ROUNDS,
        ..RoutingConfig::default()
    };
    (db, config)
}
