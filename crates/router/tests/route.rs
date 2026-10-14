//! Routing entry checks on the shared fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{def, lef};
use pare_common::util::config::RoutingConfig;
use pare_router::{RouteError, route};
use std::path::{Path, PathBuf};

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

fn tiny() -> NetlistDB {
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse(&fixture("tiny.def"), library).unwrap()
}

#[test]
fn routing_refuses_a_design_with_unplaced_cells() {
    let mut db = tiny();

    let result = route(&mut db, &RoutingConfig::default());

    assert_eq!(
        result,
        Err(RouteError::UnplacedCells {
            count: 2,
            first: "u5".to_string(),
        })
    );
}
