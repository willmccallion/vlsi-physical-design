//! Designs the CLI's tests share.

use pare_common::db::design::NetlistDB;
use pare_common::db::parser::{def, lef};
use std::path::PathBuf;

/// The shared tiny fixture design.
pub(crate) fn tiny() -> NetlistDB {
    let fixture = |name: &str| {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../common/tests/fixtures")
            .join(name)
    };
    let units = def::read_units(&fixture("tiny.def")).unwrap();
    let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
    def::parse(&fixture("tiny.def"), library).unwrap()
}
