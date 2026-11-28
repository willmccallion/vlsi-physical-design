//! The designs the placer's integration tests share.

use pare_common::db::design::NetlistDB;
pub use pare_common::testing::designs::{parse, ring_text};

/// The design of [`ring_text`].
#[allow(dead_code)] // not every test file places the plain ring
pub fn ring(gates: usize) -> NetlistDB {
    parse(&ring_text(gates))
}
