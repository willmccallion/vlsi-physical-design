use crate::db::design::NetlistDB;
use crate::testing::designs;
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

/// The design `db` written as DEF and read back.
fn round_trip(db: &NetlistDB) -> NetlistDB {
    let mut text = Vec::new();
    super::write(db, &mut text).unwrap();
    designs::parse(&String::from_utf8(text).unwrap())
}

fn config() -> Config {
    Config {
        cases: 64,
        rng_seed: RngSeed::Fixed(67),
        failure_persistence: None,
        ..Config::default()
    }
}

proptest! {
    #![proptest_config(config())]

    #[test]
    fn a_placed_ring_reads_back_as_it_was_written(
        starts in prop::collection::vec((0..59_000_i64, 0..58_000_i64), 1..200),
        blocks in prop::sample::subsequence(vec![(1000, 4000), (31_000, 4000), (1000, 30_000)], 0..=3),
    ) {
        let db = designs::parse(&designs::placed_ring_text(&starts, &blocks));

        prop_assert_eq!(round_trip(&db), db);
    }

    #[test]
    fn an_unplaced_ring_reads_back_as_it_was_written(gates in 1..200_usize) {
        let db = designs::parse(&designs::ring_text(gates));

        prop_assert_eq!(round_trip(&db), db);
    }
}
