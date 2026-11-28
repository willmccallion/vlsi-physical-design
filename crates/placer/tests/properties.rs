//! What the placer keeps, whatever the placement it starts from: each
//! legalizer leaves no cell overlapping another, off its row or off the site
//! grid, by the independent checker of the verify crate, and moves no fixed
//! block; and the improvers keep a legal placement legal without
//! lengthening its wire.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::testing::designs;
use pare_placer::legalize::abacus;
use pare_placer::legalize::opendp::detailed_placement;
use pare_placer::legalize::opendp::improve::{MaxDisplacement, improve_placement};
use pare_placer::legalize::opendp::mirror::{hpwl, optimize_mirroring};
use pare_placer::zones::Zones;
use pare_verify::placement::placement_violations;
use proptest::prelude::*;
use proptest::test_runner::{Config, RngSeed};

/// Corners a fixed RAM block may take: on the site grid and the rows, far
/// enough apart that no two blocks overlap.
const BLOCK_SLOTS: [(i64, i64); 12] = [
    (1000, 4000),
    (11_000, 4000),
    (21_000, 4000),
    (31_000, 4000),
    (41_000, 4000),
    (51_000, 4000),
    (1000, 30_000),
    (11_000, 30_000),
    (21_000, 30_000),
    (31_000, 30_000),
    (41_000, 30_000),
    (51_000, 30_000),
];

/// Where the ring's gates start, and the corners of the fixed RAM blocks.
#[derive(Clone, Debug)]
struct Placement {
    starts: Vec<(i64, i64)>,
    blocks: Vec<(i64, i64)>,
}

impl Placement {
    fn design(&self) -> NetlistDB {
        designs::parse(&designs::placed_ring_text(&self.starts, &self.blocks))
    }
}

/// Twenty to six hundred gates of the ring anywhere in the core, off the
/// site grid as global placement leaves them, beside up to four fixed RAM
/// blocks: under a fifth of the core's sites, so legalization always has room.
fn placement() -> impl Strategy<Value = Placement> {
    let starts = prop::collection::vec((0..59_000_i64, 0..58_000_i64), 20..600);
    let blocks = prop::sample::subsequence(BLOCK_SLOTS.to_vec(), 0..=4);
    (starts, blocks).prop_map(|(starts, blocks)| Placement { starts, blocks })
}

/// The design of `placement`, made legal by `OpenDP`'s detailed placement.
fn legal(placement: &Placement) -> NetlistDB {
    let mut db = placement.design();
    let _ = detailed_placement(&mut db).unwrap();
    db
}

/// Each cell's lower-left corner and orientation.
fn spots(db: &NetlistDB) -> Vec<(Point<Dbu>, Orientation)> {
    db.cells.iter().map(|c| (c.location, c.orient)).collect()
}

fn assert_blocks_stay(db: &NetlistDB, blocks: &[(i64, i64)]) -> Result<(), TestCaseError> {
    for (b, &(x, y)) in blocks.iter().enumerate() {
        let block = &db.cells[db.cell_id(&format!("ram{b}")).unwrap()];
        prop_assert_eq!((block.location.x.get(), block.location.y.get()), (x, y));
    }
    Ok(())
}

fn config(cases: u32, seed: u64) -> Config {
    Config {
        cases,
        rng_seed: RngSeed::Fixed(seed),
        failure_persistence: None,
        ..Config::default()
    }
}

proptest! {
    #![proptest_config(config(48, 43))]

    #[test]
    fn legalizing_any_placement_leaves_it_legal_and_its_blocks_where_they_were(
        placement in placement(),
    ) {
        let db = legal(&placement);

        prop_assert_eq!(placement_violations(&db), Vec::new());
        assert_blocks_stay(&db, &placement.blocks)?;
    }

    #[test]
    fn abacus_leaves_any_placement_legal_and_its_blocks_where_they_were(
        placement in placement(),
    ) {
        let mut db = placement.design();
        let zones = Zones::new(&db).unwrap();

        abacus::legalize(&mut db, &zones).unwrap();

        prop_assert_eq!(placement_violations(&db), Vec::new());
        assert_blocks_stay(&db, &placement.blocks)?;
    }
}

proptest! {
    #![proptest_config(config(24, 53))]

    #[test]
    fn improving_a_legal_placement_keeps_it_legal_and_never_lengthens_it(
        placement in placement(),
    ) {
        let mut db = legal(&placement);
        let unlimited = MaxDisplacement { x: None, y: None };

        let improvement = improve_placement(&mut db, unlimited).unwrap();

        prop_assert_eq!(placement_violations(&db), Vec::new());
        assert_blocks_stay(&db, &placement.blocks)?;
        prop_assert!(improvement.hpwl_after <= improvement.hpwl_before, "{improvement:?}");
    }

    #[test]
    fn mirroring_a_legal_placement_moves_no_cell_and_never_lengthens_it(
        placement in placement(),
    ) {
        let mut db = legal(&placement);
        let (before, hpwl_before) = (spots(&db), hpwl(&db));

        let _ = optimize_mirroring(&mut db);

        prop_assert_eq!(placement_violations(&db), Vec::new());
        for (cell, (at, _)) in db.cells.iter().zip(before) {
            prop_assert_eq!(cell.location, at, "{} moved", cell.name);
        }
        prop_assert!(hpwl(&db) <= hpwl_before, "{:?} after {:?}", hpwl(&db), hpwl_before);
    }
}
