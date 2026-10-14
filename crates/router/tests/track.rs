//! Track assignment on the shared fixture design.

#![allow(clippy::unwrap_used)]

use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{LayerId, NetId};
use pare_common::db::parser::{def, lef};
use pare_common::geom::dbu::Dbu;
use pare_common::util::config::RoutingConfig;
use pare_common::verify::shapes::wire_rect;
use pare_router::detailed::DetailedRouter;
use pare_router::global::{GCell, GlobalGrid, GlobalRoutes};
use pare_router::occupancy::Owner;
use pare_router::tech::RouteTech;
use pare_router::track::{TrackWire, assign_tracks};
use std::path::{Path, PathBuf};

const GCELL: Dbu = Dbu(2000);
const M3: LayerId = LayerId(2);

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

fn along_m3_row_0(from: u16, to: u16) -> Vec<GCell> {
    (from..=to).map(|x| GCell { layer: M3, x, y: 0 }).collect()
}

/// Assigns tracks to nets 0, 1, ... whose global routes are `paths`.
fn assign(db: &NetlistDB, paths: Vec<Vec<GCell>>) -> (RouteTech, Vec<TrackWire>) {
    let tech = RouteTech::new(db, &RoutingConfig::default()).unwrap();
    let dr = DetailedRouter::new(db, &tech, GCELL);
    let grid = GlobalGrid::new(&tech, &dr.occupancy, db.die_area, GCELL, 0.0);
    let routes = GlobalRoutes {
        paths: paths
            .into_iter()
            .enumerate()
            .map(|(n, p)| (NetId::new(n), vec![p]))
            .collect(),
    };
    let pins = vec![Vec::new(); db.nets.len()];
    let wires = assign_tracks(&tech, &dr.occupancy, &grid, &routes, &pins);
    (tech, wires)
}

#[test]
fn two_nets_along_one_panel_take_different_tracks() {
    let db = tiny();

    let (_, wires) = assign(&db, vec![along_m3_row_0(0, 1), along_m3_row_0(0, 1)]);

    assert_eq!(wires.len(), 2);
    assert_ne!(wires[0].track, wires[1].track);
    assert!(wires.iter().all(|w| w.layer == M3 && w.from < w.to));
}

#[test]
fn a_wire_is_cut_where_it_would_cross_an_obstacle() {
    let db = tiny();
    let tech = RouteTech::new(&db, &RoutingConfig::default()).unwrap();
    let occupancy = DetailedRouter::new(&db, &tech, GCELL).occupancy;

    let (tech, wires) = assign(&db, vec![along_m3_row_0(1, 2)]);

    assert!(!wires.is_empty());
    let lt = tech.layer(M3);
    let stripe_x = Dbu(5000);
    for w in &wires {
        let (a, b) = (lt.point(w.track, w.from), lt.point(w.track, w.to));
        let metal = wire_rect(a, b, lt.width).unwrap();
        let blocked = occupancy
            .rect_conflicts(&tech, M3, &metal, Owner::Net(0))
            .any(|(_, o)| o.owner == Owner::Obstacle);
        assert!(!blocked, "{w:?} crosses an obstacle");
        assert!(b.x < stripe_x, "{w:?} should stop before the stripe");
    }
    let gcell_1_centre = Dbu(3000);
    assert!(
        wires
            .iter()
            .any(|w| (lt.point(w.track, w.from).x - gcell_1_centre).abs() <= Dbu(100))
    );
}
