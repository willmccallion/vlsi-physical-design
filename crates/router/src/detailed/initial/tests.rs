use super::*;
use crate::access::PinKind;
use crate::detailed::judge::Candidate;
use crate::occupancy::{Origin, Owner};
use crate::tech::RouteSetup;
use crate::tech::RouteTech;
use crate::wiring::Wiring;
use pare_common::config::RoutingConfig;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::LayerId;

fn m2(x: u16, y: u16) -> GCell {
    GCell {
        layer: LayerId(1),
        x,
        y,
    }
}

#[test]
fn a_bridge_every_track_of_which_fixed_metal_blocks_stays_nearest_the_middle() {
    let (db, config) = crate::tests::placed_tiny();
    let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
    let gcell = crate::gcell_size(&tech, &config);
    let net = NetId::new(0);
    let clear = DetailedRouter::new(&db, &tech, gcell);
    let mut walled = DetailedRouter::new(&db, &tech, gcell);
    let boundary = db.die_area.min.y + gcell * 2;
    let wall = Rect::new(
        Point::new(db.die_area.min.x, boundary - gcell / 2),
        Point::new(db.die_area.max.x, boundary + gcell / 2),
    );
    walled
        .occupancy
        .add_rect(&tech, LayerId(1), &wall, Owner::Obstacle, Origin::Fixed);

    let free = clear.bridge(net, m2(1, 1), m2(1, 2));
    let blocked = walled.bridge(net, m2(1, 1), m2(1, 2));

    assert!(free.is_some_and(|e| !clear.blocked_by_fixed(net, e)));
    assert!(blocked.is_some_and(|e| walled.blocked_by_fixed(net, e)));
    assert_eq!(blocked, free);
}

/// The macro edge fixture with its block moved right, so that no
/// crossing falls inside its pins: a stub from the crossing before each
/// pin reaches it.
fn macro_edge_reached_by_stubs() -> NetlistDB {
    let mut db = crate::tests::macro_edge();
    let ram = db.cell_id("ram").unwrap();
    db.cells[ram].location.x = Dbu::new(11670);
    db
}

#[test]
fn the_first_pass_judges_wiring_with_its_nets_access_stubs_in_place() {
    let db = macro_edge_reached_by_stubs();
    let config = RoutingConfig::default();
    let tech = RouteTech::new(&db, &RouteSetup::from(&config)).unwrap();
    let mut dr = DetailedRouter::new(&db, &tech, crate::gcell_size(&tech, &config));
    let net = NetId::new(0);
    let access: Vec<AccessPoint> = dr.access[net].iter().map(|c| c[0].clone()).collect();
    let block_pins = db.nets[net]
        .pins
        .iter()
        .zip(&access)
        .filter(|&(&pin, _)| PinKind::of(&db, pin) == PinKind::Block);
    let to_pins = Wiring {
        edges: block_pins
            .flat_map(|(_, a)| {
                (1..=2).map(|back| {
                    Edge::Wire(Node {
                        cross: CrossIdx::new(a.node.cross.index() - back),
                        ..a.node
                    })
                })
            })
            .collect(),
        patches: Vec::new(),
    };
    let plan = Plan {
        net,
        access,
        bridges: Vec::new(),
        pieces: Vec::new(),
    };
    let grid = RegionGrid::of_pass(&dr, 0);

    dr.commit_skeletons(&[plan], &grid);
    let candidate = Candidate {
        net,
        without: &Wiring::default(),
        with: &to_pins,
    };
    let judgment = dr.judge(&db.die_area, &[candidate]);

    assert!(!to_pins.edges.is_empty());
    assert_eq!(judgment.findings.violations, []);
}
