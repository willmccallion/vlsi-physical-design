use super::*;
use pare_common::db::design::{PlacementStatus, Region, RegionKind, Row};
use pare_common::db::indices::{LayerId, SiteId};
use pare_common::db::library::{
    CoreKind, Direction, LayerElectrical, LayerRef, Library, Macro, MacroClass, MacroPin,
    PinDirection, RoutingLayer, Shape, SignalUse, Site, SiteClass, Symmetry,
};
use pare_common::db::rules::RoutingLayerRules;
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::geom::orient::Orientation;
use pare_verify::placement::placement_violations;

const fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

fn improve_zoned(db: &mut NetlistDB) -> DetailedResult {
    let zones = Zones::new(db).unwrap();
    improve(db, &zones, 2, None)
}

/// One 20-site row over a 200 x 100 die, with 20-wide one-pin cells.
fn design(cells: &[(i64, PlacementStatus)], nets: &[(usize, usize)]) -> NetlistDB {
    let (lib, m) = one_pin_cell_library();
    let mut db = NetlistDB::new("t".into(), lib);
    db.die_area = Rect::new(pt(0, 0), pt(200, 100));
    db.rows.push(Row {
        name: "r".into(),
        site: SiteId(0),
        origin: pt(0, 0),
        orient: Orientation::N,
        num_x: 20,
        num_y: 1,
        step: pt(10, 0),
    });
    for (i, &(x, status)) in cells.iter().enumerate() {
        let _ = db
            .add_cell(format!("c{i}"), m, pt(x, 0), Orientation::N, status)
            .unwrap();
    }
    let pin = pare_common::db::indices::MacroPinId(0);
    for (k, &(a, b)) in nets.iter().enumerate() {
        let net = db.net_or_insert(&format!("n{k}"));
        let _ = db.connect_cell_pin(CellId::new(a), pin, net);
        let _ = db.connect_cell_pin(CellId::new(b), pin, net);
    }
    db
}

/// M1, a 10 by 100 site, and master C, 20 wide with one pin in its
/// middle.
fn one_pin_cell_library() -> (Library, MacroId) {
    let mut lib = Library::new(DbuPerMicron::new(1000).unwrap());
    let symmetry = Symmetry {
        x: true,
        y: true,
        r90: false,
    };
    let _ = lib
        .add_routing_layer(RoutingLayer {
            name: "M1".into(),
            direction: Direction::Horizontal,
            pitch_x: Dbu::new(10),
            pitch_y: Dbu::new(10),
            offset_x: None,
            offset_y: None,
            width: Dbu::new(4),
            min_spacing: Dbu::new(4),
            spacing_table: None,
            rules: RoutingLayerRules::default(),
            electrical: LayerElectrical::default(),
        })
        .unwrap();
    let site = lib
        .add_site(Site {
            name: "s".into(),
            class: SiteClass::Core,
            size: pt(10, 100),
            symmetry,
        })
        .unwrap();
    let mut m = Macro::new(
        "C".into(),
        MacroClass::Core(CoreKind::Regular),
        pt(20, 100),
        symmetry,
        Some(site),
    );
    let _ = m.add_pin(MacroPin {
        name: "A".into(),
        direction: PinDirection::Inout,
        signal_use: SignalUse::Signal,
        shapes: vec![Shape {
            layer: LayerRef::Routing(LayerId(0)),
            rect: Rect::new(pt(8, 48), pt(12, 52)),
        }],
    });
    let m = lib.add_macro(m).unwrap();
    (lib, m)
}

fn access_layer(name: &str, direction: Direction) -> RoutingLayer {
    RoutingLayer {
        name: name.into(),
        direction,
        pitch_x: Dbu::new(200),
        pitch_y: Dbu::new(200),
        offset_x: None,
        offset_y: None,
        width: Dbu::new(100),
        min_spacing: Dbu::new(100),
        spacing_table: None,
        rules: RoutingLayerRules::default(),
        electrical: LayerElectrical::default(),
    }
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(pt(x0, y0), pt(x1, y1))
}

/// Adds horizontal M1 and vertical M2 to `lib`, joined by a 0.1 cut
/// via with 0.1 by 0.18 pads on both.
fn add_m1_m2_and_via(lib: &mut Library) -> (LayerId, LayerId) {
    use pare_common::db::library::{CutLayer, ViaDef, ViaSource};
    use pare_common::db::rules::CutLayerRules;

    let m1 = lib
        .add_routing_layer(access_layer("M1", Direction::Horizontal))
        .unwrap();
    let v1 = lib
        .add_cut_layer(CutLayer {
            name: "V1".into(),
            width: Some(Dbu::new(100)),
            spacing: Some(Dbu::new(120)),
            rules: CutLayerRules::default(),
            resistance: None,
        })
        .unwrap();
    let m2 = lib
        .add_routing_layer(access_layer("M2", Direction::Vertical))
        .unwrap();
    let pad = |layer| Shape {
        layer,
        rect: rect(-50, -90, 50, 90),
    };
    let _ = lib
        .add_via(ViaDef {
            name: "V12".into(),
            cut: v1,
            shapes: vec![
                pad(LayerRef::Routing(m1)),
                Shape {
                    layer: LayerRef::Cut(v1),
                    rect: rect(-50, -50, 50, 50),
                },
                pad(LayerRef::Routing(m2)),
            ],
            source: ViaSource::LefDefault,
        })
        .unwrap();
    (m1, m2)
}

/// The library of [`access_design`], with its masters R and E.
fn access_library() -> (Library, MacroId, MacroId) {
    let mut lib = Library::new(DbuPerMicron::new(1000).unwrap());
    let (m1, m2) = add_m1_m2_and_via(&mut lib);
    let symmetry = Symmetry {
        x: true,
        y: true,
        r90: false,
    };
    let site = lib
        .add_site(Site {
            name: "s".into(),
            class: SiteClass::Core,
            size: pt(200, 2000),
            symmetry,
        })
        .unwrap();
    let mut master = |name: &str, pin_layer, pin: Rect<Dbu>| {
        let mut m = Macro::new(
            name.into(),
            MacroClass::Core(CoreKind::Regular),
            pt(400, 2000),
            symmetry,
            Some(site),
        );
        let _ = m.add_pin(MacroPin {
            name: "a".into(),
            direction: PinDirection::Input,
            signal_use: SignalUse::Signal,
            shapes: vec![Shape {
                layer: LayerRef::Routing(pin_layer),
                rect: pin,
            }],
        });
        lib.add_macro(m).unwrap()
    };
    let right_pin = master("R", m1, rect(250, 500, 350, 1500));
    let edge_pin = master("E", m2, rect(0, 500, 150, 1500));
    (lib, right_pin, edge_pin)
}

/// A 10 by 4 die with one row of 0.2-wide sites: horizontal M1 under
/// vertical M2 joined by a 0.1 cut via with 0.1 by 0.18 pads on both
/// layers; master R, 0.4 wide, with a 0.1-wide M1 pin ending 0.05 from
/// its right edge; master E, 0.4 wide, with an M2 pin along its left
/// edge. A via on R's pin breaks M2 spacing when E abuts R.
fn access_design() -> (NetlistDB, Zones, PinAccess) {
    use pare_common::db::design::{TrackAxis, TrackDef};
    use pare_common::db::indices::MacroPinId;

    let (lib, right_pin, edge_pin) = access_library();
    let mut db = NetlistDB::new("t".into(), lib);
    db.die_area = rect(0, 0, 10_000, 4000);
    for (axis, start, count, layer) in [(TrackAxis::X, 100, 50, 1), (TrackAxis::Y, 0, 21, 0)] {
        db.tracks.push(TrackDef {
            axis,
            start: Dbu::new(start),
            count,
            step: Dbu::new(200),
            layers: vec![LayerId(layer)],
        });
    }
    db.rows.push(Row {
        name: "r".into(),
        site: SiteId(0),
        origin: pt(0, 0),
        orient: Orientation::N,
        num_x: 50,
        num_y: 1,
        step: pt(200, 0),
    });
    for (name, master, x, net) in [("r", right_pin, 1000, "nr"), ("e", edge_pin, 1400, "ne")] {
        let cell = db
            .add_cell(
                name.into(),
                master,
                pt(x, 0),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        let net = db.net_or_insert(net);
        let _ = db.connect_cell_pin(cell, MacroPinId(0), net);
    }
    let zones = Zones::new(&db).unwrap();
    let access = PinAccess::new(&db, LayerId(1)).unwrap();
    (db, zones, access)
}

#[test]
fn a_cell_blocking_its_neighbours_pin_is_moved_a_site_away() {
    let (mut db, zones, access) = access_design();

    let repair = free_blocked_pins(&mut db, &zones, &access);

    assert_eq!(
        (repair.blocked_before, repair.blocked_after),
        (1, 0),
        "{repair:?}"
    );
    let gap = db.cells[CellId(1)].location.x - (db.cells[CellId(0)].location.x + Dbu::new(400));
    assert!(gap >= Dbu::new(200), "gap {gap:?}");
    assert!(
        placement_violations(&db).is_empty(),
        "{:?}",
        placement_violations(&db)
    );
}

#[test]
fn crossed_cells_are_uncrossed() {
    use PlacementStatus::{Fixed, Placed};
    let mut db = design(
        &[(0, Fixed), (180, Fixed), (40, Placed), (60, Placed)],
        &[(2, 1), (3, 0)],
    );

    let result = improve_zoned(&mut db);

    assert!(result.hpwl_after < result.hpwl_before, "{result:?}");
    assert!(db.cells[CellId(2)].location.x > db.cells[CellId(3)].location.x);
    assert!(
        placement_violations(&db).is_empty(),
        "{:?}",
        placement_violations(&db)
    );
}

#[test]
fn a_placement_with_nothing_to_gain_is_unchanged() {
    use PlacementStatus::{Fixed, Placed};
    let mut db = design(
        &[(0, Fixed), (180, Fixed), (20, Placed), (160, Placed)],
        &[(2, 0), (3, 1)],
    );
    let before: Vec<_> = db.cells.iter().map(|c| c.location).collect();

    let result = improve_zoned(&mut db);

    assert_eq!(result.moves, 0);
    assert_eq!(
        db.cells.iter().map(|c| c.location).collect::<Vec<_>>(),
        before
    );
}

#[test]
fn a_fence_member_moves_toward_its_net_only_up_to_the_fence_edge() {
    use PlacementStatus::{Fixed, Placed};
    let mut db = design(&[(0, Fixed), (180, Fixed), (40, Placed)], &[(2, 1)]);
    let fence = db.regions.push(Region {
        name: "f".into(),
        rects: vec![Rect::new(pt(0, 0), pt(100, 100))],
        kind: RegionKind::Fence,
        group_order: None,
    });
    db.cells[CellId(2)].region = Some(fence);

    let result = improve_zoned(&mut db);

    let x = db.cells[CellId(2)].location.x;
    assert!(result.moves > 0, "{result:?}");
    assert!(Dbu::new(40) < x && x <= Dbu::new(80), "{x:?}");
    assert!(
        placement_violations(&db).is_empty(),
        "{:?}",
        placement_violations(&db)
    );
}

#[test]
fn a_row_inside_comes_with_the_rows_below_and_above() {
    assert_eq!(rows_around(2, 5).collect::<Vec<_>>(), [2, 1, 3]);
}

#[test]
fn an_edge_row_comes_only_with_its_one_neighbour() {
    assert_eq!(rows_around(0, 5).collect::<Vec<_>>(), [0, 1]);
    assert_eq!(rows_around(4, 5).collect::<Vec<_>>(), [4, 3]);
}
