use super::*;
use crate::db::design::SpecialRect;
use crate::db::indices::LayerId;

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::from_corners(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

fn wire(from: (i64, i64), to: (i64, i64), shape: Option<&str>) -> SpecialWire {
    SpecialWire {
        layer: LayerId(0),
        width: Dbu::new(100),
        from: Point::new(Dbu::new(from.0), Dbu::new(from.1)),
        to: Point::new(Dbu::new(to.0), Dbu::new(to.1)),
        extensions: (Dbu::ZERO, Dbu::ZERO),
        shape: shape.map(str::to_string),
    }
}

#[test]
fn a_wire_end_extends_by_its_own_extension_whichever_way_the_wire_runs() {
    let at = |x, y| Point::new(Dbu::new(x), Dbu::new(y));

    let forward = wire_rect_with_ends(
        at(0, 0),
        at(1000, 0),
        Dbu::new(100),
        Dbu::new(0),
        Dbu::new(30),
    );
    let backward = wire_rect_with_ends(
        at(1000, 0),
        at(0, 0),
        Dbu::new(100),
        Dbu::new(30),
        Dbu::new(0),
    );

    assert_eq!(forward, Some(rect(0, -50, 1030, 50)));
    assert_eq!(backward, forward);
}

#[test]
fn a_shaped_special_wire_keeps_its_box() {
    let w = wire((0, 500), (1000, 500), Some("STRIPE"));
    assert_eq!(special_wire_box(&w), rect(0, 450, 1000, 550));
}

#[test]
fn an_unshaped_special_wire_grows_by_half_its_width_at_both_ends() {
    assert_eq!(
        special_wire_box(&wire((0, 500), (1000, 500), None)),
        rect(-50, 450, 1050, 550)
    );
    assert_eq!(
        special_wire_box(&wire((500, 0), (500, 1000), None)),
        rect(450, -50, 550, 1050)
    );
}

#[test]
fn a_special_rect_with_one_even_side_grows_along_the_other() {
    assert_eq!(
        special_rect_box(&rect(0, 0, 900, 45)),
        rect(0, -450, 900, 495)
    );
    assert_eq!(
        special_rect_box(&rect(0, 0, 45, 900)),
        rect(-450, 0, 495, 900)
    );
}

#[test]
fn a_special_rect_even_both_ways_takes_the_shorter_side_as_its_width() {
    assert_eq!(
        special_rect_box(&rect(0, 0, 400, 100)),
        rect(-50, 0, 450, 100)
    );
    assert_eq!(
        special_rect_box(&rect(0, 0, 100, 400)),
        rect(0, -50, 100, 450)
    );
    assert_eq!(
        special_rect_box(&rect(0, 0, 200, 200)),
        rect(0, -100, 200, 300)
    );
}

#[test]
fn a_special_rect_odd_both_ways_stays_as_written() {
    assert_eq!(special_rect_box(&rect(0, 0, 45, 45)), rect(0, 0, 45, 45));
}

#[test]
fn only_special_rects_without_a_shape_are_stretched() {
    let net = SpecialNet {
        name: "VDD".into(),
        signal_use: crate::db::library::SignalUse::Power,
        connections: Vec::new(),
        wires: Vec::new(),
        rects: vec![
            SpecialRect {
                layer: LayerRef::Routing(LayerId(0)),
                rect: rect(0, 0, 900, 45),
                shape: Some("STRIPE".into()),
            },
            SpecialRect {
                layer: LayerRef::Routing(LayerId(0)),
                rect: rect(0, 0, 900, 45),
                shape: None,
            },
        ],
        vias: Vec::new(),
    };

    let boxes: Vec<Rect<Dbu>> = special_net_metal(&net).map(|s| s.rect).collect();

    assert_eq!(boxes, [rect(0, 0, 900, 45), rect(0, -450, 900, 495)]);
}

fn tiny() -> NetlistDB {
    let fixtures = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let units = crate::db::parser::def::read_units(&fixtures.join("tiny.def")).unwrap();
    let library = crate::db::parser::lef::parse_files(&[fixtures.join("tiny.lef")], units).unwrap();
    crate::db::parser::def::parse(&fixtures.join("tiny.def"), library).unwrap()
}

#[test]
fn the_router_connects_signal_nets_of_two_pins_or_more() {
    let mut db = tiny();
    let (n1, out) = (db.net_id("n1").unwrap(), db.net_id("out").unwrap());

    assert!(is_signal_net(&db, n1));
    db.nets[n1].signal_use = crate::db::library::SignalUse::Power;
    assert!(!is_signal_net(&db, n1));
    let _ = db.nets[out].pins.pop();
    assert!(!is_signal_net(&db, out));
}

#[test]
fn placement_avoids_the_shapes_of_fixed_pins_but_not_of_placed_ones() {
    let db = tiny();
    let (fixed, placed) = (db.net_id("in").unwrap(), db.net_id("out").unwrap());

    let owners: Vec<Owner> = obstruction_shapes(&db).iter().map(|s| s.owner).collect();

    assert!(owners.contains(&Owner::Net(fixed)));
    assert!(!owners.contains(&Owner::Net(placed)));
}
