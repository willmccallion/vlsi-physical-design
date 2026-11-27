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

fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

#[test]
fn a_default_width_wire_reaches_half_its_width_past_each_end() {
    let width = Dbu::new(100);

    assert_eq!(
        wire_rect(pt(0, 0), pt(1000, 0), width),
        Some(rect(-50, -50, 1050, 50))
    );
    assert_eq!(
        wire_rect(pt(0, 0), pt(0, 1000), width),
        Some(rect(-50, -50, 50, 1050))
    );
    assert_eq!(wire_rect(pt(0, 0), pt(1000, 1000), width), None);
}

/// The tiny design with net `n3` under a rule of metal1 wires 200 wide
/// reaching `extension` past their ends, and metal2 wires 300 wide.
fn under_wide_rule(extension: Option<i64>) -> (NetlistDB, NetId) {
    let mut db = tiny();
    let layer = |name: &str| db.library.routing_layer_id(name).unwrap();
    let rule_layer =
        |layer, width, wire_extension: Option<i64>| crate::db::design::NonDefaultLayer {
            layer,
            width: Dbu::new(width),
            spacing: None,
            wire_extension: wire_extension.map(Dbu::new),
        };
    let rule = crate::db::design::NonDefaultRule {
        name: "wide".into(),
        hard_spacing: false,
        layers: vec![
            rule_layer(layer("M2"), 300, None),
            rule_layer(layer("M1"), 200, extension),
        ],
        vias: Vec::new(),
        via_rules: Vec::new(),
        min_cuts: Vec::new(),
    };
    let id = db.non_default_rules.push(rule);
    let n3 = db.net_id("n3").unwrap();
    db.nets[n3].non_default_rule = Some(id);
    (db, n3)
}

/// A metal1 wire of the tiny design from (2500, 1000) to (2700, 1000).
fn m1_wire(db: &NetlistDB, extensions: (Option<i64>, Option<i64>)) -> RouteElem {
    RouteElem::Wire {
        layer: db.library.routing_layer_id("M1").unwrap(),
        from: pt(2500, 1000),
        to: pt(2700, 1000),
        extensions: (extensions.0.map(Dbu::new), extensions.1.map(Dbu::new)),
    }
}

fn rects(shapes: &[Shape]) -> Vec<Rect<Dbu>> {
    shapes.iter().map(|s| s.rect).collect()
}

#[test]
fn a_wire_takes_its_nets_rule_for_its_layer() {
    let (db, n3) = under_wide_rule(None);

    let shapes = route_elem_shapes(&db, n3, &m1_wire(&db, (None, None)));

    assert_eq!(rects(&shapes), [rect(2400, 900, 2800, 1100)]);
}

#[test]
fn a_wire_reaches_its_rules_extension_unless_its_path_gives_one() {
    let (db, n3) = under_wide_rule(Some(30));

    let shapes = route_elem_shapes(&db, n3, &m1_wire(&db, (Some(0), None)));

    assert_eq!(rects(&shapes), [rect(2500, 900, 2730, 1100)]);
}

#[test]
fn default_width_shapes_read_every_wire_at_its_layers_width() {
    let (db, _) = under_wide_rule(Some(30));

    let shapes = default_width_shapes(&db, &m1_wire(&db, (None, None)));

    assert_eq!(rects(&shapes), [rect(2450, 950, 2750, 1050)]);
}

#[test]
fn a_via_and_a_patch_are_placed_where_the_route_puts_them() {
    let db = tiny();
    let m1 = db.library.routing_layer_id("M1").unwrap();
    let via = RouteElem::Via {
        via: db.library.via_id("M1M2").unwrap(),
        at: pt(2700, 1000),
    };
    let patch = RouteElem::Patch {
        layer: m1,
        rect: rect(0, 0, 10, 20),
    };

    let via_rects = rects(&default_width_shapes(&db, &via));

    assert_eq!(
        via_rects,
        [
            rect(2650, 950, 2750, 1050),
            rect(2650, 900, 2750, 1100),
            rect(2650, 900, 2750, 1100)
        ]
    );
    assert_eq!(
        rects(&default_width_shapes(&db, &patch)),
        [rect(0, 0, 10, 20)]
    );
}

#[test]
fn fixed_shapes_keep_the_io_pins_asked_for() {
    let db = tiny();
    let out = &db.io_pins[db.io_pin_id("out").unwrap()];
    let boxes = |shapes: Vec<OwnedShape>| -> Vec<Rect<Dbu>> {
        shapes.iter().map(|s| s.shape.rect).collect()
    };

    let all = boxes(fixed_shapes(&db));
    let without_out = boxes(fixed_shapes_keeping_io(&db, |p| p.name != "out"));

    let dropped: Vec<Rect<Dbu>> = all
        .iter()
        .filter(|r| !without_out.contains(r))
        .copied()
        .collect();
    let out_pin: Vec<Rect<Dbu>> = out.placed_shapes().map(|s| s.rect).collect();
    assert_eq!(dropped, out_pin);
    assert_eq!(all.len(), without_out.len() + out_pin.len());
}
