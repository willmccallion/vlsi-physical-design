use super::*;
use pare_common::db::design::{PlacementStatus, TrackDef};
use pare_common::db::indices::CutLayerId;
use pare_common::db::library::{
    CoreKind, Macro, MacroClass, MacroPin, PinDirection, SignalUse, Symmetry,
};
use pare_common::db::parser::lef;
use pare_common::geom::dbu::DbuPerMicron;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::path::Path;

const M1: LayerId = LayerId(0);
const M2: LayerId = LayerId(1);
const M3: LayerId = LayerId(2);

const fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(pt(x0, y0), pt(x1, y1))
}

const fn on(layer: LayerRef, rect: Rect<Dbu>) -> Shape {
    Shape { layer, rect }
}

fn design() -> NetlistDB {
    let units = DbuPerMicron::new(1000).unwrap();
    let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("../common/tests/fixtures/tiny.lef");
    let library = lef::parse_files(&[fixture], units).unwrap();
    let mut db = NetlistDB::new("t".into(), library);
    db.die_area = rect(0, 0, 10_000, 10_000);
    db
}

const fn wire(layer: LayerId, from: Point<Dbu>, to: Point<Dbu>) -> RouteElem {
    RouteElem::Wire {
        layer,
        from,
        to,
        extensions: (None, None),
    }
}

fn stats_of(wires: Vec<RouteElem>, tracks: Vec<TrackDef>) -> WiringStats {
    let mut db = design();
    db.tracks = tracks;
    let net = db.net_or_insert("n");
    db.nets[net].routing = wires;
    wiring_stats(&db)
}

fn tracks(axis: TrackAxis, start: i64, layers: &[LayerId]) -> TrackDef {
    TrackDef {
        axis,
        start: Dbu::new(start),
        count: 20,
        step: Dbu::new(200),
        layers: layers.to_vec(),
    }
}

/// Horizontal M1 tracks at y = 100 + 200k and M1-unrelated vertical ones at
/// x = 200k, which a wire's own coordinate must not be confused with.
fn m1_tracks() -> Vec<TrackDef> {
    vec![
        tracks(TrackAxis::Y, 100, &[M1]),
        tracks(TrackAxis::X, 0, &[M1]),
    ]
}

#[test]
fn a_report_with_an_open_net_is_not_connected() {
    let report = RoutingReport {
        opens: vec![Open {
            net: NetId::new(0),
            components: 2,
        }],
        stats: WiringStats::default(),
    };

    assert!(!report.is_connected());
}

#[test]
fn cuts_of_one_layer_that_share_an_edge_connect() {
    let cut = LayerRef::Cut(CutLayerId(0));
    let a = on(cut, rect(0, 0, 100, 100));
    let b = on(cut, rect(100, 0, 200, 100));

    assert!(shapes_connect(&a, &b));
}

#[test]
fn cuts_of_different_layers_never_connect() {
    let a = on(LayerRef::Cut(CutLayerId(0)), rect(0, 0, 100, 100));
    let b = on(LayerRef::Cut(CutLayerId(1)), rect(0, 0, 100, 100));

    assert!(!shapes_connect(&a, &b));
}

#[test]
fn routing_shapes_of_different_layers_never_connect() {
    let a = on(LayerRef::Routing(M1), rect(0, 0, 100, 100));
    let b = on(LayerRef::Routing(M2), rect(0, 0, 100, 100));

    assert!(!shapes_connect(&a, &b));
}

#[test]
fn a_cut_connects_to_overlapping_metal_above_and_below() {
    let cut = on(LayerRef::Cut(CutLayerId(1)), rect(0, 0, 100, 100));

    assert!(shapes_connect(
        &cut,
        &on(LayerRef::Routing(M2), rect(50, 50, 150, 150))
    ));
    assert!(shapes_connect(
        &on(LayerRef::Routing(M3), rect(50, 50, 150, 150)),
        &cut
    ));
}

#[test]
fn a_cut_does_not_connect_to_metal_it_does_not_overlap() {
    let cut = on(LayerRef::Cut(CutLayerId(1)), rect(0, 0, 100, 100));

    assert!(!shapes_connect(
        &cut,
        &on(LayerRef::Routing(M2), rect(200, 0, 300, 100))
    ));
}

#[test]
fn a_cut_does_not_connect_to_metal_two_layers_away() {
    let cut = on(LayerRef::Cut(CutLayerId(1)), rect(0, 0, 100, 100));

    assert!(!shapes_connect(
        &cut,
        &on(LayerRef::Routing(M1), rect(0, 0, 100, 100))
    ));
}

#[test]
fn shapes_sharing_a_horizontal_edge_share_a_side() {
    let a = on(LayerRef::Routing(M1), rect(0, 0, 100, 100));
    let b = on(LayerRef::Routing(M1), rect(0, 100, 100, 200));

    assert!(shares_a_side(&a, &b));
}

#[test]
fn shapes_sharing_a_vertical_edge_share_a_side() {
    let a = on(LayerRef::Routing(M1), rect(0, 0, 100, 100));
    let b = on(LayerRef::Routing(M1), rect(100, 0, 200, 100));

    assert!(shares_a_side(&a, &b));
}

#[test]
fn shapes_meeting_at_a_point_share_no_side() {
    let a = on(LayerRef::Routing(M1), rect(0, 0, 100, 100));
    let b = on(LayerRef::Routing(M1), rect(100, 100, 200, 200));

    assert!(!shares_a_side(&a, &b));
}

#[test]
fn a_pin_without_shapes_is_a_group_of_its_own() {
    let mut db = design();
    let mut shapeless = Macro::new(
        "SHAPELESS".into(),
        MacroClass::Core(CoreKind::Regular),
        pt(400, 2000),
        Symmetry::default(),
        None,
    );
    let _ = shapeless.add_pin(MacroPin {
        name: "p".into(),
        direction: PinDirection::Input,
        signal_use: SignalUse::Signal,
        shapes: Vec::new(),
    });
    let shapeless = db.library.add_macro(shapeless).unwrap();
    let inv = db.library.macro_id("INV").unwrap();
    let net = db.net_or_insert("n");
    for (name, master, pin) in [("u1", inv, "A"), ("u2", shapeless, "p")] {
        let cell = db
            .add_cell(
                name.into(),
                master,
                pt(0, 0),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        let pin = db.cell_macro(cell).pin_id(pin).unwrap();
        let _ = db.connect_cell_pin(cell, pin, net);
    }

    assert_eq!(net_components(&db, net), 2);
}

#[test]
fn wirelength_adds_both_axes_of_a_diagonal_wire() {
    let stats = stats_of(vec![wire(M1, pt(0, 0), pt(300, 400))], Vec::new());

    assert_eq!(stats.wirelength[M1], Dbu::new(700));
}

#[test]
fn a_diagonal_wire_is_non_manhattan_and_nothing_else() {
    let stats = stats_of(vec![wire(M1, pt(0, 0), pt(300, 400))], m1_tracks());

    assert_eq!(
        (stats.non_manhattan, stats.wrong_way, stats.off_track),
        (1, 0, 0)
    );
}

#[test]
fn a_straight_wire_is_not_non_manhattan() {
    let stats = stats_of(vec![wire(M1, pt(0, 100), pt(300, 100))], m1_tracks());

    assert_eq!(stats.non_manhattan, 0);
}

#[test]
fn a_vertical_wire_on_a_horizontal_layer_runs_the_wrong_way() {
    let stats = stats_of(vec![wire(M1, pt(100, 0), pt(100, 500))], Vec::new());

    assert_eq!(stats.wrong_way, 1);
    assert_eq!(stats.wirelength[M1], Dbu::new(500));
}

#[test]
fn a_wire_in_the_preferred_direction_is_not_wrong_way() {
    let stats = stats_of(vec![wire(M1, pt(0, 100), pt(500, 100))], m1_tracks());

    assert_eq!(stats.wrong_way, 0);
}

#[test]
fn a_zero_length_wire_is_not_wrong_way() {
    let stats = stats_of(vec![wire(M1, pt(100, 100), pt(100, 100))], m1_tracks());

    assert_eq!(stats.wrong_way, 0);
}

#[test]
fn a_preferred_wire_off_its_layers_tracks_is_off_track() {
    let stats = stats_of(vec![wire(M1, pt(0, 150), pt(500, 150))], m1_tracks());

    assert_eq!(stats.off_track, 1);
}

#[test]
fn a_preferred_wire_on_a_track_is_on_track() {
    let stats = stats_of(vec![wire(M1, pt(0, 100), pt(500, 100))], m1_tracks());

    assert_eq!(stats.off_track, 0);
}

#[test]
fn a_layer_without_tracks_has_no_off_track_wires() {
    let stats = stats_of(vec![wire(M1, pt(0, 150), pt(500, 150))], Vec::new());

    assert_eq!(stats.off_track, 0);
}

#[test]
fn a_layer_whose_only_tracks_run_across_it_has_no_off_track_wires() {
    let stats = stats_of(
        vec![wire(M1, pt(0, 150), pt(500, 150))],
        vec![tracks(TrackAxis::X, 0, &[M1])],
    );

    assert_eq!(stats.off_track, 0);
}

#[test]
fn only_vias_are_counted_as_vias() {
    let db = design();
    let via = RouteElem::Via {
        via: db.library.default_via(CutLayerId(0)).unwrap(),
        at: pt(100, 100),
    };
    let patch = RouteElem::Patch {
        layer: M1,
        rect: rect(0, 0, 100, 100),
    };
    let stats = stats_of(vec![via, patch], Vec::new());

    assert_eq!(stats.vias, 1);
}
