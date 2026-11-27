use super::*;
use crate::db::rules::{Lef58CutSpacing, Lef58CutSpacingCondition};

#[test]
fn lef_layers_carry_their_resistance_and_capacitance() {
    let fixtures = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let units = crate::db::parser::def::read_units(&fixtures.join("tiny.def")).unwrap();
    let lib = crate::db::parser::lef::parse_files(&[fixtures.join("tiny.lef")], units).unwrap();

    let m1 = &lib.layer(LayerId(0)).electrical;
    let m2 = &lib.layer(LayerId(1)).electrical;
    assert_eq!(
        (
            m1.resistance_per_square,
            m1.capacitance_per_area,
            m1.edge_capacitance
        ),
        (Some(0.38), Some(7.7161e-05), Some(2.7365e-05))
    );
    assert_eq!(
        (
            m2.resistance_per_square,
            m2.capacitance_per_area,
            m2.edge_capacitance
        ),
        (Some(0.25), None, None)
    );
    assert_eq!(
        lib.cut_layers[CutLayerId(0)].resistance,
        Some(Ohms::new(5.0))
    );
    assert_eq!(lib.cut_layers[CutLayerId(1)].resistance, None);
}

fn routing_layer(name: String) -> RoutingLayer {
    RoutingLayer {
        name,
        direction: Direction::Horizontal,
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

#[test]
fn a_routing_layer_past_what_layer_ids_count_is_refused() {
    let mut library = Library::new(DbuPerMicron::new(1000).unwrap());
    for k in 0..256 {
        assert!(
            library
                .add_routing_layer(routing_layer(format!("m{k}")))
                .is_ok()
        );
    }

    let refused = library.add_routing_layer(routing_layer("m256".into()));

    assert!(refused.is_err());
    assert_eq!(library.routing_layers.len(), 256);
    assert_eq!(library.layer_ref("m256"), None);
}

fn tiny_library() -> Library {
    let fixtures = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let units = crate::db::parser::def::read_units(&fixtures.join("tiny.def")).unwrap();
    crate::db::parser::lef::parse_files(&[fixtures.join("tiny.lef")], units).unwrap()
}

#[test]
fn a_quarter_turn_mirrored_needs_every_symmetry() {
    let with = |x, y, r90| Symmetry { x, y, r90 };

    assert!(with(true, true, true).allows(Orientation::FW));
    assert!(!with(false, true, true).allows(Orientation::FW));
    assert!(!with(true, false, true).allows(Orientation::FE));
    assert!(!with(true, true, false).allows(Orientation::FE));
}

#[test]
fn cut_layers_are_found_by_name_and_routing_layers_are_not() {
    let lib = tiny_library();

    assert_eq!(lib.cut_layer_id("V1"), Some(CutLayerId(0)));
    assert_eq!(lib.cut_layer_id("M1"), None);
}

#[test]
fn the_default_via_of_a_cut_is_on_that_cut() {
    let lib = tiny_library();

    let via = lib.default_via(CutLayerId(1)).unwrap();
    assert_eq!(lib.via(via).name, "M2M3");
}

#[test]
fn a_wire_takes_its_layers_resistance_per_square_and_capacitance_per_width() {
    let electrical = LayerElectrical {
        resistance_per_square: Some(0.38),
        capacitance_per_area: Some(7.7161e-05),
        edge_capacitance: Some(2.7365e-05),
    };
    let units = DbuPerMicron::new(1000).unwrap();

    let (res, cap) = electrical.wire_rc(Dbu::new(100), units);

    // 0.38 / 0.1 ohm and 0.1 * 7.7161e-5 + 2 * 2.7365e-5 pF per micron.
    let close = |got: f64, want: f64| (got - want).abs() <= want * 1e-6;
    assert!(close(res.get(), 3.8e6), "{res:?}");
    assert!(close(cap.get(), 6.24461e-11), "{cap:?}");
}

#[test]
fn the_narrowest_wire_is_the_min_width_but_never_past_the_default() {
    let widths = [None, Some(80), Some(120)].map(|min_width| {
        let mut layer = routing_layer("M1".into());
        layer.rules.min_width = min_width.map(Dbu::new);
        layer.min_width()
    });

    assert_eq!(widths, [100, 80, 100].map(Dbu::new));
}

fn parallel_overlap(spacing: i64, same_net: bool) -> Lef58CutSpacing {
    Lef58CutSpacing {
        spacing: Dbu::new(spacing),
        center_to_center: false,
        same_net,
        condition: Lef58CutSpacingCondition::ParallelOverlap,
    }
}

#[test]
fn a_same_net_parallel_overlap_rule_sets_no_unconditional_spacing() {
    let rules = CutLayerRules {
        lef58_spacings: vec![parallel_overlap(150, true), parallel_overlap(100, false)],
        ..CutLayerRules::default()
    };

    assert_eq!(
        CutLayer::unconditional_spacing(None, &rules),
        Some(Dbu::new(100))
    );
}

#[test]
fn a_master_turned_half_round_needs_both_mirror_symmetries() {
    let x_only = Symmetry {
        x: true,
        y: false,
        r90: false,
    };

    assert!(x_only.allows(Orientation::FS));
    assert!(!x_only.allows(Orientation::S));
}

#[test]
fn blocks_pads_rings_and_covers_are_blocks_and_core_cells_are_not() {
    let classes = [
        MacroClass::Block,
        MacroClass::Pad,
        MacroClass::Core(CoreKind::Regular),
    ];
    let fixtures = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let units = crate::db::parser::def::read_units(&fixtures.join("tiny.def")).unwrap();
    let lib = crate::db::parser::lef::parse_files(&[fixtures.join("tiny.lef")], units).unwrap();
    let mut master = lib.macros[lib.macro_id("INV").unwrap()].clone();

    let blocks = classes.map(|class| {
        master.class = class;
        master.is_block()
    });

    assert_eq!(blocks, [true, true, false]);
}
