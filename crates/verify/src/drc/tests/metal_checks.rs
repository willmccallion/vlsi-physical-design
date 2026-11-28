//! Tests of the metal checks: shorts, spacing, notches and polygon edges,
//! shapes, minimum steps and minimum cuts.

use super::*;
use pare_common::testing::assert_golden;
use proptest::prelude::*;
use proptest::strategy::ValueTree;
use proptest::test_runner::TestRunner;
use std::fmt::Write;

const MIN_AREA: i64 = 20_000;

/// The thin pieces of one routed wire of net 0 on the first metal layer.
fn thin_pieces_of_wire(wire: Rect<Dbu>, window: Option<Rect<Dbu>>) -> Vec<ThinPiece> {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, wire);
    thin_pieces(&f.layout, &rules_with_min_area(MIN_AREA), window)
}

#[test]
fn a_piece_below_the_minimum_area_is_a_thin_piece() {
    let a = Fixture::new().a;

    let thin = thin_pieces_of_wire(rect(0, 0, 100, 150), None);

    assert_eq!(
        thin,
        [ThinPiece {
            owner: a,
            layer: LayerId(0),
            bbox: rect(0, 0, 100, 150),
            area: DbuArea(15_000),
            min_area: DbuArea(MIN_AREA),
        }]
    );
}

#[test]
fn a_piece_of_the_minimum_area_or_more_is_no_thin_piece() {
    assert_eq!(thin_pieces_of_wire(rect(0, 0, 100, 200), None), []);
    assert_eq!(thin_pieces_of_wire(rect(0, 0, 100, 250), None), []);
}

#[test]
fn a_piece_with_a_fixed_edge_is_no_thin_piece() {
    let mut f = Fixture::new();
    let a = f.a;
    f.fix(M1, a, rect(0, 0, 100, 100));
    f.route(M1, a, rect(50, 0, 150, 100));

    let thin = thin_pieces(&f.layout, &rules_with_min_area(MIN_AREA), None);

    assert_eq!(thin, []);
}

#[test]
fn a_piece_well_inside_the_window_is_a_thin_piece() {
    let window = rect(-10, -10, 110, 160);

    assert_eq!(
        thin_pieces_of_wire(rect(0, 0, 100, 150), Some(window)).len(),
        1
    );
}

#[test]
fn a_piece_reaching_a_side_of_the_window_is_never_a_thin_piece() {
    let wire = rect(0, 0, 100, 150);
    let windows = [
        rect(0, -10, 110, 160),
        rect(-10, 0, 110, 160),
        rect(-10, -10, 100, 160),
        rect(-10, -10, 110, 150),
    ];

    for window in windows {
        assert_eq!(thin_pieces_of_wire(wire, Some(window)), [], "{window:?}");
    }
}

#[test]
fn a_piece_cut_by_the_window_is_never_a_thin_piece() {
    let wire = rect(0, 0, 100, 150);

    assert_eq!(thin_pieces_of_wire(wire, Some(rect(20, 20, 80, 130))), []);
}

#[test]
fn a_manufacturing_grid_of_zero_checks_nothing() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(13, 10, 1013, 110));
    let mut rules = rules(Vec::new());
    rules.manufacturing_grid = Some(Dbu::new(0));

    assert_eq!(f.check(&rules).violations, []);
}

#[test]
fn a_cut_not_joined_to_the_wide_metal_by_the_same_net_is_exempt_from_a_min_cut_length() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 1000, 500));
    f.route(M1, a, rect(1200, 200, 1500, 300));
    f.route(V1, a, rect(1300, 200, 1400, 300));

    let report = f.check(&rules_with_minimum_cut(min_cut_with_length(800)));

    assert_eq!(report.violations, []);
}

#[test]
fn metal_of_another_net_does_not_join_a_cut_to_the_wide_metal() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(M1, a, rect(0, 0, 1000, 500));
    f.route(M1, b, rect(1000, 200, 1400, 300));
    f.route(V1, a, rect(1300, 200, 1400, 300));

    let report = f.check(&rules_with_minimum_cut(min_cut_with_length(800)));

    assert!(!kinds(&report.violations).contains(&Violation::MinimumCut));
}

fn min_cut_with_length(length: i64) -> MinimumCut {
    MinimumCut {
        length: Some((Dbu::new(length), Dbu::new(800))),
        ..minimum_cut(400)
    }
}

#[test]
fn metal_exactly_a_min_cut_length_long_is_bound_by_it_and_one_shorter_is_not() {
    let with_wide_metal_length = |length: i64| {
        let mut f = Fixture::new();
        let a = f.a;
        f.route(M1, a, rect(0, 0, length, 500));
        f.route(M1, a, rect(length, 200, length + 500, 300));
        f.route(V1, a, rect(length + 300, 200, length + 400, 300));
        kinds(
            &f.check(&rules_with_minimum_cut(min_cut_with_length(1000)))
                .violations,
        )
    };

    assert_eq!(with_wide_metal_length(1000), [Violation::MinimumCut]);
    assert_eq!(with_wide_metal_length(999), []);
}

#[test]
fn a_cut_of_fixed_metal_on_routed_wide_metal_is_a_routing_violation() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 1000, 500));
    f.fix(V1, a, rect(100, 100, 200, 200));

    let report = f.check(&rules_with_minimum_cut(minimum_cut(400)));

    assert_eq!(kinds(&report.violations), [Violation::MinimumCut]);
    assert_eq!(report.input, []);
}

#[test]
fn a_routed_cut_on_fixed_wide_metal_is_a_routing_violation() {
    let mut f = Fixture::new();
    let a = f.a;
    f.fix(M1, a, rect(0, 0, 1000, 500));
    f.route(V1, a, rect(100, 100, 200, 200));

    let report = f.check(&rules_with_minimum_cut(minimum_cut(400)));

    assert_eq!(kinds(&report.violations), [Violation::MinimumCut]);
    assert_eq!(report.input, []);
}

#[test]
fn a_routed_bridge_between_fixed_long_edges_is_a_min_step() {
    let mut f = Fixture::new();
    let a = f.a;
    f.fix(M1, a, rect(0, 0, 100, 100));
    f.fix(M1, a, rect(150, 0, 250, 100));
    f.route(M1, a, rect(100, 0, 150, 140));

    let report = f.check(&rules_with_min_step(60, Some(1)));

    assert_eq!(kinds(&report.advisory), [Violation::Lef58MinStep]);
    assert_eq!(report.advisory[0].bbox, rect(100, 100, 150, 140));
}

/// The rules of [`rules`] where metal needs no spacing at all.
fn rules_without_metal_spacing() -> Rules {
    let mut rules = rules(Vec::new());
    rules.metal[LayerId(0)].spacing = table(&[0], &[(0, &[0])]);
    rules
}

fn short_of_obstruction_and_wire(
    obstruction: Rect<Dbu>,
    wire: Rect<Dbu>,
    rules: &Rules,
) -> DrcReport {
    let mut f = Fixture::new();
    let (a, obstruction_owner) = (f.a, f.obstruction);
    f.fix(M1, obstruction_owner, obstruction);
    f.route(M1, a, wire);
    f.check(rules)
}

#[test]
fn a_wire_beside_an_obstruction_where_no_spacing_is_needed_is_legal() {
    let report = short_of_obstruction_and_wire(
        rect(0, 0, 100, 1000),
        rect(100, 0, 200, 1000),
        &rules_without_metal_spacing(),
    );

    assert_eq!(report.violations, []);
}

#[test]
fn a_wire_above_an_obstruction_where_no_spacing_is_needed_is_legal() {
    let report = short_of_obstruction_and_wire(
        rect(0, 0, 1000, 100),
        rect(0, 100, 1000, 200),
        &rules_without_metal_spacing(),
    );

    assert_eq!(report.violations, []);
}

#[test]
fn a_wire_overlapping_an_obstruction_shorts_even_where_no_spacing_is_needed() {
    let report = short_of_obstruction_and_wire(
        rect(0, 0, 1000, 100),
        rect(500, 0, 1500, 100),
        &rules_without_metal_spacing(),
    );

    assert_eq!(kinds(&report.violations), [Violation::Short]);
    assert_eq!(report.violations[0].bbox, rect(500, 0, 1000, 100));
}

#[test]
fn a_wire_beside_an_obstruction_shorts_where_spacing_is_needed() {
    let report = short_of_obstruction_and_wire(
        rect(0, 0, 100, 1000),
        rect(100, 0, 200, 1000),
        &rules(Vec::new()),
    );

    assert_eq!(kinds(&report.violations), [Violation::Short]);
    assert_eq!(report.violations[0].bbox, rect(99, 0, 101, 1000));
}

#[test]
fn a_wire_above_an_obstruction_shorts_where_spacing_is_needed() {
    let report = short_of_obstruction_and_wire(
        rect(0, 0, 1000, 100),
        rect(0, 100, 1000, 200),
        &rules(Vec::new()),
    );

    assert_eq!(kinds(&report.violations), [Violation::Short]);
    assert_eq!(report.violations[0].bbox, rect(0, 99, 1000, 101));
}

#[test]
fn a_wire_stacked_on_another_nets_pin_shorts_along_the_touching_edge() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(M1, a, rect(0, 0, 1000, 100));
    f.fix(M1, b, rect(0, 100, 100, 200));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::Short]);
    assert_eq!(report.violations[0].bbox, rect(0, 99, 100, 101));
}

#[test]
fn pins_that_short_stay_an_input_defect_with_routed_metal_inside_one_of_them() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(M1, a, rect(0, 0, 1000, 100));
    f.route(M1, a, rect(600, 0, 700, 100));
    f.fix(M1, b, rect(500, 0, 1500, 100));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(report.violations, []);
    assert_eq!(kinds(&report.input), [Violation::Short]);
}

#[test]
fn a_short_stays_an_input_defect_with_routed_metal_only_beside_it_along_x() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(M1, a, rect(0, 0, 600, 100));
    f.route(M1, a, rect(600, 0, 1000, 100));
    f.fix(M1, b, rect(500, 0, 600, 100));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(report.violations, []);
    assert_eq!(kinds(&report.input), [Violation::Short]);
}

#[test]
fn a_short_stays_an_input_defect_with_routed_metal_only_beside_it_along_y() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(M1, a, rect(0, 0, 100, 600));
    f.route(M1, a, rect(0, 600, 100, 1000));
    f.fix(M1, b, rect(0, 500, 100, 600));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(report.violations, []);
    assert_eq!(kinds(&report.input), [Violation::Short]);
}

#[test]
fn a_spacing_violation_between_pins_stays_an_input_defect_though_a_wire_leaves_one() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(M1, a, rect(0, 0, 300, 100));
    f.route(M1, a, rect(200, 0, 1000, 100));
    f.fix(M1, b, rect(0, 150, 100, 250));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(report.violations, []);
    assert_eq!(kinds(&report.input), [Violation::MetalSpacing]);
}

#[test]
fn a_spacing_violation_to_a_wire_near_where_it_leaves_a_pin_is_routing() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(M1, a, rect(0, 0, 300, 100));
    f.route(M1, a, rect(200, 0, 1000, 100));
    f.fix(M1, b, rect(250, 150, 350, 250));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::MetalSpacing]);
}

#[test]
fn a_piece_of_a_net_narrow_at_a_touching_corner_does_not_make_the_overlap_insufficient() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 200, 200));
    f.route(M1, a, rect(190, 190, 290, 240));

    let report = f.check(&rules(Vec::new()));

    assert!(!kinds(&report.violations).contains(&Violation::NonSufficientMetal));
}

#[test]
fn pieces_overlapping_by_a_diagonal_as_long_as_the_minimum_width_are_sufficient() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 200, 200));
    f.route(M1, a, rect(120, 120, 320, 320));

    assert_eq!(f.check(&rules(Vec::new())).violations, []);
}

#[test]
fn pieces_overlapping_by_a_diagonal_shorter_than_the_minimum_width_are_not() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 200, 200));
    f.route(M1, a, rect(130, 130, 330, 330));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::NonSufficientMetal]);
    assert_eq!(report.violations[0].bbox, rect(130, 130, 200, 200));
}

#[test]
fn a_third_piece_overlapping_both_by_enough_makes_a_small_overlap_sufficient() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 200, 200));
    f.route(M1, a, rect(190, 190, 390, 390));
    f.route(M1, a, rect(100, 100, 300, 300));

    assert_eq!(f.check(&rules(Vec::new())).violations, []);
}

#[test]
fn shapes_outside_a_width_range_keep_the_table_spacing() {
    let mut rules = rules(Vec::new());
    rules.metal[LayerId(0)].ranges.push(WidthRangeSpacing {
        spacing: Dbu::new(300),
        min_width: Dbu::new(400),
        max_width: Dbu::new(1000),
    });

    assert_eq!(
        pair_kinds(M1, rect(0, 0, 1000, 100), rect(0, 300, 1000, 400), &rules),
        []
    );
}

#[test]
fn a_width_range_asking_no_more_than_the_table_leaves_the_violation_a_table_one() {
    let mut rules = rules(Vec::new());
    rules.metal[LayerId(0)].ranges.push(WidthRangeSpacing {
        spacing: Dbu::new(100),
        min_width: Dbu::new(400),
        max_width: Dbu::new(1000),
    });

    assert_eq!(
        pair_kinds(M1, rect(0, 0, 1000, 500), rect(0, 550, 1000, 1050), &rules),
        [Violation::MetalSpacing]
    );
}

/// Rules spacing metal by 100, or 150 beside a long run of wide metal.
fn rules_of_random_layouts() -> Rules {
    let mut rules = rules(Vec::new());
    rules.metal[LayerId(0)].spacing = table(&[0, 150], &[(0, &[100, 100]), (150, &[100, 150])]);
    rules
}

/// A layout of up to a dozen routed or fixed rectangles of two nets and
/// an obstruction on a 50-unit grid, so that their sides often meet.
fn random_layout_strategy() -> impl Strategy<Value = Vec<(u32, bool, [i64; 4])>> {
    let side = prop::sample::select(vec![50, 100, 150, 200]);
    let piece = (
        0..3_u32,
        any::<bool>(),
        (0..8_i64, 0..8_i64, side.clone(), side),
    )
        .prop_map(|(owner, is_fixed, (x, y, w, h))| (owner, is_fixed, [x * 50, y * 50, w, h]));
    prop::collection::vec(piece, 2..13)
}

/// The layout's markers, one line each: where they are as `x0 y0 x1 y1`.
fn random_layout_report(pieces: &[(u32, bool, [i64; 4])]) -> String {
    let mut f = Fixture::new();
    let owners = [f.a, f.b, f.obstruction];
    for &(owner, is_fixed, [x, y, w, h]) in pieces {
        let shape = rect(x, y, x + w, y + h);
        if is_fixed || owner == 2 {
            f.fix(M1, owners[owner as usize], shape);
        } else {
            f.route(M1, owners[owner as usize], shape);
        }
    }
    let report = f.check(&rules_of_random_layouts());
    let mut text = String::new();
    for (class, found) in [("routing", &report.violations), ("input", &report.input)] {
        for m in found {
            let b = m.bbox;
            let (min, max) = (b.min, b.max);
            let line = (min.x.get(), min.y.get(), max.x.get(), max.y.get());
            writeln!(text, "  {class} {:?} {line:?} {:?}", m.violation, m.owners).unwrap();
        }
    }
    text
}

#[test]
fn random_layouts_keep_the_markers_recorded_for_them() {
    let strategy = random_layout_strategy();
    let mut runner = TestRunner::deterministic();
    let mut text = String::new();
    for n in 0..RANDOM_LAYOUTS {
        let pieces = strategy.new_tree(&mut runner).unwrap().current();
        writeln!(text, "layout {n}: {pieces:?}").unwrap();
        text += &random_layout_report(&pieces);
    }

    assert_golden(&golden_dir(), "random_layout_markers.txt", &text);
}

const RANDOM_LAYOUTS: usize = 300;

fn golden_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}
