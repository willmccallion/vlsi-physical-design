use super::*;
use crate::db::indices::{CellId, CutLayerId, LayerId, NetId, NonDefaultRuleId};
use crate::db::rules::{
    CutApproach, CutSpacing, CutSpacingCondition, Lef58CutSpacing, Lef58CutSpacingCondition,
    MinimumCut, ParallelRunLengthTable, SpacingTable, WidthRangeSpacing,
};
use crate::geom::dbu::DbuArea;
use crate::geom::point::Point;

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::from_corners(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
}

const M1: LayerRef = LayerRef::Routing(LayerId(0));
const M2: LayerRef = LayerRef::Routing(LayerId(1));
const V1: LayerRef = LayerRef::Cut(CutLayerId(0));

fn table(run_lengths: &[i64], rows: &[(i64, &[i64])]) -> SpacingTable {
    SpacingTable::ParallelRunLength(ParallelRunLengthTable {
        run_lengths: run_lengths.iter().copied().map(Dbu).collect(),
        rows: rows
            .iter()
            .map(|&(w, s)| (Dbu(w), s.iter().copied().map(Dbu).collect()))
            .collect(),
    })
}

fn metal(spacing: SpacingTable) -> MetalRules {
    MetalRules {
        width: Dbu(100),
        min_width: Dbu(100),
        spacing,
        ranges: Vec::new(),
        min_area: None,
        minimum_cuts: Vec::new(),
        min_enclosed_areas: Vec::new(),
    }
}

/// The rules of [`rules`] with a minimum area of `area` on the first metal
/// layer.
fn rules_with_min_area(area: i64) -> Rules {
    let mut rules = rules(Vec::new());
    rules.metal[0].min_area = Some(DbuArea(area));
    rules
}

/// Two metal layers needing 100 apart and one cut layer between them.
fn rules(cut_spacings: Vec<CutSpacing>) -> Rules {
    Rules {
        metal: vec![
            metal(table(&[0], &[(0, &[100])])),
            metal(table(&[0], &[(0, &[100])])),
        ],
        cut: vec![CutRules {
            width: Some(Dbu(100)),
            spacings: cut_spacings,
            lef58_spacings: Vec::new(),
        }],
        min_spacing_for_obstructions: true,
        manufacturing_grid: None,
        non_default: Vec::new(),
    }
}

/// The rules of [`rules`] with cut spacing property rules as well.
fn rules_with_properties(statements: Vec<CutSpacing>, properties: Vec<Lef58CutSpacing>) -> Rules {
    let mut rules = rules(statements);
    rules.cut[0].lef58_spacings = properties;
    rules
}

fn cut_rule(spacing: i64) -> CutSpacing {
    CutSpacing {
        spacing: Dbu(spacing),
        center_to_center: false,
        same_net: false,
        condition: CutSpacingCondition::Always,
    }
}

fn parallel_overlap_property(spacing: i64) -> Lef58CutSpacing {
    Lef58CutSpacing {
        spacing: Dbu(spacing),
        center_to_center: false,
        same_net: false,
        condition: Lef58CutSpacingCondition::ParallelOverlap,
    }
}

/// A layout with nets 0 and 1 and the obstructions of cell 0.
struct Fixture {
    layout: Layout,
    a: OwnerIndex,
    b: OwnerIndex,
    obstruction: OwnerIndex,
}

impl Fixture {
    fn new() -> Self {
        let mut layout = Layout::new(2, 1);
        let a = layout.owner(Owner::Net(NetId(0)), false);
        let b = layout.owner(Owner::Net(NetId(1)), false);
        let obstruction = layout.owner(Owner::Obstruction(CellId(0)), false);
        Self {
            layout,
            a,
            b,
            obstruction,
        }
    }

    fn route(&mut self, layer: LayerRef, owner: OwnerIndex, r: Rect<Dbu>) {
        self.layout.add(layer, owner, r, Origin::Routing);
    }

    fn fix(&mut self, layer: LayerRef, owner: OwnerIndex, r: Rect<Dbu>) {
        self.layout.add(layer, owner, r, Origin::Input);
    }

    fn place(&mut self, layer: LayerRef, owner: OwnerIndex, r: Rect<Dbu>) {
        self.layout.add(layer, owner, r, Origin::Placement);
    }

    fn check(&self, rules: &Rules) -> DrcReport {
        check(&self.layout, rules)
    }
}

fn kinds(markers: &[Marker]) -> Vec<Violation> {
    markers.iter().map(|m| m.violation).collect()
}

#[test]
fn wires_of_two_nets_that_overlap_short_where_they_overlap() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(M1, a, rect(0, 0, 1000, 100));
    f.route(M1, b, rect(500, 0, 1500, 100));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::Short]);
    assert_eq!(report.violations[0].bbox, rect(500, 0, 1000, 100));
}

#[test]
fn a_gap_narrower_than_the_spacing_is_a_spacing_violation_over_the_gap() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(M1, a, rect(0, 0, 1000, 100));
    f.route(M1, b, rect(0, 150, 1000, 250));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::MetalSpacing]);
    assert_eq!(report.violations[0].bbox, rect(0, 100, 1000, 150));
}

#[test]
fn a_gap_as_wide_as_the_spacing_is_legal() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(M1, a, rect(0, 0, 1000, 100));
    f.route(M1, b, rect(0, 200, 1000, 300));

    assert!(f.check(&rules(Vec::new())).violations.is_empty());
}

#[test]
fn diagonal_neighbours_are_measured_corner_to_corner() {
    let mut near = Fixture::new();
    let (a, b) = (near.a, near.b);
    near.route(M1, a, rect(0, 0, 100, 100));
    near.route(M1, b, rect(160, 160, 260, 260));
    let mut far = Fixture::new();
    far.route(M1, a, rect(0, 0, 100, 100));
    far.route(M1, b, rect(172, 172, 272, 272));

    assert_eq!(
        kinds(&near.check(&rules(Vec::new())).violations),
        [Violation::MetalSpacing]
    );
    assert!(far.check(&rules(Vec::new())).violations.is_empty());
}

#[test]
fn violations_of_fixed_metal_alone_are_reported_apart() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(M1, a, rect(0, 0, 1000, 100));
    f.fix(M1, b, rect(500, 0, 1500, 100));

    let report = f.check(&rules(Vec::new()));

    assert!(report.violations.is_empty());
    assert_eq!(kinds(&report.input), [Violation::Short]);
}

#[test]
fn a_wire_abutting_another_nets_pin_shorts() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(M1, a, rect(0, 0, 100, 1000));
    f.fix(M1, b, rect(100, 0, 200, 100));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::Short]);
    assert_eq!(report.violations[0].bbox, rect(99, 0, 101, 100));
}

#[test]
fn pieces_of_one_net_overlapping_by_less_than_the_minimum_width_are_non_sufficient_metal() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 100, 100));
    f.route(M1, a, rect(90, 90, 190, 190));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::NonSufficientMetal]);
    assert_eq!(report.violations[0].bbox, rect(90, 90, 100, 100));
}

#[test]
fn an_ell_of_one_net_is_legal() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 1000, 100));
    f.route(M1, a, rect(0, 0, 100, 1000));

    assert!(f.check(&rules(Vec::new())).violations.is_empty());
}

#[test]
fn a_notch_of_one_net_narrower_than_the_spacing_is_a_spacing_violation() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 100, 500));
    f.route(M1, a, rect(150, 0, 250, 500));
    f.route(M1, a, rect(0, 0, 250, 100));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::MetalSpacing]);
    assert_eq!(report.violations[0].bbox, rect(100, 0, 150, 500));
}

#[test]
fn a_longer_parallel_run_needs_more_spacing() {
    let mut rules = rules(Vec::new());
    rules.metal[0].spacing = table(&[0, 500], &[(0, &[100, 150])]);
    let mut short_run = Fixture::new();
    let (a, b) = (short_run.a, short_run.b);
    short_run.route(M1, a, rect(0, 0, 400, 100));
    short_run.route(M1, b, rect(0, 220, 400, 320));
    let mut long_run = Fixture::new();
    long_run.route(M1, a, rect(0, 0, 600, 100));
    long_run.route(M1, b, rect(0, 220, 600, 320));

    assert!(short_run.check(&rules).violations.is_empty());
    assert_eq!(
        kinds(&long_run.check(&rules).violations),
        [Violation::MetalSpacing]
    );
}

#[test]
fn an_obstruction_counts_as_a_default_width_wire_when_the_library_says_so() {
    let mut wide_needs_more = rules(Vec::new());
    wide_needs_more.metal[0].spacing = table(&[0], &[(0, &[100]), (500, &[200])]);
    let mut by_width = wide_needs_more.clone();
    by_width.min_spacing_for_obstructions = false;
    let mut f = Fixture::new();
    let (a, obstruction) = (f.a, f.obstruction);
    f.fix(M1, obstruction, rect(0, 0, 1000, 1000));
    f.route(M1, a, rect(0, 1150, 1000, 1250));

    assert!(f.check(&wide_needs_more).violations.is_empty());
    assert_eq!(
        kinds(&f.check(&by_width).violations),
        [Violation::MetalSpacing]
    );
}

#[test]
fn a_wire_crossing_an_obstruction_shorts_outside_its_own_pin() {
    let mut f = Fixture::new();
    let (a, obstruction) = (f.a, f.obstruction);
    f.fix(M1, obstruction, rect(0, 0, 1000, 1000));
    f.fix(M1, a, rect(400, 400, 600, 600));
    f.route(M1, a, rect(400, 400, 1500, 500));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::Short]);
    assert_eq!(report.violations[0].bbox, rect(600, 400, 1000, 500));
}

#[test]
fn shapes_in_a_width_range_keep_the_range_spacing() {
    let mut rules = rules(Vec::new());
    rules.metal[0].ranges.push(WidthRangeSpacing {
        spacing: Dbu(300),
        min_width: Dbu(400),
        max_width: Dbu(1000),
    });
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(M1, a, rect(0, 0, 1000, 500));
    f.route(M1, b, rect(0, 700, 1000, 800));

    assert_eq!(
        kinds(&f.check(&rules).violations),
        [Violation::SpacingRange]
    );
}

#[test]
fn cuts_of_two_nets_closer_than_the_spacing_violate_it() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(150, 0, 250, 100));

    let report = f.check(&rules(vec![cut_rule(100)]));

    assert_eq!(kinds(&report.violations), [Violation::CutSpacing]);
    assert_eq!(report.violations[0].bbox, rect(100, 0, 150, 100));
}

#[test]
fn touching_cuts_short() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(100, 0, 200, 100));

    let report = f.check(&rules(vec![cut_rule(100)]));

    assert_eq!(kinds(&report.violations), [Violation::CutShort]);
}

#[test]
fn identical_routed_cuts_of_one_net_short() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, a, rect(0, 0, 100, 100));

    let report = f.check(&rules(vec![cut_rule(100)]));

    assert_eq!(kinds(&report.violations), [Violation::CutShort]);
}

#[test]
fn a_routed_cut_on_a_fixed_cut_of_its_owner_counts_as_fixed() {
    let mut f = Fixture::new();
    let a = f.a;
    f.fix(V1, a, rect(0, 0, 100, 100));
    f.route(V1, a, rect(0, 0, 100, 100));

    let report = f.check(&rules(vec![cut_rule(100)]));

    assert!(report.violations.is_empty());
    assert_eq!(kinds(&report.input), [Violation::CutShort]);
}

#[test]
fn a_cut_layer_without_spacing_rules_checks_nothing() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(50, 0, 150, 100));

    assert!(f.check(&rules(Vec::new())).violations.is_empty());
}

#[test]
fn center_to_center_spacing_is_measured_between_centers() {
    let rule = CutSpacing {
        center_to_center: true,
        ..cut_rule(200)
    };
    let mut near = Fixture::new();
    let (a, b) = (near.a, near.b);
    near.route(V1, a, rect(0, 0, 100, 100));
    near.route(V1, b, rect(150, 0, 250, 100));
    let mut far = Fixture::new();
    far.route(V1, a, rect(0, 0, 100, 100));
    far.route(V1, b, rect(250, 0, 350, 100));

    assert_eq!(
        kinds(&near.check(&rules(vec![rule])).violations),
        [Violation::CutSpacing]
    );
    assert!(far.check(&rules(vec![rule])).violations.is_empty());
}

#[test]
fn cuts_of_one_net_follow_the_same_net_rule_when_there_is_one() {
    let same_net = CutSpacing {
        same_net: true,
        ..cut_rule(80)
    };
    let both = rules(vec![cut_rule(150), same_net]);
    let mut one_net = Fixture::new();
    let (a, b) = (one_net.a, one_net.b);
    one_net.route(V1, a, rect(0, 0, 100, 100));
    one_net.route(V1, a, rect(200, 0, 300, 100));
    let mut two_nets = Fixture::new();
    two_nets.route(V1, a, rect(0, 0, 100, 100));
    two_nets.route(V1, b, rect(200, 0, 300, 100));

    assert!(one_net.check(&both).violations.is_empty());
    assert_eq!(
        kinds(&one_net.check(&rules(vec![cut_rule(150)])).violations),
        [Violation::CutSpacing]
    );
    assert_eq!(
        kinds(&two_nets.check(&both).violations),
        [Violation::CutSpacing]
    );
}

#[test]
fn an_adjacent_cuts_rule_needs_enough_close_neighbours() {
    let rule = CutSpacing {
        condition: CutSpacingCondition::AdjacentCuts {
            cuts: 2,
            within: Dbu(300),
        },
        ..cut_rule(200)
    };
    let mut pair = Fixture::new();
    let (a, b) = (pair.a, pair.b);
    pair.route(V1, a, rect(0, 0, 100, 100));
    pair.route(V1, b, rect(250, 0, 350, 100));
    let mut crowd = Fixture::new();
    crowd.route(V1, a, rect(0, 0, 100, 100));
    crowd.route(V1, b, rect(250, 0, 350, 100));
    crowd.route(V1, b, rect(0, 250, 100, 350));

    assert!(pair.check(&rules(vec![rule])).violations.is_empty());
    assert!(!crowd.check(&rules(vec![rule])).violations.is_empty());
}

#[test]
fn cuts_on_shared_metal_need_no_parallel_overlap_spacing() {
    let rule = CutSpacing {
        condition: CutSpacingCondition::ParallelOverlap,
        ..cut_rule(100)
    };
    let mut joined = Fixture::new();
    let a = joined.a;
    joined.route(V1, a, rect(0, 0, 100, 100));
    joined.route(V1, a, rect(150, 0, 250, 100));
    joined.route(M2, a, rect(0, 0, 250, 100));
    let mut apart = Fixture::new();
    apart.route(V1, a, rect(0, 0, 100, 100));
    apart.route(V1, a, rect(150, 0, 250, 100));

    assert!(joined.check(&rules(vec![rule])).violations.is_empty());
    assert_eq!(
        kinds(&apart.check(&rules(vec![rule])).violations),
        [Violation::CutSpacing]
    );
}

#[test]
fn a_parallel_overlap_property_reaches_cuts_that_do_not_run_alongside() {
    let statement = CutSpacing {
        condition: CutSpacingCondition::ParallelOverlap,
        ..cut_rule(100)
    };
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(150, 150, 250, 250));

    assert!(f.check(&rules(vec![statement])).violations.is_empty());
    let report = f.check(&rules_with_properties(
        Vec::new(),
        vec![parallel_overlap_property(100)],
    ));
    assert_eq!(kinds(&report.violations), [Violation::CutSpacing]);
    assert_eq!(report.violations[0].bbox, rect(100, 100, 150, 150));
}

#[test]
fn touching_cuts_break_a_parallel_overlap_property_besides_shorting() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(100, 0, 200, 100));

    let report = f.check(&rules_with_properties(
        vec![cut_rule(100)],
        vec![parallel_overlap_property(100)],
    ));

    assert_eq!(
        kinds(&report.violations),
        [Violation::CutShort, Violation::CutSpacing]
    );
}

#[test]
fn cuts_meeting_at_a_corner_only_short_under_a_parallel_overlap_property() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(100, 100, 200, 200));

    let report = f.check(&rules_with_properties(
        vec![cut_rule(100)],
        vec![parallel_overlap_property(100)],
    ));

    assert_eq!(kinds(&report.violations), [Violation::CutShort]);
}

#[test]
fn cuts_on_shared_metal_are_exempt_from_a_parallel_overlap_property() {
    let rules = rules_with_properties(Vec::new(), vec![parallel_overlap_property(100)]);
    let mut joined = Fixture::new();
    let a = joined.a;
    joined.route(V1, a, rect(0, 0, 100, 100));
    joined.route(V1, a, rect(150, 0, 250, 100));
    joined.route(M2, a, rect(0, 0, 250, 100));
    let mut apart = Fixture::new();
    apart.route(V1, a, rect(0, 0, 100, 100));
    apart.route(V1, a, rect(150, 0, 250, 100));

    assert!(joined.check(&rules).violations.is_empty());
    assert_eq!(
        kinds(&apart.check(&rules).violations),
        [Violation::CutSpacing]
    );
}

#[test]
fn a_property_rule_is_checked_beside_the_spacing_statements() {
    let rules = rules_with_properties(vec![cut_rule(100)], vec![parallel_overlap_property(130)]);
    let mut wide = Fixture::new();
    let (a, b) = (wide.a, wide.b);
    wide.route(V1, a, rect(0, 0, 100, 100));
    wide.route(V1, b, rect(220, 0, 320, 100));
    let mut narrow = Fixture::new();
    narrow.route(V1, a, rect(0, 0, 100, 100));
    narrow.route(V1, b, rect(150, 0, 250, 100));

    assert_eq!(
        kinds(&wide.check(&rules).violations),
        [Violation::CutSpacing]
    );
    let both = narrow.check(&rules).violations;
    assert_eq!(kinds(&both), [Violation::CutSpacing, Violation::CutSpacing]);
    assert_eq!(both[0].bbox, both[1].bbox);
    assert_ne!(both[0].rule, both[1].rule);
}

#[test]
fn a_same_net_property_rule_leaves_cuts_of_different_nets_alone() {
    let same_net = Lef58CutSpacing {
        same_net: true,
        ..parallel_overlap_property(100)
    };
    let rules = rules_with_properties(Vec::new(), vec![same_net]);
    let mut one_net = Fixture::new();
    let (a, b) = (one_net.a, one_net.b);
    one_net.route(V1, a, rect(0, 0, 100, 100));
    one_net.route(V1, a, rect(150, 0, 250, 100));
    let mut two_nets = Fixture::new();
    two_nets.route(V1, a, rect(0, 0, 100, 100));
    two_nets.route(V1, b, rect(150, 0, 250, 100));

    assert_eq!(
        kinds(&one_net.check(&rules).violations),
        [Violation::CutSpacing]
    );
    assert!(two_nets.check(&rules).violations.is_empty());
}

#[test]
fn a_routed_piece_smaller_than_the_minimum_area_violates_it() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 100, 150));

    let report = f.check(&rules_with_min_area(20_000));

    assert_eq!(kinds(&report.violations), [Violation::MinArea]);
    assert_eq!(report.violations[0].bbox, rect(0, 0, 100, 150));
    assert_eq!(report.violations[0].owners, (a, a));
}

#[test]
fn a_piece_as_large_as_the_minimum_area_is_legal() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 100, 200));

    assert!(f.check(&rules_with_min_area(20_000)).violations.is_empty());
}

#[test]
fn separate_pieces_of_one_net_each_need_the_minimum_area() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 100, 150));
    f.route(M1, a, rect(500, 0, 600, 150));

    let report = f.check(&rules_with_min_area(20_000));

    assert_eq!(
        kinds(&report.violations),
        [Violation::MinArea, Violation::MinArea]
    );
}

#[test]
fn a_piece_with_an_edge_of_fixed_metal_is_exempt_from_the_minimum_area() {
    let mut f = Fixture::new();
    let a = f.a;
    f.fix(M1, a, rect(0, 0, 100, 100));
    f.route(M1, a, rect(50, 0, 150, 100));

    let report = f.check(&rules_with_min_area(20_000));

    assert!(report.violations.is_empty());
    assert!(report.input.is_empty());
}

#[test]
fn a_piece_whose_fixed_metal_is_buried_still_needs_the_minimum_area() {
    let mut f = Fixture::new();
    let a = f.a;
    f.fix(M1, a, rect(20, 20, 80, 80));
    f.route(M1, a, rect(0, 0, 100, 150));

    let report = f.check(&rules_with_min_area(20_000));

    assert_eq!(kinds(&report.violations), [Violation::MinArea]);
}

#[test]
fn a_routed_wire_narrower_than_the_minimum_width_violates_it() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 1000, 50));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::MinWidth]);
    assert_eq!(report.violations[0].bbox, rect(0, 0, 1000, 50));
}

#[test]
fn a_narrow_neck_between_wide_parts_violates_the_minimum_width() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 200, 200));
    f.route(M1, a, rect(200, 75, 400, 125));
    f.route(M1, a, rect(400, 0, 600, 200));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::MinWidth]);
    assert_eq!(report.violations[0].bbox, rect(200, 75, 400, 125));
}

#[test]
fn a_narrow_slice_under_the_owners_fixed_metal_is_exempt_from_the_minimum_width() {
    let mut f = Fixture::new();
    let a = f.a;
    f.fix(M1, a, rect(0, 0, 1000, 50));
    f.route(M1, a, rect(0, 0, 200, 50));

    assert!(f.check(&rules(Vec::new())).violations.is_empty());
}

#[test]
fn a_narrow_slice_partly_over_fixed_metal_violates_the_minimum_width() {
    let mut f = Fixture::new();
    let a = f.a;
    f.fix(M1, a, rect(0, 0, 500, 50));
    f.route(M1, a, rect(400, 0, 1000, 50));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.violations), [Violation::MinWidth]);
    assert_eq!(report.violations[0].bbox, rect(0, 0, 1000, 50));
}

/// The rules of [`rules`] with `rule` on the first metal layer.
fn rules_with_minimum_cut(rule: MinimumCut) -> Rules {
    let mut rules = rules(Vec::new());
    rules.metal[0].minimum_cuts = vec![rule];
    rules
}

fn minimum_cut(width: i64) -> MinimumCut {
    MinimumCut {
        cuts: 2,
        width: Dbu(width),
        within: None,
        from: None,
        length: None,
    }
}

#[test]
fn a_cut_on_metal_wide_enough_for_a_minimum_cut_rule_violates_it() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 1000, 500));
    f.route(V1, a, rect(100, 100, 200, 200));

    let report = f.check(&rules_with_minimum_cut(minimum_cut(400)));

    assert_eq!(kinds(&report.violations), [Violation::MinimumCut]);
    assert_eq!(report.violations[0].bbox, rect(100, 100, 200, 200));
    assert_eq!(report.violations[0].layer, M1);
}

#[test]
fn a_cut_on_metal_narrower_than_the_rule_width_is_legal() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 1000, 300));
    f.route(V1, a, rect(100, 100, 200, 200));

    assert!(
        f.check(&rules_with_minimum_cut(minimum_cut(400)))
            .violations
            .is_empty()
    );
}

#[test]
fn a_minimum_cut_rule_from_above_ignores_cuts_below() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M2, a, rect(0, 0, 1000, 500));
    f.route(V1, a, rect(100, 100, 200, 200));
    let mut rules = rules(Vec::new());
    rules.metal[1].minimum_cuts = vec![MinimumCut {
        from: Some(CutApproach::Above),
        ..minimum_cut(400)
    }];

    assert!(f.check(&rules).violations.is_empty());
}

#[test]
fn with_a_length_a_cut_inside_the_wide_metal_is_exempt() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 1000, 500));
    f.route(V1, a, rect(100, 100, 200, 200));
    let rule = MinimumCut {
        length: Some((Dbu(800), Dbu(800))),
        ..minimum_cut(400)
    };

    assert!(f.check(&rules_with_minimum_cut(rule)).violations.is_empty());
}

#[test]
fn with_a_length_a_cut_joined_to_the_wide_metal_within_the_distance_violates_it() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 1000, 500));
    f.route(M1, a, rect(1000, 200, 1500, 300));
    f.route(V1, a, rect(1300, 200, 1400, 300));
    let rule = MinimumCut {
        length: Some((Dbu(800), Dbu(800))),
        ..minimum_cut(400)
    };

    let report = f.check(&rules_with_minimum_cut(rule));

    assert_eq!(kinds(&report.violations), [Violation::MinimumCut]);
    assert_eq!(report.violations[0].bbox, rect(1000, 200, 1300, 300));
}

/// A routed ring of net 0 on the first metal layer around a 200 by 200 hole.
fn route_ring(f: &mut Fixture) {
    let a = f.a;
    f.route(M1, a, rect(0, 0, 600, 200));
    f.route(M1, a, rect(0, 400, 600, 600));
    f.route(M1, a, rect(0, 0, 200, 600));
    f.route(M1, a, rect(400, 0, 600, 600));
}

#[test]
fn a_hole_smaller_than_the_minimum_enclosed_area_violates_it() {
    let mut f = Fixture::new();
    route_ring(&mut f);
    let mut rules = rules(Vec::new());
    rules.metal[0].min_enclosed_areas = vec![DbuArea(50_000)];

    let report = f.check(&rules);

    assert_eq!(kinds(&report.violations), [Violation::MinHole]);
    assert_eq!(report.violations[0].bbox, rect(200, 200, 400, 400));
}

#[test]
fn a_hole_as_large_as_the_minimum_enclosed_area_is_legal() {
    let mut f = Fixture::new();
    route_ring(&mut f);
    let mut rules = rules(Vec::new());
    rules.metal[0].min_enclosed_areas = vec![DbuArea(40_000)];

    assert!(f.check(&rules).violations.is_empty());
}

#[test]
fn routed_metal_off_the_manufacturing_grid_violates_it() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 1003, 100));
    let rules = Rules {
        manufacturing_grid: Some(Dbu(5)),
        non_default: Vec::new(),
        ..rules(Vec::new())
    };

    let report = f.check(&rules);

    assert_eq!(kinds(&report.violations), [Violation::OffGrid]);
    assert_eq!(report.violations[0].bbox, rect(0, 0, 1003, 100));
}

#[test]
fn fixed_metal_off_the_manufacturing_grid_is_exempt() {
    let mut f = Fixture::new();
    let a = f.a;
    f.fix(M1, a, rect(0, 0, 1003, 100));
    let rules = Rules {
        manufacturing_grid: Some(Dbu(5)),
        non_default: Vec::new(),
        ..rules(Vec::new())
    };

    let report = f.check(&rules);

    assert!(report.violations.is_empty());
    assert!(report.input.is_empty());
}

#[test]
fn fixed_metal_a_placed_cell_takes_part_in_is_a_placement_defect() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.place(M1, a, rect(0, 0, 1000, 100));
    f.fix(M1, b, rect(500, 0, 1500, 100));

    let report = f.check(&rules(Vec::new()));

    assert!(report.violations.is_empty());
    assert_eq!(kinds(&report.placement), [Violation::Short]);
    assert!(report.input.is_empty());
}

#[test]
fn placed_cuts_touching_input_cuts_are_a_placement_defect() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.place(V1, a, rect(0, 0, 100, 100));
    f.fix(V1, b, rect(100, 0, 200, 100));

    let report = f.check(&rules(vec![cut_rule(100)]));

    assert_eq!(kinds(&report.placement), [Violation::CutShort]);
    assert!(report.input.is_empty());
}

#[test]
fn placed_metal_near_a_fixed_only_violation_but_not_in_it_leaves_it_an_input_defect() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(M1, a, rect(0, 0, 1000, 100));
    f.fix(M1, b, rect(1050, 0, 2000, 100));
    f.place(M1, a, rect(0, 300, 1000, 400));

    let report = f.check(&rules(Vec::new()));

    assert_eq!(kinds(&report.input), [Violation::MetalSpacing]);
    assert!(report.placement.is_empty());
}

/// The rules of [`rules`] with one nondefault rule whose hard spacing on the
/// first metal layer is `spacing`.
fn rules_with_hard_spacing(spacing: i64) -> Rules {
    let mut rules = rules(Vec::new());
    rules.non_default = vec![NonDefaultRules {
        hard_spacing: vec![Some(Dbu(spacing)), None],
    }];
    rules
}

#[test]
fn a_hard_nondefault_spacing_widens_the_gap_a_net_needs() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.layout.set_non_default_rule(a, NonDefaultRuleId::new(0));
    f.route(M1, a, rect(0, 0, 1000, 100));
    f.route(M1, b, rect(0, 250, 1000, 350));

    assert!(f.check(&rules(Vec::new())).violations.is_empty());
    let report = f.check(&rules_with_hard_spacing(200));
    assert_eq!(kinds(&report.violations), [Violation::MetalSpacing]);
    assert_eq!(report.violations[0].bbox, rect(0, 100, 1000, 250));
}

#[test]
fn a_hard_nondefault_spacing_binds_the_nets_own_pieces_too() {
    let mut f = Fixture::new();
    let a = f.a;
    f.layout.set_non_default_rule(a, NonDefaultRuleId::new(0));
    f.route(M1, a, rect(0, 0, 1000, 100));
    f.route(M1, a, rect(0, 250, 1000, 350));

    assert_eq!(
        kinds(&f.check(&rules_with_hard_spacing(200)).violations),
        [Violation::MetalSpacing]
    );
}

#[test]
fn a_hard_nondefault_spacing_on_another_layer_changes_nothing() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.layout.set_non_default_rule(a, NonDefaultRuleId::new(0));
    f.route(M2, a, rect(0, 0, 1000, 100));
    f.route(M2, b, rect(0, 250, 1000, 350));

    assert!(f.check(&rules_with_hard_spacing(200)).violations.is_empty());
}
