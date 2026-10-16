use super::*;
use crate::db::indices::{CellId, CutLayerId, LayerId, NetId};
use crate::db::rules::{
    CutSpacing, CutSpacingCondition, ParallelRunLengthTable, SpacingTable, WidthRangeSpacing,
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
        }],
        min_spacing_for_obstructions: true,
    }
}

fn cut_rule(spacing: i64) -> CutSpacing {
    CutSpacing {
        spacing: Dbu(spacing),
        center_to_center: false,
        same_net: false,
        condition: CutSpacingCondition::Always,
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
        self.layout.add(layer, owner, r, false);
    }

    fn fix(&mut self, layer: LayerRef, owner: OwnerIndex, r: Rect<Dbu>) {
        self.layout.add(layer, owner, r, true);
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
    assert_eq!(kinds(&report.fixed), [Violation::Short]);
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
    assert_eq!(kinds(&report.fixed), [Violation::CutShort]);
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
    assert!(report.fixed.is_empty());
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
