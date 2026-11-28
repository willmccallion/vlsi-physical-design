//! Tests of the end-of-line and cut checks, the layout a design gives,
//! tiling and the rules' reach.

use super::*;
use pare_common::db::design::{NonDefaultLayer, NonDefaultRule, PlacementStatus, RouteElem};
use pare_common::db::indices::{BlockageId, IoPinId, SpecialNetId};
use pare_common::db::library::SignalUse;
use pare_common::db::parser::{def, lef};
use std::path::{Path, PathBuf};

/// One of the eight turns and mirrorings of the plane, to run a scene
/// whose line end runs east in every heading.
#[derive(Clone, Copy, Debug)]
struct Symmetry {
    quarter_turns: u8,
    mirrored: bool,
}

impl Symmetry {
    const ALL: [Self; 8] = [
        Self::new(0, false),
        Self::new(1, false),
        Self::new(2, false),
        Self::new(3, false),
        Self::new(0, true),
        Self::new(1, true),
        Self::new(2, true),
        Self::new(3, true),
    ];

    const fn new(quarter_turns: u8, mirrored: bool) -> Self {
        Self {
            quarter_turns,
            mirrored,
        }
    }

    fn apply_to_point(self, (mut x, mut y): (i64, i64)) -> (i64, i64) {
        if self.mirrored {
            x = -x;
        }
        for _ in 0..self.quarter_turns {
            (x, y) = (-y, x);
        }
        (x, y)
    }

    fn apply(self, r: Rect<Dbu>) -> Rect<Dbu> {
        let (x0, y0) = self.apply_to_point((r.min.x.get(), r.min.y.get()));
        let (x1, y1) = self.apply_to_point((r.max.x.get(), r.max.y.get()));
        rect(x0.min(x1), y0.min(y1), x0.max(x1), y0.max(y1))
    }
}

#[derive(Clone, Copy)]
enum Net {
    A,
    B,
    Obstruction,
}

/// A rectangle of one of the fixture's owners on the first metal layer.
#[derive(Clone, Copy)]
struct Wire {
    net: Net,
    origin: Origin,
    rect: Rect<Dbu>,
}

fn routed(net: Net, x0: i64, y0: i64, x1: i64, y1: i64) -> Wire {
    Wire {
        net,
        origin: Origin::Routing,
        rect: rect(x0, y0, x1, y1),
    }
}

fn fixed(net: Net, x0: i64, y0: i64, x1: i64, y1: i64) -> Wire {
    Wire {
        origin: Origin::Input,
        ..routed(net, x0, y0, x1, y1)
    }
}

/// The owners of nets A and B of a [`Fixture`].
fn nets_a_and_b() -> (OwnerIndex, OwnerIndex) {
    let f = Fixture::new();
    (f.a, f.b)
}

/// The end-of-line markers of `rules` on `wires` drawn under `symmetry`.
fn eol_markers(rules: &Rules, symmetry: Symmetry, wires: &[Wire]) -> Vec<Marker> {
    let mut f = Fixture::new();
    for w in wires {
        let owner = match w.net {
            Net::A => f.a,
            Net::B => f.b,
            Net::Obstruction => f.obstruction,
        };
        f.layout.add(M1, owner, symmetry.apply(w.rect), w.origin);
    }
    f.check(rules)
        .violations
        .into_iter()
        .filter(|m| m.violation == Violation::EndOfLineSpacing)
        .collect()
}

/// Whether `wires` break the end-of-line rule between nets A and B, drawn
/// in every heading of the line end and at either of its ends.
fn breaks_eol_everywhere(rules: &Rules, wires: &[Wire]) -> bool {
    Symmetry::ALL.iter().all(|&s| {
        eol_markers(rules, s, wires)
            .iter()
            .any(|m| m.owners == nets_a_and_b())
    })
}

/// Whether `wires` break the end-of-line rule between nets A and B in no
/// heading.
fn keeps_eol_everywhere(rules: &Rules, wires: &[Wire]) -> bool {
    Symmetry::ALL.iter().all(|&s| {
        eol_markers(rules, s, wires)
            .iter()
            .all(|m| m.owners != nets_a_and_b())
    })
}

/// Net A's wire 100 wide with its line end at y 0 running east, and net B's
/// wire `across` from it.
fn line_end_and_across(across: (i64, i64, i64, i64)) -> Vec<Wire> {
    vec![
        routed(Net::A, 0, 0, 100, 1000),
        routed(Net::B, across.0, across.1, across.2, across.3),
    ]
}

fn parallel_edge_rules() -> Rules {
    rules_with_eol(Some(ParallelEdge {
        spacing: Dbu::new(150),
        within: Dbu::new(130),
        two_edges: false,
    }))
}

/// Net A's line end and net B's wide wire across from it, with a narrow wire
/// of B beside the line end.
fn line_end_with_neighbour(neighbour: (i64, i64, i64, i64)) -> Vec<Wire> {
    vec![
        routed(Net::A, 0, 0, 100, 1000),
        routed(Net::B, 0, -1150, 200, -150),
        routed(Net::B, neighbour.0, neighbour.1, neighbour.2, neighbour.3),
    ]
}

#[test]
fn a_parallel_edge_arms_the_rule_up_to_its_spacing_past_the_end() {
    let rules = parallel_edge_rules();

    assert!(breaks_eol_everywhere(
        &rules,
        &line_end_with_neighbour((-249, -300, -149, 500))
    ));
    assert!(keeps_eol_everywhere(
        &rules,
        &line_end_with_neighbour((-250, -300, -150, 500))
    ));
}

#[test]
fn a_parallel_edge_arms_the_rule_down_to_the_eol_within_outside_the_line_end() {
    let rules = parallel_edge_rules();

    assert!(breaks_eol_everywhere(
        &rules,
        &line_end_with_neighbour((-249, -500, -149, -44))
    ));
    assert!(keeps_eol_everywhere(
        &rules,
        &line_end_with_neighbour((-249, -500, -149, -45))
    ));
}

#[test]
fn a_parallel_edge_arms_the_rule_up_to_its_within_inside_the_line_end() {
    let rules = parallel_edge_rules();

    assert!(breaks_eol_everywhere(
        &rules,
        &line_end_with_neighbour((-249, 129, -149, 1500))
    ));
    assert!(keeps_eol_everywhere(
        &rules,
        &line_end_with_neighbour((-249, 130, -149, 1500))
    ));
}

#[test]
fn metal_across_from_a_line_end_reaches_the_rule_spacing_in_every_heading() {
    let rules = rules_with_eol(None);

    assert!(breaks_eol_everywhere(
        &rules,
        &line_end_and_across((-100, -1199, 200, -199))
    ));
    assert!(keeps_eol_everywhere(
        &rules,
        &line_end_and_across((-100, -1200, 200, -200))
    ));
}

#[test]
fn metal_across_from_a_line_end_reaches_the_rule_within_past_its_low_side() {
    let rules = rules_with_eol(None);

    assert!(breaks_eol_everywhere(
        &rules,
        &line_end_and_across((-344, -1150, -44, -150))
    ));
    assert!(keeps_eol_everywhere(
        &rules,
        &line_end_and_across((-345, -1150, -45, -150))
    ));
}

#[test]
fn metal_across_from_a_line_end_reaches_the_rule_within_past_its_high_side() {
    let rules = rules_with_eol(None);

    assert!(breaks_eol_everywhere(
        &rules,
        &line_end_and_across((144, -1150, 444, -150))
    ));
    assert!(keeps_eol_everywhere(
        &rules,
        &line_end_and_across((145, -1150, 445, -150))
    ));
}

#[test]
fn a_line_end_facing_metal_that_only_abuts_its_side_is_marked_on_a_line() {
    let wires = line_end_and_across((100, -1150, 400, -150));

    for s in Symmetry::ALL {
        let markers = eol_markers(&rules_with_eol(None), s, &wires);

        assert_eq!(markers.len(), 1, "{s:?}");
        assert_eq!(markers[0].bbox, s.apply(rect(100, -150, 100, 0)), "{s:?}");
    }
}

#[test]
fn metal_in_the_probe_around_a_line_marked_on_a_line_keeps_the_rule_quiet() {
    let mut wires = line_end_and_across((100, -1150, 400, -150));
    wires.push(routed(Net::Obstruction, 100, -100, 110, -50));

    assert!(keeps_eol_everywhere(&rules_with_eol(None), &wires));
}

#[test]
fn a_notch_narrower_than_the_eol_width_is_no_line_end_for_metal_across_it() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    for r in [
        rect(0, 0, 300, 100),
        rect(0, 0, 100, 140),
        rect(200, 0, 300, 140),
    ] {
        f.route(M1, a, r);
    }
    f.route(M1, b, rect(60, 150, 240, 750));

    let report = f.check(&rules_with_eol(None));

    let boxes: Vec<Rect<Dbu>> = report
        .violations
        .iter()
        .filter(|m| m.violation == Violation::EndOfLineSpacing)
        .map(|m| m.bbox)
        .collect();
    assert_eq!(boxes, [rect(60, 140, 100, 150), rect(200, 140, 240, 150)]);
}

#[test]
fn a_routed_line_end_facing_fixed_metal_breaks_the_eol_rule() {
    let wires = [
        routed(Net::A, 0, 0, 100, 1000),
        fixed(Net::B, -100, -1150, 200, -150),
    ];

    assert!(breaks_eol_everywhere(&rules_with_eol(None), &wires));
}

#[test]
fn a_fixed_line_end_facing_routed_metal_breaks_the_eol_rule() {
    let wires = [
        fixed(Net::A, 0, 0, 100, 1000),
        routed(Net::B, -100, -1150, 200, -150),
    ];

    assert!(breaks_eol_everywhere(&rules_with_eol(None), &wires));
}

/// Net B's wide fixed metal across from the line end, with routed metal
/// joined to it far outside the line end's reach, so that its facing edge
/// is not fixed but no routed metal lies where it faces the line end.
fn fixed_across_with_distant_routing() -> [Wire; 2] {
    [
        fixed(Net::B, 0, -1150, 400, -150),
        routed(Net::B, 400, -250, 1200, -150),
    ]
}

#[test]
fn a_fixed_line_end_across_from_metal_with_routing_only_elsewhere_is_left_alone() {
    let mut wires = vec![fixed(Net::A, 0, 0, 100, 1000)];
    wires.extend(fixed_across_with_distant_routing());

    assert!(keeps_eol_everywhere(&rules_with_eol(None), &wires));
}

#[test]
fn routing_inside_a_fixed_line_end_does_not_make_it_a_routed_one() {
    let mut wires = vec![
        fixed(Net::A, 0, 0, 100, 1000),
        routed(Net::A, 20, 0, 80, 300),
    ];
    wires.extend(fixed_across_with_distant_routing());

    assert!(keeps_eol_everywhere(&rules_with_eol(None), &wires));
}

#[test]
fn a_routed_parallel_edge_makes_a_fixed_line_end_a_routed_one() {
    let mut wires = vec![
        fixed(Net::A, 0, 0, 100, 1000),
        routed(Net::B, -249, -300, -149, 500),
    ];
    wires.extend(fixed_across_with_distant_routing());

    assert!(breaks_eol_everywhere(&parallel_edge_rules(), &wires));
}

#[test]
fn a_fixed_parallel_edge_does_not_make_a_fixed_line_end_a_routed_one() {
    let mut wires = vec![
        fixed(Net::A, 0, 0, 100, 1000),
        fixed(Net::B, -249, -300, -149, 500),
    ];
    wires.extend(fixed_across_with_distant_routing());

    assert!(keeps_eol_everywhere(&parallel_edge_rules(), &wires));
}

#[test]
fn a_line_end_does_not_face_another_edge_of_its_own_piece() {
    let mut f = Fixture::new();
    let a = f.a;
    for r in [
        rect(0, 0, 100, 1000),
        rect(0, 0, 500, 100),
        rect(0, 900, 500, 1000),
        rect(300, 250, 400, 900),
    ] {
        f.route(M1, a, r);
    }

    assert_eq!(f.check(&rules_with_eol(None)).violations, []);
}

/// A spacing of 200 that only applies to a cut with 3 cuts, itself
/// included, within 100.
fn crowded_cut_rule() -> CutSpacing {
    CutSpacing {
        condition: CutSpacingCondition::AdjacentCuts {
            cuts: 3,
            within: Dbu::new(100),
        },
        ..cut_rule(200)
    }
}

fn cut_markers(
    f: &Fixture,
    rules: &Rules,
) -> Vec<(Violation, Rect<Dbu>, (OwnerIndex, OwnerIndex))> {
    f.check(rules)
        .violations
        .iter()
        .map(|m| (m.violation, m.bbox, m.owners))
        .collect()
}

#[test]
fn a_blockage_cut_wider_than_a_cut_checks_spacing_with_no_close_neighbours() {
    let mut f = Fixture::new();
    let (a, obstruction) = (f.a, f.obstruction);
    f.fix(V1, obstruction, rect(0, 0, 200, 200));
    f.route(V1, a, rect(350, 0, 450, 100));

    assert_eq!(
        cut_markers(&f, &rules(vec![crowded_cut_rule()])),
        [(
            Violation::CutSpacing,
            rect(200, 0, 350, 100),
            (a, obstruction)
        )]
    );
}

#[test]
fn a_blockage_cut_as_wide_as_a_cut_needs_its_close_neighbours() {
    let mut f = Fixture::new();
    let (a, obstruction) = (f.a, f.obstruction);
    f.fix(V1, obstruction, rect(0, 0, 100, 100));
    f.route(V1, a, rect(250, 0, 350, 100));

    assert_eq!(cut_markers(&f, &rules(vec![crowded_cut_rule()])), []);
}

#[test]
fn a_net_cut_wider_than_a_cut_needs_its_close_neighbours() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 200, 200));
    f.route(V1, b, rect(350, 0, 450, 100));

    assert_eq!(cut_markers(&f, &rules(vec![crowded_cut_rule()])), []);
}

/// Cuts of nets A and B side by side 50 apart, with `third` 50 past the
/// second.
fn two_close_cuts_and_a(third: Rect<Dbu>, owner: fn(&Fixture) -> OwnerIndex) -> Fixture {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(150, 0, 250, 100));
    let third_owner = owner(&f);
    f.fix(V1, third_owner, third);
    f
}

#[test]
fn a_wide_blockage_cut_close_to_a_cut_counts_as_all_the_neighbours_the_rule_asks_for() {
    let f = two_close_cuts_and_a(rect(300, 0, 500, 200), |f| f.obstruction);
    let (a, b, obstruction) = (f.a, f.b, f.obstruction);

    assert_eq!(
        cut_markers(&f, &rules(vec![crowded_cut_rule()])),
        [
            (Violation::CutSpacing, rect(100, 0, 150, 100), (a, b)),
            (
                Violation::CutSpacing,
                rect(250, 0, 300, 100),
                (b, obstruction)
            ),
        ]
    );
}

#[test]
fn a_blockage_cut_as_wide_as_a_cut_counts_as_one_neighbour() {
    let f = two_close_cuts_and_a(rect(300, 0, 400, 100), |f| f.obstruction);

    assert_eq!(cut_markers(&f, &rules(vec![crowded_cut_rule()])), []);
}

#[test]
fn a_wide_net_cut_close_to_a_cut_counts_as_one_neighbour() {
    let f = two_close_cuts_and_a(rect(300, 0, 500, 200), |f| f.a);

    assert_eq!(cut_markers(&f, &rules(vec![crowded_cut_rule()])), []);
}

#[test]
fn a_cut_fixed_beside_a_routed_one_closer_than_the_spacing_is_a_routing_violation() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(150, 0, 250, 100));

    let report = f.check(&rules(vec![cut_rule(100)]));

    assert_eq!(kinds(&report.violations), [Violation::CutSpacing]);
    assert_eq!(report.input, []);
}

#[test]
fn cuts_fixed_closer_than_the_spacing_are_an_input_defect() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(V1, a, rect(0, 0, 100, 100));
    f.fix(V1, b, rect(150, 0, 250, 100));

    let report = f.check(&rules(vec![cut_rule(100)]));

    assert_eq!(report.violations, []);
    assert_eq!(kinds(&report.input), [Violation::CutSpacing]);
}

#[test]
fn a_cut_fixed_touching_a_routed_one_is_a_routing_short() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(100, 0, 200, 100));

    let report = f.check(&rules(vec![cut_rule(100)]));

    assert_eq!(kinds(&report.violations), [Violation::CutShort]);
    assert_eq!(report.input, []);
}

#[test]
fn a_cut_fixed_beside_a_routed_one_closer_than_a_property_spacing_is_a_routing_violation() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.fix(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(150, 0, 250, 100));

    let report = f.check(&rules_with_properties(
        Vec::new(),
        vec![parallel_overlap_property(100)],
    ));

    assert_eq!(kinds(&report.violations), [Violation::CutSpacing]);
    assert_eq!(report.input, []);
}

#[test]
fn a_property_rule_is_numbered_after_the_spacing_statements() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(200, 0, 300, 100));
    let rules = rules_with_properties(
        vec![cut_rule(50)],
        vec![
            parallel_overlap_property(100),
            parallel_overlap_property(150),
        ],
    );

    let report = f.check(&rules);

    assert_eq!(kinds(&report.violations), [Violation::CutSpacing]);
    assert_eq!(report.violations[0].rule, 2);
}

/// Cuts of net A and net B 50 apart under a parallel-overlap statement, with
/// `metal` of `owner` on the layer below.
fn parallel_overlap_cuts_on_metal(owner: fn(&Fixture) -> OwnerIndex, metal: Rect<Dbu>) -> Fixture {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(V1, a, rect(0, 0, 100, 100));
    f.route(V1, b, rect(150, 0, 250, 100));
    let metal_owner = owner(&f);
    f.route(M1, metal_owner, metal);
    f
}

fn parallel_overlap_statement() -> Rules {
    rules(vec![CutSpacing {
        condition: CutSpacingCondition::ParallelOverlap,
        ..cut_rule(100)
    }])
}

#[test]
fn metal_of_the_first_cuts_owner_under_both_cuts_exempts_them_from_a_parallel_overlap_rule() {
    let f = parallel_overlap_cuts_on_metal(|f| f.a, rect(0, 0, 250, 100));

    assert_eq!(f.check(&parallel_overlap_statement()).violations, []);
}

#[test]
fn metal_of_the_second_cuts_owner_under_both_cuts_exempts_them_from_a_parallel_overlap_rule() {
    let f = parallel_overlap_cuts_on_metal(|f| f.b, rect(0, 0, 250, 100));

    assert_eq!(f.check(&parallel_overlap_statement()).violations, []);
}

#[test]
fn metal_of_another_owner_under_both_cuts_does_not_exempt_them_from_a_parallel_overlap_rule() {
    let f = parallel_overlap_cuts_on_metal(|f| f.obstruction, rect(0, 0, 250, 100));

    assert_eq!(
        kinds(&f.check(&parallel_overlap_statement()).violations),
        [Violation::CutSpacing]
    );
}

#[test]
fn metal_that_leaves_part_of_the_gap_bare_does_not_exempt_the_cuts_from_a_parallel_overlap_rule() {
    let f = parallel_overlap_cuts_on_metal(|f| f.a, rect(0, 0, 120, 100));

    assert_eq!(
        kinds(&f.check(&parallel_overlap_statement()).violations),
        [Violation::CutSpacing]
    );
}

#[test]
fn center_to_center_spacing_is_the_euclidean_distance_between_centers() {
    let rules = rules(vec![CutSpacing {
        center_to_center: true,
        ..cut_rule(200)
    }]);
    let a = rect(0, 0, 100, 100);

    assert_eq!(pair_kinds(V1, a, rect(150, 150, 250, 250), &rules), []);
    assert_eq!(
        pair_kinds(V1, a, rect(140, 140, 240, 240), &rules),
        [Violation::CutSpacing]
    );
}

fn marker_at(violation: Violation, bbox: Rect<Dbu>, owners: (OwnerIndex, OwnerIndex)) -> Marker {
    Marker::new(violation, 0, M1, bbox, owners.0, owners.1)
}

#[test]
fn violations_are_named_as_openroad_reports_them() {
    let names: Vec<&str> = [
        Violation::Short,
        Violation::CutShort,
        Violation::NonSufficientMetal,
        Violation::MetalSpacing,
        Violation::SpacingTableTw,
        Violation::SpacingRange,
        Violation::CutSpacing,
        Violation::MinArea,
        Violation::MinWidth,
        Violation::MinimumCut,
        Violation::MinHole,
        Violation::OffGrid,
        Violation::EndOfLineSpacing,
        Violation::MinStep,
        Violation::Lef58MinStep,
    ]
    .map(Violation::name)
    .to_vec();

    assert_eq!(
        names,
        [
            "Short",
            "Cut Short",
            "NS Metal",
            "Metal Spacing",
            "SpacingTableTw",
            "SpacingRange",
            "Cut Spacing",
            "Min Area",
            "Min Width",
            "Minimum Cut",
            "Min Hole",
            "Off Grid",
            "EOL Spacing",
            "Min Step",
            "Min Step",
        ]
    );
}

fn report_of(violations: Vec<Marker>, advisory: Vec<Marker>) -> DrcReport {
    DrcReport {
        violations,
        advisory,
        ..DrcReport::default()
    }
}

#[test]
fn a_report_counts_its_violations_by_kind_in_the_kinds_order() {
    let (a, b) = nets_a_and_b();
    let at = rect(0, 0, 10, 10);
    let report = report_of(
        vec![
            marker_at(Violation::EndOfLineSpacing, at, (a, b)),
            marker_at(Violation::MetalSpacing, at, (a, b)),
            marker_at(Violation::EndOfLineSpacing, rect(5, 5, 20, 20), (a, b)),
            marker_at(Violation::EndOfLineSpacing, rect(6, 6, 30, 30), (a, b)),
        ],
        Vec::new(),
    );

    assert_eq!(
        report.counts(),
        [
            (Violation::MetalSpacing, 1),
            (Violation::EndOfLineSpacing, 3)
        ]
    );
}

#[test]
fn a_report_counts_its_advisory_violations_apart() {
    let (a, b) = nets_a_and_b();
    let at = rect(0, 0, 10, 10);
    let report = report_of(
        vec![marker_at(Violation::MinStep, at, (a, b))],
        vec![
            marker_at(Violation::Lef58MinStep, at, (a, b)),
            marker_at(Violation::Lef58MinStep, rect(5, 5, 20, 20), (a, b)),
        ],
    );

    assert_eq!(report.advisory_counts(), [(Violation::Lef58MinStep, 2)]);
    assert_eq!(report.counts(), [(Violation::MinStep, 1)]);
}

#[test]
fn checking_an_area_reports_the_markers_with_their_corner_in_it() {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(M1, a, rect(0, 0, 1000, 100));
    f.route(M1, b, rect(0, 190, 1000, 290));

    let at_corner = check_area(&f.layout, &rules(Vec::new()), rect(0, 100, 1, 101));
    let elsewhere = check_area(&f.layout, &rules(Vec::new()), rect(500, 100, 501, 101));

    assert_eq!(kinds(&at_corner.violations), [Violation::MetalSpacing]);
    assert_eq!(at_corner.violations[0].bbox, rect(0, 100, 1000, 190));
    assert_eq!(elsewhere.violations, []);
}

fn non_default_rule_on(
    db: &mut NetlistDB,
    net: NetId,
    layer: LayerId,
    width: i64,
) -> NonDefaultRuleId {
    let rule = db.non_default_rules.push(NonDefaultRule {
        name: "ndr".into(),
        hard_spacing: true,
        layers: vec![NonDefaultLayer {
            layer,
            width: Dbu::new(width),
            spacing: Some(Dbu::new(width)),
            wire_extension: None,
        }],
        vias: Vec::new(),
        via_rules: Vec::new(),
        min_cuts: Vec::new(),
    });
    db.nets[net].non_default_rule = Some(rule);
    rule
}

fn tiny_fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../common/tests/fixtures")
        .join(name)
}

fn tiny_design() -> NetlistDB {
    let units = def::read_units(&tiny_fixture("tiny.def")).expect("the fixture DEF has units");
    let library =
        lef::parse_files(&[tiny_fixture("tiny.lef")], units).expect("the fixture LEF parses");
    def::parse(&tiny_fixture("tiny.def"), library).expect("the fixture DEF parses")
}

fn net(db: &NetlistDB, name: &str) -> NetId {
    db.net_id(name).expect("the fixture has the net")
}

fn cell(db: &NetlistDB, name: &str) -> CellId {
    db.cell_id(name).expect("the fixture has the cell")
}

const M2_LAYER: LayerId = LayerId(1);
const M3_LAYER: LayerId = LayerId(2);

fn routed_net_wires(db: &mut NetlistDB, name: &str, wires: Vec<RouteElem>) -> NetId {
    let id = db.net_or_insert(name);
    db.nets[id].routing = wires;
    id
}

/// A vertical wire on M2 along `x` from `y0` to `y1`, in segments of 500.
fn vertical_segments(x: i64, y0: i64, y1: i64) -> Vec<RouteElem> {
    (y0..y1)
        .step_by(500)
        .map(|y| RouteElem::Wire {
            layer: M2_LAYER,
            from: Point::new(Dbu::new(x), Dbu::new(y)),
            to: Point::new(Dbu::new(x), Dbu::new((y + 500).min(y1))),
            extensions: (None, None),
        })
        .collect()
}

fn empty_design() -> NetlistDB {
    let mut db = tiny_design();
    db.cells = IdVec::new();
    db.nets = IdVec::new();
    db.pins = IdVec::new();
    db.io_pins = IdVec::new();
    db.special_nets = IdVec::new();
    db
}

#[test]
fn checking_a_design_finds_a_violation_along_a_long_wire_of_many_segments_whole() {
    let mut db = empty_design();
    let _ = routed_net_wires(&mut db, "n", vertical_segments(1000, 1000, 8000));
    let _ = routed_net_wires(&mut db, "m", vertical_segments(1150, 1000, 8000));

    let report = check_drc(&db);

    assert_eq!(kinds(&report.violations), [Violation::MetalSpacing]);
    assert_eq!(report.violations[0].bbox, rect(1050, 950, 1100, 8050));
}

#[test]
fn checking_a_design_with_a_nondefault_rule_finds_a_narrowed_violation_whole() {
    let mut db = empty_design();
    let n = routed_net_wires(&mut db, "n", Vec::new());
    let mut wires = vertical_segments(1000, 1000, 8000);
    wires.extend(vertical_segments(1190, 1000, 8000));
    db.nets[n].routing = wires;
    let rule = non_default_rule_on(&mut db, n, M2_LAYER, 200);
    db.non_default_rules[rule].hard_spacing = false;

    let report = check_drc(&db);

    assert_eq!(kinds(&report.violations), [Violation::MetalSpacing]);
    assert_eq!(report.violations[0].bbox, rect(1050, 950, 1140, 8050));
}

#[test]
fn a_design_without_nondefault_rules_has_no_rule_wiring() {
    let mut db = empty_design();
    let _ = routed_net_wires(&mut db, "n", vertical_segments(1000, 1000, 2000));

    assert!(!has_rule_wiring(&db));
}

#[test]
fn a_routed_net_under_a_nondefault_rule_has_rule_wiring() {
    let mut db = empty_design();
    let n = routed_net_wires(&mut db, "n", vertical_segments(1000, 1000, 2000));
    let _ = non_default_rule_on(&mut db, n, M2_LAYER, 200);

    assert!(has_rule_wiring(&db));
}

#[test]
fn an_unrouted_net_under_a_nondefault_rule_has_no_rule_wiring() {
    let mut db = empty_design();
    let n = routed_net_wires(&mut db, "n", Vec::new());
    let _ = non_default_rule_on(&mut db, n, M2_LAYER, 200);

    assert!(!has_rule_wiring(&db));
}

/// A layout with net 0 under a nondefault rule and net 1 under none.
fn layout_with_one_ruled_net() -> (Layout, OwnerIndex, OwnerIndex) {
    let mut f = Fixture::new();
    let (ruled, plain) = (f.a, f.b);
    f.layout.set_non_default_rule(ruled, NonDefaultRuleId(0));
    (f.layout, ruled, plain)
}

#[test]
fn narrowing_makes_a_line_end_of_a_ruled_net_but_not_of_unruled_nets() {
    let (layout, ruled, plain) = layout_with_one_ruled_net();
    let violation = Violation::EndOfLineSpacing;
    let at = rect(0, 0, 10, 10);

    assert!(narrowing_rule_wiring_makes(
        &marker_at(violation, at, (ruled, plain)),
        &layout
    ));
    assert!(!narrowing_rule_wiring_makes(
        &marker_at(violation, at, (plain, plain)),
        &layout
    ));
}

#[test]
fn narrowing_makes_spacing_between_pieces_of_one_ruled_net_only() {
    let (layout, ruled, plain) = layout_with_one_ruled_net();
    let at = rect(0, 0, 10, 10);
    let spacing = |owners| {
        narrowing_rule_wiring_makes(&marker_at(Violation::MetalSpacing, at, owners), &layout)
    };

    assert!(spacing((ruled, ruled)));
    assert!(!spacing((plain, plain)));
    assert!(!spacing((ruled, plain)));
}

#[test]
fn narrowing_makes_no_short_or_cut_violation() {
    let (layout, ruled, _) = layout_with_one_ruled_net();
    let at = rect(0, 0, 10, 10);

    for violation in [
        Violation::Short,
        Violation::CutShort,
        Violation::CutSpacing,
        Violation::MinimumCut,
    ] {
        assert!(
            !narrowing_rule_wiring_makes(&marker_at(violation, at, (ruled, ruled)), &layout),
            "{violation:?}"
        );
    }
}

#[test]
fn thin_pieces_lists_the_routed_pieces_below_the_minimum_area() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 100, 150));
    f.route(M1, a, rect(1000, 0, 1100, 300));

    let thin = thin_pieces(&f.layout, &rules_with_min_area(20_000), None);

    assert_eq!(
        thin,
        [ThinPiece {
            owner: a,
            layer: LayerId(0),
            bbox: rect(0, 0, 100, 150),
            area: DbuArea(15_000),
            min_area: DbuArea(20_000),
        }]
    );
}

#[test]
fn thin_pieces_in_a_window_leaves_out_a_piece_that_reaches_its_edge() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 100, 150));
    f.route(M1, a, rect(500, 0, 600, 150));
    let window = rect(-50, -50, 600, 400);

    let thin = thin_pieces(&f.layout, &rules_with_min_area(20_000), Some(window));

    assert_eq!(
        thin.iter().map(|t| t.bbox).collect::<Vec<_>>(),
        [rect(0, 0, 100, 150)]
    );
}

#[test]
fn checking_in_a_window_leaves_out_a_piece_that_reaches_its_edge() {
    let mut f = Fixture::new();
    let a = f.a;
    f.route(M1, a, rect(0, 0, 100, 150));
    let core = rect(-1000, -1000, 1000, 1000);
    let window = rect(0, -50, 400, 400);

    let found = check_window(&f.layout, &rules_with_min_area(20_000), core, window);

    assert_eq!(found.violations, []);
}

/// A fixed pin of a net under two obstructions of a cell, and a routed wire
/// ending against the pin between them; the owners are created in the
/// order given.
fn pin_under_obstructions(obstruction_first: bool) -> DrcReport {
    let mut layout = Layout::new(2, 1);
    let (obstruction, net) = if obstruction_first {
        let o = layout.owner(Owner::Obstruction(CellId(0)), false);
        (o, layout.owner(Owner::Net(NetId(0)), false))
    } else {
        let n = layout.owner(Owner::Net(NetId(0)), false);
        (layout.owner(Owner::Obstruction(CellId(0)), false), n)
    };
    layout.add(M1, net, rect(200, 300, 550, 550), Origin::Input);
    layout.add(M1, obstruction, rect(200, 100, 500, 300), Origin::Input);
    layout.add(M1, obstruction, rect(300, 350, 650, 600), Origin::Input);
    layout.add(M1, net, rect(0, 250, 300, 300), Origin::Routing);
    check(&layout, &rules(Vec::new()))
}

#[test]
fn a_violation_inside_a_pin_under_its_obstruction_is_a_pin_obstruction_marker() {
    for obstruction_first in [false, true] {
        let report = pin_under_obstructions(obstruction_first);

        assert!(
            !kinds(&report.violations).contains(&Violation::MetalSpacing),
            "{obstruction_first}"
        );
        assert_eq!(
            kinds(&report.pin_obstruction),
            [Violation::MetalSpacing],
            "{obstruction_first}"
        );
        assert_eq!(report.pin_obstruction[0].bbox, rect(300, 300, 300, 350));
    }
}

fn point(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

#[test]
fn a_rectangle_without_width_is_kept_and_one_turned_inside_out_is_ignored() {
    let mut layout = Layout::new(1, 1);
    let owner = layout.owner(Owner::Net(NetId(0)), false);

    layout.add(
        M1,
        owner,
        Rect::new(point(10, 0), point(10, 100)),
        Origin::Routing,
    );
    layout.add(
        M1,
        owner,
        Rect::new(point(0, 0), point(100, 0)),
        Origin::Routing,
    );
    layout.add(
        M1,
        owner,
        Rect::new(point(10, 0), point(0, 100)),
        Origin::Routing,
    );
    layout.add(
        M1,
        owner,
        Rect::new(point(0, 10), point(100, 0)),
        Origin::Routing,
    );

    assert_eq!(
        layout
            .metal(LayerId(0))
            .iter()
            .map(|s| s.rect)
            .collect::<Vec<_>>(),
        [rect(10, 0, 10, 100), rect(0, 0, 100, 0)]
    );
}

#[test]
fn a_layout_counts_its_layers() {
    let layout = Layout::new(3, 2);

    assert_eq!((layout.metal_layers(), layout.cut_layers()), (3, 2));
}

#[test]
fn only_owners_added_as_supplies_carry_power_or_ground() {
    let mut layout = Layout::new(1, 1);
    let supply = layout.owner(Owner::FloatingPower, true);
    let signal = layout.owner(Owner::Net(NetId(0)), false);

    assert!(layout.is_supply(supply));
    assert!(!layout.is_supply(signal));
}

#[test]
fn owners_are_named_by_what_they_are_in_the_design() {
    let db = tiny_design();
    let u7 = cell(&db, "u7");
    let y = db.cell_macro(u7).pin_id("Y").expect("INV has a pin Y");

    let names: Vec<String> = [
        Owner::Net(net(&db, "n1")),
        Owner::SpecialNet(SpecialNetId(0)),
        Owner::Obstruction(u7),
        Owner::Blockage(BlockageId(3)),
        Owner::UnconnectedPin(u7, y),
        Owner::FloatingPower,
        Owner::FloatingGround,
    ]
    .iter()
    .map(|o| o.name(&db))
    .collect();

    assert_eq!(
        names,
        [
            "n1",
            "VDD",
            "u7 (obstruction)",
            "blockage 3",
            "u7/Y",
            "floating power",
            "floating ground"
        ]
    );
}

fn tiny_layout() -> (NetlistDB, Layout) {
    let db = tiny_design();
    let layout = Layout::from_db(&db);
    (db, layout)
}

fn shape_with_rect(layout: &Layout, layer: LayerId, r: Rect<Dbu>) -> Option<LayoutShape> {
    layout.metal(layer).iter().find(|s| s.rect == r).copied()
}

#[test]
fn a_placed_cell_has_an_obstruction_owner_only_when_its_macro_has_obstructions() {
    let (db, layout) = tiny_layout();

    assert!(
        layout
            .owner_index(Owner::Obstruction(cell(&db, "u1")))
            .is_some()
    );
    assert!(
        layout
            .owner_index(Owner::Obstruction(cell(&db, "u3")))
            .is_none()
    );
}

#[test]
fn an_unplaced_cell_adds_no_shapes() {
    let (db, layout) = tiny_layout();

    assert!(
        layout
            .owner_index(Owner::Obstruction(cell(&db, "u5")))
            .is_none()
    );
}

#[test]
fn a_placed_cell_puts_its_obstruction_in_the_layout_as_a_placement_shape() {
    let (db, layout) = tiny_layout();
    let owner = layout
        .owner_index(Owner::Obstruction(cell(&db, "u1")))
        .expect("u1 is placed with obstructions");

    let shape = shape_with_rect(&layout, LayerId(0), rect(350, 800, 450, 1200));

    assert_eq!(
        shape,
        Some(LayoutShape {
            owner,
            rect: rect(350, 800, 450, 1200),
            origin: Origin::Placement,
        })
    );
}

#[test]
fn a_placed_io_pin_is_in_the_layout_as_a_fixed_shape_of_its_net() {
    let (db, layout) = tiny_layout();
    let owner = layout
        .owner_index(Owner::Net(net(&db, "in")))
        .expect("the net in has an owner");

    let shape = shape_with_rect(&layout, M2_LAYER, rect(1050, 5800, 1150, 6000));

    assert_eq!(
        shape.map(|s| (s.owner, s.origin)),
        Some((owner, Origin::Input))
    );
}

#[test]
fn an_unplaced_io_pin_adds_no_shapes() {
    let mut db = tiny_design();
    db.io_pins[IoPinId(0)].status = PlacementStatus::Unplaced;

    let layout = Layout::from_db(&db);

    assert_eq!(
        shape_with_rect(&layout, M2_LAYER, rect(1050, 5800, 1150, 6000)),
        None
    );
}

#[test]
fn an_io_pin_cell_can_move_is_a_placement_shape() {
    let (db, layout) = tiny_layout();
    let owner = layout
        .owner_index(Owner::Net(net(&db, "out")))
        .expect("the net out has an owner");

    let shape = layout
        .metal(M3_LAYER)
        .iter()
        .find(|s| s.rect == rect(5900, 3000, 6100, 3200));

    assert_eq!(
        shape.map(|s| (s.owner, s.origin)),
        Some((owner, Origin::Placement))
    );
}

#[test]
fn a_power_pin_named_by_a_special_net_belongs_to_it() {
    let (_, layout) = tiny_layout();
    let special = layout
        .owner_index(Owner::SpecialNet(SpecialNetId(0)))
        .expect("the special net VDD has an owner");

    let vdd_pin_of_u1 = shape_with_rect(&layout, LayerId(0), rect(200, 1900, 600, 2100));

    assert_eq!(vdd_pin_of_u1.map(|s| s.owner), Some(special));
    assert!(layout.is_supply(special));
}

#[test]
fn a_ground_pin_on_no_net_belongs_to_floating_ground() {
    let (_, layout) = tiny_layout();
    let ground = layout
        .owner_index(Owner::FloatingGround)
        .expect("the unconnected VSS pins share an owner");

    let vss_pin_of_u1 = shape_with_rect(&layout, LayerId(0), rect(200, -100, 600, 100));

    assert_eq!(vss_pin_of_u1.map(|s| s.owner), Some(ground));
    assert!(layout.is_supply(ground));
}

#[test]
fn a_power_pin_on_no_net_or_special_net_belongs_to_floating_power() {
    let mut db = tiny_design();
    db.special_nets[SpecialNetId(0)].connections.clear();

    let layout = Layout::from_db(&db);
    let power = layout
        .owner_index(Owner::FloatingPower)
        .expect("the unconnected VDD pins share an owner");

    let vdd_pin_of_u1 = shape_with_rect(&layout, LayerId(0), rect(200, 1900, 600, 2100));
    assert_eq!(vdd_pin_of_u1.map(|s| s.owner), Some(power));
    assert!(layout.is_supply(power));
}

#[test]
fn a_signal_pin_on_no_net_is_its_own_owner() {
    let (db, layout) = tiny_layout();
    let u7 = cell(&db, "u7");
    let y = db.cell_macro(u7).pin_id("Y").expect("INV has a pin Y");

    let owner = layout.owner_index(Owner::UnconnectedPin(u7, y));

    assert!(owner.is_some_and(|o| !layout.is_supply(o)));
}

#[test]
fn a_power_net_is_a_supply_owner_and_a_signal_net_is_not() {
    let mut db = tiny_design();
    let input = net(&db, "in");
    db.nets[input].signal_use = SignalUse::Power;

    let layout = Layout::from_db(&db);

    let owner = |name: &str| {
        layout
            .owner_index(Owner::Net(net(&db, name)))
            .expect("owned")
    };
    assert!(layout.is_supply(owner("in")));
    assert!(!layout.is_supply(owner("n1")));
}

#[test]
fn a_net_of_a_power_special_nets_name_is_a_supply_owner() {
    let mut db = tiny_design();
    let special_name = db.special_nets[SpecialNetId(0)].name.clone();
    let input = net(&db, "in");
    db.nets[input].name = special_name;

    let layout = Layout::from_db(&db);

    let renamed = layout.owner_index(Owner::Net(input)).expect("owned");
    assert!(layout.is_supply(renamed));
}

#[test]
fn a_special_net_sharing_its_name_with_a_signal_net_gives_that_net_its_supply() {
    let mut db = tiny_design();
    let vdd = db.net_or_insert("VDD");

    let layout = Layout::from_db(&db);

    let owner = layout.owner_index(Owner::Net(vdd)).expect("owned");
    assert!(layout.is_supply(owner));
    assert!(
        layout
            .owner_index(Owner::SpecialNet(SpecialNetId(0)))
            .is_none()
    );
}

#[test]
fn reading_a_ruled_nets_wiring_as_openroad_does_keeps_every_cut() {
    let mut db = tiny_design();
    let n3 = net(&db, "n3");
    let _ = non_default_rule_on(&mut db, n3, M2_LAYER, 200);
    let layout = Layout::from_db(&db);

    let view = Layout::as_openroad_reads(&db, &layout);

    assert_eq!(view.cut(CutLayerId(0)), layout.cut(CutLayerId(0)));
    assert_eq!(
        view.cut(CutLayerId(1)).len(),
        layout.cut(CutLayerId(1)).len()
    );
}

#[test]
fn the_reach_of_an_end_of_line_rule_adds_its_spacing_within_and_parallel_edge_distances() {
    let parallel = ParallelEdge {
        spacing: Dbu::new(150),
        within: Dbu::new(130),
        two_edges: false,
    };

    assert_eq!(rules_with_eol(Some(parallel)).max_reach(), Dbu::new(525));
}

#[test]
fn the_reach_of_an_adjacent_cuts_rule_is_its_within_when_that_is_more_than_the_spacing() {
    let rule = CutSpacing {
        condition: CutSpacingCondition::AdjacentCuts {
            cuts: 2,
            within: Dbu::new(300),
        },
        ..cut_rule(150)
    };

    assert_eq!(rules(vec![rule]).max_reach(), Dbu::new(300));
}

#[test]
fn the_rules_of_a_design_keep_its_nondefault_rules_hard_spacing() {
    let mut db = tiny_design();
    let n3 = net(&db, "n3");
    let rule = non_default_rule_on(&mut db, n3, M2_LAYER, 250);

    let rules = Rules::from_design(&db);

    assert_eq!(rules.hard_spacing(Some(rule), M2_LAYER), Dbu::new(250));
}

#[test]
fn checking_no_shapes_reports_the_owners_the_layout_has() {
    let mut layout = Layout::new(2, 1);
    let _ = layout.owner(Owner::Net(NetId(0)), false);

    let report = check_tiled(&layout, &rules(Vec::new()), Dbu::new(1000), Dbu::new(500));

    assert_eq!(report.owners, *layout.owners());
    assert_eq!(report.violations, []);
}

/// Two nets 90 apart along 1000, the marker of their spacing running from
/// `(0, 100)` to `(1000, 190)`, in a layout whose shapes are all in the
/// window `(-5000, -5000)` to `(5000, 5000)`.
fn spacing_violation_layout() -> Layout {
    let mut f = Fixture::new();
    let (a, b) = (f.a, f.b);
    f.route(M1, a, rect(0, 0, 1000, 100));
    f.route(M1, b, rect(0, 190, 1000, 290));
    f.layout
}

fn marker_found_in_core(core: Rect<Dbu>) -> bool {
    let window = rect(-5000, -5000, 5000, 5000);

    let found = check_window(
        &spacing_violation_layout(),
        &rules(Vec::new()),
        core,
        window,
    );

    kinds(&found.violations) == [Violation::MetalSpacing]
}

#[test]
fn a_core_owns_a_marker_whose_corner_is_on_its_lower_and_left_edges() {
    assert!(marker_found_in_core(rect(0, 100, 500, 500)));
}

#[test]
fn a_core_does_not_own_a_marker_whose_corner_is_on_its_right_edge() {
    assert!(!marker_found_in_core(rect(-500, 100, 0, 500)));
}

#[test]
fn a_core_does_not_own_a_marker_whose_corner_is_on_its_top_edge() {
    assert!(!marker_found_in_core(rect(0, -400, 500, 100)));
}

#[test]
fn a_core_does_not_own_a_marker_whose_corner_is_left_of_it() {
    assert!(!marker_found_in_core(rect(1, 100, 500, 500)));
}

#[test]
fn a_core_does_not_own_a_marker_whose_corner_is_below_it() {
    assert!(!marker_found_in_core(rect(0, 101, 500, 500)));
}

#[test]
fn tiles_far_from_the_origin_find_what_checking_whole_finds() {
    let (layout, rules) = scattered_scene();
    let mut moved = Layout::with_owners_of(&layout);
    let offset = point(7000, 3000);
    for l in layout.metal_layer_ids() {
        for s in layout.metal(l) {
            let shape = LayoutShape {
                rect: s.rect.translate(offset),
                ..*s
            };
            moved.add_shape(LayerRef::Routing(l), &shape);
        }
    }
    for c in layout.cut_layer_ids() {
        for s in layout.cut(c) {
            let shape = LayoutShape {
                rect: s.rect.translate(offset),
                ..*s
            };
            moved.add_shape(LayerRef::Cut(c), &shape);
        }
    }

    let tiled = check_tiled(&moved, &rules, Dbu::new(1000), Dbu::new(500));

    assert_eq!(tiled, check(&moved, &rules));
}

#[test]
fn a_placed_cell_in_a_marker_is_a_placement_defect_whichever_owner_it_belongs_to() {
    for placed_is_second in [false, true] {
        let mut f = Fixture::new();
        let (a, b) = (f.a, f.b);
        let (placed, input) = if placed_is_second { (b, a) } else { (a, b) };
        f.place(M1, placed, rect(0, 0, 1000, 100));
        f.fix(M1, input, rect(500, 0, 1500, 100));

        let report = f.check(&rules(Vec::new()));

        assert_eq!(
            kinds(&report.placement),
            [Violation::Short],
            "{placed_is_second}"
        );
        assert_eq!(report.input, [], "{placed_is_second}");
    }
}

#[test]
fn a_fixed_piece_with_a_hole_is_not_checked_for_the_hole_by_routing_elsewhere_in_it() {
    let mut f = Fixture::new();
    let a = f.a;
    for r in [
        rect(0, 0, 1000, 200),
        rect(0, 800, 1000, 1000),
        rect(0, 0, 200, 1000),
        rect(800, 0, 1000, 1000),
    ] {
        f.fix(M1, a, r);
    }
    f.route(M1, a, rect(450, 450, 550, 550));
    let mut rules = rules(Vec::new());
    rules.metal[LayerId(0)].min_enclosed_areas = vec![DbuArea(400_000)];

    assert_eq!(f.check(&rules).violations, []);
}
