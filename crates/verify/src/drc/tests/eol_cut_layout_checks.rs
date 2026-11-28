//! Tests of the end-of-line and cut checks, the layout a design gives,
//! tiling and the rules' reach.

use super::*;

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
