use super::*;
use std::fmt::Write;

fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

fn connected_to_root(tree: &SteinerTree) -> bool {
    (0..tree.nodes.len()).all(|mut n| {
        for _ in 0..tree.nodes.len() {
            if n == tree.root {
                return true;
            }
            let Some(parent) = tree.parent[n] else {
                return false;
            };
            n = parent;
        }
        false
    })
}

#[test]
fn a_straight_branch_across_another_branchs_box_fails_the_check() {
    // The branch from (-5, 5) to (15, 5) runs through the box of the
    // branch from (0, 0) to (10, 10) and shares none of its corners.
    let tree = SteinerTree {
        nodes: vec![pt(0, 0), pt(10, 10), pt(-5, 5), pt(15, 5)],
        parent: vec![None, Some(0), Some(0), Some(2)],
        terminals: 4,
        root: 0,
    };

    assert!(!is_valid(&tree));
}

#[test]
fn two_points_make_one_edge() {
    let tree = prim_dijkstra(&[pt(0, 0), pt(10, 5)], 0, DEFAULT_ALPHA);
    assert_eq!(tree.nodes.len(), 2);
    assert_eq!(tree.parent, vec![None, Some(0)]);
    assert_eq!(tree.length(), Dbu::new(15));
}

#[test]
fn an_l_of_three_points_gets_a_steiner_point() {
    let tree = prim_dijkstra(&[pt(0, 0), pt(10, 10), pt(10, -10)], 0, DEFAULT_ALPHA);
    assert_eq!(tree.terminals, 3);
    assert_eq!(tree.nodes.len(), 4);
    assert_eq!(tree.nodes[3], pt(10, 0));
    assert_eq!(tree.length(), Dbu::new(30));
    assert!(connected_to_root(&tree));
    assert!(is_valid(&tree));
}

#[test]
fn a_star_splits_into_degree_three_nodes() {
    let points = [pt(0, 0), pt(5, 0), pt(-5, 0), pt(0, 5), pt(0, -5)];
    let tree = prim_dijkstra(&points, 0, DEFAULT_ALPHA);
    assert!(tree.children().iter().all(|c| c.len() <= 3));
    assert!(connected_to_root(&tree));
    assert_eq!(tree.length(), Dbu::new(20));
}

#[test]
fn duplicate_points_stay_connected() {
    let points = [pt(3, 3), pt(3, 3), pt(9, 1), pt(9, 1), pt(0, 7)];
    let tree = prim_dijkstra(&points, 0, DEFAULT_ALPHA);
    assert!(connected_to_root(&tree));
    assert_eq!(tree.terminals, 5);
}

#[test]
fn a_wider_spread_is_valid_and_shorter_than_a_star() {
    let points: Vec<Point<Dbu>> = (0..12).map(|i| pt((i * 37) % 101, (i * 53) % 89)).collect();
    let tree = prim_dijkstra(&points, 0, DEFAULT_ALPHA);
    assert!(connected_to_root(&tree));
    assert!(is_valid(&tree));
    let star: Dbu = points[1..].iter().map(|&p| p.manhattan(points[0])).sum();
    assert!(tree.length() < star);
}

/// A linear congruential generator, for seeded inputs.
struct Lcg(u64);

impl Lcg {
    fn below(&mut self, bound: i64) -> i64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        i64::try_from(self.0 >> 33).unwrap() % bound
    }
}

/// Point sets of 2 to 16 points on grids from 4 to 200 wide, the narrow
/// ones full of ties and duplicates, each with a driver and an alpha.
fn seeded_cases() -> Vec<(Vec<Point<Dbu>>, usize, f64)> {
    const ALPHAS: [f64; 4] = [DEFAULT_ALPHA, 0.0, 1.0, 0.6];
    const SPANS: [i64; 5] = [4, 8, 30, 200, 6];
    let mut rng = Lcg(7);
    (0..400)
        .map(|case| {
            let count = 2 + case % 15;
            let span = SPANS[case % SPANS.len()];
            let points: Vec<Point<Dbu>> = (0..count)
                .map(|_| pt(rng.below(span), rng.below(span)))
                .collect();
            let driver = usize::try_from(rng.below(i64::try_from(count).unwrap())).unwrap();
            (points, driver, ALPHAS[case % ALPHAS.len()])
        })
        .collect()
}

fn describe(points: &[Point<Dbu>]) -> String {
    points
        .iter()
        .map(|p| format!("({},{})", p.x.get(), p.y.get()))
        .collect::<Vec<_>>()
        .join(" ")
}

#[test]
fn the_trees_of_seeded_point_sets_are_connected_of_degree_three_and_as_they_were() {
    let mut text = String::new();
    for (case, (points, driver, alpha)) in seeded_cases().into_iter().enumerate() {
        let tree = prim_dijkstra(&points, driver, alpha);

        assert!(connected_to_root(&tree), "case {case}");
        assert!(tree.children().iter().all(|c| c.len() <= 3), "case {case}");
        assert_eq!(tree.nodes[..points.len()], points, "case {case}");
        assert_eq!((tree.terminals, tree.root), (points.len(), driver));
        write!(
            text,
            "case {case} driver {driver} alpha {alpha}\n  points {}\n  nodes {}\n  parents {:?}\n  \
             length {} valid {}\n",
            describe(&points),
            describe(&tree.nodes),
            tree.parent,
            tree.length().get(),
            is_valid(&tree),
        )
        .unwrap();
    }

    pare_common::testing::assert_golden(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden"),
        "c_steiner_trees.txt",
        &text,
    );
}

#[test]
fn the_nearest_neighbors_of_seeded_point_sets_are_as_they_were() {
    let mut text = String::new();
    for (case, (points, _, _)) in seeded_cases().into_iter().enumerate() {
        write!(
            text,
            "case {case} points {}\n  neighbors {:?}\n",
            describe(&points),
            nearest_neighbors(&points)
        )
        .unwrap();
    }

    pare_common::testing::assert_golden(
        &std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden"),
        "c_steiner_neighbors.txt",
        &text,
    );
}

/// A tree of `terminals` nodes whose first four cross: the branch from
/// (-5, 5) to (15, 5) runs through the box of the one from (0, 0) to
/// (10, 10), and the others sit at the root with branches of no length.
fn crossing_tree(terminals: usize) -> SteinerTree {
    let mut nodes = vec![pt(0, 0), pt(10, 10), pt(-5, 5), pt(15, 5)];
    let mut parent = vec![None, Some(0), Some(0), Some(2)];
    while nodes.len() < terminals {
        nodes.push(pt(0, 0));
        parent.push(Some(0));
    }
    SteinerTree {
        nodes,
        parent,
        terminals,
        root: 0,
    }
}

#[test]
fn trees_of_over_a_hundred_terminals_pass_unchecked_and_smaller_ones_are_checked() {
    assert!(!is_valid(&crossing_tree(100)));
    assert!(is_valid(&crossing_tree(101)));
}

#[test]
fn a_branch_of_no_length_inside_another_branchs_box_does_not_fail_the_check() {
    let tree = SteinerTree {
        nodes: vec![pt(0, 0), pt(10, 10), pt(5, 5), pt(5, 5)],
        parent: vec![None, Some(0), Some(1), Some(2)],
        terminals: 4,
        root: 0,
    };

    assert!(is_valid(&tree));
}
