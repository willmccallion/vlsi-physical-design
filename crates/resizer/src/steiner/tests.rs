use super::*;

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
