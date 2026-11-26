use super::*;

fn length(tree: &SteinerTree) -> u32 {
    tree.order
        .iter()
        .map(|&v| distance(tree.nodes[v], tree.nodes[tree.parent[v]]))
        .sum()
}

#[test]
fn three_pins_meet_at_their_median() {
    let tree = steiner_tree(&[(0, 0), (10, 5), (4, 10)]);

    assert_eq!(length(&tree), 20);
    assert!(tree.nodes.contains(&(4, 5)));
}

#[test]
fn a_cross_of_four_pins_shares_its_centre() {
    let tree = steiner_tree(&[(5, 0), (5, 10), (0, 5), (10, 5)]);

    assert_eq!(length(&tree), 20);
    assert_eq!(tree.order.len(), tree.nodes.len());
}

#[test]
fn every_node_is_reached_after_its_parent() {
    let spots: Vec<Spot> = (0..60u16).map(|i| (i * 7 % 23, i * 11 % 31)).collect();
    let mut unique = spots;
    unique.sort_unstable();
    unique.dedup();

    let tree = steiner_tree(&unique);

    let mut placed = vec![false; tree.nodes.len()];
    for &v in &tree.order {
        assert!(v == 0 || placed[tree.parent[v]], "{v} before its parent");
        placed[v] = true;
    }
    assert!(placed.iter().all(|&p| p));
}
