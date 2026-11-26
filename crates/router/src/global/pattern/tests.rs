use super::super::Tracks;
use super::super::grid::Usage;
use super::super::tests::{COSTS, at, grid, net};
use super::*;

/// The 10x10 grid of [`grid`] with a third, horizontal, layer on top.
fn three_layers() -> GlobalGrid {
    let edges = 3 * 10 * 10;
    GlobalGrid {
        directions: vec![
            Direction::Horizontal,
            Direction::Vertical,
            Direction::Horizontal,
        ],
        capacity: vec![Tracks::whole(4); edges],
        blocked: vec![false; edges],
        via_blocked: vec![false; edges],
        via_loss: vec![Tracks::ZERO; 3],
        usage: Usage::zeroed(edges),
        history: vec![0.0; edges],
        ..grid()
    }
}

/// Fills the edges of `layer` along row `y` from column `x0` to `x1`.
fn crowd_row(grid: &mut GlobalGrid, layer: usize, y: u16, x0: u16, x1: u16) {
    for x in x0..x1 {
        grid.usage.set(grid.edge(at(layer, x, y)), Tracks::whole(8));
    }
}

fn cells(paths: &[Vec<GCell>]) -> Vec<GCell> {
    paths.iter().flatten().copied().collect()
}

#[test]
fn a_straight_edge_takes_the_less_used_layer_of_its_direction() {
    let mut grid = three_layers();
    crowd_row(&mut grid, 1, 5, 1, 8);
    let terminals = vec![at(2, 1, 5), at(2, 8, 5)];

    let paths = route_pattern(&grid, &net(terminals), COSTS);

    let along: Vec<GCell> = cells(&paths).into_iter().filter(|c| c.x == 4).collect();
    assert_eq!(along, [at(3, 4, 5)]);
}

#[test]
fn an_l_takes_the_bend_off_the_crowded_row() {
    let mut grid = grid();
    crowd_row(&mut grid, 1, 1, 1, 6);
    let terminals = vec![at(1, 1, 1), at(1, 6, 6)];

    let paths = route_pattern(&grid, &net(terminals), COSTS);

    let cells = cells(&paths);
    assert!(cells.contains(&at(2, 1, 6)), "{paths:?}");
    assert!(!cells.iter().any(|c| (c.x, c.y) == (6, 1)), "{paths:?}");
}

#[test]
fn a_bend_with_no_room_for_a_via_moves_to_the_other_corner() {
    for (blocked, other) in [((6, 1), (1, 6)), ((1, 6), (6, 1))] {
        let mut grid = grid();
        for layer in 1..=2 {
            let c = grid.edge(at(layer, blocked.0, blocked.1));
            grid.via_blocked[c] = true;
        }
        let terminals = vec![at(1, 1, 1), at(1, 6, 6)];

        let paths = route_pattern(&grid, &net(terminals), COSTS);

        let spots: Vec<(u16, u16)> = cells(&paths).iter().map(|c| (c.x, c.y)).collect();
        assert!(!spots.contains(&blocked), "{paths:?}");
        assert!(spots.contains(&other), "{paths:?}");
    }
}

#[test]
fn a_node_s_via_stack_spans_its_pins_layers() {
    let terminals = vec![at(1, 3, 3), at(2, 3, 3), at(1, 7, 3)];

    let paths = route_pattern(&grid(), &net(terminals.clone()), COSTS);

    let cells = cells(&paths);
    assert!(terminals.iter().all(|t| cells.contains(t)), "{paths:?}");
}

#[test]
fn the_paths_form_one_tree_through_every_terminal() {
    let terminals = vec![at(1, 1, 1), at(1, 8, 2), at(2, 4, 8), at(1, 6, 5)];

    let paths = route_pattern(&grid(), &net(terminals.clone()), COSTS);

    let mut tree: Vec<GCell> = Vec::new();
    for (k, path) in paths.iter().enumerate() {
        assert!(
            k == 0 || path.first().is_some_and(|c| tree.contains(c)),
            "path {k} starts off the tree: {paths:?}"
        );
        tree.extend(path);
    }
    assert!(terminals.iter().all(|t| tree.contains(t)), "{paths:?}");
}
