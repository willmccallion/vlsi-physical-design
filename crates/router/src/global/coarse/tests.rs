use super::super::grid::Usage;
use super::super::{GlobalGrid, Tracks};
use super::*;
use pare_common::config::CapacityReserve;
use pare_common::db::indices::NetId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;

#[test]
fn each_node_lies_one_below_its_parent_and_roots_at_zero() {
    // Two trees: 0 <- 2 <- 1 <- 4, and 3 alone.
    let parent = [0, 2, 0, 3, 1];

    assert_eq!(depths(&parent), [0, 2, 1, 0, 3]);
}

/// A `nx` by `ny` grid with a horizontal layer 1 over a vertical layer
/// 2, every edge holding 4 tracks.
fn grid(nx: u16, ny: u16) -> GlobalGrid {
    let edges = 2 * usize::from(nx) * usize::from(ny);
    GlobalGrid {
        origin: Point::new(Dbu::new(0), Dbu::new(0)),
        gcell: Dbu::new(100),
        nx,
        ny,
        min_layer: LayerId::new(1),
        directions: vec![Direction::Horizontal, Direction::Vertical],
        clock_layers: LayerId::new(1)..=LayerId::new(2),
        reserve: CapacityReserve::NONE,
        capacity: vec![Tracks::whole(4); edges],
        blocked: vec![false; edges],
        via_blocked: vec![false; edges],
        via_loss: vec![Tracks::ZERO; 2],
        usage: Usage::zeroed(edges),
        history: vec![0.0; edges],
    }
}

const fn cell(layer: usize, x: u16, y: u16) -> GCell {
    GCell {
        layer: LayerId::new(layer),
        x,
        y,
    }
}

fn net(terminals: &[GCell]) -> GlobalNet {
    GlobalNet {
        net: NetId::new(0),
        terminals: terminals.to_vec(),
        demand: vec![Tracks::ONE; 2],
        kind: NetKind::Signal,
    }
}

fn block(grid: &mut GlobalGrid, c: GCell) {
    let e = grid.edge(c);
    grid.capacity[e] = Tracks::ZERO;
    grid.blocked[e] = true;
}

/// The coarse layers `net`'s coarse tree on `grid` runs along.
fn coarse_wire_layers(grid: &GlobalGrid, net: &GlobalNet) -> Vec<usize> {
    let coarse = CoarseGrid::new(grid);
    let mut pins: Vec<usize> = net.terminals.iter().map(|&c| coarse.node_of(c)).collect();
    pins.sort_unstable();
    let wire_layers = coarse.wire_layers(grid, net.kind);
    let nodes = coarse
        .tree_nodes(&pins, &wire_layers, coarse.min_price)
        .unwrap();
    let mut along: Vec<usize> = nodes
        .iter()
        .filter_map(|&(node, parent)| {
            let layer = coarse.position(node).0;
            (coarse.position(parent?).0 == layer).then_some(layer)
        })
        .collect();
    along.sort_unstable();
    along.dedup();
    along
}

#[test]
fn a_clock_trunks_coarse_tree_runs_along_only_the_clock_layers() {
    let edges = 3 * 40 * 40;
    let grid = GlobalGrid {
        directions: vec![
            Direction::Horizontal,
            Direction::Vertical,
            Direction::Horizontal,
        ],
        clock_layers: LayerId::new(2)..=LayerId::new(3),
        capacity: vec![Tracks::whole(4); edges],
        blocked: vec![false; edges],
        via_blocked: vec![false; edges],
        via_loss: vec![Tracks::ZERO; 3],
        usage: Usage::zeroed(edges),
        history: vec![0.0; edges],
        ..grid(40, 40)
    };
    let signal = net(&[cell(1, 2, 4), cell(1, 37, 30)]);
    let trunk = GlobalNet {
        kind: NetKind::ClockTrunk,
        ..signal.clone()
    };

    assert_eq!(coarse_wire_layers(&grid, &signal), vec![0, 1]);
    assert_eq!(coarse_wire_layers(&grid, &trunk), vec![1, 2]);
}

#[test]
fn a_gcell_has_the_mean_of_what_its_edges_leave_free() {
    let mut grid = grid(10, 10);
    grid.usage.set(grid.edge(cell(1, 3, 2)), Tracks::whole(3));

    let between = free_tracks(&grid, cell(1, 4, 2));
    let at_the_edge = free_tracks(&grid, cell(1, 9, 2));

    assert!((between - 2.5).abs() < 1e-9);
    assert!((at_the_edge - 4.0).abs() < 1e-9);
}

#[test]
fn a_coarse_edge_is_a_wall_only_when_every_gcell_edge_across_it_is_blocked() {
    let mut grid = grid(10, 10);
    for y in 0..5 {
        block(&mut grid, cell(1, 4, y));
    }
    for y in 5..9 {
        block(&mut grid, cell(1, 4, y));
    }

    let coarse = CoarseGrid::new(&grid);

    assert!(!coarse.open[coarse.index(0, 0, 0)]);
    assert!(coarse.open[coarse.index(0, 0, 1)]);
}

/// A 60 by 45 grid whose edges are used unevenly, so that no two paths
/// cost the same.
fn uneven_grid() -> GlobalGrid {
    let mut grid = grid(60, 45);
    for e in 0..grid.usage.len() {
        let used = (e * 7919 % 37) as f64 / 10.0;
        grid.usage.set(e, Tracks::from_f64(used));
    }
    grid
}

/// The cost of the coarse tree of `nodes`, each with its parent, as the
/// search prices each step.
fn tree_cost(coarse: &CoarseGrid, nodes: &[(usize, Option<usize>)]) -> f64 {
    nodes
        .iter()
        .filter_map(|&(node, parent)| Some((node, parent?)))
        .map(|(node, parent)| {
            let both = coarse.price(node) + coarse.price(parent);
            if coarse.position(node).0 == coarse.position(parent).0 {
                f64::from(SCALE) * both
            } else {
                both
            }
        })
        .sum()
}

fn pins_of(coarse: &CoarseGrid, terminals: &[GCell]) -> Vec<usize> {
    let mut pins: Vec<usize> = terminals.iter().map(|&c| coarse.node_of(c)).collect();
    pins.sort_unstable();
    pins.dedup();
    pins
}

#[test]
fn a_tree_aimed_at_its_pins_costs_as_little_as_one_searched_unaimed() {
    let grid = uneven_grid();
    let coarse = CoarseGrid::new(&grid);
    let nets = [
        pins_of(&coarse, &[cell(1, 2, 3), cell(2, 57, 41)]),
        pins_of(&coarse, &[cell(1, 50, 2), cell(2, 4, 40), cell(1, 30, 22)]),
        pins_of(
            &coarse,
            &[
                cell(2, 10, 10),
                cell(1, 55, 12),
                cell(1, 12, 43),
                cell(2, 40, 30),
                cell(1, 28, 5),
            ],
        ),
    ];

    let costs: Vec<(f64, f64)> = nets
        .iter()
        .map(|pins| {
            let any = [true; 2];
            let aimed = coarse.tree_nodes(pins, &any, coarse.min_price).unwrap();
            let unaimed = coarse.tree_nodes(pins, &any, 0.0).unwrap();
            (tree_cost(&coarse, &aimed), tree_cost(&coarse, &unaimed))
        })
        .collect();

    for (aimed, unaimed) in costs {
        assert!(
            (aimed - unaimed).abs() <= 1e-9 * unaimed,
            "{aimed} vs {unaimed}"
        );
    }
}

#[test]
fn a_plan_joins_every_terminal() {
    let grid = grid(30, 30);
    let terminals = [cell(1, 2, 2), cell(1, 27, 3), cell(2, 14, 28)];

    let plan = CoarseGrid::new(&grid)
        .plan(&grid, &net(&terminals))
        .unwrap();

    assert!(terminals.iter().all(|c| plan.corridor().allows(c.x, c.y)));
}

#[test]
fn a_plan_detours_around_a_band_with_no_free_tracks() {
    let grid = grid(40, 30);
    for x in 10..30 {
        for y in 0..25 {
            grid.usage.add(grid.edge(cell(1, x, y)), Tracks::whole(20));
        }
    }
    let terminals = [cell(1, 2, 2), cell(1, 37, 2)];

    let plan = CoarseGrid::new(&grid)
        .plan(&grid, &net(&terminals))
        .unwrap();

    assert!(plan.corridor().allows(20, 27));
    assert!(!plan.corridor().allows(20, 2));
}

#[test]
fn a_plan_never_crosses_a_wall_of_blocked_edges() {
    let mut grid = grid(40, 30);
    for y in 0..25 {
        block(&mut grid, cell(1, 19, y));
    }
    let terminals = [cell(1, 2, 2), cell(1, 37, 2)];

    let plan = CoarseGrid::new(&grid)
        .plan(&grid, &net(&terminals))
        .unwrap();

    assert!(plan.corridor().allows(19, 27));
    assert!(!plan.corridor().allows(19, 2));
}

#[test]
fn the_path_between_two_terminals_leaves_out_the_branch_to_a_third() {
    let grid = grid(40, 40);
    let terminals = [cell(1, 2, 2), cell(1, 37, 2), cell(2, 20, 37)];

    let plan = CoarseGrid::new(&grid)
        .plan(&grid, &net(&terminals))
        .unwrap();
    let path = plan.between(terminals[0], terminals[1], &grid).unwrap();

    assert!(path.allows(2, 2) && path.allows(37, 2));
    assert!(plan.corridor().allows(20, 37));
    assert!(!path.allows(20, 37));
}

#[test]
fn terminals_in_one_coarse_cell_need_no_plan() {
    let grid = grid(30, 30);
    let terminals = [cell(1, 1, 1), cell(2, 3, 4)];

    let corridor = CoarseGrid::new(&grid).plan(&grid, &net(&terminals));

    assert!(corridor.is_none());
}
