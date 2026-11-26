use super::super::Tracks;
use super::super::coarse::CoarseGrid;
use super::super::grid::Usage;
use super::*;
use crate::tech::NetKind;
use pare_common::config::CapacityReserve;
use pare_common::db::indices::NetId;

const COSTS: GlobalCosts = GlobalCosts {
    via: 1.0,
    congestion: 1.0,
    penalty: 1.0,
};

/// A 10x10 grid with a horizontal layer 1 over a vertical layer 2,
/// every edge holding 4 tracks.
fn grid() -> GlobalGrid {
    sized_grid(10, 10)
}

/// A `nx` by `ny` grid like [`grid`].
fn sized_grid(nx: u16, ny: u16) -> GlobalGrid {
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

fn cell(layer: usize, x: u16, y: u16) -> GCell {
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

/// Blocks the horizontal edges from column `x` to `x + 1` in rows `rows`.
fn wall(grid: &mut GlobalGrid, x: u16, rows: std::ops::RangeInclusive<u16>) {
    for y in rows {
        let e = grid.edge(cell(1, x, y));
        grid.capacity[e] = Tracks::ZERO;
        grid.blocked[e] = true;
    }
}

fn crossed_blocked(grid: &GlobalGrid, paths: &[Vec<GCell>]) -> usize {
    paths
        .iter()
        .flat_map(|p| grid.path_edges(p))
        .filter(|&e| grid.blocked[e])
        .count()
}

fn reaches(paths: &[Vec<GCell>], target: GCell) -> bool {
    paths.iter().flatten().any(|&c| c == target)
}

#[test]
fn a_net_detours_around_a_blocked_wall_beyond_its_margin() {
    let mut grid = grid();
    wall(&mut grid, 4, 0..=8);
    let terminals = [cell(1, 2, 4), cell(1, 7, 4)];

    let paths = route_tree(&grid, &net(&terminals), 1, COSTS, None);

    assert!(reaches(&paths, terminals[1]));
    assert_eq!(crossed_blocked(&grid, &paths), 0);
}

#[test]
fn a_pin_no_detour_reaches_is_joined_across_blocked_edges() {
    let mut grid = grid();
    wall(&mut grid, 4, 0..=9);
    let terminals = [cell(1, 2, 4), cell(1, 7, 4)];

    let paths = route_tree(&grid, &net(&terminals), 1, COSTS, None);

    assert!(reaches(&paths, terminals[1]));
    assert_eq!(crossed_blocked(&grid, &paths), 1);
}

#[test]
fn a_net_never_climbs_into_a_cell_with_no_room_for_a_via() {
    let mut grid = grid();
    wall(&mut grid, 4, 0..=8);
    for x in 0..=3 {
        for y in 0..10 {
            let i = grid.edge(cell(2, x, y));
            grid.via_blocked[i] = true;
        }
    }
    let terminals = [cell(1, 2, 4), cell(1, 7, 4)];

    let paths = route_tree(&grid, &net(&terminals), 1, COSTS, None);

    assert!(reaches(&paths, terminals[1]));
    assert_eq!(crossed_blocked(&grid, &paths), 0);
    assert!(paths.iter().flatten().all(|&c| !grid.via_blocked(c)));
}

#[test]
fn a_net_takes_the_detour_its_corridor_plans_beyond_its_margin() {
    let mut grid = sized_grid(30, 30);
    for x in 10..20 {
        for y in 0..20 {
            grid.usage.set(grid.edge(cell(1, x, y)), Tracks::whole(40));
        }
    }
    let terminals = [cell(1, 2, 2), cell(1, 27, 2)];
    let plan = CoarseGrid::new(&grid)
        .plan(&grid, &net(&terminals))
        .unwrap();

    let planned = route_tree(&grid, &net(&terminals), 1, COSTS, Some(&plan));
    let windowed = route_tree(&grid, &net(&terminals), 1, COSTS, None);

    let highest = |paths: &[Vec<GCell>]| paths.iter().flatten().map(|c| c.y).max();
    assert!(reaches(&planned, terminals[1]));
    assert!(highest(&planned) >= Some(20));
    assert!(highest(&windowed) < Some(20));
}

#[test]
fn a_net_kept_inside_its_area_fails_where_only_a_wider_detour_reaches() {
    let mut grid = grid();
    wall(&mut grid, 4, 0..=8);
    let terminals = [cell(1, 2, 4), cell(1, 7, 4)];

    let within = route_tree_within(&grid, &net(&terminals), 1, COSTS, None);

    assert_eq!(within, Err(OutgrowsArea));
}

#[test]
fn a_net_kept_inside_its_area_takes_the_route_found_there() {
    let mut grid = grid();
    wall(&mut grid, 4, 0..=2);
    let terminals = [cell(1, 2, 4), cell(2, 7, 1), cell(1, 5, 6)];

    let within = route_tree_within(&grid, &net(&terminals), 1, COSTS, None);

    assert_eq!(
        within,
        Ok(route_tree(&grid, &net(&terminals), 1, COSTS, None))
    );
}

#[test]
fn a_net_whose_corridor_holds_no_path_is_still_routed() {
    let mut grid = grid();
    wall(&mut grid, 6, 0..=4);
    let terminals = [cell(1, 1, 1), cell(1, 8, 1)];
    let plan = CoarseGrid::new(&grid)
        .plan(&grid, &net(&terminals))
        .unwrap();

    let paths = route_tree(&grid, &net(&terminals), 1, COSTS, Some(&plan));

    assert!(!plan.corridor().allows(6, 6));
    assert!(reaches(&paths, terminals[1]));
    assert_eq!(crossed_blocked(&grid, &paths), 0);
}

#[test]
fn a_wider_net_pays_for_the_tracks_it_takes() {
    let mut grid = grid();
    for x in 2..7 {
        let e = grid.edge(cell(1, x, 4));
        grid.usage.set(e, Tracks::whole(3));
    }
    let terminals = [cell(1, 2, 4), cell(1, 7, 4)];
    let wide = GlobalNet {
        demand: vec![Tracks::whole(2); 2],
        ..net(&terminals)
    };

    let narrow_paths = route_tree(&grid, &net(&terminals), 2, COSTS, None);
    let wide_paths = route_tree(&grid, &wide, 2, COSTS, None);

    let on_row = |paths: &[Vec<GCell>]| {
        paths
            .iter()
            .flatten()
            .filter(|c| c.layer == LayerId::new(1) && c.y == 4)
            .count()
    };
    assert!(on_row(&narrow_paths) > on_row(&wide_paths));
}

#[test]
fn a_dense_net_with_an_outlier_reaches_every_terminal() {
    let grid = sized_grid(40, 40);
    let mut terminals: Vec<GCell> = (1..=2)
        .flat_map(|layer| (0..13).flat_map(move |x| (0..12).map(move |y| cell(layer, x, y))))
        .collect();
    terminals.push(cell(1, 35, 35));

    let paths = route_tree(&grid, &net(&terminals), 2, COSTS, None);

    let missed: Vec<GCell> = terminals
        .iter()
        .copied()
        .filter(|&t| !reaches(&paths, t))
        .collect();
    assert_eq!(missed, []);
}

#[test]
fn a_net_widens_past_half_a_wide_grid_to_route_around_a_wall() {
    let mut grid = sized_grid(40_000, 1);
    wall(&mut grid, 39_994, 0..=0);
    let terminals = [cell(1, 39_990, 0), cell(1, 39_999, 0)];

    let paths = route_tree(&grid, &net(&terminals), 2, COSTS, None);

    assert!(reaches(&paths, terminals[0]) && reaches(&paths, terminals[1]));
}

#[test]
fn the_estimate_spans_a_grid_wider_than_half_the_coordinate_range() {
    // The estimate reads only the grid's layers, not its edges.
    let grid = GlobalGrid {
        nx: 40_000,
        ny: 40_000,
        ..grid()
    };
    let terminals = [cell(1, 0, 0), cell(1, 39_999, 39_999)];
    let query = Query {
        net: &net(&terminals),
        costs: COSTS,
        blocked: Blocked::Avoid,
        corridor: None,
    };

    let rules = MoveRules::new(&grid, query, &terminals[..1], terminals[1]);

    assert!((rules.estimate(terminals[0]) - 79_998.0).abs() < 1e-9);
}
