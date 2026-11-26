use super::*;

const fn arc(source: usize, target: usize, capacity: i32, cost: i32) -> Arc {
    Arc {
        source,
        target,
        capacity,
        cost,
    }
}

#[test]
fn the_cheaper_of_two_paths_carries_the_flow() {
    let arcs = [
        arc(0, 1, 1, 5),
        arc(0, 2, 1, 1),
        arc(1, 3, 1, 0),
        arc(2, 3, 1, 0),
    ];

    let flows = min_cost_flow(4, &arcs, 0, 3, 1);

    assert_eq!(flows, Ok(vec![0, 1, 0, 1]));
}

#[test]
fn a_supply_no_path_can_carry_is_infeasible() {
    let arcs = [arc(0, 1, 1, 0), arc(1, 2, 1, 0)];

    let outcome = min_cost_flow(3, &arcs, 0, 2, 2);

    assert_eq!(outcome.map_err(|e| e.problem), Err(FlowProblem::Infeasible));
}

/// Every ordering of `0..n`.
fn permutations(n: usize) -> Vec<Vec<usize>> {
    let Some(last) = n.checked_sub(1) else {
        return vec![Vec::new()];
    };
    let mut all = Vec::new();
    for shorter in permutations(last) {
        for k in 0..=shorter.len() {
            let mut p = shorter.clone();
            p.insert(k, last);
            all.push(p);
        }
    }
    all
}

/// A cost for each cell and spot of an `n` by `n` assignment, every cell
/// able to keep its own spot and some able to take others.
fn random_costs(state: &mut u64, n: usize) -> Vec<Vec<Option<i32>>> {
    let mut next = |bound: u64| {
        *state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (*state >> 33) % bound
    };
    (0..n)
        .map(|i| {
            (0..n)
                .map(|j| (i == j || next(3) > 0).then(|| i32::try_from(next(1000)).unwrap()))
                .collect()
        })
        .collect()
}

/// The cost of the matching the simplex finds over `cost`, as `solveMatch`
/// builds the flow problem, and how many cells it matches.
fn simplex_matching(cost: &[Vec<Option<i32>>]) -> (i64, usize) {
    let n = cost.len();
    let (supply, demand) = (2 * n, 2 * n + 1);
    let mut arcs = Vec::new();
    let mut assignment = Vec::new();
    for (i, row) in cost.iter().enumerate() {
        arcs.push(arc(supply, 2 * i, 1, 0));
        arcs.push(arc(2 * i + 1, demand, 1, 0));
        for (j, c) in row.iter().enumerate() {
            if let Some(c) = *c {
                assignment.push((arcs.len(), c));
                arcs.push(arc(2 * i, 2 * j + 1, 1, c));
            }
        }
    }
    let flows = min_cost_flow(2 * n + 2, &arcs, supply, demand, i32::try_from(n).unwrap()).unwrap();
    let used: Vec<i32> = assignment
        .iter()
        .filter(|&&(a, _)| flows[a] == 1)
        .map(|&(_, c)| c)
        .collect();
    (used.iter().map(|&c| i64::from(c)).sum(), used.len())
}

#[test]
fn an_assignment_costs_as_little_as_the_best_permutation() {
    let mut state = 12_345;
    for _ in 0..3000 {
        let n = 1 + (state % 6) as usize;
        let cost = random_costs(&mut state, n);

        let best = permutations(n)
            .iter()
            .filter_map(|p| {
                (0..n)
                    .map(|i| cost[i][p[i]].map(i64::from))
                    .sum::<Option<i64>>()
            })
            .min();

        assert_eq!(simplex_matching(&cost), (best.unwrap(), n), "{cost:?}");
    }
}

/// The flows of assignments whose costs tie often, each asked for the
/// whole assignment and for one more than it can carry, hashed in order.
fn tied_assignment_flows_digest() -> u64 {
    let mut state = 777;
    let mut digest = 0xcbf2_9ce4_8422_2325_u64;
    for _ in 0..500 {
        let n = 1 + (state % 7) as usize;
        let cost: Vec<Vec<Option<i32>>> = random_costs(&mut state, n)
            .into_iter()
            .map(|row| row.into_iter().map(|c| c.map(|c| c % 3)).collect())
            .collect();
        let (supply, demand) = (2 * n, 2 * n + 1);
        let mut arcs = Vec::new();
        for (i, row) in cost.iter().enumerate() {
            arcs.push(arc(supply, 2 * i, 1, 0));
            arcs.push(arc(2 * i + 1, demand, 1, 0));
            for (j, c) in row.iter().enumerate() {
                if let Some(c) = *c {
                    arcs.push(arc(2 * i, 2 * j + 1, 1, c));
                }
            }
        }
        for asked in [n, n + 1] {
            let flows = min_cost_flow(
                2 * n + 2,
                &arcs,
                supply,
                demand,
                i32::try_from(asked).unwrap(),
            )
            .unwrap_or_else(|e| e.flows);
            for f in flows {
                digest = (digest ^ u64::from(f.cast_unsigned())).wrapping_mul(0x0100_0000_01b3);
            }
        }
    }
    digest
}

/// [`tied_assignment_flows_digest`] as this port gave it before its
/// setup and pivoting were split into steps; a change means the
/// pivots, and so the ties, came out differently.
const TIED_FLOWS_DIGEST: u64 = 13_035_736_887_052_952_343;

#[test]
fn tied_assignments_keep_the_flows_lemon_chooses() {
    assert_eq!(tied_assignment_flows_digest(), TIED_FLOWS_DIGEST);
}
