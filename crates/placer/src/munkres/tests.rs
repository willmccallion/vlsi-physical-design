use super::*;

fn total(costs: &[Vec<i64>], assignment: &[Option<usize>]) -> i64 {
    assignment
        .iter()
        .enumerate()
        .filter_map(|(r, c)| c.map(|c| costs[r][c]))
        .sum()
}

fn brute_force(costs: &[Vec<i64>], row: usize, used: &mut [bool; 5]) -> i64 {
    if row == costs.len() {
        return 0;
    }
    let mut best = i64::MAX;
    for c in 0..5 {
        if !used[c] {
            used[c] = true;
            best = best.min(costs[row][c] + brute_force(costs, row + 1, used));
            used[c] = false;
        }
    }
    best
}

fn random_matrices() -> impl FnMut(usize, usize) -> Vec<Vec<i64>> {
    let mut seed = 12_345_u64;
    move |rows, cols| {
        (0..rows)
            .map(|_| {
                (0..cols)
                    .map(|_| {
                        seed = seed.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
                        i64::try_from(seed >> 58).unwrap()
                    })
                    .collect()
            })
            .collect()
    }
}

#[test]
fn finds_the_cheapest_matching() {
    let costs = vec![vec![4, 1, 3], vec![2, 0, 5], vec![3, 2, 2]];

    let assignment = min_cost_assignment(&costs);

    assert_eq!(total(&costs, &assignment), 5);
}

#[test]
fn matches_brute_force_with_fewer_rows_than_columns() {
    let mut matrix = random_matrices();
    for _ in 0..200 {
        let costs = matrix(4, 5);

        let best = brute_force(&costs, 0, &mut [false; 5]);

        assert_eq!(total(&costs, &min_cost_assignment(&costs)), best);
    }
}

#[test]
fn leaves_the_rows_beyond_the_columns_unassigned_at_least_cost() {
    let mut matrix = random_matrices();
    for _ in 0..200 {
        let by_slot = matrix(5, 3);
        let by_pin: Vec<Vec<i64>> = (0..3)
            .map(|c| by_slot.iter().map(|row| row[c]).collect())
            .collect();

        let assignment = min_cost_assignment(&by_slot);

        let best = brute_force(&by_pin, 0, &mut [false; 5]);
        assert_eq!(total(&by_slot, &assignment), best);
        assert_eq!(assignment.iter().flatten().count(), 3);
    }
}

#[test]
fn takes_openroads_assignment_of_equal_costs() {
    let costs = vec![vec![0, 0], vec![0, 0], vec![0, 0]];

    let assignment = min_cost_assignment(&costs);

    assert_eq!(assignment, vec![Some(0), Some(1), None]);
}
