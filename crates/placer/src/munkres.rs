//! The Munkres assignment `OpenROAD`'s pin placer solves with, step for step,
//! so that of equally cheap assignments it takes the same one.

/// The state of the solver over a matrix of `rows` by `cols`.
struct Munkres {
    rows: usize,
    cols: usize,
    distance: Vec<i64>,
    starred: Vec<bool>,
    primed: Vec<bool>,
    covered_rows: Vec<bool>,
    covered_cols: Vec<bool>,
}

impl Munkres {
    fn new(costs: &[Vec<i64>]) -> Self {
        let rows = costs.len();
        let cols = costs[0].len();
        Self {
            rows,
            cols,
            distance: costs.iter().flatten().copied().collect(),
            starred: vec![false; rows * cols],
            primed: vec![false; rows * cols],
            covered_rows: vec![false; rows],
            covered_cols: vec![false; cols],
        }
    }

    const fn at(&self, row: usize, col: usize) -> usize {
        row * self.cols + col
    }

    fn is_zero(&self, row: usize, col: usize) -> bool {
        self.distance[self.at(row, col)] == 0
    }

    fn star(&mut self, row: usize, col: usize) {
        let at = self.at(row, col);
        self.starred[at] = true;
        self.covered_cols[col] = true;
    }

    fn subtract_row_minima(&mut self) {
        for row in 0..self.rows {
            let cells = self.at(row, 0)..self.at(row, 0) + self.cols;
            let least = self.distance[cells.clone()].iter().copied().min();
            for at in cells {
                self.distance[at] -= least.unwrap_or(0);
            }
        }
    }

    fn subtract_col_minima(&mut self) {
        for col in 0..self.cols {
            let least = (0..self.rows)
                .map(|row| self.distance[self.at(row, col)])
                .min();
            for row in 0..self.rows {
                let at = self.at(row, col);
                self.distance[at] -= least.unwrap_or(0);
            }
        }
    }

    /// Reduces the matrix along its shorter side and stars a first set of
    /// independent zeros, as the preliminary steps of `assignmentoptimal`.
    fn prepare(&mut self) {
        if self.rows <= self.cols {
            self.subtract_row_minima();
            for row in 0..self.rows {
                let free_zero =
                    (0..self.cols).find(|&col| self.is_zero(row, col) && !self.covered_cols[col]);
                if let Some(col) = free_zero {
                    self.star(row, col);
                }
            }
        } else {
            self.subtract_col_minima();
            for col in 0..self.cols {
                let free_zero =
                    (0..self.rows).find(|&row| self.is_zero(row, col) && !self.covered_rows[row]);
                if let Some(row) = free_zero {
                    self.star(row, col);
                    self.covered_rows[row] = true;
                }
            }
            self.covered_rows.fill(false);
        }
    }

    fn star_in_row(&self, row: usize) -> Option<usize> {
        (0..self.cols).find(|&col| self.starred[self.at(row, col)])
    }

    fn star_in_col(&self, col: usize) -> Option<usize> {
        (0..self.rows).find(|&row| self.starred[self.at(row, col)])
    }

    /// Primes uncovered zeros until one has no star in its row, which it
    /// returns, as `step3`; `None` when no uncovered zero is left.
    fn prime_zeros(&mut self) -> Option<(usize, usize)> {
        let mut found = true;
        while found {
            found = false;
            for col in 0..self.cols {
                if self.covered_cols[col] {
                    continue;
                }
                let zero =
                    (0..self.rows).find(|&row| !self.covered_rows[row] && self.is_zero(row, col));
                let Some(row) = zero else {
                    continue;
                };
                let at = self.at(row, col);
                self.primed[at] = true;
                let Some(star_col) = self.star_in_row(row) else {
                    return Some((row, col));
                };
                self.covered_rows[row] = true;
                self.covered_cols[star_col] = false;
                found = true;
            }
        }
        None
    }

    /// Stars the zero at `row`, `col` and flips the chain of stars and primes
    /// from it, as `step4`.
    fn augment(&mut self, row: usize, col: usize) {
        let mut next = self.starred.clone();
        next[self.at(row, col)] = true;
        let mut star_col = col;
        while let Some(star_row) = self.star_in_col(star_col) {
            next[self.at(star_row, star_col)] = false;
            let primed = (0..self.cols).find(|&c| self.primed[self.at(star_row, c)]);
            let Some(prime_col) = primed else {
                break;
            };
            next[self.at(star_row, prime_col)] = true;
            star_col = prime_col;
        }
        self.starred = next;
        self.primed.fill(false);
        self.covered_rows.fill(false);
        for col in 0..self.cols {
            self.covered_cols[col] = self.star_in_col(col).is_some();
        }
    }

    /// Opens new zeros with the least uncovered value, as `step5`.
    fn shift_by_least_uncovered(&mut self) {
        let mut least = i64::MAX;
        for row in (0..self.rows).filter(|&r| !self.covered_rows[r]) {
            for col in (0..self.cols).filter(|&c| !self.covered_cols[c]) {
                least = least.min(self.distance[self.at(row, col)]);
            }
        }
        for row in 0..self.rows {
            for col in 0..self.cols {
                let at = self.at(row, col);
                if self.covered_rows[row] {
                    self.distance[at] += least;
                }
                if !self.covered_cols[col] {
                    self.distance[at] -= least;
                }
            }
        }
    }

    fn solve(mut self) -> Vec<Option<usize>> {
        self.prepare();
        let matched = self.rows.min(self.cols);
        while self.covered_cols.iter().filter(|&&c| c).count() != matched {
            let (row, col) = loop {
                if let Some(zero) = self.prime_zeros() {
                    break zero;
                }
                self.shift_by_least_uncovered();
            };
            self.augment(row, col);
        }
        (0..self.rows).map(|row| self.star_in_row(row)).collect()
    }
}

/// The column assigned to each row of `costs` so that the total is least, as
/// `HungarianAlgorithm::solve`; `None` for rows beyond the columns.
pub(crate) fn min_cost_assignment(costs: &[Vec<i64>]) -> Vec<Option<usize>> {
    if costs.first().is_none_or(Vec::is_empty) {
        return vec![None; costs.len()];
    }
    Munkres::new(costs).solve()
}

#[cfg(test)]
mod tests {
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
}
