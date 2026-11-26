//! The Munkres assignment `OpenROAD`'s pin placer solves with, step for step,
//! so that of equally cheap assignments it takes the same one.

/// The state of the solver over a matrix of `rows` by `cols`, at least one
/// of each.
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
            let first = self.distance[cells.start];
            let least = self.distance[cells.clone()]
                .iter()
                .fold(first, |m, &d| m.min(d));
            for at in cells {
                self.distance[at] -= least;
            }
        }
    }

    fn subtract_col_minima(&mut self) {
        for col in 0..self.cols {
            let first = self.distance[self.at(0, col)];
            let least =
                (1..self.rows).fold(first, |m, row| m.min(self.distance[self.at(row, col)]));
            for row in 0..self.rows {
                let at = self.at(row, col);
                self.distance[at] -= least;
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
mod tests;
