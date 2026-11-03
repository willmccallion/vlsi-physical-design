//! The electrostatic potential and field of a density map, with zero-flux
//! edges.
//!
//! Mirroring the grid to 2N×2N makes it even, so a periodic 2D FFT solve
//! gives the Neumann (cosine transform) solution. The field is the spectral
//! derivative of the potential, as `gpl`'s FFT takes it, not a difference
//! of neighboring bins, which would weaken the field of fine detail.

use rustfft::num_complex::Complex;
use rustfft::{Fft, FftPlanner};
use std::f64::consts::PI;
use std::sync::Arc;

/// Solves `-∇²φ = ρ` on one grid of bins, reusing its buffers.
pub(super) struct PoissonSolver {
    cols: usize,
    rows: usize,
    bin: (f64, f64),
    forward_x: Arc<dyn Fft<f64>>,
    forward_y: Arc<dyn Fft<f64>>,
    inverse_x: Arc<dyn Fft<f64>>,
    inverse_y: Arc<dyn Fft<f64>>,
    scratch: Vec<Complex<f64>>,
    transposed: Vec<Complex<f64>>,
    field: Vec<Complex<f64>>,
}

impl std::fmt::Debug for PoissonSolver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PoissonSolver")
            .field("cols", &self.cols)
            .field("rows", &self.rows)
            .field("bin", &self.bin)
            .finish_non_exhaustive()
    }
}

impl PoissonSolver {
    /// A solver for `cols`×`rows` bins of the given width and height.
    pub(super) fn new(cols: usize, rows: usize, bin_width: f64, bin_height: f64) -> Self {
        let mut planner = FftPlanner::new();
        let mirrored = 4 * cols * rows;
        Self {
            cols,
            rows,
            bin: (bin_width, bin_height),
            forward_x: planner.plan_fft_forward(2 * cols),
            forward_y: planner.plan_fft_forward(2 * rows),
            inverse_x: planner.plan_fft_inverse(2 * cols),
            inverse_y: planner.plan_fft_inverse(2 * rows),
            scratch: vec![Complex::default(); mirrored],
            transposed: vec![Complex::default(); mirrored],
            field: vec![Complex::default(); mirrored],
        }
    }

    /// Writes the potential of `density` and its field `-∇φ`, all
    /// row-major, into `potential`, `field_x` and `field_y`.
    pub(super) fn solve(
        &mut self,
        density: &[f64],
        potential: &mut [f64],
        field_x: &mut [f64],
        field_y: &mut [f64],
    ) {
        let (cols, rows) = (self.cols, self.rows);
        let (width, height) = (2 * cols, 2 * rows);
        for y in 0..rows {
            for x in 0..cols {
                let val = Complex::new(density[y * cols + x], 0.0);
                self.scratch[y * width + x] = val;
                self.scratch[y * width + (width - 1 - x)] = val;
                self.scratch[(height - 1 - y) * width + x] = val;
                self.scratch[(height - 1 - y) * width + (width - 1 - x)] = val;
            }
        }

        self.forward_x.process(&mut self.scratch);
        transpose(&self.scratch, &mut self.transposed, height, width);
        self.forward_y.process(&mut self.transposed);

        let (bin_w, bin_h) = self.bin;
        for u in 0..width {
            let kx = angular_frequency(u, width) / bin_w;
            for v in 0..height {
                let ky = angular_frequency(v, height) / bin_h;
                let eigenvalue = kx.mul_add(kx, ky * ky);
                let idx = u * height + v;
                let phi = if eigenvalue > 0.0 {
                    self.transposed[idx] / eigenvalue
                } else {
                    Complex::new(0.0, 0.0)
                };
                self.transposed[idx] = phi;
                // -i·kx·φ in the real part and -i·ky·φ in the imaginary one.
                self.field[idx] = phi * Complex::new(ky, -kx);
            }
        }

        let norm = 1.0 / (width * height) as f64;
        self.inverse_2d(Spectrum::Potential);
        for y in 0..rows {
            for x in 0..cols {
                potential[y * cols + x] = self.scratch[y * width + x].re * norm;
            }
        }
        self.inverse_2d(Spectrum::Field);
        for y in 0..rows {
            for x in 0..cols {
                let e = self.scratch[y * width + x] * norm;
                field_x[y * cols + x] = e.re;
                field_y[y * cols + x] = e.im;
            }
        }
    }

    /// Inverts one transposed spectrum into `scratch`, row-major.
    fn inverse_2d(&mut self, spectrum: Spectrum) {
        let (width, height) = (2 * self.cols, 2 * self.rows);
        let transformed = match spectrum {
            Spectrum::Potential => &mut self.transposed,
            Spectrum::Field => &mut self.field,
        };
        self.inverse_y.process(transformed);
        transpose(transformed, &mut self.scratch, width, height);
        self.inverse_x.process(&mut self.scratch);
    }
}

/// Which of the solver's spectra to invert.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Spectrum {
    Potential,
    Field,
}

/// Angular frequency of DFT bin `k` out of `n`, with bins above `n / 2`
/// standing for negative frequencies.
fn angular_frequency(k: usize, n: usize) -> f64 {
    let signed = if k <= n / 2 {
        k as f64
    } else {
        k as f64 - n as f64
    };
    2.0 * PI * signed / n as f64
}

/// Writes the `rows`×`cols` row-major `src` into `dst` column by column.
fn transpose(src: &[Complex<f64>], dst: &mut [Complex<f64>], rows: usize, cols: usize) {
    for r in 0..rows {
        for c in 0..cols {
            dst[c * rows + r] = src[r * cols + c];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A grid of `cols`×`rows` bins.
    #[derive(Clone, Copy)]
    struct Grid {
        cols: usize,
        rows: usize,
        bin: (f64, f64),
    }

    const fn square(dim: usize, bin: (f64, f64)) -> Grid {
        Grid {
            cols: dim,
            rows: dim,
            bin,
        }
    }

    /// The phases of mode (`a`, `b`) at the center of bin (`x`, `y`).
    fn phases(grid: Grid, a: usize, b: usize, x: usize, y: usize) -> (f64, f64) {
        (
            PI * a as f64 * (x as f64 + 0.5) / grid.cols as f64,
            PI * b as f64 * (y as f64 + 0.5) / grid.rows as f64,
        )
    }

    fn cosine_mode(grid: Grid, a: usize, b: usize) -> Vec<f64> {
        let mut density = vec![0.0; grid.cols * grid.rows];
        for y in 0..grid.rows {
            for x in 0..grid.cols {
                let (px, py) = phases(grid, a, b, x, y);
                density[y * grid.cols + x] = px.cos() * py.cos();
            }
        }
        density
    }

    /// The wavenumbers of mode (`a`, `b`).
    fn wavenumbers(grid: Grid, a: usize, b: usize) -> (f64, f64) {
        (
            PI * a as f64 / grid.cols as f64 / grid.bin.0,
            PI * b as f64 / grid.rows as f64 / grid.bin.1,
        )
    }

    struct Solution {
        potential: Vec<f64>,
        field_x: Vec<f64>,
        field_y: Vec<f64>,
    }

    fn solve_all(density: &[f64], grid: Grid) -> Solution {
        let mut solver = PoissonSolver::new(grid.cols, grid.rows, grid.bin.0, grid.bin.1);
        let n = grid.cols * grid.rows;
        let mut solution = Solution {
            potential: vec![0.0; n],
            field_x: vec![0.0; n],
            field_y: vec![0.0; n],
        };
        solver.solve(
            density,
            &mut solution.potential,
            &mut solution.field_x,
            &mut solution.field_y,
        );
        solution
    }

    fn assert_solves_cosine_mode(grid: Grid, a: usize, b: usize) {
        let density = cosine_mode(grid, a, b);
        let (wa, wb) = wavenumbers(grid, a, b);
        let eigenvalue = wa.mul_add(wa, wb * wb);

        let potential = solve_all(&density, grid).potential;

        for (phi, rho) in potential.iter().zip(&density) {
            assert!(
                (phi - rho / eigenvalue).abs() < 1e-9 * (1.0 + (rho / eigenvalue).abs()),
                "mode ({a}, {b}): got {phi}, expected {}",
                rho / eigenvalue
            );
        }
    }

    fn assert_field_derives_cosine_mode(grid: Grid, a: usize, b: usize) {
        let density = cosine_mode(grid, a, b);
        let (wa, wb) = wavenumbers(grid, a, b);
        let eigenvalue = wa.mul_add(wa, wb * wb);

        let solution = solve_all(&density, grid);

        for y in 0..grid.rows {
            for x in 0..grid.cols {
                let (px, py) = phases(grid, a, b, x, y);
                let ex = wa * px.sin() * py.cos() / eigenvalue;
                let ey = wb * px.cos() * py.sin() / eigenvalue;
                let idx = y * grid.cols + x;
                assert!((solution.field_x[idx] - ex).abs() < 1e-9, "x at ({x}, {y})");
                assert!((solution.field_y[idx] - ey).abs() < 1e-9, "y at ({x}, {y})");
            }
        }
    }

    #[test]
    fn potential_of_a_cosine_mode_is_the_mode_over_its_eigenvalue() {
        assert_solves_cosine_mode(square(16, (1.0, 1.0)), 1, 2);
    }

    #[test]
    fn potential_distinguishes_x_and_y_frequencies() {
        assert_solves_cosine_mode(square(16, (1.0, 1.0)), 5, 0);
        assert_solves_cosine_mode(square(16, (1.0, 1.0)), 0, 5);
    }

    #[test]
    fn potential_of_high_frequency_modes_uses_negative_frequencies() {
        assert_solves_cosine_mode(square(16, (1.0, 1.0)), 15, 11);
    }

    #[test]
    fn potential_scales_each_axis_by_its_bin_size() {
        assert_solves_cosine_mode(square(16, (2.5, 0.5)), 3, 2);
    }

    #[test]
    fn potential_of_a_grid_with_more_columns_than_rows_keeps_each_axis_apart() {
        let grid = Grid {
            cols: 16,
            rows: 4,
            bin: (1.0, 2.0),
        };

        assert_solves_cosine_mode(grid, 11, 3);
    }

    #[test]
    fn the_field_is_the_exact_derivative_of_a_cosine_mode_even_at_high_frequency() {
        assert_field_derives_cosine_mode(square(16, (2.0, 0.5)), 13, 3);
    }

    #[test]
    fn the_field_of_a_grid_with_more_rows_than_columns_keeps_each_axis_apart() {
        let grid = Grid {
            cols: 4,
            rows: 16,
            bin: (2.0, 1.0),
        };

        assert_field_derives_cosine_mode(grid, 3, 11);
    }

    #[test]
    fn uniform_density_produces_no_potential() {
        let potential = solve_all(&[0.25; 64], square(8, (1.0, 1.0))).potential;

        assert!(potential.iter().all(|phi| phi.abs() < 1e-12));
    }
}
