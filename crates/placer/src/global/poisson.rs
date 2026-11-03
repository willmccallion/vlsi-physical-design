//! The electrostatic potential of a density map, with zero-flux edges.
//!
//! Mirroring the grid to 2N×2N makes it even, so a periodic 2D FFT solve
//! gives the Neumann (cosine transform) solution.

use rustfft::num_complex::Complex;
use rustfft::{Fft, FftPlanner};
use std::f64::consts::PI;
use std::sync::Arc;

/// Solves `-∇²φ = ρ` on one square grid of bins, reusing its buffers.
pub(super) struct PoissonSolver {
    dim: usize,
    bin: (f64, f64),
    forward: Arc<dyn Fft<f64>>,
    inverse: Arc<dyn Fft<f64>>,
    scratch: Vec<Complex<f64>>,
    transposed: Vec<Complex<f64>>,
}

impl std::fmt::Debug for PoissonSolver {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PoissonSolver")
            .field("dim", &self.dim)
            .field("bin", &self.bin)
            .finish_non_exhaustive()
    }
}

impl PoissonSolver {
    /// A solver for `dim`×`dim` bins of the given width and height.
    pub(super) fn new(dim: usize, bin_width: f64, bin_height: f64) -> Self {
        let mut planner = FftPlanner::new();
        let mirrored = 2 * dim;
        Self {
            dim,
            bin: (bin_width, bin_height),
            forward: planner.plan_fft_forward(mirrored),
            inverse: planner.plan_fft_inverse(mirrored),
            scratch: vec![Complex::default(); mirrored * mirrored],
            transposed: vec![Complex::default(); mirrored * mirrored],
        }
    }

    /// Writes the potential of `density`, both row-major, into `potential`.
    pub(super) fn solve(&mut self, density: &[f64], potential: &mut [f64]) {
        let dim = self.dim;
        let dim2 = 2 * dim;
        for y in 0..dim {
            for x in 0..dim {
                let val = Complex::new(density[y * dim + x], 0.0);
                self.scratch[y * dim2 + x] = val;
                self.scratch[y * dim2 + (dim2 - 1 - x)] = val;
                self.scratch[(dim2 - 1 - y) * dim2 + x] = val;
                self.scratch[(dim2 - 1 - y) * dim2 + (dim2 - 1 - x)] = val;
            }
        }

        self.forward.process(&mut self.scratch);
        transpose(&self.scratch, &mut self.transposed, dim2);
        self.forward.process(&mut self.transposed);

        let (bin_w, bin_h) = self.bin;
        for u in 0..dim2 {
            let kx = angular_frequency(u, dim2) / bin_w;
            for v in 0..dim2 {
                let ky = angular_frequency(v, dim2) / bin_h;
                let eigenvalue = kx.mul_add(kx, ky * ky);
                let idx = u * dim2 + v;
                self.transposed[idx] = if eigenvalue > 0.0 {
                    self.transposed[idx] / eigenvalue
                } else {
                    Complex::new(0.0, 0.0)
                };
            }
        }

        self.inverse.process(&mut self.transposed);
        transpose(&self.transposed, &mut self.scratch, dim2);
        self.inverse.process(&mut self.scratch);

        let norm = 1.0 / (dim2 * dim2) as f64;
        for y in 0..dim {
            for x in 0..dim {
                potential[y * dim + x] = self.scratch[y * dim2 + x].re * norm;
            }
        }
    }
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

fn transpose(src: &[Complex<f64>], dst: &mut [Complex<f64>], n: usize) {
    for r in 0..n {
        for c in 0..n {
            dst[c * n + r] = src[r * n + c];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cosine_mode(dim: usize, a: usize, b: usize) -> Vec<f64> {
        let mut density = vec![0.0; dim * dim];
        for y in 0..dim {
            for x in 0..dim {
                let cx = (PI * a as f64 * (x as f64 + 0.5) / dim as f64).cos();
                let cy = (PI * b as f64 * (y as f64 + 0.5) / dim as f64).cos();
                density[y * dim + x] = cx * cy;
            }
        }
        density
    }

    fn solve(density: &[f64], dim: usize, bin: (f64, f64)) -> Vec<f64> {
        let mut solver = PoissonSolver::new(dim, bin.0, bin.1);
        let mut potential = vec![0.0; dim * dim];
        solver.solve(density, &mut potential);
        potential
    }

    fn assert_solves_cosine_mode(dim: usize, a: usize, b: usize, bin: (f64, f64)) {
        let density = cosine_mode(dim, a, b);
        let wa = PI * a as f64 / dim as f64 / bin.0;
        let wb = PI * b as f64 / dim as f64 / bin.1;
        let eigenvalue = wa.mul_add(wa, wb * wb);

        let potential = solve(&density, dim, bin);

        for (phi, rho) in potential.iter().zip(&density) {
            assert!(
                (phi - rho / eigenvalue).abs() < 1e-9 * (1.0 + (rho / eigenvalue).abs()),
                "mode ({a}, {b}): got {phi}, expected {}",
                rho / eigenvalue
            );
        }
    }

    #[test]
    fn potential_of_a_cosine_mode_is_the_mode_over_its_eigenvalue() {
        assert_solves_cosine_mode(16, 1, 2, (1.0, 1.0));
    }

    #[test]
    fn potential_distinguishes_x_and_y_frequencies() {
        assert_solves_cosine_mode(16, 5, 0, (1.0, 1.0));
        assert_solves_cosine_mode(16, 0, 5, (1.0, 1.0));
    }

    #[test]
    fn potential_of_high_frequency_modes_uses_negative_frequencies() {
        assert_solves_cosine_mode(16, 15, 11, (1.0, 1.0));
    }

    #[test]
    fn potential_scales_each_axis_by_its_bin_size() {
        assert_solves_cosine_mode(16, 3, 2, (2.5, 0.5));
    }

    #[test]
    fn uniform_density_produces_no_potential() {
        let potential = solve(&[0.25; 64], 8, (1.0, 1.0));

        assert!(potential.iter().all(|phi| phi.abs() < 1e-12));
    }
}
