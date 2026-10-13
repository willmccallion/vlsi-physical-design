//! Electrostatic Density Force Computation.
//!
//! Implements density penalty computation using an electrostatic analogy where
//! cells are charges and density violations create repulsive forces. Uses FFT
//! with mirror padding (DCT-like Neumann boundary conditions) to efficiently
//! compute the potential field in O(N log N) time.

use super::PhysicsContext;
use pare_common::db::core::NetlistDB;
use pare_common::geom::point::Point;
use rustfft::FftPlanner;
use rustfft::num_complex::Complex;
use std::f64::consts::PI;

/// Computes density forces and adds them to the gradient array.
///
/// Uses exact area overlap for density distribution and mirror-padded FFT
/// (equivalent to DCT) for Neumann boundary conditions that prevent
/// wraparound artifacts.
pub fn compute_density_force(
    ctx: &mut PhysicsContext,
    db: &NetlistDB,
    positions: &[Point<f64>],
    target_density: f64,
    force_multiplier: f64,
    gradients: &mut [Point<f64>],
) -> f64 {
    let dim = ctx.bin_dim;
    let bin_w = db.die_area.width() / dim as f64;
    let bin_h = db.die_area.height() / dim as f64;
    let bin_area = bin_w * bin_h;

    // --- Step 1: Exact area overlap density distribution ---
    ctx.density_map.fill(0.0);

    for (i, pos) in positions.iter().enumerate() {
        let cell = &db.cells[i];

        // Cell bounding box (position is lower-left corner)
        let cell_min_x = pos.x;
        let cell_max_x = pos.x + cell.width;
        let cell_min_y = pos.y;
        let cell_max_y = pos.y + cell.height;

        // Bin range that the cell overlaps
        let start_col = ((cell_min_x - db.die_area.min.x) / bin_w).floor() as isize;
        let end_col = ((cell_max_x - db.die_area.min.x) / bin_w).ceil() as isize;
        let start_row = ((cell_min_y - db.die_area.min.y) / bin_h).floor() as isize;
        let end_row = ((cell_max_y - db.die_area.min.y) / bin_h).ceil() as isize;

        let start_col = start_col.max(0).min(dim as isize - 1);
        let end_col = end_col.max(0).min(dim as isize);
        let start_row = start_row.max(0).min(dim as isize - 1);
        let end_row = end_row.max(0).min(dim as isize);

        for r in start_row..end_row {
            for c in start_col..end_col {
                // Bin bounding box
                let bx_min = (c as f64).mul_add(bin_w, db.die_area.min.x);
                let bx_max = bx_min + bin_w;
                let by_min = (r as f64).mul_add(bin_h, db.die_area.min.y);
                let by_max = by_min + bin_h;

                // Overlap area between cell and bin
                let ox = (cell_max_x.min(bx_max) - cell_min_x.max(bx_min)).max(0.0);
                let oy = (cell_max_y.min(by_max) - cell_min_y.max(by_min)).max(0.0);
                let overlap = ox * oy;

                let idx = (r as usize) * dim + (c as usize);
                ctx.density_map[idx] += overlap;
            }
        }
    }

    // Normalize to density and compute overflow
    let mut overflow = 0.0;
    for val in &mut ctx.density_map {
        *val /= bin_area;
        if *val > target_density {
            overflow += (*val - target_density).powi(2);
        }
        *val -= target_density;
    }

    solve_potential(
        &mut ctx.fft_planner,
        &ctx.density_map,
        dim,
        &mut ctx.fft_scratch,
        &mut ctx.fft_transpose,
        &mut ctx.potential_map,
    );

    // --- Step 3: Compute force field via finite differences ---
    for y in 0..dim {
        for x in 0..dim {
            let idx = y * dim + x;

            let l = if x > 0 {
                ctx.potential_map[idx - 1]
            } else {
                ctx.potential_map[idx]
            };
            let r = if x < dim - 1 {
                ctx.potential_map[idx + 1]
            } else {
                ctx.potential_map[idx]
            };

            let d = if y > 0 {
                ctx.potential_map[idx - dim]
            } else {
                ctx.potential_map[idx]
            };
            let u = if y < dim - 1 {
                ctx.potential_map[idx + dim]
            } else {
                ctx.potential_map[idx]
            };

            ctx.electro_force_x[idx] = -(r - l) / (2.0 * bin_w);
            ctx.electro_force_y[idx] = -(u - d) / (2.0 * bin_h);
        }
    }

    // --- Step 4: Apply forces to cells using bilinear interpolation at cell center ---
    for (i, pos) in positions.iter().enumerate() {
        if db.cells[i].is_fixed {
            continue;
        }

        // Use cell center for force lookup
        let cx = pos.x + db.cells[i].width / 2.0;
        let cy = pos.y + db.cells[i].height / 2.0;

        let fx = (cx - db.die_area.min.x) / bin_w - 0.5;
        let fy = (cy - db.die_area.min.y) / bin_h - 0.5;

        let ix = fx.floor() as isize;
        let iy = fy.floor() as isize;

        let alpha_x = fx - ix as f64;
        let alpha_y = fy - iy as f64;

        let weights = [
            ((1.0 - alpha_x) * (1.0 - alpha_y), ix, iy),
            (alpha_x * (1.0 - alpha_y), ix + 1, iy),
            ((1.0 - alpha_x) * alpha_y, ix, iy + 1),
            (alpha_x * alpha_y, ix + 1, iy + 1),
        ];

        let mut force_x = 0.0;
        let mut force_y = 0.0;

        for &(w, bx, by) in &weights {
            let bx = bx.clamp(0, dim as isize - 1) as usize;
            let by = by.clamp(0, dim as isize - 1) as usize;
            let idx = by * dim + bx;
            force_x += w * ctx.electro_force_x[idx];
            force_y += w * ctx.electro_force_y[idx];
        }

        gradients[i].x -= force_x * force_multiplier;
        gradients[i].y -= force_y * force_multiplier;
    }

    overflow
}

/// Solves `-∇²φ = ρ` on a `dim`×`dim` grid in bin units with zero-flux edges.
///
/// Mirroring the grid to 2N×2N makes it even, so a periodic 2D FFT solve
/// gives the Neumann (DCT) solution. `scratch` and `transposed` hold 4N² values.
fn solve_potential(
    planner: &mut FftPlanner<f64>,
    density: &[f64],
    dim: usize,
    scratch: &mut [Complex<f64>],
    transposed: &mut [Complex<f64>],
    potential: &mut [f64],
) {
    let dim2 = 2 * dim;

    for y in 0..dim {
        for x in 0..dim {
            let val = Complex::new(density[y * dim + x], 0.0);
            scratch[y * dim2 + x] = val;
            scratch[y * dim2 + (dim2 - 1 - x)] = val;
            scratch[(dim2 - 1 - y) * dim2 + x] = val;
            scratch[(dim2 - 1 - y) * dim2 + (dim2 - 1 - x)] = val;
        }
    }

    let fft = planner.plan_fft_forward(dim2);
    let ifft = planner.plan_fft_inverse(dim2);

    fft.process(scratch);
    transpose(scratch, transposed, dim2);
    fft.process(transposed);

    for u in 0..dim2 {
        let wu = angular_frequency(u, dim2);
        for v in 0..dim2 {
            let wv = angular_frequency(v, dim2);
            let denom = wu.mul_add(wu, wv * wv);
            let idx = u * dim2 + v;
            transposed[idx] = if denom > 1e-12 {
                transposed[idx] / denom
            } else {
                Complex::new(0.0, 0.0)
            };
        }
    }

    ifft.process(transposed);
    transpose(transposed, scratch, dim2);
    ifft.process(scratch);

    let norm = 1.0 / (dim2 * dim2) as f64;
    for y in 0..dim {
        for x in 0..dim {
            potential[y * dim + x] = scratch[y * dim2 + x].re * norm;
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

    fn solve(density: &[f64], dim: usize) -> Vec<f64> {
        let mut planner = FftPlanner::new();
        let mut scratch = vec![Complex::default(); 4 * dim * dim];
        let mut transposed = vec![Complex::default(); 4 * dim * dim];
        let mut potential = vec![0.0; dim * dim];
        solve_potential(
            &mut planner,
            density,
            dim,
            &mut scratch,
            &mut transposed,
            &mut potential,
        );
        potential
    }

    fn assert_solves_cosine_mode(dim: usize, a: usize, b: usize) {
        let density = cosine_mode(dim, a, b);
        let wa = PI * a as f64 / dim as f64;
        let wb = PI * b as f64 / dim as f64;
        let eigenvalue = wa.mul_add(wa, wb * wb);

        let potential = solve(&density, dim);

        for (phi, rho) in potential.iter().zip(&density) {
            assert!(
                (phi - rho / eigenvalue).abs() < 1e-9,
                "mode ({a}, {b}): got {phi}, expected {}",
                rho / eigenvalue
            );
        }
    }

    #[test]
    fn potential_of_a_cosine_mode_is_the_mode_over_its_eigenvalue() {
        assert_solves_cosine_mode(16, 1, 2);
    }

    #[test]
    fn potential_distinguishes_x_and_y_frequencies() {
        assert_solves_cosine_mode(16, 5, 0);
        assert_solves_cosine_mode(16, 0, 5);
    }

    #[test]
    fn potential_of_high_frequency_modes_uses_negative_frequencies() {
        assert_solves_cosine_mode(16, 15, 11);
    }

    #[test]
    fn uniform_density_produces_no_potential() {
        let potential = solve(&vec![0.25; 64], 8);

        assert!(potential.iter().all(|phi| phi.abs() < 1e-12));
    }
}
