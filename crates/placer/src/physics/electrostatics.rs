//! Electrostatic Density Force Computation.
//!
//! Implements density penalty computation using an electrostatic analogy where
//! cells are charges and density violations create repulsive forces. Uses FFT
//! with mirror padding (DCT-like Neumann boundary conditions) to efficiently
//! compute the potential field in O(N log N) time.

use super::PhysicsContext;
use pare_common::db::core::NetlistDB;
use pare_common::geom::point::Point;
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

    // --- Step 2: Mirror-padded FFT (DCT-like Neumann boundaries) ---
    let dim2 = 2 * dim;

    // Build mirrored 2N×2N density map
    for y in 0..dim {
        for x in 0..dim {
            let val = ctx.density_map[y * dim + x];
            ctx.mirror_map[y * dim2 + x] = val;
            ctx.mirror_map[y * dim2 + (dim2 - 1 - x)] = val;
            ctx.mirror_map[(dim2 - 1 - y) * dim2 + x] = val;
            ctx.mirror_map[(dim2 - 1 - y) * dim2 + (dim2 - 1 - x)] = val;
        }
    }

    // FFT on 2N×2N
    let fft = ctx.fft_planner.plan_fft_forward(dim2 * dim2);
    let ifft = ctx.fft_planner.plan_fft_inverse(dim2 * dim2);

    for (i, &rho) in ctx.mirror_map.iter().enumerate() {
        ctx.fft_scratch[i] = Complex::new(rho, 0.0);
    }

    fft.process(&mut ctx.fft_scratch);

    // Apply Green's function in frequency domain
    for v in 0..dim2 {
        for u in 0..dim2 {
            let idx = v * dim2 + u;
            let wu = 2.0 * PI * (u as f64) / (dim2 as f64);
            let wv = 2.0 * PI * (v as f64) / (dim2 as f64);
            let denom = wu * wu + wv * wv;

            if denom > 1e-9 {
                ctx.fft_scratch[idx] /= denom;
            } else {
                ctx.fft_scratch[idx] = Complex::new(0.0, 0.0);
            }
        }
    }

    ifft.process(&mut ctx.fft_scratch);

    // Extract N×N potential from the top-left quadrant
    let norm = 1.0 / (dim2 * dim2) as f64;
    for y in 0..dim {
        for x in 0..dim {
            ctx.potential_map[y * dim + x] = ctx.fft_scratch[y * dim2 + x].re * norm;
        }
    }

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
