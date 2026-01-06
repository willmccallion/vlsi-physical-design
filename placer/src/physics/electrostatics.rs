use super::PhysicsContext;
use eda_common::db::core::NetlistDB;
use eda_common::geom::point::Point;
use rustfft::num_complex::Complex;
use std::f64::consts::PI;

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

    // Reset density map
    ctx.density_map.fill(0.0);

    // Binning
    for (i, pos) in positions.iter().enumerate() {
        let cell = &db.cells[i];

        let min_x = pos.x - cell.width / 2.0;
        let max_x = pos.x + cell.width / 2.0;
        let min_y = pos.y - cell.height / 2.0;
        let max_y = pos.y + cell.height / 2.0;

        let start_col = ((min_x - db.die_area.min.x) / bin_w).floor() as isize;
        let end_col = ((max_x - db.die_area.min.x) / bin_w).floor() as isize;
        let start_row = ((min_y - db.die_area.min.y) / bin_h).floor() as isize;
        let end_row = ((max_y - db.die_area.min.y) / bin_h).floor() as isize;

        let start_col = start_col.max(0).min(dim as isize - 1);
        let end_col = end_col.max(0).min(dim as isize - 1);
        let start_row = start_row.max(0).min(dim as isize - 1);
        let end_row = end_row.max(0).min(dim as isize - 1);

        let bins_covered = ((end_col - start_col + 1) * (end_row - start_row + 1)) as f64;
        let density_per_bin = (cell.width * cell.height) / bins_covered;

        for r in start_row..=end_row {
            for c in start_col..=end_col {
                let idx = (r as usize) * dim + (c as usize);
                ctx.density_map[idx] += density_per_bin;
            }
        }
    }

    // Overflow calculation
    let mut overflow = 0.0;
    for val in ctx.density_map.iter_mut() {
        *val /= bin_area;
        if *val > target_density {
            overflow += (*val - target_density).powi(2);
        }
        *val -= target_density;
    }

    // FFT
    let fft = ctx.fft_planner.plan_fft_forward(dim * dim);
    let ifft = ctx.fft_planner.plan_fft_inverse(dim * dim);

    for (i, &rho) in ctx.density_map.iter().enumerate() {
        ctx.fft_scratch[i] = Complex::new(rho, 0.0);
    }

    fft.process(&mut ctx.fft_scratch);

    for v in 0..dim {
        for u in 0..dim {
            let idx = v * dim + u;
            let wu = 2.0 * PI * (u as f64) / (dim as f64);
            let wv = 2.0 * PI * (v as f64) / (dim as f64);
            let denom = wu * wu + wv * wv;

            if denom > 1e-9 {
                ctx.fft_scratch[idx] /= denom;
            } else {
                ctx.fft_scratch[idx] = Complex::new(0.0, 0.0);
            }
        }
    }

    ifft.process(&mut ctx.fft_scratch);

    let norm = 1.0 / (dim * dim) as f64;
    for (i, c) in ctx.fft_scratch.iter().enumerate() {
        ctx.potential_map[i] = c.re * norm;
    }

    // Gradient Calculation with Boundary Fix
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

    // Apply forces
    for (i, pos) in positions.iter().enumerate() {
        if db.cells[i].is_fixed {
            continue;
        }

        let col = ((pos.x - db.die_area.min.x) / bin_w).floor() as isize;
        let row = ((pos.y - db.die_area.min.y) / bin_h).floor() as isize;

        if col >= 0 && col < dim as isize && row >= 0 && row < dim as isize {
            let idx = (row as usize) * dim + (col as usize);
            gradients[i].x -= ctx.electro_force_x[idx] * force_multiplier;
            gradients[i].y -= ctx.electro_force_y[idx] * force_multiplier;
        }
    }

    overflow
}
