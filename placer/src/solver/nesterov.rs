//! Nesterov-Accelerated Gradient Descent Optimizer.
//!
//! Implements Nesterov's accelerated gradient descent with adaptive density
//! weighting inspired by ePlace/DREAMPlace. The density force multiplier is
//! initialized from the WL/density gradient norm ratio and updated based on
//! HPWL changes, replacing the fixed electro_force_multiplier.

use crate::physics::PhysicsContext;
use eda_common::db::core::NetlistDB;
use eda_common::geom::point::Point;
use rand::Rng;

/// Parameters controlling the Nesterov optimizer's behavior.
pub struct NesterovParams {
    pub max_iterations: usize,
    pub initial_learning_rate: f64,
    pub convergence_threshold: f64,
    pub wa_gamma: f64,
    pub target_density: f64,
    pub electro_force_multiplier: f64,
}

/// Nesterov-accelerated gradient descent optimizer for placement.
pub struct NesterovOptimizer {
    params: NesterovParams,
    x_k: Vec<Point<f64>>,
    x_prev: Vec<Point<f64>>,
    y_k: Vec<Point<f64>>,
    grads: Vec<Point<f64>>,
    density_grads: Vec<Point<f64>>,
}

impl NesterovOptimizer {
    /// Creates a new Nesterov optimizer with the specified parameters.
    pub fn new(params: NesterovParams, num_movable: usize) -> Self {
        Self {
            params,
            x_k: vec![Point::default(); num_movable],
            x_prev: vec![Point::default(); num_movable],
            y_k: vec![Point::default(); num_movable],
            grads: vec![Point::default(); num_movable],
            density_grads: vec![Point::default(); num_movable],
        }
    }

    /// Optimizes cell positions to minimize wirelength and density violations.
    ///
    /// Uses Nesterov acceleration with adaptive density weighting: the density
    /// force multiplier is set from the WL/density gradient norm ratio and
    /// adjusted based on HPWL trends (DREAMPlace/RePlAce style).
    pub fn optimize(
        &mut self,
        db: &mut NetlistDB,
        physics: &mut PhysicsContext,
    ) -> Result<(), String> {
        let n = db.positions.len();
        self.x_k.copy_from_slice(&db.positions);

        let mut rng = rand::thread_rng();
        let center_x = (db.die_area.min.x + db.die_area.max.x) / 2.0;
        let center_y = (db.die_area.min.y + db.die_area.max.y) / 2.0;
        let noise_scale_x = db.die_area.width() * 0.25;
        let noise_scale_y = db.die_area.height() * 0.25;

        for (i, pos) in self.x_k.iter_mut().enumerate() {
            if !db.cells[i].is_fixed {
                pos.x = center_x + rng.gen_range(-noise_scale_x..noise_scale_x);
                pos.y = center_y + rng.gen_range(-noise_scale_y..noise_scale_y);
                pos.x = pos.x.clamp(db.die_area.min.x, db.die_area.max.x - db.cells[i].width);
                pos.y = pos.y.clamp(db.die_area.min.y, db.die_area.max.y - db.cells[i].height);
            }
        }

        self.x_prev.copy_from_slice(&self.x_k);
        self.y_k.copy_from_slice(&self.x_k);

        let mut a_k: f64 = 1.0;
        let mut step_size = self.params.initial_learning_rate;

        // --- Compute initial gradients to calibrate density weight ---
        let (wl_cost_0, _, wl_grad_norm_0, density_grad_norm_0) =
            physics.compute_gradients_separate(
                db,
                &self.y_k,
                &mut self.grads,
                &mut self.density_grads,
                self.params.wa_gamma,
                self.params.target_density,
            );

        // Initialize density weight from gradient norm ratio (ePlace approach)
        let base_ratio = if density_grad_norm_0 > 1e-20 && wl_grad_norm_0 > 1e-20 {
            wl_grad_norm_0 / density_grad_norm_0
        } else {
            self.params.electro_force_multiplier
        };
        let mut density_weight = base_ratio * 0.5;

        let total_bins = (physics.bin_dim * physics.bin_dim) as f64;
        let mut prev_hpwl = wl_cost_0;
        let ref_hpwl = wl_cost_0.max(1.0);

        for k in 0..self.params.max_iterations {
            // Compute WL and density gradients separately
            let (wl_cost, density_cost, wl_grad_norm, density_grad_norm) =
                physics.compute_gradients_separate(
                    db,
                    &self.y_k,
                    &mut self.grads,
                    &mut self.density_grads,
                    self.params.wa_gamma,
                    self.params.target_density,
                );

            // Combine: grad = wl_grad + density_weight * density_grad
            for i in 0..n {
                self.grads[i].x += self.density_grads[i].x * density_weight;
                self.grads[i].y += self.density_grads[i].y * density_weight;
            }

            // --- Density weight update (every 10 iters, DREAMPlace style) ---
            if k > 0 && k % 10 == 0 {
                let delta_hpwl = wl_cost - prev_hpwl;
                let mu = if delta_hpwl < 0.0 {
                    // HPWL improved: can afford more density pressure
                    1.05 * 0.9999_f64.powi(k as i32).max(0.98)
                } else {
                    // HPWL worsened: back off density pressure
                    1.05_f64.powf(-(delta_hpwl / ref_hpwl).min(10.0)).max(0.95)
                };
                density_weight *= mu;
                prev_hpwl = wl_cost;

                // Re-anchor to gradient norm ratio — target grows with progress
                if k % 50 == 0 && density_grad_norm > 1e-20 && wl_grad_norm > 1e-20 {
                    let progress = (k as f64 / self.params.max_iterations as f64).min(1.0);
                    let target_ratio = 0.5 + 1.5 * progress;
                    let ideal = base_ratio * target_ratio;
                    // Blend: only move toward ideal, never below current
                    density_weight = density_weight.max(density_weight * 0.7 + ideal * 0.3);
                }
            }

            // --- Metrics ---
            let mut total_disp = 0.0;
            for (curr, prev) in self.x_k.iter().zip(self.x_prev.iter()) {
                total_disp += (curr.x - prev.x).abs() + (curr.y - prev.y).abs();
            }
            let avg_disp = total_disp / n as f64;

            let overflow_ratio = physics
                .density_map
                .iter()
                .filter(|&&v| v > 0.0)
                .count() as f64
                / total_bins;

            if k % 100 == 0 {
                log::info!(
                    "Iter {}: WL={:.0} Density={:.0} DensW={:.2e} Step={:.5} AvgMove={:.4} OvfRatio={:.3}",
                    k, wl_cost, density_cost, density_weight, step_size, avg_disp, overflow_ratio,
                );
            }

            // --- Convergence ---
            if k > 200 && overflow_ratio < 0.05 && avg_disp < 0.1 {
                log::info!(
                    "Converged at iteration {}: overflow={:.4}, avg_disp={:.4}",
                    k, overflow_ratio, avg_disp
                );
                Self::apply_clamping(db, &mut self.x_k);
                db.positions.copy_from_slice(&self.x_k);
                return Ok(());
            }

            if k > 500 && avg_disp < self.params.convergence_threshold && density_cost < 50000.0 {
                log::info!("Converged: Cells stabilized at iteration {}", k);
                Self::apply_clamping(db, &mut self.x_k);
                db.positions.copy_from_slice(&self.x_k);
                return Ok(());
            }

            // --- Nesterov update (original working scheme) ---
            let mut x_next: Vec<Point<f64>> = self
                .y_k
                .iter()
                .zip(self.grads.iter())
                .map(|(y, g)| *y - *g * step_size)
                .collect();

            Self::apply_clamping(db, &mut x_next);

            let a_next = (1.0 + (4.0 * a_k * a_k + 1.0).sqrt()) / 2.0;
            let momentum = (a_k - 1.0) / a_next;

            #[allow(clippy::needless_range_loop)]
            for i in 0..n {
                if db.cells[i].is_fixed {
                    self.y_k[i] = self.x_k[i];
                    continue;
                }
                self.y_k[i] = x_next[i] + (x_next[i] - self.x_k[i]) * momentum;
            }

            Self::apply_clamping(db, &mut self.y_k);

            self.x_prev.copy_from_slice(&self.x_k);
            self.x_k = x_next;
            a_k = a_next;

            // Step decay in second half
            if k >= self.params.max_iterations / 2 {
                step_size *= 0.9995;
            }
        }

        Self::apply_clamping(db, &mut self.x_k);
        db.positions.copy_from_slice(&self.x_k);
        log::warn!("Placer reached max iterations. Proceeding with current solution.");
        Ok(())
    }

    /// Clamps cell positions to ensure they remain within die boundaries.
    fn apply_clamping(db: &NetlistDB, positions: &mut [Point<f64>]) {
        let die_min_x = db.die_area.min.x;
        let die_min_y = db.die_area.min.y;
        let die_max_x = db.die_area.max.x;
        let die_max_y = db.die_area.max.y;

        for (i, pos) in positions.iter_mut().enumerate() {
            if db.cells[i].is_fixed {
                continue;
            }
            let w = db.cells[i].width;
            let h = db.cells[i].height;
            pos.x = pos.x.clamp(die_min_x, die_max_x - w);
            pos.y = pos.y.clamp(die_min_y, die_max_y - h);
        }
    }
}
