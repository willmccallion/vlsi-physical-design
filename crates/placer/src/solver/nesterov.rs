//! Nesterov-Accelerated Gradient Descent Optimizer.
//!
//! Implements Nesterov's accelerated gradient descent with adaptive density
//! weighting inspired by ePlace/DREAMPlace. The density force multiplier is
//! initialized from the WL/density gradient norm ratio and updated based on
//! HPWL changes, replacing the fixed electro_force_multiplier.

use crate::physics::PhysicsContext;
use pare_common::db::core::NetlistDB;
use pare_common::geom::point::Point;
use pare_common::util::ui;
use rand::Rng;

/// Result returned by the placement optimizer.
pub struct PlacementResult {
    pub converged: bool,
    pub iterations: usize,
    pub final_overflow: f64,
    pub final_wirelength: f64,
}

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
    pub fn optimize(
        &mut self,
        db: &mut NetlistDB,
        physics: &mut PhysicsContext,
    ) -> Result<PlacementResult, String> {
        let n = db.positions.len();
        self.x_k.copy_from_slice(&db.positions);

        // Check whether the parsed positions are already a reasonable initial placement
        let in_bounds_count = self.x_k.iter().enumerate()
            .filter(|(i, pos)| {
                let cell = &db.cells[*i];
                !cell.is_fixed
                    && pos.x >= db.die_area.min.x
                    && pos.y >= db.die_area.min.y
                    && pos.x + cell.width <= db.die_area.max.x
                    && pos.y + cell.height <= db.die_area.max.y
            })
            .count();
        let movable_count = db.cells.iter().filter(|c| !c.is_fixed).count();
        let use_initial_placement = movable_count > 0
            && (in_bounds_count as f64 / movable_count as f64) > 0.90;

        if use_initial_placement {
            for (i, pos) in self.x_k.iter_mut().enumerate() {
                if !db.cells[i].is_fixed {
                    pos.x = pos.x.clamp(
                        db.die_area.min.x,
                        db.die_area.max.x - db.cells[i].width,
                    );
                    pos.y = pos.y.clamp(
                        db.die_area.min.y,
                        db.die_area.max.y - db.cells[i].height,
                    );
                }
            }
        } else {
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

        // Precompute bin geometry for overflow metric
        let dim = physics.bin_dim;
        let bin_w = db.die_area.width() / dim as f64;
        let bin_h = db.die_area.height() / dim as f64;
        let bin_area = bin_w * bin_h;
        let total_cell_area: f64 = db.cells.iter()
            .filter(|c| !c.is_fixed)
            .map(|c| c.width * c.height)
            .sum();

        let mut prev_hpwl = wl_cost_0;
        let ref_hpwl = wl_cost_0.max(1.0);
        let initial_hpwl = wl_cost_0;

        // Track best solution for rollback on instability
        let mut best_positions = self.x_k.clone();
        let mut best_overflow = f64::INFINITY;
        let mut best_wl = f64::INFINITY;
        let mut prev_avg_disp = 0.0;
        let mut wl_stable_iters = 0u32;
        let mut prev_wl = wl_cost_0;

        for k in 0..self.params.max_iterations {
            let (wl_cost, _density_cost, wl_grad_norm, density_grad_norm) =
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

            // Add congestion gradient if congestion map is available
            let _congestion_cost = physics.compute_congestion_gradient(
                db,
                &self.y_k,
                &mut self.grads,
                density_weight * 0.1,
            );

            // Compute overflow from the density map (populated by compute_gradients_separate)
            let overflow_area: f64 = physics
                .density_map
                .iter()
                .filter(|&&v| v > 0.0)
                .map(|&v| v * bin_area)
                .sum();
            let overflow_ratio = if total_cell_area > 0.0 {
                overflow_area / total_cell_area
            } else {
                0.0
            };

            // --- Density weight update (every 10 iters) ---
            // Only grow density weight if overflow is still too high.
            // Once overflow is acceptable, freeze the weight and let cells
            // settle under wirelength optimization with fixed density pressure.
            if k > 0 && k % 10 == 0 && overflow_ratio > 0.10 {
                // Multiplicative growth based on HPWL trend
                let delta_hpwl = wl_cost - prev_hpwl;
                let mu = if delta_hpwl < 0.0 {
                    1.05
                } else {
                    let slowdown = (delta_hpwl / ref_hpwl).min(1.0);
                    1.01 + 0.04 * (1.0 - slowdown)
                };
                density_weight *= mu;

                // Re-anchor to gradient norm ratio with capped multiplier.
                // Uses sqrt(progress) instead of progress² to ramp more
                // gently, and caps the multiplier to prevent runaway growth.
                if density_grad_norm > 1e-20 && wl_grad_norm > 1e-20 {
                    let current_ratio = wl_grad_norm / density_grad_norm;
                    let progress = (k as f64 / self.params.max_iterations as f64).min(1.0);
                    let target_multiplier = 1.0 + 15.0 * progress.sqrt();
                    let ideal = current_ratio * target_multiplier;
                    density_weight = density_weight.max(ideal);
                }

                // Cap density weight relative to initial calibration to
                // prevent unbounded growth that destabilizes large designs.
                let max_density_weight = base_ratio * 1e6;
                density_weight = density_weight.min(max_density_weight);

                prev_hpwl = wl_cost;
            }

            // --- Metrics ---
            let mut total_disp = 0.0;
            for (curr, prev) in self.x_k.iter().zip(self.x_prev.iter()) {
                total_disp += (curr.x - prev.x).abs() + (curr.y - prev.y).abs();
            }
            let avg_disp = total_disp / n as f64;

            // Track best solution (prefer lower overflow, break ties by WL)
            if overflow_ratio < best_overflow
                || (overflow_ratio < best_overflow + 0.01 && wl_cost < best_wl)
            {
                best_overflow = overflow_ratio;
                best_wl = wl_cost;
                best_positions.copy_from_slice(&self.x_k);
            }

            // Track wirelength stability: count consecutive iterations where
            // WL changes by less than 0.1% of the reference.
            if k > 0 {
                let wl_change = (wl_cost - prev_wl).abs() / ref_hpwl;
                if wl_change < 0.001 {
                    wl_stable_iters += 1;
                } else {
                    wl_stable_iters = 0;
                }
            }
            prev_wl = wl_cost;

            if k % 100 == 0 {
                ui::placement_iter(k, wl_cost, overflow_ratio, avg_disp, step_size);
            }

            // --- Instability detection ---
            // Sudden spike: movement explodes in a single step
            if k > 100 && avg_disp > prev_avg_disp * 5.0 && avg_disp > 2.0 {
                log::debug!(
                    "Instability detected at iter {} (avg_disp={:.2}). Rolling back to best (overflow={:.4})",
                    k, avg_disp, best_overflow
                );
                Self::apply_clamping(db, &mut best_positions);
                db.positions.copy_from_slice(&best_positions);
                return Ok(PlacementResult {
                    converged: true, iterations: k,
                    final_overflow: best_overflow, final_wirelength: wl_cost,
                });
            }
            // Overflow regression: had a good solution but now diverging badly
            if k > 200 && best_overflow < 0.10 && overflow_ratio > 0.50 {
                log::debug!(
                    "Divergence detected at iter {} (overflow={:.3}, best was {:.4}). Rolling back.",
                    k, overflow_ratio, best_overflow
                );
                Self::apply_clamping(db, &mut best_positions);
                db.positions.copy_from_slice(&best_positions);
                return Ok(PlacementResult {
                    converged: true, iterations: k,
                    final_overflow: best_overflow, final_wirelength: wl_cost,
                });
            }
            if k > 0 {
                prev_avg_disp = avg_disp.max(0.01);
            }

            // --- Convergence ---
            // Require that wirelength has actually improved from the initial
            // value before declaring convergence. This prevents premature exit
            // for pre-placed designs where overflow starts low but WL hasn't
            // been optimized yet.
            let wl_improved = wl_cost < initial_hpwl * 0.95;
            let wl_stable = wl_stable_iters >= 50;

            if k > 200 && overflow_ratio < 0.10 && avg_disp < 0.1
                && (wl_improved || wl_stable)
            {
                log::debug!(
                    "Converged at iteration {}: overflow={:.4}, avg_disp={:.4}, WL={:.0}",
                    k, overflow_ratio, avg_disp, wl_cost
                );
                Self::apply_clamping(db, &mut self.x_k);
                db.positions.copy_from_slice(&self.x_k);
                return Ok(PlacementResult {
                    converged: true, iterations: k,
                    final_overflow: overflow_ratio, final_wirelength: wl_cost,
                });
            }

            if k > 500 && avg_disp < self.params.convergence_threshold
                && (wl_improved || wl_stable)
            {
                log::debug!("Converged: Cells stabilized at iteration {} (overflow={:.4})", k, overflow_ratio);
                Self::apply_clamping(db, &mut self.x_k);
                db.positions.copy_from_slice(&self.x_k);
                return Ok(PlacementResult {
                    converged: true, iterations: k,
                    final_overflow: overflow_ratio, final_wirelength: wl_cost,
                });
            }

            // --- Nesterov update ---
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

        // Use best solution found during optimization
        Self::apply_clamping(db, &mut best_positions);
        db.positions.copy_from_slice(&best_positions);
        log::warn!(
            "Placer reached max iterations. Using best solution (overflow={:.4}).",
            best_overflow
        );
        Ok(PlacementResult {
            converged: false,
            iterations: self.params.max_iterations,
            final_overflow: best_overflow,
            final_wirelength: 0.0,
        })
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
