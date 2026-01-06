use crate::physics::PhysicsContext;
use eda_common::db::core::NetlistDB;
use eda_common::geom::point::Point;
use rand::Rng;

pub struct NesterovParams {
    pub max_iterations: usize,
    pub initial_learning_rate: f64,
    pub convergence_threshold: f64,
    pub wa_gamma: f64,
    pub target_density: f64,
    pub electro_force_multiplier: f64,
}

pub struct NesterovOptimizer {
    params: NesterovParams,
    x_k: Vec<Point<f64>>,
    x_prev: Vec<Point<f64>>,
    y_k: Vec<Point<f64>>,
    grads: Vec<Point<f64>>,
}

impl NesterovOptimizer {
    pub fn new(params: NesterovParams, num_movable: usize) -> Self {
        Self {
            params,
            x_k: vec![Point::default(); num_movable],
            x_prev: vec![Point::default(); num_movable],
            y_k: vec![Point::default(); num_movable],
            grads: vec![Point::default(); num_movable],
        }
    }

    pub fn optimize(
        &mut self,
        db: &mut NetlistDB,
        physics: &mut PhysicsContext,
    ) -> Result<(), String> {
        self.x_k.copy_from_slice(&db.positions);

        let mut rng = rand::thread_rng();

        let center_x = (db.die_area.min.x + db.die_area.max.x) / 2.0;
        let center_y = (db.die_area.min.y + db.die_area.max.y) / 2.0;

        let noise_scale_x = db.die_area.width() * 0.25;
        let noise_scale_y = db.die_area.height() * 0.25;

        for (i, pos) in self.x_k.iter_mut().enumerate() {
            if !db.cells[i].is_fixed {
                let noise_x = rng.gen_range(-noise_scale_x..noise_scale_x);
                let noise_y = rng.gen_range(-noise_scale_y..noise_scale_y);

                pos.x = center_x + noise_x;
                pos.y = center_y + noise_y;

                pos.x = pos
                    .x
                    .clamp(db.die_area.min.x, db.die_area.max.x - db.cells[i].width);
                pos.y = pos
                    .y
                    .clamp(db.die_area.min.y, db.die_area.max.y - db.cells[i].height);
            }
        }

        // Initialize history vectors
        self.x_prev.copy_from_slice(&self.x_k);
        self.y_k.copy_from_slice(&self.x_k);

        let mut a_k: f64 = 1.0;
        let mut step_size = self.params.initial_learning_rate;

        for k in 0..self.params.max_iterations {
            // Compute gradient at y_k
            let (wl_cost, density_cost) = physics.compute_gradients(
                db,
                &self.y_k,
                &mut self.grads,
                self.params.wa_gamma,
                self.params.target_density,
                self.params.electro_force_multiplier,
            );

            // Convergence check
            let mut total_disp = 0.0;
            for (curr, prev) in self.x_k.iter().zip(self.x_prev.iter()) {
                total_disp += (curr.x - prev.x).abs() + (curr.y - prev.y).abs();
            }
            let avg_disp = total_disp / self.x_k.len() as f64;

            if k % 100 == 0 {
                log::info!(
                    "Iter {}: WL={:.0} Density={:.0} Step={:.5} AvgMove={:.4}",
                    k,
                    wl_cost,
                    density_cost,
                    step_size,
                    avg_disp
                );
            }

            // Convergence Condition
            if k > 500 && avg_disp < self.params.convergence_threshold && density_cost < 50000.0 {
                log::info!("Converged: Cells stabilized at iteration {}", k);
                Self::apply_clamping(db, &mut self.x_k);
                db.positions.copy_from_slice(&self.x_k);
                return Ok(());
            }

            // Nesterov Update
            let mut x_next: Vec<Point<f64>> = self
                .y_k
                .iter()
                .zip(self.grads.iter())
                .map(|(y, g)| *y - *g * step_size)
                .collect();

            Self::apply_clamping(db, &mut x_next);

            let a_next = (1.0 + (4.0 * a_k * a_k + 1.0).sqrt()) / 2.0;
            let momentum = (a_k - 1.0) / a_next;

            for i in 0..self.x_k.len() {
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

            if k >= 500 {
                step_size *= 0.9995;
            }
        }

        Self::apply_clamping(db, &mut self.x_k);
        db.positions.copy_from_slice(&self.x_k);
        log::warn!("Placer reached max iterations. Proceeding with current solution.");
        Ok(())
    }

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
