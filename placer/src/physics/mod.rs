pub mod electrostatics;
pub mod wirelength;

use eda_common::db::core::NetlistDB;
use eda_common::geom::point::Point;
use rustfft::FftPlanner;

pub struct PhysicsContext {
    pub bin_dim: usize,
    pub density_map: Vec<f64>,
    pub potential_map: Vec<f64>,
    pub electro_force_x: Vec<f64>,
    pub electro_force_y: Vec<f64>,

    // FFT
    fft_planner: FftPlanner<f64>,
    fft_scratch: Vec<rustfft::num_complex::Complex<f64>>,
}

impl PhysicsContext {
    pub fn new(width: usize, height: usize) -> Self {
        let size = width * height;
        Self {
            bin_dim: width,
            density_map: vec![0.0; size],
            potential_map: vec![0.0; size],
            electro_force_x: vec![0.0; size],
            electro_force_y: vec![0.0; size],
            fft_planner: FftPlanner::new(),
            fft_scratch: vec![rustfft::num_complex::Complex::default(); size],
        }
    }

    pub fn compute_gradients(
        &mut self,
        db: &NetlistDB,
        current_positions: &[Point<f64>],
        output_gradients: &mut [Point<f64>],
        wa_gamma: f64,
        target_density: f64,
        force_multiplier: f64,
    ) -> (f64, f64) {
        // Reset gradients
        for g in output_gradients.iter_mut() {
            *g = Point::new(0.0, 0.0);
        }

        let wl_cost =
            wirelength::compute_wa_gradient(db, current_positions, wa_gamma, output_gradients);

        let density_cost = electrostatics::compute_density_force(
            self,
            db,
            current_positions,
            target_density,
            force_multiplier,
            output_gradients,
        );

        (wl_cost, density_cost)
    }
}
