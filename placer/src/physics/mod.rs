//! Physics-Based Placement Optimization.
//!
//! Implements the physics simulation used in analytical placement, including
//! wirelength computation using weighted average (WA) approximation and
//! density force computation using electrostatic analogy with FFT acceleration.

pub mod electrostatics;
/// Weighted average wirelength approximation for placement optimization.
///
/// Implements the weighted average (WA) wirelength model which approximates
/// half-perimeter wirelength using smooth exponential functions. This provides
/// differentiable gradients for optimization while closely approximating the
/// true wirelength objective. The gamma parameter controls the smoothness of
/// the approximation, with smaller values providing tighter approximation but
/// potentially steeper gradients. Computes gradients with respect to each cell's
/// position for use in gradient-based optimizers.
pub mod wirelength;

use eda_common::db::core::NetlistDB;
use eda_common::geom::point::Point;
use rustfft::FftPlanner;

/// Context structure for physics-based placement computations.
///
/// Maintains the bin grid for density computation, FFT scratch space for
/// efficient potential field computation, and intermediate arrays for
/// gradient calculations. This structure is reused across iterations to
/// avoid repeated allocations.
pub struct PhysicsContext {
    pub bin_dim: usize,
    pub density_map: Vec<f64>,
    pub potential_map: Vec<f64>,
    pub electro_force_x: Vec<f64>,
    pub electro_force_y: Vec<f64>,

    fft_planner: FftPlanner<f64>,
    fft_scratch: Vec<rustfft::num_complex::Complex<f64>>,
}

impl PhysicsContext {
    /// Creates a new physics context with the specified bin grid dimensions.
    ///
    /// Allocates arrays for density maps, potential fields, and force vectors
    /// sized to match the bin grid. Initializes the FFT planner for later use
    /// in computing electrostatic potentials.
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

    /// Computes gradients for wirelength and density objectives.
    ///
    /// Evaluates the wirelength cost using weighted average approximation
    /// and the density cost using electrostatic force computation. Accumulates
    /// gradients into the output_gradients array, which guides the optimizer's
    /// search direction. Returns both cost values for monitoring convergence.
    pub fn compute_gradients(
        &mut self,
        db: &NetlistDB,
        current_positions: &[Point<f64>],
        output_gradients: &mut [Point<f64>],
        wa_gamma: f64,
        target_density: f64,
        force_multiplier: f64,
    ) -> (f64, f64) {
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

    /// Computes WL and density gradients separately with their norms.
    ///
    /// Returns (wl_cost, density_cost, wl_grad_norm, density_grad_norm).
    /// The WL gradients are written to output_gradients, and the density
    /// gradients are written to density_gradients. The caller combines them
    /// with an adaptive weight.
    pub fn compute_gradients_separate(
        &mut self,
        db: &NetlistDB,
        current_positions: &[Point<f64>],
        output_gradients: &mut [Point<f64>],
        density_gradients: &mut [Point<f64>],
        wa_gamma: f64,
        target_density: f64,
    ) -> (f64, f64, f64, f64) {
        for g in output_gradients.iter_mut() {
            *g = Point::new(0.0, 0.0);
        }
        for g in density_gradients.iter_mut() {
            *g = Point::new(0.0, 0.0);
        }

        let wl_cost =
            wirelength::compute_wa_gradient(db, current_positions, wa_gamma, output_gradients);

        // Compute WL gradient norm
        let wl_grad_norm = output_gradients
            .iter()
            .map(|g| g.x * g.x + g.y * g.y)
            .sum::<f64>()
            .sqrt();

        // Compute density force with multiplier=1.0 into separate buffer
        let density_cost = electrostatics::compute_density_force(
            self,
            db,
            current_positions,
            target_density,
            1.0,
            density_gradients,
        );

        // Compute density gradient norm
        let density_grad_norm = density_gradients
            .iter()
            .map(|g| g.x * g.x + g.y * g.y)
            .sum::<f64>()
            .sqrt();

        (wl_cost, density_cost, wl_grad_norm, density_grad_norm)
    }
}
