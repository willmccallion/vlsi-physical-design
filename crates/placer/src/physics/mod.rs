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

use pare_common::db::core::NetlistDB;
use pare_common::geom::point::Point;
use rustfft::FftPlanner;

/// Context structure for physics-based placement computations.
///
/// Maintains the bin grid for density computation, FFT scratch space for
/// efficient potential field computation, and intermediate arrays for
/// gradient calculations. This structure is reused across iterations to
/// avoid repeated allocations.
pub struct PhysicsContext {
    /// Number of bins along each axis of the density grid.
    pub bin_dim: usize,
    /// Per-bin density values (row-major, size = `bin_dim` * `bin_dim`).
    pub density_map: Vec<f64>,
    /// Electrostatic potential at each bin center.
    pub potential_map: Vec<f64>,
    /// X-component of the electrostatic force field per bin.
    pub electro_force_x: Vec<f64>,
    /// Y-component of the electrostatic force field per bin.
    pub electro_force_y: Vec<f64>,

    fft_planner: FftPlanner<f64>,
    /// Scratch buffer sized for 2N×2N mirrored FFT (DCT via mirror padding).
    pub(crate) fft_scratch: Vec<rustfft::num_complex::Complex<f64>>,
    /// Mirrored density map for 2N×2N DCT computation.
    pub(crate) mirror_map: Vec<f64>,

    /// Routing congestion map from global router (row-major, values 0.0-2.0+).
    congestion_map: Option<Vec<f32>>,
    /// Dimensions of the congestion grid.
    congestion_grid_w: u32,
    congestion_grid_h: u32,
}

impl std::fmt::Debug for PhysicsContext {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PhysicsContext")
            .field("bin_dim", &self.bin_dim)
            .field("density_map", &self.density_map.len())
            .field("potential_map", &self.potential_map.len())
            .field("electro_force_x", &self.electro_force_x.len())
            .field("electro_force_y", &self.electro_force_y.len())
            .finish_non_exhaustive()
    }
}

impl PhysicsContext {
    /// Creates a new physics context with the specified bin grid dimensions.
    ///
    /// Allocates arrays for density maps, potential fields, and force vectors
    /// sized to match the bin grid. The FFT scratch space is sized for 2N×2N
    /// to support mirror-padded DCT boundary conditions.
    pub fn new(width: usize, height: usize) -> Self {
        let size = width * height;
        let mirror_size = (2 * width) * (2 * height);
        Self {
            bin_dim: width,
            density_map: vec![0.0; size],
            potential_map: vec![0.0; size],
            electro_force_x: vec![0.0; size],
            electro_force_y: vec![0.0; size],
            fft_planner: FftPlanner::new(),
            fft_scratch: vec![rustfft::num_complex::Complex::default(); mirror_size],
            mirror_map: vec![0.0; mirror_size],
            congestion_map: None,
            congestion_grid_w: 0,
            congestion_grid_h: 0,
        }
    }

    /// Sets the routing congestion map for congestion-driven placement.
    ///
    /// The map is a flat row-major array of per-GCell congestion ratios
    /// (usage/capacity). Values > 1.0 indicate overflow.
    pub fn set_congestion_map(&mut self, map: Vec<f32>, grid_w: u32, grid_h: u32) {
        self.congestion_map = Some(map);
        self.congestion_grid_w = grid_w;
        self.congestion_grid_h = grid_h;
    }

    /// Computes congestion-aware gradients that push cells away from congested regions.
    ///
    /// For each movable cell, maps its center to the congestion grid and computes
    /// a repulsive gradient via finite differences on the congestion map. Cells in
    /// low-congestion regions (< threshold) are not affected. Returns the total
    /// congestion cost (sum of congestion values at cell locations).
    pub fn compute_congestion_gradient(
        &self,
        db: &NetlistDB,
        positions: &[Point<f64>],
        gradients: &mut [Point<f64>],
        weight: f64,
    ) -> f64 {
        let Some(cmap) = &self.congestion_map else {
            return 0.0;
        };
        let gw = self.congestion_grid_w as usize;
        let gh = self.congestion_grid_h as usize;
        if gw == 0 || gh == 0 {
            return 0.0;
        }

        let die_w = db.die_area.width();
        let die_h = db.die_area.height();
        let bin_w = die_w / gw as f64;
        let bin_h = die_h / gh as f64;

        let congestion_threshold = 0.5;
        let mut total_cost = 0.0;

        for (i, pos) in positions.iter().enumerate() {
            if db.cells[i].is_fixed {
                continue;
            }

            let cx = db.cells[i].width.mul_add(0.5, pos.x);
            let cy = db.cells[i].height.mul_add(0.5, pos.y);

            let gx = ((cx - db.die_area.min.x) / bin_w) as usize;
            let gy = ((cy - db.die_area.min.y) / bin_h) as usize;
            let gx = gx.min(gw - 1);
            let gy = gy.min(gh - 1);

            let c = cmap[gy * gw + gx] as f64;
            if c < congestion_threshold {
                continue;
            }
            total_cost += c;

            // Finite difference gradient on the congestion map
            let cx_left = if gx > 0 { cmap[gy * gw + (gx - 1)] as f64 } else { c };
            let cx_right = if gx < gw - 1 { cmap[gy * gw + (gx + 1)] as f64 } else { c };
            let cy_down = if gy > 0 { cmap[(gy - 1) * gw + gx] as f64 } else { c };
            let cy_up = if gy < gh - 1 { cmap[(gy + 1) * gw + gx] as f64 } else { c };

            let grad_x = (cx_right - cx_left) / (2.0 * bin_w);
            let grad_y = (cy_up - cy_down) / (2.0 * bin_h);

            gradients[i].x += grad_x * weight;
            gradients[i].y += grad_y * weight;
        }

        total_cost
    }

    /// Computes gradients for wirelength and density objectives.
    ///
    /// Evaluates the wirelength cost using weighted average approximation
    /// and the density cost using electrostatic force computation. Accumulates
    /// gradients into the `output_gradients` array, which guides the optimizer's
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
    /// Returns (`wl_cost`, `density_cost`, `wl_grad_norm`, `density_grad_norm`).
    /// The WL gradients are written to `output_gradients`, and the density
    /// gradients are written to `density_gradients`. The caller combines them
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
            .map(|g| g.x.mul_add(g.x, g.y * g.y))
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
            .map(|g| g.x.mul_add(g.x, g.y * g.y))
            .sum::<f64>()
            .sqrt();

        (wl_cost, density_cost, wl_grad_norm, density_grad_norm)
    }
}
