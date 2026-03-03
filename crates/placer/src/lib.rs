//! Placement Algorithms and Optimization.
//!
//! This crate implements analytical placement algorithms using physics-based
//! optimization. It includes global placement using Nesterov-accelerated
//! gradient descent, legalization algorithms (Abacus and Tetris), and
//! physics simulation for wirelength and density optimization.

pub mod legalize;
/// Physics-based placement optimization using wirelength and density forces.
///
/// Implements the physics simulation used in analytical placement, including
/// wirelength computation using weighted average (WA) approximation and density
/// force computation using electrostatic analogy with FFT acceleration. Maintains
/// bin grids for density computation, FFT scratch space for efficient potential
/// field computation, and intermediate arrays for gradient calculations. The
/// physics context is reused across iterations to avoid repeated allocations.
pub mod physics;
/// Optimization solvers for minimizing the placement objective function.
///
/// Contains the optimization algorithms that minimize the placement objective
/// function. The main solver is Nesterov-accelerated gradient descent, which
/// provides faster convergence than standard gradient descent for smooth
/// optimization problems. Also includes preconditioners for improving convergence
/// and trait definitions for interfacing with objective functions.
pub mod solver;
