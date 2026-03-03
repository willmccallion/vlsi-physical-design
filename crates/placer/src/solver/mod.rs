//! Optimization Solvers for Placement.
//!
//! Contains the optimization algorithms that minimize the placement objective
//! function. The main solver is Nesterov-accelerated gradient descent, which
//! provides faster convergence than standard gradient descent for smooth
//! optimization problems.

pub mod nesterov;
/// Preconditioners for improving optimization convergence.
///
/// Provides preconditioning matrices to improve convergence of iterative
/// optimization algorithms. Currently implements Jacobi preconditioning which
/// scales gradients by inverse diagonal elements. This can accelerate convergence
/// for problems with ill-conditioned Hessian matrices by improving the conditioning
/// of the optimization problem and reducing the number of iterations needed.
pub mod preconditioner;
/// Trait definitions for optimization algorithm interfaces.
///
/// Defines traits that optimization algorithms can use to interface with objective
/// functions. Provides a standard interface for function evaluation and gradient
/// computation, enabling different objective functions (wirelength, density,
/// timing, etc.) to be used with the same optimizer without knowing specific
/// implementation details. This abstraction supports extensibility and modularity.
pub mod traits;
