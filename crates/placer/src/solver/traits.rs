//! Trait Definitions for Optimization.
//!
//! Defines traits that optimization algorithms can use to interface with
//! objective functions. Provides a standard interface for function evaluation
//! and gradient computation.

/// Trait for differentiable objective functions used in optimization.
///
/// Allows optimization algorithms to evaluate the objective function value
/// and compute gradients without knowing the specific implementation details.
/// This abstraction enables different objective functions (wirelength, density,
/// timing, etc.) to be used with the same optimizer.
pub trait DifferentiableFunction {
    /// Evaluates the objective function at the given point.
    ///
    /// Returns the function value without modifying any state. Used for
    /// monitoring convergence and debugging.
    fn evaluate(&mut self, x: &[f64]) -> f64;
    /// Computes the gradient of the objective function at the given point.
    ///
    /// Evaluates both the function value and its gradient, storing the gradient
    /// in the provided array. Returns the function value for convenience.
    fn gradient(&mut self, x: &[f64], grad: &mut [f64]) -> f64;
}
