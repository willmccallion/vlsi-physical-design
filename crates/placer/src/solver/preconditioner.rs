//! Preconditioner for Optimization.
//!
//! Provides preconditioning matrices to improve convergence of iterative
//! optimization algorithms. Currently implements Jacobi preconditioning
//! which scales gradients by inverse diagonal elements.

/// Jacobi preconditioner that scales gradients by inverse diagonal values.
///
/// Stores the inverse of diagonal matrix elements and applies element-wise
/// multiplication to precondition gradients. This can accelerate convergence
/// for problems with ill-conditioned Hessian matrices.
#[derive(Debug)]
pub struct JacobiPreconditioner {
    /// Inverse diagonal elements of the preconditioner matrix.
    pub inv_diag: Vec<f64>,
}

impl JacobiPreconditioner {
    /// Applies the preconditioner to a gradient vector in-place.
    ///
    /// Multiplies each gradient element by the corresponding inverse diagonal
    /// value. This scaling can improve the conditioning of the optimization
    /// problem and reduce the number of iterations needed for convergence.
    pub fn apply(&self, grad: &mut [f64]) {
        for (g, &d) in grad.iter_mut().zip(self.inv_diag.iter()) {
            *g *= d;
        }
    }
}
