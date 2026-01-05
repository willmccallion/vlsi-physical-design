pub struct JacobiPreconditioner {
    pub inv_diag: Vec<f64>,
}

impl JacobiPreconditioner {
    pub fn apply(&self, grad: &mut [f64]) {
        for (g, &d) in grad.iter_mut().zip(self.inv_diag.iter()) {
            *g *= d;
        }
    }
}
