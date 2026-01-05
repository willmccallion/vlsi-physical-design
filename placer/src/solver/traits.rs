pub trait DifferentiableFunction {
    fn evaluate(&mut self, x: &[f64]) -> f64;
    fn gradient(&mut self, x: &[f64], grad: &mut [f64]) -> f64;
}
