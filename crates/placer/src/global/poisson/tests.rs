use super::*;

fn cosine_mode(dim: usize, a: usize, b: usize) -> Vec<f64> {
    let mut density = vec![0.0; dim * dim];
    for y in 0..dim {
        for x in 0..dim {
            let cx = (PI * a as f64 * (x as f64 + 0.5) / dim as f64).cos();
            let cy = (PI * b as f64 * (y as f64 + 0.5) / dim as f64).cos();
            density[y * dim + x] = cx * cy;
        }
    }
    density
}

fn solve(density: &[f64], dim: usize, bin: (f64, f64)) -> Vec<f64> {
    let mut solver = PoissonSolver::new(dim, bin.0, bin.1);
    let mut potential = vec![0.0; dim * dim];
    solver.solve(density, &mut potential);
    potential
}

fn assert_solves_cosine_mode(dim: usize, a: usize, b: usize, bin: (f64, f64)) {
    let density = cosine_mode(dim, a, b);
    let wa = PI * a as f64 / dim as f64 / bin.0;
    let wb = PI * b as f64 / dim as f64 / bin.1;
    let eigenvalue = wa.mul_add(wa, wb * wb);

    let potential = solve(&density, dim, bin);

    for (phi, rho) in potential.iter().zip(&density) {
        assert!(
            (phi - rho / eigenvalue).abs() < 1e-9 * (1.0 + (rho / eigenvalue).abs()),
            "mode ({a}, {b}): got {phi}, expected {}",
            rho / eigenvalue
        );
    }
}

#[test]
fn potential_of_a_cosine_mode_is_the_mode_over_its_eigenvalue() {
    assert_solves_cosine_mode(16, 1, 2, (1.0, 1.0));
}

#[test]
fn potential_distinguishes_x_and_y_frequencies() {
    assert_solves_cosine_mode(16, 5, 0, (1.0, 1.0));
    assert_solves_cosine_mode(16, 0, 5, (1.0, 1.0));
}

#[test]
fn potential_of_high_frequency_modes_uses_negative_frequencies() {
    assert_solves_cosine_mode(16, 15, 11, (1.0, 1.0));
}

#[test]
fn potential_scales_each_axis_by_its_bin_size() {
    assert_solves_cosine_mode(16, 3, 2, (2.5, 0.5));
}

#[test]
fn uniform_density_produces_no_potential() {
    let potential = solve(&[0.25; 64], 8, (1.0, 1.0));

    assert!(potential.iter().all(|phi| phi.abs() < 1e-12));
}
