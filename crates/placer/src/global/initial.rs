//! Initial placement as `OpenROAD`'s `gpl` makes it before Nesterov's method:
//! rounds of a bound-to-bound quadratic wirelength model, each solved for
//! both axes by `BiCGSTAB` from the positions of the round before.

use super::wirelength::{Netlist, Terminal, position};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;

/// Rounds `gpl` runs unless told otherwise, its `initialPlaceMaxIter`.
pub const INITIAL_PLACE_ROUNDS: usize = 20;

/// Nets of this many pins or more are left out, as `initialPlaceMaxFanout`.
const MAX_FANOUT: usize = 200;

/// A net's weight before it is shared among its pins, as
/// `initialPlaceNetWeightScale`.
const NET_WEIGHT_SCALE: f64 = 800.0;

/// The pin distance below which a connection's weight stops growing, as
/// `initialPlaceMinDiffLength`.
pub(super) const MIN_DIFF_LENGTH: Dbu = Dbu(1500);

/// Solver iterations per axis and round, as `initialPlaceMaxSolverIter`.
const MAX_SOLVER_ITERATIONS: usize = 100;

/// Eigen's default solver tolerance for `gpl`'s single-precision system.
const SOLVER_TOLERANCE: f64 = f32::EPSILON as f64;

/// The relative residual below which rounds stop, once `MIN_ROUNDS` ran.
const CONVERGED_RESIDUAL: f64 = 1e-5;
const MIN_ROUNDS: usize = 5;

/// Vector entries each parallel task sums, fixed so that sums do not depend
/// on how the work is split.
const CHUNK: usize = 4096;

/// Moves `centers` as `gpl`'s `doBicgstabPlace` moves them, for at most
/// `rounds` rounds: after each, every center is held to `core` and then to
/// its own box in `bounds`. Pins closer than `min_diff` are weighted as if
/// `min_diff` apart.
pub(super) fn place(
    netlist: &Netlist,
    centers: &mut [Point<f64>],
    core: Rect<f64>,
    bounds: &[Rect<f64>],
    rounds: usize,
    min_diff: f64,
) {
    for round in 1..=rounds {
        let ((xs, x_error), (ys, y_error)) = rayon::join(
            || solve_axis(netlist, centers, min_diff, |p| p.x),
            || solve_axis(netlist, centers, min_diff, |p| p.y),
        );
        if x_error.is_nan() || y_error.is_nan() {
            log::warn!("initial placement solver failed at round {round}");
            break;
        }
        for (k, center) in centers.iter_mut().enumerate() {
            let held = clamp(Point::new(xs[k], ys[k]), core);
            *center = clamp(held, bounds[k]);
        }
        let residual = x_error.max(y_error);
        log::debug!("initial placement round {round}: residual {residual:.8}");
        if residual <= CONVERGED_RESIDUAL && round >= MIN_ROUNDS {
            break;
        }
    }
}

const fn clamp(p: Point<f64>, area: Rect<f64>) -> Point<f64> {
    Point::new(
        p.x.max(area.min.x).min(area.max.x),
        p.y.max(area.min.y).min(area.max.y),
    )
}

/// One axis's new coordinates and the solver's relative residual.
fn solve_axis(
    netlist: &Netlist,
    centers: &[Point<f64>],
    min_diff: f64,
    axis: impl Fn(Point<f64>) -> f64,
) -> (Vec<f64>, f64) {
    let (matrix, rhs) = bound_to_bound(netlist, centers, min_diff, &axis);
    let mut coords: Vec<f64> = centers.iter().map(|&c| axis(c)).collect();
    let error = bicgstab(&matrix, &rhs, &mut coords);
    (coords, error)
}

/// A square sparse matrix: its diagonal, and its other entries row by row.
#[derive(Debug)]
struct Matrix {
    diagonal: Vec<f64>,
    row_starts: Vec<usize>,
    columns: Vec<usize>,
    values: Vec<f64>,
}

impl Matrix {
    /// The matrix of `diagonal` and the `(row, column, value)` entries off
    /// it, entries at the same place summed.
    fn new(diagonal: Vec<f64>, mut off_diagonal: Vec<(usize, usize, f64)>) -> Self {
        off_diagonal.sort_unstable_by_key(|&(row, column, _)| (row, column));
        let mut row_starts = vec![0; diagonal.len() + 1];
        let mut columns = Vec::with_capacity(off_diagonal.len());
        let mut values: Vec<f64> = Vec::with_capacity(off_diagonal.len());
        let mut last = None;
        for (row, column, value) in off_diagonal {
            if last == Some((row, column)) {
                if let Some(v) = values.last_mut() {
                    *v += value;
                }
                continue;
            }
            last = Some((row, column));
            row_starts[row + 1] += 1;
            columns.push(column);
            values.push(value);
        }
        for r in 0..diagonal.len() {
            row_starts[r + 1] += row_starts[r];
        }
        Self {
            diagonal,
            row_starts,
            columns,
            values,
        }
    }

    fn multiply(&self, x: &[f64], out: &mut [f64]) {
        out.par_iter_mut().enumerate().for_each(|(r, o)| {
            let entries = self.row_starts[r]..self.row_starts[r + 1];
            *o = self.columns[entries.clone()]
                .iter()
                .zip(&self.values[entries])
                .fold(self.diagonal[r] * x[r], |sum, (&c, &v)| {
                    v.mul_add(x[c], sum)
                });
        });
    }
}

/// `gpl`'s `createSparseMatrix` on one axis: each net of two pins up to
/// `MAX_FANOUT` ties its two outermost pins on the axis to each of its other
/// pins with weight `NET_WEIGHT_SCALE / (pins - 1)` over their distance.
fn bound_to_bound(
    netlist: &Netlist,
    centers: &[Point<f64>],
    min_diff: f64,
    axis: &impl Fn(Point<f64>) -> f64,
) -> (Matrix, Vec<f64>) {
    let mut diagonal = vec![0.0; centers.len()];
    let mut off_diagonal = Vec::new();
    let mut rhs = vec![0.0; centers.len()];
    let mut coords = Vec::new();
    for pins in netlist.nets() {
        if pins.len() >= MAX_FANOUT {
            continue;
        }
        coords.clear();
        coords.extend(pins.iter().map(|&t| axis(position(t, centers))));
        let (lowest, highest) = outermost(&coords);
        let net_weight = NET_WEIGHT_SCALE / (pins.len() - 1) as f64;
        for i in 1..pins.len() {
            for j in 0..i {
                let bounds_net = [i, j].iter().any(|&k| k == lowest || k == highest);
                if !bounds_net || same_object(pins[i], pins[j]) {
                    continue;
                }
                let weight = net_weight / (coords[i] - coords[j]).abs().max(min_diff);
                match (pins[i], pins[j]) {
                    (
                        Terminal::Object {
                            index: a,
                            offset: oa,
                        },
                        Terminal::Object {
                            index: b,
                            offset: ob,
                        },
                    ) => {
                        let gap = axis(oa) - axis(ob);
                        diagonal[a] += weight;
                        diagonal[b] += weight;
                        off_diagonal.push((a, b, -weight));
                        off_diagonal.push((b, a, -weight));
                        rhs[a] = weight.mul_add(-gap, rhs[a]);
                        rhs[b] = weight.mul_add(gap, rhs[b]);
                    }
                    (Terminal::Fixed(p), Terminal::Object { index, offset })
                    | (Terminal::Object { index, offset }, Terminal::Fixed(p)) => {
                        diagonal[index] += weight;
                        rhs[index] = weight.mul_add(axis(p) - axis(offset), rhs[index]);
                    }
                    (Terminal::Fixed(_), Terminal::Fixed(_)) => {}
                }
            }
        }
    }
    (Matrix::new(diagonal, off_diagonal), rhs)
}

/// The first lowest and first highest of `coords`, as `gpl`'s
/// `updatePinInfo` marks them.
fn outermost(coords: &[f64]) -> (usize, usize) {
    let (mut lowest, mut highest) = (0, 0);
    for (k, &c) in coords.iter().enumerate().skip(1) {
        if c < coords[lowest] {
            lowest = k;
        }
        if c > coords[highest] {
            highest = k;
        }
    }
    (lowest, highest)
}

const fn same_object(a: Terminal, b: Terminal) -> bool {
    matches!(
        (a, b),
        (Terminal::Object { index: i, .. }, Terminal::Object { index: j, .. }) if i == j
    )
}

/// Solves `matrix * x = rhs` from the guess in `x` as Eigen's `bicgstab`
/// with the identity preconditioner does, returning the relative residual.
fn bicgstab(matrix: &Matrix, rhs: &[f64], x: &mut [f64]) -> f64 {
    let len = x.len();
    let mut res = vec![0.0; len];
    residual(matrix, rhs, x, &mut res);
    let mut shadow = res.clone();
    let mut shadow_sqnorm = dot(&shadow, &shadow);
    let rhs_sqnorm = dot(rhs, rhs);
    if rhs_sqnorm == 0.0 {
        x.fill(0.0);
        return 0.0;
    }
    let (mut rho, mut alpha, mut omega) = (1.0, 1.0, 1.0);
    let mut direction = vec![0.0; len];
    let mut direction_image = vec![0.0; len];
    let mut half = vec![0.0; len];
    let mut half_image = vec![0.0; len];
    let tolerance_sq = SOLVER_TOLERANCE * SOLVER_TOLERANCE * rhs_sqnorm;
    let restart_ratio = SOLVER_TOLERANCE * SOLVER_TOLERANCE;
    let mut iterations = 0;
    let mut restarts = 0;
    while dot(&res, &res) > tolerance_sq && iterations < MAX_SOLVER_ITERATIONS {
        let rho_old = rho;
        rho = dot(&shadow, &res);
        if rho.abs() < restart_ratio * shadow_sqnorm {
            residual(matrix, rhs, x, &mut res);
            shadow.copy_from_slice(&res);
            shadow_sqnorm = dot(&res, &res);
            rho = shadow_sqnorm;
            if restarts == 0 {
                iterations = 0;
            }
            restarts += 1;
        }
        let beta = (rho / rho_old) * (alpha / omega);
        direction
            .par_iter_mut()
            .zip(&res)
            .zip(&direction_image)
            .for_each(|((d, &r), &v)| *d = beta.mul_add(omega.mul_add(-v, *d), r));
        matrix.multiply(&direction, &mut direction_image);
        alpha = rho / dot(&shadow, &direction_image);
        half.par_iter_mut()
            .zip(&res)
            .zip(&direction_image)
            .for_each(|((h, &r), &v)| *h = alpha.mul_add(-v, r));
        matrix.multiply(&half, &mut half_image);
        let image_sqnorm = dot(&half_image, &half_image);
        omega = if image_sqnorm > 0.0 {
            dot(&half_image, &half) / image_sqnorm
        } else {
            0.0
        };
        x.par_iter_mut()
            .zip(&direction)
            .zip(&half)
            .for_each(|((x, &d), &h)| *x += alpha.mul_add(d, omega * h));
        res.par_iter_mut()
            .zip(&half)
            .zip(&half_image)
            .for_each(|((r, &h), &t)| *r = omega.mul_add(-t, h));
        iterations += 1;
    }
    (dot(&res, &res) / rhs_sqnorm).sqrt()
}

/// Writes `rhs - matrix * x` into `out`.
fn residual(matrix: &Matrix, rhs: &[f64], x: &[f64], out: &mut [f64]) {
    matrix.multiply(x, out);
    out.par_iter_mut().zip(rhs).for_each(|(o, &b)| *o = b - *o);
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    let partial: Vec<f64> = a
        .par_chunks(CHUNK)
        .zip(b.par_chunks(CHUNK))
        .map(|(a, b)| a.iter().zip(b).fold(0.0, |sum, (&p, &q)| p.mul_add(q, sum)))
        .collect();
    partial.iter().sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn object(index: usize) -> (Terminal, Point<f64>) {
        (
            Terminal::Object {
                index,
                offset: Point::new(0.0, 0.0),
            },
            Point::new(0.0, 0.0),
        )
    }

    fn fixed(x: f64, y: f64) -> (Terminal, Point<f64>) {
        (Terminal::Fixed(Point::new(x, y)), Point::new(0.0, 0.0))
    }

    fn whole_plane() -> Rect<f64> {
        Rect::new(Point::new(-1e9, -1e9), Point::new(1e9, 1e9))
    }

    #[test]
    fn bicgstab_solves_a_small_symmetric_system() {
        let matrix = Matrix::new(vec![4.0, 3.0], vec![(0, 1, 1.0), (1, 0, 1.0)]);
        let mut x = vec![0.0, 0.0];

        let error = bicgstab(&matrix, &[1.0, 2.0], &mut x);

        assert!(error < 1e-7);
        assert!((x[0] - 1.0 / 11.0).abs() < 1e-9 && (x[1] - 7.0 / 11.0).abs() < 1e-9);
    }

    #[test]
    fn duplicate_entries_are_summed() {
        let matrix = Matrix::new(vec![1.0, 1.0], vec![(1, 0, 2.0), (1, 0, 3.0)]);
        let mut out = vec![0.0; 2];

        matrix.multiply(&[1.0, 1.0], &mut out);

        assert_eq!(out, vec![1.0, 6.0]);
    }

    #[test]
    fn a_cell_outside_its_fixed_pins_is_pulled_between_them() {
        let mut netlist = Netlist::default();
        netlist.push([fixed(0.0, 0.0), object(0)], 1.0);
        netlist.push([object(0), fixed(10.0, 4.0)], 1.0);
        let mut centers = vec![Point::new(20.0, 20.0)];

        place(
            &netlist,
            &mut centers,
            whole_plane(),
            &[whole_plane()],
            20,
            0.75,
        );

        let c = centers[0];
        assert!(
            (0.0..=10.0).contains(&c.x) && (0.0..=4.0).contains(&c.y),
            "{c:?}"
        );
    }

    #[test]
    fn centers_are_held_to_their_bounds() {
        let mut netlist = Netlist::default();
        netlist.push([object(0), fixed(10.0, 10.0)], 1.0);
        let bound = Rect::new(Point::new(0.0, 0.0), Point::new(4.0, 6.0));
        let mut centers = vec![Point::new(1.0, 1.0)];

        place(&netlist, &mut centers, whole_plane(), &[bound], 1, 0.75);

        assert_eq!(centers[0], Point::new(4.0, 6.0));
    }

    #[test]
    fn only_pairs_with_an_outermost_pin_are_tied() {
        let mut netlist = Netlist::default();
        netlist.push([object(0), object(1), object(2), object(3)], 1.0);
        let centers = [
            Point::new(0.0, 0.0),
            Point::new(3.0, 0.0),
            Point::new(6.0, 0.0),
            Point::new(10.0, 0.0),
        ];

        let (matrix, _) = bound_to_bound(&netlist, &centers, 0.75, &|p: Point<f64>| p.x);

        assert_eq!(matrix.columns, vec![1, 2, 3, 0, 3, 0, 3, 0, 1, 2]);
    }

    #[test]
    fn nets_at_the_fanout_limit_are_left_out() {
        let mut netlist = Netlist::default();
        netlist.push((0..MAX_FANOUT).map(object), 1.0);
        let centers: Vec<Point<f64>> = (0..MAX_FANOUT).map(|i| Point::new(i as f64, 0.0)).collect();

        let (matrix, _) = bound_to_bound(&netlist, &centers, 0.75, &|p: Point<f64>| p.x);

        assert!(matrix.values.is_empty() && matrix.diagonal.iter().all(|&d| d == 0.0));
    }
}
