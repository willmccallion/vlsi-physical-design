//! Bin densities, their electric field, and the force it puts on each
//! object.
//!
//! Objects are charges equal to their area. One narrower or shorter than
//! √2 bins is spread over that width at a proportionally lower density, so
//! that its force varies smoothly as it crosses bin edges. Space outside the
//! rows and under fixed cells is charge that never moves, scaled by the
//! target density so a fully blocked bin is exactly full.

use super::poisson::PoissonSolver;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use rayon::prelude::*;
use std::f64::consts::SQRT_2;

/// How an object spreads its charge: half its smoothed size, and the
/// density at which it is spread.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Footprint {
    half: Point<f64>,
    density: f64,
}

/// The geometry of a square grid of bins.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Bins {
    dim: usize,
    origin: Point<f64>,
    size: Point<f64>,
}

impl Bins {
    const fn area(&self) -> f64 {
        self.size.x * self.size.y
    }

    /// The columns and rows `rect` covers, each with the length of overlap.
    #[allow(clippy::type_complexity)]
    fn under(&self, rect: &Rect<f64>) -> Option<(Vec<(usize, f64)>, Vec<(usize, f64)>)> {
        let cols = overlaps(rect.min.x, rect.max.x, self.origin.x, self.size.x, self.dim);
        let rows = overlaps(rect.min.y, rect.max.y, self.origin.y, self.size.y, self.dim);
        (!cols.is_empty() && !rows.is_empty()).then_some((cols, rows))
    }

    /// Adds `density` times the overlap of `rect` with each bin to `map`.
    fn spread(&self, rect: &Rect<f64>, density: f64, map: &mut [f64]) {
        let Some((cols, rows)) = self.under(rect) else {
            return;
        };
        for (r, oy) in rows {
            for &(c, ox) in &cols {
                let idx = r * self.dim + c;
                map[idx] = (ox * oy).mul_add(density, map[idx]);
            }
        }
    }
}

/// The bin grid over the die and its electrostatic state.
#[derive(Debug)]
pub(super) struct DensityGrid {
    bins: Bins,
    target: f64,
    fixed: Vec<f64>,
    cells: Vec<f64>,
    total: Vec<f64>,
    potential: Vec<f64>,
    field_x: Vec<f64>,
    field_y: Vec<f64>,
    solver: PoissonSolver,
}

impl DensityGrid {
    /// A `dim`×`dim` grid over `die`. Area outside `placeable` and under
    /// `fixed` is blocked.
    pub(super) fn new(
        die: &Rect<f64>,
        dim: usize,
        target: f64,
        placeable: &[Rect<f64>],
        fixed: &[Rect<f64>],
    ) -> Self {
        let bin = Point::new(die.width() / dim as f64, die.height() / dim as f64);
        let mut grid = Self {
            bins: Bins {
                dim,
                origin: die.min,
                size: bin,
            },
            target,
            fixed: vec![0.0; dim * dim],
            cells: vec![0.0; dim * dim],
            total: vec![0.0; dim * dim],
            potential: vec![0.0; dim * dim],
            field_x: vec![0.0; dim * dim],
            field_y: vec![0.0; dim * dim],
            solver: PoissonSolver::new(dim, bin.x, bin.y),
        };
        grid.fixed = grid.blocked_area(placeable, fixed);
        for a in &mut grid.fixed {
            *a *= target;
        }
        grid
    }

    /// Area of each bin outside every placeable rectangle or under a fixed one.
    fn blocked_area(&self, placeable: &[Rect<f64>], fixed: &[Rect<f64>]) -> Vec<f64> {
        let bin_area = self.bins.area();
        let mut open = vec![0.0; self.bins.dim * self.bins.dim];
        if placeable.is_empty() {
            open.fill(bin_area);
        }
        for r in placeable {
            self.bins.spread(r, 1.0, &mut open);
        }
        let mut blocked: Vec<f64> = open.iter().map(|&o| (bin_area - o).max(0.0)).collect();
        for r in fixed {
            self.bins.spread(r, 1.0, &mut blocked);
        }
        for b in &mut blocked {
            *b = b.min(bin_area);
        }
        blocked
    }

    /// Area cells may use: the die less blocked area.
    pub(super) fn free_area(&self) -> f64 {
        let bin_area = self.bins.area();
        self.fixed.iter().map(|f| bin_area - f / self.target).sum()
    }

    /// Width and height of one bin.
    pub(super) const fn bin(&self) -> Point<f64> {
        self.bins.size
    }

    /// The footprint of an object of `size`.
    pub(super) fn footprint(&self, size: Point<f64>) -> Footprint {
        let w = size.x.max(SQRT_2 * self.bins.size.x);
        let h = size.y.max(SQRT_2 * self.bins.size.y);
        Footprint {
            half: Point::new(w / 2.0, h / 2.0),
            density: (size.x * size.y) / (w * h),
        }
    }

    /// Deposits every object, the first `cells` of them as cells and the rest
    /// as fillers, and solves for the field. Returns the overflow: the cell
    /// area above the target density as a fraction of all cell area.
    pub(super) fn update(
        &mut self,
        centers: &[Point<f64>],
        footprints: &[Footprint],
        cells: usize,
    ) -> f64 {
        self.cells.fill(0.0);
        self.total.copy_from_slice(&self.fixed);
        for (i, (&c, f)) in centers.iter().zip(footprints).enumerate() {
            let rect = Rect::new(c - f.half, c + f.half);
            if i < cells {
                self.bins.spread(&rect, f.density, &mut self.cells);
            }
            self.bins.spread(&rect, f.density, &mut self.total);
        }

        let bin_area = self.bins.area();
        let density: Vec<f64> = self.total.iter().map(|a| a / bin_area).collect();
        self.solver.solve(&density, &mut self.potential);
        self.compute_field();

        let cell_area: f64 = self.cells.iter().sum();
        if cell_area <= 0.0 {
            return 0.0;
        }
        let capacity = self.target * bin_area;
        let over: f64 = self
            .cells
            .iter()
            .zip(&self.fixed)
            .map(|(c, f)| (c + f - capacity).max(0.0))
            .sum();
        over / cell_area
    }

    /// The field `-∇φ` at each bin, by central differences with mirrored edges.
    fn compute_field(&mut self) {
        let dim = self.bins.dim;
        let bin = self.bins.size;
        let phi = &self.potential;
        for y in 0..dim {
            for x in 0..dim {
                let idx = y * dim + x;
                let left = phi[if x > 0 { idx - 1 } else { idx }];
                let right = phi[if x + 1 < dim { idx + 1 } else { idx }];
                let down = phi[if y > 0 { idx - dim } else { idx }];
                let up = phi[if y + 1 < dim { idx + dim } else { idx }];
                self.field_x[idx] = -(right - left) / (2.0 * bin.x);
                self.field_y[idx] = -(up - down) / (2.0 * bin.y);
            }
        }
    }

    /// Writes the gradient of the density energy with respect to each
    /// object's center: minus its charge times the field it sits in.
    pub(super) fn gradient(
        &self,
        centers: &[Point<f64>],
        footprints: &[Footprint],
        out: &mut [Point<f64>],
    ) {
        out.par_iter_mut()
            .zip(centers.par_iter().zip(footprints))
            .for_each(|(g, (&c, f))| {
                let rect = Rect::new(c - f.half, c + f.half);
                let mut force = Point::new(0.0, 0.0);
                if let Some((cols, rows)) = self.bins.under(&rect) {
                    for (r, oy) in rows {
                        for &(col, ox) in &cols {
                            let q = ox * oy * f.density;
                            let idx = r * self.bins.dim + col;
                            force.x = q.mul_add(self.field_x[idx], force.x);
                            force.y = q.mul_add(self.field_y[idx], force.y);
                        }
                    }
                }
                *g = Point::new(-force.x, -force.y);
            });
    }
}

/// The bins along one axis that `[lo, hi]` overlaps, with overlap lengths.
fn overlaps(lo: f64, hi: f64, origin: f64, size: f64, count: usize) -> Vec<(usize, f64)> {
    let first = ((lo - origin) / size).floor().max(0.0) as usize;
    let last = (((hi - origin) / size).ceil().max(0.0) as usize).min(count);
    (first..last)
        .filter_map(|i| {
            let start = (i as f64).mul_add(size, origin);
            let len = hi.min(start + size) - lo.max(start);
            (len > 0.0).then_some((i, len))
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn die(w: f64, h: f64) -> Rect<f64> {
        Rect::new(Point::new(0.0, 0.0), Point::new(w, h))
    }

    #[test]
    fn a_small_object_spreads_its_area_over_a_wider_footprint() {
        let grid = DensityGrid::new(&die(8.0, 8.0), 8, 1.0, &[], &[]);

        let f = grid.footprint(Point::new(0.5, 0.5));

        let w = 2.0 * f.half.x;
        let h = 2.0 * f.half.y;
        assert!((w - SQRT_2).abs() < 1e-12);
        assert!((w * h).mul_add(f.density, -0.25).abs() < 1e-12);
    }

    #[test]
    fn overflow_is_cell_area_above_the_target_over_all_cell_area() {
        let mut grid = DensityGrid::new(&die(4.0, 4.0), 4, 0.5, &[], &[]);
        let f = grid.footprint(Point::new(2.0, 2.0));

        let overflow = grid.update(&[Point::new(1.0, 1.0)], &[f], 1);

        assert!((overflow - 0.5).abs() < 1e-12);
    }

    #[test]
    fn area_outside_rows_counts_as_fixed_charge() {
        let rows = [Rect::new(Point::new(0.0, 0.0), Point::new(4.0, 2.0))];
        let grid = DensityGrid::new(&die(4.0, 4.0), 2, 0.8, &rows, &[]);

        assert_eq!(grid.fixed, [0.0, 0.0, 0.8 * 4.0, 0.8 * 4.0]);
    }

    #[test]
    fn the_field_pushes_an_object_away_from_a_crowd() {
        let mut grid = DensityGrid::new(&die(16.0, 16.0), 16, 0.5, &[], &[]);
        let crowd = grid.footprint(Point::new(4.0, 4.0));
        let probe = grid.footprint(Point::new(1.0, 1.0));
        let centers = [Point::new(8.0, 8.0), Point::new(11.0, 8.0)];
        let _ = grid.update(&centers, &[crowd, probe], 2);
        let mut grads = [Point::new(0.0, 0.0); 2];

        grid.gradient(&centers, &[crowd, probe], &mut grads);

        assert!(grads[1].x < 0.0, "{:?}", grads[1]);
    }
}
