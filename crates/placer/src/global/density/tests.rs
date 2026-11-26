use super::*;

fn die(w: f64, h: f64) -> Rect<f64> {
    Rect::new(Point::new(0.0, 0.0), Point::new(w, h))
}

#[test]
fn a_small_object_spreads_its_area_over_a_wider_footprint() {
    let grid = DensityGrid::new(&die(8.0, 8.0), 8, 1.0, &[die(8.0, 8.0)], &[]);

    let f = grid.footprint(Point::new(0.5, 0.5), Charge::Whole);

    let w = 2.0 * f.half.x;
    let h = 2.0 * f.half.y;
    assert!((w - SQRT_2).abs() < 1e-12);
    assert!((w * h).mul_add(f.density, -0.25).abs() < 1e-12);
}

/// The objects from `start` up to `end`.
const fn objects(start: u32, end: u32) -> ObjectRange {
    ObjectRange::new(ObjectId(start), ObjectId(end))
}

#[test]
fn overflow_is_cell_area_above_the_target_over_all_cell_area() {
    let mut grid = DensityGrid::new(&die(4.0, 4.0), 4, 0.5, &[die(4.0, 4.0)], &[]);
    let f = grid.footprint(Point::new(2.0, 2.0), Charge::Whole);

    let excess = grid.update(
        &vec![Point::new(1.0, 1.0)].into(),
        &vec![f].into(),
        objects(0, 1),
        objects(1, 1),
    );

    assert!((excess.fraction() - 0.5).abs() < 1e-12);
}

#[test]
fn area_outside_rows_counts_as_fixed_charge() {
    let rows = [Rect::new(Point::new(0.0, 0.0), Point::new(4.0, 2.0))];
    let grid = DensityGrid::new(&die(4.0, 4.0), 2, 0.8, &rows, &[]);

    assert_eq!(grid.fixed, [0.0, 0.0, 0.8 * 4.0, 0.8 * 4.0]);
}

#[test]
fn a_fixed_cell_off_the_rows_blocks_nothing_more() {
    let rows = [Rect::new(Point::new(0.0, 0.0), Point::new(4.0, 2.0))];
    let fixed = [Rect::new(Point::new(0.0, 2.0), Point::new(4.0, 4.0))];

    let grid = DensityGrid::new(&die(4.0, 4.0), 2, 0.8, &rows, &fixed);

    assert_eq!(grid.fixed, [0.0, 0.0, 0.8 * 4.0, 0.8 * 4.0]);
}

#[test]
fn a_fixed_cell_on_a_row_blocks_its_overlap() {
    let rows = [Rect::new(Point::new(0.0, 0.0), Point::new(4.0, 2.0))];
    let fixed = [Rect::new(Point::new(0.0, 1.0), Point::new(2.0, 3.0))];

    let grid = DensityGrid::new(&die(4.0, 4.0), 2, 1.0, &rows, &fixed);

    assert_eq!(grid.fixed, [2.0, 0.0, 4.0, 4.0]);
}

/// A smooth cosine potential on 32 bins of 1 by 2 um: its field is
/// minus its derivative, within the 1% central differences lose at
/// these frequencies.
#[test]
fn the_field_of_a_smooth_potential_is_minus_its_derivative() {
    use std::f64::consts::PI;
    let (dim, bin) = (32, Point::new(1.0, 2.0));
    let mut grid = DensityGrid::new(&die(32.0, 64.0), dim, 1.0, &[die(32.0, 64.0)], &[]);
    let (a, b) = (1.0, 2.0);
    let phase = |k: f64, i: usize| PI * k * (i as f64 + 0.5) / dim as f64;
    for y in 0..dim {
        for x in 0..dim {
            grid.potential[y * dim + x] = phase(a, x).cos() * phase(b, y).cos();
        }
    }

    grid.compute_field();

    let (ka, kb) = (PI * a / dim as f64 / bin.x, PI * b / dim as f64 / bin.y);
    for y in 0..dim {
        for x in 0..dim {
            let idx = y * dim + x;
            let ex = ka * phase(a, x).sin() * phase(b, y).cos();
            let ey = kb * phase(a, x).cos() * phase(b, y).sin();
            assert!(
                (grid.field_x[idx] - ex).abs() < 0.01 * ka,
                "x at ({x}, {y})"
            );
            assert!(
                (grid.field_y[idx] - ey).abs() < 0.01 * kb,
                "y at ({x}, {y})"
            );
        }
    }
}

#[test]
fn the_field_pushes_an_object_away_from_a_crowd() {
    let mut grid = DensityGrid::new(&die(16.0, 16.0), 16, 0.5, &[die(16.0, 16.0)], &[]);
    let crowd = grid.footprint(Point::new(4.0, 4.0), Charge::Whole);
    let probe = grid.footprint(Point::new(1.0, 1.0), Charge::Whole);
    let centers = vec![Point::new(8.0, 8.0), Point::new(11.0, 8.0)].into();
    let _ = grid.update(
        &centers,
        &vec![crowd, probe].into(),
        objects(0, 2),
        objects(2, 2),
    );

    let probe_grad = grid.gradient_at(centers[ObjectId(1)], &probe);

    assert!(probe_grad.x < 0.0, "{probe_grad:?}");
}

#[test]
fn a_movable_macro_fills_its_bins_only_to_the_target_density() {
    let mut grid = DensityGrid::new(&die(4.0, 4.0), 2, 0.5, &[die(4.0, 4.0)], &[]);
    let block = grid.footprint(Point::new(4.0, 4.0), Charge::AtTarget);

    let excess = grid.update(
        &vec![Point::new(2.0, 2.0)].into(),
        &vec![block].into(),
        objects(0, 1),
        objects(1, 1),
    );

    assert!(excess.over.abs() < 1e-12, "{excess:?}");
    assert!((excess.cells - 8.0).abs() < 1e-12, "{excess:?}");
}
