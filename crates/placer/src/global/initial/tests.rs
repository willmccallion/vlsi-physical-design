use super::*;

fn object(index: usize) -> (Terminal, Point<f64>) {
    (
        Terminal::Object {
            index: ObjectId::new(index),
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
    let mut centers = vec![Point::new(20.0, 20.0)].into();

    place(
        &netlist,
        &mut centers,
        whole_plane(),
        &vec![whole_plane()].into(),
        20,
        0.75,
    );

    let c = centers[ObjectId(0)];
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
    let mut centers = vec![Point::new(1.0, 1.0)].into();

    place(
        &netlist,
        &mut centers,
        whole_plane(),
        &vec![bound].into(),
        1,
        0.75,
    );

    assert_eq!(centers[ObjectId(0)], Point::new(4.0, 6.0));
}

#[test]
fn only_pairs_with_an_outermost_pin_are_tied() {
    let mut netlist = Netlist::default();
    netlist.push([object(0), object(1), object(2), object(3)], 1.0);
    let centers = vec![
        Point::new(0.0, 0.0),
        Point::new(3.0, 0.0),
        Point::new(6.0, 0.0),
        Point::new(10.0, 0.0),
    ]
    .into();

    let (matrix, _) = bound_to_bound(&netlist, &centers, 0.75, &|p: Point<f64>| p.x);

    assert_eq!(matrix.columns, vec![1, 2, 3, 0, 3, 0, 3, 0, 1, 2]);
}

#[test]
fn nets_at_the_fanout_limit_are_left_out() {
    let mut netlist = Netlist::default();
    netlist.push((0..MAX_FANOUT).map(object), 1.0);
    let centers: ObjectPoints = (0..MAX_FANOUT).map(|i| Point::new(i as f64, 0.0)).collect();

    let (matrix, _) = bound_to_bound(&netlist, &centers, 0.75, &|p: Point<f64>| p.x);

    assert!(matrix.values.is_empty() && matrix.diagonal.iter().all(|&d| d == 0.0));
}
