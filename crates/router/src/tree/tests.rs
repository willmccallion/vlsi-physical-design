use super::*;

fn pt(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

fn at(x: i64, y: i64) -> Rect<Dbu> {
    Rect::new(pt(x, y), pt(x, y))
}

#[test]
fn prim_joins_each_pin_to_its_nearest_joined_pin() {
    let pins = [at(0, 0), at(100, 0), at(10, 0), at(110, 5)];

    let (first, joins) = connection_order(&pins);

    assert_eq!(first, 0);
    assert_eq!(
        joins,
        [
            Join { pin: 2, parent: 0 },
            Join { pin: 1, parent: 2 },
            Join { pin: 3, parent: 1 },
        ]
    );
}

#[test]
fn prim_measures_a_wire_from_its_nearest_point() {
    let wire = Rect::new(pt(0, 50), pt(1000, 50));
    let pins = [at(0, 0), wire, at(990, 0), at(400, 0)];

    let (_, joins) = connection_order(&pins);

    assert_eq!(
        joins,
        [
            Join { pin: 1, parent: 0 },
            Join { pin: 2, parent: 1 },
            Join { pin: 3, parent: 1 },
        ]
    );
}

#[test]
fn large_nets_join_every_pin_to_its_nearest_earlier_pin() {
    let points: Vec<Point<Dbu>> = (0..1000)
        .map(|i| pt((i * 37 % 400) * 10, (i * 91 % 250) * 10))
        .collect();

    let pins: Vec<Rect<Dbu>> = points.iter().map(|&p| Rect::new(p, p)).collect();

    let (first, joins) = connection_order(&pins);

    assert_each_joins_its_nearest_earlier_point(&points, first, &joins);
}

#[test]
fn a_far_pin_after_a_dense_cluster_joins_its_nearest_earlier_pin() {
    let cluster = (0..13).flat_map(|x| (0..12).map(move |y| pt(x, y)));
    let mut points: Vec<Point<Dbu>> = cluster.clone().chain(cluster).collect();
    points.push(pt(35, 35));
    let pins: Vec<Rect<Dbu>> = points.iter().map(|&p| Rect::new(p, p)).collect();

    let (first, joins) = connection_order(&pins);

    assert_each_joins_its_nearest_earlier_point(&points, first, &joins);
}

/// Asserts that `joins` from `first` join every other point, each to
/// the nearest point joined before it.
fn assert_each_joins_its_nearest_earlier_point(
    points: &[Point<Dbu>],
    first: usize,
    joins: &[Join],
) {
    assert_eq!(joins.len(), points.len() - 1);
    let mut earlier = vec![first];
    for &j in joins {
        let nearest = earlier
            .iter()
            .map(|&e| points[j.pin].manhattan(points[e]).get())
            .min()
            .unwrap();
        assert!(
            earlier.contains(&j.parent),
            "{j:?} joined before its parent"
        );
        assert_eq!(
            points[j.pin].manhattan(points[j.parent]).get(),
            nearest,
            "{j:?}"
        );
        earlier.push(j.pin);
    }
}

#[test]
fn the_hilbert_curve_walks_a_corner_block_one_neighbour_at_a_time() {
    let mut cells: Vec<(u64, u32, u32)> = (0..8)
        .flat_map(|x| (0..8).map(move |y| (hilbert(x, y), x, y)))
        .collect();
    cells.sort_unstable();

    let positions: Vec<u64> = cells.iter().map(|c| c.0).collect();
    assert_eq!(positions, (0..64).collect::<Vec<u64>>());
    for w in cells.windows(2) {
        assert_eq!(
            w[0].1.abs_diff(w[1].1) + w[0].2.abs_diff(w[1].2),
            1,
            "{w:?}"
        );
    }
}
