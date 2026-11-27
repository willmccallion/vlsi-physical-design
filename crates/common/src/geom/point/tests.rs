use super::*;

#[test]
fn the_manhattan_distance_adds_both_sides_whichever_way_they_run() {
    let a = Point::new(Dbu::new(10), Dbu::new(-4));
    let b = Point::new(Dbu::new(-2), Dbu::new(5));

    assert_eq!(a.manhattan(b), Dbu::new(21));
    assert_eq!(b.manhattan(a), Dbu::new(21));
    assert_eq!(a.manhattan(a), Dbu::new(0));
}

#[test]
fn points_order_by_x_then_y() {
    let mut points = vec![
        Point::new(Dbu::new(2), Dbu::new(0)),
        Point::new(Dbu::new(1), Dbu::new(9)),
        Point::new(Dbu::new(1), Dbu::new(3)),
    ];

    points.sort();

    assert_eq!(
        points,
        vec![
            Point::new(Dbu::new(1), Dbu::new(3)),
            Point::new(Dbu::new(1), Dbu::new(9)),
            Point::new(Dbu::new(2), Dbu::new(0)),
        ]
    );
}

#[test]
fn dividing_a_point_divides_both_coordinates() {
    assert_eq!(Point::new(6.0, -8.0) / 2.0, Point::new(3.0, -4.0));
}
