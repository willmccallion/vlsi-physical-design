use super::*;

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::new(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

/// Nangate45 `DFFS_X1` pin QN in 2000ths of a micron: a tab across a bar.
fn tab_and_bar() -> [Rect<Dbu>; 2] {
    [rect(6390, 1120, 6720, 1400), rect(6390, 370, 6530, 1850)]
}

const METAL1_SPACING: Dbu = Dbu::new(130);

fn at(x: i64, y: i64) -> Point<Dbu> {
    Point::new(Dbu::new(x), Dbu::new(y))
}

#[test]
fn fellow_boxes_leave_each_point_out() {
    let boxes = fellow_boxes(&[at(0, 0), at(10, 5), at(4, 20)]);

    assert_eq!(boxes[0], Some(rect(4, 5, 10, 20)));
    assert_eq!(boxes[1], Some(rect(0, 0, 4, 20)));
    assert_eq!(boxes[2], Some(rect(0, 0, 10, 5)));
}

#[test]
fn a_lone_point_has_no_fellows() {
    assert_eq!(fellow_boxes(&[at(3, 4)]), vec![None]);
}

#[test]
fn distance_outside_is_zero_inside_and_manhattan_beyond() {
    let area = rect(10, 10, 20, 20);

    assert_eq!(distance_outside(at(15, 20), &area), Dbu::ZERO);
    assert_eq!(distance_outside(at(5, 15), &area), Dbu::new(5));
    assert_eq!(distance_outside(at(23, 4), &area), Dbu::new(9));
}

#[test]
fn the_middle_of_a_span_is_rounded_down_to_the_manufacturing_grid() {
    assert_eq!(
        middle_on_grid(Dbu::new(100), Dbu::new(130), Dbu::new(10)),
        Dbu::new(110)
    );
}

#[test]
fn the_middle_of_a_span_below_the_origin_is_rounded_down_too() {
    assert_eq!(
        middle_on_grid(Dbu::new(-20), Dbu::new(-10), Dbu::new(10)),
        Dbu::new(-20)
    );
    assert_eq!(
        middle_on_grid(Dbu::new(-19), Dbu::new(0), Dbu::new(10)),
        Dbu::new(-10)
    );
}

#[test]
fn the_middle_of_a_span_on_the_grid_stays_put() {
    assert_eq!(
        middle_on_grid(Dbu::new(100), Dbu::new(140), Dbu::new(10)),
        Dbu::new(120)
    );
}

#[test]
fn a_pad_inside_one_part_of_a_pin_is_clear_of_its_other_parts() {
    let pad = rect(6580, 1120, 6720, 1400);

    let clear = pad_clear_of_pin_metal(&pad, &tab_and_bar(), METAL1_SPACING);

    assert!(clear);
}

#[test]
fn a_pad_ending_just_short_of_another_part_of_its_pin_is_not_clear() {
    let pad = rect(6580, 1260, 6720, 1540);

    let clear = pad_clear_of_pin_metal(&pad, &tab_and_bar(), METAL1_SPACING);

    assert!(!clear);
}

#[test]
fn a_pad_sticking_out_far_from_the_rest_of_its_pin_is_clear() {
    let pad = rect(6580, 1120, 6860, 1400);

    let clear = pad_clear_of_pin_metal(&pad, &tab_and_bar(), METAL1_SPACING);

    assert!(clear);
}
