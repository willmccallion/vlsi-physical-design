use super::*;

fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
    Rect::from_corners(
        Point::new(Dbu::new(x0), Dbu::new(y0)),
        Point::new(Dbu::new(x1), Dbu::new(y1)),
    )
}

#[test]
fn abutting_rectangles_touch_but_do_not_overlap() {
    let a = rect(0, 0, 10, 10);
    let b = rect(10, 0, 20, 10);

    assert!(a.touches(&b));
    assert!(!a.overlaps(&b));
}

#[test]
fn a_rectangle_split_across_two_abutting_covers_is_covered() {
    let cell = rect(5, 0, 15, 10);

    assert!(cell.covered_by(&[rect(0, 0, 10, 10), rect(10, 0, 20, 10)]));
}

#[test]
fn a_gap_between_covers_leaves_the_rectangle_uncovered() {
    let cell = rect(5, 0, 15, 10);

    assert!(!cell.covered_by(&[rect(0, 0, 10, 10), rect(11, 0, 20, 10)]));
}

#[test]
fn covers_stacked_vertically_cover_the_full_height() {
    let cell = rect(0, 0, 10, 10);

    assert!(cell.covered_by(&[rect(0, 0, 10, 4), rect(0, 3, 10, 10)]));
    assert!(!cell.covered_by(&[rect(0, 0, 10, 4), rect(0, 5, 10, 10)]));
}

#[test]
fn a_rectangle_sticking_out_of_its_cover_is_not_covered() {
    assert!(!rect(0, 0, 10, 10).covered_by(&[rect(0, 0, 10, 9)]));
}

#[test]
fn a_segment_split_across_two_abutting_covers_is_covered() {
    let covers = [rect(0, 0, 10, 5), rect(0, 5, 10, 10)];

    assert!(rect(5, 0, 5, 10).covered_by(&covers));
    assert!(rect(0, 5, 10, 5).covered_by(&covers));
    assert!(!rect(5, 0, 5, 11).covered_by(&covers));
}

#[test]
fn a_segment_across_a_gap_between_covers_is_not_covered() {
    let covers = [rect(0, 0, 10, 4), rect(0, 6, 10, 10)];

    assert!(!rect(5, 0, 5, 10).covered_by(&covers));
    assert!(!rect(0, 5, 10, 5).covered_by(&covers));
}

#[test]
fn a_point_is_covered_when_a_cover_holds_it_edges_included() {
    let covers = [rect(0, 0, 10, 10)];

    assert!(rect(10, 10, 10, 10).covered_by(&covers));
    assert!(!rect(11, 10, 11, 10).covered_by(&covers));
    assert!(!rect(5, 5, 5, 5).covered_by(&[]));
}

#[test]
fn corners_are_normalized() {
    assert_eq!(rect(10, 20, 0, 5), rect(0, 5, 10, 20));
}

#[test]
fn intersection_of_disjoint_rectangles_is_none() {
    assert_eq!(rect(0, 0, 1, 1).intersection(&rect(2, 2, 3, 3)), None);
}
