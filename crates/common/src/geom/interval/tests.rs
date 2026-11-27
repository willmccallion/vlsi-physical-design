use super::*;

fn span(lo: i64, hi: i64) -> Interval {
    Interval::new(Dbu::new(lo), Dbu::new(hi))
}

fn spans(list: &[(i64, i64)]) -> Vec<Interval> {
    list.iter().map(|&(lo, hi)| span(lo, hi)).collect()
}

#[test]
fn overlapping_intervals_meet_in_their_overlap() {
    assert_eq!(span(0, 10).between(span(5, 20)), span(5, 10));
    assert_eq!(span(0, 10).distance(span(5, 20)), Dbu::ZERO);
}

#[test]
fn apart_intervals_meet_across_the_gap_between_them_either_way_round() {
    assert_eq!(span(0, 10).between(span(14, 20)), span(10, 14));
    assert_eq!(span(14, 20).between(span(0, 10)), span(10, 14));
    assert_eq!(span(14, 20).distance(span(0, 10)), Dbu::new(4));
}

#[test]
fn touching_intervals_meet_in_a_point_zero_apart() {
    assert_eq!(span(0, 10).between(span(10, 20)), span(10, 10));
    assert_eq!(span(0, 10).distance(span(10, 20)), Dbu::ZERO);
}

#[test]
fn the_overlap_is_what_both_cover_and_empty_when_apart() {
    assert_eq!(span(0, 10).overlap(span(5, 20)), span(5, 10));
    assert!(span(0, 10).overlap(span(12, 20)).is_empty());
}

#[test]
fn the_hull_covers_both_and_any_gap_between_them() {
    assert_eq!(span(0, 4).hull(span(8, 10)), span(0, 10));
    assert_eq!(span(2, 9).hull(span(3, 5)), span(2, 9));
}

#[test]
fn closed_intervals_meet_when_they_only_touch() {
    assert!(span(0, 10).meets(span(10, 20)));
    assert!(span(10, 20).meets(span(0, 10)));
    assert!(!span(0, 10).meets(span(11, 20)));
}

#[test]
fn overlapping_and_touching_intervals_merge_and_apart_ones_stay() {
    let merged = merge_touching(spans(&[(0, 2), (1, 3), (3, 4), (6, 7)]));

    assert_eq!(merged, spans(&[(0, 4), (6, 7)]));
}

#[test]
fn a_contained_interval_merges_into_the_one_around_it() {
    assert_eq!(merge_touching(spans(&[(0, 10), (2, 3)])), spans(&[(0, 10)]));
}

#[test]
fn no_intervals_merge_into_none() {
    assert_eq!(merge_touching([]), []);
}

#[test]
fn sorted_lists_intersect_where_both_cover_more_than_a_point() {
    let a = spans(&[(0, 10), (20, 30)]);
    let b = spans(&[(5, 20), (25, 40)]);

    assert_eq!(intersect_sorted(&a, &b), spans(&[(5, 10), (25, 30)]));
}

#[test]
fn the_gaps_are_what_the_blocked_intervals_leave_of_the_whole() {
    let blocked = spans(&[(-5, 2), (4, 6), (5, 8), (12, 20)]);

    assert_eq!(gaps(blocked, span(0, 15)), spans(&[(2, 4), (8, 12)]));
}

#[test]
fn with_nothing_blocked_the_whole_is_one_gap() {
    assert_eq!(gaps([], span(3, 9)), spans(&[(3, 9)]));
}

#[test]
fn blocked_intervals_past_the_end_leave_no_gap_there() {
    assert_eq!(gaps(spans(&[(5, 30)]), span(0, 10)), spans(&[(0, 5)]));
}

#[test]
fn between_spans_the_overlap_or_the_gap() {
    assert_eq!(span(0, 5).between(span(3, 9)), span(3, 5));
    assert_eq!(span(0, 3).between(span(5, 9)), span(3, 5));
    assert_eq!(span(5, 9).between(span(0, 3)), span(3, 5));
    assert_eq!(span(0, 5).between(span(5, 9)), span(5, 5));
}

#[test]
fn intervals_that_only_touch_have_no_overlap_of_length() {
    assert_eq!(intersect_sorted(&[span(0, 5)], &[span(5, 9)]), []);
    assert_eq!(intersect_sorted(&[span(0, 5)], &[span(4, 9)]), [span(4, 5)]);
}

#[test]
fn an_interval_without_length_is_empty() {
    assert_eq!(
        [span(1, 3), span(3, 3), span(4, 3)].map(Interval::is_empty),
        [false, true, true]
    );
}

#[test]
fn sorted_lists_intersect_whichever_runs_out_first() {
    let long = spans(&[(0, 10)]);
    let short = spans(&[(1, 2), (4, 6)]);

    assert_eq!(intersect_sorted(&long, &short), spans(&[(1, 2), (4, 6)]));
    assert_eq!(intersect_sorted(&short, &long), spans(&[(1, 2), (4, 6)]));
}

#[test]
fn gaps_leave_out_the_empty_stretches_at_a_blockages_ends() {
    let within = span(0, 10);

    assert_eq!(gaps(spans(&[(0, 5)]), within), spans(&[(5, 10)]));
    assert_eq!(gaps(spans(&[(2, 10)]), within), spans(&[(0, 2)]));
}
