//! Stretches along one axis, and the merges, overlaps and gaps of sorted
//! lists of them.

use crate::geom::dbu::Dbu;

/// A stretch along one axis from `lo` to `hi`. Whether an end counts as
/// inside is up to each use; the operations here say how they treat ends.
/// Intervals order by `lo`, then `hi`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Interval {
    /// The low end.
    pub lo: Dbu,
    /// The high end.
    pub hi: Dbu,
}

impl Interval {
    /// The interval from `lo` to `hi`.
    #[must_use]
    pub const fn new(lo: Dbu, hi: Dbu) -> Self {
        Self { lo, hi }
    }

    /// How long it is, `hi - lo`.
    #[must_use]
    pub fn len(self) -> Dbu {
        self.hi - self.lo
    }

    /// Whether it has no length, or less.
    #[must_use]
    pub fn is_empty(self) -> bool {
        self.hi <= self.lo
    }

    /// What both cover, from the higher `lo` to the lower `hi`; empty
    /// where they do not overlap.
    #[must_use]
    pub fn overlap(self, other: Self) -> Self {
        Self::new(self.lo.max(other.lo), self.hi.min(other.hi))
    }

    /// The smallest interval covering both.
    #[must_use]
    pub fn hull(self, other: Self) -> Self {
        Self::new(self.lo.min(other.lo), self.hi.max(other.hi))
    }

    /// Whether the two, ends included, share a point.
    #[must_use]
    pub fn meets(self, other: Self) -> bool {
        self.lo <= other.hi && other.lo <= self.hi
    }

    /// The overlap of the two, or the gap between them when they are
    /// apart; touching intervals meet in a point.
    #[must_use]
    pub fn between(self, other: Self) -> Self {
        if self.hi < other.lo {
            Self::new(self.hi, other.lo)
        } else if other.hi < self.lo {
            Self::new(other.hi, self.lo)
        } else {
            Self::new(self.lo.max(other.lo), self.hi.min(other.hi))
        }
    }

    /// How far apart the two are; zero when they touch or overlap.
    #[must_use]
    pub fn distance(self, other: Self) -> Dbu {
        (other.lo - self.hi).max(self.lo - other.hi).max(Dbu::ZERO)
    }
}

/// `sorted`, ordered by `lo`, with the intervals that overlap or touch
/// merged into one.
#[must_use]
pub fn merge_touching(sorted: impl IntoIterator<Item = Interval>) -> Vec<Interval> {
    let mut merged: Vec<Interval> = Vec::new();
    for next in sorted {
        match merged.last_mut() {
            Some(top) if top.hi >= next.lo => top.hi = top.hi.max(next.hi),
            _ => merged.push(next),
        }
    }
    merged
}

/// The overlaps of positive length of two sorted lists of disjoint
/// intervals.
#[must_use]
pub fn intersect_sorted(a: &[Interval], b: &[Interval]) -> Vec<Interval> {
    let mut out = Vec::new();
    let (mut i, mut j) = (0, 0);
    while i < a.len() && j < b.len() {
        let lo = a[i].lo.max(b[j].lo);
        let hi = a[i].hi.min(b[j].hi);
        if lo < hi {
            out.push(Interval::new(lo, hi));
        }
        if a[i].hi < b[j].hi {
            i += 1;
        } else {
            j += 1;
        }
    }
    out
}

/// The stretches of `within` that none of `blocked`, ordered by `lo`,
/// covers; a gap between two blocked intervals has positive length.
#[must_use]
pub fn gaps(blocked: impl IntoIterator<Item = Interval>, within: Interval) -> Vec<Interval> {
    let mut free = Vec::new();
    let mut cursor = within.lo;
    for b in blocked {
        if b.lo > cursor {
            free.push(Interval::new(cursor, b.lo.min(within.hi)));
        }
        cursor = cursor.max(b.hi);
        if cursor >= within.hi {
            break;
        }
    }
    if cursor < within.hi {
        free.push(Interval::new(cursor, within.hi));
    }
    free
}

#[cfg(test)]
mod tests {
    use super::*;

    fn span(lo: i64, hi: i64) -> Interval {
        Interval::new(Dbu(lo), Dbu(hi))
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
        assert_eq!(span(14, 20).distance(span(0, 10)), Dbu(4));
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
}
