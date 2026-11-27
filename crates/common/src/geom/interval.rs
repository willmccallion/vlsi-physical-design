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
        let (lo, hi) = (self.lo.max(other.lo), self.hi.min(other.hi));
        Self::new(lo.min(hi), lo.max(hi))
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
    sweep_pairs(a, b, |i, j| {
        let lo = a[i].lo.max(b[j].lo);
        let hi = a[i].hi.min(b[j].hi);
        if lo < hi {
            out.push(Interval::new(lo, hi));
        }
    });
    out
}

/// Calls `visit` with the indices of each pair of intervals of `a` and `b`,
/// sorted lists of disjoint intervals, that a sweep up both brings side by
/// side, passing whichever of the two ends first.
pub(crate) fn sweep_pairs(a: &[Interval], b: &[Interval], mut visit: impl FnMut(usize, usize)) {
    let (mut i, mut j) = (0, 0);
    while let (Some(x), Some(y)) = (a.get(i), b.get(j)) {
        visit(i, j);
        if x.hi < y.hi {
            i += 1;
        } else {
            j += 1;
        }
    }
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
mod tests;
