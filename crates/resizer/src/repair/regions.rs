//! The boxes design repair splits a net's loads into to bring its fanout
//! under the limit.

use pare_common::db::indices::PinId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// A box of a net's loads, halved while it holds more of them than the
/// fanout limit, as `LoadRegion`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(super) struct LoadRegion {
    /// The loads it holds itself: every load of a box never halved, none of
    /// one that was.
    pub(super) pins: Vec<PinId>,
    /// Its two halves, lower side first, once halved.
    pub(super) halves: Option<Box<[Self; 2]>>,
}

impl LoadRegion {
    /// `loads` at their points, split as `findLoadRegions` and
    /// `subdivideRegion` split them: the box around the points is halved
    /// across its longer side while it holds more than `max_fanout` loads
    /// and is wider and taller than `min_side`, each load going to the half
    /// its point lies in, the lower one when it lies on the cut.
    pub(super) fn split(loads: &[(PinId, Point<Dbu>)], max_fanout: usize, min_side: Dbu) -> Self {
        let Some(bounds) = loads
            .iter()
            .map(|&(_, at)| Rect::new(at, at))
            .reduce(|a, b| a.union(&b))
        else {
            return Self::leaf(loads);
        };
        Self::split_box(loads, bounds, max_fanout, min_side)
    }

    fn split_box(
        loads: &[(PinId, Point<Dbu>)],
        bounds: Rect<Dbu>,
        max_fanout: usize,
        min_side: Dbu,
    ) -> Self {
        if loads.len() <= max_fanout || bounds.width() <= min_side || bounds.height() <= min_side {
            return Self::leaf(loads);
        }
        let cut = if bounds.width() > bounds.height() {
            Cut::AcrossWidth
        } else {
            Cut::AcrossHeight
        };
        let [lower, upper] = cut.halves(bounds);
        let (low, high): (Vec<_>, Vec<_>) =
            loads.iter().partition(|&&(_, at)| cut.in_lower(at, &lower));
        Self {
            pins: Vec::new(),
            halves: Some(Box::new([
                Self::split_box(&low, lower, max_fanout, min_side),
                Self::split_box(&high, upper, max_fanout, min_side),
            ])),
        }
    }

    fn leaf(loads: &[(PinId, Point<Dbu>)]) -> Self {
        Self {
            pins: loads.iter().map(|&(pin, _)| pin).collect(),
            halves: None,
        }
    }
}

/// Which way a box is cut in half.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Cut {
    /// At the middle of its width, into a left and a right half.
    AcrossWidth,
    /// At the middle of its height, into a lower and an upper half.
    AcrossHeight,
}

impl Cut {
    /// The two halves of `bounds`, the lower one first.
    // NOTE: kept: subdivideRegion takes the middle as (min + max) / 2 in
    // ints, which truncates toward zero where Dbu::midpoint floors.
    fn halves(self, bounds: Rect<Dbu>) -> [Rect<Dbu>; 2] {
        let middle = |low: Dbu, high: Dbu| Dbu::new(i64::midpoint(low.get(), high.get()));
        let Rect { min, max } = bounds;
        match self {
            Self::AcrossWidth => {
                let x = middle(min.x, max.x);
                [
                    Rect::new(min, Point::new(x, max.y)),
                    Rect::new(Point::new(x, min.y), max),
                ]
            }
            Self::AcrossHeight => {
                let y = middle(min.y, max.y);
                [
                    Rect::new(min, Point::new(max.x, y)),
                    Rect::new(Point::new(min.x, y), max),
                ]
            }
        }
    }

    /// Whether a load at `at` goes to the half `lower`: up to and on the cut.
    fn in_lower(self, at: Point<Dbu>, lower: &Rect<Dbu>) -> bool {
        match self {
            Self::AcrossWidth => at.x <= lower.max.x,
            Self::AcrossHeight => at.y <= lower.max.y,
        }
    }
}

#[cfg(test)]
mod tests;
