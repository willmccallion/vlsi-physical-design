//! Reordering, as `DetailedReorderer`: every window of three neighbouring
//! single-row cells is tried in each order, spread evenly over their gap,
//! and the order with the shortest horizontal wirelength kept.

use super::mis::converged;
use super::{EdgeId, NodeId, SegmentId};
use super::{Improver, Result, SKIP_NETS_LARGER_THAN};
use pare_common::db::indices::IdVec;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;

const WINDOW: usize = 3;

/// `ro -p passes -t tolerance`.
pub(super) fn run(imp: &mut Improver, passes: usize, tolerance: f64) -> Result<()> {
    let tolerance = tolerance.max(0.01);
    imp.resort_segments();
    let mut hpwl = imp.total_hpwl();
    if hpwl == Dbu::ZERO {
        return Ok(());
    }
    let mut marks = EdgeMarks::new(imp.net.edges.len());
    for _ in 0..passes.max(1) {
        let last = hpwl;
        reorder_segments(imp, &mut marks)?;
        hpwl = imp.total_hpwl();
        if converged(last, hpwl, tolerance) {
            break;
        }
    }
    imp.resort_segments();
    Ok(())
}

/// Each edge's last traversal, so a cost counts it once.
struct EdgeMarks {
    mark: IdVec<EdgeId, usize>,
    traversal: usize,
}

impl EdgeMarks {
    fn new(edges: usize) -> Self {
        Self {
            mark: IdVec::filled(0, edges),
            traversal: 0,
        }
    }
}

/// `DetailedReorderer::reorder`: windows over each run of single-row cells
/// of each segment.
fn reorder_segments(imp: &mut Improver, marks: &mut EdgeMarks) -> Result<()> {
    marks.traversal = 0;
    marks.mark.fill(0);
    for s in imp.segments.ids() {
        let n = imp.cells_in_seg[s].len();
        if n < 2 {
            continue;
        }
        sort_segment(imp, s, 0, n);
        let mut j = 0;
        while j < n {
            while j < n && imp.rows_spanned(imp.cells_in_seg[s][j]) != 1 {
                j += 1;
            }
            let jstrt = j;
            while j < n && imp.rows_spanned(imp.cells_in_seg[s][j]) == 1 {
                j += 1;
            }
            let jstop = j - 1;
            let mut i = jstrt;
            while i + WINDOW <= jstop {
                let istop = jstop.min(i + WINDOW - 1);
                let istrt = if istop == jstop {
                    jstrt.max((istop + 1).saturating_sub(WINDOW))
                } else {
                    i
                };
                let seg = imp.segments[s];
                let cells = &imp.cells_in_seg[s];
                let right_limit = cells
                    .get(istop + 1)
                    .map_or(seg.max_x, |&next| imp.left(next).min(seg.max_x));
                let left_limit = istrt
                    .checked_sub(1)
                    .map_or(seg.min_x, |p| imp.right(cells[p]).max(seg.min_x));
                let window = Window {
                    segment: s,
                    first: istrt,
                    last: istop,
                    left_limit,
                    right_limit,
                };
                reorder_window(imp, marks, &window)?;
                i += 1;
            }
        }
    }
    Ok(())
}

/// Orders segment `s`'s cells `[start, end)` by center as `std::sort` does.
fn sort_segment(imp: &mut Improver, s: SegmentId, start: usize, end: usize) {
    let mut cells = std::mem::take(&mut imp.cells_in_seg[s]);
    let centers: Vec<Dbu> = cells.iter().map(|&c| imp.center_x(c)).collect();
    let mut keyed: Vec<(Dbu, NodeId)> = centers[start..end]
        .iter()
        .copied()
        .zip(cells[start..end].iter().copied())
        .collect();
    super::super::cpp::sort(&mut keyed, &|a, b| a.0 < b.0);
    for (slot, (_, c)) in cells[start..end].iter_mut().zip(keyed) {
        *slot = c;
    }
    imp.cells_in_seg[s] = cells;
}

/// A run of a segment's cells, `first` to `last` in its order, and the
/// gap they may use.
struct Window {
    segment: SegmentId,
    first: usize,
    last: usize,
    left_limit: Dbu,
    right_limit: Dbu,
}

/// Tries every order of the window's cells, spaced evenly in its gap, and
/// keeps the cheapest legal one, as `DetailedReorderer::reorder` of a
/// window.
fn reorder_window(imp: &mut Improver, marks: &mut EdgeMarks, w: &Window) -> Result<()> {
    let cells: Vec<NodeId> = imp.cells_in_seg[w.segment][w.first..=w.last].to_vec();
    let original: Vec<Dbu> = cells.iter().map(|&n| imp.left(n)).collect();
    let width: Vec<Dbu> = cells.iter().map(|&n| imp.width(n)).collect();
    let site_width = imp.arch.rows[0].site_width;
    let Some(pads) = even_padding(&width, w.right_limit - w.left_limit, site_width) else {
        return Ok(());
    };
    let orig_cost = cost(imp, marks, &cells);
    let Some(best) = cheapest_order(imp, marks, &cells, &width, &pads, w.left_limit, orig_cost)?
    else {
        return place_at(imp, &cells, &original);
    };
    place_at(imp, &cells, &best)?;
    sort_segment(imp, w.segment, w.first, w.last + 1);
    let realigned = realign(imp, w)?;
    let ordered = imp.cells_in_seg[w.segment][w.first..=w.last].to_vec();
    let failed = match realigned {
        None => true,
        Some(shifted) => {
            (shifted && cost(imp, marks, &ordered) >= orig_cost)
                || ordered.iter().any(|&n| imp.has_placement_violation(n))
        }
    };
    if failed {
        let lefts: Vec<Dbu> = ordered
            .iter()
            .map(|&n| {
                cells
                    .iter()
                    .position(|&c| c == n)
                    .map_or_else(|| imp.left(n), |i| original[i])
            })
            .collect();
        place_at(imp, &ordered, &lefts)?;
        sort_segment(imp, w.segment, w.first, w.last + 1);
    }
    Ok(())
}

/// The site padding left and right of each cell that spreads cells of
/// `width` evenly over `span`; none when they do not fit.
fn even_padding(width: &[Dbu], span: Dbu, site_width: Dbu) -> Option<(Vec<Dbu>, Vec<Dbu>)> {
    let size = width.len();
    let total_width: Dbu = width.iter().copied().sum();
    if span < total_width {
        return None;
    }
    let space_per_cell = (span - total_width) / size as i64;
    let sites_per_cell = space_per_cell.get() / site_width.get();
    let sites_right = sites_per_cell >> 1;
    let sites_left = sites_per_cell - sites_right;
    let mut padding = Dbu::ZERO;
    let (mut left_pad, mut right_pad) = (vec![Dbu::ZERO; size], vec![Dbu::ZERO; size]);
    for i in 0..size {
        if total_width + padding + site_width * sites_right < span {
            padding += site_width * sites_right;
            right_pad[i] += site_width * sites_right;
        }
        if total_width + padding + site_width * sites_left < span {
            padding += site_width * sites_left;
            left_pad[i] += site_width * sites_left;
        }
    }
    (span >= total_width + padding).then_some((left_pad, right_pad))
}

/// The left edges of the cheapest order of `cells` packed from
/// `left_limit` with `pads`, each cell within reach of where it started,
/// if one costs less than `orig_cost`.
fn cheapest_order(
    imp: &mut Improver,
    marks: &mut EdgeMarks,
    cells: &[NodeId],
    width: &[Dbu],
    (left_pad, right_pad): &(Vec<Dbu>, Vec<Dbu>),
    left_limit: Dbu,
    orig_cost: f64,
) -> Result<Option<Vec<Dbu>>> {
    let size = cells.len();
    let mut best_cost = orig_cost;
    let mut best: Option<Vec<Dbu>> = None;
    let mut order: Vec<usize> = (0..size).collect();
    let mut positions = vec![Dbu::ZERO; size];
    loop {
        let mut within_reach = true;
        let mut x = left_limit;
        for &ix in &order {
            let n = cells[ix];
            x += left_pad[ix];
            positions[ix] = x;
            imp.erase(n);
            let bottom = imp.bottom(n);
            imp.set_position(n, Point::new(x, bottom));
            imp.paint(n)?;
            x += width[ix] + right_pad[ix];
            if (imp.left(n) - imp.orig(n).x).abs() > imp.max_displacement.x {
                within_reach = false;
            }
        }
        if within_reach {
            let c = cost(imp, marks, cells);
            if c < best_cost {
                best = Some(positions.clone());
                best_cost = c;
            }
        }
        if !next_permutation(&mut order) {
            return Ok(best);
        }
    }
}

/// Aligns the window's cells, left to right, to legal sites within reach;
/// whether any moved, or none when one cannot be aligned.
fn realign(imp: &mut Improver, w: &Window) -> Result<Option<bool>> {
    let mut shifted = false;
    let mut left = w.left_limit;
    for i in w.first..=w.last {
        let n = imp.cells_in_seg[w.segment][i];
        let Some(x) = imp.align(n, imp.left(n), left, w.right_limit) else {
            return Ok(None);
        };
        shifted |= x != imp.left(n);
        imp.erase(n);
        let bottom = imp.bottom(n);
        imp.set_position(n, Point::new(x, bottom));
        imp.paint(n)?;
        left = imp.right(n);
        if (imp.left(n) - imp.orig(n).x).abs() > imp.max_displacement.x {
            return Ok(None);
        }
    }
    Ok(Some(shifted))
}

/// Moves each cell to its left edge, keeping its row.
fn place_at(imp: &mut Improver, cells: &[NodeId], lefts: &[Dbu]) -> Result<()> {
    for (&n, &x) in cells.iter().zip(lefts) {
        imp.erase(n);
        let bottom = imp.bottom(n);
        imp.set_position(n, Point::new(x, bottom));
        imp.paint(n)?;
    }
    Ok(())
}

/// The horizontal span of the cells' nets, each net once, or the largest
/// cost if one of them breaks a placement rule, as
/// `DetailedReorderer::cost`.
fn cost(imp: &Improver, marks: &mut EdgeMarks, cells: &[NodeId]) -> f64 {
    marks.traversal += 1;
    let mut cost = 0.0;
    for &n in cells {
        if imp.has_placement_violation(n) {
            return f64::MAX;
        }
        for &pin in &imp.net.node_pins[n] {
            let e = imp.net.pins[pin].edge;
            let pins = &imp.net.edges[e];
            if pins.len() <= 1
                || pins.len() >= SKIP_NETS_LARGER_THAN
                || marks.mark[e] == marks.traversal
            {
                continue;
            }
            marks.mark[e] = marks.traversal;
            let (mut lo, mut hi) = (Dbu(i64::from(i32::MAX)), Dbu(i64::from(i32::MIN)));
            for &p in pins {
                let x = imp.pin_position(p).x;
                lo = lo.min(x);
                hi = hi.max(x);
            }
            cost += (hi - lo).to_f64();
        }
    }
    cost
}

/// The next lexicographic permutation, as `std::next_permutation`; false
/// after the last, which it leaves sorted.
fn next_permutation(order: &mut [usize]) -> bool {
    let Some(i) = (1..order.len()).rev().find(|&i| order[i - 1] < order[i]) else {
        order.reverse();
        return false;
    };
    let j = (i..order.len())
        .rev()
        .find(|&j| order[j] > order[i - 1])
        .unwrap_or(i);
    order.swap(i - 1, j);
    order[i..].reverse();
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    const SITE: Dbu = Dbu(10);

    fn lengths(values: &[i64]) -> Vec<Dbu> {
        values.iter().copied().map(Dbu).collect()
    }

    #[test]
    fn cells_are_padded_evenly_in_whole_sites_over_the_gap() {
        let padding = even_padding(&lengths(&[20, 30]), Dbu(100), SITE);

        assert_eq!(padding, Some((lengths(&[10, 10]), lengths(&[10, 10]))));
    }

    #[test]
    fn cells_filling_the_gap_get_no_padding() {
        let padding = even_padding(&lengths(&[20, 30]), Dbu(50), SITE);

        assert_eq!(padding, Some((lengths(&[0, 0]), lengths(&[0, 0]))));
    }

    #[test]
    fn padding_that_would_fill_the_gap_exactly_is_left_off() {
        // Three spare sites split one right and two left; the two left
        // would close the gap, so only the right one is taken.
        let padding = even_padding(&lengths(&[10]), Dbu(40), SITE);

        assert_eq!(padding, Some((lengths(&[0]), lengths(&[10]))));
    }

    #[test]
    fn cells_wider_than_the_gap_get_no_padding() {
        assert_eq!(even_padding(&lengths(&[20, 30]), Dbu(40), SITE), None);
    }

    #[test]
    fn permutations_step_through_every_order_then_return_sorted() {
        let mut order = vec![0, 1, 2];
        let mut seen = vec![order.clone()];

        while next_permutation(&mut order) {
            seen.push(order.clone());
        }

        assert_eq!(
            seen,
            [
                [0, 1, 2],
                [0, 2, 1],
                [1, 0, 2],
                [1, 2, 0],
                [2, 0, 1],
                [2, 1, 0]
            ]
        );
        assert_eq!(order, [0, 1, 2]);
    }

    #[test]
    fn repeated_values_are_permuted_once_each() {
        let mut order = vec![0, 0, 1];
        let mut count = 1;

        while next_permutation(&mut order) {
            count += 1;
        }

        assert_eq!(count, 3);
    }
}
