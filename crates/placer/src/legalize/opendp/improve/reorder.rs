//! Reordering, as `DetailedReorderer`: every window of three neighbouring
//! single-row cells is tried in each order, spread evenly over their gap,
//! and the order with the shortest horizontal wirelength kept.

use super::mis::converged;
use super::{Improver, SKIP_NETS_LARGER_THAN};

const WINDOW: usize = 3;

/// `ro -p passes -t tolerance`.
pub(super) fn run(imp: &mut Improver, passes: usize, tolerance: f64) {
    let tolerance = tolerance.max(0.01);
    imp.resort_segments();
    let mut hpwl = imp.total_hpwl();
    if hpwl == 0 {
        return;
    }
    let mut marks = EdgeMarks::new(imp.net.edges.len());
    for _ in 0..passes.max(1) {
        let last = hpwl;
        reorder_segments(imp, &mut marks);
        hpwl = imp.total_hpwl();
        if converged(last, hpwl, tolerance) {
            break;
        }
    }
    imp.resort_segments();
}

/// Each edge's last traversal, so a cost counts it once.
struct EdgeMarks {
    mark: Vec<usize>,
    traversal: usize,
}

impl EdgeMarks {
    fn new(edges: usize) -> Self {
        Self {
            mark: vec![0; edges],
            traversal: 0,
        }
    }
}

/// `DetailedReorderer::reorder`: windows over each run of single-row cells
/// of each segment.
fn reorder_segments(imp: &mut Improver, marks: &mut EdgeMarks) {
    marks.traversal = 0;
    marks.mark.fill(0);
    for s in 0..imp.segments.len() {
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
            let Some(jstop) = j.checked_sub(1) else {
                continue;
            };
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
                reorder_window(imp, marks, s, istrt, istop, left_limit, right_limit);
                i += 1;
            }
        }
    }
}

/// Orders segment `s`'s cells `[start, end)` by center as `std::sort` does.
fn sort_segment(imp: &mut Improver, s: usize, start: usize, end: usize) {
    let mut cells = std::mem::take(&mut imp.cells_in_seg[s]);
    let centers: Vec<i64> = cells.iter().map(|&c| imp.center_x(c)).collect();
    let mut keyed: Vec<(i64, usize)> = centers[start..end]
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

/// Tries every order of the window's cells, spaced evenly in its gap, and
/// keeps the cheapest legal one, as `DetailedReorderer::reorder` of a
/// window.
fn reorder_window(
    imp: &mut Improver,
    marks: &mut EdgeMarks,
    s: usize,
    jstrt: usize,
    jstop: usize,
    left_limit: i64,
    right_limit: i64,
) {
    let window: Vec<usize> = imp.cells_in_seg[s][jstrt..=jstop].to_vec();
    let size = window.len();
    let original: Vec<i64> = window.iter().map(|&n| imp.left(n)).collect();
    let width: Vec<i64> = window.iter().map(|&n| imp.width(n)).collect();
    let total_width: i64 = width.iter().sum();
    let span = right_limit - left_limit;
    if span < total_width {
        return;
    }
    let size_i64 = i64::try_from(size).unwrap_or(1);
    let space_per_cell = (span - total_width) / size_i64;
    let site_width = imp.arch.rows[0].site_width;
    let sites_per_cell = space_per_cell / site_width;
    let sites_right = sites_per_cell >> 1;
    let sites_left = sites_per_cell - sites_right;
    let mut padding = 0;
    let (mut left_pad, mut right_pad) = (vec![0; size], vec![0; size]);
    for i in 0..size {
        if total_width + padding + sites_right * site_width < span {
            padding += sites_right * site_width;
            right_pad[i] += sites_right * site_width;
        }
        if total_width + padding + sites_left * site_width < span {
            padding += sites_left * site_width;
            left_pad[i] += sites_left * site_width;
        }
    }
    if span < total_width + padding {
        return;
    }
    let orig_cost = cost(imp, marks, &window);
    let mut best_cost = orig_cost;
    let mut best: Option<Vec<i64>> = None;
    let mut order: Vec<usize> = (0..size).collect();
    let mut positions = vec![0; size];
    loop {
        let mut within_reach = true;
        let mut x = left_limit;
        for &ix in &order {
            let n = window[ix];
            x += left_pad[ix];
            positions[ix] = x;
            imp.erase(n);
            let bottom = imp.bottom(n);
            imp.set_position(n, x, bottom);
            imp.paint(n);
            x += width[ix] + right_pad[ix];
            if (imp.left(n) - imp.orig(n).0).abs() > imp.max_displacement.0 {
                within_reach = false;
            }
        }
        if within_reach {
            let c = cost(imp, marks, &window);
            if c < best_cost {
                best = Some(positions.clone());
                best_cost = c;
            }
        }
        if !next_permutation(&mut order) {
            break;
        }
    }
    let Some(best) = best else {
        place_at(imp, &window, &original);
        return;
    };
    place_at(imp, &window, &best);
    sort_segment(imp, s, jstrt, jstop + 1);
    let mut failed = false;
    let mut shifted = false;
    let mut left = left_limit;
    for i in jstrt..=jstop {
        let n = imp.cells_in_seg[s][i];
        let Some(x) = imp.align(n, imp.left(n), left, right_limit) else {
            failed = true;
            break;
        };
        shifted |= x != imp.left(n);
        imp.erase(n);
        let bottom = imp.bottom(n);
        imp.set_position(n, x, bottom);
        imp.paint(n);
        left = imp.right(n);
        if (imp.left(n) - imp.orig(n).0).abs() > imp.max_displacement.0 {
            failed = true;
            break;
        }
    }
    let ordered = imp.cells_in_seg[s][jstrt..=jstop].to_vec();
    if !failed && shifted && cost(imp, marks, &ordered) >= orig_cost {
        failed = true;
    }
    if !failed {
        failed = ordered.iter().any(|&n| imp.has_placement_violation(n));
    }
    if failed {
        let lefts: Vec<i64> = ordered
            .iter()
            .map(|&n| {
                window
                    .iter()
                    .position(|&w| w == n)
                    .map_or_else(|| imp.left(n), |i| original[i])
            })
            .collect();
        place_at(imp, &ordered, &lefts);
        sort_segment(imp, s, jstrt, jstop + 1);
    }
}

/// Moves each cell to its left edge, keeping its row.
fn place_at(imp: &mut Improver, cells: &[usize], lefts: &[i64]) {
    for (&n, &x) in cells.iter().zip(lefts) {
        imp.erase(n);
        let bottom = imp.bottom(n);
        imp.set_position(n, x, bottom);
        imp.paint(n);
    }
}

/// The horizontal span of the cells' nets, each net once, or the largest
/// cost if one of them breaks a placement rule, as
/// `DetailedReorderer::cost`.
#[allow(clippy::cast_precision_loss)] // as OpenDP's double
fn cost(imp: &Improver, marks: &mut EdgeMarks, cells: &[usize]) -> f64 {
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
            let (mut lo, mut hi) = (i64::from(i32::MAX), i64::from(i32::MIN));
            for &p in pins {
                let x = imp.pin_position(p).0;
                lo = lo.min(x);
                hi = hi.max(x);
            }
            cost += (hi - lo) as f64;
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
