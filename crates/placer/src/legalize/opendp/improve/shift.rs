//! `ShiftLegalizer`: puts every movable cell in its segments and clumps
//! overlapping cells apart, which on a legal placement moves nothing.

use super::{ImprovementError, Improver, Result};

/// A run of cells pushed together, as `ShiftLegalizer`'s `Clump`.
#[derive(Clone, Debug, Default)]
struct Clump {
    nodes: Vec<usize>,
    weight: f64,
    weighted_position: f64,
    position: i64,
}

/// The weight of the stand-ins pinning each segment's ends.
const END_WEIGHT: f64 = 1.0e8;

pub(super) fn legalize(imp: &mut Improver) -> Result<()> {
    imp.collect_cells();
    imp.find_blockages();
    imp.find_segments()?;
    let original: Vec<(i64, i64)> = (0..imp.net.node_count())
        .map(|n| (imp.left(n), imp.bottom(n)))
        .collect();
    let mut cells = Vec::new();
    if !imp.single_height.is_empty() {
        let single = imp.single_height.clone();
        imp.assign_cells_to_segments(&single)?;
        cells.extend(single);
    }
    for spanned in 2..imp.multi_height.len() {
        let multi = imp.multi_height[spanned].clone();
        if !multi.is_empty() {
            imp.assign_cells_to_segments(&multi)?;
            cells.extend(multi);
        }
    }
    if !ordered_without_cycles(imp, &cells) {
        return Err(ImprovementError::Internal(
            "Cells incorrectly ordered during shifting",
        ));
    }
    shift(imp, &cells)?;
    let displaced = cells
        .iter()
        .any(|&n| (imp.left(n), imp.bottom(n)) != original[n]);
    if displaced {
        log::warn!("unexpected displacement while seeding detailed improvement");
    }
    let problems = imp.placement_problems()?;
    if problems > 0 {
        log::warn!("{problems} placement problems while seeding detailed improvement");
    }
    Ok(())
}

/// Whether the segments' left-to-right orders admit one order of all the
/// cells, as the topological sort `ShiftLegalizer::legalize` checks.
fn ordered_without_cycles(imp: &Improver, cells: &[usize]) -> bool {
    let size = imp.net.node_count();
    let mut incoming = vec![0_usize; size];
    let mut outgoing = vec![Vec::new(); size];
    for seg in &imp.cells_in_seg {
        for pair in seg.windows(2) {
            incoming[pair[1]] += 1;
            outgoing[pair[0]].push(pair[1]);
        }
    }
    let mut order: Vec<usize> = cells
        .iter()
        .copied()
        .filter(|&n| incoming[n] == 0)
        .collect();
    let mut i = 0;
    while i < order.len() {
        for &next in &outgoing[order[i]] {
            incoming[next] -= 1;
            if incoming[next] == 0 {
                order.push(next);
            }
        }
        i += 1;
    }
    order.len() == cells.len()
}

/// Clumps each segment's cells between stand-ins pinned to its ends,
/// as `ShiftLegalizer::shift` and `ShiftLegalizer::clump`.
#[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)] // positions are dbu
fn shift(imp: &mut Improver, cells: &[usize]) -> Result<()> {
    let nodes = imp.net.node_count();
    let segs = imp.segments.len();
    let left_dummy = |s: usize| nodes + s;
    let right_dummy = |s: usize| nodes + segs + s;
    let total = nodes + 2 * segs;
    let mut width = vec![0_i64; total];
    for (n, w) in width.iter_mut().enumerate().take(nodes) {
        *w = imp.width(n);
    }
    let mut incoming = vec![Vec::new(); total];
    for s in 0..segs {
        let mut order = Vec::with_capacity(imp.cells_in_seg[s].len() + 2);
        order.push(left_dummy(s));
        order.extend_from_slice(&imp.cells_in_seg[s]);
        order.push(right_dummy(s));
        for pair in order.windows(2) {
            incoming[pair[1]].push(pair[0]);
        }
    }
    let mut clumps = Vec::with_capacity(2 * segs + cells.len());
    let mut owner = vec![usize::MAX; total];
    let mut offset = vec![0_i64; total];
    let mut add = |node: usize, left: i64, weight: f64, position: i64| {
        owner[node] = clumps.len();
        clumps.push(Clump {
            nodes: vec![node],
            weight,
            weighted_position: weight * left as f64,
            position,
        });
    };
    for s in 0..segs {
        add(
            left_dummy(s),
            imp.segments[s].min_x,
            END_WEIGHT,
            imp.segments[s].min_x,
        );
    }
    for &n in cells {
        let mut position = imp.left(n);
        for &s in &imp.cell_segs[n] {
            let seg = &imp.segments[s];
            position = position.max(seg.min_x).min(seg.max_x - imp.width(n));
        }
        add(n, imp.left(n), 1.0, position);
    }
    for s in 0..segs {
        add(
            right_dummy(s),
            imp.segments[s].max_x,
            END_WEIGHT,
            imp.segments[s].max_x,
        );
    }
    for c in 0..clumps.len() {
        let mut r = c;
        while let Some((l, dist)) = violated(&clumps, r, &incoming, &owner, &offset, &width) {
            let moved = std::mem::take(&mut clumps[r].nodes);
            for &node in &moved {
                offset[node] += dist;
                owner[node] = l;
            }
            let (weight, weighted) = (clumps[r].weight, clumps[r].weighted_position);
            let left = &mut clumps[l];
            left.nodes.extend(moved);
            left.weighted_position += weighted - dist as f64 * weight;
            left.weight += weight;
            left.position = (left.weighted_position / left.weight).floor() as i64;
            r = l;
        }
    }
    for &n in cells {
        let row = imp.cell_segs[n]
            .iter()
            .map(|&s| imp.segments[s].row)
            .min()
            .ok_or(ImprovementError::Internal(
                "Shifting failed during legalization",
            ))?;
        let x = clumps[owner[n]].position + offset[n];
        let y = imp.arch.rows[row].bottom;
        imp.set_position(n, x, y);
    }
    Ok(())
}

/// The clump the worst overlap of `r` on its left neighbours is with and
/// how far it must sit right of it, as `ShiftLegalizer::violated`.
fn violated(
    clumps: &[Clump],
    r: usize,
    incoming: &[Vec<usize>],
    owner: &[usize],
    offset: &[i64],
    width: &[i64],
) -> Option<(usize, i64)> {
    let mut worst: Option<(i64, usize, i64)> = None;
    for &ndr in &clumps[r].nodes {
        for &ndl in &incoming[ndr] {
            let t = owner[ndl];
            if t == r {
                continue;
            }
            let pdst = clumps[r].position + offset[ndr];
            let psrc = clumps[t].position + offset[ndl];
            let diff = pdst - (psrc + width[ndl]);
            if diff < 0 && worst.is_none_or(|w| diff < w.0) {
                worst = Some((diff, t, offset[ndl] + width[ndl] - offset[ndr]));
            }
        }
    }
    worst.map(|(_, l, dist)| (l, dist))
}
