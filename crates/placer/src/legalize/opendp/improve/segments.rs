//! Row segments and the cells in them, as `DetailedMgr` keeps them.

use super::arch::merge_intervals;
use super::{Blockage, ImprovementError, Improver, Result, Segment};
use super::{NodeId, SegmentId};
use crate::legalize::opendp::GroupId;
use pare_common::db::indices::IdVec;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;

impl Improver {
    /// Collects the movable single-row cells by node, as
    /// `DetailedMgr::collectSingleHeightCells`.
    pub(super) fn collect_cells(&mut self) {
        self.fixed_cells = self.net.nodes().filter(|&n| self.is_fixed(n)).collect();
        self.single_height.clear();
        self.multi_height = vec![Vec::new(); 2];
        for n in self.net.nodes() {
            if self.is_terminal(n) || self.is_fixed(n) {
                continue;
            }
            let spanned = self.rows_spanned(n);
            if spanned == 1 {
                self.single_height.push(n);
                continue;
            }
            if spanned >= self.multi_height.len() {
                self.multi_height.resize(spanned + 1, Vec::new());
            }
            self.multi_height[spanned].push(n);
        }
    }

    /// Every movable cell, single-row ones first, as the passes collect
    /// their candidates.
    pub(super) fn movable_cells(&self) -> Vec<NodeId> {
        let mut cells = self.single_height.clone();
        for spanned in self.multi_height.iter().skip(2) {
            cells.extend_from_slice(spanned);
        }
        cells
    }

    /// The spans of each row fixed cells and blockages take, merged, as
    /// `DetailedMgr::findBlockages`.
    pub(super) fn find_blockages(&mut self) {
        let rows = self.arch.rows.len();
        let mut blockages: Vec<Vec<Blockage>> = vec![Vec::new(); rows];
        let mut block = |min_x: Dbu, max_x: Dbu, min_y: Dbu, max_y: Dbu| {
            for (r, row) in self.arch.rows.iter().enumerate() {
                if min_y < row.top() && max_y > row.bottom {
                    blockages[r].push(Blockage { min_x, max_x });
                }
            }
        };
        for &n in &self.fixed_cells {
            if self.is_terminal(n) {
                continue;
            }
            block(
                self.arch.min_x.max(self.left(n)),
                self.arch.max_x.min(self.right(n)),
                self.arch.min_y.max(self.bottom(n)),
                self.arch.max_y.min(self.top(n)),
            );
        }
        for b in &self.net.blockages {
            block(
                self.arch.min_x.max(b.min.x),
                self.arch.max_x.min(b.max.x),
                self.arch.min_y.max(b.min.y),
                self.arch.max_y.min(b.max.y),
            );
        }
        for row in &mut blockages {
            row.sort_by_key(|b| (b.min_x, b.max_x));
            let mut merged: Vec<Blockage> = Vec::with_capacity(row.len());
            for &b in row.iter() {
                match merged.last_mut() {
                    Some(top) if top.max_x >= b.min_x => top.max_x = top.max_x.max(b.max_x),
                    _ => merged.push(b),
                }
            }
            *row = merged;
        }
        self.blockages = blockages;
    }

    /// Cuts the rows into segments between blockages and region edges,
    /// their ends on sites, as `DetailedMgr::findSegments`.
    pub(super) fn find_segments(&mut self) -> Result<()> {
        self.segments = IdVec::new();
        self.segs_in_row = vec![Vec::new(); self.arch.rows.len()];
        for r in 0..self.arch.rows.len() {
            let row = self.arch.rows[r];
            let lo = self.arch.min_x.max(row.left);
            let hi = self.arch.max_x.min(row.right());
            let mut spans = Vec::new();
            let blockages = &self.blockages[r];
            match (blockages.first(), blockages.last()) {
                (Some(first), Some(last)) => {
                    if first.min_x > lo {
                        spans.push((lo, hi.min(first.min_x)));
                    }
                    for pair in blockages.windows(2) {
                        if pair[1].min_x > pair[0].max_x {
                            spans.push((lo.max(pair[0].max_x), hi.min(pair[1].min_x)));
                        }
                    }
                    if last.max_x < hi {
                        spans.push((hi.min(lo.max(last.max_x)), hi));
                    }
                }
                _ => spans.push((lo, hi)),
            }
            for (min_x, max_x) in spans {
                if max_x > min_x {
                    self.add_segment(r, None, min_x, max_x);
                }
            }
        }
        for region in self.grid.groups.ids() {
            let intervals = self.region_intervals(region);
            for (r, row_intervals) in intervals.iter().enumerate() {
                for &(il, ir) in row_intervals {
                    self.split_segments(r, region, il, ir)?;
                }
            }
        }
        for seg in &mut self.segments {
            let row = &self.arch.rows[seg.row];
            let mut ix = (seg.min_x - row.left).get() / row.site_spacing.get();
            if row.left + row.site_spacing * ix < seg.min_x {
                ix += 1;
            }
            seg.min_x = row.left + row.site_spacing * ix;
            let ix = (seg.max_x - row.left).get() / row.site_spacing.get();
            seg.max_x = row.left + row.site_spacing * ix;
        }
        self.cells_in_seg = IdVec::filled(Vec::new(), self.segments.len());
        Ok(())
    }

    fn add_segment(&mut self, row: usize, region: Option<GroupId>, min_x: Dbu, max_x: Dbu) {
        self.segs_in_row[row].push(self.segments.next_id());
        let _ = self.segments.push(Segment {
            row,
            region,
            min_x,
            max_x,
            util: Dbu::ZERO,
        });
    }

    /// Gives the part of each segment of row `r` within `[il, ir)` to the
    /// region, splitting segments as needed; the pieces it adds are visited
    /// too, as in `DetailedMgr::findSegments`.
    fn split_segments(&mut self, r: usize, region: GroupId, il: Dbu, ir: Dbu) -> Result<()> {
        let mut s = 0;
        while s < self.segs_in_row[r].len() {
            let id = self.segs_in_row[r][s];
            s += 1;
            let Segment {
                min_x: sl,
                max_x: sr,
                region: old_region,
                ..
            } = self.segments[id];
            if ir <= sl || il >= sr {
                continue;
            }
            if il <= sl && ir >= sr {
                self.segments[id].region = Some(region);
            } else if il > sl && ir >= sr {
                self.segments[id].max_x = il;
                self.add_segment(r, Some(region), il, sr);
            } else if ir < sr && il <= sl {
                self.segments[id].min_x = ir;
                self.add_segment(r, Some(region), sl, ir);
            } else if il > sl && ir < sr {
                self.segments[id].max_x = il;
                self.add_segment(r, Some(region), il, ir);
                self.add_segment(r, old_region, ir, sr);
            } else {
                return Err(ImprovementError::Internal(
                    "Unexpected problem while constructing segments",
                ));
            }
        }
        Ok(())
    }

    /// The site-aligned spans of each row a region wholly covers in height,
    /// as `DetailedMgr::findRegionIntervals`.
    fn region_intervals(&self, region: GroupId) -> Vec<Vec<(Dbu, Dbu)>> {
        let rows = &self.arch.rows;
        let height = rows[0].height;
        let mut intervals = vec![Vec::new(); rows.len()];
        for rect in &self.grid.groups[region].rects {
            let (xmin, xmax) = (rect.min.x, rect.max.x);
            let (ymin, ymax) = (rect.min.y, rect.max.y);
            for (r, row) in rows.iter().enumerate() {
                let lb = self.arch.min_y + height * r as i64;
                if ymax < lb + height || ymin > lb {
                    continue;
                }
                let i0 = (xmin - row.left).get() / row.site_spacing.get();
                let mut i1 = (xmax - row.left).get() / row.site_spacing.get();
                if row.left + row.site_spacing * i1 != xmax {
                    i1 += 1;
                }
                if i1 > i0 {
                    intervals[r].push((
                        row.left + row.site_spacing * i0,
                        row.left + row.site_spacing * i1,
                    ));
                }
            }
        }
        for row in &mut intervals {
            row.sort_unstable();
            *row = merge_intervals(row);
        }
        intervals
    }

    /// Whether a cell's master may take the orientation of row `row`, as
    /// `checkMasterSymmetry`.
    pub(super) fn master_symmetry_allows(&self, n: NodeId, row: usize) -> bool {
        match self.arch.rows[row].orient {
            Orientation::N | Orientation::FN => true,
            Orientation::FS | Orientation::S => self.grid.nodes[n].symmetry.x,
            _ => false,
        }
    }

    /// The nearest segment of the cell's region and a row its master can
    /// take, preferring one it fits, as `DetailedMgr::findClosestSegment`.
    fn closest_segment(&self, n: NodeId) -> Option<SegmentId> {
        let row = self.arch.closest_row(self.bottom(n));
        let (left, width) = (self.left(n), self.width(n));
        let mut best1: Option<(SegmentId, Dbu)> = None;
        let mut best2: Option<(SegmentId, Dbu)> = None;
        let consider = |r: usize,
                        vert: Dbu,
                        best1: &mut Option<(SegmentId, Dbu)>,
                        best2: &mut Option<(SegmentId, Dbu)>| {
            for &s in &self.segs_in_row[r] {
                let seg = &self.segments[s];
                if self.group(n) != seg.region || !self.master_symmetry_allows(n, seg.row) {
                    continue;
                }
                let xx = seg.min_x.max((seg.max_x - width).min(left));
                let dist = (xx - left).abs() + vert;
                if best1.is_none_or(|(_, d)| dist < d) {
                    *best1 = Some((s, dist));
                }
                if width <= seg.width() && best2.is_none_or(|(_, d)| dist < d) {
                    *best2 = Some((s, dist));
                }
            }
        };
        consider(row, Dbu::ZERO, &mut best1, &mut best2);
        let within = |vert: Dbu, b1: Option<(SegmentId, Dbu)>, b2: Option<(SegmentId, Dbu)>| {
            b1.is_none_or(|(_, d1)| vert <= d1) || b2.is_none_or(|(_, d2)| vert <= d2)
        };
        let rows = self.arch.rows.len();
        for offset in 1..=rows {
            let vert = self.arch.rows[0].height * offset as i64;
            if let Some(below) = row.checked_sub(offset)
                && within(vert, best1, best2)
            {
                consider(below, vert, &mut best1, &mut best2);
            }
            let above = row + offset;
            if above < rows && within(vert, best1, best2) {
                consider(above, vert, &mut best1, &mut best2);
            }
        }
        best2.or(best1).map(|b| b.0)
    }

    /// The nearest stack of segments, one per row, a multi-row cell can
    /// take, preferring one it fits, as
    /// `DetailedMgr::findClosestSpanOfSegments`.
    fn closest_span_of_segments(&self, n: NodeId) -> Result<Option<Vec<SegmentId>>> {
        let spanned = self.rows_spanned(n);
        if spanned <= 1 {
            return Ok(None);
        }
        let mut best1: Option<(Vec<SegmentId>, f64)> = None;
        let mut best2: Option<(Vec<SegmentId>, f64)> = None;
        for b in 0..self.arch.rows.len() {
            let t = b + spanned - 1;
            if t >= self.arch.rows.len() || !self.master_symmetry_allows(n, b) {
                continue;
            }
            for &s in &self.segs_in_row[b] {
                let mut candidates = Vec::new();
                let seg = &self.segments[s];
                self.span_dfs(
                    s,
                    (seg.min_x, seg.max_x),
                    t,
                    spanned,
                    &mut Vec::new(),
                    &mut candidates,
                )?;
                for candidate in candidates {
                    if candidate
                        .iter()
                        .any(|&c| self.segments[c].region != self.group(n))
                    {
                        continue;
                    }
                    let first = &self.segments[candidate[0]];
                    let dy = (self.bottom(n) - self.arch.rows[first.row].bottom).abs();
                    let (mut xmin, mut xmax) = (first.min_x, first.max_x);
                    for &c in &candidate[1..] {
                        xmin = xmin.max(self.segments[c].min_x);
                        xmax = xmax.min(self.segments[c].max_x);
                    }
                    let width = xmax - xmin;
                    let ww = self.width(n).min(width);
                    let xc = self.center_x(n);
                    let xx = (xmin + ww / 2).max((xmax - ww / 2).min(xc));
                    let disp = ((xc - xx).abs() + dy).to_f64();
                    if best1.as_ref().is_none_or(|b| disp < b.1) {
                        best1 = Some((candidate.clone(), disp));
                    }
                    if best2.as_ref().is_none_or(|b| disp < b.1) && self.width(n) <= width {
                        best2 = Some((candidate, disp));
                    }
                }
            }
        }
        Ok(best2.or(best1).map(|b| b.0))
    }

    /// `DetailedMgr::findClosestSpanOfSegmentsDfs`.
    fn span_dfs(
        &self,
        s: SegmentId,
        (xmin, xmax): (Dbu, Dbu),
        top: usize,
        spanned: usize,
        stack: &mut Vec<SegmentId>,
        candidates: &mut Vec<Vec<SegmentId>>,
    ) -> Result<()> {
        stack.push(s);
        let row = self.segments[s].row;
        if row < top {
            for &next in &self.segs_in_row[row + 1] {
                let seg = &self.segments[next];
                if xmax.min(seg.max_x) - xmin.max(seg.min_x) > Dbu::ZERO {
                    let clipped = (xmin.max(seg.min_x), xmax.min(seg.max_x));
                    self.span_dfs(next, clipped, top, spanned, stack, candidates)?;
                }
            }
        } else {
            if stack.len() != spanned {
                return Err(ImprovementError::Internal(
                    "Multi-height cell spans an incorrect number of segments",
                ));
            }
            candidates.push(stack.clone());
        }
        let _ = stack.pop();
        Ok(())
    }

    /// Puts each cell in its nearest segment or segments, clamped into them
    /// and onto their row, as `DetailedMgr::assignCellsToSegments`.
    pub(super) fn assign_cells_to_segments(&mut self, nodes: &[NodeId]) -> Result<()> {
        for &n in nodes {
            let segs = if self.rows_spanned(n) == 1 {
                vec![self.closest_segment(n).ok_or(ImprovementError::Internal(
                    "Unable to assign single height cell to segment",
                ))?]
            } else {
                let segs = self
                    .closest_span_of_segments(n)?
                    .ok_or(ImprovementError::Internal(
                        "Unable to assign multi-height cell to segment",
                    ))?;
                if segs.len() != self.rows_spanned(n) {
                    return Err(ImprovementError::Internal(
                        "Unable to assign multi-height cell to segment",
                    ));
                }
                segs
            };
            let (mut xmin, mut xmax) = (Dbu(i64::MIN), Dbu(i64::MAX));
            for &s in &segs {
                xmin = xmin.max(self.segments[s].min_x);
                xmax = xmax.min(self.segments[s].max_x);
                self.add_cell_to_segment(n, s)?;
            }
            let x = xmin.max((xmax - self.width(n)).min(self.left(n)));
            let y = self.arch.rows[self.segments[segs[0]].row].bottom;
            self.set_position(n, Point::new(x, y));
            self.paint(n);
        }
        Ok(())
    }

    /// `DetailedMgr::addCellToSegment`: in order of center, ahead of equals.
    pub(super) fn add_cell_to_segment(&mut self, n: NodeId, s: SegmentId) -> Result<()> {
        let x = self.center_x(n);
        let at = self.cells_in_seg[s].partition_point(|&c| self.center_x(c) < x);
        self.cells_in_seg[s].insert(at, n);
        let width = self.width(n);
        self.segments[s].util += width;
        if self.cell_segs[n].contains(&s) {
            return Err(ImprovementError::Internal(
                "Segment already present in cell to segment map",
            ));
        }
        if self.cell_segs[n].len() >= self.rows_spanned(n) {
            return Err(ImprovementError::Internal(
                "Cell to segment map incorrectly sized",
            ));
        }
        self.cell_segs[n].push(s);
        Ok(())
    }

    /// `DetailedMgr::removeCellFromSegment`.
    pub(super) fn remove_cell_from_segment(&mut self, n: NodeId, s: SegmentId) -> Result<()> {
        let at =
            self.cells_in_seg[s]
                .iter()
                .position(|&c| c == n)
                .ok_or(ImprovementError::Internal(
                    "Cell not found in expected segment",
                ))?;
        let back = self.cell_segs[n]
            .iter()
            .position(|&c| c == s)
            .ok_or(ImprovementError::Internal("Cannot find segment for cell"))?;
        let _ = self.cell_segs[n].remove(back);
        let _ = self.cells_in_seg[s].remove(at);
        let width = self.width(n);
        self.segments[s].util -= width;
        Ok(())
    }

    /// Puts a segment's cells back in order of center, keeping equals in
    /// place, as `DetailedMgr::resortSegment`.
    pub(super) fn resort_segment(&mut self, s: SegmentId) {
        let mut cells = std::mem::take(&mut self.cells_in_seg[s]);
        cells.sort_by_key(|&c| self.center_x(c));
        self.segments[s].util = cells.iter().map(|&c| self.width(c)).sum();
        self.cells_in_seg[s] = cells;
    }

    pub(super) fn resort_segments(&mut self) {
        for s in self.segments.ids() {
            self.resort_segment(s);
        }
    }

    /// Counts the cells out of their region, row, sites or segment, or
    /// overlapping or breaking a placement rule, as `DetailedMgr`'s checks.
    pub(super) fn placement_problems(&self) -> Result<usize> {
        let mut problems = 0;
        for (s, cells) in self.cells_in_seg.iter_enumerated() {
            let seg = &self.segments[s];
            let mut by_center = cells.clone();
            by_center.sort_by_key(|&c| self.center_x(c));
            for pair in by_center.windows(2) {
                if self.right(pair[0]) > self.left(pair[1]) {
                    problems += 1;
                }
            }
            for &c in &by_center {
                if self.left(c) < seg.min_x || self.right(c) > seg.max_x {
                    problems += 1;
                }
                if self.group(c) != seg.region {
                    problems += 1;
                }
            }
            let mut by_left = cells.clone();
            by_left.sort_by_key(|&c| self.left(c));
            for pair in by_left.windows(2) {
                if self.has_placement_violation(pair[0]) {
                    problems += 1;
                }
            }
        }
        let height = self.arch.rows[0].height;
        for n in self.net.nodes() {
            if self.is_terminal(n) || self.is_fixed(n) {
                continue;
            }
            let rb = self.arch.closest_row(self.bottom(n));
            let rt = rb + self.rows_spanned(n) - 1;
            if rt >= self.arch.rows.len()
                || self.bottom(n) != self.arch.rows[rb].bottom
                || self.top(n) != self.arch.rows[rt].top()
            {
                problems += 1;
            }
            if self.cell_segs[n].is_empty() {
                continue;
            }
            let spanned = (self.height(n).get() / height.get()) as usize;
            if self.cell_segs[n].len() != spanned {
                return Err(ImprovementError::Internal(
                    "Reverse cell map incorrectly sized.",
                ));
            }
            let first = (self.bottom(n) - self.arch.min_y).get() / height.get();
            for r in first..first + spanned as i64 {
                let Some(row) = usize::try_from(r).ok().and_then(|r| self.arch.rows.get(r)) else {
                    problems += 1;
                    continue;
                };
                if (self.left(n) - row.left).get() % row.site_spacing.get() != 0 {
                    problems += 1;
                }
            }
        }
        Ok(problems)
    }

    /// Whether `position` lies within a blockage of a row the cell spans,
    /// as `DetailedMgr::isInsideABlockage`.
    fn inside_blockage(&self, n: NodeId, position: Dbu) -> bool {
        let first = &self.arch.rows[0];
        let rows = self.arch.rows.len() as i64;
        let start = ((self.bottom(n) - first.bottom).get() / first.height.get()).max(0);
        // NOTE: DetailedMgr::isInsideABlockage clamps its exclusive end to the
        // last row, so it never tests the top row; PARE tests every row spanned.
        let end = ((self.top(n) - first.bottom).get() / first.height.get()).min(rows);
        (start..end).any(|r| {
            let row = &self.blockages[r as usize];
            let at = row.partition_point(|b| b.max_x < position);
            row.get(at)
                .is_some_and(|b| position >= b.min_x && position <= b.max_x)
        })
    }

    /// Tries to close the one-site gaps between cells and next to
    /// blockages, returning how many remain, as
    /// `DetailedMgr::getOneSiteGapViolationsPerSegment` with fixing on.
    pub(super) fn close_one_site_gaps(&mut self) -> Result<usize> {
        let gap = self.arch.rows[0].site_width;
        let mut violations = 0;
        for s in self.segments.ids() {
            if self.cells_in_seg[s].len() < 2 {
                continue;
            }
            self.resort_segment(s);
            let mut last = self.cells_in_seg[s][0];
            let mut at_last_x = vec![last];
            let mut i = 0;
            while i < self.cells_in_seg[s].len() {
                let nd = self.cells_in_seg[s][i];
                i += 1;
                if self.right(nd) != self.right(last) {
                    if (self.left(nd) - self.right(last)).abs() == gap {
                        for &cell in &at_last_x.clone() {
                            if !spans_overlap(
                                (self.bottom(cell), self.top(cell)),
                                (self.bottom(nd), self.top(nd)),
                            ) {
                                continue;
                            }
                            if self.is_terminal(nd)
                                || self.is_fixed(nd)
                                || self.rows_spanned(nd) != 1
                            {
                                violations += 1;
                                continue;
                            }
                            if !self.fix_one_site_gap(cell, gap, s, nd)? {
                                violations += 1;
                            }
                            break;
                        }
                    }
                    at_last_x.clear();
                }
                let opens_on_blockage = (self.inside_blockage(nd, self.left(nd) - gap)
                    && !self.inside_blockage(nd, self.left(nd)))
                    || (self.inside_blockage(nd, self.right(nd) + gap)
                        && !self.inside_blockage(nd, self.right(nd)));
                if opens_on_blockage && !self.fix_one_site_gap(nd, gap, s, nd)? {
                    violations += 1;
                }
                at_last_x.push(nd);
                last = nd;
            }
        }
        Ok(violations)
    }

    /// Shifts the cells left of a gap left, or else those right of it right,
    /// as `DetailedMgr::fixOneSiteGapViolations`.
    fn fix_one_site_gap(
        &mut self,
        cell: NodeId,
        gap: Dbu,
        s: SegmentId,
        violating: NodeId,
    ) -> Result<bool> {
        let to = self.right(violating) - gap;
        if self.shift_left_helper(violating, to, s, violating)? && self.verify_move()? {
            self.accept_move();
            return Ok(true);
        }
        self.reject_move()?;
        let to = self.left(cell) + gap * 2;
        if self.shift_right_helper(cell, to, s, violating)? && self.verify_move()? {
            self.accept_move();
            return Ok(true);
        }
        self.reject_move()?;
        Ok(false)
    }
}

/// Whether two closed spans meet.
fn spans_overlap((b1, t1): (Dbu, Dbu), (b2, t2): (Dbu, Dbu)) -> bool {
    (b2 <= b1 && b1 <= t2) || (b1 <= b2 && b2 <= t1)
}

#[cfg(test)]
mod tests {
    use super::super::tests::improver;
    use pare_common::geom::dbu::Dbu;
    use pare_common::geom::point::Point;

    /// The tiny design's top row ends at 4 um; a filler blocks it from
    /// there to the core's right edge at 6 um.
    #[test]
    fn a_blockage_of_the_top_row_is_seen_from_a_cell_in_it() {
        let mut imp = improver();
        let n = imp.movable_cells()[0];
        let top_row = imp.arch.rows[imp.arch.rows.len() - 1].bottom;
        imp.set_position(n, Point::new(Dbu(3000), top_row));

        assert!(imp.inside_blockage(n, Dbu(5000)));
    }
}
