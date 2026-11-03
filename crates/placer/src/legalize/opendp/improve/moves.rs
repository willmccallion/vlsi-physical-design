//! Trial moves and swaps, journaled so they can be taken back, as
//! `DetailedMgr::tryMove`, `DetailedMgr::trySwap` and their helpers.

use super::{ImprovementError, Improver, Move, Result};

/// A node's neighbours around `x` in a segment by center, as the
/// `std::lower_bound` probes of `DetailedMgr`: the first at or right of `x`
/// and the one before it.
fn around(cells: &[usize], at: usize) -> (Option<usize>, Option<usize>) {
    if at == cells.len() {
        (cells.last().copied(), None)
    } else {
        (at.checked_sub(1).map(|i| cells[i]), Some(cells[at]))
    }
}

impl Improver {
    /// The index of the first cell of segment `s` whose center is at or
    /// right of `x`.
    fn lower_bound(&self, s: usize, x: i64) -> usize {
        self.cells_in_seg[s].partition_point(|&c| self.center_x(c) < x)
    }

    /// Moves a cell to `(xj, yj)` in segment `sj`, pushing neighbours aside
    /// as needed; the trial stays journaled when it is legal, as
    /// `DetailedMgr::tryMove`.
    pub(super) fn try_move(&mut self, n: usize, si: usize, xj: i64, sj: usize) -> Result<bool> {
        if !self.master_symmetry_allows(n, self.segments[sj].row) {
            self.reject_move()?;
            return Ok(false);
        }
        let moved = if self.rows_spanned(n) == 1 {
            if si == sj {
                self.try_move_within(n, si, xj, sj)?
            } else {
                self.try_move_across(n, si, xj, sj)?
            }
        } else {
            self.try_move_multi(n, xj, sj)?
        };
        if moved {
            return self.verify_move();
        }
        self.reject_move()?;
        Ok(false)
    }

    /// Swaps a cell with the one at `xj` in segment `sj`, as
    /// `DetailedMgr::trySwap`.
    pub(super) fn try_swap(&mut self, n: usize, si: usize, xj: i64, sj: usize) -> Result<bool> {
        if !self.master_symmetry_allows(n, self.segments[sj].row) {
            self.reject_move()?;
            return Ok(false);
        }
        if self.try_swap_single(n, si, xj, sj)? {
            return self.verify_move();
        }
        self.reject_move()?;
        Ok(false)
    }

    /// `DetailedMgr::tryMove1`: a single-row cell into another segment.
    fn try_move_across(&mut self, n: usize, si: usize, mut xj: i64, sj: usize) -> Result<bool> {
        self.journal.clear();
        if sj == si || self.group(n) != self.segments[sj].region || self.rows_spanned(n) != 1 {
            return Ok(false);
        }
        let yj = self.arch.rows[self.segments[sj].row].bottom;
        let (left, right) = if self.cells_in_seg[sj].is_empty() {
            (None, None)
        } else {
            around(&self.cells_in_seg[sj], self.lower_bound(sj, xj))
        };
        let seg = self.segments[sj];
        if self.width(n) + seg.util > seg.width() {
            return Ok(false);
        }
        let lx = left.map_or(seg.min_x, |l| self.right(l));
        let rx = right.map_or(seg.max_x, |r| self.left(r));
        let Some(x) = self.align(n, xj, lx, rx) else {
            return Ok(false);
        };
        xj = x;
        if !self.add_to_move_list(
            n,
            (self.left(n), self.bottom(n)),
            &[si],
            (xj, yj),
            &[sj],
            true,
        )? {
            return Ok(false);
        }
        if let Some(r) = right
            && !self.shift_right_helper(n, xj, sj, r)?
        {
            return Ok(false);
        }
        if let Some(l) = left
            && !self.shift_left_helper(n, xj, sj, l)?
        {
            return Ok(false);
        }
        Ok(true)
    }

    /// `DetailedMgr::tryMove2`: a single-row cell into a gap of its own
    /// segment beside the cell at `xj`.
    fn try_move_within(&mut self, n: usize, si: usize, xj: i64, sj: usize) -> Result<bool> {
        self.journal.clear();
        if sj != si || self.group(n) != self.segments[sj].region || self.rows_spanned(n) != 1 {
            return Ok(false);
        }
        let yj = self.arch.rows[self.segments[sj].row].bottom;
        let cells = &self.cells_in_seg[sj];
        if cells.is_empty() {
            return Ok(false);
        }
        let ix = self.lower_bound(sj, xj).min(cells.len() - 1);
        let ndj = cells[ix];
        let prev = ix.checked_sub(1).map(|i| cells[i]);
        let next = cells.get(ix + 1).copied();
        let seg = self.segments[sj];
        let gaps = [
            (prev.map_or(seg.min_x, |p| self.right(p)), self.left(ndj)),
            (self.right(ndj), next.map_or(seg.max_x, |p| self.left(p))),
        ];
        for (lx, rx) in gaps {
            if self.width(n) <= rx - lx {
                let Some(x) = self.align(n, xj, lx, rx) else {
                    return Ok(false);
                };
                let from = (self.left(n), self.bottom(n));
                return self.add_to_move_list(n, from, &[si], (x, yj), &[sj], true);
            }
        }
        Ok(false)
    }

    /// `DetailedMgr::tryMove3`: a multi-row cell into the gap at `xj` of a
    /// stack of segments, one per row.
    fn try_move_multi(&mut self, n: usize, xj: i64, sj: usize) -> Result<bool> {
        self.journal.clear();
        let spanned = self.rows_spanned(n);
        if spanned <= 1 || spanned != self.cell_segs[n].len() {
            return Ok(false);
        }
        let mut rb = self.segments[sj].row;
        while rb + spanned >= self.arch.rows.len() {
            let Some(below) = rb.checked_sub(1) else {
                return Ok(false);
            };
            rb = below;
        }
        let mut segs = Vec::with_capacity(spanned);
        for r in rb..rb + spanned {
            let found = self.segs_in_row[r].iter().copied().find(|&s| {
                let seg = &self.segments[s];
                seg.region == self.group(n) && xj >= seg.min_x && xj <= seg.max_x
            });
            let Some(s) = found else {
                break;
            };
            segs.push(s);
        }
        if segs.len() != spanned {
            return Ok(false);
        }
        let (mut xmin, mut xmax) = (i64::from(i32::MIN), i64::from(i32::MAX));
        for &s in &segs {
            let cells = &self.cells_in_seg[s];
            let (mut left, mut right) = (None, None);
            if !cells.is_empty() {
                let at = self.lower_bound(s, xj);
                if at == cells.len() {
                    left = cells.last().copied();
                } else {
                    right = Some(cells[at]);
                    if at > 0 {
                        left = Some(cells[at - 1]);
                        if left == Some(n) {
                            left = (at > 1).then(|| cells[at - 2]);
                        }
                    }
                }
            }
            if left == Some(n) || right == Some(n) {
                return Ok(false);
            }
            let seg = &self.segments[s];
            let lx = left.map_or(seg.min_x, |l| self.right(l));
            let rx = right.map_or(seg.max_x, |r| self.left(r));
            if self.width(n) > rx - lx {
                return Ok(false);
            }
            xmin = xmin.max(lx);
            xmax = xmax.min(rx);
        }
        if self.width(n) > xmax - xmin {
            return Ok(false);
        }
        let Some(x) = self.align(n, xj, xmin, xmax) else {
            return Ok(false);
        };
        let from_segs = self.cell_segs[n].clone();
        let from = (self.left(n), self.bottom(n));
        let to = (x, self.arch.rows[rb].bottom);
        self.add_to_move_list(n, from, &from_segs, to, &segs, false)
    }

    /// `DetailedMgr::trySwap1`: swaps two single-row cells, each into the
    /// other's gap, or, when they are neighbours, shifts the pair.
    fn try_swap_single(&mut self, n: usize, si: usize, mut xj: i64, sj: usize) -> Result<bool> {
        self.journal.clear();
        let cells_j = &self.cells_in_seg[sj];
        if cells_j.is_empty() {
            return Ok(false);
        }
        let ndj = cells_j[self.lower_bound(sj, xj).min(cells_j.len() - 1)];
        if ndj == n || !self.master_symmetry_allows(ndj, self.segments[si].row) {
            return Ok(false);
        }
        if self.rows_spanned(n) != 1 || self.rows_spanned(ndj) != 1 {
            return Ok(false);
        }
        let ix_i = self.cells_in_seg[si]
            .iter()
            .position(|&c| c == n)
            .unwrap_or(self.cells_in_seg[si].len());
        let ix_j = self.cells_in_seg[sj]
            .iter()
            .position(|&c| c == ndj)
            .unwrap_or(self.cells_in_seg[sj].len());
        let mut xi = self.left(n);
        let gap_around = |imp: &Self, s: usize, ix: usize| {
            let cells = &imp.cells_in_seg[s];
            let seg = &imp.segments[s];
            let lx = ix
                .checked_sub(1)
                .and_then(|i| cells.get(i))
                .map_or(seg.min_x, |&p| imp.right(p));
            let rx = cells.get(ix + 1).map_or(seg.max_x, |&p| imp.left(p));
            (lx, rx)
        };
        if si == sj && ix_i + 1 == ix_j {
            let rx = gap_around(self, sj, ix_j).1;
            let lx = gap_around(self, si, ix_i).0;
            if self.width(ndj) + self.width(n) > rx - lx {
                return Ok(false);
            }
            let row = self.segments[si].row;
            let Some(pos) = self.shift(&[ndj, n], &[xi, xj], lx, rx, row) else {
                return Ok(false);
            };
            (xi, xj) = (pos[0], pos[1]);
        } else if si == sj && ix_j + 1 == ix_i {
            let rx = gap_around(self, si, ix_i).1;
            let lx = gap_around(self, sj, ix_j).0;
            if self.width(n) + self.width(ndj) > rx - lx {
                return Ok(false);
            }
            let row = self.segments[si].row;
            let Some(pos) = self.shift(&[n, ndj], &[xj, xi], lx, rx, row) else {
                return Ok(false);
            };
            (xj, xi) = (pos[0], pos[1]);
        } else {
            let (lx, rx) = gap_around(self, si, ix_i);
            if self.width(ndj) > rx - lx {
                return Ok(false);
            }
            let Some(x) = self.align(ndj, xi, lx, rx) else {
                return Ok(false);
            };
            xi = x;
            let (lx, rx) = gap_around(self, sj, ix_j);
            if self.width(n) > rx - lx {
                return Ok(false);
            }
            let Some(x) = self.align(n, xj, lx, rx) else {
                return Ok(false);
            };
            xj = x;
        }
        let (x1, y1) = (self.left(n), self.bottom(n));
        let (x2, y2) = (self.left(ndj), self.bottom(ndj));
        if !self.add_to_move_list(n, (x1, y1), &[si], (xj, y2), &[sj], true)? {
            return Ok(false);
        }
        if !self.add_to_move_list(ndj, (x2, y2), &[sj], (xi, y1), &[si], true)? {
            self.paint(ndj);
            return Ok(false);
        }
        Ok(true)
    }

    /// Places `cells`, in order, on the sites of `[left_limit,
    /// right_limit)` nearest their targets by `OpenDP`'s dynamic program,
    /// whose first cell always takes the first site, as
    /// `DetailedMgr::shift`.
    #[allow(clippy::cast_precision_loss)] // costs are OpenDP's doubles
    fn shift(
        &self,
        cells: &[usize],
        targets: &[i64],
        left_limit: i64,
        right_limit: i64,
        row: usize,
    ) -> Option<Vec<i64>> {
        let row = &self.arch.rows[row];
        let (origin, spacing) = (row.left, row.site_spacing);
        let mut i0 = (left_limit - origin) / spacing;
        if origin + i0 * spacing < left_limit {
            i0 += 1;
        }
        let mut i1 = (right_limit - origin) / spacing;
        if origin + i1 * spacing + row.site_width >= right_limit {
            i1 -= 1;
        }
        let nsites = usize::try_from(i1 - i0 + 1).ok()?;
        let ncells = cells.len();
        let swid: Vec<i64> = cells.iter().map(|&c| self.width(c) / spacing).collect();
        if swid.iter().sum::<i64>() > i1 - i0 + 1 {
            return None;
        }
        let mut site_l = vec![0; ncells];
        let mut k = i0;
        for i in 0..ncells {
            site_l[i] = k;
            k += swid[i];
        }
        let mut site_r = vec![0; ncells];
        let mut k = i1 + 1;
        for i in (0..ncells).rev() {
            site_r[i] = k - swid[i];
            k = site_r[i];
            if site_r[i] < site_l[i] {
                return None;
            }
        }
        let site = |i: usize| i0 + i64::try_from(i).unwrap_or(0) - 1;
        let mut cost = vec![vec![0.0_f64; ncells + 1]; nsites + 1];
        for j in 1..=ncells {
            for (i, row_cost) in cost.iter_mut().enumerate().skip(1) {
                let site_id = site(i);
                if site_id < site_l[j - 1] || site_id > site_r[j - 1] {
                    continue;
                }
                row_cost[j] = (origin + site_id * spacing - targets[j - 1]).abs() as f64;
            }
        }
        let mut tcost = vec![vec![f64::MAX; ncells + 1]; nsites + 1];
        let mut prev = vec![vec![None::<(usize, usize)>; ncells + 1]; nsites + 1];
        tcost[0][0] = 0.0;
        for j in 1..=ncells {
            let prev_wid = if j == 1 { 1 } else { swid[j - 2] };
            let curr_wid = swid[j - 1];
            for i in 1..=nsites {
                let site_id = site(i);
                let c = tcost[i - 1][j];
                if c < tcost[i][j] {
                    tcost[i][j] = c;
                    prev[i][j] = Some((i - 1, j));
                }
                let ii = i64::try_from(i).unwrap_or(0) - prev_wid;
                if let Ok(ii) = usize::try_from(ii)
                    && site_id + curr_wid - 1 <= i1
                {
                    let c = tcost[ii][j - 1] + cost[i][j];
                    if c < tcost[i][j] {
                        tcost[i][j] = c;
                        prev[i][j] = Some((ii, j - 1));
                    }
                }
            }
        }
        let mut at = Some((nsites, ncells));
        let mut reached = false;
        while let Some((i, j)) = at {
            if (i, j) == (0, 0) {
                reached = true;
            }
            at = prev[i][j];
        }
        if !reached {
            return None;
        }
        let mut positions = vec![0; ncells];
        let mut at = (nsites, ncells);
        while at != (0, 0) {
            let Some(back) = prev[at.0][at.1] else {
                break;
            };
            if at.1 != back.1 {
                positions[at.1 - 1] = origin + site(at.0) * spacing;
            }
            at = back;
        }
        Some(positions)
    }

    /// Pushes `right` and the cells after it right of a cell placed at
    /// `xj`, as `DetailedMgr::shiftRightHelper`.
    ///
    /// `OpenDP` snaps each pushed cell to the site at `xj - origin /
    /// spacing`, not `(xj - origin) / spacing`, which flings it about
    /// `spacing` times too far, so a push only stays when its row begins at
    /// zero and the cell at it.
    pub(super) fn shift_right_helper(
        &mut self,
        mut ndi: usize,
        mut xj: i64,
        sj: usize,
        mut ndr: usize,
    ) -> Result<bool> {
        let Some(mut ix) = self.cells_in_seg[sj].iter().position(|&c| c == ndr) else {
            return Ok(false);
        };
        let last = self.cells_in_seg[sj].len() - 1;
        let row = self.arch.rows[self.segments[sj].row];
        while ix <= last && self.left(ndr) < xj + self.width(ndi) {
            if self.rows_spanned(ndr) != 1 {
                return Ok(false);
            }
            xj += self.width(ndi);
            let mut sx = misaligned_site(xj, row.left, row.site_spacing);
            if xj != sx {
                if xj > sx {
                    sx += row.site_spacing;
                }
                if xj < sx {
                    xj = sx;
                }
            }
            let from = (self.left(ndr), self.bottom(ndr));
            if !self.add_to_move_list(ndr, from, &[sj], (xj, from.1), &[sj], true)? {
                return Ok(false);
            }
            if xj + self.width(ndr) > self.segments[sj].max_x {
                return Ok(false);
            }
            if ix == last {
                break;
            }
            ndi = ndr;
            ix += 1;
            ndr = self.cells_in_seg[sj][ix];
        }
        Ok(true)
    }

    /// Pushes `left` and the cells before it left of a cell placed at `xj`,
    /// as `DetailedMgr::shiftLeftHelper`; the misplaced site of
    /// [`Self::shift_right_helper`] lies right of `xj` here, so the cells
    /// pack exactly.
    pub(super) fn shift_left_helper(
        &mut self,
        mut ndi: usize,
        mut xj: i64,
        sj: usize,
        mut ndl: usize,
    ) -> Result<bool> {
        let Some(mut ix) = self.cells_in_seg[sj].iter().position(|&c| c == ndl) else {
            return Ok(false);
        };
        let row = self.arch.rows[self.segments[sj].row];
        while self.right(ndl) > xj {
            if self.rows_spanned(ndl) != 1 {
                return Ok(false);
            }
            xj -= self.width(ndl);
            let sx = misaligned_site(xj, row.left, row.site_spacing);
            if xj > sx {
                xj = sx;
            }
            let from = (self.left(ndl), self.bottom(ndl));
            if !self.add_to_move_list(ndl, from, &[sj], (xj, from.1), &[sj], true)? {
                return Ok(false);
            }
            if xj < self.segments[sj].min_x {
                return Ok(false);
            }
            if ix == 0 {
                break;
            }
            ndi = ndl;
            ix -= 1;
            ndl = self.cells_in_seg[sj][ix];
        }
        let _ = ndi;
        Ok(true)
    }

    /// The site-aligned left edge nearest `x` that keeps the cell within
    /// `[xl, xr)`, as `DetailedMgr::alignPos`.
    pub(super) fn align(&self, n: usize, x: i64, xl: i64, xr: i64) -> Option<i64> {
        let row = &self.arch.rows[0];
        let xr = xr - self.width(n);
        let mut xp = xl.max(xr.min(x));
        xp = row.left + (xp - row.left) / row.site_spacing * row.site_spacing;
        if xp < xl {
            xp += row.site_spacing;
        } else if xp > xr {
            xp -= row.site_spacing;
        }
        (xp >= xl && xp <= xr).then_some(xp)
    }

    /// Moves a cell and journals the move, unless the trial has made too
    /// many, the spot has no row of its site or, when `limited`, it is too
    /// far from where the cell began, as `DetailedMgr::addToMoveList`.
    pub(super) fn add_to_move_list(
        &mut self,
        n: usize,
        from: (i64, i64),
        from_segs: &[usize],
        to: (i64, i64),
        to_segs: &[usize],
        limited: bool,
    ) -> Result<bool> {
        if self.journal.moves.len() >= self.move_limit {
            return Ok(false);
        }
        let x = self.grid.grid_x(to.0);
        let y = self.grid.snap_down_y(to.1);
        if self
            .grid
            .site_orientation(x, y, self.grid.nodes[n].site)
            .is_none()
        {
            return Ok(false);
        }
        let orig = self.orig(n);
        if limited
            && ((to.0 - orig.0).abs() > self.max_displacement.0
                || (to.1 - orig.1).abs() > self.max_displacement.1)
        {
            return Ok(false);
        }
        self.erase(n);
        for &s in from_segs {
            self.remove_cell_from_segment(n, s)?;
        }
        self.set_position(n, to.0, to.1);
        self.paint(n);
        for &s in to_segs {
            self.add_cell_to_segment(n, s)?;
        }
        self.journal_move(Move {
            node: n,
            from,
            from_segs: from_segs.to_vec(),
            to_segs: to_segs.to_vec(),
        });
        Ok(true)
    }

    pub(super) fn journal_move(&mut self, m: Move) {
        let _ = self.journal.nodes.insert(m.node);
        for &pin in &self.net.node_pins[m.node] {
            let _ = self.journal.edges.insert(self.net.pins[pin].edge);
        }
        self.journal.moves.push(m);
    }

    /// Keeps the trial if no cell it moved breaks a placement rule, else
    /// takes it back, as `DetailedMgr::verifyMove`.
    pub(super) fn verify_move(&mut self) -> Result<bool> {
        if self
            .journal
            .nodes
            .iter()
            .any(|&n| self.has_placement_violation(n))
        {
            self.reject_move()?;
            return Ok(false);
        }
        Ok(true)
    }

    pub(super) fn accept_move(&mut self) {
        self.journal.clear();
    }

    /// Takes the trial's moves back, last first, as `Journal::undo`.
    pub(super) fn reject_move(&mut self) -> Result<()> {
        let moves = std::mem::take(&mut self.journal.moves);
        self.undo(&moves)?;
        self.journal.clear();
        Ok(())
    }

    pub(super) fn undo(&mut self, moves: &[Move]) -> Result<()> {
        for m in moves.iter().rev() {
            self.erase(m.node);
            for &s in &m.to_segs {
                self.remove_cell_from_segment(m.node, s)?;
            }
            self.set_position(m.node, m.from.0, m.from.1);
            let x = self.grid.grid_x(m.from.0);
            let y = self.grid.round_y(m.from.1);
            self.grid.paint_pixels(m.node, x, y);
            let orient = self
                .grid
                .site_orientation(x, y, self.grid.nodes[m.node].site)
                .ok_or(ImprovementError::Internal(
                    "cell put back where no row takes it",
                ))?;
            self.orient_node(m.node, orient);
            for &s in &m.from_segs {
                self.add_cell_to_segment(m.node, s)?;
            }
        }
        Ok(())
    }
}

/// `originX + (xj - originX / spacing) * spacing` in C++ `int` arithmetic.
#[allow(clippy::cast_possible_truncation)] // wraps as OpenDP's int does
fn misaligned_site(xj: i64, origin: i64, spacing: i64) -> i64 {
    let site = (xj - origin / spacing) as i32;
    i64::from((origin as i32).wrapping_add(site.wrapping_mul(spacing as i32)))
}
