//! Trial moves and swaps, journaled so they can be taken back, as
//! `DetailedMgr::tryMove`, `DetailedMgr::trySwap` and their helpers.

use super::arch::Row;
use super::{ArchRowId, ImprovementError, Improver, Move, Result, Sealed, Trial};
use super::{NodeId, SegmentId};
use pare_common::convert::SaturatingFrom;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::Interval;
use pare_common::geom::point::Point;

/// How far a journaled move may take a cell from where improvement found
/// it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Reach {
    /// Within the improver's maximum displacement.
    Limited,
    /// Anywhere: the multi-row moves of `try_move_multi`.
    Free,
}

/// A cell's segment and where it stands among the segment's cells.
#[derive(Clone, Copy, Debug)]
struct SwapSide {
    segment: SegmentId,
    index: usize,
}

/// A node's neighbours around `x` in a segment by center, as the
/// `std::lower_bound` probes of `DetailedMgr`: the first at or right of `x`
/// and the one before it.
fn around(cells: &[NodeId], at: usize) -> (Option<NodeId>, Option<NodeId>) {
    if at == cells.len() {
        (cells.last().copied(), None)
    } else {
        (at.checked_sub(1).map(|i| cells[i]), Some(cells[at]))
    }
}

impl Improver {
    /// The index of the first cell of segment `s` whose center is at or
    /// right of `x`.
    fn lower_bound(&self, s: SegmentId, x: Dbu) -> usize {
        self.cells_in_seg[s].partition_point(|&c| self.center_x(c) < x)
    }

    /// Moves a cell to `(xj, yj)` in segment `sj`, pushing neighbours aside
    /// as needed, as `DetailedMgr::tryMove`: the trial, left open for the
    /// caller to keep or take back, where it is legal.
    pub(super) fn try_move(
        &mut self,
        n: NodeId,
        si: SegmentId,
        xj: Dbu,
        sj: SegmentId,
    ) -> Result<Option<Trial>> {
        let trial = self.begin_trial();
        let moved = if !self.master_symmetry_allows(n, self.segments[sj].row) {
            false
        } else if self.rows_spanned(n) != 1 {
            self.try_move_multi(n, xj, sj)?
        } else if si == sj {
            self.try_move_within(n, si, xj, sj)?
        } else {
            self.try_move_across(n, si, xj, sj)?
        };
        self.legal_trial(trial, moved)
    }

    /// Swaps a cell with the one at `xj` in segment `sj`, as
    /// `DetailedMgr::trySwap`: the trial, left open for the caller to keep
    /// or take back, where it is legal.
    pub(super) fn try_swap(
        &mut self,
        n: NodeId,
        si: SegmentId,
        xj: Dbu,
        sj: SegmentId,
    ) -> Result<Option<Trial>> {
        let trial = self.begin_trial();
        let swapped = self.master_symmetry_allows(n, self.segments[sj].row)
            && self.try_swap_single(n, si, xj, sj)?;
        self.legal_trial(trial, swapped)
    }

    /// `DetailedMgr::tryMove1`: a single-row cell into another segment.
    fn try_move_across(
        &mut self,
        n: NodeId,
        si: SegmentId,
        mut xj: Dbu,
        sj: SegmentId,
    ) -> Result<bool> {
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
        let room = Interval::new(
            left.map_or(seg.min_x, |l| self.right(l)),
            right.map_or(seg.max_x, |r| self.left(r)),
        );
        let Some(x) = self.align(n, xj, room) else {
            return Ok(false);
        };
        xj = x;
        if !self.add_to_move_list(
            n,
            self.position(n),
            &[si],
            Point::new(xj, yj),
            &[sj],
            Reach::Limited,
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
    fn try_move_within(
        &mut self,
        n: NodeId,
        si: SegmentId,
        xj: Dbu,
        sj: SegmentId,
    ) -> Result<bool> {
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
            Interval::new(prev.map_or(seg.min_x, |p| self.right(p)), self.left(ndj)),
            Interval::new(self.right(ndj), next.map_or(seg.max_x, |p| self.left(p))),
        ];
        for gap in gaps {
            if self.width(n) <= gap.len() {
                let Some(x) = self.align(n, xj, gap) else {
                    return Ok(false);
                };
                let from = self.position(n);
                return self.add_to_move_list(
                    n,
                    from,
                    &[si],
                    Point::new(x, yj),
                    &[sj],
                    Reach::Limited,
                );
            }
        }
        Ok(false)
    }

    /// `DetailedMgr::tryMove3`: a multi-row cell into the gap at `xj` of a
    /// stack of segments, one per row.
    fn try_move_multi(&mut self, n: NodeId, xj: Dbu, sj: SegmentId) -> Result<bool> {
        let spanned = self.rows_spanned(n);
        if spanned <= 1 || spanned != self.cell_segs[n].len() {
            return Ok(false);
        }
        let mut rb = self.segments[sj].row;
        // NOTE: DetailedMgr::tryMove3 tests >=, so a cell never reaches the
        // top `spanned` rows; PARE lowers it only when it would stick out.
        while rb.index() + spanned > self.arch.rows.len() {
            let Some(below) = rb.down(1) else {
                return Ok(false);
            };
            rb = below;
        }
        let Some(segs) = self.segment_stack(n, rb, spanned, xj) else {
            return Ok(false);
        };
        let Some(gap) = self.stack_gap(n, &segs, xj) else {
            return Ok(false);
        };
        let Some(x) = self.align(n, xj, gap) else {
            return Ok(false);
        };
        let from_segs = self.cell_segs[n].clone();
        let from = self.position(n);
        let to = Point::new(x, self.arch.rows[rb].bottom);
        self.add_to_move_list(n, from, &from_segs, to, &segs, Reach::Free)
    }

    /// The segment of `n`'s region holding `xj` in each of the `spanned`
    /// rows from `bottom` up; none where a row has none.
    fn segment_stack(
        &self,
        n: NodeId,
        bottom: ArchRowId,
        spanned: usize,
        xj: Dbu,
    ) -> Option<Vec<SegmentId>> {
        let mut segs = Vec::with_capacity(spanned);
        for r in bottom.stack(spanned) {
            let found = self.segs_in_row[r].iter().copied().find(|&s| {
                let seg = &self.segments[s];
                seg.region == self.group(n) && xj >= seg.min_x && xj <= seg.max_x
            });
            let Some(s) = found else {
                break;
            };
            segs.push(s);
        }
        (segs.len() == spanned).then_some(segs)
    }

    /// The span free in every segment of `segs` around `xj` that `n` fits
    /// in; none where it fits in no such span.
    fn stack_gap(&self, n: NodeId, segs: &[SegmentId], xj: Dbu) -> Option<Interval> {
        let mut shared =
            Interval::new(Dbu::new(i64::from(i32::MIN)), Dbu::new(i64::from(i32::MAX)));
        for &s in segs {
            let (left, right) = self.neighbours_besides(n, s, xj);
            if left == Some(n) || right == Some(n) {
                return None;
            }
            let seg = &self.segments[s];
            let gap = Interval::new(
                left.map_or(seg.min_x, |l| self.right(l)),
                right.map_or(seg.max_x, |r| self.left(r)),
            );
            if self.width(n) > gap.len() {
                return None;
            }
            shared = shared.overlap(gap);
        }
        (self.width(n) <= shared.len()).then_some(shared)
    }

    /// The cells of segment `s` left and right of `xj`, the left one taken
    /// from before `n` where `n` is the cell just left of `xj`.
    fn neighbours_besides(
        &self,
        n: NodeId,
        s: SegmentId,
        xj: Dbu,
    ) -> (Option<NodeId>, Option<NodeId>) {
        let cells = &self.cells_in_seg[s];
        if cells.is_empty() {
            return (None, None);
        }
        let at = self.lower_bound(s, xj);
        if at == cells.len() {
            return (cells.last().copied(), None);
        }
        let right = Some(cells[at]);
        let mut left = at.checked_sub(1).map(|i| cells[i]);
        if left == Some(n) {
            left = (at > 1).then(|| cells[at - 2]);
        }
        (left, right)
    }

    /// `DetailedMgr::trySwap1`: swaps two single-row cells, each into the
    /// other's gap, or, when they are neighbours, shifts the pair.
    fn try_swap_single(
        &mut self,
        n: NodeId,
        si: SegmentId,
        xj: Dbu,
        sj: SegmentId,
    ) -> Result<bool> {
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
        let at_i = SwapSide {
            segment: si,
            index: self.index_in_segment(n, si)?,
        };
        let at_j = SwapSide {
            segment: sj,
            index: self.index_in_segment(ndj, sj)?,
        };
        let Some((xi, xj)) = self.swapped_lefts((n, at_i), (ndj, at_j), xj) else {
            return Ok(false);
        };
        let (from_i, from_j) = (self.position(n), self.position(ndj));
        if !self.add_to_move_list(
            n,
            from_i,
            &[si],
            Point::new(xj, from_j.y),
            &[sj],
            Reach::Limited,
        )? {
            return Ok(false);
        }
        if !self.add_to_move_list(
            ndj,
            from_j,
            &[sj],
            Point::new(xi, from_i.y),
            &[si],
            Reach::Limited,
        )? {
            self.paint(ndj)?;
            return Ok(false);
        }
        Ok(true)
    }

    /// Where cell `n` stands in segment `s`'s cells.
    fn index_in_segment(&self, n: NodeId, s: SegmentId) -> Result<usize> {
        self.cells_in_seg[s]
            .iter()
            .position(|&c| c == n)
            .ok_or(ImprovementError::Internal(
                "Cell not found in expected segment",
            ))
    }

    /// The span between the neighbours of the cell at `index` of segment
    /// `s`, or the segment's end where it has none.
    fn gap_around(&self, s: SegmentId, index: usize) -> Interval {
        let cells = &self.cells_in_seg[s];
        let seg = &self.segments[s];
        let lx = index
            .checked_sub(1)
            .and_then(|i| cells.get(i))
            .map_or(seg.min_x, |&p| self.right(p));
        let rx = cells.get(index + 1).map_or(seg.max_x, |&p| self.left(p));
        Interval::new(lx, rx)
    }

    /// The new lefts `(xi, xj)` of a swap of cell `n` with cell `ndj`:
    /// `xi` for `ndj` near `n`'s left and `xj` for `n` near `xj`, shifting
    /// the pair as one where they are neighbours; none where either has no
    /// room.
    fn swapped_lefts(
        &self,
        (n, at_i): (NodeId, SwapSide),
        (ndj, at_j): (NodeId, SwapSide),
        xj: Dbu,
    ) -> Option<(Dbu, Dbu)> {
        let xi = self.left(n);
        let row = self.segments[at_i.segment].row;
        let same_segment = at_i.segment == at_j.segment;
        if same_segment && at_i.index + 1 == at_j.index {
            let span = Interval::new(
                self.gap_around(at_i.segment, at_i.index).lo,
                self.gap_around(at_j.segment, at_j.index).hi,
            );
            return self
                .shift_pair([ndj, n], [xi, xj], span, row)
                .map(Into::into);
        }
        if same_segment && at_j.index + 1 == at_i.index {
            let span = Interval::new(
                self.gap_around(at_j.segment, at_j.index).lo,
                self.gap_around(at_i.segment, at_i.index).hi,
            );
            let [xj, xi] = self.shift_pair([n, ndj], [xj, xi], span, row)?;
            return Some((xi, xj));
        }
        let gap = self.gap_around(at_i.segment, at_i.index);
        if self.width(ndj) > gap.len() {
            return None;
        }
        let xi = self.align(ndj, xi, gap)?;
        let gap = self.gap_around(at_j.segment, at_j.index);
        if self.width(n) > gap.len() {
            return None;
        }
        let xj = self.align(n, xj, gap)?;
        Some((xi, xj))
    }

    /// The lefts of neighbours `cells`, left one first, shifted within
    /// `room` toward `targets`; none where they do not fit.
    fn shift_pair(
        &self,
        cells: [NodeId; 2],
        targets: [Dbu; 2],
        room: Interval,
        row: ArchRowId,
    ) -> Option<[Dbu; 2]> {
        let width = self.width(cells[0]) + self.width(cells[1]);
        if width > room.len() {
            return None;
        }
        let pos = self.shift(&cells, &targets, room.lo, room.hi, row)?;
        Some([pos[0], pos[1]])
    }

    /// Places `cells`, in order, on the sites of `[left_limit,
    /// right_limit)` nearest their targets by `OpenDP`'s dynamic program,
    /// whose first cell always takes the first site, as
    /// `DetailedMgr::shift`.
    fn shift(
        &self,
        cells: &[NodeId],
        targets: &[Dbu],
        left_limit: Dbu,
        right_limit: Dbu,
        row: ArchRowId,
    ) -> Option<Vec<Dbu>> {
        let sites = ShiftSites::within(&self.arch.rows[row], left_limit, right_limit)?;
        let widths: Vec<i64> = cells
            .iter()
            .map(|&c| self.width(c).div_trunc(sites.spacing))
            .collect();
        if widths.iter().sum::<i64>() > sites.last - sites.first + 1 {
            return None;
        }
        let starts = StartBounds::packed(&sites, &widths)?;
        let cost = displacement_costs(&sites, &starts, targets);
        let steps = cheapest_steps(&sites, &widths, &cost);
        trace_positions(&sites, &steps, cells.len())
    }

    /// Pushes `right` and the cells after it right of a cell placed at
    /// `xj`, each onto the first site at or after where it must start, as
    /// `DetailedMgr::shiftRightHelper`.
    pub(super) fn shift_right_helper(
        &mut self,
        mut ndi: NodeId,
        mut xj: Dbu,
        sj: SegmentId,
        mut ndr: NodeId,
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
            let mut sx = site_holding(xj, row.left, row.site_spacing);
            if xj != sx {
                if xj > sx {
                    sx += row.site_spacing;
                }
                if xj < sx {
                    xj = sx;
                }
            }
            let from = self.position(ndr);
            if !self.add_to_move_list(
                ndr,
                from,
                &[sj],
                Point::new(xj, from.y),
                &[sj],
                Reach::Limited,
            )? {
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
    /// each onto the last site at or before where it must end, as
    /// `DetailedMgr::shiftLeftHelper`.
    pub(super) fn shift_left_helper(
        &mut self,
        mut ndi: NodeId,
        mut xj: Dbu,
        sj: SegmentId,
        mut ndl: NodeId,
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
            let sx = site_holding(xj, row.left, row.site_spacing);
            if xj > sx {
                xj = sx;
            }
            let from = self.position(ndl);
            if !self.add_to_move_list(
                ndl,
                from,
                &[sj],
                Point::new(xj, from.y),
                &[sj],
                Reach::Limited,
            )? {
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
    pub(super) fn align(&self, n: NodeId, x: Dbu, room: Interval) -> Option<Dbu> {
        let row = &self.arch.rows[ArchRowId::BOTTOM];
        let xl = room.lo;
        let xr = room.hi - self.width(n);
        let mut xp = xl.max(xr.min(x));
        xp = row.left + row.site_spacing * (xp - row.left).div_trunc(row.site_spacing);
        if xp < xl {
            xp += row.site_spacing;
        } else if xp > xr {
            xp -= row.site_spacing;
        }
        (xp >= xl && xp <= xr).then_some(xp)
    }

    /// Moves a cell and journals the move, unless the trial has made too
    /// many, the spot has no row of its site or, under [`Reach::Limited`],
    /// it is too far from where the cell began, as
    /// `DetailedMgr::addToMoveList`.
    pub(super) fn add_to_move_list(
        &mut self,
        n: NodeId,
        from: Point<Dbu>,
        from_segs: &[SegmentId],
        to: Point<Dbu>,
        to_segs: &[SegmentId],
        reach: Reach,
    ) -> Result<bool> {
        if self.journal.moves.len() >= self.move_limit {
            return Ok(false);
        }
        let x = self.grid.rows.grid_x(to.x);
        let y = self.grid.rows.snap_down_y(to.y);
        if self
            .grid
            .site_orientation(x, y, self.grid.nodes[n].site)
            .is_none()
        {
            return Ok(false);
        }
        let orig = self.orig(n);
        if reach == Reach::Limited
            && ((to.x - orig.x).abs() > self.max_displacement.x
                || (to.y - orig.y).abs() > self.max_displacement.y)
        {
            return Ok(false);
        }
        self.erase(n);
        for &s in from_segs {
            self.remove_cell_from_segment(n, s)?;
        }
        self.set_position(n, to);
        self.paint(n)?;
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

    /// Opens a trial with an empty journal, as each `DetailedMgr::tryMove`
    /// and `trySwap` variant clears it first, journaling the moves that
    /// follow.
    pub(super) fn begin_trial(&mut self) -> Trial {
        debug_assert!(self.journal.moves.is_empty(), "trials do not nest");
        self.journal.clear();
        Trial(Sealed)
    }

    /// `trial` left open where it `moved` cells and none of them breaks a
    /// placement rule, else taken back, as `DetailedMgr::verifyMove`.
    pub(super) fn legal_trial(&mut self, trial: Trial, moved: bool) -> Result<Option<Trial>> {
        let is_legal = moved
            && !self
                .journal
                .nodes
                .iter()
                .any(|&n| self.has_placement_violation(n));
        if is_legal {
            return Ok(Some(trial));
        }
        self.reject_trial(trial)?;
        Ok(None)
    }

    /// Keeps the moves of `trial`, as `DetailedMgr::acceptMove`.
    pub(super) fn accept_trial(&mut self, trial: Trial) {
        let Trial(_sealed) = trial;
        self.journal.clear();
    }

    /// Takes the moves of `trial` back, last first, as
    /// `DetailedMgr::rejectMove`.
    pub(super) fn reject_trial(&mut self, trial: Trial) -> Result<()> {
        let Trial(_sealed) = trial;
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
            self.set_position(m.node, m.from);
            let x = self.grid.rows.grid_x(m.from.x);
            let y = self.grid.rows.round_y(m.from.y);
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

/// The start of the site holding `xj` in a row whose sites start at
/// `origin`, `spacing` apart, the offset divided toward zero as C++ does.
// NOTE: shift{Right,Left}Helper divide only originX by siteSpacing, a
// precedence slip that flings cells far off; PARE divides the offset.
fn site_holding(xj: Dbu, origin: Dbu, spacing: Dbu) -> Dbu {
    origin + spacing * (xj - origin).div_trunc(spacing)
}

/// The sites `first..=last` of a row a shift may use, numbered from the
/// row's origin, as `DetailedMgr::shift` finds them.
struct ShiftSites {
    origin: Dbu,
    spacing: Dbu,
    first: i64,
    last: i64,
    /// How many there are, the dynamic program's table rows less one.
    count: usize,
}

impl ShiftSites {
    /// The sites of `row` whole within `[left, right)`; none where there
    /// are none.
    fn within(row: &Row, left: Dbu, right: Dbu) -> Option<Self> {
        let (origin, spacing) = (row.left, row.site_spacing);
        let mut first = (left - origin).div_trunc(spacing);
        if origin + spacing * first < left {
            first += 1;
        }
        let mut last = (right - origin).div_trunc(spacing);
        if origin + spacing * last + row.site_width >= right {
            last -= 1;
        }
        Some(Self {
            origin,
            spacing,
            first,
            last,
            count: usize::try_from(last - first + 1).ok()?,
        })
    }

    /// The site of table row `i`, counted from one.
    fn site(&self, i: usize) -> i64 {
        self.first + i64::saturating_from(i) - 1
    }

    fn x(&self, site: i64) -> Dbu {
        self.origin + self.spacing * site
    }
}

/// The first and last site each cell may start at, the cells before it
/// packed to the left and those after it packed to the right.
struct StartBounds {
    left: Vec<i64>,
    right: Vec<i64>,
}

impl StartBounds {
    /// The bounds of cells `widths` sites wide; none where some cell has
    /// no room.
    fn packed(sites: &ShiftSites, widths: &[i64]) -> Option<Self> {
        let mut left = vec![0; widths.len()];
        let mut k = sites.first;
        for (i, &width) in widths.iter().enumerate() {
            left[i] = k;
            k += width;
        }
        let mut right = vec![0; widths.len()];
        let mut k = sites.last + 1;
        for i in (0..widths.len()).rev() {
            right[i] = k - widths[i];
            k = right[i];
            if right[i] < left[i] {
                return None;
            }
        }
        Some(Self { left, right })
    }
}

/// The cost of starting cell `j` at the site of table row `i`: its
/// distance from its target, zero where it cannot start there.
fn displacement_costs(sites: &ShiftSites, starts: &StartBounds, targets: &[Dbu]) -> Vec<Vec<f64>> {
    let cells = targets.len();
    let mut cost = vec![vec![0.0_f64; cells + 1]; sites.count + 1];
    for j in 1..=cells {
        for (i, row_cost) in cost.iter_mut().enumerate().skip(1) {
            let site_id = sites.site(i);
            if site_id < starts.left[j - 1] || site_id > starts.right[j - 1] {
                continue;
            }
            row_cost[j] = (sites.x(site_id) - targets[j - 1]).abs().to_f64();
        }
    }
    cost
}

/// The step into each table entry of the cheapest way to have placed the
/// first `j` cells within the first `i` sites: from `(i - 1, j)`, the site
/// left empty, or from the entry the cell's predecessor ends at.
fn cheapest_steps(
    sites: &ShiftSites,
    widths: &[i64],
    cost: &[Vec<f64>],
) -> Vec<Vec<Option<(usize, usize)>>> {
    let cells = widths.len();
    let mut tcost = vec![vec![f64::MAX; cells + 1]; sites.count + 1];
    let mut prev = vec![vec![None::<(usize, usize)>; cells + 1]; sites.count + 1];
    tcost[0][0] = 0.0;
    for j in 1..=cells {
        let prev_wid = if j == 1 { 1 } else { widths[j - 2] };
        let curr_wid = widths[j - 1];
        for i in 1..=sites.count {
            let site_id = sites.site(i);
            let c = tcost[i - 1][j];
            if c < tcost[i][j] {
                tcost[i][j] = c;
                prev[i][j] = Some((i - 1, j));
            }
            let ii = i64::saturating_from(i) - prev_wid;
            if let Ok(ii) = usize::try_from(ii)
                && site_id + curr_wid - 1 <= sites.last
            {
                let c = tcost[ii][j - 1] + cost[i][j];
                if c < tcost[i][j] {
                    tcost[i][j] = c;
                    prev[i][j] = Some((ii, j - 1));
                }
            }
        }
    }
    prev
}

/// Where each of the `cells` starts, followed back from the last table
/// entry; none where the steps never reach the first.
fn trace_positions(
    sites: &ShiftSites,
    prev: &[Vec<Option<(usize, usize)>>],
    cells: usize,
) -> Option<Vec<Dbu>> {
    let mut at = Some((sites.count, cells));
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
    let mut positions = vec![Dbu::ZERO; cells];
    let mut at = (sites.count, cells);
    while at != (0, 0) {
        let Some(back) = prev[at.0][at.1] else {
            break;
        };
        if at.1 != back.1 {
            positions[at.1 - 1] = sites.x(sites.site(at.0));
        }
        at = back;
    }
    Some(positions)
}

#[cfg(test)]
mod tests {
    use super::super::tests::{improver_of, tiny_edited_from_one_spot};
    use super::*;

    fn nodes(count: u32) -> Vec<NodeId> {
        (0..count).map(NodeId).collect()
    }

    #[test]
    fn a_position_inside_a_segment_lies_between_two_cells() {
        let cells = nodes(3);

        assert_eq!(around(&cells, 1), (Some(cells[0]), Some(cells[1])));
    }

    #[test]
    fn a_position_before_the_first_cell_has_nothing_left_of_it() {
        let cells = nodes(3);

        assert_eq!(around(&cells, 0), (None, Some(cells[0])));
    }

    #[test]
    fn a_position_past_the_last_cell_has_nothing_right_of_it() {
        let cells = nodes(3);

        assert_eq!(around(&cells, 3), (Some(cells[2]), None));
    }

    #[test]
    fn a_position_snaps_to_the_start_of_the_site_holding_it() {
        let (origin, spacing) = (Dbu::new(100), Dbu::new(10));

        let inside = site_holding(Dbu::new(135), origin, spacing);
        let on_edge = site_holding(Dbu::new(130), origin, spacing);

        assert_eq!((inside, on_edge), (Dbu::new(130), Dbu::new(130)));
    }

    #[test]
    fn an_empty_segment_has_no_neighbours() {
        assert_eq!(around(&[], 0), (None, None));
    }

    /// The second row starts a site in, at 0.2 um, and holds six cells
    /// side by side from 0.4 um: a NAND2 at 0.4, then cells at 1, 1.6, 2,
    /// 2.4 and 2.8 um.
    #[test]
    fn pushed_cells_land_on_the_sites_of_a_row_not_starting_at_zero() {
        let mut imp = improver_of(tiny_edited_from_one_spot(|text| {
            text.replace(
                "ROW ROW_1 core 0 2000 FS DO 30 BY 1 STEP 200 0 ;",
                "ROW ROW_1 core 200 2000 FS DO 29 BY 1 STEP 200 0 ;",
            )
        }));
        let s = imp.segs_in_row[ArchRowId::new(1)][0];
        let cells = imp.cells_in_seg[s].clone();

        let pushed = imp
            .shift_right_helper(cells[0], Dbu::new(500), s, cells[1])
            .unwrap();

        assert!(pushed);
        let lefts: Vec<Dbu> = cells[1..].iter().map(|&n| imp.left(n)).collect();
        assert_eq!(lefts, [1200, 1800, 2200, 2600, 3000].map(Dbu::new));
    }
}
