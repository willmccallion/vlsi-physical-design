//! Detailed placement: legality-preserving moves that reduce wirelength.
//!
//! Two passes alternate. Global swap moves each standard cell toward the
//! median of its nets' other pins, into a free gap or by exchanging places
//! with an equal-width cell. Local reordering tries every order of each three
//! consecutive cells in a row. A move is kept only if it shortens the
//! half-perimeter wirelength of the nets it touches. Nets with more than
//! [`MAX_NET_PINS`] pins are left out of the cost, as is usual for
//! high-fanout nets whose bounding box barely changes. Cells only move
//! within the stretches of row their zone may use, and never onto a spot
//! where fixed metal would block a via to one of their pins; a pass before
//! the first moves the cells legalization left on such spots.

use crate::rows::{
    RowSpan, classify, expand_rows, fixed_obstacles, free_intervals, orientation_allowed,
    snap_down, snap_up,
};
use crate::zones::{Zone, Zones, intersect};
use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{CellId, NetId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::pin_access::PinAccess;
use std::collections::HashMap;

/// Nets with more pins than this do not count toward move costs.
pub const MAX_NET_PINS: usize = 64;

/// Wirelength before and after detailed placement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DetailedResult {
    /// Weighted HPWL of the cost nets before, in DBU.
    pub hpwl_before: f64,
    /// Weighted HPWL of the cost nets after, in DBU.
    pub hpwl_after: f64,
    /// Number of moves kept.
    pub moves: usize,
}

/// What freeing blocked pins did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AccessRepair {
    /// Signal pins no via could reach before.
    pub blocked_before: usize,
    /// Signal pins no via can reach after.
    pub blocked_after: usize,
    /// Cells moved.
    pub moved: usize,
}

/// Sites a blocked cell may be shifted by along its row to free its pins,
/// and how far along a row it may move or swap to do so.
const MAX_SHIFT_SITES: i64 = 40;

/// Rows above and below its own a blocked cell may move into.
const FREE_ROWS: usize = 3;

/// Cells on either side a blocked cell may swap places with.
const SWAP_REACH: usize = 8;

/// Moves every cell with a pin that fixed metal blocks to a spot where none
/// of its pins is.
///
/// The spot is the nearest along the cell's row, pushing the cells beside
/// it by the least that keeps every pushed cell clear too; else the place
/// of a nearby cell of the same width that is clear where this one stands;
/// else a gap in a nearby row.
pub fn free_blocked_pins(db: &mut NetlistDB, zones: &Zones, access: &PinAccess) -> AccessRepair {
    let mut dp = DetailedPlacer::new(db, zones, Some(access));
    let blocked_before = dp.count_blocked(db);
    let mut moved = 0;
    for s in 0..dp.segments.len() {
        let mut i = 0;
        while i < dp.segments[s].cells.len() {
            let cell = dp.segments[s].cells[i];
            let at = db.cells[cell.index()].location;
            if dp.blocked_at(db, cell, at, dp.segments[s].row) == 0 {
                i += 1;
                continue;
            }
            match dp.free_cell(db, s, i) {
                Freed::InPlace => {
                    moved += 1;
                    i += 1;
                }
                Freed::Elsewhere => moved += 1,
                Freed::Not => i += 1,
            }
        }
    }
    let blocked_after = dp.count_blocked(db);
    if blocked_after > 0 && log::log_enabled!(log::Level::Debug) {
        for (&cell, &s) in &dp.segment_of {
            let at = db.cells[cell.index()].location;
            let blocked = dp.blocked_at(db, cell, at, dp.segments[s].row);
            if blocked > 0 {
                log::debug!(
                    "cell {} at ({}, {}) keeps {blocked} blocked pins",
                    db.cells[cell.index()].name,
                    at.x.get(),
                    at.y.get()
                );
            }
        }
    }
    AccessRepair {
        blocked_before,
        blocked_after,
        moved,
    }
}

/// How a blocked cell was freed, if it was.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Freed {
    /// Shifted within its segment, which keeps its place in the order.
    InPlace,
    /// Moved to another segment.
    Elsewhere,
    Not,
}

/// Runs `passes` rounds of global swap and local reordering on a legal
/// placement, never moving a cell onto a spot `access` says blocks a pin.
pub fn improve(
    db: &mut NetlistDB,
    zones: &Zones,
    passes: usize,
    access: Option<&PinAccess>,
) -> DetailedResult {
    let mut dp = DetailedPlacer::new(db, zones, access);
    let hpwl_before = dp.total_cost(db);
    let mut moves = 0;
    for _ in 0..passes {
        let kept = dp.global_swap(db) + dp.local_reorder(db);
        moves += kept;
        if kept == 0 {
            break;
        }
    }
    DetailedResult {
        hpwl_before,
        hpwl_after: dp.total_cost(db),
        moves,
    }
}

/// A free stretch of a row in one zone and the cells in it, ordered by X.
struct Segment {
    row: usize,
    zone: Zone,
    lo: Dbu,
    hi: Dbu,
    cells: Vec<CellId>,
}

struct DetailedPlacer<'a> {
    rows: Vec<RowSpan>,
    segments: Vec<Segment>,
    /// For each row, the half-open range of its segments.
    row_segments: Vec<(usize, usize)>,
    /// For each movable standard cell, its segment.
    segment_of: HashMap<CellId, usize>,
    /// The zone of every cell.
    zone_of: Vec<Zone>,
    /// For each cell, the cost nets it is on.
    nets_of: Vec<Vec<NetId>>,
    /// Which spots block a pin, when the design has pins to check.
    access: Option<&'a PinAccess>,
}

impl<'a> DetailedPlacer<'a> {
    fn new(db: &NetlistDB, zones: &Zones, access: Option<&'a PinAccess>) -> Self {
        let rows = expand_rows(db);
        let (macros, std_cells) = classify(db, &rows);
        let mut obstacles = fixed_obstacles(db);
        obstacles.extend(macros.iter().map(|&c| db.cell_bbox(c)));

        let mut segments = Vec::new();
        let mut row_segments = Vec::with_capacity(rows.len());
        for (r, row) in rows.iter().enumerate() {
            let start = segments.len();
            let lo = row.x_min.max(db.die_area.min.x);
            let hi = row.x_max.min(db.die_area.max.x);
            let (y0, y1) = (row.y, row.y + row.height);
            let free = free_intervals(&obstacles, lo, hi, y0, y1);
            for zone in zones.all() {
                for (a, b) in intersect(&free, &zones.intervals(zone, lo, hi, y0, y1)) {
                    let (a, b) = (
                        snap_up(a, row.x_min, row.site),
                        snap_down(b, row.x_min, row.site),
                    );
                    if b > a {
                        segments.push(Segment {
                            row: r,
                            zone,
                            lo: a,
                            hi: b,
                            cells: Vec::new(),
                        });
                    }
                }
            }
            row_segments.push((start, segments.len()));
        }

        let mut segment_of = HashMap::new();
        for cell in std_cells {
            let at = db.cells[cell.index()].location;
            let width = db.cell_size(cell).x;
            let r = rows.partition_point(|row| row.y < at.y);
            let zone = zones.of(cell);
            let found = (r..rows.len())
                .take_while(|&r| rows[r].y == at.y)
                .flat_map(|r| row_segments[r].0..row_segments[r].1)
                .find(|&s| {
                    let seg = &segments[s];
                    seg.zone == zone && seg.lo <= at.x && at.x + width <= seg.hi
                });
            if let Some(s) = found {
                segments[s].cells.push(cell);
                let _ = segment_of.insert(cell, s);
            }
        }
        for seg in &mut segments {
            seg.cells.sort_by_key(|&c| db.cells[c.index()].location.x);
        }

        let mut nets_of = vec![Vec::new(); db.num_cells()];
        for (i, net) in db.nets.iter().enumerate() {
            if net.signal_use.is_supply() || net.pins.len() < 2 || net.pins.len() > MAX_NET_PINS {
                continue;
            }
            for &pin in &net.pins {
                if let PinOwner::Cell { cell, .. } = db.pins[pin.index()].owner {
                    let list: &mut Vec<NetId> = &mut nets_of[cell.index()];
                    if list.last() != Some(&NetId::new(i)) {
                        list.push(NetId::new(i));
                    }
                }
            }
        }

        Self {
            rows,
            segments,
            row_segments,
            segment_of,
            zone_of: (0..db.num_cells())
                .map(|i| zones.of(CellId::new(i)))
                .collect(),
            nets_of,
            access,
        }
    }

    /// Signal pins of `cell` that fixed metal would block at `at` in `row`.
    fn blocked_at(&self, db: &NetlistDB, cell: CellId, at: Point<Dbu>, row: usize) -> usize {
        self.access
            .map_or(0, |a| a.blocked_pins(db, cell, at, self.rows[row].orient))
    }

    /// Blocked signal pins over every movable standard cell.
    fn count_blocked(&self, db: &NetlistDB) -> usize {
        self.segment_of
            .iter()
            .map(|(&cell, &s)| {
                self.blocked_at(
                    db,
                    cell,
                    db.cells[cell.index()].location,
                    self.segments[s].row,
                )
            })
            .sum()
    }

    /// Frees the pins of cell `i` of segment `s`: by the shortest shift
    /// along the segment that leaves it and every cell it pushes clear, else
    /// by swapping with a nearby cell of its width that is clear where it
    /// stands, else by moving into a gap of a nearby row.
    fn free_cell(&mut self, db: &mut NetlistDB, s: usize, i: usize) -> Freed {
        let cell = self.segments[s].cells[i];
        let row = self.segments[s].row;
        let site = self.rows[row].site;
        let x = db.cells[cell.index()].location.x;
        for k in 1..=MAX_SHIFT_SITES {
            for dir in [-1, 1] {
                if let Some(moves) = self.shift_plan(db, s, i, x + site * (k * dir)) {
                    log::trace!(
                        "cell {} shifted {} sites with {} pushed",
                        cell.index(),
                        k * dir,
                        moves.len() - 1
                    );
                    for (c, at) in moves {
                        db.cells[c.index()].location = at;
                    }
                    return Freed::InPlace;
                }
            }
        }
        let size = db.cell_size(cell);
        let zone = self.zone_of[cell.index()];
        let reach = site * MAX_SHIFT_SITES;
        let rows_near: Vec<usize> = (0..=FREE_ROWS)
            .flat_map(|d| [row.checked_sub(d), Some(row + d)])
            .flatten()
            .filter(|&r| {
                r < self.rows.len()
                    && self.rows[r].height == size.y
                    && orientation_allowed(db, cell, self.rows[r].orient)
            })
            .collect();
        for &other in &rows_near {
            for t in self.row_segments[other].0..self.row_segments[other].1 {
                if self.segments[t].zone != zone {
                    continue;
                }
                if let Some(partner) = self.swap_partner(db, cell, s, t, x, reach) {
                    let here = db.cells[cell.index()].location;
                    let there = db.cells[partner.index()].location;
                    log::trace!(
                        "cell {} at ({}, {}) swaps with {} at ({}, {})",
                        cell.index(),
                        here.x.get(),
                        here.y.get(),
                        partner.index(),
                        there.x.get(),
                        there.y.get()
                    );
                    db.cells[cell.index()].location = there;
                    db.cells[cell.index()].orient = self.rows[other].orient;
                    db.cells[partner.index()].location = here;
                    db.cells[partner.index()].orient = self.rows[row].orient;
                    self.swap_entries(db, cell, s, partner, t);
                    return Freed::Elsewhere;
                }
            }
        }
        for &other in &rows_near {
            for t in self.row_segments[other].0..self.row_segments[other].1 {
                if self.segments[t].zone != zone {
                    continue;
                }
                let Some(gx) = self.gap_near(db, t, x, size.x) else {
                    continue;
                };
                if (gx - x).abs() > reach {
                    continue;
                }
                let at = Point::new(gx, self.rows[other].y);
                if self.blocked_at(db, cell, at, other) == 0 {
                    let from = db.cells[cell.index()].location;
                    log::trace!(
                        "cell {} moves from ({}, {}) to a gap at ({}, {})",
                        cell.index(),
                        from.x.get(),
                        from.y.get(),
                        at.x.get(),
                        at.y.get()
                    );
                    db.cells[cell.index()].location = at;
                    db.cells[cell.index()].orient = self.rows[other].orient;
                    self.relocate(db, cell, s, t);
                    return Freed::Elsewhere;
                }
            }
        }
        Freed::Not
    }

    /// A cell of segment `t` within `reach` of `x`, as wide as `cell` and of
    /// its zone, that is clear where `cell` stands while `cell` is clear
    /// where it stands, so that swapping them frees `cell` without blocking
    /// it.
    fn swap_partner(
        &self,
        db: &NetlistDB,
        cell: CellId,
        s: usize,
        t: usize,
        x: Dbu,
        reach: Dbu,
    ) -> Option<CellId> {
        let width = db.cell_size(cell).x;
        let here = db.cells[cell.index()].location;
        let (row_here, row_there) = (self.segments[s].row, self.segments[t].row);
        let cells = &self.segments[t].cells;
        let k = cells.partition_point(|&c| db.cells[c.index()].location.x < x);
        cells[k.saturating_sub(SWAP_REACH)..(k + SWAP_REACH).min(cells.len())]
            .iter()
            .copied()
            .filter(|&c| c != cell && db.cell_size(c).x == width)
            .filter(|&c| (db.cells[c.index()].location.x - x).abs() <= reach)
            .filter(|&c| self.zone_of[c.index()] == self.zone_of[cell.index()])
            .filter(|&c| orientation_allowed(db, c, self.rows[row_here].orient))
            .find(|&c| {
                let there = db.cells[c.index()].location;
                self.blocked_at(db, cell, there, row_there) == 0
                    && self.blocked_at(db, c, here, row_here) == 0
            })
    }

    /// The moves that put cell `i` of segment `s` at `nx` and push the
    /// cells it would overlap out of the way, or `None` when a pushed cell
    /// would leave the segment or land where a pin is blocked.
    fn shift_plan(
        &self,
        db: &NetlistDB,
        s: usize,
        i: usize,
        nx: Dbu,
    ) -> Option<Vec<(CellId, Point<Dbu>)>> {
        let seg = &self.segments[s];
        let row = &self.rows[seg.row];
        let cell = seg.cells[i];
        let width = db.cell_size(cell).x;
        if nx < seg.lo
            || nx + width > seg.hi
            || self.blocked_at(db, cell, Point::new(nx, row.y), seg.row) > 0
        {
            return None;
        }
        let mut moves = vec![(cell, Point::new(nx, row.y))];
        let mut bound = nx;
        for &c in seg.cells[..i].iter().rev() {
            let bbox = db.cell_bbox(c);
            if bbox.max.x <= bound {
                break;
            }
            let at = snap_down(bound - bbox.width(), row.x_min, row.site);
            if at < seg.lo || self.blocked_at(db, c, Point::new(at, row.y), seg.row) > 0 {
                return None;
            }
            moves.push((c, Point::new(at, row.y)));
            bound = at;
        }
        let mut bound = nx + width;
        for &c in &seg.cells[i + 1..] {
            let bbox = db.cell_bbox(c);
            if bbox.min.x >= bound {
                break;
            }
            let at = snap_up(bound, row.x_min, row.site);
            if at + bbox.width() > seg.hi
                || self.blocked_at(db, c, Point::new(at, row.y), seg.row) > 0
            {
                return None;
            }
            moves.push((c, Point::new(at, row.y)));
            bound = at + bbox.width();
        }
        Some(moves)
    }

    fn total_cost(&self, db: &NetlistDB) -> f64 {
        let mut seen = vec![false; db.nets.len()];
        let mut total = 0.0;
        for nets in &self.nets_of {
            for &n in nets {
                if !std::mem::replace(&mut seen[n.index()], true) {
                    total += net_cost(db, n);
                }
            }
        }
        total
    }

    fn cost_of(db: &NetlistDB, nets: &[NetId]) -> f64 {
        nets.iter().map(|&n| net_cost(db, n)).sum()
    }

    fn affected(&self, cells: &[CellId]) -> Vec<NetId> {
        let mut nets: Vec<NetId> = cells
            .iter()
            .flat_map(|c| self.nets_of[c.index()].iter().copied())
            .collect();
        nets.sort_unstable();
        nets.dedup();
        nets
    }

    /// Applies `placements`, keeping them if they lower the cost of the nets
    /// on `cells` and block no pin; returns whether they were kept.
    fn try_move(
        &self,
        db: &mut NetlistDB,
        cells: &[CellId],
        placements: &[(CellId, Point<Dbu>, usize)],
    ) -> bool {
        if placements
            .iter()
            .any(|&(c, at, row)| self.blocked_at(db, c, at, row) > 0)
        {
            return false;
        }
        let nets = self.affected(cells);
        let before = Self::cost_of(db, &nets);
        let saved: Vec<_> = placements
            .iter()
            .map(|&(c, _, _)| (c, db.cells[c.index()].location, db.cells[c.index()].orient))
            .collect();
        for &(c, at, row) in placements {
            db.cells[c.index()].location = at;
            db.cells[c.index()].orient = self.rows[row].orient;
        }
        if Self::cost_of(db, &nets) < before - 1e-9 {
            return true;
        }
        for (c, at, orient) in saved {
            db.cells[c.index()].location = at;
            db.cells[c.index()].orient = orient;
        }
        false
    }

    /// The box spanned by the medians of each net's other-pin bounding box.
    fn optimal_region(&self, db: &NetlistDB, cell: CellId) -> Option<Rect<Dbu>> {
        let mut xs = Vec::new();
        let mut ys = Vec::new();
        for &n in &self.nets_of[cell.index()] {
            let others = db.nets[n.index()].pins.iter().filter(|&&p| {
                !matches!(db.pins[p.index()].owner, PinOwner::Cell { cell: c, .. } if c == cell)
            });
            let Some(bbox) = others
                .map(|&p| {
                    let at = db.pin_position(p);
                    Rect::new(at, at)
                })
                .reduce(|a, b| a.union(&b))
            else {
                continue;
            };
            xs.extend([bbox.min.x, bbox.max.x]);
            ys.extend([bbox.min.y, bbox.max.y]);
        }
        if xs.is_empty() {
            return None;
        }
        xs.sort_unstable();
        ys.sort_unstable();
        let mid = xs.len() / 2;
        Some(Rect::new(
            Point::new(xs[mid - 1], ys[mid - 1]),
            Point::new(xs[mid], ys[mid]),
        ))
    }

    fn global_swap(&mut self, db: &mut NetlistDB) -> usize {
        let mut cells: Vec<CellId> = self.segment_of.keys().copied().collect();
        cells.sort_unstable();
        let mut kept = 0;
        for cell in cells {
            if self.global_move(db, cell) {
                kept += 1;
            }
        }
        kept
    }

    fn global_move(&mut self, db: &mut NetlistDB, cell: CellId) -> bool {
        let Some(region) = self.optimal_region(db, cell) else {
            return false;
        };
        let size = db.cell_size(cell);
        let here = db.cells[cell.index()].location;
        if region.contains(here + Point::new(size.x / 2, size.y / 2)) {
            return false;
        }
        let target = region.center() - Point::new(size.x / 2, size.y / 2);
        let r = self
            .rows
            .partition_point(|row| row.y + row.height <= target.y)
            .min(self.rows.len() - 1);
        let Some(&from) = self.segment_of.get(&cell) else {
            return false;
        };

        for row in [r, r.saturating_sub(1), (r + 1).min(self.rows.len() - 1)] {
            if self.rows[row].height != size.y
                || !orientation_allowed(db, cell, self.rows[row].orient)
            {
                continue;
            }
            for s in self.row_segments[row].0..self.row_segments[row].1 {
                let seg = &self.segments[s];
                if seg.zone != self.zone_of[cell.index()]
                    || seg.hi < target.x - size.x * 4
                    || seg.lo > target.x + size.x * 4
                {
                    continue;
                }
                if let Some(x) = self.gap_near(db, s, target.x, size.x) {
                    let at = Point::new(x, self.rows[row].y);
                    if self.try_move(db, &[cell], &[(cell, at, row)]) {
                        self.relocate(db, cell, from, s);
                        return true;
                    }
                }
                if let Some(other) = self.equal_width_near(db, s, target.x, size.x, cell) {
                    let other_at = db.cells[other.index()].location;
                    let other_row = self.segments[from].row;
                    if self.zone_of[other.index()] != self.segments[from].zone
                        || !orientation_allowed(db, other, self.rows[other_row].orient)
                    {
                        continue;
                    }
                    if self.try_move(
                        db,
                        &[cell, other],
                        &[(cell, other_at, row), (other, here, other_row)],
                    ) {
                        self.swap_entries(db, cell, from, other, s);
                        return true;
                    }
                }
            }
        }
        false
    }

    /// A site-aligned X for a cell of `width` in a free gap of segment `s`,
    /// as close as possible to `target`, looking only at the gaps between
    /// the few cells nearest `target`.
    fn gap_near(&self, db: &NetlistDB, s: usize, target: Dbu, width: Dbu) -> Option<Dbu> {
        const REACH: usize = 8;
        let seg = &self.segments[s];
        let row = &self.rows[seg.row];
        let k = seg
            .cells
            .partition_point(|&c| db.cells[c.index()].location.x < target);
        let first = k.saturating_sub(REACH);
        let last = (k + REACH).min(seg.cells.len());
        let mut best: Option<Dbu> = None;
        for j in first..=last {
            let gap_lo = if j == 0 {
                seg.lo
            } else {
                db.cell_bbox(seg.cells[j - 1]).max.x
            };
            let gap_hi = if j == seg.cells.len() {
                seg.hi
            } else {
                db.cells[seg.cells[j].index()].location.x
            };
            if gap_hi - gap_lo < width {
                continue;
            }
            let x =
                snap_down(target.clamp(gap_lo, gap_hi - width), row.x_min, row.site).max(gap_lo);
            if best.is_none_or(|b| (x - target).abs() < (b - target).abs()) {
                best = Some(x);
            }
        }
        best
    }

    /// An equal-width cell in segment `s` near `target`, other than `cell`.
    fn equal_width_near(
        &self,
        db: &NetlistDB,
        s: usize,
        target: Dbu,
        width: Dbu,
        cell: CellId,
    ) -> Option<CellId> {
        let seg = &self.segments[s];
        let k = seg
            .cells
            .partition_point(|&c| db.cells[c.index()].location.x < target);
        seg.cells[k.saturating_sub(2)..(k + 2).min(seg.cells.len())]
            .iter()
            .copied()
            .filter(|&c| c != cell && db.cell_size(c).x == width)
            .min_by_key(|&c| (db.cells[c.index()].location.x - target).abs())
    }

    fn relocate(&mut self, db: &NetlistDB, cell: CellId, from: usize, to: usize) {
        self.segments[from].cells.retain(|&c| c != cell);
        let x = db.cells[cell.index()].location.x;
        let cells = &mut self.segments[to].cells;
        let k = cells.partition_point(|&c| db.cells[c.index()].location.x < x);
        cells.insert(k, cell);
        let _ = self.segment_of.insert(cell, to);
    }

    fn swap_entries(&mut self, db: &NetlistDB, a: CellId, seg_a: usize, b: CellId, seg_b: usize) {
        for s in [seg_a, seg_b] {
            for c in &mut self.segments[s].cells {
                if *c == a {
                    *c = b;
                } else if *c == b {
                    *c = a;
                }
            }
            if seg_a == seg_b {
                break;
            }
        }
        let _ = (
            self.segment_of.insert(a, seg_b),
            self.segment_of.insert(b, seg_a),
        );
        for s in [seg_a, seg_b] {
            self.segments[s]
                .cells
                .sort_by_key(|&c| db.cells[c.index()].location.x);
        }
    }

    fn local_reorder(&mut self, db: &mut NetlistDB) -> usize {
        const ORDERS: [[usize; 3]; 5] = [[0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
        let mut kept = 0;
        for s in 0..self.segments.len() {
            let row = self.segments[s].row;
            let mut i = 0;
            while i + 3 <= self.segments[s].cells.len() {
                let window: [CellId; 3] = [
                    self.segments[s].cells[i],
                    self.segments[s].cells[i + 1],
                    self.segments[s].cells[i + 2],
                ];
                let left = db.cells[window[0].index()].location.x;
                let y = db.cells[window[0].index()].location.y;
                for order in ORDERS {
                    let mut x = left;
                    let placements: Vec<_> = order
                        .iter()
                        .map(|&k| {
                            let c = window[k];
                            let at = Point::new(x, y);
                            x += db.cell_size(c).x;
                            (c, at, row)
                        })
                        .collect();
                    if self.try_move(db, &window, &placements) {
                        for (slot, &(c, _, _)) in placements.iter().enumerate() {
                            self.segments[s].cells[i + slot] = c;
                        }
                        kept += 1;
                        break;
                    }
                }
                i += 1;
            }
        }
        kept
    }
}

/// Weighted half-perimeter wirelength of one net.
fn net_cost(db: &NetlistDB, net: NetId) -> f64 {
    let n = &db.nets[net.index()];
    let mut pins = n.pins.iter().map(|&p| db.pin_position(p));
    let Some(first) = pins.next() else { return 0.0 };
    let bbox = pins.fold(Rect::new(first, first), |b, p| b.union(&Rect::new(p, p)));
    n.weight * (bbox.width() + bbox.height()).to_f64()
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::design::{PlacementStatus, Region, RegionKind, Row};
    use pare_common::db::indices::{LayerId, RegionId, SiteId};
    use pare_common::db::library::{
        CoreKind, Direction, LayerRef, Library, Macro, MacroClass, MacroPin, PinDirection,
        RoutingLayer, Shape, SignalUse, Site, SiteClass, Symmetry,
    };
    use pare_common::db::rules::RoutingLayerRules;
    use pare_common::geom::dbu::DbuPerMicron;
    use pare_common::geom::orient::Orientation;
    use pare_common::verify::placement::placement_violations;

    const fn pt(x: i64, y: i64) -> Point<Dbu> {
        Point::new(Dbu(x), Dbu(y))
    }

    fn improve_zoned(db: &mut NetlistDB) -> DetailedResult {
        let zones = Zones::new(db).unwrap();
        improve(db, &zones, 2, None)
    }

    /// One 20-site row over a 200 x 100 die, with 20-wide one-pin cells.
    fn design(cells: &[(i64, PlacementStatus)], nets: &[(usize, usize)]) -> NetlistDB {
        let mut lib = Library::new(DbuPerMicron::new(1000).unwrap());
        let symmetry = Symmetry {
            x: true,
            y: true,
            r90: false,
        };
        let _ = lib
            .add_routing_layer(RoutingLayer {
                name: "M1".into(),
                direction: Direction::Horizontal,
                pitch_x: Dbu(10),
                pitch_y: Dbu(10),
                offset_x: None,
                offset_y: None,
                width: Dbu(4),
                min_spacing: Dbu(4),
                spacing_table: None,
                rules: RoutingLayerRules::default(),
            })
            .unwrap();
        let site = lib
            .add_site(Site {
                name: "s".into(),
                class: SiteClass::Core,
                size: pt(10, 100),
                symmetry,
            })
            .unwrap();
        let mut m = Macro::new(
            "C".into(),
            MacroClass::Core(CoreKind::Regular),
            pt(20, 100),
            symmetry,
            Some(site),
        );
        let _ = m.add_pin(MacroPin {
            name: "A".into(),
            direction: PinDirection::Inout,
            signal_use: SignalUse::Signal,
            shapes: vec![Shape {
                layer: LayerRef::Routing(LayerId(0)),
                rect: Rect::new(pt(8, 48), pt(12, 52)),
            }],
        });
        let m = lib.add_macro(m).unwrap();
        let mut db = NetlistDB::new("t".into(), lib);
        db.die_area = Rect::new(pt(0, 0), pt(200, 100));
        db.rows.push(Row {
            name: "r".into(),
            site: SiteId(0),
            origin: pt(0, 0),
            orient: Orientation::N,
            num_x: 20,
            num_y: 1,
            step: pt(10, 0),
        });
        for (i, &(x, status)) in cells.iter().enumerate() {
            let _ = db
                .add_cell(format!("c{i}"), m, pt(x, 0), Orientation::N, status)
                .unwrap();
        }
        let pin = pare_common::db::indices::MacroPinId(0);
        for (k, &(a, b)) in nets.iter().enumerate() {
            let net = db.net_or_insert(&format!("n{k}"));
            let _ = db.connect_cell_pin(CellId::new(a), pin, net);
            let _ = db.connect_cell_pin(CellId::new(b), pin, net);
        }
        db
    }

    #[test]
    fn crossed_cells_are_uncrossed() {
        use PlacementStatus::{Fixed, Placed};
        let mut db = design(
            &[(0, Fixed), (180, Fixed), (40, Placed), (60, Placed)],
            &[(2, 1), (3, 0)],
        );

        let result = improve_zoned(&mut db);

        assert!(result.hpwl_after < result.hpwl_before, "{result:?}");
        assert!(db.cells[2].location.x > db.cells[3].location.x);
        assert!(
            placement_violations(&db).is_empty(),
            "{:?}",
            placement_violations(&db)
        );
    }

    #[test]
    fn a_placement_with_nothing_to_gain_is_unchanged() {
        use PlacementStatus::{Fixed, Placed};
        let mut db = design(
            &[(0, Fixed), (180, Fixed), (20, Placed), (160, Placed)],
            &[(2, 0), (3, 1)],
        );
        let before: Vec<_> = db.cells.iter().map(|c| c.location).collect();

        let result = improve_zoned(&mut db);

        assert_eq!(result.moves, 0);
        assert_eq!(
            db.cells.iter().map(|c| c.location).collect::<Vec<_>>(),
            before
        );
    }

    #[test]
    fn a_fence_member_moves_toward_its_net_only_up_to_the_fence_edge() {
        use PlacementStatus::{Fixed, Placed};
        let mut db = design(&[(0, Fixed), (180, Fixed), (40, Placed)], &[(2, 1)]);
        db.regions.push(Region {
            name: "f".into(),
            rects: vec![Rect::new(pt(0, 0), pt(100, 100))],
            kind: RegionKind::Fence,
        });
        db.cells[2].region = Some(RegionId(0));

        let result = improve_zoned(&mut db);

        let x = db.cells[2].location.x;
        assert!(result.moves > 0, "{result:?}");
        assert!(Dbu(40) < x && x <= Dbu(80), "{x:?}");
        assert!(
            placement_violations(&db).is_empty(),
            "{:?}",
            placement_violations(&db)
        );
    }
}
