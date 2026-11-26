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
    RowSpan, classify, expand_rows, fixed_obstacles, free_intervals, snap_down, snap_up,
};
use crate::zones::{Zone, Zones};
use pare_common::db::design::{NetlistDB, PinOwner};
use pare_common::db::indices::{CellId, IdVec, LayerId, MacroId, NetId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::{Interval, intersect_sorted};
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_verify::pin_access::{Neighbour, PinAccess};
use std::collections::{HashMap, HashSet};
use std::sync::{Mutex, PoisonError};

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

/// Moves every cell with a pin that fixed metal or a neighbouring cell's
/// metal blocks to a spot where none of its pins is.
///
/// Each row stretch holding such a cell is first re-placed in order by the
/// least total displacement, within the shift reach of every cell, that
/// leaves no pin blocked. A cell that stays blocked is then shifted to the
/// nearest clear spot, pushing the cells beside it by the least that keeps
/// every pushed cell clear too; else it takes the place of a nearby cell of
/// the same width that is clear where this one stands; else a gap in a
/// nearby row.
pub fn free_blocked_pins(db: &mut NetlistDB, zones: &Zones, access: &PinAccess) -> AccessRepair {
    let mut dp = DetailedPlacer::new(db, zones, Some(access));
    let blocked_before = dp.count_blocked(db);
    let mut moved = 0;
    for s in dp.segments.ids() {
        let any_blocked = dp.segments[s]
            .cells
            .iter()
            .any(|&c| dp.blocked_now(db, c, s) > 0);
        if any_blocked {
            moved += dp.free_segment(db, s).unwrap_or(0);
        }
    }
    for s in dp.segments.ids() {
        let mut i = 0;
        while i < dp.segments[s].cells.len() {
            let cell = dp.segments[s].cells[i];
            if dp.blocked_now(db, cell, s) == 0 {
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
            let at = db.cells[cell].location;
            let blocked = dp.blocked_now(db, cell, s);
            if blocked > 0 {
                let beside: Vec<&str> = dp
                    .beside(db, cell, at, dp.segments[s].row, &[], Absent::NONE)
                    .iter()
                    .map(|n| db.cells[n.cell].name.as_str())
                    .collect();
                let alone = dp
                    .access
                    .map_or(0, |a| a.blocked_pins(db, cell, at, db.cells[cell].orient));
                log::debug!(
                    "cell {} at ({}, {}) keeps {blocked} blocked pins ({alone} by fixed metal alone) beside {beside:?}",
                    db.cells[cell].name,
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

pare_common::define_index!(
    /// A row of the detailed placer, from the bottom up.
    RowId(u32)
);

pare_common::define_index!(
    /// A free stretch of a row the detailed placer places cells in.
    SegmentId(u32)
);

/// The segments of one row: consecutive, from `start` up to `end`.
#[derive(Clone, Copy, Debug)]
struct SegmentSpan {
    start: SegmentId,
    end: SegmentId,
}

impl SegmentSpan {
    fn ids(self) -> impl Iterator<Item = SegmentId> {
        (self.start.index()..self.end.index()).map(SegmentId::new)
    }
}

/// A free stretch of a row in one zone and the cells in it, ordered by X.
struct Segment {
    row: RowId,
    zone: Zone,
    lo: Dbu,
    hi: Dbu,
    cells: Vec<CellId>,
}

struct DetailedPlacer<'a> {
    rows: IdVec<RowId, RowSpan>,
    segments: IdVec<SegmentId, Segment>,
    /// For each row, its segments.
    row_segments: IdVec<RowId, SegmentSpan>,
    /// For each movable standard cell, its segment.
    segment_of: HashMap<CellId, SegmentId>,
    /// The zone of every cell.
    zone_of: IdVec<CellId, Zone>,
    /// For each cell, the cost nets it is on.
    nets_of: IdVec<CellId, Vec<NetId>>,
    /// Which spots block a pin, when the design has pins to check.
    access: Option<&'a PinAccess>,
    /// The widest movable standard cell.
    max_width: Dbu,
    /// The tallest row.
    max_row_height: Dbu,
    /// The metal each cell's pin access relies on where it stood when last
    /// asked, for cells whose pins were all free there; dropped when the
    /// cell moves.
    reliance: Mutex<HashMap<CellId, Relied>>,
}

/// The metal a cell's pin access relies on, by layer.
type Reliance = Vec<(LayerId, Rect<Dbu>)>;

/// Cells a pin check takes as standing nowhere.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Absent {
    /// Every cell of this segment, which is being re-placed as a whole.
    segment: Option<SegmentId>,
    /// This cell.
    cell: Option<CellId>,
}

impl Absent {
    const NONE: Self = Self {
        segment: None,
        cell: None,
    };

    const fn segment(s: SegmentId) -> Self {
        Self {
            segment: Some(s),
            cell: None,
        }
    }

    /// These cells and `cell` too, which must not already name a cell.
    const fn and(self, cell: CellId) -> Self {
        Self {
            cell: Some(cell),
            ..self
        }
    }
}

/// What a cell's pin access relied on, and where the cell stood then.
#[derive(Clone, Debug)]
struct Relied {
    at: Point<Dbu>,
    orient: Orientation,
    metal: Reliance,
}

/// A cell standing at a point in a row, for checks of placements not yet
/// made.
#[derive(Clone, Copy, Debug)]
struct Placement {
    cell: CellId,
    at: Point<Dbu>,
    row: RowId,
}

/// Whether two cells of a row can stand at a given gap without either
/// blocking a pin of the other, remembered per pair of masters and gap
/// once it is known to hold, since nets can only exempt more.
#[derive(Default)]
struct PairCheck {
    clear: HashSet<(MacroId, MacroId, Dbu)>,
}

impl PairCheck {
    /// Whether `left` and `right`, each a cell and the X of its lower-left
    /// corner in `row`, leave each other's pins free.
    fn clear(
        &mut self,
        dp: &DetailedPlacer<'_>,
        db: &NetlistDB,
        row: RowId,
        left: (CellId, Dbu),
        right: (CellId, Dbu),
    ) -> bool {
        let Some(access) = dp.access else {
            return true;
        };
        let gap = right.1 - (left.1 + db.cell_size(left.0).x);
        let key = (db.cells[left.0].macro_id, db.cells[right.0].macro_id, gap);
        if self.clear.contains(&key) {
            return true;
        }
        let y = dp.rows[row].y;
        let orient = dp.rows[row].orient;
        let stand = |(cell, x): (CellId, Dbu)| Neighbour {
            cell,
            at: Point::new(x, y),
            orient,
        };
        let (a, b) = (stand(left), stand(right));
        let clear = access.blocked_pins_among(db, a.cell, a.at, orient, &[b]) == 0
            && access.blocked_pins_among(db, b.cell, b.at, orient, &[a]) == 0;
        if clear {
            let _ = self.clear.insert(key);
        }
        clear
    }
}

impl<'a> DetailedPlacer<'a> {
    fn new(db: &NetlistDB, zones: &Zones, access: Option<&'a PinAccess>) -> Self {
        let rows: IdVec<RowId, RowSpan> = expand_rows(db).into();
        let (macros, std_cells) = classify(db, rows.iter().as_slice());
        let mut obstacles = fixed_obstacles(db);
        obstacles.extend(macros.iter().map(|&c| db.cell_bbox(c)));
        let mut cut = cut_segments(db, zones, &rows, &obstacles);
        let segment_of = assign_segments(db, zones, &rows, &mut cut, &std_cells);
        let max_width = std_cells
            .iter()
            .map(|&c| db.cell_size(c).x)
            .max()
            .unwrap_or(Dbu::ZERO);
        let max_row_height = rows.iter().map(|r| r.height).max().unwrap_or(Dbu::ZERO);
        Self {
            rows,
            segments: cut.segments,
            row_segments: cut.of_row,
            segment_of,
            zone_of: db.cells.ids().map(|id| zones.of(id)).collect(),
            nets_of: cost_nets_of(db),
            access,
            max_width,
            max_row_height,
            reliance: Mutex::new(HashMap::new()),
        }
    }

    /// The row whose bottom is at `y`.
    fn row_of(&self, y: Dbu) -> RowId {
        let rows = self.rows.iter().as_slice();
        RowId::new(
            rows.partition_point(|r| r.y < y)
                .min(rows.len().saturating_sub(1)),
        )
    }

    /// Forgets what the pin access of `cells` relied on, as they move.
    fn forget(&self, cells: impl IntoIterator<Item = CellId>) {
        let mut reliance = self.reliance.lock().unwrap_or_else(PoisonError::into_inner);
        for c in cells {
            let _ = reliance.remove(&c);
        }
    }

    /// The metal the pin access of `n`, standing where the placement has
    /// it, relies on, if all its pins are free; remembered until it moves.
    /// Only meaningful while every cell stands where the placement has it.
    fn relied(&self, db: &NetlistDB, n: &Neighbour) -> Option<Reliance> {
        if let Some(found) = self
            .reliance
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .get(&n.cell)
            && found.at == n.at
            && found.orient == n.orient
        {
            return Some(found.metal.clone());
        }
        let access = self.access?;
        let row = self.row_of(n.at.y);
        let beside = self.beside(db, n.cell, n.at, row, &[], Absent::NONE);
        let found = access.reliance(db, n.cell, n.at, n.orient, &beside)?;
        let _ = self
            .reliance
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .insert(
                n.cell,
                Relied {
                    at: n.at,
                    orient: n.orient,
                    metal: found.clone(),
                },
            );
        Some(found)
    }

    /// Signal pins of `cell` that fixed metal or the cells beside it would
    /// block at `at` in `row`, oriented as the row is, with every cell of
    /// `moved` standing where it says and none of `absent` standing anywhere.
    fn blocked_at(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        row: RowId,
        moved: &[Placement],
        absent: Absent,
    ) -> usize {
        let who = Neighbour {
            cell,
            at,
            orient: self.rows[row].orient,
        };
        self.blocked_in(db, who, row, moved, absent)
    }

    /// [`Self::blocked_at`] for `who`, standing and oriented as it says.
    fn blocked_in(
        &self,
        db: &NetlistDB,
        who: Neighbour,
        row: RowId,
        moved: &[Placement],
        absent: Absent,
    ) -> usize {
        let Some(access) = self.access else {
            return 0;
        };
        let beside = self.beside(db, who.cell, who.at, row, moved, absent);
        access.blocked_pins_among(db, who.cell, who.at, who.orient, &beside)
    }

    /// The cells whose metal can block a pin of `cell` standing at `at` in
    /// `row`: those within the check's reach along the row, in that row and
    /// the rows the escape along a track can run through, other than `cell`,
    /// `absent` and the ones `moved` names, which stand where `moved` says.
    fn beside(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        row: RowId,
        moved: &[Placement],
        absent: Absent,
    ) -> Vec<Neighbour> {
        let reach = self.access.map_or(Dbu::ZERO, PinAccess::reach);
        let escape = self.access.map_or(Dbu::ZERO, PinAccess::escape_reach);
        let lo = at.x - reach;
        let hi = at.x + db.cell_size(cell).x + reach;
        let elsewhere =
            |c: CellId| c == cell || absent.cell == Some(c) || moved.iter().any(|m| m.cell == c);
        let gone = |c: CellId| {
            absent.cell == Some(c)
                || absent
                    .segment
                    .is_some_and(|s| self.segment_of.get(&c) == Some(&s))
        };
        let y0 = self.rows[row].y - escape;
        let y1 = self.rows[row].y + self.rows[row].height + escape;
        let in_reach = |r: RowId| self.rows[r].y < y1 && self.rows[r].y + self.rows[r].height > y0;
        let rows = self.rows.iter().as_slice();
        let first = rows.partition_point(|r| r.y + self.max_row_height <= y0);
        let last = rows.partition_point(|r| r.y < y1).max(first);
        let mut beside: Vec<Neighbour> = (first..last)
            .map(RowId::new)
            .filter(|&r| in_reach(r))
            .flat_map(|r| self.row_segments[r].ids())
            .filter(|&s| absent.segment != Some(s))
            .flat_map(|s| {
                let cells = &self.segments[s].cells;
                let start =
                    cells.partition_point(|&c| db.cells[c].location.x + self.max_width <= lo);
                cells[start..]
                    .iter()
                    .copied()
                    .take_while(move |&c| db.cells[c].location.x < hi)
            })
            .filter(|&c| !elsewhere(c) && db.cell_bbox(c).max.x > lo)
            .map(|c| Neighbour {
                cell: c,
                at: db.cells[c].location,
                orient: db.cells[c].orient,
            })
            .collect();
        beside.extend(
            moved
                .iter()
                .filter(|m| {
                    m.cell != cell
                        && in_reach(m.row)
                        && !gone(m.cell)
                        && m.at.x < hi
                        && m.at.x + db.cell_size(m.cell).x > lo
                })
                .map(|m| Neighbour {
                    cell: m.cell,
                    at: m.at,
                    orient: self.rows[m.row].orient,
                }),
        );
        beside
    }

    /// Pins a placement would leave blocked: those of `cell` standing at
    /// `at` in `row`, and those of the cells beside it there that it would
    /// newly block, with every cell of `moved` standing where it says and
    /// none of `absent` standing anywhere.
    fn blocked_around(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        row: RowId,
        moved: &[Placement],
        absent: Absent,
    ) -> usize {
        let Some(access) = self.access else {
            return 0;
        };
        let own = self.blocked_at(db, cell, at, row, moved, absent);
        if own > 0 {
            return own;
        }
        let with_cell: Vec<Placement> = moved
            .iter()
            .copied()
            .chain(std::iter::once(Placement { cell, at, row }))
            .collect();
        let arriving: Vec<(LayerId, Rect<Dbu>)> = with_cell
            .iter()
            .flat_map(|m| access.cell_metal(db, m.cell, m.at, self.rows[m.row].orient))
            .collect();
        let gone = absent.and(cell);
        let disturbed: usize = self
            .beside(db, cell, at, row, moved, absent)
            .iter()
            .map(|n| {
                let standing = !with_cell.iter().any(|m| m.cell == n.cell);
                if standing
                    && let Some(relied) = self.relied(db, n)
                    && !access.disturbs(&arriving, &relied)
                {
                    return 0;
                }
                let r = self.row_of(n.at.y);
                let now = self.blocked_in(db, *n, r, &with_cell, absent);
                if now == 0 {
                    return 0;
                }
                let without = self.blocked_in(db, *n, r, moved, gone);
                now.saturating_sub(without)
            })
            .sum();
        own + disturbed
    }

    /// Signal pins of `cell` blocked where it stands.
    fn blocked_now(&self, db: &NetlistDB, cell: CellId, s: SegmentId) -> usize {
        let at = db.cells[cell].location;
        self.blocked_at(db, cell, at, self.segments[s].row, &[], Absent::NONE)
    }

    /// Blocked signal pins over every movable standard cell.
    fn count_blocked(&self, db: &NetlistDB) -> usize {
        self.segment_of
            .iter()
            .map(|(&cell, &s)| self.blocked_now(db, cell, s))
            .sum()
    }

    /// Re-places the cells of segment `s`, in their order, at the positions
    /// within the shift reach of each that leave no pin blocked and move
    /// them least in total; returns how many moved, or `None` when no such
    /// placement exists.
    fn free_segment(&self, db: &mut NetlistDB, s: SegmentId) -> Option<usize> {
        let plan = self.segment_plan(db, s)?;
        let mut moved = 0;
        for (c, p) in plan {
            if db.cells[c].location.x != p {
                db.cells[c].location.x = p;
                self.forget([c]);
                moved += 1;
            }
        }
        Some(moved)
    }

    /// The X of every cell of segment `s` in the re-placement
    /// [`Self::free_segment`] makes, without making it.
    fn segment_plan(&self, db: &NetlistDB, s: SegmentId) -> Option<Vec<(CellId, Dbu)>> {
        let candidates = self.spot_candidates(db, s)?;
        let best = self.least_displacement(db, s, &candidates);
        trace_plan(&self.segments[s].cells, &candidates, &best)
    }

    /// For each cell of segment `s`, the spots within its shift reach that
    /// leave no pin blocked with the rest of the segment away, with how far
    /// each is from where it stands; none when some cell has no such spot.
    fn spot_candidates(&self, db: &NetlistDB, s: SegmentId) -> Option<Vec<Vec<(Dbu, Dbu)>>> {
        let seg = &self.segments[s];
        let row = &self.rows[seg.row];
        let mut candidates: Vec<Vec<(Dbu, Dbu)>> = Vec::with_capacity(seg.cells.len());
        for &c in &seg.cells {
            let x = db.cells[c].location.x;
            let width = db.cell_size(c).x;
            let spots: Vec<(Dbu, Dbu)> = (-MAX_SHIFT_SITES..=MAX_SHIFT_SITES)
                .map(|k| x + row.site * k)
                .filter(|&p| p >= seg.lo && p + width <= seg.hi)
                .filter(|&p| {
                    self.blocked_around(
                        db,
                        c,
                        Point::new(p, row.y),
                        seg.row,
                        &[],
                        Absent::segment(s),
                    ) == 0
                })
                .map(|p| (p, (p - x).abs()))
                .collect();
            if spots.is_empty() {
                return None;
            }
            candidates.push(spots);
        }
        Some(candidates)
    }

    /// For each cell of segment `s` and each of its `candidates`, the least
    /// total displacement of the cells up to it with it there, and the
    /// spot of the cell before it that reaches it; none when unreachable.
    fn least_displacement(
        &self,
        db: &NetlistDB,
        s: SegmentId,
        candidates: &[Vec<(Dbu, Dbu)>],
    ) -> Vec<Vec<Reached>> {
        let seg = &self.segments[s];
        let reach = self.access.map_or(Dbu::ZERO, PinAccess::reach);
        let mut pairs = PairCheck::default();
        let mut best: Vec<Vec<Reached>> = Vec::with_capacity(candidates.len());
        best.push(
            candidates[0]
                .iter()
                .map(|&(_, cost)| Some((cost, 0)))
                .collect(),
        );
        for i in 1..candidates.len() {
            let (prev, cur) = (seg.cells[i - 1], seg.cells[i]);
            let width_before = db.cell_size(prev).x;
            let prefix = least_so_far(&best[i - 1]);
            let before = &candidates[i - 1];
            let here: Vec<Reached> = candidates[i]
                .iter()
                .map(|&(p, cost)| {
                    let fits = before.partition_point(|&(q, _)| q + width_before <= p);
                    let far = before.partition_point(|&(q, _)| q + width_before + reach <= p);
                    let distant = far.checked_sub(1).and_then(|k| prefix[k]);
                    let near = (far..fits)
                        .filter_map(|k| best[i - 1][k].map(|(total, _)| (total, k)))
                        .filter(|&(_, k)| {
                            pairs.clear(self, db, seg.row, (prev, before[k].0), (cur, p))
                        });
                    let (total, j) = distant.into_iter().chain(near).min()?;
                    Some((total + cost, j))
                })
                .collect();
            best.push(here);
        }
        best
    }

    /// Frees the pins of cell `i` of segment `s`: by the shortest shift
    /// along the segment that leaves it and every cell it pushes clear, else
    /// by swapping with a nearby cell of its width that is clear where it
    /// stands, else by moving into a gap of a nearby row.
    fn free_cell(&mut self, db: &mut NetlistDB, s: SegmentId, i: usize) -> Freed {
        let cell = self.segments[s].cells[i];
        let site = self.rows[self.segments[s].row].site;
        let x = db.cells[cell].location.x;
        if self.shift_free(db, s, i, x) {
            return Freed::InPlace;
        }
        let blocked = BlockedCell {
            cell,
            segment: s,
            x,
            reach: site * MAX_SHIFT_SITES,
        };
        let rows_near = self.rows_near(db, cell, self.segments[s].row);
        if self.swap_free(db, blocked, &rows_near) || self.gap_free(db, blocked, &rows_near) {
            return Freed::Elsewhere;
        }
        Freed::Not
    }

    /// Shifts cell `i` of segment `s`, standing at `x`, by the fewest sites
    /// that leave it and every cell it pushes clear; whether one did.
    fn shift_free(&self, db: &mut NetlistDB, s: SegmentId, i: usize, x: Dbu) -> bool {
        let cell = self.segments[s].cells[i];
        let site = self.rows[self.segments[s].row].site;
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
                        db.cells[c].location = at;
                        self.forget([c]);
                    }
                    return true;
                }
            }
        }
        false
    }

    /// The rows within [`FREE_ROWS`] of `row`, each nearer one first, of
    /// `cell`'s height and an orientation its master allows; `row` itself
    /// comes twice, as it is both zero rows below and zero above.
    fn rows_near(&self, db: &NetlistDB, cell: CellId, row: RowId) -> Vec<RowId> {
        let height = db.cell_size(cell).y;
        (0..=FREE_ROWS)
            .flat_map(|d| [row.index().checked_sub(d), Some(row.index() + d)])
            .flatten()
            .filter(|&r| r < self.rows.len())
            .map(RowId::new)
            .filter(|&r| {
                self.rows[r].height == height
                    && db.cell_macro(cell).symmetry.allows(self.rows[r].orient)
            })
            .collect()
    }

    /// Swaps the blocked cell with a partner from a segment of its zone in
    /// `rows_near`; whether it found one.
    fn swap_free(&mut self, db: &mut NetlistDB, blocked: BlockedCell, rows_near: &[RowId]) -> bool {
        let BlockedCell {
            cell,
            segment: s,
            x,
            reach,
        } = blocked;
        let row = self.segments[s].row;
        for &other in rows_near {
            for t in self.row_segments[other].ids() {
                if self.segments[t].zone != self.zone_of[cell] {
                    continue;
                }
                let Some(partner) = self.swap_partner(db, cell, s, t, x, reach) else {
                    continue;
                };
                let here = db.cells[cell].location;
                let there = db.cells[partner].location;
                log::trace!(
                    "cell {} at ({}, {}) swaps with {} at ({}, {})",
                    cell.index(),
                    here.x.get(),
                    here.y.get(),
                    partner.index(),
                    there.x.get(),
                    there.y.get()
                );
                db.cells[cell].location = there;
                db.cells[cell].orient = self.rows[other].orient;
                db.cells[partner].location = here;
                db.cells[partner].orient = self.rows[row].orient;
                self.forget([cell, partner]);
                self.swap_entries(db, cell, s, partner, t);
                return true;
            }
        }
        false
    }

    /// Moves the blocked cell into a clear gap of a segment of its zone in
    /// `rows_near`; whether it found one.
    fn gap_free(&mut self, db: &mut NetlistDB, blocked: BlockedCell, rows_near: &[RowId]) -> bool {
        let BlockedCell {
            cell,
            segment: s,
            x,
            reach,
        } = blocked;
        for &other in rows_near {
            for t in self.row_segments[other].ids() {
                if self.segments[t].zone != self.zone_of[cell] {
                    continue;
                }
                let Some(gx) = self.clear_spot_near(db, t, cell, x, reach) else {
                    continue;
                };
                let at = Point::new(gx, self.rows[other].y);
                let from = db.cells[cell].location;
                log::trace!(
                    "cell {} moves from ({}, {}) to a gap at ({}, {})",
                    cell.index(),
                    from.x.get(),
                    from.y.get(),
                    at.x.get(),
                    at.y.get()
                );
                db.cells[cell].location = at;
                db.cells[cell].orient = self.rows[other].orient;
                self.forget([cell]);
                self.relocate(db, cell, s, t);
                return true;
            }
        }
        false
    }

    /// A cell of segment `t` within `reach` of `x`, as wide as `cell` and of
    /// its zone, that is clear where `cell` stands while `cell` is clear
    /// where it stands, so that swapping them frees `cell` without blocking
    /// it.
    fn swap_partner(
        &self,
        db: &NetlistDB,
        cell: CellId,
        s: SegmentId,
        t: SegmentId,
        x: Dbu,
        reach: Dbu,
    ) -> Option<CellId> {
        let width = db.cell_size(cell).x;
        let here = db.cells[cell].location;
        let (row_here, row_there) = (self.segments[s].row, self.segments[t].row);
        let cells = &self.segments[t].cells;
        let k = cells.partition_point(|&c| db.cells[c].location.x < x);
        cells[k.saturating_sub(SWAP_REACH)..(k + SWAP_REACH).min(cells.len())]
            .iter()
            .copied()
            .filter(|&c| c != cell && db.cell_size(c).x == width)
            .filter(|&c| (db.cells[c].location.x - x).abs() <= reach)
            .filter(|&c| self.zone_of[c] == self.zone_of[cell])
            .filter(|&c| db.cell_macro(c).symmetry.allows(self.rows[row_here].orient))
            .find(|&c| {
                let there = db.cells[c].location;
                let swapped = [
                    Placement {
                        cell,
                        at: there,
                        row: row_there,
                    },
                    Placement {
                        cell: c,
                        at: here,
                        row: row_here,
                    },
                ];
                self.blocked_around(db, cell, there, row_there, &swapped, Absent::NONE) == 0
                    && self.blocked_around(db, c, here, row_here, &swapped, Absent::NONE) == 0
            })
    }

    /// The moves that put cell `i` of segment `s` at `nx` and push the
    /// cells it would overlap out of the way, or `None` when a pushed cell
    /// would leave the segment or land where a pin is blocked.
    fn shift_plan(
        &self,
        db: &NetlistDB,
        s: SegmentId,
        i: usize,
        nx: Dbu,
    ) -> Option<Vec<(CellId, Point<Dbu>)>> {
        let seg = &self.segments[s];
        let row = &self.rows[seg.row];
        let cell = seg.cells[i];
        let width = db.cell_size(cell).x;
        if nx < seg.lo || nx + width > seg.hi {
            return None;
        }
        let stand = |cell: CellId, x: Dbu| Placement {
            cell,
            at: Point::new(x, row.y),
            row: seg.row,
        };
        let mut moves = vec![stand(cell, nx)];
        let mut bound = nx;
        for &c in seg.cells[..i].iter().rev() {
            let bbox = db.cell_bbox(c);
            if bbox.max.x <= bound {
                break;
            }
            let at = snap_down(bound - bbox.width(), row.x_min, row.site);
            if at < seg.lo {
                return None;
            }
            moves.push(stand(c, at));
            bound = at;
        }
        let mut bound = nx + width;
        for &c in &seg.cells[i + 1..] {
            let bbox = db.cell_bbox(c);
            if bbox.min.x >= bound {
                break;
            }
            let at = snap_up(bound, row.x_min, row.site);
            if at + bbox.width() > seg.hi {
                return None;
            }
            moves.push(stand(c, at));
            bound = at + bbox.width();
        }
        let clear = moves
            .iter()
            .all(|m| self.blocked_around(db, m.cell, m.at, m.row, &moves, Absent::NONE) == 0);
        clear.then(|| moves.into_iter().map(|m| (m.cell, m.at)).collect())
    }

    fn total_cost(&self, db: &NetlistDB) -> f64 {
        let mut seen = IdVec::filled(false, db.nets.len());
        let mut total = 0.0;
        for nets in &self.nets_of {
            for &n in nets {
                if !std::mem::replace(&mut seen[n], true) {
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
            .flat_map(|&c| self.nets_of[c].iter().copied())
            .collect();
        nets.sort_unstable();
        nets.dedup();
        nets
    }

    /// Applies `placements`, keeping them if they lower the cost of the nets
    /// on `cells` and block no pin; returns whether they were kept.
    fn try_move(&self, db: &mut NetlistDB, cells: &[CellId], placements: &[Placement]) -> bool {
        let nets = self.affected(cells);
        let before = Self::cost_of(db, &nets);
        let saved: Vec<_> = placements
            .iter()
            .map(|m| (m.cell, db.cells[m.cell].location, db.cells[m.cell].orient))
            .collect();
        let apply = |db: &mut NetlistDB| {
            for m in placements {
                db.cells[m.cell].location = m.at;
                db.cells[m.cell].orient = self.rows[m.row].orient;
            }
        };
        apply(db);
        let improves = Self::cost_of(db, &nets) < before - 1e-9;
        for &(c, at, orient) in &saved {
            db.cells[c].location = at;
            db.cells[c].orient = orient;
        }
        // Pin checks read the placement as it stands, so they run undone.
        let clear = improves
            && !placements.iter().any(|m| {
                self.blocked_around(db, m.cell, m.at, m.row, placements, Absent::NONE) > 0
            });
        if clear {
            apply(db);
            self.forget(placements.iter().map(|m| m.cell));
        }
        clear
    }

    /// The box spanned by the medians of each net's other-pin bounding box.
    fn optimal_region(&self, db: &NetlistDB, cell: CellId) -> Option<Rect<Dbu>> {
        let mut xs = Vec::new();
        let mut ys = Vec::new();
        for &n in &self.nets_of[cell] {
            let others = db.nets[n].pins.iter().filter(
                |&&p| !matches!(db.pins[p].owner, PinOwner::Cell { cell: c, .. } if c == cell),
            );
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

    /// Moves `cell` toward its optimal region: into a gap there, or by
    /// swapping places with a cell of its width there; whether it moved.
    fn global_move(&mut self, db: &mut NetlistDB, cell: CellId) -> bool {
        let Some(region) = self.optimal_region(db, cell) else {
            return false;
        };
        let size = db.cell_size(cell);
        let here = db.cells[cell].location;
        if region.contains(here + Point::new(size.x / 2, size.y / 2)) {
            return false;
        }
        let target = region.center() - Point::new(size.x / 2, size.y / 2);
        let r = self
            .rows
            .iter()
            .as_slice()
            .partition_point(|row| row.y + row.height <= target.y)
            .min(self.rows.len() - 1);
        let Some(&from) = self.segment_of.get(&cell) else {
            return false;
        };
        let wanted = WantedMove {
            cell,
            from,
            here,
            target,
            size,
        };
        for row in rows_around(r, self.rows.len()).map(RowId::new) {
            if self.rows[row].height != size.y
                || !db.cell_macro(cell).symmetry.allows(self.rows[row].orient)
            {
                continue;
            }
            for s in self.row_segments[row].ids() {
                let seg = &self.segments[s];
                if seg.zone != self.zone_of[cell]
                    || seg.hi < target.x - size.x * 4
                    || seg.lo > target.x + size.x * 4
                {
                    continue;
                }
                if self.move_into_segment(db, wanted, row, s) {
                    return true;
                }
            }
        }
        false
    }

    /// Moves the wanted cell into a gap of segment `s` of `row` near its
    /// target, or else swaps it with a cell of its width there; whether it
    /// moved.
    fn move_into_segment(
        &mut self,
        db: &mut NetlistDB,
        wanted: WantedMove,
        row: RowId,
        s: SegmentId,
    ) -> bool {
        let WantedMove {
            cell,
            from,
            here,
            target,
            size,
        } = wanted;
        if let Some(x) = self.gap_near(db, s, target.x, size.x) {
            let at = Point::new(x, self.rows[row].y);
            if self.try_move(db, &[cell], &[Placement { cell, at, row }]) {
                self.relocate(db, cell, from, s);
                return true;
            }
        }
        let Some(other) = self.equal_width_near(db, s, target.x, size.x, cell) else {
            return false;
        };
        let other_at = db.cells[other].location;
        let other_row = self.segments[from].row;
        if self.zone_of[other] != self.segments[from].zone
            || !db
                .cell_macro(other)
                .symmetry
                .allows(self.rows[other_row].orient)
        {
            return false;
        }
        let swapped = [
            Placement {
                cell,
                at: other_at,
                row,
            },
            Placement {
                cell: other,
                at: here,
                row: other_row,
            },
        ];
        if !self.try_move(db, &[cell, other], &swapped) {
            return false;
        }
        self.swap_entries(db, cell, from, other, s);
        true
    }

    /// The free gaps of segment `s` between the few cells nearest `target`
    /// that hold a cell of `width`, as the X range its left edge may take.
    fn gaps_near(&self, db: &NetlistDB, s: SegmentId, target: Dbu, width: Dbu) -> Vec<Interval> {
        const REACH: usize = 8;
        let seg = &self.segments[s];
        let k = seg
            .cells
            .partition_point(|&c| db.cells[c].location.x < target);
        let first = k.saturating_sub(REACH);
        let last = (k + REACH).min(seg.cells.len());
        (first..=last)
            .filter_map(|j| {
                let gap_lo = if j == 0 {
                    seg.lo
                } else {
                    db.cell_bbox(seg.cells[j - 1]).max.x
                };
                let gap_hi = if j == seg.cells.len() {
                    seg.hi
                } else {
                    db.cells[seg.cells[j]].location.x
                };
                (gap_hi - gap_lo >= width).then_some(Interval::new(gap_lo, gap_hi - width))
            })
            .collect()
    }

    /// A site-aligned X for a cell of `width` in a free gap of segment `s`,
    /// as close as possible to `target`.
    fn gap_near(&self, db: &NetlistDB, s: SegmentId, target: Dbu, width: Dbu) -> Option<Dbu> {
        let row = &self.rows[self.segments[s].row];
        self.gaps_near(db, s, target, width)
            .into_iter()
            .map(|left| snap_down(target.clamp(left.lo, left.hi), row.x_min, row.site).max(left.lo))
            .min_by_key(|&x| (x - target).abs())
    }

    /// The site-aligned X nearest `target`, within `reach` of it, in a free
    /// gap of segment `s` where none of `cell`'s pins is blocked.
    fn clear_spot_near(
        &self,
        db: &NetlistDB,
        s: SegmentId,
        cell: CellId,
        target: Dbu,
        reach: Dbu,
    ) -> Option<Dbu> {
        let row_index = self.segments[s].row;
        let row = &self.rows[row_index];
        let width = db.cell_size(cell).x;
        let mut best: Option<Dbu> = None;
        for Interval { lo, hi } in self.gaps_near(db, s, target, width) {
            let lo = snap_up(lo.max(target - reach), row.x_min, row.site);
            let hi = snap_down(hi.min(target + reach), row.x_min, row.site);
            let mut x = lo;
            while x <= hi {
                let closer = best.is_none_or(|b| (x - target).abs() < (b - target).abs());
                let at = Point::new(x, row.y);
                if closer
                    && self.blocked_around(
                        db,
                        cell,
                        at,
                        row_index,
                        &[Placement {
                            cell,
                            at,
                            row: row_index,
                        }],
                        Absent::NONE,
                    ) == 0
                {
                    best = Some(x);
                }
                x += row.site;
            }
        }
        best
    }

    /// An equal-width cell in segment `s` near `target`, other than `cell`.
    fn equal_width_near(
        &self,
        db: &NetlistDB,
        s: SegmentId,
        target: Dbu,
        width: Dbu,
        cell: CellId,
    ) -> Option<CellId> {
        let seg = &self.segments[s];
        let k = seg
            .cells
            .partition_point(|&c| db.cells[c].location.x < target);
        seg.cells[k.saturating_sub(2)..(k + 2).min(seg.cells.len())]
            .iter()
            .copied()
            .filter(|&c| c != cell && db.cell_size(c).x == width)
            .min_by_key(|&c| (db.cells[c].location.x - target).abs())
    }

    fn relocate(&mut self, db: &NetlistDB, cell: CellId, from: SegmentId, to: SegmentId) {
        self.segments[from].cells.retain(|&c| c != cell);
        let x = db.cells[cell].location.x;
        let cells = &mut self.segments[to].cells;
        let k = cells.partition_point(|&c| db.cells[c].location.x < x);
        cells.insert(k, cell);
        let _ = self.segment_of.insert(cell, to);
    }

    fn swap_entries(
        &mut self,
        db: &NetlistDB,
        a: CellId,
        seg_a: SegmentId,
        b: CellId,
        seg_b: SegmentId,
    ) {
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
                .sort_by_key(|&c| db.cells[c].location.x);
        }
    }

    fn local_reorder(&mut self, db: &mut NetlistDB) -> usize {
        const ORDERS: [[usize; 3]; 5] = [[0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
        let mut kept = 0;
        for s in self.segments.ids() {
            let row = self.segments[s].row;
            let mut i = 0;
            while i + 3 <= self.segments[s].cells.len() {
                let window: [CellId; 3] = [
                    self.segments[s].cells[i],
                    self.segments[s].cells[i + 1],
                    self.segments[s].cells[i + 2],
                ];
                let left = db.cells[window[0]].location.x;
                let y = db.cells[window[0]].location.y;
                for order in ORDERS {
                    let mut x = left;
                    let placements: Vec<_> = order
                        .iter()
                        .map(|&k| {
                            let c = window[k];
                            let at = Point::new(x, y);
                            x += db.cell_size(c).x;
                            Placement { cell: c, at, row }
                        })
                        .collect();
                    if self.try_move(db, &window, &placements) {
                        for (slot, m) in placements.iter().enumerate() {
                            self.segments[s].cells[i + slot] = m.cell;
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

/// The segments of every row, and which belong to each row.
struct CutSegments {
    segments: IdVec<SegmentId, Segment>,
    of_row: IdVec<RowId, SegmentSpan>,
}

/// Cuts each row into the stretches free of `obstacles` within each zone,
/// their ends on sites.
fn cut_segments(
    db: &NetlistDB,
    zones: &Zones,
    rows: &IdVec<RowId, RowSpan>,
    obstacles: &[Rect<Dbu>],
) -> CutSegments {
    let mut segments: IdVec<SegmentId, Segment> = IdVec::new();
    let mut of_row = IdVec::with_capacity(rows.len());
    for (r, row) in rows.iter_enumerated() {
        let start = segments.next_id();
        let span = Interval::new(
            row.x_min.max(db.die_area.min.x),
            row.x_max.min(db.die_area.max.x),
        );
        let band = row.band();
        let free = free_intervals(obstacles, span, band);
        for zone in zones.all() {
            for Interval { lo: a, hi: b } in
                intersect_sorted(&free, &zones.intervals(zone, span, band))
            {
                let (a, b) = (
                    snap_up(a, row.x_min, row.site),
                    snap_down(b, row.x_min, row.site),
                );
                if b > a {
                    let _ = segments.push(Segment {
                        row: r,
                        zone,
                        lo: a,
                        hi: b,
                        cells: Vec::new(),
                    });
                }
            }
        }
        let _ = of_row.push(SegmentSpan {
            start,
            end: segments.next_id(),
        });
    }
    CutSegments { segments, of_row }
}

/// Puts each of `std_cells` in the segment of its zone that holds it, each
/// segment's cells ordered by X, and returns each placed cell's segment.
fn assign_segments(
    db: &NetlistDB,
    zones: &Zones,
    rows: &IdVec<RowId, RowSpan>,
    cut: &mut CutSegments,
    std_cells: &[CellId],
) -> HashMap<CellId, SegmentId> {
    let mut segment_of = HashMap::new();
    for &cell in std_cells {
        let at = db.cells[cell].location;
        let width = db.cell_size(cell).x;
        let r = rows.iter().as_slice().partition_point(|row| row.y < at.y);
        let zone = zones.of(cell);
        let found = (r..rows.len())
            .map(RowId::new)
            .take_while(|&r| rows[r].y == at.y)
            .flat_map(|r| cut.of_row[r].ids())
            .find(|&s| {
                let seg = &cut.segments[s];
                seg.zone == zone && seg.lo <= at.x && at.x + width <= seg.hi
            });
        if let Some(s) = found {
            cut.segments[s].cells.push(cell);
            let _ = segment_of.insert(cell, s);
        }
    }
    for seg in &mut cut.segments {
        seg.cells.sort_by_key(|&c| db.cells[c].location.x);
    }
    segment_of
}

/// For each cell, the nets on it the cost counts: signal nets of two to
/// [`MAX_NET_PINS`] pins.
fn cost_nets_of(db: &NetlistDB) -> IdVec<CellId, Vec<NetId>> {
    let mut nets_of = IdVec::filled(Vec::new(), db.num_cells());
    for (id, net) in db.nets.iter_enumerated() {
        if net.signal_use.is_supply() || net.pins.len() < 2 || net.pins.len() > MAX_NET_PINS {
            continue;
        }
        for &pin in &net.pins {
            if let PinOwner::Cell { cell, .. } = db.pins[pin].owner {
                let list: &mut Vec<NetId> = &mut nets_of[cell];
                if list.last() != Some(&id) {
                    list.push(id);
                }
            }
        }
    }
    nets_of
}

/// The least total displacement of the cells up to one with it at a spot,
/// and the spot of the cell before it that reaches it; none when
/// unreachable.
type Reached = Option<(Dbu, usize)>;

/// For each spot, the least reachable total among it and the spots before
/// it, with the spot that has it.
fn least_so_far(reached: &[Reached]) -> Vec<Reached> {
    let mut prefix: Vec<Reached> = Vec::with_capacity(reached.len());
    let mut low: Reached = None;
    for (j, at) in reached.iter().enumerate() {
        if let Some((cost, _)) = *at
            && low.is_none_or(|(least, _)| cost < least)
        {
            low = Some((cost, j));
        }
        prefix.push(low);
    }
    prefix
}

/// The spot of each of `cells` in the least total displacement `best`
/// reaches, followed back from the cheapest spot of the last cell.
fn trace_plan(
    cells: &[CellId],
    candidates: &[Vec<(Dbu, Dbu)>],
    best: &[Vec<Reached>],
) -> Option<Vec<(CellId, Dbu)>> {
    let last = best.len() - 1;
    let (mut k, _) = best[last]
        .iter()
        .enumerate()
        .filter_map(|(k, reached)| reached.map(|(total, _)| (k, total)))
        .min_by_key(|&(_, total)| total)?;
    let mut plan = Vec::with_capacity(best.len());
    for i in (0..=last).rev() {
        let (p, _) = candidates[i][k];
        plan.push((cells[i], p));
        k = best[i][k]?.1;
    }
    Some(plan)
}

/// A cell [`DetailedPlacer::global_move`] moves: its segment and place,
/// where it is wanted and its size.
#[derive(Clone, Copy, Debug)]
struct WantedMove {
    cell: CellId,
    from: SegmentId,
    here: Point<Dbu>,
    target: Point<Dbu>,
    size: Point<Dbu>,
}

/// A cell with a blocked pin, its segment and X, and how far it may move.
#[derive(Clone, Copy, Debug)]
struct BlockedCell {
    cell: CellId,
    segment: SegmentId,
    x: Dbu,
    reach: Dbu,
}

/// Weighted half-perimeter wirelength of one net.
fn net_cost(db: &NetlistDB, net: NetId) -> f64 {
    let n = &db.nets[net];
    let mut pins = n.pins.iter().map(|&p| db.pin_position(p));
    let Some(first) = pins.next() else { return 0.0 };
    let bbox = pins.fold(Rect::new(first, first), |b, p| b.union(&Rect::new(p, p)));
    n.weight * (bbox.width() + bbox.height()).to_f64()
}

/// Row `r` and the rows below and above it, those that exist of `rows`.
fn rows_around(r: usize, rows: usize) -> impl Iterator<Item = usize> {
    [Some(r), r.checked_sub(1), (r + 1 < rows).then_some(r + 1)]
        .into_iter()
        .flatten()
}

#[cfg(test)]
mod tests;
