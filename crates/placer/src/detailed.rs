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
use pare_common::db::indices::{CellId, LayerId, MacroId, NetId};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::verify::pin_access::{Neighbour, PinAccess};
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
    for s in 0..dp.segments.len() {
        let any_blocked = dp.segments[s]
            .cells
            .iter()
            .any(|&c| dp.blocked_now(db, c, s) > 0);
        if any_blocked {
            moved += dp.free_segment(db, s).unwrap_or(0);
        }
    }
    for s in 0..dp.segments.len() {
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
            let at = db.cells[cell.index()].location;
            let blocked = dp.blocked_now(db, cell, s);
            if blocked > 0 {
                let beside: Vec<&str> = dp
                    .beside(db, cell, at, dp.segments[s].row, &[], &[])
                    .iter()
                    .map(|n| db.cells[n.cell.index()].name.as_str())
                    .collect();
                let alone = dp.access.map_or(0, |a| {
                    a.blocked_pins(db, cell, at, db.cells[cell.index()].orient)
                });
                log::debug!(
                    "cell {} at ({}, {}) keeps {blocked} blocked pins ({alone} by fixed metal alone) beside {beside:?}",
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

/// What a cell's pin access relied on, and where the cell stood then.
#[derive(Clone, Debug)]
struct Relied {
    at: Point<Dbu>,
    orient: Orientation,
    metal: Reliance,
}

/// A cell standing at a point in a row, for checks of placements not yet
/// made.
type Placement = (CellId, Point<Dbu>, usize);

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
        row: usize,
        left: (CellId, Dbu),
        right: (CellId, Dbu),
    ) -> bool {
        let Some(access) = dp.access else {
            return true;
        };
        let gap = right.1 - (left.1 + db.cell_size(left.0).x);
        let key = (
            db.cells[left.0.index()].macro_id,
            db.cells[right.0.index()].macro_id,
            gap,
        );
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

        let max_width = std_cells
            .iter()
            .map(|&c| db.cell_size(c).x)
            .max()
            .unwrap_or(Dbu::ZERO);
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

        let max_row_height = rows.iter().map(|r| r.height).max().unwrap_or(Dbu::ZERO);
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
            max_width,
            max_row_height,
            reliance: Mutex::new(HashMap::new()),
        }
    }

    /// The row whose bottom is at `y`.
    fn row_of(&self, y: Dbu) -> usize {
        self.rows
            .partition_point(|r| r.y < y)
            .min(self.rows.len().saturating_sub(1))
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
        let beside = self.beside(db, n.cell, n.at, row, &[], &[]);
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
    /// `moved` standing where it says and none of `away` standing anywhere.
    fn blocked_at(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        row: usize,
        moved: &[Placement],
        away: &[CellId],
    ) -> usize {
        let who = Neighbour {
            cell,
            at,
            orient: self.rows[row].orient,
        };
        self.blocked_in(db, who, row, moved, away)
    }

    /// [`Self::blocked_at`] for `who`, standing and oriented as it says.
    fn blocked_in(
        &self,
        db: &NetlistDB,
        who: Neighbour,
        row: usize,
        moved: &[Placement],
        away: &[CellId],
    ) -> usize {
        let Some(access) = self.access else {
            return 0;
        };
        let beside = self.beside(db, who.cell, who.at, row, moved, away);
        access.blocked_pins_among(db, who.cell, who.at, who.orient, &beside)
    }

    /// The cells whose metal can block a pin of `cell` standing at `at` in
    /// `row`: those within the check's reach along the row, in that row and
    /// the rows the escape along a track can run through, other than `cell`,
    /// `away` and the ones `moved` names, which stand where `moved` says.
    fn beside(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        row: usize,
        moved: &[Placement],
        away: &[CellId],
    ) -> Vec<Neighbour> {
        let reach = self.access.map_or(Dbu::ZERO, PinAccess::reach);
        let escape = self.access.map_or(Dbu::ZERO, PinAccess::escape_reach);
        let lo = at.x - reach;
        let hi = at.x + db.cell_size(cell).x + reach;
        let elsewhere =
            |c: CellId| c == cell || away.contains(&c) || moved.iter().any(|&(m, _, _)| m == c);
        let y0 = self.rows[row].y - escape;
        let y1 = self.rows[row].y + self.rows[row].height + escape;
        let in_reach = |r: usize| self.rows[r].y < y1 && self.rows[r].y + self.rows[r].height > y0;
        let first = self
            .rows
            .partition_point(|r| r.y + self.max_row_height <= y0);
        let last = self.rows.partition_point(|r| r.y < y1).max(first);
        let mut beside: Vec<Neighbour> = (first..last)
            .filter(|&r| in_reach(r))
            .flat_map(|r| self.row_segments[r].0..self.row_segments[r].1)
            .flat_map(|s| {
                let cells = &self.segments[s].cells;
                let start = cells
                    .partition_point(|&c| db.cells[c.index()].location.x + self.max_width <= lo);
                cells[start..]
                    .iter()
                    .copied()
                    .take_while(move |&c| db.cells[c.index()].location.x < hi)
            })
            .filter(|&c| !elsewhere(c) && db.cell_bbox(c).max.x > lo)
            .map(|c| Neighbour {
                cell: c,
                at: db.cells[c.index()].location,
                orient: db.cells[c.index()].orient,
            })
            .collect();
        beside.extend(
            moved
                .iter()
                .filter(|&&(m, p, r)| {
                    m != cell
                        && in_reach(r)
                        && !away.contains(&m)
                        && p.x < hi
                        && p.x + db.cell_size(m).x > lo
                })
                .map(|&(m, p, r)| Neighbour {
                    cell: m,
                    at: p,
                    orient: self.rows[r].orient,
                }),
        );
        beside
    }

    /// Pins a placement would leave blocked: those of `cell` standing at
    /// `at` in `row`, and those of the cells beside it there that it would
    /// newly block, with every cell of `moved` standing where it says and
    /// none of `away` standing anywhere.
    fn blocked_around(
        &self,
        db: &NetlistDB,
        cell: CellId,
        at: Point<Dbu>,
        row: usize,
        moved: &[Placement],
        away: &[CellId],
    ) -> usize {
        let Some(access) = self.access else {
            return 0;
        };
        let own = self.blocked_at(db, cell, at, row, moved, away);
        if own > 0 {
            return own;
        }
        let with_cell: Vec<Placement> = moved
            .iter()
            .copied()
            .chain(std::iter::once((cell, at, row)))
            .collect();
        let arriving: Vec<(LayerId, Rect<Dbu>)> = with_cell
            .iter()
            .flat_map(|&(c, p, r)| access.cell_metal(db, c, p, self.rows[r].orient))
            .collect();
        let mut gone = away.to_vec();
        gone.push(cell);
        let disturbed: usize = self
            .beside(db, cell, at, row, moved, away)
            .iter()
            .map(|n| {
                let standing = !with_cell.iter().any(|&(m, _, _)| m == n.cell);
                if standing
                    && let Some(relied) = self.relied(db, n)
                    && !access.disturbs(&arriving, &relied)
                {
                    return 0;
                }
                let r = self.row_of(n.at.y);
                let now = self.blocked_in(db, *n, r, &with_cell, away);
                if now == 0 {
                    return 0;
                }
                let without = self.blocked_in(db, *n, r, moved, &gone);
                now.saturating_sub(without)
            })
            .sum();
        own + disturbed
    }

    /// Signal pins of `cell` blocked where it stands.
    fn blocked_now(&self, db: &NetlistDB, cell: CellId, s: usize) -> usize {
        let at = db.cells[cell.index()].location;
        self.blocked_at(db, cell, at, self.segments[s].row, &[], &[])
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
    fn free_segment(&self, db: &mut NetlistDB, s: usize) -> Option<usize> {
        let plan = self.segment_plan(db, s)?;
        let mut moved = 0;
        for (c, p) in plan {
            if db.cells[c.index()].location.x != p {
                db.cells[c.index()].location.x = p;
                self.forget([c]);
                moved += 1;
            }
        }
        Some(moved)
    }

    /// The X of every cell of segment `s` in the re-placement
    /// [`Self::free_segment`] makes, without making it.
    fn segment_plan(&self, db: &NetlistDB, s: usize) -> Option<Vec<(CellId, Dbu)>> {
        let seg = &self.segments[s];
        let row = &self.rows[seg.row];
        let mut candidates: Vec<Vec<(Dbu, i64)>> = Vec::with_capacity(seg.cells.len());
        for &c in &seg.cells {
            let x = db.cells[c.index()].location.x;
            let width = db.cell_size(c).x;
            let spots: Vec<(Dbu, i64)> = (-MAX_SHIFT_SITES..=MAX_SHIFT_SITES)
                .map(|k| x + row.site * k)
                .filter(|&p| p >= seg.lo && p + width <= seg.hi)
                .filter(|&p| {
                    self.blocked_around(db, c, Point::new(p, row.y), seg.row, &[], &seg.cells) == 0
                })
                .map(|p| (p, (p - x).abs().get()))
                .collect();
            if spots.is_empty() {
                return None;
            }
            candidates.push(spots);
        }
        let reach = self.access.map_or(Dbu::ZERO, PinAccess::reach);
        let mut pairs = PairCheck::default();
        let mut best: Vec<Vec<(i64, usize)>> = Vec::with_capacity(candidates.len());
        best.push(candidates[0].iter().map(|&(_, cost)| (cost, 0)).collect());
        for i in 1..candidates.len() {
            let (prev, cur) = (seg.cells[i - 1], seg.cells[i]);
            let width_before = db.cell_size(prev).x;
            let mut prefix: Vec<(i64, usize)> = Vec::with_capacity(best[i - 1].len());
            let mut low = (i64::MAX, 0);
            for (j, &(cost, _)) in best[i - 1].iter().enumerate() {
                if cost < low.0 {
                    low = (cost, j);
                }
                prefix.push(low);
            }
            let before = &candidates[i - 1];
            let here: Vec<(i64, usize)> = candidates[i]
                .iter()
                .map(|&(p, cost)| {
                    let fits = before.partition_point(|&(q, _)| q + width_before <= p);
                    let far = before.partition_point(|&(q, _)| q + width_before + reach <= p);
                    let distant = far.checked_sub(1).map(|k| prefix[k]);
                    let near = (far..fits)
                        .filter(|&k| best[i - 1][k].0 < i64::MAX)
                        .filter(|&k| pairs.clear(self, db, seg.row, (prev, before[k].0), (cur, p)))
                        .map(|k| (best[i - 1][k].0, k));
                    match distant.into_iter().chain(near).min() {
                        Some((total, j)) if total < i64::MAX => (total + cost, j),
                        _ => (i64::MAX, 0),
                    }
                })
                .collect();
            best.push(here);
        }
        let last = best.len() - 1;
        let (mut k, total) = best[last]
            .iter()
            .enumerate()
            .map(|(k, &(total, _))| (k, total))
            .min_by_key(|&(_, total)| total)?;
        if total == i64::MAX {
            return None;
        }
        let mut plan = Vec::with_capacity(best.len());
        for i in (0..=last).rev() {
            let (p, _) = candidates[i][k];
            plan.push((seg.cells[i], p));
            k = best[i][k].1;
        }
        Some(plan)
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
                        self.forget([c]);
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
                    self.forget([cell, partner]);
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
                let Some(gx) = self.clear_spot_near(db, t, cell, x, reach) else {
                    continue;
                };
                let at = Point::new(gx, self.rows[other].y);
                {
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
                    self.forget([cell]);
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
                let swapped = [(cell, there, row_there), (c, here, row_here)];
                self.blocked_around(db, cell, there, row_there, &swapped, &[]) == 0
                    && self.blocked_around(db, c, here, row_here, &swapped, &[]) == 0
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
        if nx < seg.lo || nx + width > seg.hi {
            return None;
        }
        let mut moves = vec![(cell, Point::new(nx, row.y), seg.row)];
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
            moves.push((c, Point::new(at, row.y), seg.row));
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
            moves.push((c, Point::new(at, row.y), seg.row));
            bound = at + bbox.width();
        }
        let clear = moves
            .iter()
            .all(|&(c, at, r)| self.blocked_around(db, c, at, r, &moves, &[]) == 0);
        clear.then(|| moves.into_iter().map(|(c, at, _)| (c, at)).collect())
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
        let nets = self.affected(cells);
        let before = Self::cost_of(db, &nets);
        let saved: Vec<_> = placements
            .iter()
            .map(|&(c, _, _)| (c, db.cells[c.index()].location, db.cells[c.index()].orient))
            .collect();
        let apply = |db: &mut NetlistDB| {
            for &(c, at, row) in placements {
                db.cells[c.index()].location = at;
                db.cells[c.index()].orient = self.rows[row].orient;
            }
        };
        apply(db);
        let improves = Self::cost_of(db, &nets) < before - 1e-9;
        for &(c, at, orient) in &saved {
            db.cells[c.index()].location = at;
            db.cells[c.index()].orient = orient;
        }
        // Pin checks read the placement as it stands, so they run undone.
        let clear = improves
            && !placements
                .iter()
                .any(|&(c, at, row)| self.blocked_around(db, c, at, row, placements, &[]) > 0);
        if clear {
            apply(db);
            self.forget(placements.iter().map(|&(c, _, _)| c));
        }
        clear
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

    /// The free gaps of segment `s` between the few cells nearest `target`
    /// that hold a cell of `width`, as the X range its left edge may take.
    fn gaps_near(&self, db: &NetlistDB, s: usize, target: Dbu, width: Dbu) -> Vec<(Dbu, Dbu)> {
        const REACH: usize = 8;
        let seg = &self.segments[s];
        let k = seg
            .cells
            .partition_point(|&c| db.cells[c.index()].location.x < target);
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
                    db.cells[seg.cells[j].index()].location.x
                };
                (gap_hi - gap_lo >= width).then_some((gap_lo, gap_hi - width))
            })
            .collect()
    }

    /// A site-aligned X for a cell of `width` in a free gap of segment `s`,
    /// as close as possible to `target`.
    fn gap_near(&self, db: &NetlistDB, s: usize, target: Dbu, width: Dbu) -> Option<Dbu> {
        let row = &self.rows[self.segments[s].row];
        self.gaps_near(db, s, target, width)
            .into_iter()
            .map(|(lo, hi)| snap_down(target.clamp(lo, hi), row.x_min, row.site).max(lo))
            .min_by_key(|&x| (x - target).abs())
    }

    /// The site-aligned X nearest `target`, within `reach` of it, in a free
    /// gap of segment `s` where none of `cell`'s pins is blocked.
    fn clear_spot_near(
        &self,
        db: &NetlistDB,
        s: usize,
        cell: CellId,
        target: Dbu,
        reach: Dbu,
    ) -> Option<Dbu> {
        let row_index = self.segments[s].row;
        let row = &self.rows[row_index];
        let width = db.cell_size(cell).x;
        let mut best: Option<Dbu> = None;
        for (lo, hi) in self.gaps_near(db, s, target, width) {
            let lo = snap_up(lo.max(target - reach), row.x_min, row.site);
            let hi = snap_down(hi.min(target + reach), row.x_min, row.site);
            let mut x = lo;
            while x <= hi {
                let closer = best.is_none_or(|b| (x - target).abs() < (b - target).abs());
                let at = Point::new(x, row.y);
                if closer
                    && self.blocked_around(db, cell, at, row_index, &[(cell, at, row_index)], &[])
                        == 0
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

    /// A 10 by 4 die with one row of 0.2-wide sites: horizontal M1 under
    /// vertical M2 joined by a 0.1 cut via with 0.1 by 0.18 pads on both
    /// layers; master R, 0.4 wide, with a 0.1-wide M1 pin ending 0.05 from
    /// its right edge; master E, 0.4 wide, with an M2 pin along its left
    /// edge. A via on R's pin breaks M2 spacing when E abuts R.
    fn access_design() -> (NetlistDB, Zones, PinAccess) {
        use pare_common::db::design::{TrackAxis, TrackDef};
        use pare_common::db::indices::{CutLayerId, MacroPinId};
        use pare_common::db::library::{CutLayer, ViaDef};
        use pare_common::db::rules::CutLayerRules;

        let layer = |name: &str, direction| RoutingLayer {
            name: name.into(),
            direction,
            pitch_x: Dbu(200),
            pitch_y: Dbu(200),
            offset_x: None,
            offset_y: None,
            width: Dbu(100),
            min_spacing: Dbu(100),
            spacing_table: None,
            rules: RoutingLayerRules::default(),
        };
        let rect = |x0, y0, x1, y1| Rect::new(pt(x0, y0), pt(x1, y1));
        let mut lib = Library::new(DbuPerMicron::new(1000).unwrap());
        let m1 = lib
            .add_routing_layer(layer("M1", Direction::Horizontal))
            .unwrap();
        let v1 = lib
            .add_cut_layer(CutLayer {
                name: "V1".into(),
                width: Some(Dbu(100)),
                spacing: Some(Dbu(120)),
                rules: CutLayerRules::default(),
            })
            .unwrap();
        let m2 = lib
            .add_routing_layer(layer("M2", Direction::Vertical))
            .unwrap();
        let _ = lib
            .add_via(ViaDef {
                name: "V12".into(),
                is_default: true,
                cut: v1,
                shapes: vec![
                    Shape {
                        layer: LayerRef::Routing(m1),
                        rect: rect(-50, -90, 50, 90),
                    },
                    Shape {
                        layer: LayerRef::Cut(v1),
                        rect: rect(-50, -50, 50, 50),
                    },
                    Shape {
                        layer: LayerRef::Routing(m2),
                        rect: rect(-50, -90, 50, 90),
                    },
                ],
                from_def: false,
            })
            .unwrap();
        let symmetry = Symmetry {
            x: true,
            y: true,
            r90: false,
        };
        let site = lib
            .add_site(Site {
                name: "s".into(),
                class: SiteClass::Core,
                size: pt(200, 2000),
                symmetry,
            })
            .unwrap();
        let mut master = |name: &str, pin_layer, pin: Rect<Dbu>| {
            let mut m = Macro::new(
                name.into(),
                MacroClass::Core(CoreKind::Regular),
                pt(400, 2000),
                symmetry,
                Some(site),
            );
            let _ = m.add_pin(MacroPin {
                name: "a".into(),
                direction: PinDirection::Input,
                signal_use: SignalUse::Signal,
                shapes: vec![Shape {
                    layer: LayerRef::Routing(pin_layer),
                    rect: pin,
                }],
            });
            lib.add_macro(m).unwrap()
        };
        let right_pin = master("R", m1, rect(250, 500, 350, 1500));
        let edge_pin = master("E", m2, rect(0, 500, 150, 1500));
        let mut db = NetlistDB::new("t".into(), lib);
        db.die_area = rect(0, 0, 10_000, 4000);
        db.tracks.push(TrackDef {
            axis: TrackAxis::X,
            start: Dbu(100),
            count: 50,
            step: Dbu(200),
            layers: vec![m2],
        });
        db.tracks.push(TrackDef {
            axis: TrackAxis::Y,
            start: Dbu(0),
            count: 21,
            step: Dbu(200),
            layers: vec![m1],
        });
        db.rows.push(Row {
            name: "r".into(),
            site: SiteId(0),
            origin: pt(0, 0),
            orient: Orientation::N,
            num_x: 50,
            num_y: 1,
            step: pt(200, 0),
        });
        let r = db
            .add_cell(
                "r".into(),
                right_pin,
                pt(1000, 0),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        let e = db
            .add_cell(
                "e".into(),
                edge_pin,
                pt(1400, 0),
                Orientation::N,
                PlacementStatus::Placed,
            )
            .unwrap();
        for (cell, name) in [(r, "nr"), (e, "ne")] {
            let net = db.net_or_insert(name);
            let _ = db.connect_cell_pin(cell, MacroPinId(0), net);
        }
        let _ = CutLayerId(0);
        let zones = Zones::new(&db).unwrap();
        let access = PinAccess::new(&db, LayerId(1)).unwrap();
        (db, zones, access)
    }

    #[test]
    fn a_cell_blocking_its_neighbours_pin_is_moved_a_site_away() {
        let (mut db, zones, access) = access_design();

        let repair = free_blocked_pins(&mut db, &zones, &access);

        assert_eq!(
            (repair.blocked_before, repair.blocked_after),
            (1, 0),
            "{repair:?}"
        );
        let gap = db.cells[1].location.x - (db.cells[0].location.x + Dbu(400));
        assert!(gap >= Dbu(200), "gap {gap:?}");
        assert!(
            placement_violations(&db).is_empty(),
            "{:?}",
            placement_violations(&db)
        );
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
