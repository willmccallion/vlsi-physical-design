//! The placer's floating-point view of a design.
//!
//! Analytical placement works in continuous coordinates, so the design is
//! converted to microns once on entry and the result rounded back to database
//! units on exit. Supply nets are left out: they connect almost every cell
//! and would only pull everything together.

use crate::rows::{RowSpan, blocked_stretches, expand_rows, free_intervals};
use crate::zones::{Zone, ZoneId, Zones};
use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, IdVec, NetId, PinId};
use pare_common::geom::dbu::{Dbu, DbuArea, DbuPerMicron};
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// Where a net connection sits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PinAnchor {
    /// At `offset` from the lower-left corner of a cell.
    Cell {
        /// The cell, whose entries the model's tables hold in database order.
        cell: CellId,
        /// Offset from the cell's lower-left corner.
        offset: Point<f64>,
    },
    /// At a fixed point, such as a top-level I/O pin.
    Fixed(Point<f64>),
}

/// How the placer treats a cell.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CellKind {
    /// A standard cell.
    Standard,
    /// A block, or a cell more than six rows high, as in `OpenROAD`.
    Macro,
}

/// Rows of height a cell must exceed to count as a macro, `gpl`'s
/// `Instance` `row_limit` of 6.
const MACRO_ROWS: i64 = 6;

/// One connection of a net.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModelPin {
    /// Where it sits.
    pub anchor: PinAnchor,
    /// Where `OpenROAD`'s `dbITerm::getAvgXY` puts it, relative to where it
    /// sits: the mean of its shapes' centers for a cell pin, and none for an
    /// I/O pin, which sits at the center of its shapes.
    pub average: Point<f64>,
}

/// A net as the placer sees it.
#[derive(Clone, Debug, PartialEq)]
pub struct ModelNet {
    /// The net in the database.
    pub net: NetId,
    /// The connections it ties together, at least two.
    pub pins: Vec<ModelPin>,
    /// Wirelength weight.
    pub weight: f64,
}

/// The stretches of row the cells of one zone may use.
#[derive(Clone, Debug, PartialEq)]
pub struct ZoneRows {
    /// The zone whose cells the rows hold.
    pub zone: Zone,
    /// Its stretches of row, each one row high; the zone's whole area for a
    /// design without rows.
    pub rects: Vec<Rect<f64>>,
}

/// A design in microns, with one entry per cell in the same order as the database.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementModel {
    /// Placement region.
    pub die: Rect<f64>,
    /// Rows cells may sit in, each one row high.
    pub rows: Vec<Rect<f64>>,
    /// Width and height of each cell as placed.
    pub sizes: IdVec<CellId, Point<f64>>,
    /// Width and height of each cell in the density, about its center: its
    /// size, scaled to the design's average pin density for a movable cell
    /// of three signal pins or more.
    pub density_sizes: IdVec<CellId, Point<f64>>,
    /// Whether the placer may move each cell.
    pub movable: IdVec<CellId, bool>,
    /// Whether each cell is a standard cell or a macro.
    pub kinds: IdVec<CellId, CellKind>,
    /// Lower-left corner of each cell.
    pub positions: IdVec<CellId, Point<f64>>,
    /// Signal nets with at least two connections.
    pub nets: Vec<ModelNet>,
    /// Every zone's stretches of row, without what global placement keeps
    /// out of: each fence, then the space outside.
    pub zones: IdVec<ZoneId, ZoneRows>,
    /// The zone of each cell.
    pub zone_of: IdVec<CellId, ZoneId>,
    /// Width of the narrowest row site, one micron for a design without
    /// rows.
    pub site_width: f64,
    /// The design's database units.
    pub units: DbuPerMicron,
}

/// The rectangle from (`x0`, `y0`) to (`x1`, `y1`) in microns.
fn microns_rect(units: DbuPerMicron, x0: Dbu, x1: Dbu, y0: Dbu, y1: Dbu) -> Rect<f64> {
    Rect::new(
        units.point_in_microns(Point::new(x0, y0)),
        units.point_in_microns(Point::new(x1, y1)),
    )
}

/// The signal nets of `db` with two pins or more among those that anchor
/// them: every cell pin, and the I/O pins `io` takes.
fn model_nets(db: &NetlistDB, io: IoAnchors) -> Vec<ModelNet> {
    let units = db.dbu();
    let um = |p: Point<Dbu>| units.point_in_microns(p);
    let model_pin = |pin: PinId| {
        let at = db.pin_position(pin);
        let (anchor, average) = match db.pins[pin].owner {
            PinOwner::Cell { cell, .. } => (
                PinAnchor::Cell {
                    cell,
                    offset: um(at - db.cells[cell].location),
                },
                um(db.average_pin_position(pin) - at),
            ),
            PinOwner::Io(pin) => {
                let fixed = !db.io_pins[pin].status.is_movable();
                if io == IoAnchors::FixedOnly && !fixed {
                    return None;
                }
                (PinAnchor::Fixed(um(at)), Point::default())
            }
        };
        Some(ModelPin { anchor, average })
    };
    db.nets
        .iter()
        .enumerate()
        .filter(|(_, n)| !n.signal_use.is_supply())
        .map(|(i, n)| ModelNet {
            net: NetId::new(i),
            weight: n.weight,
            pins: n.pins.iter().filter_map(|&pin| model_pin(pin)).collect(),
        })
        .filter(|n| n.pins.len() >= 2)
        .collect()
}

/// The stretches of row each zone's cells may use, in microns: the free
/// parts of the rows `spans` inside the zone, or the whole zone without
/// rows.
fn zone_rows(db: &NetlistDB, zones: &Zones, spans: &[RowSpan]) -> IdVec<ZoneId, ZoneRows> {
    let units = db.dbu();
    let blocked = blocked_stretches(db, spans);
    zones
        .all()
        .map(|zone| {
            let rects = if spans.is_empty() {
                whole_zone(db, zones, zone)
                    .iter()
                    .map(|r| microns_rect(units, r.min.x, r.max.x, r.min.y, r.max.y))
                    .collect()
            } else {
                spans
                    .iter()
                    .zip(&blocked)
                    .flat_map(|(r, stretches)| {
                        let (y0, y1) = (r.y, r.y + r.height);
                        zones
                            .intervals(zone, r.x_min, r.x_max, y0, y1)
                            .into_iter()
                            .flat_map(move |(a, b)| free_intervals(stretches, a, b, y0, y1))
                            .map(move |(a, b)| microns_rect(units, a, b, y0, y1))
                    })
                    .collect()
            };
            ZoneRows { zone, rects }
        })
        .collect()
}

/// Each cell's size in the density, as `OpenROAD`'s `PlacerBaseCommon`
/// extends instances by pin density: a movable cell of three signal pins or
/// more is scaled toward the area that holds its pins at the average density
/// of the movable cells, by no more than 1.2 nor less than 0.95.
fn pin_density_sizes(db: &NetlistDB) -> IdVec<CellId, Point<Dbu>> {
    let signal_pins = |i: CellId| {
        let pins = &db.cell_macro(i).pins;
        pins.iter().filter(|p| !p.signal_use.is_supply()).count()
    };
    let area = |s: Point<Dbu>| s.x * s.y;
    let (mut total_pins, mut movable_area) = (0, DbuArea::ZERO);
    for i in db.cells.ids() {
        if db.cells[i].status.is_movable() {
            total_pins += signal_pins(i);
            movable_area += area(db.cell_size(i));
        }
    }
    let average = if movable_area > DbuArea::ZERO {
        total_pins as f64 / movable_area.to_f64()
    } else {
        0.0
    };
    db.cells
        .ids()
        .map(|i| {
            let size = db.cell_size(i);
            let pins = signal_pins(i);
            if !db.cells[i].status.is_movable()
                || pins <= 2
                || average <= 0.0
                || area(size) == DbuArea::ZERO
            {
                return size;
            }
            let target = pins as f64 / average;
            let scale = (target / area(size).to_f64()).sqrt().clamp(0.95, 1.2);
            let scaled = |side: Dbu| Dbu((side.get() as f64 * scale) as i64 / 2 * 2);
            Point::new(scaled(size.x), scaled(size.y))
        })
        .collect()
}

/// The kind of every cell of `db`, given the height of the shortest row.
fn cell_kinds(db: &NetlistDB, row_height: Option<Dbu>) -> IdVec<CellId, CellKind> {
    db.cells
        .ids()
        .map(|id| {
            let tall = row_height.is_some_and(|h| db.cell_size(id).y > h * MACRO_ROWS);
            if db.cell_macro(id).is_block() || tall {
                CellKind::Macro
            } else {
                CellKind::Standard
            }
        })
        .collect()
}

/// Which I/O pins anchor the model's nets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IoAnchors {
    /// Every I/O pin, where it is now.
    All,
    /// Only those placement may not move.
    FixedOnly,
}

impl PlacementModel {
    /// Builds the model from the current state of `db`, its cells in
    /// `zones`.
    pub fn from_db(db: &NetlistDB, zones: &Zones) -> Self {
        Self::build(db, zones, IoAnchors::All)
    }

    /// Builds the model from `db` without the I/O pins placement may still
    /// move, so that cells are not pulled toward where those pins are now.
    pub fn from_db_without_movable_io(db: &NetlistDB, zones: &Zones) -> Self {
        Self::build(db, zones, IoAnchors::FixedOnly)
    }

    fn build(db: &NetlistDB, zones: &Zones, io: IoAnchors) -> Self {
        let units = db.dbu();
        let um = |p: Point<Dbu>| units.point_in_microns(p);
        let spans = expand_rows(db);
        Self {
            units,
            die: Rect::new(um(db.die_area.min), um(db.die_area.max)),
            rows: spans
                .iter()
                .map(|r| microns_rect(units, r.x_min, r.x_max, r.y, r.y + r.height))
                .collect(),
            sizes: db.cells.ids().map(|id| um(db.cell_size(id))).collect(),
            density_sizes: pin_density_sizes(db).into_iter().map(um).collect(),
            movable: db.cells.iter().map(|c| c.status.is_movable()).collect(),
            kinds: cell_kinds(db, spans.iter().map(|r| r.height).min()),
            positions: db.cells.iter().map(|c| um(c.location)).collect(),
            nets: model_nets(db, io),
            zones: zone_rows(db, zones, &spans),
            zone_of: db.cells.ids().map(|id| zones.id_of(id)).collect(),
            site_width: spans
                .iter()
                .map(|r| units.to_microns(r.site))
                .reduce(f64::min)
                .unwrap_or(1.0),
        }
    }

    /// Number of cells.
    pub const fn num_cells(&self) -> usize {
        self.positions.len()
    }

    /// Every cell of the model, in database order.
    pub fn cells(&self) -> impl Iterator<Item = CellId> + use<> {
        self.positions.ids()
    }

    /// Position of a connection for the given cell positions.
    #[inline]
    pub fn pin_position(pin: &PinAnchor, positions: &IdVec<CellId, Point<f64>>) -> Point<f64> {
        match *pin {
            PinAnchor::Cell { cell, offset } => positions[cell] + offset,
            PinAnchor::Fixed(p) => p,
        }
    }

    /// Area of the zones' rows not covered by fixed cells, or of the die for
    /// a design without rows.
    pub fn free_area(&self) -> f64 {
        if self.rows.is_empty() {
            return self.die.area();
        }
        let mut open: Vec<&Rect<f64>> = self.zones.iter().flat_map(|z| &z.rects).collect();
        open.sort_by(|a, b| a.min.y.total_cmp(&b.min.y));
        let rows: f64 = open.iter().map(|r| r.area()).sum();
        let tallest = open.iter().map(|r| r.height()).fold(0.0, f64::max);
        let fixed: f64 = self
            .cells()
            .filter(|&i| !self.movable[i])
            .map(|i| {
                let cell = Rect::new(self.positions[i], self.positions[i] + self.sizes[i]);
                let first = open.partition_point(|r| r.min.y + tallest <= cell.min.y);
                open[first..]
                    .iter()
                    .take_while(|r| r.min.y < cell.max.y)
                    .map(|r| overlap_area(r, &cell))
                    .sum::<f64>()
            })
            .sum();
        rows - fixed
    }

    /// Half-perimeter wirelength of all nets at the current positions,
    /// without net weights.
    pub fn hpwl(&self) -> f64 {
        self.nets
            .iter()
            .map(|net| {
                let mut lo = Point::new(f64::INFINITY, f64::INFINITY);
                let mut hi = Point::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
                for pin in &net.pins {
                    let p = Self::pin_position(&pin.anchor, &self.positions);
                    lo = Point::new(lo.x.min(p.x), lo.y.min(p.y));
                    hi = Point::new(hi.x.max(p.x), hi.y.max(p.y));
                }
                (hi.x - lo.x) + (hi.y - lo.y)
            })
            .sum()
    }

    /// Writes movable cell positions back to `db`, rounded to database units.
    pub fn write_back(&self, db: &mut NetlistDB) {
        let units = db.dbu();
        for (cell, (&p, &movable)) in db
            .cells
            .iter_mut()
            .zip(self.positions.iter().zip(&self.movable))
        {
            if !movable {
                continue;
            }
            cell.location = Point::new(units.to_dbu_rounded(p.x), units.to_dbu_rounded(p.y));
            cell.status = PlacementStatus::Placed;
        }
    }
}

/// The area of a zone in a design without rows: a fence's rectangles, or
/// the die outside them, which only a design without fences has whole.
fn whole_zone(db: &NetlistDB, zones: &Zones, zone: Zone) -> Vec<Rect<Dbu>> {
    match zone {
        Zone::Fence(_) => zones.fence_rects(zone).to_vec(),
        Zone::Outside if !zones.has_fences() => vec![db.die_area],
        Zone::Outside => Vec::new(),
    }
}

fn overlap_area(a: &Rect<f64>, b: &Rect<f64>) -> f64 {
    let w = a.max.x.min(b.max.x) - a.min.x.max(b.min.x);
    let h = a.max.y.min(b.max.y) - a.min.y.max(b.min.y);
    w.max(0.0) * h.max(0.0)
}
