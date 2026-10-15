//! The placer's floating-point view of a design.
//!
//! Analytical placement works in continuous coordinates, so the design is
//! converted to microns once on entry and the result rounded back to database
//! units on exit. Supply nets are left out: they connect almost every cell
//! and would only pull everything together.

use crate::rows::expand_rows;
use crate::zones::{Zone, Zones};
use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::CellId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// Where a net connection sits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PinAnchor {
    /// At `offset` from the lower-left corner of a cell.
    Cell {
        /// Index into the model's cells.
        cell: usize,
        /// Offset from the cell's lower-left corner.
        offset: Point<f64>,
    },
    /// At a fixed point, such as a top-level I/O pin.
    Fixed(Point<f64>),
}

/// A net as the placer sees it.
#[derive(Clone, Debug, PartialEq)]
pub struct ModelNet {
    /// Its connections.
    pub pins: Vec<PinAnchor>,
    /// Wirelength weight.
    pub weight: f64,
}

/// The stretches of row the cells of one zone may use.
#[derive(Clone, Debug, PartialEq)]
pub struct ZoneRows {
    /// The zone.
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
    pub sizes: Vec<Point<f64>>,
    /// Whether the placer may move each cell.
    pub movable: Vec<bool>,
    /// Lower-left corner of each cell.
    pub positions: Vec<Point<f64>>,
    /// Signal nets with at least two connections.
    pub nets: Vec<ModelNet>,
    /// Every zone's stretches of row: each fence, then the space outside.
    pub zones: Vec<ZoneRows>,
    /// For each cell, the index of its zone in `zones`.
    pub zone_of: Vec<usize>,
    /// Width of the narrowest row site, one micron for a design without
    /// rows.
    pub site_width: f64,
}

impl PlacementModel {
    /// Builds the model from the current state of `db`, its cells in
    /// `zones`.
    pub fn from_db(db: &NetlistDB, zones: &Zones) -> Self {
        Self::build(db, zones, true)
    }

    /// Builds the model from `db` without the I/O pins placement may still
    /// move, so that cells are not pulled toward where those pins are now.
    pub fn from_db_without_movable_io(db: &NetlistDB, zones: &Zones) -> Self {
        Self::build(db, zones, false)
    }

    fn build(db: &NetlistDB, zones: &Zones, movable_io: bool) -> Self {
        let units = db.dbu();
        let um = |p: Point<pare_common::geom::dbu::Dbu>| {
            Point::new(units.to_microns(p.x), units.to_microns(p.y))
        };
        let nets = db
            .nets
            .iter()
            .filter(|n| !n.signal_use.is_supply())
            .map(|n| ModelNet {
                weight: n.weight,
                pins: n
                    .pins
                    .iter()
                    .filter_map(|&pin| {
                        let at = db.pin_position(pin);
                        match db.pins[pin.index()].owner {
                            PinOwner::Cell { cell, .. } => Some(PinAnchor::Cell {
                                cell: cell.index(),
                                offset: um(at - db.cells[cell.index()].location),
                            }),
                            PinOwner::Io(io) => (movable_io
                                || !db.io_pins[io.index()].status.is_movable())
                            .then(|| PinAnchor::Fixed(um(at))),
                        }
                    })
                    .collect(),
            })
            .filter(|n| n.pins.len() >= 2)
            .collect();
        let spans = expand_rows(db);
        let um_rect = |x0: Dbu, x1: Dbu, y0: Dbu, y1: Dbu| {
            Rect::new(um(Point::new(x0, y0)), um(Point::new(x1, y1)))
        };
        let rows = spans
            .iter()
            .map(|r| um_rect(r.x_min, r.x_max, r.y, r.y + r.height))
            .collect();
        let zone_rows = zones
            .all()
            .map(|zone| {
                let rects = if spans.is_empty() {
                    whole_zone(db, zones, zone)
                        .iter()
                        .map(|r| um_rect(r.min.x, r.max.x, r.min.y, r.max.y))
                        .collect()
                } else {
                    spans
                        .iter()
                        .flat_map(|r| {
                            let (y0, y1) = (r.y, r.y + r.height);
                            zones
                                .intervals(zone, r.x_min, r.x_max, y0, y1)
                                .into_iter()
                                .map(move |(a, b)| um_rect(a, b, y0, y1))
                        })
                        .collect()
                };
                ZoneRows { zone, rects }
            })
            .collect();
        Self {
            die: Rect::new(um(db.die_area.min), um(db.die_area.max)),
            rows,
            sizes: (0..db.num_cells())
                .map(|i| um(db.cell_size(CellId::new(i))))
                .collect(),
            movable: db.cells.iter().map(|c| c.status.is_movable()).collect(),
            positions: db.cells.iter().map(|c| um(c.location)).collect(),
            nets,
            zones: zone_rows,
            zone_of: (0..db.num_cells())
                .map(|i| zones.index(zones.of(CellId::new(i))))
                .collect(),
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

    /// Position of a connection for the given cell positions.
    #[inline]
    pub fn pin_position(pin: &PinAnchor, positions: &[Point<f64>]) -> Point<f64> {
        match *pin {
            PinAnchor::Cell { cell, offset } => positions[cell] + offset,
            PinAnchor::Fixed(p) => p,
        }
    }

    /// Area of the rows not covered by fixed cells, or of the die for a
    /// design without rows.
    pub fn free_area(&self) -> f64 {
        if self.rows.is_empty() {
            return self.die.area();
        }
        let rows: f64 = self.rows.iter().map(Rect::area).sum();
        let tallest = self.rows.iter().map(Rect::height).fold(0.0, f64::max);
        let fixed: f64 = (0..self.num_cells())
            .filter(|&i| !self.movable[i])
            .map(|i| {
                let cell = Rect::new(self.positions[i], self.positions[i] + self.sizes[i]);
                let first = self
                    .rows
                    .partition_point(|r| r.min.y + tallest <= cell.min.y);
                self.rows[first..]
                    .iter()
                    .take_while(|r| r.min.y < cell.max.y)
                    .map(|r| overlap_area(r, &cell))
                    .sum::<f64>()
            })
            .sum();
        rows - fixed
    }

    /// Half-perimeter wirelength of all nets at the current positions.
    pub fn hpwl(&self) -> f64 {
        self.nets
            .iter()
            .map(|net| {
                let mut lo = Point::new(f64::INFINITY, f64::INFINITY);
                let mut hi = Point::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
                for pin in &net.pins {
                    let p = Self::pin_position(pin, &self.positions);
                    lo = Point::new(lo.x.min(p.x), lo.y.min(p.y));
                    hi = Point::new(hi.x.max(p.x), hi.y.max(p.y));
                }
                net.weight * ((hi.x - lo.x) + (hi.y - lo.y))
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
