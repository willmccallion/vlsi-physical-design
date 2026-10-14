//! The placer's floating-point view of a design.
//!
//! Analytical placement works in continuous coordinates, so the design is
//! converted to microns once on entry and the result rounded back to database
//! units on exit. Supply nets are left out: they connect almost every cell
//! and would only pull everything together.

use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::CellId;
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

/// A design in microns, with one entry per cell in the same order as the database.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementModel {
    /// Placement region.
    pub die: Rect<f64>,
    /// Width and height of each cell as placed.
    pub sizes: Vec<Point<f64>>,
    /// Whether the placer may move each cell.
    pub movable: Vec<bool>,
    /// Lower-left corner of each cell.
    pub positions: Vec<Point<f64>>,
    /// Signal nets with at least two connections.
    pub nets: Vec<ModelNet>,
}

impl PlacementModel {
    /// Builds the model from the current state of `db`.
    pub fn from_db(db: &NetlistDB) -> Self {
        let units = db.dbu();
        let um = |p: Point<pare_common::geom::dbu::Dbu>| {
            Point::new(units.to_microns(p.x), units.to_microns(p.y))
        };
        let nets = db
            .nets
            .iter()
            .filter(|n| !n.signal_use.is_supply() && n.pins.len() >= 2)
            .map(|n| ModelNet {
                weight: n.weight,
                pins: n
                    .pins
                    .iter()
                    .map(|&pin| {
                        let at = db.pin_position(pin);
                        match db.pins[pin.index()].owner {
                            PinOwner::Cell { cell, .. } => PinAnchor::Cell {
                                cell: cell.index(),
                                offset: um(at - db.cells[cell.index()].location),
                            },
                            PinOwner::Io(_) => PinAnchor::Fixed(um(at)),
                        }
                    })
                    .collect(),
            })
            .collect();
        Self {
            die: Rect::new(um(db.die_area.min), um(db.die_area.max)),
            sizes: (0..db.num_cells())
                .map(|i| um(db.cell_size(CellId::new(i))))
                .collect(),
            movable: db.cells.iter().map(|c| c.status.is_movable()).collect(),
            positions: db.cells.iter().map(|c| um(c.location)).collect(),
            nets,
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
