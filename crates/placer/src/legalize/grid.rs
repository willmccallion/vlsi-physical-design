//! `OpenDP`'s grid of the core's rows, and its coordinates as dpl's
//! `Coordinates.h` types them: a site column, a row, and points and boxes
//! of the two.
//!
//! As in dpl, a coordinate and a span along one axis share a type.
//! Converting to and from database units goes through the methods that
//! name the rounding, never arithmetic on the raw values.

use super::opendp::DetailedPlacementError;
use pare_common::db::design::{NetlistDB, Row};
use pare_common::db::library::SiteClass;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::ops::{Add, Mul, Sub};

/// Defines one axis of the grid, as an instance of dpl's `TypedCoordinate`.
macro_rules! grid_axis {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
        pub struct $name(i64);

        impl $name {
            /// The grid's first line along this axis.
            pub const ZERO: Self = Self(0);

            /// The coordinate `value` lines from the grid's first.
            pub const fn new(value: i64) -> Self {
                Self(value)
            }

            /// The raw number of lines.
            pub const fn get(self) -> i64 {
                self.0
            }

            /// The span's length, whichever way it runs.
            pub const fn abs(self) -> Self {
                Self(self.0.abs())
            }

            /// The coordinates from `self` up to `end`, `end` excluded.
            pub fn until(self, end: Self) -> impl DoubleEndedIterator<Item = Self> {
                (self.0..end.0).map(Self)
            }

            /// The coordinates from `self` up to `end`, `end` included.
            pub fn through(self, end: Self) -> impl Iterator<Item = Self> {
                (self.0..=end.0).map(Self)
            }
        }

        impl Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                Self(self.0 + rhs.0)
            }
        }

        impl Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self {
                Self(self.0 - rhs.0)
            }
        }

        /// A step of whole lines, as dpl adds an `int`.
        impl Add<i64> for $name {
            type Output = Self;
            fn add(self, rhs: i64) -> Self {
                Self(self.0 + rhs)
            }
        }

        /// A step of whole lines, as dpl subtracts an `int`.
        impl Sub<i64> for $name {
            type Output = Self;
            fn sub(self, rhs: i64) -> Self {
                Self(self.0 - rhs)
            }
        }
    };
}

grid_axis!(
    /// A site column of the grid, or a width in sites, as dpl's `GridX`.
    GridX
);

grid_axis!(
    /// A row of the grid, the index of the row edge at its bottom, or a
    /// height in rows, as dpl's `GridY`.
    GridY
);

impl GridX {
    /// The column holding `x`, truncating toward zero, as `Grid::gridX`
    /// and dpl's `dbuToGridFloor`, whose `divFloor` truncates.
    pub const fn from_dbu_trunc(x: Dbu, site_width: Dbu) -> Self {
        Self(x.get() / site_width.get())
    }

    /// The first column at or right of `x`, in doubles as dpl's
    /// `dbuToGridCeil` and `Grid::gridEndX` divide.
    pub fn from_dbu_ceil(x: Dbu, site_width: Dbu) -> Self {
        Self((x.to_f64() / site_width.to_f64()).ceil() as i64)
    }

    /// The column nearest `x`, halves away from zero, in doubles as dpl's
    /// `divRound` divides.
    pub fn from_dbu_round(x: Dbu, site_width: Dbu) -> Self {
        Self((x.to_f64() / site_width.to_f64()).round() as i64)
    }

    /// The column's left edge, or the span's width, as dpl's `gridToDbu`.
    pub const fn to_dbu(self, site_width: Dbu) -> Dbu {
        Dbu(self.0 * site_width.get())
    }
}

/// A width scaled by a whole factor, as dpl scales a `GridX`'s value.
impl Mul<i64> for GridX {
    type Output = Self;
    fn mul(self, rhs: i64) -> Self {
        Self(self.0 * rhs)
    }
}

/// A point of the grid, as dpl's `GridPt`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct GridPt {
    pub x: GridX,
    pub y: GridY,
}

impl GridPt {
    pub const fn new(x: GridX, y: GridY) -> Self {
        Self { x, y }
    }
}

/// A box of grid points, `max` excluded, as dpl's `GridRect`.
#[derive(Clone, Copy, Debug)]
pub struct GridRect {
    pub min: GridPt,
    pub max: GridPt,
}

impl GridRect {
    /// Whether `p` lies in the box.
    pub fn contains(&self, p: GridPt) -> bool {
        (self.min.x..self.max.x).contains(&p.x) && (self.min.y..self.max.y).contains(&p.y)
    }
}

/// The core's rows, which pad sites do not make.
pub fn core_rows(db: &NetlistDB) -> Vec<&Row> {
    db.rows
        .iter()
        .filter(|r| db.library.site(r.site).class != SiteClass::Pad)
        .collect()
}

/// The core's rows as `OpenDP`'s `Grid` lays them out: columns one site
/// wide from the core's left, and rows between the bottom and top edges
/// of every row, all relative to the core.
#[derive(Clone, Debug)]
pub struct RowGrid {
    core: Rect<Dbu>,
    site_width: Dbu,
    /// The bottom and top of every row, ascending.
    row_edges: Vec<Dbu>,
    row_site_count: GridX,
}

impl RowGrid {
    /// The grid of `rows`, each `DO … BY n` statement as its `n` rows, as
    /// `Grid::allocateGrid` and `Grid::examineRows` lay it out.
    ///
    /// # Errors
    ///
    /// When there is no row, or the rows' sites differ in width.
    pub fn new(db: &NetlistDB, rows: &[&Row]) -> Result<Self, DetailedPlacementError> {
        let core = db.core_area();
        let first = rows.first().ok_or(DetailedPlacementError::NoRows)?;
        let site_width = db.library.site(first.site).size.x;
        let mut row_edges = Vec::new();
        for r in rows {
            let site = db.library.site(r.site).size;
            if site.x != site_width {
                return Err(DetailedPlacementError::SiteWidths(site.x, site_width));
            }
            let bottom = r.origin.y - core.min.y;
            for k in 0..i64::from(r.num_y) {
                let y = bottom + r.step.y * k;
                row_edges.push(y);
                row_edges.push(y + site.y);
            }
        }
        row_edges.sort_unstable();
        row_edges.dedup();
        Ok(Self {
            core,
            site_width,
            row_edges,
            row_site_count: GridX::from_dbu_trunc(core.width(), site_width),
        })
    }

    /// The core, in absolute coordinates.
    pub const fn core(&self) -> Rect<Dbu> {
        self.core
    }

    pub const fn site_width(&self) -> Dbu {
        self.site_width
    }

    /// The sites across the core, as `Grid::getRowSiteCount`.
    pub const fn row_site_count(&self) -> GridX {
        self.row_site_count
    }

    /// The rows, one between each pair of adjacent row edges, as
    /// `Grid::getRowCount`.
    pub const fn row_count(&self) -> GridY {
        GridY::new(self.row_edges.len() as i64 - 1)
    }

    /// Whether `at` lies on the grid.
    pub fn contains(&self, at: GridPt) -> bool {
        (GridX::ZERO..self.row_site_count).contains(&at.x)
            && (GridY::ZERO..self.row_count()).contains(&at.y)
    }

    /// A rectangle relative to the core.
    pub fn relative(&self, r: Rect<Dbu>) -> Rect<Dbu> {
        Rect::new(r.min - self.core.min, r.max - self.core.min)
    }

    /// The site column at or left of `x`, truncating as `Grid::gridX`.
    pub const fn grid_x(&self, x: Dbu) -> GridX {
        GridX::from_dbu_trunc(x, self.site_width)
    }

    /// The first site column at or right of `x`, as `Grid::gridEndX`.
    pub fn grid_end_x(&self, x: Dbu) -> GridX {
        GridX::from_dbu_ceil(x, self.site_width)
    }

    /// The last row edge at or below `y`, as `Grid::gridSnapDownY`.
    pub fn snap_down_y(&self, y: Dbu) -> GridY {
        let above = self.row_edges.partition_point(|&e| e <= y);
        GridY::new(above.saturating_sub(1) as i64)
    }

    /// The first row edge at or above `y`, as `Grid::gridEndY`.
    pub fn end_y(&self, y: Dbu) -> GridY {
        GridY::new(self.row_edges.partition_point(|&e| e < y) as i64)
    }

    /// The nearer row edge to `y`, the upper on a tie, as `Grid::gridRoundY`.
    pub fn round_y(&self, y: Dbu) -> GridY {
        let below = self.snap_down_y(y);
        if below < self.row_count()
            && (self.row_y(below) - y).abs() >= (self.row_y(below + 1) - y).abs()
        {
            return below + 1;
        }
        below
    }

    /// The offset of row edge `index` from the core's bottom, the core's
    /// height past the last, as `Grid::gridYToDbu`.
    pub fn row_y(&self, index: GridY) -> Dbu {
        // NOTE: Grid::gridYToDbu gives the core's absolute top past the last
        // edge, though edges are relative; PARE gives its height.
        usize::try_from(index.get())
            .ok()
            .and_then(|i| self.row_edges.get(i))
            .copied()
            .unwrap_or_else(|| self.core.height())
    }

    /// How tall row `y` is, from its bottom edge to the next.
    pub fn row_height(&self, y: GridY) -> Dbu {
        self.row_y(y + 1) - self.row_y(y)
    }

    /// The grid cells a rectangle covers, ends excluded, as
    /// `Grid::gridCovering`.
    pub fn covering(&self, r: &Rect<Dbu>) -> GridRect {
        GridRect {
            min: GridPt::new(self.grid_x(r.min.x), self.snap_down_y(r.min.y)),
            max: GridPt::new(self.grid_end_x(r.max.x), self.end_y(r.max.y)),
        }
    }

    /// Where a cell of `size` at `at` goes: clamped into the core and
    /// rounded onto the site grid and a row, as `Opendp::legalPt` with a
    /// point.
    pub fn legal_pt(&self, at: Point<Dbu>, size: Point<Dbu>) -> Point<Dbu> {
        let core_x = at.x.clamp(
            Dbu::ZERO,
            (self.row_site_count.to_dbu(self.site_width) - size.x).max(Dbu::ZERO),
        );
        let column = GridX::from_dbu_round(core_x, self.site_width);
        // NOTE: diverges from OpenDP, which bounds this by the core's absolute top.
        let core_y =
            at.y.clamp(Dbu::ZERO, (self.core.height() - size.y).max(Dbu::ZERO));
        Point::new(
            column.to_dbu(self.site_width),
            self.row_y(self.round_y(core_y)),
        )
    }

    /// A cell of `size` at `at` moved past the nearest edge of `block`, as
    /// `Opendp::nearestBlockEdge`.
    pub fn nearest_block_edge(
        &self,
        at: Point<Dbu>,
        size: Point<Dbu>,
        block: &Rect<Dbu>,
    ) -> Point<Dbu> {
        let x_min = (at.x - block.min.x).abs();
        let x_max = (block.max.x - (at.x + size.x)).abs();
        let y_min = (at.y - block.min.y).abs();
        let y_max = (block.max.y - (at.y + size.y)).abs();
        let nearest = x_min.min(x_max).min(y_min).min(y_max);
        let to = if nearest == x_min {
            Point::new(block.min.x - size.x, at.y)
        } else if nearest == x_max {
            Point::new(block.max.x, at.y)
        } else if nearest == y_min {
            Point::new(at.x, block.min.y - size.y)
        } else {
            Point::new(at.x, block.max.y)
        };
        self.legal_pt(to, size)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pare_common::db::parser::{def, lef};
    use std::path::{Path, PathBuf};

    fn fixture(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../common/tests/fixtures")
            .join(name)
    }

    /// The tiny design's three rows, 2 um high, raised 2 um off the die's
    /// bottom, so the core runs from 2 to 8 um.
    fn raised_rows() -> RowGrid {
        let units = def::read_units(&fixture("tiny.def")).unwrap();
        let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
        let mut db = def::parse(&fixture("tiny.def"), library).unwrap();
        for row in &mut db.rows {
            row.origin.y += Dbu(2000);
        }
        RowGrid::new(&db, &core_rows(&db)).unwrap()
    }

    #[test]
    fn past_the_last_row_edge_lies_the_core_top_relative_to_the_core() {
        let rows = raised_rows();

        let past_last = rows.row_y(rows.row_count() + 1);

        assert_eq!(past_last, Dbu(6000));
    }
}
