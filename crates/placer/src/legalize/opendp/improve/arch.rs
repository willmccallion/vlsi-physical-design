//! The rows the improver places in, as `Opendp::createArchitecture` and
//! `Architecture::postProcess` make them: the core rows of the shortest
//! site, those at one height merged into one, with fillers over the gaps.

use super::{ImprovementError, Network, Placer, Result};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::IdVec;
use pare_common::db::library::{SiteClass, Symmetry};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::interval::{Interval, merge_touching};
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

pare_common::define_index!(
    /// A row of the improver's architecture, counted from the bottom.
    ArchRowId(u32)
);

impl ArchRowId {
    /// The bottom row.
    pub(super) const BOTTOM: Self = Self::new(0);

    /// The row `rows` above this one.
    pub(super) const fn up(self, rows: usize) -> Self {
        Self::new(self.index() + rows)
    }

    /// The row `rows` below this one; none past the bottom row.
    pub(super) fn down(self, rows: usize) -> Option<Self> {
        self.index().checked_sub(rows).map(Self::new)
    }

    /// This row and the `count - 1` above it, bottom first.
    pub(super) fn stack(self, count: usize) -> impl Iterator<Item = Self> {
        (self.index()..self.index() + count).map(Self::new)
    }
}

/// A row, as `Architecture::Row`; positions are relative to the core.
#[derive(Clone, Copy, Debug)]
pub(super) struct Row {
    pub(super) left: Dbu,
    pub(super) bottom: Dbu,
    pub(super) height: Dbu,
    pub(super) site_spacing: Dbu,
    pub(super) site_width: Dbu,
    pub(super) num_sites: i64,
    pub(super) orient: Orientation,
    pub(super) symmetry: Symmetry,
}

impl Row {
    pub(super) fn right(&self) -> Dbu {
        self.left + self.site_spacing * self.num_sites
    }

    pub(super) fn top(&self) -> Dbu {
        self.bottom + self.height
    }

    fn bbox(&self) -> Rect<Dbu> {
        Rect::new(
            Point::new(self.left, self.bottom),
            Point::new(self.right(), self.top()),
        )
    }
}

#[derive(Debug)]
pub(super) struct Arch {
    /// The rows from the bottom up.
    pub(super) rows: IdVec<ArchRowId, Row>,
    pub(super) min_x: Dbu,
    pub(super) max_x: Dbu,
    pub(super) min_y: Dbu,
    pub(super) max_y: Dbu,
}

impl Arch {
    /// The improver's rows, as `Opendp::createArchitecture`.
    ///
    /// # Errors
    ///
    /// When no row is one the improver places in, or a row of several
    /// sites has no step between them.
    pub(super) fn new(db: &NetlistDB, grid: &Placer, network: &mut Network) -> Result<Self> {
        let mut rows = improver_rows(db, grid)?;
        let bounds = bounds_of(&rows)?;
        for row in &mut rows {
            let end_gap = row.site_width - row.site_spacing;
            row.left = row.left.max(bounds.min.x);
            if row.left + row.site_spacing * row.num_sites + end_gap > bounds.max.x {
                row.num_sites = (bounds.max.x - end_gap - row.left).div_trunc(row.site_spacing);
            }
        }
        rows.sort_by_key(|r| r.bottom);
        let bounds = bounds_of(&rows)?;
        let mut merged = merge_rows(&rows, bounds, network);
        merged.sort_by_key(|r| r.bottom);
        Ok(Self {
            rows: merged.into(),
            min_x: bounds.min.x,
            max_x: bounds.max.x,
            min_y: bounds.min.y,
            max_y: bounds.max.y,
        })
    }

    /// The row `rows` above `row`; none past the top row.
    pub(super) fn row_up(&self, row: ArchRowId, rows: usize) -> Option<ArchRowId> {
        (row.index() + rows < self.rows.len()).then(|| row.up(rows))
    }

    /// Row number `r` counted from the bottom; none outside the rows.
    pub(super) fn row_number(&self, r: i64) -> Option<ArchRowId> {
        usize::try_from(r)
            .ok()
            .filter(|&r| r < self.rows.len())
            .map(ArchRowId::new)
    }

    pub(super) fn width(&self) -> Dbu {
        self.max_x - self.min_x
    }

    pub(super) fn height(&self) -> Dbu {
        self.max_y - self.min_y
    }

    /// How many rows a cell of this height spans, as
    /// `Architecture::getCellHeightInRows`.
    pub(super) fn rows_spanned(&self, height: Dbu) -> usize {
        let rows = (height.to_f64() / self.rows[ArchRowId::BOTTOM].height.to_f64()).round();
        rows as usize
    }

    /// The row whose bottom is nearest `y`, the lower on a tie, as
    /// `Architecture::find_closest_row`.
    pub(super) fn closest_row(&self, y: Dbu) -> ArchRowId {
        let rows = self.rows.iter().as_slice();
        if y <= rows[0].bottom {
            return ArchRowId::BOTTOM;
        }
        let mut r = rows.partition_point(|row| row.bottom < y);
        if r == rows.len() || rows[r].bottom > y {
            r -= 1;
        }
        if r + 1 < rows.len() && (rows[r + 1].bottom - y).abs() < (rows[r].bottom - y).abs() {
            r += 1;
        }
        ArchRowId::new(r)
    }
}

/// The box around `rows`, as `Opendp::createArchitecture` and
/// `Architecture::postProcess` measure it.
fn bounds_of(rows: &[Row]) -> Result<Rect<Dbu>> {
    rows.iter()
        .map(Row::bbox)
        .reduce(|a, b| a.union(&b))
        .ok_or(ImprovementError::NoRows)
}

/// `rows`, sorted by height, merged at each height into one spanning them
/// all, with fillers closing the gaps between and around them within
/// `bounds`, as `Architecture::postProcess`.
fn merge_rows(rows: &[Row], bounds: Rect<Dbu>, network: &mut Network) -> Vec<Row> {
    let mut merged = Vec::new();
    let mut r = 0;
    while r < rows.len() {
        let start = r;
        while r < rows.len() && rows[r].bottom == rows[start].bottom {
            r += 1;
        }
        let mut spans: Vec<Interval> = rows[start..r]
            .iter()
            .map(|row| Interval::new(row.left, row.right()))
            .collect();
        spans.sort_unstable();
        let spans = merge_touching(spans);
        let mut row = rows[start];
        if r - start > 1 {
            let (lx, rx) = (spans[0].lo, spans[spans.len() - 1].hi);
            row.num_sites = (rx - lx).div_trunc(row.site_spacing);
            row.left = lx;
        }
        add_gap_fillers(network, &row, &spans, bounds);
        merged.push(row);
    }
    merged
}

/// Fills the gaps of `row` outside `spans`, the merged stretches of the
/// rows at its height, out to `bounds`.
fn add_gap_fillers(network: &mut Network, row: &Row, spans: &[Interval], bounds: Rect<Dbu>) {
    let (min_x, max_x) = (bounds.min.x, bounds.max.x);
    if min_x < spans[0].lo {
        network.add_filler(min_x, row.bottom, spans[0].lo - min_x, row.height);
    }
    for pair in spans.windows(2) {
        if pair[1].lo > pair[0].hi {
            network.add_filler(pair[0].hi, row.bottom, pair[1].lo - pair[0].hi, row.height);
        }
    }
    let last = spans[spans.len() - 1].hi;
    if max_x > last {
        network.add_filler(last, row.bottom, max_x - last, row.height);
    }
}

/// The core's horizontal rows of the shortest site, relative to the core,
/// as `Opendp::createArchitecture` takes them.
fn improver_rows(db: &NetlistDB, grid: &Placer) -> Result<Vec<Row>> {
    let core = grid.rows.core();
    let min_row_height = db.rows.iter().map(|r| db.library.site(r.site).size.y).min();
    let mut rows = Vec::new();
    for r in &db.rows {
        let site = db.library.site(r.site);
        if site.class == SiteClass::Pad
            || r.num_y != 1
            || min_row_height.is_some_and(|min| site.size.y > min)
        {
            continue;
        }
        rows.push(Row {
            left: r.origin.x - core.min.x,
            bottom: r.origin.y - core.min.y,
            height: site.size.y,
            site_spacing: site_spacing(r, site.size.x)?,
            site_width: site.size.x,
            num_sites: i64::from(r.num_x),
            orient: r.orient,
            symmetry: site.symmetry,
        });
    }
    Ok(rows)
}

/// The distance between a row's sites; a lone site is as wide as itself.
// NOTE: Opendp::createArchitecture takes the row's spacing, which a lone
// site written STEP 0 0 has as zero, leaving a row no wider than a point.
fn site_spacing(row: &pare_common::db::design::Row, site_width: Dbu) -> Result<Dbu> {
    if row.num_x == 1 {
        return Ok(site_width);
    }
    if row.step.x <= Dbu::ZERO {
        return Err(ImprovementError::RowWithoutStep(row.name.clone()));
    }
    Ok(row.step.x)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ROW_HEIGHT: Dbu = Dbu(100);

    /// Rows `ROW_HEIGHT` high stacked from the bottom at zero.
    fn rows(count: i64) -> Arch {
        let rows: Vec<Row> = (0..count)
            .map(|r| Row {
                left: Dbu(0),
                bottom: ROW_HEIGHT * r,
                height: ROW_HEIGHT,
                site_spacing: Dbu(10),
                site_width: Dbu(10),
                num_sites: 10,
                orient: Orientation::N,
                symmetry: Symmetry::default(),
            })
            .collect();
        Arch {
            rows: rows.into(),
            min_x: Dbu(0),
            max_x: Dbu(100),
            min_y: Dbu(0),
            max_y: ROW_HEIGHT * count,
        }
    }

    #[test]
    fn a_point_below_the_rows_is_closest_to_the_first() {
        assert_eq!(rows(3).closest_row(Dbu(-5)), ArchRowId::new(0));
    }

    #[test]
    fn a_point_above_the_rows_is_closest_to_the_last() {
        assert_eq!(rows(3).closest_row(Dbu(500)), ArchRowId::new(2));
    }

    #[test]
    fn a_point_is_closest_to_the_row_whose_bottom_is_nearest() {
        let arch = rows(3);

        assert_eq!(
            [40, 60, 100, 160].map(|y| arch.closest_row(Dbu(y))),
            [0, 1, 1, 2].map(ArchRowId::new)
        );
    }

    #[test]
    fn a_point_halfway_between_rows_is_closest_to_the_lower() {
        assert_eq!(rows(3).closest_row(Dbu(150)), ArchRowId::new(1));
    }

    #[test]
    fn a_cell_spans_its_height_in_rows_rounded_half_up() {
        let arch = rows(3);

        assert_eq!(
            [100, 140, 150, 200].map(|h| arch.rows_spanned(Dbu(h))),
            [1, 1, 2, 2]
        );
    }
}

#[cfg(test)]
mod design_tests {
    use super::super::{Network, Placer};
    use super::*;
    use pare_common::db::parser::{def, lef};
    use std::path::{Path, PathBuf};

    fn fixture(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../common/tests/fixtures")
            .join(name)
    }

    /// The improver's rows of the tiny design with its top row as `row`.
    fn arch_with_top_row(row: &str) -> Result<Arch> {
        let text = std::fs::read_to_string(fixture("tiny.def"))
            .unwrap()
            .replace("ROW ROW_2 core 0 4000 N DO 20 BY 1 STEP 200 0 ;", row);
        let units = def::read_units(&fixture("tiny.def")).unwrap();
        let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
        let db = def::parse_str(&text, &fixture("tiny.def"), library).unwrap();
        let grid = Placer::new(&db).unwrap();
        let mut network = Network::new(&db, &grid).unwrap();
        Arch::new(&db, &grid, &mut network)
    }

    #[test]
    fn a_one_site_row_without_a_step_spans_its_site() {
        let arch = arch_with_top_row("ROW ROW_2 core 0 4000 N DO 1 BY 1 STEP 0 0 ;").unwrap();

        let top = *arch.rows.iter().last().unwrap();

        assert_eq!((top.left, top.right()), (Dbu(0), Dbu(200)));
    }

    #[test]
    fn a_row_of_sites_without_a_step_is_an_error() {
        let arch = arch_with_top_row("ROW ROW_2 core 0 4000 N DO 20 BY 1 STEP 0 0 ;");

        assert_eq!(
            arch.map(|_| ()),
            Err(ImprovementError::RowWithoutStep("ROW_2".into()))
        );
    }
}
