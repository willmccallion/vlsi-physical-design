//! The rows the improver places in, as `Opendp::createArchitecture` and
//! `Architecture::postProcess` make them: the core rows of the shortest
//! site, those at one height merged into one, with fillers over the gaps.

use super::{ImprovementError, Network, Placer, Result};
use pare_common::db::design::NetlistDB;
use pare_common::db::library::{SiteClass, Symmetry};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

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
    pub(super) rows: Vec<Row>,
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
        let mut arch = Self {
            rows: improver_rows(db, grid)?,
            min_x: Dbu::ZERO,
            max_x: Dbu::ZERO,
            min_y: Dbu::ZERO,
            max_y: Dbu::ZERO,
        };
        arch.find_bounds()?;
        for row in &mut arch.rows {
            let end_gap = row.site_width - row.site_spacing;
            row.left = row.left.max(arch.min_x);
            if row.left + row.site_spacing * row.num_sites + end_gap > arch.max_x {
                row.num_sites = (arch.max_x - end_gap - row.left).get() / row.site_spacing.get();
            }
        }
        arch.merge_rows(network)?;
        Ok(arch)
    }

    /// The box around the rows, as `Opendp::createArchitecture` and
    /// `Architecture::postProcess` measure it.
    fn find_bounds(&mut self) -> Result<()> {
        let bounds = self
            .rows
            .iter()
            .map(Row::bbox)
            .reduce(|a, b| a.union(&b))
            .ok_or(ImprovementError::NoRows)?;
        self.min_x = bounds.min.x;
        self.max_x = bounds.max.x;
        self.min_y = bounds.min.y;
        self.max_y = bounds.max.y;
        Ok(())
    }

    /// Merges the rows at each height into one spanning them all and closes
    /// the gaps between and around them with fillers, as
    /// `Architecture::postProcess`.
    fn merge_rows(&mut self, network: &mut Network) -> Result<()> {
        self.rows.sort_by_key(|r| r.bottom);
        self.find_bounds()?;
        let mut merged = Vec::new();
        let mut r = 0;
        while r < self.rows.len() {
            let start = r;
            while r < self.rows.len() && self.rows[r].bottom == self.rows[start].bottom {
                r += 1;
            }
            let mut spans: Vec<(Dbu, Dbu)> = self.rows[start..r]
                .iter()
                .map(|row| (row.left, row.right()))
                .collect();
            spans.sort_unstable();
            let spans = merge_intervals(&spans);
            let mut row = self.rows[start];
            if r - start > 1 {
                let (lx, rx) = (spans[0].0, spans[spans.len() - 1].1);
                row.num_sites = (rx - lx).get() / row.site_spacing.get();
                row.left = lx;
            }
            if self.min_x < spans[0].0 {
                network.add_filler(self.min_x, row.bottom, spans[0].0 - self.min_x, row.height);
            }
            for pair in spans.windows(2) {
                if pair[1].0 > pair[0].1 {
                    network.add_filler(pair[0].1, row.bottom, pair[1].0 - pair[0].1, row.height);
                }
            }
            let last = spans[spans.len() - 1].1;
            if self.max_x > last {
                network.add_filler(last, row.bottom, self.max_x - last, row.height);
            }
            merged.push(row);
        }
        self.rows = merged;
        self.rows.sort_by_key(|r| r.bottom);
        Ok(())
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
        let rows = (height.to_f64() / self.rows[0].height.to_f64()).round();
        rows as usize
    }

    /// The row whose bottom is nearest `y`, the lower on a tie, as
    /// `Architecture::find_closest_row`.
    pub(super) fn closest_row(&self, y: Dbu) -> usize {
        if y <= self.rows[0].bottom {
            return 0;
        }
        let mut r = self.rows.partition_point(|row| row.bottom < y);
        if r == self.rows.len() || self.rows[r].bottom > y {
            r -= 1;
        }
        if r + 1 < self.rows.len()
            && (self.rows[r + 1].bottom - y).abs() < (self.rows[r].bottom - y).abs()
        {
            r += 1;
        }
        r
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

/// Sorted intervals with the overlapping and touching ones merged, as the
/// stack merges `OpenDP` makes of rows, blockages and regions.
pub(super) fn merge_intervals(sorted: &[(Dbu, Dbu)]) -> Vec<(Dbu, Dbu)> {
    let mut merged: Vec<(Dbu, Dbu)> = Vec::with_capacity(sorted.len());
    for &(lo, hi) in sorted {
        match merged.last_mut() {
            Some(top) if top.1 >= lo => top.1 = top.1.max(hi),
            _ => merged.push((lo, hi)),
        }
    }
    merged
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
            rows,
            min_x: Dbu(0),
            max_x: Dbu(100),
            min_y: Dbu(0),
            max_y: ROW_HEIGHT * count,
        }
    }

    #[test]
    fn overlapping_and_touching_intervals_merge_and_apart_ones_stay() {
        let spans = |s: &[(i64, i64)]| -> Vec<(Dbu, Dbu)> {
            s.iter().map(|&(lo, hi)| (Dbu(lo), Dbu(hi))).collect()
        };

        let merged = merge_intervals(&spans(&[(0, 2), (1, 3), (3, 4), (6, 7)]));

        assert_eq!(merged, spans(&[(0, 4), (6, 7)]));
    }

    #[test]
    fn no_intervals_merge_into_none() {
        assert_eq!(merge_intervals(&[]), []);
    }

    #[test]
    fn a_point_below_the_rows_is_closest_to_the_first() {
        assert_eq!(rows(3).closest_row(Dbu(-5)), 0);
    }

    #[test]
    fn a_point_above_the_rows_is_closest_to_the_last() {
        assert_eq!(rows(3).closest_row(Dbu(500)), 2);
    }

    #[test]
    fn a_point_is_closest_to_the_row_whose_bottom_is_nearest() {
        let arch = rows(3);

        assert_eq!(
            [40, 60, 100, 160].map(|y| arch.closest_row(Dbu(y))),
            [0, 1, 1, 2]
        );
    }

    #[test]
    fn a_point_halfway_between_rows_is_closest_to_the_lower() {
        assert_eq!(rows(3).closest_row(Dbu(150)), 1);
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

        let top = arch.rows[arch.rows.len() - 1];

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
