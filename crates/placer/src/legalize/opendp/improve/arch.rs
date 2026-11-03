//! The rows the improver places in, as `Opendp::createArchitecture` and
//! `Architecture::postProcess` make them: the core rows of the shortest
//! site, those at one height merged into one, with fillers over the gaps.

use super::{Network, Placer};
use pare_common::db::design::NetlistDB;
use pare_common::db::library::{SiteClass, Symmetry};
use pare_common::geom::orient::Orientation;

/// A row, as `Architecture::Row`; positions are relative to the core.
#[derive(Clone, Copy, Debug)]
pub(super) struct Row {
    pub(super) left: i64,
    pub(super) bottom: i64,
    pub(super) height: i64,
    pub(super) site_spacing: i64,
    pub(super) site_width: i64,
    pub(super) num_sites: i64,
    pub(super) orient: Orientation,
    pub(super) symmetry: Symmetry,
}

impl Row {
    pub(super) const fn right(&self) -> i64 {
        self.left + self.num_sites * self.site_spacing
    }

    pub(super) const fn top(&self) -> i64 {
        self.bottom + self.height
    }
}

#[derive(Debug)]
pub(super) struct Arch {
    /// The rows from the bottom up.
    pub(super) rows: Vec<Row>,
    pub(super) min_x: i64,
    pub(super) max_x: i64,
    pub(super) min_y: i64,
    pub(super) max_y: i64,
}

impl Arch {
    pub(super) fn new(db: &NetlistDB, grid: &Placer, network: &mut Network) -> Self {
        let core = grid.core;
        let min_row_height = db
            .rows
            .iter()
            .map(|r| db.library.site(r.site).size.y.get())
            .min()
            .unwrap_or(i64::MAX);
        let mut rows = Vec::new();
        for r in &db.rows {
            let site = db.library.site(r.site);
            if site.class == SiteClass::Pad || r.num_y != 1 || site.size.y.get() > min_row_height {
                continue;
            }
            rows.push(Row {
                left: (r.origin.x - core.min.x).get(),
                bottom: (r.origin.y - core.min.y).get(),
                height: site.size.y.get(),
                site_spacing: r.step.x.get(),
                site_width: site.size.x.get(),
                num_sites: i64::from(r.num_x),
                orient: r.orient,
                symmetry: site.symmetry,
            });
        }
        let mut arch = Self {
            rows,
            min_x: i64::MAX,
            max_x: i64::MIN,
            min_y: i64::MAX,
            max_y: i64::MIN,
        };
        arch.find_bounds();
        for row in &mut arch.rows {
            let end_gap = row.site_width - row.site_spacing;
            row.left = row.left.max(arch.min_x);
            if row.left + row.num_sites * row.site_spacing + end_gap > arch.max_x {
                row.num_sites = (arch.max_x - end_gap - row.left) / row.site_spacing;
            }
        }
        arch.merge_rows(network);
        arch
    }

    fn find_bounds(&mut self) {
        for row in &self.rows {
            self.min_x = self.min_x.min(row.left);
            self.max_x = self.max_x.max(row.right());
            self.min_y = self.min_y.min(row.bottom);
            self.max_y = self.max_y.max(row.top());
        }
    }

    /// Merges the rows at each height into one spanning them all and closes
    /// the gaps between and around them with fillers, as
    /// `Architecture::postProcess`.
    fn merge_rows(&mut self, network: &mut Network) {
        self.rows.sort_by_key(|r| r.bottom);
        self.min_x = i64::MAX;
        self.max_x = i64::MIN;
        self.min_y = i64::MAX;
        self.max_y = i64::MIN;
        self.find_bounds();
        let mut merged = Vec::new();
        let mut r = 0;
        while r < self.rows.len() {
            let start = r;
            while r < self.rows.len() && self.rows[r].bottom == self.rows[start].bottom {
                r += 1;
            }
            let mut spans: Vec<(i64, i64)> = self.rows[start..r]
                .iter()
                .map(|row| (row.left, row.right()))
                .collect();
            spans.sort_unstable();
            let spans = merge_intervals(&spans);
            let mut row = self.rows[start];
            if r - start > 1 {
                let (lx, rx) = (spans[0].0, spans[spans.len() - 1].1);
                row.num_sites = (rx - lx) / row.site_spacing;
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
    }

    pub(super) const fn width(&self) -> i64 {
        self.max_x - self.min_x
    }

    pub(super) const fn height(&self) -> i64 {
        self.max_y - self.min_y
    }

    /// How many rows a cell of this height spans, as
    /// `Architecture::getCellHeightInRows`.
    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)] // rows fit
    pub(super) fn rows_spanned(&self, height: i64) -> usize {
        let rows = (height as f64 / self.rows[0].height as f64).round();
        usize::try_from(rows as i64).unwrap_or(0)
    }

    /// The row whose bottom is nearest `y`, the lower on a tie, as
    /// `Architecture::find_closest_row`.
    pub(super) fn closest_row(&self, y: i64) -> usize {
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

/// Sorted intervals with the overlapping and touching ones merged, as the
/// stack merges `OpenDP` makes of rows, blockages and regions.
pub(super) fn merge_intervals(sorted: &[(i64, i64)]) -> Vec<(i64, i64)> {
    let mut merged: Vec<(i64, i64)> = Vec::with_capacity(sorted.len());
    for &(lo, hi) in sorted {
        match merged.last_mut() {
            Some(top) if top.1 >= lo => top.1 = top.1.max(hi),
            _ => merged.push((lo, hi)),
        }
    }
    merged
}
