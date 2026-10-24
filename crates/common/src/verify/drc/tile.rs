//! Checking a layout tile by tile, so that the geometry of only one tile is
//! built at a time.
//!
//! The design is cut into square tiles. Each tile is checked from the shapes
//! meeting its window, the tile grown by a halo at least as deep as any
//! rule reaches, so every pair a rule can relate within the tile is there.
//! A marker belongs to the tile holding its lower-left corner, which keeps
//! each violation to one tile; a piece cut short by a window's edge is never
//! judged for its area, and its false ends lie in the halo, outside the tile.
//! Tiles are checked in parallel; within a tile every check runs on one
//! thread, since a tile or a router's region is small and the checks of
//! many of them run at once.

use super::layout::{Layout, LayoutShape, Owner};
use super::model::Model;
use super::rules::Rules;
use super::{DrcReport, Findings, Marker, classify, run_checks};
use crate::db::indices::{CutLayerId, LayerId};
use crate::db::library::LayerRef;
use crate::geom::dbu::Dbu;
use crate::geom::point::Point;
use crate::geom::rect::Rect;
use rayon::prelude::*;

/// Checks `layout` in square tiles of `core` a side, each seeing `halo`
/// beyond its edges.
pub fn check_tiled(layout: &Layout, rules: &Rules, core: Dbu, halo: Dbu) -> DrcReport {
    let Some(grid) = layout.bounds().map(|b| Grid::from_bounds(b, core)) else {
        return DrcReport {
            owners: layout.owners().to_vec(),
            ..DrcReport::default()
        };
    };
    let tiles = grid.bucket(layout, halo);
    let found: Vec<Findings> = tiles
        .into_par_iter()
        .map(|tile| check_window(&tile.layout, rules, tile.core, tile.window))
        .collect();
    merged_report(found, layout.owners().to_vec())
}

/// Checks the tiles of `core` a side over `bounds` as [`check_tiled`] does.
///
/// Each tile's layout is built only when it is checked: `layout_of` gives
/// the shapes meeting a window, with owner indices into `owners`.
pub fn check_tiles_lazily(
    bounds: Rect<Dbu>,
    owners: Vec<Owner>,
    rules: &Rules,
    (core, halo): (Dbu, Dbu),
    layout_of: impl Fn(&Rect<Dbu>) -> Layout + Sync,
) -> DrcReport {
    let grid = Grid::from_bounds(bounds, core);
    let found: Vec<Findings> = (0..grid.nx * grid.ny)
        .into_par_iter()
        .map(|k| {
            let (core, window) = grid.tile_area(k, halo);
            check_window(&layout_of(&window), rules, core, window)
        })
        .collect();
    merged_report(found, owners)
}

fn merged_report(found: Vec<Findings>, owners: Vec<Owner>) -> DrcReport {
    let merged = found.into_iter().fold(Findings::default(), Findings::merge);
    DrcReport::from_findings(merged, owners)
}

/// Checks the shapes of `layout`, which are those meeting `window`, and
/// reports the markers whose lower-left corner lies in `core`.
pub fn check_window(
    layout: &Layout,
    rules: &Rules,
    core: Rect<Dbu>,
    window: Rect<Dbu>,
) -> Findings {
    let model = Model::in_window(layout, window);
    let mut found = run_checks(&model, rules);
    found.violations.retain(|m| owns(&core, m));
    found.fixed.retain(|m| owns(&core, m));
    classify(found, &model)
}

/// Whether the marker's lower-left corner falls in `core`, counting the
/// core's lower and left edges as its own.
fn owns(core: &Rect<Dbu>, m: &Marker) -> bool {
    let p = m.bbox.min;
    core.min.x <= p.x && p.x < core.max.x && core.min.y <= p.y && p.y < core.max.y
}

/// The tiles as a grid over the layout's shapes.
struct Grid {
    origin: Point<Dbu>,
    core: Dbu,
    nx: usize,
    ny: usize,
}

impl Grid {
    /// The grid of `core`-sized tiles covering `bounds`.
    fn from_bounds(bounds: Rect<Dbu>, core: Dbu) -> Self {
        let core = core.max(Dbu(1));
        let count = |span: Dbu| usize::try_from(span.get() / core.get() + 1).unwrap_or(1);
        Self {
            origin: bounds.min,
            core,
            nx: count(bounds.width()),
            ny: count(bounds.height()),
        }
    }

    /// The core of tile `k` and its window, the core grown by `halo`.
    fn tile_area(&self, k: usize, halo: Dbu) -> (Rect<Dbu>, Rect<Dbu>) {
        let (i, j) = (k % self.nx, k / self.nx);
        let min = self.origin + Point::new(self.core * i as i64, self.core * j as i64);
        let core = Rect::new(min, min + Point::new(self.core, self.core));
        (core, core.expand(halo))
    }

    /// The tiles, each holding the shapes that meet its window.
    fn bucket(&self, layout: &Layout, halo: Dbu) -> Vec<Tile> {
        let mut tiles: Vec<Tile> = (0..self.nx * self.ny)
            .map(|k| {
                let (core, window) = self.tile_area(k, halo);
                Tile {
                    core,
                    window,
                    layout: Layout::with_owners_of(layout),
                }
            })
            .collect();
        for l in 0..layout.metal_layers() {
            for s in layout.metal(l) {
                self.spread(s, halo, |t| {
                    tiles[t]
                        .layout
                        .add_shape(LayerRef::Routing(LayerId::new(l)), s);
                });
            }
        }
        for c in 0..layout.cut_layers() {
            for s in layout.cut(c) {
                self.spread(s, halo, |t| {
                    tiles[t]
                        .layout
                        .add_shape(LayerRef::Cut(CutLayerId::new(c)), s);
                });
            }
        }
        tiles
    }

    /// Calls `add` with the index of every tile whose window `shape` meets.
    fn spread(&self, shape: &LayoutShape, halo: Dbu, mut add: impl FnMut(usize)) {
        let grown = shape.rect.expand(halo);
        let (x0, x1) = self.span(grown.min.x, grown.max.x, self.origin.x, self.nx);
        let (y0, y1) = self.span(grown.min.y, grown.max.y, self.origin.y, self.ny);
        for j in y0..=y1 {
            for i in x0..=x1 {
                add(j * self.nx + i);
            }
        }
    }

    /// The tile columns or rows from `lo` to `hi`, clamped to the grid.
    fn span(&self, lo: Dbu, hi: Dbu, origin: Dbu, count: usize) -> (usize, usize) {
        let index = |v: Dbu| {
            usize::try_from((v - origin).get().div_euclid(self.core.get()))
                .unwrap_or(0)
                .min(count - 1)
        };
        (index(lo), index(hi))
    }
}

/// One tile: the area whose markers it reports and the shapes it sees.
struct Tile {
    core: Rect<Dbu>,
    window: Rect<Dbu>,
    layout: Layout,
}
