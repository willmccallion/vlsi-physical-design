//! Detailed placement as `OpenDP`'s `detailed_placement` legalizes a
//! placement.
//!
//! Every movable standard cell is taken off the rows and placed again, the
//! biggest first and then those nearest the core's center: each goes to the
//! free site nearest the one its position rounds to, found by a search that
//! widens in Manhattan distance up to 500 sites and 100 rows away; a cell
//! that finds none rips up the cells around it, takes its spot and puts them
//! back. A spot is free when its sites belong to rows of the cell's site,
//! hold no other cell, suit the cell's symmetry in the row's orientation and
//! break no placement rule: cell edge spacing, power stripes on metal2 and
//! metal3 over pins that need those layers, and one-site gaps where the
//! library has no one-site cell to fill them. Cells take their row's
//! orientation. The members of each placement region are placed first,
//! inside it, and other cells kept out of it. Hybrid rows and placement
//! padding, which the designs here do not use, are not modelled.

mod cpp;
mod groups;
pub mod mirror;

use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, SiteId};
use pare_common::db::library::{LayerRef, MacroClass, SiteClass, Symmetry};
use pare_common::db::rules::{CellSide, EdgeType};
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::cmp::Reverse;
use std::collections::HashMap;
use std::collections::{BTreeSet, BinaryHeap};

/// Sites a cell may move horizontally, as `detailed_placement`'s default.
const MAX_DISPLACEMENT_X: i64 = 500;
/// Rows a cell may move vertically, as `detailed_placement`'s default.
const MAX_DISPLACEMENT_Y: i64 = 100;
/// Rows around a cell that fails to place whose cells are ripped up.
const RIP_UP_ROWS: i64 = 3;

/// Why detailed placement failed.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum DetailedPlacementError {
    /// The design has no core rows.
    #[error("no rows defined in design")]
    NoRows,
    /// The rows are made of sites of different widths.
    #[error("site widths are not equal: {0} != {1}")]
    SiteWidths(i64, i64),
    /// A cell is wider or taller than the core.
    #[error("instance {0} does not fit inside the ROW core area")]
    DoesNotFit(String),
    /// Some cells found no free spot.
    #[error("detailed placement failed on {} instances, the first {}", .0.len(), .0.first().map_or("", String::as_str))]
    Failed(Vec<String>),
}

/// What detailed placement did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DetailedPlacement {
    /// Cells placed.
    pub cells: usize,
    /// Cells the search placed at the first attempt.
    pub diamond_moves: usize,
    /// Cells placed after ripping up their neighbours.
    pub rip_up_moves: usize,
    /// The sum of every cell's displacement.
    pub total_displacement: Dbu,
    /// The largest displacement.
    pub max_displacement: Dbu,
}

/// Legalizes every movable standard cell of `db` as `detailed_placement`
/// does.
///
/// # Errors
///
/// When the design has no rows, rows of different site widths, a cell bigger
/// than the core, or a cell no free spot within reach fits.
pub fn detailed_placement(db: &mut NetlistDB) -> Result<DetailedPlacement, DetailedPlacementError> {
    let mut placer = Placer::new(db)?;
    placer.paint_fixed_cells();
    placer.mark_region_edges();
    placer.mark_region_sites();
    if !placer.groups.is_empty() {
        placer.place_groups()?;
    }
    let summary = placer.place()?;
    placer.write_back(db);
    Ok(summary)
}

/// A grid coordinate: a site column and a row-edge index.
type GridPt = (i64, i64);

const NO_CELL: u32 = u32::MAX;
/// The fixed stand-in `OpenDP` puts on sites a region only partly covers.
const DUMMY_CELL: u32 = u32::MAX - 1;
const NO_GROUP: u32 = u32::MAX;

/// One site of one row band, as `OpenDP`'s `Pixel`.
#[derive(Clone, Copy, Debug)]
struct Pixel {
    cell: u32,
    group: u32,
    /// The share of the site a region covers, as `Pixel::util` while
    /// regions are laid on the grid.
    util: f32,
    valid: bool,
    hopeless: bool,
    blocked_layers: u8,
}

/// A cell as the legalizer sees it, as `OpenDP`'s `Node`.
#[derive(Clone, Debug)]
#[allow(clippy::struct_excessive_bools)] // each is one of the node's independent flags
struct Node {
    cell: CellId,
    name: String,
    fixed: bool,
    placed: bool,
    /// Placed early, which later passes leave alone.
    hold: bool,
    group: Option<usize>,
    /// The rectangle of its group's region it belongs to.
    region: Option<usize>,
    left: i64,
    bottom: i64,
    width: i64,
    height: i64,
    orient: Orientation,
    /// Where the design had it, relative to the core.
    init: (i64, i64),
    /// Whether it is a standard cell or an end cap, which get written back.
    std_cell: bool,
    /// Whether its master is of class `CORE`, which gets placed.
    core: bool,
    block: bool,
    multi_row: bool,
    site: Option<SiteId>,
    symmetry: Symmetry,
    used_layers: u8,
    /// Its edges with their edge types, in its master's frame.
    edges: Vec<(usize, Rect<Dbu>)>,
    master_size: (i64, i64),
}

/// A row's stretch of sites with its site and orientation.
#[derive(Clone, Copy, Debug)]
struct RowSites {
    x0: i64,
    x1: i64,
    site: SiteId,
    orient: Orientation,
}

/// The cell edge spacing table, as `PlacementDRC` builds it.
#[derive(Debug, Default)]
struct EdgeSpacing {
    index: HashMap<String, usize>,
    table: Vec<Vec<i64>>,
}

impl EdgeSpacing {
    fn new(db: &NetlistDB) -> Self {
        let mut spacing = Self::default();
        let rules = &db.library.rules.cell_edge_spacing;
        for rule in rules {
            for t in [&rule.first, &rule.second] {
                let next = spacing.index.len();
                let _ = spacing.index.entry(t.0.clone()).or_insert(next);
            }
        }
        let n = spacing.index.len();
        spacing.table = vec![vec![0; n]; n];
        for rule in rules {
            let a = spacing.index[&rule.first.0];
            let b = spacing.index[&rule.second.0];
            spacing.table[a][b] = rule.spacing.get();
            spacing.table[b][a] = rule.spacing.get();
        }
        spacing
    }

    const fn is_empty(&self) -> bool {
        self.table.is_empty()
    }

    fn type_index(&self, t: &EdgeType) -> Option<usize> {
        self.index.get(&t.0).copied()
    }

    fn max_spacing(&self, t: usize) -> i64 {
        self.table[t].iter().copied().max().unwrap_or(0)
    }
}

struct Placer {
    core: Rect<Dbu>,
    site_width: i64,
    /// Bottom and top of every row, relative to the core, ascending.
    row_edges: Vec<i64>,
    row_site_count: i64,
    row_count: i64,
    pixels: Vec<Pixel>,
    row_sites: Vec<Vec<RowSites>>,
    nodes: Vec<Node>,
    edge_spacing: EdgeSpacing,
    disallow_one_site_gaps: bool,
    search: SearchMarks,
    groups: Vec<groups::Group>,
    /// Every region rectangle with its top and right edges pulled in by one,
    /// as `OpenDP`'s region R-tree holds them.
    region_boxes: Vec<Rect<Dbu>>,
}

impl Placer {
    fn new(db: &NetlistDB) -> Result<Self, DetailedPlacementError> {
        let core = db.core_area();
        let rows: Vec<_> = db
            .rows
            .iter()
            .filter(|r| db.library.site(r.site).class != SiteClass::Pad)
            .collect();
        let first = rows.first().ok_or(DetailedPlacementError::NoRows)?;
        let site_width = db.library.site(first.site).size.x.get();
        let mut row_edges = Vec::new();
        for r in &rows {
            let width = db.library.site(r.site).size.x.get();
            if width != site_width {
                return Err(DetailedPlacementError::SiteWidths(width, site_width));
            }
            let bottom = r.origin.y.get() - core.min.y.get();
            let height = db.library.site(r.site).size.y.get();
            for k in 0..i64::from(r.num_y) {
                let y = bottom + r.step.y.get() * k;
                row_edges.push(y);
                row_edges.push(y + height);
            }
        }
        row_edges.sort_unstable();
        row_edges.dedup();
        let row_site_count = core.width().get() / site_width;
        let row_count = i64::try_from(row_edges.len()).unwrap_or(1) - 1;
        let uniform_row_height = uniform_row_height(db, &rows);
        let edge_spacing = EdgeSpacing::new(db);
        let mut placer = Self {
            core,
            site_width,
            row_edges,
            row_site_count,
            row_count,
            pixels: vec![
                Pixel {
                    cell: NO_CELL,
                    group: NO_GROUP,
                    util: 0.0,
                    valid: false,
                    hopeless: false,
                    blocked_layers: 0,
                };
                usize::try_from(row_site_count * row_count).unwrap_or(0)
            ],
            row_sites: vec![Vec::new(); usize::try_from(row_count).unwrap_or(0)],
            nodes: Vec::new(),
            edge_spacing,
            disallow_one_site_gaps: !has_one_site_master(db),
            search: SearchMarks::new(),
            groups: Vec::new(),
            region_boxes: Vec::new(),
        };
        placer.mark_rows(db, &rows);
        placer.mark_blocked(db);
        placer.nodes = placer.make_nodes(db, &rows, uniform_row_height);
        for node in &mut placer.nodes {
            if !node.fixed {
                node.placed = false;
            }
        }
        placer.make_groups(db);
        Ok(placer)
    }

    fn pixel_index(&self, x: i64, y: i64) -> Option<usize> {
        (x >= 0 && x < self.row_site_count && y >= 0 && y < self.row_count)
            .then(|| usize::try_from(y * self.row_site_count + x).unwrap_or(0))
    }

    fn pixel(&self, x: i64, y: i64) -> Option<&Pixel> {
        self.pixel_index(x, y).map(|i| &self.pixels[i])
    }

    fn pixel_mut(&mut self, x: i64, y: i64) -> Option<&mut Pixel> {
        self.pixel_index(x, y).map(move |i| &mut self.pixels[i])
    }

    /// The site column at or left of `x`, truncating as `Grid::gridX`.
    const fn grid_x(&self, x: i64) -> i64 {
        x / self.site_width
    }

    /// The first site column at or right of `x`, as `Grid::gridEndX`.
    fn grid_end_x(&self, x: i64) -> i64 {
        div_ceil(x, self.site_width)
    }

    /// The last row edge at or below `y`, as `Grid::gridSnapDownY`.
    fn snap_down_y(&self, y: i64) -> i64 {
        let above = self.row_edges.partition_point(|&e| e <= y);
        i64::try_from(above.saturating_sub(1)).unwrap_or(0)
    }

    /// The first row edge at or above `y`, as `Grid::gridEndY`.
    fn end_y(&self, y: i64) -> i64 {
        i64::try_from(self.row_edges.partition_point(|&e| e < y)).unwrap_or(0)
    }

    /// The nearer row edge to `y`, the upper on a tie, as `Grid::gridRoundY`.
    fn round_y(&self, y: i64) -> i64 {
        let below = self.snap_down_y(y);
        if below < self.row_count
            && (self.row_y(below) - y).abs() >= (self.row_y(below + 1) - y).abs()
        {
            return below + 1;
        }
        below
    }

    /// The offset of row edge `index`, the core's top past the last, as
    /// `Grid::gridYToDbu`.
    fn row_y(&self, index: i64) -> i64 {
        usize::try_from(index)
            .ok()
            .and_then(|i| self.row_edges.get(i))
            .copied()
            .unwrap_or_else(|| self.core.max.y.get())
    }

    fn grid_width(&self, node: &Node) -> i64 {
        div_ceil(node.width, self.site_width)
    }

    /// The orientation of the row at `(x, y)` made of `site`, as
    /// `Grid::getSiteOrientation`.
    fn site_orientation(&self, x: i64, y: i64, site: Option<SiteId>) -> Option<Orientation> {
        let row = self.row_sites.get(usize::try_from(y).ok()?)?;
        row.iter()
            .find(|s| s.x0 <= x && x < s.x1 && Some(s.site) == site)
            .map(|s| s.orient)
    }

    /// Marks the sites of every row valid and the sites too far from any row
    /// for the search hopeless, as `Grid::markHopeless`.
    fn mark_rows(&mut self, db: &NetlistDB, rows: &[&pare_common::db::design::Row]) {
        const SAFETY: i64 = 20;
        let mut reachable = vec![false; self.pixels.len()];
        for r in rows {
            let height = db.library.site(r.site).size.y.get();
            for k in 0..i64::from(r.num_y) {
                let origin_y = r.origin.y.get() + r.step.y.get() * k;
                let x_start = (r.origin.x.get() - self.core.min.x.get()) / self.site_width;
                let x_end = x_start + i64::from(r.num_x);
                let y_row = self.snap_down_y(origin_y - self.core.min.y.get());
                for x in x_start..x_end {
                    if let Some(p) = self.pixel_mut(x, y_row) {
                        p.valid = true;
                    }
                }
                if let Some(row) = usize::try_from(y_row)
                    .ok()
                    .and_then(|y| self.row_sites.get_mut(y))
                {
                    row.push(RowSites {
                        x0: x_start,
                        x1: x_end,
                        site: r.site,
                        orient: r.orient,
                    });
                }
                let _ = height;
                let xl = (x_start - MAX_DISPLACEMENT_X + SAFETY).max(0);
                let xh = (x_end + MAX_DISPLACEMENT_X - SAFETY).min(self.row_site_count);
                let yl = (y_row - MAX_DISPLACEMENT_Y + SAFETY).max(0);
                let yh = (y_row + MAX_DISPLACEMENT_Y - SAFETY).min(self.row_count);
                for y in yl..yh {
                    for x in xl..xh {
                        if let Some(i) = self.pixel_index(x, y) {
                            reachable[i] = true;
                        }
                    }
                }
            }
        }
        for (p, reachable) in self.pixels.iter_mut().zip(reachable) {
            p.hopeless = !reachable;
        }
    }

    /// Marks the sites under power stripes on metal2 and metal3 and under
    /// hard placement blockages, as `Grid::markBlocked`.
    fn mark_blocked(&mut self, db: &NetlistDB) {
        let mut stripes: Vec<(usize, Rect<Dbu>)> = Vec::new();
        for net in &db.special_nets {
            for wire in &net.wires {
                if wire.shape.as_deref() != Some("DRCFILL") {
                    stripes.push((wire.layer.index() + 1, wire.rect()));
                }
            }
            for rect in &net.rects {
                if let LayerRef::Routing(layer) = rect.layer
                    && rect.shape.as_deref() != Some("DRCFILL")
                {
                    stripes.push((layer.index() + 1, rect.rect));
                }
            }
        }
        for (level, rect) in stripes {
            if !(2..=3).contains(&level) || rect.width() >= rect.height() {
                continue;
            }
            let (x0, y0, x1, y1) = self.covering(&self.relative(rect));
            for y in y0.max(0)..y1.min(self.row_count) {
                for x in x0.max(0)..x1.min(self.row_site_count) {
                    if let Some(p) = self.pixel_mut(x, y) {
                        p.blocked_layers |= 1 << level;
                    }
                }
            }
        }
        for blockage in &db.placement_blockages {
            let (x0, y0, x1, y1) = self.covering(&self.relative(*blockage));
            for y in y0.max(0)..y1.min(self.row_count) {
                for x in x0.max(0)..x1.min(self.row_site_count) {
                    if let Some(p) = self.pixel_mut(x, y) {
                        p.valid = false;
                    }
                }
            }
        }
    }

    fn relative(&self, r: Rect<Dbu>) -> Rect<Dbu> {
        Rect::new(r.min - self.core.min, r.max - self.core.min)
    }

    /// The grid cells a rectangle covers, ends excluded, as
    /// `Grid::gridCovering`.
    fn covering(&self, r: &Rect<Dbu>) -> (i64, i64, i64, i64) {
        (
            self.grid_x(r.min.x.get()),
            self.snap_down_y(r.min.y.get()),
            self.grid_end_x(r.max.x.get()),
            self.end_y(r.max.y.get()),
        )
    }

    /// The legalizer's nodes for the placeable cells, by name, as
    /// `Opendp::createNetwork`: fixed cells only where they meet the rows.
    fn make_nodes(
        &self,
        db: &NetlistDB,
        rows: &[&pare_common::db::design::Row],
        uniform_row_height: Option<i64>,
    ) -> Vec<Node> {
        let shell = RowShell::new(db, rows);
        let mut order: Vec<usize> = (0..db.cells.len())
            .filter(|&i| !db.cells[i].retired)
            .collect();
        order.sort_by(|&a, &b| db.cells[a].name.cmp(&db.cells[b].name));
        let mut nodes = Vec::new();
        for i in order {
            let id = CellId::new(i);
            let c = &db.cells[i];
            let m = db.cell_macro(id);
            let placeable = matches!(
                m.class,
                MacroClass::Core(_) | MacroClass::Block | MacroClass::Endcap
            );
            if !placeable {
                continue;
            }
            let fixed = matches!(c.status, PlacementStatus::Fixed | PlacementStatus::Cover);
            let bbox = db.cell_bbox(id);
            if fixed && !shell.intersects(&bbox) {
                continue;
            }
            let rel = self.relative(bbox);
            let size = c.orient.oriented_size(m.size);
            nodes.push(Node {
                cell: id,
                name: c.name.clone(),
                fixed,
                placed: c.status != PlacementStatus::Unplaced,
                hold: false,
                group: None,
                region: None,
                left: rel.min.x.get(),
                bottom: rel.min.y.get(),
                width: size.x.get(),
                height: size.y.get(),
                orient: c.orient,
                init: (
                    c.location.x.get() - self.core.min.x.get(),
                    c.location.y.get() - self.core.min.y.get(),
                ),
                std_cell: matches!(m.class, MacroClass::Core(_) | MacroClass::Endcap),
                core: matches!(m.class, MacroClass::Core(_)),
                block: m.class == MacroClass::Block,
                multi_row: uniform_row_height.is_some_and(|h| m.size.y.get() > h),
                site: m.site,
                symmetry: m.symmetry,
                used_layers: used_layers(db, id),
                edges: self.master_edges(db, id),
                master_size: (m.size.x.get(), m.size.y.get()),
            });
        }
        nodes
    }

    /// The typed edges of a cell's master, as `Network::addMaster` builds
    /// them; none without a cell edge spacing table or for a spacer.
    fn master_edges(&self, db: &NetlistDB, cell: CellId) -> Vec<(usize, Rect<Dbu>)> {
        let m = db.cell_macro(cell);
        if self.edge_spacing.is_empty()
            || m.class == MacroClass::Core(pare_common::db::library::CoreKind::Spacer)
        {
            return Vec::new();
        }
        let (w, h) = (m.size.x, m.size.y);
        let segment = |side: CellSide| -> Rect<Dbu> {
            match side {
                CellSide::Right => Rect::new(Point::new(w, Dbu(0)), Point::new(w, h)),
                CellSide::Left | CellSide::Both => {
                    Rect::new(Point::new(Dbu(0), Dbu(0)), Point::new(Dbu(0), h))
                }
                CellSide::Top => Rect::new(Point::new(Dbu(0), h), Point::new(w, h)),
                CellSide::Bottom => Rect::new(Point::new(Dbu(0), Dbu(0)), Point::new(w, Dbu(0))),
            }
        };
        let mut edges = Vec::new();
        let mut typed: Vec<(CellSide, Rect<Dbu>)> = Vec::new();
        for edge in &m.edges {
            let sides: &[CellSide] = match edge.side {
                CellSide::Both => &[CellSide::Left, CellSide::Right],
                CellSide::Left => &[CellSide::Left],
                CellSide::Right => &[CellSide::Right],
                CellSide::Top => &[CellSide::Top],
                CellSide::Bottom => &[CellSide::Bottom],
            };
            for &side in sides {
                let rect = segment(side);
                typed.push((side, rect));
                if let Some(t) = self.edge_spacing.type_index(&edge.edge_type) {
                    edges.push((t, rect));
                }
            }
        }
        if let Some(default) = self.edge_spacing.type_index(&EdgeType("DEFAULT".into())) {
            for side in [
                CellSide::Top,
                CellSide::Right,
                CellSide::Bottom,
                CellSide::Left,
            ] {
                if !typed.iter().any(|&(s, _)| s == side) {
                    edges.push((default, segment(side)));
                }
            }
        }
        edges
    }

    /// Puts every fixed cell on the grid, as `Opendp::setFixedGridCells`;
    /// the pixels of a macro become hopeless so cells move off it.
    fn paint_fixed_cells(&mut self) {
        for n in 0..self.nodes.len() {
            if !self.nodes[n].fixed {
                continue;
            }
            let node = &self.nodes[n];
            let block = node.block;
            let rect = Rect::new(
                Point::new(Dbu(node.left), Dbu(node.bottom)),
                Point::new(Dbu(node.left + node.width), Dbu(node.bottom + node.height)),
            );
            let (x0, y0, x1, y1) = self.covering(&rect);
            for x in x0..x1 {
                for y in y0..y1 {
                    if let Some(p) = self.pixel_mut(x, y) {
                        p.cell = u32::try_from(n).unwrap_or(NO_CELL);
                        if block {
                            p.hopeless = true;
                        }
                    }
                }
            }
        }
    }

    /// Places every movable standard cell, as `Opendp::place`.
    fn place(&mut self) -> Result<DetailedPlacement, DetailedPlacementError> {
        let mut order: Vec<usize> = Vec::new();
        for (n, node) in self.nodes.iter().enumerate() {
            if !node.core || node.fixed || node.group.is_some() || node.placed {
                continue;
            }
            if self.grid_width(node) > self.row_site_count || node.height > self.core.height().get()
            {
                return Err(DetailedPlacementError::DoesNotFit(node.name.clone()));
            }
            order.push(n);
        }
        let center = (
            i64::midpoint(self.core.min.x.get(), self.core.max.x.get()),
            i64::midpoint(self.core.min.y.get(), self.core.max.y.get()),
        );
        let center_dist = |n: &Node| (n.left - center.0).abs() + (n.bottom - center.1).abs();
        order.sort_by(|&a, &b| {
            let (na, nb) = (&self.nodes[a], &self.nodes[b]);
            nb.multi_row
                .cmp(&na.multi_row)
                .then_with(|| area(nb).cmp(&area(na)))
                .then_with(|| center_dist(na).cmp(&center_dist(nb)))
                .then_with(|| na.name.cmp(&nb.name))
        });
        let mut summary = DetailedPlacement {
            cells: order.len(),
            ..DetailedPlacement::default()
        };
        let mut failures = Vec::new();
        for n in order {
            if self.diamond_move(n) {
                summary.diamond_moves += 1;
            } else if self.rip_up_and_replace(n, &mut failures) {
                summary.rip_up_moves += 1;
            }
        }
        if !failures.is_empty() {
            return Err(DetailedPlacementError::Failed(
                failures
                    .iter()
                    .map(|&n| self.nodes[n].name.clone())
                    .collect(),
            ));
        }
        for node in &self.nodes {
            let d = (node.init.0 - node.left).abs() + (node.init.1 - node.bottom).abs();
            summary.total_displacement += Dbu(d);
            summary.max_displacement = summary.max_displacement.max(Dbu(d));
        }
        Ok(summary)
    }

    /// Places a cell at the free spot nearest its legal position, as
    /// `Opendp::diamondMove`.
    fn diamond_move(&mut self, n: usize) -> bool {
        let start = self.legal_grid_pt(n);
        self.diamond_move_from(n, start)
    }

    fn diamond_move_from(&mut self, n: usize, start: GridPt) -> bool {
        match self.diamond_search(n, start) {
            Some((x, y)) => {
                self.place_cell(n, x, y);
                true
            }
            None => false,
        }
    }

    /// Takes up the cells around a cell that found no spot, places it and
    /// then them, as `Opendp::ripUpAndReplace`; failures are recorded.
    fn rip_up_and_replace(&mut self, n: usize, failures: &mut Vec<usize>) -> bool {
        let (tx, ty) = self.legal_grid_pt(n);
        let margin_width = self.grid_width(&self.nodes[n]) * (1 + RIP_UP_ROWS);
        let mut around: BTreeSet<usize> = BTreeSet::new();
        for x in tx - margin_width..=tx + margin_width {
            for y in ty - RIP_UP_ROWS..=ty + RIP_UP_ROWS {
                if let Some(other) = self.pixel(x, y).and_then(|p| node_at(p.cell))
                    && !self.nodes[other].fixed
                {
                    let _ = around.insert(other);
                }
            }
        }
        let in_group = self.nodes[n].group.is_some();
        let around: Vec<usize> = around
            .into_iter()
            .filter(|&other| self.nodes[other].group.is_some() == in_group)
            .collect();
        for &other in &around {
            self.unplace_cell(other);
        }
        let mut success = self.diamond_move(n);
        if !success {
            failures.push(n);
        }
        for &other in &around {
            if !self.diamond_move(other) {
                failures.push(other);
                success = false;
            }
        }
        success
    }

    /// The grid point a cell's legal position falls in, as
    /// `Opendp::legalGridPt`.
    fn legal_grid_pt(&self, n: usize) -> GridPt {
        let (x, y) = self.legal_pt_moved(n);
        (self.grid_x(x), self.snap_down_y(y))
    }

    /// A cell's position made legal and moved off any macro, as
    /// `Opendp::legalPt` with padding.
    fn legal_pt_moved(&self, n: usize) -> (i64, i64) {
        let node = &self.nodes[n];
        let mut legal = self.legal_pt(node, node.init);
        let mut gx = self.grid_x(legal.0);
        let mut gy = self.snap_down_y(legal.1);
        let Some(mut pixel) = self.pixel(gx, gy).copied() else {
            return legal;
        };
        if pixel.hopeless && self.move_hopeless(node, &mut gx, &mut gy) {
            legal = (gx * self.site_width, self.row_y(gy));
            pixel = self.pixel(gx, gy).copied().unwrap_or(pixel);
        }
        if let Some(block) = node_at(pixel.cell) {
            let block = &self.nodes[block];
            if block.block {
                let b = (
                    block.left,
                    block.bottom,
                    block.left + block.width,
                    block.bottom + block.height,
                );
                if legal.0 + node.width >= b.0
                    && legal.0 <= b.2
                    && legal.1 + node.height >= b.1
                    && legal.1 <= b.3
                {
                    legal = self.nearest_block_edge(node, legal, b);
                }
            }
        }
        legal
    }

    /// A point clamped into the core and rounded onto the site grid and a
    /// row, as `Opendp::legalPt` with a point.
    fn legal_pt(&self, node: &Node, at: (i64, i64)) -> (i64, i64) {
        let core_x = at.0.clamp(
            0,
            (self.row_site_count * self.site_width - node.width).max(0),
        );
        let column = div_round(core_x, self.site_width);
        let core_y = at.1.clamp(0, (self.core.max.y.get() - node.height).max(0));
        (column * self.site_width, self.row_y(self.round_y(core_y)))
    }

    /// The cell moved past the nearest edge of a macro, as
    /// `Opendp::nearestBlockEdge`.
    fn nearest_block_edge(
        &self,
        node: &Node,
        at: (i64, i64),
        b: (i64, i64, i64, i64),
    ) -> (i64, i64) {
        let x_min = (at.0 - b.0).abs();
        let x_max = (b.2 - (at.0 + node.width)).abs();
        let y_min = (at.1 - b.1).abs();
        let y_max = (b.3 - (at.1 + node.height)).abs();
        let nearest = x_min.min(x_max).min(y_min).min(y_max);
        let to = if nearest == x_min {
            (b.0 - node.width, at.1)
        } else if nearest == x_max {
            (b.2, at.1)
        } else if nearest == y_min {
            (at.0, b.1 - node.height)
        } else {
            (at.0, b.3)
        };
        self.legal_pt(node, to)
    }

    /// The nearest valid site left, right, below or above a hopeless one,
    /// as `Opendp::moveHopeless`.
    fn move_hopeless(&self, node: &Node, gx: &mut i64, gy: &mut i64) -> bool {
        let (x0, y0) = (*gx, *gy);
        let mut best: Option<(i64, i64, i64)> = None;
        let valid = |x: i64, y: i64| self.pixel(x, y).is_some_and(|p| p.valid);
        if let Some(x) = (0..x0).rev().find(|&x| valid(x, y0)) {
            best = Some(((x0 - x - 1) * self.site_width, x, y0));
        }
        if let Some(x) = (x0 + 1..self.row_site_count).find(|&x| valid(x, y0)) {
            let dist = (x - x0) * self.site_width - node.width;
            if best.is_none_or(|(d, _, _)| dist < d) {
                best = Some((dist, x, y0));
            }
        }
        if let Some(y) = (0..y0).rev().find(|&y| valid(x0, y)) {
            let dist = self.row_y(y0) - self.row_y(y);
            if best.is_none_or(|(d, _, _)| dist < d) {
                best = Some((dist, x0, y));
            }
        }
        if let Some(y) = (y0 + 1..self.row_count).find(|&y| valid(x0, y)) {
            let dist = self.row_y(y) - self.row_y(y0);
            if best.is_none_or(|(d, _, _)| dist < d) {
                best = Some((dist, x0, y));
            }
        }
        let Some((_, x, y)) = best else {
            return false;
        };
        *gx = x;
        *gy = y;
        true
    }

    /// The nearest spot from `(x, y)` where the cell can be placed, in
    /// Manhattan distance and then the order the search reached it, as
    /// `Opendp::diamondSearch`.
    fn diamond_search(&mut self, n: usize, (x, y): GridPt) -> Option<GridPt> {
        let (mut x_min, mut y_min) = (x - MAX_DISPLACEMENT_X, y - MAX_DISPLACEMENT_Y);
        let (mut x_max, mut y_max) = (x + MAX_DISPLACEMENT_X, y + MAX_DISPLACEMENT_Y);
        if let Some(g) = self.nodes[n].group {
            let (gx0, gy0, gx1, gy1) = self.grid_within(&self.groups[g].bbox);
            let (lx, hx) = (gx0.min(gx1), gx0.max(gx1));
            let (ly, hy) = (gy0.min(gy1), gy0.max(gy1));
            let clamp = |(px, py): GridPt| (px.clamp(lx, hx), py.clamp(ly, hy));
            (x_min, y_min) = clamp((x_min, y_min));
            (x_max, y_max) = clamp((x_max, y_max));
        }
        let x_min = x_min.max(0);
        let y_min = y_min.max(0);
        let x_max = x_max.min(self.row_site_count);
        let y_max = y_max.min(self.row_count);
        let mut search = std::mem::take(&mut self.search);
        search.start((x, y));
        let mut heap: BinaryHeap<Reverse<(i64, u64, i64, i64)>> = BinaryHeap::new();
        let mut sequence = 0u64;
        heap.push(Reverse((0, sequence, x, y)));
        sequence += 1;
        let mut found = None;
        while let Some(Reverse((_, _, px, py))) = heap.pop() {
            if self.can_be_placed(n, px, py) {
                found = Some((px, py));
                break;
            }
            for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                let (nx, ny) = (px + dx, py + dy);
                if nx < x_min || nx > x_max || ny < y_min || ny > y_max || !search.visit((nx, ny)) {
                    continue;
                }
                let dist =
                    (self.row_y(y) - self.row_y(ny)).abs() + (x - nx).abs() * self.site_width;
                heap.push(Reverse((dist, sequence, nx, ny)));
                sequence += 1;
            }
        }
        self.search = search;
        found
    }

    /// Whether the cell fits at `(x, y)`, as `Opendp::canBePlaced`.
    fn can_be_placed(&self, n: usize, x: i64, y: i64) -> bool {
        if y >= self.row_count {
            return false;
        }
        let node = &self.nodes[n];
        let x_end = x + self.grid_width(node);
        let y_end = self.end_y(self.row_y(y) + node.height);
        self.check_pixels(node, x, y, x_end, y_end)
    }

    /// `Opendp::checkPixels`.
    fn check_pixels(&self, node: &Node, x: i64, y: i64, x_end: i64, y_end: i64) -> bool {
        if x_end > self.row_site_count {
            return false;
        }
        if !self.region_overlap_allowed(node, x, y, x_end, y_end) {
            return false;
        }
        let group = node
            .group
            .map_or(NO_GROUP, |g| u32::try_from(g).unwrap_or(NO_GROUP));
        for y1 in y..y_end {
            for x1 in x..x_end {
                let Some(p) = self.pixel(x1, y1) else {
                    return false;
                };
                if p.cell != NO_CELL || !p.valid || p.group != group {
                    return false;
                }
                if y1 == y && self.site_orientation(x1, y1, node.site).is_none() {
                    return false;
                }
            }
        }
        if self.disallow_one_site_gaps && !self.no_one_site_gap_neighbourhood(x, y, x_end, y_end) {
            return false;
        }
        let Some(orient) = self.site_orientation(x, y, node.site) else {
            return false;
        };
        if !symmetry_allows(node.symmetry, orient) {
            return false;
        }
        self.check_edge_spacing(node, x, y, orient)
            && self.check_blocked_layers(node, x, y)
            && self.check_one_site_gap(node, x, y)
    }

    /// The one-site gap test `Opendp::checkPixels` makes itself.
    fn no_one_site_gap_neighbourhood(&self, x: i64, y: i64, x_end: i64, y_end: i64) -> bool {
        let x_begin = (x - 1).max(0);
        let y_begin = y.max(0);
        let x_finish = x_end.min(self.row_site_count - 1);
        let y_finish = y_end.min(self.row_count - 1);
        let abutted = |x: i64, y: i64| self.pixel(x, y).is_none_or(|p| p.cell != NO_CELL);
        let cell_at = |x: i64, y: i64| self.pixel(x, y).is_some_and(|p| p.cell != NO_CELL);
        for y in y_begin..y_finish {
            if !abutted(x_begin, y) && cell_at(x_begin - 1, y) {
                return false;
            }
            if !abutted(x_finish, y) && cell_at(x_finish + 1, y) {
                return false;
            }
        }
        true
    }

    /// `PlacementDRC::checkOneSiteGap`.
    fn check_one_site_gap(&self, node: &Node, x: i64, y: i64) -> bool {
        if !self.disallow_one_site_gaps {
            return true;
        }
        let x_begin = x - 1;
        let x_finish = x + self.grid_width(node);
        let y_finish = self.end_y(self.row_y(y) + node.height);
        let taken = |x: i64, y: i64| self.pixel(x, y).is_none_or(|p| p.cell != NO_CELL);
        for y in y..y_finish {
            if !taken(x_begin, y) && taken(x_begin - 1, y) {
                return false;
            }
            if !taken(x_finish, y) && taken(x_finish + 1, y) {
                return false;
            }
        }
        true
    }

    /// `PlacementDRC::checkBlockedLayers`.
    fn check_blocked_layers(&self, node: &Node, x: i64, y: i64) -> bool {
        let x_end = x + self.grid_width(node);
        let y_end = self.end_y(self.row_y(y) + node.height);
        for y1 in y..y_end {
            for x1 in x..x_end {
                if self
                    .pixel(x1, y1)
                    .is_some_and(|p| p.blocked_layers & node.used_layers != 0)
                {
                    return false;
                }
            }
        }
        true
    }

    /// `PlacementDRC::checkEdgeSpacing`.
    fn check_edge_spacing(&self, node: &Node, x: i64, y: i64, orient: Orientation) -> bool {
        if self.edge_spacing.is_empty() {
            return true;
        }
        let x_real = x * self.site_width;
        let y_real = self.row_y(y);
        for &(t1, edge1) in &node.edges {
            let max_spacing = self.edge_spacing.max_spacing(t1) + 1;
            let edge1_box = transform_edge(edge1, node.master_size, x_real, y_real, orient);
            let vertical = is_vertical(&edge1_box);
            let query = if vertical {
                Rect::new(
                    Point::new(edge1_box.min.x - Dbu(max_spacing), edge1_box.min.y),
                    Point::new(edge1_box.max.x + Dbu(max_spacing), edge1_box.max.y),
                )
            } else {
                Rect::new(
                    Point::new(edge1_box.min.x, edge1_box.min.y - Dbu(max_spacing)),
                    Point::new(edge1_box.max.x, edge1_box.max.y + Dbu(max_spacing)),
                )
            };
            let x_min = self.grid_x(query.min.x.get());
            let x_max = self.grid_end_x(query.max.x.get());
            let y_min = self.end_y(query.min.y.get()) - 1;
            let y_max = self.end_y(query.max.y.get());
            let mut checked = BTreeSet::new();
            for y1 in y_min..=y_max {
                for x1 in x_min..=x_max {
                    let Some(p) = self.pixel(x1, y1) else {
                        continue;
                    };
                    let Some(other) = node_at(p.cell) else {
                        continue;
                    };
                    if self.nodes[other].cell == node.cell || !checked.insert(other) {
                        continue;
                    }
                    let other = &self.nodes[other];
                    for &(t2, edge2) in &other.edges {
                        let spacing = self.edge_spacing.table[t1][t2];
                        let edge2_box = transform_edge(
                            edge2,
                            other.master_size,
                            other.left,
                            other.bottom,
                            other.orient,
                        );
                        if is_vertical(&edge2_box) != vertical || !open_overlap(&query, &edge2_box)
                        {
                            continue;
                        }
                        let merged = edge1_box.union(&edge2_box);
                        let dist = if vertical {
                            merged.width()
                        } else {
                            merged.height()
                        };
                        if dist.get() < spacing {
                            return false;
                        }
                    }
                }
            }
        }
        true
    }

    /// Puts a cell at `(x, y)` in its row's orientation, as
    /// `Opendp::placeCell`.
    fn place_cell(&mut self, n: usize, x: i64, y: i64) {
        let left = x * self.site_width;
        let bottom = self.row_y(y);
        let x_end = x + self.grid_width(&self.nodes[n]);
        let y_end = self.end_y(bottom + self.nodes[n].height);
        for gx in x..x_end {
            for gy in y..y_end {
                if let Some(p) = self.pixel_mut(gx, gy) {
                    p.cell = u32::try_from(n).unwrap_or(NO_CELL);
                }
            }
        }
        let orient = self.site_orientation(x, y, self.nodes[n].site);
        let node = &mut self.nodes[n];
        node.left = left;
        node.bottom = bottom;
        node.placed = true;
        if let Some(orient) = orient {
            node.orient = orient;
        }
    }

    /// Takes a cell off the grid, as `Opendp::unplaceCell`.
    fn unplace_cell(&mut self, n: usize) {
        let node = &self.nodes[n];
        if node.fixed || !node.placed {
            return;
        }
        let rect = Rect::new(
            Point::new(Dbu(node.left), Dbu(node.bottom)),
            Point::new(Dbu(node.left + node.width), Dbu(node.bottom + node.height)),
        );
        let (x0, y0, x1, y1) = self.covering(&rect);
        let id = u32::try_from(n).unwrap_or(NO_CELL);
        for x in x0..x1 {
            for y in y0..y1 {
                if let Some(p) = self.pixel_mut(x, y)
                    && p.cell == id
                {
                    p.cell = NO_CELL;
                }
            }
        }
        self.nodes[n].placed = false;
        self.nodes[n].hold = false;
    }

    /// The grid cells wholly inside a rectangle, as `Grid::gridWithin`.
    fn grid_within(&self, r: &Rect<Dbu>) -> (i64, i64, i64, i64) {
        (
            div_ceil(r.min.x.get(), self.site_width),
            self.end_y(r.min.y.get()),
            r.max.x.get() / self.site_width,
            self.snap_down_y(r.max.y.get()),
        )
    }

    /// Whether the cell may cover `[x, x_end) x [y, y_end)` given the
    /// regions: within one region box for a member of a group, clear of
    /// every box otherwise, as `Opendp::checkRegionOverlap`.
    fn region_overlap_allowed(&self, node: &Node, x: i64, y: i64, x_end: i64, y_end: i64) -> bool {
        let query = Rect::new(
            Point::new(Dbu(x * self.site_width), Dbu(self.row_y(y))),
            Point::new(Dbu(x_end * self.site_width - 1), Dbu(self.row_y(y_end) - 1)),
        );
        let mut hits = self
            .region_boxes
            .iter()
            .filter(|b| closed_overlap(b, &query));
        if node.region.is_some() {
            let (Some(only), None) = (hits.next(), hits.next()) else {
                return false;
            };
            return only.min.x <= query.min.x
                && query.max.x <= only.max.x
                && only.min.y <= query.min.y
                && query.max.y <= only.max.y;
        }
        hits.next().is_none()
    }

    /// Writes the movable standard cells' places and orientations back, as
    /// `Opendp::updateDbInstLocations`.
    fn write_back(&self, db: &mut NetlistDB) {
        for node in &self.nodes {
            if node.fixed || !node.std_cell {
                continue;
            }
            let c = &mut db.cells[node.cell.index()];
            c.orient = node.orient;
            c.location = Point::new(
                Dbu(self.core.min.x.get() + node.left),
                Dbu(self.core.min.y.get() + node.bottom),
            );
            if c.status == PlacementStatus::Unplaced {
                c.status = PlacementStatus::Placed;
            }
        }
    }
}

/// The node a pixel holds, if a real one does.
const fn node_at(cell: u32) -> Option<usize> {
    if cell == NO_CELL || cell == DUMMY_CELL {
        None
    } else {
        Some(cell as usize)
    }
}

/// The search window's visited marks, reused between searches.
#[derive(Debug, Default)]
struct SearchMarks {
    stamp: u32,
    marks: Vec<u32>,
    center: GridPt,
}

impl SearchMarks {
    const WIDTH: i64 = 2 * MAX_DISPLACEMENT_X + 3;
    const HEIGHT: i64 = 2 * MAX_DISPLACEMENT_Y + 3;

    fn new() -> Self {
        Self {
            stamp: 0,
            marks: vec![0; usize::try_from(Self::WIDTH * Self::HEIGHT).unwrap_or(0)],
            center: (0, 0),
        }
    }

    fn start(&mut self, center: GridPt) {
        self.stamp = self.stamp.wrapping_add(1);
        if self.stamp == 0 {
            self.marks.fill(0);
            self.stamp = 1;
        }
        self.center = center;
        let _ = self.visit(center);
    }

    /// Marks a point visited; false when it already was.
    fn visit(&mut self, (x, y): GridPt) -> bool {
        let dx = x - self.center.0 + MAX_DISPLACEMENT_X + 1;
        let dy = y - self.center.1 + MAX_DISPLACEMENT_Y + 1;
        let i = usize::try_from(dy * Self::WIDTH + dx).unwrap_or(0);
        if self.marks[i] == self.stamp {
            return false;
        }
        self.marks[i] = self.stamp;
        true
    }
}

/// The rows of the core with the holes they enclose, as `OpenDP`'s row
/// outer shell: fixed cells outside it are left out.
struct RowShell {
    rows: Vec<Rect<Dbu>>,
    holes: Vec<Rect<Dbu>>,
}

impl RowShell {
    fn new(db: &NetlistDB, rows: &[&pare_common::db::design::Row]) -> Self {
        let rects: Vec<Rect<Dbu>> = rows.iter().map(|r| r.bbox(&db.library)).collect();
        let holes = enclosed_holes(&rects);
        Self { rows: rects, holes }
    }

    fn intersects(&self, r: &Rect<Dbu>) -> bool {
        self.rows
            .iter()
            .chain(&self.holes)
            .any(|s| closed_overlap(s, r))
    }
}

/// The parts of the rows' bounding box no row covers and that do not reach
/// the box's edge, as rectangles of the compressed coordinate grid.
fn enclosed_holes(rows: &[Rect<Dbu>]) -> Vec<Rect<Dbu>> {
    let Some(bbox) = rows.iter().copied().reduce(|a, b| a.union(&b)) else {
        return Vec::new();
    };
    let mut xs: Vec<Dbu> = rows.iter().flat_map(|r| [r.min.x, r.max.x]).collect();
    let mut ys: Vec<Dbu> = rows.iter().flat_map(|r| [r.min.y, r.max.y]).collect();
    xs.sort_unstable();
    xs.dedup();
    ys.sort_unstable();
    ys.dedup();
    let (nx, ny) = (xs.len() - 1, ys.len() - 1);
    let covered: Vec<bool> = (0..nx * ny)
        .map(|k| {
            let (i, j) = (k % nx, k / nx);
            let cell = Rect::new(Point::new(xs[i], ys[j]), Point::new(xs[i + 1], ys[j + 1]));
            rows.iter().any(|r| {
                r.min.x <= cell.min.x
                    && cell.max.x <= r.max.x
                    && r.min.y <= cell.min.y
                    && cell.max.y <= r.max.y
            })
        })
        .collect();
    let mut component = vec![usize::MAX; nx * ny];
    let mut holes = Vec::new();
    for start in 0..nx * ny {
        if covered[start] || component[start] != usize::MAX {
            continue;
        }
        let mut stack = vec![start];
        let mut members = Vec::new();
        let mut touches_edge = false;
        component[start] = start;
        while let Some(k) = stack.pop() {
            members.push(k);
            let (i, j) = (k % nx, k / nx);
            if i == 0 || j == 0 || i + 1 == nx || j + 1 == ny {
                touches_edge = true;
            }
            let neighbours = [
                (i > 0).then(|| k - 1),
                (i + 1 < nx).then(|| k + 1),
                (j > 0).then(|| k - nx),
                (j + 1 < ny).then(|| k + nx),
            ];
            for next in neighbours.into_iter().flatten() {
                if !covered[next] && component[next] == usize::MAX {
                    component[next] = start;
                    stack.push(next);
                }
            }
        }
        if !touches_edge {
            holes.extend(members.into_iter().map(|k| {
                let (i, j) = (k % nx, k / nx);
                Rect::new(Point::new(xs[i], ys[j]), Point::new(xs[i + 1], ys[j + 1]))
            }));
        }
    }
    let _ = bbox;
    holes
}

/// Rectangles that share at least a point, as `odb::Rect::intersects`.
fn closed_overlap(a: &Rect<Dbu>, b: &Rect<Dbu>) -> bool {
    a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y
}

/// Rectangles whose interiors meet, as `odb::Rect::overlaps`.
fn open_overlap(a: &Rect<Dbu>, b: &Rect<Dbu>) -> bool {
    a.min.x < b.max.x && b.min.x < a.max.x && a.min.y < b.max.y && b.min.y < a.max.y
}

/// Whether a rectangle is taller than wide, as `odb::Rect::getDir`.
fn is_vertical(r: &Rect<Dbu>) -> bool {
    r.width() < r.height()
}

/// An edge of a master of `size` placed with its lower left at `(x, y)` in
/// `orient`, as `cell_edges::transformEdgeRect`.
fn transform_edge(
    edge: Rect<Dbu>,
    size: (i64, i64),
    x: i64,
    y: i64,
    orient: Orientation,
) -> Rect<Dbu> {
    let apply = |p: (i64, i64)| orient_point(orient, p);
    let corners = [apply((0, 0)), apply((size.0, size.1))];
    let bbox_min = (
        corners[0].0.min(corners[1].0),
        corners[0].1.min(corners[1].1),
    );
    let a = apply((edge.min.x.get(), edge.min.y.get()));
    let b = apply((edge.max.x.get(), edge.max.y.get()));
    let (dx, dy) = (x - bbox_min.0, y - bbox_min.1);
    Rect::new(
        Point::new(Dbu(a.0.min(b.0) + dx), Dbu(a.1.min(b.1) + dy)),
        Point::new(Dbu(a.0.max(b.0) + dx), Dbu(a.1.max(b.1) + dy)),
    )
}

/// A point turned by an orientation about the origin, as `dbTransform`.
const fn orient_point(orient: Orientation, (x, y): (i64, i64)) -> (i64, i64) {
    match orient {
        Orientation::N => (x, y),
        Orientation::S => (-x, -y),
        Orientation::W => (-y, x),
        Orientation::E => (y, -x),
        Orientation::FN => (-x, y),
        Orientation::FS => (x, -y),
        Orientation::FW => (y, x),
        Orientation::FE => (-y, -x),
    }
}

/// Whether a master's symmetry lets it take a row's orientation, as
/// `Opendp::checkMasterSym`.
const fn symmetry_allows(symmetry: Symmetry, orient: Orientation) -> bool {
    match orient {
        Orientation::N => true,
        Orientation::FS => symmetry.x,
        Orientation::FN => symmetry.y,
        Orientation::S => symmetry.x && symmetry.y,
        Orientation::W | Orientation::E => symmetry.r90,
        Orientation::FW | Orientation::FE => symmetry.r90 && symmetry.x && symmetry.y,
    }
}

/// The layers a cell's signal pins use and the one above each, by routing
/// level, up to metal3, as `Network::addPin` marks them.
fn used_layers(db: &NetlistDB, cell: CellId) -> u8 {
    let mut layers = 0u8;
    for &pin in &db.cells[cell.index()].pins {
        let net = db.pins[pin.index()].net;
        if db.nets[net.index()].signal_use.is_supply() {
            continue;
        }
        let PinOwner::Cell { pin: macro_pin, .. } = db.pins[pin.index()].owner else {
            continue;
        };
        for shape in &db.cell_macro(cell).pin(macro_pin).shapes {
            if let LayerRef::Routing(layer) = shape.layer {
                let level = layer.index() + 1;
                if level <= 3 {
                    layers |= 1 << level;
                    layers |= 1 << (level + 1);
                }
            }
        }
    }
    layers
}

/// The row height every row's is a multiple of, as `Grid::examineRows`
/// finds it; none when the heights do not divide each other.
fn uniform_row_height(db: &NetlistDB, rows: &[&pare_common::db::design::Row]) -> Option<i64> {
    let mut uniform: Option<i64> = None;
    for r in rows {
        let height = db.library.site(r.site).size.y.get();
        uniform = match uniform {
            None => Some(height),
            Some(u) => {
                let (small, large) = (u.min(height), u.max(height));
                if large % small != 0 {
                    return None;
                }
                Some(small)
            }
        };
    }
    uniform
}

/// Whether the library has a placeable cell one site wide, as
/// `odb::hasOneSiteMaster`.
fn has_one_site_master(db: &NetlistDB) -> bool {
    db.library.macros.iter().any(|m| {
        if matches!(
            m.class,
            MacroClass::Block | MacroClass::Pad | MacroClass::Cover
        ) {
            return false;
        }
        m.site.is_some_and(|s| {
            let site = db.library.site(s);
            site.class != SiteClass::Pad && site.size.x == m.size.x
        })
    })
}

const fn area(node: &Node) -> i64 {
    node.master_size.0 * node.master_size.1
}

fn div_ceil(a: i64, b: i64) -> i64 {
    (a as f64 / b as f64).ceil() as i64
}

fn div_round(a: i64, b: i64) -> i64 {
    (a as f64 / b as f64).round() as i64
}
