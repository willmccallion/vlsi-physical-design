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
pub mod improve;
pub mod mirror;

use super::grid::{GridPt, GridRect, GridX, GridY, RowGrid, core_rows};
use pare_common::db::design::{NetlistDB, PinOwner, PlacementStatus};
use pare_common::db::indices::{CellId, IdVec, SiteId};
use pare_common::db::library::{LayerRef, MacroClass, SiteClass, Symmetry};
use pare_common::db::rules::{CellSide, EdgeType};
use pare_common::geom::dbu::{Dbu, DbuArea};
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::cmp::Reverse;
use std::collections::HashMap;
use std::collections::{BTreeSet, BinaryHeap};

/// Sites a cell may move horizontally, as `detailed_placement`'s default.
const MAX_DISPLACEMENT_X: GridX = GridX::new(500);
/// Rows a cell may move vertically, as `detailed_placement`'s default.
const MAX_DISPLACEMENT_Y: GridY = GridY::new(100);
/// Rows around a cell that fails to place whose cells are ripped up, as
/// `Opendp::ripUpAndReplace`'s `boundary_margin` of 3.
const RIP_UP_ROWS: GridY = GridY::new(3);

/// Why detailed placement failed.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum DetailedPlacementError {
    /// The design has no core rows.
    #[error("no rows defined in design")]
    NoRows,
    /// The rows are made of sites of different widths.
    #[error("site widths are not equal: {0} != {1}")]
    SiteWidths(Dbu, Dbu),
    /// A cell is wider or taller than the core.
    #[error("instance {0} does not fit inside the ROW core area")]
    DoesNotFit(String),
    /// A region with member cells has no rectangle inside the core.
    #[error("region {0} lies outside the core")]
    RegionOutsideCore(String),
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
/// When the design has no rows, rows of different site widths, a region
/// with members outside the core, a cell bigger than the core, or a cell no
/// free spot within reach fits.
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

pare_common::define_index!(
    /// A cell the legalizer places, in [`Placer`]'s nodes.
    NodeId(u32)
);

pare_common::define_index!(
    /// A region's group, in [`Placer`]'s groups.
    GroupId(u32)
);

/// What a pixel holds, as `OpenDP`'s `Pixel::cell` points to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Occupant {
    Free,
    /// The fixed stand-in `OpenDP` puts on sites a region only partly covers.
    Dummy,
    Node(NodeId),
}

/// One site of one row band, as `OpenDP`'s `Pixel`.
#[derive(Clone, Copy, Debug)]
struct Pixel {
    /// The occupant in 32 bits, as `OpenDP` keeps a pointer: a node's index,
    /// or [`Self::FREE`] or [`Self::DUMMY`], past every node's.
    occupant: u32,
    /// The group whose region wholly covers the site, its index or
    /// [`Self::NO_GROUP`].
    group: u32,
    valid: bool,
    hopeless: bool,
    blocked_layers: u8,
}

impl Pixel {
    const FREE: u32 = u32::MAX;
    const DUMMY: u32 = u32::MAX - 1;
    const NO_GROUP: u32 = u32::MAX;

    /// A free, invalid site of no group.
    const EMPTY: Self = Self {
        occupant: Self::FREE,
        group: Self::NO_GROUP,
        valid: false,
        hopeless: false,
        blocked_layers: 0,
    };

    const fn occupant(&self) -> Occupant {
        match self.occupant {
            Self::FREE => Occupant::Free,
            Self::DUMMY => Occupant::Dummy,
            node => Occupant::Node(NodeId(node)),
        }
    }

    /// Whether nothing, not even the dummy, holds the site.
    const fn is_free(&self) -> bool {
        self.occupant == Self::FREE
    }

    /// The node holding the site, if one does.
    const fn node(&self) -> Option<NodeId> {
        match self.occupant() {
            Occupant::Node(node) => Some(node),
            Occupant::Free | Occupant::Dummy => None,
        }
    }

    fn set_occupant(&mut self, occupant: Occupant) {
        self.occupant = match occupant {
            Occupant::Free => Self::FREE,
            Occupant::Dummy => Self::DUMMY,
            Occupant::Node(node) => {
                debug_assert!(node.0 < Self::DUMMY, "node {node:?} past the pixel's ids");
                node.0
            }
        };
    }

    const fn group(&self) -> Option<GroupId> {
        match self.group {
            Self::NO_GROUP => None,
            group => Some(GroupId(group)),
        }
    }

    fn set_group(&mut self, group: GroupId) {
        debug_assert!(
            group.0 < Self::NO_GROUP,
            "group {group:?} past the pixel's ids"
        );
        self.group = group.0;
    }
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
    group: Option<GroupId>,
    left: Dbu,
    bottom: Dbu,
    width: Dbu,
    height: Dbu,
    orient: Orientation,
    /// Where the design had it, relative to the core.
    init: Point<Dbu>,
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
    master_size: Point<Dbu>,
}

impl Node {
    /// Its width and height as it stands.
    const fn size(&self) -> Point<Dbu> {
        Point::new(self.width, self.height)
    }

    /// The box it covers where it stands.
    fn bbox(&self) -> Rect<Dbu> {
        Rect::new(
            Point::new(self.left, self.bottom),
            Point::new(self.left + self.width, self.bottom + self.height),
        )
    }
}

/// A row's stretch of sites with its site and orientation.
#[derive(Clone, Copy, Debug)]
struct RowSites {
    x0: GridX,
    x1: GridX,
    site: SiteId,
    orient: Orientation,
}

/// The cell edge spacing table, as `PlacementDRC` builds it.
#[derive(Debug, Default)]
struct EdgeSpacing {
    index: HashMap<String, usize>,
    table: Vec<Vec<Dbu>>,
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
        spacing.table = vec![vec![Dbu::ZERO; n]; n];
        for rule in rules {
            let a = spacing.index[&rule.first.0];
            let b = spacing.index[&rule.second.0];
            spacing.table[a][b] = rule.spacing;
            spacing.table[b][a] = rule.spacing;
        }
        spacing
    }

    const fn is_empty(&self) -> bool {
        self.table.is_empty()
    }

    fn type_index(&self, t: &EdgeType) -> Option<usize> {
        self.index.get(&t.0).copied()
    }

    fn max_spacing(&self, t: usize) -> Dbu {
        self.table[t].iter().copied().max().unwrap_or(Dbu::ZERO)
    }
}

struct Placer {
    rows: RowGrid,
    pixels: Vec<Pixel>,
    row_sites: Vec<Vec<RowSites>>,
    nodes: IdVec<NodeId, Node>,
    edge_spacing: EdgeSpacing,
    disallow_one_site_gaps: bool,
    search: SearchMarks,
    groups: IdVec<GroupId, groups::Group>,
    /// Every region rectangle with its top and right edges pulled in by one,
    /// as `OpenDP`'s region R-tree holds them.
    region_boxes: Vec<Rect<Dbu>>,
}

impl Placer {
    fn new(db: &NetlistDB) -> Result<Self, DetailedPlacementError> {
        let rows = core_rows(db);
        let grid = RowGrid::new(db, &rows)?;
        let (row_site_count, row_count) = (grid.row_site_count(), grid.row_count());
        let uniform_row_height = uniform_row_height(db, &rows);
        let edge_spacing = EdgeSpacing::new(db);
        let mut placer = Self {
            rows: grid,
            pixels: vec![Pixel::EMPTY; (row_site_count.get() * row_count.get()).max(0) as usize],
            row_sites: vec![Vec::new(); row_count.get().max(0) as usize],
            nodes: IdVec::new(),
            edge_spacing,
            disallow_one_site_gaps: !has_one_site_master(db),
            search: SearchMarks::new(),
            groups: IdVec::new(),
            region_boxes: Vec::new(),
        };
        placer.mark_rows(&rows);
        placer.mark_blocked(db);
        placer.nodes = placer.make_nodes(db, &rows, uniform_row_height).into();
        for node in &mut placer.nodes {
            if !node.fixed {
                node.placed = false;
            }
        }
        placer.make_groups(db)?;
        Ok(placer)
    }

    fn pixel_index(&self, x: GridX, y: GridY) -> Option<usize> {
        self.rows
            .contains(GridPt::new(x, y))
            .then(|| (y.get() * self.rows.row_site_count().get() + x.get()) as usize)
    }

    fn pixel(&self, x: GridX, y: GridY) -> Option<&Pixel> {
        self.pixel_index(x, y).map(|i| &self.pixels[i])
    }

    fn pixel_mut(&mut self, x: GridX, y: GridY) -> Option<&mut Pixel> {
        self.pixel_index(x, y).map(move |i| &mut self.pixels[i])
    }

    fn grid_width(&self, node: &Node) -> GridX {
        GridX::from_dbu_ceil(node.width, self.rows.site_width())
    }

    /// The orientation of the row at `(x, y)` made of `site`, as
    /// `Grid::getSiteOrientation`.
    fn site_orientation(&self, x: GridX, y: GridY, site: Option<SiteId>) -> Option<Orientation> {
        let row = self.row_sites.get(usize::try_from(y.get()).ok()?)?;
        row.iter()
            .find(|s| s.x0 <= x && x < s.x1 && Some(s.site) == site)
            .map(|s| s.orient)
    }

    /// Marks the sites of every row valid and the sites too far from any row
    /// for the search hopeless, as `Grid::markHopeless`.
    fn mark_rows(&mut self, rows: &[&pare_common::db::design::Row]) {
        /// Lines short of the reach `Grid::markHopeless` keeps reachable,
        /// its `safety` of 20 along both axes.
        const SAFETY: i64 = 20;
        let mut reachable = vec![false; self.pixels.len()];
        for r in rows {
            for k in 0..i64::from(r.num_y) {
                let origin_y = r.origin.y + r.step.y * k;
                let x_start = self.rows.grid_x(r.origin.x - self.rows.core().min.x);
                let x_end = x_start + i64::from(r.num_x);
                let y_row = self.rows.snap_down_y(origin_y - self.rows.core().min.y);
                for x in x_start.until(x_end) {
                    if let Some(p) = self.pixel_mut(x, y_row) {
                        p.valid = true;
                    }
                }
                if let Some(row) = usize::try_from(y_row.get())
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
                let xl = (x_start - MAX_DISPLACEMENT_X + SAFETY).max(GridX::ZERO);
                let xh = (x_end + MAX_DISPLACEMENT_X - SAFETY).min(self.rows.row_site_count());
                let yl = (y_row - MAX_DISPLACEMENT_Y + SAFETY).max(GridY::ZERO);
                let yh = (y_row + MAX_DISPLACEMENT_Y - SAFETY).min(self.rows.row_count());
                for y in yl.until(yh) {
                    for x in xl.until(xh) {
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
            let cells = self.rows.covering(&self.rows.relative(rect));
            for y in cells
                .min
                .y
                .max(GridY::ZERO)
                .until(cells.max.y.min(self.rows.row_count()))
            {
                for x in cells
                    .min
                    .x
                    .max(GridX::ZERO)
                    .until(cells.max.x.min(self.rows.row_site_count()))
                {
                    if let Some(p) = self.pixel_mut(x, y) {
                        p.blocked_layers |= 1 << level;
                    }
                }
            }
        }
        for blockage in db.hard_placement_blockages() {
            let cells = self.rows.covering(&self.rows.relative(blockage));
            for y in cells
                .min
                .y
                .max(GridY::ZERO)
                .until(cells.max.y.min(self.rows.row_count()))
            {
                for x in cells
                    .min
                    .x
                    .max(GridX::ZERO)
                    .until(cells.max.x.min(self.rows.row_site_count()))
                {
                    if let Some(p) = self.pixel_mut(x, y) {
                        p.valid = false;
                    }
                }
            }
        }
    }

    /// The legalizer's nodes for the placeable cells, by name, as
    /// `Opendp::createNetwork`: fixed cells only where they meet the rows.
    fn make_nodes(
        &self,
        db: &NetlistDB,
        rows: &[&pare_common::db::design::Row],
        uniform_row_height: Option<Dbu>,
    ) -> Vec<Node> {
        let shell = RowShell::new(db, rows);
        let mut order: Vec<CellId> = db.cells.ids().filter(|&i| !db.cells[i].retired).collect();
        order.sort_by(|&a, &b| db.cells[a].name.cmp(&db.cells[b].name));
        let mut nodes = Vec::new();
        for id in order {
            let c = &db.cells[id];
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
            let rel = self.rows.relative(bbox);
            let size = c.orient.oriented_size(m.size);
            nodes.push(Node {
                cell: id,
                name: c.name.clone(),
                fixed,
                placed: c.status != PlacementStatus::Unplaced,
                hold: false,
                group: None,
                left: rel.min.x,
                bottom: rel.min.y,
                width: size.x,
                height: size.y,
                orient: c.orient,
                init: c.location - self.rows.core().min,
                std_cell: matches!(m.class, MacroClass::Core(_) | MacroClass::Endcap),
                core: matches!(m.class, MacroClass::Core(_)),
                block: m.class == MacroClass::Block,
                multi_row: uniform_row_height.is_some_and(|h| m.size.y > h),
                site: m.site,
                symmetry: m.symmetry,
                used_layers: used_layers(db, id),
                edges: self.master_edges(db, id),
                master_size: m.size,
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
        for n in self.nodes.ids() {
            if !self.nodes[n].fixed {
                continue;
            }
            let node = &self.nodes[n];
            let block = node.block;
            let cells = self.rows.covering(&node.bbox());
            for x in cells.min.x.until(cells.max.x) {
                for y in cells.min.y.until(cells.max.y) {
                    if let Some(p) = self.pixel_mut(x, y) {
                        p.set_occupant(Occupant::Node(n));
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
        let mut order: Vec<NodeId> = Vec::new();
        for (n, node) in self.nodes.iter_enumerated() {
            if !node.core || node.fixed || node.group.is_some() || node.placed {
                continue;
            }
            if self.grid_width(node) > self.rows.row_site_count()
                || node.height > self.rows.core().height()
            {
                return Err(DetailedPlacementError::DoesNotFit(node.name.clone()));
            }
            order.push(n);
        }
        let center = core_center(&self.rows.core());
        let center_dist = |n: &Node| (n.left - center.x).abs() + (n.bottom - center.y).abs();
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
            let d = (node.init.x - node.left).abs() + (node.init.y - node.bottom).abs();
            summary.total_displacement += d;
            summary.max_displacement = summary.max_displacement.max(d);
        }
        Ok(summary)
    }

    /// Places a cell at the free spot nearest its legal position, as
    /// `Opendp::diamondMove`.
    fn diamond_move(&mut self, n: NodeId) -> bool {
        let start = self.legal_grid_pt(n);
        self.diamond_move_from(n, start)
    }

    fn diamond_move_from(&mut self, n: NodeId, start: GridPt) -> bool {
        match self.diamond_search(n, start) {
            Some(at) => {
                self.place_cell(n, at);
                true
            }
            None => false,
        }
    }

    /// Takes up the cells around a cell that found no spot, places it and
    /// then them, as `Opendp::ripUpAndReplace`; failures are recorded.
    fn rip_up_and_replace(&mut self, n: NodeId, failures: &mut Vec<NodeId>) -> bool {
        let target = self.legal_grid_pt(n);
        let margin_width = self.grid_width(&self.nodes[n]) * (1 + RIP_UP_ROWS.get());
        let mut around: BTreeSet<NodeId> = BTreeSet::new();
        for x in (target.x - margin_width).through(target.x + margin_width) {
            for y in (target.y - RIP_UP_ROWS).through(target.y + RIP_UP_ROWS) {
                if let Some(other) = self.pixel(x, y).and_then(Pixel::node)
                    && !self.nodes[other].fixed
                {
                    let _ = around.insert(other);
                }
            }
        }
        let in_group = self.nodes[n].group.is_some();
        let around: Vec<NodeId> = around
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
    fn legal_grid_pt(&self, n: NodeId) -> GridPt {
        let legal = self.legal_pt_moved(n);
        GridPt::new(self.rows.grid_x(legal.x), self.rows.snap_down_y(legal.y))
    }

    /// A cell's position made legal and moved off any macro, as
    /// `Opendp::legalPt` with padding.
    fn legal_pt_moved(&self, n: NodeId) -> Point<Dbu> {
        let node = &self.nodes[n];
        let mut legal = self.rows.legal_pt(node.init, node.size());
        let at = GridPt::new(self.rows.grid_x(legal.x), self.rows.snap_down_y(legal.y));
        let Some(mut pixel) = self.pixel(at.x, at.y).copied() else {
            return legal;
        };
        if pixel.hopeless
            && let Some(valid) = self.move_hopeless(node, at)
        {
            legal = Point::new(
                valid.x.to_dbu(self.rows.site_width()),
                self.rows.row_y(valid.y),
            );
            pixel = self.pixel(valid.x, valid.y).copied().unwrap_or(pixel);
        }
        if let Some(block) = pixel.node() {
            let block = &self.nodes[block];
            if block.block {
                let b = block.bbox();
                if legal.x + node.width >= b.min.x
                    && legal.x <= b.max.x
                    && legal.y + node.height >= b.min.y
                    && legal.y <= b.max.y
                {
                    legal = self.rows.nearest_block_edge(legal, node.size(), &b);
                }
            }
        }
        legal
    }

    /// The nearest valid site left, right, below or above a hopeless one,
    /// as `Opendp::moveHopeless`; none when its row and column have none.
    fn move_hopeless(&self, node: &Node, from: GridPt) -> Option<GridPt> {
        let GridPt { x: x0, y: y0 } = from;
        let mut best: Option<(Dbu, GridPt)> = None;
        let valid = |x: GridX, y: GridY| self.pixel(x, y).is_some_and(|p| p.valid);
        if let Some(x) = GridX::ZERO.until(x0).rev().find(|&x| valid(x, y0)) {
            best = Some((
                (x0 - x - 1).to_dbu(self.rows.site_width()),
                GridPt::new(x, y0),
            ));
        }
        if let Some(x) = (x0 + 1)
            .until(self.rows.row_site_count())
            .find(|&x| valid(x, y0))
        {
            let dist = (x - x0).to_dbu(self.rows.site_width()) - node.width;
            if best.is_none_or(|(d, _)| dist < d) {
                best = Some((dist, GridPt::new(x, y0)));
            }
        }
        if let Some(y) = GridY::ZERO.until(y0).rev().find(|&y| valid(x0, y)) {
            let dist = self.rows.row_y(y0) - self.rows.row_y(y);
            if best.is_none_or(|(d, _)| dist < d) {
                best = Some((dist, GridPt::new(x0, y)));
            }
        }
        if let Some(y) = (y0 + 1)
            .until(self.rows.row_count())
            .find(|&y| valid(x0, y))
        {
            let dist = self.rows.row_y(y) - self.rows.row_y(y0);
            if best.is_none_or(|(d, _)| dist < d) {
                best = Some((dist, GridPt::new(x0, y)));
            }
        }
        best.map(|(_, at)| at)
    }

    /// The nearest spot from `start` where the cell can be placed, in
    /// Manhattan distance and then the order the search reached it, as
    /// `Opendp::diamondSearch`.
    fn diamond_search(&mut self, n: NodeId, start: GridPt) -> Option<GridPt> {
        let GridPt { x, y } = start;
        let (mut x_min, mut y_min) = (x - MAX_DISPLACEMENT_X, y - MAX_DISPLACEMENT_Y);
        let (mut x_max, mut y_max) = (x + MAX_DISPLACEMENT_X, y + MAX_DISPLACEMENT_Y);
        if let Some(g) = self.nodes[n].group {
            let within = self.grid_within(&self.groups[g].bbox);
            let (lx, hx) = (
                within.min.x.min(within.max.x),
                within.min.x.max(within.max.x),
            );
            let (ly, hy) = (
                within.min.y.min(within.max.y),
                within.min.y.max(within.max.y),
            );
            (x_min, x_max) = (x_min.clamp(lx, hx), x_max.clamp(lx, hx));
            (y_min, y_max) = (y_min.clamp(ly, hy), y_max.clamp(ly, hy));
        }
        let x_min = x_min.max(GridX::ZERO);
        let y_min = y_min.max(GridY::ZERO);
        let x_max = x_max.min(self.rows.row_site_count());
        let y_max = y_max.min(self.rows.row_count());
        let mut search = std::mem::take(&mut self.search);
        search.start(start);
        let mut heap: BinaryHeap<Reverse<(Dbu, u64, GridPt)>> = BinaryHeap::new();
        let mut sequence = 0u64;
        heap.push(Reverse((Dbu::ZERO, sequence, start)));
        sequence += 1;
        let mut found = None;
        while let Some(Reverse((_, _, at))) = heap.pop() {
            if self.can_be_placed(n, at) {
                found = Some(at);
                break;
            }
            for (dx, dy) in [(-1, 0), (1, 0), (0, -1), (0, 1)] {
                let next = GridPt::new(at.x + dx, at.y + dy);
                if next.x < x_min
                    || next.x > x_max
                    || next.y < y_min
                    || next.y > y_max
                    || !search.visit(next)
                {
                    continue;
                }
                let dist = (self.rows.row_y(y) - self.rows.row_y(next.y)).abs()
                    + (x - next.x).abs().to_dbu(self.rows.site_width());
                heap.push(Reverse((dist, sequence, next)));
                sequence += 1;
            }
        }
        self.search = search;
        found
    }

    /// Whether the cell fits at `at`, as `Opendp::canBePlaced`.
    fn can_be_placed(&self, n: NodeId, at: GridPt) -> bool {
        if at.y >= self.rows.row_count() {
            return false;
        }
        let node = &self.nodes[n];
        let x_end = at.x + self.grid_width(node);
        let y_end = self.rows.end_y(self.rows.row_y(at.y) + node.height);
        self.check_pixels(node, at.x, at.y, x_end, y_end)
    }

    /// `Opendp::checkPixels`.
    fn check_pixels(&self, node: &Node, x: GridX, y: GridY, x_end: GridX, y_end: GridY) -> bool {
        if x_end > self.rows.row_site_count() {
            return false;
        }
        if !self.region_overlap_allowed(node, x, y, x_end, y_end) {
            return false;
        }
        let group = node.group;
        for y1 in y.until(y_end) {
            for x1 in x.until(x_end) {
                let Some(p) = self.pixel(x1, y1) else {
                    return false;
                };
                if !p.is_free() || !p.valid || p.group() != group {
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
        if !node.symmetry.allows(orient) {
            return false;
        }
        self.check_edge_spacing(node, x, y, orient)
            && self.check_blocked_layers(node, x, y)
            && self.check_one_site_gap(node, x, y)
    }

    /// The one-site gap test `Opendp::checkPixels` makes itself.
    fn no_one_site_gap_neighbourhood(
        &self,
        x: GridX,
        y: GridY,
        x_end: GridX,
        y_end: GridY,
    ) -> bool {
        let x_begin = (x - 1).max(GridX::ZERO);
        let y_begin = y.max(GridY::ZERO);
        let x_finish = x_end.min(self.rows.row_site_count() - 1);
        let y_finish = y_end.min(self.rows.row_count() - 1);
        let abutted = |x: GridX, y: GridY| self.pixel(x, y).is_none_or(|p| !p.is_free());
        let cell_at = |x: GridX, y: GridY| self.pixel(x, y).is_some_and(|p| !p.is_free());
        for y in y_begin.until(y_finish) {
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
    fn check_one_site_gap(&self, node: &Node, x: GridX, y: GridY) -> bool {
        if !self.disallow_one_site_gaps {
            return true;
        }
        let x_begin = x - 1;
        let x_finish = x + self.grid_width(node);
        let y_finish = self.rows.end_y(self.rows.row_y(y) + node.height);
        let taken = |x: GridX, y: GridY| self.pixel(x, y).is_none_or(|p| !p.is_free());
        for y in y.until(y_finish) {
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
    fn check_blocked_layers(&self, node: &Node, x: GridX, y: GridY) -> bool {
        let x_end = x + self.grid_width(node);
        let y_end = self.rows.end_y(self.rows.row_y(y) + node.height);
        for y1 in y.until(y_end) {
            for x1 in x.until(x_end) {
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
    fn check_edge_spacing(&self, node: &Node, x: GridX, y: GridY, orient: Orientation) -> bool {
        if self.edge_spacing.is_empty() {
            return true;
        }
        let at = Point::new(x.to_dbu(self.rows.site_width()), self.rows.row_y(y));
        for &(t1, edge1) in &node.edges {
            let max_spacing = self.edge_spacing.max_spacing(t1) + Dbu(1);
            let edge1_box = transform_edge(edge1, node.master_size, at, orient);
            let vertical = is_vertical(&edge1_box);
            let query = if vertical {
                Rect::new(
                    Point::new(edge1_box.min.x - max_spacing, edge1_box.min.y),
                    Point::new(edge1_box.max.x + max_spacing, edge1_box.max.y),
                )
            } else {
                Rect::new(
                    Point::new(edge1_box.min.x, edge1_box.min.y - max_spacing),
                    Point::new(edge1_box.max.x, edge1_box.max.y + max_spacing),
                )
            };
            let x_min = self.rows.grid_x(query.min.x);
            let x_max = self.rows.grid_end_x(query.max.x);
            let y_min = self.rows.end_y(query.min.y) - 1;
            let y_max = self.rows.end_y(query.max.y);
            let mut checked = BTreeSet::new();
            for y1 in y_min.through(y_max) {
                for x1 in x_min.through(x_max) {
                    let Some(p) = self.pixel(x1, y1) else {
                        continue;
                    };
                    let Some(other) = p.node() else {
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
                            Point::new(other.left, other.bottom),
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
                        if dist < spacing {
                            return false;
                        }
                    }
                }
            }
        }
        true
    }

    /// Puts a cell at `at` in its row's orientation, as
    /// `Opendp::placeCell`.
    fn place_cell(&mut self, n: NodeId, at: GridPt) {
        self.paint_pixels(n, at.x, at.y);
        let orient = self.site_orientation(at.x, at.y, self.nodes[n].site);
        let bottom = self.rows.row_y(at.y);
        let node = &mut self.nodes[n];
        node.left = at.x.to_dbu(self.rows.site_width());
        node.bottom = bottom;
        node.placed = true;
        if let Some(orient) = orient {
            node.orient = orient;
        }
    }

    /// Marks the sites a cell covers from `(x, y)` as its own, as
    /// `Grid::paintPixel`.
    fn paint_pixels(&mut self, n: NodeId, x: GridX, y: GridY) {
        let x_end = x + self.grid_width(&self.nodes[n]);
        let y_end = self.rows.end_y(self.rows.row_y(y) + self.nodes[n].height);
        for gx in x.until(x_end) {
            for gy in y.until(y_end) {
                if let Some(p) = self.pixel_mut(gx, gy) {
                    p.set_occupant(Occupant::Node(n));
                }
            }
        }
    }

    /// Takes a cell off the grid, as `Opendp::unplaceCell`.
    fn unplace_cell(&mut self, n: NodeId) {
        let node = &self.nodes[n];
        if node.fixed || !node.placed {
            return;
        }
        self.erase_pixels(n);
        self.nodes[n].placed = false;
        self.nodes[n].hold = false;
    }

    /// Frees the sites a cell holds where it stands, as `Grid::erasePixel`.
    fn erase_pixels(&mut self, n: NodeId) {
        let cells = self.rows.covering(&self.nodes[n].bbox());
        for x in cells.min.x.until(cells.max.x) {
            for y in cells.min.y.until(cells.max.y) {
                if let Some(p) = self.pixel_mut(x, y)
                    && p.node() == Some(n)
                {
                    p.set_occupant(Occupant::Free);
                }
            }
        }
    }

    /// The grid cells wholly inside a rectangle, as `Grid::gridWithin`.
    fn grid_within(&self, r: &Rect<Dbu>) -> GridRect {
        GridRect {
            min: GridPt::new(
                GridX::from_dbu_ceil(r.min.x, self.rows.site_width()),
                self.rows.end_y(r.min.y),
            ),
            max: GridPt::new(
                GridX::from_dbu_trunc(r.max.x, self.rows.site_width()),
                self.rows.snap_down_y(r.max.y),
            ),
        }
    }

    /// Whether the cell may cover `[x, x_end) x [y, y_end)` given the
    /// regions: within one region box for a member of a group, clear of
    /// every box otherwise, as `Opendp::checkRegionOverlap`.
    fn region_overlap_allowed(
        &self,
        node: &Node,
        x: GridX,
        y: GridY,
        x_end: GridX,
        y_end: GridY,
    ) -> bool {
        let query = Rect::new(
            Point::new(x.to_dbu(self.rows.site_width()), self.rows.row_y(y)),
            Point::new(
                x_end.to_dbu(self.rows.site_width()) - Dbu(1),
                self.rows.row_y(y_end) - Dbu(1),
            ),
        );
        let mut hits = self
            .region_boxes
            .iter()
            .filter(|b| closed_overlap(b, &query));
        if node.group.is_some() {
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
            let c = &mut db.cells[node.cell];
            c.orient = node.orient;
            c.location = Point::new(
                self.rows.core().min.x + node.left,
                self.rows.core().min.y + node.bottom,
            );
            if c.status == PlacementStatus::Unplaced {
                c.status = PlacementStatus::Placed;
            }
        }
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
    const WIDTH: GridX = GridX::new(2 * MAX_DISPLACEMENT_X.get() + 3);
    const HEIGHT: GridY = GridY::new(2 * MAX_DISPLACEMENT_Y.get() + 3);

    fn new() -> Self {
        Self {
            stamp: 0,
            marks: vec![0; (Self::WIDTH.get() * Self::HEIGHT.get()) as usize],
            center: GridPt::default(),
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
    fn visit(&mut self, at: GridPt) -> bool {
        let dx = at.x - self.center.x + MAX_DISPLACEMENT_X + 1;
        let dy = at.y - self.center.y + MAX_DISPLACEMENT_Y + 1;
        let i = (dy.get() * Self::WIDTH.get() + dx.get()) as usize;
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
    let mut visited = vec![false; nx * ny];
    let mut holes = Vec::new();
    for start in 0..nx * ny {
        if covered[start] || visited[start] {
            continue;
        }
        let mut stack = vec![start];
        let mut members = Vec::new();
        let mut touches_edge = false;
        visited[start] = true;
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
                if !covered[next] && !visited[next] {
                    visited[next] = true;
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

/// An edge of a master of `size` placed with its lower left at `at` in
/// `orient`, as `cell_edges::transformEdgeRect`.
fn transform_edge(
    edge: Rect<Dbu>,
    size: Point<Dbu>,
    at: Point<Dbu>,
    orient: Orientation,
) -> Rect<Dbu> {
    let corners = [
        orient.apply(Point::new(Dbu::ZERO, Dbu::ZERO)),
        orient.apply(size),
    ];
    let bbox_min = Point::new(
        corners[0].x.min(corners[1].x),
        corners[0].y.min(corners[1].y),
    );
    let a = orient.apply(edge.min);
    let b = orient.apply(edge.max);
    let shift = at - bbox_min;
    Rect::new(
        Point::new(a.x.min(b.x) + shift.x, a.y.min(b.y) + shift.y),
        Point::new(a.x.max(b.x) + shift.x, a.y.max(b.y) + shift.y),
    )
}

/// The layers a cell's signal pins use and the one above each, by routing
/// level, up to metal3, as `Network::addPin` marks them.
fn used_layers(db: &NetlistDB, cell: CellId) -> u8 {
    let mut layers = 0u8;
    for &pin in &db.cells[cell].pins {
        let net = db.pins[pin].net;
        if db.nets[net].signal_use.is_supply() {
            continue;
        }
        let PinOwner::Cell { pin: macro_pin, .. } = db.pins[pin].owner else {
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
fn uniform_row_height(db: &NetlistDB, rows: &[&pare_common::db::design::Row]) -> Option<Dbu> {
    let mut uniform: Option<Dbu> = None;
    for r in rows {
        let height = db.library.site(r.site).size.y;
        uniform = match uniform {
            None => Some(height),
            Some(u) => {
                let (small, large) = (u.min(height), u.max(height));
                if large.get() % small.get() != 0 {
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

/// The area of a node's master.
fn area(node: &Node) -> DbuArea {
    node.master_size.x * node.master_size.y
}

/// The core's center relative to its lower left, as the nodes' positions
/// are, rounding down as `CellPlaceOrderLess`'s `int` halves do.
// NOTE: CellPlaceOrderLess measures nodes, relative to the core, from the
// core's absolute center; PARE measures both relative to the core.
fn core_center(core: &Rect<Dbu>) -> Point<Dbu> {
    Point::new(core.width() / 2, core.height() / 2)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    #[test]
    fn rows_framing_a_gap_enclose_it() {
        let rows = [
            rect(0, 0, 30, 10),
            rect(0, 10, 10, 20),
            rect(20, 10, 30, 20),
            rect(0, 20, 30, 30),
        ];

        assert_eq!(enclosed_holes(&rows), vec![rect(10, 10, 20, 20)]);
    }

    #[test]
    fn a_gap_reaching_the_rows_edge_encloses_nothing() {
        let rows = [rect(0, 0, 30, 10), rect(0, 10, 10, 20)];

        assert_eq!(enclosed_holes(&rows), Vec::new());
    }

    #[test]
    fn no_rows_enclose_nothing() {
        assert_eq!(enclosed_holes(&[]), Vec::new());
    }
}
