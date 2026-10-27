//! Placement regions, as `OpenDP` places the members of each region's group
//! before the other cells: the sites a region wholly covers become its own,
//! the ones it partly covers are blocked, its cells outside it are moved in
//! and other cells inside it are moved out, its cells are placed, falling
//! back to packing them against its edges, and then refined and annealed.

use super::cpp::{Mt19937, sort};
use super::{
    DUMMY_CELL, DetailedPlacementError, GridPt, MAX_DISPLACEMENT_X, MAX_DISPLACEMENT_Y, Node,
    Placer, area,
};
use pare_common::db::design::NetlistDB;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;

/// Utilization above which a region's cells are packed from its outer
/// corners rather than from the corners of their own rectangles.
const BRICK_UTILIZATION: f64 = 0.95;
/// The share of a group's cells, the most displaced, refinement tries to
/// move.
const GROUP_REFINE_PERCENT: f64 = 0.05;
/// The seed of `OpenDP`'s annealing.
const RANDOM_SEED: u32 = 777;
/// Refinement and annealing passes per group.
const PASSES: usize = 3;

/// A region's group, as `OpenDP`'s `Group`.
#[derive(Clone, Debug)]
pub(super) struct Group {
    /// The region's rectangles within the core, relative to it.
    pub(super) rects: Vec<Rect<Dbu>>,
    pub(super) bbox: Rect<Dbu>,
    /// Its member nodes, in the order `OpenDB` lists a group's instances.
    pub(super) cells: Vec<usize>,
    util: f64,
}

impl Placer {
    /// One group per region with its member cells, as
    /// `Opendp::setUpPlacementGroups`: in the order of the DEF's groups, each
    /// member where `OpenDB` lists it.
    pub(super) fn make_groups(&mut self, db: &NetlistDB) {
        let mut node_of = vec![usize::MAX; db.cells.len()];
        for (n, node) in self.nodes.iter().enumerate() {
            node_of[node.cell.index()] = n;
        }
        let mut regions: Vec<usize> = (0..db.regions.len()).collect();
        regions.sort_by_key(|&r| (db.regions[r].group_order.unwrap_or(usize::MAX), r));
        for r in regions {
            let region = &db.regions[r];
            let mut rects = Vec::new();
            for rect in &region.rects {
                let Some(inside) = rect.intersection(&self.core) else {
                    continue;
                };
                let rel = self.relative(inside);
                self.region_boxes.push(Rect::new(
                    rel.min,
                    Point::new(rel.max.x - Dbu(1), rel.max.y - Dbu(1)),
                ));
                rects.push(rel);
            }
            let bbox = rects
                .iter()
                .copied()
                .reduce(|a, b| a.union(&b))
                .unwrap_or(Rect::new(
                    Point::new(Dbu(0), Dbu(0)),
                    Point::new(Dbu(0), Dbu(0)),
                ));
            let g = self.groups.len();
            let mut members: Vec<usize> = (0..db.cells.len())
                .filter(|&i| {
                    let c = &db.cells[i];
                    !c.retired
                        && c.region.map(pare_common::db::indices::RegionId::index) == Some(r)
                        && node_of[i] != usize::MAX
                })
                .collect();
            members.sort_by_key(|&i| (db.cells[i].group_rank, std::cmp::Reverse(i)));
            let mut cells = Vec::with_capacity(members.len());
            for i in members {
                let n = node_of[i];
                self.nodes[n].group = Some(g);
                cells.push(n);
            }
            self.groups.push(Group {
                rects,
                bbox,
                cells,
                util: 0.0,
            });
        }
    }

    /// Blocks every site a region rectangle overlaps without covering, as
    /// `Opendp::groupInitPixels2`.
    pub(super) fn mark_region_edges(&mut self) {
        let rects: Vec<Rect<Dbu>> = self.groups.iter().flat_map(|g| g.rects.clone()).collect();
        for rect in rects {
            let x0 = (rect.min.x.get() / self.site_width).max(0);
            let x1 = self.grid_end_x(rect.max.x.get()).min(self.row_site_count);
            let y0 = self.snap_down_y(rect.min.y.get()).max(0);
            let y1 = self.end_y(rect.max.y.get()).min(self.row_count);
            for x in x0..x1 {
                for y in y0..y1 {
                    let sub = Rect::new(
                        Point::new(Dbu(x * self.site_width), Dbu(self.row_y(y))),
                        Point::new(Dbu((x + 1) * self.site_width), Dbu(self.row_y(y + 1))),
                    );
                    let inside = rect.min.x <= sub.min.x
                        && sub.max.x <= rect.max.x
                        && rect.min.y <= sub.min.y
                        && sub.max.y <= rect.max.y;
                    let overlaps = rect.min.x < sub.max.x
                        && rect.max.x > sub.min.x
                        && rect.min.y < sub.max.y
                        && rect.max.y > sub.min.y;
                    if !inside
                        && overlaps
                        && let Some(p) = self.pixel_mut(x, y)
                    {
                        p.util = 0.0;
                        p.cell = DUMMY_CELL;
                        p.valid = false;
                    }
                }
            }
        }
    }

    /// Gives each group the sites its region covers, as
    /// `Opendp::groupInitPixels`.
    // A site is the region's when its share is exactly one, as OpenDP tests it.
    #[allow(clippy::float_cmp)]
    pub(super) fn mark_region_sites(&mut self) {
        for p in &mut self.pixels {
            p.util = 0.0;
        }
        let site = self.site_width;
        for g in 0..self.groups.len() {
            if self.groups[g].cells.is_empty() {
                if g != 0 {
                    log::warn!("No cells found in group {g}.");
                }
                continue;
            }
            let rects = self.groups[g].rects.clone();
            for rect in &rects {
                let (x0, y0, x1, y1) = self.grid_within(rect);
                for y in y0..y1 {
                    for x in x0..x1 {
                        if let Some(p) = self.pixel_mut(x, y) {
                            p.util += 1.0;
                        }
                    }
                    if rect.min.x.get() % site != 0
                        && let Some(p) = self.pixel_mut(x0, y)
                    {
                        p.util = (f64::from(p.util)
                            - (rect.min.x.get() % site) as f64 / site as f64)
                            as f32;
                    }
                    if rect.max.x.get() % site != 0
                        && let Some(p) = self.pixel_mut(x1 - 1, y)
                    {
                        p.util = (f64::from(p.util)
                            - ((site - rect.max.x.get()) % site) as f64 / site as f64)
                            as f32;
                    }
                }
            }
            let id = u32::try_from(g).unwrap_or(u32::MAX);
            for rect in &rects {
                let (x0, y0, x1, y1) = self.grid_within(rect);
                for y in y0..y1 {
                    for x in x0..x1 {
                        let Some(p) = self.pixel_mut(x, y) else {
                            continue;
                        };
                        if p.util == 1.0 {
                            p.group = id;
                            p.valid = true;
                            p.util = 1.0;
                        } else if p.util > 0.0 && p.util < 1.0 {
                            p.cell = DUMMY_CELL;
                            p.util = 0.0;
                            p.valid = false;
                        }
                    }
                }
            }
        }
    }

    /// Places every group's cells, as `Opendp::placeGroups`.
    pub(super) fn place_groups(&mut self) -> Result<(), DetailedPlacementError> {
        self.assign_cell_regions();
        self.pre_place_groups();
        self.pre_place();
        self.place_group_cells()?;
        for g in 0..self.groups.len() {
            for _ in 0..PASSES {
                let refined = self.group_refine(g);
                let annealed = self.anneal(g);
                if refined < 10 || annealed < 100 {
                    break;
                }
            }
        }
        Ok(())
    }

    /// Each group's utilization and each member's rectangle, as
    /// `Opendp::groupAssignCellRegions`.
    fn assign_cell_regions(&mut self) {
        for g in 0..self.groups.len() {
            let id = u32::try_from(g).unwrap_or(u32::MAX);
            let mut site_area: i64 = 0;
            if !self.groups[g].cells.is_empty() {
                for x in 0..self.row_site_count {
                    for y in 0..self.row_count {
                        if self.pixel(x, y).is_some_and(|p| p.valid && p.group == id) {
                            site_area += self.pixel_height(y) * self.site_width;
                        }
                    }
                }
            }
            let mut cell_area = 0.0;
            for k in 0..self.groups[g].cells.len() {
                let n = self.groups[g].cells[k];
                cell_area += area(&self.nodes[n]) as f64;
                let mut region = None;
                for (r, rect) in self.groups[g].rects.iter().enumerate() {
                    if is_inside(&self.nodes[n], rect) {
                        region = Some(r);
                    }
                }
                self.nodes[n].region = Some(region.unwrap_or(0));
            }
            self.groups[g].util = if site_area == 0 {
                0.0
            } else {
                cell_area / site_area as f64
            };
        }
    }

    /// The height `OpenDP` gives grid row `y`: from the row edge below it,
    /// as `Grid::rowHeight`.
    fn pixel_height(&self, y: i64) -> i64 {
        let edge = |i: i64| {
            usize::try_from(i)
                .ok()
                .and_then(|i| self.row_edges.get(i))
                .copied()
        };
        edge(y).unwrap_or(0) - if y > 0 { edge(y - 1).unwrap_or(0) } else { 0 }
    }

    /// Moves each group's members that lie outside its region into it, as
    /// `Opendp::prePlaceGroups`.
    fn pre_place_groups(&mut self) {
        for g in 0..self.groups.len() {
            for k in 0..self.groups[g].cells.len() {
                let n = self.groups[g].cells[k];
                if self.nodes[n].fixed || self.nodes[n].placed {
                    continue;
                }
                let mut distance = i64::MAX;
                let mut inside = false;
                let mut nearest = None;
                for rect in &self.groups[g].rects {
                    if is_inside(&self.nodes[n], rect) {
                        inside = true;
                    }
                    let d = dist_to_rect(&self.nodes[n], rect);
                    if d < distance {
                        distance = d;
                        nearest = Some(*rect);
                    }
                }
                let Some(rect) = nearest else {
                    continue;
                };
                if !inside {
                    let to = nearest_pt(&self.nodes[n], &rect);
                    let legal = self.legal_grid_pt_at(n, to);
                    if self.diamond_move_from(n, legal) {
                        self.nodes[n].hold = true;
                    }
                }
            }
        }
    }

    /// Moves the cells outside every group that sit over a region's left
    /// edge out of it, as `Opendp::prePlace`.
    fn pre_place(&mut self) {
        for n in 0..self.nodes.len() {
            let node = &self.nodes[n];
            if node.group.is_some() || node.placed {
                continue;
            }
            let mut over = None;
            for group in &self.groups {
                for rect in &group.rects {
                    if overlaps_left_edge(node, rect) {
                        over = Some(*rect);
                    }
                }
            }
            if let Some(rect) = over {
                let to = nearest_pt(node, &rect);
                let legal = self.legal_grid_pt_at(n, to);
                if self.diamond_move_from(n, legal) {
                    self.nodes[n].hold = true;
                }
            }
        }
    }

    /// The grid point of a legal position near `at`, as `Opendp::legalGridPt`
    /// with a point.
    fn legal_grid_pt_at(&self, n: usize, at: (i64, i64)) -> GridPt {
        let (x, y) = self.legal_pt(&self.nodes[n], at);
        (self.grid_x(x), self.snap_down_y(y))
    }

    /// Places each group's remaining members, packing them against the
    /// region's corners when one does not fit, as `Opendp::placeGroups2`.
    fn place_group_cells(&mut self) -> Result<(), DetailedPlacementError> {
        for g in 0..self.groups.len() {
            let mut order: Vec<usize> = self.groups[g]
                .cells
                .iter()
                .copied()
                .filter(|&n| !self.nodes[n].fixed && !self.nodes[n].placed)
                .collect();
            let less = self.place_order_less();
            sort(&mut order, &|&a, &b| less(&self.nodes[a], &self.nodes[b]));
            let mut pass = true;
            for n in order {
                if !self.nodes[n].fixed && !self.nodes[n].placed {
                    pass = self.diamond_move(n);
                    if !pass {
                        break;
                    }
                }
            }
            if pass {
                continue;
            }
            for k in 0..self.groups[g].cells.len() {
                let n = self.groups[g].cells[k];
                self.unplace_cell(n);
            }
            if self.groups[g].util > BRICK_UTILIZATION {
                self.brick_place_outer(g)?;
            } else {
                self.brick_place_inner(g)?;
            }
        }
        Ok(())
    }

    /// Packs a group's cells toward the corners of its region's bounding
    /// box, nearest first, as `Opendp::brickPlace1`.
    fn brick_place_outer(&mut self, g: usize) -> Result<(), DetailedPlacementError> {
        let boundary = self.groups[g].bbox;
        let mut order = self.groups[g].cells.clone();
        sort(&mut order, &|&a, &b| {
            corner_dist(&self.nodes[a], &boundary) < corner_dist(&self.nodes[b], &boundary)
        });
        for n in order {
            let to = corner(&self.nodes[n], &boundary);
            let legal = self.legal_grid_pt_at(n, to);
            if !self.diamond_move_from(n, legal) {
                return Err(DetailedPlacementError::Failed(vec![
                    self.nodes[n].name.clone(),
                ]));
            }
        }
        Ok(())
    }

    /// Packs a group's cells toward the corners of their own rectangles,
    /// nearest first, as `Opendp::brickPlace2`.
    fn brick_place_inner(&mut self, g: usize) -> Result<(), DetailedPlacementError> {
        let rects = self.groups[g].rects.clone();
        let rect_of = |node: &Node| rects[node.region.unwrap_or(0)];
        let mut order = self.groups[g].cells.clone();
        sort(&mut order, &|&a, &b| {
            let (na, nb) = (&self.nodes[a], &self.nodes[b]);
            corner_dist(na, &rect_of(na)) < corner_dist(nb, &rect_of(nb))
        });
        for n in order {
            if self.nodes[n].hold {
                continue;
            }
            let to = corner(&self.nodes[n], &rect_of(&self.nodes[n]));
            let legal = self.legal_grid_pt_at(n, to);
            if !self.diamond_move_from(n, legal) {
                return Err(DetailedPlacementError::Failed(vec![
                    self.nodes[n].name.clone(),
                ]));
            }
        }
        Ok(())
    }

    /// Moves the most displaced twentieth of a group's cells nearer to where
    /// they started, as `Opendp::groupRefine`; returns how many moved.
    fn group_refine(&mut self, g: usize) -> usize {
        let mut by_displacement = self.groups[g].cells.clone();
        sort(&mut by_displacement, &|&a, &b| {
            displacement(&self.nodes[a]) > displacement(&self.nodes[b])
        });
        let mut count = 0;
        let limit = by_displacement.len() as f64 * GROUP_REFINE_PERCENT;
        for (i, &n) in by_displacement.iter().enumerate() {
            if i as f64 >= limit {
                break;
            }
            if !self.nodes[n].hold && !self.nodes[n].fixed && self.refine_move(n) {
                count += 1;
            }
        }
        count
    }

    /// Moves a cell to the free spot nearest its legal position when that
    /// brings it nearer to where it started, as `Opendp::refineMove`.
    fn refine_move(&mut self, n: usize) -> bool {
        let grid_pt = self.legal_grid_pt(n);
        let Some((x, y)) = self.diamond_search(n, grid_pt) else {
            return false;
        };
        if (grid_pt.0 - x).abs() > MAX_DISPLACEMENT_X || (grid_pt.1 - y).abs() > MAX_DISPLACEMENT_Y
        {
            return false;
        }
        if self.dist_change(n, x * self.site_width, self.row_y(y)) < 0 {
            self.unplace_cell(n);
            self.place_cell(n, x, y);
            return true;
        }
        false
    }

    /// How much nearer to where it started a cell would be at `(x, y)`, as
    /// `Opendp::distChange`.
    fn dist_change(&self, n: usize, x: i64, y: i64) -> i64 {
        let node = &self.nodes[n];
        let now = (node.left - node.init.0).abs() + (node.bottom - node.init.1).abs();
        let then = (node.init.0 - x).abs() + (node.init.1 - y).abs();
        then - now
    }

    /// Swaps random pairs of a group's cells when that brings them nearer to
    /// where they started, as `Opendp::anneal`; returns how many swapped.
    fn anneal(&mut self, g: usize) -> usize {
        let mut generator = Mt19937::new(RANDOM_SEED);
        let cells = self.groups[g].cells.clone();
        let Ok(high) = i32::try_from(cells.len()) else {
            return 0;
        };
        let mut count = 0;
        for _ in 0..100 * cells.len() {
            let a = generator.uniform_int(0, high - 1);
            let b = generator.uniform_int(0, high - 1);
            let (Ok(a), Ok(b)) = (usize::try_from(a), usize::try_from(b)) else {
                continue;
            };
            if self.swap_cells(cells[a], cells[b]) {
                count += 1;
            }
        }
        count
    }

    /// Swaps two cells of one size when that brings them nearer to where
    /// they started and breaks no placement rule, as `Opendp::swapCells`.
    fn swap_cells(&mut self, a: usize, b: usize) -> bool {
        let (na, nb) = (&self.nodes[a], &self.nodes[b]);
        if a == b
            || na.hold
            || nb.hold
            || na.width != nb.width
            || na.height != nb.height
            || na.fixed
            || nb.fixed
        {
            return false;
        }
        let change =
            self.dist_change(a, nb.left, nb.bottom) + self.dist_change(b, na.left, na.bottom);
        if change >= 0 {
            return false;
        }
        let (a_was, b_was) = (
            (na.left, na.bottom, na.placed),
            (nb.left, nb.bottom, nb.placed),
        );
        let (ax, ay) = (self.grid_x(nb.left), self.snap_down_y(nb.bottom));
        let (bx, by) = (self.grid_x(na.left), self.snap_down_y(na.bottom));
        self.unplace_cell(a);
        self.unplace_cell(b);
        self.place_cell(a, ax, ay);
        self.place_cell(b, bx, by);
        if self.check_drc_here(a) && self.check_drc_here(b) {
            return true;
        }
        self.undo_move(b, b_was);
        self.undo_move(a, a_was);
        false
    }

    /// The placement rules checked where a cell stands, as
    /// `PlacementDRC::checkDRC`.
    fn check_drc_here(&self, n: usize) -> bool {
        let node = &self.nodes[n];
        let (x, y) = (self.grid_x(node.left), self.round_y(node.bottom));
        self.check_edge_spacing(node, x, y, node.orient)
            && self.check_blocked_layers(node, x, y)
            && self.check_one_site_gap(node, x, y)
    }

    /// Puts a moved cell back, as the journal undoes a `MoveCellAction`:
    /// its pixels cleared where it stands, its place restored and painted
    /// in that row's orientation when it was placed.
    fn undo_move(&mut self, n: usize, (left, bottom, was_placed): (i64, i64, bool)) {
        let node = &self.nodes[n];
        let rect = Rect::new(
            Point::new(Dbu(node.left), Dbu(node.bottom)),
            Point::new(Dbu(node.left + node.width), Dbu(node.bottom + node.height)),
        );
        let (x0, y0, x1, y1) = self.covering(&rect);
        let id = u32::try_from(n).unwrap_or(super::NO_CELL);
        for x in x0..x1 {
            for y in y0..y1 {
                if let Some(p) = self.pixel_mut(x, y)
                    && p.cell == id
                {
                    p.cell = super::NO_CELL;
                }
            }
        }
        self.nodes[n].left = left;
        self.nodes[n].bottom = bottom;
        if !was_placed {
            self.nodes[n].placed = false;
            return;
        }
        let (gx, gy) = (self.grid_x(left), self.round_y(bottom));
        let x_end = gx + self.grid_width(&self.nodes[n]);
        let y_end = self.end_y(self.row_y(gy) + self.nodes[n].height);
        for x in gx..x_end {
            for y in gy..y_end {
                if let Some(p) = self.pixel_mut(x, y) {
                    p.cell = id;
                }
            }
        }
        if let Some(orient) = self.site_orientation(gx, gy, self.nodes[n].site) {
            self.nodes[n].orient = orient;
        }
    }

    /// The order cells are placed in, as `CellPlaceOrderLess`: taller than
    /// a row first, then larger, then nearer the core's center, then by
    /// name.
    pub(super) fn place_order_less(&self) -> impl Fn(&Node, &Node) -> bool + use<> {
        let center = (
            i64::midpoint(self.core.min.x.get(), self.core.max.x.get()),
            i64::midpoint(self.core.min.y.get(), self.core.max.y.get()),
        );
        move |a: &Node, b: &Node| {
            let dist = |n: &Node| (n.left - center.0).abs() + (n.bottom - center.1).abs();
            if a.multi_row != b.multi_row {
                return a.multi_row;
            }
            let (area_a, area_b) = (area(a), area(b));
            let (dist_a, dist_b) = (dist(a), dist(b));
            area_a > area_b
                || (area_a == area_b && (dist_a < dist_b || (dist_a == dist_b && a.name < b.name)))
        }
    }
}

/// Whether a cell's starting position lies wholly in a rectangle, as
/// `Opendp::isInside`.
const fn is_inside(node: &Node, rect: &Rect<Dbu>) -> bool {
    let (x, y) = node.init;
    x >= rect.min.x.get()
        && x + node.width <= rect.max.x.get()
        && y >= rect.min.y.get()
        && y + node.height <= rect.max.y.get()
}

/// How far a cell's starting position lies outside a rectangle, as
/// `Opendp::distToRect`.
const fn dist_to_rect(node: &Node, rect: &Rect<Dbu>) -> i64 {
    let (x, y) = node.init;
    let dx = if x < rect.min.x.get() {
        rect.min.x.get() - x
    } else if x + node.width > rect.max.x.get() {
        x + node.width - rect.max.x.get()
    } else {
        0
    };
    let dy = if y < rect.min.y.get() {
        rect.min.y.get() - y
    } else if y + node.height > rect.max.y.get() {
        y + node.height - rect.max.y.get()
    } else {
        0
    };
    dx + dy
}

/// Whether a cell's starting position straddles a rectangle's left edge,
/// as the `Node` form of `Opendp::checkOverlap` tests it.
const fn overlaps_left_edge(node: &Node, rect: &Rect<Dbu>) -> bool {
    let (x, y) = node.init;
    x + node.width > rect.min.x.get()
        && x < rect.min.x.get()
        && y + node.height > rect.min.y.get()
        && y < rect.max.y.get()
}

/// The nearest position to a cell's start inside a rectangle, or just out
/// of it across its nearest edge when the cell straddles its left edge, as
/// `Opendp::nearestPt`.
const fn nearest_pt(node: &Node, rect: &Rect<Dbu>) -> (i64, i64) {
    let (x, y) = node.init;
    let (xl, yl, xh, yh) = (
        rect.min.x.get(),
        rect.min.y.get(),
        rect.max.x.get(),
        rect.max.y.get(),
    );
    if overlaps_left_edge(node, rect) {
        let (dist_x, to_x) = if (x + node.width - xl).abs() > (xh - x).abs() {
            ((xh - x).abs(), xh)
        } else {
            ((x - xl).abs(), xl - node.width)
        };
        let (dist_y, to_y) = if (y + node.height - yl).abs() > (yh - y).abs() {
            ((yh - y).abs(), yh)
        } else {
            ((y - yl).abs(), yl - node.height)
        };
        return if dist_x < dist_y {
            (to_x, y)
        } else {
            (x, to_y)
        };
    }
    let to_x = if x < xl {
        xl
    } else if x + node.width > xh {
        xh - node.width
    } else {
        x
    };
    let to_y = if y < yl {
        yl
    } else if y + node.height > yh {
        yh - node.height
    } else {
        y
    };
    (to_x, to_y)
}

/// The corner of a rectangle on the side of its center a cell's start is
/// on, as `Opendp::rectDist` finds it.
const fn corner(node: &Node, rect: &Rect<Dbu>) -> (i64, i64) {
    let (x, y) = node.init;
    let cx = i64::midpoint(rect.min.x.get(), rect.max.x.get());
    let cy = i64::midpoint(rect.min.y.get(), rect.max.y.get());
    (
        if x > cx {
            rect.max.x.get()
        } else {
            rect.min.x.get()
        },
        if y > cy {
            rect.max.y.get()
        } else {
            rect.min.y.get()
        },
    )
}

/// How far a cell's start is from that corner, as `Opendp::rectDist`.
const fn corner_dist(node: &Node, rect: &Rect<Dbu>) -> i64 {
    let (cx, cy) = corner(node, rect);
    (node.init.0 - cx).abs() + (node.init.1 - cy).abs()
}

/// How far a cell stands from where it started, as `Opendp::disp`.
const fn displacement(node: &Node) -> i64 {
    (node.init.0 - node.left).abs() + (node.init.1 - node.bottom).abs()
}
