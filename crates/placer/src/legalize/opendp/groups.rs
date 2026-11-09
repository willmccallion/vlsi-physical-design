//! Placement regions, as `OpenDP` places the members of each region's group
//! before the other cells: the sites a region wholly covers become its own,
//! the ones it partly covers are blocked, its cells outside it are moved in
//! and other cells inside it are moved out, its cells are placed, falling
//! back to packing them against its edges, and then refined and annealed.

use super::cpp::{Mt19937, sort};
use super::{
    DetailedPlacementError, GroupId, MAX_DISPLACEMENT_X, MAX_DISPLACEMENT_Y, Node, NodeId,
    Occupant, Placer, area, core_center,
};
use crate::legalize::grid::{GridPt, GridX, GridY};
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::{CellId, IdVec, RegionId};
use pare_common::geom::dbu::{Dbu, DbuArea};
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
    /// Its members, in the order `OpenDB` lists a group's instances.
    members: Vec<Member>,
    util: f64,
}

impl Group {
    /// Its member nodes, in order.
    fn nodes(&self) -> impl Iterator<Item = NodeId> + '_ {
        self.members.iter().map(|m| m.node)
    }
}

/// A node of a group with the rectangle of the region it belongs to.
#[derive(Clone, Copy, Debug)]
struct Member {
    node: NodeId,
    /// The rectangle it is packed toward when its group is placed as
    /// bricks, as `Node::getRegion`.
    home: Rect<Dbu>,
}

impl Placer {
    /// One group per region with its member cells, as
    /// `Opendp::setUpPlacementGroups`: in the order of the DEF's groups, each
    /// member where `OpenDB` lists it.
    ///
    /// # Errors
    ///
    /// When a region with members has no rectangle inside the core.
    pub(super) fn make_groups(&mut self, db: &NetlistDB) -> Result<(), DetailedPlacementError> {
        let mut node_of = IdVec::filled(None, db.cells.len());
        for (n, node) in self.nodes.iter_enumerated() {
            node_of[node.cell] = Some(n);
        }
        let mut regions: Vec<RegionId> = db.regions.ids().collect();
        regions.sort_by_key(|&r| {
            let order = db.regions[r].group_order;
            (order.is_none(), order, r)
        });
        for r in regions {
            let region = &db.regions[r];
            let mut rects = Vec::new();
            for rect in &region.rects {
                let Some(inside) = rect.intersection(&self.rows.core()) else {
                    continue;
                };
                let rel = self.rows.relative(inside);
                self.region_boxes.push(Rect::new(
                    rel.min,
                    Point::new(rel.max.x - Dbu(1), rel.max.y - Dbu(1)),
                ));
                rects.push(rel);
            }
            let g = self.groups.next_id();
            let mut members: Vec<(CellId, NodeId)> = db
                .cells
                .ids()
                .filter(|&i| {
                    let c = &db.cells[i];
                    !c.retired && c.region == Some(r)
                })
                .filter_map(|i| node_of[i].map(|n| (i, n)))
                .collect();
            members
                .sort_by_key(|&(i, _)| (db.cells[i].region_member_order(), std::cmp::Reverse(i)));
            let mut group_members = Vec::with_capacity(members.len());
            for (_, n) in members {
                // NOTE: Opendp::setUpPlacementGroups keeps a region off the core
                // and brickPlace2 dereferences its missing rectangle; PARE rejects it.
                let home = home_rect(&self.nodes[n], &rects).ok_or_else(|| {
                    DetailedPlacementError::RegionOutsideCore(region.name.clone())
                })?;
                self.nodes[n].group = Some(g);
                group_members.push(Member { node: n, home });
            }
            let Some(bbox) = rects.iter().copied().reduce(|a, b| a.union(&b)) else {
                // Off the core and so, having no members, nothing to place.
                continue;
            };
            let _ = self.groups.push(Group {
                rects,
                bbox,
                members: group_members,
                util: 0.0,
            });
        }
        Ok(())
    }

    /// Blocks every site a region rectangle overlaps without covering, as
    /// `Opendp::groupInitPixels2`.
    pub(super) fn mark_region_edges(&mut self) {
        let rects: Vec<Rect<Dbu>> = self.groups.iter().flat_map(|g| g.rects.clone()).collect();
        for rect in rects {
            let x0 = self.rows.grid_x(rect.min.x).max(GridX::ZERO);
            let x1 = self
                .rows
                .grid_end_x(rect.max.x)
                .min(self.rows.row_site_count());
            let y0 = self.rows.snap_down_y(rect.min.y).max(GridY::ZERO);
            let y1 = self.rows.end_y(rect.max.y).min(self.rows.row_count());
            for x in x0.until(x1) {
                for y in y0.until(y1) {
                    let sub = Rect::new(
                        Point::new(x.to_dbu(self.rows.site_width()), self.rows.row_y(y)),
                        Point::new(
                            (x + 1).to_dbu(self.rows.site_width()),
                            self.rows.row_y(y + 1),
                        ),
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
                        p.set_occupant(Occupant::Dummy);
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
        let site = self.rows.site_width().get();
        for g in self.groups.ids() {
            if self.groups[g].members.is_empty() {
                if g != GroupId(0) {
                    log::warn!("No cells found in group {}.", g.index());
                }
                continue;
            }
            let rects = self.groups[g].rects.clone();
            for rect in &rects {
                let within = self.grid_within(rect);
                let (x0, x1) = (within.min.x, within.max.x);
                for y in within.min.y.until(within.max.y) {
                    for x in x0.until(x1) {
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
            for rect in &rects {
                let within = self.grid_within(rect);
                for y in within.min.y.until(within.max.y) {
                    for x in within.min.x.until(within.max.x) {
                        let Some(p) = self.pixel_mut(x, y) else {
                            continue;
                        };
                        if p.util == 1.0 {
                            p.set_group(g);
                            p.valid = true;
                            p.util = 1.0;
                        } else if p.util > 0.0 && p.util < 1.0 {
                            p.set_occupant(Occupant::Dummy);
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
        self.measure_group_utilization();
        self.pre_place_groups();
        self.pre_place();
        self.place_group_cells()?;
        for g in self.groups.ids() {
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

    /// Each group's utilization, as `Opendp::groupAssignCellRegions`
    /// measures it; [`Self::make_groups`] gives each member its rectangle.
    fn measure_group_utilization(&mut self) {
        for g in self.groups.ids() {
            let mut site_area = DbuArea::ZERO;
            if !self.groups[g].members.is_empty() {
                for x in GridX::ZERO.until(self.rows.row_site_count()) {
                    for y in GridY::ZERO.until(self.rows.row_count()) {
                        if self
                            .pixel(x, y)
                            .is_some_and(|p| p.valid && p.group() == Some(g))
                        {
                            site_area += self.pixel_height(y) * self.rows.site_width();
                        }
                    }
                }
            }
            let cell_area: f64 = self.groups[g]
                .nodes()
                .map(|n| area(&self.nodes[n]).to_f64())
                .sum();
            self.groups[g].util = if site_area == DbuArea::ZERO {
                0.0
            } else {
                cell_area / site_area.to_f64()
            };
        }
    }

    /// The height `OpenDP` gives grid row `y`: from the row edge below it,
    /// as `Grid::rowHeight`.
    fn pixel_height(&self, y: GridY) -> Dbu {
        let below = if y > GridY::ZERO {
            self.rows.row_y(y - 1)
        } else {
            Dbu::ZERO
        };
        self.rows.row_y(y) - below
    }

    /// Moves each group's members that lie outside its region into it, as
    /// `Opendp::prePlaceGroups`.
    fn pre_place_groups(&mut self) {
        for g in self.groups.ids() {
            for k in 0..self.groups[g].members.len() {
                let n = self.groups[g].members[k].node;
                let node = &self.nodes[n];
                let rects = &self.groups[g].rects;
                if node.fixed || node.placed || rects.iter().any(|r| is_inside(node, r)) {
                    continue;
                }
                let Some(rect) = nearest_rect(node, rects) else {
                    continue;
                };
                let to = nearest_pt(node, &rect);
                let legal = self.legal_grid_pt_at(n, to);
                if self.diamond_move_from(n, legal) {
                    self.nodes[n].hold = true;
                }
            }
        }
    }

    /// Moves the cells outside every group that sit over a region's left
    /// edge out of it, as `Opendp::prePlace`.
    fn pre_place(&mut self) {
        for n in self.nodes.ids() {
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
    fn legal_grid_pt_at(&self, n: NodeId, at: Point<Dbu>) -> GridPt {
        let legal = self.rows.legal_pt(at, self.nodes[n].size());
        GridPt::new(self.rows.grid_x(legal.x), self.rows.snap_down_y(legal.y))
    }

    /// Places each group's remaining members, packing them against the
    /// region's corners when one does not fit, as `Opendp::placeGroups2`.
    fn place_group_cells(&mut self) -> Result<(), DetailedPlacementError> {
        for g in self.groups.ids() {
            let mut order: Vec<NodeId> = self.groups[g]
                .nodes()
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
            for k in 0..self.groups[g].members.len() {
                let n = self.groups[g].members[k].node;
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
    fn brick_place_outer(&mut self, g: GroupId) -> Result<(), DetailedPlacementError> {
        let boundary = self.groups[g].bbox;
        let mut order: Vec<NodeId> = self.groups[g].nodes().collect();
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
    fn brick_place_inner(&mut self, g: GroupId) -> Result<(), DetailedPlacementError> {
        let mut order = self.groups[g].members.clone();
        sort(&mut order, &|a, b| {
            corner_dist(&self.nodes[a.node], &a.home) < corner_dist(&self.nodes[b.node], &b.home)
        });
        for Member { node: n, home } in order {
            if self.nodes[n].hold {
                continue;
            }
            let to = corner(&self.nodes[n], &home);
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
    fn group_refine(&mut self, g: GroupId) -> usize {
        let mut by_displacement: Vec<NodeId> = self.groups[g].nodes().collect();
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
    fn refine_move(&mut self, n: NodeId) -> bool {
        let grid_pt = self.legal_grid_pt(n);
        let Some(to) = self.diamond_search(n, grid_pt) else {
            return false;
        };
        if (grid_pt.x - to.x).abs() > MAX_DISPLACEMENT_X
            || (grid_pt.y - to.y).abs() > MAX_DISPLACEMENT_Y
        {
            return false;
        }
        let at = Point::new(to.x.to_dbu(self.rows.site_width()), self.rows.row_y(to.y));
        if self.dist_change(n, at) < Dbu::ZERO {
            self.unplace_cell(n);
            self.place_cell(n, to);
            return true;
        }
        false
    }

    /// How much nearer to where it started a cell would be at `at`, as
    /// `Opendp::distChange`.
    fn dist_change(&self, n: NodeId, at: Point<Dbu>) -> Dbu {
        let node = &self.nodes[n];
        let now = (node.left - node.init.x).abs() + (node.bottom - node.init.y).abs();
        let then = (node.init.x - at.x).abs() + (node.init.y - at.y).abs();
        then - now
    }

    /// Swaps random pairs of a group's cells when that brings them nearer to
    /// where they started, as `Opendp::anneal`; returns how many swapped.
    fn anneal(&mut self, g: GroupId) -> usize {
        let mut generator = Mt19937::new(RANDOM_SEED);
        let cells: Vec<NodeId> = self.groups[g].nodes().collect();
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
    fn swap_cells(&mut self, a: NodeId, b: NodeId) -> bool {
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
        let (a_at, b_at) = (
            Point::new(na.left, na.bottom),
            Point::new(nb.left, nb.bottom),
        );
        let change = self.dist_change(a, b_at) + self.dist_change(b, a_at);
        if change >= Dbu::ZERO {
            return false;
        }
        let (a_was, b_was) = ((a_at, na.placed), (b_at, nb.placed));
        let a_to = GridPt::new(self.rows.grid_x(nb.left), self.rows.snap_down_y(nb.bottom));
        let b_to = GridPt::new(self.rows.grid_x(na.left), self.rows.snap_down_y(na.bottom));
        self.unplace_cell(a);
        self.unplace_cell(b);
        self.place_cell(a, a_to);
        self.place_cell(b, b_to);
        if self.check_drc_here(a) && self.check_drc_here(b) {
            return true;
        }
        self.undo_move(b, b_was);
        self.undo_move(a, a_was);
        false
    }

    /// The placement rules checked where a cell stands, as
    /// `PlacementDRC::checkDRC`.
    fn check_drc_here(&self, n: NodeId) -> bool {
        let node = &self.nodes[n];
        let (x, y) = (self.rows.grid_x(node.left), self.rows.round_y(node.bottom));
        self.check_edge_spacing(node, x, y, node.orient)
            && self.check_blocked_layers(node, x, y)
            && self.check_one_site_gap(node, x, y)
    }

    /// Puts a moved cell back, as the journal undoes a `MoveCellAction`:
    /// its pixels cleared where it stands, its place restored and painted
    /// in that row's orientation when it was placed.
    fn undo_move(&mut self, n: NodeId, (at, was_placed): (Point<Dbu>, bool)) {
        self.erase_pixels(n);
        self.nodes[n].left = at.x;
        self.nodes[n].bottom = at.y;
        if !was_placed {
            self.nodes[n].placed = false;
            return;
        }
        let (gx, gy) = (self.rows.grid_x(at.x), self.rows.round_y(at.y));
        self.paint_pixels(n, gx, gy);
        if let Some(orient) = self.site_orientation(gx, gy, self.nodes[n].site) {
            self.nodes[n].orient = orient;
        }
    }

    /// The order cells are placed in, as `CellPlaceOrderLess`: taller than
    /// a row first, then larger, then nearer the core's center, then by
    /// name.
    pub(super) fn place_order_less(&self) -> impl Fn(&Node, &Node) -> bool + use<> {
        let center = core_center(&self.rows.core());
        move |a: &Node, b: &Node| {
            let dist = |n: &Node| (n.left - center.x).abs() + (n.bottom - center.y).abs();
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
fn is_inside(node: &Node, rect: &Rect<Dbu>) -> bool {
    let Point { x, y } = node.init;
    x >= rect.min.x
        && x + node.width <= rect.max.x
        && y >= rect.min.y
        && y + node.height <= rect.max.y
}

/// The rectangle of a region a member belongs to: the last that holds its
/// start, else the nearest; none when the region has no rectangle.
// NOTE: Opendp::groupAssignCellRegions falls back to the first rectangle,
// which can pack a member far from its start; PARE takes the nearest.
fn home_rect(node: &Node, rects: &[Rect<Dbu>]) -> Option<Rect<Dbu>> {
    rects
        .iter()
        .rev()
        .find(|r| is_inside(node, r))
        .copied()
        .or_else(|| nearest_rect(node, rects))
}

/// The rectangle nearest a cell's start, the first of equals, as
/// `Opendp::prePlaceGroups` picks it.
fn nearest_rect(node: &Node, rects: &[Rect<Dbu>]) -> Option<Rect<Dbu>> {
    rects.iter().copied().min_by_key(|r| dist_to_rect(node, r))
}

/// How far a cell's starting position lies outside a rectangle, as
/// `Opendp::distToRect`.
fn dist_to_rect(node: &Node, rect: &Rect<Dbu>) -> Dbu {
    let Point { x, y } = node.init;
    let dx = if x < rect.min.x {
        rect.min.x - x
    } else if x + node.width > rect.max.x {
        x + node.width - rect.max.x
    } else {
        Dbu::ZERO
    };
    let dy = if y < rect.min.y {
        rect.min.y - y
    } else if y + node.height > rect.max.y {
        y + node.height - rect.max.y
    } else {
        Dbu::ZERO
    };
    dx + dy
}

/// Whether a cell's starting position straddles a rectangle's left edge,
/// as the `Node` form of `Opendp::checkOverlap` tests it.
fn overlaps_left_edge(node: &Node, rect: &Rect<Dbu>) -> bool {
    let Point { x, y } = node.init;
    x + node.width > rect.min.x && x < rect.min.x && y + node.height > rect.min.y && y < rect.max.y
}

/// The nearest position to a cell's start inside a rectangle, or just out
/// of it across its nearest edge when the cell straddles its left edge, as
/// `Opendp::nearestPt`.
fn nearest_pt(node: &Node, rect: &Rect<Dbu>) -> Point<Dbu> {
    let Point { x, y } = node.init;
    let (xl, yl, xh, yh) = (rect.min.x, rect.min.y, rect.max.x, rect.max.y);
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
            Point::new(to_x, y)
        } else {
            Point::new(x, to_y)
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
    Point::new(to_x, to_y)
}

/// The corner of a rectangle on the side of its center a cell's start is
/// on, as `Opendp::rectDist` finds it.
fn corner(node: &Node, rect: &Rect<Dbu>) -> Point<Dbu> {
    let Point { x, y } = node.init;
    let cx = Dbu(i64::midpoint(rect.min.x.get(), rect.max.x.get()));
    let cy = Dbu(i64::midpoint(rect.min.y.get(), rect.max.y.get()));
    Point::new(
        if x > cx { rect.max.x } else { rect.min.x },
        if y > cy { rect.max.y } else { rect.min.y },
    )
}

/// How far a cell's start is from that corner, as `Opendp::rectDist`.
fn corner_dist(node: &Node, rect: &Rect<Dbu>) -> Dbu {
    let c = corner(node, rect);
    (node.init.x - c.x).abs() + (node.init.y - c.y).abs()
}

/// How far a cell stands from where it started, as `Opendp::disp`.
fn displacement(node: &Node) -> Dbu {
    (node.init.x - node.left).abs() + (node.init.y - node.bottom).abs()
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

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(Point::new(Dbu(x0), Dbu(y0)), Point::new(Dbu(x1), Dbu(y1)))
    }

    /// The tiny design with `u1` and `u2` fenced into a region of a first
    /// rectangle at the bottom left and a second in the top row, and `u1`
    /// starting outside both, nearer the second.
    fn tiny_with_two_rectangle_fence() -> NetlistDB {
        let text = std::fs::read_to_string(fixture("tiny.def"))
            .unwrap()
            .replace(
                "COMPONENTS 8 ;",
                "REGIONS 1 ;\n- fence ( 0 0 ) ( 2000 2000 ) ( 3000 4000 ) ( 4000 6000 ) \
                 + TYPE FENCE ;\nEND REGIONS\nCOMPONENTS 8 ;",
            )
            .replace(
                "END DESIGN",
                "GROUPS 1 ;\n- g u1 u2 + REGION fence ;\nEND GROUPS\nEND DESIGN",
            );
        let units = def::read_units(&fixture("tiny.def")).unwrap();
        let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
        let mut db = def::parse_str(&text, Path::new("fenced.def"), library).unwrap();
        let u1 = db.cell_id("u1").unwrap();
        db.cells[u1].location = Point::new(Dbu(3800), Dbu(3000));
        db
    }

    #[test]
    fn a_member_outside_its_region_is_packed_into_the_nearest_rectangle() {
        let db = tiny_with_two_rectangle_fence();
        let mut placer = Placer::new(&db).unwrap();
        placer.paint_fixed_cells();
        placer.mark_region_edges();
        placer.mark_region_sites();
        placer.measure_group_utilization();
        let u1 = placer
            .nodes
            .ids()
            .find(|&n| placer.nodes[n].name == "u1")
            .unwrap();

        placer.brick_place_inner(GroupId(0)).unwrap();

        assert!(rect(3000, 4000, 4000, 6000).encloses(&placer.nodes[u1].bbox()));
    }
}
