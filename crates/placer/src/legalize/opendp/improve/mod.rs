//! `OpenDP`'s detailed improvement, as OpenROAD-flow-scripts runs it with
//! `improve_placement`.
//!
//! A legal placement is cut into row segments between fixed cells and region
//! edges, and five passes then trade wirelength for nothing but legality:
//! set matching of same-sized cells, global and vertical swaps toward each
//! cell's optimal region, reordering of three neighbours, and random moves
//! and swaps nearby, before cells are flipped where that shortens their nets.

// The arithmetic keeps OpenDP's doubles and grouping so results match it.
#![allow(clippy::suboptimal_flops)]

mod arch;
mod mis;
mod moves;
mod network;
mod orient;
mod random;
mod reorder;
mod segments;
mod shift;
mod swap;

use super::cpp::Mt19937;
use super::{DetailedPlacementError, GroupId, NodeId, Placer};
use arch::Arch;
use network::Network;
use pare_common::db::design::NetlistDB;
use pare_common::db::indices::IdVec;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::orient::Orientation;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use std::collections::BTreeSet;

/// The seed `improve_placement` gives the random number generator.
const RANDOM_SEED: u32 = 1;
/// The most moves one trial may make.
const MOVE_LIMIT: usize = 100;
/// Nets with this many pins or more are left out of the wirelength the
/// passes weigh.
const SKIP_NETS_LARGER_THAN: usize = 100;

/// Why detailed improvement stopped.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum ImprovementError {
    /// The design cannot be laid on `OpenDP`'s grid.
    #[error(transparent)]
    Grid(#[from] DetailedPlacementError),
    /// An I/O pin has no shapes to place it by.
    #[error("I/O pin {0} has no shapes")]
    PinWithoutShapes(String),
    /// The improver lost track of its own state, as `OpenDP` reports with
    /// `DPL-400`.
    #[error("detailed improvement internal error: {0}")]
    Internal(&'static str),
}

type Result<T> = std::result::Result<T, ImprovementError>;

/// What detailed improvement did to the half-perimeter wirelength of the
/// signal nets' pins.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Improvement {
    /// Before, in database units.
    pub hpwl_before: Dbu,
    /// After, in database units.
    pub hpwl_after: Dbu,
}

/// A distance in heights of the first row, the unit
/// `DetailedMgr::setMaxDisplacement` scales both displacement limits by.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RowHeights(pub i64);

impl RowHeights {
    const fn to_dbu(self, row_height: Dbu) -> Dbu {
        Dbu(self.0 * row_height.get())
    }
}

/// How far `improve_placement -max_displacement {x y}` lets a cell move
/// across and up or down; `None` for no limit, which `OpenDP` reads from
/// a zero.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MaxDisplacement {
    /// Across.
    pub x: Option<RowHeights>,
    /// Up or down.
    pub y: Option<RowHeights>,
}

/// Improves a legal placement's wirelength as `improve_placement
/// -max_displacement {x y}` does, keeping it legal.
///
/// # Errors
///
/// When the design cannot be laid on `OpenDP`'s grid, an I/O pin has no
/// shapes, or the improver's state comes apart.
pub fn improve_placement(
    db: &mut NetlistDB,
    max_displacement: MaxDisplacement,
) -> Result<Improvement> {
    let hpwl_before = super::mirror::hpwl(db);
    if hpwl_before == Dbu::ZERO {
        return Ok(Improvement {
            hpwl_before,
            hpwl_after: hpwl_before,
        });
    }
    let mut improver = Improver::new(db, max_displacement)?;
    shift::legalize(&mut improver)?;
    improver.grid.paint_fixed_cells();
    improver.run_script()?;
    improver.grid.write_back(db);
    Ok(Improvement {
        hpwl_before,
        hpwl_after: super::mirror::hpwl(db),
    })
}

pare_common::define_index!(
    /// A signal net of the improver's network, as `Edge`.
    EdgeId(u32)
);

pare_common::define_index!(
    /// A pin of the improver's network, as `Pin`.
    NetworkPinId(u32)
);

pare_common::define_index!(
    /// A stretch of a row the improver places cells in, as `DetailedSeg`.
    SegmentId(u32)
);

/// A stretch of a row between blockages and region edges, as `DetailedSeg`.
#[derive(Clone, Copy, Debug)]
struct Segment {
    row: usize,
    region: Option<GroupId>,
    min_x: Dbu,
    max_x: Dbu,
    /// The width of the cells in it.
    util: Dbu,
}

impl Segment {
    fn width(&self) -> Dbu {
        self.max_x - self.min_x
    }
}

/// A fixed cell's or blockage's span of a row, as `Blockage`.
#[derive(Clone, Copy, Debug)]
struct Blockage {
    min_x: Dbu,
    max_x: Dbu,
}

/// One move a trial made, as `MoveCellAction`.
#[derive(Clone, Debug)]
struct Move {
    node: NodeId,
    from: Point<Dbu>,
    from_segs: Vec<SegmentId>,
    to_segs: Vec<SegmentId>,
}

/// The moves of the trial under way, as `Journal`.
#[derive(Debug, Default)]
struct Journal {
    moves: Vec<Move>,
    nodes: BTreeSet<NodeId>,
    edges: BTreeSet<EdgeId>,
}

impl Journal {
    fn clear(&mut self) {
        self.moves.clear();
        self.nodes.clear();
        self.edges.clear();
    }
}

/// The improver's state, as `DetailedMgr` with the network, architecture
/// and grid it works on.
struct Improver {
    grid: Placer,
    net: Network,
    arch: Arch,
    segments: IdVec<SegmentId, Segment>,
    /// The segments of each row.
    segs_in_row: Vec<Vec<SegmentId>>,
    /// The cells of each segment by center, as `cellsInSeg_`.
    cells_in_seg: IdVec<SegmentId, Vec<NodeId>>,
    /// The segments each cell lies in, as `reverseCellToSegs_`.
    cell_segs: IdVec<NodeId, Vec<SegmentId>>,
    /// The blockages of each row.
    blockages: Vec<Vec<Blockage>>,
    single_height: Vec<NodeId>,
    /// Multi-row cells by the rows they span.
    multi_height: Vec<Vec<NodeId>>,
    fixed_cells: Vec<NodeId>,
    journal: Journal,
    rng: Mt19937,
    /// How far a cell may move across and up or down from where it began.
    max_displacement: Point<Dbu>,
    move_limit: usize,
    disallow_one_site_gaps: bool,
}

impl Improver {
    fn new(db: &NetlistDB, max_displacement: MaxDisplacement) -> Result<Self> {
        let mut grid = Placer::new(db)?;
        for node in &mut grid.nodes {
            node.orient = Orientation::N;
            node.width = node.master_size.x;
            node.height = node.master_size.y;
        }
        let mut net = Network::new(db, &grid)?;
        let arch = Arch::new(db, &grid, &mut net);
        let node_count = net.node_count();
        let limit = Dbu(arch.width().max(arch.height()).get() << 1);
        let row_height = arch.rows[0].height;
        let displacement = |rows: Option<RowHeights>| {
            rows.map_or(limit, |rows| rows.to_dbu(row_height).min(limit))
        };
        let disallow_one_site_gaps = grid.disallow_one_site_gaps;
        Ok(Self {
            grid,
            net,
            max_displacement: Point::new(
                displacement(max_displacement.x),
                displacement(max_displacement.y),
            ),
            arch,
            segments: IdVec::new(),
            segs_in_row: Vec::new(),
            cells_in_seg: IdVec::new(),
            cell_segs: IdVec::filled(Vec::new(), node_count),
            blockages: Vec::new(),
            single_height: Vec::new(),
            multi_height: Vec::new(),
            fixed_cells: Vec::new(),
            journal: Journal::default(),
            rng: Mt19937::new(RANDOM_SEED),
            move_limit: MOVE_LIMIT,
            disallow_one_site_gaps,
        })
    }

    /// The passes `Opendp::improvePlacement` scripts, then the flips
    /// `Detailed::improve` ends with.
    fn run_script(&mut self) -> Result<()> {
        mis::run(self, 10, 0.005)?;
        swap::run_global(self, 10, 0.005)?;
        swap::run_vertical(self, 10, 0.005)?;
        reorder::run(self, 10, 0.005);
        random::run(self, 5, 0.005, 20.0)?;
        orient::run(self);
        if self.disallow_one_site_gaps {
            let limit = self.move_limit;
            self.move_limit = 10_000;
            let unfixed = self.close_one_site_gaps()?;
            self.move_limit = limit;
            if unfixed > 0 {
                log::warn!("{unfixed} one-site gaps remain after detailed improvement");
            }
        }
        Ok(())
    }

    fn left(&self, n: NodeId) -> Dbu {
        self.grid
            .nodes
            .get(n)
            .map_or_else(|| self.net.fixed_node(n).left, |c| c.left)
    }

    fn bottom(&self, n: NodeId) -> Dbu {
        self.grid
            .nodes
            .get(n)
            .map_or_else(|| self.net.fixed_node(n).bottom, |c| c.bottom)
    }

    fn width(&self, n: NodeId) -> Dbu {
        self.grid
            .nodes
            .get(n)
            .map_or_else(|| self.net.fixed_node(n).width, |c| c.width)
    }

    fn height(&self, n: NodeId) -> Dbu {
        self.grid
            .nodes
            .get(n)
            .map_or_else(|| self.net.fixed_node(n).height, |c| c.height)
    }

    fn right(&self, n: NodeId) -> Dbu {
        self.left(n) + self.width(n)
    }

    fn top(&self, n: NodeId) -> Dbu {
        self.bottom(n) + self.height(n)
    }

    fn center_x(&self, n: NodeId) -> Dbu {
        self.left(n) + self.width(n) / 2
    }

    fn center_y(&self, n: NodeId) -> Dbu {
        self.bottom(n) + self.height(n) / 2
    }

    /// Where it stands, its lower left corner.
    fn position(&self, n: NodeId) -> Point<Dbu> {
        Point::new(self.left(n), self.bottom(n))
    }

    fn orig(&self, n: NodeId) -> Point<Dbu> {
        self.net
            .orig
            .get(n)
            .copied()
            .unwrap_or_else(|| self.position(n))
    }

    fn is_fixed(&self, n: NodeId) -> bool {
        self.grid.nodes.get(n).is_none_or(|c| c.fixed)
    }

    fn is_terminal(&self, n: NodeId) -> bool {
        !self.net.is_cell(n) && self.net.fixed_node(n).terminal
    }

    fn group(&self, n: NodeId) -> Option<GroupId> {
        self.grid.nodes.get(n).and_then(|c| c.group)
    }

    fn set_position(&mut self, n: NodeId, at: Point<Dbu>) {
        let node = &mut self.grid.nodes[n];
        node.left = at.x;
        node.bottom = at.y;
    }

    fn rows_spanned(&self, n: NodeId) -> usize {
        self.arch.rows_spanned(self.height(n))
    }

    /// Where a pin is, from its cell's center.
    fn pin_position(&self, pin: NetworkPinId) -> Point<Dbu> {
        let p = &self.net.pins[pin];
        Point::new(
            self.center_x(p.node) + p.offset.x,
            self.center_y(p.node) + p.offset.y,
        )
    }

    /// The half-perimeter wirelength of an edge's pins, as `Edge::hpwl`.
    fn edge_hpwl(&self, edge: EdgeId) -> Dbu {
        let mut bounds: Option<Rect<Dbu>> = None;
        for &pin in &self.net.edges[edge] {
            let p = self.pin_position(pin);
            let at = Rect::new(p, p);
            bounds = Some(bounds.map_or(at, |b| b.union(&at)));
        }
        bounds.map_or(Dbu::ZERO, |b| b.width() + b.height())
    }

    /// The wirelength of every edge of two or more pins, as `Utility::hpwl`.
    fn total_hpwl(&self) -> Dbu {
        self.net
            .edges
            .iter_enumerated()
            .filter(|(_, pins)| pins.len() > 1)
            .map(|(e, _)| self.edge_hpwl(e))
            .sum()
    }

    /// A draw below `limit`, as `DetailedMgr::getRandom`.
    fn random(&mut self, limit: usize) -> usize {
        (self.rng.next_u32() % limit as u32) as usize
    }

    /// `Utility::random_shuffle` with the improver's generator.
    fn shuffle(&mut self, nodes: &mut [NodeId]) {
        for i in 1..nodes.len() {
            let r = self.rng.next_u32() as usize % (i + 1);
            nodes.swap(i, r);
        }
    }

    /// Takes a cell off the grid, as `DetailedMgr::eraseFromGrid`.
    fn erase(&mut self, n: NodeId) {
        self.grid.erase_pixels(n);
    }

    /// Puts a cell on the grid where it stands in its row's orientation, as
    /// `DetailedMgr::paintInGrid`.
    fn paint(&mut self, n: NodeId) {
        let x = self.grid.grid_x(self.left(n));
        let y = self.grid.snap_down_y(self.bottom(n));
        self.grid.paint_pixels(n, x, y);
        if let Some(orient) = self.grid.site_orientation(x, y, self.grid.nodes[n].site) {
            self.orient_node(n, orient);
        }
    }

    /// Whether the cell breaks a placement rule where it stands, as
    /// `DetailedMgr::hasPlacementViolation`.
    fn has_placement_violation(&self, n: NodeId) -> bool {
        let node = &self.grid.nodes[n];
        let x = self.grid.grid_x(node.left);
        let y = self.grid.round_y(node.bottom);
        !(self.grid.check_edge_spacing(node, x, y, node.orient)
            && self.grid.check_blocked_layers(node, x, y)
            && self.grid.check_one_site_gap(node, x, y))
    }

    /// Turns a cell to a new orientation about its lower-left corner,
    /// carrying its pins along, as `Node::adjustCurrOrient`.
    fn orient_node(&mut self, n: NodeId, to: Orientation) {
        use Orientation::{E, FE, FN, FS, FW, N, S, W};
        let mut from = self.grid.nodes[n].orient;
        if from == to {
            return;
        }
        let rotated = |o: Orientation| matches!(o, W | FW | E | FE);
        if rotated(from) != rotated(to) {
            let back = rotated(from);
            for &pin in &self.net.node_pins[n] {
                let Point { x: dx, y: dy } = self.net.pins[pin].offset;
                self.net.pins[pin].offset = if back {
                    Point::new(-dy, dx)
                } else {
                    Point::new(dy, -dx)
                };
            }
            let node = &mut self.grid.nodes[n];
            std::mem::swap(&mut node.width, &mut node.height);
            from = match from {
                W => N,
                FW => FS,
                FE => FN,
                E => S,
                N => W,
                FS => FW,
                FN => FE,
                S => E,
            };
        }
        let (flip_x, flip_y) = if rotated(from) {
            (
                matches!(from, W | FE) != matches!(to, W | FE),
                matches!(from, W | FW) != matches!(to, W | FW),
            )
        } else {
            (
                matches!(from, N | FS) != matches!(to, N | FS),
                matches!(from, N | FN) != matches!(to, N | FN),
            )
        };
        for &pin in &self.net.node_pins[n] {
            let offset = &mut self.net.pins[pin].offset;
            if flip_x {
                offset.x = -offset.x;
            }
            if flip_y {
                offset.y = -offset.y;
            }
        }
        self.grid.nodes[n].orient = to;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::legalize::opendp::detailed_placement;
    use pare_common::db::design::PlacementStatus;
    use pare_common::db::parser::{def, lef};
    use std::path::{Path, PathBuf};

    fn fixture(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../common/tests/fixtures")
            .join(name)
    }

    /// The tiny design's six movable cells legalized from one spot, laid
    /// out for improvement as `improve_placement` lays them before its
    /// passes.
    fn improver() -> Improver {
        let units = def::read_units(&fixture("tiny.def")).unwrap();
        let library = lef::parse_files(&[fixture("tiny.lef")], units).unwrap();
        let mut db = def::parse(&fixture("tiny.def"), library).unwrap();
        for name in ["u1", "u2", "u4", "u5", "u6", "u7"] {
            let id = db.cell_id(name).unwrap();
            let cell = &mut db.cells[id];
            cell.location = Point::new(Dbu(1000), Dbu(1000));
            cell.status = PlacementStatus::Placed;
        }
        let _ = detailed_placement(&mut db).unwrap();
        let max_displacement = MaxDisplacement {
            x: Some(RowHeights(5)),
            y: Some(RowHeights(1)),
        };
        let mut improver = Improver::new(&db, max_displacement).unwrap();
        shift::legalize(&mut improver).unwrap();
        improver.grid.paint_fixed_cells();
        improver
    }

    /// Where every movable cell stands and faces, and the cells of each
    /// segment.
    #[derive(Debug, PartialEq, Eq)]
    struct Layout {
        cells: Vec<(Dbu, Dbu, Orientation)>,
        segments: Vec<Vec<NodeId>>,
    }

    fn layout(imp: &Improver) -> Layout {
        Layout {
            cells: imp
                .movable_cells()
                .into_iter()
                .map(|n| (imp.left(n), imp.bottom(n), imp.grid.nodes[n].orient))
                .collect(),
            segments: imp.cells_in_seg.iter().cloned().collect(),
        }
    }

    /// The first move of a movable cell to a site of any segment that
    /// lands legally, left journaled.
    fn first_legal_move(imp: &mut Improver) -> bool {
        let spacing = imp.arch.rows[0].site_spacing;
        for n in imp.movable_cells() {
            let si = imp.cell_segs[n][0];
            for sj in imp.segments.ids() {
                let segment = imp.segments[sj];
                let mut x = segment.min_x;
                while x < segment.max_x {
                    if x != imp.left(n) && imp.try_move(n, si, x, sj).unwrap() {
                        return true;
                    }
                    x += spacing;
                }
            }
        }
        false
    }

    #[test]
    fn a_legalized_placement_has_no_placement_problems() {
        let imp = improver();

        assert_eq!(imp.placement_problems().unwrap(), 0);
    }

    #[test]
    fn a_rejected_move_puts_every_cell_back() {
        let mut imp = improver();
        let before = layout(&imp);
        let hpwl = imp.total_hpwl();
        assert!(first_legal_move(&mut imp));
        assert_ne!(layout(&imp), before);

        imp.reject_move().unwrap();

        assert_eq!(layout(&imp), before);
        assert_eq!(imp.total_hpwl(), hpwl);
        assert_eq!(imp.placement_problems().unwrap(), 0);
    }

    #[test]
    fn an_accepted_move_stays_and_stays_legal() {
        let mut imp = improver();
        let before = layout(&imp);
        assert!(first_legal_move(&mut imp));

        imp.accept_move();
        imp.reject_move().unwrap();

        assert_ne!(layout(&imp), before);
        assert_eq!(imp.placement_problems().unwrap(), 0);
    }

    #[test]
    fn a_cell_aligns_down_to_the_site_grid_within_its_span() {
        let imp = improver();
        let n = imp.movable_cells()[0];
        let row = imp.arch.rows[0];
        let site = |i: i64| row.left + row.site_spacing * i;

        let aligned = imp.align(n, site(3) + Dbu(1), site(0), site(40));

        assert_eq!(aligned, Some(site(3)));
    }

    #[test]
    fn a_cell_aligned_past_its_span_stops_at_the_last_site_it_fits() {
        let imp = improver();
        let n = imp.movable_cells()[0];
        let row = imp.arch.rows[0];
        let site = |i: i64| row.left + row.site_spacing * i;
        let right = site(40);

        let aligned = imp.align(n, right, site(0), right).unwrap();

        assert_eq!(aligned + imp.width(n), right);
    }

    #[test]
    fn a_cell_wider_than_its_span_has_no_alignment() {
        let imp = improver();
        let n = imp.movable_cells()[0];
        let left = imp.arch.rows[0].left;

        let aligned = imp.align(n, left, left, left + imp.width(n) - Dbu(1));

        assert_eq!(aligned, None);
    }
}
