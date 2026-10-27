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
use super::{DetailedPlacementError, Placer};
use arch::Arch;
use network::Network;
use pare_common::db::design::NetlistDB;
use pare_common::geom::orient::Orientation;
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
    pub hpwl_before: i64,
    /// After, in database units.
    pub hpwl_after: i64,
}

/// Improves a legal placement's wirelength as `improve_placement
/// -max_displacement {x y}` does, keeping it legal; the displacement limits
/// are in rows, zero for no limit.
///
/// # Errors
///
/// When the design cannot be laid on `OpenDP`'s grid, an I/O pin has no
/// shapes, or the improver's state comes apart.
pub fn improve_placement(db: &mut NetlistDB, max_displacement: (i64, i64)) -> Result<Improvement> {
    let hpwl_before = super::mirror::hpwl(db);
    if hpwl_before == 0 {
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

/// A stretch of a row between blockages and region edges, as `DetailedSeg`.
#[derive(Clone, Copy, Debug)]
struct Segment {
    row: usize,
    region: Option<usize>,
    min_x: i64,
    max_x: i64,
    /// The width of the cells in it.
    util: i64,
}

impl Segment {
    const fn width(&self) -> i64 {
        self.max_x - self.min_x
    }
}

/// A fixed cell's or blockage's span of a row, as `Blockage`.
#[derive(Clone, Copy, Debug)]
struct Blockage {
    min_x: i64,
    max_x: i64,
}

/// One move a trial made, as `MoveCellAction`.
#[derive(Clone, Debug)]
struct Move {
    node: usize,
    from: (i64, i64),
    from_segs: Vec<usize>,
    to_segs: Vec<usize>,
}

/// The moves of the trial under way, as `Journal`.
#[derive(Debug, Default)]
struct Journal {
    moves: Vec<Move>,
    nodes: BTreeSet<usize>,
    edges: BTreeSet<usize>,
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
    segments: Vec<Segment>,
    segs_in_row: Vec<Vec<usize>>,
    /// The cells of each segment by center, as `cellsInSeg_`.
    cells_in_seg: Vec<Vec<usize>>,
    /// The segments each cell lies in, as `reverseCellToSegs_`.
    cell_segs: Vec<Vec<usize>>,
    blockages: Vec<Vec<Blockage>>,
    single_height: Vec<usize>,
    /// Multi-row cells by the rows they span.
    multi_height: Vec<Vec<usize>>,
    fixed_cells: Vec<usize>,
    journal: Journal,
    rng: Mt19937,
    max_displacement: (i64, i64),
    move_limit: usize,
    disallow_one_site_gaps: bool,
}

impl Improver {
    fn new(db: &NetlistDB, max_displacement: (i64, i64)) -> Result<Self> {
        let mut grid = Placer::new(db)?;
        for node in &mut grid.nodes {
            node.orient = Orientation::N;
            node.width = node.master_size.0;
            node.height = node.master_size.1;
        }
        let mut net = Network::new(db, &grid)?;
        let arch = Arch::new(db, &grid, &mut net);
        let node_count = net.node_count();
        let limit = arch.width().max(arch.height()) << 1;
        let row_height = arch.rows[0].height;
        let displacement = |rows: i64| {
            if rows == 0 {
                limit
            } else {
                (rows * row_height).min(limit)
            }
        };
        let disallow_one_site_gaps = grid.disallow_one_site_gaps;
        Ok(Self {
            grid,
            net,
            max_displacement: (
                displacement(max_displacement.0),
                displacement(max_displacement.1),
            ),
            arch,
            segments: Vec::new(),
            segs_in_row: Vec::new(),
            cells_in_seg: Vec::new(),
            cell_segs: vec![Vec::new(); node_count],
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
            let unfixed = self.one_site_gap_violations(true)?;
            self.move_limit = limit;
            if unfixed > 0 {
                log::warn!("{unfixed} one-site gaps remain after detailed improvement");
            }
        }
        Ok(())
    }

    const fn cell_count(&self) -> usize {
        self.net.cells
    }

    fn left(&self, n: usize) -> i64 {
        self.grid
            .nodes
            .get(n)
            .map_or_else(|| self.net.fixed[n - self.net.cells].left, |c| c.left)
    }

    fn bottom(&self, n: usize) -> i64 {
        self.grid
            .nodes
            .get(n)
            .map_or_else(|| self.net.fixed[n - self.net.cells].bottom, |c| c.bottom)
    }

    fn width(&self, n: usize) -> i64 {
        self.grid
            .nodes
            .get(n)
            .map_or_else(|| self.net.fixed[n - self.net.cells].width, |c| c.width)
    }

    fn height(&self, n: usize) -> i64 {
        self.grid
            .nodes
            .get(n)
            .map_or_else(|| self.net.fixed[n - self.net.cells].height, |c| c.height)
    }

    fn right(&self, n: usize) -> i64 {
        self.left(n) + self.width(n)
    }

    fn top(&self, n: usize) -> i64 {
        self.bottom(n) + self.height(n)
    }

    fn center_x(&self, n: usize) -> i64 {
        self.left(n) + self.width(n) / 2
    }

    fn center_y(&self, n: usize) -> i64 {
        self.bottom(n) + self.height(n) / 2
    }

    fn orig(&self, n: usize) -> (i64, i64) {
        self.net
            .orig
            .get(n)
            .copied()
            .unwrap_or_else(|| (self.left(n), self.bottom(n)))
    }

    fn is_fixed(&self, n: usize) -> bool {
        self.grid.nodes.get(n).is_none_or(|c| c.fixed)
    }

    fn is_terminal(&self, n: usize) -> bool {
        n >= self.net.cells && self.net.fixed[n - self.net.cells].terminal
    }

    fn group(&self, n: usize) -> Option<usize> {
        self.grid.nodes.get(n).and_then(|c| c.group)
    }

    fn set_position(&mut self, n: usize, left: i64, bottom: i64) {
        let node = &mut self.grid.nodes[n];
        node.left = left;
        node.bottom = bottom;
    }

    fn rows_spanned(&self, n: usize) -> usize {
        self.arch.rows_spanned(self.height(n))
    }

    /// Where a pin is, from its cell's center.
    fn pin_position(&self, pin: usize) -> (i64, i64) {
        let p = &self.net.pins[pin];
        (
            self.center_x(p.node) + p.offset.0,
            self.center_y(p.node) + p.offset.1,
        )
    }

    /// The half-perimeter wirelength of an edge's pins, as `Edge::hpwl`.
    fn edge_hpwl(&self, edge: usize) -> i64 {
        let mut bounds: Option<(i64, i64, i64, i64)> = None;
        for &pin in &self.net.edges[edge] {
            let (x, y) = self.pin_position(pin);
            bounds = Some(bounds.map_or((x, y, x, y), |(x0, y0, x1, y1)| {
                (x0.min(x), y0.min(y), x1.max(x), y1.max(y))
            }));
        }
        bounds.map_or(0, |(x0, y0, x1, y1)| (x1 - x0) + (y1 - y0))
    }

    /// The wirelength of every edge of two or more pins, as `Utility::hpwl`.
    fn total_hpwl(&self) -> i64 {
        (0..self.net.edges.len())
            .filter(|&e| self.net.edges[e].len() > 1)
            .map(|e| self.edge_hpwl(e))
            .sum()
    }

    /// A draw below `limit`, as `DetailedMgr::getRandom`.
    fn random(&mut self, limit: usize) -> usize {
        let limit = u32::try_from(limit).unwrap_or(u32::MAX);
        usize::try_from(self.rng.next_u32() % limit).unwrap_or(0)
    }

    /// `Utility::random_shuffle` with the improver's generator.
    fn shuffle(&mut self, nodes: &mut [usize]) {
        for i in 1..nodes.len() {
            let r = usize::try_from(self.rng.next_u32()).unwrap_or(0) % (i + 1);
            nodes.swap(i, r);
        }
    }

    /// Takes a cell off the grid, as `DetailedMgr::eraseFromGrid`.
    fn erase(&mut self, n: usize) {
        self.grid.erase_pixels(n);
    }

    /// Puts a cell on the grid where it stands in its row's orientation, as
    /// `DetailedMgr::paintInGrid`.
    fn paint(&mut self, n: usize) {
        let x = self.grid.grid_x(self.left(n));
        let y = self.grid.snap_down_y(self.bottom(n));
        self.grid.paint_pixels(n, x, y);
        if let Some(orient) = self.grid.site_orientation(x, y, self.grid.nodes[n].site) {
            self.orient_node(n, orient);
        }
    }

    /// Whether the cell breaks a placement rule where it stands, as
    /// `DetailedMgr::hasPlacementViolation`.
    fn has_placement_violation(&self, n: usize) -> bool {
        let node = &self.grid.nodes[n];
        let x = self.grid.grid_x(node.left);
        let y = self.grid.round_y(node.bottom);
        !(self.grid.check_edge_spacing(node, x, y, node.orient)
            && self.grid.check_blocked_layers(node, x, y)
            && self.grid.check_one_site_gap(node, x, y))
    }

    /// Turns a cell to a new orientation about its lower-left corner,
    /// carrying its pins along, as `Node::adjustCurrOrient`.
    fn orient_node(&mut self, n: usize, to: Orientation) {
        use Orientation::{E, FE, FN, FS, FW, N, S, W};
        let mut from = self.grid.nodes[n].orient;
        if from == to {
            return;
        }
        let rotated = |o: Orientation| matches!(o, W | FW | E | FE);
        if rotated(from) != rotated(to) {
            let back = rotated(from);
            for &pin in &self.net.node_pins[n] {
                let (dx, dy) = self.net.pins[pin].offset;
                self.net.pins[pin].offset = if back { (-dy, dx) } else { (dy, -dx) };
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
                offset.0 = -offset.0;
            }
            if flip_y {
                offset.1 = -offset.1;
            }
        }
        self.grid.nodes[n].orient = to;
    }
}
