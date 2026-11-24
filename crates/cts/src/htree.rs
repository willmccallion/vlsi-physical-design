//! The H-tree of one clock net, as `TritonCTS`'s `HTreeBuilder` over its
//! `TreeBuilder`: the sinks are clustered under leaf buffers, the region
//! they span is halved level by level with a branching point at the middle
//! of each half, the branching points are drawn toward their sinks by
//! capacitated k-means and kept clear of blockages, and each branch is a
//! chain of characterized wire segments.
//!
//! Positions are kept in the tree's length unit, as `TritonCTS` keeps them,
//! with its mix of single and double precision so that the same placement
//! gives the same tree.

// The arithmetic keeps TritonCTS's operations, whose rounding fusing would change.
#![allow(clippy::suboptimal_flops)]

use crate::capacitance::Farads32;
use crate::clock::{Clock, InstRef};
use crate::kmeans::{Clustering, SinkPoint};
use crate::options::{
    CLOCK_TREE_MAX_DEPTH, CLUSTERING_CAPACITY, CLUSTERING_POWER,
    FANOUT_LIMITED_CLUSTERING_CAPACITY, MACRO_SINK_CLUSTERING_SIZE, MAX_DIAMETER, MAX_LEAF_SINKS,
    MAX_SLEW, Microns, SINK_CLUSTERING_SIZE, Settings, SinkClusteringMode,
};
use crate::sink_clustering::SinkClustering;
use crate::techchar::{
    CapSteps, LengthSteps, LengthUnit, SegmentLength, SlewSteps, TechChar, WireSegment,
};
use pare_common::convert::{NearestFrom, SaturatingFrom};
use pare_common::db::liberty::BufferCell;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::point::Point;
use pare_common::geom::rect::Rect;
use pare_common::units::Seconds;
use rustc_hash::{FxHashMap, FxHashSet};

/// The fewest register sinks a tree clusters under leaf buffers, as
/// `HTreeBuilder`'s `min_clustering_sinks_`.
const MIN_CLUSTERING_SINKS: usize = 200;
/// The fewest macro sinks a tree clusters, as `HTreeBuilder`'s
/// `min_clustering_macro_sinks_`.
const MIN_CLUSTERING_MACRO_SINKS: usize = 10;
/// The cluster diameters tried, as `CtsOptions`' default
/// `sinkClusteringDiameters`.
const CLUSTER_DIAMETERS: [Microns; 3] =
    [Microns::new(50.0), Microns::new(100.0), Microns::new(200.0)];
/// How far a segment search widens its slew and load match before giving
/// up, as `HTreeBuilder::computeMinDelaySegment` stops.
const MAX_TOLERANCE: u32 = 10;
/// Rounds of capacitated k-means splitting a branch's sinks, as
/// `HTreeBuilder::refineBranchingPointsWithClustering` runs it.
const KMEANS_ROUNDS: u32 = 5;

/// A position in the tree's length unit, a whole number of database units
/// that the characterization chose.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct TreePoint {
    pub x: f64,
    pub y: f64,
}

impl TreePoint {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    fn from_f32(p: SinkPoint) -> Self {
        Self::new(f64::from(p.x), f64::from(p.y))
    }

    /// The point in database units, `unit` of them to the tree's length
    /// unit, truncated to `int` as `TritonCTS` stores positions.
    pub(crate) fn to_dbu(self, unit: f64) -> Point<Dbu> {
        let truncate = |v: f64| Dbu::new(i64::from(i32::saturating_from(v * unit)));
        Point::new(truncate(self.x), truncate(self.y))
    }

    pub fn dist(self, other: Self) -> f64 {
        (self.x - other.x).abs() + (self.y - other.y).abs()
    }

    /// `TritonCTS`'s point equality, to [`FUZZY_EPSILON`].
    pub fn same(self, other: Self) -> bool {
        fuzzy_eq(self.x, other.x, FUZZY_EPSILON) && fuzzy_eq(self.y, other.y, FUZZY_EPSILON)
    }

    /// The exact key points are hashed by.
    // NOTE: TreeBuilder hashes points exactly but matches them by fuzzy
    // equality, so near points sharing a bucket merge; PARE keys exactly.
    fn key(self) -> (u64, u64) {
        let bits = |v: f64| if v == 0.0 { 0 } else { v.to_bits() };
        (bits(self.x), bits(self.y))
    }

    /// The key of the point rounded to single precision, as sink positions
    /// are stored.
    fn key_f32(self) -> (u64, u64) {
        Self::from_f32(self.to_sink_point()).key()
    }

    /// The point rounded to the single precision sinks are kept in.
    fn to_sink_point(self) -> SinkPoint {
        SinkPoint::new(f32::nearest_from(self.x), f32::nearest_from(self.y))
    }
}

/// A position in database units in the tree's length unit, `unit`
/// database units long, in single precision as `TritonCTS` takes sinks.
fn in_segment_units(p: Point<Dbu>, unit: f32) -> SinkPoint {
    SinkPoint::new(p.x.get() as f32 / unit, p.y.get() as f32 / unit)
}

/// `TritonCTS`'s `fuzzyEqual`.
fn fuzzy_eq(a: f64, b: f64, epsilon: f64) -> bool {
    (a - b).abs() < epsilon
}

/// The tolerance `TritonCTS` compares positions and lengths to.
const FUZZY_EPSILON: f64 = 1e-6;

fn fuzzy_ge(a: f64, b: f64) -> bool {
    fuzzy_eq(a, b, FUZZY_EPSILON) || a > b
}

fn fuzzy_le(a: f64, b: f64) -> bool {
    fuzzy_eq(a, b, FUZZY_EPSILON) || a < b
}

/// A blockage in the tree's length unit, by its lower left and upper right
/// corners.
#[derive(Clone, Copy, Debug)]
struct TreeBox {
    low: TreePoint,
    high: TreePoint,
}

impl TreeBox {
    /// Whether `p` lies on the box's boundary, as `TreeBuilder::isAlongBbox`.
    fn has_on_edge(self, p: TreePoint) -> bool {
        let Self {
            low: TreePoint { x: x1, y: y1 },
            high: TreePoint { x: x2, y: y2 },
        } = self;
        let on_vertical = (fuzzy_eq(p.x, x1, FUZZY_EPSILON) || fuzzy_eq(p.x, x2, FUZZY_EPSILON))
            && fuzzy_ge(p.y, y1)
            && fuzzy_le(p.y, y2);
        let on_horizontal = (fuzzy_eq(p.y, y1, FUZZY_EPSILON) || fuzzy_eq(p.y, y2, FUZZY_EPSILON))
            && fuzzy_ge(p.x, x1)
            && fuzzy_le(p.x, x2);
        on_vertical || on_horizontal
    }
}

/// A region with a center that, once read, stays put unless set, as
/// `TritonCTS`'s `Box`.
#[derive(Clone, Copy, Debug, Default)]
struct Region {
    x_min: f64,
    y_min: f64,
    width: f64,
    height: f64,
    center: Option<TreePoint>,
}

impl Region {
    fn new(x_min: f64, y_min: f64, x_max: f64, y_max: f64) -> Self {
        Self {
            x_min,
            y_min,
            width: x_max - x_min,
            height: y_max - y_min,
            center: None,
        }
    }

    fn center(&mut self) -> TreePoint {
        *self.center.get_or_insert_with(|| {
            TreePoint::new(
                self.x_min + self.width / 2.0,
                self.y_min + self.height / 2.0,
            )
        })
    }
}

/// The compass direction a branch leaves its parent in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Direction {
    North,
    East,
    South,
    West,
}

/// Which sinks a tree drives.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum TreeType {
    /// Both macros and registers.
    #[default]
    Regular,
    /// Only macros and other sinks with insertion delay.
    Macro,
    /// Only registers, under a macro tree.
    Register,
}

/// One level of the H-tree: the wire segments of its branches, its
/// branching points and their sinks.
///
/// Points `2 p` and `2 p + 1` are the two branches of point `p` of the
/// level above, or of the tree's root for the first level.
#[derive(Clone, Debug)]
struct LevelTopology {
    length: f64,
    wire_segments: Vec<usize>,
    points: Vec<TreePoint>,
    sinks: Vec<BranchSinks>,
    /// Where the level's last segment leaves the slew, the load and the
    /// wire since the last buffer, for the level below to start from.
    end: SegmentChoice,
}

impl LevelTopology {
    const fn new(length: f64, wire_segments: Vec<usize>, end: SegmentChoice) -> Self {
        Self {
            length,
            wire_segments,
            points: Vec::new(),
            sinks: Vec::new(),
            end,
        }
    }

    /// Adds the two branching points of the next parent in order, as
    /// `TritonCTS` adds both points of each parent in turn; returns their
    /// indices.
    fn add_branches(&mut self, low: TreePoint, high: TreePoint) -> (usize, usize) {
        let low_idx = self.points.len();
        for point in [low, high] {
            self.points.push(point);
            self.sinks.push(BranchSinks::default());
        }
        (low_idx, low_idx + 1)
    }

    /// The parent of point `idx`: its index in the level above, or zero for
    /// the tree's root above the first level.
    const fn parent_of(idx: usize) -> usize {
        idx / 2
    }

    /// The other branch of point `idx`'s parent.
    const fn sibling_of(idx: usize) -> usize {
        idx ^ 1
    }

    /// The two branches of parent `p`.
    const fn branches_of(p: usize) -> [usize; 2] {
        [2 * p, 2 * p + 1]
    }
}

/// The sinks and leaf buffers a branch reaches: where each is, in the
/// single precision `TritonCTS` keeps them in, and which it is.
#[derive(Clone, Debug, Default)]
struct BranchSinks {
    points: Vec<SinkPoint>,
    insts: Vec<InstRef>,
}

impl BranchSinks {
    fn push(&mut self, point: SinkPoint, inst: InstRef) {
        self.points.push(point);
        self.insts.push(inst);
    }

    const fn len(&self) -> usize {
        self.points.len()
    }

    const fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Where each is, as tree points.
    fn tree_points(&self) -> Vec<TreePoint> {
        self.points
            .iter()
            .map(|&p| TreePoint::from_f32(p))
            .collect()
    }
}

/// What every tree builder of a design shares.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TreeContext<'a> {
    pub settings: &'a Settings,
    pub core: Rect<Dbu>,
    pub dbu: i32,
    pub root_buffer: BufferCell,
    pub sink_buffer: BufferCell,
    pub buffer_sizes: &'a FxHashMap<BufferCell, Point<Dbu>>,
    pub blockages: &'a [Rect<Dbu>],
}

/// The tree of one clock net.
#[derive(Clone, Debug)]
pub(crate) struct Builder {
    pub clock: Clock,
    pub tree_type: TreeType,
    pub parent: Option<usize>,
    pub children: Vec<usize>,
    pub top_buffer_name: String,
    pub driving_net: Option<pare_common::db::indices::NetId>,
    pub top_input_net: Option<pare_common::db::indices::NetId>,
    occupied: FxHashSet<(u64, u64)>,
    insertion_lengths: FxHashMap<(u64, u64), SegmentLength>,
    sink_region: Region,
    topologies: Vec<LevelTopology>,
    /// Where the sinks and leaf buffers are, which no buffer may take.
    sink_locations: FxHashSet<(u64, u64)>,
    top_level_sinks_clustered: BranchSinks,
    unit: LengthUnit,
    min_input_cap: CapSteps,
    min_length_sink_region: LengthSteps,
}

impl Builder {
    pub fn new(clock: Clock, parent: Option<usize>, unit: LengthUnit) -> Self {
        Self {
            clock,
            tree_type: TreeType::Regular,
            parent,
            children: Vec::new(),
            top_buffer_name: String::new(),
            driving_net: None,
            top_input_net: None,
            occupied: FxHashSet::default(),
            insertion_lengths: FxHashMap::default(),
            sink_region: Region::default(),
            topologies: Vec::new(),
            sink_locations: FxHashSet::default(),
            top_level_sinks_clustered: BranchSinks::default(),
            unit,
            min_input_cap: CapSteps::new(0),
            min_length_sink_region: LengthSteps::new(0),
        }
    }

    /// Whether the tree drives no other tree, as `TreeBuilder::isLeafTree`.
    pub(crate) fn is_leaf_tree(&self) -> bool {
        if self.tree_type == TreeType::Macro {
            return self.children.len() == 1;
        }
        self.children.is_empty()
    }

    /// The length of clock wire a sink at `p` stands for beyond its pin.
    fn insertion_length(&self, p: TreePoint) -> SegmentLength {
        self.insertion_lengths
            .get(&p.key())
            .copied()
            .unwrap_or_default()
    }

    /// The distance between two points plus the clock wire the sinks there
    /// stand for, as `TreeBuilder::computeDist`.
    pub(crate) fn compute_dist(&self, a: TreePoint, b: TreePoint) -> f64 {
        a.dist(b) + self.insertion_length(a).get() + self.insertion_length(b).get()
    }

    fn is_occupied(&self, p: TreePoint) -> bool {
        self.occupied.contains(&p.key())
    }

    fn commit(&mut self, p: TreePoint) {
        let _ = self.occupied.insert(p.key());
    }

    fn commit_move(&mut self, from: TreePoint, to: TreePoint) {
        let _ = self.occupied.remove(&from.key());
        let _ = self.occupied.insert(to.key());
    }

    /// The tree's length unit in database units.
    pub fn unit(&self) -> f64 {
        self.unit.to_f64()
    }

    /// The blockage `p` lies strictly inside, scaled to the tree's unit,
    /// as `TreeBuilder::findBlockage`.
    fn find_blockage(&self, ctx: &TreeContext<'_>, p: TreePoint) -> Option<TreeBox> {
        let scale = self.unit();
        let (bx, by) = (p.x * scale, p.y * scale);
        ctx.blockages.iter().find_map(|b| {
            let [x1, y1, x2, y2] = [b.min.x, b.min.y, b.max.x, b.max.y].map(|v| v.get() as f64);
            (bx > x1 && bx < x2 && by > y1 && by < y2).then(|| TreeBox {
                low: TreePoint::new(x1 / scale, y1 / scale),
                high: TreePoint::new(x2 / scale, y2 / scale),
            })
        })
    }

    /// Whether a buffer may stand at `p`: free, in the core and outside
    /// every blockage, as `TreeBuilder::checkLegalityLoc`.
    fn legal_location(&self, ctx: &TreeContext<'_>, p: TreePoint) -> bool {
        if self.is_occupied(p) {
            return false;
        }
        let x = i32::saturating_from(p.x * self.unit());
        let y = i32::saturating_from(p.y * self.unit());
        let core = ctx.core;
        let inside = core.min.x.get() <= i64::from(x)
            && i64::from(x) <= core.max.x.get()
            && core.min.y.get() <= i64::from(y)
            && i64::from(y) <= core.max.y.get();
        inside && self.find_blockage(ctx, p).is_none()
    }

    fn buffer_size(&self, ctx: &TreeContext<'_>, buffer: BufferCell) -> (f64, f64) {
        let size = ctx.buffer_sizes[&buffer];
        (
            size.x.get() as f64 / self.unit(),
            size.y.get() as f64 / self.unit(),
        )
    }

    /// Moves a buffer inside a blockage to the free spot beside it nearest
    /// to where it was, as `TreeBuilder::legalizeOneBuffer`.
    fn legalize_in_blockages(
        &self,
        ctx: &TreeContext<'_>,
        p: TreePoint,
        buffer: BufferCell,
    ) -> TreePoint {
        let Some(TreeBox {
            low: TreePoint { x: x1, y: y1 },
            high: TreePoint { x: x2, y: y2 },
        }) = self.find_blockage(ctx, p)
        else {
            return p;
        };
        let (w, h) = self.buffer_size(ctx, buffer);
        let candidates = [
            TreePoint::new(x1 - w, p.y),
            TreePoint::new(x1 - w, p.y + h),
            TreePoint::new(x1 - w, p.y - h),
            TreePoint::new(x2, p.y),
            TreePoint::new(x2, p.y + h),
            TreePoint::new(x2, p.y - h),
            TreePoint::new(p.x, y1 - h),
            TreePoint::new(p.x + w, y1 - h),
            TreePoint::new(p.x - w, y1 - h),
            TreePoint::new(p.x, y2),
            TreePoint::new(p.x + w, y2),
            TreePoint::new(p.x - w, y2),
        ];
        // NOTE: legalizeOneBuffer marks no best with a (-1e6, -1e6) point; PARE with None.
        let mut best: Option<(TreePoint, f64)> = None;
        for c in candidates {
            if self.is_occupied(c) {
                continue;
            }
            let d = self.compute_dist(c, p);
            if best.is_none_or(|(_, b)| d < b) {
                best = Some((c, d));
            }
        }
        best.map_or(p, |(c, _)| c)
    }

    /// Legalizes a buffer and moves it off any sink or leaf buffer it
    /// lands on, as `HTreeBuilder::legalizeOneBuffer`.
    pub(crate) fn legalize_one_buffer(
        &self,
        ctx: &TreeContext<'_>,
        p: TreePoint,
        buffer: BufferCell,
    ) -> TreePoint {
        let legal = self.legalize_in_blockages(ctx, p, buffer);
        let mut resolved = legal;
        let mut jitter = 0;
        while self.sink_locations.contains(&resolved.key_f32()) {
            jitter += 1;
            resolved.x = legal.x + f64::from(jitter) / self.unit();
        }
        resolved
    }

    /// Builds the tree, as `HTreeBuilder::run`.
    pub fn run(&mut self, ctx: &TreeContext<'_>, tech: &mut TechChar) {
        self.min_input_cap = tech.actual_min_input_cap();
        self.min_length_sink_region = tech.min_segment_length() * 2;
        self.init_sink_region(ctx, tech);
        for level in 1..=CLOCK_TREE_MAX_DEPTH {
            let per_region = self.sinks_per_sub_region(ctx, level);
            let (width, height) = self.sub_region_size(level);
            let min_length = self.min_length_sink_region.get() as f64;
            if width < min_length || height < min_length {
                tech.create_fake_entries(self.min_length_sink_region, LengthSteps::new(1));
                self.min_length_sink_region = LengthSteps::new(1);
            }
            self.level_topology(ctx, tech, level, width, height);
            if per_region < ctx.settings.max_fanout.unwrap_or(MAX_LEAF_SINKS) {
                break;
            }
        }
        if self.topologies.is_empty() {
            self.single_buffer_clock_net(ctx);
            return;
        }
        self.legalize(ctx);
        self.create_clock_subnets(ctx, tech);
    }

    fn is_vertical(&self, level: u32) -> bool {
        (level % 2 == 1) == (self.sink_region.height >= self.sink_region.width)
    }

    fn is_horizontal(&self, level: u32) -> bool {
        !self.is_vertical(level)
    }

    fn sub_region_size(&self, level: u32) -> (f64, f64) {
        let grid_a = f64::from(2_u32.pow(level.div_ceil(2)));
        let grid_b = f64::from(2_u32.pow(level / 2));
        let (gx, gy) = if self.is_vertical(1) {
            (grid_b, grid_a)
        } else {
            (grid_a, grid_b)
        };
        (self.sink_region.width / gx, self.sink_region.height / gy)
    }

    fn min_clustering_sinks(&self) -> usize {
        if self.tree_type == TreeType::Macro {
            MIN_CLUSTERING_MACRO_SINKS
        } else {
            MIN_CLUSTERING_SINKS
        }
    }

    fn sinks_per_sub_region(&self, ctx: &TreeContext<'_>, level: u32) -> u32 {
        let total = if self.clock.sinks.len() > self.min_clustering_sinks()
            && ctx.settings.sink_clustering == SinkClusteringMode::Clustered
        {
            self.top_level_sinks_clustered.len()
        } else {
            self.clock.sinks.len()
        };
        let roots = u32::saturating_from(2_f64.powi(i32::saturating_from(level)));
        u32::saturating_from((total as f64 / f64::from(roots)).ceil())
    }

    /// Clusters the sinks and finds the region the tree spans, as
    /// `HTreeBuilder::initSinkRegion`.
    fn init_sink_region(&mut self, ctx: &TreeContext<'_>, tech: &TechChar) {
        let (diameter, size) = if self.tree_type == TreeType::Macro {
            (
                ctx.settings.macro_max_diameter(),
                MACRO_SINK_CLUSTERING_SIZE,
            )
        } else {
            (MAX_DIAMETER, SINK_CLUSTERING_SIZE)
        };
        let unit = self.unit.to_f32();
        let sinks: Vec<SinkPoint> = self
            .clock
            .sinks
            .iter()
            .map(|s| in_segment_units(s.location, unit))
            .collect();
        let max_diameter = diameter.in_tree_units(ctx.dbu, self.unit());
        self.pre_sink_clustering(ctx, tech, &sinks, max_diameter, size);
        if sinks.len() <= self.min_clustering_sinks()
            || ctx.settings.sink_clustering == SinkClusteringMode::Unclustered
        {
            let region = self.clock.sink_region();
            let f = 1.0 / self.unit();
            self.sink_region = Region::new(
                region.min.x.get() as f64 * f,
                region.min.y.get() as f64 * f,
                region.max.x.get() as f64 * f,
                region.max.y.get() as f64 * f,
            );
        } else {
            let (mut x_min, mut x_max) = (f32::MAX, f32::MIN);
            let (mut y_min, mut y_max) = (f32::MAX, f32::MIN);
            for p in &self.top_level_sinks_clustered.points {
                x_min = x_min.min(p.x);
                x_max = x_max.max(p.x);
                y_min = y_min.min(p.y);
                y_max = y_max.max(p.y);
            }
            self.sink_region = Region::new(
                f64::from(x_min),
                f64::from(y_min),
                f64::from(x_max),
                f64::from(y_max),
            );
        }
    }

    /// Groups the sinks under leaf buffers, as
    /// `HTreeBuilder::preSinkClustering` for the first level.
    fn pre_sink_clustering(
        &mut self,
        ctx: &TreeContext<'_>,
        tech: &TechChar,
        sinks: &[SinkPoint],
        max_diameter: f32,
        size: u32,
    ) {
        self.index_sinks(tech);
        let mut top_level = BranchSinks::default();
        if sinks.len() <= self.min_clustering_sinks()
            || ctx.settings.sink_clustering == SinkClusteringMode::Unclustered
        {
            for (i, &point) in sinks.iter().enumerate() {
                top_level.push(point, InstRef::Sink(i));
            }
            self.top_level_sinks_clustered = top_level;
            return;
        }
        let solution = self.cluster_sinks(ctx, tech, sinks, max_diameter, size);
        for (count, cluster) in solution.iter().enumerate() {
            if cluster.len() == 1 {
                top_level.push(sinks[cluster[0]], InstRef::Sink(cluster[0]));
            }
            if cluster.len() > 1 {
                let (point, buffer) = self.add_leaf_buffer(ctx, sinks, cluster, count);
                top_level.push(point, buffer);
            }
        }
        self.top_level_sinks_clustered = top_level;
    }

    /// Records each sink's location in wire segment units, and where it
    /// has an insertion delay, the length of the tree's own clock wire with
    /// that delay, as `HTreeBuilder::preSinkClustering` does.
    fn index_sinks(&mut self, tech: &TechChar) {
        let unit = self.unit.to_f32();
        for s in &self.clock.sinks {
            let location = TreePoint::from_f32(in_segment_units(s.location, unit));
            let delay = s.insertion_delay;
            let _ = self.sink_locations.insert(location.key());
            if delay == Seconds::ZERO {
                continue;
            }
            // NOTE: diverges from TritonCTS, which takes delay / (r * c) of
            // the signal wire per micron, an area rather than a length, and
            // divides it by the segment unit in database units; PARE takes
            // the length of the tree's own buffered wire with that delay.
            let Some(rate) = tech.wire_delay_rate() else {
                log::warn!(
                    "insertion delay of {} is not used: no characterized buffered \
                     segment keeps its slew",
                    s.name
                );
                continue;
            };
            let _ = self
                .insertion_lengths
                .insert(location.key(), rate.length_of(delay));
        }
    }

    /// The clusters of `sinks` that sink clustering finds, each a list of
    /// sink indices, trying the sizes and diameters the settings give.
    fn cluster_sinks(
        &self,
        ctx: &TreeContext<'_>,
        tech: &TechChar,
        sinks: &[SinkPoint],
        max_diameter: f32,
        size: u32,
    ) -> Vec<Vec<usize>> {
        let unit = self.unit.to_f32();
        let is_macro = self.tree_type == TreeType::Macro;
        let mut matching =
            SinkClustering::new(!is_macro, tech.cap_per_dbu(), tech.sink_buffer_input_cap());
        for (i, &SinkPoint { x, y }) in sinks.iter().enumerate() {
            let cap = self.clock.sinks[i].input_cap;
            let cap = if cap == Farads32::ZERO {
                tech.sink_buffer_input_cap()
            } else {
                cap
            };
            let at = TreePoint::new(f64::from(x), f64::from(y));
            matching.add_point(at.x, at.y, cap, self.insertion_length(at));
        }
        let (diameter_set, size_set) = if is_macro {
            (ctx.settings.macro_max_diameter.is_some(), true)
        } else {
            (false, false)
        };
        let unit_i = i32::saturating_from(self.unit.dbu());
        let dbu = ctx.dbu as f32;
        match (size_set, diameter_set) {
            (true, true) => matching.run(size as usize, max_diameter, unit_i),
            (false, true) => {
                for &s in &ctx.settings.sink_clustering_sizes {
                    matching.run(s as usize, max_diameter, unit_i);
                }
            }
            (true, false) => {
                for d in CLUSTER_DIAMETERS {
                    let diameter = d.in_tree_units_single(dbu, unit);
                    matching.run(size as usize, diameter, unit_i);
                }
            }
            (false, false) => {
                for d in CLUSTER_DIAMETERS {
                    for &s in &ctx.settings.sink_clustering_sizes {
                        let diameter = d.in_tree_units_single(dbu, unit);
                        matching.run(s as usize, diameter, unit_i);
                    }
                }
            }
        }
        matching.solution().to_vec()
    }

    /// Puts leaf buffer `count` at the legalized center of the sinks of
    /// `cluster`, driving them; returns where it went and the buffer.
    fn add_leaf_buffer(
        &mut self,
        ctx: &TreeContext<'_>,
        sinks: &[SinkPoint],
        cluster: &[usize],
        count: usize,
    ) -> (SinkPoint, InstRef) {
        let (mut x_sum, mut y_sum) = (0.0_f32, 0.0_f32);
        for &idx in cluster {
            x_sum += sinks[idx].x;
            y_sum += sinks[idx].y;
        }
        let n = cluster.len() as f32;
        let center = TreePoint::new(f64::from(x_sum / n), f64::from(y_sum / n));
        let legal = self.legalize_one_buffer(ctx, center, ctx.sink_buffer);
        self.commit_move(center, legal);
        let location = legal.to_dbu(self.unit());
        let buffer =
            self.clock
                .add_buffer(&format!("clkbuf_leaf_{count}"), ctx.sink_buffer, location);
        let subnet = self.clock.add_subnet(&format!("clknet_leaf_{count}"));
        self.clock.subnets[subnet].add(buffer);
        // NOTE: preSinkClustering finds each sink by its location
        // (mapLocationToSink_), losing all but one of the sinks on one spot;
        // PARE takes them by index.
        for &idx in cluster {
            self.clock.subnets[subnet].add(InstRef::Sink(idx));
        }
        self.clock.subnets[subnet].leaf = true;
        let point = legal.to_sink_point();
        let _ = self.sink_locations.insert(TreePoint::from_f32(point).key());
        (point, buffer)
    }

    /// Picks the wire segments of a level's branches, as
    /// `HTreeBuilder::computeLevelTopology`.
    fn level_topology(
        &mut self,
        ctx: &TreeContext<'_>,
        tech: &TechChar,
        level: u32,
        width: f64,
        height: f64,
    ) {
        let side = if self.is_vertical(level) {
            height
        } else {
            width
        };
        let segment_length = self.branch_length(side);
        let threshold = wire_length_threshold(ctx, tech);
        let mut wire_segments = Vec::new();
        let end = self.choose_segments(
            tech,
            (segment_length, threshold),
            self.level_start(level),
            &mut wire_segments,
        );
        let mut topology = LevelTopology::new(segment_length.get() as f64, wire_segments, end);
        self.branching_points(ctx, level, &mut topology);
        self.topologies.push(topology);
    }

    /// The length of a branch `side` long, in whole halves of the shortest
    /// sink region, at least one length unit.
    fn branch_length(&self, side: f64) -> LengthSteps {
        let min_length = self.min_length_sink_region;
        let clamped = min_length.max(LengthSteps::new(1)).get() as f64;
        LengthSteps::new(i64::from(
            u32::saturating_from((side / clamped).round() * min_length.get() as f64 / 2.0).max(1),
        ))
    }

    /// Where a level's segments start: from the root for the first level,
    /// else where the level above ended.
    fn level_start(&self, level: u32) -> SegmentChoice {
        if level > 1 {
            return self.topologies[level as usize - 2].end;
        }
        SegmentChoice {
            slew: SlewSteps::FIRST,
            cap: self.min_input_cap,
            curr_wl: LengthSteps::new(0),
        }
    }

    /// Fills `segment_length` with segments, longest characterized length
    /// first, from `start` under the wire length threshold, pushing each
    /// chosen segment's key; returns where the last one leaves off.
    fn choose_segments(
        &self,
        tech: &TechChar,
        (segment_length, threshold): (LengthSteps, LengthSteps),
        start: SegmentChoice,
        keys: &mut Vec<usize>,
    ) -> SegmentChoice {
        let mut state = start;
        let mut length = LengthSteps::new(0);
        let mut char_length = tech.max_segment_length();
        while char_length >= LengthSteps::new(1) {
            let wires = (segment_length - length).count_of(char_length);
            for _ in 0..wires {
                let Some((key, next)) = self.next_segment(tech, char_length, state, threshold)
                else {
                    continue;
                };
                state = next;
                length += char_length;
                keys.push(key);
            }
            if wires >= 1 && length == segment_length {
                break;
            }
            char_length = char_length - LengthSteps::new(1);
        }
        state
    }

    /// The fastest segment `length` long from `state`, and where it leaves
    /// off, its load no less than the least input capacitance.
    fn next_segment(
        &self,
        tech: &TechChar,
        length: LengthSteps,
        state: SegmentChoice,
        wirelength_threshold: LengthSteps,
    ) -> Option<(usize, SegmentChoice)> {
        let mut out = SegmentChoice {
            slew: SlewSteps::new(0),
            cap: CapSteps::new(0),
            curr_wl: state.curr_wl,
        };
        let query = SegmentQuery {
            length,
            input_slew: state.slew,
            input_cap: state.cap,
            slew_threshold: SlewSteps::new(MAX_SLEW),
            wirelength_threshold,
        };
        let key = min_delay_segment(tech, &query, 1, &mut out)?;
        out.cap = out.cap.max(self.min_input_cap);
        Some((key, out))
    }

    /// Places a level's branching points either side of each parent and
    /// splits the parent's sinks between them, as
    /// `HTreeBuilder::computeBranchingPoints`.
    fn branching_points(
        &mut self,
        ctx: &TreeContext<'_>,
        level: u32,
        topology: &mut LevelTopology,
    ) {
        let horizontal = self.is_horizontal(level);
        let spread = |root: TreePoint, length: f64| -> (TreePoint, TreePoint) {
            if horizontal {
                (
                    TreePoint::new(root.x - length, root.y),
                    TreePoint::new(root.x + length, root.y),
                )
            } else {
                (
                    TreePoint::new(root.x, root.y - length),
                    TreePoint::new(root.x, root.y + length),
                )
            }
        };
        if level == 1 {
            let root = self.sink_region.center();
            let (low, high) = spread(root, topology.length);
            let (a, b) = topology.add_branches(low, high);
            let sinks = self.top_level_sinks_clustered.clone();
            refine_with_clustering(ctx, topology, a, b, root, &sinks);
            return;
        }
        let parent = &self.topologies[level as usize - 2];
        let roots: Vec<(TreePoint, BranchSinks)> = parent
            .points
            .iter()
            .zip(&parent.sinks)
            .map(|(&p, sinks)| (p, sinks.clone()))
            .collect();
        for (root, sinks) in roots {
            let (low, high) = spread(root, topology.length);
            let (a, b) = topology.add_branches(low, high);
            refine_with_clustering(ctx, topology, a, b, root, &sinks);
        }
    }

    /// Where the parent of point `idx` of level `level_idx` is, `top` being
    /// the tree's root.
    fn parent_point(&self, level_idx: usize, idx: usize, top: TreePoint) -> TreePoint {
        if level_idx == 0 {
            return top;
        }
        self.topologies[level_idx - 1].points[LevelTopology::parent_of(idx)]
    }

    fn weighted_distance(&self, new: TreePoint, old: TreePoint, sinks: &[TreePoint]) -> f64 {
        sinks
            .iter()
            .map(|&s| self.compute_dist(new, s) + self.compute_dist(new, old))
            .sum()
    }

    /// Keeps each branching point off blockages and taken spots at its
    /// level's distance from its parent, as `HTreeBuilder::legalize`.
    fn legalize(&mut self, ctx: &TreeContext<'_>) {
        let old_top = self.sink_region.center();
        let new_top = self.legalize_one_buffer(ctx, old_top, ctx.root_buffer);
        self.sink_region.center = Some(new_top);
        self.commit_move(old_top, new_top);
        for level_idx in 0..self.topologies.len() {
            for idx in 0..self.topologies[level_idx].points.len() {
                let point = self.topologies[level_idx].points[idx];
                let parent = self.parent_point(level_idx, idx, new_top);
                let direction = if self.is_horizontal(u32::saturating_from(level_idx) + 1) {
                    if point.x - parent.x > 0.0 {
                        Direction::East
                    } else {
                        Direction::West
                    }
                } else if point.y - parent.y > 0.0 {
                    Direction::North
                } else {
                    Direction::South
                };
                let sinks = self.topologies[level_idx].sinks[idx].tree_points();
                let length = self.topologies[level_idx].length;
                // NOTE: legalize measures with computeDist, insertion lengths
                // included; PARE the geometric distance between the buffers.
                let leng = point.dist(parent);
                let blockage = if self.is_occupied(point) {
                    None
                } else {
                    self.find_blockage(ctx, point)
                };
                let moved = if let Some(blockage) = blockage {
                    let candidates = Self::legal_locations(parent, point, blockage);
                    let search = LegalSearch {
                        ctx,
                        target: length,
                        parent,
                        sinks: &sinks,
                        direction,
                    };
                    let best = self.best_legal_location(&search, point, &candidates, blockage);
                    self.commit_move(point, best);
                    best
                } else if self.is_occupied(point) || !fuzzy_eq(leng, length, 0.01) {
                    let search = LegalSearch {
                        ctx,
                        target: length,
                        parent,
                        sinks: &sinks,
                        direction,
                    };
                    let best = self.adjust_beyond_blockage(&search, point);
                    self.commit_move(point, best);
                    best
                } else {
                    self.commit(point);
                    point
                };
                self.topologies[level_idx].points[idx] = moved;
            }
        }
        self.legalize_dummies(ctx);
    }

    /// Moves branching points with no sinks, or sitting on their sibling or
    /// parent, as `HTreeBuilder::legalizeDummy`.
    fn legalize_dummies(&mut self, ctx: &TreeContext<'_>) {
        let top = self.sink_region.center();
        for level_idx in 0..self.topologies.len() {
            for idx in 0..self.topologies[level_idx].points.len() {
                let parent = self.parent_point(level_idx, idx, top);
                let topology = &self.topologies[level_idx];
                let sinks = topology.sinks[idx].tree_points();
                let length = topology.length;
                let sibling = topology.points[LevelTopology::sibling_of(idx)];
                let mut point = topology.points[idx];
                // NOTE: legalizeDummy tests with computeDist; PARE geometrically.
                let overlap = point.dist(sibling) == 0.0 || point.dist(parent) == 0.0;
                if sinks.is_empty() {
                    point = TreePoint::new(2.0 * parent.x - sibling.x, 2.0 * parent.y - sibling.y);
                    point = scale_position(point, parent, length, 0.1);
                } else if overlap {
                    point = scale_position(point, parent, length, 0.1);
                } else {
                    continue;
                }
                if !self.is_occupied(point)
                    && let Some(blockage) = self.find_blockage(ctx, point)
                {
                    let candidates = Self::legal_locations(parent, point, blockage);
                    let search = LegalSearch {
                        ctx,
                        target: length,
                        parent,
                        sinks: &sinks,
                        direction: Direction::North,
                    };
                    // NOTE: legalizeDummy's typo passes x1, y2, x2, y2; PARE the whole box.
                    let best = self.best_legal_location(&search, point, &candidates, blockage);
                    self.commit_move(point, best);
                    point = best;
                }
                self.topologies[level_idx].points[idx] = point;
            }
        }
    }

    /// The corners of a blockage, where the branch's axes meet its edges
    /// and where the line from the parent crosses them, as
    /// `HTreeBuilder::findLegalLocations`.
    fn legal_locations(parent: TreePoint, branch: TreePoint, blockage: TreeBox) -> Vec<TreePoint> {
        let TreeBox {
            low: TreePoint { x: x1, y: y1 },
            high: TreePoint { x: x2, y: y2 },
        } = blockage;
        let (bx, by) = (branch.x, branch.y);
        let m = (parent.y - by) / (parent.x - bx);
        let candidates = [
            (x1, y1),
            (x1, y2),
            (x2, y2),
            (x2, y1),
            (bx, y1),
            (bx, y2),
            (x1, by),
            (x2, by),
            (x1, m * (x1 - bx) + by),
            (x2, m * (x2 - bx) + by),
            ((y1 - by) / m + bx, y1),
            ((y2 - by) / m + bx, y2),
        ];
        candidates
            .iter()
            .map(|&(x, y)| TreePoint::new(x, y))
            .filter(|&c| !c.same(parent) && blockage.has_on_edge(c))
            .collect()
    }

    /// The candidate around `blockage` whose path through it is
    /// nearest the search's target length, adjusted along or beyond the
    /// blockage when it is not at the target from the parent, as
    /// `HTreeBuilder::findBestLegalLocation`.
    fn best_legal_location(
        &self,
        search: &LegalSearch<'_>,
        branch: TreePoint,
        candidates: &[TreePoint],
        blockage: TreeBox,
    ) -> TreePoint {
        let (target, parent) = (search.target, search.parent);
        let mut best = TreePoint::new(0.0, 0.0);
        let mut min_diff = f64::MAX;
        for &loc in candidates {
            let d = self.compute_dist(loc, parent) + self.compute_dist(loc, branch);
            let diff = (d - target).abs();
            if diff < min_diff {
                min_diff = diff;
                best = loc;
            }
        }
        if fuzzy_eq(target, self.compute_dist(best, parent), FUZZY_EPSILON) {
            return best;
        }
        if let Some(along) = self.adjust_along_blockage(search, best, blockage) {
            return along;
        }
        self.adjust_beyond_blockage(search, best)
    }

    /// The point at the target from the parent along the blockage's edge
    /// nearest the sinks, as `HTreeBuilder::adjustAlongBlockage`.
    fn adjust_along_blockage(
        &self,
        search: &LegalSearch<'_>,
        current: TreePoint,
        blockage: TreeBox,
    ) -> Option<TreePoint> {
        let LegalSearch {
            ctx,
            target,
            parent,
            sinks,
            ..
        } = *search;
        let (x, y) = (current.x, current.y);
        let (px, py) = (parent.x, parent.y);
        let candidates = [
            TreePoint::new(x, py - target + (px - x).abs()),
            TreePoint::new(x, py + target - (px - x).abs()),
            TreePoint::new(px - target + (py - y).abs(), y),
            TreePoint::new(px + target - (py - y).abs(), y),
        ];
        // NOTE: OpenROAD marks no best with a DBL_MIN point (noBest); PARE with None.
        let mut best = None;
        let mut best_cost = f64::MAX;
        for c in candidates {
            if !fuzzy_eq(self.compute_dist(c, parent), target, FUZZY_EPSILON) {
                continue;
            }
            let legal =
                (!self.is_occupied(c) && blockage.has_on_edge(c)) || self.legal_location(ctx, c);
            if !legal {
                continue;
            }
            let cost = self.weighted_distance(c, current, sinks);
            if cost < best_cost {
                best = Some(c);
                best_cost = cost;
            }
        }
        best
    }

    /// The legal point at the target from the parent, toward the search's
    /// direction, nearest the sinks, as `HTreeBuilder::adjustBeyondBlockage`.
    fn adjust_beyond_blockage(&self, search: &LegalSearch<'_>, branch: TreePoint) -> TreePoint {
        let LegalSearch {
            ctx,
            target,
            parent,
            sinks,
            direction,
        } = *search;
        let mut candidates = arc_candidates(parent, target, direction);
        let straight = candidates[0];
        if let Some(blockage) = self.find_blockage(ctx, straight) {
            candidates.extend(along_blockage(parent, target, blockage, direction));
        }
        let (w, h) = self.buffer_size(ctx, ctx.root_buffer);
        candidates.extend(beside_branch(
            parent,
            branch,
            target,
            (10.0 * w, 10.0 * h),
            direction,
        ));
        // NOTE: OpenROAD marks no best with a DBL_MIN point (noBest); PARE with None.
        let mut best = None;
        let mut best_cost = f64::MAX;
        for c in candidates {
            if fuzzy_eq(self.compute_dist(c, parent), target, FUZZY_EPSILON)
                && self.legal_location(ctx, c)
            {
                let cost = self.weighted_distance(c, branch, sinks);
                if cost < best_cost {
                    best = Some(c);
                    best_cost = cost;
                }
            }
        }
        best.unwrap_or(branch)
    }

    /// Writes the buffers and subnets of the tree, as
    /// `HTreeBuilder::createClockSubNets`.
    fn create_clock_subnets(&mut self, ctx: &TreeContext<'_>, tech: &TechChar) {
        let center = self.sink_region.center();
        let legal_center = self.legalize_one_buffer(ctx, center, ctx.root_buffer);
        self.sink_region.center = Some(legal_center);
        self.commit_move(center, legal_center);
        let unit = self.unit();
        let root = self
            .clock
            .add_buffer("clkbuf_0", ctx.root_buffer, legal_center.to_dbu(unit));
        if self.top_buffer_name.is_empty() {
            self.top_buffer_name = self.clock.name(root).to_owned();
        }
        let root_net = self.clock.add_subnet("clknet_0");
        self.clock.subnets[root_net].level = Some(0);
        self.clock.subnets[root_net].add(root);
        // Where each branch of a level starts and the subnet driving it,
        // by parent; a point without sinks has none, nor do its branches.
        let mut parents = vec![Some((legal_center, root_net))];
        for level_idx in 0..self.topologies.len() {
            let mut drivers = vec![None; self.topologies[level_idx].points.len()];
            for (p, parent) in parents.iter().enumerate() {
                let Some((from, driving)) = *parent else {
                    continue;
                };
                for idx in LevelTopology::branches_of(p) {
                    if self.topologies[level_idx].sinks[idx].is_empty() {
                        continue;
                    }
                    let point = self.topologies[level_idx].points[idx];
                    let end = self.build_branch(ctx, tech, (level_idx, idx), from, driving);
                    drivers[idx] = Some((point, end));
                }
            }
            parents = drivers;
        }
        let leaf = self.topologies.len() - 1;
        for (idx, driver) in parents.iter().enumerate() {
            let Some((_, subnet)) = *driver else {
                continue;
            };
            self.clock.subnets[subnet].leaf = true;
            // NOTE: createClockSubNets finds each sink by its location
            // (mapLocationToSink_); PARE carries which sink it is.
            for &inst in &self.topologies[leaf].sinks[idx].insts {
                self.clock.subnets[subnet].add(inst);
            }
        }
    }

    /// Lays the buffers of the branch from `from` to point `idx` of level
    /// `level_idx`, driven by subnet `driving`; returns the subnet the
    /// branch's end drives.
    fn build_branch(
        &mut self,
        ctx: &TreeContext<'_>,
        tech: &TechChar,
        (level_idx, idx): (usize, usize),
        from: TreePoint,
        driving: usize,
    ) -> usize {
        let point = self.topologies[level_idx].points[idx];
        let legal = self.legalize_one_buffer(ctx, point, ctx.root_buffer);
        self.commit_move(point, legal);
        if level_idx > 0 && self.clock.subnets[driving].level.is_none() {
            self.clock.subnets[driving].level = Some(u32::saturating_from(level_idx));
        }
        let wires = self.topologies[level_idx].wire_segments.clone();
        let mut segment = SegmentBuilder {
            inst_prefix: format!("clkbuf_{}_{idx}_", level_idx + 1),
            net_prefix: format!("clknet_{}_{idx}_", level_idx + 1),
            root: from,
            target: legal,
            driving,
            levels: 0,
        };
        segment.build(self, ctx, tech, &wires);
        if level_idx == self.topologies.len() - 1 {
            segment.force_buffer(self, ctx.root_buffer);
        }
        segment.driving
    }

    /// One buffer driving every sink, as
    /// `HTreeBuilder::createSingleBufferClockNet`.
    fn single_buffer_clock_net(&mut self, ctx: &TreeContext<'_>) {
        let center = self.sink_region.center();
        let legal = self.legalize_one_buffer(ctx, center, ctx.root_buffer);
        self.sink_region.center = Some(legal);
        self.commit_move(center, legal);
        let unit = self.unit();
        let location = legal.to_dbu(unit);
        let root = self.clock.add_buffer("clkbuf_0", ctx.root_buffer, location);
        if self.top_buffer_name.is_empty() {
            self.top_buffer_name = self.clock.name(root).to_owned();
        }
        let subnet = self.clock.add_subnet("clknet_0");
        self.clock.subnets[subnet].level = Some(0);
        self.clock.subnets[subnet].add(root);
        for i in 0..self.clock.sinks.len() {
            self.clock.subnets[subnet].add(InstRef::Sink(i));
        }
    }
}

/// What a search for a legal branching point looks for: a point `target`
/// from `parent`, toward `direction`, near `sinks`.
#[derive(Clone, Copy, Debug)]
struct LegalSearch<'a> {
    ctx: &'a TreeContext<'a>,
    target: f64,
    parent: TreePoint,
    sinks: &'a [TreePoint],
    direction: Direction,
}

/// Pulls a point toward its parent until it is `length` away, after
/// pushing it a `scale` share off the axes, as
/// `HTreeBuilder::scalePosition`.
fn scale_position(p: TreePoint, parent: TreePoint, length: f64, scale: f64) -> TreePoint {
    // NOTE: scalePosition measures with computeDist; PARE geometrically.
    let d = p.dist(parent);
    if d > 0.0 {
        let delta = d * scale;
        let mut dx = p.x - parent.x;
        let mut dy = p.y - parent.y;
        dx += if dx > 0.0 { delta } else { -delta };
        dy += if dy > 0.0 { -delta } else { delta };
        let s = length / d;
        TreePoint::new(parent.x + dx * s, parent.y + dy * s)
    } else {
        TreePoint::new(parent.x + length / 2.0, parent.y + length / 2.0)
    }
}

/// Splits sinks between two branching points by capacitated k-means and
/// moves the points to the means, as
/// `HTreeBuilder::refineBranchingPointsWithClustering`.
fn refine_with_clustering(
    ctx: &TreeContext<'_>,
    topology: &mut LevelTopology,
    a: usize,
    b: usize,
    root: TreePoint,
    sinks: &BranchSinks,
) {
    let mut engine = Clustering::new(&sinks.points, root.to_sink_point());
    let pa = topology.points[a];
    let pb = topology.points[b];
    let mut means = [pa.to_sink_point(), pb.to_sink_point()];
    let share = if ctx.settings.max_fanout.is_some() {
        FANOUT_LIMITED_CLUSTERING_CAPACITY
    } else {
        CLUSTERING_CAPACITY
    };
    let cap = usize::saturating_from(sinks.len() as f64 * share);
    engine.run(cap, KMEANS_ROUNDS, CLUSTERING_POWER, &mut means);
    topology.points[a] = TreePoint::from_f32(means[0]);
    topology.points[b] = TreePoint::from_f32(means[1]);
    for (cluster, members) in engine.clusters().iter().enumerate() {
        let branch = if cluster == 0 { a } else { b };
        for &s in members {
            topology.sinks[branch].push(sinks.points[s], sinks.insts[s]);
        }
    }
}

/// The most wire allowed since the last buffer, from the max wire length
/// when the resizer gave one, else `MAX_SLEW` of the longest segments.
fn wire_length_threshold(ctx: &TreeContext<'_>, tech: &TechChar) -> LengthSteps {
    match ctx.settings.max_wire_length {
        None => tech.max_segment_length() * i64::from(MAX_SLEW),
        // NOTE: computeLevelTopology divides by the characterization unit,
        // twice the length unit the segments count in; PARE by the latter.
        Some(max) => tech.length_unit().steps_in(max),
    }
}

/// What a segment search starts from.
#[derive(Clone, Copy, Debug)]
struct SegmentQuery {
    length: LengthSteps,
    input_slew: SlewSteps,
    input_cap: CapSteps,
    slew_threshold: SlewSteps,
    /// The most wire since the last buffer.
    wirelength_threshold: LengthSteps,
}

/// The slew, load and wire since the last buffer a chosen segment leaves,
/// and so where the next segment starts.
#[derive(Clone, Copy, Debug)]
struct SegmentChoice {
    slew: SlewSteps,
    cap: CapSteps,
    curr_wl: LengthSteps,
}

/// The fastest segment of `query.length` that starts within `tolerance`
/// of the input slew and load and keeps the wire since the last buffer
/// short enough, preferring a buffered one once the slew reaches its
/// threshold, as `HTreeBuilder::computeMinDelaySegment`.
fn min_delay_segment(
    tech: &TechChar,
    query: &SegmentQuery,
    tolerance: u32,
    out: &mut SegmentChoice,
) -> Option<usize> {
    let fastest = fastest_segments(tech, query, tolerance, out.curr_wl);
    if query.input_slew >= query.slew_threshold {
        if let Some(key) = fastest.buffered {
            leave_buffered(tech.segment(key), out);
            return Some(key);
        }
        if tolerance < MAX_TOLERANCE {
            return min_delay_segment(tech, query, tolerance + 1, out);
        }
    }
    let Some(key) = fastest.any else {
        if tolerance >= MAX_TOLERANCE {
            return None;
        }
        return min_delay_segment(tech, query, tolerance + 1, out);
    };
    let seg = tech.segment(key);
    if seg.is_buffered() {
        leave_buffered(seg, out);
    } else {
        out.slew = seg.output_slew_steps().max(query.input_slew.next());
        out.curr_wl += seg.after_last_buffer();
        out.cap = seg.load_steps();
    }
    Some(key)
}

/// The fastest candidate segment, and the fastest buffered one.
struct Fastest {
    any: Option<usize>,
    buffered: Option<usize>,
}

/// The fastest of the [`candidates`], the first of equal delays winning.
fn fastest_segments(
    tech: &TechChar,
    query: &SegmentQuery,
    tolerance: u32,
    curr_wl: LengthSteps,
) -> Fastest {
    let mut fastest = Fastest {
        any: None,
        buffered: None,
    };
    let (mut min_delay, mut min_buf_delay) = (u32::MAX, u32::MAX);
    for key in candidates(tech, query, tolerance, curr_wl) {
        let seg = tech.segment(key);
        if seg.delay < min_delay {
            min_delay = seg.delay;
            fastest.any = Some(key);
        }
        if seg.is_buffered() && seg.delay < min_buf_delay {
            min_buf_delay = seg.delay;
            fastest.buffered = Some(key);
        }
    }
    fastest
}

/// The segments of `query.length` that start within `tolerance` of its
/// slew and load and keep the wire since the last buffer, `curr_wl` so
/// far, under its threshold; in table order, by load then output slew.
fn candidates<'t>(
    tech: &'t TechChar,
    query: &'t SegmentQuery,
    tolerance: u32,
    curr_wl: LengthSteps,
) -> impl Iterator<Item = usize> + 't {
    CapSteps::through(tech.max_capacitance())
        .flat_map(move |load| SlewSteps::through(tech.max_slew()).map(move |slew| (load, slew)))
        .flat_map(move |(load, slew)| tech.segments_with(query.length, load, slew).iter().copied())
        .filter(move |&key| fits(tech, tech.segment(key), query, (tolerance, curr_wl)))
}

/// Whether `seg` starts within `tolerance` of the query's slew and load and
/// keeps the wire since the last buffer, `curr_wl` so far, under the limit.
fn fits(
    tech: &TechChar,
    seg: &WireSegment,
    query: &SegmentQuery,
    (tolerance, curr_wl): (u32, LengthSteps),
) -> bool {
    if seg.input_cap_steps().steps_from(query.input_cap) > tolerance
        || seg.input_slew_steps().steps_from(query.input_slew) > tolerance
    {
        return false;
    }
    let limit = if seg.is_buffered() {
        query.wirelength_threshold
    } else {
        // NOTE: computeMinDelaySegment subtracts in unsigned, so a limit below
        // the shortest segment wraps and admits any bare wire; PARE's is signed.
        query.wirelength_threshold - tech.min_segment_length()
    };
    curr_wl + seg.to_first_buffer() <= limit
}

/// Leaves `out` where buffered segment `seg` ends: its slew and load, and
/// the wire after its last buffer.
fn leave_buffered(seg: &WireSegment, out: &mut SegmentChoice) {
    out.slew = seg.output_slew_steps();
    out.cap = seg.load_steps();
    out.curr_wl = seg.after_last_buffer();
}

/// The buffers along one branch, from its parent's point to its own, as
/// `TritonCTS`'s `SegmentBuilder`.
struct SegmentBuilder {
    inst_prefix: String,
    net_prefix: String,
    root: TreePoint,
    target: TreePoint,
    driving: usize,
    levels: u32,
}

impl SegmentBuilder {
    fn build(
        &mut self,
        tree: &mut Builder,
        ctx: &TreeContext<'_>,
        tech: &TechChar,
        wires: &[usize],
    ) {
        let length_x = (self.root.x - self.target.x).abs();
        let low_to_high_x = self.root.x < self.target.x;
        let low_to_high_y = self.root.y < self.target.y;
        let mut connection = 0.0_f64;
        let unit = tree.unit();
        for &wire in wires {
            let segment = tech.segment(wire);
            let length = f64::from(segment.length);
            let mut to_last_buffer = 0.0;
            for (&location, &master) in segment.buffer_locations.iter().zip(&segment.buffer_masters)
            {
                connection += location * length;
                to_last_buffer += location * length;
                let (x, y) = if connection < length_x {
                    let x = if low_to_high_x {
                        self.root.x + connection
                    } else {
                        self.root.x - connection
                    };
                    (x, self.root.y)
                } else {
                    let y = if low_to_high_y {
                        self.root.y + (connection - length_x)
                    } else {
                        self.root.y - (connection - length_x)
                    };
                    (self.target.x, y)
                };
                let at = TreePoint::new(x, y);
                let legal = tree.legalize_one_buffer(ctx, at, master);
                tree.commit_move(at, legal);
                let buffer = tree.clock.add_buffer(
                    &format!("{}{}", self.inst_prefix, self.levels),
                    master,
                    legal.to_dbu(unit),
                );
                if !at.same(legal) {
                    connection -= tree.compute_dist(at, legal);
                }
                tree.clock.subnets[self.driving].add(buffer);
                self.driving = tree
                    .clock
                    .add_subnet(&format!("{}{}", self.net_prefix, self.levels));
                tree.clock.subnets[self.driving].add(buffer);
                self.levels += 1;
            }
            // NOTE: SegmentBuilder::build adds the whole length after the
            // buffers, counting the wire up to them twice; PARE the rest.
            connection += length - to_last_buffer;
        }
    }

    /// Puts a buffer at the branch's end when the segments placed none, as
    /// `SegmentBuilder::forceBufferInSegment`.
    fn force_buffer(&mut self, tree: &mut Builder, master: BufferCell) {
        if self.levels != 0 {
            return;
        }
        let unit = tree.unit();
        let buffer = tree.clock.add_buffer(
            &format!("{}_f", self.inst_prefix),
            master,
            self.target.to_dbu(unit),
        );
        tree.clock.subnets[self.driving].add(buffer);
        self.driving = tree.clock.add_subnet(&format!("{}_leaf", self.net_prefix));
        tree.clock.subnets[self.driving].add(buffer);
        self.levels += 1;
    }
}

/// The points at distance `target` from `parent` toward `direction`:
/// straight ahead, then half and quarter of the way across to each side.
fn arc_candidates(parent: TreePoint, target: f64, direction: Direction) -> Vec<TreePoint> {
    let (px, py) = (parent.x, parent.y);
    let l50 = target * 0.5;
    let l25 = target * 0.25;
    let l75 = target * 0.75;
    match direction {
        Direction::North => vec![
            TreePoint::new(px, py + target),
            TreePoint::new(px + l50, py + l50),
            TreePoint::new(px - l50, py + l50),
            TreePoint::new(px + l25, py + l75),
            TreePoint::new(px - l25, py + l75),
        ],
        Direction::East => vec![
            TreePoint::new(px + target, py),
            TreePoint::new(px + l50, py + l50),
            TreePoint::new(px + l50, py - l50),
            TreePoint::new(px + l75, py + l25),
            TreePoint::new(px + l75, py - l25),
        ],
        Direction::South => vec![
            TreePoint::new(px, py - target),
            TreePoint::new(px + l50, py - l50),
            TreePoint::new(px - l50, py - l50),
            TreePoint::new(px + l25, py - l75),
            TreePoint::new(px - l25, py - l75),
        ],
        Direction::West => vec![
            TreePoint::new(px - target, py),
            TreePoint::new(px - l50, py - l50),
            TreePoint::new(px - l50, py + l50),
            TreePoint::new(px - l75, py - l25),
            TreePoint::new(px - l75, py + l25),
        ],
    }
}

/// The two points at distance `target` from `parent` along the side of
/// `blockage` facing it.
fn along_blockage(
    parent: TreePoint,
    target: f64,
    blockage: TreeBox,
    direction: Direction,
) -> [TreePoint; 2] {
    let TreeBox {
        low: TreePoint { x: x1, y: y1 },
        high: TreePoint { x: x2, y: y2 },
    } = blockage;
    let (px, py) = (parent.x, parent.y);
    match direction {
        // NOTE: addCandidatePointsAlongBlockage sets the second point's y
        // where its own derivation gives x; PARE puts it at the target.
        Direction::North => [
            TreePoint::new(-target - py + px + y1, y1),
            TreePoint::new(target + py + px - y1, y1),
        ],
        Direction::East => [
            TreePoint::new(x1, target + py + px - x1),
            TreePoint::new(x1, -target + py - px + x1),
        ],
        Direction::South => [
            TreePoint::new(-target + py + px - y2, y2),
            TreePoint::new(target - py + px + y2, y2),
        ],
        Direction::West => [
            TreePoint::new(x2, target + py - px + x2),
            TreePoint::new(x2, -target + py + px - x2),
        ],
    }
}

/// The two points at distance `target` from `parent` that lie `margin`
/// to either side of `branch`, across `direction`.
fn beside_branch(
    parent: TreePoint,
    branch: TreePoint,
    target: f64,
    (min_x, min_y): (f64, f64),
    direction: Direction,
) -> [TreePoint; 2] {
    let (px, py) = (parent.x, parent.y);
    let x_plus = branch.x + min_x;
    let x_minus = branch.x - min_x;
    let y_plus = branch.y + min_y;
    let y_minus = branch.y - min_y;
    match direction {
        Direction::South => [
            TreePoint::new(x_plus, py - target + (px - x_plus).abs()),
            TreePoint::new(x_minus, py - target + (px - x_minus).abs()),
        ],
        Direction::North => [
            TreePoint::new(x_plus, py + target - (px - x_plus).abs()),
            TreePoint::new(x_minus, py + target - (px - x_minus).abs()),
        ],
        Direction::West => [
            TreePoint::new(px - target + (py - y_plus).abs(), y_plus),
            TreePoint::new(px - target + (py - y_minus).abs(), y_minus),
        ],
        Direction::East => [
            TreePoint::new(px + target - (py - y_plus).abs(), y_plus),
            TreePoint::new(px + target - (py - y_minus).abs(), y_minus),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clock::ClockSink;
    use crate::options::CtsOptions;
    use pare_common::db::indices::{NetId, PinId};
    use pare_common::db::liberty::{LibCellId, LibPortId};

    fn rect(x0: i64, y0: i64, x1: i64, y1: i64) -> Rect<Dbu> {
        Rect::new(
            Point::new(Dbu::new(x0), Dbu::new(y0)),
            Point::new(Dbu::new(x1), Dbu::new(y1)),
        )
    }

    fn buffer() -> BufferCell {
        BufferCell {
            id: LibCellId::new(0),
            input: LibPortId::new(0),
            output: LibPortId::new(1),
        }
    }

    /// What a tree context borrows: blockages and buffer sizes in a core
    /// 100 database units a side.
    struct Scene {
        settings: Settings,
        buffer_sizes: FxHashMap<BufferCell, Point<Dbu>>,
        blockages: Vec<Rect<Dbu>>,
    }

    impl Scene {
        /// The scene of `blockages`, its buffer of no size.
        fn new(blockages: Vec<Rect<Dbu>>) -> Self {
            Self {
                settings: Settings::new(CtsOptions::default()),
                buffer_sizes: FxHashMap::default(),
                blockages,
            }
            .with_buffer_size(0, 0)
        }

        fn with_buffer_size(mut self, width: i64, height: i64) -> Self {
            let _ = self
                .buffer_sizes
                .insert(buffer(), Point::new(Dbu::new(width), Dbu::new(height)));
            self
        }

        fn ctx(&self) -> TreeContext<'_> {
            TreeContext {
                settings: &self.settings,
                core: rect(0, 0, 100, 100),
                dbu: 1000,
                root_buffer: buffer(),
                sink_buffer: buffer(),
                buffer_sizes: &self.buffer_sizes,
                blockages: &self.blockages,
            }
        }
    }

    /// A builder whose tree unit is one database unit, its sinks spread
    /// over the whole core.
    fn builder() -> Builder {
        let mut builder = Builder::new(
            Clock::new(NetId::new(0), "clk", "clk"),
            None,
            LengthUnit::new(1),
        );
        builder.sink_region = Region::new(0.0, 0.0, 100.0, 100.0);
        builder
    }

    #[test]
    fn a_dummy_in_a_blockage_moves_along_the_blockages_full_box() {
        // The dummy lands inside the first blockage; the first point at the
        // level's length from the parent along it is its lower left corner,
        // which the second blockage covers.
        let scene = Scene::new(vec![rect(60, 40, 80, 60), rect(55, 35, 65, 45)]);
        let mut builder = builder();
        let mut level = LevelTopology::new(20.0, Vec::new(), NO_CHOICE);
        let (dummy, branch) =
            level.add_branches(TreePoint::new(0.0, 0.0), TreePoint::new(30.0, 50.0));
        level.sinks[branch].push(SinkPoint::new(30.0, 40.0), InstRef::Sink(0));
        builder.topologies.push(level);

        builder.legalize_dummies(&scene.ctx());

        let moved = builder.topologies[0].points[dummy];
        assert!(moved.same(TreePoint::new(60.0, 40.0)), "{moved:?}");
    }

    #[test]
    fn a_buffer_outside_every_blockage_stays_put() {
        let scene = Scene::new(vec![rect(40, 40, 60, 60)]).with_buffer_size(4, 2);
        let at = TreePoint::new(20.0, 50.0);

        let legal = builder().legalize_in_blockages(&scene.ctx(), at, buffer());

        assert!(legal.same(at), "{legal:?}");
    }

    #[test]
    fn a_buffer_on_a_blockage_edge_is_outside_it() {
        let scene = Scene::new(vec![rect(40, 40, 60, 60)]).with_buffer_size(4, 2);
        let at = TreePoint::new(40.0, 50.0);

        let legal = builder().legalize_in_blockages(&scene.ctx(), at, buffer());

        assert!(legal.same(at), "{legal:?}");
    }

    #[test]
    fn a_buffer_in_a_blockage_moves_its_width_clear_of_the_nearest_side() {
        let scene = Scene::new(vec![rect(40, 40, 60, 60)]).with_buffer_size(4, 2);

        let legal =
            builder().legalize_in_blockages(&scene.ctx(), TreePoint::new(45.0, 50.0), buffer());

        assert!(legal.same(TreePoint::new(36.0, 50.0)), "{legal:?}");
    }

    #[test]
    fn a_buffer_in_a_blockage_skips_a_taken_spot_beside_it() {
        let scene = Scene::new(vec![rect(40, 40, 60, 60)]).with_buffer_size(4, 2);
        let mut builder = builder();
        builder.commit(TreePoint::new(36.0, 50.0));

        let legal =
            builder.legalize_in_blockages(&scene.ctx(), TreePoint::new(45.0, 50.0), buffer());

        assert!(legal.same(TreePoint::new(45.0, 60.0)), "{legal:?}");
    }

    /// A segment `length` units long with `delay`, entering and leaving at
    /// slew and load 1, with a buffer at each share of `buffers`, the
    /// first `wl_to_first_buffer` units in.
    fn segment(length: u8, delay: u32, buffers: &[f64], wl_to_first_buffer: i32) -> WireSegment {
        WireSegment {
            length,
            load: 1,
            output_slew: 1,
            delay,
            input_cap: 1,
            input_slew: 1,
            buffer_locations: buffers.to_vec(),
            buffer_masters: buffers.iter().map(|_| buffer()).collect(),
            wl_to_first_buffer,
            last_wl: 0,
        }
    }

    /// A search for a segment `length` long from slew and load 1 with no
    /// wire since the last buffer, under `wirelength_threshold`.
    const fn query(length: i64, wirelength_threshold: i64) -> SegmentQuery {
        SegmentQuery {
            length: LengthSteps::new(length),
            input_slew: SlewSteps::FIRST,
            input_cap: CapSteps::new(1),
            slew_threshold: SlewSteps::new(MAX_SLEW),
            wirelength_threshold: LengthSteps::new(wirelength_threshold),
        }
    }

    const NO_CHOICE: SegmentChoice = SegmentChoice {
        slew: SlewSteps::new(0),
        cap: CapSteps::new(0),
        curr_wl: LengthSteps::new(0),
    };

    /// The segment `search` finds in `tech`, starting from no wire since
    /// the last buffer.
    fn choose(tech: &TechChar, search: &SegmentQuery) -> Option<usize> {
        let mut out = NO_CHOICE;
        min_delay_segment(tech, search, 1, &mut out)
    }

    #[test]
    fn the_fastest_segment_within_the_tolerance_of_the_input_is_chosen() {
        let tech = TechChar::from_segments(
            vec![
                segment(2, 5, &[], 2),
                segment(2, 3, &[], 2),
                WireSegment {
                    input_cap: 5,
                    ..segment(2, 1, &[], 2)
                },
            ],
            1,
        );

        let chosen = choose(&tech, &query(2, 100));

        assert_eq!(chosen, Some(1));
    }

    #[test]
    fn a_search_finding_nothing_widens_its_tolerance() {
        let tech = TechChar::from_segments(
            vec![WireSegment {
                input_cap: 3,
                ..segment(2, 1, &[], 2)
            }],
            1,
        );

        let chosen = choose(&tech, &query(2, 100));

        assert_eq!(chosen, Some(0));
    }

    #[test]
    fn once_the_slew_reaches_its_threshold_a_buffered_segment_beats_a_faster_bare_one() {
        let at_threshold = |s: WireSegment| WireSegment { input_slew: 4, ..s };
        let tech = TechChar::from_segments(
            vec![
                at_threshold(segment(2, 1, &[], 2)),
                at_threshold(segment(2, 5, &[0.5], 1)),
            ],
            1,
        );
        let search = SegmentQuery {
            input_slew: SlewSteps::new(4),
            ..query(2, 100)
        };

        let chosen = choose(&tech, &search);

        assert_eq!(chosen, Some(1));
    }

    #[test]
    fn a_bare_segment_raises_the_slew_and_adds_its_wire_to_the_unbuffered_run() {
        let tech = TechChar::from_segments(
            vec![WireSegment {
                last_wl: 2,
                ..segment(2, 1, &[], 2)
            }],
            1,
        );
        let mut out = SegmentChoice {
            curr_wl: LengthSteps::new(3),
            ..NO_CHOICE
        };

        let _ = min_delay_segment(&tech, &query(2, 100), 1, &mut out);

        assert_eq!(
            (out.slew, out.curr_wl),
            (SlewSteps::new(2), LengthSteps::new(5))
        );
    }

    #[test]
    fn a_buffered_segment_whose_first_buffer_is_past_the_wire_limit_is_passed_over() {
        let tech =
            TechChar::from_segments(vec![segment(2, 1, &[0.5], 6), segment(2, 5, &[0.5], 1)], 1);

        let chosen = choose(&tech, &query(2, 5));

        assert_eq!(chosen, Some(1));
    }

    #[test]
    fn a_levels_segments_add_up_to_its_branch_length() {
        let scene = Scene::new(Vec::new());
        let tech = TechChar::from_segments(vec![segment(2, 1, &[], 2), segment(1, 1, &[], 1)], 1);
        let mut tree = builder();
        tree.min_input_cap = CapSteps::new(1);
        tree.min_length_sink_region = LengthSteps::new(2);
        tree.top_level_sinks_clustered = BranchSinks {
            points: vec![SinkPoint::new(10.0, 50.0), SinkPoint::new(90.0, 50.0)],
            insts: vec![InstRef::Sink(0), InstRef::Sink(1)],
        };

        tree.level_topology(&scene.ctx(), &tech, 1, 10.0, 10.0);

        let lengths: Vec<u8> = tree.topologies[0]
            .wire_segments
            .iter()
            .map(|&k| tech.segment(k).length)
            .collect();
        assert_eq!(lengths, [2, 2, 1]);
    }

    #[test]
    fn a_bare_segment_is_refused_under_a_limit_shorter_than_the_shortest_segment() {
        let tech = TechChar::from_segments(vec![segment(2, 1, &[], 2)], 1);

        assert_eq!(choose(&tech, &query(2, 1)), None);
    }

    /// The segments chosen for the first level of a tree whose length unit
    /// is 14000 database units, 7 microns, under a wire length limit of
    /// `limit_um` microns: 0, buffered 10 units (70 microns) in, or 1, a
    /// slower bare segment.
    fn first_level_under(limit_um: i64) -> Vec<usize> {
        let mut scene = Scene::new(Vec::new());
        scene.settings.max_wire_length = Some(Dbu::new(limit_um * 2000));
        let tech = TechChar::from_segments(
            vec![segment(10, 1, &[1.0], 10), segment(10, 9, &[], 10)],
            14_000,
        );
        let mut tree = builder();
        tree.unit = LengthUnit::new(14_000);
        tree.min_input_cap = CapSteps::new(1);
        tree.min_length_sink_region = LengthSteps::new(20);
        tree.sink_region = Region::new(0.0, 0.0, 40.0, 40.0);
        tree.top_level_sinks_clustered = BranchSinks {
            points: vec![SinkPoint::new(1.0, 1.0), SinkPoint::new(39.0, 39.0)],
            insts: vec![InstRef::Sink(0), InstRef::Sink(1)],
        };

        tree.level_topology(&scene.ctx(), &tech, 1, 20.0, 20.0);

        tree.topologies[0].wire_segments.clone()
    }

    #[test]
    fn a_wire_length_limit_admits_a_first_buffer_within_it() {
        assert_eq!(first_level_under(100), [0]);
    }

    /// Where `SegmentBuilder` puts the buffers of `segments`, laid end to
    /// end along the branch from `root` to `target`, in a tree whose unit
    /// is one database unit.
    fn buffers_along(
        root: TreePoint,
        target: TreePoint,
        segments: Vec<WireSegment>,
    ) -> Vec<Point<Dbu>> {
        let scene = Scene::new(Vec::new()).with_buffer_size(1, 1);
        let wires: Vec<usize> = (0..segments.len()).collect();
        let tech = TechChar::from_segments(segments, 1);
        let mut tree = builder();
        let driving = tree.clock.add_subnet("root");
        let mut branch = SegmentBuilder {
            inst_prefix: "b_".into(),
            net_prefix: "n_".into(),
            root,
            target,
            driving,
            levels: 0,
        };

        branch.build(&mut tree, &scene.ctx(), &tech, &wires);

        tree.clock.buffers.iter().map(|b| b.location).collect()
    }

    fn at(x: i64, y: i64) -> Point<Dbu> {
        Point::new(Dbu::new(x), Dbu::new(y))
    }

    #[test]
    fn a_buffer_halfway_along_a_straight_branch_sits_at_its_middle() {
        let placed = buffers_along(
            TreePoint::new(0.0, 50.0),
            TreePoint::new(10.0, 50.0),
            vec![segment(10, 1, &[0.5], 5)],
        );

        assert_eq!(placed, [at(5, 50)]);
    }

    #[test]
    fn buffers_of_successive_segments_sit_where_each_segment_puts_them() {
        let placed = buffers_along(
            TreePoint::new(0.0, 50.0),
            TreePoint::new(20.0, 50.0),
            vec![segment(10, 1, &[0.5], 5), segment(10, 1, &[0.5], 5)],
        );

        assert_eq!(placed, [at(5, 50), at(15, 50)]);
    }

    #[test]
    fn a_buffer_past_the_branchs_horizontal_run_turns_toward_its_target() {
        let placed = buffers_along(
            TreePoint::new(0.0, 0.0),
            TreePoint::new(4.0, 10.0),
            vec![segment(14, 1, &[0.5], 7)],
        );

        assert_eq!(placed, [at(4, 3)]);
    }

    #[test]
    fn every_sink_hangs_off_one_leaf_net_driven_by_a_buffer() {
        let scene = Scene::new(Vec::new()).with_buffer_size(1, 1);
        let mut tech =
            TechChar::from_segments(vec![segment(2, 1, &[0.5], 1), segment(1, 1, &[], 1)], 1);
        let mut clock = Clock::new(NetId::new(0), "clk", "clk");
        for (i, (x, y)) in [(10, 10), (90, 10), (10, 90), (90, 90)]
            .into_iter()
            .enumerate()
        {
            clock.add_sink(ClockSink {
                name: format!("ff{i}/CK"),
                location: at(x, y),
                pin: PinId::new(i),
                input_cap: Farads32::ZERO,
                insertion_delay: Seconds::ZERO,
            });
        }
        let mut tree = Builder::new(clock, None, tech.length_unit());

        tree.run(&scene.ctx(), &mut tech);

        let mut reached = Vec::new();
        for subnet in tree.clock.subnets.iter().filter(|s| s.leaf) {
            assert!(matches!(subnet.driver(), InstRef::Buffer(_)), "{subnet:?}");
            for &sink in subnet.sinks() {
                if let InstRef::Sink(i) = sink {
                    reached.push(i);
                }
            }
        }
        reached.sort_unstable();
        assert_eq!(reached, [0, 1, 2, 3]);
    }

    #[test]
    fn a_blockages_legal_locations_are_its_corners_and_where_the_branch_axes_cross_it() {
        let blockage = TreeBox {
            low: TreePoint::new(40.0, 40.0),
            high: TreePoint::new(60.0, 60.0),
        };

        let found = Builder::legal_locations(
            TreePoint::new(0.0, 50.0),
            TreePoint::new(50.0, 50.0),
            blockage,
        );

        let found: Vec<(f64, f64)> = found.iter().map(|p| (p.x, p.y)).collect();
        assert_eq!(
            found,
            [
                (40.0, 40.0),
                (40.0, 60.0),
                (60.0, 60.0),
                (60.0, 40.0),
                (50.0, 40.0),
                (50.0, 60.0),
                (40.0, 50.0),
                (60.0, 50.0),
                (40.0, 50.0),
                (60.0, 50.0),
            ]
        );
    }

    #[test]
    fn a_point_moved_along_a_blockage_keeps_its_distance_and_nears_the_sinks() {
        let scene = Scene::new(vec![rect(40, 20, 70, 60)]);
        let blockage = TreeBox {
            low: TreePoint::new(40.0, 20.0),
            high: TreePoint::new(70.0, 60.0),
        };
        let ctx = scene.ctx();
        let sinks = [TreePoint::new(66.0, 18.0)];
        let search = LegalSearch {
            ctx: &ctx,
            target: 30.0,
            parent: TreePoint::new(50.0, 0.0),
            sinks: &sinks,
            direction: Direction::North,
        };

        let moved = builder().adjust_along_blockage(&search, TreePoint::new(45.0, 20.0), blockage);

        assert!(
            moved.is_some_and(|p| p.same(TreePoint::new(60.0, 20.0))),
            "{moved:?}"
        );
    }

    #[test]
    fn a_north_branch_blocked_ahead_moves_along_the_blockage_edge_toward_its_sinks() {
        let scene = Scene::new(vec![rect(40, 20, 70, 60)]);
        let ctx = scene.ctx();
        let sinks = [TreePoint::new(62.0, 18.0)];
        let search = LegalSearch {
            ctx: &ctx,
            target: 30.0,
            parent: TreePoint::new(50.0, 0.0),
            sinks: &sinks,
            direction: Direction::North,
        };

        let moved = builder().adjust_beyond_blockage(&search, TreePoint::new(50.0, 30.0));

        assert!(moved.same(TreePoint::new(60.0, 20.0)), "{moved:?}");
    }
}
