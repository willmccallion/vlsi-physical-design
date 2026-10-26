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

use crate::clock::{Clock, InstRef};
use crate::kmeans::Clustering;
use crate::options::Settings;
use crate::sink_clustering::SinkClustering;
use crate::techchar::TechChar;
use pare_common::db::liberty::LibCellId;
use pare_common::geom::dbu::Dbu;
use pare_common::geom::rect::Rect;
use rustc_hash::{FxHashMap, FxHashSet};

const MIN_CLUSTERING_SINKS: usize = 200;
const MIN_CLUSTERING_MACRO_SINKS: usize = 10;
const CLUSTER_DIAMETERS: [u32; 3] = [50, 100, 200];
const MAX_TOLERANCE: u32 = 10;

/// A position in the tree's length unit.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    fn from_f32(p: (f32, f32)) -> Self {
        Self::new(f64::from(p.0), f64::from(p.1))
    }

    pub fn dist(self, other: Self) -> f64 {
        (self.x - other.x).abs() + (self.y - other.y).abs()
    }

    /// `TritonCTS`'s point equality, to a millionth of a unit.
    pub fn same(self, other: Self) -> bool {
        (self.x - other.x).abs() < 1e-6 && (self.y - other.y).abs() < 1e-6
    }

    /// The exact key `TritonCTS` hashes and orders points by.
    fn key(self) -> (u64, u64) {
        let bits = |v: f64| if v == 0.0 { 0 } else { v.to_bits() };
        (bits(self.x), bits(self.y))
    }

    /// The key of the point rounded to single precision, as sink positions
    /// are stored.
    fn key_f32(self) -> (u64, u64) {
        Self::new(f64::from(self.x as f32), f64::from(self.y as f32)).key()
    }
}

/// `TritonCTS`'s `fuzzyEqual`.
fn fuzzy_eq(a: f64, b: f64, epsilon: f64) -> bool {
    (a - b).abs() < epsilon
}

fn fuzzy_ge(a: f64, b: f64) -> bool {
    fuzzy_eq(a, b, 1e-6) || a > b
}

fn fuzzy_le(a: f64, b: f64) -> bool {
    fuzzy_eq(a, b, 1e-6) || a < b
}

/// A region with a center that, once read, stays put unless set, as
/// `TritonCTS`'s `Box`.
#[derive(Clone, Copy, Debug, Default)]
struct Region {
    x_min: f64,
    y_min: f64,
    width: f64,
    height: f64,
    center: Option<Point>,
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

    fn center(&mut self) -> Point {
        *self.center.get_or_insert_with(|| {
            Point::new(
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
/// branching points with their parents and sinks.
#[derive(Clone, Debug)]
struct LevelTopology {
    length: f64,
    wire_segments: Vec<usize>,
    points: Vec<Point>,
    parents: Vec<Option<usize>>,
    sinks: Vec<Vec<Point>>,
    driving_subnet: Vec<Option<usize>>,
    output_slew: u32,
    output_cap: u32,
    curr_wl: i32,
}

impl LevelTopology {
    const fn new(length: f64) -> Self {
        Self {
            length,
            wire_segments: Vec::new(),
            points: Vec::new(),
            parents: Vec::new(),
            sinks: Vec::new(),
            driving_subnet: Vec::new(),
            output_slew: 0,
            output_cap: 0,
            curr_wl: 0,
        }
    }

    fn add_point(&mut self, point: Point, parent: Option<usize>) -> usize {
        self.points.push(point);
        self.parents.push(parent);
        self.sinks.push(Vec::new());
        self.driving_subnet.push(None);
        self.points.len() - 1
    }

    fn sibling(&self, i: usize, parent: Option<usize>) -> usize {
        (0..self.points.len())
            .find(|&idx| idx != i && self.parents[idx] == parent)
            .unwrap_or(i)
    }
}

/// A parent branching point: its index, position and sinks.
type BranchRoot = (usize, Point, Vec<(f32, f32)>);

/// What every tree builder of a design shares.
#[derive(Clone, Copy, Debug)]
pub(crate) struct TreeContext<'a> {
    pub settings: &'a Settings,
    pub core: Rect<Dbu>,
    pub dbu: i32,
    pub root_buffer: LibCellId,
    pub sink_buffer: LibCellId,
    pub buffer_sizes: &'a FxHashMap<LibCellId, (i64, i64)>,
    pub blockages: &'a [Rect<Dbu>],
}

/// The tree of one clock net.
#[derive(Clone, Debug)]
pub(crate) struct Builder {
    pub clock: Clock,
    pub tree_type: TreeType,
    pub parent: Option<usize>,
    pub children: Vec<usize>,
    pub tree_buf_levels: u32,
    pub first_level_sink_drivers: FxHashSet<InstRef>,
    pub tree_level_buffers: FxHashSet<InstRef>,
    pub top_buffer_name: String,
    pub driving_net: Option<pare_common::db::indices::NetId>,
    pub top_input_net: Option<pare_common::db::indices::NetId>,
    pub n_dummies: usize,
    occupied: FxHashSet<(u64, u64)>,
    insertion_delays: FxHashMap<(u64, u64), f64>,
    sink_region: Region,
    topologies: Vec<LevelTopology>,
    location_to_sink: FxHashMap<(u64, u64), InstRef>,
    top_level_sinks_clustered: Vec<(f32, f32)>,
    wire_segment_unit: i32,
    min_input_cap: u32,
    min_length_sink_region: u32,
}

impl Builder {
    pub fn new(clock: Clock, parent: Option<usize>) -> Self {
        Self {
            clock,
            tree_type: TreeType::Regular,
            parent,
            children: Vec::new(),
            tree_buf_levels: 0,
            first_level_sink_drivers: FxHashSet::default(),
            tree_level_buffers: FxHashSet::default(),
            top_buffer_name: String::new(),
            driving_net: None,
            top_input_net: None,
            n_dummies: 0,
            occupied: FxHashSet::default(),
            insertion_delays: FxHashMap::default(),
            sink_region: Region::default(),
            topologies: Vec::new(),
            location_to_sink: FxHashMap::default(),
            top_level_sinks_clustered: Vec::new(),
            wire_segment_unit: 0,
            min_input_cap: 0,
            min_length_sink_region: 0,
        }
    }

    /// Whether the tree drives no other tree, as `TreeBuilder::isLeafTree`.
    pub fn is_leaf_tree(&self) -> bool {
        if self.tree_type == TreeType::Macro {
            return self.children.len() == 1;
        }
        self.children.is_empty()
    }

    fn insertion_delay(&self, p: Point) -> f64 {
        self.insertion_delays.get(&p.key()).copied().unwrap_or(0.0)
    }

    /// The distance between two points plus the insertion delays at them,
    /// as `TreeBuilder::computeDist`.
    pub fn compute_dist(&self, a: Point, b: Point) -> f64 {
        a.dist(b) + self.insertion_delay(a) + self.insertion_delay(b)
    }

    fn is_occupied(&self, p: Point) -> bool {
        self.occupied.contains(&p.key())
    }

    fn commit(&mut self, p: Point) {
        let _ = self.occupied.insert(p.key());
    }

    fn commit_move(&mut self, from: Point, to: Point) {
        let _ = self.occupied.remove(&from.key());
        let _ = self.occupied.insert(to.key());
    }

    pub fn unit(&self) -> f64 {
        f64::from(self.wire_segment_unit)
    }

    /// The blockage `p` lies strictly inside, scaled to the tree's unit,
    /// as `TreeBuilder::findBlockage`.
    fn find_blockage(ctx: &TreeContext<'_>, p: Point, scale: f64) -> Option<[f64; 4]> {
        let (bx, by) = (p.x * scale, p.y * scale);
        ctx.blockages.iter().find_map(|b| {
            let [x1, y1, x2, y2] = [b.min.x, b.min.y, b.max.x, b.max.y].map(|v| v.get() as f64);
            (bx > x1 && bx < x2 && by > y1 && by < y2)
                .then(|| [x1 / scale, y1 / scale, x2 / scale, y2 / scale])
        })
    }

    /// Whether a buffer may stand at `p`: free, in the core and outside
    /// every blockage, as `TreeBuilder::checkLegalityLoc`.
    fn legal_location(&self, ctx: &TreeContext<'_>, p: Point, scale: f64) -> bool {
        if self.is_occupied(p) {
            return false;
        }
        let x = (p.x * scale) as i32;
        let y = (p.y * scale) as i32;
        let core = ctx.core;
        let inside = core.min.x.get() <= i64::from(x)
            && i64::from(x) <= core.max.x.get()
            && core.min.y.get() <= i64::from(y)
            && i64::from(y) <= core.max.y.get();
        inside && Self::find_blockage(ctx, p, scale).is_none()
    }

    fn buffer_size(&self, ctx: &TreeContext<'_>, buffer: LibCellId) -> (f64, f64) {
        let (w, h) = ctx.buffer_sizes.get(&buffer).copied().unwrap_or((0, 0));
        (w as f64 / self.unit(), h as f64 / self.unit())
    }

    /// Moves a buffer inside a blockage to the free spot beside it nearest
    /// to where it was, as `TreeBuilder::legalizeOneBuffer`.
    fn legalize_in_blockages(&self, ctx: &TreeContext<'_>, p: Point, buffer: LibCellId) -> Point {
        if !ctx.settings.obstruction_aware {
            return p;
        }
        let Some([x1, y1, x2, y2]) = Self::find_blockage(ctx, p, self.unit()) else {
            return p;
        };
        let (w, h) = self.buffer_size(ctx, buffer);
        let candidates = [
            Point::new(x1 - w, p.y),
            Point::new(x1 - w, p.y + h),
            Point::new(x1 - w, p.y - h),
            Point::new(x2, p.y),
            Point::new(x2, p.y + h),
            Point::new(x2, p.y - h),
            Point::new(p.x, y1 - h),
            Point::new(p.x + w, y1 - h),
            Point::new(p.x - w, y1 - h),
            Point::new(p.x, y2),
            Point::new(p.x + w, y2),
            Point::new(p.x - w, y2),
        ];
        let mut best: Option<(Point, f64)> = None;
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
    pub fn legalize_one_buffer(&self, ctx: &TreeContext<'_>, p: Point, buffer: LibCellId) -> Point {
        let legal = self.legalize_in_blockages(ctx, p, buffer);
        let mut resolved = legal;
        let mut jitter = 0;
        while self.location_to_sink.contains_key(&resolved.key_f32()) {
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
        for level in 1..=ctx.settings.clock_tree_max_depth {
            let per_region = self.sinks_per_sub_region(ctx, level);
            let (width, height) = self.sub_region_size(level);
            let min_length = f64::from(self.min_length_sink_region);
            if width < min_length || height < min_length {
                if ctx.settings.fake_lut_entries {
                    tech.create_fake_entries(self.min_length_sink_region, 1);
                    self.min_length_sink_region = 1;
                } else {
                    break;
                }
            }
            self.level_topology(ctx, tech, level, width, height);
            let limit = if ctx.settings.max_fanout > 0 {
                ctx.settings.max_fanout
            } else {
                ctx.settings.num_max_leaf_sinks
            };
            if per_region < limit {
                break;
            }
        }
        if self.topologies.is_empty() {
            self.single_buffer_clock_net(ctx);
            self.tree_buf_levels += 1;
            return;
        }
        self.clock.max_level = self.topologies.len();
        if ctx.settings.obstruction_aware {
            self.legalize(ctx);
        }
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
            && ctx.settings.sink_clustering
        {
            self.top_level_sinks_clustered.len()
        } else {
            self.clock.sinks.len()
        };
        let roots = 2_f64.powi(level as i32) as u32;
        (total as f64 / f64::from(roots)).ceil() as u32
    }

    /// Clusters the sinks and finds the region the tree spans, as
    /// `HTreeBuilder::initSinkRegion`.
    fn init_sink_region(&mut self, ctx: &TreeContext<'_>, tech: &TechChar) {
        self.wire_segment_unit = tech.length_unit() as i32;
        let (diameter, size) = if self.tree_type == TreeType::Macro {
            (
                ctx.settings.macro_max_diameter,
                ctx.settings.macro_sink_clustering_size,
            )
        } else {
            (ctx.settings.max_diameter, ctx.settings.sink_clustering_size)
        };
        let unit = self.wire_segment_unit as f32;
        let sinks: Vec<(f32, f32)> = self
            .clock
            .sinks
            .iter()
            .map(|s| (s.location.0 as f32 / unit, s.location.1 as f32 / unit))
            .collect();
        let max_diameter = (diameter * f64::from(ctx.dbu) / self.unit()) as f32;
        self.pre_sink_clustering(ctx, tech, &sinks, max_diameter, size);
        if sinks.len() <= self.min_clustering_sinks() || !ctx.settings.sink_clustering {
            let (lo, hi) = self.clock.sink_region();
            let f = 1.0 / self.unit();
            self.sink_region = Region::new(
                lo.0 as f64 * f,
                lo.1 as f64 * f,
                hi.0 as f64 * f,
                hi.1 as f64 * f,
            );
        } else {
            let (mut x_min, mut x_max) = (f32::MAX, f32::MIN);
            let (mut y_min, mut y_max) = (f32::MAX, f32::MIN);
            for &(x, y) in &self.top_level_sinks_clustered {
                x_min = x_min.min(x);
                x_max = x_max.max(x);
                y_min = y_min.min(y);
                y_max = y_max.max(y);
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
        sinks: &[(f32, f32)],
        max_diameter: f32,
        size: u32,
    ) {
        let unit = self.wire_segment_unit as f32;
        for i in 0..self.clock.sinks.len() {
            let s = &self.clock.sinks[i];
            let location = Point::new(
                f64::from(s.location.0 as f32 / unit),
                f64::from(s.location.1 as f32 / unit),
            );
            let delay = s.insertion_delay;
            let _ = self
                .location_to_sink
                .insert(location.key(), InstRef::Sink(i));
            if !fuzzy_eq(delay, 0.0, 1e-6) {
                let _ = self
                    .insertion_delays
                    .insert(location.key(), delay / self.unit());
            }
        }
        if sinks.len() <= self.min_clustering_sinks() || !ctx.settings.sink_clustering {
            self.top_level_sinks_clustered = sinks.to_vec();
            return;
        }
        let is_macro = self.tree_type == TreeType::Macro;
        let use_max_cap = !is_macro && ctx.settings.sink_clustering_use_max_cap;
        let mut matching = SinkClustering::new(
            use_max_cap,
            tech.cap_per_dbu(),
            f64::from(tech.sink_buffer_input_cap()),
        );
        for (i, &(x, y)) in sinks.iter().enumerate() {
            let cap = self.clock.sinks[i].input_cap;
            let cap = if cap == 0.0 {
                tech.sink_buffer_input_cap()
            } else {
                cap
            };
            matching.add_point(f64::from(x), f64::from(y), cap);
        }
        let (diameter_set, size_set) = if is_macro {
            (ctx.settings.macro_max_diameter_set, true)
        } else {
            (false, false)
        };
        let unit_i = self.wire_segment_unit;
        let dbu = ctx.dbu as f32;
        let dist = |a: Point, b: Point| self.compute_dist(a, b);
        match (size_set, diameter_set) {
            (true, true) => matching.run(size as usize, max_diameter, unit_i, &dist),
            (false, true) => {
                for &s in &ctx.settings.sink_clustering_sizes {
                    matching.run(s as usize, max_diameter, unit_i, &dist);
                }
            }
            (true, false) => {
                for &d in &ctx.settings.sink_clustering_diameters {
                    let diameter = d as f32 * dbu / unit;
                    matching.run(size as usize, diameter, unit_i, &dist);
                }
            }
            (false, false) => {
                for d in CLUSTER_DIAMETERS {
                    for &s in &ctx.settings.sink_clustering_sizes {
                        let diameter = d as f32 * dbu / unit;
                        matching.run(s as usize, diameter, unit_i, &dist);
                    }
                }
            }
        }
        let solution = matching.solution().to_vec();
        let mut new_locations = Vec::with_capacity(solution.len());
        let mut count = 0;
        for cluster in &solution {
            if cluster.len() == 1 {
                new_locations.push(sinks[cluster[0]]);
            }
            if cluster.len() > 1 {
                let mut members = Vec::with_capacity(cluster.len());
                let (mut x_sum, mut y_sum) = (0.0_f32, 0.0_f32);
                for &idx in cluster {
                    let point = Point::from_f32(sinks[idx]);
                    x_sum += sinks[idx].0;
                    y_sum += sinks[idx].1;
                    if let Some(&inst) = self.location_to_sink.get(&point.key()) {
                        members.push(inst);
                    }
                }
                let n = cluster.len() as f32;
                let center = Point::new(f64::from(x_sum / n), f64::from(y_sum / n));
                let legal = self.legalize_one_buffer(ctx, center, ctx.sink_buffer);
                self.commit_move(center, legal);
                let location = (
                    (legal.x * self.unit()) as i32 as i64,
                    (legal.y * self.unit()) as i32 as i64,
                );
                let buffer = self.clock.add_buffer(
                    &format!("clkbuf_leaf_{count}"),
                    ctx.sink_buffer,
                    location,
                );
                let _ = self.first_level_sink_drivers.insert(buffer);
                let subnet = self.clock.add_subnet(&format!("clknet_leaf_{count}"));
                self.clock.subnets[subnet].add(buffer);
                for member in members {
                    self.clock.subnets[subnet].add(member);
                }
                self.clock.subnets[subnet].leaf = true;
                let point = (legal.x as f32, legal.y as f32);
                new_locations.push(point);
                let _ = self
                    .location_to_sink
                    .insert(Point::from_f32(point).key(), buffer);
            }
            count += 1;
        }
        self.top_level_sinks_clustered = new_locations;
        if count > 0 {
            self.tree_buf_levels += 1;
        }
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
        let min_length = self.min_length_sink_region;
        let clamped = f64::from(min_length.max(1));
        let side = if self.is_vertical(level) {
            height
        } else {
            width
        };
        let segment_length =
            (((side / clamped).round() * f64::from(min_length) / 2.0) as u32).max(1);
        let mut topology = LevelTopology::new(f64::from(segment_length));
        let (mut input_cap, mut input_slew, mut curr_wl) = (self.min_input_cap, 1, 0);
        if level > 1 {
            let previous = &self.topologies[level as usize - 2];
            input_cap = previous.output_cap;
            input_slew = previous.output_slew;
            curr_wl = previous.curr_wl;
        }
        let slew_threshold = ctx.settings.max_slew;
        let wirelength_threshold = if ctx.settings.max_wire_length == 0 {
            (slew_threshold * tech.max_segment_length()) as i32
        } else {
            (ctx.settings.max_wire_length as u32 / tech.char_unit()) as i32
        };
        let mut length = 0;
        let mut char_length = tech.max_segment_length();
        while char_length >= 1 {
            let wires = (segment_length - length) / char_length;
            if wires >= 1 {
                for _ in 0..wires {
                    let mut out = SegmentChoice {
                        slew: 0,
                        cap: 0,
                        curr_wl,
                    };
                    let Some(key) = min_delay_segment(
                        tech,
                        &SegmentQuery {
                            length: char_length,
                            input_slew,
                            input_cap,
                            slew_threshold,
                            wirelength_threshold,
                        },
                        1,
                        &mut out,
                    ) else {
                        continue;
                    };
                    curr_wl = out.curr_wl;
                    length += char_length;
                    input_cap = out.cap.max(self.min_input_cap);
                    input_slew = out.slew;
                    topology.wire_segments.push(key);
                }
                if length == segment_length {
                    break;
                }
            }
            char_length -= 1;
        }
        topology.output_slew = input_slew;
        topology.output_cap = input_cap;
        topology.curr_wl = curr_wl;
        self.branching_points(ctx, level, &mut topology);
        self.topologies.push(topology);
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
        let spread = |root: Point, length: f64| -> (Point, Point) {
            if horizontal {
                (
                    Point::new(root.x - length, root.y),
                    Point::new(root.x + length, root.y),
                )
            } else {
                (
                    Point::new(root.x, root.y - length),
                    Point::new(root.x, root.y + length),
                )
            }
        };
        if level == 1 {
            let root = self.sink_region.center();
            let (low, high) = spread(root, topology.length);
            let a = topology.add_point(low, None);
            let b = topology.add_point(high, None);
            let sinks = self.top_level_sinks_clustered.clone();
            refine_with_clustering(ctx, topology, level, a, b, root, &sinks);
            return;
        }
        let parent = &self.topologies[level as usize - 2];
        let roots: Vec<BranchRoot> = parent
            .points
            .iter()
            .enumerate()
            .map(|(idx, &p)| {
                let sinks = parent.sinks[idx]
                    .iter()
                    .map(|s| (s.x as f32, s.y as f32))
                    .collect();
                (idx, p, sinks)
            })
            .collect();
        for (idx, root, sinks) in roots {
            let (low, high) = spread(root, topology.length);
            let a = topology.add_point(low, Some(idx));
            let b = topology.add_point(high, Some(idx));
            refine_with_clustering(ctx, topology, level, a, b, root, &sinks);
        }
    }

    fn parent_point(&self, level_idx: usize, parent: Option<usize>, top: Point) -> Point {
        match (level_idx, parent) {
            (0, _) | (_, None) => top,
            (_, Some(p)) => self.topologies[level_idx - 1].points[p],
        }
    }

    fn weighted_distance(&self, new: Point, old: Point, sinks: &[Point]) -> f64 {
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
        let unit = self.unit();
        for level_idx in 0..self.topologies.len() {
            for idx in 0..self.topologies[level_idx].points.len() {
                let point = self.topologies[level_idx].points[idx];
                let parent_idx = self.topologies[level_idx].parents[idx];
                let parent = self.parent_point(level_idx, parent_idx, new_top);
                let direction = if self.is_horizontal(level_idx as u32 + 1) {
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
                let sinks = self.topologies[level_idx].sinks[idx].clone();
                let length = self.topologies[level_idx].length;
                let leng = point.dist(parent);
                let blockage = if self.is_occupied(point) {
                    None
                } else {
                    Self::find_blockage(ctx, point, unit)
                };
                let moved = if let Some(rect) = blockage {
                    let candidates = Self::legal_locations(parent, point, rect);
                    let best = self.best_legal_location(
                        ctx,
                        length,
                        point,
                        parent,
                        &candidates,
                        &sinks,
                        rect,
                        direction,
                    );
                    self.commit_move(point, best);
                    best
                } else if self.is_occupied(point) || !fuzzy_eq(leng, length, 0.01) {
                    let best =
                        self.adjust_beyond_blockage(ctx, point, parent, length, &sinks, direction);
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
        let unit = self.unit();
        for level_idx in 0..self.topologies.len() {
            for idx in 0..self.topologies[level_idx].points.len() {
                let parent_idx = self.topologies[level_idx].parents[idx];
                let parent = self.parent_point(level_idx, parent_idx, top);
                let topology = &self.topologies[level_idx];
                let sinks = topology.sinks[idx].clone();
                let length = topology.length;
                let sibling = topology.points[topology.sibling(idx, parent_idx)];
                let mut point = topology.points[idx];
                let overlap = point.dist(sibling) == 0.0 || point.dist(parent) == 0.0;
                if sinks.is_empty() {
                    point = Point::new(2.0 * parent.x - sibling.x, 2.0 * parent.y - sibling.y);
                    point = scale_position(point, parent, length, 0.1);
                } else if overlap {
                    point = scale_position(point, parent, length, 0.1);
                } else {
                    continue;
                }
                if !self.is_occupied(point)
                    && let Some([x1, y1, x2, y2]) = Self::find_blockage(ctx, point, unit)
                {
                    let candidates = Self::legal_locations(parent, point, [x1, y1, x2, y2]);
                    let best = self.best_legal_location(
                        ctx,
                        length,
                        point,
                        parent,
                        &candidates,
                        &sinks,
                        [x1, y2, x2, y2],
                        Direction::North,
                    );
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
    fn legal_locations(parent: Point, branch: Point, rect: [f64; 4]) -> Vec<Point> {
        let [x1, y1, x2, y2] = rect;
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
            .map(|&(x, y)| Point::new(x, y))
            .filter(|&c| !c.same(parent) && along_box(c, rect))
            .collect()
    }

    #[allow(clippy::too_many_arguments)]
    fn best_legal_location(
        &self,
        ctx: &TreeContext<'_>,
        target: f64,
        branch: Point,
        parent: Point,
        candidates: &[Point],
        sinks: &[Point],
        rect: [f64; 4],
        direction: Direction,
    ) -> Point {
        let mut best = Point::new(0.0, 0.0);
        let mut min_diff = f64::MAX;
        for &loc in candidates {
            let d = self.compute_dist(loc, parent) + self.compute_dist(loc, branch);
            let diff = (d - target).abs();
            if diff < min_diff {
                min_diff = diff;
                best = loc;
            }
        }
        if fuzzy_eq(target, self.compute_dist(best, parent), 1e-6) {
            return best;
        }
        if let Some(along) = self.adjust_along_blockage(ctx, target, best, parent, sinks, rect) {
            return along;
        }
        self.adjust_beyond_blockage(ctx, best, parent, target, sinks, direction)
    }

    /// The point at `target` from the parent along the blockage's edge
    /// nearest the sinks, as `HTreeBuilder::adjustAlongBlockage`.
    fn adjust_along_blockage(
        &self,
        ctx: &TreeContext<'_>,
        target: f64,
        current: Point,
        parent: Point,
        sinks: &[Point],
        rect: [f64; 4],
    ) -> Option<Point> {
        let (x, y) = (current.x, current.y);
        let (px, py) = (parent.x, parent.y);
        let candidates = [
            Point::new(x, py - target + (px - x).abs()),
            Point::new(x, py + target - (px - x).abs()),
            Point::new(px - target + (py - y).abs(), y),
            Point::new(px + target - (py - y).abs(), y),
        ];
        let mut best = None;
        let mut best_cost = f64::MAX;
        for c in candidates {
            if !fuzzy_eq(self.compute_dist(c, parent), target, 1e-6) {
                continue;
            }
            let legal = (!self.is_occupied(c) && along_box(c, rect))
                || self.legal_location(ctx, c, self.unit());
            if !legal {
                continue;
            }
            let cost = self.weighted_distance(c, current, sinks);
            if cost < best_cost {
                best = Some(c);
                best_cost = cost;
            }
        }
        best.filter(|b| !b.same(Point::new(f64::MIN_POSITIVE, f64::MIN_POSITIVE)))
    }

    /// The legal point at `target` from the parent, toward `direction`,
    /// nearest the sinks, as `HTreeBuilder::adjustBeyondBlockage`.
    fn adjust_beyond_blockage(
        &self,
        ctx: &TreeContext<'_>,
        branch: Point,
        parent: Point,
        target: f64,
        sinks: &[Point],
        direction: Direction,
    ) -> Point {
        let (px, py) = (parent.x, parent.y);
        let l50 = target * 0.5;
        let l25 = target * 0.25;
        let l75 = target * 0.75;
        let mut candidates: Vec<Point> = match direction {
            Direction::North => vec![
                Point::new(px, py + target),
                Point::new(px + l50, py + l50),
                Point::new(px - l50, py + l50),
                Point::new(px + l25, py + l75),
                Point::new(px - l25, py + l75),
            ],
            Direction::East => vec![
                Point::new(px + target, py),
                Point::new(px + l50, py + l50),
                Point::new(px + l50, py - l50),
                Point::new(px + l75, py + l25),
                Point::new(px + l75, py - l25),
            ],
            Direction::South => vec![
                Point::new(px, py - target),
                Point::new(px + l50, py - l50),
                Point::new(px - l50, py - l50),
                Point::new(px + l25, py - l75),
                Point::new(px - l25, py - l75),
            ],
            Direction::West => vec![
                Point::new(px - target, py),
                Point::new(px - l50, py - l50),
                Point::new(px - l50, py + l50),
                Point::new(px - l75, py - l25),
                Point::new(px - l75, py + l25),
            ],
        };
        let straight = match direction {
            Direction::North => Point::new(px, py + target),
            Direction::East => Point::new(px + target, py),
            Direction::South => Point::new(px, py - target),
            Direction::West => Point::new(px - target, py),
        };
        if let Some([x1, y1, x2, y2]) = Self::find_blockage(ctx, straight, self.unit()) {
            let along = match direction {
                Direction::North => [
                    Point::new(-target - py + px + y1, y1),
                    Point::new(-target - py + px + y1, target + py + px - y1),
                ],
                Direction::East => [
                    Point::new(x1, target + py + px - x1),
                    Point::new(x1, -target + py - px + x1),
                ],
                Direction::South => [
                    Point::new(-target + py + px - y2, y2),
                    Point::new(target - py + px + y2, y2),
                ],
                Direction::West => [
                    Point::new(x2, target + py - px + x2),
                    Point::new(x2, -target + py + px - x2),
                ],
            };
            candidates.extend(along);
        }
        let (bx, by) = (branch.x, branch.y);
        let (w, h) = self.buffer_size(ctx, ctx.root_buffer);
        let (min_x, min_y) = (10.0 * w, 10.0 * h);
        let x_plus = bx + min_x;
        let x_minus = bx - min_x;
        let y_plus = by + min_y;
        let y_minus = by - min_y;
        match direction {
            Direction::South => {
                candidates.push(Point::new(x_plus, py - target + (px - x_plus).abs()));
                candidates.push(Point::new(x_minus, py - target + (px - x_minus).abs()));
            }
            Direction::North => {
                candidates.push(Point::new(x_plus, py + target - (px - x_plus).abs()));
                candidates.push(Point::new(x_minus, py + target - (px - x_minus).abs()));
            }
            Direction::West => {
                candidates.push(Point::new(px - target + (py - y_plus).abs(), y_plus));
                candidates.push(Point::new(px - target + (py - y_minus).abs(), y_minus));
            }
            Direction::East => {
                candidates.push(Point::new(px + target - (py - y_plus).abs(), y_plus));
                candidates.push(Point::new(px + target - (py - y_minus).abs(), y_minus));
            }
        }
        let mut best = None;
        let mut best_cost = f64::MAX;
        for c in candidates {
            if fuzzy_eq(self.compute_dist(c, parent), target, 1e-6)
                && self.legal_location(ctx, c, self.unit())
            {
                let cost = self.weighted_distance(c, branch, sinks);
                if cost < best_cost {
                    best = Some(c);
                    best_cost = cost;
                }
            }
        }
        best.filter(|b| !b.same(Point::new(f64::MIN_POSITIVE, f64::MIN_POSITIVE)))
            .unwrap_or(branch)
    }

    /// Writes the buffers and subnets of the tree, as
    /// `HTreeBuilder::createClockSubNets`.
    fn create_clock_subnets(&mut self, ctx: &TreeContext<'_>, tech: &TechChar) {
        let center = self.sink_region.center();
        let legal_center = self.legalize_one_buffer(ctx, center, ctx.root_buffer);
        self.sink_region.center = Some(legal_center);
        self.commit_move(center, legal_center);
        let unit = self.unit();
        let to_dbu = |p: Point| ((p.x * unit) as i32 as i64, (p.y * unit) as i32 as i64);
        let root = self
            .clock
            .add_buffer("clkbuf_0", ctx.root_buffer, to_dbu(legal_center));
        if self.top_buffer_name.is_empty() {
            self.top_buffer_name = self.clock.inst(root).name.clone();
        }
        let _ = self.tree_level_buffers.insert(root);
        let root_net = self.clock.add_subnet("clknet_0");
        self.clock.subnets[root_net].level = Some(0);
        self.clock.subnets[root_net].add(root);
        self.tree_buf_levels += 1;
        let levels = self.topologies.len();
        for level_idx in 0..levels {
            let mut first = true;
            for idx in 0..self.topologies[level_idx].points.len() {
                if self.topologies[level_idx].sinks[idx].is_empty() {
                    continue;
                }
                let point = self.topologies[level_idx].points[idx];
                let (from, driving) = if level_idx == 0 {
                    (legal_center, root_net)
                } else {
                    let parent_idx = self.topologies[level_idx].parents[idx].unwrap_or(0);
                    let parent = &self.topologies[level_idx - 1];
                    (
                        parent.points[parent_idx],
                        parent.driving_subnet[parent_idx].unwrap_or(root_net),
                    )
                };
                let legal = self.legalize_one_buffer(ctx, point, ctx.root_buffer);
                self.commit_move(point, legal);
                if level_idx > 0 && self.clock.subnets[driving].level.is_none() {
                    self.clock.subnets[driving].level = Some(level_idx as u32);
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
                if level_idx == levels - 1 {
                    segment.force_buffer(self, ctx.root_buffer);
                }
                if first {
                    self.tree_buf_levels += segment.levels;
                    first = false;
                }
                self.topologies[level_idx].driving_subnet[idx] = Some(segment.driving);
            }
        }
        let leaf = levels - 1;
        for idx in 0..self.topologies[leaf].points.len() {
            let Some(subnet) = self.topologies[leaf].driving_subnet[idx] else {
                continue;
            };
            self.clock.subnets[subnet].leaf = true;
            for loc in self.topologies[leaf].sinks[idx].clone() {
                if let Some(&inst) = self.location_to_sink.get(&loc.key()) {
                    self.clock.subnets[subnet].add(inst);
                }
            }
        }
    }

    /// One buffer driving every sink, as
    /// `HTreeBuilder::createSingleBufferClockNet`.
    fn single_buffer_clock_net(&mut self, ctx: &TreeContext<'_>) {
        let center = self.sink_region.center();
        let legal = self.legalize_one_buffer(ctx, center, ctx.root_buffer);
        self.sink_region.center = Some(legal);
        self.commit_move(center, legal);
        let unit = self.unit();
        let location = (
            (legal.x * unit) as i32 as i64,
            (legal.y * unit) as i32 as i64,
        );
        let root = self.clock.add_buffer("clkbuf_0", ctx.root_buffer, location);
        if self.top_buffer_name.is_empty() {
            self.top_buffer_name = self.clock.inst(root).name.clone();
        }
        let _ = self.tree_level_buffers.insert(root);
        let subnet = self.clock.add_subnet("clknet_0");
        self.clock.subnets[subnet].level = Some(0);
        self.clock.subnets[subnet].add(root);
        for i in 0..self.clock.sinks.len() {
            self.clock.subnets[subnet].add(InstRef::Sink(i));
        }
    }
}

/// Whether `p` lies on the boundary of `rect`, as `TreeBuilder::isAlongBbox`.
fn along_box(p: Point, rect: [f64; 4]) -> bool {
    let [x1, y1, x2, y2] = rect;
    let on_vertical = (fuzzy_eq(p.x, x1, 1e-6) || fuzzy_eq(p.x, x2, 1e-6))
        && fuzzy_ge(p.y, y1)
        && fuzzy_le(p.y, y2);
    let on_horizontal = (fuzzy_eq(p.y, y1, 1e-6) || fuzzy_eq(p.y, y2, 1e-6))
        && fuzzy_ge(p.x, x1)
        && fuzzy_le(p.x, x2);
    on_vertical || on_horizontal
}

/// Pulls a point toward its parent until it is `length` away, after
/// pushing it a `scale` share off the axes, as
/// `HTreeBuilder::scalePosition`.
fn scale_position(p: Point, parent: Point, length: f64, scale: f64) -> Point {
    let d = p.dist(parent);
    if d > 0.0 {
        let delta = d * scale;
        let mut dx = p.x - parent.x;
        let mut dy = p.y - parent.y;
        dx += if dx > 0.0 { delta } else { -delta };
        dy += if dy > 0.0 { -delta } else { delta };
        let s = length / d;
        Point::new(parent.x + dx * s, parent.y + dy * s)
    } else {
        Point::new(parent.x + length / 2.0, parent.y + length / 2.0)
    }
}

/// Splits sinks between two branching points by capacitated k-means and
/// moves the points to the means, as
/// `HTreeBuilder::refineBranchingPointsWithClustering`.
fn refine_with_clustering(
    ctx: &TreeContext<'_>,
    topology: &mut LevelTopology,
    level: u32,
    a: usize,
    b: usize,
    root: Point,
    sinks: &[(f32, f32)],
) {
    let mut engine = Clustering::new(sinks, (root.x as f32, root.y as f32));
    let pa = topology.points[a];
    let pb = topology.points[b];
    let mut means = [(pa.x as f32, pa.y as f32), (pb.x as f32, pb.y as f32)];
    let cap = if ctx.settings.max_fanout > 0 {
        (sinks.len() as f64 * 0.5) as usize
    } else {
        (sinks.len() as f64 * ctx.settings.clustering_capacity) as usize
    };
    engine.run(cap, 5, ctx.settings.clustering_power, &mut means);
    if (ctx.settings.num_static_layers as i64) - i64::from(level) < 0 {
        topology.points[a] = Point::from_f32(means[0]);
        topology.points[b] = Point::from_f32(means[1]);
    }
    for (cluster, members) in engine.clusters().iter().enumerate() {
        let branch = if cluster == 0 { a } else { b };
        for &s in members {
            topology.sinks[branch].push(Point::from_f32(sinks[s]));
        }
    }
}

/// What a segment search starts from.
#[derive(Clone, Copy, Debug)]
struct SegmentQuery {
    length: u32,
    input_slew: u32,
    input_cap: u32,
    slew_threshold: u32,
    wirelength_threshold: i32,
}

/// The slew, load and wire since the last buffer a chosen segment leaves.
#[derive(Clone, Copy, Debug)]
struct SegmentChoice {
    slew: u32,
    cap: u32,
    curr_wl: i32,
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
    let mut min_key = None;
    let mut min_delay = u32::MAX;
    let mut min_buf_key = None;
    let mut min_buf_delay = u32::MAX;
    for load in 1..=tech.max_capacitance() {
        for out_slew in 1..=tech.max_slew() {
            for &key in tech.segments_with(query.length, load, out_slew) {
                let seg = tech.segment(key);
                if (i64::from(seg.input_cap) - i64::from(query.input_cap)).abs()
                    > i64::from(tolerance)
                    || (i64::from(seg.input_slew) - i64::from(query.input_slew)).abs()
                        > i64::from(tolerance)
                {
                    continue;
                }
                let wl = out.curr_wl + seg.wl_to_first_buffer;
                let too_long = if seg.is_buffered() {
                    wl > query.wirelength_threshold
                } else {
                    // The threshold less the shortest length is unsigned in `TritonCTS`.
                    let limit =
                        (query.wirelength_threshold as u32).wrapping_sub(tech.min_segment_length());
                    (wl as u32) > limit
                };
                if too_long {
                    continue;
                }
                if seg.delay < min_delay {
                    min_delay = seg.delay;
                    min_key = Some(key);
                }
                if seg.is_buffered() && seg.delay < min_buf_delay {
                    min_buf_delay = seg.delay;
                    min_buf_key = Some(key);
                }
            }
        }
    }
    if query.input_slew >= query.slew_threshold {
        if let Some(key) = min_buf_key {
            let seg = tech.segment(key);
            out.slew = u32::from(seg.output_slew);
            out.cap = u32::from(seg.load);
            out.curr_wl = seg.last_wl;
            return Some(key);
        }
        if tolerance < MAX_TOLERANCE {
            return min_delay_segment(tech, query, tolerance + 1, out);
        }
    }
    let Some(key) = min_key else {
        if tolerance >= MAX_TOLERANCE {
            return None;
        }
        return min_delay_segment(tech, query, tolerance + 1, out);
    };
    let seg = tech.segment(key);
    if seg.is_buffered() {
        out.slew = u32::from(seg.output_slew);
        out.curr_wl = seg.last_wl;
    } else {
        out.slew = u32::from(seg.output_slew).max(query.input_slew + 1);
        out.curr_wl += seg.last_wl;
    }
    out.cap = u32::from(seg.load);
    Some(key)
}

/// The buffers along one branch, from its parent's point to its own, as
/// `TritonCTS`'s `SegmentBuilder`.
struct SegmentBuilder {
    inst_prefix: String,
    net_prefix: String,
    root: Point,
    target: Point,
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
            for (b, &location) in segment.buffer_locations.iter().enumerate() {
                connection += location * length;
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
                let master = segment
                    .buffer_masters
                    .get(b)
                    .copied()
                    .unwrap_or(ctx.root_buffer);
                let at = Point::new(x, y);
                let legal = tree.legalize_one_buffer(ctx, at, master);
                tree.commit_move(at, legal);
                let buffer = tree.clock.add_buffer(
                    &format!("{}{}", self.inst_prefix, self.levels),
                    master,
                    (
                        (legal.x * unit) as i32 as i64,
                        (legal.y * unit) as i32 as i64,
                    ),
                );
                if !at.same(legal) {
                    connection -= tree.compute_dist(at, legal);
                }
                let _ = tree.tree_level_buffers.insert(buffer);
                tree.clock.subnets[self.driving].add(buffer);
                self.driving = tree
                    .clock
                    .add_subnet(&format!("{}{}", self.net_prefix, self.levels));
                tree.clock.subnets[self.driving].add(buffer);
                self.levels += 1;
            }
            connection += length;
        }
    }

    /// Puts a buffer at the branch's end when the segments placed none, as
    /// `SegmentBuilder::forceBufferInSegment`.
    fn force_buffer(&mut self, tree: &mut Builder, master: LibCellId) {
        if self.levels != 0 {
            return;
        }
        let unit = tree.unit();
        let buffer = tree.clock.add_buffer(
            &format!("{}_f", self.inst_prefix),
            master,
            (
                (self.target.x * unit) as i32 as i64,
                (self.target.y * unit) as i32 as i64,
            ),
        );
        let _ = tree.tree_level_buffers.insert(buffer);
        tree.clock.subnets[self.driving].add(buffer);
        self.driving = tree.clock.add_subnet(&format!("{}_leaf", self.net_prefix));
        tree.clock.subnets[self.driving].add(buffer);
        self.levels += 1;
    }
}
