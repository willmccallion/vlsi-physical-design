//! What clock tree synthesis is asked to do, and the settings `TritonCTS`
//! fixes or derives before building the trees.

use pare_common::convert::NearestFrom;
use pare_common::geom::dbu::Dbu;

/// Which levels of each clock tree get the clock's non-default rule of
/// doubled spacing, as `clock_tree_synthesis -apply_ndr`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NdrStrategy {
    /// No net.
    None,
    /// The root net only.
    RootOnly,
    /// The nets of the upper half of the levels, `TritonCTS`'s default.
    #[default]
    Half,
    /// Every net but the leaves.
    Full,
}

/// Whether sinks are clustered under leaf buffers before the H-tree is
/// built (`-sink_clustering_enable`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SinkClusteringMode {
    /// Sinks are clustered, `OpenROAD-flow-scripts`' setting.
    #[default]
    Clustered,
    /// Every sink is a leaf of the tree itself.
    Unclustered,
}

/// Whether dummy cells balance the loads of a tree's buffers (no
/// `-dont_use_dummy_load`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DummyLoads {
    /// Dummy cells are added, `TritonCTS`'s default.
    #[default]
    Balance,
    /// No dummy cells are added.
    Omit,
}

/// Whether sinks with a Liberty insertion delay get trees of their own (no
/// `-no_insertion_delay`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum InsertionDelays {
    /// Such sinks get their own trees, `TritonCTS`'s default.
    #[default]
    SeparateTrees,
    /// Insertion delays are ignored.
    Ignored,
}

/// The requests `clock_tree_synthesis` takes; the defaults are what
/// `OpenROAD-flow-scripts` runs it with.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CtsOptions {
    /// Whether sinks are clustered under leaf buffers.
    pub sink_clustering: SinkClusteringMode,
    /// Whether dummy cells balance the buffers' loads.
    pub dummy_loads: DummyLoads,
    /// Whether sinks with insertion delay get trees of their own.
    pub insertion_delays: InsertionDelays,
    /// The non-default rule's levels (`-apply_ndr`).
    pub ndr: NdrStrategy,
}

/// The most sinks a leaf region holds without a fanout limit, as
/// `CtsOptions`' default `numMaxLeafSinks`.
pub(crate) const MAX_LEAF_SINKS: u32 = 15;
/// The slew limit of the characterized segments, in characterization
/// steps, as `CtsOptions`' default `maxSlew`.
pub(crate) const MAX_SLEW: u32 = 4;
/// The share of a branch's sinks each of its two clusters may take without
/// a fanout limit, as `CtsOptions`' default `clusteringCapacity`.
pub(crate) const CLUSTERING_CAPACITY: f64 = 0.6;
/// The share of a branch's sinks each of its two clusters may take under a
/// fanout limit, as `HTreeBuilder::refineBranchingPointsWithClustering`
/// fixes it.
pub(crate) const FANOUT_LIMITED_CLUSTERING_CAPACITY: f64 = 0.5;
/// The exponent of the distance in branch-point clustering, as
/// `CtsOptions`' default `clusteringPower`.
pub(crate) const CLUSTERING_POWER: u32 = 4;
/// The most levels a tree is built to, as `CtsOptions`' default
/// `clockTreeMaxDepth`.
pub(crate) const CLOCK_TREE_MAX_DEPTH: u32 = 100;
/// A length in microns, as `CtsOptions` gives cluster diameters.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub(crate) struct Microns(f64);

impl Microns {
    /// `microns` microns.
    pub(crate) const fn new(microns: f64) -> Self {
        Self(microns)
    }

    /// The longer of `self` and `other`.
    #[must_use]
    pub(crate) const fn max(self, other: Self) -> Self {
        Self(self.0.max(other.0))
    }

    /// The length in a tree's length unit of `unit` database units, at
    /// `dbu` of them to a micron, worked out in double precision and
    /// rounded once to single, as `HTreeBuilder::initSinkRegion` scales
    /// the largest diameter.
    #[must_use]
    pub(crate) fn in_tree_units(self, dbu: i32, unit: f64) -> f32 {
        f32::nearest_from(self.0 * f64::from(dbu) / unit)
    }

    /// The length in a tree's length unit as [`Microns::in_tree_units`]
    /// gives it, but in single precision throughout, as
    /// `HTreeBuilder::preSinkClustering` scales the diameters it tries.
    #[must_use]
    pub(crate) fn in_tree_units_single(self, dbu: f32, unit: f32) -> f32 {
        f32::nearest_from(self.0) * dbu / unit
    }
}

#[cfg(test)]
impl Microns {
    /// The length in microns.
    pub(crate) const fn get(self) -> f64 {
        self.0
    }
}

/// The largest diameter of a cluster of register sinks, as `CtsOptions`'
/// default `maxDiameter`.
pub(crate) const MAX_DIAMETER: Microns = Microns::new(50.0);
/// The cluster sizes tried for register sinks before a buffer's fanout
/// limit caps them, as `CtsOptions`' default `sinkClusteringSizes`.
const SINK_CLUSTERING_SIZES: [u32; 3] = [10, 20, 30];
/// The most register sinks in a cluster, as `CtsOptions`' default
/// `sinkClusteringSize`.
pub(crate) const SINK_CLUSTERING_SIZE: u32 = 20;
/// The largest diameter of a cluster of macro sinks before the macros give
/// one, as `CtsOptions`' default `macroMaxDiameter`.
pub(crate) const MACRO_MAX_DIAMETER: Microns = Microns::new(50.0);
/// The most macro sinks in a cluster, as `CtsOptions`' default
/// `macroSinkClusteringSize`.
pub(crate) const MACRO_SINK_CLUSTERING_SIZE: u32 = 4;

/// The settings the tree builders read that a run sets or derives from the
/// library; those `TritonCTS` fixes are the constants above.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Settings {
    /// Whether sinks are clustered under leaf buffers.
    pub sink_clustering: SinkClusteringMode,
    /// Whether dummy loads balance the buffers' loads.
    pub dummy_loads: DummyLoads,
    /// The levels that get the clock's non-default rule.
    pub ndr: NdrStrategy,
    /// Whether sinks with insertion delay get trees of their own.
    pub insertion_delays: InsertionDelays,
    /// The most sinks a buffer drives, from the root and sink buffers'
    /// limits; none without one.
    pub max_fanout: Option<u32>,
    /// The cluster sizes tried for register sinks.
    pub sink_clustering_sizes: Vec<u32>,
    /// The largest diameter of a cluster of macro sinks once the macros
    /// set it.
    pub macro_max_diameter: Option<Microns>,
    /// The longest wire worth driving whole, when the resizer gave one.
    pub max_wire_length: Option<Dbu>,
}

impl Settings {
    pub(crate) fn new(options: CtsOptions) -> Self {
        Self {
            sink_clustering: options.sink_clustering,
            dummy_loads: options.dummy_loads,
            ndr: options.ndr,
            insertion_delays: options.insertion_delays,
            max_fanout: None,
            sink_clustering_sizes: SINK_CLUSTERING_SIZES.to_vec(),
            macro_max_diameter: None,
            max_wire_length: None,
        }
    }

    /// The largest diameter of a cluster of macro sinks.
    pub(crate) fn macro_max_diameter(&self) -> Microns {
        self.macro_max_diameter.unwrap_or(MACRO_MAX_DIAMETER)
    }

    /// Caps the cluster sizes tried at a buffer's fanout limit, as
    /// `CtsOptions::limitSinkClusteringSizes` without a size given.
    pub(crate) fn limit_sink_clustering_sizes(&mut self, limit: u32) {
        let keep = self.sink_clustering_sizes.partition_point(|&s| s < limit);
        self.sink_clustering_sizes.truncate(keep);
        self.sink_clustering_sizes.push(limit);
    }
}
