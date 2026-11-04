//! What clock tree synthesis is asked to do, and the settings `TritonCTS`
//! fixes or derives before building the trees.

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

/// The requests `clock_tree_synthesis` takes; the defaults are what
/// `OpenROAD-flow-scripts` runs it with.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)] // each is an independent command flag
pub struct CtsOptions {
    /// Clusters the sinks under leaf buffers (`-sink_clustering_enable`).
    pub sink_clustering: bool,
    /// Buffers clock nets longer than the longest wire worth driving whole
    /// (`-repair_clock_nets`).
    pub repair_clock_nets: bool,
    /// Balances the buffers' loads with dummy cells (no
    /// `-dont_use_dummy_load`).
    pub dummy_loads: bool,
    /// Builds separate trees for sinks with insertion delay (no
    /// `-no_insertion_delay`).
    pub insertion_delay: bool,
    /// The non-default rule's levels (`-apply_ndr`).
    pub ndr: NdrStrategy,
}

impl Default for CtsOptions {
    fn default() -> Self {
        Self {
            sink_clustering: true,
            repair_clock_nets: true,
            dummy_loads: true,
            insertion_delay: true,
            ndr: NdrStrategy::Half,
        }
    }
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
/// The largest diameter of a cluster of register sinks, in microns, as
/// `CtsOptions`' default `maxDiameter`.
pub(crate) const MAX_DIAMETER: f64 = 50.0;
/// The cluster sizes tried for register sinks before a buffer's fanout
/// limit caps them, as `CtsOptions`' default `sinkClusteringSizes`.
const SINK_CLUSTERING_SIZES: [u32; 3] = [10, 20, 30];
/// The most register sinks in a cluster, as `CtsOptions`' default
/// `sinkClusteringSize`.
pub(crate) const SINK_CLUSTERING_SIZE: u32 = 20;
/// The largest diameter of a cluster of macro sinks before the macros give
/// one, in microns, as `CtsOptions`' default `macroMaxDiameter`.
pub(crate) const MACRO_MAX_DIAMETER: f64 = 50.0;
/// The most macro sinks in a cluster, as `CtsOptions`' default
/// `macroSinkClusteringSize`.
pub(crate) const MACRO_SINK_CLUSTERING_SIZE: u32 = 4;

/// The settings the tree builders read that a run sets or derives from the
/// library; those `TritonCTS` fixes are the constants above.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Settings {
    /// Whether sinks are clustered under leaf buffers.
    pub sink_clustering: bool,
    /// Whether dummy loads balance the buffers' loads.
    pub dummy_loads: bool,
    /// The levels that get the clock's non-default rule.
    pub ndr: NdrStrategy,
    /// Whether sinks with insertion delay get trees of their own.
    pub insertion_delay: bool,
    /// The most sinks a buffer drives, from the root and sink buffers'
    /// limits; none without one.
    pub max_fanout: Option<u32>,
    /// The cluster sizes tried for register sinks.
    pub sink_clustering_sizes: Vec<u32>,
    /// The largest diameter of a cluster of macro sinks once the macros
    /// set it, in microns.
    pub macro_max_diameter: Option<f64>,
    /// The longest wire worth driving whole, in database units; zero when
    /// unknown.
    pub max_wire_length: i64,
}

impl Settings {
    pub(crate) fn new(options: CtsOptions) -> Self {
        Self {
            sink_clustering: options.sink_clustering,
            dummy_loads: options.dummy_loads,
            ndr: options.ndr,
            insertion_delay: options.insertion_delay,
            max_fanout: None,
            sink_clustering_sizes: SINK_CLUSTERING_SIZES.to_vec(),
            macro_max_diameter: None,
            max_wire_length: 0,
        }
    }

    /// The largest diameter of a cluster of macro sinks, in microns.
    pub(crate) fn macro_max_diameter(&self) -> f64 {
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
