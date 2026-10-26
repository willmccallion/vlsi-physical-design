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

/// The settings the tree builders read, `TritonCTS`'s `CtsOptions` with the
/// values it fixes and those it derives from the library.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::struct_excessive_bools)] // each mirrors an independent TritonCTS option
pub(crate) struct Settings {
    pub sink_clustering: bool,
    pub dummy_loads: bool,
    pub ndr: NdrStrategy,
    pub insertion_delay: bool,
    pub obstruction_aware: bool,
    pub num_max_leaf_sinks: u32,
    pub max_fanout: u32,
    pub max_slew: u32,
    pub clustering_capacity: f64,
    pub clustering_power: u32,
    pub clock_tree_max_depth: u32,
    pub sink_clustering_sizes: Vec<u32>,
    pub sink_clustering_diameters: Vec<u32>,
    pub max_diameter: f64,
    pub sink_clustering_size: u32,
    pub macro_max_diameter: f64,
    pub macro_max_diameter_set: bool,
    pub macro_sink_clustering_size: u32,
    pub sink_clustering_use_max_cap: bool,
    pub sink_clustering_levels: u32,
    pub num_static_layers: u32,
    pub fake_lut_entries: bool,
    pub max_wire_length: i64,
}

impl Settings {
    pub(crate) fn new(options: CtsOptions) -> Self {
        Self {
            sink_clustering: options.sink_clustering,
            dummy_loads: options.dummy_loads,
            ndr: options.ndr,
            insertion_delay: options.insertion_delay,
            obstruction_aware: true,
            num_max_leaf_sinks: 15,
            max_fanout: 0,
            max_slew: 4,
            clustering_capacity: 0.6,
            clustering_power: 4,
            clock_tree_max_depth: 100,
            sink_clustering_sizes: vec![10, 20, 30],
            sink_clustering_diameters: vec![50, 100, 200],
            max_diameter: 50.0,
            sink_clustering_size: 20,
            macro_max_diameter: 50.0,
            macro_max_diameter_set: false,
            macro_sink_clustering_size: 4,
            sink_clustering_use_max_cap: true,
            sink_clustering_levels: 0,
            num_static_layers: 0,
            fake_lut_entries: true,
            max_wire_length: 0,
        }
    }

    /// Caps the cluster sizes tried at a buffer's fanout limit, as
    /// `CtsOptions::limitSinkClusteringSizes` without a size given.
    pub(crate) fn limit_sink_clustering_sizes(&mut self, limit: u32) {
        let keep = self.sink_clustering_sizes.partition_point(|&s| s < limit);
        self.sink_clustering_sizes.truncate(keep);
        self.sink_clustering_sizes.push(limit);
    }
}
