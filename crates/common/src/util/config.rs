//! TOML run configuration.

use crate::db::indices::LayerId;
use crate::db::library::Library;
use serde::Deserialize;
use std::ops::RangeInclusive;

/// A run configuration. Unknown keys are rejected so typos do not pass silently.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// What the design is a benchmark of.
    pub purpose: Purpose,
    /// Worker threads for every parallel step; every core if unset.
    pub threads: Option<usize>,
    /// Global placement parameters.
    #[serde(default)]
    pub global_placement: GlobalPlacementConfig,
    /// Placement of top-level I/O pins.
    #[serde(default)]
    pub io_placement: IoPlacementConfig,
    /// Routing parameters.
    #[serde(default)]
    pub routing: RoutingConfig,
    /// Input and output files.
    #[serde(default)]
    pub input: InputConfig,
}

/// What a design is a benchmark of, which decides what running it does.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum Purpose {
    /// Made to test placement: it is placed and never routed.
    Placement,
    /// Made to test routing: its placement is part of the input, and it is
    /// only routed.
    Routing,
    /// Placed and then routed.
    PlaceAndRoute,
}

/// Parameters of the analytical global placer.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GlobalPlacementConfig {
    /// Target bin density; derived from utilization as
    /// `OpenROAD-flow-scripts` does if unset.
    pub target_density: Option<f64>,
    /// Bins per side of the density grid; derived from the cell count if unset.
    pub bin_dimension: Option<usize>,
    /// Iteration limit.
    #[serde(default = "default_placer_max_iterations")]
    pub placer_max_iterations: usize,
    /// Cell area above the target density, as a fraction of all cell area,
    /// at which global placement stops.
    #[serde(default = "default_target_overflow")]
    pub target_overflow: f64,
    /// Seed for the random initial placement.
    #[serde(default = "default_seed")]
    pub seed: u64,
    /// Whether standard cells in congested gcells are inflated during
    /// global placement, as in `OpenROAD`.
    #[serde(default = "default_routability_driven")]
    pub routability_driven: bool,
    /// Mean congestion of the most congested gcells below which
    /// routability-driven placement stops inflating cells.
    #[serde(default = "default_target_congestion")]
    pub target_congestion: f64,
    /// How routability-driven placement measures congestion.
    #[serde(default)]
    pub congestion_estimate: CongestionEstimate,
}

/// How routability-driven placement measures routing congestion.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum CongestionEstimate {
    /// RUDY: each net's wire spread evenly over its bounding box, as
    /// `OpenROAD` does by default.
    #[default]
    Rudy,
    /// Global routing of every net where the cells are, as `OpenROAD`'s
    /// `-routability_use_grt`. Its congestion readings run far above RUDY's,
    /// and on designs with macro channels the inflation they drive has
    /// destabilized placement; measure before relying on it.
    GlobalRouter,
}

impl Default for GlobalPlacementConfig {
    fn default() -> Self {
        Self {
            target_density: None,
            bin_dimension: None,
            placer_max_iterations: default_placer_max_iterations(),
            target_overflow: default_target_overflow(),
            seed: default_seed(),
            routability_driven: default_routability_driven(),
            target_congestion: default_target_congestion(),
            congestion_estimate: CongestionEstimate::default(),
        }
    }
}

/// Placement of top-level I/O pins that are not fixed.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IoPlacementConfig {
    /// Whether placement moves unplaced and placed (not fixed) I/O pins.
    #[serde(default = "default_io_placement_enabled")]
    pub enabled: bool,
    /// Horizontal routing layer for pins on the left and right edges.
    pub horizontal_layer: Option<String>,
    /// Vertical routing layer for pins on the top and bottom edges.
    pub vertical_layer: Option<String>,
}

impl Default for IoPlacementConfig {
    fn default() -> Self {
        Self {
            enabled: default_io_placement_enabled(),
            horizontal_layer: None,
            vertical_layer: None,
        }
    }
}

const fn default_io_placement_enabled() -> bool {
    true
}

/// Parameters of the global and detailed router.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoutingConfig {
    /// Lowest routing layer for signal wires; defaults to the second layer.
    pub min_layer: Option<String>,
    /// Highest routing layer for signal wires; defaults to the top layer.
    pub max_layer: Option<String>,
    /// Global routing cell edge, in gcell pitches: the middle track spacing
    /// of the second to fourth routing layers, as `OpenROAD` sizes gcells.
    #[serde(default = "default_gcell_tracks")]
    pub gcell_tracks: u32,
    /// Rip-up and reroute rounds of global routing.
    #[serde(default = "default_global_iterations")]
    pub global_iterations: usize,
    /// Fraction of each gcell edge's tracks that global routing leaves for
    /// what detailed routing needs beyond the wires and vias it plans: wire
    /// ends, jogs and room to negotiate, in `[0, 1)`.
    #[serde(default = "default_global_capacity_reserve")]
    pub global_capacity_reserve: f64,
    /// Rip-up and reroute rounds of detailed routing.
    #[serde(default = "default_detailed_iterations")]
    pub detailed_iterations: usize,
}

/// Why the configured routing layers cannot be used.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum LayerRangeError {
    /// The library defines no routing layer.
    #[error("the library has no routing layers")]
    NoLayers,
    /// A configured layer name is not a routing layer.
    #[error("unknown routing layer '{0}'")]
    Unknown(String),
    /// The minimum routing layer is above the maximum.
    #[error("the minimum routing layer is above the maximum")]
    Empty,
}

impl RoutingConfig {
    /// The layers signal wires may use: `min_layer`, by default the second
    /// layer (the first when there is only one), up to `max_layer`, by
    /// default the top layer.
    pub fn layer_range(&self, lib: &Library) -> Result<RangeInclusive<LayerId>, LayerRangeError> {
        let top = lib
            .routing_layers
            .len()
            .checked_sub(1)
            .ok_or(LayerRangeError::NoLayers)?;
        let resolve = |name: &Option<String>, default: usize| match name {
            None => Ok(LayerId::new(default)),
            Some(n) => lib
                .routing_layer_id(n)
                .ok_or_else(|| LayerRangeError::Unknown(n.clone())),
        };
        let min = resolve(&self.min_layer, top.min(1))?;
        let max = resolve(&self.max_layer, top)?;
        if min > max {
            return Err(LayerRangeError::Empty);
        }
        Ok(min..=max)
    }
}

impl Default for RoutingConfig {
    fn default() -> Self {
        Self {
            min_layer: None,
            max_layer: None,
            gcell_tracks: default_gcell_tracks(),
            global_iterations: default_global_iterations(),
            global_capacity_reserve: default_global_capacity_reserve(),
            detailed_iterations: default_detailed_iterations(),
        }
    }
}

/// Input design and output locations.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InputConfig {
    /// LEF files, technology first.
    #[serde(default)]
    pub lef_files: Vec<String>,
    /// DEF design.
    #[serde(default)]
    pub def_file: String,
    /// Where to write the placed DEF.
    #[serde(default = "default_output_def")]
    pub output_def: String,
    /// Bookshelf `.aux` file, used instead of LEF/DEF when set.
    pub bookshelf_aux_file: Option<String>,
    /// Routing layers to synthesize for a Bookshelf design.
    #[serde(default = "default_num_layers")]
    pub num_layers: usize,
    /// Directory for generated files; defaults to `output_def`'s directory.
    pub output_dir: Option<String>,
}

impl Default for InputConfig {
    fn default() -> Self {
        Self {
            lef_files: Vec::new(),
            def_file: String::new(),
            output_def: default_output_def(),
            bookshelf_aux_file: None,
            num_layers: default_num_layers(),
            output_dir: None,
        }
    }
}

const fn default_placer_max_iterations() -> usize {
    2000
}

const fn default_target_overflow() -> f64 {
    0.1
}

const fn default_seed() -> u64 {
    1
}

const fn default_routability_driven() -> bool {
    true
}

const fn default_target_congestion() -> f64 {
    1.01
}

const fn default_gcell_tracks() -> u32 {
    15
}

const fn default_global_iterations() -> usize {
    50
}

const fn default_global_capacity_reserve() -> f64 {
    0.25
}

const fn default_detailed_iterations() -> usize {
    40
}

const fn default_num_layers() -> usize {
    6
}

fn default_output_def() -> String {
    "output/placed.def".to_string()
}
