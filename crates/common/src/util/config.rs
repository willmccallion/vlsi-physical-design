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
    /// Netlist repair around placement.
    #[serde(default)]
    pub netlist: NetlistConfig,
    /// Input and output files.
    #[serde(default)]
    pub input: InputConfig,
}

/// Netlist repair around placement, as OpenROAD-flow-scripts runs it. It
/// runs when the input names Liberty files, and not otherwise.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
#[allow(clippy::struct_excessive_bools)] // each is an independent on/off switch in the TOML
pub struct NetlistConfig {
    /// Whether the buffers synthesis left are removed before placement.
    #[serde(default = "default_enabled")]
    pub remove_buffers: bool,
    /// Whether every port gets a buffer before placement.
    #[serde(default = "default_enabled")]
    pub buffer_ports: bool,
    /// Whether drivers are resized and repeaters inserted after global
    /// placement until slews and loads meet the library's limits.
    #[serde(default = "default_enabled")]
    pub repair_design: bool,
    /// Whether the paths failing setup are repaired after design repair,
    /// when the input names an SDC file.
    #[serde(default = "default_enabled")]
    pub repair_timing: bool,
    /// Whether clock trees are built after detailed placement, when the
    /// input names an SDC file, and the clocks propagated through them.
    #[serde(default = "default_enabled")]
    pub clock_tree: bool,
    /// Whether the design and timing are repaired again after global
    /// routing, on the parasitics of the global routes, when the input
    /// names an SDC file.
    #[serde(default = "default_enabled")]
    pub repair_on_global_routes: bool,
    /// The routing layer signal wires are estimated on.
    pub signal_layer: Option<String>,
    /// Resistance of that layer's wire, in ohms per micron.
    pub wire_resistance_ohm_per_um: Option<f64>,
    /// Capacitance of that layer's wire, in femtofarads per micron.
    pub wire_capacitance_ff_per_um: Option<f64>,
    /// The routing layer clock wires are estimated on, as `set_wire_rc
    /// -clock` sets it, which clock trees need; without it the nets
    /// between clock buffers are estimated as signal wires.
    pub clock_layer: Option<String>,
    /// Resistance of the clock layer's wire, in ohms per micron.
    pub clock_wire_resistance_ohm_per_um: Option<f64>,
    /// Capacitance of the clock layer's wire, in femtofarads per micron.
    pub clock_wire_capacitance_ff_per_um: Option<f64>,
    /// Wire resistance and capacitance per routing layer, as the platform's
    /// `set_layer_rc -layer` gives them, which global-route parasitics use;
    /// a layer left out takes its LEF values.
    #[serde(default)]
    pub layer_rc: Vec<LayerRcConfig>,
    /// Resistance of one cut per cut layer, as `set_layer_rc -via` gives
    /// it; a layer left out takes its LEF value.
    #[serde(default)]
    pub via_rc: Vec<ViaRcConfig>,
    /// Cells repair neither inserts nor swaps to, beyond those the library
    /// marks.
    #[serde(default)]
    pub dont_use: Vec<String>,
    /// The I/O pins clocks enter at, which repair leaves alone; only
    /// without an SDC file, which names the clocks itself.
    #[serde(default)]
    pub clock_ports: Vec<String>,
}

/// One routing layer's wire resistance and capacitance per length.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LayerRcConfig {
    /// The routing layer's name.
    pub layer: String,
    /// Ohms per micron.
    pub resistance_ohm_per_um: f64,
    /// Femtofarads per micron.
    pub capacitance_ff_per_um: f64,
}

/// One cut layer's resistance per cut.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ViaRcConfig {
    /// The cut layer's name.
    pub layer: String,
    /// Ohms per cut.
    pub resistance_ohm: f64,
}

impl Default for NetlistConfig {
    fn default() -> Self {
        Self {
            remove_buffers: default_enabled(),
            buffer_ports: default_enabled(),
            repair_design: default_enabled(),
            repair_timing: default_enabled(),
            clock_tree: default_enabled(),
            repair_on_global_routes: default_enabled(),
            signal_layer: None,
            wire_resistance_ohm_per_um: None,
            wire_capacitance_ff_per_um: None,
            clock_layer: None,
            clock_wire_resistance_ohm_per_um: None,
            clock_wire_capacitance_ff_per_um: None,
            layer_rc: Vec::new(),
            via_rc: Vec::new(),
            dont_use: Vec::new(),
            clock_ports: Vec::new(),
        }
    }
}

const fn default_enabled() -> bool {
    true
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
    /// Share of each routing layer's capacity that routability-driven
    /// placement counts as taken, as `OpenROAD`'s RUDY counts
    /// OpenROAD-flow-scripts' `ROUTING_LAYER_ADJUSTMENT`, in `[0, 1]`.
    #[serde(default = "default_routing_layer_adjustment")]
    pub routing_layer_adjustment: f64,
    /// Whether the design is repaired and its critical nets weighted
    /// during global placement, as `OpenROAD`'s timing-driven mode, when
    /// the input has an SDC file.
    #[serde(default = "default_enabled")]
    pub timing_driven: bool,
    /// The overflows at which timing-driven placement repairs the design,
    /// each once as the overflow falls below it, as `OpenROAD`'s
    /// `timing_driven_net_reweight_overflow`.
    #[serde(default = "default_timing_driven_overflows")]
    pub timing_driven_overflows: Vec<f64>,
    /// The wirelength weight the net of worst slack gets, as `OpenROAD`'s
    /// `timing_driven_net_weight_max`.
    #[serde(default = "default_timing_net_weight_max")]
    pub timing_net_weight_max: f64,
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
            routing_layer_adjustment: default_routing_layer_adjustment(),
            timing_driven: default_enabled(),
            timing_driven_overflows: default_timing_driven_overflows(),
            timing_net_weight_max: default_timing_net_weight_max(),
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
    /// Liberty files; netlist repair runs when there are any.
    #[serde(default)]
    pub liberty_files: Vec<String>,
    /// SDC timing constraints; timing repair runs when set.
    pub sdc_file: Option<String>,
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
            liberty_files: Vec::new(),
            sdc_file: None,
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

fn default_timing_driven_overflows() -> Vec<f64> {
    vec![0.64, 0.20]
}

const fn default_timing_net_weight_max() -> f64 {
    5.0
}

const fn default_routability_driven() -> bool {
    true
}

const fn default_target_congestion() -> f64 {
    1.01
}

/// OpenROAD-flow-scripts' default `ROUTING_LAYER_ADJUSTMENT`, which its
/// Nangate45 platform keeps.
const fn default_routing_layer_adjustment() -> f64 {
    0.5
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

/// `TritonRoute`'s last iteration, the last row of its schedule, where its
/// rounds end before `END_ITERATION` caps them.
const fn default_detailed_iterations() -> usize {
    64
}

const fn default_num_layers() -> usize {
    6
}

fn default_output_def() -> String {
    "output/placed.def".to_string()
}
