//! TOML run configuration.

use serde::Deserialize;

/// A run configuration. Unknown keys are rejected so typos do not pass silently.
#[derive(Debug, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// Global placement parameters.
    #[serde(default)]
    pub global_placement: GlobalPlacementConfig,
    /// Routing parameters.
    #[serde(default)]
    pub routing: RoutingConfig,
    /// Input and output files.
    #[serde(default)]
    pub input: InputConfig,
}

/// Parameters of the analytical global placer.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GlobalPlacementConfig {
    /// Target bin density; derived from utilization if unset.
    pub target_density: Option<f64>,
    /// Bins per side of the density grid; derived from the cell count if unset.
    pub bin_dimension: Option<usize>,
    /// Iteration limit.
    #[serde(default = "default_placer_max_iterations")]
    pub placer_max_iterations: usize,
    /// Initial step size; derived from the median cell height if unset.
    pub initial_learning_rate: Option<f64>,
    /// Average per-iteration cell movement below which placement has converged.
    #[serde(default = "default_convergence_threshold")]
    pub convergence_threshold: f64,
    /// Weighted-average wirelength smoothing; derived from the die size if unset.
    pub wa_gamma: Option<f64>,
    /// Density weight used if it cannot be calibrated from gradient norms.
    #[serde(default = "default_electro_force_multiplier")]
    pub electro_force_multiplier: f64,
}

impl Default for GlobalPlacementConfig {
    fn default() -> Self {
        Self {
            target_density: None,
            bin_dimension: None,
            placer_max_iterations: default_placer_max_iterations(),
            initial_learning_rate: None,
            convergence_threshold: default_convergence_threshold(),
            wa_gamma: None,
            electro_force_multiplier: default_electro_force_multiplier(),
        }
    }
}

/// Parameters of the global and detailed router.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RoutingConfig {
    /// Lowest routing layer for signal wires; defaults to the second layer.
    pub min_layer: Option<String>,
    /// Highest routing layer for signal wires; defaults to the top layer.
    pub max_layer: Option<String>,
    /// Global routing cell edge, in tracks of the lowest routing layer.
    #[serde(default = "default_gcell_tracks")]
    pub gcell_tracks: u32,
    /// Rip-up and reroute rounds of global routing.
    #[serde(default = "default_global_iterations")]
    pub global_iterations: usize,
    /// Rip-up and reroute rounds of detailed routing.
    #[serde(default = "default_detailed_iterations")]
    pub detailed_iterations: usize,
}

impl Default for RoutingConfig {
    fn default() -> Self {
        Self {
            min_layer: None,
            max_layer: None,
            gcell_tracks: default_gcell_tracks(),
            global_iterations: default_global_iterations(),
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

const fn default_convergence_threshold() -> f64 {
    2e-4
}

const fn default_electro_force_multiplier() -> f64 {
    20.0
}

const fn default_gcell_tracks() -> u32 {
    15
}

const fn default_global_iterations() -> usize {
    50
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
