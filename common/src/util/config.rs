//! Configuration Management for EDA Toolchain Parameters.
//!
//! Defines the configuration structure that controls algorithm behavior
//! across placement and routing. Parameters are loaded from TOML files
//! with sensible defaults for all settings.

use serde::Deserialize;

/// Root configuration structure containing all algorithm parameters.
///
/// Groups configuration into logical sections: global placement, legalization,
/// global routing, detailed routing, and input file paths. All sections
/// have default implementations that provide reasonable starting values.
#[derive(Debug, Deserialize)]
pub struct Config {
    #[serde(default)]
    pub global_placement: GlobalPlacementConfig,
    #[serde(default)]
    pub legalization: LegalizationConfig,
    #[serde(default)]
    pub global_routing: GlobalRoutingConfig,
    #[serde(default)]
    pub detailed_routing: DetailedRoutingConfig,
    #[serde(default)]
    pub input: InputConfig,
}

impl Default for Config {
    /// Creates a default configuration with all sub-configs set to defaults.
    ///
    /// Initializes all configuration sections (placement, routing, input) with
    /// their default parameter values. Used when no configuration file is provided.
    fn default() -> Self {
        Self {
            global_placement: GlobalPlacementConfig::default(),
            legalization: LegalizationConfig::default(),
            global_routing: GlobalRoutingConfig::default(),
            detailed_routing: DetailedRoutingConfig::default(),
            input: InputConfig::default(),
        }
    }
}

/// Configuration parameters for global placement optimization.
///
/// Controls the behavior of the analytical placer including bin grid resolution,
/// convergence criteria, learning rates, and objective function weights. These
/// parameters significantly affect placement quality and runtime.
#[derive(Debug, Deserialize)]
pub struct GlobalPlacementConfig {
    pub target_density: Option<f64>,
    pub bin_dimension: Option<usize>,
    #[serde(default = "default_placer_max_iterations")]
    pub placer_max_iterations: usize,
    pub initial_learning_rate: Option<f64>,
    #[serde(default = "default_convergence_threshold")]
    pub convergence_threshold: f64,
    pub wa_gamma: Option<f64>,
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

/// Configuration parameters for legalization algorithms.
///
/// Specifies which legalization algorithm to use (Abacus or Tetris) and any
/// algorithm-specific parameters. Legalization converts the analytically
/// placed design (which may have overlaps) into a legal placement.
#[derive(Debug, Deserialize)]
pub struct LegalizationConfig {
    #[serde(default = "default_legalization_algo")]
    pub algorithm: String,
}

impl Default for LegalizationConfig {
    /// Creates a default legalization configuration using the Abacus algorithm.
    ///
    /// Sets the default legalization algorithm to "abacus", which provides
    /// better placement quality than Tetris for most designs.
    fn default() -> Self {
        Self {
            algorithm: default_legalization_algo(),
        }
    }
}

/// Configuration parameters for global routing.
///
/// Controls the coarse-grid routing that generates routing guides for detailed
/// routing. Includes grid cell size, iteration limits, congestion penalties,
/// and A* search parameters. Global routing operates on a much coarser grid
/// than detailed routing to reduce computational complexity.
#[derive(Debug, Deserialize)]
pub struct GlobalRoutingConfig {
    #[serde(default = "default_gcell_size")]
    pub gcell_size: usize,
    #[serde(default = "default_gr_max_iterations")]
    pub max_iterations: usize,
    #[serde(default = "default_gr_history_increment")]
    pub history_increment: f64,
    #[serde(default = "default_gr_initial_penalty")]
    pub initial_penalty: f64,
    #[serde(default = "default_gr_penalty_multiplier")]
    pub penalty_multiplier: f64,
    #[serde(default = "default_gr_capacity")]
    pub capacity: u32,
    #[serde(default = "default_gr_heuristic")]
    pub heuristic_weight: f64,
    #[serde(default = "default_gr_margin")]
    pub margin: u32,
}

impl Default for GlobalRoutingConfig {
    /// Creates a default global routing configuration with standard parameters.
    ///
    /// Initializes all global routing parameters to values suitable for
    /// coarse-grid routing that generates guides for detailed routing.
    fn default() -> Self {
        Self {
            gcell_size: default_gcell_size(),
            max_iterations: default_gr_max_iterations(),
            history_increment: default_gr_history_increment(),
            initial_penalty: default_gr_initial_penalty(),
            penalty_multiplier: default_gr_penalty_multiplier(),
            capacity: default_gr_capacity(),
            heuristic_weight: default_gr_heuristic(),
            margin: default_gr_margin(),
        }
    }
}

/// Configuration parameters for detailed routing.
///
/// Controls the fine-grid routing that produces final wire paths. Includes
/// iteration limits, congestion penalties, A* search parameters, rip-up
/// strategies, and stagnation detection. Detailed routing operates on a
/// fine grid aligned to routing tracks and must satisfy all design rules.
#[derive(Debug, Deserialize, Clone)]
pub struct DetailedRoutingConfig {
    #[serde(default = "default_dr_max_iterations")]
    pub max_iterations: usize,
    #[serde(default = "default_dr_history_increment")]
    pub history_increment: f64,
    #[serde(default = "default_dr_initial_penalty")]
    pub initial_penalty: f64,
    #[serde(default = "default_dr_penalty_multiplier")]
    pub penalty_multiplier: f64,
    #[serde(default = "default_dr_capacity")]
    pub capacity: u32,
    #[serde(default = "default_dr_heuristic")]
    pub astar_heuristic_weight: f64,
    #[serde(default = "default_dr_margin_base")]
    pub astar_window_margin_base: u32,
    #[serde(default = "default_dr_margin_max")]
    pub astar_window_margin_max: u32,
    #[serde(default = "default_stagnation_threshold")]
    pub stagnation_threshold: usize,
    #[serde(default = "default_ripup_radius")]
    pub ripup_radius: i32,
    #[serde(default = "default_dr_gcell_size")]
    pub gcell_size: f64,
}

impl Default for DetailedRoutingConfig {
    /// Creates a default detailed routing configuration with standard parameters.
    ///
    /// Initializes all detailed routing parameters to values suitable for
    /// fine-grid routing that produces final wire paths satisfying design rules.
    fn default() -> Self {
        Self {
            max_iterations: default_dr_max_iterations(),
            history_increment: default_dr_history_increment(),
            initial_penalty: default_dr_initial_penalty(),
            penalty_multiplier: default_dr_penalty_multiplier(),
            capacity: default_dr_capacity(),
            astar_heuristic_weight: default_dr_heuristic(),
            astar_window_margin_base: default_dr_margin_base(),
            astar_window_margin_max: default_dr_margin_max(),
            stagnation_threshold: default_stagnation_threshold(),
            ripup_radius: default_ripup_radius(),
            gcell_size: default_dr_gcell_size(),
        }
    }
}

/// Configuration for input and output file paths.
///
/// Specifies the locations of input files (LEF libraries, DEF netlists,
/// Bookshelf files) and output files (placed DEF, routed DEF). Supports
/// both industry-standard LEF/DEF format and academic Bookshelf format.
#[derive(Debug, Deserialize)]
pub struct InputConfig {
    #[serde(default = "default_lef_files")]
    pub lef_files: Vec<String>,
    #[serde(default = "default_def_file")]
    pub def_file: String,
    #[serde(default = "default_output_def")]
    pub output_def: String,
    pub bookshelf_aux_file: Option<String>,
}

impl Default for InputConfig {
    /// Creates a default input configuration with empty file paths.
    ///
    /// Initializes all file paths to empty strings. The caller must provide
    /// actual file paths via configuration file or command-line arguments.
    fn default() -> Self {
        Self {
            lef_files: default_lef_files(),
            def_file: default_def_file(),
            output_def: default_output_def(),
            bookshelf_aux_file: None,
        }
    }
}


/// Default maximum iterations for placement optimization (2000).
fn default_placer_max_iterations() -> usize {
    2000
}


/// Default convergence threshold for placement (average cell movement < 0.0002).
fn default_convergence_threshold() -> f64 {
    2e-4
}


/// Default multiplier for electrostatic density forces (20.0).
fn default_electro_force_multiplier() -> f64 {
    20.0
}

/// Default legalization algorithm name ("abacus").
fn default_legalization_algo() -> String {
    "abacus".to_string()
}

/// Default global routing grid cell size in physical units (128).
fn default_gcell_size() -> usize {
    128
}

/// Default maximum iterations for global routing (300).
fn default_gr_max_iterations() -> usize {
    300
}

/// Default history cost increment for global routing (0.5).
fn default_gr_history_increment() -> f64 {
    0.5
}

/// Default initial congestion penalty for global routing (0.5).
fn default_gr_initial_penalty() -> f64 {
    0.5
}

/// Default penalty multiplier per iteration for global routing (1.1).
fn default_gr_penalty_multiplier() -> f64 {
    1.1
}

/// Default routing capacity per grid cell for global routing (10).
fn default_gr_capacity() -> u32 {
    10
}

/// Default A* heuristic weight for global routing (1.5).
fn default_gr_heuristic() -> f64 {
    1.5
}

/// Default search window margin for global routing (10 cells).
fn default_gr_margin() -> u32 {
    10
}

/// Default maximum iterations for detailed routing (2000).
fn default_dr_max_iterations() -> usize {
    2000
}

/// Default history cost increment for detailed routing (0.5).
fn default_dr_history_increment() -> f64 {
    0.5
}

/// Default initial congestion penalty for detailed routing (0.5).
fn default_dr_initial_penalty() -> f64 {
    0.5
}

/// Default penalty multiplier per iteration for detailed routing (1.3).
fn default_dr_penalty_multiplier() -> f64 {
    1.3
}

/// Default routing capacity per grid cell for detailed routing (1).
fn default_dr_capacity() -> u32 {
    1
}

/// Default A* heuristic weight for detailed routing (3.0).
fn default_dr_heuristic() -> f64 {
    3.0
}

/// Default base search window margin for detailed routing (30 cells).
fn default_dr_margin_base() -> u32 {
    30
}

/// Default maximum search window margin for detailed routing (200 cells).
fn default_dr_margin_max() -> u32 {
    200
}

/// Default stagnation threshold before triggering aggressive rip-up (25 iterations).
fn default_stagnation_threshold() -> usize {
    25
}

/// Default radius for rip-up operations in detailed routing (2 cells).
fn default_ripup_radius() -> i32 {
    2
}

/// Default gcell size for detailed routing (2.0 microns).
fn default_dr_gcell_size() -> f64 {
    2.0
}

fn default_lef_files() -> Vec<String> {
    vec![]
}

fn default_def_file() -> String {
    String::new()
}

fn default_output_def() -> String {
    "output/placed.def".to_string()
}
