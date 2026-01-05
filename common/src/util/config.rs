use serde::Deserialize;

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

#[derive(Debug, Deserialize)]
pub struct GlobalPlacementConfig {
    #[serde(default = "default_target_density")]
    pub target_density: f64,
    #[serde(default = "default_bin_dimension")]
    pub bin_dimension: usize,
    #[serde(default = "default_placer_max_iterations")]
    pub placer_max_iterations: usize,
    #[serde(default = "default_initial_learning_rate")]
    pub initial_learning_rate: f64,
    #[serde(default = "default_convergence_threshold")]
    pub convergence_threshold: f64,
    #[serde(default = "default_wa_gamma")]
    pub wa_gamma: f64,
    #[serde(default = "default_electro_force_multiplier")]
    pub electro_force_multiplier: f64,
}

impl Default for GlobalPlacementConfig {
    fn default() -> Self {
        Self {
            target_density: default_target_density(),
            bin_dimension: default_bin_dimension(),
            placer_max_iterations: default_placer_max_iterations(),
            initial_learning_rate: default_initial_learning_rate(),
            convergence_threshold: default_convergence_threshold(),
            wa_gamma: default_wa_gamma(),
            electro_force_multiplier: default_electro_force_multiplier(),
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct LegalizationConfig {
    #[serde(default = "default_legalization_algo")]
    pub algorithm: String,
}

impl Default for LegalizationConfig {
    fn default() -> Self {
        Self {
            algorithm: default_legalization_algo(),
        }
    }
}

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
}

impl Default for DetailedRoutingConfig {
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
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct InputConfig {
    #[serde(default = "default_lef_files")]
    pub lef_files: Vec<String>,
    #[serde(default = "default_def_file")]
    pub def_file: String,
    #[serde(default = "default_output_def")]
    pub output_def: String,
}

impl Default for InputConfig {
    fn default() -> Self {
        Self {
            lef_files: default_lef_files(),
            def_file: default_def_file(),
            output_def: default_output_def(),
        }
    }
}

fn default_target_density() -> f64 {
    0.60
}

fn default_bin_dimension() -> usize {
    256
}

fn default_placer_max_iterations() -> usize {
    2000
}

fn default_initial_learning_rate() -> f64 {
    0.003
}

fn default_convergence_threshold() -> f64 {
    2e-4
}

fn default_wa_gamma() -> f64 {
    4.0
}

fn default_electro_force_multiplier() -> f64 {
    20.0
}

fn default_legalization_algo() -> String {
    "abacus".to_string()
}

fn default_gcell_size() -> usize {
    128
}

fn default_gr_max_iterations() -> usize {
    300
}

fn default_gr_history_increment() -> f64 {
    0.5
}

fn default_gr_initial_penalty() -> f64 {
    0.5
}

fn default_gr_penalty_multiplier() -> f64 {
    1.1
}

fn default_gr_capacity() -> u32 {
    10
}

fn default_gr_heuristic() -> f64 {
    1.5
}

fn default_gr_margin() -> u32 {
    10
}

fn default_dr_max_iterations() -> usize {
    2000
}

fn default_dr_history_increment() -> f64 {
    0.2
}

fn default_dr_initial_penalty() -> f64 {
    1.5
}

fn default_dr_penalty_multiplier() -> f64 {
    1.05
}

fn default_dr_capacity() -> u32 {
    1
}

fn default_dr_heuristic() -> f64 {
    5.0
}

fn default_dr_margin_base() -> u32 {
    20
}

fn default_dr_margin_max() -> u32 {
    200
}

fn default_stagnation_threshold() -> usize {
    20
}

fn default_ripup_radius() -> i32 {
    1
}

fn default_lef_files() -> Vec<String> {
    vec!["inputs/simple.lef".to_string()]
}

fn default_def_file() -> String {
    "inputs/simple.def".to_string()
}

fn default_output_def() -> String {
    "output/placed.def".to_string()
}
