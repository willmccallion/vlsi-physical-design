//! Utility Modules for Configuration, Logging, and Verification.
//!
//! This module provides supporting functionality for the EDA toolchain including
//! configuration management, logging setup, design verification (DRC/LVS),
//! benchmark generation, performance profiling, and visualization utilities.

pub mod check;
/// Structured CLI output with colored formatting.
///
/// Provides presentation functions for banners, section headers, stat rows,
/// check marks, progress bars, and file-wrote lines. All output goes to stderr
/// with automatic TTY color detection via `yansi`.
pub mod ui;
/// Configuration management for algorithm parameters.
///
/// Defines the configuration structure that controls algorithm behavior across
/// placement and routing. Parameters are loaded from TOML files with sensible
/// defaults for all settings. Groups configuration into logical sections:
/// global placement, legalization, global routing, detailed routing, and input
/// file paths. All sections have default implementations that provide reasonable
/// starting values for typical designs.
pub mod config;
/// Synthetic benchmark generator for testing and evaluation.
///
/// Generates random DEF files with specified cell counts, net counts, and target
/// utilization for testing placement and routing algorithms. Creates designs
/// using a chain topology where each net connects one cell's output to the next
/// cell's input. The die size is computed to accommodate the cell area at the
/// target utilization, enabling controlled experiments with varying design
/// characteristics.
pub mod generator;
/// Logging infrastructure initialization.
///
/// Sets up the logging system using env_logger with default Info level filtering.
/// Configures timestamp output format and log level filtering. This should be
/// called once at program startup before any log statements to enable diagnostic
/// output throughout the toolchain execution.
pub mod logger;
/// Performance profiling utilities for algorithm analysis.
///
/// Provides timing and performance measurement tools for profiling placement and
/// routing algorithms. Tracks execution time of algorithm phases, memory usage,
/// and convergence metrics. Used for performance analysis and optimization of
/// the EDA toolchain.
pub mod profiler;
/// Visualization utilities for debugging and analysis.
///
/// Generates visual representations of placement and routing results including
/// congestion heatmaps, cell placement diagrams, and routing path visualizations.
/// Outputs images in standard formats (PNG) for debugging routing congestion,
/// placement quality, and algorithm convergence. These visualizations are critical
/// for understanding algorithm behavior and diagnosing routing failures.
pub mod visualization;
