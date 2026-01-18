//! Logging Initialization.
//!
//! Sets up the logging infrastructure using env_logger with a default
//! log level of Info. This is called at program startup to enable
//! diagnostic output throughout the toolchain execution.

/// Initializes the logging system with default settings.
///
/// Configures env_logger to use Info level filtering and disables
/// timestamp output for cleaner log messages. This should be called
/// once at the beginning of the program before any log statements.
pub fn init() {
    env_logger::builder()
        .filter_level(log::LevelFilter::Info)
        .format_timestamp(None)
        .init();
}
