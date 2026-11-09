//! Logging Initialization.
//!
//! Sets up the logging infrastructure using `env_logger` with styled output.
//! Info messages get a clean 2-space indent, warnings get a yellow prefix,
//! errors get a red prefix, and debug/trace use a bracketed format.

/// Initializes the logging system with styled formatting.
///
/// Filters at Info level unless `RUST_LOG` says otherwise, with clean,
/// minimal output formatting. Initializes `yansi` TTY detection
/// for stderr so colors are automatically disabled when piped.
pub fn init() {
    // Enable yansi colors only when stderr is a TTY
    yansi::whenever(yansi::Condition::TTY_AND_COLOR);

    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format_timestamp(None)
        .format(|_buf, record| {
            use std::io::Write;
            use yansi::Paint;
            match record.level() {
                log::Level::Error => {
                    writeln!(
                        std::io::stderr(),
                        "  {} {}",
                        "ERROR".red().bold(),
                        record.args()
                    )
                }
                log::Level::Warn => {
                    writeln!(
                        std::io::stderr(),
                        "  {} {}",
                        "WARN".yellow().bold(),
                        record.args()
                    )
                }
                log::Level::Info => {
                    writeln!(std::io::stderr(), "  {}", record.args())
                }
                log::Level::Debug | log::Level::Trace => {
                    writeln!(
                        std::io::stderr(),
                        "  {} {}",
                        format!("[{:5}]", record.level()).dim(),
                        record.args()
                    )
                }
            }
        })
        .init();
}
