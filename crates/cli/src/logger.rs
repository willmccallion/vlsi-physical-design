//! Logging Initialization.
//!
//! Sets up the logging infrastructure using `env_logger` with styled output.
//! Info messages get a clean 2-space indent, warnings get a yellow prefix,
//! errors get a red prefix, and debug/trace use a bracketed format.

/// Whether to colour PARE's output, all of which goes to stderr: when
/// stderr is a terminal and the environment allows colours, whatever stdout
/// is, as yansi's own example builds it.
const STDERR_COLOR: yansi::Condition = yansi::Condition::from(|| {
    yansi::Condition::stderr_is_tty()
        && yansi::Condition::clicolor()
        && yansi::Condition::no_color()
});

/// Initializes the logging system with styled formatting.
///
/// Filters at Info level unless `RUST_LOG` says otherwise, with clean,
/// minimal output formatting, and colours output only when stderr is a
/// terminal.
pub fn init() {
    yansi::whenever(STDERR_COLOR);

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
