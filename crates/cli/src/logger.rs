//! Logging setup: `env_logger` writing PARE's styled lines to stderr, where
//! all of PARE's terminal output goes. Info messages read as plain output,
//! warnings and errors get a coloured prefix and debug and trace a dimmed
//! bracketed level.

use log::Level;
use std::io::Write;
use yansi::{Condition, Paint, Painted};

/// Whether to colour PARE's output: when stderr is a terminal and the
/// environment allows colours, whatever stdout is, as yansi's own example
/// builds it.
const STDERR_COLOR: Condition = Condition::from(|| {
    Condition::stderr_is_tty() && Condition::clicolor() && Condition::no_color()
});

/// Initializes logging at the info level unless `RUST_LOG` says otherwise,
/// and colours for every styled line PARE prints.
pub(crate) fn init() {
    yansi::whenever(STDERR_COLOR);
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .format(|buf, record| match level_prefix(record.level()) {
            Some(prefix) => writeln!(buf, "  {prefix} {}", record.args()),
            None => writeln!(buf, "  {}", record.args()),
        })
        .init();
}

/// The styled prefix of a line logged at `level`; none for info.
fn level_prefix(level: Level) -> Option<Painted<&'static str>> {
    match level {
        Level::Error => Some("ERROR".red().bold()),
        Level::Warn => Some("WARN".yellow().bold()),
        Level::Info => None,
        Level::Debug => Some("[DEBUG]".dim()),
        Level::Trace => Some("[TRACE]".dim()),
    }
}
