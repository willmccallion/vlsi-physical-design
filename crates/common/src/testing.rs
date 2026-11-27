//! Helpers for the workspace's tests, built only with the `testing`
//! feature.

use std::cell::RefCell;
use std::path::Path;
use std::sync::Once;

/// The variable that makes [`assert_golden`] write its files instead of
/// checking against them.
pub const BLESS: &str = "PARE_BLESS";

/// Checks `actual` against the golden file `name` in `dir`, or writes it
/// there when [`BLESS`] is set, for pinning down whole results.
///
/// # Panics
///
/// When the file is missing or differs from `actual`, naming the first line
/// that does.
pub fn assert_golden(dir: &Path, name: &str, actual: &str) {
    let path = dir.join(name);
    if std::env::var_os(BLESS).is_some() {
        if let Err(e) = std::fs::create_dir_all(dir).and_then(|()| std::fs::write(&path, actual)) {
            panic!("{}: {e}", path.display());
        }
        return;
    }
    let expected = match std::fs::read_to_string(&path) {
        Ok(text) => text,
        Err(e) => panic!("{}: {e}; {BLESS}=1 writes it", path.display()),
    };
    if let Some((line, (want, got))) = expected
        .lines()
        .zip(actual.lines())
        .enumerate()
        .find(|(_, (want, got))| want != got)
    {
        panic!(
            "{} differs at line {}:\n  golden: {want}\n  now:    {got}",
            path.display(),
            line + 1
        );
    }
    assert_eq!(
        expected.lines().count(),
        actual.lines().count(),
        "{} differs in length",
        path.display()
    );
}

thread_local! {
    /// The lines logged on this thread while [`capture_logs`] runs.
    static CAPTURED: RefCell<Option<Vec<String>>> = const { RefCell::new(None) };
}

/// A logger keeping what each thread logs while it captures.
struct CaptureLogger;

impl log::Log for CaptureLogger {
    fn enabled(&self, _metadata: &log::Metadata<'_>) -> bool {
        CAPTURED.with(|captured| captured.borrow().is_some())
    }

    fn log(&self, record: &log::Record<'_>) {
        CAPTURED.with(|captured| {
            if let Some(lines) = captured.borrow_mut().as_mut() {
                lines.push(format!("{}: {}", record.level(), record.args()));
            }
        });
    }

    fn flush(&self) {}
}

/// The lines `run` logs on this thread, each as `LEVEL: message`; what
/// other threads log is not kept.
///
/// # Panics
///
/// When a logger other than this capture is installed.
pub fn capture_logs(run: impl FnOnce()) -> Vec<String> {
    static INSTALL: Once = Once::new();
    INSTALL.call_once(|| {
        assert!(
            log::set_logger(&CaptureLogger).is_ok(),
            "another logger is installed, so log lines cannot be captured"
        );
        log::set_max_level(log::LevelFilter::Trace);
    });
    CAPTURED.with(|captured| *captured.borrow_mut() = Some(Vec::new()));
    run();
    CAPTURED.with(|captured| captured.borrow_mut().take().unwrap_or_default())
}
