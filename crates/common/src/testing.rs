//! Helpers for the workspace's tests, built only with the `testing`
//! feature.

use std::path::Path;

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
