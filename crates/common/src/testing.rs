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

/// A small generator for tests that try many inputs: the same seed gives
/// the same sequence on every machine, so a failure reproduces.
#[derive(Clone, Debug)]
pub struct SeededRng(u64);

impl SeededRng {
    /// The generator started from `seed`.
    #[must_use]
    pub const fn new(seed: u64) -> Self {
        Self(seed)
    }

    /// A number in `0..n`.
    ///
    /// # Panics
    ///
    /// When `n` is zero.
    pub const fn below(&mut self, n: u64) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 33) % n
    }

    /// One of `choices`.
    ///
    /// # Panics
    ///
    /// When `choices` is empty.
    pub fn pick<T: Copy>(&mut self, choices: &[T]) -> T {
        let n = u64::try_from(choices.len()).unwrap_or(u64::MAX);
        choices[usize::try_from(self.below(n)).unwrap_or(0)]
    }
}
