//! Performance Profiling Utilities.
//!
//! Provides scoped timing utilities for measuring execution time of
//! code sections. Uses RAII to automatically log elapsed time when
//! the timer goes out of scope.

use std::time::Instant;

/// A scoped timer that logs elapsed time when dropped.
///
/// Measures the time between creation and destruction, automatically
/// logging the elapsed duration with the provided name. Useful for
/// profiling individual functions or code blocks without manual timing
/// code.
pub struct ScopedTimer {
    name: &'static str,
    start: Instant,
}

impl ScopedTimer {
    /// Creates a new scoped timer with the given name.
    ///
    /// The timer starts immediately upon creation. When the timer is
    /// dropped (goes out of scope), it will log the elapsed time.
    pub fn new(name: &'static str) -> Self {
        Self {
            name,
            start: Instant::now(),
        }
    }
}

impl Drop for ScopedTimer {
    /// Logs the elapsed time when the timer is dropped.
    ///
    /// Automatically called when the ScopedTimer goes out of scope,
    /// providing a convenient way to measure function execution time
    /// without explicit cleanup code.
    fn drop(&mut self) {
        log::info!("{} took {:?}", self.name, self.start.elapsed());
    }
}
