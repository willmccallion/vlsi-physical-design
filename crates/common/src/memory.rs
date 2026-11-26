//! This process's resident memory, from the kernel's accounting where it is
//! available (`/proc/self/status` on Linux).

/// The peak resident memory of this process in megabytes.
pub fn peak_rss_mb() -> Option<u64> {
    status_kb("VmHWM:").map(|kb| kb / 1024)
}

/// The resident memory of this process now, in megabytes.
pub fn current_rss_mb() -> Option<u64> {
    status_kb("VmRSS:").map(|kb| kb / 1024)
}

fn status_kb(key: &str) -> Option<u64> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    field_kb(&status, key)
}

/// The value of the `key` line of a `/proc/<pid>/status` text, in
/// kilobytes.
fn field_kb(status: &str, key: &str) -> Option<u64> {
    status
        .lines()
        .find_map(|line| line.strip_prefix(key))
        .and_then(|rest| rest.split_whitespace().next())
        .and_then(|kb| kb.parse().ok())
}

#[cfg(test)]
mod tests;
