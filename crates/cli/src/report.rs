//! The record of a run that `--report` writes as JSON: what was run, how
//! long each phase took, what the checks found and how much memory it took.

use pare_common::util::ui;
use serde::Serialize;
use std::collections::BTreeMap;
use std::path::Path;
use std::time::Instant;

/// Everything a run reports.
#[derive(Debug, Default, Serialize)]
pub struct Report {
    /// Design name from the input.
    pub design: String,
    /// The config file the run used.
    pub config: String,
    /// Wall time of each phase, in order.
    pub phases: Vec<Phase>,
    /// Placement results, when the run placed.
    pub placement: Option<PlacementSummary>,
    /// Routing results, when the run routed.
    pub routing: Option<RoutingSummary>,
    /// What the checks found.
    pub checks: CheckSummary,
    /// Whether the run finished without an error and passed its checks.
    pub passed: bool,
    /// Wall time of the whole run in seconds.
    pub wall_seconds: f64,
    /// Peak resident memory of the process in megabytes, where known.
    pub peak_rss_mb: Option<u64>,
}

/// One timed phase.
#[derive(Debug, Serialize)]
pub struct Phase {
    /// What the phase did.
    pub name: String,
    /// Its wall time in seconds.
    pub seconds: f64,
}

/// What placement did.
#[derive(Debug, Default, Serialize)]
pub struct PlacementSummary {
    /// Movable cells placed.
    pub cells: usize,
    /// Nets the placer optimised.
    pub nets: usize,
    /// Global placement iterations run.
    pub iterations: usize,
    /// Density overflow where global placement stopped.
    pub overflow: f64,
    /// Half-perimeter wirelength after detailed placement, in microns.
    pub hpwl_um: f64,
    /// Moves detailed placement kept.
    pub detailed_moves: usize,
    /// Signal pins left where fixed metal blocks every via to them.
    pub blocked_pins: usize,
}

/// What routing did.
#[derive(Debug, Default, Serialize)]
pub struct RoutingSummary {
    /// Signal nets routed.
    pub nets: usize,
    /// Repair rounds run.
    pub rounds: usize,
    /// Nets with a pin no access point reaches.
    pub inaccessible: usize,
    /// Nets whose search failed to connect every pin.
    pub incomplete: usize,
    /// Nets still in conflict when routing stopped.
    pub conflicted: usize,
}

/// What the placement, connectivity and design rule checks found.
#[derive(Debug, Default, Serialize)]
pub struct CheckSummary {
    /// Placement violations.
    pub placement_violations: usize,
    /// Nets with disconnected pins.
    pub opens: usize,
    /// Total centerline wirelength in microns.
    pub wirelength_um: f64,
    /// Via instances.
    pub vias: usize,
    /// Design rule violations involving routed metal, by kind.
    pub violations: BTreeMap<String, usize>,
    /// Advisory violations, by kind.
    pub advisory: BTreeMap<String, usize>,
    /// Violations among fixed metal that placement takes part in.
    pub from_placement: usize,
    /// Violations among fixed metal the input alone makes.
    pub in_input: usize,
}

impl Report {
    /// Records the time since `start` as phase `name` and prints it.
    pub fn phase(&mut self, name: &str, start: Instant) {
        let seconds = start.elapsed().as_secs_f64();
        ui::phase_time(seconds);
        self.phases.push(Phase {
            name: name.to_string(),
            seconds,
        });
    }

    /// Writes the report as JSON to `path`.
    pub fn write(&self, path: &Path) -> anyhow::Result<()> {
        let text = serde_json::to_string_pretty(self)?;
        std::fs::write(path, text)?;
        Ok(())
    }
}

/// The peak resident memory of this process in megabytes, from the kernel's
/// accounting where it is available.
pub fn peak_rss_mb() -> Option<u64> {
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    vm_hwm_kb(&status).map(|kb| kb / 1024)
}

/// The `VmHWM` line of a `/proc/<pid>/status` text, in kilobytes.
fn vm_hwm_kb(status: &str) -> Option<u64> {
    status
        .lines()
        .find_map(|line| line.strip_prefix("VmHWM:"))
        .and_then(|rest| rest.split_whitespace().next())
        .and_then(|kb| kb.parse().ok())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_peak_memory_line_of_a_status_file_is_read_in_kilobytes() {
        let status = "Name:\tpare\nVmPeak:\t  123456 kB\nVmHWM:\t   98304 kB\nVmRSS:\t 1000 kB\n";

        assert_eq!(vm_hwm_kb(status), Some(98_304));
        assert_eq!(vm_hwm_kb("Name:\tpare\n"), None);
    }
}
