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
    /// Netlist repair results, when the run repaired the netlist.
    pub netlist: Option<NetlistSummary>,
    /// Timing repair results, when the run had constraints.
    pub timing: Option<TimingSummary>,
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

/// What netlist repair did.
#[derive(Debug, Default, Serialize)]
pub struct NetlistSummary {
    /// Buffers from synthesis removed before placement.
    pub buffers_removed: usize,
    /// Buffers added after input ports.
    pub input_buffers: usize,
    /// Buffers added before output ports.
    pub output_buffers: usize,
    /// The buffer cell the ports got.
    pub buffer_cell: String,
    /// Drivers whose own or whose loads' slew broke a limit after placement.
    pub slew_violations: usize,
    /// Drivers whose load broke their capacitance limit after placement.
    pub cap_violations: usize,
    /// Nets repair changed.
    pub repaired_nets: usize,
    /// Instances repair resized.
    pub resized: usize,
    /// Repeaters repair inserted.
    pub repeaters: usize,
}

/// What setup repair did, with slacks in nanoseconds.
#[derive(Debug, Default, Serialize)]
pub struct TimingSummary {
    /// Endpoints failing setup before repair.
    pub failing_before: usize,
    /// Endpoints failing setup after repair.
    pub failing_after: usize,
    /// Worst setup slack before repair.
    pub wns_before_ns: f64,
    /// Worst setup slack after repair.
    pub wns_after_ns: f64,
    /// Total negative setup slack before repair.
    pub tns_before_ns: f64,
    /// Total negative setup slack after repair.
    pub tns_after_ns: f64,
    /// Buffers repair removed.
    pub removed: usize,
    /// Gates repair resized.
    pub resized: usize,
    /// Buffers repair inserted.
    pub inserted: usize,
    /// Gates repair cloned.
    pub cloned: usize,
    /// Input pins repair swapped.
    pub swapped: usize,
    /// Repair iterations.
    pub iterations: usize,
    /// Cell area change, as a percentage.
    pub area_change_percent: f64,
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
    /// Timing-driven repairs made during global placement.
    pub timing_rounds: usize,
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
    /// Violations between routed metal inside a pin and the obstruction
    /// covering that pin, which the library makes unavoidable.
    pub pin_obstructions: usize,
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

pub use pare_common::util::memory::peak_rss_mb;
