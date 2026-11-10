//! The record of a run that `--report` writes as JSON: what was run, how
//! long each phase took, what the checks found and how much memory it took.

use anyhow::Context;
use pare_common::verify::drc::{DrcReport, Violation};
use pare_common::verify::routing::RoutingReport;
use pare_resizer::{HoldSummary, RepairSummary, SetupSummary};
use serde::{Serialize, Serializer};
use std::collections::BTreeMap;
use std::path::Path;
use std::time::Duration;

/// Everything a run reports.
#[derive(Debug, Default, Serialize)]
pub(crate) struct Report {
    /// Design name from the input.
    pub(crate) design: String,
    /// The config file the run used.
    pub(crate) config: String,
    /// Wall time of each phase, in order.
    pub(crate) phases: Vec<PhaseTime>,
    /// Netlist repair results, when the run repaired the netlist.
    pub(crate) netlist: Option<NetlistSummary>,
    /// Clock tree synthesis results, when the run built clock trees.
    pub(crate) clock_tree: Option<ClockTreeSummary>,
    /// Timing repair results, when the run had constraints.
    pub(crate) timing: Option<TimingSummary>,
    /// Repair results on the global routes' parasitics, when the run
    /// repaired after global routing.
    pub(crate) global_route_repair: Option<GlobalRouteRepairSummary>,
    /// Placement results, when the run placed.
    pub(crate) placement: Option<PlacementSummary>,
    /// Routing results, when the run routed.
    pub(crate) routing: Option<RoutingSummary>,
    /// What the checks found.
    pub(crate) checks: CheckSummary,
    /// Whether the run finished without an error and passed its checks.
    pub(crate) passed: bool,
    /// Wall time of the whole run.
    #[serde(rename = "wall_seconds", serialize_with = "as_seconds")]
    pub(crate) wall: Duration,
    /// Peak resident memory of the process in megabytes, where known.
    pub(crate) peak_rss_mb: Option<u64>,
}

/// The wall time of one phase.
#[derive(Debug, Serialize)]
pub(crate) struct PhaseTime {
    /// What the phase did.
    pub(crate) name: &'static str,
    /// Its wall time.
    #[serde(rename = "seconds", serialize_with = "as_seconds")]
    pub(crate) elapsed: Duration,
}

/// Writes a wall time as seconds, the unit the keys name.
fn as_seconds<S: Serializer>(time: &Duration, serializer: S) -> Result<S::Ok, S::Error> {
    serializer.serialize_f64(time.as_secs_f64())
}

/// What netlist repair did.
#[derive(Debug, Default, Serialize)]
pub(crate) struct NetlistSummary {
    /// Buffers from synthesis removed before placement.
    pub(crate) buffers_removed: usize,
    /// Buffers added after input ports.
    pub(crate) input_buffers: usize,
    /// Buffers added before output ports.
    pub(crate) output_buffers: usize,
    /// The buffer cell the ports got.
    pub(crate) buffer_cell: String,
    /// What design repair after placement did.
    #[serde(flatten)]
    pub(crate) design: DesignRepairSummary,
}

/// What design repair did.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub(crate) struct DesignRepairSummary {
    /// Drivers whose own or whose loads' slew broke a limit.
    pub(crate) slew_violations: usize,
    /// Drivers whose load broke their capacitance limit.
    pub(crate) cap_violations: usize,
    /// Nets repair changed.
    pub(crate) repaired_nets: usize,
    /// Instances repair resized.
    pub(crate) resized: usize,
    /// Repeaters repair inserted.
    pub(crate) repeaters: usize,
}

impl DesignRepairSummary {
    /// What the repair `done` reports, for the report.
    pub(crate) const fn of(done: &RepairSummary) -> Self {
        Self {
            slew_violations: done.slew_violations,
            cap_violations: done.cap_violations,
            repaired_nets: done.repaired_nets,
            resized: done.resized,
            repeaters: done.inserted,
        }
    }
}

/// What clock tree synthesis did.
#[derive(Debug, Default, Serialize)]
pub(crate) struct ClockTreeSummary {
    /// Clock nets a tree was built for.
    pub(crate) trees: usize,
    /// Sinks the trees reach.
    pub(crate) sinks: usize,
    /// Clock buffers inserted.
    pub(crate) buffers: usize,
    /// Dummy loads inserted.
    pub(crate) dummies: usize,
    /// Clock gates cloned.
    pub(crate) cloned_gates: usize,
    /// Clock inverters cloned at their loads.
    pub(crate) cloned_inverters: usize,
    /// Repeaters inserted on long clock nets.
    pub(crate) clock_net_repeaters: usize,
    /// Delay buffers inserted to balance the trees' latencies.
    pub(crate) delay_buffers: usize,
}

/// What design and timing repair did on the global routes' parasitics.
#[derive(Debug, Default, Serialize)]
pub(crate) struct GlobalRouteRepairSummary {
    /// What design repair did.
    #[serde(flatten)]
    pub(crate) design: DesignRepairSummary,
    /// What timing repair did.
    pub(crate) timing: Option<TimingSummary>,
}

/// What setup repair did, with slacks in nanoseconds.
#[derive(Debug, Default, Serialize)]
pub(crate) struct TimingSummary {
    /// Endpoints failing setup before repair.
    pub(crate) failing_before: usize,
    /// Endpoints failing setup after repair.
    pub(crate) failing_after: usize,
    /// Worst setup slack before repair.
    pub(crate) wns_before_ns: f64,
    /// Worst setup slack after repair.
    pub(crate) wns_after_ns: f64,
    /// Total negative setup slack before repair.
    pub(crate) tns_before_ns: f64,
    /// Total negative setup slack after repair.
    pub(crate) tns_after_ns: f64,
    /// Buffers repair removed.
    pub(crate) removed: usize,
    /// Gates repair resized.
    pub(crate) resized: usize,
    /// Buffers repair inserted.
    pub(crate) inserted: usize,
    /// Gates repair cloned.
    pub(crate) cloned: usize,
    /// Input pins repair swapped.
    pub(crate) swapped: usize,
    /// Repair iterations.
    pub(crate) iterations: usize,
    /// Cell area change, as a percentage.
    pub(crate) area_change_percent: f64,
    /// Endpoints failing hold before hold repair.
    pub(crate) hold_failing_before: usize,
    /// Endpoints failing hold after it.
    pub(crate) hold_failing_after: usize,
    /// Worst hold slack before hold repair.
    pub(crate) hold_wns_before_ns: f64,
    /// Worst hold slack after it.
    pub(crate) hold_wns_after_ns: f64,
    /// Hold buffers inserted.
    pub(crate) hold_buffers: usize,
}

impl TimingSummary {
    /// What `setup` and `hold` repair did, for the report.
    pub(crate) const fn of(setup: &SetupSummary, hold: &HoldSummary) -> Self {
        Self {
            failing_before: setup.failing_before,
            failing_after: setup.failing_after,
            wns_before_ns: setup.wns_before.nanos(),
            wns_after_ns: setup.wns_after.nanos(),
            tns_before_ns: setup.tns_before.nanos(),
            tns_after_ns: setup.tns_after.nanos(),
            removed: setup.removed,
            resized: setup.resized,
            inserted: setup.inserted,
            cloned: setup.cloned,
            swapped: setup.swapped,
            iterations: setup.iterations,
            area_change_percent: setup.area_change_percent,
            hold_failing_before: hold.failing_before,
            hold_failing_after: hold.failing_after,
            hold_wns_before_ns: hold.wns_before.nanos(),
            hold_wns_after_ns: hold.wns_after.nanos(),
            hold_buffers: hold.inserted,
        }
    }
}

/// What placement did.
#[derive(Debug, Serialize)]
pub(crate) struct PlacementSummary {
    /// What global placement did, when the run placed from scratch.
    pub(crate) global: Option<GlobalPlacementSummary>,
    /// Signal pins left where fixed metal blocks every via to them, when
    /// vias reach the lowest signal layer.
    pub(crate) blocked_pins: Option<usize>,
    /// Cells detailed placement moved or turned.
    pub(crate) detailed_moves: usize,
    /// Half-perimeter wirelength after detailed placement, in microns.
    pub(crate) detailed_hpwl_um: f64,
    /// Half-perimeter wirelength of the placement written out, after the
    /// clock trees, timing repair and legalization that follow detailed
    /// placement, in microns.
    pub(crate) final_hpwl_um: f64,
}

/// What global placement did.
#[derive(Debug, Serialize)]
pub(crate) struct GlobalPlacementSummary {
    /// Movable cells placed.
    pub(crate) cells: usize,
    /// Nets the placer optimised.
    pub(crate) nets: usize,
    /// Iterations run.
    pub(crate) iterations: usize,
    /// Density overflow where it stopped.
    pub(crate) overflow: f64,
    /// Half-perimeter wirelength where it stopped, in microns.
    pub(crate) hpwl_um: f64,
    /// Timing-driven repairs made along the way.
    pub(crate) timing_rounds: usize,
}

/// What routing did.
#[derive(Debug, Default, Serialize)]
pub(crate) struct RoutingSummary {
    /// Signal nets routed.
    pub(crate) nets: usize,
    /// Repair rounds run.
    pub(crate) rounds: usize,
    /// Nets with a pin no access point reaches.
    pub(crate) inaccessible: usize,
    /// Nets whose search failed to connect every pin.
    pub(crate) incomplete: usize,
    /// Nets still in conflict when routing stopped.
    pub(crate) conflicted: usize,
}

/// What the placement, connectivity and design rule checks found.
#[derive(Debug, Default, Serialize)]
pub(crate) struct CheckSummary {
    /// Placement violations.
    pub(crate) placement_violations: usize,
    /// Nets with disconnected pins.
    pub(crate) opens: usize,
    /// Total centerline wirelength in microns.
    pub(crate) wirelength_um: f64,
    /// Via instances.
    pub(crate) vias: usize,
    /// Design rule violations involving routed metal, by kind.
    pub(crate) violations: BTreeMap<String, usize>,
    /// Advisory violations, by kind.
    pub(crate) advisory: BTreeMap<String, usize>,
    /// Violations between routed metal inside a pin and the obstruction
    /// covering that pin, which the library makes unavoidable.
    pub(crate) pin_obstructions: usize,
    /// Violations among fixed metal that placement takes part in.
    pub(crate) from_placement: usize,
    /// Violations among fixed metal the input alone makes.
    pub(crate) in_input: usize,
}

impl CheckSummary {
    /// Records what the connectivity and design rule checks found, with
    /// the routing's total wirelength in microns.
    pub(crate) fn record_routing(
        &mut self,
        connectivity: &RoutingReport,
        drc: &DrcReport,
        wirelength_um: f64,
    ) {
        let by_kind = |counts: Vec<(Violation, usize)>| {
            counts
                .into_iter()
                .map(|(kind, n)| (kind.name().to_string(), n))
                .collect()
        };
        self.opens = connectivity.opens.len();
        self.wirelength_um = wirelength_um;
        self.vias = connectivity.stats.vias;
        self.violations = by_kind(drc.counts());
        self.advisory = by_kind(drc.advisory_counts());
        self.pin_obstructions = drc.pin_obstruction.len();
        self.from_placement = drc.placement.len();
        self.in_input = drc.input.len();
    }
}

impl Report {
    /// Writes the report as JSON to `path`.
    pub(crate) fn write(&self, path: &Path) -> anyhow::Result<()> {
        let text = serde_json::to_string_pretty(self)?;
        std::fs::write(path, text)
            .with_context(|| format!("cannot write report {}", path.display()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn design_repair_reports_as_fields_of_its_summary() {
        let summary = NetlistSummary {
            buffers_removed: 1,
            design: DesignRepairSummary {
                slew_violations: 2,
                repeaters: 3,
                ..DesignRepairSummary::default()
            },
            ..NetlistSummary::default()
        };

        let json = serde_json::to_string(&summary).unwrap();

        assert_eq!(
            json,
            "{\"buffers_removed\":1,\"input_buffers\":0,\"output_buffers\":0,\"buffer_cell\":\"\",\
             \"slew_violations\":2,\"cap_violations\":0,\"repaired_nets\":0,\"resized\":0,\
             \"repeaters\":3}"
        );
    }

    /// The keys of a report `scripts/compare.py` reads, by path.
    const COMPARED_KEYS: [&str; 5] = [
        "/netlist",
        "/checks/wirelength_um",
        "/checks/vias",
        "/checks/violations",
        "/checks/opens",
    ];

    #[test]
    fn a_report_has_the_keys_compare_reads() {
        let report = Report {
            netlist: Some(NetlistSummary::default()),
            ..Report::default()
        };

        let json = serde_json::to_value(&report).unwrap();

        for key in COMPARED_KEYS {
            assert!(json.pointer(key).is_some(), "no {key} in {json}");
        }
    }

    #[test]
    fn wall_times_report_as_seconds() {
        let report = Report {
            phases: vec![PhaseTime {
                name: "routing",
                elapsed: Duration::from_millis(1500),
            }],
            wall: Duration::from_millis(2250),
            ..Report::default()
        };

        let json = serde_json::to_value(&report).unwrap();

        assert_eq!(json["phases"][0]["seconds"], 1.5);
        assert_eq!(json["wall_seconds"], 2.25);
    }
}
