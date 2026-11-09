//! The timed phases of a run: the title each shows as it starts and the
//! name the report records its time under.

use crate::report::{PhaseTime, Report};
use crate::ui;
use std::time::Instant;

/// A timed phase of a run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Phase {
    BufferRemoval,
    PortBuffering,
    GlobalPlacementWithoutIo,
    IoPinPlacement,
    GlobalPlacement,
    DesignRepair,
    Legalization,
    DetailedPlacement,
    ClockTreeSynthesis,
    LegalizationAfterClockTrees,
    TimingRepair,
    LegalizationAfterTimingRepair,
    PlacementOutput,
    GlobalRouting,
    DesignRepairOnGlobalRoutes,
    TimingRepairOnGlobalRoutes,
    LegalizationOnGlobalRoutes,
    Routing,
    Verification,
    RoutingOutput,
}

impl Phase {
    /// The title shown as the phase starts; none for a phase the section
    /// it opens already names.
    const fn title(self) -> Option<&'static str> {
        match self {
            Self::BufferRemoval => Some("Buffer Removal"),
            Self::PortBuffering => Some("Port Buffering"),
            Self::GlobalPlacementWithoutIo => Some("Global Placement Without I/O"),
            Self::IoPinPlacement => Some("I/O Pin Placement"),
            Self::GlobalPlacement | Self::Routing => None,
            Self::DesignRepair | Self::DesignRepairOnGlobalRoutes => Some("Design Repair"),
            Self::Legalization
            | Self::LegalizationAfterClockTrees
            | Self::LegalizationAfterTimingRepair
            | Self::LegalizationOnGlobalRoutes => Some("Legalization"),
            Self::DetailedPlacement => Some("Detailed Placement"),
            Self::ClockTreeSynthesis => Some("Clock Tree Synthesis"),
            Self::TimingRepair | Self::TimingRepairOnGlobalRoutes => Some("Timing Repair"),
            Self::PlacementOutput | Self::RoutingOutput => Some("Output"),
            Self::GlobalRouting => Some("Global Route"),
            Self::Verification => Some("Verification"),
        }
    }

    /// The name the report records the phase's time under.
    const fn name(self) -> &'static str {
        match self {
            Self::BufferRemoval => "buffer removal",
            Self::PortBuffering => "port buffering",
            Self::GlobalPlacementWithoutIo => "global placement without I/O",
            Self::IoPinPlacement => "I/O pin placement",
            Self::GlobalPlacement => "global placement",
            Self::DesignRepair => "design repair",
            Self::Legalization => "legalization",
            Self::DetailedPlacement => "detailed placement",
            Self::ClockTreeSynthesis => "clock tree synthesis",
            Self::LegalizationAfterClockTrees => "legalization after clock tree synthesis",
            Self::TimingRepair => "timing repair",
            Self::LegalizationAfterTimingRepair => "legalization after timing repair",
            Self::PlacementOutput => "placement output",
            Self::GlobalRouting => "global routing",
            Self::DesignRepairOnGlobalRoutes => "design repair on global routes",
            Self::TimingRepairOnGlobalRoutes => "timing repair on global routes",
            Self::LegalizationOnGlobalRoutes => "legalization on global routes",
            Self::Routing => "routing",
            Self::Verification => "verification",
            Self::RoutingOutput => "routing output",
        }
    }
}

/// A phase under way.
#[must_use = "a phase is recorded only when it is finished"]
pub(crate) struct PhaseTimer {
    phase: Phase,
    start: Instant,
}

impl PhaseTimer {
    /// Starts `phase`, showing its title.
    pub(crate) fn start(phase: Phase) -> Self {
        if let Some(title) = phase.title() {
            ui::phase(title);
        }
        Self {
            phase,
            start: Instant::now(),
        }
    }

    /// Ends the phase, showing how long it took and recording that in
    /// `report`.
    pub(crate) fn finish(self, report: &mut Report) {
        let elapsed = self.start.elapsed();
        ui::phase_time(elapsed);
        report.phases.push(PhaseTime {
            name: self.phase.name(),
            elapsed,
        });
    }
}
