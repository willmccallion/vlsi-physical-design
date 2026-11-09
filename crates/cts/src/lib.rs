//! Clock tree synthesis, as `OpenROAD`'s `TritonCTS`: clock sinks are
//! clustered under leaf buffers, an H-tree of buffered wire segments taken
//! from a characterization of the clock wire reaches the clusters, and the
//! tree is written into the design with dummy loads balancing its levels.

use pare_common::db::library::DuplicateName;
use pare_common::units::Seconds;

mod buffers;
mod capacitance;
mod clock;
#[cfg(test)]
mod fixtures;
mod htree;
mod kmeans;
mod latency;
mod options;
mod sink_clustering;
mod techchar;
mod triton;

pub use latency::balance_latencies;
pub use options::{CtsOptions, NdrStrategy};
pub use triton::{ClockTrees, CtsInput, CtsSummary, synthesize};

/// Why a clock tree could not be built.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum CtsError {
    /// No usable buffer was found for the clock tree.
    #[error("no clock buffers")]
    NoBuffers,
    /// A clock buffer's master has no pin for one of its Liberty ports.
    #[error("the master of cell '{0}' has no pin for its input or output")]
    MissingPin(String),
    /// A clock buffer has no delay arc from its input to its output.
    #[error("cell '{0}' has no delay arc from its input to its output")]
    NoDelayArc(String),
    /// The root buffer has no rising delay to balance latencies with.
    #[error("cell '{0}' has no rising delay arc to balance clock latencies with")]
    NoRisingDelay(String),
    /// A clock buffer has no physical master.
    #[error("cell '{0}' has no master")]
    NoMaster(String),
    /// A clock buffer has no load limit.
    #[error("no max capacitance found for cell '{0}'")]
    NoMaxCapacitance(String),
    /// A clock buffer has no slew limit.
    #[error("no max slew found for cell '{0}'")]
    NoMaxSlew(String),
    /// A clock buffer's delay tables lack slew or load axes.
    #[error("the delay tables of cell '{0}' have no slew or load axis")]
    NoTableAxes(String),
    /// No characterized segment keeps its slew within the characterization
    /// buffer's limit.
    #[error(
        "no characterized segment of '{buffer}' keeps its slew within {} ns",
        .max_transition.nanos()
    )]
    NoSegmentWithinSlewLimit {
        /// The characterization buffer.
        buffer: String,
        /// Its slew limit.
        max_transition: Seconds,
    },
    /// The characterization buffer's master has no height to measure the
    /// characterized wires by.
    #[error("the characterization buffer '{0}' has no height")]
    ZeroHeightBuffer(String),
    /// A normalized characterization value overflows the table's fields.
    #[error("normalized characterization values must lie in [1, {0}]")]
    CharacterizationRange(u32),
    /// Two clocks enter on the same net.
    #[error("clock '{0}' overlaps a previous clock")]
    OverlappingClock(String),
    /// A cell or net to add has the name of one already there.
    #[error("cannot edit the design: {0}")]
    Edit(#[from] DuplicateName),
}
