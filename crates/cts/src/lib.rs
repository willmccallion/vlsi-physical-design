//! Clock tree synthesis, as `OpenROAD`'s `TritonCTS`: clock sinks are
//! clustered under leaf buffers, an H-tree of buffered wire segments taken
//! from a characterization of the clock wire reaches the clusters, and the
//! tree is written into the design with dummy loads balancing its levels.

pub mod buffers;
mod clock;
mod htree;
mod kmeans;
mod latency;
pub mod options;
mod sink_clustering;
pub mod techchar;
mod triton;

pub use latency::balance_latencies;
pub use options::{CtsOptions, NdrStrategy};
pub use triton::{ClockTrees, CtsInput, CtsSummary, synthesize};

/// Why a clock tree could not be built.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum CtsError {
    /// No usable buffer was found for the clock tree.
    #[error("no clock buffers")]
    NoBuffers,
    /// A cell named as a clock buffer is not a buffer.
    #[error("cell '{0}' is not a buffer")]
    NotABuffer(String),
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
    /// A normalized characterization value overflows the table's fields.
    #[error("normalized characterization values must lie in [1, {0}]")]
    CharacterizationRange(u32),
    /// Two clocks enter on the same net.
    #[error("clock '{0}' overlaps a previous clock")]
    OverlappingClock(String),
    /// An edit of the design failed.
    #[error("cannot edit the design: {0}")]
    Edit(String),
}
