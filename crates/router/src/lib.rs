//! Global and detailed routing on the design's tracks.
//!
//! Global routing finds a corridor of gcells for every net; detailed routing
//! then draws wires on tracks and vias at track crossings inside those
//! corridors, negotiating with other nets until no two touch or come closer
//! than the layer spacing.

pub mod access;
pub mod tech;

/// Why routing could not run.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum RouteError {
    /// A configured layer name is not a routing layer.
    #[error("unknown routing layer '{0}'")]
    UnknownLayer(String),
    /// The minimum routing layer is above the maximum.
    #[error("the minimum routing layer is above the maximum")]
    EmptyLayerRange,
    /// A routable layer has no tracks and no pitch to derive them from.
    #[error("routing layer '{0}' has no tracks")]
    NoTracks(String),
    /// No usable via is defined through a cut layer.
    #[error("no via is defined through cut layer '{0}'")]
    NoVia(String),
}
