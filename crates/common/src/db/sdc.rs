//! Timing constraints as an SDC file states them.
//!
//! The clocks, the delays outside the design at its ports, the uncertainty
//! of each clock, the exceptions that relax or drop paths, derating and
//! design-wide limits. Times are in seconds and capacitances in farads, converted from the
//! library's units, which `OpenSTA` reads constraints in whatever
//! `set_units` says.

use crate::db::indices::{CellId, IdVec, IoPinId};
use crate::db::liberty::{Edge, EdgeValues, MinMax, SideValues};
use crate::units::{Farads, Seconds};
use std::collections::HashMap;

crate::define_index!(
    /// A clock in [`Sdc::clocks`].
    ClockId(u32)
);

/// One edge of a clock's waveform.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ClockEdge {
    /// The clock whose waveform the edge is of.
    pub clock: ClockId,
    /// Which edge.
    pub edge: Edge,
}

/// Times for each edge and each side of the delay range: the shape of a
/// port delay or an input slew, which the commands set piecewise.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RiseFallMinMax {
    values: EdgeValues<SideValues<Option<Seconds>>>,
}

impl RiseFallMinMax {
    /// The value for `edge` on `side`, if set.
    #[must_use]
    pub fn get(&self, edge: Edge, side: MinMax) -> Option<Seconds> {
        self.values[edge][side]
    }

    /// Sets the value for every edge in `edges` and every side in `sides`.
    pub fn set(&mut self, edges: &[Edge], sides: &[MinMax], value: Seconds) {
        for &edge in edges {
            for &side in sides {
                self.values[edge][side] = Some(value);
            }
        }
    }

    /// Keeps the more pessimistic of the value held and `value`: the
    /// larger on the late side, the smaller on the early side, as
    /// `OpenSTA` merges an added delay.
    pub fn merge(&mut self, edges: &[Edge], sides: &[MinMax], value: Seconds) {
        for &edge in edges {
            for &side in sides {
                let slot = &mut self.values[edge][side];
                *slot = Some(match *slot {
                    Some(held) if !side.beats(value, held) => held,
                    _ => value,
                });
            }
        }
    }

    /// Whether any value is set.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        Edge::BOTH.iter().all(|&edge| {
            MinMax::BOTH
                .iter()
                .all(|&side| self.values[edge][side].is_none())
        })
    }
}

/// A clock entering the design.
#[derive(Clone, Debug, PartialEq)]
pub struct Clock {
    /// The name constraints refer to it by.
    pub name: String,
    /// The time from one rising edge to the next.
    pub period: Seconds,
    /// When within the period it rises.
    pub rise: Seconds,
    /// When within the period it falls.
    pub fall: Seconds,
    /// The ports it enters at.
    pub sources: Vec<IoPinId>,
    /// Setup (late) and hold (early) uncertainty, where set.
    pub uncertainty: SideValues<Option<Seconds>>,
    /// Whether it arrives through its network's delays rather than
    /// everywhere at once, as `set_propagated_clock` makes it.
    pub propagated: bool,
}

impl Clock {
    /// When the clock's `edge` happens within its first period.
    #[must_use]
    pub const fn edge_time(&self, edge: Edge) -> Seconds {
        match edge {
            Edge::Rise => self.rise,
            Edge::Fall => self.fall,
        }
    }

    /// The uncertainty applied to checks on `side`; zero where none is set.
    #[must_use]
    pub fn uncertainty(&self, side: MinMax) -> Seconds {
        self.uncertainty[side].unwrap_or(Seconds::ZERO)
    }
}

/// A delay outside the design at one of its ports, relative to a clock
/// edge: the arrival at an input, or the time an output must be settled
/// before the capturing edge.
#[derive(Clone, Debug, PartialEq)]
pub struct PortDelay {
    /// The clock edge the delay is measured from.
    pub reference: ClockEdge,
    /// The delay for each edge at the port and each side.
    pub delays: RiseFallMinMax,
}

/// Which paths an exception names: those launched by the given clocks or
/// ports and captured by the given clocks or ports; an empty list leaves
/// that end unconstrained.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PathSpec {
    /// Launching clocks.
    pub from_clocks: Vec<ClockId>,
    /// Launching ports.
    pub from_ports: Vec<IoPinId>,
    /// Capturing clocks.
    pub to_clocks: Vec<ClockId>,
    /// Capturing ports.
    pub to_ports: Vec<IoPinId>,
}

impl PathSpec {
    /// Whether a path launched by `from` into `to`, captured by
    /// `to_clock`, is one the exception names.
    #[must_use]
    pub fn matches(&self, from: PathEnd, to: PathEnd, to_clock: Option<ClockId>) -> bool {
        let from_ok = (self.from_clocks.is_empty() && self.from_ports.is_empty())
            || match from {
                PathEnd::Clock(clock) => self.from_clocks.contains(&clock),
                PathEnd::Port(port) => self.from_ports.contains(&port),
            };
        let to_ok = (self.to_clocks.is_empty() && self.to_ports.is_empty())
            || match to {
                PathEnd::Clock(clock) => self.to_clocks.contains(&clock),
                PathEnd::Port(port) => {
                    self.to_ports.contains(&port)
                        || to_clock.is_some_and(|c| self.to_clocks.contains(&c))
                }
            };
        from_ok && to_ok
    }
}

/// Where a path starts or ends, as exceptions name it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PathEnd {
    /// A register clocked by this clock, or the clock itself.
    Clock(ClockId),
    /// A port of the design.
    Port(IoPinId),
}

/// Paths timing ignores.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FalsePath {
    /// Which checks it drops; none for both.
    pub side: Option<MinMax>,
    /// The paths it applies to.
    pub paths: PathSpec,
}

/// Paths allowed more or fewer cycles than the default.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MultiCyclePath {
    /// Which checks it changes; none for both, as `OpenSTA` reads a bare
    /// `set_multicycle_path`.
    pub side: Option<MinMax>,
    /// Cycles the path may take.
    pub multiplier: i32,
    /// Whether the cycles are counted in the capturing clock's period
    /// rather than the launching clock's.
    pub use_end_clock: bool,
    /// The paths it applies to.
    pub paths: PathSpec,
}

impl MultiCyclePath {
    /// The multiplier `OpenSTA` applies on `side`: zero for the early side
    /// of a path that names neither setup nor hold.
    #[must_use]
    pub const fn multiplier(&self, side: MinMax) -> i32 {
        if self.side.is_none() && matches!(side, MinMax::Min) {
            0
        } else {
            self.multiplier
        }
    }
}

/// Factors delays are scaled by, as `set_timing_derate` sets them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Derating {
    /// Cell delay factors for the early and late sides.
    pub cell: SideValues<f64>,
    /// Wire delay factors for the early and late sides.
    pub net: SideValues<f64>,
}

impl Default for Derating {
    fn default() -> Self {
        Self {
            cell: SideValues::both(1.0),
            net: SideValues::both(1.0),
        }
    }
}

/// The cell delay factors `set_timing_derate` sets on one instance, for
/// the early and late sides, where set.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CellDerating {
    /// The factor on each side, if set.
    pub cell: SideValues<Option<f64>>,
}

/// Limits applied to every pin of the design.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DesignLimits {
    /// Largest fanout a driver may have.
    pub max_fanout: Option<f64>,
    /// Largest transition time.
    pub max_transition: Option<Seconds>,
    /// Largest load.
    pub max_capacitance: Option<Farads>,
}

/// The constraints of a design.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Sdc {
    /// The clocks, in the order created.
    pub clocks: IdVec<ClockId, Clock>,
    /// Arrivals outside the design at its input ports.
    pub input_delays: HashMap<IoPinId, Vec<PortDelay>>,
    /// Delays outside the design after its output ports.
    pub output_delays: HashMap<IoPinId, Vec<PortDelay>>,
    /// Paths dropped from timing.
    pub false_paths: Vec<FalsePath>,
    /// Paths given other cycle counts.
    pub multicycle_paths: Vec<MultiCyclePath>,
    /// Delay derating of the whole design.
    pub derating: Derating,
    /// Delay derating of single instances, over the design's.
    pub cell_derates: HashMap<CellId, CellDerating>,
    /// Design-wide limits.
    pub limits: DesignLimits,
    /// Capacitance outside the design on its output ports.
    pub port_loads: HashMap<IoPinId, Farads>,
    /// Transition times at input ports.
    pub input_slews: HashMap<IoPinId, RiseFallMinMax>,
}

impl Sdc {
    /// The clock named `name`.
    #[must_use]
    pub(crate) fn clock_id(&self, name: &str) -> Option<ClockId> {
        self.clocks
            .iter_enumerated()
            .find(|(_, c)| c.name == name)
            .map(|(id, _)| id)
    }

    /// The clock with this id.
    #[must_use]
    pub fn clock(&self, id: ClockId) -> &Clock {
        &self.clocks[id]
    }

    /// When `edge` happens within its clock's first period.
    #[must_use]
    pub fn edge_time(&self, edge: ClockEdge) -> Seconds {
        self.clock(edge.clock).edge_time(edge.edge)
    }

    /// The factor the cell delays of `cell` are scaled by on `side`: the
    /// instance's own where set, else the design's.
    #[must_use]
    pub fn cell_derate(&self, cell: Option<CellId>, side: MinMax) -> f64 {
        let design = self.derating.cell[side];
        cell.and_then(|c| self.cell_derates.get(&c))
            .and_then(|d| d.cell[side])
            .unwrap_or(design)
    }

    /// The clock entering at `port`, if one does.
    #[must_use]
    pub fn clock_at(&self, port: IoPinId) -> Option<ClockId> {
        self.clocks
            .iter_enumerated()
            .find(|(_, c)| c.sources.contains(&port))
            .map(|(id, _)| id)
    }

    /// Every port a clock enters at.
    pub fn clock_ports(&self) -> impl Iterator<Item = IoPinId> + '_ {
        self.clocks.iter().flat_map(|c| c.sources.iter().copied())
    }

    /// Whether a false path drops the check on `side` of a path launched
    /// by `from` into `to`, captured by `to_clock`.
    #[must_use]
    pub fn is_false_path(
        &self,
        from: PathEnd,
        to: PathEnd,
        to_clock: Option<ClockId>,
        side: MinMax,
    ) -> bool {
        self.false_paths
            .iter()
            .any(|f| f.side.is_none_or(|s| s == side) && f.paths.matches(from, to, to_clock))
    }

    /// The multicycle exception on a path launched by `from` into `to`,
    /// captured by `to_clock`, that bears on checks of `side`: the last
    /// one stated, as a later command overrides an earlier one.
    #[must_use]
    pub fn multicycle_path(
        &self,
        from: PathEnd,
        to: PathEnd,
        to_clock: Option<ClockId>,
        side: MinMax,
    ) -> Option<&MultiCyclePath> {
        self.multicycle_paths
            .iter()
            .rev()
            .find(|m| m.side.is_none_or(|s| s == side) && m.paths.matches(from, to, to_clock))
    }
}
