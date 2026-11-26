//! Which kind of repair each round runs, as `TritonRoute`'s `FlexDRFlow`
//! decides it (`src/drt/src/dr/FlexDRFlow.cpp`): the regular pass over the
//! die while violations are many, then, once they are few, passes over the
//! global routes of the nets in them and over tiles around them, tried with
//! several costs at once, and rounds skipped while neither helps.

use super::schedule::{RipUp, Strategy, round_strategy};

/// Violations above which every round makes the regular pass, as
/// `TritonRoute`'s `STUBBORN_FLOW_VIOLATION_THRESHOLD`.
const FEW_VIOLATIONS: usize = 100;

pare_common::keyword_enum! {
    /// The kind of repair a round runs, named by its keyword in logs and
    /// saved states.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum Flow {
        /// The regular pass, region by region over the die.
        Regular = "regular",
        /// Workers over the global routes of the nets in each violation.
        Guides = "guides",
        /// Workers on tiles around clusters of violations, each tried with
        /// several costs, the best kept.
        Stubborn = "stubborn",
        /// No repair this round.
        Skip = "skip",
    }
}

/// The state that decides each round's flow.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlowState {
    current: Flow,
    /// Whether the last guides or stubborn round reduced any worker's
    /// violations.
    last_effective: bool,
    /// The strategy of the last round decided.
    last_strategy: Strategy,
}

impl FlowState {
    /// The state after round `round`, whose flow was `current`.
    pub fn after(round: usize, current: Flow, last_effective: bool) -> Self {
        Self {
            current,
            last_effective,
            last_strategy: round_strategy(round),
        }
    }

    /// The state after the first pass, which rips up every net.
    pub(crate) fn after_first_pass() -> Self {
        Self::after(0, Flow::Regular, true)
    }

    /// The flow of round `round`, with `violations` left, as
    /// `FlexDRFlow::determineNextFlow` chooses it.
    pub fn next(&mut self, round: usize, violations: usize) -> Flow {
        let strategy = round_strategy(round);
        self.current = if violations > FEW_VIOLATIONS || strategy.ripup == RipUp::All {
            Flow::Regular
        } else if self.last_effective || strategy != self.last_strategy {
            Flow::Guides
        } else if self.current == Flow::Guides {
            Flow::Stubborn
        } else {
            Flow::Skip
        };
        self.last_strategy = strategy;
        self.current
    }

    /// Records whether a guides or stubborn round reduced any worker's
    /// violations.
    pub(crate) const fn set_effective(&mut self, effective: bool) {
        self.last_effective = effective;
    }

    /// The flow of the last round decided.
    pub const fn current(&self) -> Flow {
        self.current
    }

    /// Whether the last guides or stubborn round was effective.
    pub(crate) const fn last_effective(&self) -> bool {
        self.last_effective
    }
}

#[cfg(test)]
mod tests;
