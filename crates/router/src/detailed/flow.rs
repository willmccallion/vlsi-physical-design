//! Which kind of repair each round runs, as `TritonRoute`'s `FlexDRFlow`
//! decides it (`src/drt/src/dr/FlexDRFlow.cpp`): the regular pass over the
//! die while violations are many, then, once they are few, passes over the
//! global routes of the nets in them and over tiles around them, tried with
//! several costs at once, and rounds skipped while neither helps.

use super::schedule::{RipUp, Strategy, round_strategy};

/// Violations above which every round makes the regular pass, as
/// `TritonRoute`'s `STUBBORN_FLOW_VIOLATION_THRESHOLD`.
const FEW_VIOLATIONS: usize = 100;

/// The kind of repair a round runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flow {
    /// The regular pass, region by region over the die.
    Regular,
    /// Workers over the global routes of the nets in each violation.
    Guides,
    /// Workers on tiles around clusters of violations, each tried with
    /// several costs, the best kept.
    Stubborn,
    /// No repair this round.
    Skip,
}

impl Flow {
    /// The flow's name in logs and saved states.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Regular => "regular",
            Self::Guides => "guides",
            Self::Stubborn => "stubborn",
            Self::Skip => "skip",
        }
    }

    /// The flow named `name`.
    pub fn from_name(name: &str) -> Option<Self> {
        [Self::Regular, Self::Guides, Self::Stubborn, Self::Skip]
            .into_iter()
            .find(|f| f.name() == name)
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
    pub fn after_first_pass() -> Self {
        Self::after(0, Flow::Regular, true)
    }

    /// The flow of round `round`, with `violations` left.
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
    pub const fn set_effective(&mut self, effective: bool) {
        self.last_effective = effective;
    }

    /// The flow of the last round decided.
    pub const fn current(&self) -> Flow {
        self.current
    }

    /// Whether the last guides or stubborn round was effective.
    pub const fn last_effective(&self) -> bool {
        self.last_effective
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn many_violations_take_the_regular_pass() {
        let mut state = FlowState::after_first_pass();

        assert_eq!(state.next(5, 101), Flow::Regular);
    }

    #[test]
    fn a_round_ripping_up_every_net_takes_the_regular_pass_however_few_violations() {
        let mut state = FlowState::after(48, Flow::Stubborn, false);

        assert_eq!(state.next(49, 3), Flow::Regular);
    }

    #[test]
    fn few_violations_after_an_effective_round_take_the_guides() {
        let mut state = FlowState::after(4, Flow::Regular, true);

        assert_eq!(state.next(5, 50), Flow::Guides);
    }

    #[test]
    fn ineffective_guides_are_followed_by_stubborn_tiles() {
        let mut state = FlowState::after(4, Flow::Regular, true);
        let _ = state.next(5, 50);
        state.set_effective(false);

        assert_eq!(state.next(6, 50), Flow::Stubborn);
    }

    #[test]
    fn ineffective_stubborn_tiles_skip_rounds_until_the_strategy_changes() {
        let mut state = FlowState::after(5, Flow::Guides, false);
        let _ = state.next(6, 50);
        state.set_effective(false);

        let skipped = [state.next(7, 50), state.next(8, 50)];
        let changed = state.next(10, 50);

        assert_eq!(skipped, [Flow::Skip, Flow::Skip]);
        assert_eq!(changed, Flow::Guides);
    }

    #[test]
    fn an_effective_stubborn_round_goes_back_to_the_guides() {
        let mut state = FlowState::after(5, Flow::Guides, false);
        let _ = state.next(6, 50);
        state.set_effective(true);

        assert_eq!(state.next(7, 50), Flow::Guides);
    }

    /// The flows of `rounds` from `state`, with many violations before round
    /// 3, and guides and stubborn rounds effective every seventh round.
    fn decide(state: &mut FlowState, rounds: std::ops::RangeInclusive<usize>) -> Vec<Flow> {
        rounds
            .map(|round| {
                let violations = if round < 3 { 500 } else { 40 };
                let flow = state.next(round, violations);
                if matches!(flow, Flow::Guides | Flow::Stubborn) {
                    state.set_effective(round % 7 == 0);
                }
                flow
            })
            .collect()
    }

    #[test]
    fn a_state_rebuilt_from_a_saved_round_decides_later_rounds_as_the_live_one() {
        let mut live = FlowState::after_first_pass();
        let _ = decide(&mut live, 1..=20);
        let mut rebuilt = FlowState::after(20, live.current(), live.last_effective());

        assert_eq!(decide(&mut rebuilt, 21..=64), decide(&mut live, 21..=64));
    }

    #[test]
    fn flows_are_found_by_their_names() {
        let names: Vec<Option<Flow>> = ["regular", "guides", "stubborn", "skip", "other"]
            .into_iter()
            .map(Flow::from_name)
            .collect();

        assert_eq!(
            names,
            [
                Some(Flow::Regular),
                Some(Flow::Guides),
                Some(Flow::Stubborn),
                Some(Flow::Skip),
                None
            ]
        );
    }
}
