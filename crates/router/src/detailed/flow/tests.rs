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
        .map(Flow::from_keyword)
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
