# Global routing

`router::global` routes every net over a grid of gcells, then rips up and
reroutes the nets that cross an overflowed edge, round after round. This
page draws the state that carries over between rounds.

## Negotiation

`negotiate` keeps a `Negotiation` between rounds:

- `best`: the round with the lowest overflow so far. It holds that round's
  edge usage, and the routes later rounds have since replaced.
- `costs`: the costs the next round routes with. Only the overflow
  penalty changes from round to round.
- `progress_from` and `stalled`: the overflow when negotiation last made
  enough progress, and the rounds since then.

```mermaid
stateDiagram-v2
    direction TB
    [*] --> Round: Negotiation::new (best = initial routes)
    Round --> Round: after_round / see below
    Round --> Cleared: best overflow is zero
    Round --> Stalled: STALL_ROUNDS (3) rounds without progress
    Round --> OutOfRounds: the round limit is reached
    Cleared --> [*]: finish
    Stalled --> [*]: finish
    OutOfRounds --> [*]: finish

    state Round {
        direction LR
        [*] --> History: accrue_history
        History --> RipUp: rip_up_overflowed (shelve with best)
        RipUp --> Reroute: reroute_levels
        Reroute --> [*]
    }
```

After each round, `after_round` updates the state from the overflow the
round left behind:

| The round's overflow | Penalty | Best | Progress |
| --- | --- | --- | --- |
| below the best | `× PENALTY_STEP` (1.5) | this round | |
| not below the best | `÷ PENALTY_STEP` | unchanged | |
| at most `(1 − MIN_PROGRESS)` (0.995) of `progress_from` | | | `progress_from` = this round, `stalled` = 0 |
| above that | | | `stalled` + 1 |

`finish` puts the grid and the routes back to the best round when the last
round was worse.
