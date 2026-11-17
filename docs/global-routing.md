# Global routing

`router::global` routes every net over a grid of gcells, then rips up and
reroutes the nets that cross an overflowed edge, round after round. This
page draws the state that carries over between rounds.

## A net's tree on the grid

A net's global route adds demand to the edges it crosses. The demand
depends on the net as well as its paths: its track demand on each layer,
and the terminals whose vias add no via loss. So lifting a tree is only
right when it uses the same net it was placed with.

The types make that impossible to get wrong. A `NetTree` is a net with its
paths. `GlobalGrid::place` takes a `NetTree` and returns a `PlacedTree`,
and `GlobalGrid::lift` takes the `PlacedTree` back and returns the
`NetTree`. `GlobalRoutes` holds only `PlacedTree`s, and `PlacedTree` cannot
be cloned. So a tree is lifted once, after it was placed, and with its own
net.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> NetTree: routed (pattern, maze or saved state)
    NetTree --> PlacedTree: GlobalGrid::place / demand added
    PlacedTree --> NetTree: GlobalGrid::lift / the same demand removed
    PlacedTree --> PlacedTree: renumber / net id follows the design
    NetTree --> [*]: dropped or rerouted
```

One place makes a `PlacedTree` without `place`: `Best::restore`. It puts
back the usage of the best round wholesale, and that usage already holds
the demand of the routes it shelved, so it re-wraps those routes without
placing them again.

When pin access is found again after repair, `settled_stage` lifts each
tree it keeps, then places the same paths again with the net as it is now.

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
