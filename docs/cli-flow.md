# The flow of a run

`pare` takes a design from its input through placement, global routing and
detailed routing. `crates/cli/src/flow.rs` makes that order part of the
types: a `Design<S>` is at stage `S`, and each step takes the design by
value and returns it at the next stage. Routing a design that was never
placed, or routing it in detail before the repair on its global routes,
does not compile.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Loaded: Design::load
    Loaded --> Placed: place / placed, checked and written
    Loaded --> Placed: placed_by_input / a routing benchmark or `route`
    Placed --> GloballyRouted: route_globally / repair on the global routes
    Placed --> [*]: route_globally_only / `route --global-only`
    GloballyRouted --> [*]: route / routed, checked and written
```

| Stage | What it holds beside the design |
| --- | --- |
| `Loaded` | nothing |
| `Placed` | the netlist repair placement ran, for the repair on the global routes; none when it ran none |
| `GloballyRouted` | the global routes that repair ran on, which detailed routing goes on from; none when no repair needed them, and the detailed router then routes globally itself |

The commands walk the stages as far as they go:

| Command | Path |
| --- | --- |
| `place` | `load`, `place` |
| `route` | `load`, `placed_by_input`, `route_globally`, `route` |
| `route --global-only` | `load`, `placed_by_input`, `route_globally_only` |
| `run`, placement benchmark | `load`, `place` |
| `run`, routing benchmark | `load`, `placed_by_input`, `route_globally`, `route` |
| `run`, place and route | `load`, `place`, `route_globally`, `route` |
| `check` | `load`, then the checks on the design as read |

`place` resolves the config against the library into a `FlowPlan` first, so
a mistake in it fails before placement starts. A design placed by its input
has no plan and no netlist repair, so `route_globally` leaves it unrouted,
and the detailed router routes it globally itself.
