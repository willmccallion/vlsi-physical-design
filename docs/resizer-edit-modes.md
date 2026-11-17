# Resizer edit modes

A `Resizer` session edits the netlist and keeps parasitics and timing up to
date as it goes. Three modes change how it does that. Each opens with a
call that returns a token and closes with a call that takes the token back,
so a mode can be closed only once, and only after it was opened. Rust cannot
force a token to be used, but each token is `#[must_use]`, and a mode opened
while it is already open trips a `debug_assert`.

| Mode | Opens with | Token | Closes with | Mirrors |
| --- | --- | --- | --- | --- |
| Trial | `journal_begin` | `Trial` | `journal_end` (keep) or `journal_restore` (undo) | `Resizer::journalBegin`/`journalEnd`/`journalRestore` |
| Edit batch | `batch_edits` | `EditBatch` | `flush` | OpenROAD marking parasitics invalid and updating them at the next query |
| Deferred timing | `defer_timing` | `DeferredTiming` | `resume_timing` | `OpenSTA` finding arrivals only when asked |

## Trial

While a trial is open, every edit is recorded. Keeping the trial drops the
record. Restoring it undoes the edits, last first, and brings timing back
to where it was.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Idle
    Idle --> Recording: journal_begin() -> Trial
    Recording --> Recording: edit (recorded)
    Recording --> Idle: journal_end(Trial)
    Recording --> Idle: journal_restore(Trial)
```

Setup repair keeps a trial open across the passes for one endpoint. After a
pass that improves things it closes the trial and opens a new one, and it
restores the trial once a run of passes has stopped paying off.

```mermaid
stateDiagram-v2
    direction TB
    [*] --> Open: journal_begin
    Open --> Pass
    Pass --> Kept: pass improves
    Kept --> [*]: last pass
    Kept --> Open: journal_end, journal_begin
    Pass --> Pass: worse, within tolerance
    Pass --> Restored: worse past tolerance or terminated
    Pass --> Closed: endpoint met or no move found
    Restored --> [*]: journal_restore
    Closed --> [*]: journal_end after pass 1, else journal_restore
```

## Edit batch

While a batch is open, parasitics and timing updates for nets that existed
before it opened wait until the flush. Nets the batch itself creates are
still estimated as they change, because the batch's own edits read them.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Current
    Current --> Batched: batch_edits() -> EditBatch
    Batched --> Batched: edit (drivers and older nets queued)
    Batched --> Current: flush(EditBatch)
```

## Deferred timing

While timing is deferred, slews and delays are still propagated through the
graph, but the timing analysis only collects the drivers it will need to
update. Design repair reads slews and never timing, so it runs this way.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Timed
    Timed --> Deferred: defer_timing() -> DeferredTiming
    Deferred --> Deferred: edit (graph updated, timing stale)
    Deferred --> Timed: resume_timing(DeferredTiming)
```

`Resizer::sta` must not be read while a batch is open or timing is
deferred, since the analysis is behind the design then; a `debug_assert`
enforces this.
