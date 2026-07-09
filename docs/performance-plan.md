# SignalK-mini performance and memory plan

This plan keeps the server single-threaded, typed-model-first, and friendly to both Linux and MCU builds. Each phase should remain small enough to review and validate with CI.

## Phase 1: counters and queue observability

Status: implemented.

Goals:

- Expose how many model changes were marked.
- Expose how many changes were actually enqueued.
- Expose how many pending changes were coalesced before publish.
- Preserve the existing dropped-change counter.
- Track the change queue high-watermark.

Rationale:

Performance work needs visible counters before larger rewrites. Queue high-watermark and coalesced-change counts show whether parser bursts, repeated sensor values, or slow clients are creating publish pressure.

Counters:

- `marked_change_count()`
- `enqueued_change_count()`
- `coalesced_change_count()`
- `dropped_change_count()`
- `pending_change_count()`
- `change_queue_capacity()`
- `change_queue_high_watermark()`

## Phase 2: duplicate model-change coalescing

Status: implemented.

Goals:

- Collapse repeated changes for the same `(ModelField, SourceId)` while they are still pending in the queue.
- Keep the newest timestamp and sequence for the coalesced change.
- Preserve separate pending changes for the same field from different sources.
- Preserve queue drop behavior for genuinely distinct pending changes when capacity is exhausted.

Rationale:

Many NMEA sentences mark multiple fields, and high-rate sources can update the same field repeatedly before the publisher tick drains the queue. Coalescing avoids repeated Signal K mapping and JSON publishing for stale intermediate values.

## Phase 3: batch Signal K values per source

Status: implemented in this PR for scalar mapped values. Object-valued deltas remain single-message to keep AIS and notification object serialization isolated.

Goals:

- Emit one Signal K delta per source per publish tick, with multiple `values` entries.
- Keep hard limits for maximum JSON bytes, maximum values per delta, and maximum work per tick.
- Reduce TCP writes and JSON overhead.

Expected impact:

- Lower CPU use.
- Lower WiFi/TCP bandwidth.
- Lower latency under bursts because fewer writes are needed.

## Phase 4: sentence ID dispatch

Status: implemented in this PR.

Goals:

- Classify NMEA sentence IDs once into a compact enum.
- Replace repeated string comparisons in sentence-to-change marking with a switch.
- Move sentence-specific marking into narrow handlers.

Expected impact:

- Less CPU per sentence.
- Better readability.
- Fewer semantic mistakes such as true/magnetic heading confusion.

## Phase 5: parser buffer/copy reduction

Goals:

- Add parser support for spans or in-place token views.
- Avoid copying the selected NMEA sentence into a temporary buffer when the input line is already mutable and bounded.

Expected impact:

- Less stack use.
- Less memory bandwidth.
- Faster high-rate serial/UDP input.

## Phase 6: MCU memory profile

Goals:

- Evaluate compact `ModelChange` layouts for MCU builds.
- Consider smaller default Signal K JSON buffers for MCU builds.
- Replace dynamic connector slot storage with fixed-capacity storage where practical.

Expected impact:

- Lower RAM use on ESP32-class targets.
- More deterministic memory behavior.

## Phase 7: subscription-aware publishing

Goals:

- Track per-client subscription masks.
- Avoid mapping and serializing paths that no client will receive.
- Keep subscribe-all as the simplest first mode.

Expected impact:

- Lower CPU and bandwidth when clients only need a subset of paths.

## Validation strategy

Every phase should include at least one direct unit test and should keep these CI targets green:

- Linux configure/build/tests.
- AtomS3R sketch compile.

For performance-sensitive phases, add counters before changing behavior and update tests to assert the new counter semantics.
