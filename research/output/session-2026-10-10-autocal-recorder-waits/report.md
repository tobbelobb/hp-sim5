# Autocal return waits fixed by lossless recorder batching

The recorded reference run showed a 16,162-command return burst delaying event logging by 21.88 seconds and blocking physics after about 0.64 simulation seconds. The implementation batches browser/headless event transport and writes complete logical event rows as RRD columns, preserving payloads, original timestamps, receipt ordering and clock fields. Bounded queues, counted post-write acknowledgements, partial-batch flushes and failure handling retain capture integrity. An isolated localhost replay of the exact recorded envelopes acknowledged the following physics sample in 0.175 seconds with every envelope preserved. Direct writer replay improved from 1.670 to 0.109 seconds (15.4x). Recorder, headless lifecycle and calibration guard tests passed. These measurements used no Viewer and synthetic physics samples; they do not establish full browser collection speed or calibration accuracy. Observed-clock waits and force/slip guards remain unchanged.


## Findings

The text-log message at line 77 describes a return move inside force tuning, not an added 32-second settling window. The recorded G-code is `G1 H2 Z-753.982 F1400`. Its firmware motion duration is 32.321 seconds. The collector receives the firmware reply in 78 ms, then `runMoveWithWait` waits for that duration on the observed physics clock. It takes 27.286 wall seconds before the next collector G-code, which reports about 33.16 more simulation seconds than the prior stale observation.

That return creates 16,162 low-level `Move` events, consistent with 500 firmware buckets per second. Browser source timestamps span only 0.25 seconds. The final command event has source wall time 1791640948887 and recorder logging wall time 1791640970767: 21.88 seconds later. This latter field is sampled inside `log_event`, so it measures source-to-log latency, not network receipt latency alone.

During that backlog the browser event clocks remain near 1286 seconds. The text log shows roughly 0.6 seconds of physics advance after 20 wall seconds. `MAX_PENDING_STEPS=32`, sampling stride 10, and dt=0.002 explain the plateau: 32*10*0.002=0.64 simulation seconds. The first 26 force-tuning return waits sum to 368.179 wall seconds. This aggregate includes actual movement and polling, so it is not all removable overhead.

## Mechanism and relevant source

1. `integrations/rrf/rrfHttpBridge.js` expands firmware trajectories into 2 ms Move buckets and emits each separately.
2. `hp-sim-3d/app/externalCommandSocket.js:224` records every translated incoming command as an event.
3. `hp-sim-3d/app/flightRecorder.js:140` immediately sends each event on the same socket as sampled physics. Events do not share the physics in-flight bound.
4. `scripts/hangprinter_flight_recorder.py:329` reads that socket sequentially. Each event awaits an individually locked `asyncio.to_thread` write and an event acknowledgement. `log_event` performs two SDK logging operations when a simulation clock is present.
5. Physics samples arriving after the burst cannot be handled until all preceding events have been processed. Their acknowledgements stop. `flightRecorder.readyForStep` and `runner.remoteQueueReady` stop advancing physics at the 32-sample bound.
6. The collector correctly keeps waiting for the observed simulation clock. Once the backlog clears, physics resumes at roughly 6 simulation seconds per wall second in the nearby logs.

This is an event throughput and acknowledgement scheduling defect. Simply moving one-event writes back onto the async event loop or increasing the physics buffer would not address the underlying workload safely. Increasing sampling stride does not reduce these unsampled command events. Hiding progress lines or dividing waits by requested speed would conceal unfinished physics.

## Commit history and intent

- `54e88492` (Oct 8): faster collection with guarded settling. Kept encoder stability checks while shortening the nominal stability window.
- `e1044a50` (Oct 9): extended reference capture in a single RRD. Added browser capture of translated commands, encoder traffic, Python output and collector boundaries. This introduced the per-command event volume.
- `fa90f64e` (Oct 9): observed simulation clock, stride-10 physics capture, compact geometry, asynchronous recorder SDK writes, independent wall deadlines, and preservation of settling samples across slow observations. Intended to prevent requested 25x playback from being mistaken for achieved physics progress, and to prevent settling from stalling when transport outlasts a whole observation window. This introduced the current wait semantics and per-message thread dispatch. The unsampled command burst remained.
- `77d17ca6`: holds moving force trials before return settling and holds motors even after zero-distance returns. Reverting it risks waiting for a force-driven motor to stop spontaneously.
- `adb7cbbb`: gives active workspace pulls a longer simulation-time deadline and releases force on errors. These are distinct safety/correctness fixes.
- `f695f6ed`, `51b5ee20`: detect quiet-but-slipped fixed motors, cap collection forces, and recover/recollect failed directions. Preserve these protections.
- `8ded7420`, `0f519862`: headless extended recording and lifecycle changes. They do not cause this browser receiver backlog.

The recording establishes the present cause. A before/after run proving how much each historical change contributed has not been performed.

## Original fix plan

Keep observed-clock durations, actual encoder settling, hold-before-return behavior, force/slip guards, sample coverage checks and wall deadlines.

Start with the recorder event path rather than calibration algorithms:

1. Batch browser events by a bounded byte/count limit and a short flush interval. Retain every original envelope, payload, wall/simulation timestamp and within-source order. This should reduce WebSocket messages and acknowledgement overhead without sampling events.
2. Have the receiver write batches on its serialized writer, amortizing thread dispatch and SDK work. Use multirow RRD event writes rather than merely looping through the same two SDK calls per event. Keep existing `event_json` rows and clock attribution queryable. A transport-only batch still leaves expensive per-event SDK logging and may fail the performance goal.
3. Acknowledge the number of logical events only after successful recording. Update pending-event accounting and drain/error handling accordingly. Flush a partial batch on shutdown/reset, and fail explicitly on overflow or write/transport errors.
4. Ensure event ingestion is bounded and cannot prevent sampled physics from reaching the writer for tens of seconds. If throughput batching alone does not meet the replay target, separate bounded ingestion from the serialized writer and interleave event batches with physics samples. Assign receipt order at ingestion if writer scheduling changes; preserve source timestamps and keep acknowledgements after writes. Do not replace this with an unbounded queue or early success acknowledgements.

Likely primary files: `hp-sim-3d/app/flightRecorder.js`, `scripts/hangprinter_flight_recorder.py`, plus their tests and protocol documentation. The recorder class is shared with headless recording, so check both modes. Keep the firmware Move queue and physics trajectory unchanged. Source-side capture can continue to observe individual incoming commands; batching belongs in its recorder transport/write path.

## Original validation plan (completed checks below)

- Reproduce the recorded 16,162-event burst in an isolated recorder with interleaved physics samples. Measure maximum sample-ack delay, event queue depth and drain time. Provisional acceptance target: under one second of recorder-induced physics stoppage for this burst on the same machine; tune after measuring the baseline in isolation.
- Verify all 16,162 envelopes, source timestamps and ordering survive decoding, and no expected physics samples are lost. Verify per-event clocks and final partial-batch flushes.
- Test bounded overflow, disconnect, write failure, delayed acknowledgements, scene reset and successful finalization. Check acknowledgements cannot falsely release the physics buffer.
- Re-run existing recorder, clock, force-trial, return-to-origin and slip-recovery checks, plus headless recorder checks.
- Compare the same browser return with recording disabled and enabled in fresh isolated runs. Remaining wait should track actual movement at achieved speed. A 32.3-s move at 6x still needs roughly 5.4 wall seconds; eliminating the extra 22 seconds does not guarantee requested 25x throughput.
- Compare tuning decisions and encoder return/settling behavior; only timing should change. Assess calibration accuracy separately.

## Evidence and limits

[Extracted measurements](evidence.json) were obtained by filtered `RrdReader` scans of the active [RRD](../../extended-autocal/no-buildup-reference-001/hangprinter-20261010T135406.255382Z-2af430aa.rrd). [Text log](../../extended-autocal/no-buildup-reference-001/sweeps.full_auto.log) supplies progress messages. The recording was still growing, so counts are prefix observations, not final coverage. During the investigation phase, no new experiments or tests were launched, no code files were changed, and the active services/run directory was left untouched. The implementation and validation after recording stopped are described below.

## Implemented fix and validation

The user stopped the reference recording and authorized implementation. The fix is limited to recorder transport, writer code, tests and protocol documentation. Calibration clock waits, firmware trajectories, tuning algorithms, motor hold/settling checks and slip recovery were not changed.

Browser/headless transport batches at most 256 events or 1 MiB; an individual larger envelope travels alone. It serializes envelopes at capture so mutable payloads and scene clocks cannot change while waiting to flush. Partial batches flush within 20 ms, before physics samples, and on drain/disconnect. Outstanding logical events are bounded at 65,536 or 32 MiB; overflow fails capture.

The receiver preserves one canonical event_json row and TextLog per logical event, but uses columnar SDK writes and one worker dispatch per batch. It accepts existing scalar Python/collector traffic through the same writer. Counted batch acknowledgements follow successful writes. Invalid batches receive no acknowledgement and are recorded as rejected. Clock scalar order is preserved even with equal wall timestamps and optional observation metadata. Event rows never inherit a physics sim_time timeline.

EXP-001 (offline writer replay): the exact 16,162-command burst took 1.670 s using the previous row-by-row writer and 0.109 s using batches of 256, a 15.4x speedup. Both outputs preserved all source envelopes in event order. This measures only SDK logging, without the old per-event worker scheduling and WebSocket overhead. [Results and RRD paths](batch-benchmark/results.json).

EXP-002 (localhost receiver replay): the production receiver consumed the exact envelopes in 64 batches and acknowledged a following synthetic physics sample in 0.175 s. Its finalized RRD retained every original envelope. [Results and RRD path](batch-benchmark/websocket/results.json). This meets the proposed under-one-second target on this machine. The original run's 21.88-second backlog is contextual evidence, not a matched before/after whole-browser benchmark.

The recorder/headless Python suite passed (24 tests, two slow tests excluded by the repository default). JavaScript recorder, reference logger, clock, force-trial, return and slip-recovery checks passed. Coverage includes the full 16,162-event burst with interleaved physics, exact RRD envelope preservation, source timelines, final partial batches, reset/disconnect, byte/count bounds, acknowledgement validation and write failures.

A fresh browser calibration run with a live Viewer has not been performed, and no calibration-accuracy gain is claimed. The benchmark used --no-viewer. No physics motion was commanded outside existing regression tests. Batching leaves the bounded serialized receiver in place; the measured throughput made a larger scheduler redesign unnecessary. Restart the recorder and reload the simulator before the next collection so both sides use the new batch protocol.
