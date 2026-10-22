# Autocal recorder wait investigation

Objective: identify the recent regression behind long force-tuning waits and plan a safe fix.

Accepted steering: the reference run remains active; do not edit code files, interrupt collection, reset the simulator, connect another physics source, or change recording settings. Investigation and planning only.

Budgets: zero commanded motion, zero new physics experiments, zero code edits. Offline source/history/log inspection only. Read the live RRD using filtered RrdReader streams; extracted temporary event tables are under /tmp. The run directory was read only.

OBS-001: existing recording observation, not a newly launched experiment. Hypothesis: recorder command-event bursts delay sample acknowledgements and suspend physics. Supported by 16,162 events from one return, 21.88-second event logging lag, the approximately 0.64-second buffer plateau, and receiver/runner source inspection.

Hypothesis that batching fixes performance while retaining coverage is not tested. Next decision: after the reference run finishes, implement bounded lossless batching, then replay the observed burst in an isolated recorder and compare browser collection with recording on/off. Preserve observed-clock timing and stall/slip guards.

No calibration accuracy result is claimed. The run was still active and its RRD unfinalized when inspected. Final artifacts: report.md, abstract.md, evidence.json.

## Implementation turn

Accepted steering: the user stopped recording and explicitly authorized implementing the fix. Source edits are now permitted. Keep the original recording untouched; launch no new calibration or motion experiments. Isolated localhost recorder tests and offline replay are authorized validation.

The existing mapping stays unchanged: raw live JSON envelopes become temporal TextLog plus AnyValues(event_json, received_wall_time_ms) on /autocal/<source>, indexed by source wall_time and event_order. Clock observations stay temporal Scalars plus source/observation context on /clocks/<source> or /clocks/<source>/research_clock, indexed by the same two timelines. Physics entities, properties and timelines are unchanged. Live columnar logging uses send_columns; saved RRD verification uses RrdReader streams.

EXP-001: replayed the original 16,162 envelopes with the old log_event implementation and the batched writer, no Viewer. Results: 1.670 s versus 0.109 s for SDK logging; both RRDs preserved all envelopes exactly. This excludes per-event thread dispatch and WebSocket overhead.

EXP-002: replayed those same envelopes through the production WebSocket receiver as 64 batches, then sent a synthetic physics sample. Its acknowledgement arrived after 0.175 s. The finalized RRD preserved all original envelopes exactly. The sandbox denied creating a socket, so this benchmark was rerun with explicit localhost-network escalation. No active user services were accessed.

Implementation: 256-event / 1 MiB transport batches, 20 ms partial flush, 65,536-event / 32 MiB outstanding event bounds; counted acknowledgements only after columnar writes; batch flushes before samples and on drain/disconnect. Preserve original source timestamp/payload serialization at capture, and clocks on original source timelines. Failure stops recorder-dependent physics instead of resuming with incomplete capture.

Regression checks: recorder persistence, live WebSocket burst/invalid-batch rejection, headless resets/shutdown/transport failures, browser batching/byte limits/count accounting/failures, observed clock, force trials, return-to-origin and slip recovery. Full calibration/browser performance remains untested. Next decision: user can restart the recorder and reload the simulator for the next run.
