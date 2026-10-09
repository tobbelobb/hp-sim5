# Extended autocal reference data

New browser collection can record physics, calibration output and communication
in **one `.rrd`**, written live by the flight recorder. `.rrd` is Rerun's recording
format; RRF is the firmware. No separate timeline format is needed. Historical
fixtures cannot acquire missing physics or original collection events retroactively.

## Manual collection at 25×

Use a fresh output directory and a new dataset path for each run. Start the usual
Vite development server (`npx vite --host 127.0.0.1`). In another terminal, start:

```bash
.venv/bin/python scripts/hangprinter_flight_recorder.py \
  --extended-reference --output output/extended-autocal/trial-001
```

Open the **visual 3D simulator**, with these query parameters:

<http://localhost:5173/hp-sim5/hp-sim-3d/?gcode_ws=ws://localhost:8790&rerun_ws=ws://127.0.0.1:9877&extended_autocal=1>

Select the intended machine and scene options, leave physics paused initially,
and check that the Rerun button says **recording**. The first snapshot must reach
the recorder before autocal starts; it rejects collection if no browser is connected.
Use one simulator tab. For example, for the HP3 scene (`hangprinter_4`):

```bash
.venv/bin/python autocal/autocal.py \
  --sim --machine-type hangprinter_4 --speedup 25 \
  --dataset output/extended-autocal/trial-001/sweeps.json \
  --extended-reference-ws ws://127.0.0.1:9877
```

Use your usual optimizer flags and matching firmware configuration. Autocal
forwards 25× to the collector, which sets the browser speed scale. Start playback
if the collection is waiting for the simulator. Reload the simulator after updating
the code: the collector requires its new clock response. Requested speed is only
a playback setting. Timed operations use the observed simulation clock even if
playback achieves 0.57×, and hardware uses wall time.

Extended recording defaults to **every tenth physics step** and **compact cable
geometry**. This lowers snapshot, transport and recording cost. It does not change
physics integration; G-code, replies, measurements, text and artifacts remain
unsampled. For every-step physics with full sag/guide drawings, add
`--sample-stride 1 --geometry-detail full` to the recorder command.

Wait for autocal to exit, then pause physics, wait for pending samples to drain,
and stop the flight recorder with Ctrl-C. Its RRD includes events from before and
after collection, and across scene resets; it stays open until you stop it. Open
that RRD with `.venv/bin/rerun path/to/file.rrd`. `--no-viewer` suppresses the live
Rerun viewer while retaining disk recording and the visual simulator.

The working JSON, `.full_auto.log`, `.full_auto_log.jsonl`, and stage snapshots
remain ordinary files too. Extended mode automatically enables the existing
stage-artifact writer; `--stage-artifacts` can select its directory. Keep the
whole run directory, including failed trials. `AUTOCAL_REFERENCE_WS` is the
inherited transport setting; the CLI option sets it for Python and child collectors.
This route currently requires the visual browser, not `--headless-sim`.

## Data model and clocks

| Source | Entity | Data | Time |
| --- | --- | --- | --- |
| Browser physics | `/world`, `/line_lengths`, `/line_errors`, `/cable_forces` | Transforms, geometry, lengths and forces at the configured stride | UTC `wall_time`, `sim_time`, `sim_step`, `scene_generation` |
| Browser recorder clock | `/clocks/browser/flight_recorder` | Simulation seconds, session, configured speed scale, sampling stride and geometry detail | UTC `wall_time` and physics indices |
| Browser runner clock | `/clocks/browser/research_clock` | Independently observed simulation seconds and source | UTC `wall_time` |
| Python, collector and browser events | `/autocal/python`, `/autocal/collector`, `/autocal/browser` | `TextLog` plus complete JSON in `event_json` | UTC `wall_time`, receipt `event_order` |
| Collector clock, when available | `/clocks/collector` | Latest browser clock observation, or native clock, with source and observation wall time | UTC `wall_time` |
| Source provenance | `/provenance` | Revision, working diff, untracked source files and runtime versions | Static |

`wall_time` is Unix UTC timestamp time, sampled at the source to millisecond
precision. Events remain indexed by that timestamp even when transport delivers
them out of order. `event_order` records receipt order for events, **not** global
causal ordering. For equal wall timestamps, use each source's sequence/step and
G-code `commandId` or encoder `requestId`. A millisecond timestamp alone cannot
prove ordering between concurrent processes. Source monotonic observations are
also retained on events, and `received_wall_time_ms` measures recorder receipt.

Simulation time is never synthesized as wall time multiplied by 25. Each event
carries `sim_time_s` and `sim_time_source`; unavailable clocks are explicitly null.
The browser collector queries `browser.researchClock` through the encoder bridge;
`sim_time_observed_wall_ms` records when that clock was observed. G-code events can
carry the most recent observation rather than a simultaneous clock reading.
These are observations of the browser clock, not an independent simulation clock.
Python has no simulation clock and records null. Wall timing and speed settings
remain visible without claiming clock agreement. The browser recorder's clock starts on connection/scene reset; the
runner's `researchClock` starts on scene reset. A snapshot taken during
`world.update` sees the runner clock **before** the runner advances it for that
step; the recording labels this observation phase. Browser events observe the
runner clock at event receipt, and also retain `recorder_sim_time_s`.

The default blueprint follows wall time and offers 3D physics, readable event logs,
clock comparisons and tabs for lengths/errors/forces. Select `sim_time` separately
for physics playback; events with unknown simulation time are intentionally absent
from that timeline. Rerun's automatic `log_time` is SDK logging time, not source time.

Settling retains the observation before each window boundary, including when
transport is slower than the whole window. Its default timeout is 30 seconds of
the selected clock, with a separate 120-second wall deadline. Force trials and
clock waits also have wall deadlines. Settling and active trials print progress
every five wall seconds; a paused clock eventually fails instead of waiting forever.
Requested speed never divides poll intervals, motion waits or force windows.

## What is preserved

Collector events capture each G-code request, response or error with a shared
command ID and the caller's `.mjs` file/line. Point events contain the measurement
and sweep config. Browser events capture incoming bridge payloads (including
translated motor commands, resets, speed scale and encoder requests), outgoing
encoder responses, and scene context with original USDA source and initial
physics/inspection settings. Those are boundary observations; they do not claim
the motor command has already executed when it enters the browser queue.

Python mirrors exact text-log writes, including redirected optimizer output, with
source timestamps. Text logs retain their existing plain format for compatibility;
the RRD contains the timestamped writes. Artifact events embed the initial/final
working dataset, completed collector datasets, sweep/firmware configs, numerical
stage snapshots, and final text/JSONL logs. `event_json` is the canonical complete
envelope; the displayed `TextLog` summarizes large artifacts.

The envelope has `version=1`, `type=autocal_event`, `source`, `kind`, `wall_time_ms`,
`sim_time_s`, `sim_time_source`, and `payload`. Additional source/transport fields
are retained. Artifact payloads have `path` and UTF-8 `content`; paths describe
original files and are not instructions to overwrite them. A reader can inspect
these using `rerun.chunk.RrdReader(path).stream().filter(content='/autocal/python',
components='event_json').to_chunks()` and each chunk's `to_record_batch()`.

This is a replay of recorded observations and numerical inputs, **not a resumable
physics checkpoint or a guarantee of bit-for-bit deterministic resimulation**.
Physics snapshots use the existing recorder's coverage; they do not serialize
all solver internals, firmware state or RNG state. Keep the recorded revision
available and the matching dependencies for resimulation. Manual UI interventions
after the initial context are not exhaustively recorded: preserve them in the run
notes, or collect without changing physics settings during the run.

The adjacent recorder manifest reports `physics_samples`, `event_count`,
`autocal_complete`, and `rejected_messages`. Finalized means the recorder closed;
it does not by itself certify successful calibration or full coverage. Missing
steps are rejected; concurrent physics tabs are rejected. Python and collector
transport failures fail collection instead of silently omitting their events.
With sampling, the expected next step is the previous step plus the reported
stride; unexpected gaps are still rejected. The last recorded step can precede
the stop by up to stride minus one steps. Compact geometry draws straight cable
spans and omits wrap/sag curves; measured lengths, errors and force values retain
their original numerical precision. Unchanged poses use Rerun's temporal latest-at
values rather than repeating identical transforms.

## Performance and file size

`.rrd` is already binary Arrow data with LZ4 compression; a new binary format
would not solve the per-step work. Sampling happens in the browser before building
the snapshot. The recorder serializes SDK writes on a worker thread so WebSocket
handling stays responsive. Collector events use a bounded queue, retaining their
source timestamps; G-code does not await each recorder acknowledgement. Successful
completion drains all events. Queue overflow, disconnect or missing acknowledgements
fail capture explicitly. The queue limits are 4096 events or 8 MiB, with a 10-second
acknowledgement watchdog.

To save more resources, use `--no-viewer` while capturing. The visual simulator
still runs; inspect the RRD afterwards. Choose a larger stride when lower temporal
resolution is acceptable. With physics timestep `dt`, retained observations are
`stride * dt` simulation seconds apart, regardless of requested playback speed.
Short contacts or vibrations between samples will be absent from a sampled replay.

For existing recordings, inspect and compact into a **new** file:

```bash
.venv/bin/rerun rrd stats --no-decode path/to/capture.rrd
.venv/bin/rerun rrd optimize path/to/capture.rrd -o path/to/capture.compacted.rrd
```

Compaction reduces chunk overhead and improves viewer/query performance; file
size savings vary. It needs memory for the recording and does not reduce the
cost of an already completed collection. Larger SDK microbatches or columnar
live logging are further options if recording remains a bottleneck. Rerun describes
[its binary format](https://rerun.io/docs/concepts/logging-and-ingestion/rrd-format)
and [microbatching and compaction](https://rerun.io/docs/howto/logging-and-ingestion/optimize-chunks).
