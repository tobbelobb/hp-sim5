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
if the collection is waiting for the simulator. Recording backpressure can reduce
achieved speed; it does not intentionally drop or decimate physics timesteps.

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
| Browser physics | `/world`, `/line_lengths`, `/line_errors`, `/cable_forces` | Existing transforms, geometry, lengths and forces at every step | UTC `wall_time`, `sim_time`, `sim_step`, `scene_generation` |
| Browser recorder clock | `/clocks/browser/flight_recorder` | Simulation seconds, session and actual speed scale | UTC `wall_time` and physics indices |
| Browser runner clock | `/clocks/browser/research_clock` | Independently observed simulation seconds and source | UTC `wall_time` |
| Python, collector and browser events | `/autocal/python`, `/autocal/collector`, `/autocal/browser` | `TextLog` plus complete JSON in `event_json` | UTC `wall_time`, receipt `event_order` |
| Collector clock, when available | `/clocks/collector` | Reported simulation seconds and source | UTC `wall_time` |
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
Currently the browser collector and Python have no independent simulation clock.
Their wall timing and speedup settings remain visible, without claiming physics
agreement. The browser recorder's clock starts on connection/scene reset; the
runner's `researchClock` starts on scene reset. A snapshot taken during
`world.update` sees the runner clock **before** the runner advances it for that
step; the recording labels this observation phase. Browser events observe the
runner clock at event receipt, and also retain `recorder_sim_time_s`.

The default blueprint follows wall time and offers 3D physics, readable event logs,
clock comparisons and tabs for lengths/errors/forces. Select `sim_time` separately
for physics playback; events with unknown simulation time are intentionally absent
from that timeline. Rerun's automatic `log_time` is SDK logging time, not source time.

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
