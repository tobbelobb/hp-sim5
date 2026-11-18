# Simulator service / worker boundary: completion, pacing and settings state

Session: `session-2026-11-18-investigate-the-simulator-service-worker-boundary-ui-worker-auth-7838ab14`
Program: `program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7`
Date: 2026-11-18. Code: `74ecdc951666821ac1ba44250fb69badfbc46c26` (only submodule pointers dirty).
Environment: Linux x64, Node 26.13.0. No source file was modified. No browser, no
server, no worker, no build, no native addon, no Python.

## Short version

Two defects at the UI ⇄ playback-worker ⇄ world boundary. Both are pure state-machine
defects, reproducible in milliseconds with stubbed workers and without a browser.

1. **Job completion is decided by the producer, not by the world, so the Quality
   History entry measures a prefix of the print.** The playback worker's `done`
   message means "no more commands will be *sent*"; `finishJob()` treats it as "the
   print is *finished*" and takes the final quality snapshot immediately, while
   `RemoteSpoolSystem` still holds up to `highWaterMark` commands (320 at 1×,
   2 560 at 8×, 81 920–163 840 under Finish ASAP's 256×…512×) that the world has not
   executed. Measured on a 1000-command square path: the record holds **680 of 1000 samples, 2 of 3 corners,
   and score 100.0** while the completed print measures **94.3** (a 3 mm error on the
   last side is invisible to the record). The Finish-ASAP path already does this
   correctly — it waits for `queueLen === 0` before recording — so the linear path is
   an oversight, not a design choice.

2. **The world's pacing mirrors survive the worker they describe.** `fastModeActive`
   is the world's record of what it last told *the attached worker*; when the worker
   is replaced (job end, Reset) the mirror is kept, so the next job's worker is never
   told `set_fast_mode(true)` even though its queue is below the low water mark. That
   is the only mechanism that lets a worker run ahead of the wall clock, so the
   prefetch buffer the pacing design relies on is never built for that job. Measured:
   job 1's worker receives `set_fast_mode`; job 2's identical worker receives nothing
   at queue length 159 < 160, while a fresh `RemoteSpoolSystem` at the same queue
   length does send it.

Negative results (kept, all verified): substep changes do not desynchronise the
one-command-per-step contract; a superseded worker's late commands are dropped by the
existing identity gate; a paused world consumes no commands and duplicates none;
`set_dt`/`set_speed_scale` are re-sent at every job start; every message the UI posts
has a handler in all three playback workers.

## Method and exact commands

Read first: `hp-sim-3d/app/{workerController,runner,remoteSpoolSystem,commandJobController,
qualityController,quality-monitor,appBootstrap,machineSceneController,externalCommandSocket,
physics-substeps,simulationSystems}.js`, `hp-sim-3d/app/workers/{moveCommander,simulationClock}.js`,
`integrations/{rrf/rrfCanPlayer,klipper/klipperMcuCommandPlayer,klipper/klipperSimulatorBridge}.js`,
`src/js/cable_joints_3d/ecs.js`, `autocal/control/primitives/{browser_clock,encoder_utils}.mjs`,
`tests/js/hp-sim-3d/{autocalSettings,runner,wiring}.test.js`. Then micro-experiments only,
one process at a time, `timeout 60`, RSS via `/usr/bin/time -v` (47–68 MB, ≤0.10 s wall).

```bash
cd research/output/program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7/session-2026-11-18-investigate-the-simulator-service-worker-boundary-ui-worker-auth-7838ab14

node repro_quality_recorded_before_drain.mjs   # exit 1 = defect 1 reproduced
node repro_worker_replacement_state.mjs        # exit 1 = defect 2 reproduced
node probe_substeps_cadence.mjs                # exit 0 = negative result holds
```

`workerController.js` imports Vite-only `?worker&url` modules, so it cannot be imported
by plain Node. Both reproducers therefore mirror the two boundary functions they test
(4 lines: the `done` branch and `finishJob`) and **fail with `sourceDrift` if the app
source no longer matches the quoted shape** — the drift check is part of every run and
currently passes.

## Defect 1 — the final quality record is taken before the world executes the queue

### Mechanism (source)

`hp-sim-3d/app/workerController.js` (done branch):

```js
if (event.data.type === 'done') {
  const remoteSystem = getRemoteSystem();
  if (remoteSystem && remoteSystem.worker === worker) {
    remoteSystem.worker = null;
    onWorkerDone(worker);           // appBootstrap: () => commands?.finishJob?.()
  }
```

`hp-sim-3d/app/commandJobController.js` `finishJob()`:

```js
setPrintActive(false);
if (asapState.active) { asapState.pendingFinalCheck = true; }
else { recordFinalQualityAndResetJob(); }      // -> quality.runFinalQualityChecks(...)
```

`hp-sim-3d/app/qualityController.js` `runFinalQualityChecks()` → `monitor.runFinalCheck()`
→ `refreshHud(true)` → `recordQualityHistoryEntry()` stores `metrics.score`.

`hp-sim-3d/app/remoteSpoolSystem.js` consumes exactly one queued command per world step
(`update()`, `commandHead += 1`), and lets the worker run at most `highWaterMark =
max(320, ceil(320 × max(1, timeScale)))` commands ahead (lines 38–42, 376–408). So `done`
always arrives with a non-empty queue: the worker is only unpaused below 160 commands and
bursts until it is paused again above 320, so the hand-over depth is in `[160, 320]` at 1×
(measured 320, the cap, in the reproducer) = 0.32–0.64 s of simulated time at `dt = 0.002`
(every shipped scene is `timeCodesPerSecond = 500`), and 8× larger at 8×. Finish ASAP
scales the cap by its 256×…512× playback (`asapScale = min(512, max(256, timeScale × 8))`),
i.e. 81 920–163 840 commands.

Contrast: the ASAP path is correct. `runAsapFastForward()` waits until
`!workerActive && queueLen === 0` before `finalizeAsapMode()` runs the deferred
`recordFinalQualityAndResetJob()`, so an ASAP job *is* measured on the drained world.

### Experiment

A synthetic but faithful boundary: a real `World`, the real `RemoteSpoolSystem`, the real
`createQualityController`/`QualityMonitor`, a fake DOM element (the monitor only touches
`classList`/`style`/`textContent`/`innerHTML`), and a worker model that keeps the queue at
`baseHighWaterMark` exactly as the app's fast-mode/backpressure loop does. Path: a closed
1000-command square (1 mm per command, 3 interior 90° corners at commands 250/500/750).
The world's effector tip is fed per command (in the app the extruder system computes it
from motor angles); the reference path is supplied the way the inspection tools supply it.
The last side carries a lateral tracking error of 3 mm in one variant and 0 in the control.

### Expected vs actual (tolerance 0.5 score points, exact counts)

| quantity | expected (completed print) | actual (history entry) |
| --- | --- | --- |
| samples | 1000 | **680** (68 %) |
| corners | 3 | **2** |
| progress | 1.0 | 0.68 |
| score, 3 mm error on the last side | 94.313 | **100.0** |
| score, perfect tip path (control) | 100.0 | 100.0 |
| commands the world executed *after* the record | — | **320** (0.64 s of sim time) |

The control row is the important one: with a perfect tail the record and the completed run
agree, so nothing looks wrong; the moment the tail contains a defect the record is
systematically optimistic. The same mechanism shortens `cornerCount`, so the score's
corner/settling terms are computed from fewer corners than the print has: the corner
inside the tail (command 750 of 1000) never activates before the record is taken, so it
is absent from `cornerCount` (`2` recorded, `3` in the completed run).

The HUD and the record then disagree for the same job: after the queue drains the card
shows the completed-run score (94.3) while Quality History keeps 100. (Sampling is not gated
by the HUD toggle, so the entry is written even with the toggle off; the visible
disagreement needs it on.) The record is persisted (`QUALITY_HISTORY_MAX_ENTRIES = 20`,
sorted by score) and is the only cross-job comparison the app keeps.

### Reachability: genuine, shipped, one click

Tick **Live Quality Checks** and press **Print logo** (default preset), or **Print square**
(`draw_squares.gcode`: 11 G-code moves at F12000 → ~10 000 interpolated commands, so
320 ≈ 3 %), and let the job finish. The record is taken at the current prefetch depth, so
the blind tail is 160–320 commands (0.32–0.64 s of motion) at 1×; the shipped logo is ~20 000 G-code moves (64 634 for
`Hangprinter_logo2.gcode`), so ~1.6–3 % of its samples, and proportionally more at higher
playback speeds. By inspection of the same code path, a stream shorter than the prefetch
depth hands over while the world has executed 0 commands, `_composeScore` returns NaN and
`recordQualityHistoryEntry` skips the entry entirely, so a short uploaded job would be
recorded as no print at all (inferred from the code, not measured here).

Severity: a user-visible, persisted, systematically one-sided metric error plus a
HUD/history disagreement. Not a physics error, and not a determinism error.

## Defect 2 — pacing state is not reset when the worker is replaced

### Mechanism (source)

`hp-sim-3d/app/remoteSpoolSystem.js`:

```js
this.fastModeActive = false;          // constructor; "the last set_fast_mode we sent"
...
worker set: this._worker = value; ... postMessage({type:'set_asap_mode', ...})   // only ASAP
...
} else if (!this.fastModeActive && queueSize < this.lowWaterMark) {
  this.worker.postMessage({ type: 'set_fast_mode', enable: true });
  this.fastModeActive = true;
```

Every place that swaps the worker forgets the two mirror fields: `workerController.js`
`done`/`error` handlers (`remoteSystem.worker = null`) and
`resetRemoteQueue()` (which sets `worker` and `wasPaused = false` but not
`fastModeActive`). Only Finish ASAP clears it (`set_fast_mode false` when
`asapModeActive`). So if the last command of a job was produced while the queue was below
the turn-off point (`max(lowWaterMark, 0.7 × highWaterMark) = 224`), the next job's worker
is never told to enter fast mode.

Fast mode is not a nicety: in all three players it is the only difference between "pace
each command at `dt / speedScale`" and "run ahead and let the world's queue bound you"
(`waitScale = this.fastMode ? Math.max(1, this.speedScale) : 1` in
`workers/moveCommander.js:294,389` and `rrf/rrfCanPlayer.js:349`; `setFastMode()` flushes
immediately in `klipperSimulatorBridge.js:221`). The codebase also works around coarse
timer granularity with a dedicated `Atomics.wait` timing worker
(`integrations/klipper/klipperPacerWorker.js`), which is what limits a
non-fast worker to roughly `1 / timer-resolution` commands per second.

### Experiment

Real `RemoteSpoolSystem` + real `World` + stub workers that record the messages they
receive. Job 1: fresh mirror, empty queue → `set_fast_mode(true)`. Job ends with 200
commands still queued (inside the app's own 160…320 steady-state band). Job 2 attaches a
new worker and drains the queue to 159 (< `lowWaterMark`).

### Expected vs actual

| step | expected | actual |
| --- | --- | --- |
| job 1 worker messages | `set_asap_mode`, `set_fast_mode` | `set_asap_mode`, `set_fast_mode` |
| job 2 worker messages | `set_asap_mode`, `set_fast_mode` | **`set_asap_mode` only** |
| control: fresh world, queue 0 | `set_fast_mode` | `set_fast_mode` |

The failure message is `job 2 worker never received set_fast_mode(true) (mirror leaked
from job 1)`; the control runs the same code path with a fresh mirror and passes, so the
mirror is the only difference.

### Reachability: genuine but second-order

Reachable by any two jobs in one page session, and by Reset, since
`handleUserReset → stopAndClearWorkers → resetRemoteQueue(null)` also leaves
`fastModeActive` untouched. The probability of ending job 1 in the "mirror stuck true"
state is the fraction of time the queue sits between 160 and 224 commands at the last
command (≈40 % of the drain phase, and 100 % for streams shorter than 160 commands). The
consequence is not a wrong number: the second job is fed at wall-clock pace with no
prefetch, so playback throughput is limited by worker timer granularity instead of
physics. I did not measure a wall-clock slowdown, because that requires the browser's
timer clamp, which cannot be reproduced headlessly; I therefore classify this as a genuine
state-machine defect with a performance (not numeric) consequence.

## Negative results (preserved)

* `probe_substeps_cadence.mjs` (exit 0): for `physicsSubsteps` 1, 2, 4, 8, 16 and for
  changes 1→8 and 8→1 mid-run, one `World.update(dt)` still (a) advances the physics
  systems by exactly `dt` (`substeps × dt/substeps`), (b) calls the non-physics systems
  (the command consumer) exactly once with the full `dt`, and (c) leaves the `dt`
  resource at 0.002 s. So a substep change cannot desynchronise the worker's
  one-command-per-tick stream, and a `set_dt` already sent stays valid.
* Late messages from a superseded worker are dropped by
  `remoteSystem.worker === worker` in `workerController.handleWorkerMessage` (exercised by
  the drift-checked 3-line mirror of that branch); a current worker's command is accepted
  (measured 0 accepted from the old worker, 1 from the new one). No duplication: a 1000-command stream produced exactly 1000 samples and 1000
  history entries.
* Pause semantics: with `pauseState.paused = true`, `world.update()` skips
  `RemoteSpoolSystem` (no `runInPause`), the queue is unchanged (2 → 2) and nothing
  executes; unpausing executes exactly one command. Pausing therefore neither loses nor
  duplicates commands — it only lets the queue grow until the backpressure `pause` is
  posted, which is the documented intent.
* The UI's message vocabulary is covered: `set_dt`, `set_speed_scale`, `set_asap_mode`,
  `set_fast_mode`, `pause`, `resume`, `set_kinematics_config`, `set_driver_to_axis`,
  `filename_fetch`, `filename_upload` all have handlers in `workers/moveCommander.js`,
  `integrations/rrf/rrfCanPlayer.js` and `integrations/klipper/klipperMcuCommandPlayer.js`.
  `set_dt` is posted again at every job start (`commandJobController.runPreset`,
  `queueCommandFile`), so a mid-session dt change is not stale for the player.
* `RemoteSpoolSystem.worker` setter re-sends `set_asap_mode` on every attach, and
  `startSimulationWithWorker` re-sends `resume` + `set_speed_scale`, so those three
  pieces of worker state do follow the worker. Only `fastModeActive`/`wasPaused` do not.

## Latent hazards (observed, not claimed as findings)

* `state.simDtSec` is set from the *first* scene's `timeCodesPerSecond` and never updated
  (`machineSceneController.js:551-556, 599-606`); on a later scene with a different
  `timeCodesPerSecond` the code warns "Using existing dt=…" but the world's `dt` *is*
  replaced from the new scene (`sceneResources.js` `world.setResource('dt', …)`), so the
  player would pace against the wrong tick. Every catalogued scene is 500 tcps, so only an
  uploaded non-500 USDA reaches it (`public/usd_scenes/flipper_scene.usda` is 1500 tcps but
  is not in `machines/catalog.json`).
* `encoder_response.simulationClock` pairs `time_ms` from `researchClock.time` with
  `generation` from `sceneGeneration`; they can disagree for one message after a scene
  rebuild (the runner lazily re-keys `researchClock` in `advancePhysics`). Not exercised:
  every non-append rebuild in the app is followed by `resetGame()`, which re-keys the clock
  before the next encoder reply.
* With two machines loaded, `axisToEntity` maps one axis to every machine's spool, so an
  encoder reply returns the first matching entity and one command stream drives all
  machines. Autocal (`simulation_settings.machineType`) always reduces to one machine, so
  the calibration consumer is unaffected.

## Limitations

* The worker is modelled, not run: no `Worker`, no browser, no `setTimeout` clamp, no Vite
  `?worker&url` modules. `/tmp`-free, single process, ≤0.10 s and ≤68 MB RSS per run;
  `free -h` checked before (1.5–1.7 GiB available) and after.
* Defect 1 uses the real quality controller/monitor, the real `RemoteSpoolSystem` and the
  real `World`, but a synthetic path and a synthetic 3 mm tail error. The measured claim is
  the truncation (samples, corners, progress, executed-after-record) and the resulting
  insensitivity; the 5.7-point score drop is illustrative of that sensitivity, not a claim
  about a specific print.
* Defect 2's core claim (the message is absent) is measured; its throughput consequence is
  argued from the players' pacing code and the codebase's own timer workaround, not
  measured in a browser.
* Nothing here is a hardware-validity claim; all evidence is simulation-side.

## Artifacts

| file | what |
| --- | --- |
| `repro_quality_recorded_before_drain.mjs` | one-command reproducer for defect 1 (exit 1) |
| `repro_worker_replacement_state.mjs` | one-command reproducer for defect 2 (exit 1), plus three negative controls |
| `probe_substeps_cadence.mjs` | negative result: substeps keep the command cadence (exit 0) |
| `evidence/*.out`, `evidence/*.err` | captured stdout/stderr, exit codes and `/usr/bin/time -v` |
