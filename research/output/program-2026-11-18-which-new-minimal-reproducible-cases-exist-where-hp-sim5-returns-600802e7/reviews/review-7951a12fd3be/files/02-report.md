# Bookkeeping and telemetry conservation in hp-sim5

Session: `session-2026-11-18-investigate-bookkeeping-and-telemetry-conservation-including-ext-b4ebb599`
Program: *New reproducible incorrect-behaviour cases in hp-sim5* (2026-11-18)
Revision under test: `74ecdc951666821ac1ba44250fb69badfbc46c26` (submodules `RRF/ReprapFirmware`, `klipper`,
`flex-compensation-dev/...`, `forward-transform-dev/...` dirty, as found)
Date: 2026-11-18. Environment: Linux x64, Node 26.13.0, Python 3.12 (`.venv`).

**Verdict.** Two genuine bookkeeping defects were confirmed, plus one documented-telemetry gap:

1. **Python/native length telemetry is structurally wrong**: `mirror.capture_snapshot` writes
   `commanded = actual` and `error = actual - actual = 0` for every cable, and does not suppress either
   value in torque mode. On the same world where the JS recorder reports a **15.087 mm** length error, the
   Python telemetry and every native recording report **0.000000 mm**, and the `peak_abs_length_error_m`
   metric is always zero.
2. **The G-code upload path mis-accounts extrusion**: `moveCommander.js` starts in relative-E mode (both
   RRF and Klipper default to absolute), ignores `M83`, ignores `G92 E`, and keeps its `last_e` odometer
   across jobs. On the shipped PrusaSlicer file it sends **49,719.25 mm** of extrusion where the app's own
   reference parser and the RRF compilation both say **49,950.46 mm** (−231.21 mm, −0.463 %). Inputs relying
   on the firmware's absolute-E default are over-extruded by 50 % on a two-move file (337× on the shipped
   file's E values).
3. **Native recordings do not contain the extrusion/tool-point telemetry the manual advertises**: no
   `extrusion_lengths/*` or `tool_points/*` is ever logged, and the Python mirror ignores
   `ExtruderComponent.extrusions` entirely.

The preset/RRF playback path conserves extrusion to **−0.0015 mm (−2.9e-8)** on the same file, and JS and
Python agree exactly on `actual`, `geometric`, `stretch` and force values. No power/energy/odometer
accumulator exists to test. Details, negative controls and limitations below.

---

## Method

Read-first investigation of every accumulator/consumer involved in deposition and line material:

- `hp-sim-3d/app/workers/moveCommander.js` (G-code upload → `Move`/`E` commands),
  `integrations/rrf/rrfCanPlayer.js`, `integrations/klipper/klipperMotionCore.js`,
  `integrations/klipper/klipperPacerWorker.js` (preset and Klipper players).
- `hp-sim-3d/app/remoteSpoolSystem.js` (`ExtruderComponent.extrusions`, `onExtrusion`, command queue),
  `hp-sim-3d/app/hangprinter_extruder.js`, `render_system_3d.js` (`_syncExtrusions`),
  `hp-sim-3d/app/quality-monitor.js` (`totalExtrusion`, `extrusionCursor`).
- `hp-sim-3d/app/flightRecorderSnapshot.js` / `flightRecorder.js` (browser recording),
  `src/python/hp_sim5_research/mirror.py` (Python observation mirror and native recording),
  `src/python/hp_sim5_research/recording.py` (`RerunSystem`), `experiments.py` (`metrics`),
  `scripts/research_physics.mjs` (worker protocol), `hp-sim-3d/FLIGHT_RECORDER.md`.
- `src/wasm/cable_solver/src/engine.rs` motor/encoder/slip bookkeeping.

Then micro-experiments only: one 6-step native run, one 3-sample Wasm/native probe, and pure text-level
drivers for the G-code players (no physics). No browser, no Vite, no full print, no test suite.

**Implementation note.** The checked-in native addon was stale by source hash at the start of the session.
Running the production Python path for evidence auto-invoked `npm run build:native` (≈14 s, one core) and
refreshed it (`physics.node.json` artifact `4dda8987…`). The reported length values were reproduced
independently on the checked-in Wasm build and agree to 1e-9 m, so the conclusions do not depend on that
rebuild. See *Resource notes*.

---

## Finding 1 — Python/native length telemetry aliases `commanded` to `actual`

**Where.** `src/python/hp_sim5_research/mirror.py`, `capture_snapshot`, cable `lengths` dict:

```python
guides = sum(path.stored[1:-1]) if path.stored else 0.
actual = sum(segment['rest_length'] for segment in segments) + guides
geometric = sum(segment['geometric_length'] for segment in segments) + guides
...
'lengths': {'commanded': actual, 'actual': actual,
            'geometric': geometric, 'error': actual - actual,
            'stretch': geometric - actual}
```

`error` is literally `actual - actual`, so it is bitwise zero for every cable on every step. The JS
recorder (`flightRecorderSnapshot.js:collectCables`) computes `commanded` from the position-mode motor
target with `cableStoredLengthAfterRotation`, sets `error = actual - commanded`, and sets both to `null`
when the endpoint motor is in torque mode (matching `FLIGHT_RECORDER.md`: “Torque mode has no
commanded-length target, so its commanded and error series are omitted”).

**Measured.** Same world (`public/usd_scenes/slideprinter.usda`, no layering, dt 0.002), one `Move A=0.5 rad`
then 4 steps. JS and Python are fed the same state (the probe prints both from one process).

| cable | JS commanded (m) | JS error (m) | Python commanded (m) | Python error (m) |
|---|---|---|---|---|
| `CablePathA` | 1.917265792 | **−0.015086662** | 1.902179130 | **0.0** |
| `CablePathB` | 1.902260241 | +0.000006931 | 1.902267172 | **0.0** |
| `CablePathC` | 1.902262889 | +0.000006884 | 1.902269773 | **0.0** |

After `SetTorqueMode C torqueNm=0.02`, the JS recorder reports `CablePathC` `commanded = null`,
`error = null`; the Python mirror reports `commanded = 1.902308667 m`, `error = 0.0`.

Tolerance: the two implementations must describe the same world, so `|Δcommanded| ≤ 1e-9 m`. Actual
disagreement is 15.0867 mm. Fields the mirror does compute are faithful: `actual` differs by 0.0 m,
`geometric` and `stretch` by at most 2.2e-16 m (summation order).

**Production evidence.** A real `run_experiment` (6 steps, native, scene as above, `Move A=0.5`) produced:

```json
"metrics": {"peak_abs_length_error_m": 0.0, "peak_abs_tracking_error_rad": 0.5032862095, ...}
```

The same run's `telemetry.jsonl` has `commanded` numerically identical to `actual` and `error: 0.0` on every row.
`peak_abs_length_error_m` is therefore not a measurement: it cannot become nonzero.

**Reachability.** Every Python `run_experiment`/`read_run`, every research-MCP numeric telemetry read,
every native `.rrd` (`line_lengths/*/commanded`, `line_errors/*/error`), and the `peak_abs_length_error_m`
metric. The browser HUD/recording uses the JS path and is unaffected. **Genuine, not latent**; severity is
to research/telemetry trust rather than to the physics: `actual`, `stretch` and forces remain usable, and
the physics itself is not changed.

**Reproducer.** `repro_length_telemetry.py` (spawns `probe_length_telemetry.mjs`; exits 1 and lists the
15 mm and torque-mode disagreements). Output: `logs_length_telemetry.txt`,
`probe_length_telemetry.json`.

---

## Finding 2 — G-code upload extrusion bookkeeping (`MoveCommander`)

**Where.** `hp-sim-3d/app/workers/moveCommander.js`. The `.gcode` upload path
(`FileFormat.GCODE` → `queueCommandFile` → `workers.ensureMoveWorker()`) is the only consumer of this
module. Presets play compiled RRF CAN captures instead (`commandPresetResolver.js`), so they are unaffected
(negative control below).

Four independent bookkeeping errors, all in extrusion accounting:

1. **Default extrusion mode contradicts the firmware.** `this.absolute_extrusion = false` (line 41);
   RRF initialises `drivesRelative(false)` (`RRF/.../GCodeMachineState.cpp:21`) and Klipper
   `absolute_extrude = True` (`klipper/klippy/extras/gcode_move.py:43`). The app's own reference parser
   defaults to absolute (`inspectionToolsController.js:74`). A file that omits `M82` is therefore parsed
   correctly for the reference overlay but simulated as if every `E` value were a delta.
2. **`M83` is ignored.** The run loop only recognises `M82` (lines 208/216); a later `M83` leaves
   `absolute_extrusion = true`.
3. **`G92 E` is ignored.** `_parse_g92_command` reads only `X/Y/Z` (lines 127–139), and returns `null`
   when there are no coordinates, so `G92 E0` neither consumes a command nor resets `last_e`.
4. **`last_e` survives jobs.** `configure()` resets `current_angles_rad`, anchors and spool params
   (lines 55–78) but not `last_e`, `current_pos_mm` or `absolute_extrusion`. A second upload on the same
   worker starts with the previous job's extruder odometer.

**Synthetic results** (reference = `parseGcodeText`, the same function that builds the reference overlay
and quality-monitor segments):

| input | reference | simulation | ratio |
|---|---|---|---|
| control `M82; G1 X10 E5; G1 X20 E10` | 10 mm | 10 mm | 1.000 |
| `G92 E0; G1 X10 E5; G1 X20 E10` (firmware default, no `M82`) | 10 mm | 15 mm | **1.500** |
| `M82; G92 E0; G1 X10 E5; G1 X20 E10; G92 E0; G1 X30 E5` | 15 mm | 10 mm | **0.667** |
| `M82; M83; G1 X10 E5; G1 X20 E5` | 10 mm | 5 mm | **0.500** |
| second job `M82; G92 E0; G1 X10 E5` after a 100 mm job | 5 mm | 0 mm | **0.000** |

**Shipped-file result.** `public/gcode/Hangprinter_logo6.gcode` (PrusaSlicer 2.9.3, 20,017 `G1` lines,
123 `G92 E` lines/121 distinct resets, 121 retract/unretract pairs of 2 mm):

- reference positive extrusion (`parseGcodeText`): **49,950.460240 mm**
- `MoveCommander` positive extrusion: **49,719.253160 mm**
- difference: **−231.207080 mm (−0.462873 %)**, tolerance 0.01 mm.

Mechanism: at 119 resets the post-reset unretract `G1 E2` is dropped because the stale `last_e` makes the
delta negative (sum −233.207 mm); one reset (the initial one) over-counts +2 mm; net −231.207 mm. The
`G92 E0` lines are not pathological: they are how this slicer emits a retract/wipe sequence, and the same
file is shipped in the repository as the reference preset.

**No-`M82` variant.** If the file's single `M82` line is removed (a valid file for either firmware, whose
default is absolute), the same 49,950.46 mm of filament becomes **16,835,596.18 mm** (337.05×), because
every cumulative `E` is treated as a per-move delta.

**Consequence.** The simulation deposits 231.21 mm less filament than the reference path and never emits
the 121 unretract primes. The quality monitor advances `extrusionCursor` by the event lengths against
`totalExtrusion = 49,950.46`, so a completed upload ends at **99.537 %** progress (derived:
1 − 231.207/49,950.46), and the per-reset 2 mm segments are never matched with samples. The motion/geometry
bookkeeping is unaffected.

**Reachability.** Interactive `.gcode` upload (`handleUpload` → `queueCommandFile`) on any machine;
`M82`-bearing files are the normal case and still lose 0.46 %. **Genuine** for the upload path; **latent**
for files without `M82`/with `M83` in the shipped catalog (no such file). The cross-job state leak is
reachable by uploading a second file after a print finishes without pressing Reset (`finishJob` does not
terminate the worker; `ensureReadyForNewJob` only resets when a print is still active).

**Reproducers.**

- `node repro_gcode_extrusion_bookkeeping.mjs` — four synthetic cases + the cross-job leak; exits 1.
- `node repro_shipped_gcode_extrusion.mjs` — the shipped-file comparison; exits 1.
- Logs: `logs_gcode_bookkeeping.txt`, `logs_shipped_gcode.txt`.

---

## Finding 3 — Documented native extrusion telemetry is not produced (gap, not a wrong number)

`hp-sim-3d/FLIGHT_RECORDER.md` advertises for native recordings:

- “Tool center/root/tip/cold-end points and extrusion deposits — `world/machines/MACHINE/tool_points`
  and `extrusions`”
- “Total deposited length — `extrusion_lengths/MACHINE`”

Neither string exists in shipped code (grep over `src/`, `hp-sim-3d/`, `integrations/` finds only the
documentation). `RerunSystem.update` logs only frames and cables; `mirror.capture_snapshot` reads
`ExtruderComponent` only for effector centres, never `extrusions`; `mirror.sample` has no extrusion field.
So a Python/native run cannot report deposited length, filament consumed, or tool-point history, and the
documented paths are dead.

Related latent unit-contract hazard: the headless/MCP command contract documents `E` as **metres**
(`scripts/hp_sim5_mcp.py`: `'E': 'deposited length in m'`; `FLIGHT_RECORDER.md`: “metres of extrusion”),
while the browser G-code/RRF/Klipper producers put **millimetres** into the same `command.E` /
`ExtruderComponent.extrusions[].length` field. No shipped code mixes the two in one run, so this is latent,
but any cross-path comparison of deposited length would be off by 1000×.

---

## Negative results and controls

1. **The preset / RRF-CAN path conserves extrusion.** Summing the E driver's positive steps in
   `public/RRF_CAN_commands/Hangprinter_logo6_hp4_no_buildup.can` (driver 44, 200,416 moves) and converting
   with `M92 E95.922` gives **49,950.458783 mm** vs the reference **49,950.460240 mm** — difference
   **−0.001457 mm (−2.9e-8 relative)**, well inside one step of quantisation. Net E after retracts is
   49,708.458789 mm ≈ 49,950.46 − 242.0 (121 × 2 mm retracts; the 0.0015 mm gap is quantisation). The HP3,
   CubeCorners and Skycam compilations give the same positive total (49,950.458783 mm). Preset playback is
   therefore clean; the defect is isolated to `MoveCommander`.
   Evidence: `check_rrf_extrusion_conservation.mjs`, `logs_rrf_conservation.txt`.
2. **JS and Python agree on everything except `commanded`/`error`.** Across the three probe samples:
   `actual` max |Δ| = 0.0 m; `geometric` and `stretch` max |Δ| = 2.2e-16 m. Cable force and force-vector
   values come from the same joint fields and are not separately accumulated. The Python mirror is a
   faithful observation model apart from Finding 1.
3. **No power/energy/odometer accumulators exist.** A repository-wide search finds “energy” only in Rust
   tests (`src/wasm/cable_solver/tests/`), not in shipped telemetry. Force is instantaneous per segment;
   the only odometers are `EncoderComponent.angle` (bit-identical in JS and Python) and the players'
   per-bucket accumulators. There is nothing to double count, and no unit error was found in the shipped
   configs (`M92 E95.922` matches `EXTRUDER_MM_PER_STEP_RRF = 1/95.922`; Klipper's `33.5/(200·16)` matches
   the bundled example configs).
4. **Extrusion sign and pause/resume are handled correctly.** In a 5-command probe: `E=5` produces one
   5 mm deposit; `E=-2` (retract) and `E=0` add nothing; with `pauseState.paused = true` a queued `E=3`
   command stays queued and deposits nothing; on resume it deposits once. Evidence:
   `probe_extrusion_pause.mjs`, `logs_extrusion_pause.txt`.
5. **Reset does not leak deposition.** The production Reset path rebuilds the scene
   (`sceneResources.applySceneResources` calls `world.clear()` for the non-appended first machine), so the
   `ExtruderComponent` and its `extrusions`, the motor slip counters and `StepperMotorComponent` targets are
   recreated; `stopAndClearWorkers()` → `resetRemoteQueue()` clears the remote history/queue and the rebase
   set. No reset leak was found in the deposition path itself. (The `last_e` worker leak across jobs is the
   exception, reported as Finding 2.4; it is not cleared by Reset-free job transitions.)
6. **No double counting in bucket emission.** RRF `distributeEvenly` assigns an explicit residual to the
   tail bucket and both players force-flush at end of stream; the CAN totals above confirm conservation.
   The known float-step-as-extrusion case (multi-driver CAN) was already reported by the 2026-11-17
   program and was not re-investigated.

---

## Limitations

- All evidence is simulation; no hardware validity is claimed.
- No browser/UI run (resource limits). The UI path was established by reading `handleUpload`,
  `queueCommandFile`, `workerController`, `startSimulationWithWorker` and `finishJob`; the G-code driver
  itself was executed headlessly with `dt = 10 s` and no pacing. Extrusion sums are dt-independent by
  construction (per-move `E` is divided over `num_steps` and summed back), and the 49,719.253160 mm total
  was reproduced by an independent text-level emulation.
- The quality-monitor “99.537 % progress” is derived from its documented linear accumulator, not measured
  in a browser over a full print.
- The reported length values were produced on both the checked-in Wasm build and the (auto-rebuilt) native
  addon; they agree to 1e-9 m. The native rebuild is disclosed above; no source file was modified.
- Multi-machine (two loaded scenes) extrusion attribution was not tested. `RemoteSpoolSystem` records one
  extrusion event per machine touched by a command's axis keys, so a single command stream that names
  shared axes could deposit the same `E` on every loaded machine. This is an open question, not a finding.
- The E-as-metres vs E-as-millimetres contract mismatch is latent: no shipped run mixes the producers.

---

## Exact commands

From the artifact directory unless noted. All runs used `timeout 60` and one process at a time.

```bash
# Finding 1 (cross-language, checked-in Wasm; ~1 s)
.venv/bin/python repro_length_telemetry.py          # exits 1

# Finding 2 (synthetic G-code cases; ~2 s)
node repro_gcode_extrusion_bookkeeping.mjs          # exits 1

# Finding 2 (shipped PrusaSlicer file; ~4 s, ~90 MB RSS)
node repro_shipped_gcode_extrusion.mjs              # exits 1

# Negative control (RRF preset conservation; ~2 s)
node check_rrf_extrusion_conservation.mjs           # exits 0

# Secondary ledger/pause probe; ~1 s
node probe_extrusion_pause.mjs

# Production Python path used for the metric evidence (from repo root)
PYTHONPATH=src/python \
HP_SIM5_SESSION_DIR=<this session dir> \
.venv/bin/python - <<'PY'
from hp_sim5_research.experiments import run_experiment
m = run_experiment('.', scene='public/usd_scenes/slideprinter.usda', steps=6,
                   commands=[{'type':'Move','A':0.5,'B':0,'C':0},{},{},{},{},{}], record=False)
print(m['metrics'])
PY
# -> peak_abs_length_error_m == 0.0, peak_abs_tracking_error_rad == 0.5033
```

## Evidence files

| file | contents |
|---|---|
| `report.md` | this report |
| `measurements.json` | every number quoted above, with provenance |
| `repro_length_telemetry.py` | Finding 1 reproducer (exits 1) |
| `probe_length_telemetry.mjs` | one-world JS-recorder + worker-state probe |
| `probe_length_telemetry.json` | raw probe output (states + JS lengths) |
| `logs_length_telemetry.txt` | Finding 1 run log |
| `repro_gcode_extrusion_bookkeeping.mjs` | Finding 2 synthetic/cross-job reproducer (exits 1) |
| `logs_gcode_bookkeeping.txt` | its output |
| `repro_shipped_gcode_extrusion.mjs` | Finding 2 shipped-file reproducer (exits 1) |
| `logs_shipped_gcode.txt` | its output |
| `check_rrf_extrusion_conservation.mjs` | negative control, preset path (exits 0) |
| `logs_rrf_conservation.txt` | its output |
| `probe_extrusion_pause.mjs` | E-sign and pause/resume ledger probe |
| `logs_extrusion_pause.txt` | its output |
| `experiments/492c707f966648e8b9e5ac2e1aecd478/` | production `run_experiment` manifest, telemetry, commands, scene |
| `experiments/.../manifest.json` | `metrics.peak_abs_length_error_m == 0.0` with a 0.5033 rad tracking error |

## Resource notes

- `free -h` before/after each run: available memory stayed between **1.5 GiB and 1.8 GiB**; the smallest
  observed was 1.5 GiB. No run exceeded ~90 MB RSS. All heavy-limit rules respected except the automatic
  native rebuild described below (one core, ~14 s).
- One process at a time; `timeout 60` on every run; no browser, Vite, test suite, build sweep, or full
  simulation.
- **Disclosure:** the first production Python run (`run_experiment`) found the native addon stale and
  auto-ran `npm run build:native` (8.5 s instrumented + 5.4 s release/PGO) before completing. This was not
  a deliberate build; it refreshed `src/wasm/cable_solver/target/native/physics.node` (gitignored) and left
  `pgo-*` build directories. No source file was edited. The Wasm-based probes were run afterwards and
  agree with the native numbers to 1e-9 m.
