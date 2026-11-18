# Flight-recorder and serialization round-trip correctness

Session: `session-2026-11-18-investigate-flight-recorder-and-serialization-round-trip-correct-fe3f2bc3`
Program: `program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7`
Date: 2026-11-18. Code: `74ecdc951666821ac1ba44250fb69badfbc46c26`.
Environment: Linux x64, Node 26.13.0, Python 3.14.4, Rerun 0.38.1, native addon
`src/wasm/cable_solver/target/native/physics.node` (sha256 `4dda8987…` when the final
repros ran; the same numbers were also obtained with the earlier `eef3d79d…` build).

This lens was explicitly listed as uncovered by the 2026-11-16 report (agenda item 1).
It is not a re-report of any prior finding. No source file was modified.

## Short version

The **recorded trajectory round trip is good**: two independent recordings are
semantically identical, the `.rrd` reproduces the telemetry trajectory to 1.9e-9 m,
`sim_step` is `0..N`, `sim_time` is exactly `step*dt`, recording does not perturb
physics, and pause/resume does not move the clock.

The **documented headless-recording contract is not met** in four ways:

1. **The line-length error channel is fabricated.** `commanded` is copied from
   `actual` and `error` is hard-coded to `0.0` for every cable and every sample, so
   `run_experiment` always reports `peak_abs_length_error_m: 0.0`. On HP4 with a
   0.1 rad move the production JS snapshot measures **6.326 mm** peak error, and
   motor tracking error is 0.135 rad. This is a silently wrong counter, not a
   missing optional field.
2. **Headless rigid-member frames are wrong.** All 33 HP4 rigid members are written
   as flat `/bodies/NAME` transforms at their *parent body's world pose*; the worst
   member is **0.424 m** from its true world position. Anchors are also written under
   `/bodies/`, and the documented motor/encoder/velocity/tool-point/extrusion/
   `extrusion_lengths` entity groups are absent from the `.rrd`.
3. **An in-place session reset is not a new generation.** `NativeSession.handle({'type':'reset'})`
   increments `epoch` but `scene_generation` stays `1`, the `.rrd` has no
   `scene_generation` timeline, and `sim_step` restarts at 0. Two different poses are
   stored at the same `sim_step` (4.118 mm apart), so a latest-at replay cannot
   recover the first generation.
4. **(Latent) Extrusion deposit positions are nulled by the worker export.**
   `research_physics.mjs` calls `xyz(r.pos)` on an Array, so a deposit with
   `pos: [0,0,0]` arrives as `pos: [null,null,null]` and decodes to NaN. No current
   consumer reads it, so this is latent.

## Method

Read first: `hp-sim-3d/FLIGHT_RECORDER.md`, `hp-sim-3d/app/flightRecorder.js`,
`hp-sim-3d/app/flightRecorderSnapshot.js`, `scripts/hangprinter_flight_recorder.py`,
`src/python/hp_sim5_research/{experiments,mirror,recording,session}.py`,
`scripts/research_physics.mjs`, and the recorder tests. Then micro-experiments only:
2–10 fixed steps on `public/usd_scenes/hp4.usda` (2 mm), one process at a time,
`timeout 60`, RSS measured with `/usr/bin/time -v`.

Recordings were read back with **`rerun.chunk.RrdReader`** (the SDK reader already used
by the test suite), not by inspecting the viewer. Two recordings were additionally
compared with `rerun rrd compare --unordered` after
`rerun rrd filter --drop-entity /experiment` (the embedded manifest legitimately
differs by run id/path).

The checked-in addon was used through a byte-identical copy outside the checkout:
during the session the default loader considered the addon stale (one Rust source
mismatch) and would have rebuilt it. `HP_SIM5_NATIVE_ENGINE=<copy>` is the loader's
supported way to use an existing artifact. Another process rebuilt the addon at
16:34; the final repros use the rebuilt `4dda8987…` and the earlier probe numbers
were identical.

## Finding 1 — the recorded `commanded`/`error` channel is fabricated (genuine, user-visible)

**Documented contract** (`FLIGHT_RECORDER.md:203-211`):

| trace | definition |
|---|---|
| commanded | predicted paid-out length at the position-mode motor target |
| actual | paid-out rest length |
| error | actual − commanded |
| stretch | geometric − actual |

and `FLIGHT_RECORDER.md:211-214`: torque mode has no commanded target, so its
commanded/error series are *omitted*. `hp-sim-3d/README.md:99-101` turns this into a
user-facing acceptance check: "loss of commanded-length/error traces in torque mode,
and their return when position mode resumes".

**Observed** (`mirror.py:239-240`): `commanded` is set to `actual` unconditionally,
`error` to `actual - actual`; torque mode is not considered. Consequently
`experiments.py:88` (`peak_abs_length_error_m`) can only ever return 0.

Repro: `repro_length_error_channel.py` (HP4, 5 steps, `Move A=B=C=D=0.1 rad`).
It compares the manifest/telemetry/.rrd against the production
`captureFlightRecorderSnapshot` on the same scene and commands
(`snapshot_truth.mjs`).

| quantity | expected | actual |
|---|---|---|
| `metrics.peak_abs_length_error_m` | ≈ 6.326e-3 m | **0.0** |
| telemetry rows with `commanded == actual` and `error == 0.0` | few/none | **54 / 54** |
| `.rrd` `line_errors/*/error` scalars | nonzero where motors lag | **54 / 54 exactly 0.0** |
| `metrics.peak_abs_tracking_error_rad` | nonzero (sanity) | 0.1346 rad |
| first nonzero true error | — | `CablePathAL`: commanded 4.469435 m, actual 4.474976 m, error 5.542e-3 m |
| replay fidelity (control) | ≤ 1e-6 m | 1.893e-9 m |

Tolerance: the true peak error is 6.3 mm, six orders of magnitude above the 1e-6 m
round-trip tolerance; the reported value is structurally exact zero, so no tolerance
choice can make it correct.

Reachability: every documented headless run (`run_experiment(..., record=True)`) and
every `NativeSession` telemetry sample / `.rrd` line-error plot. The browser receiver
is unaffected (its `captureFlightRecorderSnapshot` computes the target with
`cableStoredLengthAfterRotation`, and its test asserts a nonzero error). This is a
shipped, user-visible wrong result for the research API, not an invalid-input hazard.
It also means `scripts/replay_collection.py`'s `max_cable_length_error_mm` cannot see
length errors (it compares 0 with 0).

Evidence: `evidence_length_error_channel.json`, run `01afad00037e41ff80428b07c8424eca`,
`recording.rrd` 744,042 B, sha256 `afe6ee937615d5f064bea0fd18aa7ac45a08d7598f6c1f40cc8719e0e18dadce`.

## Finding 2 — headless `.rrd` / `final.json` scene content is flattened and incomplete (genuine, user-visible on replay)

**Documented contract** (`FLIGHT_RECORDER.md:199-201`): "Each machine has a frame tree
under `world/machines/MACHINE`; rigid members use transforms relative to their parent
body." The "Native recording" column (`FLIGHT_RECORDER.md:162-175`) also promises
motor, encoder, velocity, tool-point, extrusion-deposit and `extrusion_lengths` data.
`hp-sim-3d/README.md:96-103` also asks a reviewer to "inspect ten deposition points
totaling 0.01 m, encoder tracking and zero missed steps" on a native recording.

**Observed** (`mirror.py:199-215`):

- For an entity with `RigidBodyMemberComponent`, the frame position/orientation is the
  **parent body's world pose**, not the member's local transform, and the path is
  `world/machines/MACHINE/bodies/NAME` (no `/members/` level).
- All frames, including anchor-tagged ones, use the `bodies/` category.
- `RerunSystem` (`recording.py`) logs only frames, cable strips/forces and length
  scalars. There are no `motors/`, `encoders/`, `velocities/`, `tool_points/`,
  `extrusions/` or `extrusion_lengths/` entities, and the effector quaternion is
  always identity (`mirror.py:214`).

Repro: `repro_recording_scene_content.py` (1 HP4 sample, read back with `RrdReader`).

| quantity | expected | actual |
|---|---|---|
| rigid members on HP4 | 33 with local offsets | 33, all at body pose |
| worst member position error | ≤ 1e-6 m | **0.424359 m** (`PinholeCR_bottom`) |
| recorded vs truth (`PinholeCR_bottom`) | (−0.381051, −0.18, 0.0405) | (3.7e-11, −5.2e-12, 0.090304) |
| paths containing `/members/` | > 0 | **0** |
| anchor paths containing `/anchors/` | 12 | **0** (all under `/bodies/`) |
| `motors`, `encoders`, `velocities`, `tool_points`, `extrusions`, `extrusion_lengths` | present | **0 paths each** |
| total entity paths | — | 236 |

The same `capture_snapshot` feeds `final.json`, so the final snapshot has the same
33 collapsed members (verified on run `01afad…`: 60 frames, 12 anchors, 1 effector).

The pre-rewrite implementation is decisive evidence of intent: at `817a7773^`,
`machine_snapshot.py:31-42,54-60` computed `local_position`/`local_orientation` and
`Body/members/Member` paths, and `rerun_system.py:166-190` logged motors, encoders,
velocities, tool points, extrusions and deposited length. The legacy-removal rewrite
(`817a7773`, 2026-11-10) replaced this with the flat `mirror.capture_snapshot` and
dropped all of it.

Reachability: every headless recording and every `final.json`. On HP4 replay the
wheels/pinholes/corners collapse onto the effector body; on any scene the
anchor/body distinction, motor/encoder diagnostics and extrusion deposits are
unavailable. This is a visible replay defect (not solver arithmetic), and the doc
table is the contract that is violated.

Evidence: `evidence_scene_content.json`; `repro-output/scene-content/recording.rrd`
672,555 B, sha256 `a0b361684beb8f92001ca6e7e0b2be4de12ef562e80eea17f766434fb74e19e4`.

## Finding 3 — in-place reset keeps generation 1 and collides clocks (genuine, medium reach)

**Documented contract** (`FLIGHT_RECORDER.md:57-59`): "`sim_time` and `sim_step` advance
on positive unpaused updates; scene resets restart them and set the `scene_generation`
timeline."

**Observed** (`session.py:237-250`): reset sets
`sceneGeneration = epoch + 1`, then `_start_physics()` calls
`mirror.apply_state()`, which overwrites it with the fresh worker's constant
`scene_generation = 1`. `recording.py` never writes a `scene_generation` timeline.

Repro: `repro_reset_generation.py` (HP4: move +0.1 rad, 50 steps; reset; move −0.2 rad,
50 steps; one `.rrd`).

| quantity | expected | actual |
|---|---|---|
| `epoch` | 0 → 1 | 0 → 1 |
| `scene_generation` after reset | 2 | **1** |
| `scene_generation` timeline in `.rrd` | present | **absent** (`log_time, sim_step, sim_time` only) |
| `events.jsonl` generations | {1, 2} | **{1}** |
| `sim_step` | unique within a generation | **`[0, 50, 0, 50]`** |
| same-index poses | identical or disambiguated | two poses at step 50 differ by **4.118 mm** |

A latest-at query at `sim_step=50` (what a viewer does when scrubbing) sees only the
later-logged generation; generation 1's data is shadowed. The browser receiver already
does this correctly: `scripts/hangprinter_flight_recorder.py:195-221` sets
`scene_generation` and clears previous paths on a session/generation change.

Reachability: calling the documented bridge `reset` while a `NativeSession` is
recording, e.g. the session API used by `test_native_collector.py:62`. The production
runtime's `reset_session` currently creates a new world and a new recording file, so
not every production reset hits the collision; it is still a supported in-place reset
whose recording is not replayable per the documented invariant. Classify as genuine
but medium reach, not an invalid-input hazard.

Evidence: `evidence_reset_generation.json`; `repro-output/reset-generation/recording.rrd`
712,708 B, sha256 `01461d76f3bb29aa0117e1ef4f5fc2ea3f3f5a528dcc085a972ad124ef73a767`.

## Finding 4 — extrusion deposit positions are nulled by the worker export (latent)

`ExtruderComponent.extrusions` rows are `{pos: [x,y,z], length, machineId, color}`
(`remoteSpoolSystem.js:335-340`). `scripts/research_physics.mjs:45` exports them with
`xyz(r.pos)`, which reads `.x/.y/.z` from an Array; JSON therefore contains
`[null,null,null]`, and `mirror._decode` turns that into `[nan,nan,nan]`.

Repro: `repro_extrusion_position.py` (HP4, 10 steps, `Move E=0.2`):

| quantity | expected | actual |
|---|---|---|
| JS deposit | `{pos: [0,0,0], length: 0.2, …}` | same |
| worker-exported `pos` | `[0.0, 0.0, 0.0]` | **`[null, null, null]`** |
| Python-decoded `pos` | finite | `[nan, nan, nan]` |

No current consumer reads `ExtruderComponent.extrusions` (the recorder omits deposits
entirely — Finding 2), so this is latent. It is still a serialization round-trip loss
of the only position carried by a deposit.

Evidence: `evidence_extrusion_position.json`.

## Negative results (preserved)

- **Replay is deterministic and faithful.** Two independent 5-step `record=True` runs
  are semantically identical after dropping `/experiment`:
  `rerun rrd compare --unordered` exits 0. The `.rrd` effector translations match
  `telemetry.jsonl` to **1.893e-9 m** (tolerance 1e-6; Rerun `Transform3D`/`Points3D`
  are f32), and `sim_step` is `0..5`, `sim_time` `0, 0.002, …, 0.010 s`.
  `check_replay_roundtrip.py` (exit 0), `evidence_replay_roundtrip.json`.
- **Recording does not perturb physics.** A `record=True` and a `record=False` 10-step
  run produce identical `effectors`/`motors`/`cables` on 11/11 rows and identical
  metrics (`logs/check_record_perturbation.out`).
- **Pause/resume and generation changes are correct in `FlightRecorder`.**
  Step/time stay at 3/0.006 s while paused (the runner stops ticking), resume to
  4/0.008 s, and a generation change restarts to 1/0.002 s; sample steps are
  `0,1,2,3,4,1`. `check_pause_resume.mjs` (exit 0).
- **The only export/import feature works.** `archive.export_handoff` /
  `read_handoff` round-trips selected bytes with `verified_selected_files`, all hashes
  match, and a tampered copy is detected (`partial`, SHA-256 mismatch).
  `check_handoff_roundtrip.py` (exit 0).
- **No recording export/import or resume-from-recording exists.** `.rrd` is written
  by the recorders and can only be read by a generic Rerun reader (`RrdReader`); no
  code restores physics state from a recording. The documented "resume" actions are
  browser pause/resume and workspace resume, not recording replay.
- **Seek within one generation is sound.** `sim_step` values are unique and sorted in
  the plain runs; `sim_time` is a duration index; `line_lengths`, `line_errors`,
  `cable_forces` have stable paths.
- **Torque-mode omission is not separately reachable in a useful way** — the same
  hard-coded `commanded = actual` keeps `error` at 0 instead of omitting the series,
  so Finding 1 covers it.

## Minor observations (not counted as findings)

- The `.rrd`'s static `/experiment` text is the pre-run manifest: `status: "running"`,
  no `metrics` (verified on run `01afad…`). The final manifest is only in
  `manifest.json`; a replay alone cannot report the final counters.
- `RerunSystem` logs row-by-row, not through `send_columns()`, so the doc statement
  that "both recorders batch raw telemetry into `send_columns()` calls" and "do not
  add the SDK's automatic `log_time`" no longer holds for the Python path; every data
  chunk carries `log_time` in addition to `sim_step`/`sim_time`. This is doc drift and
  a likely size/performance regression, not a wrong value.
- `mirror.py:214` always logs the effector quaternion as identity, although
  `FLIGHT_RECORDER.md:190-192` says rigid bodies supply the orientation. HP4's
  effector is unrotated in these runs, so no error was measured; 6-DOF scenes would
  show it.

## Limitations

- All evidence is simulation; no hardware validity is claimed.
- HP4 is the main fixture (33 members, 12 anchors, 9 cables). The recorder code paths
  are scene-independent, but member counts and local offsets differ by scene.
- The checked-in native addon was used, not rebuilt by this session. It was rebuilt
  by another process mid-session; the reported numbers were identical before and
  after, and the findings are in JS/Python serialization code, not the solver.
- No browser, Vite server or Rerun viewer was run (host limit). Browser-receiver
  behavior is taken from `scripts/hangprinter_flight_recorder.py` and its tests, not
  from a live viewer session.
- Rerun stores `Transform3D`/`Points3D` as f32; "exact replay" means f32-exact.
  Length/force scalars are f64.
- No test suite was run, and no source file was edited. The repros are research
  scripts, not CI tests.

## Reproducers and commands

Run from `/home/torbjorn/repos/hp-sim5`:

```bash
S=research/output/program-2026-11-18-which-new-minimal-reproducible-cases-exist-where-hp-sim5-returns-600802e7/session-2026-11-18-investigate-flight-recorder-and-serialization-round-trip-correct-fe3f2bc3

# Finds the faults (each exits 1 and prints expected vs actual):
.venv/bin/python $S/repro_length_error_channel.py          # Finding 1
.venv/bin/python $S/repro_recording_scene_content.py       # Finding 2
.venv/bin/python $S/repro_reset_generation.py              # Finding 3
.venv/bin/python $S/repro_extrusion_position.py            # Finding 4 (latent)

# Negative controls (each exits 0):
.venv/bin/python $S/check_replay_roundtrip.py
node $S/check_pause_resume.mjs
.venv/bin/python $S/check_handoff_roundtrip.py
```

Each script copies the checked-in addon to `/tmp/opencode` and sets
`HP_SIM5_NATIVE_ENGINE` itself; no environment setup is required.

Evidence files: `evidence_length_error_channel.json`, `evidence_scene_content.json`,
`evidence_reset_generation.json`, `evidence_extrusion_position.json`,
`evidence_replay_roundtrip.json`, plus captured stdout/stderr in `logs/`.

## Resource notes

Every run was one process at a time with `timeout 60`. Peak RSS: 204,664 KiB
(Finding 1), 122,256 KiB (Finding 2), 127,836 KiB (Finding 3), 89,860 KiB
(Finding 4), 200,076 KiB (replay control); wall time 1.0–2.9 s. `free -h` stayed at
≥1.5 GiB available during the work; swap usage was already ~1.1–1.4 GiB before these
runs and did not grow materially. No OOM or timeout occurred.
