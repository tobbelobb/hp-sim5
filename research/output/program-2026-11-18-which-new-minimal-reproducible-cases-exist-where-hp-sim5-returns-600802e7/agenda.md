# New reproducible incorrect-behaviour cases in hp-sim5 (2026-11-18)

Goal: find **new**, reproducible incorrect behaviour that the two earlier correctness
programs did not already report. For each genuine case: the smallest possible
one-command reproducer and quantitative evidence (expected vs actual, error size,
tolerance, revision). Negative results are valuable and must be preserved.

Scope exclusions:

- Do not fix anything. No source edits. Researchers write only inside their session dir.
- Encoder autocalibration is owned by a separate program: do not investigate it.
- No rebuilds, no full simulations, no long benchmark sweeps.
- **Host is small: 3.8 GiB RAM, 2 GiB swap (already partly used), 2 cores.**
  Read files first; run only micro-experiments (target < 30 s, < 300 MB RSS, one
  process at a time, always with `timeout`). Check `free -h` before/after. Never run
  the full test suite, a browser, a Vite server, or a >10k-step simulation.

## Already found (do not re-report unless you find the prior claim is wrong)

2026-11-16 program: hp5/hp3 layered bake slack; stale "Rebase reference" surviving
Reset; layering toggle during Finish ASAP skipping rebake; native/Wasm friction
divergence; unbounded encoder phase unwrap.

2026-11-17 program (with independent review): USD loader ignores `xformOpOrder`;
`metersPerUnit` ignored; missing spool attribute leaves dangling joints; forward-
transform dataset unit labels too weak; RRF host standstill off-by-one; multi-driver
CAN unrepresentable; Klipper MCU player drops first queue_step batch; raw Klipper
E→D axis mapping; NaN poisoning; layering `split()` NaN radius; scene-plan ambient
layering flag; `rotateAroundAxis` zero axis; `normalize()` overflow; layered preset
radius scale; CubeCorners tip path. Also its strong negatives: physics closed forms,
state lifecycle/determinism, Wasm/native parity, firmware planning, forward transform,
catalog consistency, per-machine direction/sign.

## Deliberately uncovered areas (candidate lenses)

The 2026-11-16 report itself lists these as not covered:

1. **Flight-recorder / serialization round trip.** Record a run, replay it, seek,
   pause/resume, export/import, and resume from an imported recording. Does replay
   reproduce the recorded trajectory bit-for-bit or within a stated tolerance? Are
   counters (time, steps, line material, extrusion, energy) preserved across save/load
   and record→replay? (`hp-sim-3d/FLIGHT_RECORDER.md`, recorder modules.)
2. **Bookkeeping / telemetry conservation over long runs.** Extrusion length, filament
   consumed, line material, force/power/energy accumulators, odometers: do they match
   closed-form/analytic expectations and stay consistent between JS and Python/Rust
   readouts? The prior program explicitly skipped extrusion bookkeeping.
3. **External command socket / bridge protocol lifecycle.** Multi-client attach and
   detach, reconnect, queue overflow/backpressure, malformed messages, pause/resume,
   `set_simulation_settings`, time-scale, and ordering of `Reset` vs queued commands
   (distinct from the already-known rebase leak). Does the documented protocol behave
   as documented under two clients?
4. **Ingestion formats not covered.** `.can` files, RRF binary/G-code capture parsing,
   CSV exports, scene/config import/export, and the converter between formats: are
   round trips lossless and are documented units/axes honored? (Avoid re-reporting the
   multi-driver CAN and raw-Klipper findings.)
5. **Simulator service / worker boundary.** Message protocol between UI, worker and
   headless world: dropped/duplicated updates, stale state after reconfigure, quality
   or substep changes mid-run, pause semantics, and whether the worker's reported state
   matches the authoritative world. Testable headlessly without a browser.
6. **Numerical integration accuracy on analytic systems** (as opposed to invariants):
   damped harmonic oscillator, pendulum, Atwood with inertia, constant-acceleration
   free mass, rigid body rotation. Compare against closed-form/analytical solutions at
   the documented default dt; quantify order of accuracy and whether documented
   tolerances/claims are violated. Prior program checked statics/closed forms, not
   time-integration accuracy.
7. **CLI/scripts and Python analysis layer.** `scripts/`, `src/js/python`, benchmark
   helpers, calibration-data readers: unit conversions, axis conventions, aggregation,
   and agreement with the JS implementation. Prior parity check covered kinematics only.

## Current understanding

### Confirmed 2026-11-18 (bookkeeping lens, manager re-ran both headline repros)

1. **Python/native length telemetry aliases `commanded` to `actual`.**
   `src/python/hp_sim5_research/mirror.py::capture_snapshot` writes
   `commanded = actual` and `error = actual - actual`, so `error` is always exactly
   0. On one slideprinter world the JS recorder reports 15.0867 mm of length error
   while Python and every native recording report 0.000000 mm; torque mode should
   omit both series but Python still reports them. `peak_abs_length_error_m` can
   never be nonzero. Genuine telemetry defect, not a physics defect. Repro:
   `session-...-ext-b4ebb599/repro_length_telemetry.py` (exit 1, re-run by manager).
2. **G-code upload path (`moveCommander.js`) mis-accounts extrusion.** Starts in
   relative-E mode although RRF and Klipper default to absolute, ignores `M83`,
   ignores `G92 E`, and keeps `last_e` across jobs. Shipped
   `public/gcode/Hangprinter_logo6.gcode`: reference 49,950.460 mm vs simulation
   49,719.253 mm (-231.21 mm, -0.463 %); without the file's single `M82` the same
   file becomes 337x over-extruded. Preset/RRF playback is a clean negative control
   (-0.0015 mm). Repro: `repro_gcode_extrusion_bookkeeping.mjs` and
   `repro_shipped_gcode_extrusion.mjs` (both exit 1, re-run by manager).
3. Documented native extrusion/tool-point telemetry (`extrusion_lengths/*`,
   `tool_points/*`) is never logged; latent metres-vs-millimetres E contract
   mismatch between the headless API and browser players.

### Confirmed 2026-11-18 (recorder/serialization lens, manager re-ran both headline repros)

4. **Native recordings write every rigid member at its parent body's pose.**
   All 33 HP4 members are recorded at one identical transform under flat
   `/bodies/NAME` paths (no `/members/`, no `/anchors/`); worst member
   `PinholeCR_bottom` is 0.424359 m from its true world pose. Motors, encoders,
   velocities, tool points and extrusion deposits are absent from the `.rrd`.
   This corroborates and quantifies finding 3 above. Repro:
   `session-...-round-trip-correct-fe3f2bc3/repro_recording_scene_content.py`
   (exit 1, re-run by manager; 672 KB `.rrd`, 236 entity paths).
5. **Reset is not a distinct replayable generation.** An in-place reset keeps
   `scene_generation=1`, writes no `scene_generation` timeline, restarts the clock
   but reuses `sim_step` 0..50, so two poses 4.118 mm apart share `sim_step=50`
   and the first generation is shadowed on replay. Repro:
   `repro_reset_generation.py` (exit 1, re-run by manager).
6. Corroboration of finding 1: `peak_abs_length_error_m` is 0 while the JS
   snapshot measures 6.326 mm; 54/54 telemetry and `.rrd` error values are 0.
   Latent: extrusion deposits round-trip `pos: [null,null,null]` to NaNs.

Recorder negatives: replay/round trip is deterministic and sound (two recordings
equal; `.rrd` vs telemetry <=1.9e-9 m; recording does not perturb physics; pause/
resume and generation changes work in `FlightRecorder`); handoff export/import
verifies hashes and detects tampering; there is no recording export/import or
resume-from-recording feature. The pre-rewrite code (`817a7773^`) did compute the
missing hierarchical frames and channels — the legacy-removal rewrite dropped them.

### Confirmed 2026-11-18 (worker-boundary lens, manager re-ran both repros)

7. **The Quality History entry is taken before the world drains its command
   queue.** The worker's `done` means "no more commands will be sent", but
   `finishJob()` treats it as "print finished", so `quality.runFinalQualityChecks`
   snapshots while `RemoteSpoolSystem` still holds up to 320 commands (0.32-0.64 s
   at 1x). A 1000-command square with a 3 mm error on the last side records
   **680/1000 samples, 2/3 corners, score 100.0**, while the completed print scores
   **94.313**. The Finish-ASAP path already waits for `queueLen === 0`. Repro:
   `session-...-worker-auth-7838ab14/repro_quality_recorded_before_drain.mjs`
   (exit 1, re-run by manager).
8. **`fastModeActive` outlives the worker it describes.** After job 1, job 2's
   worker never receives `set_fast_mode(true)` at queue length 159 < 160, while a
   fresh world at the same length does. Consequence is throughput/prefetch, not a
   wrong measured value. Repro: `repro_worker_replacement_state.mjs` (exit 1).

Worker-boundary negatives: substep changes (1..16, mid-run 1<->8) preserve the
one-command-per-`update` contract; superseded workers' late commands are gated out;
a paused world consumes none and duplicates none; every UI->worker message type has
a handler in all three players; `set_dt`/`set_speed_scale`/`set_asap_mode` are
re-sent per job.

**Reproducer caveat:** `workerController.js` imports Vite-only modules, so the
worker-boundary repros mirror four boundary lines instead of importing the module;
each run fails as `sourceDrift` if those source lines change.

### Confirmed 2026-11-18 (bridge-protocol lens, manager re-ran the two headline repros)

9. **A stopped flight recorder silently disables the whole external bridge.**
   `handlePayload` calls `recordEvent()` before dispatch; once the recorder has an
   error the call throws, the socket logs a warning and drops the payload. After the
   receiver closes: 0/1 commands accepted, encoder requests never answered, while
   clock requests still are — the session looks half-alive. Clicking the default
   **Rerun** button with no receiver is enough. Contradicts `flightRecorder.js:167`
   ("never to G-code dispatch"). Repro:
   `session-...-two-c-312cceb7/repro_recorder_blocks_protocol.mjs` (exit 1, re-run
   by manager).
10. **Any payload without motion un-pauses the simulation.** `{type:'reply'}` (sent
    for every line, including `M105`/`M114`), unknown types, `{}` and arrays all call
    `runtime.resume()`. 5/5 reproduced; `position_trace_mode` is the control that
    does not resume. Repro: `repro_reply_resumes_pause.mjs` (exit 1, re-run).
11. **Two-client encoder requests stick to a silent client.** After an encoder
    timeout the preference is nulled but the next call re-picks the same newest
    silent client; 3/3 requests went to the silent tab and the healthy tab was never
    asked. Docs require one tab, so this is robustness, not normal use.
12. **Latent:** both 5000-entry backlogs evict oldest silently; a single 6000-command
    message exceeds the browser limit by 1000. Reset not clearing the browser backlog
    was refuted for the shipped app (near-dead path).

Bridge negatives: attach/detach/reconnect is clean (one socket, no duplicate history),
malformed JSON is inert, duplicate encoder replies are dropped, speed/command ordering
is harmless either way, `set_speed_scale` alone does not resume, machine swaps keep the
queue.

### Confirmed 2026-11-18 (integration-accuracy lens, manager re-ran both repros)

13. **Authored pulley inertia is ignored on frictionless passive wheels.**
    `engine.rs::friction` only uses a wheel's `physics:inertiaTensor` when the wheel
    has a worn `BEARING`; otherwise a passive wheel on a rolling link is massless.
    Atwood over one hoop-like pulley: measured **3.270 m/s^2** vs **2.4525**
    with inertia (**+33.3 %**); over four shipped HP4 D-line idlers: 3.270 vs
    2.3532 (**+39.0 %**). HP4's 24 wheels each have `I/r^2 = 0.0118 kg` (5.9x their
    own mass) against an 0.081 kg effector, so "negligible inertia" is not
    quantitatively true. The prior `a=g/3` negative used a fixed zero-inertia
    roller; it does not validate authored inertia. **Review correction: this is a
    latent fidelity gap (a deliberate massless-pulley idealisation), not a measured
    shipped-trajectory error.** Repro:
    `session-...-damped-h-c6fd65ea/repro_atwood_wheel_inertia.mjs` (exit 1, re-run
    by manager).
14. **No gyroscopic term for a freely rotating asymmetric body.** With no torque,
    omega is held constant in the world frame instead of integrating Euler's
    equations: after 0.5 s the orientation is **14.19 deg** off the exact torque-free
    solution and the angular-momentum direction has rotated **27.25 deg**. Latent:
    shipped bodies are symmetric tops/spherical. Repro:
    `repro_gyroscopic_free_body.mjs` (exit 1, re-run).
15. Latent: `physicsSubsteps` is silently ignored unless the world has a
    `RigidBodyComponent` (`simulationSystems.js:19-22`), so a point-mass cable scene
    is identical at 1 and 16 substeps.

Integration negatives (strong): first-order semi-implicit/PBD as documented; free
mass position error exactly `-1/2*g*t*dt` with fitted order 1.000000; velocity exact;
substeps reduce it exactly 1/n; damped cable oscillator at 2 ms has +0.745 % period
error for a soft visible mode but +16.0 % for a 56 Hz production-like mode (PBD
stiffness limit, not a defect); pendulum period +0.00075 %, energy loss first-order;
solver iterations 1/4/64 bit-identical for a single row.

Bookkeeping lens negatives: preset/RRF extrusion conservation, pause/resume and
sign handling, no reset leak in deposition, JS/Python agreement on `actual`,
`geometric`, `stretch` (<=2.2e-16 m), no energy/odometer accumulators exist.

Prior program's review warns: distinguish a shipped, user-visible wrong result from a
latent/API-order/invalid-input hazard, and state which one a finding is. Do not repeat
prior findings.

**Resource note:** one researcher's Python run auto-triggered a native addon rebuild
(14 s) and swap use rose to ~1.4 GiB. No further builds are allowed; prefer pure JS
and stub-based tests.

## Review outcome (2026-11-18)

Sealed bundle `reviews/review-7951a12fd3be` verified (candidate `212c56ea`, code
`74ecdc95`, zero non-research diffs). A fresh reviewer reproduced 7 of 11 headlines
exactly; native `.rrd` and two-client claims were confirmed by inspection plus
supplied evidence. Confirmed: upload extrusion accounting, quality-record timing,
recorder-kills-bridge, non-motion resume, length telemetry. Confirmed with corrected
wording: in-place reset generation, unrecognised-payload scope, conditional
`fastModeActive` leak. Behaviour confirmed but not classified as a shipped bug:
passive-wheel inertia (deliberate massless-pulley idealisation) and the missing
gyroscopic term (standard PBD simplification); two-client encoder stickiness is a
documented-unsupported setup. Summary has been corrected.

## Next decision

Program is complete for this question. Preserve reports, reproducers, sealed review and
the negative results; do not fix, push or merge. Highest-value follow-ups if continued:
(a) the reviewer's decisive test for passive-wheel inertia on a representative HP4
trajectory, (b) an end-to-end UI/worker test for the quality-record timing and bridge
recorder paths, and (c) a decision on whether the Python/native recording layer is
meant to mirror the JS recorder (findings 5-7). These are proposals only; no code
changes were made.
