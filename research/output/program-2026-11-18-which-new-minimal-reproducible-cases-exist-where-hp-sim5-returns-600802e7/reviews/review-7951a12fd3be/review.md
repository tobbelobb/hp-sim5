# Independent review — new reproducible hp-sim5 correctness failures (2026-11-18)

Reviewer: fresh subagent; did not produce the candidate.
Bundle: `reviews/review-7951a12fd3be` (verified below).
Code under test: **74ecdc95**; candidate commit **212c56ea**.
Date: 2026-11-18.

## 1. Bundle and revision verification

`./hp-sim5-research-program --resume ... --verify ...` returned:

```json
{"verified": true, "candidate_revision": "212c56ea7b894cf0e138590e1fa750a522aadbb6",
 "candidate_tree": "261d3ff9c2b9c25a5625751e9392c6e3de6ce4dd",
 "scientific_validity": "not_assessed"}
```

`git diff --name-only 74ecdc95 212c56ea` has **0 non-research files** (`non_research_count=0`). The
candidate commit adds only `research/` artifacts, so the code under test is exactly 74ecdc95, as the
handoff states. The working tree is at 212c56ea with submodule pointers and untracked files (research
artifacts, `selection.json`, and an older bug-report text) dirty. No source or selected evidence was
modified by this review.

The selected `files/` copies carry numeric prefixes. Their relative imports (`../../../../...`, `./harness.mjs`,
`probe_length_telemetry.mjs`, `common.mjs`) therefore break when run from `files/`; I ran the originals from
their session directories, which the manifest maps. This is a presentation issue, not a scientific one, but
the bundle as copied is not directly one-command reproducible.

## 2. Method and resource use

I ran **exactly eight** tiny reproducers, one process at a time, each under `timeout 90`, with no build, no
browser, no test suite, and no `run_experiment`. Logs are in `logs/`.

Before: `free -h` 1.3 GiB available. Lowest observed during the runs: 799 MiB available (swap ~1.3 GiB
used); after the runs 1.0 GiB available (swap 1.8 GiB used), no OOM. I stopped running when available memory
approached the 700 MiB floor.

Reproduced (8):

- `repro_gcode_extrusion_bookkeeping.mjs`
- `repro_shipped_gcode_extrusion.mjs`
- `repro_quality_recorded_before_drain.mjs`
- `repro_recorder_blocks_protocol.mjs`
- `repro_reply_resumes_pause.mjs`
- `repro_atwood_wheel_inertia.mjs`
- `repro_gyroscopic_free_body.mjs`
- `repro_length_telemetry.py` (Wasm probe + pure-Python mirror; no native rebuild)

Inspected only (4 headlines + all negative results): the native `.rrd` reproducers
(`repro_recording_scene_content.py`, `repro_reset_generation.py`, `repro_length_error_channel.py`), the
worker-replacement reproducer, the two-client reproducer, and the RRF-conservation negative control. This is
**inspection + supplied evidence**, not independent reproduction.

## 3. Verdict table

| # | Headline | Verdict | Basis | Decisive numbers |
|---|---|---|---|---|
| 1 | G-code upload mis-accounts extrusion | **Confirmed** | reproduced both | reference **49,950.460240 mm** vs MoveCommander **49,719.253160 mm**; **−231.207080 mm (−0.462873 %)**; synthetic 10→15, 15→10, 10→5, second job 5→0 |
| 2 | Quality History recorded before the world drains | **Confirmed** | reproduced | **680/1000** samples, **2/3** corners, score **100.0** vs completed **94.313247**, **320** commands executed after the record |
| 3 | Stopped recorder disables the external bridge | **Confirmed** | reproduced | after loss **0/1** commands, encoder request **unanswered**, clock request answered, **2** warnings; control 1/1 |
| 4 | Any no-motion payload resumes the simulation | **Confirmed with overbroad wording** | reproduced | 5/5 unrecognised non-motion payloads resumed; `position_trace_mode` stayed paused; `command` resumed |
| 5 | Python/native length telemetry aliases `commanded` to `actual` | **Confirmed** | reproduced | JS error **−0.015086662 m** vs Python **0.0**; B/C **+6.93085e−6 / +6.88416e−6 m**; torque C JS `null` vs Python **1.902308667** |
| 6 | Native `.rrd` collapses all members and omits groups | **Confirmed by inspection + evidence** | inspected, not re-run | 33/33 members; worst **0.424358821 m**; **0** `/members/`, **0** anchors, **0** motor/encoder/velocity/tool-point/extrusion groups; 236 paths |
| 7 | In-place reset is not a new replayable generation | **Confirmed for the in-place API; summary wording overbroad** | inspected, not re-run | generation stays **1**; no `scene_generation` timeline; `sim_step` **[0,50,0,50]**; two poses at step 50 differ **4.118e−3 m** |
| 8 | Authored passive-wheel inertia is ignored | **Behaviour confirmed; “shipped user-visible numerical error” NOT confirmed** | reproduced | 3.270000 vs 2.452500 (**+33.33 %**, synthetic hoop); one shipped HP4 wheel 3.270000 vs 2.979765 (**+9.74 %**); four 3.270000 vs 2.353183 (**+38.96 %**) |
| 9 | No gyroscopic term for an asymmetric free body | **Behaviour confirmed; a standard PBD simplification, not a contract bug** | reproduced | orientation error **0.247622 rad (14.19°)**; angular-momentum direction change **0.475688 rad (27.25°)** |
| 10 | `fastModeActive` survives worker replacement | **State leak confirmed; “every job” and impact NOT confirmed** | inspected code/report, not re-run | job 2 worker receives only `set_asap_mode` at queue 159 < 160; leak requires job 1 to end at queue 160–224 |
| 11 | Two-client encoder requests stick to the newest client | **Confirmed behaviour; documented unsupported setup** | inspected code/report, not re-run | 3/3 requests to the silent newest client; healthy client never asked; docs say use one tab |

## 4. Per-headline assessment

### 1. G-code upload extrusion — confirmed, shipped

**Reproduced exactly.** From `logs/repro_gcode_bookkeeping.txt` and `logs/repro_shipped_gcode.txt`:

```text
ok   control: absolute E with M82: reference 10 mm, MoveCommander 10 mm
FAIL no M82: reference 10 mm, MoveCommander 15 mm (150.000 %)
FAIL mid-file G92 E0: reference 15 mm, MoveCommander 10 mm (66.667 %)
FAIL M82 then M83: reference 10 mm, MoveCommander 5 mm (50.000 %)
second job: 0 mm (expect 5)
shipped: reference 49950.460240 mm; MoveCommander 49719.253160 mm
         difference 231.207080 mm (0.462873 %)
```

Source confirms each mechanism: `hp-sim-3d/app/workers/moveCommander.js:41` starts
`absolute_extrusion = false`; `:127-139` parses only X/Y/Z in `G92`; `:208-218` recognises only `M82`;
`configure()` at `:55-78` does not reset `last_e`. The app's own reference parser defaults absolute and
handles `M83`/`G92 E` (`hp-sim-3d/app/inspectionToolsController.js:74,88-121`), and RRF/Klipper default to
absolute (`RRF/ReprapFirmware/src/GCodes/GCodeMachineState.cpp:21`;
`klipper/klippy/extras/gcode_move.py:43`).

**Strongest alternative explanation.** The comparison is of *positive-E* totals, and the 121 lost `G1 E2`
primes are unretract refills, not material laid on the part. A reader could argue the “deposited filament”
loss is smaller than 231 mm, or that the reference parser itself over-counts retract/unretract cycles. This
does not rescue MoveCommander: the upload path also fails to execute `M83`, `G92 E`, and the firmware's
absolute default, and it leaves `last_e` across jobs. The app's reference path and the RRF-compiled CAN
capture agree on 49,950.46 mm; the upload path does not implement the same file semantics.

**Severity/reachability.** Genuine for interactive `.gcode` upload, including the shipped logo. The
no-`M82` 337× case is hypothetical (no shipped file); the cross-job leak is reachable by a second upload
without Reset. The derived 99.537 % quality progress is arithmetic, not a measured browser print. The
highest-value user-facing defect in the set.

### 2. Quality History before drain — confirmed, shipped

**Reproduced exactly.** `logs/repro_quality.txt`:

```json
"withTailError": {"atDone": {"samples": 680, "cornerCount": 2, "score": 100},
                  "completed": {"samples": 1000, "cornerCount": 3, "score": 94.31324717531413},
                  "commandsExecutedAfterDone": 320}
```

Source confirms the boundary: `workerController.js:73-79` clears `remoteSystem.worker` and calls
`onWorkerDone` immediately on `done`; `commandJobController.js:136-143` calls
`recordFinalQualityAndResetJob()` on the linear path, while the ASAP path waits for
`!workerActive && queueLen === 0` (`:429-462`). The reproducer uses the real `World`, `RemoteSpoolSystem`,
`QualityController`, and `QualityMonitor`, with a modelled worker that fills the 320-command prefetch.

**Strongest alternative explanation.** “Record when the producer stops” could be the intended semantics.
The ASAP contrast and the later HUD/History disagreement (94.3 vs 100 on the same job) are strong evidence
against that reading; the code comment/invariant and the persisted history make the prefix record a defect.

**Severity/reachability.** Genuine and user-visible for linear prints. The exact 680/1000 is a synthetic
320-deep queue; a real print's tail is 160–320 commands, so the score error varies, but the truncation is
structural. The report's extra inference that a stream shorter than the prefetch depth records *nothing*
(`_composeScore` NaN) is code-read, not measured.

### 3. Recorder failure blocks the bridge — confirmed, shipped

**Reproduced exactly.** `logs/repro_recorder.txt`:

```json
"control": {"commandsAccepted": 1, "encoderRequestAnswered": true},
"afterRecorderLoss": {"commandsAccepted": 0, "encoderRequestAnswered": false,
                      "clockRequestAnswered": true, "consoleWarnings": 2}
```

Source confirms: `externalCommandSocket.js:242-243` calls `recordEvent` before dispatch;
`flightRecorder.js:190-195` throws when `this.error` is set and `nonFatal` is false; the app constructs the
recorder without `nonFatal` (`appBootstrap.js:119-124`); the socket message listener catches and logs,
dropping the payload. A closed Rerun socket sets `error` (`flightRecorder.js:100-108`). The `Rerun` button is
in the default panel (`hp-sim-3d/index.html:187`) and the README tells users to click it before printing.

**Strongest alternative explanation.** Fail-closed on a recording error could be intentional. But the code's
own invariant says backpressure applies to sampled physics, never G-code dispatch, and the bridge has no
feedback channel; the clock poll continues to succeed, producing a half-alive session. The defect is real.

**Severity/reachability.** Genuine, user-visible: click Rerun with no receiver, or lose the receiver mid-run,
then all commands/encoder requests are silently dropped. High value.

### 4. No-motion payload resumes — confirmed, but the summary wording is too broad

**Reproduced exactly.** `logs/repro_reply_pause.txt` shows 5/5: `reply-to-M105`, `reply-empty`,
`unknown-type`, `empty-object`, `json-array` all resumed; `position_trace_mode` stayed paused; `command`
resumed. Source: `externalCommandSocket.js:374-375` calls `runtime?.resume?.()` at the bottom of
`handlePayload`, after `pushCommands(batch)`. Recognised control types return earlier.

**Strongest alternative explanation.** Resuming on a real motion command is intended
(`pushExternalCommands` already does it); resuming on an `M105` reply, an empty object, or an unknown type is
not. The defect is the fallthrough, not the command path.

**Severity/reachability.** Genuine but low-medium: it cancels a user pause and starts physics on a status
poll. The summary's “**Any** payload without motion resumes” is false for `position_trace_mode`,
`set_speed_scale`, and `reset`, which are recognised and stay paused. The report's table is correct; the
summary is not.

### 5. Length telemetry aliases commanded to actual — confirmed

**Reproduced exactly.** `logs/repro_length_telemetry.txt`:

```text
position-move/CablePathA: JS commanded=1.917265792 error=-0.015086662
                          Python commanded=1.902179130 error=+0.000000000
position-move/CablePathB: JS error=+0.000006931, Python 0.0
position-move/CablePathC: JS error=+0.000006884, Python 0.0
torque-mode/CablePathC: JS commanded=None, Python commanded=1.902308667
```

Source: `src/python/hp_sim5_research/mirror.py:239-240` writes `'commanded': actual` and
`'error': actual - actual`; `experiments.py:88` computes `peak_abs_length_error_m` from that error, so it is
structurally zero. The JS recorder computes the target with `cableStoredLengthAfterRotation` and nulls both
fields in torque mode (`flightRecorderSnapshot.js:119-144`), matching `FLIGHT_RECORDER.md:203-214`.

**Strongest alternative explanation.** The Python mirror might be a deliberately reduced observation model
that only promises `actual`/`geometric`/`stretch`. The field is named `commanded`, the metric is named
`peak_abs_length_error_m`, and the docs define both; the mirror already has the motor target data it would
need. The supplied production run also reports `peak_abs_tracking_error_rad = 0.5033` while
`peak_abs_length_error_m = 0.0`, which is internally implausible.

**Severity/reachability.** Genuine for every `run_experiment`, `read_run`, research-MCP numeric read, and
native `.rrd`. It does not change the physics, but it makes one research metric unusable and hides real
millimetre-scale tracking errors. High value for research trust.

### 6. Native recording content — confirmed by inspection and evidence, not re-run

The supplied evidence (`files/15-evidence_scene_content.json`) reports 33/33 members, worst
`PinholeCR_bottom` at **0.424358821 m** from its true pose, zero `/members/` paths, zero anchor paths, zero
motor/encoder/velocity/tool-point/extrusion groups, 236 entity paths. I confirmed the mechanism in source:
`mirror.py:204-205` uses the **parent body's** position/orientation for every member and writes every frame
under `.../bodies/...`; `recording.py:48-82` logs only frames, effector points, cable strips/forces, line
lengths/errors, and cable forces. `FLIGHT_RECORDER.md:162-214` promises the missing groups and the
parent/member hierarchy.

**Strongest alternative explanation.** A body-level recording could be an intentional reduction, with
members reconstructed from static local transforms. But the recorder writes no local transforms and gives
all 33 members the same pose; the pre-rewrite implementation (cited in the report) logged local positions and
the missing groups. This is a regression/contract violation, not a design choice.

**Severity/reachability.** Every native `.rrd` and `final.json`; replay is visibly wrong for any multi-member
body. Research/telemetry trust, not solver arithmetic.

**Minor evidence inconsistency:** `10-report.md` quotes `recording.rrd` 672,555 B / sha `a0b361...`, while
`15-evidence_scene_content.json` records 672,425 B / `ea126...`; `10-report.md` quotes 712,708 B / `01461...`
for the reset recording, while `16-evidence_reset_generation.json` records 710,374 B / `26e61...`. This looks
like a re-run between the report text and the final evidence files; the measured poses/counts agree, so it
does not change the finding, but the report's hashes should be reconciled.

### 7. In-place reset generation — confirmed for the API, summary overbroad

Evidence (`files/16-evidence_reset_generation.json`): after reset `epoch` 0→1 but `scene_generation` stays
**1**; no `scene_generation` timeline; `sim_step` **`[0,50,0,50]`**; two poses at step 50 differ
**0.00411846 m**; `events.jsonl` generation set is `{1}`. Source: `session.py:237-250` sets
`sceneGeneration = epoch + 1`, then `_start_physics()` → `mirror.apply_state()` overwrites it with the
worker's constant 1; `recording.py:54` only sets the `sim_step` timeline.

**Strongest alternative explanation.** The production `reset_session` path creates a new world and a new
recording file, so not every user Reset hits this. The report says so; the summary's “**Any** native session
reset” is too broad. For the supported in-place `NativeSession.handle({'type':'reset'})` API, the invariant
is violated.

**Severity/reachability.** Genuine but medium/low; a replay scrub at `sim_step=50` sees only the later
generation. Not a shipped default user action.

### 8. Passive-wheel inertia — behaviour confirmed, classification not

**Reproduced exactly.** `logs/repro_atwood.txt`:

```text
hoop-like pulley: expected with inertia 2.452500, massless 3.270000, measured 3.270000 (+33.33%)
one shipped HP4 wheel: expected 2.979765, massless 3.270000, measured 3.270000 (+9.74%)
four HP4 wheels: expected 2.353183, massless 3.270000, measured 3.270000 (+38.96%)
```

Source: `src/wasm/cable_solver/src/engine.rs:2274` `turn_wheel` sets a free wheel's orientation
kinematically from material travel, with no inertial reaction; `:2376` `passive_wheel` treats non-motor
rolling wheels as ideal. The Rust test `free_wheel_atwood_turns_with_line`
(`src/wasm/cable_solver/tests/cable_physics.rs:152-181`) asserts the massless-pulley result
`a = g(m1−m2)/(m1+m2)` and checks that the wheel turns with the line, so this is an **explicitly intended
idealization**, not an accidental omission. It is, however, not stated in the user-facing README, while
`hp-sim-3d/README.md:287-300` advertises a full 3×3 inertia tensor.

**Strongest alternative explanation.** The cable model is a massless-tendon model (`README.md:40`); passive
idlers are often idealised. The authored `physics:inertiaTensor` on those wheels is dead data for free
rolling, but the machine-level consequence is conditional on those idlers actually spinning with the line
and on their inertia being large relative to the effective moving mass. The report's own limitations say
“No shipped trajectory error is claimed; this is a latent fidelity gap”. The summary's classification as a
“**shipped, user-visible numerical error**” is therefore inconsistent with the report. The synthetic
hoop-like +33.3 % is also not the shipped one-wheel number (+9.74 %).

**Severity/reachability.** A latent model-fidelity gap / undocumented simplification. It is a valid
“authored data is ignored” finding, but not a demonstrated shipped wrong result. If the project wants to
count it as a bug, it needs the machine-level measurement described in §7.

### 9. Missing gyroscopic term — behaviour confirmed, a PBD simplification

**Reproduced exactly.** `logs/repro_gyro.txt`: orientation error **0.247622 rad (14.19°)** and angular
momentum direction change **0.475688 rad (27.25°)** after 0.5 s, with `I = diag(0.002, 0.502, 0.502)`,
`ω0 = (1,2,0.5)`. Source: `engine.rs:707-724` integrates the orientation with a constant world-frame
`ω` (`axis_angle(ω/|ω|, |ω| dt)`) and never applies `ω × (Iω)`.

**Strongest alternative explanation.** This is a standard PBD simplification; the PBD reference in `ai_docs`
treats the gyroscopic term as an extension. All shipped bodies are symmetric tops (`Ixx=Iyy`,
`Izz≈2Ixx`) or spherical, and for `ω` along the symmetry axis the omission is exactly zero. The report
correctly labels it latent. It is not a violation of a documented contract, because the user-facing docs do
not promise Euler-equation rigid-body dynamics.

**Severity/reachability.** Latent physics-fidelity gap. No shipped trajectory error; an authored asymmetric
body or a large off-axis `ω` would expose it. This is a model limitation, not a bug in the sense used for
#1–#5.

### 10. `fastModeActive` leak — state leak confirmed, “every job” not

Source confirms the leak: `remoteSpoolSystem.js:43` initialises `fastModeActive=false`; the worker setter
(`:53-68`) re-sends only `set_asap_mode`; `clearPlaybackState()` (`:97-102`) clears the queue/history but not
the mirror; `resetRemoteQueue` sets `worker` but not `fastModeActive`. The enable/disable conditions are
`:389-398`. The reproducer (inspected) models job 1 ending at 200 queued and job 2 at 159 < 160.

**Strongest alternative explanation.** A fresh worker in non-fast mode still receives every command; the
consequence is pacing/prefetch, not a wrong trajectory. The report did not measure a wall-clock slowdown.
The summary's “**Every** job after the first” is false: the leak requires job 1 to end with the queue below
the turn-off point (`max(low, 0.7·high) = 224`), i.e. roughly the 160–224 band or a stream shorter than 160;
if job 1 ends above 224, `fastModeActive` is false and job 2 gets the message. The report itself says ~40 %
of the drain phase.

**Severity/reachability.** Genuine state-machine defect; performance/second-order, not a numeric error. Mark
as confirmed behaviour, unmeasured impact.

### 11. Two-client sticky encoder — confirmed, documented unsupported

`integrations/rrf/rrfSimulatorBridge.mjs:27-45` picks the highest connection sequence; the timeout branch
(`:115-128`) deletes the request and clears `preferredEncoderClient`, but the next call re-derives the same
newest client. The fallback to an older client only runs on `close` (`:137-145`). The supplied report
measures 3/3 requests to the silent client and none to the healthy one. The docs say “Use one simulator tab”
(`autocal/data/references/extended-reference.md:127`).

**Strongest alternative explanation.** Two tabs are documented as unsupported, so this is a robustness
defect with a documented workaround, not a default-path bug. It is real for a frozen/throttled newest tab or
a headless client attached to the same bridge.

**Severity/reachability.** Low-medium; every encoder request pays the full timeout, and the error message
misdiagnoses the cause. Not a default single-tab failure.

## 5. Internal inconsistencies in the summary (consequential)

1. **#8 severity.** The summary lists #8 under “Shipped, user-visible numerical error: 1, 2, 8”, but
   `31-report.md` says “No shipped trajectory error is claimed; this is a latent fidelity gap” and “the
   machine-level consequence is inferred, not measured”. This is the most consequential inconsistency.
2. **#8 headline numbers.** The summary's “3.270 vs 2.4525 (+33.3 %, one pulley)” is the synthetic hoop-like
   pulley. The shipped single HP4 idler is **+9.74 %**; the +39.0 % is four idlers. A reader can easily
   mistake the +33.3 % for the shipped case.
3. **#4 “Any payload without motion”.** The report's own table shows `position_trace_mode`,
   `set_speed_scale`, and `reset` stay paused; only unrecognised payloads (including `reply`) resume. The
   summary's “any” is too broad.
4. **#7 “Any native session reset”.** The report says the production `reset_session` creates a new world and
   recording; only the in-place `NativeSession.handle({'type':'reset'})` path collides. The summary
   overstates reach.
5. **#10 “Every job after the first”.** The leak is conditional on where job 1 ends relative to the 224
   turn-off point; the report says ~40 % of the drain phase.
6. **#2 exact numbers.** 680/1000, 2/3 corners, and 94.313 are a synthetic 1000-command square with a
   synthetic 3 mm tail; the defect is structural, but the exact score is illustrative. The report says this;
   the summary presents the numbers without that qualifier, and “Every linear print” ignores the inferred
   short-stream “no entry” case.
7. **Minor evidence drift.** The report/evidence `.rrd` byte counts and hashes for findings 6 and 7 disagree
   (§4.6). Reconcile before archival.

## 6. Documented model simplification vs bug

- **Passive-wheel inertia (#8):** a **deliberate model simplification**. `turn_wheel` is kinematic;
  the Rust Atwood test explicitly expects the massless-pulley `g/3`. It is not documented in the user-facing
  README, and the authored `physics:inertiaTensor` is dead data for passive wheels, so a user can reasonably
  be surprised. But the correct label is “undocumented fidelity gap / ignored authored data”, not a
  demonstrated shipped wrong result. The report says this; the summary does not.
- **Gyroscopic term (#9):** a **standard PBD simplification**. The engine integrates constant world-frame
  `ω`; the PBD reference treats `ω × (Iω)` as an extra term. All shipped bodies are symmetric tops, so the
  shipped default is unaffected. Not a contract violation; a physics-fidelity limitation.
- **No physics changes were made** in this candidate, so the project's physics gate was not required. If
  either simplification is “fixed”, that is a physics change: it must run the project's physics checks on
  the final candidate. The repository exposes `npm run test:physics` (builds native/Wasm and runs the HP4
  stability/pretension benchmarks). This review did not run it (no build allowed, no physics change).

## 7. The additional test most likely to change my judgment

**Primary:** measure whether the shipped HP4 idlers actually rotate with the line in a representative
print, and quantify the trajectory change when their authored inertia is included (or when the passive-wheel
inertia is injected into the solver). If those idlers spin and the effector trajectory changes materially,
#8 becomes a shipped numerical error and the summary's severity is right; if they are effectively fixed
guides or the change is below the print tolerance, the report's “latent fidelity gap” is right. This is the
only headline whose *classification* I dispute; the behaviour is already reproduced.

**Secondary (if a browser is available):** a real upload of `public/gcode/Hangprinter_logo6.gcode` with the
production worker and Quality History enabled, reading the persisted record after the queue drains. That
would confirm #1 and #2 end-to-end and test whether the synthetic 320-command tail is representative.

## 8. Negative results and scientific value

The report preserves useful negatives: replay/serialization round-trip is faithful (1.9e−9 m); recording
does not perturb physics; the RRF/CAN preset path conserves extrusion to −0.0015 mm (−2.9e−8) on the same
shipped file; pause/resume deposits once; integration is first-order semi-implicit/PBD as documented, with
the free-mass error `−½ g t dt` and fitted order 1.000000; there is no record-resume feature. I inspected the
RRF-conservation script and the integration report's method; I did not rerun them (eight-run cap). The RRF
control is logically sound and corroborates the 49,950.46 mm reference, which is important for #1. These
negatives make the positive findings more credible and should be kept.

Scientific value: the engineering findings (#1–#5) are actionable reliability/trust defects; #6–#7 are
observation-layer defects; #8–#9 are model-fidelity documentation, not new physics. Code readiness is low:
no fixes, no CI tests, and several reproducers are research scripts with stubs/mirrors. The highest-value
fixes are #1 (upload E semantics), #2 (drain before recording), and #5 (compute the real commanded length).

## 9. Explicit assessment

- **Bundle validity:** verified; candidate adds only research files; code under test is 74ecdc95.
- **Reproduction:** 7/11 headlines reproduced exactly (eight reproducers; headline 1 used two); 4/11
  (#6, #7, #10, #11) confirmed by source inspection and supplied evidence only. No source or selected
  evidence was modified.
- **Core verdict:** the core defects (#1–#5) are real. #1–#3 are shipped and user-facing; #4 is low-medium;
  #5 is a research/telemetry trust defect. #6–#7 are real but reach-limited. #8–#9 are genuine model
  simplifications; #8 in particular is misclassified in the summary as a shipped user-visible error, and the
  summary's reach/wording for #4, #7, and #10 is overbroad. #10–#11 are second-order.
- **Acceptance:** I would accept the bundle as a valid bug report after correcting the #8 severity and the
  #4/#7/#10 wording. I would not accept the summary as written as a claim that #8 is a shipped wrong result.
- **Physics gate:** no physics changes; any fix to #8/#9 must run `npm run test:physics` on the final
  candidate.

## Evidence index

- My run logs: `logs/repro_gcode_bookkeeping.txt`, `logs/repro_shipped_gcode.txt`, `logs/repro_quality.txt`,
  `logs/repro_recorder.txt`, `logs/repro_reply_pause.txt`, `logs/repro_length_telemetry.txt`,
  `logs/repro_atwood.txt`, `logs/repro_gyro.txt`.
- Source (base 74ecdc95): `hp-sim-3d/app/workers/moveCommander.js:41,127-139,208-218`;
  `hp-sim-3d/app/inspectionToolsController.js:74,88-121`; `hp-sim-3d/app/commandJobController.js:136-143,429-462`;
  `hp-sim-3d/app/workerController.js:73-79`; `hp-sim-3d/app/externalCommandSocket.js:242-243,374-375`;
  `hp-sim-3d/app/flightRecorder.js:100-108,190-195`; `hp-sim-3d/app/remoteSpoolSystem.js:43,53-68,97-102,389-398`;
  `src/python/hp_sim5_research/mirror.py:204-205,239-240`;
  `src/python/hp_sim5_research/experiments.py:88`; `src/python/hp_sim5_research/recording.py:54`;
  `src/python/hp_sim5_research/session.py:237-250`;
  `src/wasm/cable_solver/src/engine.rs:707-724,2274,2376`;
  `src/wasm/cable_solver/tests/cable_physics.rs:152-181`;
  `integrations/rrf/rrfSimulatorBridge.mjs:27-45,115-145`;
  `hp-sim-3d/FLIGHT_RECORDER.md:162-214`; `README.md:40`; `hp-sim-3d/README.md:287-300`;
  `autocal/data/references/extended-reference.md:120,127`.
- Supplied evidence: `files/00-summary.md`, `files/02-report.md`, `files/10-report.md`,
  `files/15-evidence_scene_content.json`, `files/16-evidence_reset_generation.json`,
  `files/17-evidence_length_error_channel.json`, `files/18-report.md`, `files/22-report.md`,
  `files/31-report.md`, `files/07-measurements.json`.
