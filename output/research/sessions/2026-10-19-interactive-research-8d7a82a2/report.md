# HP4 autocal failures: numerical slowing, missing travel budget and a repeatable four-substep workflow

Status: complete investigation; the underlying dynamics correction remains open.

## Abstract

We investigated recurring HP4 autocal failures using October 16–19 logs and Rerun recordings, native Rust force comparisons, an exact replay and bounded headless calibration trials. Both movers weigh 81 g. At equal configured effective force, one-substep HP3 reaches Y=423 mm at 30 seconds versus HP4 80 mm; four substeps raises HP4 to 335 mm. Motor stabilization introduces timestep-dependent slowing amplified by reeving. The reproduced failure at 515.42 seconds instead expires in a separate 30-second preparation wait while free motors still pull. We extended that transit budget and corrected headless speedup to preserve physical wait durations. A faster motor formula was rejected because calibration fits deteriorated. With original physics and four substeps, 60 bootstrap measurements exactly repeat prior evidence. A fresh-world continuation from 100 measured points collects 40 more and stops automatically, with applied-anchor RMSE 4.02 mm versus nominal geometry. This establishes a controlled continuation workflow, not universal reliability or a completed fresh single-invocation run. All evidence is simulated; hardware accuracy and a complete dynamics repair remain untested.

## Topics

HP4, HP3, Hangprinter, autocalibration, torque mode, numerical damping, reeving, implicit integration, physics substeps, headless speedup, settling, dataset continuation, Rerun

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| HP4 moves substantially less at equal configured effective force | E02, E06 | Supported for controlled pulls | [Motion comparison](motion-comparison.png), [matched forces](E06-matched-forces/) |
| The remaining default timeout is missed fixed targets or rapid wobble | E04, E07 | Rejected for this failure: held motors stationary, free motors pulling | [Exact replay](E07-coast/replay.json), [failure screenshot](failure-viewer.png) |
| Longer size/pullout budgets cover the failing preparation wait | E04, E07 | Rejected: a separate ordinary 30 s wait remains | [Failure log](E04-sub1-clock/sweeps.full_auto.log), [coast check](coast-settling.json) |
| Headless speedup preserved physical collection durations before the fix | E01, E04, E17 | Rejected before the fix; corrected durations supported at 1/10/25 | [Original source](autocal_headless-before.mjs), [clock invariance](E17-clock-invariance.json) |
| Correcting the local steady motor slowing term is sufficient for production | E08–E12, E15 | Rejected: better travel but poor data and degenerate fits | [Rejected patch](rejected-motor-correction.patch), [residual audit](nominal-model-residuals.json), [seeded diagnostic](E15-seeded-fit-diagnostic.json) |
| Extending preparation completes stock one-substep calibration | E16 | Inconclusive: 50 bootstrap points before wall budget; former failure crossed | [Trial log](E16-preparation-budget/sweeps.full_auto.log), [manifest](E16-preparation-budget/sweeps.headless/manifest.json) |
| Stock four-substep data and fits repeat with the prior manual forces | E14 and prior f3c115bf control | Supported for bootstrap and fit; fresh full automatic completion budget-censored | [Exact 60-point repeat](E14-bootstrap-repeat.json), [control log](E14-stock-sub4/sweeps.full_auto.log) |
| A fresh matching world can continue measured four-substep data to automatic acceptance | E19 | Supported for this controlled trial: 100 seed + 40 fresh points | [Acceptance audit](E19-acceptance-audit.json), [fresh-sweep comparison](E19-fresh-continuation-parity.json) |

## References

- [Full report](report.md): equations, measured results, operating recipe and limitations.
- [Research notebook](research.md): steering, compute budgets and chronological decisions.
- [Prior damping investigation, f3c115bf](../2026-10-19-interactive-research-f3c115bf/report.md): the preceding four-substep control and unresolved post-move drift.
- [Prior endpoint investigation, bf5d807a](../2026-10-19-interactive-research-bf5d807a/report.md): rejected local controller and interpretation of logs 008/009.
- [Prior motor load study](../2026-10-16-hp4-roller-propagation/report.md): older stiffness counterfactual and alternative mechanisms.
- [Original reference-005 log](../../../extended-autocal/no-buildup-reference-005/sweeps.full_auto_1.log) and [RRD audit](E03-rrd-summary.json): 120 measurements, four completed collections and final browser-clock stall.
- [Scene complexity](scene-complexity.json), [world configuration audit](world-configuration-audit.json): actual mass, solver work and HP4/Q0/R30/native settings.
- [E19 dataset](E19-clean-continuation/sweeps.json), [seed contract](E19-clean-continuation/seed-contract.json), [run manifest](E19-clean-continuation/sweeps.headless/manifest.json), [finalized event RRD](E19-clean-continuation/sweeps.headless/hangprinter-20261019T215304.204951Z-8626c63b.rrd): provenance and automatic completion evidence.
- [Code provenance](provenance.json), [retained patch](retained-changes.patch), [source snapshot](retained-source/): starting revision 3b3a7fec and the six retained changes.

## Why HP4 behaves differently

The mover masses are equal, but their motor/cable integration and computational workloads differ.

| Quantity | HP3 | HP4 |
| --- | ---: | ---: |
| Horizontal mechanical advantage U | 1 | 2 |
| Cable joints | 18 | 51 |
| Constraint visits per physics substep | 18 | 102 |
| Passive rotating pulleys | 0 | 24, including 18 on the mover |
| Mover mass, kg | 0.0810000021 | 0.0810000019 |

The extra constraints and rotors cost computation. Four substeps repeats integration four times. Requested playback speed does not establish actual throughput. No matched wall-time performance ratio is claimed: trials overlapped and some used different native build profiles. The following comparisons concern movement per simulated second.

### Measured physical response

These native Rust worlds are unlayered, use closed-loop position motors, hold D and apply horizontal torques at t=2 s. HP3 torques are doubled relative to HP4 to compensate U=1 versus U=2, giving equal configured effective forces: A≈0.02227 N and B/C≈2.51553 N.

| Effector Y at t=30 s, mm | One substep | Four substeps |
| --- | ---: | ---: |
| HP3 | 422.900 | 629.403 |
| HP4 | 79.591 | 334.974 |

These are finite-transit comparisons, not constant-speed ratios; geometry and eventual equilibria differ. [Raw torque trials](E02-pulls/), [force-matched HP3 trials](E06-matched-forces/), [plot](motion-comparison.png). The copied E06/E08 harness stdout labels its final sample “y120”, but those runs end at t=30 s; the table uses explicit row timestamps.

### Timestep-dependent motor slowing

The current torque update is

    omega_next = [omega + dt/I * (drive + load + D*omega)]
                 / [1 + dt*D/I + dt²*K/I].

Its internal steady update requires drive+load = dt*K*omega. Thus the stabilizer adds a timestep-dependent opposing term even at constant internal rotation. Measured K=36 Nm/rad and dt=2 ms give dt*K=0.072 Nms/rad. Reflecting that local motor port through r=30 mm and U gives (dt*K)*(U/r)² = 80 Ns/m for HP3 and 320 Ns/m for HP4. Reeving magnifies this numerical slowing. This is a local calculation, not a complete transfer function of the projected cable network. Internal omega is not always observed encoder velocity; real modeled motor friction, cogging and pulley inertia also remain.

Four substeps reduces this timestep term and reduces numerical transients. That provides a concrete explanation for the earlier improvement. The large simulated difference cannot be explained by mover mass, which is the same.

### A faster formula was tested and rejected

E08 corrected the predicted-pose numerator while retaining the implicit denominator. HP4 one-substep Y rose to 637.856 mm at t=30 s; four-substep candidate Y was 614.892 mm. Local checks looked promising. However, E09 automatic force selection chose 0.8731 N, collected 160 points in eight sweeps and produced underconstrained fits at every iteration. E11 reproduced that behavior. Median residual against the nominal geometric model was 72.44 mm. Stronger manual forces in E12 reduced it to 3.91 mm, but fitting still degenerated. Sampled tilt stayed around 1.6 degrees or less, so a gross effector flip was not the explanation.

A truth-seeded E15 diagnostic also did not recover nominal geometry, but used different least-squares settings; it does not exclude every optimizer explanation. The candidate was rejected as a production correction. All candidate JS/Rust/Python/test/Wasm changes were reverted. [Rejected patch](rejected-motor-correction.patch), [candidate source](rejected-source/), [native artifact](E08-candidate-native/physics.node), [model residuals](nominal-model-residuals.json), [seeded diagnostic](E15-seeded-fit-diagnostic.json). Faster motion and passing parity tests did not establish calibration fidelity.

## Why increasing timeouts kept failing

Force tuning, size transit, fixed-target preparation, endpoint pullout, endpoint preload, point measurement, fixed-anchor validation and transport watchdogs have separate limits. Previous changes extended size transit and pullout to 600 physical seconds. E04 still fails in an ordinary 30-second wait at performForceSweep entry, after fixed-target preparation and before pullout.

At 515.42 physical seconds, no commands are queued. E07 replays all 840 nonempty encoder reads with zero error. Held A/D motors are stationary; B/C remain in torque mode, while the body translates at 5.155 mm/s with angular speed 0.000117 rad/s. This failure is slow transit, separate from the high-frequency wobble fixed in earlier work. [Replay](E07-coast/replay.json), [failure screenshot](failure-viewer.png), [coast plot](post-failure-coast.png).

Continuing the stock one-substep numeric state first passes the unchanged encoder quiet check after another 180 seconds. It can creep later, so a short quiet interval does not prove global equilibrium. Switching that state to four substeps passes after five seconds. These are numerical physics restarts, not firmware checkpoints. Actual encoder differences establish motion; nonzero internal velocity fields alone do not.

The retained change gives only active fixed-target preparation the existing 600-second force-travel budget, with a 630-second independent wall floor and phase logging. Ordinary point waits, encoder quiet criteria, noise samples, fixed-anchor limits and fit criteria remain unchanged.

E16, using original one-substep physics, crossed the former failure and collected 50 bootstrap points before its 1,200-second wall limit (3,064.324 physical seconds). It did not complete bootstrap or fit, so complete one-substep calibration remains inconclusive. Its first-direction nominal-model median residual was 16.08 mm; the span/history differ from four-substep trials, so this is a warning about data fidelity, not an isolated measurement of timestep error or a negative full-calibration result. [Partial-direction audit](partial-direction-model-audit.json), [E16 manifest](E16-preparation-budget/sweeps.headless/manifest.json).

Older one-substep research also found force tuning without a knee at 20 N, a fallback force, tiny sweep coverage and underconstrained fits. Extra time can permit travel but cannot create missing geometric information or make a biased physical model accurate. That older default automatic-force behavior was not retested to full completion here.

## Headless clock correction

Previously headless advance(seconds) executed seconds*speed/dt steps while incrementing collector time by dt/speed. A 30-second collector wait at speedup 10 therefore covered 300 physical seconds. Force trials, ramps, polling windows and trajectories changed with speedup. This is a separate headless bug; it does not directly explain the user's original browser invocation.

Headless now advances seconds/dt and increments collector time by dt. It already steps unthrottled; --speedup retains the requested playback setting. E17 repeats native commanded motion at speedups 1/10/25: all produce 1,000 steps, two physical seconds and identical encoder replies. Corrected speedup 10 in E04 reproduces the previous speedup 1 failure at exactly 515.42 seconds. These establish clock/workload consistency, not accuracy. [Clock audit](E17-clock-invariance.json).

## What the old logs and RRDs establish

- Reference 004: size-tuning limit of 120 s while still travelling. Later held samples alone do not describe the failed interval.
- Reference 005: 120 measurement events and four collection-complete events in the RRD. Revision 966ead90 had a four-substep source default, but the recording does not directly identify actual substep count. The browser clock later stops advancing. This was substantial collection, not complete automatic acceptance.
- Reference 006_1/_2: pullout reaches 120/600 s, then a different ordinary 30-second wait fails.
- Reference 007: transport “Message not received”; extending physical settling cannot repair that transport failure.
- References 008/009/010_1: wall deadlines in an experimental local endpoint controller absent from current HEAD. These are separate from current force pullout.
- Reference 010: explicitly two substeps and still an ordinary 30-second settling failure. Its nearby newer recording directory contains a manifest/log but no available RRD.
- The exact reference-011 example was unavailable; no user reference dataset was modified.

The October 16 load bookkeeping fix (6dfd772c) and October 19 damping feedback fix (bf9f5399) are already in starting HEAD. They are preceding improvements, not newly proposed fixes. [RRD audit](E03-rrd-summary.json), [catalog search](prior-catalog.md), [damping report](../2026-10-19-interactive-research-f3c115bf/report.md), [bookkeeping report](../2026-10-16-hp4-load-bookkeeping/report.md), [endpoint investigation](../2026-10-19-interactive-research-bf5d807a/report.md).

## Repeatable four-substep workflow

### E14: exact bootstrap repetition, full run budget-censored

Original motor physics, four substeps, native headless, speedup 10, fixed forces 0.0223/0.4455/6.5739 N and automatic optimizer stopping bounded by max 20 iterations. Size travel was 677.199 mm raw / 541.759 mm scaled. All 60 bootstrap points match the prior f3c115bf control exactly: drive/sensor lengths, raw encoders, means, sigmas, setpoints, sample counts and durations have zero difference. Nominal-model median residual is 0.2415 mm. The first fit also repeats score 0.5295; a later best score is 0.4261. [Repeat audit](E14-bootstrap-repeat.json).

The outer 1,500-second wall limit interrupted E14 after six complete sweeps/120 points, during the next collection. Its terminal CLI message accepts best-so-far because SIGTERM is translated into the existing Ctrl-C path. The manifest correctly says interrupted, normal_completion=false. That terminal quality message is not automatic acceptance. [Manifest](E14-stock-sub4/sweeps.headless/manifest.json), [log](E14-stock-sub4/sweeps.full_auto.log).

Its bootstrap collector had already loaded the original 30-second preparation wait when that source was changed; later collectors used 600 seconds. Four-substep preparation did not require over 30 seconds, so this difference was nonbinding. Its immutable native binary is byte-identical to the restored final default native artifact.

### E19: normal automatic acceptance in a fresh matching world

We copied E14's first five complete sweeps/100 actual measured points before continuing; the input and hash are preserved. No nominal or truth-generated measurements were seeded. The same CLI, four substeps, original physics and manual forces ran in a fresh matching world. Existing dataset history was replayed through the normal CLI, then two new sweeps/40 points were physically collected. This is dataset continuation, not physics-checkpoint restoration.

E19 exited 0 after 375.804 wall / 490.732 simulated seconds, with status complete, stop_reason=patience-or-threshold and normal_completion=true. The recorder finalized and read back without rejected messages. It contains 40 new measurement and two collection-complete events. The final dataset has seven sweeps/140 points; fresh samples use 16 reads, and maximum fixed-anchor mean error is 0.0544 degrees. [Acceptance audit](E19-acceptance-audit.json), [seed input](E19-clean-continuation/seed-input.json), [seed contract](E19-clean-continuation/seed-contract.json), [dataset](E19-clean-continuation/sweeps.json).

The applied anchor coordinates have RMSE 4.022 mm against the initial nominal preset; individual errors are 3.164, 2.550, 2.619 and 6.430 mm. All-data fitted residual RMS is 0.0466 mm, but this is in-sample model agreement, not positional accuracy or held-out validation. The fresh sixth sweep is close to the continuous-world sixth sweep, not bit-identical: maximum drive-length difference 0.0812 mm and raw-encoder difference 0.31 degrees. [Fresh-world comparison](E19-fresh-continuation-parity.json).

This supports reproducible controlled data/fit and a functioning continuation workflow. It does not establish universal reliability, layered-spool performance, hardware accuracy or a completed fresh single-invocation automatic trial.

### Practical command for the tested configuration

Use a new dataset path for a fresh run; keep --collector-args last. This is the tested stock HP4 zero-buildup, 30 mm radius, closed-loop setup, with manual forces that avoid the sensitive force-selection stage:

```bash
.venv/bin/python autocal/autocal.py \
  --headless-sim --machine-type hangprinter_4 \
  --physics-substeps 4 --speedup 10 \
  --recording-mode events --max-steps 20 \
  --dataset output/extended-autocal/hp4-repeat/sweeps.json \
  --collector-args --no-auto-tune-force \
  --force-low 0.0223 --force-mid 0.4455 --force-max 6.5739
```

Headless uses the native Rust engine in the audited manifests. Events mode records collection evidence without physics snapshots; use sampled when motion inspection is needed. Do not combine --recording-mode with --record. Four substeps increases computation, and the from-scratch control reached its 25-minute experiment bound; --speedup 10 is not a promise of ten times real-time throughput. Reusing an existing dataset invokes continuation; E19 validates this only for matching configuration and measured data.

## Validation, provenance and remaining work

The retained patch changes six files, 42 insertions and 13 deletions: headless clock, active-transit wait, tests and explanation. No motor physics change is retained. Final current-source validation: 36 headless checks, 95 collector primitive/behavior checks and 113 native/Wasm/loaded-stability checks pass, for 244 passes and one GC-related skip; two slow headless checks were excluded. The 20 targeted preparation checks overlap the 95-check set. [Headless log](final-headless-tests.log), [collector log](final-collector-tests.log), [backend log](final-backend-tests.log). Candidate-only passing tests do not override its rejected calibration evidence.

Starting revision was 3b3a7feca285a8b0d9e7d20a2652a66b0542e605 with a clean worktree. [Retained patch](retained-changes.patch), [source snapshot](retained-source/), [provenance and hashes](provenance.json), [final native build](final-native-build.json). E19's source hash matches the retained production source. The restored native release artifact is 99c4f129fbf7949b795d3f2f4074dea02ec3e879c782d95da402584748566c60. Different build profiles prevent performance comparisons with the starting artifact.

E05 was censored at its 900-second wall budget during stock-four collection. E01/E11/E12 were deliberately interrupted; their manifests remain stale “running” and their RRDs lack final footers. They are preserved as partial records, not finalized or accepted runs. All nine owned full-auto physics services are retired. The attached supervisor world remains generation 1, step zero. [Final services audit](final-services-audit.json), [attached-world state](final-attached-world.json).

All long trials used native Rust headless physics and speedup 10; routine evidence used events, with sampled evidence for motion questions. HP4/Q0/R30/closed-loop settings were audited. Tool preflight and the separate command socket probe passed; neither establishes accuracy. No hardware experiments were performed.

**Proposed, not tested:** construct a reduced reeved motor/path/body model with analytic torque, length and energy invariants, audit effective path stiffness, mounted-rotor coupling and winding history, then validate transients and calibration convergence across timestep. The current evidence justifies four substeps as a workaround and a preparation-budget correction. A physically complete numerical repair remains open.

## Source references

- [Motor integration](../../../../hp-sim-3d/app/torqueModeSystem.js) and [native engine](../../../../src/wasm/cable_solver/src/engine.rs), revision 3b3a7fec: the slowing equation and integration order.
- [Firmware force conversion](../../../../RRF/ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp): force-to-torque conversion through mechanical advantage and radius.
- [HP3 scene](../../../../public/usd_scenes/hp3_rigid_body.usda) and [HP4 scene](../../../../public/usd_scenes/hp4_rigid_body.usda): masses, geometry, pulley inertias and solver passes.
- [Headless owner](../../../../scripts/autocal_headless.mjs) and [collector wait](../../../../autocal/control/behaviors/sweep_data_collection.mjs): retained corrections.
- [Recording-cost study](../../../../research/autocal-recording-performance.md): informed evidence budgets, not a performance ratio for these full-auto runs.
- [HP3 settling review](../../../../research/autocal-settling-review.md): earlier accepted HP3 result and distinction between fit quality and nominal-anchor error.
