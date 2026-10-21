# HP4 Squares: substep bias and review of PR 77

Status: open-loop torque-balance correction validated on 2026-10-21; rotor damping correction deferred.

## Abstract

PR [#77](https://github.com/tobbelobb/hp-sim5/pull/77) contains a useful diagnosis:
coarse physics steps give open-loop motors artificial holding strength. A passive
spring-loaded spool on current main settles at 0.002386 rad instead of the
analytic 0.005946 rad at 2 ms. Removing cable-only electromagnetic stiffness
recovers the analytic equilibrium across five timesteps and four JS/Rust
execution paths, with independent Python/Warp checks. However, the PR's separate
rotor damping correction causes persistent 5.008 mm vertical motion in the
required loaded-state test at one substep, both alone and combined with the
inertia correction. Only the inertia correction is retained. All 11 mandatory
physics checks pass. Squares still pulls B out at finer steps; the change removes
a numerical bias rather than making that motion loss-free. All evidence is
simulated; hardware behavior and full-print accuracy are unmeasured.

## Review and integration decision

The PR was reviewed at `fcdaf7fed43a4cb1a9efdd289a58980ea2840984` against
main `2c0b86ff`. Its physics commit is `63b847c8`.

1. **Keep the open-loop physical inertia correction.** Electromagnetic torque
   already accelerates the rotor in the motor system. The cable solver alone
   adds `K*h²` to inertia, where `K=holdingTorque*polePairs`. At rest, balancing
   the explicit motor prediction and cable correction therefore requires
   `motorTorque = cableTorque*I/(I+K*h²)`. With `I=5e-5`, `K=25`, this ratio is
   1/3 at 2 ms and 8/9 at four substeps. The simulated equilibrium violates the
   physical law. Removing this term in JS, Rust native/Wasm and Python also
   corrects the Warp preparation path; closed-loop constraint stiffness remains.
2. **Defer the standalone rotor damping correction.** The PR's pure-damper
   analytic check confirms double counting. Nevertheless, transplanting the
   correction makes the unmodified one-substep loaded HP4 gate fail. The failure
   also occurs with only that correction, isolating it from open-loop inertia.
   Its 5.008 mm last-five-second vertical range exceeds the required 10 nm bound.
   A correct local damper equation is insufficient evidence for stable integrated
   cable/rotor dynamics. Do not merge this part without resolving that failure.
3. **The QC refresh is useful independent work.** Source inspection confirms
   that current HUD refresh depends on extrusion, and hides diagnostics without
   a reference path. The PR's frame callback/throttle addresses this. It is not
   included in this focused physics commit and was not browser-validated here.
4. **Expand validation beyond the PR's higher-substep logo runs.** Those runs
   cannot substitute for the default-one-substep loaded-state gate. The PR's
   squares test checks B, so it does not establish zero losses on other axes.

## Matched Squares replays

The production `RrfCanPlayer` decodes the bundled layered HP4 Squares CAN file
with its matching firmware axis map. Each command executes at the normal 2 ms
cadence in a fresh native headless production world. Scene, layering, motor
parameters, commands and solver iterations are unchanged across cases.

Peak diagnostic counts after two simulated seconds:

| Physics substeps | Baseline total | Both PR physics fixes | Retained inertia correction | Retained B / C |
| --- | ---: | ---: | ---: | --- |
| 1 | 0 | 0 | 0 | 0 / 0 |
| 2 | 0 | 0 | 2 | 2 / 0 |
| 4 | 2 | 2 | 2 | 2 / 0 |
| 8 | 3 | 3 | 3 | 2 / 1 |
| 16 | 3 | 3 | 3 | 2 / 1 |

B's final encoder-minus-command lag with the retained change is 0.25532,
0.25454, 0.25407 and 0.25388 rad at 2, 4, 8 and 16 substeps. Two magnetic
periods are 0.25133 rad. At one substep the final lag is 0.01100 rad. A separate
20 s replay retains zero losses at one substep and two B periods at four;
peak body speeds are 0.2143 and 0.3865 m/s respectively. These are bounded
prefixes, not complete Squares jobs or hardware pull-out thresholds.

The diagnostic is a peak magnetic-period offset, summed over motors, rather
than a cumulative count of physical full steps or slip/recovery events. The
unloaded default rotor's small-signal natural period is about 8.89 ms; 2 ms
integration provides only about 4.44 samples per period. That and the proven
artificial holding term make coarse-step reassurance unreliable. The tested
1–16 substeps do not establish continuum or hardware accuracy.

## Validation and limits

- `npm run test:physics`: **11 passed**, including native/Wasm 20 s default-logo
  motion, JS fallback and loaded settling at one, two and four substeps.
- Independent equilibrium: **20 passed** across JS, Wasm cable kernel, full
  Wasm and native; Python/Warp suite **117 passed, 5 GPU skips**.
- Native/Wasm, mounted-rotor and existing substep suites: **141 passed, 1 GC skip**.
- Full Jest: **801 passed, 1 skipped**. Existing Python differential tests pass
  in the first combined run; its failures came from the initial exploratory
  multi-projection equilibrium assertions and a read-only Warp cache. The latter
  was resolved with `WARP_CACHE_PATH=/tmp/hp-sim5-pr77-warp-cache` and rerun.

The analytic regression uses one projection per step to isolate torque balance.
An exploratory native audit at two/four projections finds a small remaining
finite-projection error: up to 7.34e-8 rad equilibrium-angle error or 3.76e-8 Nm
reported torque residual. Doubling duration from one to two seconds does not
remove it. Those strict exploratory failures are retained; exact iteration
invariance is not claimed. The delivered fix does not repair the remaining
rotor damping, cable transport or full-print tracking limitations.

## Evidence and reproduction

- [Session report](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/report.md)
  and [notebook](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/research.md):
  H1–H6 decisions, budgets, inputs, measured outcomes and failed trials.
- E01/E03 [baseline analytic failures](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E01-baseline-analytic.log)
  and [combined candidate passes](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E03-candidate-analytic.log):
  independent torque/damping evidence.
- E02/E03/E06 [baseline](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E02-baseline.json),
  [combined corrections](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E03-candidate.json)
  and [retained correction](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E06-inertia-only.json):
  matched commands, hashes, numerical samples and motor diagnostics.
- E04/E05 [combined failure](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E04-candidate-physics-gate.log)
  and [damping-only failure](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E05-damping-only-physics-gate.log):
  reason for deferring the damping correction.
- E06 [passing mandatory gate](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E06-inertia-only-physics-gate.log),
  E07 [projection audit](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E07-equilibrium-audit.json)
  and E08 [20 s Squares prefixes](../output/research/sessions/2026-10-21-interactive-research-cdfb5278/E08-inertia-only-20s.json):
  accepted stability, remaining numerical error and motion scope.
- [Equilibrium regression](../tests/benchmark/open_loop_torque_balance.test.mjs)
  and [Python/Warp check](../tests/python/cable_joints_3d/test_open_loop_torque_balance.py):
  executable physical acceptance laws, wired into CI.
- [Previous HP4 stability investigation](hp4-logo-stability.md) and
  [motor integration investigation](faster-motor-update.md): preceding evidence
  motivating the unmodified default-step gate and continued timestep caution.

Reproduce the principal checks with:

```bash
npm run test:physics
node --test tests/benchmark/open_loop_torque_balance.test.mjs
WARP_CACHE_PATH=/tmp/hp-sim5-pr77-warp-cache .venv/bin/python -m pytest \
  tests/python/cable_joints_3d/test_open_loop_torque_balance.py \
  tests/python/cable_joints_3d/test_warp_solver.py -q
```

The archived session's `squares-replay.mjs` reproduces numerical motion without
sockets or a viewer; for example, `node <session>/squares-replay.mjs new-label 2`.
The session harness writes a new `<label>.json` containing inputs/hashes and samples.
