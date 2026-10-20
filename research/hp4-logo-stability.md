# HP4 Print Logo stability correction

Status: production stability correction complete; coupled no-slip repair remains open.

## Abstract

Removing independent rolling-slip redistribution suppressed HP4 startup wheel spin but caused explosive Print Logo motion. Shipping it despite known loaded-state failures was incorrect. The normal layered, open-loop, one-substep replay produced 79 missed steps, 11.69 m/s peak speed and 50.53 rad/s rotation in 20 seconds. Withdrawing that shortcut while retaining tension readback and signed motor-feedback corrections recovers the baseline prefix trajectory and loaded-state settling. A mandatory headless native/Wasm/JavaScript gate rejects the broken engine after 0.586 simulated seconds and passes the correction. It runs automatically through npm test and CI and is required by researcher instructions. A full 1,038,258-step native job remains bounded and settles afterward, but terminal commands lose 300 motor steps; replaying the same terminal checkpoint through the original engine produces exactly the same motion and counts. This is an inherited tracking limitation, not a complete-print accuracy pass. All evidence is simulated or synthetic. Artificial bearing slip/spin, a coupled no-slip solver, gate dynamics and calibration accuracy remain open.

## What went wrong

Commit `9ebc8337` skipped redistribution for inertial rolling links in JS, Python
and Rust. An idle-wheel check passed, but loaded-state settling already failed.
Accepting that failure and omitting the production logo workflow were errors.
The existing logo substep suite also omitted the browser's default **one**
substep. Its other cases were insufficient substitutes for this setting.

S01 replays the bundled layered HP4 CAN logo with the production scene builder,
2 ms firmware cadence, open-loop motors and one substep. S02 uses the archived
pre-change native engine; S03 restores only the rolling redistribution operation,
keeping commits `41010aa1` and `628a8963`.

| First 20 simulated seconds | Broken S01 | Baseline S02 / corrected S03 |
| --- | ---: | ---: |
| Missed steps | 79 | 0 |
| Peak COM speed, m/s | 11.6888 | 0.31565 |
| Peak angular speed, rad/s | 50.5257 | 0.02366 |
| COM Z range, m | 0.01119–0.59584 | 0.089911–0.090475 |

S02 and S03 have identical sampled COM positions, speed and angular speed, and
identical extrema and motor diagnostics. The corrected loaded checkpoint settles
at one, two and four substeps with its original tolerances. No default substep,
material parameter or settling tolerance was changed.

This isolates the regression to the slip-removal shortcut. It does not establish
the precise energy defect in the remaining coupled transport/rotor solver. The
earlier observations of artificial slip and bearing spin still apply. Their
isolated removal is withdrawn, along with tests asserting that rejected model.
It must return with a stable coupled formulation and movement evidence.

## Automatic protection

Run from the repository root:

```bash
npm run test:physics
```

The command rebuilds both engines and runs six checks in about 30 seconds on
the research host. The logo gate runs 10,000 steps in native and browser Wasm,
and 1,000 steps in the JavaScript fallback, all headlessly. At every step it
requires finite pose/velocity, a bounded workspace, COM height within 5 mm of
the print plane, speed below 0.5 m/s and rotation below 0.1 rad/s. It requires
all queued commands to execute, no physics error/pause and zero missed steps.
The existing loaded-state test adds 20 seconds each at one, two and four
substeps. These are regression limits for this production workload.

S06 is a negative control: the new test actually fails against the archived
broken engine at step 293, when angular speed reaches 0.101506 rad/s. It passes
the corrected code. The gate runs automatically under `npm test` and CI. Root
and research `AGENTS.md` require a green gate before delivering dynamics changes;
idle checks, backend parity and higher substeps cannot replace it.

Research instructions now prefer headless production backends. Browser
experiments should be rare and preferably performed by a human. One isolated
browser reproduction preceded that steering; all remaining validation is
headless. Running browser Wasm inside Node requires no browser experiment.

## Validation and limits

The bounded gate passes all six cases. Additional checks pass: 157 native/Wasm
and host tests, 196 Python differential/core checks, 110 Warp CPU checks
(five CUDA skips), 800 Jest tests (one skip), four existing logo substep tests,
and three Rust native tests. A stale Jest expectation of zero displayed tension
on an immobile stretched span was corrected to its analytic 100 N; the requested
force-display change is retained. A fresh 30-second torque-mode pull also
completes with bounded motion, exercising retained motor feedback.

The complete native diagnostic runs all 1,038,258 commands (34.6 simulated
minutes): peak speed 0.3583 m/s, peak rotation 0.1844 rad/s, COM Z range
0.089604–0.104983 m. It remains bounded and settles after the job, but the
terminal commands lose 300 motor steps. The initial fixed-plane full acceptance
trial therefore remains failed, not relabeled as a perfect print.

From the same checkpoint 500 commands before the end, the original c6f0b5a1
engine and correction produce exactly equal body motion and missed-step counts
for the remaining commands plus ten seconds of holding (5500 samples). The
corrected checkpoint continuation also exactly matches the full run. Both
settle to zero angular speed; last-five-second vertical range is 2.45e-13 m.
This reproduces the ending limitation in the original engine; it is not an
independent full baseline replay. See the [comparison](../output/research/sessions/2026-10-20-interactive-research-62300644/integration/tail-comparison.json).

The actual `npm test -- --runInBand` invocation passes the six-case gate and
all 800 active Jest tests. No hardware or autocal accuracy claim is made.

The artificial rolling slip and underlying timestep bias remain unresolved.
The experimental faster motor update was never merged.

## Evidence

- [S01–S06 session notebook](../output/research/sessions/2026-10-20-interactive-research-62300644/research.md): decisions, budgets, user steering and failed trials.
- [Detailed session report](../output/research/sessions/2026-10-20-interactive-research-62300644/report.md): preceding mechanics experiments and historical failed acceptance.
- [Numerical trials and validation logs](../output/research/sessions/2026-10-20-interactive-research-62300644/integration/): source/artifact provenance, logo trajectories, negative control and passing checks.
- [Production logo gate](../tests/benchmark/hp4_logo_stability.test.mjs), [command scheduling](../tests/benchmark/hp4_logo_commands.mjs), [loaded-state test](../tests/benchmark/hp4_stability.test.mjs): executable acceptance criteria.
- [Faster motor formulation report](faster-motor-update.md): separate experimental theory, not part of this correction.
