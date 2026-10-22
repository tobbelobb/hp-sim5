# HP4 Print Squares substep bias and PR 77 review

Status: complete; inertia correction validated, damping correction deferred.

## Abstract

PR #77 proposes two corrections for HP4’s greater step loss at finer physics substeps. We reproduced both local analytic defects on main, replayed the bundled Squares commands in fresh native production worlds, and separated the corrections with the mandatory default-step stability gate. Removing cable-only electromagnetic stiffness changes a 2 ms spring-loaded spool equilibrium from 0.002386 to the analytic 0.005946 rad; 20 checks across JS, Wasm kernel/full engine and native pass, with independent Python/Warp validation. The separate rotor damping correction fails loaded-state settling, alone and combined, producing 5.008 mm persistent vertical motion; it is deferred. The retained inertia correction passes all 11 mandatory physics checks. Squares still loses two B magnetic periods at finer settings, and one C period at 8–16 substeps. This corrects artificial coarse-step holding power, not full-print tracking. All evidence is simulated; hardware, continuum accuracy and calibration accuracy remain unmeasured.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| H1: Cable-only motor stiffness biases static torque balance | E01, E03, E06, E07 | Supported | [Baseline failures](E01-baseline-analytic.log), [final equilibrium checks](E07-static-equilibrium-final.log) |
| H2: Standalone rotor damping double-counts displacement, and its local correction integrates safely | E01, E03–E05 | Mixed: local law supported, integrated safety rejected | [Analytic passes](E03-candidate-analytic.log), [damping-only failure](E05-damping-only-physics-gate.log) |
| H3: Finer-step Squares pull-out persists after correction | E02, E03, E06, E08 | Supported for bounded prefixes | [Baseline](E02-baseline.json), [retained correction](E06-inertia-only.json), [20 s prefixes](E08-inertia-only-20s.json) |
| H4: Both PR corrections can land together on current main | E04, E05 | Rejected | [Combined failure](E04-candidate-physics-gate.log), [damping-only failure](E05-damping-only-physics-gate.log) |
| H5: Inertia-only correction repairs local balance and preserves required stability | E06, E07 | Supported for tested scope | [Passing gate](E06-inertia-only-physics-gate.log), [analytic checks](E07-static-equilibrium-final.log), [Python/Warp](E07-python-warp-final.log) |
| H6: Exact equilibrium assertions remain valid at four nonlinear projections | E07 audit | Rejected at strict roundoff tolerances; small remaining projection error | [Audit](E07-equilibrium-audit.json), [initial failures](E07-static-equilibrium-initial-failed.log) |
| QC refresh fixes startup HUD in an interactive browser | None | Not tested; source review only | [PR input](pr77-review-input.json) |

## Method and result

See the [tracked review](../../../../research/hp4-squares-substeps.md) for the derivation, full numerical table and integration decision. The experiments use fresh native Rust production worlds, unchanged HP4 layered scene, open-loop motors, two solver projections, normal 2 ms firmware command cadence and the bundled production Squares CAN file decoded by RrfCanPlayer. No browser, hardware or autocalibration trial was run. The continuing supervisor world was not advanced. No long live collection was needed; the separate command socket probe passed tool checks only.

E01 fails 30/30 original PR analytic cases on baseline. E03 passes all 30 after both corrections. E04 and E05 independently fail the same default-one-substep loaded-state check: vertical range 0.00500792772 m, against the unchanged 1e-8 m limit. Other substeps and passing logo cases cannot replace this failure. Both failed source patches and logs remain in this session.

E06 keeps only physical open-loop cable inertia. It passes all 11 required checks. E07 final adds 20 analytic checks on four execution paths over 1–16 substeps, plus 6 independent Python/Warp equilibrium cases. The complete Python/Warp run passes 117 checks with 5 GPU skips; native/Wasm, mounted rotors and existing logo-substep checks pass 141 with one GC skip; full Jest passes 801 with one skip. The original combined Python run's failures are retained: one overly broad exact projection-invariance assertion and 38 attempts to write a read-only Warp cache. A writable /tmp cache and the corrected isolation test resolve the final run. Existing differential cases passed in that original run.

E07's separate multi-projection audit establishes a limitation rather than silently widening tolerances: equilibrium angle differs by up to 7.34e-8 rad and actual torque residual by up to 3.76e-8 Nm at two/four projections. Doubling duration does not remove it. The delivered tight analytic test therefore uses one projection, the same isolation as the PR. It detects the original approximately 0.00356 rad equilibrium error without claiming exact multi-projection behavior.

E02/E03/E06 each include five two-second Squares cases. Baseline and combined fixes report total peak periods 0,0,2,3,3 at 1,2,4,8,16 substeps; retained inertia-only correction reports 0,2,2,3,3. B contributes two periods; C contributes one at 8/16. E08 extends retained native 1/4-substep cases to 20 seconds: zero and two B periods persist, with bounded motion. These are 17 short motion replay cases, within the 20-case budget. No complete Squares job or continuum reference was attempted.

![B startup phase error](squares-startup.png)

## Code and decision

The useful commit takes only the open-loop physical-inertia part of PR 77, mirrors it in JS/Python/Rust (Warp shares Python preparation), reserves the retired Rust ABI option slot, rebuilds Wasm, updates old stiffness expectations and wires independent static balance checks into CI. It preserves closed-loop holding stiffness, physics defaults, motor parameters, solver iterations and required stability tolerances.

The damping fix remains a valid local defect report but needs a stable coupled integration before delivery. QC refresh is useful separate work; no HUD code was changed here. The next mechanics decision should examine why removing duplicate damping excites the loaded one-step world, using the preserved E04/E05 failure before trying another local deletion. Increasing substeps or weakening settling tests would not resolve that acceptance requirement.

## Validation artifacts

- [E01 baseline failures](E01-baseline-analytic.log), [E03 combined analytic passes](E03-candidate-analytic.log).
- [E03 full candidate patch](E03-full-physics-candidate.patch), [E05 damping-only patch](E05-damping-only-candidate.patch).
- [E04 failed gate](E04-candidate-physics-gate.log), [E05 failed gate](E05-damping-only-physics-gate.log), [E06 accepted gate](E06-inertia-only-physics-gate.log).
- [E07 final analytic tests](E07-static-equilibrium-final.log), [initial strict failures](E07-static-equilibrium-initial-failed.log), [projection audit](E07-equilibrium-audit.json).
- [E07 native/Wasm tests](E07-backends.log), [full Jest](E07-jest-full.log), [Python/Warp final](E07-python-warp-final.log), [initial Python/cache failures](E07-python-warp.log).
- [E02 baseline](E02-baseline.json), [E03 full candidate](E03-candidate.json), [E06 retained change](E06-inertia-only.json), [E08 longer prefixes](E08-inertia-only-20s.json).

## References

- [Full report](report.md), [research notebook](research.md): accepted scope, decisions and limitations.
- [PR #77](https://github.com/tobbelobb/hp-sim5/pull/77), reviewed head [fcdaf7fe](https://github.com/tobbelobb/hp-sim5/commit/fcdaf7fed43a4cb1a9efdd289a58980ea2840984), physics commit [63b847c8](https://github.com/tobbelobb/hp-sim5/commit/63b847c842f79519104f7626563f1a2e71e09658): source of proposed corrections and analytic laws; [archived connector input](pr77-review-input.json).
- [Provenance](provenance.json), [replay harness](squares-replay.mjs): baseline revision 2c0b86ff, hashes of production scene, firmware axis map, source CAN, decoded commands and native artifacts.
- [Tracked integration review](../../../../research/hp4-squares-substeps.md): derivation, measured tables, test commands and code references.
- [Previous HP4 stability session](../2026-10-20-interactive-research-62300644/report.md), [stability correction](../../../../research/hp4-logo-stability.md), [motor formulation](../../../../research/faster-motor-update.md): prior dynamics limitations and rationale for the unchanged gate.
- [Recording cost guidance](../../../../research/autocal-recording-performance.md): reason to choose numerical-only evidence; no performance comparison is claimed here.

## Commit artifact

The original repository’s `.git` is mounted read-only by this workspace. Creating `fix/hp4-open-loop-torque-balance` there failed before any ref/index mutation. The tested tree was committed through a separate Git directory inside this session as `838133d4ed03662a55581322367a9c309ac47e38`. The [complete commit patch](hp4-open-loop-torque-balance.patch) and [incremental Git bundle](hp4-open-loop-torque-balance.bundle) are ready to import; the bundle depends on original main `2c0b86ff189d27859b83543ed3fa837cfa658a3b`. Original working-tree source edits remain present and uncommitted. No remote branch was created or message posted.
