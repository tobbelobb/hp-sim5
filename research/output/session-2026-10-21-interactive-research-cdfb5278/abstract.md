# HP4 Print Squares substep bias and PR 77 review

Status: complete; inertia correction validated, damping correction deferred.

## Abstract

PR #77 proposes two corrections for HP4’s greater step loss at finer physics substeps. We reproduced both local analytic defects on main, replayed the bundled Squares commands in fresh native production worlds, and separated the corrections with the mandatory default-step stability gate. Removing cable-only electromagnetic stiffness changes a 2 ms spring-loaded spool equilibrium from 0.002386 to the analytic 0.005946 rad; 20 checks across JS, Wasm kernel/full engine and native pass, with independent Python/Warp validation. The separate rotor damping correction fails loaded-state settling, alone and combined, producing 5.008 mm persistent vertical motion; it is deferred. The retained inertia correction passes all 11 mandatory physics checks. Squares still loses two B magnetic periods at finer settings, and one C period at 8–16 substeps. This corrects artificial coarse-step holding power, not full-print tracking. All evidence is simulated; hardware, continuum accuracy and calibration accuracy remain unmeasured.

## Topics

HP4, Print Squares, missed steps, physics substeps, open-loop torque balance, rotor inertia, cable damping, loaded settling, PR 77

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

## References

- [Full report](report.md), [research notebook](research.md): accepted scope, decisions and limitations.
- [PR #77](https://github.com/tobbelobb/hp-sim5/pull/77), reviewed head [fcdaf7fe](https://github.com/tobbelobb/hp-sim5/commit/fcdaf7fed43a4cb1a9efdd289a58980ea2840984), physics commit [63b847c8](https://github.com/tobbelobb/hp-sim5/commit/63b847c842f79519104f7626563f1a2e71e09658): source of proposed corrections and analytic laws; [archived connector input](pr77-review-input.json).
- [Provenance](provenance.json), [replay harness](squares-replay.mjs): baseline revision 2c0b86ff, hashes of production scene, firmware axis map, source CAN, decoded commands and native artifacts.
- [Tracked integration review](../../../../research/hp4-squares-substeps.md): derivation, measured tables, test commands and code references.
- [Previous HP4 stability session](../2026-10-20-interactive-research-62300644/report.md), [stability correction](../../../../research/hp4-logo-stability.md), [motor formulation](../../../../research/faster-motor-update.md): prior dynamics limitations and rationale for the unchanged gate.
- [Recording cost guidance](../../../../research/autocal-recording-performance.md): reason to choose numerical-only evidence; no performance comparison is claimed here.

- [Commit patch](hp4-open-loop-torque-balance.patch) and [bundle](hp4-open-loop-torque-balance.bundle): packaged commit `838133d4`; original `.git` is read-only and source edits remain in the working tree.
