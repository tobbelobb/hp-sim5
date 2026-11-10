# HP4 motor load bookkeeping: one contribution per physical constraint

Recording motor cable load torque, stiffness and damping once per constraint, rather than per solver iteration, improves a matched one-step 30 s pull from56.8to84.8mm without extra integration work. The correction is mirrored in JavaScript, Rust native/Wasm, Python and Warp. A fresh headless one-substep autocal run completed three bootstrap sweeps (60 points) plus a fourth sweep, then stopped at its configured one-iteration limit. It did not time out. Force tuning fell back to1N and the fit remained underconstrained, so this is collection validation, not successful calibration. Default substeps, forces and settling criteria are unchanged.

## Change

Load accumulation now runs only on the first constraint visit, matching the existing physical-force snapshot convention. Repeated numerical refinement no longer adds parallel physical springs or duplicate torque contributions. The fixed-geometry regression checks analytic torque/stiffness/damping at1,2and5iterations. The native and browser artifacts were rebuilt (native release without PGO for this test).

The trial also exposed a headless settings acknowledgement bug: the service awaited but discarded the encoder acknowledgement, and the bridge returned nothing. Passing that existing response through fixes explicit --physics-substeps startup validation. Unit and real service/reset integration tests cover it.

## Tested hypotheses and evidence

| Hypothesis | Experiment | Outcome |
| --- | --- | --- |
| Repeated solver visits multiply a fixed physical load | Analytic fixed-body regression | Corrected; torque,K,Dindependent of iteration count |
| Correction improves response without more integration work | E01 matched 30 s pull | Supported:84.795mm versus preceding baseline56.835mm; K36instead of72Nm/rad |
| Backends agree after correction | E02 differential/parity checks | Supported for tested cases |
| One-step collection can pass the previous failure stage | E04 fresh bounded headless autocal | Supported:60bootstrap points plus fourth sweep; configured max-steps reached |
| Entire calibration is accurate/well constrained | E04 fit | Not established; underconstrained sentinel after1Nfallback |

Validation:343JavaScript tests;99Wasm parity tests;6native tests with1GCskip; opt-in loaded-state stability regression;21targeted Python checks;8Warp checks; additional fixed-load invariant3checks and Wasm kernel check; headless acknowledgement unit check and2real settings/reset checks. No tolerance was loosened. git diff --check passed. Initial restricted compiler/cache/socket failures were rerun with the needed execution permissions.

## Autocal trial and stopping point

[Run log](autocal-retry/sweeps.full_auto.log), [manifest](autocal-retry/sweeps.headless/manifest.json), [dataset](autocal-retry/sweeps.json). Command uses --headless-sim --speedup1 --physics-substeps1 --max-steps1 and events recording. Speedup1 avoids the known headless/browser wait-scaling difference. It completed the fourth sweep and reached max steps before the user's requested shutdown was issued. The attempted shutdown found no matching active autocal. No further experiments were started.

Force tuning found no comfortable knee before its20Ncap, fell back to1N, and size tuning produced only15.394mm raw travel. The fit was underconstrained. Thus the bookkeeping bug is corrected and collection completed, but force selection/coverage still needs work. The known loaded-state rocking at one step is not claimed fixed; its explicit substep stability regression remains passing. No energy-convergence or complete calibration validation is claimed.

The first trial stopped before motion on the acknowledgement bug; its artifacts remain in autocal/. All user reference-006/007 files and browser settings were untouched. User requested immediate wrap-up and commit; no more trials planned.

## References

- [Preceding diagnosis](../2026-10-16-hp4-roller-propagation/report.md): matched baseline, numerical damping, historical attempts.
- [30 s corrected pull](bookkeeping.json) and [harness](pull.mjs).
- Tests/fixtures/python_3d_parity/torque_motor_load_iterations.json: fixed geometry and analytic load invariant.
