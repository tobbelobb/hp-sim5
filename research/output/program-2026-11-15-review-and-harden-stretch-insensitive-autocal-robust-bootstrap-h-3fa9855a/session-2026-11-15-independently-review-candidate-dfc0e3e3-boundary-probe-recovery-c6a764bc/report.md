# Boundary recovery and feasible force-profile review

Reviewed committed candidate **`dfc0e3e3f47be4e7ae915211a42645d40e5e8746`**, with primary delta from `8338be8f585f4552bca4af74155bd87132daa9a9`. The new frozen-fit rejection loop and feasible v2 force tables pass the checked offline comparisons. **Review outcome: changes requested for two P2 recovery/application gaps in surrounding production flow.** Ordinary return errors can still be swallowed, and the return primitive does not verify that an individual commanded return tracked its target. The latter weakens the new rejection handler's claim that recovery completed.

No physics, simulator, service, hardware, scene state, or running process was started, stopped, or accessed. Production/test source and Git state were unchanged. This report is a pinned assessment, not acceptance of any later parent fix. [Source manifest](evidence/source-manifest.json) records the revision and eleven source hashes; [candidate diff](evidence/candidate.diff) preserves the reviewed delta.

## P2: ordinary return errors do not stop the collection/application path

At candidate `autocal/control/behaviors/sweep_data_collection.mjs:1939` and `:2013`, the between-sweep and final return blocks catch movement errors and emit warnings. The final dataset is written at `:2010` before that final return. Consequently the collector can resolve successfully after a movement failure, whereas the parent at `autocal/autocal.py:1421` depends on a nonzero collector exit to stop without applying calibration.

The [command-mock helper](evidence/boundary-review.mjs), `finalReturnFailure`, uses the actual collector and return primitive. Only `collectWithSlipRecovery` is stubbed to supply a completed measurement direction. Its send function throws `Review injected movement failure` on final `G1`; collection still resolves, writes one sweep, and logs a warning. [Full trace](evidence/boundary-review.json) retains the commands and written dataset. The same catch-and-warn pattern between sweeps was inspected; the decisive injected failure is the final-return case.

The [Python control-flow helper](evidence/offline-review.py) then supplies that exact HP3 collector artifact through a successful collector-status stub to the parent. With fitting and firmware sends stubbed, the parent eventually calls its application endpoint once, returning zero. [Results](evidence/offline-review.json) and [parent trace](evidence/tmp/swallowed-return-reaches-parent/trace.json) show that the existing exit-status contract cannot detect the swallowed failure. This is a control-flow counterexample, not a numerical fit or physical motion test.

Propagate both ordinary return failures. Retain already-written measurements as partial evidence, but let the collector fail so the parent does not merge or apply. The new rejection-recovery block already does this correctly: the identical injected movement exception there aborts and publishes no rejection dataset. A transport failure before recovery is likewise fatal. The parent was notified before any source change.

## P2: verify each commanded recovery return before treating it as resolved

The new rejection handler at `sweep_data_collection.mjs:1859` records `acquisition_rejections` only after awaiting `returnMotorsToOriginOneAtATime`. That ordering is good. However, candidate `autocal/control/primitives/uncalibrated_actions.mjs:544–608` waits for stationary encoders without checking the commanded return axis against its target. A completed quiet wait establishes stability, not movement completion.

In `returnPrimitive` of the [offline trace](evidence/boundary-review.json), all four encoders remain permanently at `[100,200,300,400]` mm while every `G1` is acknowledged. The real primitive commands `X-100`, `Y-200`, `Z-300`, `U-400`, then resolves with the unchanged lengths. Feeding that real primitive through the new rejection handler produces a persisted boundary rejection despite none of the commanded moves occurring. An explicitly thrown return error, by comparison, publishes no boundary.

This does **not** establish that every line can or must remain at zero simultaneously. The helper deliberately places other motors in force mode, which can move an already-returned line; the zero-reference lengths may also describe a slack/incompatible tensioned pose. A final all-lines-at-zero assertion would not follow from this evidence. Check the just-commanded axis after settling and holding it, before a subsequent force-mode change, using a justified encoder tolerance and valid conversion. Fail if that command did not track.

Subsequent `prepareSweepPositioning` reads live encoder lengths and computes moves to the next fixed targets; `performForceSweep` then enforces its fixed-angle guards. Thus the next sweep does not require a final globally zero pose. That does not justify treating a failed position-command return as successful recovery or safe boundary evidence. The parent was given this distinction and is preparing a per-command check, outside this pinned review.

## Favorable recovery, merge, and stopping results

The actual local-probe checks retain the intended distinction: reversible coupling is accepted, a flat response and hysteretic sensor return are rejected, while failed held-drive restoration and transport failure throw. The probe restores the drive before returning a sensor-hysteresis rejection. Local probes are empirical transmission evidence under the tested preload; they do not sense absolute empty-spool occupancy.

The collector handles only `UnmeasurableSweepError` as a recoverable acquisition rejection, and only at `completedSweeps >= 3`. The planner supplies that count from prior nonempty sweeps and owns the forwarded `--completed-sweeps` argument. The first three nonempty sweeps therefore retain the mandatory half-spindle-turn reserve. Bootstrap does not opt into later probing. Actual probe drive-target tracking errors remain ordinary failures, not learned boundaries. The tests also cover fixed-anchor slip, settling/transport errors, insufficient noise, force-specific local probes, and reserve checks on raw and mean readings.

The [independent loop experiments](evidence/offline-review.json) show:

| Comparison | Result |
| --- | --- |
| Three consecutive rejection-only collections, `max_steps=4`, `patience=1` | One fit and one stopping-policy update; two frozen replans; terminal reason `max-steps`, never patience/spread consensus. |
| Rejection then collector transport failure | Exit 2; no application; no refit or extra stopping update. |
| Immediate collector transport failure | Exit 2; no application. |
| Empty collector result without measurements or rejection | Exit 2; no application. |
| Resolved rejection followed by no remaining candidate | One stopping update; explicit `workspace-exhausted` acceptance. |

The budget counts rejected attempts, so they cannot silently create repeated fits, consume patience, or accumulate geometric consensus. Best-so-far application on the explicit max-step budget is still possible; it is not classified as normal patience/spread completion. `workspace-exhausted` is now a normal completion reason and should be interpreted as planner exhaustion, not proof of calibration accuracy or true workspace coverage.

Rejection-only merge retains exactly the original measurements and force reference; an artificial rejected fixed target at 1000 mm does not enlarge the exploration bounds. Replanning filters recorded rejected configurations, and the novelty-spacing fallback does not re-admit them. The production planner uses the dataset and fitted geometry; the inspected planning/profile/workspace paths do not read scene truth.

## Force-profile corrections

The v2 generator removes infeasible rows rather than clamping forces, requires both interchangeable sensor lines above the tuned minimum, and keeps the longest contiguous feasible run. An artificial infeasible-gap test selected only indices `[3,4,5]`, excluding the shorter disconnected runs. Both direction tables contained exactly those three poses, so interpolation cannot bridge that sampled gap.

The [analytical comparison](evidence/offline-review.json) uses the same nominal anchor fixture, N = `[2,2,2,3]`, U = `[2,2,2,4]`, Y = `[528.3,528.3,528.3,398.9]` mm, mover weight `0.081*9.81` N, tuned middle force 0.23 N, and strand target 0.2875 N for both revisions. For each generated table row, it solves static equilibrium at the actual tabulated sensor force and evaluates the firmware-weighted target.

| Fixed A/D targets | Prior table | v2 table |
| --- | --- | --- |
| `[390,-1170]` mm | 50 rows/direction; maximum target error 0.209613 N | 38 rows/direction; maximum target error < 8e−16 N |
| `[390,-100]` mm | 54 rows/direction; maximum target error 0.073092 N | 50 rows/direction; maximum target error < 5e−16 N |

Every retained v2 nodal equilibrium has positive line tensions and the tabulated sensor force is at least 0.23 N. These are static algebraic checks at table nodes. They do not certify interpolation accuracy at arbitrary poses or actual force tracking.

The collector now intersects observed drive travel with each direction's profile support. In an independent encoder mock with observed travel `[-600,0]` mm and table support `[-200,-100]` mm, all five retained raw and mean measurements lie inside that table and record the expected interpolated setpoint force. A disjoint table yields `UnmeasurableSweepError` with zero published points; a flat later sensor likewise retains no points. A drive-target tracking error throws an ordinary error. Holding an endpoint force outside support remains available for transit, but the reviewed successful measurement path does not retain those transit observations.

The profile policy is versioned `reeving-aware-v2`; old v1 profiles, mixed profiled/constant-force datasets, and inconsistent targets are refused for continued collection, while offline replay remains available. The targeted continuation tests passed.

## Reproduction and limits

From repository root, with `D` set to this report's directory:

```bash
D=research/output/program-2026-11-15-review-and-harden-stretch-insensitive-autocal-robust-bootstrap-h-3fa9855a/session-2026-11-15-independently-review-candidate-dfc0e3e3-boundary-probe-recovery-c6a764bc
node "$D/evidence/boundary-review.mjs"
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. .venv/bin/python "$D/evidence/offline-review.py"
PYTHONDONTWRITEBYTECODE=1 TMPDIR="$PWD/$D/evidence/tmp" .venv/bin/python -m pytest -p no:cacheprovider --basetemp "$D/evidence/tmp/pytest" autocal/tests/test_autocal_no_collect.py autocal/tests/test_equal_strain.py autocal/tests/test_workspace.py autocal/tests/test_stage_replay.py autocal/tests/test_rough_bootstrap.py -q
TMPDIR="$PWD/$D/evidence/tmp" node node_modules/jest/bin/jest.js --runInBand --no-cache --cacheDirectory "$D/evidence/tmp/jest-cache" --runTestsByPath autocal/control/tests/behaviors/force_sweep_settling.test.mjs autocal/control/tests/behaviors/force_tuning_checkpoint.test.mjs autocal/control/tests/behaviors/spool_reserve.test.mjs
```

Both independent helpers' assertions passed. The Python suite passed **69 tests in 3.80 s** ([log](evidence/pytest.txt)); Jest passed **38 tests across 3 suites** ([log](evidence/jest.txt)). Helpers fail if the guarded live source no longer matches the frozen candidate. The JS helper reads the changed entrypoints from Git and guards its real imported dependencies. The Python comparison reads the prior equal-strain module from Git and checks current source hashes.

The strongest alternative explanation for limited practical impact is that real firmware normally completes these returns or reports errors before they reach the swallowed cleanup block. This review establishes missing enforcement under reproducible command/encoder failure models; it estimates neither failure frequency nor resulting mechanical harm. Force algebra assumes ideal static equilibrium, correct fitted geometry, common compliance, and the firmware reeving model. Reversible encoders do not identify actual carriage pose, absolute spool occupancy, routing, slack, pulley/friction losses, force error, or absolute home.

The parent's separate HP3/HP4 physics runs, achieved coverage, anchor accuracy, and all hardware validity remain outside this report. Acceptance would change after both movement gaps are fixed and their decisive offline cases reject instead of publishing success, followed by appropriate production-path validation. No global corpus index, shared topic, agenda, report, or source file was changed because this assignment allows writes only inside its artifact directory.
