# Physical validation of HP4 local endpoint discovery and backoff

Status: interrupted at user request — no validated HP4 fix.

## Abstract

Following a failed manual HP4 test, we inspected actual effector translation, rotation and cable slack in sampled production Rust Native recordings, rather than accepting point counts as evidence of sound motion. The committed baseline reproduced a roughly34cm size-probe snap-back. Earlier plateau rollback and all-line backoff alone did not cure it. A smaller-step reeved probe stopping at sensor/drive encoder response3, before the observed gain peak, reduced the size-probe turnback to about33mm in E44 and reached about0.85m effector X during the first sweep probe. Backoff now retraces observed line-length states with all motors held; size return retraces the observed path. HP3 completed a four-point bidirectional sweep in E46; HP4 E45 failed at the first measurement in its second direction. These are simulated motion checks at requested25x with an explicit1physical-second-per-probe-RPC delay model, not hardware tests or measured browser timing. Persistent postfailure jostling from the user's browser was not reproduced during the15-second Native held observations. No calibration-accuracy or speedup claim is made.

E45 reached both HP4 endpoints, but the second direction failed its30-second physical encoder-settling deadline. Its final held observation retained body/cable oscillation. E47 increased the relative-response stop to90% of peak; it prematurely selected only44.5mm size travel and was cancelled at the user’s request. The candidate controller/profile changes were reverted; the full patch is preserved in `rejected-candidate.patch`. Existing d9620fd8 behavior is still unfixed.

## Topics

HP4, HP3, autocalibration, endpoint discovery, pullout, encoder response, cable slack, effector trajectory, backoff, Rust Native, physical smoke test, Rerun.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Previous numerical collection success establishes sound physical motion | E39, user008 | Rejected | [Baseline physical metrics](E39-baseline-physical-smoke/physical-metrics.json), [plot](E39-baseline-physical-smoke/physical-smoke.png) |
| Earlier plateau rollback alone removes the snap | E40 | Rejected | [Metrics](E40-early-retreat-physical-smoke/physical-metrics.json) |
| All-line observed-state backoff alone removes the snap | E41 | Rejected | [Plot](E41-observed-backoff-physical-smoke/physical-smoke.png) |
| Waiting for relative-response decline sufficiently avoids the problematic region | E43 | Mixed; still90mm size turnback | [Plot](E43-early-response-stop-smoke/physical-smoke.png) |
| Stop on rising encoder gain before its peak improves physical endpoint motion | E44 | Supported for one size probe and first sweep endpoint | [Plot](E44-before-gain-peak-smoke/physical-smoke.png), [metrics](E44-before-gain-peak-smoke/physical-metrics.json) |
| Observed-path size return supports complete HP3/HP4 collection | E45,E46 | Rejected for HP4: second-direction point settling timeout; HP3 four points complete | [HP4 log](E45-hp4-retrace-full-sweep/trial.log), [HP3 result](E46-hp3-retrace-full-sweep/result.json) |
| Persistent postfailure held-body motion is reproduced | E39–E44 | Not reproduced under this model | Recorded15-second held observations |

| A90%-of-peak relative stop preserves useful range | E47 | Rejected for size discovery:44.5mm free travel; remainder interrupted | [Partial log](E47-hp4-before-response-decline/trial.log) |

## References

- [Research notebook](research.md): accepted steering, budgets and experimental decisions.
- [Previous numerical report at d9620fd8](report-d9620fd8.md): preserved previous conclusions; the new physical checks show their acceptance criteria were insufficient.
- [User008 log](../../../extended-autocal/no-buildup-reference-008/sweeps.full_auto.log):91.369wall-second endpoint failure before measurement collection.
- [Production smoke harness](smoke_trial.mjs), [launcher](run_smoke.py), [physical inspection](inspect_smoke.py): experiment implementation; each experiment archives its tested controller/profile sources and diff.
- [Pullout terminology](../../../../autocal/control/behaviors/PULLOUTS.md): physical anchors stay stationary; positioning sets held line lengths.

## Scope and limitations

All new experiments used isolated production Rust Native worlds and sampled RRD. The attached supervisor failed a stale-source preflight; its world was not advanced. Command-context socket connectivity passed, which establishes only tool connectivity. The explicit RPC delay models continuing physics during requests and is not a browser parity benchmark. Some trials overlap, so wall timings are not used for comparative performance claims. These smoke tests cover one sweep configuration, not all fixed-offset combinations or fitted calibration accuracy. Failed trials are preserved.

Tool audit: replay/telemetry already exist in scripts/profile_headless.mjs, scripts/replay_collection.py and tests/benchmark/autocal_recording_modes.mjs, with production FlightRecorder and MCP experiment comparison/context tools. No general world-plus-RRF restoration found; README says snapshots are observations. Missing capability narrowed to integrated production single-probe stage selection, phase-aligned diagnostics and explicit starting-state/replay-versus-live contracts. Wrote research/prompts/single-probe-diagnostic-workflow.md; corrected earlier handoff. Source audit only; no new simulation or runtime validation.

Owner steering: HP4 wobble is unrealistic and is to be treated as a simulation bug. Created research/prompts/hp4-simulation-wobble-bug.md for a separate root-cause investigation, reduced physical reproducer and Native before/after validation. Specific subsystem remains unproven; no new simulation or source changes this turn.
