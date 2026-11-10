# HP3 and HP4 bounded pulse-and-hold force tuning

Status: in progress — experimental implementation is not ready as a replacement.

## Abstract

A pulse-and-hold force tuner was implemented and evaluated in fresh production native HP3 and HP4 simulations. It limits displacement, detects declining force/displacement compliance, confirms at the same load, and returns to an observed interior endpoint. Two integration defects were corrected: RRF simulation holds resumed stale targets, and sweep positioning left motors pulling during an ordinary settling check. In an eight-point HP3 comparison, collection changed from 28.16 to 26.35 wall seconds, 1476.23 to 1125.54 simulated seconds, and 3.096 to 1.408 N peak segment force, with a smaller sampled cable-length range. The full HP3 bootstrap completed 24 points but required 2922.93 simulated seconds versus 2423.04 for the baseline. HP4 collected eight points where the baseline failed before collecting; its full bootstrap subsequently failed during paired-drive size tuning on reversed travel. The implementation therefore does not establish an overall improvement across both machines. Hardware safety, true reachable-volume fraction and calibration accuracy remain unvalidated.

## Topics

HP3, HP4, Hangprinter, autocalibration, force tuning, momentum, controlled pullout, cable loads, encoder settling, workspace coverage.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Supplied failure exhausted trial count | E1 | Rejected: cap reached after 13 trials | [Log analysis](E1-log-analysis.json) |
| RRF simulation held the current position correctly | E3, E5, E9 | Rejected before correction; corrected hold passes regression and simulated endpoint return | [E9](E9-hp4-endpoint-return/result.json), [replay](E9-hp4-endpoint-return/metrics.json), [checks](checks.md) |
| Fixed pulse-count stages are adequate knee evidence | E5, E6 | Rejected as a valid inference; replaced with low-progress stages | [Notebook](research.md) |
| New HP3 origin collection lowers load and time | E15, E16 | Mixed: lower load and modestly faster, smaller sampled range | [Comparison](summary.json), [old replay](E15-hp3-baseline-repeat/metrics.json), [new replay](E16-hp3-final-repeat/metrics.json) |
| New HP4 origin collection completes | E13, E14, E17 | Supported after positioning fix: eight points versus baseline failure | [Baseline](E13-hp4-baseline/result.json), [failed new trial](E14-hp4-final/result.json), [corrected trial](E17-hp4-positioning-fixed/result.json) |
| Full HP3 bootstrap is faster | E18, E20 | Rejected: 20.6% more simulated time, both collect 24 points | [New dataset](E18-hp3-bootstrap/sweeps.json), [baseline dataset](E20-hp3-bootstrap-baseline/sweeps.json) |
| New procedure completes full HP4 bootstrap | E19 | Rejected in this trial: size-tuning travel reversal, zero collected points | [Failure](E19-hp4-bootstrap/result.json), [log](E19-hp4-bootstrap/trial.log) |

## References

- [Full report](report.md): measurements, method and limitations.
- [Notebook](research.md): steering, budgets, failures and next decision.
- [Original design](initial-design.md): initial diagnosis and proposed procedure.
- [Trial summary](summary.json): all measured timings and completion counts.
- [Final changes](final-changes.diff) and [source archive](final-source/): reviewable experimental implementation, based on revision 3c30e6b9f8d5dc4de15b188448b0b8e5b68ea79d. Per-trial source snapshots and manifests retain intermediate versions.
- [Supplied browser log](../../../extended-autocal/no-buildup-reference-008/sweeps.full_auto.log): motivating failure, not a matched native performance baseline.
- [Earlier HP3 investigation](../../autocal-comfortable-force-20261010/report.md): prior finite-duration method and disclosed inertia limitation.
- [RRF firmware source](../../../../RRF/ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp): SetODrive3PosMode captures the current encoder position.

## Result and adoption decision

The requested overall improvement is NOT established. The experimental code is in the working tree, but should not be treated as a validated production replacement. It is useful for further research: it removes guessed successful maxima, exposes explicit failures, reduces loads in the measured HP3 origin trial and enables the measured HP4 origin collection. Its pulsed traversal adds cost over the complete HP3 bootstrap, and extending it to paired-drive HP4 size tuning fails. No further live trials were started after the user's request to wrap up the current evidence.

## Measured comparisons

| Workload | Baseline | New procedure | Interpretation |
| --- | --- | --- | --- |
| HP3 origin-fixed, 8 points | 28.160 s wall; 1476.234 s simulated | 26.353 s wall; 1125.542 s simulated | 6.4% lower wall time, 23.8% fewer steps; engineering observations, not a repeated performance benchmark |
| HP3 origin peak segment load | 3.096 N | 1.408 N | 54.5% lower in exact frozen-journal replays |
| HP3 sampled whole-path extension | 0.0619% | 0.0151% | Lower geometric extension proxy; not a measured material safety margin |
| HP3 canonical sampled cable-length span | 861.01 mm | 627.35 mm | 27.1% smaller; cannot claim increased workspace coverage |
| HP3 full bootstrap, 24 points | 2423.044 s simulated | 2922.928 s simulated | 20.6% more simulation work with new method |
| HP3 automatic fixed-anchor travel | 516.951 mm | 476.475 mm | New size estimate is smaller |
| HP4 origin-fixed | Failed before first point, 240.290 s wall | 8 points, 258.269 s wall | Completion improves; failed-run timing is not a speedup baseline |
| HP4 full bootstrap | Not run as a matched baseline | Failed in size tuning, 193.847 s wall | Full replacement remains unvalidated |

HP3 full-bootstrap wall observations are 48.73 s old and 66.77 s new. Frozen-journal validation jobs overlapped part of this later phase, so these are not used as a clean wall-time performance comparison. Simulation work and point counts are directly recorded. The earlier eight-point comparisons ran sequentially before validation replays.

The corrected HP4 origin controller selected 4.6709 N and 7.4734 N in opposite directions, with observed pullout excursions 376.18 and 373.14 mm relative to each direction's starting encoder state. Its replay observed 26.501 N maximum segment force and 0.506% maximum sampled whole-path extension. A commanded drive-force ceiling does not bound every support-line force. No material strain limit or hardware safety claim follows from these values.

## Implemented procedure

The controller uses encoders only. It applies force for at most one second, polls every 500 ms, stops a pulse if average driven-cable displacement reaches 10 mm, and holds position for 300 ms. Three pulses with at most 2 mm progress end a force stage. A decline in incremental displacement per newton below 20% of the observed peak triggers three shorter same-load probes. It then returns to the preceding observed interior endpoint before sampling. Held-anchor drift, free-encoder movement, reversal, force limits, per-stage pulse counts and independent wall/selected-clock budgets are checked. Low progress is not equilibrium. Failure holds the motors and raises a diagnostic; it never substitutes 1 N or treats a reached ceiling as a measured knee.

Automatic sweep directions search at their actual fixed lengths and use the measured endpoint directly. The paired-drive extension is also used by automatic size tuning, but that extension is the unresolved HP4 failure. Explicit manual-force collection retains its requested-force path. The default search ceiling remains 20 N; an explicit --auto-tune-force --force-max N can set another ceiling, including above 20 N.

## Integration findings

RRF firmware's SetODrive3PosMode reads the current encoder and commands a hold there. The simulator previously resumed its retained target. RRF live and recorded commands now request captureCurrent explicitly; raw SetPositionMode retains its existing semantics. JS and Python update the reference offset so subsequent relative moves remain continuous.

prepareSweepPositioning also left the free pair in force mode after moving a fixed anchor. The next ordinary encoder-stability wait then watched motors still pulling. An explicit hold now ends positioning. E13 and E14 preserve the old and new failures before that fix; E17 completes afterward.

## Comparison and evidence contract

Trials use isolated, fresh production native Rust physics worlds with matching no-layering HP3/HP4 scenes and firmware. Both archived old algorithms and the new controller receive corrected RRF holds, separating that integration fix from the algorithm comparison. E15/E16 use one configuration at fixed targets 0,0, with four samples in each direction. E18/E20 use three configurations with automatic size tuning and 24 samples. E19 exercises that full path on HP4 and fails before sampling.

The baseline sources are [baseline_force_tuning.mjs](baseline_force_tuning.mjs) and [baseline_sweep_collection.mjs](baseline_sweep_collection.mjs). Each trial retains a world.headless manifest, baked scene, firmware, source hashes and command/encoder journal. [Replay harness](replay_metrics.mjs) reconstructs production native physics and checks encoder agreement. E15, E16 and E17 replays match every recorded encoder value with maximum error 0 degrees. Peak segment forces are checked at every 2 ms step; whole-path extension and Cartesian trajectories are sampled every 500 ms. Short-segment extension ratios are saved as diagnostics and are not interpreted as material strain.

## Failure preservation and limits

E2 was interrupted after finding a floating-point polling deadline issue. E3 exposed stale holds. E5's apparent knee was rejected because a fixed pulse-count limit can fabricate a plateau. E6 exhausted its wall budget. E14 failed before the positioning hold correction. E19's paired-drive curve reached 747.46 mm average excursion at 7.4734 N, then reversed at the next stage; the controller aborted rather than accepting a false knee. Commands and stopped-world evidence remain in each trial.

The algorithm remains dynamic and parameter-dependent. A lower peak load with less coverage is a tradeoff, not proof of better calibration. No optimizer comparison, hardware measurement, material safety certification, mass/damping sweep or true reachable-volume fraction was established. The full HP3 result rejects the hope of a general speedup. The next research decision is whether to retain the single-drive controller while redesigning paired-drive size tuning and reducing repeated braking cost; that work is not tested here.

## Checks and service state

[Validation record](checks.md): 98 JS tests pass; focused Python hold and command-pipeline regressions pass. Ten native collector tests passed; a Warp cache-write failure passed on retry with a writable temporary cache. git diff --check passes. Source and test logs are preserved beside this report.

All isolated trials and replay jobs have ended. The attached supervisor was refreshed to HP4/no-layering and remains at step zero, but requires a launcher restart to reload changed Python modules. Its preflight is a source-staleness check, not evidence about calibration accuracy. Do not infer hardware readiness from the passing tests.
