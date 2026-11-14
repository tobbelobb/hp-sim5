# Repaired HP3 autocal comparison reaches applied results

Both current and historical native/wrapper runs now complete naturally and apply the same calibration. Their A/B/C/D anchor errors are 4.388/2.892/3.627/2.022 mm, maximum 4.388 mm and anchor RMSE 3.349 mm. The latest open-loop inertia correction produces no applied-accuracy change in this exercised closed-loop workflow. This resolves the earlier session's inability to obtain an applied result, but does not meet the under-1 mm goal.

## What failed and what changed

The [prior comparison](../attempt-0b9a865f76514b04930528d43f2c31ea/report.md) used manual low/mid/max forces 0.01/0.05/0.5 N and sensor force 1 N. Its size measurement was only 1.269203 mm, and its three bootstrap fixed lengths were about 1.29 mm. Given that travel limit, the planner generated 18 feasible candidates. At its normal 10 mm novelty tolerance, every candidate rounded to a bootstrap key, leaving zero novel candidates before information scoring. This explains the immediate no-valid-candidate stop. The very poorly constrained initial fits were a separate observed limitation of that sparse, narrow acquisition. [Saved-data diagnosis](evidence/prior-diagnosis.json).

I repaired the experiment by preserving stock automatic force tuning, without changing planner tolerance, fitter or stopping logic. The new bootstrap measures 474.564797 mm usable travel and fixed lengths 474.537570 mm. It yields 693 feasible candidates, 690 novel. Automatic tuning selects low/mid/max forces 0.012396628/0.247932561/2.613544005 N, using the production incremental-compliance rule. The workflow then collects four further sweeps and reaches automatic consensus. This shows that the prior failure was caused by an unsuitable experiment acquisition regime and its resulting planner input, rather than establishing a general production pipeline defect. It does not isolate one manual force parameter as the mechanical cause. [Stock diagnosis](evidence/stock-diagnosis.json).

A second prior defect masked the historical simulation: the current JavaScript wrapper wrote zero to ABI slot 12 even when loading the old native binary. The new [snapshot helper](../../../autocal/tools/native_snapshot.py) archives the exact Git-selected wrapper and engine source, verifies the binary and build-source hashes, and uses a Node module loader to load the archived wrapper at its original URL. Relative imports work normally, and no checkout wrapper is edited. It records an actual module-load receipt. Launch validation rejects a changed binary, wrapper changes are rejected by the loader, and inherited NODE_OPTIONS are rejected to avoid an unrecorded preload. [Regression tests](../../../autocal/tests/test_native_snapshot.py) exercise mismatched engine source, changed binary and historical wrapper loading without checkout edits.

No production calibration or physics algorithm change is proposed. The candidate adds the supported research snapshot mechanism and tests, plus the repaired automatic-force reproduction runner.

## Protocol and source identity

I read the exact queued task and role once, consulted the current autocal_parameter_variation_focus constraints, recording guidance, researcher wisdom and prior evidence, then ran supervisor preflight and a separate command-context socket probe. Both passed. The initial allocation was 20 minutes. Each calibration had a 480 wall-second limit plus eight seconds for cancellation, five million physics steps, at most 20 active-learning iterations, 20 points per direction, 400 solver iterations and four restarts. Both finished before those bounds, with no retries or cancellation. Two additional noncalibration torque controls each use 500 steps at 2 ms. Simulation jobs ran sequentially.

Both fresh worlds use HP3, known 30/30/30/30 mm radii, zero buildup, disabled layering, one 2 ms physics substep, native Rust, speedup 25 and event recording. Optimizer is production fast L-BFGS-B, normal one-click automatic force tuning and normal consensus stopping. No noise, geometry, friction or force settings were selected using simulated truth. Plant truth was inspected only after collection for diagnosis and scoring. This is one nominal deterministic plant, not a varied evaluation matrix. No common injected-noise seed or extra noise intervention was added.

The calibration code revision is `587f67f7`, held fixed across the pair. The per-file fitter/stage source hashes match exactly. Aggregate autocal-tree hashes differ because the new research helper and tests were written while the stock run was underway; those additions do not alter the fitter or collector. Exact source hashes and budgets are in the [experiment ledger](evidence/experiment-ledger.json) and the complete raw stage envelopes.

Historical simulator scope is the latest native dynamics correction only: `2c0b86ff189d27859b83543ed3fa837cfa658a3b` immediately before `c7713d82`, paired with its matching wrapper, while the surrounding app and calibration remain current. Old native SHA-256 is `98f4c408c637ed7d081a279d4ed048e3390722919c797b030a4fa5b591848742`; current is `66a8451bc1a42310e4dfae166288737e595f75bc1173c8f88e92163afb94a5d8`. These are the preserved prior-session release builds, both Rust 1.99.1, no PGO and no additional Rust flags. The current default artifact matches that same current build. [Historical snapshot](evidence/historical-v2/snapshot.json), [build metadata](evidence/historical-v2/native-build.json), [archived wrapper](evidence/historical-v2/wrapper.js), [archived engine](evidence/historical-v2/engine.rs).

The native wrapper is really loaded through the hook, as the [load receipts](evidence/historical-v2/loads.jsonl) show. A control using this same loader, with no manual ABI override, reproduces old open-loop angle 0.002386002181 rad and current angle 0.005946490468 rad. Current agrees with the analytic equilibrium; old motor/cable torques are 0.059509/0.178526 Nm. [Torque results](evidence/torque-control.json). Thus the equal calibration result is not an artifact of another masked version swap.

## Applied results and effort

| Native/wrapper case | Stop | Sweeps / points | Iterations | Physics steps | Simulated seconds | Complete wall seconds |
|---|---|---:|---:|---:|---:|---:|
| Current | Automatic spread-consensus, exit 0 | 7 / 280 | 5 | 2,240,486 | 4,480.972 | 325.141 |
| Historical matching wrapper | Automatic spread-consensus, exit 0 | 7 / 280 | 5 | 2,240,486 | 4,480.972 | 299.092 |

| Case | A error, mm | B error, mm | C error, mm | D error, mm | Maximum, mm | Anchor RMSE, mm |
|---|---:|---:|---:|---:|---:|---:|
| Current applied | 4.388405 | 2.892334 | 3.627368 | 2.022004 | 4.388405 | 3.349253 |
| Historical applied | 4.388405 | 2.892334 | 3.627368 | 2.022004 | 4.388405 | 3.349253 |

Both manifests have normal_completion=true and the applied command is verified by final firmware M669 readback:

```text
M669 A0.00:-1904.34:-279.35 B1647.90:951.39:-279.35 C-1648.45:951.93:-279.35 D-0.09:0.00:1902.02
```

Final M666 readback confirms R30 and Q0. The complete sweep objects match exactly, not just final errors. Automatically tuned forces match after removing their timestamp; so do measured travel, M666 settings and baked scene hash. All observed simulation_settings messages enable closed-loop motors and disable layering, beginning at step zero. The latest correction acts on open-loop inertia, so it is inactive in this operational path. [Pair equality gates, exact dataset hashes, every candidate's per-anchor errors and distribution tails](evidence/summary.json).

Errors are Euclidean distances to the authored scene-home virtual anchor coordinates, in millimetres, without fitting an alignment. Anchor RMSE is sqrt of the mean of the four squared Euclidean errors. [Frame audit](evidence/frame-and-mode-audit.json) derives the A/B/C virtual anchors by subtracting each moving attachment's authored translation from its matching fixed guide, obtaining A=(0,-1900,-280), B=(1645.448267,950,-280), C=(-1645.448267,950,-280) mm. All three D paths give (0,0,1900) mm. The attachment centroid z=0.1 m and tip offset z=-0.1 m give tip-home zero. Native encoder readback returns authored-zero angles directly; no encoder-zero command or world reset occurs in either trace. The settled mover pose was not separately measured, and settling/elasticity/model bias remains part of the error. These are authored-home scoring coordinates, not a post-hoc translation to an unobserved settled frame.

The [reference audit](evidence/reference-audit.json) finds exact consistency between recorded encoder mean angles, configured 30 mm radius conversion and l_drive_mu/l_sensor_mu in both prior and stock bootstrap data. No missing radius/unit conversion is established. Per-point fixed-axis encoder means have zero range in these nominal bootstrap sweeps; the earlier point-varying fixed-length remedy is not needed to explain this failure.

The two runs share observations and one plant, so they are not independent reliability evidence. All applied anchors are below the earlier 5 mm threshold. Every anchor still exceeds 1 mm. The UI calls the fit ideal at score 0.2818; that score is not millimetres and does not establish accurate geometry. Consensus can agree on a biased result. Wall times are single sequential samples affected by warmup and scheduling, not a speed improvement claim.

## Evidence and reproduction

[Current raw bundle](evidence/stock-auto-raw.json) and [historical raw bundle](evidence/historical-auto-raw.json) retain every original fit/history stage envelope with exact inputs, source hashes, options, per-iteration decisions, logs, force checkpoints, manifests, clocks and firmware readbacks. The [current dataset](evidence/stock-auto/sweeps.json) and [historical dataset](evidence/historical-auto/sweeps.json) are separately committed. [Artifact manifest](evidence/artifact-manifest.json) records hashes and sizes of original artifacts. Large RRDs, event journals and service files remain workspace-only. This limits future motion audits if that workspace disappears; all small numerical and fitting evidence is committed.

From the repository root, source `.agents/skills/cloud-research-client/scripts/env.sh` in every shell. Run `.venv/bin/python -m pytest -q autocal/tests/test_native_snapshot.py`, which passed all three tests. The [runner](scratchbook/run.py) starts an automatic-force calibration with fixed budgets and preserves any failed/partial output. It refuses existing output directories. `.venv/bin/python research/sessions/attempt-64d28c7e5e6c4e778656afa4639a46c6/scratchbook/run.py NEW_CURRENT_LABEL` repeats current collection.

To rebuild the historical binary after losing the retained prior artifact, use [build_old_native.py](scratchbook/build_old_native.py), which copies the current crate to an isolated directory and replaces only engine.rs from the selected historical commit before a no-PGO build. Run `autocal/tools/native_snapshot.py --revision 2c0b86ff --native PATH_TO_OLD_NODE --output NEW_SNAPSHOT_DIR`, then run the collection runner with `NEW_HISTORICAL_LABEL NEW_SNAPSHOT_DIR`. Exact executed commands and environment selections are preserved in each raw bundle. Reproduction needs fresh output paths. The [summarizer](scratchbook/summarize.py) recreates scoring and pair checks. [Torque control](scratchbook/torque_control.mjs) can run with snapshot_environment to verify the historical branch without modifying source.

Validation includes three helper regression tests, two naturally completed physical calibrations, exact paired-data/force/source checks, native and firmware readbacks, frame/unit audits and the loader's independent torque control. No production physics or fitting source changed. Earlier rotor coupling, substepping, force accumulation and cable-material changes remain untested interventions. A future simulator-history study should select a correction exercised by closed-loop autocal. Remaining sub-millimetre work needs a separate residual/model audit using the preserved valid data, rather than another force or solver tweak selected from truth.
