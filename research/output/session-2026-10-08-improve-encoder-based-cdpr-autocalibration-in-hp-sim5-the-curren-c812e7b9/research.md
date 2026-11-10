# Encoder-only autocalibration: speed, robustness and physical ranking

Objective: reproduce slow/fragile fitting and score/anchor disagreement; implement the best supported small improvement while retaining one-click collection and automatic completion.

Constraints: encoder-only calibration observations; ground truth is evaluation-only. Preserve failed trials and independent sweeps. Use .venv/bin/python. Fresh launcher world, no resumed physics. No hardware accuracy claims.

Budgets: initially up to 12 offline experiment groups, 60 CPU minutes; at most 2 simulated collection jobs if offline evidence requires them. No movement started. Reassess after baselines. No user-imposed numeric budget.

## Initial ideas from first principles (before prior reports/code)

1. Joint anchor/offset/latent-position distance closure with redundant cables; eliminate latent poses by variable projection. Relative encoders require a coordinate gauge and a metric scale; redundant constraints cannot identify arbitrary rigid transformations or uniform scale without a known reference.
2. Fit geometric sensor-distance residuals rather than algebraic conic error; density/span/noise normalization could prevent low-score distorted anchors and short-arc degeneracy.
3. Select diverse sweep configurations or high-information points (Jacobian rank/log determinant), rather than many nearly duplicate samples. Sequential batches should stop once uncertainty and held-out prediction stabilize.
4. Reuse warm starts, analytic/JIT derivatives, cached geometry and staged fitting; reserve multi-start/restarts for ambiguous cases.
5. Whole-sweep holdout and bootstrap stability to reject updates which overfit nuisance spool/elasticity parameters; retain a safe incumbent automatically.
6. Robust losses and balanced sweep weights to tolerate outliers, hysteresis and uneven point counts without repeating collection.
7. Bidirectional sweeps and changing held lengths to separate spool/anchor ambiguity; encoder plateau-based settling and continuous acquisition could reduce time per point.

Leading hypotheses pending literature, prior evidence and code inspection: physical residual ranking; informative thinning; numerical reuse/staged fitting. No experiments completed yet.

Accepted steering: user explicitly requests literature/first-principles ideas before searching prior reports, reproduction before implementation, recorded and synthetic comparisons, shorter/faster collection and unchanged one-click automatic stopping.

Next decision: inspect prior evidence and current optimizer to choose discriminating baseline experiments.

## Experiment decisions
E01: fresh full-auto baseline matrix (11 recorded simulation fixtures), running. Frozen source c5e6b90f2c97139e92c4e7e5a4a82b448f448e38.
E02: recorded thinning, retain endpoints and at least 5 points per physical drive direction, half where possible; test bad-points, ten-points, seventh and April HP3. This is retrospective point deletion, not a measured collection time saving.
E03: controlled settling traces, baseline retains vibration history when testing quiet stability. Candidate restricts quiet test to last stableWindowMs; leave vibration fallback, tolerances, sampling and automatic stopping unchanged.
E04: matched headless-js/RRF collections from fresh worlds, 3 configurations x 2 directions x 6 points. Baseline job 5c3fe5661f19484cb54361b1250ce5cc, running. Compare simulation time, points, residual validity and post-decision sensor drift; nominal point-anchor truth differs from distributed authored cable geometry, so do not label residual agreement as accuracy.
Next decision: reject thinning if recorded physical error/robustness deteriorates; retain settling fix only after trace safeguards and native comparison.

E03-v1 failed: naive recent-array filtering dereferenced empty samples after malformed reads and would lack full coverage under poll jitter. Preserved v1 code/results; corrected candidate retains the guard and the sample at/before the quiet-window boundary. This is a failed implementation trial, not a negative result for the window hypothesis.

E02 complete: four recorded half-point replays. Bad-points and ten-points anchors worsen; seventh HP3 improves but needs 6 vs 5 iterations; April needs 8 iterations. Reject unconditional thinning as a default.
E03 corrected window candidate passes 8 settling tests; baseline fails 3 of those.
E04 baseline complete: 36 points, 3 sweeps, 429.668 simulated s, 250.220 wall s, residual ingestion 36/36. Candidate job a4498f9082e44e85b1c1b66c66bd229b running in fresh world 77bb83dffe5c4a74920b2d6ad6e296d6.
E05 complete: 12 known-radius analytical HP4 fits. 12 points remain accurate in clean cases, while 6 points violate the current min_points=10 (all 9 sweeps invalid/sentinel). Synthetic outlier benefit does not transfer uniformly to recorded spool fits.
E06 baseline encoder trace replay: 121 heuristic episodes, 118 paired completions, 39 earlier returns, total 347.0 -> 219.5 s, max extra drift to later recorded endpoint 1.932 degrees. This is off-policy trace evidence, not an end-to-end saving or calibration accuracy. Exploratory rate-normalized quiet threshold reduces max drift to 0.325 degrees, 114 paired completions/7 censored, 341 ->242 s. Additional collection only warranted if unguarded window changes measurement validity materially. Initial E06 parser failed to account for duplicate bridge responses and found zero episodes; preserved v1.
E07: 310 Python tests pass, 5 slow tests deselected. Collector Jest 59/60 pass; HTTP fixture cannot bind loopback in sandbox (listen EPERM), timed out. Existing native supervised service collections independently exercise actual firmware/bridge. No permission escalation requested.

Evidence steering: unguarded native quiet-window candidate retains 36/36 predictions, but recorded trace has up to 1.932 degrees of later drift. Do not accept the largest speed gain by itself. Adopt a stricter recent-window range proportional to stableWindowMs/vibrationWindowMs, preserving the prior long-window drift-rate allowance; vibration fallback keeps its existing tolerance. Expand movement budget to 3 matched jobs (108 points total) to test the conservative version from a fresh world. No hardware movements. E04b unguarded candidate complete, preserved as an exploratory trial; E08 will test final rate-guarded window.

## Final decisions and completed evidence
E08 final guarded collection 7f11879604bb4ce6bc577625c0259cd8: complete, 36/36 observations, 320.364 simulated s vs baseline 429.668 (25.44% saved); 180.898 vs 250.220 wall s (27.70% saved in overlapping workload). Nominal firmware-anchor residual 0.2878 vs 0.2822 mm; noise sigma changes are disclosed, not hidden as accuracy equivalence. Retain duration-scaled recent quiet window; reject unconditional thinning and unguarded speed-only candidate.
E09 complete: all eleven final replays match fresh baseline and stored references at 0.01 mm comparator tolerance, all 5,184 non-timing numerical event fields and stop decisions identical. E08 final Jest: 55 pass with blocked HTTP file excluded; E07 Python: 310 pass, 5 slow tests deselected.
Native evidence and Rerun inspection complete, all 3 jobs finished automatically. No job active. After capturing final evidence, the service was reset to reload the changed collector; current world is fresh and idle. Completed jobs, datasets and RRDs remain archived. Initial budgets reassessed once to allow 3 native jobs for conservative guard verification; 108 total measured observations; positioning and force-transition movements are additional and retained in traces. No hardware tests.
Report and abstract complete. Next scientific decision beyond this implemented change: nuisance-aware information per collection time with independent configuration holdouts and explicit scale/gauge, then hardware post-window drift and positioning measurement. Proposed, not tested.

## Verification handoff
User asks which commands verify the change and whether to collect a fresh dataset. Checked repository test, regression and browser CLI entry points. Recommend settling tests, Python coverage and recorded regressions first, followed by a fresh full-auto simulated dataset using usual point count and spool options. Existing-data replay cannot measure changed collection waits. Fresh collection should use a unique missing dataset path and leave automatic stopping enabled. No new experiment was run during this handoff; completed results and limitations above remain unchanged.

## Browser verification and coverage clarification
User reports starting a visible browser full-auto HP3 run with global radius fit, base radii 30, buildup factor 0.636619, r0 bounds 39,40, speedup 25 and verbose sweep traces. They ask whether fresh collection was tested, why no browser moved, and which tools are useful or missing. Rechecked E08 metrics and MCP capabilities: collection used production headless-js, not independent native-python physics; browser-js and browser automation are available. Prior matched collections used HP4, fixed configurations and force autotuning off, and did not run the fresh-data adaptive full-auto CLI through acceptance. Recorded replays verify unchanged optimizer/stopping on stored observations only. The user browser trial is in progress; no result has been inspected or inferred. Do not reset, steer or launch competing collection while this run is active. No extra experiment or movement budget consumed. Next decision remains assessment of fresh full-auto collection/calibration evidence; an integrated headless full-auto CLI and matching browser/headless tests would make that coverage easier.
