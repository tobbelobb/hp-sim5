# Extra-sensor pipeline and time-to-first-good-fit audit

Question: does the extra range reach a good fit earlier, and does every stage use
it? This is saved-data analysis only. No production code or simulation changed.

## Pipeline audit

Accepted fifth ranges enter the rigid prefit, anchor fit, joint home-offset fit,
per-pose stretch and observed information matrix. Extra commanded forces also
enter force-profile planning. But integration is incomplete:

* Candidate information predicts only fixed + drive + sensor ranges; the extra
  future length is masked out (`autocal/pose_planning.py:174–176`). Existing extra
  observations influence the base information, but their future gain is missing.
* Frozen history validation and selection use EllipseCostFunction over the
  drive/sensor pair (`autocal/predictive_validation.py:54`), not all-range residuals.
* Initial rough collection keeps the spare line compliant. The benefit starts
  during profiled remeasurement, after checked rough geometry.
* Stopping uses fitted-anchor changes and patience, so benefits enter indirectly.
  Score normalisation still divides by d+1, although accepted points can have
  four or five ranges (`autocal/fit_stage.py:429`). The information matrix uses
  rigid consistency rather than derivatives of the full stretch-corrected model.

The score/covariance approximations are limitations, not a demonstrated cause of
any particular inaccurate fit. The candidate and history omissions are confirmed
code paths. This was an incomplete integration of the extra measurement model.

## Prefix replay

Starting from each final recorded dataset, use every prefix from three profiled
sweeps onward. Both arms use the production seed built from the recorded encoder
travel, the joint anchor/home-offset fit, and the shifted range fit. Both retain
all actual torque loads. The only difference is admitting accepted extra ranges.
No nominal geometry seeds, new noise, altered trajectories or adaptive planner
replays are used. Nominal geometry is used only to score results.

Define "good" as at most 2 mm maximum absolute anchor-coordinate error. Also report
5 mm as a coarser threshold. These are diagnostic thresholds, not production
stopping rules. Each sweep has 20 points.

| Machine | First ≤2 mm, 4 / 5 ranges | First ≤5 mm, 4 / 5 ranges | Prefix fits >5 mm, 4 / 5 ranges | Prefix fits >2 mm, 4 / 5 ranges |
|---|---|---|---|---|
| Direct | none through 12 / 4 | 5 / 4 | 2/10 / 1/10 | 10/10 / 1/10 |
| Reduced | 4 / 4 | 4 / 4 | 1/11 / 1/11 | 6/11 / 9/11 |

At three sweeps, worst errors are 203.56 → 34.43 mm direct and 170.15 → 9.31 mm
reduced. At four, they are 11.12 → 1.01 mm direct and 1.72 → 1.32 mm reduced.
Thus extra data improves early weak-geometry fits substantially, but only direct
HP5 shows an earlier crossing of the 2 mm threshold. Reduced late-data fits can
worsen, and it has more prefixes above 2 mm with the extra range.

## What actually happened in the logs

The uninterrupted first attempts started refinement only after five profiled
sweeps: their first logged fits already had worst errors 1.326 mm direct and
1.033 mm reduced. The later continuation replay examined three- and four-sweep
prefixes before collecting more. Its ~8–9 seconds between replay rows is fitting
old data, not acquisition time. Do not report it as time-to-first-good-calibration.

In the continuation logs, direct selected fits after the fourth profiled sweep
stay within 1.51 mm. Reduced reaches 1.32 mm after four, improves to 1.03 after
five, then drifts to 4.54 mm at twelve before 3.43 mm at thirteen. History selection
finally applies a near-1 mm earlier/consensus estimate. A good final selection
therefore does not imply fewer bad intermediate fits.

This supports better direct HP5 time-to-quality under the recorded acquisition
schedule, measured in required sweeps, rather than a proven wall-time speedup.
The four-range counterfactual uses data collected under the new force policy;
an actual old-policy run would plan differently and spend less time probing.
The unchanged rough-bootstrap cost is also excluded from these prefix counts.

Reproduce from the repo root with `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
PYTHONPATH=. .venv/bin/python .../prefix-audit/replay.py direct` (or `reduced`).
Results: direct.json, reduced.json and summary.json. Fits use deterministic data;
these are single trajectories, not distributions over independent machines.
