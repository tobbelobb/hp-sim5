# HP5 integrated sensing and early acceptance

Implementation on main: `68260871`. This supersedes the profiled-only rollout in
`089a7ad3` and closes the confirmed pipeline omissions in the prefix audit.

## Changes

* Both free low lines are sensors from the first recorded HP5 measurement,
  including initial collection. Pullout, tuning and transit can still show slack:
  idle force preserves coverage. Before sampling, both sensors are preloaded,
  the spare line is held at the measurement force, and its range is gated by
  reversible coupling evidence. Rejected ranges keep their commanded load.
* HP5 constant-force measurements now use force-aware stretch correction. Once
  checked, bootstrap data enter refinement directly. They are not collected a
  second time for equal-strain profiles. HP3/HP4 and historical policies remain.
* Future candidate information includes the fifth range. Observed information,
  covariance and rough observability use derivatives of the force-aware residual.
  Candidate forecasts remain ideal geometric forecasts conditional on usable ranges.
* Frozen validation scores all accepted ranges, with each candidate's own offsets
  and the recorded loads. It does not borrow offsets learned on future data.
  HP5 applies the best validated model with its matched offsets and diagnostics,
  instead of averaging its geometry with an unvalidated newer model.
* Cost normalisation counts independent range constraints after eliminating the
  three nuisance pose coordinates for each point, handling mixed four/five ranges.
* The `good-fit` stopping rule needs two successive eligible HP5 fits: >=4 sweeps,
  extra ranges accepted on >=50% of points, full-rank information, force-aware
  correction, range RMS <=0.05 mm, estimated max anchor std <=1 mm, increasing
  measurement count, and <=2 mm change in every anchor coordinate. Poor or
  repeated fits do not count. Agreement alone cannot bypass the quality checks.
  The existing patience limit remains a fallback. `--stable-step-mm 0` disables
  the new early criterion too.

These are local model-consistency checks, not guarantees of true anchor accuracy.

## Validation and time to a useful fit

Three development/validation passes were used. Pass 1 was a fresh full run with
normal automatic force tuning on each stock machine. Pass 2 was another fresh
run with those measured forces explicitly supplied, exercising the stricter
quality guards and force-aware information calculation. Pass 3 replayed the saved
six-sweep data through final selection and RRF application, after the last fix
removed unvalidated consensus averaging. No additional physical sweeps were
collected in pass 3. All six runs completed normally, applied M669, and retired
all owned processes. Physics and the 20 solver iterations were unchanged.

| Machine | Fresh pass 1 total sweeps | First logged refinement fit within 2 mm: sweeps / elapsed wall time | Fresh pass 1 total wall time | Final replay/applied worst coordinate error |
|---|---:|---|---:|---:|
| direct | 6 | 5 / 5.54 min | 6.56 min | 1.16 mm |
| reduced | 6 | 4 / 7.73 min | 10.54 min | 1.92 mm |

Each fresh run started refinement with four checked sweeps, collected two further
sweeps, and stopped after three fit iterations once its last two eligible fits
agreed. Both sensor ranges were recorded from the first sweep. Direct pass 1 had
118 retained points/108 accepted extras; reduced had 120/111. The manual-force
fresh pass retained 119/109 direct and 120/111 reduced.

For comparison, the previous rollout collected five initial rough sweeps and then
remeasured them before refinement. Its first logged fine fits therefore followed
ten collected sweeps, while these runs first logged <=2 mm fits after five direct
or four reduced sweeps. Earlier internal rough fits are not included in this
logged-refinement metric. The old final datasets had 12/13 profiled sweeps in
addition to rough collection. This establishes fewer required collections in
these trajectories, not a controlled statistical wall-time speedup across hosts.

Pass 1 timing includes force tuning; pass 2 does not. Replay timing is excluded
from acquisition claims. Wall times are single observations under concurrent host
load. Pass 1 code preceded the final uncertainty/selection guards; final selection
is established by pass 3, whose data came from the fresh pass 2 runs. Applied
errors are against nominal stock anchors; final direct/reduced absolute errors
are 1.16/1.92 mm and translation-aligned errors are recorded in summary.json.
Hardware robustness beyond these simulations has not been established.

703 fast Python tests passed, with 80 slow/heavy cases deselected; 169 collector
JS tests passed. Focused regressions cover early constant-force sensing, fifth-
range forecast information, fifth-range held-out residuals, frozen offsets,
bootstrap persistence, uncertain low-residual fits, repeated data, and validated
model application. One unrelated owner-crash cleanup test transiently failed in
an earlier suite and passed both in isolation and in the final complete suite;
its production code was left unchanged.

## Manual run and reproduction

Read-only inspection found the second manual attempt had entered the old extra-
sensor phase but stopped on `Browser scene/clock reset during a collector wait`.
Its dataset is marked failed. No user run or browser was controlled or modified.
Use a new dataset path when restarting the updated command. Slack during pullout
is expected; the logs identify the measurement sensors and their load.

Reproduce a fresh run from the repo root with:

```bash
.venv/bin/python autocal/autocal.py --headless-sim --recording-mode none \
  --machine-type hangprinter_5_pulley_reduced --dataset output/new-hp5-check/sweeps.json
```

The added heavy regression is
`pytest -m heavy autocal/tests/test_extra_sensor_headless.py` using `.venv/bin/python -m pytest`.
It requires a `good-fit` stop within six sweeps on each stock machine.
Recompute archived summaries with `.venv/bin/python <this-directory>/summarize.py`.
[Hashes and provenance](manifest.json), [raw summaries](summary.json) and the
archived datasets/iteration logs retain both successful and intermediate results.
