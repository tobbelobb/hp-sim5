# Integration review

Reviewed and imported as commit `b56d08e4` on 2026-10-15. The worktree change
was based on `72523b9d`; this checkout also contains the later consensus-based
automatic stopping commits, so the gate was integrated on top of that code.

## Assessment

No evidence of benchmark fabrication or a timed benchmark being manipulated.
The collection-time result uses deterministic simulator step counts, matched
configs and point counts, a repeat run, and two independently measured sweep
counts. The report distinguishes simulated time from host wall time, records a
failed/unusable end-to-end quality comparison, and preserves rejected
hypotheses. The calibration regression uses simulated truth from the harness;
it does not establish hardware accuracy.

The 5 mm gate is a narrow, corpus-supported guard. In the worktree replay it
changed two of 11 datasets, lowering aggregate true-anchor error by 11.62 mm
versus unconditional consensus; the separate stop-chain replay was mixed and
slightly negative. In an integrated replay on this checkout, with the newer
automatic spread stop disabled to match the stored reference run lengths, 9 of
11 datasets were identical and the two changed cases improved by 11.143 mm and
3.994 mm. Aggregate `true_err_total` delta was -15.137 mm. The regression
runner still prints `ONE OR MORE DATASETS: FAIL` because it uses strict
parameter-difference checks; changed anchors can fail that check while having
lower simulated true-anchor error. The complete replay output is in
`artifacts/integration_regression_no_spread_stop.log`.

This is not evidence that 5 mm is a universal threshold: the gate's result is
driven by two wide pairs, and the stop-chain result is mixed. The source comment
therefore calls the threshold provisional. A larger corpus is still needed.

The 0.05 mm fixed-anchor tolerance has a direct, deterministic collection-time
benefit in the recorded simulated A/Bs: 24.05% and 24.25% less simulated time,
about 1.3 seconds per point. However, the three-sweep end-to-end fit comparison
is unusable because the last sweep trajectories differ and three sweeps are
underidentified. The 9-sweep sensitivity study also shows that the fit can move
materially under tiny per-point length changes. Thus collection time is
supported, but equal calibration quality is not established, and no hardware
measurement was made.

An additional integrated replay with the newer automatic spread stop enabled
saved six sweeps compared with the stored references and produced a -14.064 mm
aggregate true-error delta, but it is confounded by that newer stop behavior and
reports a sweep-count mismatch. It is not used to claim a gate effect.

## Verification

- `.venv/bin/python -m pytest autocal/tests/test_history_selection.py` — 12 passed
  after adapting the existing percentage-stop test to request an 8 mm consensus
  explicitly above the new 5 mm gate.
- `npx jest tests/js/sweep_data_collection.test.js --runInBand` — 6 passed.
- `.venv/bin/python autocal/tools/regress_calibration_logs.py` with
  `HP_SIM5_AUTOCAL_SPREAD_STOP_FRAC=off` — all 11 datasets retained their
  reference sweep/point counts; the runner's strict aggregate verdict remains
  `FAIL` due parameter differences, while the simulated true-anchor error sum
  improved by 15.137 mm.

Research artifacts copied from the worktree preserve their relative paths.
Files over 5 MiB, RRD recordings, runtime connection files, and Python cache
files were excluded. The session is listed in `.worktreeinclude` for future
research worktrees.
