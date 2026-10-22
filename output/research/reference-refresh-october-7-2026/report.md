# October 7 references and research_findings_1 comparison

Created eleven `full_auto_reference_run_october_7_2026.log` references from main at `b60856d9d347c3bcacfd9a5bbf34c44aeda5176b`. Historical references are retained. No solver or runner source was edited. The new reference files are uncommitted. The checkout is now `research_findings_1`, at `4daac40c77276c14d62c86db51abfc2a40292998`.

## Baseline verification

Called the runner’s reference resolver for every DatasetSpec: all eleven select the October 7 logs. An independent full regression run returned exit 0 and `ALL DATASETS: PASS`. Every summary, intermediate parameter tuple, fit score and rank score is unchanged; every ground-truth comparison delta and the aggregate final score is zero. Absolute fit scores and trajectory standard deviations remain nonzero.

[Full baseline verification output](baseline-verification-bounded.log). Each new log includes source revision, dataset SHA-256, Python/dependency versions and originating run paths in a parser-neutral provenance header. The headers were verified to preserve the parsed log data.

## Branch result

The branch lowers the sum of returned ground-truth parameter errors from 885.158 to 816.936 mm: a reduction of 68.223 mm, or 7.71%. Three datasets improve, five worsen and three are unchanged. All intermediate fits and scores are identical, and the intermediate mean-delta sum is exactly zero. Therefore the runner’s final score of -68.223 reflects changed returned-calibration errors in this comparison.

These are sums of Euclidean anchor errors plus 2π-weighted radius errors against the fixture ground truth, not measured end-effector positioning errors.

| Dataset | Main error (mm) | Branch error (mm) | Difference | Selected iteration |
| --- | ---: | ---: | ---: | --- |
| five_points_bigger_deltas | 11.805 | 6.809 | -4.997 | 4 → 1 |
| flexible_lines_2000 | 23.393 | 27.671 | +4.278 | 3 → 2 |
| layered_manual_some_bad_points | 6.129 | 19.974 | +13.846 | 4 → 1 |
| ten_points_bigger_deltas | 18.207 | 18.207 | +0.000 | 3 → 3 |
| ten_points_w_buildup | 4.645 | 8.277 | +3.632 | 3 → 2 |
| ten_points_w_buildup2 | 5.411 | 5.795 | +0.384 | 6 → 1 |
| fifth_hp3_dataset | 70.314 | 70.787 | +0.473 | 3 → 2 |
| seventh_hp3_dataset | 262.236 | 202.974 | -59.262 | 2 → 4 |
| sixteenth_hp3_dataset_even_more_pressure | 52.008 | 52.008 | +0.000 | 3 → 3 |
| nineteenth_hp3_dataset_even_more_pressure | 169.642 | 169.642 | +0.000 | 2 → 2 |
| 23_april_2026 | 261.369 | 234.793 | -26.576 | 3 → 2 |

## What improved

The change is in final history selection. It remembers which whole sweeps trained each candidate, scores its anchors on subsequently available sweeps outside that training set, and makes prediction score the first sorting criterion. Existing history rank is retained as a tie-breaker. Candidates without a prediction score are excluded whenever validated candidates exist. The optimizer and the candidates it generates are unchanged.

- `seventh_hp3_dataset`: chooses iteration 4 instead of 2. Parameter error falls 262.236 → 202.974 mm. Prediction score is 15.61 versus 1241. The error reduction consists of 59.011 mm of summed anchor error and 0.251 mm of weighted radius error.
- `23_april_2026`: chooses iteration 2 instead of 3. Parameter error falls 261.369 → 234.793 mm. Prediction score is 158 versus 206.6. Anchor error improves by 26.827 mm; weighted radius error worsens by 0.251 mm.
- `five_points_bigger_deltas`: chooses iteration 1 instead of 4. Parameter error falls 11.805 → 6.809 mm. Prediction score is 29.32 versus 39.84. Both anchor and radius errors improve.

The largest regression is `layered_manual_some_bad_points`, 6.129 → 19.974 mm; fifth worsens only 70.314 → 70.787 mm. Thus this is a measured net improvement on this battery, rather than a uniform improvement. It also leaves more accurate candidates unselected: seventh’s iteration 3 has 161.500 mm error, and the April dataset has later candidates around 98–104 mm.

One selection limitation visible in these logs is that candidates are compared on different future-sweep sets: seventh’s chosen iteration 4 is scored on one sweep, while iteration 2 is scored on three. The current result does not establish that this validation score reliably identifies the most accurate calibration on every dataset. A common validation set and scoring each candidate with its own full line model would be sensible follow-up experiments; neither was implemented here.

[Full branch comparison output](branch-comparison.log), [machine-readable comparison](comparison.json). The comparison exits 1 because changed returned parameters exceed the reference tolerance; all eleven datasets completed without runtime errors. Both tests in `autocal/tests/test_predictive_validation.py` pass.

## Native runtime failures and reproducible execution

Three full-concurrency baseline verification attempts had native JAX segmentation faults; a temporary GC-disabled attempt also failed with “double free or corruption” during compilation. Those incomplete runs were not accepted as verification. A six-dataset Slideprinter run passed, and the complete two-worker baseline and branch runs both finished without native failures. These observations do not pinpoint the native-library defect or establish that reduced concurrency will always prevent it.

The successful verification and comparison used a temporary startup hook that bounds only the regression runner’s outer ThreadPoolExecutor to two concurrent datasets. The actual calibration subprocesses, optimizer settings, automatic GC, dependencies and fixture contents were unchanged. The hook and failed-run traces are retained here; no runtime patch was added to production code.

```bash
PYTHONPATH="$PWD/output/research/reference-refresh-october-7-2026/runtime-bounded-workers" \
AUTOCAL_REGRESSION_MAX_WORKERS=2 PYTHONFAULTHANDLER=1 \
.venv/bin/python autocal/tools/regress_calibration_logs.py --no-fail-score-mismatch --keep-going
```

[Runtime and source provenance](provenance.json). The bounded scheduling was verified to reproduce all eleven original main reference results exactly.

## New reference files

- [five_points_bigger_deltas.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/five_points_bigger_deltas.full_auto_reference_run_october_7_2026.log)
- [flexible_lines_2000.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/flexible_lines_2000.full_auto_reference_run_october_7_2026.log)
- [layered_manual_some_bad_points.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/layered_manual_some_bad_points.full_auto_reference_run_october_7_2026.log)
- [ten_points_bigger_deltas.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/ten_points_bigger_deltas.full_auto_reference_run_october_7_2026.log)
- [ten_points_w_buildup.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/ten_points_w_buildup.full_auto_reference_run_october_7_2026.log)
- [ten_points_w_buildup2.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/ten_points_w_buildup2.full_auto_reference_run_october_7_2026.log)
- [fifth_hp3_dataset.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/fifth_hp3_dataset.full_auto_reference_run_october_7_2026.log)
- [seventh_hp3_dataset.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/seventh_hp3_dataset.full_auto_reference_run_october_7_2026.log)
- [sixteenth_hp3_dataset_even_more_pressure.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/sixteenth_hp3_dataset_even_more_pressure.full_auto_reference_run_october_7_2026.log)
- [nineteenth_hp3_dataset_even_more_pressure.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/nineteenth_hp3_dataset_even_more_pressure.full_auto_reference_run_october_7_2026.log)
- [23_april_2026.full_auto_reference_run_october_7_2026.log](/home/torbjorn/repos/hp-sim5/autocal/data/references/23_april_2026.full_auto_reference_run_october_7_2026.log)
