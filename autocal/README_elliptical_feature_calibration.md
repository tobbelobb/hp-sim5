# Elliptical feature calibration

`autocal.py` collects sweeps, fits anchors, and chooses informative new sweeps.
It can also fit spool radii and buildup from raw encoder angles.
Final acceptance checks saved models on sweeps absent from their training data.

## Quick start

Set up the simulator connection as described in [README.md](README.md).
Then run:

```bash
.venv/bin/python autocal/autocal.py \
  --sim \
  --machine-type slideprinter \
  --residuals-csv autocal/data/default_dataset.residuals.csv \
  --speedup 25
```

`--speedup` is forwarded to the collector.
Use `--collector-args` for other raw collector flags.
The working dataset defaults to `autocal/data/default_dataset.json`.
Remove `--sim` and `--speedup` for a real machine.
RRF is the default firmware. `--firmware klipper` selects the Klipper API backend.

For fitting and replay without firmware startup or new collection, supply an
existing dataset:

```bash
.venv/bin/python autocal/autocal.py \
  --sim --no-collect \
  --machine-type slideprinter \
  --dataset autocal/data/default_dataset.json
```

`--no-collect` stops before live collection. A missing dataset still triggers
bootstrap. Patience and thresholds can stop replay before all stored sweeps are used.

## Dataset format

A version-2 dataset contains machine metadata, `config`, and `sweeps[]`.
The metadata includes `machine_type`, `num_anchors`, `dimensions`, and `timestamp`.
The config records machine settings, encoder conversion, force tuning, and travel.

Each sweep records:

- `id`, `fixed_anchors`, and `fixed_lengths`
- `drive_anchor` and `sensor_anchor`
- `data_points`, with `l_drive`, `l_sensor`, and collection metadata
- Optional raw encoder angles, noise statistics, drive range, and sweep metadata

`l_drive`, `l_sensor`, and `fixed_lengths` are length deltas from the origin in mm.
The solver reconstructs absolute lengths from each anchor guess.
Noise-mean lengths are used when available unless `--use-raw-lengths` is set.
Swapped drive/sensor sub-sweeps are normalized to the sweep's canonical roles.
Raw angles are required when fitting radii or buildup.

## Collection and active learning

A missing dataset starts with up to three representative sweeps.
The collector tunes force when no force settings override tuning.
It also sizes travel. Later sweeps reuse the recorded force settings.
Collection commands request return to origin.

Existing datasets with more than three sweeps start replay from the first three.
Replay adds one stored sweep per iteration to `<dataset-stem>.replay_tmp.json`.
The original dataset is untouched during replay.
When replay is exhausted, live collection can append to the original dataset.

Every iteration fits the current data again.
It ranks candidate sweeps by D-optimal information gain.
The score is the increase in the information matrix's log determinant.
Higher gain is better. Duplicate and closely spaced candidates are removed.
This planning score is separate from the prediction checks at final acceptance.

`--full-auto-run` adds solver/settings variants for each iteration.
`--shotgun` appends variants from `shotgun.conf`.
The loop selects one valid run per iteration and saves constrained selections in history.

## Anchor and spool fitting

Spool fitting defaults to off.
Without it, each planning pass calls the pointwise anchor solver directly.
With point filtering enabled, the solver runs wide-Huber, tight-Huber, and trim stages.
Sweep filtering can also reject inconsistent sweeps.
By default, each local solver restart freezes sweep rejection at its starting guess.
The trim stage also freezes point rejection there.
The default residual is Sampson distance to the predicted ellipse in squared lengths.
`--pointwise-residual euclidean` selects the alternative distance metric.
Noise normalization uses encoder statistics and the configured noise model.

Enable spool fitting with `--find-radii` or `--find-buildup-factor`.
Each accepts `global` or `per-anchor`; a bare flag means `per-anchor`.
The pipeline transforms raw encoder angles into modeled length deltas.
It alternates spool steps with anchor proposals and can polish their shared scale.
Spool priors regularize radii and buildup.

The default spool schedules are:

```text
--filter-schedule warmup,warmup,warmup,dynamic
--objective-schedule 1,1,1,1
```

In this schedule, warmup passes disable point and sweep filtering.
The dynamic pass enables the requested filters and builds a reusable mask.
A later `constant` pass can reuse that mask with runtime filtering disabled.
Passes warm-start from the previous pass's fitted parameters.

The objective schedule controls spool steps and anchor proposals.
`0` selects ellipse prefit/comparison, `1` selects pointwise forward modeling,
and `2` selects position-reconstruction spread.
The final anchor solve after each pass still uses the pointwise solver.
Acceptance inside refinement compares spool rank first, then data cost plus priors.

`--solve-optimizer` accepts `lbfgsb` (default), `lm`, and `trf`.
`--optimizer-mode fast` uses CPU JAX derivatives when available.
It falls back to numerical gradients when JAX is unavailable.
`fast-fd` uses the JAX value with finite differences. `legacy` disables JAX.
`--flex` enables cable stretch correction. It is off by default in this CLI.

If filtering leaves too few constraints, full-auto retries with sweep filtering off,
then with point filtering off. Recovery uses the current data and warm-start seeds.
`--sparse-recovery` also fits an endpoint-preserving sparse subset.
That result seeds a full-data fit. It is never accepted directly.
Invalid or degenerate recovery geometries are rejected.
If recovery fails, the loop continues toward more data.

## Stopping and final selection

`history_rank_score` controls "The best try so far" and patience.
It combines the plan's `rank_score` with iteration and retained-data adjustments.
The default patience is three non-improving iterations.
`--stop-cost` and `--stop-std-mm` add optional stop thresholds.
Ctrl-C during the loop or a `<dataset-stem>.full_auto.stop` file requests acceptance.

At acceptance, each saved model predicts whole sweeps absent from its training IDs.
The check uses the latest saved history plan's datasets.
It keeps anchors and spool parameters frozen and turns filtering off.
A robust loss scores every valid observation. Invalid or incomplete predictions fail.
Sweeps removed during training still count as training sweeps.
Unreplayed sweeps are not used.

Final selection uses two prediction scores:

- `heldout_prediction` evaluates raw data with each candidate's own spool and
  measurement-model settings. A finite value is required when validation is available.
- `anchor_comparison` evaluates anchors on the latest plan's transformed dataset.
  Candidates share that length model and the validation helper's default settings.
  This is the first ranking key among candidates that pass validation.

Then selection compares own-model prediction, history rank, raw rank,
relative uncertainty, primary cost, and iteration recency.
The latest model usually has no held-out sweeps.
If older models predict successfully, they take priority over that unvalidated model.
If all attempted predictions fail, no calibration is applied.
If no held-out data exists, selection falls back to history ranking.

The summary prints M669 anchors and fitted M666 spool settings.
It sends M669 to RRF, but does not send M666.
Klipper skips M669. `--sim --no-collect` skips sending unless a server was supplied.
Reaching `--max-steps` ends the loop without final selection or calibration send.
The default limit is 20 iterations.

## Fit quality scores

`raw_fit_score_ui` describes the current plan.
`rank_score` orders run variants in the current iteration.
`history_rank_score` controls patience across iterations.
In layered spool fitting, displayed `fit_score_ui` is mapped from history rank.
Otherwise, it equals the raw plan score.
Neither displayed score is derived from held-out prediction costs.

The displayed bands are below `2` (ideal), `2` to below `5` (good),
`5` to below `10` (usable), and `10` or above (concerning).
These bands describe fit quality. They do not prove anchor accuracy.
Prediction candidates can also have different held-out sweep sets.

The calibration-log regression tool reports score/ground-truth disagreements
and selection regret separately from reference-log comparisons.
See [objective_functions_overview](objective_functions_overview) for score details.

## Residuals and logs

For anchor-only fitting, `--residuals-csv` writes per-point diagnostics.
`residual_mm` is an approximate mm residual from local linearization.
`cutoff_mm` records the trim threshold.
Render a histogram separately:

```bash
.venv/bin/python autocal/plot_residual_hist.py \
  autocal/data/default_dataset.residuals.csv \
  --output autocal/data/default_dataset.residuals.png
```

The normal multi-pass spool path reuses its final calibration.
Those per-pass solves currently receive neither the requested CSV path nor report flag.
So `--residuals-csv` and `--report` are skipped on that path.
The legacy `--plot-residual-histogram` flag is also unwired in full-auto.

Detailed output goes to `<dataset-stem>.full_auto.log`.
Machine-readable iteration events go to `<dataset-stem>.full_auto_log.jsonl`.
Existing log files get a unique suffix.
`--verbose` adds anchors and score details to the console.

| Log field or prefix | Meaning |
| --- | --- |
| `bootstrapping dataset` | Initial collection for a missing dataset |
| `full-auto replay` | Incremental replay of stored sweeps |
| `[gnc]`, `[robust]` | Pointwise stages and filtering diagnostics |
| `line_model_fit` | Spool/anchor refinement summary |
| `line_model_filter_schedule` | Filter pass order and results |
| `underconstrained_recovery` | Retries that restore constraints |
| `next_sweep score=...` | Information gain for a proposed movement |
| `selected run=...` | Current iteration's selected variant and fit scores |
| `history_select` | Final history ranking, including both prediction scores |
| `history_validate ... failed_prediction=True` | A candidate failed own-model validation |

## Manual utilities

Plan one sweep without collecting it:

```bash
.venv/bin/python autocal/ellipse_active.py autocal/data/merged.json \
  --collector-args --return-to-origin
```

Merge datasets:

```bash
.venv/bin/python autocal/merge_sweep_datasets.py \
  autocal/data/base.json extra.json -o autocal/data/merged.json
```

`tools/regress_calibration_logs.py` compares runs with stored reference logs.
`tools/generate_reference_runs.py` generates dated reference sets from isolated
dataset copies. It records revision, dataset hash, interpreter, and dependencies.
It publishes the set only after all selected runs succeed.

See `autocal/autocal.py --help` for CLI options.
The [algorithm overview](optimization_algorithm_overview) describes each loop
and maps it to the current modules.
