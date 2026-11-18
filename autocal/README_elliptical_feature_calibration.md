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
Spool fitting turns off home-offset fitting and needs constant-force data;
joint spool/offset and profiled-spool fitting are not implemented.
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
The optional `--trf-solver-fallback` retries unsuccessful or invalid `lbfgsb`
restarts with bounded `trf`, including restarts that reach their iteration limit.
It is off by default. `--no-trf-solver-fallback` explicitly disables it.
Choosing `--solve-optimizer trf` runs `trf` directly.
`--optimizer-mode fast` uses CPU JAX derivatives when available.
It falls back to numerical gradients when JAX is unavailable.
`fast-fd` uses the JAX value with finite differences. `legacy` disables JAX.
Ellipse calibration no longer accepts `--flex` or `--spring-k-multiplier`.
The collector does not record the per-point tensions that correction required.
The legacy `calibrate.py point` solver retains its separate elasticity model.

Zero-tension extrapolation was retired from collection after failing the stretch
comparison. New HP3/HP4 collection uses equal-strain profiles and home-offset
fitting. Historical projected datasets remain readable with `--no-collect`;
they require `raw_angles_zero_tension_deg` on every point, use projected lengths
rather than tensioned noise means, and skip home-offset fitting.

If filtering leaves too few constraints, full-auto retries with sweep filtering off,
then with point filtering off. Recovery uses the current data and warm-start seeds.
`--sparse-recovery` also fits an endpoint-preserving sparse subset.
That result seeds a full-data fit. It is never accepted directly.
Invalid or degenerate recovery geometries are rejected.
If recovery fails, the loop continues toward more data.

## Stopping and final selection

Patience counts iterations without a new lowest anchor uncertainty.
The default patience is three.
Collection also stops once two consecutive sweeps each move every anchor
coordinate by at most `--stable-step-mm` (0.5 mm). Stability does not establish
accuracy. Cost/std stopping thresholds have been removed because
their scales were not calibrated to physical error. The manifest reason for
patience acceptance is `patience`.
Ctrl-C during the loop or a `<dataset-stem>.full_auto.stop` file requests acceptance.

At acceptance, the newest admissible fit is applied. It uses all the data.
With `--find-radii` or `--find-buildup-factor`, adding sweeps does not reliably improve the fit, so those runs instead score every saved fit, frozen, on the sweeps that arrived after it, and apply the consensus of the best-predicting fits (`history_selection.py`). A failed or diverged jackknife (above 3 mm and 10 times the lowest so far) is never applied.

The summary prints M669 anchors and fitted M666 spool settings.
It sends M669 to RRF, but does not send M666.
Klipper skips M669. `--sim --no-collect` skips sending unless a server was supplied.
Reaching `--max-steps` applies the fit acceptance would apply.
The default limit is 20 iterations.

## Anchor uncertainty

Each iteration reports `anchor_uncertainty_mm`, the largest anchor-coordinate
standard deviation of point fits that each leave out one sweep. The summary labels
up to 1 mm good, up to 3 mm usable and anything above poor. Patience counts sweeps
without a new lowest uncertainty. Fit cost only picks between variants that fit the
same data. The uncertainty misses bias that every sweep shares; see
`anchor_uncertainty.py`.

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
The unused `--plot-residual-histogram` option has been removed. Use the standalone
histogram command above with a residual CSV.

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
| `selected run=...` | Current iteration's selected variant, its cost and anchor uncertainty |

Full-auto applies the newest admissible fit, or with spool fitting the held-out
consensus. Its anchor uncertainty is the spread
of point fits that each leave out one sweep; it misses bias shared by all sweeps.

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
