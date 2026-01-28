# Autocal

This directory contains the fully automated elliptical feature calibration pipeline. The default workflow uses active learning plus pointwise robust fitting.

## Quick start (simulation)

```bash
python autocal/active_calibrate.py \
  --residuals-csv /tmp/residuals1.csv \
  --sim \
  --collector-args --speedup 25
```

Plot the residual histogram from the first iteration:

```bash
python scripts/plot_residual_hist.py /tmp/residuals1_001.csv --output /tmp/residuals1_001.png
```

## Typical workflow (real machine)

- Remove `--sim` and any `--speedup` args.
- Keep `--work-dataset` pointing at your calibration dataset JSON.
- Let the loop collect sweeps and stop when you are satisfied with the cost and residuals.

```bash
python autocal/active_calibrate.py \
  autocal/data/my_active.json
```

You can specify an old, or a new explicitly named dataset with ???.
See `python autocal/active_calibrate.py --help` for more options.

For the full details and log interpretation, see `autocal/README_elliptical_feature_calibration.md`.
