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
- Use `--dataset` to choose where the working dataset is stored, or to continue working on a pre-existing dataset.
- Let the loop collect sweeps and stop when you are satisfied with the cost and residuals.

```bash
python autocal/active_calibrate.py \
  --dataset autocal/data/my_active.json
```

For the full details and log interpretation, see `autocal/README_elliptical_feature_calibration.md`.
