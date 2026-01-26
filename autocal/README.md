# Autocal

This directory contains the fully automated elliptical feature calibration pipeline. The default workflow uses active learning plus pointwise robust fitting.

## Quick start (simulation)

```bash
python autocal/active_calibrate.py ellipse-loop \
  --residuals-csv /tmp/residuals1.csv \
  --sim \
  --work-dataset autocal/data/test1.json \
  --robust-debug \
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
python autocal/active_calibrate.py ellipse-loop \
  --work-dataset autocal/data/my_active.json \
  --robust-debug \
  --collector-args --return-to-origin
```

## Key commands

- Merge datasets:
  ```bash
  python autocal/active_calibrate.py merge autocal/data/base.json extra.json -o autocal/data/merged.json
  ```
- Plan a single next sweep (no collection):
  ```bash
  python autocal/active_calibrate.py ellipse autocal/data/merged.json --collector-args --return-to-origin
  ```

For the full details and log interpretation, see `autocal/README_elliptical_feature_calibration.md`.
