# Autocal

![Autocal's own logo](autocal_logo_shine.jpeg)

This directory contains the fully automated elliptical feature calibration pipeline. The default workflow uses active learning and pointwise robust fitting.

## Python dependencies

The Python dependencies for this `autocal/` subtree are covered by the root
`requirements.txt`.

JAX is optional but recommended for the default `--optimizer-mode fast` path.
Autocal explicitly uses JAX on the CPU and falls back to numerical gradients
when JAX is unavailable. Use `--optimizer-mode legacy` to disable the JAX
objective explicitly.

From the root of the hp-sim5 repo, run:

```bash
.venv/bin/python -m pip install -r autocal/requirements-jax-cpu.txt
```

Quick verification if jax is present:

```bash
.venv/bin/python - <<'PYCODE'
import jax
print(jax.__version__)
PYCODE
```


## Quick start (simulation)

Follow the root README to start Vite. For a Slideprinter, open
<http://localhost:5173/hp-sim5/hp-sim/>. For a 3D machine, use
<http://localhost:5173/hp-sim5/hp-sim-3d/>.

Autocal simulation requires a WebSocket connection to the open simulator. Add
`?gcode_ws=ws://localhost:8790` to the selected simulator URL; for example:
<http://localhost:5173/hp-sim5/hp-sim/?gcode_ws=ws://localhost:8790>.

If you see this, you're good to go:
![Image of hp-sim app](doc/hp-sim-startscreen.png)

Initiate simulated full-auto calibration with:

```bash
.venv/bin/python autocal/autocal.py \
  --sim \
  --machine-type slideprinter \
  --speedup 25
```

`--speedup` is forwarded to the collector automatically; use `--collector-args` only for other raw collector flags.

Replace `slideprinter` with your machine type: `slideprinter`, `hangprinter_4`,
`hangprinter_5`, `cubecorners`, or `skycam`. The aliases `hp3`, `hp4`, and
`hangprinter_3` currently normalize to `hangprinter_4`.
Keep the hp-sim web page visible during the whole procedure, otherwise your browser might pause the simulation and break the autocalibration.

If everything went well you should see something like this:
![Image of autocal step1 finished](doc/hp-sim-after-autocal.png)

The default working dataset is `autocal/data/default_dataset.json`. To inspect
residuals with the currently wired output path, add:

```bash
--residuals-csv autocal/data/default_dataset.residuals.csv
```

Then render a histogram with:

```bash
.venv/bin/python autocal/plot_residual_hist.py \
  autocal/data/default_dataset.residuals.csv \
  --output autocal/data/default_dataset.residuals.png
```

Here's a demo of the autocal loop on a simulated Slideprinter: https://youtu.be/XLmpuAQYbG4


## Typical workflow (real machine)

- Remove `--sim` and any `--speedup` args.
- The current modular entrypoint is `autocal/autocal.py`, which runs the full-auto loop.

```bash
.venv/bin/python autocal/autocal.py \
  --machine-type slideprinter
```

More quick tips:
- Use `--dataset` to choose where the working dataset is stored, or to continue working on a pre-existing dataset.
- Use `--firmware klipper` for the Klipper API-mode simulation backend; RRF is the default.
- `--solve-optimizer` accepts `lbfgsb` (default), `lm`, or `trf`.
- Let the loop collect sweeps and stop when you are satisfied with the cost and residuals.
- There's also a `--shotgun` flag that makes the autocal loop try harder.


For full details and log interpretation, see
[`README_elliptical_feature_calibration.md`](README_elliptical_feature_calibration.md).
