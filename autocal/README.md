# Autocal

![Autocal's own logo](autocal_logo_shine.jpeg)

This directory contains the fully automated elliptical feature calibration pipeline. The default workflow uses active learning and pointwise robust fitting.

## Python dependencies

The Python dependencies for this `autocal/` subtree are mostly covered by the requirements.txt at the root level.

There's an additional Jax dependency that will cut 80% of computation time if you choose to install it.
You only need jax on the CPU for this, so don't worry about making a full GPU install.

With venv activated at the root level of the hp-sim5 repo, do

```bash
python -m pip install -r autocal/requirements-jax-cpu.txt
```

Quick verification if jax is present:

```bash
python - <<'PYCODE'
import jax
print(jax.__version__)
PYCODE
```


## Quick start (simulation)

Follow the hp-sim5 README.md to get an instance of hp-sim available at <http://localhost:5173/hp-sim5/hp-sim>.

The autocal requires a websocket between it and hp-sim5. Open it by adding `?gcode_ws=ws://localhost:8790` to your url, so open
<http://localhost:5173/hp-sim5/hp-sim/?gcode_ws=ws://localhost:8790>.

If you see this, you're good to go:
![Image of hp-sim app](doc/hp-sim-startscreen.png)

Initiate simulated full-auto calibration with:

```bash
python autocal/autocal.py \
  --sim \
  --machine-type slideprinter \
  --speedup 25
```

`--speedup` is forwarded to the collector automatically; use `--collector-args` only for other raw collector flags.

Replace `slideprinter` with your actual type of machine (one of `slideprinter`,`hangprinter_4`,`hangprinter_5`,`cubecorners`, or `skycam`).
Keep the hp-sim web page visible during the whole procedure, otherwise your browser might pause the simulation and break the autocalibration.

If everything went well you should see something like this:
![Image of autocal step1 finished](doc/hp-sim-after-autocal.png)

You can add `--plot-residual-histogram` to learn about the quality of your data.
It writes `autocal/data/default_dataset.csv` and `autocal/data/default_dataset.png`.

Here's a demo of the autocal loop on a simulated Slideprinter: https://youtu.be/XLmpuAQYbG4


## Typical workflow (real machine)

- Remove `--sim` and any `--speedup` args.
- The current modular entrypoint is `autocal/autocal.py`, which runs the full-auto loop.

```bash
python autocal/autocal.py \
  --machine-type slideprinter
```

More quick tips:
- Use `--dataset` to choose where the working dataset is stored, or to continue working on a pre-existing dataset.
- Let the loop collect sweeps and stop when you are satisfied with the cost and residuals.
- There's also a `--shotgun` flag that makes the autocal loop try harder.


For the full details and log interpretation, see `autocal/README_elliptical_feature_calibration.md`.
