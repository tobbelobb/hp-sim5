# hp-sim5 - Hangprinter Simulator

![Demo](slideprinter_Hangprinter_logo_wiggling_and_losing_steps2.gif)

hp-sim5 simulates the physics of [Hangprinter](https://hangprinter.org).
Lines, spools, motors, and firmware are all simulated in great detail.

Simulation makes experimentation fast and cheap.
Structured experiments enable us to improve Hangprinter.

Example usecases:
 - **Automate and Validate Hardware Design**
 - **Digital Twin**
 - **Automate and Validate Software Design**

Try the live apps:
- A family of Slideprinters (2d Hangprinters) is currently deployed here: [hp-sim](https://tobbelobb.github.io/hp-sim5/hp-sim/).
- A flipper game that tests the Cable Physics engine here: [flipper](https://tobbelobb.github.io/hp-sim5/example_apps/js/flipper/index.html).

## Cable Joints and Physics Engine
hp-sim5 includes a Cable Joints library and XPBD physics engine inspired and coded from the work of [Matthias
Müller](https://matthias-research.github.io/pages/index.html).

hp-sim5 includes two implementations of both the physics engine and its Cable Joints library; one JavaScript and one in Python.
They are equivalent so for simplicity we call sometimes refer to them as "the physics engine".

The physics engine is the heart of hp-sim5, and lives in the src directory.
For a deeper dive into the physics engine and the flipper demo see
`README_adv.md`.

hp-sim5 also includes lots of other code (subrepos, or "sub-projects") to make the best use of the simulation within the Hangprinter Project:
 - **klipper (fork)**
 - **ReprapFirmware (fork)**
 - **flex-compensation-dev**
 - **forward-transform-dev**
 - **autocal**

hp-sim5 has added quite a bit to each of its sub-projects:

## Klipper fork
 - Has flex compensation from flex-compensation-dev
 - Has forward transform from forward-transform-dev
 - Much improved support for Hangprinter and other cable robots. On par with ReprapFirmware (except the closed-loop parts, such as autocal and force/torque mode).
 - Learn more in this Klipper pull request: [klipper/pull/7093](https://github.com/Klipper3d/klipper/pull/7093)

## ReprapFirmware
 - Has flex compensation from flex-compensation-dev
 - Has forward transform from forward-transform-dev
 - Has host control mode support similar to Klipper.
 - Learn more in [RRF/README.md](RRF/README.md).

## Flex Compensation Dev
 - Our own repo solely devoted to developing flex compensation for all cable robots.
 - Provides two algorithms for flex compensation, called QP and Tikhonov. Both are implemented in our ReprapFirmware and Klipper forks.
 - Supports any anchor configuration.
 - Learn more in [flex-compensation-dev/hangprinter-flex-compensation/README.md](flex-compensation-dev/hangprinter-flex-compensation/README.md).

## Forward Transform Dev
 - Our own repo devoted to developing forward transforms for all cable robots.
 - Provides state-of-the-art forward transform for a wide range of cable robots.
 - Tested on simulated data for five setups called Slideprinter, Hangprinter v3, Hangprinter v4, CubeCorners, and SkyCam.
 - Implements three approaches:
   * "Pott", based on "On the forward kinematics of cable-driven parallel robots", Pott & Schmidt (2015).
   * "Nice", based on "Kinematics and statics of cable-driven parallel robots by interval-analysis-based methods", Berti (2015)
   * "Quadratic", based on "Fast and Reliable Iterative Cable-Driven Parallel Robot Forward Kinematics: A Quadratic Approximation Approach", by Mahnke & Caverly (2025)
 - The quadratic approach is generally best, and was chosen for ReprapFirmware and Klipper.
 - Learn more in [forward-transform-dev/hangprinter-forward-transform/README.md](forward-transform-dev/hangprinter-forward-transform/README.md).

## Autocal
 - Our own directory for developing automatic calibration.
 - Provides a uniquely user friendly way to find anchor positions using torque/force mode + encoders only.
 - Very high level of abstraction, enabled by
   * Self-tuning of torques/forces to use during calibration
   * Self-calibrating encoder noise
   * Feature based optimizations, exploiting geometric patterns rather than collecting random samples
   * Active learning, actively searching for the best places to collect the next data
   * Outlier-robust filtering on two levels. If a few data points are bad, they will be detected and discarded thanks to its GNC-IRLS style pointwise loss.
 - As a result, it will try very hard to find the anchors, without requiring any human guidance.
 - Compatible with all cable driven robots, real and sim. (Currently only been tested on simulated Slideprinters).
 - Learn more in [autocal/README.md](autocal/README.md) and [autocal/README_elliptical_feature_calibration.md](autocal/README_elliptical_feature_calibration.md).


## Quick Start

### Running Demos Locally
1. Install Node.js. Run for example:
   ```
   nvm install 24
   nvm use 24
   nvm alias default 24
   ```
2. In this repository run:
   ```bash
   npm install        # only needed the first time
   npx vite           # Needed every time to serve the html and js
   ```
3. Open <http://localhost:5173/hp-sim5/hp-sim> in your browser.
   There's also <http://localhost:5173/hp-sim5/flipper> for the flipper demo.
4. Ready to hack away on the js side!


## Python dependencies

Use environment management to install Python dependencies before running Python code. Here's how it might look with Ubuntu + venv:

```
sudo apt install python3-venv
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

# Optional dependency for the warp version of the python version of the physics engine:
python -m pip install -r requirements-warp.txt

# Optional dependency for the jax-driven autocal (5x faster):
python -m pip install -r autocal/requirements-jax-cpu.txt

# Optional dependency for klipper
python -m pip install -r klipper/scripts/klippy-requirements.txt
```



## Tests

These commands cover hp-sim5-specific checks. Subrepos (like RRF, klipper, autocal, etc.) often have their own internal test suites; run those from within each subrepo when you need deeper coverage.

### Run the full local suite (no simulator / no visuals)
```bash
npx jest
python -m pytest
```

Also check out `scripts/run_ci_tests.sh` which helps you run some more types of tests.

### Autocal-only filtering
```bash
python -m pytest autocal/tests
python -m pytest autocal/tests -k autocal
python -m pytest autocal/tests/test_autocal_cli.py
python -m pytest autocal/tests/test_filter_pass_spool_fit.py
```

### Tests of ReprapFirmware's http endpoint
```
tests/run_all_rrf_http_endpoint_tests.sh
```

### Simulator / E2E tests (requires visible browser + rrf_simulator build)
```bash
cmake --build RRF/build --target rrf_simulator -j
node autocal/control/tests/e2e/collect_single_sweep.e2e.test.mjs --sim
node autocal/control/tests/e2e/find_minimum_moving_force.e2e.test.mjs --sim
node autocal/control/tests/e2e/find_edge_force.e2e.test.mjs --sim
node autocal/control/tests/e2e/wait_for_stable_encoders.e2e.test.mjs --sim
node autocal/control/tests/e2e/return_to_origin_one_at_a_time.e2e.test.mjs --sim
node autocal/control/tests/e2e/return_to_origin_all_at_once.e2e.test.mjs --sim
node autocal/control/tests/e2e/calibrate_encoder_noise.e2e.test.mjs --sim
```

### Visual/manual checks
```bash
npx vite
# open tests/html/*.html in the browser for cable_joints 2D/3D visual tests
```

### Determinism scripts
```bash
RRF/tests/run_draw_squares_determinism_test.sh
RRF/tests/run_logo_determinism_test.sh
RRF/tests/run_logo_slideprinter_determinism_test.sh
```

### Autocal e2e tests
There's also a bunch of generated autocal datasets, which you can try to solve against with:
```bash
python autocal/tools/regress_calibration_logs.py --no-fail-score-mismatch --keep-going
```

## hp-sim5 context: the Hangprinter Project
hp-sim5 is part of an effort to automate the Hangprinter Project.
We want to automate everything except the actual users,
and digitize everything except the finished working machines and their output.
