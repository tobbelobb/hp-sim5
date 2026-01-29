# hp-sim5 - Hangprinter Simulator

![Demo](slideprinter_Hangprinter_logo_wiggling_and_losing_steps2.gif)

hp-sim5 simulates the physics of [Hangprinter](https://hangprinter.org).
hp-sim5 currently simulates:

 - How lines behave
 - How spools behave
 - How motors behave
 - etc etc

The goal is to reproduce real world Hangprints as closely as possible.

The `Slideprinter` demo reproduces infills with resonance patterns, ringing after
sharp corners, slack lines along smooth curves and even lost steps from overly
aggressive moves.  All these issues become easier to understand and fix with
this tool.

Try the live demos:

- A 2d Hangprinter called Slideprinter at [tobbelobb.github.io/hp-sim5/hp-sim/](https://tobbelobb.github.io/hp-sim5/hp-sim/)
- A flipper game that tests the Cable Physics engine at [tobbelobb.github.io/hp-sim5/flipper](https://tobbelobb.github.io/hp-sim5/examples/js/flipper/index.html)

Intended uses:
 - **Hardware design** -- reduce guesswork when building new machines.
 - **Digital twin** -- run the simulator before and during prints to optimise
   speed and quality while avoiding catastrophes.
 - **Software design** -- enables rapid firmware development and experiments
   with advanced control and AI.

## Cable Joints and Physics Engine
hp-sim5 includes a Cable Joints library and XPBD physics engine inspired and coded from the work of [Matthias
Müller](https://matthias-research.github.io/pages/index.html).
hp-sim5 includes two fully functional Cable Joints implementations; one in JavaScript and one in Python.

The physics engine is the heart of hp-sim, and lives in the src directory.

For a deeper dive into the physics engine and the flipper demo see
`README_adv.md`.

hp-sim5 also includes lots of other code (subrepos, or "sub-projects") to make the best use of the simulation within the Hangprinter Project:
 - *Klipper fork*, up-to-date Hangprinter compatibility, tested with hp-sim5.
 - *ReprapFirmware fork*, up-to-date Hangprinter compatibility, includes a new host version to enable hp-sim5 compatibility.
 - *flex-compensation-dev*. Our own repo solely devoted to developing flex compensation for all cable robots.
 - *forward-transform-dev*. Our own repo devoted to developing forward transforms for all cable robots.
 - *autocal*. Our own repo for developing automatic calibration.

hp-sim5 has added quite a bit to each of its sub-projects:

## Klipper fork
 - Has flex compensation from flex-compensation-dev
 - Has forward transform from forward-transform-dev
 - Has Hangprinter (and other cable robots) support on par with ReprapFirmware

## ReprapFirmware
 - Has flex compensation from flex-compensation-dev
 - Has forward transform from forward-transform-dev
 - Has host control mode support on par with Klipper.
 - Learn more in RRF/README.md

## Flex Compensation Dev
 - Provides two algorithms for flex compensation, called QP and Tikhonov.
 - Supports any anchor configuration, up to 26 anchors.
 - Tested on simulated data for 3 and 4 anchor configurations.
 - Learn more here in a Klipper pull request I made: [klipper/pull/7093](https://github.com/Klipper3d/klipper/pull/7093)

## Forward Transform Dev
 - Provides state-of-the-art forward transform for a wide range of cable robots.
 - Tested on simulated data for five setups called Slideprinter, Hangprinter v3, Hangprinter v4, CubeCorners, and SkyCam.
 - Implements three approaches:
   * "Pott", based on "On the forward kinematics of cable-driven parallel robots", Pott & Schmidt (2015).
   * "Nice", based on "Kinematics and statics of cable-driven parallel robots by interval-analysis-based methods", Berti (2015)
   * "Quadratic", based on "Fast and Reliable Iterative Cable-Driven Parallel Robot Forward Kinematics: A Quadratic Approximation Approach", by Mahnke & Caverly (2025)
 - The quadratic approach is generally best, and was chosen for ReprapFirmware and Klipper.
 - See detailed comparisons in `Comparison_report_of_the_three_algorithms.md`.

## Autocal
 - Provides a uniquely user friendly way to find anchor positions using torque/force mode + encoders only.
 - Very high level of abstraction, enabled by
   * Self-tuning of torques/forces to use during calibration
   * Self-calibrating encoder noise
   * Feature based optimizations, exploiting geometric patterns rather than collecting random samples
   * Active learning, actively searching for the best places to collect the next data
   * Outlier-robust filtering on two levels. If a few data points are bad, they will be detected and discarded thanks to its GNC-IRLS style pointwise loss.
 - As a result, it will try very hard to find the anchors, without requiring any human guidance.
 - Compatible with all cable driven robots, real and sim. (Currently only been tested on simulated Slideprinters).
 - Learn more in `autocal/README.md` and `autocal/README_elliptical_feature_calibration.md`


## Quick Start

### Running Demos Locally
1. Install Node.js
2. In this repository run:
   ```bash
   npm install        # only needed the first time
   npx vite           # Needed every time to serve the html and js
   ```
3. Open <http://localhost:5173/hp-sim5/hp-sim> in your browser.
   There's also <http://localhost:5173/hp-sim5/flipper> for the flipper demo.
4. Hack away!

## Tests

### Run the full local suite (no simulator / no visuals)
```bash
npx test
python -m pytest
```

Also check out `scripts/run_ci_tests.sh`.

### Autocal-only filtering
```bash
python -m pytest autocal/tests
python -m pytest autocal/tests -k active_calibrate
python -m pytest autocal/tests/test_active_calibrate_golden.py
```

### Simulator / E2E tests (requires visible browser + rrf_simulator build)
```bash
cmake --build RRF/build --target rrf_simulator -j
tests/run_all_e2e_tests.sh
node scripts/e2e_test_collect_single_sweep.mjs --sim
node scripts/e2e_test_find_minimum_moving_force.mjs --sim
node scripts/e2e_test_find_edge_force.mjs --sim
node scripts/e2e_test_wait_for_stable_encoders.mjs --sim
node scripts/e2e_test_return_to_origin_one_at_a_time.mjs --sim
node scripts/e2e_test_return_to_origin_all_at_once.mjs --sim
node scripts/e2e_test_calibrate_encoder_noise.mjs --sim
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

## hp-sim5 context: the Hangprinter Project
hp-sim5 is part of an effort to automate the Hangprinter Project.
We want to automate everything except the actual users,
and digitize everything except the finished working machines and their output.
