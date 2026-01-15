<query>
Look into the master plan at specs/elliptic_calibration_planning/masterplan_for_implementing_elliptical_feature_calibration.md and also the data colletion sweeps that get invoked by `python autocal/active_calibrate.py ellipse-loop`.

Currently each sweep just uses one "drive" motor and one "sensor" motor, and then collects data points back towards the starting point. This results in what I call a "half sweep".
The data points the optimizer receives comes from only a small sector of the circular path.
It's better to use all the non-fixed motors to take turns to be the "drive" motor while all the others can be "sensor".


You can look at the docs we wrote when we first designed the elliptic calibration:
See eg specs/elliptic_calibration_planning/masterplan_for_implementing_elliptical_feature_calibration.md for an overview of the context around the "sweeps", how the collected data should be used and so on.
Also relevant are ./specs/elliptic_calibration_planning/elliptical_feature_calibration_substep_[1, 2, and 3].md.

It's actually very important to not just do one sweep but to sweep in every allowed direction with a given fixed motor,
and to combine all those sweeps into a single ellipse when fitting ellipses and optimizing over them later.
See specs/elliptic_calibration_planning/conversation_with_Gemini_about_it_15_jan_2026.md for some details on that.

Now, make a minimal change so that a "sweep" isn't just a single drive/sensor pair, but the sequence of all "sub-sweep" that is every allowed set of drive+sensors given 1 fixed motor (in 2d/slideprinter), or 2 fixed motors (in 3d/all non-slideprinter configs).

When you're finished I want to see that
```
node scripts/e2e_test_collect_single_sweep.mjs --sim
```
... makes use of both the available drive/sensor pairs in the single sweep it performs on a simulated slideprinter.

I also want to see that
```
python autocal/active_calibrate.py ellipse-loop --work-dataset autocal/data/test.json --collector-args
```
... also makes use of all drive/sensor paris in its sweeps, and that it combines the collected data in the proper way to make full use of every feature.

Expect things to mostly work already as the ./specs/elliptic_calibration_planning/elliptical_feature_calibration_substep_*.md were implemented carefully.
Some small things have broken after that since we don't have proper tests in place.

I believe `autocal/data/test.bootstrap_cfg.txt` previously had this form:
```
[0] 1 2
[1] 0 2
[2] 0 1
```
... where the square brackets signified the "fixed" motors of the "Constraint motors". Something happened along the way that changed the form and meaning of the config.txt files, but I think we should reintroduce the square brackets and their meaning.

Add unit tests that check if the spec has been fulfilled.

</query>
<answer>
• - Sweep collection now treats each sweep config as a composite: configs are generated with the correct fixed-count (dimensions‑1), canonical drive/sensor ordering that avoids forbidden sensors, duplicates from config files are dropped, and each sweep
    executes all valid sub-sweeps for that anchor pair (roles swapped when allowed). Data points from sub-sweeps are remapped into the canonical pair and aggregated, and drive ranges are derived from the combined samples (scripts/
    sweep_data_collection.mjs, scripts/e2e_test_collect_single_sweep.mjs).
  - Config file handling/output now uses bracketed fixed anchors again (bootstrap files and _write_sweep_config_file), and default fixed-anchor selection in the e2e helper respects the required fixed-count (autocal/active_calibrate.py, autocal/data/
    test.bootstrap_cfg.txt, scripts/e2e_test_collect_single_sweep.mjs).
  - Active-learning candidate generation is aware of carrying/forbidden sensors, keeping sensor roles valid, with a new test covering it (autocal/active_learning.py, autocal/tests/test_active_learning.py).
  - Added Jest coverage to ensure sweep generation/sub-sweep expansion hit both directions and forbid illegal sensors; Jest/Babel configs were converted to ESM so the tests run under the module-based codebase (tests/js/sweep_data_collection.test.js,
    jest.config.js, babel.config.js, scripts/test_collect_sweep_data.mjs).

  Tests run:

  - python -m pytest autocal/tests/test_active_learning.py
  - NODE_OPTIONS=--experimental-vm-modules npm test -- sweep_data_collection.test.js (Jest now needs that env flag for ESM)

  If you want to double-check behavior end-to-end, next run the simulated sweep collector (node scripts/e2e_test_collect_single_sweep.mjs --sim) and the ellipse loop (python autocal/active_calibrate.py ellipse-loop ...) to see the multi-direction sweeps
  in action.
</answer>
