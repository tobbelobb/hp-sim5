# Predeclared paired full autocalibration test on HP5 pulley-reduced: partial delivery

Investigation: `session-2026-11-17-run-a-predeclared-paired-full-autocalibration-test-on-hp5-pulley-3e03f53e`
Program: `program-2026-11-17-compare-single-sensor-versus-all-non-fixed-non-driving-sensor-po-796a850f`
Evidence: **native-headless simulation and saved-data replay only**. No hardware validity. The paired full run is **not complete**: the baseline was interrupted by the hard deadline during bootstrap and the treatment was not launched. This report distinguishes complete preflight/code validation, the interrupted baseline, and the unrun treatment.

## Bottom line

* The experiment-only integrated prototype is **built, tested and validated end to end on short and saved data**: collector phase sequence, explicit extra-sensor force/axis metadata, gate, five-range fit, two-torque static solve, offset shift and production-baseline parity.
* A fresh short treatment run reproduced the reviewed two-sensor recording exactly (same poses and gate metrics): the gate accepted 4/4 sweeps and the five-range fit gave **2.62 mm** translation-aligned max anchor error versus **5.45 mm** for the same data with the extra range discarded (production two-stage offset gauge; rigid short-data path). The saved matched recording gives the reviewed 2.62 mm with this production-structure fitter (the review's joint-offset variant: 1.85 mm).
* The known frozen current-policy recording is **rejected 4/4** by the predeclared gate; the collapsed 1 N arm is **admitted** as low-signal, and a synthetic insensitive sweep is admitted rather than rejected. A synthetic 5 mm single-point spike is rejected.
* The **full baseline is interrupted** after 116 s / 494,955 physics steps / 989.91 simulated s, during the initial bootstrap. It has no dataset, no applied anchors, `normal_completion: false`, `stop_reason: bootstrap-interrupted`. It is **not** a baseline result.
* The **full treatment is unrun**. No paired full-run comparison exists, so the empirical question "is the full pipeline faster or more accurate?" remains **open**. The favorable short-record evidence is not promoted to a full-loop claim.

## Why the full runs stopped

The manager set a hard deadline of 2026-11-17 13:58:22 UTC, about ten minutes after the baseline was launched. The prior reduced production full run (documented in `tests/benchmark/hp5-validation.md`) took 632.74 s wall and the treatment adds roughly 7 % simulated collection time. The baseline could not fit the remaining wall budget, and the instruction was not to start a run that cannot fit. I sent SIGINT to the owned `timeout`, which let autocal stop its own RRF/physics/collector children and write the interrupted manifest. No retry, no budget enlargement, no treatment launch.

## What is complete

### 1. Experiment-only integrated prototype

Files (all outside root production):

* [extra_range_fit.py](evidence/extra_range_fit.py) — runtime fitter integration. It monkey-patches only the treatment process, keeps the production two-stage offset fit, adds the accepted extra range to `dataset_line_lengths`, generalises the static solve to one known-force row per torque-controlled line (physical sensor from `source_sensor_anchor`, plus the extra line at its explicit commanded force), extends the home-offset shift to the extra encoder, and writes validity evidence beside the dataset.
* [build_extra_range_collector.py](evidence/build_extra_range_collector.py) and [extra_range_collector/](evidence/extra_range_collector/) — experiment collector. Preload `2*force_mid` (≈2.06 N) and measurement `sensor_collection_force` (≈4.99 N) include the one remaining free low axis; main pullout and idle stay at `force_low` (≈0.052 N). Every point records `extra_sensor_anchor`, `extra_sensor_force_n`, `extra_sensor_force_mode`; every sweep records `extra_sensor_anchors` and `extra_sensor_policy`. There is no force-mode inference from a path or file name.
* [run_treatment.py](evidence/run_treatment.py) — installs the fit module and selects the experiment collector for planner-requested sweeps in-process.
* [autocal_treatment.patch](evidence/autocal_treatment.patch) — the one tracked experiment edit, prepared and `git apply --check`-verified against pristine `c2450975`: an optional `collector` argument for the equal-strain re-measurement, selected by `HP5_EXTRA_RANGE_COLLECTOR`. It is **not applied** in the worktree now. The initial bootstrap, rough extra sweeps and `assess_rough_geometry` always use production.
* [collect_short.py](evidence/collect_short.py), [run_pair.py](evidence/run_pair.py), [analyze_pair.py](evidence/analyze_pair.py) — isolated collection, arm driver and paired scorer.
* [predeclaration.md](predeclaration.md) — question, arms, scope, gate constants, contract, outcomes, decision rule and limitations, written before the full runs.
* [tests/test_extra_range.py](tests/test_extra_range.py) — **6 passed**.

### 2. Preflight physics (fresh private worlds)

| Run | Collector | Result |
|---|---|---|
| [preflight/baseline-short](preflight/baseline-short) | production | 1 sweep, 12 points, 270.77 simulated s, 135,385 steps, 33.4 s wall; no extra policy/metadata |
| [preflight/treatment-short](preflight/treatment-short) | experiment | 4 sweeps, 48 points, 1,017.34 simulated s, 508,670 steps, 124.4 s wall; all four extra axes (2, 3, 0, 1) recorded at 4.9914 N |

Short treatment gate: **4/4 accepted**, segment extra travel 78.6–139.3 mm against 269.7–470.9 mm sensor travel, |correlation| 0.946–0.963, cubic residual max 0.024–0.397 mm. Short treatment fit (test hook for constant-force data): five-range **2.62 mm** aligned / 3.43 mm absolute max coordinate, rms 0.024 mm; four-range **5.45 mm** aligned / 5.93 mm absolute, rms 0.029 mm. Diagnostics only; no nominal anchors enter collection, gating, fitting or stopping.

### 3. Saved-data regressions

| Check | Result |
|---|---|
| Baseline parity on saved manual reduced full dataset (production vs patched, no policy) | anchors and offsets **identical (max diff 0.0)**; same rms/cost |
| Baseline parity on the fresh short baseline dataset | anchors and offsets **identical (max diff 0.0)** |
| `twosensor-1` saved matched recording | gate **accept 4/4**; five-range **2.62 mm** aligned, rms 0.024 mm; four-range 22.0 mm in the independent review with its stretch/start model |
| `ctl-1` saved current-policy frozen recording | gate **reject 4/4**, extra travel 0.0 mm |
| `ctl-low1p0` saved collapsed 1 N arm | gate **accept 4/4**, 3–8 mm extra travel (low signal) |
| Synthetic smooth valid / frozen / insensitive / 5 mm spike | accept / reject / accept-untestable / reject |
| Force-role and offset tests | explicit `extra_sensor_force_n` wins regardless of a `prototype-control` path; extra angle shift is `offset / mm_per_degree` |

Known detection limit: a 3 mm single-point spike on a 6-point preflight segment can be absorbed by the cubic; 10-point production segments and the 5 mm synthetic spike are rejected. Sub-millimetre pose-dependent slack below the cubic floor may pass.

## Interrupted baseline record

[preflight-free full-run record](runs/baseline/run.json), [manifest](runs/baseline/sweeps.headless/manifest.json), [run log](runs/baseline/run.log), [full-auto log](runs/baseline/sweeps.full_auto.log).

* Status `interrupted`, `stop_reason: bootstrap-interrupted`, `normal_completion: false`.
* 116 s wall, 494,955 physics steps, 989.91 simulated s.
* No `sweeps.json` was written; no calibration was fitted or applied.
* Pristine source hash `a99803c4…` and git `c2450975d7a3dcf1e6bd90d60afc1e7c3acc51f6`; no treatment patch applied.
* All owned RRF/physics/collector processes were stopped by their owning `HeadlessSimulation`; no listeners or processes remain.

## Unrun treatment

[Treatment record](runs/treatment/run.json) — `unrun: true`. No treatment dataset, manifest, gate record or anchors exist. Do not read the interrupted baseline as a paired result.

## Handoff: how to complete the pair

1. Run the baseline on pristine source:
   ```bash
   HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=6000000 timeout -s INT 1000 \
     .venv/bin/python autocal/autocal.py --machine-type hangprinter_5_pulley_reduced \
     --dataset <session>/runs/baseline/sweeps.json --headless-sim --recording-mode none \
     --patience 20 --verbose
   ```
   Use a fresh path; the interrupted path already has bootstrap state and must not be reused.
2. After the baseline completes, apply [autocal_treatment.patch](evidence/autocal_treatment.patch) and run:
   ```bash
   HP5_EXTRA_RANGE_COLLECTOR=<session>/evidence/extra_range_collector/collect_sweep_data.mjs \
   HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=6000000 timeout -s INT 1000 \
     .venv/bin/python <session>/evidence/run_treatment.py --machine-type hangprinter_5_pulley_reduced \
     --dataset <session>/runs/treatment/sweeps.json --headless-sim --recording-mode none \
     --patience 20 --verbose
   ```
3. Score with `evidence/analyze_pair.py`; require `normal_completion: true` and `stop_reason: stable-estimate` in both arms. Do not enlarge the 6,000,000-step or 1000 s limits. Expect roughly 11 min for a reduced baseline, and slightly more with the extra measurement; both must fit the wall budget before starting.

## Limitations

* Simulation only; no hardware evidence. Extra-range absolute scale, torque-floor slack and friction hysteresis are unmeasured on hardware.
* The two full runs did not complete, so there is **no** full-loop accuracy, time, sweep-count or stopping comparison. Nothing here establishes that the integrated pipeline is faster or more accurate.
* The short treatment fit uses a constant-force test hook; the full run's equal-strain path is validated by the general solver's exact baseline parity and the saved-data fit, not by a completed full run.
* The production two-stage offset gauge frees two offsets; the extra line's constant offset is not always free. The short review found this did not drive its gain, but it remains a model limitation.
* The extra torque changes the self-stress force balance; the two-torque static solve is the experiment's model, not a hardware-verified model. Bearing friction, tangency and mover orientation are not modeled.
* Nominal `M669` is the simulation's data-generating set; anchor error is an input-truth diagnostic, not hardware accuracy.

## Isolation and cleanup

No root source, config, scene or shared binary was edited; the worktree is clean except this untracked session directory and the read-only build symlinks. The manual `output/first-manual-autocal-of-hp5-pulley-reduced` tree and the other research program's output were never modified. No existing control port was touched, no blanket kill was issued, no commit or push was made. The interrupted baseline's own child processes were stopped through its `HeadlessSimulation`; `ps` shows none remaining.
