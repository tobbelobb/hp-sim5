# Predeclared paired full autocalibration test: current policy vs measurement-only second sensor

Written before either full run. Session
`session-2026-11-17-run-a-predeclared-paired-full-autocalibration-test-on-hp5-pulley-3e03f53e`.

## Question

On `hangprinter_5_pulley_reduced`, does using the one free low line as a
measurement-only second range — while keeping the production pullout/idle force
policy and the production stopping rules — reduce the final anchor error of a
full autocalibration compared with the current single-active-sensor policy?

This is the integrated-loop question left open by the matched short study
(`session-...-f359cbd6`) and its independent review
(`session-...-748ceb19`): the short study found a large five-range fitting gain
on a saved 4-sweep recording but did not run the actual autocal loop, and the
review required force/offset bookkeeping and a validity gate. The direct-HP5
review (`session-...-b225ae5c`) found no gain on narrow direct sweeps and warned
that pose-dependent slack is the dangerous failure. This experiment is the
single paired reduced-machine full run authorized by the program.

## Arms

| Arm | Collector | Fitter | Extra range |
|---|---|---|---|
| baseline | production `autocal/control/cli/collect_sweep_data.mjs` | production | recorded by encoder but discarded; line at `force_low` |
| treatment | experiment `evidence/extra_range_collector/collect_sweep_data.mjs` | experiment `evidence/extra_range_fit.py` installed in-process | preload `2*force_mid` ≈ 2.06 N, measurement `sensor_collection_force` ≈ 4.99 N, main pullout/idle `force_low` ≈ 0.052 N |

Both arms use the same machine, scene, firmware config, physics binary,
`M666`, force-tuning sequence, initial dataset state (fresh, no prior sweeps),
planner, solver settings, patience and stopping rule.

### Treatment scope

The initial bootstrap (3 auto-size-tuned sweeps), the rough extra sweeps and
`assess_rough_geometry` are **unchanged production**: they use the production
collector and the 4-range fitter. Treatment starts at the equal-strain
re-measurement that creates the refined dataset, and continues for every
planner-requested sweep. This preserves the rough-bootstrap prerequisite and
the tuned forces. The equal-strain force-profile planner is not changed: the
extra line follows the same per-point `sensor_collection_force`; the fitter
models both commanded torques instead of assuming the extra line is passive.

## Predeclared validity gate

A predeclared, anchor-free, production-observable gate decides per sweep whether
the extra range enters the fit. It uses only recorded encoders and commanded
motion; no simulation tension or pose truth is an input.

For each sub-sweep (split at `step_index == 0`):

* If fewer than 4 points or drive travel < 20 mm: `untestable_short`, admitted.
* If sensor travel < 10 mm and extra travel < 0.3 mm: `untestable_insensitive`,
  admitted (the sweep is simply insensitive to the line).
* If extra travel < 0.3 mm: `rejected_frozen_or_slack`.
* If |corr(extra length, drive length)| < 0.5: `rejected_no_coupling`.
* If a cubic fit of extra length against drive has RMS > 0.5 mm or max
  |residual| > 1.0 mm: `rejected_shape_or_slack`.
* Otherwise admitted; extra travel below max(3 mm, 2 % of sensor travel) is
  flagged `accepted_low_information` rather than rejected.

Rejected sweeps keep their four standard ranges and all raw evidence; the
validity record and metrics are written to
`runs/<arm>/sweeps.extra_range_validity.json[l]` on every fit. The gate constants
are frozen here and are recorded in `runs/treatment-launcher.json`.

Preflight validation of the gate:

| Case | Expected | Observed |
|---|---|---|
| saved matched two-sensor recording (`twosensor-1`) | accept 4/4 | accept 4/4 |
| saved frozen current-policy recording (`ctl-1`) | reject 4/4 | reject 4/4 (extra travel 0.0 mm against 270–470 mm sensor travel) |
| saved collapsed-interval 1 N arm (`ctl-low1p0`) | accept, low signal | accept 4/4 (3–8 mm extra travel) |
| synthetic smooth valid range | accept | accept |
| synthetic frozen range | reject | reject |
| synthetic insensitive sweep | accept untestable | accept untestable |
| synthetic single-point 5 mm spike on 10 points | reject | reject |

Known detection limit: a single-point slack spike is only reliably rejected when
the segment has enough points for the cubic not to absorb it (10-point production
segments); a 3 mm spike on a 6-point preflight segment can be absorbed. This is a
limitation, not a tuning target.

## Fitter and bookkeeping

The experiment keeps the production two-stage fit (joint anchor + home-offset
fit, shifted primary fit) and the production `n-d = 2` home-offset gauge on lines
3 and 4. The differences are:

* the extra line's measured length is added only for gate-accepted sweeps;
* the static tension solve includes every line and one known-force row per
  torque-controlled line: the physical sensor from `source_sensor_anchor` and
  the extra line, both at their explicitly recorded commanded forces;
* the firmware compliance inverse uses the existing corrected
  `S * U² * N` model and the solved tension of every line;
* rejected extra ranges still contribute their **commanded force** to the static
  solve, because the physical line is at that force even when its encoder is
  untrusted; only its length equation is dropped;
* planning information uses the same five measured ranges as the fit.

The baseline data has no extra-sensor policy and takes the original production
functions unchanged. A separate parity test fits the saved manual reduced dataset
with production and with the experiment module installed (no policy) and requires
identical anchors and offsets.

## Contract and budget

Both arms:

```bash
HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=6000000 timeout -s INT 1000 \
  .venv/bin/python <autocal.py | evidence/run_treatment.py> \
  --machine-type hangprinter_5_pulley_reduced --dataset runs/<arm>/sweeps.json \
  --headless-sim --recording-mode none --patience 20 --verbose
```

* Fresh dataset path per arm; isolated headless world with free ports.
* `recording-mode none`, patience 20, default `--stable-step-mm 0.5` (two
  consecutive sweeps), default `--max-steps 20`.
* 6,000,000 physics steps and 1000 s external wall limit, unchanged.
* No scene, firmware, physics, force-tuning, planner, stopping-threshold or
  solver change. The treatment's only tracked source edit is a gated collector
  selection for the equal-strain phase; it is applied **after** the baseline run,
  so the baseline runs on pristine source.
* The treatment launcher selects the experiment collector for planner-requested
  sweeps in-process (no production file edit).

A run is a success only if the manifest has `normal_completion: true` and
`stop_reason: stable-estimate`. A timeout, physics-budget exhaustion, bootstrap
failure or non-normal stop is reported as a negative/incomplete result; the
budget is not enlarged and no retry is launched without a new decision.

## Outcomes

Primary: **final applied** anchor error against the nominal firmware `M669`
anchors, as both absolute maximum coordinate/Euclidean error and
translation-aligned maximum coordinate/Euclidean error. Applied values are taken
from the manifest `firmware_final` M669 reply (falling back to the summary M669
command).

Secondary:

* latest selected/estimate anchors (summary `fitted_anchors` and the anchor
  consensus), same two metric families;
* retained sweeps and points, acquisition rejections;
* accepted/rejected/low-information extra ranges and their gate metrics;
* final model residual (`pose_fit rms`) and fit score/cost;
* native physics steps, simulated seconds, collector seconds, wall seconds;
* stop reason, normal completion, last stable steps;
* source, autocal source, scene, firmware config, RRF, physics artifact and
  baked-scene hashes.

Scoring uses nominal `M669` only as a diagnostic. It is never an input to
collection, gating, fitting, planning or stopping.

## Decision rule

Given one deterministic paired realization:

* **Supported improvement**: both arms normal-complete with `stable-estimate`,
  the treatment has at least 80 % of treatment sweeps with an accepted extra
  range, the treatment's final applied aligned max-coordinate error is lower
  than the baseline's, and its final applied absolute max-coordinate error is
  not worse by more than 0.5 mm.
* **Ambiguous / partial**: normal completion but a lower accepted fraction,
  a worse aligned error, or an absolute regression larger than 0.5 mm.
* **Negative**: treatment fails the completion/stopping contract, or its final
  applied error is worse on both metric families.

No magnitude is claimed beyond this one paired run. A null or negative result
is preserved and reported; the short-recording gain is not treated as proof of
full-loop benefit.

## Limitations stated in advance

* Simulation only; no hardware validity. The extra range's absolute scale,
  torque-floor slack and friction hysteresis are not measured on hardware.
* One scene, one machine, one force sequence, one paired run. The simulation is
  deterministic, so the two arms are a matched realization, not a sample.
* The home-offset gauge is production's two offsets; the extra line's own
  constant offset is not always free. The short review found this did not drive
  its gain, but it remains a model limitation.
* The extra 4.99 N torque changes the self-stress force balance; the two-torque
  static solve is the experiment's model of that change, not a hardware-verified
  model. Bearing friction, pulley tangency and changing mover orientation are
  not modeled.
* The validity gate detects frozen/gross-slack responses; sub-millimetre
  pose-dependent slack below the cubic residual floor may pass.
* Nominal `M669` is the simulation's data-generating anchor set; anchor error is
  a valid input-truth comparison in simulation, not a hardware accuracy claim.

## Isolation

The manual `output/first-manual-autocal-of-hp5-pulley-reduced` tree and the other
research program's output directory are read-only. No root source/config change,
shared binary rebuild, existing control port, blanket kill or production default
change. New worlds use free ports and are stopped by their owning process. No
commits or pushes.
