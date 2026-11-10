# Stateful pinhole gates for the experimental HP4

## Objective

Improve the experimental HP4 pinhole-gate model so the condensed ABC reeving
unit has small, tunable entry/exit tension differences from summed bearing
inertia and resistance, without restoring its internal six-joint cable chain.
Render the hidden bearings as rotating wheels while keeping the production
headless Native Rust path cheap.

## Final hypothesis and result

A hidden material-flow coordinate `q`, plus independent unilateral entry and
exit rest allocations, can represent the internal reeving. The final gate law is

```text
Tin - Tout = M qdd + B qdot + Fc sign(qdot)
```

with static friction and no reversal. This was implemented in JS, Native Rust
and Wasm with ABI version 3. It produces nonzero entry/exit differences in an
isolated probe and a short HP4 prefix while retaining 33 cable joints instead of
51. See [report.md](report.md) for measurements and limitations.

## Experiment record

- `PG-01` — `probes/gate_isolated_probe.mjs`: synthetic 1500-step two-sided gate,
  JS and Wasm. Supported static breakaway, inertia and velocity-dependent
  difference, unilateral stopping and backend identity.
  Evidence: `evidence/gate-isolated-js-summed.json`,
  `evidence/gate-isolated-wasm-summed.json`.
- `PG-02` — `probes/gate_runtime_probe.mjs`: 200-step HP4 Print Logo prefix on
  Native, Wasm and JS. Supported backend agreement and nonzero gate force
  asymmetry. Evidence: `evidence/gate-native-summed-200.json`,
  `evidence/gate-wasm-summed-200.json`,
  `evidence/gate-js-summed-200.json`.
- `PG-03` — `probes/gate_perf_probe.mjs` and original-scene replays: matched
  1000-step native performance plus wheel-direction inspection. The single trial
  measured 386.02 ms original vs 146.28 ms gate (`2.64x`) and a 0.0001836 m
  final-Z difference. Original bearings counter-rotate; bottom/top magnitude is
  only provisionally scaled. Evidence: `evidence/gate-summary.json`,
  `evidence/original-wheel-ratio-200.json`,
  `evidence/original-wheel-ratio-1000.json`.
- `PG-04` — `npm run test:physics`: final source state passed 11 of 11 checks.
  Evidence: `evidence/stability-gate.log`. The preceding passing log is
  preserved as `evidence/stability-gate-pre-final-node-bridge.log`.
- A loaded-fixture format migration was required because the path header grew
  from 9 to 16 values. Its original failure is preserved in
  `evidence/stability-gate-pre-checkpoint-migration.log`; settling thresholds
  were not weakened.
- Not tested: isolated numerical-damping reduction, long-run or full-print
  stability, physical calibration, browser visual inspection and Python backend
  support.

## Constraints and budgets

- Used `.venv/bin/python`, headless production backends and recording `none` for
  numerical work; no browser experiments were run.
- Kept source changes focused on the experimental attachment and removed the
  superseded ideal-gate solve.
- Preserved 33 HP4 path joints and retained one gate per ABC line.
- Limited exploratory trials to hundreds or a few thousand steps, with one
  matched performance comparison rather than a large benchmark campaign.
- Ran the mandatory physics gate on the final source state and preserved all
  prior failing evidence.

## Accepted steering

- Use a custom gate with separate entry and exit ports, not sequential
  pinholes or a common-tension ideal pinhole.
- Model summed bearing inertia and small resistance so forces on entering and
  exiting CableJoints differ.
- Expose explicit parameters for later physical tuning.
- Render hidden bearings as cylinders driven by actual material transport, with
  visualization kept separate from the physical solve.

## Next decision

The experimental implementation is ready for review. The next research step is
to obtain measured pulley/bearing inertia, breakaway torque and drag, then use
bounded parameter identification and longer, calibrated comparisons before
promoting the scene from experimental status. A dedicated quasistatic sweep is
also needed before making any claim about reduced numerical damping.
