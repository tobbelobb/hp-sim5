# HP4 effector rocking: separate the firmware tick from physics integration

Status: complete for the reproduced vibration; original full-autocal retry remains untested.

## Abstract

The production pipeline used the 2 ms firmware tick as one mechanical integration step. Loaded HP4 states with stiff cables and light rotating members can sustain a numerical rocking cycle at that step size. An offline replay of the supplied recording produced a stressed checkpoint whose corner moved approximately 16.05 mm peak to peak even after all motors returned to position mode. Four internal 0.5 ms physics steps settled that same checkpoint in JavaScript, native Rust and browser Wasm, with angular velocity reaching zero. Command dispatch, extrusion and recording retain their 2 ms cadence. A separate cable-damping sign error allowed compressive corrections; clamping it fixed a simpler trial but did not fix the full stress case alone. A small-angle quaternion cancellation error in rotational cable damping was also corrected across backends. No autocal settling/acceptance thresholds changed. The original recording also shows substantial slow travel during size tuning, so successful full-autocal collection is not established. This is simulated stability evidence, not calibration-accuracy or hardware validation.

## Topics

HP4, effector corner vibration, rigid body, numerical stability, physics substeps, firmware cadence, unilateral cable damping, Rust, Wasm, autocalibration.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| One 2 ms integration step sustains loaded-state rocking | E05/E06 | Supported | [Report](report.md) |
| Four 0.5 ms internal steps settle the same state | E07 | Supported | [Measurements](stress-summary.json) |
| Cable damping sign correction alone is sufficient | E05 | Rejected | [Report](report.md) |
| Original full-autocal succeeds after correction | None | Not tested | [Limits](report.md) |

## References

- [Report](report.md): causes, implementation, experiments and limitations.
- [Notebook](research.md): accepted steering and budgets.
- [Numerical results](stress-summary.json) and [plot](stress-stability.png): final before/after comparison.


Follow-up: default-four integration was rejected for performance and restored to one. The original browser command still fails at one step. See [single-step follow-up](../2026-10-16-hp4-one-step-autocal/report.md).
