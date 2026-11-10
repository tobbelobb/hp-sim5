# Stateful pinhole gates for the experimental HP4

Status: complete; experimental model delivered with measurement gaps.

## Abstract

This study replaced the ideal, massless ABC reeving abstraction in the
experimental HP4 scene with six stateful two-port pinhole gates. Each gate has
one hidden cable-flow state, summed bearing inertia, viscous damping and dry
friction, while retaining distinct unilateral entry and exit constraints and no
internal reeving CableJoints. Native Rust, Wasm and JavaScript were kept on the
same state law, and the twelve hidden bearings were rendered as cylinders driven
by material transport. The model reduced cable joints from 51 to 33. An isolated
probe produced a nonzero static breakaway and up to 0.0157 N entry/exit
difference; a 200-step HP4 prefix produced up to 0.0781 N. A matched 1000-step
native trial was 2.64 times faster with a 0.000184 m final-Z difference. The
final mandatory physics gate passed all 11 checks. All evidence is simulated;
the friction parameters are not hardware-calibrated, reduced numerical damping
was not isolated, and long-run accuracy and browser appearance were not
established.

## Topics

HP4, Hangprinter, CableJoints, pinhole gate, reeving, bearing inertia, dry
friction, cable transport, Native Rust, Wasm, visualization, numerical damping

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Summed bearing inertia and resistance make entry and exit tensions differ while retaining static friction and unilateral stopping | PG-01, PG-02 | Supported in isolated and short HP4 trials | [Evidence summary](evidence/gate-summary.json), [isolated JS](evidence/gate-isolated-js-summed.json), [HP4 native](evidence/gate-native-summed-200.json) |
| JS, Native Rust and Wasm produce the same gate response | PG-01, PG-02, PG-04 | Supported for the checked state law and production prefix | [Wasm isolated](evidence/gate-isolated-wasm-summed.json), [HP4 Wasm](evidence/gate-wasm-summed-200.json), [final gate](evidence/stability-gate.log) |
| The condensed topology improves runtime while staying near the original short trajectory | PG-03 | Supported only for one matched 1000-step native trial | [Evidence summary](evidence/gate-summary.json), [performance probe](probes/gate_perf_probe.mjs) |
| Hidden bearings can be rendered as cylinders whose rotation follows material transport and original counter-rotation | PG-03 | Mixed: structure and signs checked headlessly; ratio is approximate and browser appearance was not inspected | [Wheel ratios](evidence/original-wheel-ratio-1000.json), [scene generator](../../../../scripts/create_hp4_gate_scene.mjs) |
| The gate reduces numerical damping | None | Not tested: short trajectories do not isolate numerical damping | [Full report](report.md) |
| The synthetic gate parameters reproduce a physical HP4 | None | Not tested: no hardware measurements or calibration were available | [Full report](report.md) |

## References

- [Full report](report.md): implementation, measurements and limitations.
- [Research notebook](research.md): experiment decisions and accepted steering.
- [Müller et al. (2018), Cable Joints](https://matthias-research.github.io/pages/publications/cableJoints.pdf):
  condensed cable constraints, unilateral distance limits, dynamic rest length
  and pinholes.
- [Prior gate investigation](../2026-10-20-interactive-research-62300644/report.md):
  motivation, failed simple bridge and ideal-gate baseline.
- Baseline revision `b9983100`; final evidence was collected from the
  uncommitted stateful-gate worktree.
