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

## Result

The experimental scene now contains six condensed ABC units instead of the
original pinhole-wheel-wheel-pinhole CableJoint sequence. Each unit exposes
separate entry and exit ports. Its hidden flow coordinate `q` obeys

```text
Tin - Tout = M qdd + B qdot + Fc sign(qdot)
```

with static friction and non-reversal. Entry and exit remain unilateral, so the
gate can still produce slack. The flow state is frozen during the position
solve and advanced from the final side multipliers, preserving the existing
shallow solver structure rather than adding a nested iterative solve.

Effective inertia is derived from the two original bearing tensors. Viscous and
Coulomb resistance use explicit synthetic placeholders: `B =
0.9467455621301777 Ns/m` and total dry force `4e-5 N` at radius `6.5 mm`.
JS, native Rust and Wasm share the state law; the native/Wasm ABI is version 3.

The visual gate route is independent of the solve. Hidden wheels are cylinders,
and their angle comes from actual cable stored-length transport rather than an
arbitrary clock. Original HP4 replay showed counter-rotating bearings, so the
bottom visual scale is negative. Its magnitude remains approximate because the
measured bottom/top ratio varied by branch and duration.

## Measurements

| Check | Result | Scope |
| --- | --- | --- |
| Isolated summed-friction probe | breakaway at step 100; max difference 0.015716878 N; mean absolute 0.003840474 N; steady sliding 0.006630394 N; final static offset -0.003097137 N | Synthetic 1500-step gate; JS and Wasm identical |
| HP4 Print Logo prefix, 200 steps | max gate difference 0.078113990 N; mean absolute 0.008975926 N; max `abs(q)` 4.827871e-5 m | Native, Wasm and JS agree on sampled state |
| Transport, 200 steps | AL/AR ~19.29 mm, BL/BR ~8.66 mm, CL/CR ~31.29 mm | Actual stored-length change through gates |
| Matched native runtime, 1000 steps, recording `none` | 386.02 ms original vs 146.28 ms gate; 2.64x; final Z difference 0.0001836 m | Single matched short trial; 51 vs 33 joints |
| Mandatory physics gate | 11 passed, 0 failed | Final source state; includes settling at 1/2/4 substeps |

Evidence is indexed by [gate-summary.json](evidence/gate-summary.json), with the
final gate in [stability-gate.log](evidence/stability-gate.log). The earlier
passing log from before the last bridge guard is preserved separately as
[stability-gate-pre-final-node-bridge.log](evidence/stability-gate-pre-final-node-bridge.log).

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Summed bearing inertia and resistance make entry and exit tensions differ while retaining static friction and unilateral stopping | PG-01, PG-02 | Supported in isolated and short HP4 trials | [Summary](evidence/gate-summary.json), [isolated JS](evidence/gate-isolated-js-summed.json), [HP4 native](evidence/gate-native-summed-200.json) |
| JS, Native Rust and Wasm produce the same gate response | PG-01, PG-02, PG-04 | Supported for the checked state law and production prefix | [Isolated Wasm](evidence/gate-isolated-wasm-summed.json), [HP4 Wasm](evidence/gate-wasm-summed-200.json), [final gate](evidence/stability-gate.log) |
| The condensed topology improves runtime while staying near the original short trajectory | PG-03 | Supported only for one matched 1000-step native trial | [Summary](evidence/gate-summary.json), [probe](probes/gate_perf_probe.mjs) |
| Hidden bearings can be rendered as cylinders whose rotation follows material transport and original counter-rotation | PG-03 | Mixed: structure and signs checked headlessly; ratio is approximate and browser appearance was not inspected | [Wheel ratios](evidence/original-wheel-ratio-1000.json), [scene generator](../../../../scripts/create_hp4_gate_scene.mjs) |
| The gate reduces numerical damping | None | Not tested: short trajectories do not isolate numerical damping | This report |
| The synthetic gate parameters reproduce a physical HP4 | None | Not tested: no hardware measurements or calibration were available | This report |

## Limitations

- All results are simulated or synthetic. No gate parameter has been fitted to
  physical HP4 measurements.
- The wheel animation ratio is provisional. Headless checks prove cylinder
  geometry, material-transport dependence and counter-rotation signs, not the
  final browser appearance.
- The `2.64x` speed result is one matched 1000-step trial, not a general
  throughput claim.
- Reduced numerical damping was not isolated or measured. The short trajectory
  and final-Z comparison do not substitute for a damping study.
- Long-duration motion, full Print Logo completion and calibrated accuracy were
  not established.
- The independent Python physics backend remains unsupported.
- The untracked file `research/prompts/autocal_move_the_origin_idea.txt` is
  unrelated and was left untouched.

## References

- [Müller, Chentanez, Jeschke and Macklin (2018), Cable Joints](https://matthias-research.github.io/pages/publications/cableJoints.pdf):
  condensed cable constraints, unilateral rest lengths, dynamic attachments,
  pinholes and the inspiration for the gate abstraction.
- [Prior investigation](../2026-10-20-interactive-research-62300644/report.md):
  HP4 bearing behavior, failed simple bridge and ideal common-tension gate.
- Baseline revision `b9983100`; final measurements come from the uncommitted
  stateful-gate worktree described in [research.md](research.md).
