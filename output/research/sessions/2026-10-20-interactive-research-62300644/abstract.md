# HP4 bearing spin, tension reporting and a two-port pinhole gate

Status: two-port gate implemented as an opt-in variant; physical validation remains open.

## Abstract

Implemented an opt-in HP4 two-port pinhole_gate attachment with a condensed unilateral length solve, common tension and summed carrier force/torque Jacobians. It removes internal CableJoints rather than replacing them with sequential pinholes. The variant preserves hidden material length and series compliance, reduces cable joints from 51 to 33 and projection visits from 102 to 42 per two-iteration substep. Four analytic JS/Rust checks pass for combined mass response and asymmetric torque; three one-second native/Wasm/JS logo checks pass with sampled positions within one micrometre. The focused JS suites pass 350 checks and backend suites pass 119. Existing stability gate completion is recorded in gate-stability-final.log. All evidence is simulated. This first version has zero flow inertia and internal friction; the independent Python engine explicitly rejects it. Speedup, timestep-bias improvement, physical accuracy and long-run stability were not established. Visualization is independently configurable through a local polyline. The production default scene remains unchanged.

## Topics

HP4, Hangprinter, Cable Joints, passive pulley, bearing spin, pinhole gate, reeving, XPBD, numerical damping, tension telemetry, multiplier accumulation, torque mode, timestep convergence, energy conservation, autocalibration, Print Logo, stability regression, headless validation

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Rolling links transport material independently of wheel rotation | E01, E15, E16 | Supported: frozen rotation still permits rest redistribution; isolated pulley spins under equalized tension | [Frozen wheel](E01-frozen-wheel.json), [pulley](E15-pulley.json), [energy audit](pulley-energy.json) |
| This redistribution explains startup bearing runaway | E02, E03, E06, E07, E18, E28 | Supported in held HP4 trials; removing it suppresses persistent spin | [Motion summary](motion-summary.json), [plot](mechanics-summary.png) |
| Removing rolling redistribution alone repairs HP4 dynamics | E09, E18–E20, E26; candidate integration checks | Rejected as a sufficient correction: slow motion/timestep dependence and loaded-state instability remain | [Candidate patch](no-slip-candidate.patch), [integration failures](backend-tests.log) |
| Large displayed tension differences necessarily mean equally large physical imbalance | E27, E28 | Rejected for the held D path: accumulated multipliers nearly agree | [Multiplier audit](multiplier-audit.json) |
| A rigid two-port gate preserves ideal ABC path geometry and wrench | E25 | Supported for geometry and an ideal common tension; no dynamic validation | [Gate contract](E25-gate-contract.json), [prototype](pinhole_gate.py) |
| Replacing the internal route with a simple two-pinhole bridge is sufficient to remove timestep bias | E10–E13, E21, E22 | Rejected as sufficient; gate abstraction remains promising | [Inputs and motion](motion-summary.json), [bridge harness](mechanics.mjs) |
| The faster motor formula remains a promising mechanics direction | E14, E23, E24, E29–E31, compared with E19, E20, E26 | Supported as a direction by shrinking formula disagreement with timestep; physical accuracy remains inconclusive | [Native convergence results](motion-summary.json) |
| Removing passive-wheel backdrive through a pinhole is sufficient | E17 | Rejected as sufficient; this isolated change reduces Y travel further | [E17](E17.json), [frozen variant source](E17-candidate-core.mjs) |
| Accumulated motor-load feedback alone or with the faster update improves dynamics | E33–E36 | Mixed: better force coupling/lateral response; timestep error and integrated stability remain unresolved | [Saved trials](motion-summary.json) |
| The mandatory headless gate catches the shipped default-logo regression | S01–S03, S06 | Supported: broken engine fails after 0.586 s; corrected prefix and loaded checks pass | [Negative control](integration/gate-rejects-broken-final.log), [gate](integration/stability-gate-final.log) |
| Terminal tracking loss is reproduced by the original engine from the same checkpoint | S04, S07 | Supported: 5500 identical body samples and identical 300 lost steps; no whole-job accuracy pass | [Tail comparison](integration/tail-comparison.json) |
| 64-substep comparison | E32 | Not tested: the supported 1–16 range rejects construction before physics advances | [Failure log](E32.log), [input](E32-input.usda) |

## References

- [Full report](report.md): mechanics, measured comparisons, failed checks and limits.
- [Research notebook](research.md): accepted steering, budgets and next decision.
- [Provenance](provenance.json): input/model mapping, source and binary hashes.
- [Rollout summary](motion-summary.json), [multiplier audit](multiplier-audit.json), [pulley energy](pulley-energy.json), [gate contract](E25-gate-contract.json): measured and synthetic evidence.
- [Previous HP4 investigation](../2026-10-19-interactive-research-8d7a82a2/report.md): continued faster-motor and autocal work.
- Müller, Chentanez, Jeschke and Macklin (2018), *Cable Joints*, [DOI 10.1111/cgf.13507](https://doi.org/10.1111/cgf.13507): rolling/no-slip and pinhole model.
- Macklin, Müller and Chentanez (2016), *XPBD*, [DOI 10.1145/2994258.2994272](https://doi.org/10.1145/2994258.2994272): accumulated multiplier force estimate.
- [PinholeGate prototype](pinhole_gate.py) and [reproduction commands](REPRODUCE.md): reusable ideal geometry implementation and all candidate workflows.

## Subsequent requested integration

The user requested three separate commits: `9ebc8337` removes independent rolling slip; `41010aa1` fixes total-multiplier force readback and final force transfers; `628a8963` fixes signed motor-load accumulation and counts K/D once. The alternative motor formula is unchanged. [Separate formulation report](../../../../research/faster-motor-update.md) explains its origin, derivation and timestep behavior.

Final checks: 188 Python, 110 Warp CPU, 20 JS and 115 native/Wasm passes, with six CUDA/GC skips. Five known HP4 failures remain: three legacy loaded-checkpoint settling cases and two tight JS/Wasm trajectory comparisons. [Integration manifest](integration/provenance.json) links the exact logs, rebuilt binaries and I01–I04 trials. I01/I03 have exactly equal sampled physical state across the display-only fix; I04 reproduces the earlier feedback candidate. These checks do not establish calibration accuracy or a complete dynamics repair.

The bridge experiment solved each remaining span sequentially; deleting wheels did not combine those spans into one gate/path constraint. That finite-iteration splitting still changes with timestep. A properly coupled two-port gate remains a proposed extension.

## Stability correction

The prior three-commit acceptance was invalid. See [S01–S06 evidence](integration/),
[notebook](research.md#stability-correction-progress) and the forthcoming correction
section in the full report. S01/S02/S03 support restoring stable logo behavior;
S06 supports the gate detecting the rejected engine. The complete native replay and original-engine terminal comparison are complete; the inherited terminal tracking loss is documented below.

See the [stability correction report](../../../../research/hp4-logo-stability.md) and [validation index](integration/STABILITY_VALIDATION.md) for the final source decision and executed checks.

Corrective source revision: `06868dd8d1d8f45f950b629da8d73a180708c9c7`; unsafe shortcut withdrawn.

## Gate implementation

Implemented an opt-in HP4 two-port pinhole_gate attachment with a condensed unilateral length solve, common tension and summed carrier force/torque Jacobians. It removes internal CableJoints rather than replacing them with sequential pinholes. The variant preserves hidden material length and series compliance, reduces cable joints from 51 to 33 and projection visits from 102 to 42 per two-iteration substep. Four analytic JS/Rust checks pass for combined mass response and asymmetric torque; three one-second native/Wasm/JS logo checks pass with sampled positions within one micrometre. The focused JS suites pass 350 checks and backend suites pass 119. Existing stability gate completion is recorded in gate-stability-final.log. All evidence is simulated. This first version has zero flow inertia and internal friction; the independent Python engine explicitly rejects it. Speedup, timestep-bias improvement, physical accuracy and long-run stability were not established. Visualization is independently configurable through a local polyline. The production default scene remains unchanged.

- G01: combined mass/Jacobian and separate-port wrench, four analytic JS/Rust checks: supported. [Final checks](gate-tests-final3.log).
- G02: one-second production motion and shared-tension readback, native/Wasm/JS: supported for this bounded prefix; sampled position agreement within1e-6m. [Checks](gate-tests-final3.log).
- G03: baseline preservation and automatic protection:119 backend checks and350 focused Jest checks pass; mandatory gate log below. [Backend](gate-backends.log), [JS](gate-js-final2.log), [stability](gate-stability-final.log).
- Reduced numerical damping or overall wall time: not tested. Structural projection-count reduction is not a speed benchmark.

Initial failures are preserved: native enum validation initially rejected kind5; a test cleanup called an unavailable dispose method; JS initially double-counted spool rotation in damping, corrected by separating carrier angular and rotor coordinates. A broad textual edit also touched the unrelated merge function; focused tests caught it and the duplicate block was removed. Source wiring/preset test fixtures were corrected and rerun.

[Implementation and usage](../../../../research/pinhole-gate.md),
[generated gate scene](../../../../public/usd_scenes/hp4_pinhole_gate.usda),
[scene generator](../../../../scripts/create_hp4_gate_scene.mjs),
[executable checks](../../../../tests/benchmark/pinhole_gate.test.mjs).
No browser experiments, long collection or full-logo runs were performed for this feature.
