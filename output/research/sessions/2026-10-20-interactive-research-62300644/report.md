# HP4 bearing spin, tension reporting and a two-port pinhole gate

Status: stability correction complete; unsafe slip removal withdrawn. A complete solver repair remains open.

## Abstract

Implemented an opt-in HP4 two-port pinhole_gate attachment with a condensed unilateral length solve, common tension and summed carrier force/torque Jacobians. It removes internal CableJoints rather than replacing them with sequential pinholes. The variant preserves hidden material length and series compliance, reduces cable joints from 51 to 33 and projection visits from 102 to 42 per two-iteration substep. Four analytic JS/Rust checks pass for combined mass response and asymmetric torque; three one-second native/Wasm/JS logo checks pass with sampled positions within one micrometre. The focused JS suites pass 350 checks and backend suites pass 119. Existing stability gate completion is recorded in gate-stability-final.log. All evidence is simulated. This first version has zero flow inertia and internal friction; the independent Python engine explicitly rejects it. Speedup, timestep-bias improvement, physical accuracy and long-run stability were not established. Visualization is independently configurable through a local polyline. The production default scene remains unchanged.


## Topics

HP4, Hangprinter, Cable Joints, passive pulley, bearing spin, pinhole gate, reeving, XPBD, numerical damping, tension telemetry, multiplier accumulation, torque mode, timestep convergence, energy conservation, autocalibration

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
| 64-substep comparison | E32 | Not tested: the supported 1–16 range rejects construction before physics advances | [Failure log](E32.log), [input](E32-input.usda) |

## What is physically wrong with the startup spin

The paper makes an explicit no-slip assumption for rolling contacts. Rest length moves as the wheel rotates and tangent points change. A pinhole is different: cable can slide through it, so neighboring rest lengths are redistributed. These are different constitutive operations, even if a nearly massless bearing eventually gives almost equal tensions. [Müller et al. (2018), sections 1, 3.1 and 3.3](https://matthias-research.github.io/pages/publications/cableJoints.pdf).

HP4 gives every wheel a `SpoolStateComponent`, orientation, inertia and encoder. Its attachment update transports cable through rotation. The friction pass then recognizes a free rolling wheel, bypasses its cable-contact friction coefficient, and falls through to unconditional equal-tension rest redistribution. This is an extra slip degree of freedom. JS, Python and Rust all implement it. The coefficient 0.5 in `WheelMaterial` concerns cable contact; it is not a measured bearing-axle resistance. The ABC pinholes have coefficient zero.

E01 isolates that operation without advancing mechanics. Two one-metre spans have rest lengths 0.999 and 1.000 m. One call changes both to 0.9995 m while the wheel encoder remains exactly zero: **0.5 mm of material passes a stationary rolling surface**. The algebra preserves total length, but violates the rotation/material relation.

In the complete held-motor HP4 trial, the mover settles near its initial position while wheels keep turning:

| Trial | Model | Substeps | Maximum wheel angle at 30 s |
| --- | --- | ---: | ---: |
| E02 | Original, native | 1 | 988.463 rad = 157.3 turns |
| E03 | Original, native | 4 | 44.870 rad |
| E06 | Original, JS | 1 | 988.463 rad |
| E07/E18 | No independent rolling redistribution, JS/native | 1 | 0.015199 rad |

The worst original wheel is `WheelD2_left_top`; its angular velocity is about 32.96 rad/s at 30 s. Some ABC wheels also rotate despite stationary body and motor commands. The no-slip candidate's final body height differs by only **5.42 micrometres** from the original held trial. This isolates the bearing artifact without relying on calibration fits. These fresh worlds have zero buildup, 30 mm spools, closed loop enabled, a 2 ms outer step and an 81 g mover.

### Isolated energy and equilibrium check

E15/E16 use one wheel with a fixed axle and two horizontal spans ending at immobile anchors. Radius is 6.5 mm, inertia 5e-7 kg m², each span stiffness is 20,000 N/m, and initial rest-length asymmetry is 1 mm. The analytic common tension is 10 N and equilibrium rotation magnitude is 0.076923 rad. There is no gravity or motor, and damping is nonnegative.

At a 2 ms timestep, original redistribution gives **215.23 rad** rotation after one second. Elastic plus rotational energy increases from **0.0100 J to 0.01763 J**, despite no external work. With redistribution disabled, rotation stops at 0.09939 rad and energy falls to 0.005427 J. At 0.125 ms, the latter rotation is 0.07613 rad, about 1.0% below the analytic value. Intermediate errors are nonmonotonic. Thus the slip mechanism has a mechanical counterexample, and the remaining solver still has finite-step bias. [Fixture source](E15-source.mjs), [convergence source](pulley-convergence.mjs), [measured energy](pulley-energy.json).

![Held wheels, controlled pulls and isolated pulley energy](mechanics-summary.png)

## Much of the visible tension difference is a reporting problem

In XPBD the force estimate uses the **accumulated constraint multiplier divided by timestep squared**. A later solver visit applies a correction to that multiplier. Both implementations currently write `constraintForceMagnitude` and cable-load feedback only on the first visit. Some mechanically fixed spans return before writing any force at all. Consequently the displayed number can be a partial solve, or zero, while the span carries tension.

E28 traces the actual original JS solver at two simulated seconds. For D0, in path order:

| Span | Displayed first-visit force, N | Accumulated force estimate, N |
| --- | ---: | ---: |
| Spool → upper left wheel | 0.007330 | 0.066468 |
| Upper left → lower left | 0.037301 | 0.066293 |
| Lower left → upper right | 0.036195 | 0.066296 |
| Upper right → lower right | 0.035082 | 0.066299 |
| Lower right → fixed attachment | 0.059893 | 0.066305 |

The 81 g mover weighs 0.79542 N. Twelve nearly vertical supporting legs imply about **0.066285 N per leg**. The accumulated estimates are close to this simple force balance; the displayed values are not. E27, with rolling redistribution removed, gives essentially the same accumulated D forces. This demonstrates a diagnostic error in this held case; it does not establish equal physical tensions during every transient.

The same first-visit policy affects torque-mode motor feedback, so this is also a dynamics hypothesis. Motor feedback should accumulate signed multiplier corrections while counting stiffness/damping coefficients once per physical constraint. Updating telemetry alone would not correct that feedback. E33–E36 explore this distinction. [Trace and calculation](multiplier-audit.json), [Rust implementation](../../../../src/wasm/cable_solver/src/lib.rs), [JS implementation](../../../../src/js/cable_joints_3d/cable_joints_core.js).

### Accumulated motor feedback: a useful incremental result

The October 16 bookkeeping repair correctly requires one stiffness/damping contribution per physical constraint. E33–E36 keep that requirement and sum the **signed incremental multiplier corrections** into torque. This is the small change archived in [accumulated-feedback.patch](accumulated-feedback.patch). Display force fields are not changed in these candidates.

All entries below use **JavaScript**, no independent rolling redistribution, the same commands, and ten simulated seconds:

| Motor/load formula | Substeps | X, mm | Y, mm | Experiments |
| --- | ---: | ---: | ---: | --- |
| Original motor, first-visit load | 1 | 8.868 | 19.605 | E09 |
| Original motor, accumulated load | 1 | -6.771 | 34.145 | E33 |
| Original motor, first-visit load | 4 | 5.568 | 107.098 | E36 |
| Original motor, accumulated load | 4 | 0.293 | 108.422 | E34 |
| Faster motor, first-visit load | 1 | 565.904 | 612.454 | E23 |
| Faster motor, accumulated load | 1 | 93.020 | 594.754 | E35 |

The four-substep load change greatly reduces lateral error with a small Y change. Combining it with the faster motor also reduces the extreme one-substep lateral excursion, but does not match the finer-step trajectory. This supports further incremental repair of the coupled update. Loaded-state stability, energy/passivity and backend parity of this feedback candidate are **not tested**; the isolated pulley energy check concerns rolling redistribution, not this later candidate. [Previous bookkeeping report](../2026-10-16-hp4-load-bookkeeping/report.md), [all rollouts](motion-summary.json).

## The faster motor update should remain under investigation

The previous report correctly withheld production acceptance after poor calibration fits, but those fits do not prove that faster motion is unphysical. This session reproduces the formula and evaluates mechanics separately.

The original motor denominator contains `1 + h D/I + h² K/I`. Its steady-motion equation includes an additional `h K omega` resistance. The candidate compensates this term because cable load was evaluated at an already predicted pose. That local argument does not prove the complete staggered motor/cable/carrier update is correct.

With rolling slip removed, the matched **native** comparison is:

| Substeps | Original motor: Y at 30 s | Faster motor: Y at 30 s | Faster motor: X at 30 s |
| ---: | ---: | ---: | ---: |
| 1 | 66.30 mm | 633.67 mm | 675.47 mm |
| 4 | 322.78 mm | 602.15 mm | 73.58 mm |
| 16 | 548.79 mm | 572.41 mm | 3.94 mm |

The original/faster Y difference shrinks from **567.38 mm to 23.62 mm** as the timestep decreases. At 16 substeps the original X is 2.17 mm, also close to the faster result. This supports revisiting the faster formula, while its enormous one-substep lateral error shows that replacing one formula is insufficient. No exact continuum solution or hardware trajectory is available, and 16 substeps are not proof of convergence. We did not repeat autocal fitting.

All pull comparisons settle for two seconds, then command A=-0.000334089 Nm and B=C=-0.037733 Nm, holding D fixed. This repeats the earlier matched-force HP4 workload. E04/E05 reproduce its original approximately 80/335 mm Y positions. Position includes the body COM frame; these values are not commanded Cartesian targets. [Input/rollout harness](mechanics.mjs), [earlier matched-force study](../2026-10-19-interactive-research-8d7a82a2/report.md).

## A two-port pinhole gate has a precise mechanical contract

Your proposed abstraction is sound for the ideal low-inertia case. Both ports should remain at their actual body-local positions. The hidden cable length is constant under a rigid motion, so the solver does not need to resolve the internal bearings to calculate the external length gradient or wrench.

![Two-port gate schematic](gate-schematic.png)

For world body position `x`, rotation `R`, local ports `p_in/p_out`, and adjacent anchor points `a_in/a_out`, use

```text
L_gate = |x + R p_in - a_in| + L_internal + |x + R p_out - a_out|
g_translation = u_in + u_out
g_rotation = (R p_in) × u_in + (R p_out) × u_out
body force/torque = -T * [g_translation, g_rotation]
```

Here each `u` points from its anchor toward its port. The other external spans, spool payout and total compliance belong to the enclosing cable-path constraint. For frictionless quasistatic flow, one tension acts at both ports. A single point at their midpoint changes the wrench and pose dependence. A single **component with two ports** preserves both without retaining separate physical bearing constraints.

E25 transforms the actual six ABC routes through 500 random poses each: translations up to ±0.3 m per axis and rotation-vector components up to ±0.1 rad. A reusable [PinholeGate prototype](pinhole_gate.py) matches full routed length to **4.44e-16 m** and its analytic gradient to finite differences within **1.11e-9**. An initially aligned midpoint approximation reaches **0.134 mm** length error and **0.000244 Nm** wrench error at 1 N tension in these poses. These are synthetic geometric checks, not time integration or hardware validation.

The hidden route measures **105.448 mm** per branch, including both wheel arcs. Entry and exit are only **59.5 mm** apart. Preserve the **45.948 mm difference** in internal stored length or the gate's explicit constant, particularly for winding/material bookkeeping. Collapsing geometry must not silently discard cable length.

### Finite inertia, compliance and animation

The scene gives each ABC wheel inertia 5e-7 kg m² at 6.5 mm radius. Its reflected inertia in the cable-flow coordinate is `I/r² = 11.83 g`; the pair is **23.67 g**. That is not automatically negligible relative to an 81 g mover, although its effect depends on flow acceleration and reeving. D wheels use 5e-8 kg m². A gate can first expose an explicit ideal zero-inertia limit, then include measured or deliberately chosen flow inertia and bearing resistance.

A finite-inertia gate needs a consistent flow coordinate: the tension difference accelerates its effective flow mass, and viscous/Coulomb resistance dissipates energy. Cable contact friction and axle resistance should remain distinct. Animate bearing angle from that flow, `delta_theta = ±delta_s/r`, rather than evolving another unconstrained visual rotor. Carrier/rotor angular momentum coupling requires care if finite inertia is retained. These extensions are **proposed, not tested**.

Also preserve compliance. The current path stiffness is per joint, not a measured material modulus. Six identical springs in series have compliance `6/k`; four have `4/k`. Removing two joints without changing stiffness changes the model. E13 uses four-joint stiffness/damping equal to 4/6 of the original values as a series-model control, not as a claim about real cable stiffness.

### What the integrated bridge prototype actually established

E10–E13/E21/E22 replace each ABC pinhole-wheel-wheel-pinhole route with two original pinholes and a straight internal bridge. The source scene stays intact; generated scene inputs are frozen beside each run. Original wheel masses and carrier inertia remain present, and unused free rotors are still part of the assembly. This is a routing experiment, not a completed gate implementation.

It reduces total cable constraints from **51 to 39**, and two-iteration visits from **102 to 78** per substep. It removes ABC bearing runaway. However, Y at 30 seconds is about **-0.50 mm with one substep** versus **363.49 mm with four** after the no-slip change. The compliance-matched bridge also remains biased. Simply shortening the sequential constraint chain therefore does not solve the numerical coupling problem.

The next implementation should solve the combined external path-length constraint, or an equivalent coupled block, with the gate's internal coordinate eliminated. Summing frictionless spans requires summing their compliance and rigid-body Jacobians. It can avoid dependence on several sequential local solves. This solver integration and any speedup are **not tested**. Per-case wall times are exploratory workload costs; changed trajectories, build profiles and some overlapping analysis/build work prevent matched performance claims.

## Validation, artifacts and scope

The no-slip experimental patch changes only the three physics implementations and two tests, plus the rebuilt Wasm artifact. It is preserved with source and native binary. It passed three friction checks and 176 Python differential parity checks. The backend run passed 109 checks, skipped one, and failed five: three loaded-state settling cases and two HP4 JS/Wasm comparisons with angular-velocity differences of order 2–3e-8 rad/s. No tolerance was loosened.

The old loaded checkpoint does not settle under the isolated change. In the last five seconds of a 20-second replay, its vertical range is 0.938 m at one substep, 45.9 micrometres at two, and 5.39 micrometres at four. It was created using the original slip model; it is not a measured real-world state. This is an integration limitation and a demand for a coupled solver/convergence audit, not proof that the no-slip model is physically wrong. [All failed checks](backend-tests.log), [experimental patch](no-slip-candidate.patch), [candidate source](no-slip-source/), [native candidate](no-slip-physics.node).

All trials use fresh numerical worlds, not the attached launcher's continuing world. The command socket probe passed. Supervisor preflight passed services, firmware, collector and recorder but correctly reported a mismatch: the attached world is layered while these experiments require zero buildup. It stayed untouched. Tool checks establish connectivity only. No full autocal collection, calibration accuracy measurement or hardware experiment was performed. E32 ran no physics because 64 substeps exceeded the supported range.

At the end of the initial experiment phase, the production source tree was restored to its clean starting revision. The later requested integration below supersedes that source state. The original native and Wasm artifacts were rebuilt; the restored backend/loaded-state checks pass **113 checks with one skip**, and the restored friction checks pass **3**. The default native artifact SHA-256 is `99c4f129fbf7949b795d3f2f4074dea02ec3e879c782d95da402584748566c60`. The attached world remains generation 1, step zero, with no reset required and no changed-source flags. [Restored checks](restored-backend-tests.log), [friction checks](restored-friction-tests.log), [attached state](final-attached-world.json).

We completed 31 bounded full-world trials plus 12 one-second pulley subcases, a frozen constitutive check and 3,000 synthetic gate poses. Full-world integration took approximately 887 seconds summed across trials, within the 1,200-second compute budget. Tests, builds, plotting and analysis are separate. One rejected construction (E32) advanced no world. See [provenance and model mapping](provenance.json), [reproduction instructions](REPRODUCE.md), and the immutable input/rollout files referenced there. No production physics change was accepted; the candidates remain available for continued mechanics work.

## Next decision

Implement accumulated force telemetry and audit motor-load feedback against actual final multipliers. Then add an ideal two-port gate to a combined path constraint or a small coupled constraint block, preserving both ports, hidden material length, series compliance and spool payout. Validate that implementation against fixed-anchor pulley energy/rotation, static gravity balance, torque balance `I*alpha = r*(T_in-T_out)`, and matched timestep refinements. Add flow inertia/resistance only with an explicit energy and momentum contract. Only after those mechanics checks should autocal fitting be used to assess the collector/model interaction. These integrations and a fresh accurate calibration are **proposed, not tested**.

## References

- [Previous HP4 autocal investigation](../2026-10-19-interactive-research-8d7a82a2/report.md), starting at revision 3b3a7fec: earlier faster-motor experiment, matched forces and calibration limitations; its rejected patch informed E14/E23/E29–E31.
- [HP4 scene](../../../../public/usd_scenes/hp4_rigid_body.usda), revision c6f0b5a1bb501100706ecdc5e47294b74dac5ab8: actual routing, dimensions, masses, inertia, stiffness and materials used here.
- Matthias Müller, Nuttapong Chentanez, Stefan Jeschke and Miles Macklin (2018), *Cable Joints*, Computer Graphics Forum 37(8), pp. 1–10, [DOI: 10.1111/cgf.13507](https://doi.org/10.1111/cgf.13507), [author PDF](https://matthias-research.github.io/pages/publications/cableJoints.pdf), [local transcription](../../../../ai_docs/CableJoints/CableJoints.md): rolling/no-slip assumptions, pinhole redistribution and cable-joint model.
- Miles Macklin, Matthias Müller and Nuttapong Chentanez (2016), *XPBD: Position-Based Simulation of Compliant Constrained Dynamics*, [DOI: 10.1145/2994258.2994272](https://doi.org/10.1145/2994258.2994272), [author PDF](https://matthias-research.github.io/pages/publications/XPBD.pdf), [local paper](../../../../ai_docs/XPBD/XPBD.md): accumulated multiplier/force interpretation used in E27/E28. The repository's implementation uses its own damping and motor extensions.
- [Mechanical rollout harness](mechanics.mjs), [isolated pulley source](E15-source.mjs), [pulley convergence harness](pulley-convergence.mjs), [gate audit](gate_contract.py), [analysis/plots](analyze.py): experiment construction and calculations used for this report.
- [Recording cost study](../../../../research/autocal-recording-performance.md): informed numerical JSON evidence and compute budgets; no recording-performance ratio is claimed here.
- [October 16 load bookkeeping](../2026-10-16-hp4-load-bookkeeping/report.md): the once-per-constraint stiffness/damping requirement retained in E33–E35.
- [Experiment provenance](provenance.json) and [reproduction workflow](REPRODUCE.md): exact model/artifact mapping, hashes, input contracts and bounded commands.
- [Research notebook](research.md): accepted steering, decisions, budget changes and failed trials.

## Subsequent requested integration — withdrawn in part after stability failure

The user requested three separate commits: `9ebc8337` removes independent rolling slip; `41010aa1` fixes total-multiplier force readback and final force transfers; `628a8963` fixes signed motor-load accumulation and counts K/D once. The alternative motor formula is unchanged. [Separate formulation report](../../../../research/faster-motor-update.md) explains its origin, derivation and timestep behavior.

Historical checks at the incorrect acceptance (superseded below): 188 Python, 110 Warp CPU, 20 JS and 115 native/Wasm passes, with six CUDA/GC skips. Five known HP4 failures were left unresolved: three loaded-checkpoint settling cases and two tight JS/Wasm trajectory comparisons. [Integration manifest](integration/provenance.json) links the exact logs, rebuilt binaries and I01–I04 trials. I01/I03 have exactly equal sampled physical state across the display-only fix; I04 reproduces the earlier feedback candidate. These checks do not establish calibration accuracy or a complete dynamics repair.

The bridge experiment solved each remaining span sequentially; deleting wheels did not combine those spans into one gate/path constraint. That finite-iteration splitting still changes with timestep. A properly coupled two-port gate remains a proposed extension.


## Stability correction after the browser Print Logo failure

The previous three-commit acceptance was wrong. In particular, the loaded-state
regression should have blocked shipping `9ebc8337`. Labeling a checkpoint as
legacy did not make its stability failure acceptable. The ordinary one-substep
logo was not tested. The following correction supersedes that acceptance; all
previous failed experiments and logs remain preserved.

### Tested hypotheses and evidence

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| The shipped changes break normal HP4 logo motion | S01, S01-browser | Supported: native 20 s gives 79 missed steps, 11.69 m/s peak speed and 50.53 rad/s peak rotation; browser reproduction also leaves the print plane | [Native failure](integration/S01-before.json), [browser observation](integration/browser-reproduction.json) |
| Restoring rolling redistribution while retaining force and feedback fixes recovers the baseline logo trajectory | S02, S03 | Supported for prefix: sampled body trajectory and all extrema/diagnostics match exactly, zero missed steps | [Baseline](integration/S02-baseline.json), [corrected](integration/S03-recovered.json), [comparison](integration/prefix-equivalence.json) |
| The new short headless gate detects the shipped instability | S06 | Supported: archived broken engine fails at step 293 (0.586 s); corrected code passes all six checks | [Negative control](integration/gate-rejects-broken-final.log), [passing gate](integration/stability-gate-final.log) |
| Retained signed motor feedback executes a bounded torque-mode trial with restored redistribution | S05 | Supported for 30 s only; no accuracy conclusion | [Torque pull](S05-feedback-pull.json) |
| Full job is bounded and settles, and the terminal excursion is reproduced by the original engine | S04, S07 | Mixed: complete stream is bounded; terminal motion/counts match the original engine exactly, including 300 lost steps; no full tracking pass | [Failed fixed-plane trial](integration/full-logo.log), [complete diagnostic](integration/S04-full-retry.json), [tail comparison](integration/tail-comparison.json) |

The source correction restores the three redistribution implementations and
withdraws assertions specific to the rejected no-slip shortcut. Final-multiplier
tension readback and signed motor-load accumulation remain. Default substeps,
scene parameters, logo commands and loaded-state settling tolerances are unchanged.
A stale Jest expectation that an immobile stretched span display zero force was
updated to its analytic 100 N, consistent with the requested tension display fix.

`npm run test:physics` rebuilds native and browser Wasm physics and runs the new
production logo gate plus existing loaded-state tests. The gate uses the actual
bundled CAN file and production firmware player, scene bake and physics system
order. It exercises native/Wasm 20-second prefixes and the JS fallback for 2 seconds
at default layering, open-loop motors and one substep. Every step checks finite
pose/velocity, bounded workspace, print-plane displacement, speed and rocking;
completion requires no queue residue, physics errors or missed steps. The loaded
state runs 20 seconds at 1/2/4 substeps with its original strict tolerances.

The command takes about 30 seconds on this host. It is automatic under `npm test`
and CI, and both root and research AGENTS instructions make failures block delivery.
The source comments document why simply reinstating the skip is unsafe. The
separate [correction report](../../../../research/hp4-logo-stability.md) gives the
acceptance criteria and limits. [Source/artifact manifest](integration/stability-provenance.json)
and [working patch](integration/stability-working.diff) identify this correction.

Focused validation: 157 backend/host checks; 196 Python core/differential checks;
110 Warp CPU checks with 5 CUDA skips; 800 Jest checks with 1 skip; 4 existing
logo substep checks; 6 new gate/loaded checks; 3 Rust native tests. All pass.
These are the suites actually run, not a claim that the entire CI pipeline or
a full autocalibration was run. Initial mistyped-path, stale-build and stale
Jest-expectation failures are retained beside their successful reruns.

Accepted user steering: browser experiments should be rare and preferably human
driven. One isolated browser reproduction preceded that message and was stopped;
the tab was closed. All remaining validation uses fresh headless production
worlds, including browser Wasm executed in Node. The continuing attached native
world was not advanced or reset.

This recovers stable production behavior. It does **not** fix artificial bearing
slip/spin or establish physical accuracy. A coupled no-slip transport/rotor solver
remains open; the experimental faster motor formula has never been merged.


## Final stability decision

The unsafe rolling-slip shortcut is withdrawn. The two requested bookkeeping
fixes remain. Default substeps, physical parameters and existing settling
tolerances remain unchanged. `npm test -- --runInBand` passes the mandatory
six-case stability gate and all 800 active Jest tests. The focused Python/Warp,
backend/host, higher-substep logo and Rust checks listed above also pass.

S07 completes all 1,038,258 commands (2076.516 simulated seconds) in a fresh
native world without divergence. Peak body speed is 0.358324 m/s, peak rotation
0.184433 rad/s, and COM Z lies in [0.0896043,0.1049830] m. The last return-to-zero
commands lose 300 motor steps and leave the print plane. Therefore the original
full-stream 5 mm-plane trial remains a failed acceptance test; this result is
not renamed a perfect full-print pass.

A checkpoint 500 commands before the end supports a controlled comparison.
The corrected-engine replay reproduces every one of the full run's last 500
body samples exactly, establishing continuation fidelity. The original
c6f0b5a1 engine then reproduces all 5500 tail/hold samples and all missed-step
counts exactly from that same numeric state. After ten extra simulated seconds,
both bodies have speed 1.30e-13 m/s and zero angular velocity; the last-five-second
vertical range is 2.45e-13 m. The terminal tracking loss is therefore reproduced
by the original engine; it remains a separate job-ending limitation. This is
not an independent baseline replay of the entire 34.6-minute stream.

Evidence: [complete diagnostic](integration/S04-full-retry.json),
[original-engine tail](integration/S07-baseline-tail.json),
[corrected-engine tail](integration/S07-corrected-tail.json),
[exact comparisons](integration/tail-comparison.json),
[checkpoint](integration/S04-full-retry-tail-checkpoint.json),
[validation index](integration/STABILITY_VALIDATION.md) and
[provenance](integration/stability-provenance.json).

S08 additionally completes the same 30-second torque pull at four substeps; its
finite trajectory differs from the one-substep trial, so it is not presented
as convergence or calibration evidence. [S08 snapshots](S08-feedback-pull-4.json).

The separate [short correction report](../../../../research/hp4-logo-stability.md)
and [motor formulation report](../../../../research/faster-motor-update.md)
make the production rollback and remaining research directions explicit.
Future dynamics changes must pass the automatic headless motion gate before
delivery. Browser experiments should be rare and preferably human driven.


Corrective source revision: `06868dd8d1d8f45f950b629da8d73a180708c9c7`. Working tree clean after commit.
The retained force-display and feedback revisions are `41010aa1` and `628a8963`;
`9ebc8337` is withdrawn by this correction. All experiment processes completed.


## Implemented two-port gate (G01–G03)

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

Gate feature revision: `b99831003719cfe56d74f67da817201d2af8ce37`. Mandatory stability gate:13 passed; backend:119 passed; focused JS:350 passed. No further experiments performed.
