# HP4 passive pulley mechanics and a two-port pinhole gate

Status: complete investigation. Measured and synthetic results are available in [report.md](report.md); a complete solver repair remains open.

## Objective and accepted steering

Continue [the October 19 investigation](../2026-10-19-interactive-research-8d7a82a2/report.md). The user asks for detailed HP4 mechanics, especially wheels spinning with apparently stationary cables and unequal reeved-segment tensions. Failed autocal fitting alone does not reject faster dynamics. Investigate a rigid effector `pinhole_gate` with distinct entry and exit points, inspired by Müller et al., Cable Joints (2018). Keep production edits small; use `.venv/bin/python`.

## Hypotheses

- H1: passive rolling links receive nonphysical rest-length redistribution independently of their rotation, violating the paper's no-slip model.
- H2: the observed HP4 wheel spin and tension differences persist under nominal held commands, with measurable timestep/solver dependence.
- H3: replacing ABC's internal pinhole-wheel-wheel-pinhole route with two pinholes and a fixed internal span preserves external wrench/reeving while reducing numerical artifacts and cost.
- H4: correcting the motor stabilization term remains promising when judged using mechanical invariants; previous calibration results alone do not establish the physics error.

## Initial constraints and budgets (historical)

Simulation only; no hardware claims. Work at explicit zero cable buildup, 30 mm spools and closed loop unless a case states otherwise. Attached world stays untouched; it is layered and therefore does not match this numerical contract. Fresh standalone worlds share the production builder. Initial budget: at most 24 bounded cases, 120 simulated seconds each, at most 20 minutes total physics compute; no long autocal collection. Use numeric JSON rather than RRD for the matrix; add recording only if needed for physics inspection. Preserve failed cases and inputs. Mechanics and timestep convergence take priority over fit quality.

## Checks and provenance

Starting Git revision: c6f0b5a1bb501100706ecdc5e47294b74dac5ab8, clean worktree. Required research instructions, integration rationale and recording-cost study read. Command-context socket probe passed socket creation, bind and HTTP round trip. Supervisor preflight 838a43ade26c467592178cfce1d42629 passed service/firmware/collector/recorder checks but correctly rejected requested zero-buildup settings against attached layered world at step zero. These checks do not establish physics accuracy.

## Initial next decision (historical)

Read pinhole/rolling update and solver equations; build a minimal invariant experiment before selecting a production correction or a reduced routing prototype.

## First results and decision

E01 frozen-wheel operation transfers 0.5 mm with zero angle. E02/E06 reproduce startup wheel runaway in native and JS; E07 removes rolling redistribution and reduces maximum 30-second angle from 988.46 to 0.0152 rad. E09 pull remains numerically slow. E10–E13 test an explicit two-pinhole bridge, with and without matched series compliance: it removes ABC solver wheels but retains severe timestep dependence. E14 reproduces previous faster motor formula in JS, without using a calibration fit as the physical criterion. E15/E16 isolate an inertial pulley and show spurious steady spin under equal tension; disabling redistribution settles near the analytic angle, with residual timestep bias.

Decision: retain a narrowly scoped rolling-material conservation fix across JS/Python/Rust, validate it separately, and preserve the gate as a research prototype. Investigate duplicate passive-wheel spin coupling through pinholes (E17). Extend the budget to at most 36 cases including sub-second analytic fixtures, still at most 20 minutes physics compute and no full autocal. Record sampled JSON geometry/forces/angles; no RRD conversion is needed.

## Integration limit and revised decision

The small no-slip candidate was built for native and Wasm. Friction tests pass (3); Python differential parity passes (176). Backend validation has 109 passes, one skip and five failures: the old loaded-state settling fixture no longer settles (one substep has 0.938 m vertical range in its last five seconds; two/four substeps have 45.9/5.39 micrometre range), and two HP4 JS/Wasm trajectory comparisons narrowly fail their ~2e-8 angular-velocity tolerances. These failures remain preserved, without relaxing gates. The legacy checkpoint was produced by the slip model, so its failure does not by itself reject non-slip physics; it prevents accepting the isolated edit as an integrated production repair. Archive patch, source and native binary, finish E18–E27, then restore the production source/artifacts.

Code inspection finds a second mechanistic concern: displayed force and motor cable feedback use only the first Gauss-Seidel multiplier, while later visits accumulate corrections. E27 will audit first versus accumulated multipliers. A full repair should use the converged constraint force and resolve inertia consistently. A gate must preserve distinct port positions and total compliance, and ideally solve a single path-length constraint rather than merely removing wheels from sequential solves.

## Force audit and motor convergence

E27/E28 trace the final Gauss-Seidel visit and show that the reported D0 span forces (~0.0073–0.0599 N for stock) differ strongly from accumulated multipliers (~0.0663 N). The latter agree with the gravity/12-support-leg estimate. This is direct evidence of misleading telemetry, not a physical capstan measurement. The first-visit multiplier also drives torque-mode load feedback. E33/E34 test using accumulated signed corrections for torque while recording K/D only once; E35 combines that with the faster motor formula. These are bounded exploratory candidates, not accepted solver changes.

With material slip removed, native stock versus faster-motor Y at 30 s is 66.30/633.67 mm (1 substep), 322.78/602.15 mm (4), and 548.79/572.41 mm (16). The shrinking difference supports keeping the faster formula as a promising direction. Strong one-substep lateral bias remains. E32 requested 64 substeps but construction correctly rejected the supported 1–16 range, before any physics ran; this is not a negative physics result. No limit was bypassed.

Final bounded scope: at most 36 named experiments, with at most 12 one-second pulley subcases inside E15/E16, and 20 minutes summed numerical compute. No autocal datasets or hardware measurements are being collected.

## Initial investigation outcome and next decision

All bounded mechanics experiments are finished. E36's matched JS control exactly agrees with the native four-substep control to approximately 1e-13 m at ten seconds. E33/E34 accumulated signed torque feedback reduces lateral bias while K/D remain counted once; E35 combines it with the faster motor and further improves the large one-step lateral excursion. These are mixed exploratory results, not accepted integrated repairs. Gate geometry/wrench and the material-slip counterexample are supported independently of autocal fitting. Force display differences are largely a first-visit telemetry artifact in the held D path.

All production edits were restored, native/Wasm rebuilt, and the clean source tree verified. Restored validation: 113 backend/loaded-state passes plus 3 friction passes, one skip. Preserve the earlier candidate's 5 failed checks. The attached supervisor stays at generation 1 / step zero, original layered configuration, with no changed-source flags. Source snapshots, patches and three native artifacts remain in the session. No long collection or calibration fit ran. Compute: approximately 887 s of full-world integration plus 12 one-second pulley subcases (about one CPU second to execute), under the 1,200 s numerical budget; builds/tests/analysis are separate.

Next decision: implement final-multiplier telemetry/feedback and an ideal two-port gate inside a combined path or coupled block; validate mechanical invariants and timestep behavior before selecting a production dynamics change or repeating autocal. This integration is proposed, not tested. The searchable abstract and report contain the tested hypotheses and references. Regenerate the catalog at turn end.

## Accepted steering: integrate three fixes and explain the motor candidate

The user requests three separate commits: (1) remove independent rolling slip, (2) fix tension display, (3) fix motor-load accumulation. They also request an explanation of sequential-pinholes timestep bias and a separate short report deriving and assessing the faster motor update. Implement the three changes across supported solver backends; retain the current motor update. Known prior loaded-checkpoint instability is evidence to investigate and report, not a reason to undo the explicitly requested material-law correction. Preserve the earlier research and rejected trial outcomes.

Integration budget: up to eight additional fresh numerical checks, 120 simulated seconds per case, and 600 seconds numerical compute; no autocal or live attached-world motion. Run focused invariants and backend parity for the changed code, preserving failures. Tests and builds are separate. Evidence/logs go in `integration/`; the current source tree starts clean at c6f0b5a1. Next decision: complete commit 1 and inspect force readback/transfer semantics before commit 2.

## Requested integration completed

Three separate commits now retain the requested changes:

1. `9ebc8337` — remove independent material slip at inertial rolling links, across JS/Python/Rust, with material-conservation and passive-bearing checks.
2. `41010aa1` — display accumulated constraint forces, including fixed compliant spans, and transfer final forces instead of intermediate peaks. Includes JS, Python, Warp and both Rust solve entry points.
3. `628a8963` — accumulate all signed motor-load multiplier corrections, count K/D once, and preserve small refinement terms. Adds dynamic-load and release invariants and the separate formulation report.

Both native and browser Wasm engines were rebuilt at each stage. Final validation: 188 Python parity/invariant passes, 110 Warp CPU passes (5 GPU skips), 20 JS passes, 115 native/Wasm passes (1 GC skip), and 5 known HP4 failures. The three legacy loaded-state settling failures remain identical to the isolated no-slip study; the two HP4 JS/Wasm tolerance failures remain. No thresholds were loosened and no old failure was reclassified as an accuracy success. One outdated assertion expecting zero force on a fixed stretched span now expects its analytic 40 N.

I01/I03 compare the ten-second native pull before/after commit 2: all 101 sampled physical states match exactly with force telemetry excluded. I02 preserves an intermediate transfer implementation. I04 repeats the final three-fix native pull: X=-6.77063 mm and Y=34.14493 mm at 10 s, matching the earlier E33 JS feedback candidate. Summed fresh integration compute is under ten seconds; builds/tests are separate. See [integration provenance](integration/provenance.json) and logs.

The separate [motor-formulation report](../../../../research/faster-motor-update.md) derives the predicted-pose correction, identifies the original update's local h*K resistance term, and distinguishes coarse-step errors from physical speed. Historical Y endpoints are not path length or a continuum reference. It also includes ten-second data showing that similar final endpoints can conceal different time histories. The alternative motor formula was not integrated.

Clarification of the gate sentence: the bridge experiment retained multiple separate cable constraints, each solved sequentially with finite iterations. Removing wheel DOFs shortened that chain but did not turn it into one combined gate/path constraint, and the motor/constraint splitting still depended strongly on timestep. This does not reject a properly coupled two-port gate.

The attached world was never advanced/reset; its old loaded binaries need a session reset/restart before live experiments can use these source changes. Next research decision: repair loaded-state stability and audit refined motor/cable trajectories with the corrected feedback; a combined gate/path solver and physically validated alternative motor update remain proposed work. The requested commits and short theory report are complete.


## Steering: browser Print Logo regression (2026-10-20)

User reports explosive HP4 motion after retained commits 9ebc8337, 41010aa1 and
628a8963. Previous acceptance was wrong: the loaded-state stability regression
was known and not resolved; the actual logo workflow was not checked.
Objective: recover stable production HP4 logo motion, preserve valid force
readback/motor feedback corrections, and enforce a short production stability
gate in normal validation and researcher instructions.
Hypothesis S01: disabling rolling-link redistribution exposes unstable dynamics;
the force display correction is observational and motor feedback does not affect
open-loop logo position motors. Test exact bundled logo with production scene,
layering and browser/native backends. Initial budget: 20 simulated seconds per
quick case, extend a successful case through the complete logo; no autocal
collection. Numerical trials record JSON; browser validation uses the actual UI.
Acceptance requires green logo and loaded-state tests, finite bounded body state,
no missed steps, and backend regression checks. No new result available yet.


### Stability correction progress

Accepted user steering: use headless production backends for experiments; browser
experiments should be rare and preferably human driven. One isolated browser
reproduction was already done (S01-browser, stopped at step 3874, body Z=.726 m);
its tab was closed. All subsequent experiments are headless.

- S01: rejected source 628a8963, native, default layered open-loop HP4 logo
  prefix: 79 missed steps, peak speed 11.689 m/s, rotation 50.526 rad/s.
- S02: archived c6f0b5a1 native engine on identical commands: no missed steps,
  speed 0.31565 m/s, rotation 0.02366 rad/s, COM Z 0.089911–.090475 m.
- S03: restore rolling redistribution only; retain force display and signed
  feedback corrections. Same measured prefix body trajectory and extrema as S02.
  Loaded-state tests at 1/2/4 substeps pass; all 117 initial backend checks pass.
- S04: complete 1,038,258-command logo replay in progress; extend numerical budget
  to 2076.516 simulated seconds and 600 wall seconds. JSON evidence without RRD;
  this is stability validation, no performance comparison or autocal collection.
- S05: fresh 30-second zero-buildup closed-loop torque pull, retained feedback
  with restored friction, finishes at [.004816,.107743,.093064] m. This confirms
  bounded movement only, not calibration accuracy or physical correctness.
- S06: new test rejects archived broken native engine at step 293 (.586 s),
  rocking .101506 rad/s > .1 threshold. It passes corrected native/Wasm 20 s,
  JS fallback 2 s, and loaded 1/2/4 substeps (6 checks, about 30 s on this host).
- Final focused validations: 157 backend/host tests; 196 Python differential/core;
  110 Warp CPU passes and 5 CUDA skips; 800 Jest passes and 1 skip; 4 existing
  logo substep tests pass. Rust native tests pass. A stale Jest expectation for
  zero displayed force on a fixed 100 N segment was corrected to 100 N, preserving
  the already requested final-multiplier display behavior. Initial stale-build
  and mistyped Python path failures are retained in integration logs.

Decision: withdraw unsafe slip removal (9ebc8337) across JS/Python/Rust. It is
not a safe production no-slip model. Do not replace it with extra default
substeps, loosen settling tolerances, or disguise the old slip as repaired
mechanics. The two valid bookkeeping changes remain. New automatic gate is
part of npm test and CI and mandatory in both AGENTS documents. Full logo
result and final revision are pending; do not call validation complete yet.


### Extended replay finding (S04), diagnostic retry (S07)

S04 reached step 1,038,238 of 1,038,258 and stopped on the diagnostic's fixed
5 mm print-plane bound: COM Z=0.0958588 m, speed=0.352224 m/s, rotation=0.0769501 rad/s.
This is in the last21 commands, after essentially the entire printing phase.
It is a failed full acceptance trial; do not claim complete-logo stability.
The public G-code ends with retraction/G92 E0 and G28 X0; the bundled CAN stream
then rapidly brings all motor targets toward zero. Whether this late excursion
is an existing terminal-command behavior or a regression needs comparison.
The committed short gate and all its limits remain unchanged.

S07 repeats the complete stream as a diagnostic, retaining finite/world bounds,
recording all terminal samples and a numeric checkpoint 500 commands before the
end. The same terminal state/commands will then be replayed using the archived
pre-change engine. This is an explicit terminal-command audit, not a passing
full-print result or a relaxation of the production prefix acceptance test.
Extended numerical budget: 1200 wall seconds total for the two full replays and
short tail comparisons;2076.516 simulated seconds per full replay. No RRD,
collection, firmware rebuild or browser experiment is needed.


S08: extend retained motor-feedback movement evidence to the four-substep native
configuration used in earlier HP4 autocal work: fresh zero-buildup closed-loop
world, two seconds settling then the same fixed torques, 30 simulated seconds.
Record JSON only. This is bounded movement/finite-state evidence, not calibration
accuracy. It is independent of the pending terminal-command comparison.

The actual `npm test -- --runInBand` invocation now passes: 6 mandatory stability
checks then 800 Jest checks, one skip. This verifies the automatic route as well
as the separately invoked suites.


S08 completed 30 simulated seconds at four substeps. Final COM
[.00109685,.36576645,.12582219] m; no divergence guard triggered. This is a
finite/bounded trajectory observation under the retained motor-feedback law,
not a claim of substep convergence or accuracy. Full torque-pull snapshots are
in S08-feedback-pull-4.json; preserve their visible timestep dependence.


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


## Pinhole gate implementation steering

Implement a real custom two-port attachment, not sequential replacement pins.
Use an opt-in selectable HP4 scene while preserving the production baseline.
The first model is ideal zero-flow-inertia, frictionless, with a single condensed
constraint for connected frictionless spans and explicit effective K/D. Preserve
port wrench and internal material length. Short headless tests only (subsecond
fixtures and <=2s fresh gate motion), plus the required existing stability gate.
No browser, no long autocal or full-logo rerun. Pending: no gate result yet.


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
