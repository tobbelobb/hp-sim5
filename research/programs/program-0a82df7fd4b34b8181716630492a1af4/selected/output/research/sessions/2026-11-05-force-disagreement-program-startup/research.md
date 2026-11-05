# Program startup notebook

Program: program-0a82df7fd4b34b8181716630492a1af4
Date: 2026-11-05
Base revision: 865c9ab8aac926e258e9fe797141dd26e41e259e
Status: startup interview pending, no experiments commissioned.

## Objective and authority

The human asked to refresh research/prompts/autocal_parameter_variation_focus.txt
against earlier research, choose a question, launch a Research Program, obtain
concrete autocal results and grill them. They gave a wide budget and asked the
manager to use judgment. No merge or push requested.

Use HP3, known radii, zero buildup, disabled layering, one-click operation and
automatic stopping. Ground truth is available to construct/evaluate simulation,
never to the deployable measurement/selection policy.

## Inspected history and prompt changes

The updated prompt links the actual reports and retains their scope limits.
General profiling, native throughput, warm starts and consensus-spread stopping
are already investigated. Blindly fewer points is unsafe. The one-world premise
was false after the ten-world mass/gravity matrix. That matrix has no candidate
above 20 mm and cannot establish large-error avoidance. Force tuning is
nonmonotonic on one world. Global TRF/default fallback and unconditional
membership changes have regressions. Fresh centered-origin collection did not
save data. Solver-disagreement-driven fresh collection remains proposed.

Historical reports are inspected evidence, not new replications. Some source
harnesses named in the reports are absent in this checkout.

## Candidate explanations and first experiment

Main explanation: a new feasible sweep, perhaps at another force, exposes a
measurement difference between frozen L-BFGS-B and TRF fits and helps select or
refit better geometry.

Rival explanation: both solvers share systematic bias or weakly observed geometry.
They may agree or predict independent measurements well while retaining wrong
anchors. A force change might diagnose model error rather than choose a solver.

First cheap discriminator proposed, not commissioned: verify same-input solver
disagreement and predicted fresh-sweep contrast on saved HP3 prefixes. Use
synthetic controls to test scorer sensitivity, then collect only if the predicted
contrast is observable. Do not reinterpret retrospective used sweeps as unseen.
Compare to a matched ordinary extra sweep. Report applied geometry as well as
residual prediction; an oracle is evaluation context only.

## Proposed finite first-iteration budget

Not allocated or dispatched. Candidate envelope: 100 worker minutes, 20 review
minutes, 15 startup/manager minutes and 190 seconds cleanup. Up to 60 fresh
fit calls and 8 fresh collection jobs, each bounded at 3 million physics steps.
Target 4-6 independently constructed worlds rather than another mass/gravity
replicate matrix, contingent on verified native scene interventions. A worker
must declare sweep/point and simulated-time limits from the actual job before
collection. Use events routinely, none for fit-only work, native Rust and
supported speedup 25. This is an upper proposal, not an observed workload.

## Startup interview

Questions sent asynchronously. No answers have arrived.

- Practical priority among calibration accuracy, recovery robustness, runtime and
  explaining failures; claim the human most wants challenged.
- Priority among force robustness, solver-disagreement measurement selection and
  encoder-only geometry/scale limits.
- Useful anchor error and failure threshold.
- Real HP3 physical variations or failure cases to represent.

Pending assumptions are not human agreement. The initial task already authorizes
simulation and a manager-chosen finite budget, but the requested startup grilling
policy says to interview before commissioning. No workers have been dispatched.
The human may stop the interview without stopping the research program.

## Checks and next decision

Manager runtime_check returned ok=true. Cargo, rustc, LLVM tools, native physics,
firmware simulator, Python and Node dependencies passed. Worker-launcher socket,
loopback bind and local HTTP checks passed, wall time 0.022680779 seconds.
A separate agent-command scripts/research_socket_probe.py passed all three
checks in 0.021497377 seconds. No physics world started. These checks establish
tool readiness, not calibration accuracy.

Next decision: incorporate real interview answers, record the brief, choose
one finite discriminator and commission one worker plus a separate fixed-evidence
review. Keep the same program ID and live controller attachment.
