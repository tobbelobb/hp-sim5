# Program startup notebook

Program: program-0a82df7fd4b34b8181716630492a1af4
Date: 2026-11-05
Base revision: 865c9ab8aac926e258e9fe797141dd26e41e259e
Status: interview completed; first bounded investigation dispatched.

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

## Accepted interview answers and dispatch, 2026-11-05

Interview ended at the human's request. No further startup questions are needed.
The human ranks robustness across setups first, accuracy second, runtime third,
and explanation fourth. They want to challenge the claim of real-machine
transfer using modest USDA/noise/friction perturbations. Any anchor above 20 mm
is failure; all anchors below 5 mm is good performance. Friction has priority.
A 20% collection-time increase is acceptable for substantially fewer large errors.

These answers supersede the earlier provisional solver-disagreement emphasis.
The first commissioned investigation tests force/measurement robustness against
shared model bias. Solver disagreement is optional diagnostic work.

Question: question-0e6d4ed320fe44a793f3bc5eb35f9c34
Assignment: assignment-c6f56395495f43cfb1d1d5e03b01c593
Attempt: attempt-939c073f49004eb6949910600200be51
Allocation: runtime_allocation-2bf15473de9946e49587d6ae3077cab6

Actual allocation is 135 wall minutes: 100 worker, 20 separate review and
15 startup/manager margin, plus 190 seconds cleanup. Contract allows up to
12 fresh collection jobs, 80 fresh fits, 48 million physics steps and 4000
measurement points total. Each job is bounded at 4 million steps and 8000
simulated seconds. Worker must reserve final 10 minutes for summaries/cleanup.
Use one isolated worktree worker, no Viewer, routine event recording within
experiments, native Rust and speedup 25. No model or API credentials are needed.

No calibration result is available at dispatch. Holdout policy, loaded parameter
verification and full per-anchor reporting are required before scientific claims.

## External source inspected during collection

Piao, Kim, Choi, Moon, Choi, Park and Kim, 2019. Indirect Force Control of a
Cable-Driven Parallel Robot: Tension Estimation using Artificial Neural Network
trained by Force Sensor Measurements. Sensors 19, 2520.
DOI https://doi.org/10.3390/s19112520 . Full text
https://pmc.ncbi.nlm.nih.gov/articles/PMC6603654/ .

Sections 2.2 and 3.1 report pulley loading/unloading hysteresis and distinguish
bearing and sliding friction. Additional force sensors supply training/reference
data. This motivates treating our static capstan stress as a limited friction
model; it does not validate encoder-only autocal, determine our coefficient
ranges or support hardware accuracy of the program. No algorithm is adopted
from the paper. The separate IEEE 2025 conference abstract at DOI
10.1109/ICECER65523.2025.11401211 was found in search, but its full page failed to
load; it is not used to support a result.

## First measured results, unreviewed

Nominal numerical run completed automatically by spread-consensus after five
iterations, with applied A/B/C/D errors 3.840677/3.599241/3.463745/1.860538 mm.
Maximum 3.840677 mm, four-anchor RMSE 3.284983 mm. It used 140 points, 2091604
physics steps and 4183.208 simulated seconds. One intermediate cold fit was
catastrophically wrong, but warm/history selection rejected it.

Verified active-friction/noise dev1 default failed the settling guard before
points or fitting: 749017 steps, 1498.034 simulated seconds. Clean sensor-C
last-second range23.239717 degrees versus .3 degree quiet limit; added Gaussian
noise is too small to explain this failure boundary. This is persistent encoder
motion in simulation, not proof of effector movement or hardware failure.

One development policy uses the existing bounded measurement force
min(forceMax,5*forceMid) for held-drive preload, replacing2*forceMid. It passed
the first failed boundary and began collecting points. It has not yet completed.
No guards or stopping logic were relaxed. Policy and nominal/evaluation controls
remain pending, as does separate fixed-evidence review.

Evidence: ../attempt-939c073f49004eb6949910600200be51/evidence/measurements.json
and dev1-failure-diagnostic.json. No scientific admission has been applied.
