# Proposed motor-friction discriminator

Status: not launched or tested.

The guide-friction study did not deliver an autocal improvement. It found a model boundary that limits interpretation of those failures. Test the existing motor-friction mechanism independently, with all guide coefficients zero.

Use native Rust HP3, known 30 mm radii, zero buildup, no layering, current production collection defaults, sequential jobs, no routine recording and preserved command/noise journals. Verify modified ECS fields and native readback after a normal first step. Pin Node 23.11.1 in every shell. Do not manually prepare the native engine.

Declare the worlds before numerical work:

| World | Split | Coulomb factors A/B/C/D | Stiction factors A/B/C/D | Encoder sigma deg | Seed |
| --- | --- | --- | --- | ---: | ---: |
| motor-dev | development | 2 / .5 / 1.5 / 1 | 2 / .5 / 1 / 1.5 | .03 | 4101 |
| motor-eval | evaluation | .5 / 1.5 / 2 / .75 | 1.5 / 2 / .5 / 1 | .05 | 4201 |

These are proposed hypothetical perturbations, not hardware measurements. Recompute effective defaults per motor before multiplication. Stock effective values reported in the first study were .001 Nm Coulomb, .0015 Nm stiction, 1 rad/s stiction speed and .000011 windage coefficient at .5 Nm holding torque and .011 damping. Native NaN fields select fallback formulas; they are not zero. Preserve windage, stiction speed and cogging.

Test current defaults first. If development fails, choose at most one bounded force/measurement change using noisy encoders and automatic tuning. Freeze it before inspecting evaluation-world accuracy. Do not use test truth, weaken guards or change solver defaults. Include a nominal regression control if retaining a candidate.

Primary endpoint is any final anchor above 20 mm or collection failure. Good performance requires every anchor below 5 mm. Report each anchor, maximum, candidate tails, acceptance, points, sweeps, steps and simulated time. Permit up to20% extra collection time only for a demonstrated reduction in failures; compare completed pairs, not a completed run to an early failed timeout.

Proposed finite allocation:75 worker minutes,15 review minutes,10 startup/manager minutes,190s cleanup; at most5 collections,60 fresh fits,4Msteps/8000sim s per job,20Mtotal steps and1400points. These are unallocated proposals, not spent or hard aggregate tool quotas.

A positive delivery requires a supported production change and relevant checks, plus separately reviewed paired results. A negative result must identify the failed hypothesis and exact scope. A passing small matrix is not a hardware reliability guarantee.

Preserve and push a small checkpoint after every completed experiment. Do not wait until the entire research turn ends.
