# HP4 sluggish torque response: implicit motor stiffness dominates a roller redistribution counterfactual

A controlled 30 s HP4 pull at fixed torques tested slow local rest-length propagation against motor-load numerical damping. Replacing local frictionless redistribution with an exact whole-path solve changed forward travel from 56.84 to 57.43 mm; two substeps reached 105.25 mm. Suppressing only the implicit motor stiffness term, retaining the implicit damping branch, reached 588.48 mm. This supports implicit motor/cable stiffness as the dominant bottleneck for this pull, rather than redistribution latency. The stabilizing term was removed only in an offline diagnostic, not deployed. A separate check found reported motor stiffness doubling from 36 to 72 Nm/rad when solver iterations increase from one to two. This is not an exact reproduction of the user's second half-sweep or a proof that disabling stabilization is safe.

## Results

| Variant | Substeps | Forward position after 30 s, mm |
| --- | ---: | ---: |
| Current production reference | 1 | 56.8352 |
| Exact frictionless whole-path redistribution | 1 | 57.4268 |
| Current production reference | 2 | 105.2451 |
| Negligible implicit load stiffness, implicit damping retained | 1 | 588.4799 |

Inputs: stock HP4, no layering, closed-loop position motors, D held, torque targets A=-0.000334089 Nm and B/C=-0.037733 Nm applied at 2 s. Only the named mechanism changes. The baseline agrees with the preceding native experiment's 30 s y position within 3e-15 m. Counterfactuals use the production JavaScript reference for access to individual systems; native/Wasm source contains the same motor and redistribution formulas. [Raw results](results.json), [harness](compare.mjs), [provenance](provenance.json).

The global variant verifies that all internal links are frictionless under the current model, conserves each path's total free rest length, solves equal absolute extension across taut segments with nonnegative rest lengths, and distributes slack proportionally. It replaces four local passes with their frictionless whole-path fixed point in the same pipeline slot. This is a diagnostic of local redistribution latency, not a general roller/friction implementation.

The stiffness variant retains a tiny positive stiffness (1e-30) to keep the original implicit damping branch and numerator. It suppresses only the dt²*K/I contribution. An initial trial used zero stiffness, accidentally bypassing the damping branch as well; that result is preserved as no-implicit-load-legacy.json and excluded from the comparison. No accuracy, stability, energy or performance benefit is claimed for shipping this bypass.

## Mechanism

TorqueModeSystem computes a motor angular-velocity update divided by 1 + dt*D/I + dt²*K/I. The timestep-dependent stiffness correction produces strong artificial slowing in this split motor/cable integration. With measured K=72 Nm/rad and authored rotor I=5e-5 kg*m², its dt²*K/I term is 5.76 at 2 ms versus 1.44 at 1 ms. The existing real damping, load torque and other terms remain relevant; these numbers are not a full motor transfer function.

A 3 s probe measures K=72 with the stock two solver iterations and K=36 with one. Source inspection confirms physical stiffness/damping coefficients are added inside each joint solve on every iteration. Torque forces are also accumulated there and need to be considered when correcting the coupling; blindly deleting the denominator is not a validated fix.

The friction pass budget is four per 2 ms tick either way: four at dt=2 ms, or two per 1 ms substep. Smaller substeps interleave geometry, constraints and motor updates; their benefit cannot be attributed simply to a larger count of redistribution passes.

## Tested hypotheses

| Hypothesis | Experiment | Outcome |
| --- | --- | --- |
| Local rest-length propagation is the dominant cause in the controlled pull | E01 global versus baseline | Not supported; only ~1% travel increase |
| Implicit motor stiffness is a major cause of sluggishness | E02 stiffness-only counterfactual | Supported for this pull; ~10x forward travel |
| More substeps merely execute more friction passes | E03 source budget check | Rejected for one versus two substeps |
| Reported stiffness is independent of constraint iteration count | E04 3 s coefficient probe | Rejected: 36 versus 72 Nm/rad |
| A proposed production correction preserves energy/stability and full autocal behavior | None | Not tested |

## Cheaper directions

1. Audit and remove iteration-dependent duplication of physical load coefficients; count physical connections, not visits. Keep torque-force bookkeeping consistent and test the existing energy/force invariants.
2. Derive the motor's effective load from the freely sliding path, rather than treating the adjacent cable segment as a spring against fixed neighbors. For an ideal fixed-geometry frictionless series chain, compliance adds: k_eff=1/sum(1/k_j), and angular stiffness is r²*k_eff. Moving bodies and shared motor paths require coupled Jacobians; this formula is a starting point, not a replacement implementation.
3. If needed, use a small coupled rotor/path solve or a better time discretization instead of substepping the entire world. Validate energy and transient response before performance claims. Exact redistribution alone is low priority for this symptom based on E01.

## Existing attempts in history

[Commit details](history.txt): aa070d36 (2026-05-06, Looking slightly better?) introduced the implicit motor stiffness/damping denominator. 6e688850 (2026-04-30) introduced PBDCablePathSolver, including whole-component frictionless redistribution; 2657fb62 changed it back toward earlier behavior the same day, restricting transfer to pinholes and forwarding driven rest-length changes. 83d79e59 (2026-04-28) explicitly prevented slack from crossing rolling links in that older path. 9c988bba (2026-05-06) corrected redistribution to equal absolute PBD extension instead of strain ratios. 8d3914b9 (2025-05-01) replaced a torque/inertia transfer approach with direct delta-length to delta-angle coupling. These earlier approaches should not be blindly reinstated.

## Current run logs and limits

Reference-006 full_auto_2.log reaches both 600 s pullout limits, then fails a 30000 ms wait. The extended waits cover pullout and immediate endpoint preload only; ordinary per-point preparation, movement and fixed-anchor waits retain ordinary budgets. The log lacks a call-site marker to identify the exact remaining wait. This investigation did not raise any timeout.

Reference-007 full_auto.log ended during force tuning with Message not received. It does not establish two-substep sweep settling success. No browser, recorder, firmware or ongoing user process was controlled in this investigation. No production files were changed. No end-to-end full-autocal run was performed.
