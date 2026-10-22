# HP4 sluggish torque response: implicit motor stiffness dominates a roller redistribution counterfactual

A controlled 30 s HP4 pull at fixed torques tested slow local rest-length propagation against motor-load numerical damping. Replacing local frictionless redistribution with an exact whole-path solve changed forward travel from 56.84 to 57.43 mm; two substeps reached 105.25 mm. Suppressing only the implicit motor stiffness term, retaining the implicit damping branch, reached 588.48 mm. This supports implicit motor/cable stiffness as the dominant bottleneck for this pull, rather than redistribution latency. The stabilizing term was removed only in an offline diagnostic, not deployed. A separate check found reported motor stiffness doubling from 36 to 72 Nm/rad when solver iterations increase from one to two. This is not an exact reproduction of the user's second half-sweep or a proof that disabling stabilization is safe.

Topics: HP4, rollers, cable transport, implicit motor coupling, numerical damping, substeps.

See [report](report.md) for hypotheses, history, and limitations.
