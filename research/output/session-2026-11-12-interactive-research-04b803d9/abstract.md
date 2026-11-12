# HP4 logo path error: two firmware flex errors and an unconverged HP4 solver

HP4's 0.39/0.62/0.69 mm logo path error comes from three causes: flex compensation that ignores MA² (over-reeling 2×/4×), a +0.16 mm Z belief created when `M666 F1` follows `G28`, and a 2-iteration HP4 solver whose biased static equilibrium adds the XY offset. With the two firmware errors removed and a converged solver, error falls to 0.07/0.13/0.22 mm, on par with HP3.

[Full report and evidence](report.md)
