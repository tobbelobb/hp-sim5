# Stable stronger stepper motors at the 2 ms step

Simulated motors at 1 Nm (or anywhere from 0.25 to 4 Nm) no longer oscillate or spin away at the default 2 ms step: the servo's current-limit check and the open-loop detent torque had numerical artifacts, and fixing them made a gain retune unnecessary.

[Full report and evidence](report.md)
