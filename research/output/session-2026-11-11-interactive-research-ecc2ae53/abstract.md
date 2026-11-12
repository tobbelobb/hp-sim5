# HP4 Print Squares corner bumps from saturated closed-loop servos under firmware flex pretension

The bumps are handovers between torque-saturated A/B/C servos: `M666 F1` flex compensation assumes one line per spool, but the HP4 scene has two (three for D), so it demands roughly double pretension (beyond 0.5 Nm). Halving the flex pretension removed the bumps (contour error RMS fell from 1.38 to 0.37 mm), making this plausible physics from a firmware/config mismatch, not an engine artifact.

[Full report and evidence](report.md)
