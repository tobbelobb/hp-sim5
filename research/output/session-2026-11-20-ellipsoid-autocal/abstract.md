# Ellipsoid autocal: sweeps still win; the held anchor's height trades against stretch scale

[Simulation evidence](report.md): on error × time with disturbances, flat sweeps with the point fit beat every sphere-mesh plan. The remaining 1–2 mm comes from the free lines' stretch correction being 10–20% off: it trades exactly against the held anchor's height, and the fit rms cannot see it. Two collector bugs were fixed along the way: relative moves after motor saturation, and held lengths that double-counted line drift.
