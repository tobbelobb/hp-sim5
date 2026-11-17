# Rust cable core: wrong-result probe (degenerate geometry, non-finite, residuals, isolation)

Closed-form cable statics/dynamics, spool winding, degenerate geometry (zero radius, coincident anchors, zero rest, duplicate points) and world isolation/reset are numerically exact on revision c2450975, but one non-finite input coordinate silently poisons every body and path rest length while reporting exactly 0 N, and the opt-in layering `split` pass writes NaN path records for a rolling link without a finite radius. See [report.md](report.md) and [results.json](results.json).
