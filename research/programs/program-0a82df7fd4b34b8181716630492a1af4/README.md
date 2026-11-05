# Autocal robustness: results and recovery

**Production improvement adopted so far: none.** The first experiment rejected a
preload prototype. A separate lower-measurement-force candidate is now running;
its paired numerical results are pending.

## First experiment: preload under guide friction

Two isolated preload prototypes were tried. Neither recovered the tested
friction-stressed collection. Four active-guide trials failed before any fit or
anchor placement could be applied. These are collection failures, not accepted
placements above 20 mm.

The completed calibrations were:

| Case | Maximum anchor error | Every anchor below 5 mm | Points |
| --- | ---: | --- | ---: |
| Default, nominal | 3.841 mm | yes | 140 |
| Consistent bounded preload, nominal | 3.873 mm | yes | 140 |
| Default, matched noise-only control | 4.349 mm | yes | 140 |

A constructed native probe found modeled load torque with macroscopic local
cable slack. That raises a guide-transport/load-model concern; it does not prove
hardware failure or a physical root cause. The independent review supported
this scoped negative result as **report only**. No production code was adopted.

Read the [experiment report](workers/attempt-939c073f49004eb6949910600200be51/report.md)
for all per-anchor errors, failures, budgets, diagnostics and limitations.

## Second experiment: lower sensor measurement force

The fixed candidate uses `min(forceMax, 4 * forceMid)` instead of the current
`min(forceMax, 5 * forceMid)` during measurement. Pullout and preload forces,
settling checks, fitting and automatic stopping remain fixed. Earlier independent
research motivates this candidate; the new worlds do not choose the multiplier.

Fresh paired runs cover nominal motors, asymmetric motor Coulomb/stiction, and
encoder-read noise. Guide friction stays zero while its model is unresolved.
Adoption requires a meaningful applied-accuracy improvement on the declared
evaluation pair, no new calibration failure, and at most 20% extra simulated
collection effort. These hypothetical worlds cannot establish hardware reliability.

Assignment: `assignment-a64d3f287c374d4e8b0e697ee33afe68`.
Status: worker dispatched; no new numerical results yet. Checkpoints will be
committed and pushed after each collection, including unsuccessful trials.

## What happened to the workspace

Command access failed, then recovered. The original filesystem survived; no
file deletion was observed. The cause of the access failure is unknown.

All 407 original research files were recovered and archived, including raw
command/noise journals, solver histories, source snapshots, failed trials and
partial recordings. Every archived file was verified by SHA-256 readback.
Six live connection/control files were deliberately excluded.

The [archive and restore instructions](recovery-originals/README.md) explain how
to verify and recover the evidence from a fresh Git checkout. Both remote
research branches contain the originals. Subsequent experiments use remote
checkpoints instead of relying on a single end-of-turn save.

The [controller export](report.md) preserves the full decision and admission
history. This README is the concise entry point; generated controller records
are not a substitute for the measured experiment reports.
