# Autocal robustness: results and recovery

**Production improvement adopted so far: none.** The first experiment rejected a
preload prototype. The lower-measurement-force continuation has four fresh attempts and a concrete development accuracy gain. Nominal transport failure blocks adoption, and the evaluation pair remains untested.

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

The earlier setup-only attempt was cancelled before collection and is preserved in [continuation-setup](continuation-setup/README.md). Fresh attempt `attempt-7b09fff3cc5b4bb89c33586f94cc4ebe` completed three calibrations and preserved one nominal candidate transport failure. Development applied maximum error improved3.696→2.853mm with6.41% extra simulated effort; all anchors in both completed development arms stayed below5mm. The nominal failure stays in the denominator and blocks force adoption. Evaluation calibration is NOT TESTED yet.

The manager required a controlled four-attempt partial because both evaluation jobs plus pushes and the10-minute report reserve could not conservatively fit the live deadline. Read the [partial report](lower-force/report.md), [raw evidence entry](lower-force/README.md) and [exact remaining-job handoff](lower-force/CONTINUE.md). Only the two unrun evaluation jobs and a separate bounded transport follow-up move to a fresh allocation. No successful primary run is repeated. All four raw checkpoints were committed and pushed before the next collection.

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
