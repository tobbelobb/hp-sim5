# Paused before final review and integration

All six frozen primary attempts and one invalid startup are finished and pushed.
The lower-force default is not adopted because the nominal candidate failed.
The producer-tested encoder reply-budget patch is candidate `95a0af8f`; final
independent review and integration are pending at the human's request.

Read [resume state](CONTINUE.md) and [complete results](lower-force/report.md).
No more collection is authorized under the seven-attempt budget.

# Autocal robustness: results and recovery

Production force remains 5mid. The preload prototype and fixed 4mid adoption were rejected. All six lower-force primary attempts are complete. Development/evaluation accuracy gains did not overcome the original nominal workflow failure. A supported encoder reply-budget transport candidate is ready for manager fixed review and integration. No production merge is claimed here.

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

The earlier setup-only attempt was cancelled before collection and is preserved in [continuation-setup](continuation-setup/README.md). The first bounded continuation produced the four-attempt partial, independently reviewed as report only and preserved at commit 57658240. The fresh continuation then completed only the two unrun evaluation jobs. All six primary attempts are now recorded, and every raw checkpoint was pushed before the next collection.

Development applied maximum error improved from 3.6959 to 2.8529 mm at 1.0641 times simulated effort. Evaluation improved from 3.4116 to 2.7518 mm at 1.0709 times effort. All five completed calibrations kept every anchor below 5 mm. The nominal candidate's transport failure remains in the denominator and blocks force adoption. The separate encoder reply-budget patch has bounded real-WebSocket before/after evidence and relevant software checks, with no full-autocal-with-fix confirmation. Read the [combined report](lower-force/report.md), [raw evidence entry](lower-force/README.md) and [completed handoff](lower-force/CONTINUE.md).

## What happened to the workspace

Command access failed, then recovered. The original filesystem survived; no
file deletion was observed. The cause of the access failure is unknown.

All 407 original research files were recovered and archived, including raw
command/noise journals, solver histories, source snapshots, failed trials and
partial recordings. Every archived file was verified by SHA-256 readback.
Six live connection/control files were deliberately excluded.

The original archive is not stored in `main`; it is preserved on the
`research/autocal-originals-2026-11-05-0a82df7f` and
`research/autocal-recovery-2026-11-05-0a82df7f` branches. Subsequent
experiments use remote checkpoints instead of relying on a single end-of-turn
save.

The [controller export](report.md) preserves the full decision and admission
history. This README is the concise entry point; generated controller records
are not a substitute for the measured experiment reports.
