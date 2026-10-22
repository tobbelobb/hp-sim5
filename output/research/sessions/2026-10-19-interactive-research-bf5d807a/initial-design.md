# HP4 force tuning failure and controlled pullout design

Status: in progress.

## Abstract

Offline analysis of the supplied HP4 simulation log finds that the force search reached its 20 N cap after 13 edge probes, rather than exhausting its 120-trial limit. All probes reported no stall. Incremental compliance fell to 17.3% of its previous peak at 18.0389 N but recovered to 84.9% at 20 N, preventing the required two consecutive declines. The failed search selected a hardcoded 1 N fallback. Size tuning then measured 6.003 mm raw travel and selected 4.802 mm; sweep pullout repeatedly approached its 600 simulated second limit. A proposed replacement uses controlled displacement and short, bounded force probes to separate inertia from load growth. No new physical simulation or hardware experiment has validated that replacement. Tool checks passed but establish neither calibration accuracy nor parity with the supplied browser world.

## Topics

HP4, Hangprinter, autocalibration, force tuning, momentum, incremental compliance, encoder stability, workspace coverage.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Trial count exhaustion caused the supplied failure | E1 | Rejected: 13 of 120 trials; force cap reached | [Log analysis](E1-log-analysis.json) |
| Two consecutive compliance declines occurred in the supplied curve | E1 | Rejected | [Log analysis](E1-log-analysis.json) |
| Controlled displacement improves edge detection and coverage | none | Not tested | [Design](report.md) |

## References

- [Report](report.md): diagnosis and proposed procedure.
- [Notebook](research.md): constraints and next decision.
- [Supplied simulation log](../../../extended-autocal/no-buildup-reference-008/sweeps.full_auto.log): original commands, probe measurements, sizing and waits.
- [Earlier HP3 investigation](../../autocal-comfortable-force-20261010/report.md): finite-duration compliance method and disclosed inertia limitation.
- [Force tuning source](../../../../autocal/control/behaviors/force_tuning.mjs): search windows, caps and fallback at revision 3c30e6b9f8d5dc4de15b188448b0b8e5b68ea79d.

## Diagnosis

The search uses repeated excursions from the origin, each observed for 30 simulation seconds. This measures a dynamic response; it does not isolate equilibrium compliance. The log alone cannot identify how much of the final rebound comes from inertia, changed geometry, residual motion or observation timing. Absolute encoder travel is also not proof of Cartesian workspace coverage or safe cable strain.

The hardcoded cap is 20 N, and the failure branch substitutes max(forceStart, min(1 N, cap)). The supplied forceStart is approximately 0.4455 N, so collection receives 1 N. Sweep pullout additionally uses min(forceMax, 20 * forceMid), so merely increasing forceMax would eventually encounter another policy limit. A force measured with one fixed anchor is transferred across different sweep geometries. Held-motor drift is checked, but cable material safety is not established by that check.

## Proposed procedure — not tested

1. Keep the existing low-force movement/noise probe as an initial scale estimate, with bounded duration. A failed movement probe must produce a diagnostic failure rather than guessed collection forces.
2. Explore each required fixed-anchor/drive configuration separately. Advance the drive in small, slow position increments with bounded acceleration, maintaining a modest sensor preload. Record all encoder changes and held-anchor drift. First verify that production firmware can impose a useful torque limit while position control is active; do not assume this capability.
3. At each held position, perform a short ramp of force pulses, with position holds between pulses to remove accumulated momentum. Bracket the force that produces significant motion in the intended direction, confirmed by another free encoder. Use actual elapsed selected-clock time and reject large velocity or residual motion. Return to the held position after each pulse. Force pulses remain bounded by an independent machine load limit. If torque-limited position control is unavailable, this held-position/pulse sequence is the alternative actuator strategy, not permission for unlimited position moves.
4. Build the local required-force versus achieved displacement curve. Seek sustained growth in force per useful displacement, normalized against the interior response. Confirm a candidate with repeated low-speed probes and a lower approach speed. Back off to the last confirmed interior point; retain its position and force as a pair. Do not infer a universal numeric knee threshold from this single failed curve.
5. Reuse that measured endpoint directly for the sweep. Do not repeat a long constant-force excursion and wait for equilibrium. Settle only after holding the endpoint and applying measurement preload. Reject endpoints that violate held-anchor drift or fail to provide a useful sensor-length span.
6. Treat load limit, no knee before limit, motion timeout, fixed-anchor slip and insufficient coverage as distinct terminal outcomes. Preserve all observations. No knee is inconclusive, never an instruction to substitute 1 N or label the force cap a measured edge.

A machine-specific upper load bound remains necessary: commanded torque and encoder motion cannot universally distinguish useful travel from cable stretch, pulley compliance, slipping or damage. The geometric force asymptote motivates looking for declining useful motion, but the physical workspace also depends on gravity, preload, direction and which lengths are held. Approaching it need not produce the same observable curve in every sweep. Simulation should measure actual individual line forces and strain as validation signals; those must not leak into the encoder-only controller being evaluated.

## Next experiments — not tested

First capture matched HP4 settings from the browser run; the attached fresh supervisor currently has layering enabled and config_hp4_w_line_layers.g, whereas the supplied log starts config_hp4.g. Do not claim matched reproduction from these tool checks. Establish actuator capabilities before coding the controller. Then compare original and proposed methods on matched fresh HP4 worlds, followed by HP3 and changes in mass, damping and cable stiffness. Measure useful workspace coverage, maximum and residual strain, held-motor slip, steps, wall time and selected-clock time. Vary probe speed and duration to test whether chosen endpoints depend on momentum. Record sampled physics for initial validation and none for later numerical sweeps. No numerical or hardware improvement is claimed yet.
