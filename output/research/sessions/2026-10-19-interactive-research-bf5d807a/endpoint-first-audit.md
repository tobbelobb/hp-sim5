# Replace edge-force tuning with local pullout endpoint discovery

Status: code audit complete; proposed endpoint controller not tested.

## Abstract

We traced forceMax from automatic tuning through size discovery, sweep positioning, endpoint pullout, sampling, recovery, metadata and subsequent Python-driven collection. Sweep sampling already uses measured drive endpoint and start lengths to command position moves: it does not require a reusable geometric edge force. The automatic path currently repeats endpoint discovery at each actual geometry, making the initial global pullout largely redundant for endpoint selection. However, forceMax also caps measurement preload, supplies optional zero-tension force sweeps, affects uncertainty estimates and is automatically replayed into later collections. Simply deleting it would change those unrelated behaviors. We propose separating minimum useful preload, local measured endpoints and per-axis load limits. Discover endpoints with slow bounded position increments and all-motor load monitoring, then retreat to a measured interior state. Firmware provides rough motor-current-derived force reads through M569.8; telemetry fidelity and line-load interpretation remain unverified. Encoders alone cannot distinguish useful rigid-body travel from cable stretching or certify a safe boundary. No new physics experiments or production changes were made in this audit.

## Topics

forceMax, force_max_n, pullout endpoint, position control, max travel, tension limits, M569.8, autocalibration, HP3, HP4, uncertainty, replay.

## Tested hypotheses

| Claim | Evidence/method | Outcome |
| --- | --- | --- |
| Sweep position moves require a reusable max-force number | Source audit at d4a6fd75 | Rejected: moves use measured driveRange |
| forceMax is only used for initial pullout | Source audit | Rejected: preload, projection, recovery, replay and uncertainty also use it |
| Endpoint-first control safely reaches useful boundaries | No experiment | Not tested |
| M569.8 accurately captures limiting line loads in native/hardware | Firmware source only | Not tested |

## References

- [Audit and proposed replacement](endpoint-first-audit.md): call sites, migration and next experiment.
- [Notebook](research.md): accepted steering and scope.
- [Previous force fit report](force-fit-report.md): preceding measurements disproving a duration-independent fitted F80 selector.
- [Full current report](report.md).
## Usage audit

All paths below refer to repository source at d4a6fd75/cce7eddb. No source modifications in this turn.

| Consumer | What forceMax actually does | Endpoint-first treatment |
| --- | --- | --- |
| control/behaviors/force_tuning.mjs:578 tuneForce | Global pullout chooses forceMid, forceMax; returns home. Endpoint retained in metadata, not reused as sweep endpoint | Keep a small preload/movement-threshold calibration; remove global edge search |
| control/behaviors/sweep_data_collection.mjs:944 measureMaxTravelMm | Paired-drive search measures OTHER free-anchor travel to choose fixed-target offsets | Return measured travel/envelope; paired path is distinct from single-drive endpoint and cannot reuse scalar boundary |
| sweep_data_collection.mjs:263,838 prepareSweepPositioning | ForceMax caps free-motor preload while fixed anchors move | Separate positioning preload; check loads during movement |
| sweep_data_collection.mjs:1112 performForceSweep | Manual path pulls with capped force; automatic path searches local endpoint with controlledForceLimit | Replace endpoint discovery with position increments at actual fixed lengths/direction |
| sweep_data_collection.mjs:1192 onward | Captures drive, settles under preload, reads driveEndPointMm; totalDelta=driveStartPointMm-driveEndPointMm. Subsequent G1 H2 commands step inward | Retain this measured endpoint-to-start trajectory logic |
| control/primitives/uncalibrated_actions.mjs:291 resolveCollectionForce | Default sensor force=min(forceMax,5*forceMid); explicit sensorCollectionForce bypasses this cap | Explicit measurement preload and independent validated limit; do not derive from boundary exploration |
| uncalibrated_actions.mjs:352,451 | Fallback sensor preload and optional projectZeroTension ten-force sequence from forceMax down to forceMin | Give projection a dedicated measurement-force range |
| control/behaviors/slip_recovery.mjs | Halves forceMid, forceMax and sensorCollectionForce after fixed-anchor drift | Retreat/reduce endpoint envelope as well; drift is late evidence, not primary safety sensing |
| sweep_data_collection.mjs:1651,1727 | Local force replaces global max and is exported as force_max_n and pullout_force_n | Save endpoint, direction/configuration, probe load observations, limits and stop reason separately |
| autocal/_autocal_common.py:2675,2733 | Reads saved low/mid/max; injects them as CLI overrides plus --no-auto-tune-force on later collections | Version discovery metadata; do not turn endpoint-probe peak into a reusable force command |
| autocal/ellipse_cost.py:785,888 | Uses max-low force span as fallback flex uncertainty when zero-tension estimate unavailable | Use actual measurement-force range; probing force is not measurement flex |

The current automatic sweep path ALREADY discovers an endpoint in the actual local configuration. It receives the original tuning force_cap rather than forceMax, and overwrites forceMax with its own observed command. Thus tuning a single global edge force does not even eliminate the later local searches. Global tuning still influences forceMid and preloads, so it is not a literal no-op.

## Proposed replacement

Change the goal from “find max force” to “find a repeatable, sufficiently wide, load-bounded interval in this fixed-anchor configuration.” Return an observed endpoint state and drive interval, not an estimated infinite-force maximum. An exact geometric boundary is not required for useful calibration coverage.

1. Determine low slack-removal/measurement preload separately. Begin from a held, known interior state; preserve fixed-anchor encoder targets.
2. Set the drive to position control and free observation lines to low preload. Advance the drive slowly in small cable-length increments using existing uncalibrated G1 H2 support. Bound velocity, acceleration, step size and total travel/time. Position control by itself is not load protection.
3. At each increment check measured motion, ALL motor loads including held anchors, fixed-anchor drift, free-encoder response and bounded quietness. Stop before configured per-axis margins are consumed. A steep load rise per millimetre can trigger earlier retreat, but is supplementary to explicit limits. Do not deliberately slip a held motor to discover a boundary.
4. Store the last accepted encoder state. Retreat through measured positions to a tested interior endpoint, confirm preload/quietness and fixed anchors, then use the existing inward position-sweep logic. A percentage of prior travel is a candidate backoff, not proof of force margin; recheck loads after retreat.
5. Rediscover per fixed-length configuration and drive direction. Keep paired-drive fixed-offset discovery separate because it explores a different one-dimensional path; no global scalar max travel is transferable across these paths.

M569.8 supports multiple motor IDs; firmware ReadODrive3AxisForce uses measured motor current, torque constant, transmission, radius and mechanical advantage. Its own comments call the calculation very rough and assume origin spool buildup and no transmission losses. The returned value is mechanical-advantage-adjusted effective axis force; it is not automatically the highest tension in every cable segment. Need confirm mappings, signal sign, calibration, position-mode values and telemetry delay against native ground-truth line tension. Hardware current-derived estimates need conservative allowances. Enforce fast drive-level torque/current limits where available; polling and controller reaction distance alone cannot ensure prevention of load spikes. Held motors can still overload with a controlled drive.

If reliable load telemetry/limits are unavailable, encoder-only exploration cannot guarantee avoiding overstrain. Encoder motion may be elastic winding, and encoder quietness can coexist with body motion. A conservative user-specified travel envelope is then an honest fallback; guessing a universal force is not.

## Next bounded experiment (proposed, not run)

Focus only on endpoint discovery at the original force-tuning geometry: HP3 and HP4 Rust Native at requested10–25x. First validate M569.8 telemetry under held, moving and low-preload modes against per-step native motor and segment loads. Then compare a slow position-step search against current continuous pullout, recording selected travel, all-line peak tension, extension, drift, stopping overshoot and physical/wall duration. Test repeatability from identical states and different step rates; require selected endpoint stability rather than timing-dependent F80. Use declared machine-specific load bounds in simulation; these are experimental inputs, not inferred material ratings. No full sweeps or calibration runs are needed to settle the first controller question.

The endpoint formulation removes the unnecessary force/displacement extrapolation. It does not remove the physical need for load constraints or resolve missing telemetry. No endpoint controller has been implemented or validated by this source audit.
