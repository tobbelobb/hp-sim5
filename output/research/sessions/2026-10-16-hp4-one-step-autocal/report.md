# Restore HP4 single-step performance; autocal remains unresolved

Default physics integration is restored to one 2 ms step in JavaScript and Python. Four steps remain explicit opt-in and are not an accepted production fix. The exact browser autocal command was retried with only its dataset directory relocated: force tuning completed, then size tuning failed its unchanged 120000 ms encoder-stability deadline. During the last seconds of the pull, the effector still travelled approximately 2 mm/s horizontally with small rotation. This failure is not established to be the earlier corner-rocking mode. Focused tests pass. The original-command completion requirement remains unmet; no throughput recovery is claimed.

## Changes

Defaults are one in both composition roots. The one-step path skips extra rigid/cable component queries. Existing scheduler machinery remains available through the physicsSubsteps world resource. The loaded-state regression explicitly opts into four steps, and assertions cover the one-step defaults. README documents the CPU cost and opt-in setting. No collector tolerances, deadlines, or force protocol changed.

## Tested hypotheses

| Hypothesis | Trial | Outcome | Evidence |
| --- | --- | --- | --- |
| Single-step defaults and explicit substeps remain available | E00 focused tests | Supported | JS scheduler/setup 11 passed; Python scheduler 2 passed; Python composition 1 passed; loaded-state opt-in 1 passed |
| Original browser autocal completes with retained damping fixes at one step | E02 | Rejected | [Original-command log](browser/sweeps.full_auto.log) |
| Headless speedup-25 trial matches current browser wait semantics | E01 | Rejected by code inspection; trial interrupted | [Headless log](sweeps.full_auto.log), scripts/autocal_headless.mjs, primitives/headless_bridge.mjs and browser_clock.mjs |
| Original size-tune timeout is caused by sustained effector corner rocking | E02 recording | Not supported for the observed pull | [Body samples](browser-body.npy), [inspection script](inspect_rrd.py), [collector events](browser-events.json) |
| Four-step overhead has been recouped | None | Not tested; no claim | Four-step default removed |

## E02 method and result

Required launcher preflight passed; command socket probe passed with local socket access. Started an owned external recorder on 9877 and reloaded the existing stopped simulator tab. Ran the original --sim --machine-type hangprinter_4 --speedup 25 --extended-reference-ws ws://127.0.0.1:9877 command with dataset browser/sweeps.json. Force tuning selected idle 0.0223 N, start 0.4455 N, edge 3.3540 N. Size tuning timed out after 120000 simulation ms, ending around simulation time 1149.9 s. Autocal returned failure. The recording includes post-error held motion; those later stationary samples must not be mistaken for settling during the force pull.

At simulation time 1120–1121 s, horizontal y travel was about 2.08 mm; at 1140–1141 s it was about 1.65 mm. Rotation was small during these intervals. Thus accepting the pull as settled would be incorrect. This is simulation evidence only, not hardware validation or calibration accuracy.

The browser simulator was paused after the trial. Owned collector/firmware and recorder services were stopped. Earlier E01 artifacts remain preserved as an interrupted mismatched validation setup. The user's reference-005 files were not changed.

## Limitation and next decision

Restoring performance defaults is complete. Ensuring original autocal completes at one step is not complete. A suitable physics correction still needs to be identified and verified; four-step stability alone does not satisfy the user's performance constraint. This report supersedes the preceding session's default-four recommendation, not its measured opt-in stability result.

## References

- [Previous investigation](../2026-10-16-hp4-effector-instability/report.md): corner-rocking stress case and damping corrections.
- [E02 recording manifest](browser/hangprinter-20261016T111734.305415Z-517150c2.json): source recording and finalization details.
- [Working diff](working.diff): changes in this turn, based on 35f18b77.
