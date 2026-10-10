# Default HP3 settling recovery and supervised-job review

Status: complete; verified 2026-10-10.

Reviewed commit `383c6d453a2027fb99ebc7936862053878661e57` after fixing the
settling failure retained by [the implementer](autocal-jobs-verification.md).
Experiments E11–E12 continue the
[encoder-autocalibration session](../output/research/sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/report.md).

## Settling failure and fix

The failed default-force job `a792904dd547473ca0f6583f5ba547b9` timed out during
sweep endpoint pullout, before recording a measurement. Its last 1.5-second
encoder ranges were `[0, 0.186, 0.804, 0]` degrees. Fixed anchors held, but the
sensor was still moving; the ordinary quiet threshold was 0.45 degrees.
Raising the allowed drift would have admitted moving measurements.

Pullout now ends at its existing 120-second travel limit, then holds the drive
and applies sensor preload. The existing settling check must pass before the
endpoint is used and before any measurement is recorded. Only a typed settling
time-limit error triggers this transition. Fixed-anchor slip, transport errors,
stalled advances and wall-clock deadlines still propagate as failures. On
hardware, the wall watchdog allows the selected 120-second travel clock to
expire first. Point counts, noise sampling, planner, solver, ranking and
automatic patience/threshold stopping are unchanged.

E11 reproduced all 2,457 encoder readings and clocks exactly through the old
1332.834-second failure boundary. The patched run then collected 120 bootstrap
points and another 40 planned points, all with 16 noise samples, no recollection
and at most 0.042 degrees mean fixed-anchor error. It exited at the deliberately
chosen one-iteration experiment budget. This verifies settling recovery;
it is not an automatic-acceptance result.

## Review finding and correction

**P1, fixed: the new `layering=false` default was not applied to the physical
headless world.** The worker left layering unset and baked authored cable width.
`layeringEnabled` in `src/js/cable_joints_3d/ecs.js` defaults an unset resource to
true, while job status used `resource === true` and reported false. The firmware's
Q0 length conversion and layered physics therefore disagreed. Disabling runtime
flags alone would also require rebaking initial lengths. The original manual-force acceptance verifies
job lifecycle, not a valid no-layering calibration comparison. E11 deliberately
retained that geometry to isolate settling; its first fit had 558.69 mm anchor
RMSE despite completing collection.

Headless construction now follows the browser: zero cable width and `deriveAll`
when layering is off, with matching runtime flags. Startup uses the explicit
collector layering setting or the requested/firmware buildup. Layering changes
rebake and reset the world. Each generation preserves its baked scene and hash;
the final manifest identifies the current bake and retains the initial hash.

An adjacent existing reset bug also discarded closed-loop and layering settings.
The worker now preserves them. HP3/HP4 construction checks and live reset/toggle
checks cover the correction. The layered browser-parity fixture now specifies
its startup layering explicitly; its expensive layered collection was not run
in this no-buildup investigation.

The supervisor's structured settings, isolated job ownership, cancellation,
shutdown, crash cleanup and terminal classification were reviewed. No additional
blocking issue was found. Forty-two headless/job checks, five live lifecycle
checks, 85 collector behavior/primitive checks and 19 collector CLI checks pass.
Two HP3 replay regressions remain exactly equal. HTTP binding and MCP discovery
failed/timed out inside the command sandbox; required network-enabled retries
passed. These environment failures and the original physical failure are retained.

## Full default-force verification

E12 job `b32931beeb2c40d98ed1159aceb2a3c5` completed with the actual tool defaults:
HP3, known 30 mm radii, Q0, layering off, automatic forces and automatic stopping.
No acceptance threshold was changed. The SDK client denies its own socket and
connect syscalls after starting the external MCP server. The job returned in
0.003267 seconds. Its frozen no-layering bake hash is
`54267133f732ec8d52c8818e2b36c67a3b1178cbca808ae4ac7b0fb43f6e900e`.
The CLI accepted automatically after seven iterations, six collect decisions and
the final accept decision. It collected nine sweeps / 360 points with 16 noise
samples each, no direction recollection and at most 0.0608 degrees mean
fixed-anchor error. Applied anchors have 2.604 mm RMSE against nominal simulated
truth, measured from M669 rounded to 0.01 mm. History selection chose iteration
5; the first recovered fit was slightly better geometrically (2.384 mm), so
score-versus-geometry disagreement remains.

The job took 6,011.018 simulated seconds and 1,100.446 wall seconds, including
recording and computation. The attached HP4 world retained its identity and
step zero, and every registered child PID was absent at completion.
Independent RRD inspection verified all 360 measurements, seven complete
collections, 300,551 physical samples, the stop boundary at step 3,005,509,
twelve embedded stage artifacts, frozen scene/firmware hashes, recorded bake
hashes and exact final dataset/log content. The finalized RRD is 1,582,674,623
bytes. See [acceptance](../output/research/sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E12-acceptance.json)
and [recording audit](../output/research/sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E12-rrd-verification.json).

This is a complete workflow/geometry check, not a paired speed benchmark.
It used 360 points versus the supplied reference's 180. Reliable point-count
reduction is not established. Numerical underconstraint occurred at iterations
1 and 4; existing recovery and continuation handled it without restarting data
collection. No settling failure occurred with the corrected scene.

Reproduction commands (run in a network-enabled launcher or approved command
context for socket-dependent checks):

```bash
npm test -- --runInBand autocal/control/tests/behaviors autocal/control/tests/primitives autocal/control/tests/cli
npm test -- --runInBand tests/js/hp-sim-3d/headlessWorld.test.js tests/js/hp-sim-3d/noLineLayeringBakeTightness.test.js
.venv/bin/python -m pytest -q autocal/tests/test_headless_sim.py tests/python/test_autocal_jobs.py -m 'not slow'
.venv/bin/python -m pytest -q tests/python/test_autocal_jobs.py -m slow
.venv/bin/python -m pytest -q tests/python/test_research_agent.py -k stdio
.venv/bin/python autocal/tools/regress_calibration_logs.py --dataset-name fifth_hp3_dataset --dataset-name seventh_hp3_dataset --jobs 1 --color never
.venv/bin/python output/research/sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E12-supervised-verification.py
.venv/bin/python output/research/sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/E12-inspect-recording.py
```

This chat has no attached research connector; verification uses the real SDK
stdio MCP path. Evidence is simulated, with no hardware trial or full HP4
calibration. A new launcher session is still needed to check direct Codex or
desktop attachment. Existing score-versus-geometry selection limitations remain.

Commands, complete/failed traces, settings, source diff and results are retained
in the [session notebook](../output/research/sessions/2026-10-10-improve-encoder-based-cdpr-autocalibration-in-hp-sim5-the-curren-51aa157a/research.md).
