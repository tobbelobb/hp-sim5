# Native HP4/RRF collection

The researcher launcher owns one continuing Python world, a private RRF
simulator and the production WebSocket bridge. Collection uses
`autocal/control/behaviors/sweep_data_collection.mjs`; it does not synthesize
records from motor targets. RRF plans G-code and converts force to torque, Python
executes those motor records, and the collector observes actual encoders.

Call `capabilities()` first, then `runtime_status()`. For example:

```json
{"configs":[{"fixed":[2,3],"drive":0,"sensor":1}],"options":{"sweepPoints":6,"noiseSamples":16}}
```

Pass that object to `collect_sweeps`. Indices 0–3 correspond to physical spools
A/B/C/D and CAN addresses 40–43; firmware movement axes are X/Y/Z/U. HP4 always
keeps anchor 3 fixed. The production collector performs both physical drive
directions and stores them in canonical drive/sensor orientation. The default
six points per direction produce twelve measurements in one combined sweep.

The default collection uses fixed deltas `0,0` mm, forces low/mid/max of
0.01/0.05/0.5 N, and a 1 N measurement sensor preload. The larger sensor preload
avoids the stationary sensor observed with the original 0.25 N default on this
authored scene. Overrides are explicit in `options`: `fixedTargets` (comma-separated
millimetres), `feed`, `forceLow`, `forceMid`, `forceMax`, `sensorCollectionForce`,
`noiseSamples`, `returnToOrigin`, `projectZeroTension` and `preserveBuildupFactor`.
`sweepPoints` accepts 3–100; 1–12 configurations can be collected together.

## Clock and completion

Physics uses the RRF bridge's 0.002 s buckets. A world survives positioning,
force transitions, settling, noise sampling and subsequent calls. The runtime
advances when the collector waits or a caller requests `step_physics`; it does
not consume CPU while idle. This makes simulation time independent of Python
throughput. A requested delay rounds up to whole timesteps; the default 25 ms
noise interval becomes 26 ms and its reported effective rate reflects that.

Empty-axis encoder requests acknowledge delivery without advancing physics.
Real reads drain queued commands before returning angles. `send_gcode` returns
the firmware reply; use `M569.3 P40.0:41.0:42.0:43.0` to wait for motor playback
and observe actual motion. Queue completion alone does not prove settling.
The existing collector's stability/vibration windows and tolerances are retained,
with a default 30 s simulation-time deadline (`settling_timeout_s`, range 1–120).
Collection also has a wall-time budget and queues have a 60,000-command bound.

The WebSocket adapter returns the first mapped raw, unwrapped encoder angle in
degrees, exactly as the browser endpoint does. No diagnostic offset is subtracted.
The existing bridge owns encoder references; the collector's `raw_angles_deg`
therefore retains its existing reference-relative meaning. Experiment telemetry
separately names raw and offset-adjusted diagnostic angles.

## Evidence and lifecycle

`collect_sweeps` returns absolute paths to the collector's version-2 JSON, a
finalized RRD, frozen authored scene and firmware configuration, an immutable
command/sensor trace, and a manifest. The manifest records step ranges, hashes
of loaded sources/firmware/inputs, package versions, effective collector options,
validation and wall time. A trace includes earlier session actions so replay can
reconstruct the world; byte offsets identify the individual collection.

Rerun records at 10 simulated Hz and at encoder observations. It batches output
for up to five wall seconds and streams to the launcher's Viewer when enabled.
Each collection finalizes its own RRD without replacing the physics world.
Later collections cannot change already returned evidence.

`reset_session` archives the old world and restarts world, firmware and bridge
references together. A failed collection saves evidence and requires that reset.
`runtime_status` reports stale source changes: restart the launcher to load changed
Python physics; reset the session to reload changed JS bridge/collector code.
`start_browser_service` supervises Vite on a free loopback port. Its ordinary web
scene is independent of the native world; native visualization uses Rerun.

## What explicit doctor proves

`./hp-sim5-research-agent --doctor` explicitly runs the collection proof. Normal
prompt launches prepare services without collecting data; the agent chooses
whether and when to call `collect_sweeps`.

Doctor performs actual collection and checks version-2 schema, raw-angle presence,
canonical roles, angle-to-millimetre conversion, noise statistics, drive travel,
physical sensor response in each sub-sweep, and finite autocal residual evaluation.
The short six-point proof does not identify anchors or radii. The current solver
requires more points and multiple independent sweeps; its optimizer `success`
flag alone can coexist with an underconstrained score of 100 and no valid sweeps.

Use full independent sweeps/configurations for training and holdout evaluation.
Compare fitted radii with the USD spool radii and disclose the distributed
anchor/pulley geometry when comparing effective point-anchor fits. Do not treat
firmware M669 guesses as authored scene ground truth. The browser/native replay
check exercises production browser physics and its encoder endpoint under the
same real firmware schedule; it does not test browser rendering or an independent
wall-clock collection.

The [verification report](native-collection-verification.md) links actual
collections, browser replay, Viewer inspection and fitting limitations.

Klipper streamed stepper/trapq motion and hardware control are outside this adapter.

```bash
.venv/bin/python -m pytest -q tests/python/test_native_collector.py
.venv/bin/python -m pytest -q tests/python/test_native_collector.py -m slow
node tests/parity3d/collector_replay.mjs SCENE_USDA EVENTS_JSONL OUTPUT_JSON
```
