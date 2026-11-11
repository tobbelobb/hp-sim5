# Hangprinter flight recorder

Run every command below from the repository root. Install the Python dependencies
with `.venv/bin/python -m pip install -r requirements.txt`; they include OpenUSD,
Rerun 0.38 and the browser receiver's WebSocket library. The main
[README](../README.md#python-dependencies) describes creating `.venv`.
Build the native Rust engine with `npm run build:native` (Rust, C compiler and
Node development headers; the default PGO build also needs
`rustup component add llvm-tools-preview`). Browser recording uses the Wasm build of the same core.

Both recorders batch raw telemetry into `send_columns()` calls, keeping every
requested sample and its explicit simulation clocks. Batches flush at 128
observations, 65,536 component instances or one second of active logging. Live
viewers also receive partial batches while idle. Resets, removals and finalization
flush pending data first. Column writes carry explicit simulation/wall timelines;
they do not add the SDK's automatic `log_time`/`log_tick` timelines.
Direct users of Python `RerunSystem` must call its `flush()` before flushing or
disconnecting their `RecordingStream`; the CLI and collection services do this.

There are two recording workflows:

| Workflow | Physics runs in | Start recording with | Output |
| --- | --- | --- | --- |
| Browser print | Rust/Wasm in hp-sim-3d, with JS application systems | Receiver process, then the **Rerun** button | A new `.rrd` for each connection/scene generation |
| Headless machine | Native Rust hosted in Node; Python records observations, without Vite | `run_experiment(..., record=True)` or the research MCP tools | One `.rrd`, plus a final JSON snapshot |

Running `.venv/bin/rerun FILE.rrd` **opens an existing recording**. It does not
run physics, create a recording or connect to the simulator. Recording happens
only when you explicitly run one of the workflows below.

## Headless simulation

Headless recording uses the same native Rust world as the browser without Vite.
Python owns the observation mirror and Rerun recording; it does not advance a
second physics implementation.

```bash
PYTHONPATH=src/python .venv/bin/python - <<'PY'
from hp_sim5_research.experiments import read_run, run_experiment

manifest = run_experiment('.', steps=200, record=True)
print(manifest['artifacts']['rrd'])
print(read_run('.', manifest['run_id'], 0, 5)['samples'][-1]['effectors'])
PY
.venv/bin/rerun output/research/experiments/*/recording.rrd
```

The `steps` argument selects the number of updates. Commands are scheduled one
per unpaused timestep and use motor radians plus metres of extrusion:
`{"type":"Move","A":0.1,"B":-0.05}`, `{"type":"SetTorqueMode","axis":"D","torqueNm":-0.01}`,
`{"type":"SetPositionMode","axis":"D"}`, `{"type":"Add to reference","A":0.01}`,
and `{}` to hold targets for one step. The headless worker does not parse G-code;
use the browser uploader or firmware command players for G-code prints.

The recording includes the initial state and every completed step, with live
rigid-member hierarchies, cable lengths/forces, motor and encoder state, tool
points and extrusion deposits. `sim_time` and `sim_step` advance on positive
unpaused updates; scene resets restart them and set the `scene_generation`
timeline. Use a separate recording stream/file for independent scene runs.

Native visuals use straight constraint spans, points and frame axes. Stored
intermediate wraps remain in length traces; browser sag and wrap tessellation are
presentation differences. Each scalar/segment has a stable plot path. Entering
torque mode clears only the commanded/error traces and preserves actual/geometric
series identities.

## Browser simulation

### Record a logo print yourself

1. Start Vite in one terminal with `npx vite`, and open
   <http://localhost:5173/hp-sim5/hp-sim-3d/>. The default machine is HP4.
2. Start the receiver in another terminal:

   ```bash
   .venv/bin/python scripts/hangprinter_flight_recorder.py
   ```

3. Open the **Rerun viewer URL printed by the receiver**. The URL includes the
   connection to the local data server; opening port 9090 without that query opens
   Rerun's welcome screen.
4. In hp-sim-3d, expand the controls with **▼**. Click **Reset**, then **Rerun**;
   wait for **Rerun: recording**, then click **Print Logo**. Connecting alone
   does not start a print. Watch the receiver's `Recording: output/rerun/...rrd`
   line: it identifies the file being written.
5. Try **Pause** and resume, or **Finish ASAP**. The recorder captures every
   completed physics step, so playback speed changes wall time rather than
   sample spacing. Click **Rerun** again to finish this connection. Ctrl-C stops
   the receiver and flushes its recordings.

Replay the **exact path printed by the receiver** with `.venv/bin/rerun PATH.rrd`.
The receiver can stay running for another print. Resetting/rebuilding the browser
scene starts a new recording file; it is not appended to the previous timeline.
Multiple loaded machines are included in each timestep.

To connect automatically, open:

```text
http://localhost:5173/hp-sim5/hp-sim-3d/?rerun=1
```

For another telemetry port, use `--port 9887` and
`?rerun_ws=ws://127.0.0.1:9887`. The viewer's HTTP and gRPC ports are configurable
with `--web-port` and `--grpc-port`. `--no-viewer` records to disk only.

Each browser connection and scene reset creates a separate `.rrd` file in
`output/rerun/`; use `--output PATH` for another directory. Replay a saved file:

```bash
.venv/bin/rerun output/rerun/hangprinter-RECORDING.rrd
```

The live viewer and saved file receive the same recording through Rerun's
[multiple sinks](https://rerun.io/docs/concepts/logging-and-ingestion/sinks).
Its default layout has a 3D view and synchronized length, error, and force plots.
Select `sim_time` to scrub in seconds or `sim_step` to select a precise timestep.

`--output` on the browser receiver means an **output directory**, unlike the
native CLI's single output file. Defaults are telemetry WebSocket port 9877,
viewer HTTP port 9090 and viewer gRPC port 9876. A port-in-use error means another
receiver/viewer is already using that port; stop it or choose another port with
the corresponding option. The simulator's Rerun button must use the receiver's
WebSocket port, not its viewer HTTP/gRPC port.

## Inspecting the recording

### Finding the effector and motion

Both workflows record an **orange effector center point and coordinate axes**,
not a solid effector mesh. The point radius is 0.008 m and the axes are 0.15 m
long. In a whole-machine view the marker can be small or overlap nearby members.

In the native default HP4, look for `world/machines/default/effector` in the
entity tree, including its `shape` and `axes` children. Browser machine IDs differ;
look under `world/machines/MACHINE/effector`. Select that entity, check that it is
included/visible in the 3D view, and zoom around the cable convergence point.
Rerun's view **?** control lists navigation gestures. Its
[viewer guide](https://rerun.io/docs/getting-started/configure-the-viewer)
explains selection, zoom and restoring the view. Temporarily hiding the cable
force arrows can make the point easier to distinguish.

For a close view, select the 3D view and edit its entity query to include only
these native paths (restore the original query for a whole-machine view):

```text
+ /world/machines/default/effector/**
+ /world/machines/default/tool_points/**
+ /world/machines/default/extrusions/**
```

The [Rerun navigation guide](https://rerun.io/docs/getting-started/configure-the-viewer/navigating-the-viewer)
describes editing a view's query. Keep its origin at `/world` so the positions
remain in the machine's world frame.

Select `sim_time` or `sim_step`, rather than the logging/wall-clock timeline.
Compare the beginning and end with the timeline cursor. For the native ramp,
expect millimetres of travel, ten extrusion points and changing angle/length
traces. Playback of a saved file is independent of the simulator and can be
scrubbed repeatedly. A 0.4-second idle recording is a construction/settling
check, not convincing evidence of commanded movement.

### Available telemetry

| Data | Browser receiver | Native recording |
| --- | --- | --- |
| Anchor/body/member frames, effector center/orientation | Yes | Yes; live parent/member hierarchy |
| Cable spans and per-segment force arrows/plots | Yes | Yes |
| Rest/commanded/geometric lengths, error and stretch | Yes | Yes; stable path for each scalar |
| Slack sag and intermediate wrap tessellation | Yes | Straight constraint spans; stored wraps remain in length totals |
| Separate motor targets, torque mode and missed-step plots | No | `motors/MACHINE/ENTITY` |
| Separate encoder angle and axis plots | No | `encoders/MACHINE/ENTITY` |
| Stored linear/angular ECS velocities | No | `velocities/MACHINE/ENTITY` |
| Tool center/root/tip/cold-end points and extrusion deposits | No | `world/machines/MACHINE/tool_points` and `extrusions` |
| Total deposited length | No | `extrusion_lengths/MACHINE` |

The default native layout includes motor and encoder plots; add a time-series
view for `velocities` or `extrusion_lengths` when inspecting those quantities.
Motor plots mix radians, N m, boolean mode values and step counts; select the
individual metric rather than interpreting all curves as the same unit.
Open-loop motors count slips (whole electrical cycles, four full steps each)
in either direction; `current_missed_steps` is the signed net slip offset. See the [simulation notes](README.md#spools-and-motors)
for position/torque behavior and the specialized spool model.

## Lengths, forces and frames

Every completed physics timestep logs all loaded machines, independently of
rendering frequency, playback speed, or the Show Forces checkbox:

- Named anchor positions and all machine body frames.
- Effector center position and orientation. Rigid bodies supply the orientation;
  other scenes use the simulator's estimate from the authored effector sources.
- Every cable constraint span as
  [LineStrips3D](https://rerun.io/docs/reference/types/archetypes/line_strips3d),
  with browser slack sag/guide wraps or native straight spans as listed above.
- Commanded, actual, and geometric line lengths, plus error and stretch.
- Per-segment cable forces in newtons, including forces transferred through
  guides, and arrows showing the force on endpoint A.

Positions and lengths are in metres. Quaternions use XYZW order. Each machine
has a frame tree under `world/machines/MACHINE`; rigid members use transforms
relative to their parent body. Cable geometry uses world coordinates.

| Length trace | Definition |
| --- | --- |
| commanded | Predicted paid-out length at the position-mode motor target, using the solver's winding direction and layer/ramp model. |
| actual | Paid-out rest length: segment rest lengths plus intermediate guide wraps, excluding cable stored on endpoint spools. |
| geometric | Attachment-to-attachment straight span lengths plus intermediate guide wraps. |
| error | Actual minus commanded. |
| stretch | Geometric minus actual; negative values indicate slack. |

Torque mode has no commanded-length target, so its commanded and error series
are omitted. Guide friction can make segment forces differ along one cable;
the recorder retains every segment. Force arrows use 0.01 metres per newton by
default. The browser receiver exposes `--force-scale`; native API users can set
`RerunSystem.force_scale`. The native CLI uses the default.

The browser allows at most 32 unacknowledged samples. Python acknowledges after
logging each sample; linear playback, ASAP playback, and single stepping wait
when the receiver is busy. Recording can therefore reduce simulation speed.
No timesteps are decimated. A disconnect ends the recording and releases the
simulation to continue; reconnecting starts a new recording at time zero.

## Implementation

`app/flightRecorder.js` owns the connection, timeline, and backpressure.
`app/flightRecorderSnapshot.js` reads the ECS into the versioned JSON protocol.
The recorder system runs after encoders and motor diagnostics. The Python
receiver is `scripts/hangprinter_flight_recorder.py`; its Rerun calls use the
0.38 API. Scene identities are retained in `SceneEntityInfoComponent` so the
recording uses authored names rather than guessing anchors from render shapes.
