# Hangprinter flight recorder

Run every command below from the repository root. Install the Python dependencies
with `.venv/bin/python -m pip install -r requirements.txt`; they include OpenUSD,
Rerun 0.38 and the browser receiver's WebSocket library. The main
[README](../README.md#python-dependencies) describes creating `.venv`.

There are two recording workflows:

| Workflow | Physics runs in | Start recording with | Output |
| --- | --- | --- | --- |
| Browser print | JavaScript in hp-sim-3d | Receiver process, then the **Rerun** button | A new `.rrd` for each connection/scene generation |
| Native machine | Python, without Vite | `python -m cable_joints_3d` | One `.rrd`, plus an optional final JSON snapshot |

Running `.venv/bin/rerun FILE.rrd` **opens an existing recording**. It does not
run physics, create a recording or connect to the simulator. Recording happens
only when you explicitly run one of the workflows below.

## Native Python simulation

### Record a settling run yourself

This starts a fresh HP4 at its authored motor targets and records initial
settling. With HP4's `timeCodesPerSecond = 500`, one step is 0.002 seconds:
200 steps cover **0.4 seconds**, 1,000 cover **2 seconds**. These are simulation
durations; computation and recording can take longer than playback.

```bash
PYTHONPATH=src/python .venv/bin/python -m cable_joints_3d \
  public/usd_scenes/hp4_rigid_body.usda --steps 200 \
  --output output/rerun/hp4-python.rrd --snapshot output/rerun/hp4-python.json
.venv/bin/rerun output/rerun/hp4-python.rrd
```

The first command loads USDA, constructs the ECS, registers the production
pipeline, advances physics and writes the recording. The second opens it.
Holding the initial targets produces little visible travel. Use the following
motion run to check commanded motion and extrusion.

### Record a visible motion and extrusion run

First create a JSON command file. This is the same two-second HP4 ramp exercised
by the sustained cross-language test; it drives all four motors and deposits
ten extrusion records. The angles are absolute motor radians, not XYZ positions
or cable lengths.

```bash
.venv/bin/python - <<'PY'
import json
from pathlib import Path

rates = {'A': .0016, 'B': -.0012, 'C': .0008, 'D': .0004}
commands = [dict(type='Move', **{axis: rate * i for axis, rate in rates.items()},
                 **({'E': .001} if i % 100 == 0 else {})) for i in range(1000)]
path = Path('output/commands/hp4-motion.json')
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps(commands, indent=2) + '\n')
print(f'Wrote {len(commands)} commands: {path}')
PY
```

Then run and open your recording:

```bash
PYTHONPATH=src/python .venv/bin/python -m cable_joints_3d \
  public/usd_scenes/hp4_rigid_body.usda --steps 1000 \
  --commands output/commands/hp4-motion.json \
  --output output/rerun/hp4-motion.rrd --snapshot output/rerun/hp4-motion.json
.venv/bin/rerun output/rerun/hp4-motion.rrd
```

Expect `Recorded 1000 steps (2 s)` in the terminal and native timelines spanning
`sim_step` 0–1,000 / `sim_time` 0–2 seconds. The tested ramp moves the effector
about **29 mm**, has ten deposits totaling **0.01 m**, tracks the motor angles
and reports zero missed steps. Zoom in around the effector to see that travel.
Deposition is represented by points at the tool tip, with deposited length in
the `extrusion_lengths` trace. This example exercises the physics command path;
it is not a G-code logo print.

Every step is recorded. The two-second HP4 example can produce roughly 170 MB
of RRD data. Use short runs first. Explicit output names above are reused on
reruns; omit `--output` when you want a new timestamped file each time.

### Command records and CLI options

`--commands PATH` reads a JSON array and consumes **one record per unpaused
timestep**. Records are motor inputs already scheduled at the simulation rate;
the native CLI does not parse G-code, plan Cartesian paths or drive firmware.
For a logo/G-code print, use the browser workflow below. Firmware conversion
and the browser's upload/worker UI are outside the native physics port.

| Record | Meaning |
| --- | --- |
| `{"type":"Move","A":0.1,"B":-0.05}` | Set absolute position-mode targets, in radians; omitted axes retain their targets. `"axes":{"A":0.1}` is also accepted. |
| `{"type":"Move","A":0.1,"E":0.001}` | Also deposit this increment of extrusion length, in metres, at the tip before this step's physics. Positive `E` is required for a deposit. |
| `{"type":"SetTorqueMode","axis":"D","torqueNm":-0.01}` | Enter torque mode with signed motor torque in N m. Cable loads and body reactions remain active. |
| `{"type":"SetPositionMode","axis":"D"}` | Leave torque mode, clear requested torque and resume the retained position target. It does not capture the current encoder angle as a new target. |
| `{"type":"Add to reference","A":0.01}` | Add radians to the motor reference offset; the position target is `commanded_angle - delta_angle`. |
| `null` | Consume one timestep without a command; use this to leave time between mode changes. |

A `Move` angle is ignored for an axis currently in torque mode. It does not
switch that axis back to position mode. Once the queue is empty, remaining
`--steps` still advance physics with the retained motor state. Too few steps
leave commands unconsumed. Do not space a long move with a single large target
jump when you intend a smooth ramp; supply successive targets as above.

For a short torque/position transition, generate a separate command file and
run it for 500 steps (one second):

```bash
.venv/bin/python - <<'PY'
import json
from pathlib import Path

commands = ([{'type': 'SetTorqueMode', 'axis': 'D', 'torqueNm': -.01}]
            + [None] * 199 + [{'type': 'SetPositionMode', 'axis': 'D'}]
            + [None] * 299)
path = Path('output/commands/hp4-torque.json')
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps(commands, indent=2) + '\n')
PY
PYTHONPATH=src/python .venv/bin/python -m cable_joints_3d \
  public/usd_scenes/hp4_rigid_body.usda --steps 500 \
  --commands output/commands/hp4-torque.json --output output/rerun/hp4-torque.rrd
.venv/bin/rerun output/rerun/hp4-torque.rrd
```

The D motor's `torque_mode` trace is 1 through step 200 and 0 from step 201.
During torque mode the D cable has no commanded-length/error series, while
actual/geometric lengths and forces remain. A negative D request loads the
cable in this authored HP4; torque signs depend on the scene's winding/axis.

| CLI option | Use |
| --- | --- |
| `SCENE.usda` | Load the same authored machine description as JS, baking the cable initialization through OpenUSD. |
| `--steps N` | Number of updates; default 200. `--steps 0` records only the initial state. |
| `--dt SECONDS` | Override both the update timestep and cable-solver `dt` resource; positive and finite. Prefer the authored timestep for comparisons. |
| `--scene-prim /World/HangprinterScene` | Select a machine root explicitly instead of its authored default. |
| `--commands PATH` | JSON array described above. |
| `--output PATH.rrd` | Save the recording. Omit for a timestamped file in `output/rerun`. |
| `--snapshot PATH.json` | Save detached final **frames and cable telemetry**. This is not an ECS checkpoint and cannot resume a run. |
| `--connect URI` | Add a live Rerun gRPC sink alongside the saved file. |

Run `PYTHONPATH=src/python .venv/bin/python -m cable_joints_3d --help` for the
installed entry point's options. To view a native run live, start a viewer server
in one terminal, open the viewer URL it prints, then run the simulation in another:

```bash
.venv/bin/rerun --serve-web --bind 127.0.0.1 --port 9878 --web-viewer-port 9091
```

```bash
PYTHONPATH=src/python .venv/bin/python -m cable_joints_3d \
  public/usd_scenes/hp4_rigid_body.usda --steps 1000 \
  --commands output/commands/hp4-motion.json \
  --connect rerun+http://127.0.0.1:9878/proxy
```

These ports allow this viewer to coexist with the browser receiver's defaults.
Live viewing does not pace native physics to real time. The saved file remains
available after the process exits.

The native recording includes the initial state and every completed step, with
live rigid-member hierarchies, cable lengths/forces, motor and encoder state,
stored ECS velocities, tool points and extrusion deposits. `sim_time` and
`sim_step` advance on positive unpaused updates; scene resets restart them and
set the `scene_generation` timeline. Use a separate recording stream/file for
independent scene runs. Removed geometry clears both static and temporal data.

Native visuals use straight constraint spans, points and frame axes. Stored
intermediate wraps remain in length traces; browser sag and wrap tessellation are
presentation differences. Each scalar/segment has a stable plot path. Entering
torque mode clears only the commanded/error traces and preserves actual/geometric
series identities. Snapshots and scalar traces retain Python numerical precision;
Rerun's transform/geometry archetypes encode float32 values.

The optional JSON contains detached final frames and cable telemetry. The
native recording path reads the ECS without synchronizing members or mutating
physics state. See [the parity checklist](../PYTHON_3D_PARITY.md) for cross-language
recording checks and numerical bounds.

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

| Data | Browser receiver | Native Python |
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
`current_missed_steps` estimates current tracking error; `peak_missed_steps`
retains the diagnostic maximum. See the [simulation notes](README.md#spools-and-motors)
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
