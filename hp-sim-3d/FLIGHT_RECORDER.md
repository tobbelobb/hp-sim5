# Hangprinter flight recorder

Run from the repository root, with the simulator served by Vite:

```bash
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python scripts/hangprinter_flight_recorder.py
```

Open the **Rerun viewer URL printed by the recorder**. The URL includes the
connection to the local data server; opening port 9090 without that query opens
Rerun's welcome screen.

In hp-sim-3d, expand the simulation controls with the ▼ button and click **Rerun**.
The button reads **Rerun: recording** once connected. Start playback or a print.
Click Rerun again to finish the recording. The Python process can stay running
for another recording. Ctrl-C closes the server and flushes its active files.

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

## Recorded data

Every completed physics timestep logs all loaded machines, independently of
rendering frequency, playback speed, or the Show Forces checkbox:

- Named anchor positions and all machine body frames.
- Effector center position and orientation. Rigid bodies supply the orientation;
  other scenes use the simulator's estimate from the authored effector sources.
- Every cable constraint span as
  [LineStrips3D](https://rerun.io/docs/reference/types/archetypes/line_strips3d),
  with the simulator's slack sag and intermediate guide wraps.
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
default; change this with `--force-scale`.

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
