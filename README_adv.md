# hp-sim5 Advanced Guide

This guide expands on the main `README.md`. It covers native 3D machine
experiments, the older Python demos and the XPBD cable-joints library.

## Slideprinter Demo Advanced
The older 2D Slideprinter demo uses MoveCommander and WebSocket communication.
Its Python browser integration is currently broken. For current machines, use
the 3D browser app or the native Python workflow below. Native 3D parity covers
machine physics and command semantics; it does not require that older browser
integration.


## XPBD Physics Engine and Cable Joints Library
The simulator is built on a physics engine implementing (extended) Position‑Based Dynamics (XPBD).
Cable segments slide over wheels, wrap, and maintain tension through constraints solved with XPBD.
The 2D engine lives under `src/js/cable_joints/`, with a related Python
implementation in `src/python/cable_joints/`. The JavaScript and Python 3D
engines live in `src/js/cable_joints_3d/` and `src/python/cable_joints_3d/`.
Python loads the authored USDA Hangprinter machines and runs the cable,
rigid-body, spool/motor, encoder, command and extrusion systems headlessly.
The [live JS differential harness](tests/parity3d/README.md) compares construction
and every timestep, including complete machines. [PYTHON_3D_PARITY.md](PYTHON_3D_PARITY.md)
records coverage, tolerances, explicit differences and the review stack.
Rerun is the primary Python visualization and recording path; browser rendering
remains available for the older Python demos.

### Physics Engine Purpose
 - A physics engine for cables interacting with rolling wheels and other obstacles.
 - Position-Based Dynamics (PBD) solver ensuring cable length constraints.
 - Supports generic ECS-based entities/components/systems.
 - Includes interactive demos such as the Slideprinter and Flipper, along with
   various integration-, functional-, and unit tests.

## Python Port

A Python implementation of the 2D cable-joints engine is available in
`src/python/cable_joints/`; the native 3D engine is in
`src/python/cable_joints_3d/`. Use the
[recording guide](hp-sim-3d/FLIGHT_RECORDER.md#native-python-simulation) to create
your own native RRD: its settling, motion/extrusion and torque examples include
the exact commands and expected results. A 200-step HP4 settling run lasts only
0.4 simulated seconds and has no commanded travel. Opening an existing RRD
does not execute the simulation.

  - Dependencies:
    - python 3.10+
    - numpy
    - pytest
    - websockets
    - rerun-sdk
    - warp-lang[extras] (optional)
    - pytest-asyncio
    - usd-core
  - Usage:
    1. Install the core dependencies: `.venv/bin/python -m pip install -r requirements.txt`
    2. Optionally install Warp: `.venv/bin/python -m pip install -r requirements-warp.txt`
    3. Run Python tests: `.venv/bin/python -m pytest tests/python`

### Native Python 3D experiments

For programmatic control, use the same production composition root as the CLI.
Run this from the repository root with `PYTHONPATH=src/python`:

```bash
PYTHONPATH=src/python .venv/bin/python - <<'PY'
from cable_joints_3d.machine_simulation import load_machine_world
from cable_joints_3d.remote_spool_system import RemoteSpoolSystem
from cable_joints_3d.motor_diagnostics import get_machine_motor_diagnostics
from cable_joints_3d.machine_snapshot import capture_machine_snapshot

world = load_machine_world('public/usd_scenes/hp4_rigid_body.usda')
remote = world.get_system(RemoteSpoolSystem)
remote.commands = [{'type': 'Move', 'A': .0001}] + [None] * 9
dt = world.get_resource('dt')
for _ in range(10):
    world.update(dt)
print(remote.get_playback_state())
print(get_machine_motor_diagnostics(world))
print(next(frame for frame in capture_machine_snapshot(world)['frames']
           if frame['kind'] == 'effector'))
PY
```

The loader bakes authored cable initialization, builds the ECS and registers
systems in the JS simulation order. `world.update(dt)` executes one whole
pipeline step; there is no hidden whole-step substep loop. Use the authored
`world.get_resource('dt')` for both execution and parity experiments. If you
change the timestep programmatically, also set `world.set_resource('dt', dt)`:
the cable solver reads that resource independently of the update argument.

`remote.add_command(record)` appends to the queue. `get_playback_state()` returns
copied history/queue records, and `set_playback_state(state)` restores them.
`clear_command_queue()` removes queued work; `clear_playback_state()` also
removes history. Commands are consumed before prediction, including deposition
at the previous final tool tip. The [command table](hp-sim-3d/FLIGHT_RECORDER.md#command-records-and-cli-options)
defines angles, reference offsets, torque transitions and extrusion units.

Set `world.get_resource('pauseState').paused = True` to pause simulation
systems and command consumption. A registered native Rerun recorder can still
observe the paused state without advancing `sim_step` or `sim_time`. Set it to
`False` and call `world.update(dt)` for the next active step. `world.update(0)`
does not represent a pause: unpaused command processing can still consume work.

`get_machine_motor_diagnostics(world, machine_id=None)` reports tracking/missed
steps; `reset_machine_motor_diagnostics(world, machine_id=None)` resets their
baseline and peak. These are diagnostics, not a replacement for inspecting
physical encoder/pose state. Native Rerun also exposes stored velocities, motor
targets, encoder angles, tool points and deposited length.

To record from this API, create a `rerun.RecordingStream`, attach a `rr.FileSink`,
and pass `recording=stream` to `load_machine_world`. This records initial state
and each update; flush and disconnect the stream after the run. The CLI handles
that lifecycle and provides a default viewer layout. Physics-only API runs do
not need a viewer process.

#### Append or replace authored machines

The CLI creates one fresh world per invocation. For a shared world, use unique
namespaces and the existing native USD loader:

```python
from usd.cable_scene_loader import open_cable_scene
from cable_joints_3d.machine_scene import populate_machine_scene
from cable_joints_3d.machine_simulation import load_machine_world, register_machine_systems

world = load_machine_world('public/usd_scenes/hp4_rigid_body.usda', namespace='hp4')
populate_machine_scene(world, open_cable_scene('public/usd_scenes/hp3_rigid_body.usda'),
                       '/World/HangprinterScene', namespace='hp3', append=True)
register_machine_systems(world)  # reuse systems; initialize updated tool bindings
```

Append retains the first machine's gravity/timestep and existing entity/load
state. Axis commands broadcast to all matching spools, so an `A` target affects
both machines above. Positive `E` deposits for machines touched by that command;
without touched axes, a sole available machine supplies the default tool.

Use `append=False` to replace the scene. Replacement clears entities and their
cable-load maps, increments `sceneGeneration`, and reloads gravity/timestep.
System instances, pause state and command history/queue are retained; clear
playback explicitly if the old queued commands should not drive the new machine.
The axis cache is rebuilt for the live entities. Call `register_machine_systems`
after loading to initialize tool bindings; if recording, pass the same stream
again. Registration is idempotent and rejects a different stream on that world.
Native Rerun resets its clock on replacement and clears obsolete shapes/traces;
append retains its clock. Use fresh worlds and files for independent experiments.

#### Feature settings and authored data

The API defaults to line layering enabled and position motors in open-loop mode,
matching the browser defaults. Set `world.set_resource('enableLayering', False)`
to disable layering or `world.set_resource('closedLoopMotorsEnabled', True)` for
closed-loop position drive; these are API settings, not native CLI switches.
Compare engines with identical settings. Changing layering during a browser
session rebuilds its authored cable initialization with zero cable half-width.
To match that initialization in a native experiment, set the flag before scene
construction and use the baker's half-width override:

```python
from cable_joints_3d.ecs import World
from usd.cable_scene_loader import open_cable_scene
from cable_joints_3d.machine_scene import populate_machine_scene
from cable_joints_3d.machine_simulation import register_machine_systems

world = World()
world.set_resource('enableLayering', False)
stage = open_cable_scene('public/usd_scenes/hp4_rigid_body.usda',
                        cable_path_half_width_override=0.)
populate_machine_scene(world, stage, '/World/HangprinterScene')
register_machine_systems(world)
```

Edit USDA to change masses/inertia, attachment frames, spool axes/radii, cable
stored/rest lengths, stiffness/damping, friction or `cablePath:solverIterations`.
Gravity and timestep come from the stage. Keep declared USD float/double types
consistent: both loaders preserve authored precision, which can affect a long
trajectory. The rigid-body/member and spool semantics are explained in the
[3D README](hp-sim-3d/README.md); browser UI, firmware planning and generic hinge
physics are separate from the native simulation pipeline.


### Running the basic Python demos

  * Flipper:
    - Start the vite server
      ```
      npx vite
      ```
    - Start the demo server (in another terminal)
      ```bash
      .venv/bin/python -m example_apps.python.flipper.server
      ```
    - Visit <http://localhost:5173/hp-sim5/example_apps/python/flipper/index.html>
The older Python 2D Slideprinter browser demo is not a supported quick start.
Use the [native 3D experiments](#native-python-3d-experiments) for current
Slideprinter USDA machines and the [3D browser app](hp-sim-3d/README.md) for
G-code printing.


### Warp Version of Cable Joints
Warp is a Python library that can do many cool things.
In our case it helps us offload physics solvers to the GPU.
We plan to use it for other things in the future.
Warp can run on CPU or GPU, so there are two demos in one here:

Run the demo of the current Warp version of the Python Cable Joints library on the CPU:
 - Start the flipper server in Warp mode
  ```bash
  # Assumes npx vite is already running
  .venv/bin/python -m example_apps.python.flipper.server --warp
  ```
 - Visit <http://localhost:5173/hp-sim5/example_apps/python/flipper/index_warp.html>

For a GPU demo give `server` a `--device cuda:0` flag:
```bash
.venv/bin/python -m example_apps.python.flipper.server --warp --device cuda:0
```

### Flipper Overlay
You can play the (non-warp) and js driven flipper both at the same time.
This is fun and usefult for testing js/Python equivalence.

 - Start the python flipper server
  ```bash
  # Assumes npx vite is already running
  .venv/bin/python -m example_apps.python.flipper.server
  ```
 - Visit <http://localhost:5173/hp-sim5/example_apps/js/flipper/flipper_overlay.html>

## Further Tests and Demos
### Cable Joints Visual "Unit Tests"
 - Deployed at: <https://tobbelobb.github.io/hp-sim5/tests/html/cable_joints_test.html>
 - Locally: <http://localhost:5173/hp-sim5/tests/html/cable_joints_test.html>

## 3D Visual Tests
 - Deployed at: <https://tobbelobb.github.io/hp-sim5/tests/html/3d_tests.html>
 - Locally: <http://localhost:5173/hp-sim5/tests/html/3d_tests.html>

## Hybrid Attachment Visual Test
 - Deployed at: <https://tobbelobb.github.io/hp-sim5/tests/html/hybrid_test1.html>
 - Locally: <http://localhost:5173/hp-sim5/tests/html/hybrid_test1.html>

## Cable Joints 3d Visual Test
 - Deployed at: <https://tobbelobb.github.io/hp-sim5/tests/html/cable_joints_3d.html>
 - Locally: <http://localhost:5173/hp-sim5/tests/html/cable_joints_3d.html>

## hp-sim-3d Implementation Notes

The 3D Hangprinter app lives in `hp-sim-3d/`. Scene interpretation is split
between the coordinator in `app/setupScene.js`, the builders in `app/scene/`,
and system registration in `app/sceneSystems.js`. The 3D ECS now uses full 3x3
inertia tensors in its constraint and motor reaction paths. Spool rotors still
use a specialized one-axis projection rather than independent rotor bodies and
generic hinge constraints.

See [`hp-sim-3d/README.md`](hp-sim-3d/README.md) for the current system order,
cable behavior, rigid-body model, and remaining hinge-joint gap.

## Cable Joints Hanging Visual Test
 - Deployed at: <https://tobbelobb.github.io/hp-sim5/tests/html/cable_joints.html>
 - Locally: <http://localhost:5173/hp-sim5/tests/html/cable_joints.html>


## 3D Visualization
 `tests/html/3d_tests.html` tries to render cable joint points using Three.js.
 Utility modules `vector3.js` and `geometry3.js` provide basic 3D math helpers and now live under `src/js/cable_joints_3d/`.
 The Python side includes `src/python/cable_joints_3d/vector3.py` and `src/python/cable_joints_3d/geometry3.py` for analogous 3D helpers.

 
