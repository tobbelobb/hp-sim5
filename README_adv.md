# hp-sim5 Advanced Guide

This guide expands on the main `README.md`.  It covers how to run the Python
equivalents of the demos, describes the Flipper example and gives an
overview of the XPBD‑based cable joints library that powers the simulator.

## Slideprinter Demo Advanced
Both the js and Python implementations of the Slideprinter demo consist of a "firmware" called MoveCommander and a simulated "3d printer" called Slideprinter.
Communication between them goes via (real or simulated) websockets.
This means the Python MoveCommander can in principle command the js Slideprinter and vice versa.
This demo frequently breaks and depends on fiddling with port numbers, so is not presented in the main README.md.
But in principle it should work all the time for the js and Python ports to be fully equivalent.


## XPBD Physics Engine and Cable Joints Library
The simulator is built on a physics engine implementing (extended) Position‑Based Dynamics (XPBD).
Cable segments slide over wheels, wrap, and maintain tension through constraints solved with XPBD.
The engine lives under `src/js/cable_joints/` with a line‑for‑line Python port in `src/python/cable_joints/`.
These modules form a small but powerful library that can be used outside of the Slideprinter app for custom robotics experiments.

### Physics Engine Purpose
 - A physics engine for cables interacting with rolling wheels and other obstacles.
 - Position-Based Dynamics (PBD) solver ensuring cable length constraints.
 - Supports generic ECS-based entities/components/systems.
 - Includes interactive demos such as the Slideprinter and Flipper, along with
   various integration-, functional-, and unit tests.

## Python Port

A complete Python port of the cable joints engine is available in the `src/python/cable_joints/` directory.
  - Dependencies:
    - python 3.10+
    - numpy
    - pytest
    - websockets
    - warp-lang[extras]
    - pytest-asyncio
    - usd-core
  - Usage:
    1. Install dependencies: `pip install numpy pytest websockets warp-lang[extras] pytest-asyncio usd-core`
    2. Run Python tests: `python -m pytest`


### Running the basic Python demos

  * Flipper:
    - Start the vite server
      ```
      npx vite
      ```
    - Start the demo server (in another terminal)
      ```bash
      python -m example_apps.python.flipper.server
      ```
    - Visit <http://localhost:5173/hp-sim5/example_apps/python/flipper/index.html>
  * Slideprinter:
    - Note: The Python Slideprinter Demo is broken, as the js hp-sim demo has replaced the js slideprinter demo, and no equivalent Python hp-sim demo has been developed.
      The core Python physics engine and all python tests still work, but the js part of python slideprinter demo has changed and need to be refitted.
      /tobben on Nov 4, 2025
    - Start the demo server
      ```bash
      # Assumes npx vite is already running
      python -m example_apps.python.slideprinter.server
      ```
    - Visit <http://localhost:5173/hp-sim5/example_apps/python/slideprinter/index.html>
    - Send some gcode commands with the Python Move Comander:
      ```bash
      python -m example_apps.python.slideprinter.move_commander public/gcode/draw_squares.gcode
      ```


### Warp Version of Cable Joints
Warp is a Python library that can do many cool things.
In our case it helps us offload physics solvers to the GPU.
We plan to use it for other things in the future.
Warp can run on CPU or GPU, so there are two demos in one here:

Run the demo of the current Warp version of the Python Cable Joints library on the CPU:
 - Start the flipper server in Warp mode
  ```bash
  # Assumes npx vite is already running
  python -m example_apps.python.flipper.server --warp
  ```
 - Visit <http://localhost:5173/hp-sim5/example_apps/python/flipper/index_warp.html>

For a GPU demo give `server` a `--device cuda:0` flag:
```bash
python -m example_apps.python.flipper.server --warp --device cuda:0
```

### Flipper Overlay
You can play the (non-warp) and js driven flipper both at the same time.
This is fun and usefult for testing js/Python equivalence.

 - Start the python flipper server
  ```bash
  # Assumes npx vite is already running
  python -m example_apps.python.flipper.server
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
The 3D Hangprinter app lives in `hp-sim-3d/`. Its system registration order in
`hp-sim-3d/app/setupScene.js` is the actual simulation loop: save previous pose,
apply motor/external changes, integrate predicted pose, sync rigid-body members,
update cable geometry, solve position constraints, then update velocities.

The current rigid-body model is a practical parent/member design rather than a
fully general XPBD rigid-body joint graph. Spool rotors keep one local twist DOF
through direct projection and a custom stepper motor path; they are not yet
separate rotor bodies connected by hinge/motor constraints. This is deliberate
for now because hp-sim-3d only needs scalar/effective inertia and plausible motor
reaction behavior.

For the detailed tradeoffs and the future hinge-joint direction, see
`hp-sim-3d/README.md`.

## Cable Joints Hanging Visual Test
 - Deployed at: <https://tobbelobb.github.io/hp-sim5/tests/html/cable_joints.html>
 - Locally: <http://localhost:5173/hp-sim5/tests/html/cable_joints.html>


## 3D Visualization
 `tests/html/3d_tests.html` tries to render cable joint points using Three.js.
 Utility modules `vector3.js` and `geometry3.js` provide basic 3D math helpers and now live under `src/js/cable_joints_3d/`.
 The Python side includes `src/python/cable_joints_3d/vector3.py` and `src/python/cable_joints_3d/geometry3.py` for analogous 3D helpers.

 
