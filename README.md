# hp-sim5 - Hangprinter Simulator

![Demo](slideprinter_Hangprinter_logo_wiggling_and_losing_steps2.gif)

hp-sim5 simulates the physics of [Hangprinter](https://hangprinter.org).
hp-sim5 currently simulates:

 - How lines behave
 - How spools behave
 - How motors behave
 - etc etc

The goal is to reproduce real world Hangprints as closely as possible.

The `Slideprinter` demo reproduces infills with resonance patterns, ringing after
sharp corners, slack lines along smooth curves and even lost steps from overly
aggressive moves.  All these issues become easier to understand and fix with
this tool.

Try the live demos:

- A 2d Hangprinter called Slideprinter at [tobbelobb.github.io/hp-sim5/hp-sim/](https://tobbelobb.github.io/hp-sim5/hp-sim/)
- A flipper game that tests the Cable Physics engine at [tobbelobb.github.io/hp-sim5/flipper](https://tobbelobb.github.io/hp-sim5/examples/js/flipper/index.html)

Intended uses:
 - **Hardware design** -- reduce guesswork when building new machines.
 - **Digital twin** -- run the simulator before and during prints to optimise
   speed and quality while avoiding catastrophes.
 - **Software design** -- enables rapid firmware development and experiments
   with advanced control and AI.

## Cable Joints and Physics Engine
hp-sim5 includes a Cable Joints library and XPBD physics engine inspired and coded from the work of [Matthias
Müller](https://matthias-research.github.io/pages/index.html).
hp-sim5 includes two fully functional Cable Joints implementations; one in JavaScript and one in Python.

For a deeper dive into the physics engine and the flipper demo see
`README_adv.md`.

The physics engine is the heart of hp-sim, and lives in the src directory.

hp-sim5 also includes lots of other code (subrepos, or "sub-projects") to make the best use of the simulation within the Hangprinter Project:
 - Klipper fork, up-to-date Hangprinter compatibility, tested with hp-sim5.
 - ReprapFirmware fork, up-to-date Hangprinter compatibility, includes a new host version to enable hp-sim5 compatibility.
 - flex-compensation-dev. Our own repo solely devoted to developing flex compensation for all cable robots.
 - forward-transform-dev. Our own repo devoted to developing forward transforms for all cable robots.
 - autocal. Our own repo for developing automatic calibration.



## Host Version of ReprapFirmware
To compile and invoke the `x86_64` version of ReprapFirmware, do:
```
cmake --build RRF/build --target rrf_simulator -j
./RRF/build/rrf_simulator --vsd RRF/run/vsd --gcode gcodes/draw_squares.gcode --can-log logs/draw_squares.csv -c sys/config_slideprinter.g
```

### HTTP Endpoint Mode

The rrf_simulator supports an HTTP server mode for interactive G-code execution:

#### Starting the Server

```bash
./RRF/build/rrf_simulator \
    --vsd RRF/run/vsd \
    -c sys/config_slideprinter.g \
    --server \
    -p 8080
```

or just

```
./scripts/rrf_server_slideprinter.sh
```

#### Endpoints

- `POST /machine/code` - Execute G-code, returns reply text
- `GET /machine/status` - Get server status

#### Example Usage

```bash
# Set torque mode
curl http://localhost:8080/machine/code -d "M569.4 P40.0 T0.001" -H "Content-Type: text/plain"
# Response: 0.001000 Nm,

# Return to position mode
curl http://localhost:8080/machine/code -d "M569.4 P40.0 T0" -H "Content-Type: text/plain"
# Response: pos_mode,

# Execute move
curl http://localhost:8080/machine/code -d "G1 X10 F1000" -H "Content-Type: text/plain"
```

or just open a `rrf_http_bridge` like this:

```
node scripts/rrf_http_bridge.mjs
```

... Wait for it to connect with the RRF Http Bridge and type the Gcodes in directly, like this:

```
$ node scripts/rrf_http_bridge.mjs
disconnected> WebSocket feed ready on ws://localhost:8790
Open hp-sim with ?gcode_ws=ws://localhost:8790 to follow along.
gcode> M569.3 P40.0:41.0:42.0
> M569.3 P40.0:41.0:42.0
[0.00, -0.00, 0.00, ]
gcode> G1 H2 X10
> G1 H2 X10
```

#### JavaScript Integration

See `examples/js/slideprinter/rrfHttpBridge.js` for programmatic access.

#### hp-sim CLI bridge (no UI changes)

- Start the simulator in server mode (as above), then run
  `node scripts/rrf_http_bridge.mjs --server http://localhost:8080 --ws-port 8790`
- Type G-code lines into the CLI (or pass `--cmd "G1 X10"` for one shots); replies are printed immediately.
- Open hp-sim locally with `?gcode_ws=ws://localhost:8790` appended to the URL so the visualization consumes the streamed motion without new UI controls.



## Quick Start

### Running Demos Locally
1. Install Node.js
2. In this repository run:
   ```bash
   npm install        # only needed the first time
   npx vite           # Needed every time to serve the html and js
   ```
3. Open <http://localhost:5173/hp-sim5/hp-sim> in your browser.
   There's also <http://localhost:5173/hp-sim5/flipper> for the flipper demo.
4. Hack away!

## Tests

### Run the full local suite (no simulator / no visuals)
```bash
npx test
python -m pytest
```

Also check out `scripts/run_ci_tests.sh`.

### Autocal-only filtering
```bash
python -m pytest autocal/tests
python -m pytest autocal/tests -k active_calibrate
python -m pytest autocal/tests/test_active_calibrate_golden.py
```

### Simulator / E2E tests (requires visible browser + rrf_simulator build)
```bash
cmake --build RRF/build --target rrf_simulator -j
tests/run_all_e2e_tests.sh
node scripts/e2e_test_collect_single_sweep.mjs --sim
node scripts/e2e_test_find_minimum_moving_force.mjs --sim
node scripts/e2e_test_find_edge_force.mjs --sim
node scripts/e2e_test_wait_for_stable_encoders.mjs --sim
node scripts/e2e_test_return_to_origin_one_at_a_time.mjs --sim
node scripts/e2e_test_return_to_origin_all_at_once.mjs --sim
node scripts/e2e_test_calibrate_encoder_noise.mjs --sim
```

### Visual/manual checks
```bash
npx vite
# open tests/html/*.html in the browser for cable_joints 2D/3D visual tests
```

### Determinism scripts
```bash
./run_draw_squares_determinism_test.sh
./run_logo_determinism_test.sh
./run_logo_slideprinter_determinism_test.sh
```

## hp-sim5 context: the Hangprinter Project
hp-sim5 is part of an effort to automate the Hangprinter Project.
We want to automate everything except the actual users,
and digitize everything except the finished working machines and their output.
