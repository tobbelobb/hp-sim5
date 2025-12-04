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

hp-sim5 have two fully functional, equivalent implementations; one in JavaScript and one in Python.
hp-sim5 includes a Cable Joints library and XPBD physics engine inspired and coded from the work of [Matthias
Müller](https://matthias-research.github.io/pages/index.html).

For a deeper dive into the physics engine and the classic flipper demo see
`README_adv.md`.

To compile and invoke the `x86_64` version of ReprapFirmware, do:
```
cmake --build RRF/build --target rrf_simulator -j
./RRF/build/rrf_simulator --vsd RRF/run/vsd --gcode gcodes/draw_squares.gcode --can-log logs/draw_squares.csv -c sys/config_slideprinter.g
```

## HTTP Endpoint Mode

The rrf_simulator supports an HTTP server mode for interactive G-code execution:

### Starting the Server

```bash
./RRF/build/rrf_simulator \
    --vsd RRF/run/vsd \
    -c sys/config_slideprinter.g \
    --server \
    -p 8080
```

### Endpoints

- `POST /machine/code` - Execute G-code, returns reply text
- `GET /machine/status` - Get server status

### Example Usage

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

### JavaScript Integration

See `examples/js/slideprinter/rrfHttpBridge.js` for programmatic access.

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

## hp-sim5 context: the Hangprinter Project
hp-sim5 is part of an effort to automate the Hangprinter Project.
We want to automate everything except the actual users,
and digitize everything except the finished working machines and their output.
