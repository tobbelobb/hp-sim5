# hp-sim5 - Hangprinter Simulator

hp-sim5 simulates the physics of Hangprinter.
hp-sim5 currently simulates:

 - How lines behave
 - How spools behave
 - How motors behave
 - etc etc

The goal is to reproduce real world Hangprintes as closely as possible.

The `Slideprinter` demo reproduces infills with resonance patterns, ringing after
sharp corners, slack lines along smooth curves and even lost steps from overly
aggressive moves.  All these issues become easier to understand and fix with
this tool.

Try the live demos:

- A 2d Hangprinter called Slideprinter at [tobbelobb.github.io/hp-sim5/slideprinter](https://tobbelobb.github.io/hp-sim5/examples/js/slideprinter/index.html)
- A flipper game that tests the Cable Physics engine at [tobbelobb.github.io/hp-sim5/flipper](https://tobbelobb.github.io/hp-sim5/examples/js/flipper/index.html)

Intended uses:
 - **Hardware design** -- reduce guesswork when building new machines.
 - **Digital twin** -- run the simulator before and during prints to optimise
   speed and quality while avoiding catastrophes.
 - **Software design** -- enables rapid firmware development and experiments
   with advanced control and AI.

hp-sim5 have two fully functional, equivalent implementations; one in JavaScript and one in Python.
hp-sim5 includes a Cable Joints library and XPBD physics engine inspired and coded from the work of Matthias Müller.
For a deeper dive into the physics engine and the classic flipper demo see
`README_adv.md`.

## Quick Start

### Running Demos Locally
1. Install Node.js
2. In this repository run:
   ```bash
   npm install        # only needed the first time
   npx vite           # Needed every time to serve the html and js
   ```
3. Open <http://localhost:5173/hp-sim5/slideprinter> in your browser.
   There's also <http://localhost:5173/hp-sim5/flipper> for the flipper demo.
4. Hack away!

### Running the basic Python demos
1. Make sure Python 3.10+ is installed.
2. Install the optional dependencies:
   ```bash
   pip install numpy pytest websockets warp-lang[extras] pytest-asyncio
   ```
3. Flipper:
   - Start the demo server
     ```bash
     python -m examples.python.flipper.server
     ```
   - Visit <http://localhost:5173/hp-sim5/examples/python/flipper/index.html>
4. Slideprinter:
   - Start the demo server
     ```bash
     python -m examples.python.slideprinter.server
     ```
   - Visit <http://localhost:5173/hp-sim5/examples/python/slideprinter/index.html>
   - Send some gcode commands with the Python Move Comander:
     ```bash
     python -m examples.python.slideprinter.move_commander public/examples/gcode/draw_squares.gcode
     ```

## hp-sim5 context: the Hangprinter Project
hp-sim5 is part of an effort to automate the Hangprinter Project.
We want to automate everything except the actual users,
and digitize everything except the finished working machines and their output.
