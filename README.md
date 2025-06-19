# Cable Joints Physics Engine

This project simulates cables using a Position‑Based Dynamics (PBD) solver built on top of a small Entity‑Component‑System (ECS) framework.  Both JavaScript and Python implementations are provided.  The long‑term goal is to drive a Hangprinter style robot in environments such as Omniverse or Isaac Lab.

## Quick Start

### Running the JavaScript demos
1. Install Node.js (v18 or later recommended).
2. In this repository run:
   ```bash
   npm install        # only needed the first time
   npx vite
   ```
3. Open <http://localhost:5173/flipper.html> in your browser.

### Running the Python demos
1. Make sure Python 3.10+ is installed.
2. Install the optional dependencies:
   ```bash
   pip install numpy pytest websockets warp-lang[extras]
   ```
3. Start the demo server and open the HTML front end:
   ```bash
   python -m examples.python.flipper.server
   # then visit examples/python/flipper/index.html in your browser
   ```
4. To run the Warp version use `python -m examples.python.flipper.server --warp` and open `examples/python/flipper/index_warp.html`.

### Running the tests
JavaScript tests use Jest and Python tests use pytest.

```bash
npm test             # runs tests under tests/
python -m pytest tests/python  # runs tests for the Python port
```

## Project Layout
- **src/js/cable_joints/** – JavaScript source modules
- **src/python/cable_joints/** – Python port mirroring the JS implementation
- **tests/** – Jest unit and integration tests
- **tests/python/** – pytest suite for the Python port
- **src/js/cable_joints_3d/** – experimental 3D viewer using Three.js

## 3D Visualization
`tests/html/3d_tests.html` shows cable joint points in 3D using Three.js.  Equivalent helpers live in `src/python/cable_joints_3d/vector3.py` and `src/python/cable_joints_3d/geometry3.py`.

For more details on the physics algorithms see the papers in `ai_docs/`.
