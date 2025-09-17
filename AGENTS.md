# Instructions for Agents

## Research Papers
- The directory `ai_docs/` contains `.md` versions of research papers that inform the physics algorithms used here.
- **Always read the relevant paper and confirm your approach before changing any physics behavior.**
- Each subfolder of `ai_docs/` (such as `CableJoints`, `PBDBodies`, `XPBD`, and `smallsteps`) contains the paper in markdown form.

## JavaScript and Python Ports
- The JavaScript version of the flipper and cable joints lives in `flipper.html (redirects to examples/js/flipper/index.html)` and the modules under `src/js/cable_joints/` and `examples/js/flipper/`.
- The Python port lives in `examples/python/flipper/index.html`, `examples/python/flipper/server.py`, and the modules under `src/python/cable_joints/` and
  `examples/python/flipper/`.
- When working on one implementation, cross reference the corresponding files from the other implementation to understand how features translate between JS and Python.

## Tests
- Don't change integration test expectations unless explicitly asked to do so.

## Attic
Don't look at files you find in directories called `attic`. Those are put there to be forgotten and then removed.
