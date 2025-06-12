# Instructions for Agents

## Research Papers
- The directory `ai_docs/` contains `.md` versions of research papers that inform the physics algorithms used here.
- **Always read the relevant paper and confirm your approach before changing any physics behavior.**
- Each subfolder of `ai_docs/` (such as `CableJoints`, `PBDBodies`, `XPBD`, and `smallsteps`) contains the paper in markdown form.

## JavaScript and Python Ports
- The JavaScript version of the flipper and cable joints lives in `flipper.html` and the modules under `cable_joints/`.
- The Python port lives in `flipper_python.html`, `flipper_server.py`, and the modules under `python/`.
- When working on one implementation, cross reference the corresponding files from the other implementation to understand how features translate between JS and Python.

## Tests
- Don't change integration test expectations unless explicitly asked to do so.

