# Instructions for Agents

## Research Papers Regarding the Simulator
- The directory `ai_docs/` contains `.md` versions of research papers that inform the physics algorithms used here.
- **Always read the relevant paper and confirm your approach before changing any physics behavior.**
- Each subfolder of `ai_docs/` (such as `CableJoints`, `PBDBodies`, `XPBD`, and `smallsteps`) contains the paper in markdown form.

## JavaScript and Python Ports of the Cable Joints library
- The JavaScript version of the cable joints library lives in `src/js/cable_joints/`.
- When working on one implementation, cross reference the corresponding files from the other implementation to understand how features translate between JS and Python.

## Tests
- Don't change integration test expectations unless explicitly asked to do so.

## Attic
Don't look at files you find in directories called `attic`. Those are put there to be forgotten and then removed.

## The Most Powerful Methodology
The Designer Routine and methodology outlined in `ai_docs/how_to_design_programs.md`.
It is powerful but also costly.
Use it for really tough problems, large features, or if explicitly asked to use it.
