# Fuller Rust simulation engine for hp-sim-3d

Status: complete.

## Abstract

The full Rust/Wasm engine is 2.37–2.86 times faster than the previous Wasm default
on eight frozen HP3/HP4 logo and autocal workloads. Three repeats per backend
cover 7,544,232 measured updates. Rust owns persistent physics state, winding,
topology, constraints, friction, integration, motors and encoders. JavaScript
retains scene preparation, commands, edits and observations. The same source
builds and steps natively. Short differential checks and bounded one-step forks
agree tightly, but five long feedback cases fail the initial pointwise tolerance.
The layered HP3 autocal replay reaches 2.85 degrees of encoder-read difference;
this is recorded as drift. Repeats remain deterministic within each backend.
These are simulated results from one host, with no hardware or fresh adaptive
calibration accuracy claim.

## Topics

Rust, Wasm, native physics, hp-sim-3d, HP3, HP4, simulation performance, autocal
replay, floating-point drift, cable winding.

## Tested hypotheses

| Hypothesis | Experiment IDs | Outcome | Evidence |
| --- | --- | --- | --- |
| Persistent Rust physics improves throughput beyond the old kernel | E01 | Supported | [Final matrix](matrix-precision/matrix.json) |
| Rust math retains close JS trajectories | E02 | Mixed: short/fork agreement, long feedback drift | [Early forks](one-step-forks.json), [late forks](late-forks.json), [matrix errors](matrix-precision/matrix.json) |
| The same core compiles and steps natively | E03 | Supported | [Native tests](../../../../src/wasm/cable_solver/tests/native_engine.rs) |

## References

- [Report](report.md): measurements and limits.
- [Notebook](research.md): constraints, decisions and failed trials.
- [Final source manifest](final-source-manifest.json): source and binary hashes.
- [Previous solver report](../../../../research/rust-wasm-solver.md): prior boundary.
- [Original inputs](../2026-10-13-headless-js-profiling/inputs/): frozen workloads.
