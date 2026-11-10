# Native PGO and columnar Rerun performance

Status: complete.

## Abstract

This study compares native headless simulation with the fuller Rust/Wasm runtime, using the same Rust physics code, commands, fixed timesteps and output cadence. Raw column buffering replaces per-observation Rerun object construction in both recorders, and optimized native builds now use deterministic profile-guided compilation with explicit opt-out and artifact selection. Three sequential repetitions per case measure HP3/HP4, recording on/off, fresh and continuing worlds, the standalone autocal service and real six-point RRF collections, including finalization. Recorded runs improve 1.73–3.83× over Wasm with the earlier logging; most of that gain is logging and also benefits Wasm. Native physics improves 1.25–1.31× on eight 30,000-step prefixes and 1.39×/1.27× on full HP3/HP4 autocal replays. Native checkpoint hashes and audited RRD values remain unchanged, and all 42 real collections retain exact selected measurements and step counts. These measurements exclude full optimizer/fitting runs and do not establish calibration accuracy.

## Topics

Native Rust, Wasm, PGO, Rerun, send_columns, logging overhead, headless simulation, HP3, HP4, firmware collection.

## Tested hypotheses

| Hypothesis | Experiments | Outcome | Evidence |
| --- | --- | --- | --- |
| Raw column batching reduces recorder overhead | O01 | Supported | [Paired consumers](summary.json), [RRD audit](final-recording-audit.json) |
| Redundant-work source micro-optimizations improve physics | O02 initial | Rejected on tested HP4 workload; changes reverted | [Notebook](research.md) |
| PGO and host tuning improve native physics | O02 follow-up | Supported | [Prefixes](final-physics-matrix.json), [long replays](long-matrix.json) |
| Reusing captured snapshots reduces observation work | O03 | Supported as part of combined logging change; not isolated | [Report](report.md) |

## References

- [Report](report.md): method, timing scope, results and limitations.
- [Notebook](research.md): decisions, failed trials and budgets.
- [Source manifest](source-manifest.json): exact source/artifact identity.
- [Previous study](../2026-10-14-native-throughput/report.md): earlier Wasm/native measurements.
