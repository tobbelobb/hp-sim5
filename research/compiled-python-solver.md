# Compiled Python cable solver

The Python 3D machine pipeline has an opt-in Warp cable solver. It compiles the
complete ordered constraint loop together with quaternion, frame, tensor-inertia
and spin math. Other physics systems still execute in Python. The existing
headless JS backend remains the collection default and is faster overall.

No additional dependency is needed beyond the existing optional Warp setup:

```bash
.venv/bin/python -m pip install -r requirements-warp.txt
./hp-sim5-research-agent --machine hp3 --physics-backend native-warp
```

For numerical-only collection, add `--viewer none --no-record`. Direct Python
construction uses `load_machine_world(path, cable_solver_device='cpu')`. Fresh
`run_experiment` accepts `cable_solver_device='cpu'`; its manifest records the
backend, device and Warp package version. Frozen collection replay accepts
`--backend native-warp`. The standalone Python simulator accepts
`--cable-solver-device cpu`. Normal Python operation does not import Warp.

## Numerical contract

All solver state uses double precision. One thread solves one world's coupled
constraints in the existing alternating path/joint order; corrections to A and B
remain sequential. Tensor inertia, rigid members, position/torque motor modes,
holding-torque release, hybrid stored gradients, pinhole force transfer,
zero-stiffness damping, encoder corrections and load dictionaries are retained.
No parallel atomics, reduced iteration counts or larger timesteps are used.

The adapter packs cable endpoints and their parent bodies into reusable compact
structured arrays. CPU views share the Warp buffers without a transfer. It
recomputes coupling metadata each update, so topology edits and motor/configuration
changes cannot silently reuse stale constraint data. CUDA uploads and readbacks
are explicit. Compilation/device diagnostics go to stderr to protect MCP stdio.

## Measurements on 2026-10-08

Three fresh sequential repetitions per HP3/HP4 backend, each 500 fixed 2 ms
steps: an A-axis ramp followed by a B-axis torque transition. Whole-world time
includes state packing and every physics system, excluding scene construction,
first compilation, recording and observation logging. Solver time includes
packing, compiled execution and unpacking. Warp 1.12.0, Python 3.12.3, same host
as the preceding collection benchmark.

| Machine | Reference world (s) | Warp world (s) | Whole-world speedup | Solver speedup |
| --- | ---: | ---: | ---: | ---: |
| HP3 | 1.839 | 1.327 | 1.39× | 3.00× |
| HP4 | 9.964 | 5.092 | 1.96× | 7.90× |

These are one simulated second per trial: Warp reaches 0.754× realtime for HP3
and 0.196× for HP4. Python remains below realtime. The initial uncached kernel
compilation took about 1.45 seconds; that is a separate cold-start cost.

Maximum final differences over the three controlled trials:

| Quantity | HP3 | HP4 |
| --- | ---: | ---: |
| Position (m) | 2.663e-8 | 7.558e-12 |
| Quaternion component | 4.307e-7 | 1.691e-6 |
| Encoder (rad) | 8.630e-7 | 5.103e-6 |
| Cable force component (N) | 1.111e-4 | 1.434e-8 |

Within-backend repetitions are exact. The largest encoder difference is about
0.000292 degrees, below the preceding replay's 0.01 degree endpoint tolerance.
This is a numerical comparison, not calibration or hardware accuracy evidence.
Short-term tests compare the full snapshot and load maps under each existing
fixture's tolerances. Long trajectories are not bit-identical.

The profile after compilation identifies attachment updates, rigid-body sync,
friction and over-correction as remaining interpreted costs. Packing also costs
more than the small compiled kernel alone. Further useful work would compile
attachment/frame updates and keep state resident across multiple physics
systems, while preserving topology and command boundaries.

## Optional CUDA

`--physics-backend native-warp-cuda` selects CUDA for the cable solver only;
Python systems remain on the host. Fresh trials and direct construction use
`cable_solver_device='cuda:0'`. A missing device is an error, not a silent CPU
fallback. This host has no CUDA-capable GPU, so GPU parity and throughput are
**unmeasured**. CUDA comparison tests run when such hardware is available.

A single machine is deliberately solved sequentially. This mode does not promise
a GPU speedup: transfer/launch costs may dominate, and it does not parallelize
constraints sharing a body. Independent worlds would be the natural future
parallelism, with one ordered solve per world; that batching API is not provided.

## Evidence and reproduction

Evidence and retained early trials are in
`output/research/sessions/fdb616a454af4a6d9c1db074fbdc207d/compiled-solver/`:
`C03-timing.json`, `C03-timing-v1.json`, final-state files, `benchmark.py`,
`C03-warp-profile.pstats`, `C02-parity-v1.txt`, and regression output.

```bash
WARP_CACHE_PATH=/tmp/hp-sim5-warp-cache .venv/bin/python -m pytest -q \
  tests/python/cable_joints_3d/test_warp_solver.py
PYTHONPATH=src/python WARP_CACHE_PATH=/tmp/hp-sim5-warp-cache .venv/bin/python \
  output/research/sessions/fdb616a454af4a6d9c1db074fbdc207d/compiled-solver/benchmark.py
```

The implementation uses Warp's documented native compilation and structured
array mechanisms: [NVIDIA Warp code generation](https://nvidia.github.io/warp/v1.15/deep_dive/codegen.html)
and [runtime/struct arrays](https://github.com/NVIDIA/warp/blob/main/docs/user_guide/runtime.rst).
The installed 1.12.0 implementation was inspected and tested; these online
references describe the mechanism, not this repository's measured performance.

Verification: 737 tests pass in the full relevant regression run, including all
300 autocal tests; five CUDA tests skip and 14 tests are deselected (13 opt-in
slow tests plus the known stdio environment timeout). After final source cleanup,
104 focused tests pass with five CUDA skips. Compilation and diff checks pass.
See `C02-regressions.txt` and `C02-final-focused.txt`. The exact measured solver
is retained as `solver-source.py` with its SHA-256 in `C03-timing.json`.
