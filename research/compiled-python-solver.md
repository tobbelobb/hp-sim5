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
fallback. The original restricted process could not access the host GPU. After disabling
the sandbox on 2026-10-08, Warp detected the RTX 3070 Ti and all five targeted
CUDA parity fixtures passed. These cover tensor bodies, spools, pinholes and
HP3/loaded-HP4 pipelines. The subsequent 20-second comparison below measures
throughput and sampled agreement; CUDA is slower than CPU and JS on that workload.
Coverage remains narrower than the full CPU fixture matrix.

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

Historical restricted-process diagnosis: the host has an NVIDIA GeForce RTX 3070 Ti and
NVIDIA driver 580.178.04. The restricted execution environment overlaid `/dev` with
a minimal tmpfs without `/dev/nvidia*`, so `nvidia-smi` and Warp could not access
it then. `CUDA_VISIBLE_DEVICES` is unset. The earlier claim that the host had no
GPU was incorrect. See `C04-device-visibility.json` and `C04-user-diagnostic.txt`
in the session evidence directory. GPU parity and throughput were unmeasured
at that point; the follow-up below supersedes the access limitation.

Unsandboxed follow-up: `nvidia-smi` works and `/dev/nvidia0`, `nvidiactl` and
`nvidia-uvm` are visible. Warp 1.12.0 detects `cuda:0`; the five CUDA checks pass
(`C04-cuda-parity.txt`). JAX 0.9.2/jaxlib 0.9.2 remain deliberately CPU-only; CUDA plugin/PJRT packages
were excluded after earlier profiling found the GPU path slower, per user clarification. No dependencies were
changed. `C04-unsandboxed-diagnostic.txt` records the new detection result; the
earlier restricted-process diagnostic remains historical evidence.

## Longer CUDA comparison, 2026-10-08 (L01–L03)

CUDA did not match compiled CPU or JS on this controlled workload. Each trial
advanced 20 simulated seconds / 10,000 fixed 2 ms steps, using the same frozen
scene, 500-step A-axis ramp, B-axis torque transition and subsequent hold.
HP3 has three fresh repetitions per backend, with rotated backend order; HP4
has one repetition per backend. There were 12 completed trials and no physics
failures. A preliminary script import-path failure is retained separately.

The table includes state packing, host/device transfers, synchronization,
all other physics systems, 10 Hz numerical observations and finalization.
It excludes scene construction/reset and disposable compilation warmup.
Rerun recording was disabled equally for all engines. A newly reset JS worker
can still incur V8 warmup during the timed run. This is controlled simulation,
not a fresh firmware/planner collection or a calibration fit.

| Machine | Warp CPU wall seconds | Warp CUDA wall seconds | JS wall seconds |
| --- | ---: | ---: | ---: |
| HP3, median of three | 26.87 | 37.24 | 1.27 |
| HP4, one trial | 105.95 | 168.61 | 6.63 |

CUDA is 38.6% slower than compiled CPU on HP3 and 59.1% slower on HP4. JS is
29.3× / 25.4× faster than CUDA on these cases. Realtime factors for CPU / CUDA /
JS are 0.744× / 0.537× / 15.756× on HP3 and 0.189× / 0.119× / 3.019× on HP4.
The desktop and Rerun remained active on the RTX 3070 Ti; GPU status was sampled
before/after each trial. This was not an isolated GPU benchmark. HP3 timing
ranges are CPU 26.624–27.021 s, CUDA 37.231–37.740 s, JS 1.250–1.294 s.

Every comparison checks all 201 matching observation steps and all motor,
effector, cable and force identities, with explicit coverage checks. Each HP3
backend repeats all sampled observations exactly. Maximum sampled CUDA
comparison differences are:

| Machine / comparison | Encoder degrees | Effector mm | Cable length mm | Segment force N |
| --- | ---: | ---: | ---: | ---: |
| HP3 CUDA vs CPU | 0.001036 | 0.000581 | 0.000705 | 0.002992 |
| HP3 CUDA vs JS | 0.001452 | 0.000227 | 0.001525 | 0.006671 |
| HP4 CUDA vs CPU | 0.0000102 | 0.00000284 | 0.00000837 | 0.00000502 |
| HP4 CUDA vs JS | 0.002161 | 0.000603 | 0.001772 | 0.001062 |

All encoder comparisons pass the prior 0.01° endpoint bound. Pose, cable and
force differences are separate measured quantities, not implied accuracy
thresholds. No anchor/radius error or hardware fidelity was established.
Sampling at 10 Hz can miss intermediate transients.

The complete solver adapter takes median 5.419 s on CPU versus 15.329 s on
CUDA for HP3; HP4 takes 14.389 s versus 77.105 s. Most of the added whole-world
cost is therefore in the CUDA adapter. Other Python physics remains a large
cost: HP4 spends about 91.5 s outside the solver, already far more than JS's
6.63 s complete run.

L03 profiles a separate 500-step HP3 startup ramp. CPU launch calls total
27.8 ms including synchronous execution; CUDA host launch calls total 22.9 ms.
The CUDA profile records 4,000 copy calls (five uploads and three readbacks per
step), totalling 577.8 ms. A separate instrumented GPU-event run measures
539.7 ms across 500 kernel intervals (1.079 ms average). Those values come from
separate instrumented runs: they are not independent costs to add together.
Readback timing includes waits for the kernel. This points to both expensive
serial GPU execution and per-step transfers; it does not justify attributing
the entire loss to transfer bandwidth. The single-thread ordered kernel is not
using independent-world parallelism. Kernel-event timing also includes enqueue
interval and scheduling effects; it is not an isolated instruction benchmark.

Keep JS as the collection default and compiled CPU as the faster Python option
for the tested workloads. CUDA remains useful for numerical experimentation;
it is not currently a performance option for one world. A stronger GPU design
would keep several physics systems resident and batch independent worlds while
preserving each world's solver order. Neither change was implemented in this
comparison.

The user clarified that CPU-only JAX is deliberate: earlier profiling found
GPU JAX slower for this workload, with better dependency and sandbox access as
additional benefits. This run did not benchmark JAX or change its dependencies.

Evidence: `cuda-long-run/results.json`, `validation.json`, `repeatability.json`,
`benchmark.py`, immutable scene/event/command files in each trial directory,
`solver-source.py`, `L03-native-warp.pstats`, `L03-native-warp-cuda.pstats`,
`L03-kernel-events.json` and `profile_cuda.py`. The recorded revision is
62c086eb; exact solver and command hashes are in results.json. All earlier
failed setup and device-access evidence remains retained.
