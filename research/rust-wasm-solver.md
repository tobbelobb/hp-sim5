# Rust/WASM cable solver performance, 2026-10-13

Rust/WASM reduces total physics time by **17.4–29.3% on HP4** (1.21–1.41×
throughput) against today's optimized JavaScript. HP3 improves **0.2–6.2%**;
its layered autocal replay is essentially unchanged. All 48 measured trajectories
retain exact checkpoints. These are sequential single-host measurements with
other research processes active; small gains need cautious interpretation.

## History and implementation

Commit `0b5afc52` established the original frozen workloads and attributed the
largest costs to constraints, attachment updates and over-correction. The later
commits `c5769eff`, `71642cf8` and `d0dca351` reused component stores, transform
scratch and attachment preparation. Their cumulative report recorded 6.7–16.0%
reductions; `466242f0` added solver endpoint/joint/path caches. This comparison
uses current `ded5842e` as its JS baseline, including all those optimizations.

The [Rust crate](../src/wasm/cable_solver/README.md) implements all alternating
path/joint/iteration loops, rigid-body inverse inertia and rotation, direct and
pinhole-coupled spin, motor holding-torque release, XPBD damping, encoder updates,
force transfers and torque-load coefficients. The JS bridge packs live state
once per update and writes back changes after one batched solver call. Attachment
tangent geometry runs in a separate reusable buffer, one call per joint. Scene
construction, winding/topology/history, motor/force integration and the other
systems remain in JS. The default remains JS; select WASM explicitly.

Arithmetic remains f64, with the same operation order and host Math trig
functions. Vector lengths execute in WASM using V8-compatible scaled sums.
No timestep, solver-iteration, compliance, layering or settling simplification
was made. The release artifact is about 35 KB with no Cargo dependencies.

## Uninstrumented measurements

Three fresh-world repeats per backend, each after a separate 5,000-step warmup.
Node 24.14.1 / V8 13.6.233.17-node.44, Rust 1.98.1, Intel i7-12700K. The eight
original frozen HP3/HP4 logo and autocal workloads cover layering on/off:
**48 trajectories and 7,544,232 measured physics updates**. Timers include bridge
packing/unpacking and GC, and exclude construction, observations, firmware,
transport, rendering and recording.

| Case | JS seconds | WASM seconds | Speedup | Time reduction |
| --- | ---: | ---: | ---: | ---: |
| hp3-logo-on | 2.530 | 2.372 | 1.07× | 6.2% |
| hp3-logo-off | 2.043 | 2.012 | 1.02× | 1.5% |
| hp3-autocal-on | 36.085 | 36.013 | 1.00× | 0.2% |
| hp3-autocal-off | 37.508 | 35.731 | 1.05× | 4.7% |
| hp4-logo-on | 14.529 | 10.379 | 1.40× | 28.6% |
| hp4-logo-off | 10.474 | 8.648 | 1.21× | 17.4% |
| hp4-autocal-on | 36.984 | 27.152 | 1.36× | 26.6% |
| hp4-autocal-off | 38.112 | 26.955 | 1.41× | 29.3% |

All HP4 WASM repeat ranges lie below their matching JS ranges. HP3's unlayered
logo and layered autocal ranges overlap. The layered HP3 autocal replay preserves
all 1,956 original encoder observations within 1.14e-13 degrees. The unlayered
trace remains a frozen stress workload with different physical behavior from
its original collection. HP4's historical trace lacks explicit encoder reads.
These runs establish engine parity and physics throughput, not a fresh adaptive
collection or hardware performance.

## Attribution

Separate instrumented 60-second layered logo runs match the same checkpoints.
System times are diagnostic; the speedup claims above use only plain runs.

| Machine/system | JS system seconds | WASM system seconds |
| --- | ---: | ---: |
| hp3/PBDCableConstraintSolver | 0.956 | 0.681 |
| hp3/CableAttachmentUpdateSystem | 0.718 | 0.721 |
| hp3/PBDResolveCableOverCorrections | 0.396 | 0.396 |
| hp4/PBDCableConstraintSolver | 6.588 | 3.154 |
| hp4/CableAttachmentUpdateSystem | 3.433 | 3.178 |
| hp4/PBDResolveCableOverCorrections | 1.803 | 1.542 |

The full HP4 constraint system, including the JS bridge, is approximately 2.09×
faster. Its attachment update becomes the largest system, almost tied with the
solver; most attachment/winding/history work still executes in JS. HP3's solver
is about 1.40× faster in this diagnostic run, but its smaller constraints and
other pipeline costs limit the end-to-end gain. Further improvements would
require reducing these remaining costs or moving more state and systems into
Rust. This experiment does not separately time kernel execution and marshaling.

Development trials retained in the session show HP4 logo medians of 13.956 s
(JS), 10.816 s (solver with imported hypot), 10.222 s (native vector lengths) and
10.424 s (added tangent geometry). These trials preceded the final matrix;
changing host load/JIT and small differences prevent attributing a reliable
additional total gain to geometry alone. The final combined implementation is
measured directly above, so isolated percentages are not added.

## Verification and reproduction

- Exact checkpoint hashes across all 48 plain trajectories, and exact equality
  with the original morning profiling checkpoint hashes in all eight JS cases.
- 49 WASM Node checks: 31 differential fixtures, an empty/clear/reused-ID case,
  10,000 vector-length samples plus extremes, and 16 every-timestep commanded
  scene comparisons (all eight designs, layering on/off).
- Four existing Node profiling checks, 764 Jest tests (one skipped), and 160
  independent Python/JS differential tests passed.
- Release build, Rust formatting, Clippy with warnings denied, and diff checks.
- Exact numerical claims are tested on Node/V8. Browser timing and other JS
  engines are not established by these experiments.

```bash
rustup target add wasm32-unknown-unknown
npm run build:wasm
node --test tests/benchmark/profile_headless.test.mjs tests/benchmark/wasm_cable_solver.test.mjs
node scripts/benchmark_cable_wasm.mjs --inputs FROZEN_INPUT_DIR --output NEW_DIR
```

For a single run, add `--solver wasm` to the
[profiling CLI](../tests/benchmark/HEADLESS_JS_PROFILING.md), using `--compare`
a matching JS baseline. WASM backend/artifact hashes are recorded separately
from the shared workload contract.

## Evidence and limitations

The [session report](../output/research/sessions/2026-10-13-rust-wasm/report.md),
[matrix](../output/research/sessions/2026-10-13-rust-wasm/matrix/matrix.json),
[raw runs/checkpoints](../output/research/sessions/2026-10-13-rust-wasm/matrix/) and
[notebook](../output/research/sessions/2026-10-13-rust-wasm/research.md) retain
repeat distributions, flags, source/binary/scene/input hashes and encoder checks.
The measured source snapshot and binary are archived in the session's `source/`
directory. Research artifacts are local and Git-ignored, following the original
profiling workflow. The Rust/JS implementation, build, tests and this report are
repository changes.

CPU-heavy Python research, Rerun and other desktop processes were active on this
host. Our benchmark cases ran sequentially and our tests/profiles did not overlap
plain measurements. Three repeats on one host are insufficient to establish
small improvements across hardware; the large HP4 reductions are more convincing
than the small HP3 changes. Final-build CPU-0 affinity check: JS median **14.748 s**, WASM **10.914 s**, **1.35×** throughput. JS repeats span 14.671–14.870 s, WASM 10.837–11.209 s. All checkpoints match. Affinity constrains this process and its GC threads to one logical CPU; other research processes were not stopped. This is a separate robustness check, not pooled into the matrix. The final build's only Rust changes after the matrix were
safety documentation, a localized lint annotation and an equivalent Option
short-circuit; the final binary is separately checked against the checkpoints.

References: [original profiling](headless-js-profiling.md) supports workload
selection and hotspot attribution; [cumulative JS work](attachment-calculation-reuse.md)
explains the optimized baseline; [V8 13.6 math implementation](https://github.com/v8/v8/blob/13.6.233/src/builtins/math.tq)
supports the vector-length arithmetic choice. All performance claims above come
from the local experiment, not external sources.
