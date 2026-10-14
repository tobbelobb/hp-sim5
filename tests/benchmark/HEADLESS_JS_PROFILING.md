# Headless JavaScript physics profiling

`scripts/profile_headless.mjs` runs `createHeadlessWorld()` and the production
system order, at the scene's 2 ms timestep. It does no rendering, firmware HTTP,
worker transport, pacing or Rerun recording. No physics settings are simplified.
Headless worlds install a disabled `PerformanceMonitor` by default; pass
`profiling: { enabled: true, logEverySteps: 0, slowStepMs: Infinity }` to enable
silent accumulation. `snapshot()` returns structured per-system statistics;
`resetWindow()` starts a new measurement window. The browser's periodic console
reporting still uses the same monitor.

Run these examples from the repository root with Node 24 or newer. Keep
benchmarks sequential, with no tests or other simulation jobs competing for CPU.

## Freeze a workload

Schedule the existing real firmware logo commands with the production player:

```bash
mkdir -p output/headless-profile
node scripts/headless_commands.mjs \
  public/RRF_CAN_commands/Hangprinter_logo6_hp4_w_line_layers.can \
  output/headless-profile/logo.json .002 RRF/run/vsd/sys/config_hp4_w_line_layers.g
```

Use the corresponding HP3 input and config for HP3. Each JSON object occupies
one physics timestep. `{}` holds the preceding targets. `--steps 30000` measures
60 simulated seconds; omit it for the full trajectory. For compact frozen input,
save just the prefix to a new JSON file before benchmarking.

Alternatively pass `--events PATH/events.jsonl` from an actual autocal collection
or headless service. This preserves command arrival steps, queued FIFO execution,
closed-loop settings, idle settling intervals and available encoder reads.
Multiple commands arriving at one step are not flattened into a simultaneous
motor update. One epoch is required; resets and layering settings in the trace
are rejected. Layering is chosen explicitly at construction, including the
browser's matching initial cable-length bake. Trace files and scene files must
come from the same machine/configuration for a faithful replay.

A frozen autocal replay measures its saved movement/settling pattern. The
collector does not adapt to new numerical behavior, and its HTTP, sensor polling,
firmware planning and recording costs are absent. Changing layering while holding
commands fixed is a controlled physics cost comparison; the resulting positions,
forces and settling behavior can differ from the original trace.

## Measure first, attribute separately

```bash
node scripts/profile_headless.mjs --machine hp4 \
  --commands output/headless-profile/logo.json --steps 30000 --layering on \
  --output output/headless-profile/plain.json

node scripts/profile_headless.mjs --machine hp4 \
  --commands output/headless-profile/logo.json --steps 30000 --layering on \
  --systems --repeats 1 --compare output/headless-profile/plain.json \
  --output output/headless-profile/systems.json

node --cpu-prof --cpu-prof-dir=output/headless-profile --cpu-prof-name=physics.cpuprofile \
  scripts/profile_headless.mjs --machine hp4 \
  --commands output/headless-profile/logo.json --steps 30000 --layering on \
  --repeats 1 --compare output/headless-profile/plain.json \
  --output output/headless-profile/cpu.json

node scripts/summarize_v8_profile.mjs output/headless-profile/physics.cpuprofile \
  output/headless-profile/cpu.json > output/headless-profile/cpu-summary.json

node scripts/profile_headless.mjs --machine hp4 \
  --commands output/headless-profile/logo.json --steps 30000 --layering on \
  --heap-sampling --repeats 1 --compare output/headless-profile/plain.json \
  --output output/headless-profile/heap.json

node scripts/summarize_v8_profile.mjs output/headless-profile/heap.json.0.heapprofile \
  > output/headless-profile/heap-summary.json
```

Repeat for HP3/HP4, `--layering on/off`, and logo/autocal workloads. Keep the same
command stream across layering cases to isolate the layering flag and bake.
`--scene FILE.usda` selects a frozen original scene with non-default physical
parameters. `--closed-loop` selects initial motor mode; saved settings in an
autocal trace are then applied at their original steps.

By default the process warms the VM with 5,000 steps in a separate world, then
constructs a fresh world for each of three measured repetitions. `--warmup 0`
measures a cold VM. Warmup is a workload prefix: if torque transitions/movement
occur later, their first occurrence can still include JIT compilation. Long
trajectories amortize this. Record the warmup length when comparing results.

JSON output includes inputs and baked-scene hashes, actual JS source hash, Git
revision, Node/V8/CPU/platform identity, flags, construction time, physics-update
time, observation time, total replay wall time, process CPU usage, memory usage,
GC count/duration/kinds and optional per-system times. Median throughput uses
only time inside the update batches, including GC during those batches. Command
arrival handling and numerical observations are outside those timers. GC event
durations describe Node's reported events, not all concurrent collector CPU work.
Heap growth is retained memory change, not total allocation.

CPU summaries filter native `--cpu-prof` samples to measured replay intervals,
excluding imports, construction and warmup. Replay observations remain in these
intervals; their measured cost is reported separately. Self time identifies
hot functions; inclusive time identifies caller stacks and can exceed 100% when
summed. Open the raw `.cpuprofile` in Chrome DevTools to inspect full call trees.
Allocation sampling uses V8's existing inspector `HeapProfiler.startSampling`
at 32 KiB, retaining samples for objects collected by minor and major GC. The
result estimates allocated bytes, including temporary objects, and is not an
exact byte count. Open `.heapprofile` in DevTools' Memory panel for stacks.
Instrumentation changes speed; use the plain runs for throughput claims.

## Numerical checks and isolated changes

Every run captures finite frame positions/orientations, motor targets/encoders,
velocities, cable topology/lengths/forces and queue length every 1,000 steps,
plus recorded encoder observation times. Each repetition must match the complete
checkpoint hash exactly. `--compare BASELINE.json` first requires identical
workload/scene/settings/step/checkpoint/warmup inputs, then exact trajectory
equality. Differing inputs and numerical regressions fail the command; checkpoints
from a failed comparison are retained for diagnosis. Raw encoder differences
against old recorded reads are reported separately and do not establish parity
with an earlier engine revision or hardware accuracy.

For an optimization, change one code path, rerun the same uninstrumented inputs
with `--compare`, and compare the repeat distributions as well as medians. Then
rerun attribution. A numerical change needs an explicit tolerance/physics review
and the relevant parity tests; this harness does not silently relax equality.
No optimization is accepted solely because a profiled run became faster.

## Rust/WASM comparison

The batched Rust/WASM cable solver and attachment geometry backend is the
default in the browser app and profiling CLI. The browser loads the checked-in
binary at startup; use `?solver=js` to select JavaScript. Build the binary with
`npm run build:wasm`; the checked-in artifact works without a Rust toolchain.
For comparisons, pass `--solver js` to make a JS baseline and `--compare` that
baseline to the default WASM run. The same workload contract and exact numerical
checks apply. Output identifies the selected backend and WASM SHA-256, and the
source hash includes the Rust source and build recipe. Packing/unpacking stays
inside the production system timers.

Run the complete matrix sequentially with:

```bash
node scripts/benchmark_cable_wasm.mjs --inputs FROZEN_INPUT_DIR --output NEW_DIR
node --test tests/benchmark/wasm_cable_solver.test.mjs
```

See [the Rust crate](../../src/wasm/cable_solver/README.md) for setup, the
headless/browser API and the measured engine compatibility scope.

Harness checks:

```bash
node --test tests/benchmark/profile_headless.test.mjs
npx jest --runInBand tests/js/hp-sim-3d/headlessWorld.test.js \
  tests/js/hp-sim-3d/performanceMonitor.test.js
```

Native APIs: [Node CPU profiler](https://nodejs.org/api/cli.html#--cpu-prof),
[Node performance hooks](https://nodejs.org/api/perf_hooks.html), and
[V8 allocation sampling](https://chromedevtools.github.io/devtools-protocol/tot/HeapProfiler/#method-startSampling).
