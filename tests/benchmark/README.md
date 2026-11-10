# Native HP4 logo benchmark

For the required bounded regression gate, run `npm run test:physics`. It rebuilds
native/Wasm physics and checks the first 20 seconds of the production layered
HP4 logo at the browser's default one substep plus loaded-state settling at one,
two and four substeps. It runs
automatically under `npm test` and CI. The longer benchmarks below are optional.

Failures report the backend and motion step or settling substep count. Reproduce
with `npm run test:physics`; startup/parity checks and passing at higher substep
counts do not establish stability at browser defaults. Keep the one-substep case
when diagnosing failures. These tests use production backends without a browser.

For production Rust HP3/HP4 profiling, see
[headless profiling](HEADLESS_JS_PROFILING.md), including per-system timing,
V8 CPU/allocation profiles, GC measurements and frozen autocal replay.

For full-autocal recording-mode time/storage comparisons, use
[`autocal_recording_modes.py`](autocal_recording_modes.py). The
[recording cost report](../../research/autocal-recording-performance.md) contains
measured results, workload/trajectory equivalence gates and reproduction steps.

The headless profiler selects native or Wasm explicitly and records the loaded
artifact and source hashes:

```bash
npm run build:native -- --no-pgo
npm run build:wasm
node scripts/profile_headless.mjs --solver native --machine hp4 \
  --commands /tmp/hp4-rrf.json --layering on --output /tmp/hp4-native.json
node scripts/profile_headless.mjs --solver wasm --machine hp4 \
  --commands /tmp/hp4-rrf.json --layering on --output /tmp/hp4-wasm.json
```

Use `--compare /tmp/hp4-native.json` to check another engine against the
recorded checkpoints. Keep repeats sequential and avoid other benchmarks or
tests competing for CPU. The profiler uses the production headless world with a
2 ms timestep, line layering, open-loop motors and two cable solver iterations.

For full-autocal recording-mode time/storage comparisons, use
[`autocal_recording_modes.py`](autocal_recording_modes.py). The
[recording cost report](../../research/autocal-recording-performance.md) contains
measured results, workload/trajectory equivalence gates and reproduction steps.
