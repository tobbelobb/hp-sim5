# Native HP4 logo benchmark

For the required bounded regression gate, run `npm run test:physics`. It rebuilds
native/Wasm physics and checks the first 20 seconds of the production layered
HP4 logo at the catalogue default of one substep/ten iterations and an explicit
four-substep override, plus loaded-state settling at one, two and four substeps
and symmetric pretension convergence in native/Wasm. It runs automatically
under `npm test` and CI. The longer benchmarks below are optional.

Failures report the backend and motion step or settling substep count. Reproduce
with `npm run test:physics`; startup/parity checks and passing at higher substep
counts do not establish stability at browser defaults. The motion gate also
bounds tilt and verifies that the effector settles after commands stop. Keep
both the catalogue-default and one-substep cases when diagnosing failures. These
tests use production backends without a browser.

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
2 ms firmware tick, one substep, catalogue solver iterations (HP4: ten; HP3:
one), line layering and open-loop motors.

For complete-logo path accuracy (native, closed loop), run:

```bash
node tests/benchmark/logo_accuracy.mjs --machine hp4
node tests/benchmark/logo_accuracy.mjs --machine hp4 --iterations 8
node tests/benchmark/logo_accuracy.mjs --machine hp4 --substeps 4 --iterations 2
node tests/benchmark/logo_accuracy.mjs --machine hp4_w_line_layers --open-loop
```

[HP4 solver convergence measurements](hp4_substeps.md) explain the catalogue
default and the stability bounds.
