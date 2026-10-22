# Stability correction validation

Starting revision: `628a8963` (unsafe no-slip shortcut integrated).
Production correction: withdraw shortcut across JS/Python/Rust; retain the
force readback and signed motor feedback corrections. Defaults/tolerances unchanged.

## Required gate

```bash
npm run test:physics
```

[`stability-gate-final.log`](stability-gate-final.log): six passes, about 30 s.
[`npm-test-final.log`](npm-test-final.log): the actual default npm command runs
the gate, then 162 Jest suites (800 passes, one skip).

The negative control intentionally exits 1: [`gate-rejects-broken-final.log`](gate-rejects-broken-final.log),
using the archived rejected engine, fails at step 293 / 0.586 simulated seconds.
This is expected evidence that the gate detects the shipped defect.

## Additional executed checks

- [`backend-final.log`](backend-final.log): 157 native/Wasm/host passes.
- [`python-after-final.log`](python-after-final.log): 196 Python core/differential passes.
- [`warp-after.log`](warp-after.log): 110 CPU passes, five CUDA skips.
- [`logo-substeps-after.log`](logo-substeps-after.log): four existing substep cases pass.
- [`rust-final.log`](rust-final.log): three native Rust tests pass.
- [`prefix-equivalence.json`](prefix-equivalence.json): exact sampled COM motion,
  extrema and motor-diagnostic equality for archived baseline versus correction.
- [`feedback-pull-final.log`](feedback-pull-final.log): fresh 30 s torque pull
  completes; no calibration accuracy claim.

## Extended replay

[`full-logo.log`](full-logo.log) is a preserved failed acceptance trial: the
fixed 5 mm print-plane bound is crossed at step 1,038,238, within the final 21
commands. The full stream has 1,038,258 commands. The diagnostic retry completes all commands with finite bounded motion, but
the terminal return loses 300 motor steps. Corrected and original engines
produce exactly equal 5500 tail/hold body samples and diagnostics from the same
checkpoint, then settle. See [comparison](tail-comparison.json) and
[complete diagnostic](S04-full-retry.json). This is not a perfect tracking pass.
The committed short-gate criteria have not changed.

Earlier `python-after.log` has a mistyped file path; `feedback-pull.log` has an
expected stale-build refusal after source comments changed; `jest-after.log`
contains the stale zero-force expectation corrected to analytic 100 N. These
failures remain as history; their successful reruns above supersede them.

See [`stability-provenance.json`](stability-provenance.json) for artifacts, source
hashes and input schedule. All numerical trials are fresh worlds, not the
continuing supervisor world.
