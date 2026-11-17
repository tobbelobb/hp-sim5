# Hangprinter validation — 2026-11-17

Work is on superproject main. Forward development was tested, committed and pushed
first (implementation `ef77f41`, documentation/head `ab981d7` in
`forward-transform-dev/hangprinter-forward-transform`), then
ported to RRF and Klipper. Evidence for this follow-up is under
`/tmp/hp5-followup/`; suite logs are `/tmp/hp5-*.log`.

## Tikhonov correction

Inspection of RRF 1295c622e and Klipper 97900ec65 found no hard-Tmin clamp to
revert. Both retain the projected-gradient soft minimum; the moved clamp is
`[0,Tmax]`, already present in their parent revisions when pretension was enabled.
It now also applies with pretension disabled. Keep that fix, QP bound transitions,
axis-major derivative buffers, double row-space arithmetic and spool precision.

Standalone flex and real firmware regressions explicitly require Tikhonov's
centre Tmin shortfall to lie between .2 and .3 N (observed .233079 N), separately
from hard zero/Tmax checks. The cumulative reference patches and SYNC_AUDIT.md
now describe that distinction. Prints use QP and autocal collects with firmware
flex disabled, so neither was affected by a supposed hard-Tmin change.

## Forward transform and remaining force tradeoff

The hybrid tries a double-precision squared-range rigid seed, keeping it only
when it improves the full length-space residual. It rejects near-planar layouts
with four or more anchors when a normal-system elimination pivot is <=1e-8 of
the largest diagonal. The .01 mm flattened-top mirror probe now fails explicitly
instead of returning a plausible wrong branch. Exactly planar three-anchor
research cases retain their branch-selecting seed convention.

Both development suites pass. The forward tests cover 1,788 workspace/anchor
approaches, 400 motor roundtrips, 800 inconsistent-length comparisons to SciPy,
50 noisy flex cases, and mirror/near-planar probes. Clean workspace error remains
below .02 mm. Some .01 mm anchor-collision flex probes remain nonconvergent
(worst .423 mm); production autocal enforces 10% home-distance clearance.

QP's centre imbalance (.025637 N direct HP5 / .107010 N reduced) is the finite
ridge objective's tradeoff, not an active-set failure. This change preserves
that objective and its independently checked optimum. Neither force solver
promises exact static equilibrium at all poses.

## Autocal range model

The redundant-machine point fit previously treated every range as rigid. It now reconstructs each
pose's active line forces from static equilibrium, the recorded commanded sensor
force and compliant unused-line loads. Source sensor axes are preserved when
reverse sub-sweeps are mapped into canonical drive/sensor coordinates. It then
inverts the firmware compliance formula
`material = distance - F*(U*distance+Y)/(EA*U²*N)`.

The shared flex inverse had an obsolete MA/strand formula and is corrected.
Rough bootstrap, missing historical force records and projected datasets retain their rigid fit.
A synthetic reduced-HP5 regression independently generates stretched ranges,
including both sensor directions and passive pretension; corrected consistency
residuals are below 1e-8 mm. The model uses configured EA and static loads; it
does not model bearing friction, pulley tangency or changing mover orientation.
A rigid prefit supplies a nearby seed before stretch refinement, avoiding
singular force systems at distant trial geometries. HP3/HP4 retain their ellipse
objective and its rigid offset model. Stop rules, iteration limits, patience and
external timeouts are unchanged.

### Reference measurements before this follow-up

The e5593f44-era validation recorded stable-stop maximum anchor errors of
1.023 mm (HP3), 4.559 mm (HP4), and 1.851 mm (HP5). Reduced HP5 was interrupted
at 1000 seconds with 2.414 mm selected-fit error and 14 retained sweeps; that
was not a stable-stop pass. Its last fit steps were 3.254/.535/1.288/.494/.523 mm.

## Reproduction

Use a fresh dataset path for each run, including after interrupted bootstrap:

```bash
cmake --build RRF/build --target rrf_simulator -j 4
bash forward-transform-dev/hangprinter-forward-transform/test.sh
bash flex-compensation-dev/hangprinter-flex-compensation/test.sh
.venv/bin/python scripts/compile_machine_commands.py --machine hp3 --machine hp4 --machine hp3_w_line_layers --machine hp4_w_line_layers --machine hp5 --machine hp5_pulley_reduced
HP_SIM5_AUTOCAL_MAX_PHYSICS_STEPS=6000000 timeout -s INT 1000 .venv/bin/python autocal/autocal.py --machine-type hangprinter_5_pulley_reduced --dataset /tmp/hp5-followup/fresh-reduced.json --headless-sim --recording-mode none --patience 20 --verbose
.venv/bin/python -m pytest -m 'not heavy' -q
.venv/bin/python -m pytest -m heavy -q
.venv/bin/python RRF/tests/test_flex_mode_position.py
.venv/bin/python -m pytest klipper/test/klippy/test_winch.py -q
npx jest --runInBand
npm run build:wasm
node --test --test-concurrency=1 tests/benchmark/*.test.mjs
node tests/benchmark/logo_accuracy.mjs --machine hp5_pulley_reduced --firmware klipper --open-loop
```

Require manifest `normal_completion: true` and `stop_reason: stable-estimate`.
An interrupted run is not a pass. HP4 Klipper's full logo needs an explicit
command stream because its catalogue defaults to squares:

```bash
.venv/bin/python klipper/klippy/klippy.py public/klipper/hp4/printer-hp4-linux-mcu.cfg -i public/gcode/Hangprinter_logo6.gcode -o /tmp/hp4-logo.serial -d public/klipper/linux_mcu/klipper.dict
node tests/benchmark/logo_accuracy.mjs --machine hp4 --firmware klipper --commands /tmp/hp4-logo.serial --open-loop
```

## Follow-up validation results

The first physics run detected a stale browser Wasm artifact. Rebuilding from the
current Rust sources fixes it: 86 physics tests pass, one optional GC probe skips.
Fast Python: 688 pass. Jest: 402 pass. RRF: 12 pass. Klipper: 15 pass.
The actual-kernel Python tests also cover the soft-minimum contract and all three
range-seed implementations. Both standalone development suites pass.

The broad non-heavy Python rerun passed 758 with one optional skip. An earlier
run had three service startup/shutdown failures under concurrent validation; all
three also passed in an isolated slow-marker rerun (28.44 s). The heavy run passed the interruption test
and reproduced the pre-existing layered HP3 global-radius bootstrap/return
failure; it is not a clean full-suite pass. No timeout was increased to hide it.

All eight complete-logo checks have zero missed steps. HP4 Klipper uses its
explicitly compiled logo stream. Tracking and contour errors remain comparable
to the earlier validation, with the documented large Klipper outliers absent.

| Machine | Firmware | Tracking RMS mm | Contour max mm |
|---|---|---:|---:|
| hp3 | rrf | 0.2196 | 0.1922 |
| hp3 | klipper | 0.2192 | 0.1320 |
| hp4 | rrf | 0.2173 | 0.1934 |
| hp4 | klipper | 0.2283 | 0.1842 |
| hp5 | rrf | 0.2148 | 0.1762 |
| hp5 | klipper | 0.2328 | 0.1840 |
| hp5 reduced | rrf | 0.2315 | 0.2192 |
| hp5 reduced | klipper | 0.2429 | 0.2276 |

Fresh baseline comparison (`reduced-baseline2`, `hp4-baseline2`): HP4 reached
stable-estimate in 996.18 s with 13 retained sweeps. Reduced HP5 again hit the
1000 s limit (999.43 s recorded), with 13 sweeps and no stable-stop. Its latest
fit worsened as coverage expanded, while the selected fit stayed near the earlier
result. This reproduces the original issue independently of Tikhonov.

### Final autocal runs

All four production-policy machines reach stable-estimate with normal completion.
The reduced run retains the original two-sweep .5 mm rule and 1000 s timeout;
its final two steps are recorded below. Anchor errors use the applied, rounded
M669 values against firmware anchors, without alignment.

| Machine | Evidence stem | Stop | Max anchor error mm | Sweeps | Wall s |
|---|---|---|---:|---:|---:|
| HP3 | `hp3-confirmed` | stable-estimate | 1.030 | 9 | 475.78 |
| HP4 | `hp4-baseline2` | stable-estimate | 4.570 | 13 | 996.18 |
| HP5 | `hp5-production` | stable-estimate | 1.258 | 11 | 522.27 |
| HP5 reduced | `reduced-production` | stable-estimate | 2.378 | 7 | 632.74 |

Reduced final coordinate steps: 0.4094 and 0.2251 mm.

The Tikhonov audit correction does not change QP print tracking or F0 autocal.
HP3/HP4 accuracy remains comparable to the e5593f44-era measurements; direct
HP5's selected error improves, and reduced HP5 now satisfies stable-stop. This
is convergence of the estimate, not proof of unbiased anchor metrology: nominal
compliance, friction and pulley routing still limit absolute accuracy.

Committed component heads: forward `ab981d7` (pushed), flex `815fa31`,
RRF `f4b08734b`, Klipper `546416321`. Firmware and superproject are local only.
